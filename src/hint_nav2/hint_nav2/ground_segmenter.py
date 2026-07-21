"""Ground segmenter -> obstacle point cloud (fused, one node) for the Nav2 costmap.

Runs a semantic-segmentation ONNX model via OpenVINO (Intel iGPU by default), mirroring
``depth_anything``'s inference plumbing (same ``Core`` / ``compile_model`` setup, same
ImageNet preprocessing). Consumes ``/camera/image_raw/compressed`` — the node runs
off-robot and pulling the raw stream costs more latency than the JPEG decode.

Unlike the earlier segmenter this node does **not** publish a binary mask. It owns the
camera geometry and turns the ground segmentation straight into an **obstacle point
cloud** for Nav2's local costmap:

1. Infer per-pixel ground probability (model-output layout detected at load time, exactly
   as before), threshold into a ground mask at the camera frame.
2. Warp the mask through the ground-plane homography into a metric top-down (BEV) grid
   (camera model ``camera_height`` / ``camera_tilt`` / ``camera_hfov_deg`` /
   ``camera_forward_offset``, the same projection ``odom_waypoint_tracker`` uses); a cell
   that is **known but not ground** is an obstacle.
3. Publish those obstacle cell centres as a ``sensor_msgs/PointCloud2`` on
   ``/ground/obstacles`` in ``base_link`` (z = 0), stamped with the **source frame's
   header stamp** — Nav2's obstacle layer TF-transforms it ``base_link -> odom`` at that
   stamp, so segmenter latency lands the points where they were seen, not where the robot
   is now (free latency compensation). One point per BEV cell keeps the cloud light.

It also publishes a green/red ground-overlay debug image on ``/camera/ground/debug``
(subscriber-gated).

The output layout is detected at load time, so a model pulled from Hugging Face works
without code edits:

* ``(1, C, H, W)`` / ``(1, H, W, C)`` with ``C > 1`` — per-class logits. Softmaxed, then
  the ``ground_class_ids`` probabilities are **summed** into one ground score.
* ``(1, 1, H, W)`` / ``(1, H, W)`` float — a single ground score. Sigmoid applied when the
  values look like logits (outside ``[0, 1]``).
* ``(1, H, W)`` integer — an already-argmaxed class map; ``ground_class_ids`` selects.
"""

import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from openvino.runtime import Core
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# ImageNet normalization (RGB order) — the standard for HF segmentation backbones.
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# ADE20K (150-class) id for 'floor' — the only ground an indoor robot really drives on.
# Verify against your model's own label map, and widen per environment (28 rug, 6 road,
# 13 earth) — the exporter prints the candidate ids.
_DEFAULT_GROUND_IDS = [3]

# Latest-frame-wins. Inference is slower than the camera, so anything deeper than 1 buys
# a backlog: the node would work through frames the robot has already driven past. Depth 1
# + BEST_EFFORT lets the middleware drop stale frames instead.
_LATEST_FRAME_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


class GroundSegmenter(Node):
    def __init__(self):
        super().__init__("ground_segmenter_node")
        self.bridge = CvBridge()

        default_model_path = os.path.join(
            get_package_share_directory("hint_nav2"),
            "models", "segformer-b5-ade.onnx",
        )
        model_path = os.path.expanduser(os.path.expandvars(
            self.declare_parameter("model_path", default_model_path)
            .get_parameter_value().string_value
        ))
        device = (
            self.declare_parameter("device", "AUTO").get_parameter_value().string_value
        )

        self.declare_parameter("ground_class_ids", _DEFAULT_GROUND_IDS)
        self.declare_parameter("ground_threshold", 0.5)  # ground-probability cut
        self.declare_parameter("morph_kernel", 7)  # hole-fill / speck-drop size, px

        # --- Camera geometry for the ground->BEV homography (match the real rig) ---
        self.declare_parameter("camera_height", 0.14)  # m above the ground plane
        self.declare_parameter("camera_forward_offset", 0.0)  # m ahead of base origin
        self.declare_parameter("camera_tilt", 0.0)  # rad, positive = pitched down
        self.declare_parameter("camera_hfov_deg", 62.2)  # horizontal FOV (Pi cam v2)
        # --- BEV window (base_link: x forward, y left); one obstacle point per cell ---
        self.declare_parameter("bev_range", 3.0)  # m forward coverage
        self.declare_parameter("bev_half_width", 1.5)  # m lateral each side
        self.declare_parameter("bev_resolution", 0.05)  # m per cell (~ costmap res)
        self.declare_parameter("obstacle_frame", "base_link")  # cloud frame_id
        self.declare_parameter("overlay_alpha", 0.4)  # debug ground-tint strength

        self.get_logger().info(f"Loading ONNX model '{model_path}' on device '{device}'")
        core = Core()
        model = core.read_model(model_path)
        config = {"PERFORMANCE_HINT": "LATENCY"}
        if device in ("GPU", "AUTO"):
            config["INFERENCE_PRECISION_HINT"] = "f16"
        self.compiled_model = core.compile_model(model, device_name=device, config=config)
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)
        _, _, self.in_h, self.in_w = (int(d) for d in self.input_layer.shape)
        self.get_logger().info(f"Model input size: {self.in_w}x{self.in_h} (WxH)")

        self._class_axis = self._detect_class_axis()

        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self.callback, _LATEST_FRAME_QOS,
        )
        # Reliable pub so a best-effort costmap observation sub is still compatible.
        self.pub_obstacles = self.create_publisher(PointCloud2, "/ground/obstacles", 5)
        self.pub_debug = self.create_publisher(Image, "/camera/ground/debug", 1)

        self.get_logger().info("OpenVINO Ground Segmenter (obstacle cloud) ready!")

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Output-layout detection

    def _detect_class_axis(self):
        """Locate the class axis in the model's output, or None if there isn't one.

        The input H/W are known, so the spatial axes are the pair whose aspect ratio
        matches the input's — whatever is left over is the class axis. Segmentation
        heads often emit at a reduced stride, hence the ratio (not exact) match.
        """
        shape = [int(d) for d in self.output_layer.partial_shape.get_max_shape()]
        self.get_logger().info(f"Model output shape: {tuple(shape)}")
        if len(shape) == 3:  # (1, H, W) — a score map or an argmaxed class map
            return None
        if len(shape) != 4:
            self.get_logger().warn(
                f"Unexpected output rank {len(shape)}; assuming NCHW class logits."
            )
            return 1

        target = self.in_h / float(self.in_w)
        _, a, b, c = shape
        nchw_err = abs(b / float(c) - target) if c else 1e9
        nhwc_err = abs(a / float(b) - target) if b else 1e9
        if nhwc_err < nchw_err:
            self.get_logger().info(f"Detected NHWC output ({c} classes)")
            return 3
        self.get_logger().info(f"Detected NCHW output ({a} classes)")
        return 1

    # ------------------------------------------------------------------
    # Mask extraction

    def _ids(self):
        return np.asarray(self._p("ground_class_ids"), dtype=np.int64)

    def _ground_score(self, out):
        """Reduce a raw model output to a ground probability map at the model's stride.

        Multi-class logits are softmaxed and the ground classes' probabilities summed,
        so labels that split the ground vote (floor / rug / carpet) reinforce each other
        instead of competing for a single argmax winner.
        """
        out = np.squeeze(out, axis=0) if out.ndim == 4 and out.shape[0] == 1 else out

        if self._class_axis is None or out.ndim == 2:
            arr = np.squeeze(out)
            if not np.issubdtype(arr.dtype, np.floating):  # argmaxed class map
                return np.isin(arr, self._ids()).astype(np.float32)
            if arr.min() < 0.0 or arr.max() > 1.0:  # logits -> probability
                arr = 1.0 / (1.0 + np.exp(-arr))
            return arr.astype(np.float32)

        axis = self._class_axis - 1  # the batch dim was squeezed off
        if out.shape[axis] == 1:  # single-channel score in a class-shaped tensor
            self._class_axis = None
            return self._ground_score(out)

        e = np.exp(out - out.max(axis=axis, keepdims=True))  # stable softmax
        prob = e / e.sum(axis=axis, keepdims=True)
        ids = self._ids()
        ids = ids[(ids >= 0) & (ids < out.shape[axis])]  # ignore out-of-range ids
        if ids.size == 0:
            return np.zeros(np.delete(out.shape, axis), dtype=np.float32)
        return np.take(prob, ids, axis=axis).sum(axis=axis).astype(np.float32)

    def _clean(self, mask):
        """Close small holes, then drop small specks. No-op when ``morph_kernel`` <= 1."""
        k = int(self._p("morph_kernel"))
        if k <= 1:
            return mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        return m.astype(bool)

    # ------------------------------------------------------------------
    # Camera model + BEV geometry (same projection as odom_waypoint_tracker / pursuit)

    def _bev_geom(self):
        res = max(1e-3, float(self._p("bev_resolution")))
        rng = float(self._p("bev_range"))
        half = float(self._p("bev_half_width"))
        return (max(2, int(round(rng / res))), max(2, int(round(2 * half / res))),
                res, rng, half)

    def _metric_to_cell(self, x, y, res, rng, half):
        return (rng - x) / res - 0.5, (half - y) / res - 0.5  # (row, col)

    def _ground_to_pixels(self, gxy, w, h):
        """Project ground points (N,2 metric base_link) to image pixels (N,2)."""
        hfov = math.radians(float(self._p("camera_hfov_deg")))
        f = (w / 2.0) / math.tan(hfov / 2.0)
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        tilt = float(self._p("camera_tilt"))
        cos_t, sin_t = math.cos(tilt), math.sin(tilt)
        cam_h = float(self._p("camera_height"))
        x_off = float(self._p("camera_forward_offset"))
        dx = gxy[:, 0] - x_off
        cam_z = cos_t * dx + sin_t * cam_h
        cam_x = -gxy[:, 1]
        cam_y = -sin_t * dx + cos_t * cam_h
        z = np.where(cam_z > 1e-6, cam_z, 1e-6)
        pix = np.stack([cx + f * cam_x / z, cy + f * cam_y / z], axis=1)
        return pix, cam_z > 1e-6  # pixels + in-front mask

    def _mask_to_obstacle_cells(self, ground, w, h):
        """Warp the image-space ground mask to BEV; return obstacle (mx, my) points.

        ``ground`` is a boolean image mask (True = traversable). A BEV cell is an
        obstacle when it is **known** (inside the camera wedge) but **not ground**.
        """
        rows, cols, res, rng, half = self._bev_geom()
        x_off = float(self._p("camera_forward_offset"))
        x_near = max(0.25 * rng, x_off + 0.2)
        gx = np.array([[rng, half], [rng, -half], [x_near, half], [x_near, -half]])
        img_pts = self._ground_to_pixels(gx, w, h)[0].astype(np.float32)
        bev_pts = np.array(
            [self._metric_to_cell(x, y, res, rng, half)[::-1] for x, y in gx],
            dtype=np.float32,
        )
        m = cv2.getPerspectiveTransform(bev_pts, img_pts)

        flags = cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP
        g = cv2.warpPerspective(
            ground.astype(np.uint8), m, (cols, rows), flags=flags, borderValue=0) > 0
        known = cv2.warpPerspective(
            np.full((h, w), 255, np.uint8), m, (cols, rows), flags=flags, borderValue=0
        ) > 0
        # Rows whose ground points sit at/behind the image plane sample garbage.
        tilt = float(self._p("camera_tilt"))
        cam_h = float(self._p("camera_height"))
        cell_x = rng - (np.arange(rows) + 0.5) * res
        behind = (np.cos(tilt) * (cell_x - x_off) + np.sin(tilt) * cam_h) <= 1e-3
        known[behind, :] = False
        g[behind, :] = False

        ys, xs = np.nonzero(known & ~g)
        mx = rng - (ys + 0.5) * res            # metric x forward
        my = half - (xs + 0.5) * res           # metric y left
        return mx, my

    # ------------------------------------------------------------------
    # Preprocess / callback

    def _preprocess(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        rgb = rgb.astype(np.float32) / 255.0
        rgb = (rgb - _MEAN) / _STD
        return np.transpose(rgb, (2, 0, 1))[np.newaxis, ...]  # (1, 3, H, W)

    def callback(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        out = self.compiled_model([self._preprocess(cv_img)])[self.output_layer]
        score = self._ground_score(np.asarray(out))

        # Upsample the score to the camera frame, then threshold — never the reverse. The
        # head emits at a coarse stride; cutting first and resizing the binary mask
        # staircases every boundary by the stride factor.
        h, w = cv_img.shape[:2]
        score = cv2.resize(score, (w, h), interpolation=cv2.INTER_LINEAR)
        ground = self._clean(score >= float(self._p("ground_threshold")))

        # Obstacle cloud (base_link, z=0), stamped at the source frame for latency comp.
        mx, my = self._mask_to_obstacle_cells(ground, w, h)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = str(self._p("obstacle_frame"))
        pts = np.stack([mx, my, np.zeros_like(mx)], axis=1).astype(np.float32)
        self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, pts))

        if self.pub_debug.get_subscription_count() > 0:
            self._publish_debug(cv_img, ground, msg.header)

    def _publish_debug(self, frame, ground, src_header):
        """Green where ground, red where not, blended onto the frame."""
        tint = np.where(ground[..., None], (0, 255, 0), (0, 0, 255)).astype(np.float32)
        a = float(np.clip(self._p("overlay_alpha"), 0.0, 1.0))
        blended = ((1.0 - a) * frame.astype(np.float32) + a * tint).astype(np.uint8)
        out = self.bridge.cv2_to_imgmsg(blended, encoding="bgr8")
        out.header = src_header
        self.pub_debug.publish(out)


def main():
    rclpy.init()
    node = GroundSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
