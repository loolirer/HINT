"""Ground segmenter -> obstacle point cloud (fused, one node) for the Nav2 costmap.

Runs a semantic-segmentation ONNX model via OpenVINO (Intel iGPU by default), mirroring
``depth_anything``'s inference plumbing (same ``Core`` / ``compile_model`` setup; the
ImageNet preprocessing is folded into the graph). Consumes ``/camera/image_raw/compressed`` — the node runs
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

It also publishes the binary ground mask on ``/camera/ground/mask`` (``mono8``, 255 = ground)
at the processing-grid resolution (camera aspect, 1/``proc_scale`` of camera res) —
consumed by ``hint_navigation``'s trajectory clipping (normalized coords, so any res
works) and by ``visual_debug``'s overlay (which resizes it to the frame). Visual debug
(the green/red overlay etc.) lives in the ``visual_debug`` node, not here.

Preprocessing (BGR->RGB, resize, ImageNet normalize, HWC->CHW) is folded into the
compiled model via OpenVINO's PrePostProcessor, so it runs on the inference device and
the callback feeds the raw uint8 BGR frame.

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
from openvino.preprocess import ColorFormat, PrePostProcessor, ResizeAlgorithm
from openvino.runtime import Core, Layout, Type
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

        # Model is chosen by NAME (with or without the .onnx suffix), always resolved
        # against the package's models/ dir — never a full path. Pass e.g.
        # "segformer-b2-ade", or a prefix like "segformer-b2" (first match wins).
        self._models_dir = os.path.join(
            get_package_share_directory("hint_perception"), "models")
        model_name = (
            self.declare_parameter("model", "segformer-b0-ade")
            .get_parameter_value().string_value
        )
        model_path = self._resolve_model(model_name)
        device = (
            self.declare_parameter("device", "AUTO").get_parameter_value().string_value
        )
        self.declare_parameter("ground_class_ids", _DEFAULT_GROUND_IDS)
        self.declare_parameter("ground_threshold", 0.5)  # ground-probability cut
        # Hole-fill / speck-drop kernel, in PROCESSING-grid pixels (the camera-aspect
        # grid at 1/proc_scale of camera res) — 3 there ≈ the old 7 at full res.
        self.declare_parameter("morph_kernel", 3)
        # Ground-score reduction, live-adjustable:
        #   "softmax" — summed ground-class probabilities, cut at ground_threshold;
        #   "argmax"  — pixel is ground iff its argmax class is in ground_class_ids.
        #               Skips the full-class softmax (big CPU cut); ground_threshold
        #               is inert in this mode.
        self.declare_parameter("score_mode", "argmax")
        # All post-processing (threshold, morphology, BEV warp) and the published mask
        # run on a camera-aspect grid at 1/proc_scale of the camera resolution — the
        # score comes out of the model at a coarse stride anyway, so full-res work is
        # wasted. Consumers are resolution-agnostic (normalized coords / they resize).
        self.declare_parameter("proc_scale", 4)

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

        self.get_logger().info(f"Loading ONNX model '{model_path}' on device '{device}'")
        core = Core()
        model = core.read_model(model_path)
        # Model spatial dims, read BEFORE the preprocessor rewrites the input to a
        # dynamic-shape u8 tensor.
        _, _, self.in_h, self.in_w = (int(d) for d in model.input(0).shape)
        self.get_logger().info(f"Model input size: {self.in_w}x{self.in_h} (WxH)")

        # Fold the whole CPU preprocessing chain (BGR->RGB, resize to the model dims,
        # /255 + ImageNet normalize, HWC->CHW) into the compiled graph so it runs on
        # the inference device: the callback feeds the raw uint8 BGR frame directly.
        ppp = PrePostProcessor(model)
        (ppp.input().tensor()
            .set_element_type(Type.u8)
            .set_layout(Layout("NHWC"))
            .set_color_format(ColorFormat.BGR)
            .set_spatial_dynamic_shape())
        (ppp.input().preprocess()
            .convert_element_type(Type.f32)
            .convert_color(ColorFormat.RGB)
            .resize(ResizeAlgorithm.RESIZE_LINEAR)
            .mean([m * 255.0 for m in _MEAN])
            .scale([s * 255.0 for s in _STD]))
        ppp.input().model().set_layout(Layout("NCHW"))
        model = ppp.build()

        config = {"PERFORMANCE_HINT": "LATENCY"}
        if device in ("GPU", "AUTO"):
            config["INFERENCE_PRECISION_HINT"] = "f16"
        self.compiled_model = core.compile_model(model, device_name=device, config=config)
        self.output_layer = self.compiled_model.output(0)

        self._class_axis = self._detect_class_axis()

        # Naive by design: this node just infers and publishes, synchronously, in the
        # callback. It has NO notion that inference can hang or that it can stop — if the
        # inference device wedges (e.g. an Intel iGPU hang), an EXTERNAL monitor
        # (segmenter_watchdog in hint_bringup, + launch respawn) notices the mask going
        # silent and kills/restarts the process. Keeping the node dumb avoids the in-node
        # watchdog trap: a GIL-holding native hang freezes any same-process watchdog too.
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self.callback, _LATEST_FRAME_QOS,
        )
        # Reliable pub so a best-effort costmap observation sub is still compatible.
        self.pub_obstacles = self.create_publisher(PointCloud2, "/ground/obstacles", 5)
        # Binary ground mask (255 = ground, 0 = not) at the processing-grid resolution,
        # header inherited from the source frame. trajectory_navigator uses it to clip
        # the VLM pixel trajectory to the ground (via normalized coords, res-agnostic).
        self.pub_mask = self.create_publisher(Image, "/camera/ground/mask", 1)

        self.get_logger().info("OpenVINO Ground Segmenter (mask + obstacle cloud) ready!")

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Model resolution + device (re)compile

    def _resolve_model(self, name):
        """Resolve a model NAME to an .onnx file inside the package models/ dir.

        Accepts "segformer-b2-ade", "segformer-b2-ade.onnx", or a prefix like
        "segformer-b2" (first sorted match wins). Any path components are stripped — the
        search is always models/ — so the model can't be pointed elsewhere by accident.
        Raises FileNotFoundError (listing what's available) on no match.
        """
        base = os.path.basename(name.strip())
        if base.endswith(".onnx"):
            base = base[: -len(".onnx")]
        available = sorted(
            f for f in os.listdir(self._models_dir) if f.endswith(".onnx")
        ) if os.path.isdir(self._models_dir) else []
        # Exact "<name>.onnx" first, then a "<name>*.onnx" prefix match.
        for cand in [f"{base}.onnx"] + [f for f in available if f.startswith(base)]:
            path = os.path.join(self._models_dir, cand)
            if os.path.isfile(path):
                return path
        raise FileNotFoundError(
            f"No .onnx model matching '{name}' in {self._models_dir}. "
            f"Available: {available or 'none'}"
        )

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

        if str(self._p("score_mode")).lower() == "argmax":
            # Fast path: ground iff the winning class is a ground class. Skips the
            # full-class softmax; ground_threshold plays no role here (the map is
            # already binary, and 0/1 passes any threshold in (0, 1]).
            return np.isin(out.argmax(axis=axis), self._ids()).astype(np.float32)

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
    # Callback (preprocessing lives inside the compiled model — see __init__)

    def callback(self, msg):
        # Naive synchronous inference: decode -> infer -> mask + obstacle cloud, right here.
        # No hang detection, no recovery — if the device wedges, an external monitor
        # (hint_bringup's segmenter_watchdog + launch respawn) restarts this process.
        # A truncated JPEG (best-effort WiFi stream) decodes to None or raises; skip the
        # frame instead of crashing — that's normal operation, not device recovery.
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Skipping undecodable frame: {e}")
            return
        if cv_img is None:
            self.get_logger().warn("Skipping undecodable frame (decode returned None)")
            return

        out = self.compiled_model([cv_img[np.newaxis]])[self.output_layer]
        score = self._ground_score(np.asarray(out))

        # Resample the score to the PROCESSING grid — camera aspect (the head's stride may
        # not match it) at 1/proc_scale of camera res — then threshold, never the reverse:
        # cutting first and resizing the binary mask staircases every boundary. Everything
        # downstream (morphology, BEV warp, published mask) stays on this small grid; the
        # camera model in _ground_to_pixels is resolution-invariant.
        h, w = cv_img.shape[:2]
        scale = max(1, int(self._p("proc_scale")))
        ws, hs = max(2, w // scale), max(2, h // scale)
        score = cv2.resize(score, (ws, hs), interpolation=cv2.INTER_LINEAR)
        ground = self._clean(score >= float(self._p("ground_threshold")))

        # Binary ground mask (image space, processing-grid res), for the trajectory
        # ground-clipping consumer (which maps normalized coords into mask.shape).
        self._publish_mask(ground, msg.header)

        # Obstacle cloud (base_link, z=0), stamped at the source frame for latency comp.
        mx, my = self._mask_to_obstacle_cells(ground, ws, hs)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = str(self._p("obstacle_frame"))
        pts = np.stack([mx, my, np.zeros_like(mx)], axis=1).astype(np.float32)
        self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def _publish_mask(self, ground, src_header):
        """Publish the binary ground mask (mono8, 255=ground) at processing-grid res."""
        out = self.bridge.cv2_to_imgmsg(ground.astype(np.uint8) * 255, encoding="mono8")
        out.header = src_header
        self.pub_mask.publish(out)


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
