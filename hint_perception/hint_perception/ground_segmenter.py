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
from sensor_msgs.msg import CompressedImage, Image

# ImageNet normalization (RGB order) — the standard for HF segmentation backbones.
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# ADE20K 'floor' class id; widen per environment (28 rug, 6 road, 13 earth).
_DEFAULT_GROUND_IDS = [3]

# Latest-frame-wins: inference is slower than the camera, so depth > 1 buys a backlog
# of already-driven-past frames. Depth 1 + BEST_EFFORT drops stale frames instead.
_LATEST_FRAME_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


class GroundSegmenter(Node):
    def __init__(self):
        super().__init__("ground_segmenter_node")
        self.bridge = CvBridge()

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
        self.declare_parameter("ground_threshold", 0.5)
        self.declare_parameter("morph_kernel", 3)   # processing-grid px
        self.declare_parameter("score_mode", "argmax")
        self.declare_parameter("proc_scale", 4)

        self.get_logger().info(f"Loading ONNX model '{model_path}' on device '{device}'")
        core = Core()
        model = core.read_model(model_path)
        # Read the model spatial dims before the preprocessor rewrites the input.
        _, _, self.in_h, self.in_w = (int(d) for d in model.input(0).shape)
        self.get_logger().info(f"Model input size: {self.in_w}x{self.in_h} (WxH)")

        # Fold preprocessing (BGR->RGB, resize, /255 + ImageNet normalize, HWC->CHW) into
        # the compiled graph so it runs on-device; the callback feeds a raw uint8 BGR frame.
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

        # Naive by design — infer and publish synchronously, no hang detection. If the
        # inference device wedges, an external monitor (hint_bringup's segmenter_watchdog
        # + launch respawn) restarts the process; a same-process watchdog would freeze too.
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self.callback, _LATEST_FRAME_QOS,
        )
        self.pub_mask = self.create_publisher(Image, "/camera/ground", 1)

        self.get_logger().info("OpenVINO Ground Segmenter (image -> ground mask) ready!")

    def _p(self, name):
        return self.get_parameter(name).value

    def _resolve_model(self, name):
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

    def _detect_class_axis(self):
        # Spatial axes are the pair whose aspect ratio matches the input's; the leftover
        # axis is the class axis. Ratio (not exact) match — heads emit at a reduced stride.
        shape = [int(d) for d in self.output_layer.partial_shape.get_max_shape()]
        self.get_logger().info(f"Model output shape: {tuple(shape)}")
        if len(shape) == 3:  # (1, H, W) — score map or argmaxed class map
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

    def _ids(self):
        return np.asarray(self._p("ground_class_ids"), dtype=np.int64)

    def _ground_score(self, out):
        out = np.squeeze(out, axis=0) if out.ndim == 4 and out.shape[0] == 1 else out

        if self._class_axis is None or out.ndim == 2:
            arr = np.squeeze(out)
            if not np.issubdtype(arr.dtype, np.floating):  # argmaxed class map
                return np.isin(arr, self._ids()).astype(np.float32)
            if arr.min() < 0.0 or arr.max() > 1.0:  # logits -> probability
                arr = 1.0 / (1.0 + np.exp(-arr))
            return arr.astype(np.float32)

        axis = self._class_axis - 1  # batch dim was squeezed off
        if out.shape[axis] == 1:  # single-channel score in a class-shaped tensor
            self._class_axis = None
            return self._ground_score(out)

        if str(self._p("score_mode")).lower() == "argmax":
            # Ground iff the winning class is a ground class; skips the full-class softmax.
            return np.isin(out.argmax(axis=axis), self._ids()).astype(np.float32)

        e = np.exp(out - out.max(axis=axis, keepdims=True))  # stable softmax
        prob = e / e.sum(axis=axis, keepdims=True)
        ids = self._ids()
        ids = ids[(ids >= 0) & (ids < out.shape[axis])]  # ignore out-of-range ids
        if ids.size == 0:
            return np.zeros(np.delete(out.shape, axis), dtype=np.float32)
        return np.take(prob, ids, axis=axis).sum(axis=axis).astype(np.float32)

    def _clean(self, mask):
        k = int(self._p("morph_kernel"))
        if k <= 1:
            return mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        return m.astype(bool)

    def callback(self, msg):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Skipping undecodable frame: {e}")
            return
        if cv_img is None:  # truncated JPEG decodes to None without raising
            self.get_logger().warn("Skipping undecodable frame (decode returned None)")
            return

        out = self.compiled_model([cv_img[np.newaxis]])[self.output_layer]
        score = self._ground_score(np.asarray(out))

        # Resample to the processing grid, then threshold — cutting first and resizing the
        # binary mask staircases every boundary.
        h, w = cv_img.shape[:2]
        scale = max(1, int(self._p("proc_scale")))
        ws, hs = max(2, w // scale), max(2, h // scale)
        score = cv2.resize(score, (ws, hs), interpolation=cv2.INTER_LINEAR)
        ground = self._clean(score >= float(self._p("ground_threshold")))

        self._publish_mask(ground, msg.header)

    def _publish_mask(self, ground, src_header):
        out = self.bridge.cv2_to_imgmsg(ground.astype(np.uint8) * 255, encoding="mono8")
        out.header = src_header  # source-frame stamp -> navigation-side latency compensation
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
