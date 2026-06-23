"""Depth Anything node running an ONNX model via OpenVINO.

Targets the Intel iGPU by default (device="AUTO" picks GPU, falls back to CPU).
Replicates DA3's preprocessing (RGB + ImageNet normalization, fixed input size
baked into the ONNX) and reuses the same depth visualization as the torch node.

Export the ONNX first with scripts/export_onnx.py, then point `model_path` at it.
"""

import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from openvino.runtime import Core

# ImageNet normalization, matching DA3's InputProcessor (RGB order).
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class DepthAnything(Node):
    def __init__(self):
        super().__init__("depth_anything_node")
        self.bridge = CvBridge()

        default_model_path = os.path.join(
            get_package_share_directory("depth_anything"), "models", "da3-base.onnx"
        )
        model_path = (
            self.declare_parameter("model_path", default_model_path)
            .get_parameter_value()
            .string_value
        )
        model_path = os.path.expanduser(os.path.expandvars(model_path))

        # "AUTO" -> iGPU when available, else CPU. Use "GPU" to force the iGPU,
        # "CPU" to force CPU.
        device = (
            self.declare_parameter("device", "AUTO").get_parameter_value().string_value
        )

        # Percentile clipping: normalize each frame between its p_low and p_high
        # percentiles instead of a fixed range. This handles DA3's scale drift between
        # frames (monocular relative depth has no consistent absolute scale).
        self._p_low = (
            self.declare_parameter("percentile_low", 2.0).get_parameter_value().double_value
        )
        self._p_high = (
            self.declare_parameter("percentile_high", 98.0).get_parameter_value().double_value
        )
        # EMA alpha on the *normalized* map (after percentile clipping).
        # Lower = smoother but more lag. 0.3 is a good starting point.
        self._ema_alpha = (
            self.declare_parameter("ema_alpha", 1.0).get_parameter_value().double_value
        )
        self._frame_count = 0
        self._prev_normed = None

        self.get_logger().info(
            f"Loading ONNX model '{model_path}' on device '{device}'"
        )

        core = Core()
        model = core.read_model(model_path)
        # LATENCY: optimize for the fastest single-inference time (one live stream),
        # rather than batched throughput.
        config = {"PERFORMANCE_HINT": "LATENCY"}
        if device in ("GPU", "AUTO"):
            # Let the iGPU run in fp16 for speed.
            config["INFERENCE_PRECISION_HINT"] = "f16"
        self.compiled_model = core.compile_model(
            model, device_name=device, config=config
        )
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)

        # Fixed input size baked into the ONNX: shape is (1, 3, H, W).
        _, _, self.in_h, self.in_w = (int(d) for d in self.input_layer.shape)
        self.get_logger().info(f"Model input size: {self.in_w}x{self.in_h} (WxH)")

        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.callback, 1
        )
        self.pub_distance = self.create_publisher(Image, "/camera/depth/image_raw", 10)

        self.get_logger().info("OpenVINO Depth Node Ready!")

    def _preprocess(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        # Straight resize to the model's fixed input. Since the camera resolution
        # is constant, pick export H/W matching its aspect ratio to avoid distortion.
        rgb = cv2.resize(rgb, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        rgb = rgb.astype(np.float32) / 255.0
        rgb = (rgb - _MEAN) / _STD
        chw = np.transpose(rgb, (2, 0, 1))  # HWC -> CHW
        return chw[np.newaxis, ...]  # (1, 3, H, W)

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        inp = self._preprocess(cv_img)
        result = self.compiled_model([inp])[self.output_layer]
        raw_output = np.squeeze(result)  # -> (H, W)

        # Per-frame percentile normalization — adapts to DA3's scale drift per frame.
        lo = np.percentile(raw_output, self._p_low)
        hi = np.percentile(raw_output, self._p_high)
        normed = np.clip((raw_output - lo) / (hi - lo + 1e-5), 0.0, 1.0)

        # EMA on the *normalized* map for temporal smoothing.
        if self._prev_normed is None:
            self._prev_normed = normed
        else:
            self._prev_normed = self._ema_alpha * normed + (1.0 - self._ema_alpha) * self._prev_normed

        self._frame_count += 1
        if self._frame_count % 30 == 1:
            self.get_logger().info(
                f"raw depth range: min={float(raw_output.min()):.3f} "
                f"max={float(raw_output.max()):.3f} "
                f"clip=[{lo:.3f}, {hi:.3f}]"
            )

        depth_u8 = (self._prev_normed * 255).astype(np.uint8)
        depth_image = cv2.bitwise_not(depth_u8)  # black = close, white = far
        self.pub_distance.publish(self.bridge.cv2_to_imgmsg(depth_image, "mono8"))


def main():
    rclpy.init()
    node = DepthAnything()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
