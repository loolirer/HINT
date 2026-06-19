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

        # Fixed depth range for visualization. Clamping/normalizing against a fixed
        # [depth_min, depth_max] keeps gray levels stable across frames, instead of
        # rescaling per-frame (which flickers). Tune these to your model's raw output
        # range -- the node logs the live min/max periodically to help you calibrate.
        self.depth_min = (
            self.declare_parameter("depth_min", 0.0).get_parameter_value().double_value
        )
        self.depth_max = (
            self.declare_parameter("depth_max", 2.0).get_parameter_value().double_value
        )
        self._frame_count = 0
        self.prev_raw_depth = None
        self.alpha = 0.8  # Smoothing factor: 1.0 = no smoothing, 0.0 = frozen. Tune between 0.2 and 0.8.
        self.diff_threshold = 0.15 * (
            self.depth_max - self.depth_min
        )  # Ignore changes larger than 15% of range

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

        if self.prev_raw_depth is None:
            self.prev_raw_depth = raw_output
        else:
            # Find the absolute difference between this frame and the last frame
            diff = np.abs(raw_output - self.prev_raw_depth)

            # Create a mask of pixels where the change is small enough to be considered "noise"
            noise_mask = diff < self.diff_threshold

            # Apply EMA smoothing ONLY to the noisy pixels.
            # Fast moving pixels update instantly (raw_output), preventing ghosting.
            smoothed_output = np.where(
                noise_mask,
                (self.alpha * raw_output) + ((1.0 - self.alpha) * self.prev_raw_depth),
                raw_output,
            )

            raw_output = smoothed_output
            self.prev_raw_depth = smoothed_output

        # Log the live raw range every ~30 frames so the fixed range can be tuned.
        self._frame_count += 1
        if self._frame_count % 30 == 1:
            self.get_logger().info(
                f"raw depth range: min={float(raw_output.min()):.3f} "
                f"max={float(raw_output.max()):.3f} "
                f"(clamping to [{self.depth_min}, {self.depth_max}])"
            )

        # Fixed-range normalization: stable gray levels across frames.
        rng = self.depth_max - self.depth_min
        clamped = np.clip(raw_output, self.depth_min, self.depth_max)
        normalized_base = ((clamped - self.depth_min) / (rng + 1e-5) * 255).astype(
            np.uint8
        )
        depth_image = cv2.bitwise_not(normalized_base)  # Black = Close, White = Far

        self.pub_distance.publish(self.bridge.cv2_to_imgmsg(depth_image, "mono8"))


def main():
    rclpy.init()
    node = DepthAnything()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
