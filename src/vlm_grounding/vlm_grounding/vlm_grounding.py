import json
import os
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from google import genai
from google.genai import types
from PIL import Image as PILImage
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hint_interfaces.action import GroundDescription
from sensor_msgs.msg import CompressedImage, Image, RegionOfInterest

MODEL_ID = "gemini-robotics-er-1.6-preview"

_BBOX_PROMPT = (
    'Return a single bounding box for: "{description}". '
    "Return [] if the described region is not visible. "
    "JSON only — no markdown fencing: "
    '[{{"box_2d": [ymin, xmin, ymax, xmax], "label": "<label>"}}] '
    "normalized to 0-1000, integer values only."
)


class VLMGroundingNode(Node):
    def __init__(self):
        super().__init__("vlm_grounding_node")

        self.declare_parameter("api_key_path", "")
        self.declare_parameter("model_id", MODEL_ID)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("api_timeout", 10.0)

        self._client = genai.Client(api_key=self._load_api_key())
        self._bridge = CvBridge()
        self._goal_lock = threading.Lock()
        self._latest_compressed: CompressedImage | None = None

        self._debug_pub = self.create_publisher(Image, "~/debug", 10)
        self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self._camera_cb,
            1,
        )

        self._action_server = ActionServer(
            self,
            GroundDescription,
            "~/ground_description",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("VLM grounding node ready — call ~/ground_description.")

    # ------------------------------------------------------------------
    # Action callbacks

    def _goal_cb(self, goal_request):
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn("Rejecting goal — a grounding call is already running.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            self._goal_lock.release()

    # ------------------------------------------------------------------
    # Camera subscriber

    def _camera_cb(self, msg):
        self._latest_compressed = msg

    # ------------------------------------------------------------------
    # Main execution

    def _run(self, goal_handle):
        goal = goal_handle.request

        self._publish_feedback(goal_handle, "RUNNING")

        # Resolve image source to a cv2 BGR frame.
        if goal.image.width > 0:
            cv_bgr = self._bridge.imgmsg_to_cv2(goal.image, desired_encoding="bgr8")
            stamp = goal.image.header.stamp
        elif self._latest_compressed is not None:
            cv_bgr = self._bridge.compressed_imgmsg_to_cv2(
                self._latest_compressed, desired_encoding="bgr8"
            )
            stamp = self._latest_compressed.header.stamp
        else:
            return self._abort(
                goal_handle,
                "No image provided and no camera frame received yet.",
            )

        pil_img = PILImage.fromarray(cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB))

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        prompt = _BBOX_PROMPT.format(description=goal.description)
        try:
            raw = self._call_api(pil_img, prompt)
        except TimeoutError as e:
            return self._abort(goal_handle, str(e))
        except Exception as e:
            return self._abort(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        boxes = self._parse_boxes(raw)
        if not boxes:
            return self._abort(
                goal_handle, f'No match found for: "{goal.description}"'
            )

        box = boxes[0]
        h, w = cv_bgr.shape[:2]
        roi = self._box_to_roi(box["box_2d"], w, h)
        label = box.get("label", goal.description)

        self._publish_debug(cv_bgr, stamp, roi, label)

        result = GroundDescription.Result()
        result.success = True
        result.message = label
        result.roi = roi
        result.stamp = stamp
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # API

    def _call_api(self, pil_img, prompt):
        timeout = float(self._p("api_timeout"))
        result = [None]
        error = [None]

        def _call():
            try:
                result[0] = self._client.models.generate_content(
                    model=self._p("model_id"),
                    contents=[pil_img, prompt],
                    config=types.GenerateContentConfig(
                        temperature=float(self._p("temperature")),
                        thinking_config=types.ThinkingConfig(thinking_budget=0),
                    ),
                )
            except Exception as e:
                error[0] = e

        t = threading.Thread(target=_call, daemon=True)
        t.start()
        t.join(timeout=timeout)

        if t.is_alive():
            raise TimeoutError(f"Gemini API did not respond within {timeout}s")
        if error[0]:
            raise error[0]
        return result[0].text

    # ------------------------------------------------------------------
    # Helpers

    def _parse_boxes(self, raw):
        text = raw.strip()
        # Strip markdown fencing if the model ignores the prompt instruction.
        lines = text.splitlines()
        for i, line in enumerate(lines):
            if line.strip() == "```json":
                text = "\n".join(lines[i + 1:])
                text = text.split("```")[0]
                break
        try:
            data = json.loads(text)
            if isinstance(data, list):
                return data
        except json.JSONDecodeError:
            self.get_logger().warn(f"Could not parse JSON response: {raw!r}")
        return []

    def _box_to_roi(self, box_2d, img_w, img_h):
        ymin, xmin, ymax, xmax = box_2d
        x0 = max(0, int(min(xmin, xmax) / 1000.0 * img_w))
        y0 = max(0, int(min(ymin, ymax) / 1000.0 * img_h))
        x1 = min(img_w, int(max(xmin, xmax) / 1000.0 * img_w))
        y1 = min(img_h, int(max(ymin, ymax) / 1000.0 * img_h))
        roi = RegionOfInterest()
        roi.x_offset = x0
        roi.y_offset = y0
        roi.width = max(0, x1 - x0)
        roi.height = max(0, y1 - y0)
        return roi

    def _publish_debug(self, cv_bgr, stamp, roi, label):
        try:
            frame = cv_bgr.copy()
            x, y, w, h = roi.x_offset, roi.y_offset, roi.width, roi.height
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x, max(y - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out.header.stamp = stamp
            self._debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"Debug publish failed: {e}")

    def _publish_feedback(self, goal_handle, state):
        fb = GroundDescription.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _abort(self, goal_handle, message):
        self.get_logger().warn(message)
        result = GroundDescription.Result()
        result.success = False
        result.message = message
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle):
        goal_handle.canceled()
        result = GroundDescription.Result()
        result.success = False
        result.message = "Cancelled"
        return result

    def _load_api_key(self):
        path = self._p("api_key_path")
        if path:
            with open(path, "r") as f:
                return f.read().strip()
        key = os.environ.get("GEMINI_API_KEY", "")
        if not key:
            raise RuntimeError(
                "No Gemini API key found. "
                "Set the api_key_path parameter or the GEMINI_API_KEY env var."
            )
        return key

    def _p(self, name):
        return self.get_parameter(name).value


def main(args=None):
    rclpy.init(args=args)
    node = VLMGroundingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
