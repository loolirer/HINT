import cv2
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from hint_interfaces.action import GroundDescription
from sensor_msgs.msg import Image, RegionOfInterest

from gemini_robotics_er.gemini_base import GeminiActionNode

_BBOX_PROMPT = (
    'Return a single bounding box for: "{description}". '
    "Return [] if the described region is not visible. "
    "JSON only — no markdown fencing: "
    '[{{"box_2d": [ymin, xmin, ymax, xmax], "label": "<label>"}}] '
    "normalized to 0-1000, integer values only."
)


class DescriptionDetectorNode(GeminiActionNode):
    def __init__(self):
        super().__init__("description_detector_node")

        self._debug_pub = self.create_publisher(Image, "~/debug", 10)

        self._action_server = ActionServer(
            self,
            GroundDescription,
            "~/ground_description",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            "Description detector ready — call ~/ground_description."
        )

    # ------------------------------------------------------------------
    # Main execution

    def _run(self, goal_handle):
        goal = goal_handle.request

        self._publish_feedback(goal_handle, "RUNNING")

        compressed, stamp = self._resolve_frame(goal.stamp)
        if compressed is None:
            return self._abort(goal_handle, "No camera frame received yet.")

        try:
            cv_bgr, pil_img = self._frame_to_pil(compressed)
        except Exception as e:
            return self._abort(goal_handle, f"Image conversion failed: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        prompt = _BBOX_PROMPT.format(description=goal.description)
        try:
            raw = self._call_api([pil_img, prompt])
        except TimeoutError as e:
            return self._abort(goal_handle, str(e))
        except Exception as e:
            return self._abort(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        data = self._parse_json(raw)
        boxes = data if isinstance(data, list) else []
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
    # Helpers

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


def main(args=None):
    rclpy.init(args=args)
    node = DescriptionDetectorNode()
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
