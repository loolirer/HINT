import cv2
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from hint_interfaces.action import VisualQuestion
from sensor_msgs.msg import Image

from gemini_robotics_er.gemini_base import GeminiActionNode


class VisualQuestionNode(GeminiActionNode):
    """Answers a yes/no question about a camera frame with a VLM.

    Intended as a sanity-check fallback for the approach pipeline: when a
    sequential behavior fails (e.g. the tracker lost the target on close
    approach), the tree can ask "is the target still in view?" and use the
    answer to decide between retrying and giving up.

    The result carries three states, not two: ``answered`` is False whenever
    the check could not run (no frame, decode error, timeout, API error,
    unparseable reply); otherwise ``affirmative`` holds the yes/no verdict.
    Every path — verdict or "couldn't determine" — *succeeds* at the ROS
    layer, so the ``rationale`` always reaches the caller and a sanity check
    that cannot run never masquerades as a "no".
    """

    def __init__(self):
        super().__init__("visual_question_node")

        self._debug_pub = self.create_publisher(Image, "~/debug", 10)

        self._action_server = ActionServer(
            self,
            VisualQuestion,
            "~/ask",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Visual question node ready — call ~/ask.")

    # ------------------------------------------------------------------
    # Main execution

    def _run(self, goal_handle):
        goal = goal_handle.request

        self._publish_feedback(goal_handle, "RUNNING")

        # A sanity check that cannot run must not masquerade as a "no": every
        # "couldn't determine" path (empty question, no frame, decode error,
        # timeout, API error, unparseable reply) completes with answered=False
        # so the caller can abstain rather than treat it as a verdict.
        if not goal.question.strip():
            return self._unknown(goal_handle, "Empty question.")

        compressed, stamp = self._resolve_frame(goal.stamp)
        if compressed is None:
            return self._unknown(goal_handle, "No camera frame received yet.")

        try:
            cv_bgr, pil_img = self._frame_to_pil(compressed)
        except Exception as e:
            return self._unknown(goal_handle, f"Image conversion failed: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        prompt = self._fill_prompt("visual_question.txt", question=goal.question)
        try:
            raw = self._call_api([prompt, pil_img])
        except TimeoutError as e:
            return self._unknown(goal_handle, str(e))
        except Exception as e:
            return self._unknown(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        data = self._parse_json(raw)
        if not isinstance(data, dict) or "answer" not in data:
            return self._unknown(goal_handle, f"Unparseable answer: {raw!r}")

        answer = str(data.get("answer", "")).strip().lower()
        rationale = str(data.get("rationale", "")).strip()
        is_yes = answer.startswith("y")

        self._publish_debug(cv_bgr, stamp, goal.question, True, is_yes, rationale)
        self.get_logger().info(
            f'Q: "{goal.question}" → {"YES" if is_yes else "NO"} ({rationale})'
        )

        # A genuine verdict: goal succeeds at the ROS layer, answered=True and
        # the yes/no lives in result.affirmative.
        result = VisualQuestion.Result()
        result.answered = True
        result.affirmative = is_yes
        result.rationale = rationale or ("yes" if is_yes else "no")
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Helpers

    def _publish_debug(self, cv_bgr, stamp, question, answered, is_yes,
                       rationale):
        try:
            frame = cv_bgr.copy()
            h, w = frame.shape[:2]

            # Aesthetic answer colors (BGR): emerald / alizarin / sunflower.
            if not answered:
                color = (15, 196, 241)
            elif is_yes:
                color = (113, 204, 46)
            else:
                color = (60, 76, 231)
            text_color = (56, 44, 33)  # dark charcoal, reads on all three

            font = cv2.FONT_HERSHEY_SIMPLEX
            scale, th = 0.6, 1
            pad = max(6, round(h / 40))
            (_, cap_h), base = cv2.getTextSize("Ay", font, scale, th)
            line_h = cap_h + base + max(2, round(h / 120))

            # Colored frame border.
            border = max(1, round(h / 25))
            cv2.rectangle(frame, (0, 0), (w - 1, h - 1), color, thickness=border)

            # Text boxes over the frame: question on top, rationale on bottom,
            # both wrapped and drawn on solid answer-colored rectangles.
            q_lines = self._wrap_text(question, font, scale, th, w - 2 * pad)
            r_lines = self._wrap_text(rationale, font, scale, th, w - 2 * pad)
            self._draw_box(frame, q_lines, 0, color, text_color, pad, line_h,
                           cap_h, font, scale, th)
            r_box_h = len(r_lines) * line_h + 2 * pad
            self._draw_box(frame, r_lines, h - r_box_h, color, text_color, pad,
                           line_h, cap_h, font, scale, th)

            out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out.header.stamp = stamp
            self._debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"Debug publish failed: {e}")

    @staticmethod
    def _draw_box(img, lines, y0, color, text_color, pad, line_h, cap_h,
                  font, scale, th):
        w = img.shape[1]
        box_h = len(lines) * line_h + 2 * pad
        cv2.rectangle(img, (0, y0), (w, y0 + box_h), color, thickness=-1)
        y = y0 + pad + cap_h
        for line in lines:
            cv2.putText(img, line, (pad, y), font, scale, text_color, th,
                        cv2.LINE_AA)
            y += line_h

    @staticmethod
    def _wrap_text(text, font, scale, thickness, max_width):
        lines, current = [], ""
        for word in text.split():
            candidate = f"{current} {word}".strip()
            width = cv2.getTextSize(candidate, font, scale, thickness)[0][0]
            if width <= max_width or not current:
                current = candidate
            else:
                lines.append(current)
                current = word
        if current:
            lines.append(current)
        return lines or [""]

    def _publish_feedback(self, goal_handle, state):
        fb = VisualQuestion.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _unknown(self, goal_handle, message):
        # "Couldn't determine" is a valid completed result, not an error: the
        # goal succeeds with answered=False so the rationale reaches the caller
        # and the tree can apply its abstain policy.
        self.get_logger().warn(message)
        result = VisualQuestion.Result()
        result.answered = False
        result.affirmative = False
        result.rationale = message
        goal_handle.succeed()
        return result

    def _cancel(self, goal_handle):
        goal_handle.canceled()
        result = VisualQuestion.Result()
        result.answered = False
        result.affirmative = False
        result.rationale = "Cancelled"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VisualQuestionNode()
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
