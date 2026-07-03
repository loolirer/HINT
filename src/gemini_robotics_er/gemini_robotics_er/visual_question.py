import cv2
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from hint_interfaces.action import VisualQuestion
from sensor_msgs.msg import Image

from gemini_robotics_er.gemini_base import GeminiActionNode

# A yes/no sanity-check over a single frame. Kept deliberately strict so the
# answer maps cleanly onto action success/failure, with a one-line rationale
# the behavior tree can log or branch on.
_QUESTION_PROMPT = (
    'Look at the image and answer this yes/no question: "{question}". '
    "Answer only about what is actually visible in this image. "
    "Respond with JSON only — no markdown fencing: "
    '{{"answer": "yes" or "no", "rationale": "<one short sentence>"}}.'
)


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

        prompt = _QUESTION_PROMPT.format(question=goal.question)
        try:
            raw = self._call_api([pil_img, prompt])
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
            if not answered:
                color, verdict = (0, 165, 255), "UNKNOWN"
            elif is_yes:
                color, verdict = (0, 200, 0), "YES"
            else:
                color, verdict = (0, 0, 255), "NO"
            cv2.putText(
                frame,
                f"{verdict}: {question}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
            cv2.putText(
                frame,
                rationale[:80],
                (10, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )
            out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out.header.stamp = stamp
            self._debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"Debug publish failed: {e}")

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
