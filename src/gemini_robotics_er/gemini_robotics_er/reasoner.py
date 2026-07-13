import json

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from hint_interfaces.action import Reason

from gemini_robotics_er.gemini_base import GeminiActionNode


class ReasonerNode(GeminiActionNode):
    """A generic text-in / JSON-out LLM reasoner.

    Unlike its siblings (``description_detector``, ``visual_question``,
    ``trajectory_planner``) this node does **not** look at the camera: it
    reasons purely over the text it is handed. It is the model call the
    semantic mission planner leans on for the parts a single-frame VLM check
    can't answer — judging from a run log whether a task was completed,
    revising a trajectory-planner prompt after a failure, and rolling an area's
    log up into a summary. Perception stays in the VLM nodes (whose grounded
    outputs land in the log); this node reasons *over* that log.

    It inherits ``GeminiActionNode``'s API client, timeout-guarded call and
    single-goal lifecycle. The inherited camera ring buffer is simply unused.

    Contract: the goal carries a ``prompt`` and an optional ``schema`` (a JSON
    shape the reply must match). When a schema is given the reply is parsed and
    re-serialized so the caller gets canonical JSON; when it is empty the raw
    text is returned. A genuine reply *succeeds* (BT ``SUCCESS``); anything that
    stops the reasoning from running (empty prompt, timeout, API error,
    unparseable JSON) *aborts* (BT ``FAILURE``) with the reason in ``response``.

    ``model_id`` defaults to the Robotics-ER model like the rest of the package,
    but for pure text reasoning it can be pointed at a general Gemini model via
    the parameter.
    """

    def __init__(self):
        super().__init__("reasoner_node")

        self._action_server = ActionServer(
            self,
            Reason,
            "~/reason",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Reasoner node ready — call ~/reason.")

    # ------------------------------------------------------------------
    # Main execution

    def _run(self, goal_handle):
        goal = goal_handle.request

        self._publish_feedback(goal_handle, "RUNNING")

        if not goal.prompt.strip():
            return self._fail(goal_handle, "Empty prompt.")

        want_json = bool(goal.schema.strip())
        prompt = goal.prompt
        if want_json:
            prompt = (
                f"{goal.prompt}\n\n"
                "Respond with JSON only — no prose, no markdown fencing — "
                f"matching this shape:\n{goal.schema}"
            )

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        try:
            raw = self._call_api([prompt])
        except TimeoutError as e:
            return self._fail(goal_handle, str(e))
        except Exception as e:  # noqa: BLE001 — surfaced to caller as FAILURE
            return self._fail(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        if raw is None or not raw.strip():
            return self._fail(goal_handle, "Empty response from model.")

        if want_json:
            data = self._parse_json(raw)
            if data is None:
                return self._fail(
                    goal_handle, f"Unparseable JSON response: {raw!r}"
                )
            response = json.dumps(data)
        else:
            response = raw.strip()

        self.get_logger().info(f"Reasoned → {response}")

        result = Reason.Result()
        result.success = True
        result.response = response
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Helpers

    def _publish_feedback(self, goal_handle, state):
        fb = Reason.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _fail(self, goal_handle, message):
        # A reasoning call that could not run is a real failure — abort so the
        # BT leaf sees FAILURE and can retry / branch, with the reason carried
        # in response.
        self.get_logger().warn(message)
        result = Reason.Result()
        result.success = False
        result.response = message
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle):
        goal_handle.canceled()
        result = Reason.Result()
        result.success = False
        result.response = "Cancelled"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ReasonerNode()
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
