import json

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from hint_interfaces.action import Reason

from gemini_robotics_er.gemini_base import GeminiActionNode


class ReasonerNode(GeminiActionNode):
    """A generic text-(and-optional-image)-in / JSON-out LLM reasoner.

    It reasons over the text it is handed and, when the goal carries ``images``,
    over those frames too. It is the model call the semantic mission planner
    leans on as its **director**: each cycle the planner hands it the before/after
    frames of the move just executed plus the running narrative, and it assesses
    the move against what it actually sees and emits the next instruction. With
    an empty ``images`` list it degrades to pure text reasoning, so any text-only
    caller still works unchanged.

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

        # Output control (quality vs validity) when a schema is requested, same
        # knob as trajectory_planner, live-adjustable:
        #   "json"   (default) JSON mode — valid JSON, keeps reasoning freedom;
        #   "off"    unconstrained — best quality, but a reply can be unparseable;
        #   "schema" constrained decoding to the schema (enum-enforced), but the
        #            hard grammar can cost reasoning quality.
        self.declare_parameter("structured_output", "json")

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
        mode = str(self._p("structured_output")).lower()
        prompt = goal.prompt
        response_schema = None
        json_output = False
        if want_json:
            # "schema": constrain to the schema (only when it's real JSON). Else
            # ("json"/"off", or a loose shape string) the decoder can't enforce the
            # shape, so hint it in the prompt — and gate validity with JSON mode.
            schema_obj = (self._as_response_schema(goal.schema)
                          if mode == "schema" else None)
            if schema_obj is not None:
                response_schema = schema_obj
            else:
                prompt = (
                    f"{goal.prompt}\n\n"
                    "Respond with JSON only — no prose, no markdown fencing — "
                    f"matching this shape:\n{goal.schema}"
                )
                json_output = (mode == "json")

        # Optional grounding frames (e.g. the mission director's before/after
        # views). Empty list -> pure text reasoning, exactly as before. Order is
        # preserved so the prompt can refer to "the first / second image".
        pil_frames = []
        for img in goal.images:
            try:
                _, pil_img = self._frame_to_pil(img)
                pil_frames.append(pil_img)
            except Exception as e:  # noqa: BLE001 — skip an unreadable frame
                self.get_logger().warn(f"Skipping an unreadable image: {e}")
        # Text before images (Gemini best practice); image order preserved so the
        # prompt can refer to "the first / second image".
        contents = [prompt] + pil_frames

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        try:
            raw = self._call_api(contents, response_schema=response_schema,
                                 json_output=json_output)
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

    @staticmethod
    def _as_response_schema(schema_str):
        """Return a JSON-schema dict for constrained decoding when the goal's
        ``schema`` is real JSON (a dict), else ``None`` — loose shape strings like
        ``'{"x": bool}'`` aren't valid JSON and stay as a prompt hint."""
        try:
            obj = json.loads(schema_str)
        except (json.JSONDecodeError, TypeError):
            return None
        return obj if isinstance(obj, dict) else None

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
