import json

import rclpy
from rclpy.executors import MultiThreadedExecutor

from hint_interfaces.action import VisualReason

from hint_vlm.gemini.gemini_base import GeminiActionNode


class ReasonerNode(GeminiActionNode):
    def __init__(self):
        super().__init__("visual_reasoner", VisualReason, "~/visual_reason", "response")

        self.declare_parameter("structured_output", "json")

        self.get_logger().info("Reasoner node ready — call ~/visual_reason.")

    def _run(self, goal_handle):
        goal = goal_handle.request

        self._publish_feedback(goal_handle, "RUNNING")

        if not goal.prompt.strip():
            return self._abort(goal_handle, "Empty prompt.")

        want_json = bool(goal.schema.strip())
        mode = str(self._p("structured_output")).lower()
        prompt = goal.prompt
        response_schema = None
        json_output = False
        if want_json:
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

        pil_frames = []
        for img in goal.images:
            try:
                _, pil_img = self._frame_to_pil(img)
                pil_frames.append(pil_img)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Skipping an unreadable image: {e}")
        contents = [prompt] + pil_frames

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        try:
            raw = self._call_api(contents, response_schema=response_schema,
                                 json_output=json_output)
        except TimeoutError as e:
            return self._abort(goal_handle, str(e))
        except Exception as e:  # noqa: BLE001
            return self._abort(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        if raw is None or not raw.strip():
            return self._abort(goal_handle, "Empty response from model.")

        if want_json:
            data = self._parse_json(raw)
            if data is None:
                return self._abort(
                    goal_handle, f"Unparseable JSON response: {raw!r}"
                )
            response = json.dumps(data)
        else:
            response = raw.strip()

        self.get_logger().info(f"Reasoned → {response}")

        result = VisualReason.Result()
        result.response = response
        if goal.images:
            result.stamp = goal.images[-1].header.stamp
        goal_handle.succeed()
        return result

    @staticmethod
    def _as_response_schema(schema_str):
        try:
            obj = json.loads(schema_str)
        except (json.JSONDecodeError, TypeError):
            return None
        return obj if isinstance(obj, dict) else None


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
