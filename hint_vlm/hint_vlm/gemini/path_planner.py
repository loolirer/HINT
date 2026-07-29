import numpy as np
import rclpy
from google.genai import types
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from hint_interfaces.action import PlanVisualPath

from hint_vlm.gemini.gemini_base import GeminiActionNode


class PathPlannerNode(GeminiActionNode):
    def __init__(self):
        super().__init__("path_planner", PlanVisualPath, "~/plan_visual_path", "message")

        self.declare_parameter("min_row", 500)
        self.declare_parameter("n_candidates", 1)
        self.declare_parameter("structured_output", "json")

        self.get_logger().info("Path planner ready — call ~/plan_visual_path.")

    def _run(self, goal_handle):
        goal = goal_handle.request

        self._publish_feedback(goal_handle, "RUNNING")

        if not goal.images:
            return self._abort(goal_handle, "No camera frame supplied in the goal.")
        *past_msgs, current_msg = goal.images
        try:
            _, pil_img = self._frame_to_pil(current_msg)
        except Exception as e:
            return self._abort(goal_handle, f"Image conversion failed: {e}")
        stamp = current_msg.header.stamp
        hist = []
        for msg in past_msgs:
            try:
                _, pil = self._frame_to_pil(msg)
                hist.append(pil)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Skipping an unreadable continuity frame: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        n = max(1, int(self._p("n_candidates")))
        prompt = self._fill_prompt(
            "path_planner.txt", description=goal.description,
            min_row=int(self._p("min_row")), return_spec=self._return_spec(n),
            continuity=self._continuity_text(len(hist)))
        contents = [prompt] + hist + [pil_img]
        mode = str(self._p("structured_output")).lower()
        schema = self._response_schema(n) if mode == "schema" else None
        try:
            raw = self._call_api(contents, response_schema=schema,
                                 json_output=(mode == "json"))
        except TimeoutError as e:
            return self._abort(goal_handle, str(e))
        except Exception as e:
            return self._abort(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        parsed = self._parse_json(raw)
        if parsed is None:
            return self._abort(
                goal_handle,
                f'VLM response for "{goal.description}" was not parseable JSON',
            )
        cand_dicts = self._candidate_dicts(parsed)
        path_cands, first_reason, first_turn = [], "", 0.0
        for i, cd in enumerate(cand_dicts):
            reasoning, points, turn = self._parse_reasoning_points(cd)
            if i == 0:
                first_reason, first_turn = reasoning, turn
            waypoints = self._points_to_waypoints(points)
            if waypoints:
                path_cands.append((reasoning, waypoints, points, turn))

        if path_cands:
            reasoning, waypoints, points, turn = self._select_medoid(path_cands)
            if len(cand_dicts) > 1:
                self.get_logger().info(
                    f"Chose medoid of {len(path_cands)}/{len(cand_dicts)} candidate paths.")
        else:
            reasoning, waypoints, points, turn = first_reason, [], [], first_turn
            self.get_logger().info(f"No waypoints — turn-only/no-op move ({turn:+.0f} deg).")

        result = PlanVisualPath.Result()
        result.message = reasoning or f"{len(waypoints)} waypoint(s), turn {turn:+.0f} deg"
        result.waypoints = waypoints
        result.turn_degrees = float(turn)
        result.stamp = stamp
        goal_handle.succeed()
        return result

    @staticmethod
    def _continuity_text(n_past):
        if n_past <= 0:
            return ""
        return (
            f"{n_past + 1} images are attached, oldest first; the LAST is my CURRENT view — the "
            f"earlier {n_past} are my recent past view(s) from the moves that led here. I read how "
            "the view has changed and CONTINUE my approach across it — build on the progress, do not "
            "re-plan from scratch. But the CURRENT instruction WINS: if it now points somewhere "
            "different, I follow it and drop the old plan.")

    @staticmethod
    def _response_schema(n):
        waypoint = types.Schema(
            type=types.Type.OBJECT,
            required=["point"],
            properties={
                "point": types.Schema(
                    type=types.Type.ARRAY, items=types.Schema(type=types.Type.INTEGER)),
                "label": types.Schema(type=types.Type.STRING),
            },
        )
        plan = types.Schema(
            type=types.Type.OBJECT,
            required=["reasoning", "waypoints", "turn_degrees"],
            properties={
                "reasoning": types.Schema(type=types.Type.STRING),
                "waypoints": types.Schema(type=types.Type.ARRAY, items=waypoint),
                "turn_degrees": types.Schema(type=types.Type.NUMBER),
            },
        )
        if n <= 1:
            return plan
        return types.Schema(
            type=types.Type.OBJECT,
            required=["candidates"],
            properties={"candidates": types.Schema(type=types.Type.ARRAY, items=plan)},
        )

    @staticmethod
    def _return_spec(n):
        shape = ('{"reasoning": <plan the path in words, then draw it>,'
                 '"waypoints": [{"point": [y, x], "label": <n>}, ...],'
                 '"turn_degrees": <in-place turn at the end, + left / - right / 0 none>}')
        single = 'Return JSON only, no markdown fencing:\n' + shape
        if n <= 1:
            return single
        return (
            f"Give my top {n} candidate paths for this. If the way is clear they will be similar; if "
            "the scene is ambiguous (two ways around something) let them differ so the real options "
            "show. Each is a COMPLETE plan by the rules above. Return JSON only, no markdown fencing:\n"
            '{"candidates": [' + shape + ', ...]}')

    @staticmethod
    def _candidate_dicts(data):
        if isinstance(data, dict) and isinstance(data.get("candidates"), list):
            cands = [c for c in data["candidates"] if isinstance(c, (dict, list))]
            if cands:
                return cands
        return [data]

    def _select_medoid(self, cands):
        if len(cands) == 1:
            return cands[0]
        curves = [self._resample([(m.x, m.y) for m in c[1]]) for c in cands]
        best_i, best_cost = 0, float("inf")
        for i in range(len(curves)):
            cost = sum(float(np.mean(np.linalg.norm(curves[i] - curves[j], axis=1)))
                       for j in range(len(curves)) if j != i)
            if cost < best_cost:
                best_cost, best_i = cost, i
        return cands[best_i]

    @staticmethod
    def _resample(pts, k=12):
        a = np.asarray(pts, dtype=float)
        if len(a) < 2:
            base = a[:1] if len(a) else np.zeros((1, 2))
            return np.repeat(base, k, axis=0)
        seg = np.linalg.norm(np.diff(a, axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(arc[-1])
        if total < 1e-9:
            return np.repeat(a[:1], k, axis=0)
        targets = np.linspace(0.0, total, k)
        return np.stack([np.interp(targets, arc, a[:, 0]),
                         np.interp(targets, arc, a[:, 1])], axis=1)

    def _parse_reasoning_points(self, data):
        if isinstance(data, dict):
            reasoning = data.get("reasoning", "") or ""
            points = data.get("waypoints", [])
            turn = data.get("turn_degrees", 0.0)
        elif isinstance(data, list):
            reasoning, points, turn = "", data, 0.0
        else:
            reasoning, points, turn = "", [], 0.0
        if not isinstance(points, list):
            points = []
        try:
            turn = max(-360.0, min(360.0, float(turn)))
        except (TypeError, ValueError):
            turn = 0.0
        return str(reasoning), points, turn

    def _points_to_waypoints(self, points):
        min_row = int(self._p("min_row"))
        waypoints = []
        for p in points:
            pt = p.get("point") if isinstance(p, dict) else None
            if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
                continue
            y, x = pt
            y = max(float(y), float(min_row))
            waypoint = Point()
            waypoint.x = float(min(max(2.0 * x / 1000.0 - 1.0, -1.0), 1.0))
            waypoint.y = float(min(max(2.0 * y / 1000.0 - 1.0, -1.0), 1.0))
            waypoint.z = 0.0
            waypoints.append(waypoint)
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
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
