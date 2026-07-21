import cv2
import numpy as np
import rclpy
from google.genai import types
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from hint_interfaces.action import PlanTrajectory
from sensor_msgs.msg import Image

from gemini_robotics_er.gemini_base import GeminiActionNode

# Below this magnitude (deg) a turn is treated as "no turn" when deciding whether an
# empty-path reply is a valid turn-only move vs a genuine no-path (blocked / arrived).
_TURN_EPS = 1.0


class TrajectoryPlannerNode(GeminiActionNode):
    def __init__(self):
        super().__init__("trajectory_planner_node")

        # Farthest image row (of 1000) a waypoint may occupy — caps how far ahead
        # the trajectory reaches. Smaller row = farther/higher in the frame = more
        # error-prone; larger = nearer/more conservative. Live-adjustable.
        self.declare_parameter("min_row", 500)

        # Consensus sampling: ask for N candidate trajectories in ONE reply (this
        # model rejects candidate_count>1, so the prompt requests a candidates list)
        # and keep the medoid — the path closest to all the others — robust to the
        # model splitting between routes at temperature > 0. 1 = a single plan.
        self.declare_parameter("n_candidates", 1)

        # Output control (quality vs validity), live-adjustable:
        #   "json"   (default) JSON mode — valid JSON, keeps reasoning freedom;
        #   "off"    unconstrained — best quality, but can return unparseable text;
        #   "schema" constrained to the schema — always valid+shaped, but the hard
        #            grammar can cost spatial-reasoning quality.
        self.declare_parameter("structured_output", "json")

        # Continuity buffer: the last N (frame, my-own-reasoning) pairs are attached
        # ahead of the current frame, so each plan continues my own approach across
        # the view change instead of starting cold. I store my OWN reasoning (spatial
        # read + path intent), not the instruction I was fed. N = history_frames,
        # live-adjustable:
        #   0 -> stateless (current frame only); 1 -> last step; 3-4 -> deeper history
        #   (each extra frame is more image tokens = more latency/cost).
        self.declare_parameter("history_frames", 1)
        self._history = []   # list of (pil_img, reasoning), oldest first

        self._debug_pub = self.create_publisher(Image, "~/debug", 10)

        self._action_server = ActionServer(
            self,
            PlanTrajectory,
            "~/plan_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            "Trajectory planner ready — call ~/plan_trajectory."
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

        # N-candidate sampling is done IN ONE call: this model rejects
        # candidate_count>1, so the prompt asks for N paths in a single response
        # (via {return_spec}) and we pick the medoid. n<=1 is the plain single plan.
        n = max(1, int(self._p("n_candidates")))
        # Continuity buffer: the last N past frames (oldest first) go before the
        # current one, so "the last image attached" is my current view.
        hist = self._history_window()
        prompt = self._fill_prompt(
            "trajectory_planner.txt", description=goal.description,
            min_row=int(self._p("min_row")), return_spec=self._return_spec(n),
            continuity=self._continuity_text(hist))
        contents = [prompt] + [img for img, _ in hist] + [pil_img]
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

        # Parse every candidate the reply carries. A candidate with a usable path is a
        # path candidate; one with no path but a real turn is a turn-only candidate
        # (scan / re-orient in place).
        cand_dicts = self._candidate_dicts(self._parse_json(raw))
        path_cands, turn_cands, first_reason = [], [], ""
        for cd in cand_dicts:
            reasoning, points, turn = self._parse_reasoning_points(cd)
            first_reason = first_reason or reasoning
            markers = self._points_to_markers(points)
            if markers:
                path_cands.append((reasoning, markers, points, turn))
            elif abs(turn) >= _TURN_EPS:
                turn_cands.append((reasoning, [], [], turn))

        if path_cands:
            # Consensus: the medoid path — the candidate closest to all the others —
            # carrying its own end-of-path turn.
            reasoning, markers, points, turn = self._select_medoid(path_cands)
            if len(cand_dicts) > 1:
                self.get_logger().info(
                    f"Chose medoid of {len(path_cands)}/{len(cand_dicts)} candidate paths.")
        elif turn_cands:
            # Turn-only move: no path, just rotate in place.
            reasoning, markers, points, turn = turn_cands[0]
            self.get_logger().info(f"Turn-only move: {turn:+.0f} deg, no trajectory.")
        else:
            # No path and no turn anywhere: blocked / already there / unparseable.
            reason = first_reason or "no traversable ground path was visible"
            return self._abort(
                goal_handle,
                f'No trajectory for "{goal.description}": {reason}',
            )

        # Push this frame + the chosen reasoning into the buffer, trimmed to N.
        self._history.append((pil_img, reasoning))
        k = max(0, int(self._p("history_frames")))
        self._history = self._history[-k:] if k else []

        self._publish_debug(cv_bgr, stamp, markers, [c[1] for c in path_cands], turn)

        result = PlanTrajectory.Result()
        result.success = True
        # The VLM's brief explanation of the chosen path rides on `message`.
        result.message = reasoning or f"{len(markers)} waypoint(s), turn {turn:+.0f} deg"
        result.markers = markers
        result.turn_degrees = float(turn)
        result.stamp = stamp
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Helpers

    def _history_window(self):
        """The last N (frame, reasoning) pairs to attach as continuity (N =
        history_frames; empty when 0)."""
        k = max(0, int(self._p("history_frames")))
        return self._history[-k:] if k else []

    @staticmethod
    def _continuity_text(hist):
        """The {continuity} token: the N past frames (oldest first) + what I planned
        at each, or empty when the buffer is off/empty.

        The frames are attached BEFORE the current view; this note labels them and
        says to continue my own approach across the change — the current instruction
        still wins.
        """
        if not hist:
            return ""
        k = len(hist)
        lines = [f"{k + 1} images are attached, oldest first; the LAST is my CURRENT view — the "
                 f"earlier {k} are my recent past view(s), with what I planned at each:"]
        for i, (_, reasoning) in enumerate(hist, 1):
            lines.append(f'- view {i}: "{(reasoning or "(no note)").strip()}"')
        lines.append(
            "I continue that approach across how the view has changed — build on the progress, do "
            "not re-plan from scratch. But the CURRENT instruction WINS: if it now points somewhere "
            "different, I follow it and drop the old plan.")
        return "\n".join(lines)

    @staticmethod
    def _response_schema(n):
        """Constrained-output schema: a single plan, or a candidates list.

        Passed as ``response_schema`` so the model can only emit well-formed JSON
        matching it — the fix for the corrupted/degenerate long replies. Mirrors
        the ``{return_spec}`` prompt shape (single vs candidates) for ``n``.
        """
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
        """The {return_spec} block: single plan, or N candidate plans in ONE reply.

        candidate_count>1 is unsupported by this model, so N-sampling is done by
        asking for N paths in a single response and taking the medoid.
        """
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
        """Return the list of candidate replies from a parsed model response.

        Accepts the multi form ``{"candidates": [...]}``, a single object
        ``{"reasoning", "waypoints"}``, or a bare ``[...]`` waypoint list — each
        element is then handed to ``_parse_reasoning_points``.
        """
        if isinstance(data, dict) and isinstance(data.get("candidates"), list):
            cands = [c for c in data["candidates"] if isinstance(c, (dict, list))]
            if cands:
                return cands
        return [data]

    def _select_medoid(self, cands):
        """Return the consensus candidate — the one whose (arc-length-resampled)
        path is closest, summed, to all the others.

        ``cands`` is a list of ``(reasoning, markers, points, turn)`` tuples (the whole
        tuple is returned, so the chosen path keeps its own turn). Averaging whole paths
        is wrong (two valid routes average to a path between them), so this picks the
        most central *actual* candidate instead. A single candidate is returned as-is.
        """
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
        """Resample an ordered polyline to ``k`` points, evenly by arc length, so
        candidates of different lengths compare pointwise."""
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
        """Split the model reply into ``(reasoning, points, turn_degrees)``.

        Accepts the documented object form
        ``{"reasoning": ..., "waypoints": [...], "turn_degrees": ...}`` and tolerates a
        bare ``[...]`` list (reasoning empty, no turn).
        """
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

    def _points_to_markers(self, points):
        """Convert Gemini ``[{"point": [y, x], ...}]`` to normalized markers.

        Each returned ``Point`` has ``x``/``y`` in ``[-1, 1]`` (image space,
        center = 0), ``z`` unused. Malformed entries are skipped.
        """
        min_row = int(self._p("min_row"))
        markers = []
        for p in points:
            pt = p.get("point") if isinstance(p, dict) else None
            if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
                continue
            y, x = pt
            # Hard cap on forward reach: pull any point past the limit (too far /
            # too high in the frame) down to min_row. Far points are where the VLM's
            # ground grounding is least reliable; this backstops the prompt.
            y = max(float(y), float(min_row))
            marker = Point()
            marker.x = float(min(max(2.0 * x / 1000.0 - 1.0, -1.0), 1.0))
            marker.y = float(min(max(2.0 * y / 1000.0 - 1.0, -1.0), 1.0))
            marker.z = 0.0
            markers.append(marker)
        return markers

    def _publish_debug(self, cv_bgr, stamp, selected, candidates, turn=0.0):
        """All candidate paths in grey, the chosen medoid in green (tracker style).

        ``selected`` is the medoid markers (empty for a turn-only move); ``candidates``
        is every path candidate's markers (medoid included); ``turn`` is the chosen
        end-of-path rotation in degrees. The medoid is drawn last so it sits on top.
        """
        try:
            frame = cv_bgr.copy()
            h, w = frame.shape[:2]
            green = (0, 255, 0)

            def to_px(markers):
                return [(int((m.x + 1.0) / 2.0 * w), int((m.y + 1.0) / 2.0 * h))
                        for m in markers]

            # Every candidate in grey.
            for markers in candidates:
                if markers is selected:
                    continue
                pts = to_px(markers)
                if len(pts) >= 2:
                    cv2.polylines(frame, [np.array(pts, np.int32)], False,
                                  (150, 150, 150), 2, cv2.LINE_AA)

            # The chosen medoid in green, on top: line + waypoint dots.
            px = to_px(selected)
            if len(px) >= 2:
                cv2.polylines(frame, [np.array(px, np.int32)], False,
                              green, 2, cv2.LINE_AA)
            for cx, cy in px:
                cv2.circle(frame, (cx, cy), 4, green, -1)

            label = (f"medoid of {len(candidates)}" if len(candidates) > 1
                     else f"{len(px)} waypoints" if px else "turn-only")
            label += f"  turn {turn:+.0f}deg"
            cv2.putText(frame, label, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, green, 1, cv2.LINE_AA)

            out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out.header.stamp = stamp
            self._debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"Debug publish failed: {e}")

    def _publish_feedback(self, goal_handle, state):
        fb = PlanTrajectory.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _abort(self, goal_handle, message):
        self.get_logger().warn(message)
        result = PlanTrajectory.Result()
        result.success = False
        result.message = message
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle):
        goal_handle.canceled()
        result = PlanTrajectory.Result()
        result.success = False
        result.message = "Cancelled"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
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
