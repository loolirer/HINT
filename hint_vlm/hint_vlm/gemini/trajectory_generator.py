import numpy as np
import rclpy
from google.genai import types
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from hint_interfaces.action import PlanTrajectory

from hint_vlm.gemini.gemini_base import GeminiActionNode


class TrajectoryPlannerNode(GeminiActionNode):
    """Plans a ground-restricted trajectory from a text instruction + goal frames.

    Like ``visual_reasoner``, it holds no camera buffer of its own: the frames to
    plan over arrive in the ``PlanTrajectory`` goal's ``images`` list (the single
    buffer owned by ``hint_narrative``), oldest first — the LAST is the current
    view the path is planned on, any earlier ones are recent past views attached
    for continuity. This keeps the planner and the mission director reasoning over
    exactly the same frames (no buffer to drift out of sync). Buffer depth is set
    once, on ``hint_narrative``'s ``history_frames``.
    """

    def __init__(self):
        super().__init__("trajectory_generator")

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

        # Frames come from the goal (the unified hint_narrative buffer), oldest
        # first: the LAST is the current view the path is planned on, the earlier
        # ones are continuity. The current view must decode — its stamp grounds the
        # follow — while an unreadable past frame is just dropped.
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
            except Exception as e:  # noqa: BLE001 — skip an unreadable continuity frame
                self.get_logger().warn(f"Skipping an unreadable continuity frame: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        # N-candidate sampling is done IN ONE call: this model rejects
        # candidate_count>1, so the prompt asks for N paths in a single response
        # (via {return_spec}) and we pick the medoid. n<=1 is the plain single plan.
        n = max(1, int(self._p("n_candidates")))
        prompt = self._fill_prompt(
            "trajectory_generator.txt", description=goal.description,
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

        # Parse the reply. **Any interpretable response is a success** — the generator only
        # relays what the model said (waypoints, possibly empty; a turn, possibly 0), and
        # whether an environment is done / blocked is the narrative's call, not ours. The
        # only failures are *no usable response*: a timeout / API error (handled above), or
        # an unparseable reply here — a genuine model malfunction, distinct from a valid
        # "empty" decision, so the BT/narrative sees an error rather than a silent no-op.
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
            markers = self._points_to_markers(points)
            if markers:
                path_cands.append((reasoning, markers, points, turn))

        if path_cands:
            # Consensus: the medoid path — the candidate closest to all the others —
            # carrying its own end-of-path turn.
            reasoning, markers, points, turn = self._select_medoid(path_cands)
            if len(cand_dicts) > 1:
                self.get_logger().info(
                    f"Chose medoid of {len(path_cands)}/{len(cand_dicts)} candidate paths.")
        else:
            # No waypoints in any candidate: a turn-only (scan / re-orient) or a no-op
            # move — still a valid response. Relay the first candidate's turn verbatim.
            reasoning, markers, points, turn = first_reason, [], [], first_turn
            self.get_logger().info(f"No waypoints — turn-only/no-op move ({turn:+.0f} deg).")

        result = PlanTrajectory.Result()
        # The VLM's brief explanation of the chosen path rides on `message`.
        result.message = reasoning or f"{len(markers)} waypoint(s), turn {turn:+.0f} deg"
        result.markers = markers
        result.turn_degrees = float(turn)
        result.stamp = stamp
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Helpers

    @staticmethod
    def _continuity_text(n_past):
        """The {continuity} token describing the ``n_past`` past frames attached
        ahead of the current view (empty when there are none).

        Frames arrive in the goal (the unified hint_narrative buffer), oldest first;
        the last is the current view, the earlier ``n_past`` are recent past views.
        Only the images travel — no per-frame reasoning note — so the caption just
        labels them and says to continue the approach across the view change, with
        the current instruction still winning.
        """
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

    def _publish_feedback(self, goal_handle, state):
        fb = PlanTrajectory.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _abort(self, goal_handle, message):
        self.get_logger().warn(message)
        result = PlanTrajectory.Result()
        result.message = message
        goal_handle.abort()   # ABORTED status is the failure signal
        return result

    def _cancel(self, goal_handle):
        goal_handle.canceled()
        result = PlanTrajectory.Result()
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
