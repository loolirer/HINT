import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from hint_interfaces.action import PlanTrajectory
from sensor_msgs.msg import Image

from gemini_robotics_er.gemini_base import GeminiActionNode


class TrajectoryPlannerNode(GeminiActionNode):
    def __init__(self):
        super().__init__("trajectory_planner_node")

        # Farthest image row (of 1000) a waypoint may occupy — caps how far ahead
        # the trajectory reaches. Smaller row = farther/higher in the frame = more
        # error-prone; larger = nearer/more conservative. Live-adjustable.
        self.declare_parameter("min_row", 600)

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

        prompt = self._fill_prompt(
            "trajectory_planner.txt", description=goal.description,
            min_row=int(self._p("min_row")))
        try:
            raw = self._call_api([pil_img, prompt])
        except TimeoutError as e:
            return self._abort(goal_handle, str(e))
        except Exception as e:
            return self._abort(goal_handle, f"API error: {e}")

        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle)

        data = self._parse_json(raw)
        reasoning, points = self._parse_reasoning_points(data)

        markers = self._points_to_markers(points)
        if not markers:
            # Still report the model's rationale for why no path was planned.
            reason = reasoning or "no traversable ground path was visible"
            return self._abort(
                goal_handle,
                f'No trajectory for "{goal.description}": {reason}',
            )

        self._publish_debug(cv_bgr, stamp, markers, points)

        result = PlanTrajectory.Result()
        result.success = True
        # The VLM's brief explanation of the chosen path rides on `message`.
        result.message = reasoning or f"{len(markers)} waypoint(s)"
        result.markers = markers
        result.stamp = stamp
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Helpers

    def _parse_reasoning_points(self, data):
        """Split the model reply into ``(reasoning, points)``.

        Accepts the documented object form
        ``{"reasoning": ..., "waypoints": [...]}`` and tolerates a bare
        ``[...]`` list (reasoning empty).
        """
        if isinstance(data, dict):
            reasoning = data.get("reasoning", "") or ""
            points = data.get("waypoints", [])
        elif isinstance(data, list):
            reasoning = ""
            points = data
        else:
            reasoning = ""
            points = []
        if not isinstance(points, list):
            points = []
        return str(reasoning), points

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

    def _heatmap_color(self, t):
        """BGR color for ``t`` in ``[0, 1]`` — 1.0 hottest, 0.0 coldest."""
        val = np.uint8([[int(round(t * 255))]])
        bgr = cv2.applyColorMap(val, cv2.COLORMAP_JET)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    def _publish_debug(self, cv_bgr, stamp, markers, points):
        try:
            frame = cv_bgr.copy()
            h, w = frame.shape[:2]
            n = len(markers)
            px = [
                (int((m.x + 1.0) / 2.0 * w), int((m.y + 1.0) / 2.0 * h))
                for m in markers
            ]

            for i, (cx, cy) in enumerate(px):
                # First point hottest (t=1), last coldest (t=0).
                t = 1.0 - (i / max(n - 1, 1))
                color = self._heatmap_color(t)
                cv2.circle(frame, (cx, cy), 8, color, -1)

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
