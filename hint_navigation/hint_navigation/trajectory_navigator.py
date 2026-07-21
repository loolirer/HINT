"""Trajectory navigator — adapter from the VLM trajectory to Nav2's FollowPath.

Exposes the **same** ``hint_interfaces/action/FollowTrajectory`` action the behaviour
tree already calls (so only its action name is repointed), and internally drives Nav2's
``nav2_msgs/action/FollowPath`` (MPPI controller). The whole chain stays action-based.

Per goal it:

1. Grounds the normalized image markers (``x``/``y in [-1, 1]``, center 0, nearest-first)
   onto the ground plane in ``base_link`` via the analytic camera model — the same
   projection ``odom_waypoint_tracker`` uses.
2. Re-expresses them in ``odom`` using the odometry pose at the goal's stamp (so the path
   is anchored in the world, exactly as the odom tracker anchored its reference frame),
   and builds a ``nav_msgs/Path`` with tangent yaws, prepended by the robot's pose so the
   path starts at the robot.
3. Calls ``follow_path`` and relays the outcome: Nav2 SUCCEEDED -> ``success=true``;
   ABORTED (incl. the ``SimpleProgressChecker`` firing on an unreachable goal — the native
   equivalent of the old stall watchdog) / CANCELED / rejected -> ``success=false``.

Single goal at a time, mirroring ``pursuit_servo``'s lifecycle; a cancel forwards a Nav2
cancel. Runs under a ``MultiThreadedExecutor`` with a reentrant action server so the
FollowPath client futures resolve while the execute callback polls them.
"""

import math
import threading
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from nav2_msgs.action import FollowPath

from hint_interfaces.action import FollowTrajectory


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))  # (x, y, z, w)


class TrajectoryNavigatorNode(Node):
    def __init__(self):
        super().__init__("trajectory_navigator_node")

        # --- Camera geometry for the normalized-marker -> ground projection ---
        self.declare_parameter("camera_height", 0.14)  # m above the ground plane
        self.declare_parameter("camera_forward_offset", 0.0)  # m ahead of base origin
        self.declare_parameter("camera_tilt", 0.0)  # rad, positive = pitched down
        self.declare_parameter("camera_hfov_deg", 62.2)  # horizontal FOV (Pi cam v2)
        # Image size the markers are normalized against — only the aspect ratio and hfov
        # actually affect grounding, so the camera's nominal resolution is enough.
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # --- Nav2 wiring ---
        # Nav2's controller_server advertises this action at the root (/follow_path):
        # action names resolve against the node namespace (/), not the node name.
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("server_timeout", 10.0)  # s to wait for controller_server
        self.declare_parameter("control_rate", 20.0)  # Hz feedback/poll loop

        # --- Nav2 FollowPath client ---
        self._fp_client = ActionClient(
            self, FollowPath, str(self._p("follow_path_action")),
            callback_group=ReentrantCallbackGroup(),
        )

        # --- Odometry (world anchor for the grounded path) ---
        self._odom_buf = deque(maxlen=200)
        self._odom_lock = threading.Lock()
        self.create_subscription(
            Odometry, str(self._p("odom_topic")), self._odom_cb, 20
        )

        # Latched debug publisher for the grounded path (RViz: add a Path display on
        # /trajectory_navigator_node/path, fixed frame = path_frame). The path otherwise
        # only lives inside the follow_path goal, so this is how you inspect exactly what
        # MPPI was asked to follow — and whether the VLM markers grounded correctly.
        self._path_pub = self.create_publisher(
            Path, "~/path",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            FollowTrajectory,
            "~/follow_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            "Trajectory navigator ready — call ~/follow_trajectory (drives Nav2 "
            f"{self._p('follow_path_action')})."
        )

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Action lifecycle

    def _goal_cb(self, goal_request):
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn("Rejecting goal — another trajectory is running.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            self._goal_lock.release()

    # ------------------------------------------------------------------
    # Odometry

    def _odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._odom_lock:
            self._odom_buf.append((t, p.x, p.y, yaw))

    def _pose_at(self, stamp):
        """Odom pose (x, y, yaw) nearest ``stamp`` (None/zero -> latest), or None."""
        with self._odom_lock:
            buf = list(self._odom_buf) if self._odom_buf else None
        if buf is None:
            return None
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            s = buf[-1]
        else:
            key = stamp.sec + stamp.nanosec * 1e-9
            s = min(buf, key=lambda e: abs(e[0] - key))
        return (s[1], s[2], s[3])

    # ------------------------------------------------------------------
    # Grounding: normalized markers -> ground (base_link) -> odom Path

    def _pixels_to_ground(self, pts, w, h):
        """Back-project pixel points (N,2) onto the ground plane (base_link).

        Same model as ``odom_waypoint_tracker._pixels_to_ground``: (X forward, Y left).
        Points on/above the horizon clamp to a far ground distance rather than diverging.
        """
        hfov = math.radians(float(self._p("camera_hfov_deg")))
        f = (w / 2.0) / math.tan(hfov / 2.0)
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        tilt = float(self._p("camera_tilt"))
        cos_t, sin_t = math.cos(tilt), math.sin(tilt)
        cam_h = float(self._p("camera_height"))
        x_off = float(self._p("camera_forward_offset"))
        xn = (pts[:, 0] - cx) / f
        yn = (pts[:, 1] - cy) / f
        denom = cos_t * yn + sin_t
        denom = np.where(denom > 1e-4, denom, 1e-4)  # clamp horizon/above to far
        t = cam_h / denom
        X = x_off + t * (cos_t - sin_t * yn)
        Y = t * (-xn)
        return np.stack([X, Y], axis=1).astype(np.float64)

    def _build_path(self, goal):
        """Ground the goal's markers into a ``nav_msgs/Path`` in ``path_frame``.

        Returns the Path, or None (no usable waypoints / no odom).
        """
        wps = goal.waypoints
        if not wps:
            return None
        ref = self._pose_at(goal.stamp)
        if ref is None:
            self.get_logger().warn("No odometry yet — cannot ground the trajectory.")
            return None

        w = int(self._p("image_width"))
        h = int(self._p("image_height"))
        norm = np.array([[p.x, p.y] for p in wps], dtype=np.float64)
        pix = np.empty_like(norm)
        pix[:, 0] = (norm[:, 0] + 1.0) * 0.5 * (w - 1)
        pix[:, 1] = (norm[:, 1] + 1.0) * 0.5 * (h - 1)
        ground = self._pixels_to_ground(pix, w, h)  # (N,2) base_link (X fwd, Y left)

        # base_link (ref pose) -> odom. Prepend the robot's own pose so the path starts
        # at the robot, then the grounded markers nearest-first.
        rx, ry, ryaw = ref
        c, s = math.cos(ryaw), math.sin(ryaw)
        odom_pts = [(rx, ry)]
        for gx, gy in ground:
            odom_pts.append((rx + c * gx - s * gy, ry + s * gx + c * gy))
        odom_pts = np.array(odom_pts, dtype=np.float64)

        # Drop any zero-length leading segment (ref coincident with the first marker).
        keep = [0]
        for i in range(1, len(odom_pts)):
            if math.hypot(*(odom_pts[i] - odom_pts[keep[-1]])) > 1e-3:
                keep.append(i)
        odom_pts = odom_pts[keep]
        if len(odom_pts) < 2:
            return None

        path = Path()
        path.header.frame_id = str(self._p("path_frame"))
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(odom_pts):
            nxt = odom_pts[min(i + 1, len(odom_pts) - 1)]
            prv = odom_pts[max(i - 1, 0)]
            yaw = math.atan2(nxt[1] - prv[1], nxt[0] - prv[0])
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            qx, qy, qz, qw = _quat_from_yaw(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        return path

    # ------------------------------------------------------------------
    # Main run: ground -> FollowPath -> relay

    def _run(self, goal_handle):
        goal = goal_handle.request

        # Turn-only move: the planner returned no path (just a turn). Nothing to follow —
        # succeed immediately so the BT's SpinAction, which runs next in the sequence,
        # performs the rotation. (A real grounding failure with waypoints present still
        # aborts below.)
        if not goal.waypoints:
            result = FollowTrajectory.Result()
            result.success = True
            result.message = "Turn-only move — no trajectory to follow"
            goal_handle.succeed()
            return result

        path = self._build_path(goal)
        if path is None:
            return self._abort(goal_handle, "Could not ground trajectory (no odometry)")
        self._publish_path(path)  # RViz: the exact path handed to MPPI

        if not self._fp_client.wait_for_server(
                timeout_sec=float(self._p("server_timeout"))):
            return self._abort(goal_handle, "Nav2 controller_server (follow_path) "
                                            "unavailable")

        fp_goal = FollowPath.Goal()
        fp_goal.path = path
        fp_goal.controller_id = str(self._p("controller_id"))
        fp_goal.goal_checker_id = str(self._p("goal_checker_id"))
        # Present since Iron; guard so older FollowPath.action definitions still work.
        if hasattr(fp_goal, "progress_checker_id"):
            fp_goal.progress_checker_id = str(self._p("progress_checker_id"))

        rate = self.create_rate(float(self._p("control_rate")))

        # Send and wait for acceptance.
        send_future = self._fp_client.send_goal_async(fp_goal)
        while rclpy.ok() and not send_future.done():
            self._publish_path(path)
            self._publish_feedback(goal_handle, "IDLE")
            rate.sleep()
        fp_handle = send_future.result()
        if fp_handle is None or not fp_handle.accepted:
            return self._abort(goal_handle, "Nav2 rejected the path")

        # Follow until the FollowPath result arrives, forwarding cancels.
        result_future = fp_handle.get_result_async()
        canceling = False
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested and not canceling:
                canceling = True
                fp_handle.cancel_goal_async()
            # Re-stamp + republish each tick so the path is transformed against the
            # CURRENT odom->base_link, not the frozen plan-time transform — otherwise it
            # sits locked in an ego (base_link) view instead of sliding as the robot moves.
            self._publish_path(path)
            self._publish_feedback(goal_handle, "WAITING" if canceling else "RUNNING")
            rate.sleep()

        wrapped = result_future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN

        if goal_handle.is_cancel_requested or status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            result = FollowTrajectory.Result()
            result.success = False
            result.message = "Cancelled by client"
            return result

        result = FollowTrajectory.Result()
        if status == GoalStatus.STATUS_SUCCEEDED:
            result.success = True
            result.message = "Trajectory complete (Nav2 FollowPath reached the goal)"
            goal_handle.succeed()
        else:
            result.success = False
            result.message = (
                "Nav2 FollowPath did not reach the goal "
                f"(status {status}) — path blocked / no progress"
            )
            goal_handle.abort()
        return result

    def _abort(self, goal_handle, message):
        goal_handle.abort()
        result = FollowTrajectory.Result()
        result.success = False
        result.message = message
        self.get_logger().warn(f"FollowTrajectory aborted: {message}")
        return result

    def _publish_path(self, path):
        """Publish the (odom-frame) path with a CURRENT stamp.

        Re-stamping matters for ego (base_link) views: RViz transforms a Path at its
        header stamp, so a once-published, frozen-stamp path stays pinned to the plan-time
        robot pose. A fresh stamp each tick makes it slide as the robot moves — the same
        reason MPPI's transformed_global_plan tracks correctly.
        """
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(path)

    def _publish_feedback(self, goal_handle, state):
        fb = FollowTrajectory.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNavigatorNode()
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
