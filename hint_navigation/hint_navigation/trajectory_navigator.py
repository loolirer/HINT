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

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

from nav2_msgs.action import FollowPath

from hint_interfaces.action import FollowTrajectory

from hint_navigation.camera_rig import CameraRig


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))  # (x, y, z, w)


def _smooth_resample(pts, spacing, samples_per_seg=24):
    """Densify a sparse polyline into a smooth, uniformly-spaced curve.

    Nav2's MPPI path critics (``offset_from_furthest``, path-align) assume a path sampled
    near costmap resolution; the raw VLM markers are far too sparse (a handful of points
    over metres), which stalls the optimizer mid-path. This fits a **centripetal**
    Catmull-Rom spline (alpha=0.5) through ``pts`` (an ``(N, 2)`` array, in order) and
    resamples it at ~``spacing`` m arc-length steps. Centripetal parameterization keeps the
    curve close to the polyline with no cusps or self-intersections, so the smoothed path
    never bows far from the waypoints. Endpoints are clamped (first/last control points are
    duplicated) so the curve starts/ends exactly on ``pts``.

    Two points give a straight resampled segment; ``pts`` passes through unchanged when it
    has fewer than 2 points or is already shorter than one ``spacing`` step.
    """
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 2:
        return pts

    if len(pts) == 2:
        curve = pts
    else:
        # Centripetal knots: t_{i+1} = t_i + |P_{i+1} - P_i|^0.5.
        d = np.hypot(*np.diff(pts, axis=0).T)
        t = np.concatenate([[0.0], np.cumsum(np.sqrt(np.maximum(d, 1e-9)))])
        # Pad with clamped endpoints so every real segment has 4 control points.
        P = np.vstack([pts[0], pts, pts[-1]])
        tt = np.concatenate([[t[0] - (t[1] - t[0])], t, [t[-1] + (t[-1] - t[-2])]])
        segs = []
        for i in range(1, len(P) - 2):                     # segment P[i] -> P[i+1]
            t0, t1, t2, t3 = tt[i - 1], tt[i], tt[i + 1], tt[i + 2]
            if min(t1 - t0, t2 - t1, t3 - t2) <= 0.0:      # degenerate knot — skip
                continue
            u = np.linspace(t1, t2, samples_per_seg, endpoint=False)[:, None]
            A1 = (t1 - u) / (t1 - t0) * P[i - 1] + (u - t0) / (t1 - t0) * P[i]
            A2 = (t2 - u) / (t2 - t1) * P[i] + (u - t1) / (t2 - t1) * P[i + 1]
            A3 = (t3 - u) / (t3 - t2) * P[i + 1] + (u - t2) / (t3 - t2) * P[i + 2]
            B1 = (t2 - u) / (t2 - t0) * A1 + (u - t0) / (t2 - t0) * A2
            B2 = (t3 - u) / (t3 - t1) * A2 + (u - t1) / (t3 - t1) * A3
            segs.append((t2 - u) / (t2 - t1) * B1 + (u - t1) / (t2 - t1) * B2)
        segs.append(pts[-1][None, :])                      # close on the final waypoint
        curve = np.vstack(segs)

    # Uniform arc-length resample of the smooth curve at ~spacing.
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(curve, axis=0).T))])
    if s[-1] < spacing:
        return pts
    su = np.linspace(0.0, s[-1], int(s[-1] / spacing) + 1)
    return np.column_stack([np.interp(su, s, curve[:, 0]),
                            np.interp(su, s, curve[:, 1])])


class TrajectoryNavigatorNode(Node):
    def __init__(self):
        super().__init__("trajectory_navigator_node")

        # --- Camera rig for the normalized-marker -> ground projection (shared,
        # live-adjustable; re-snapshotted per goal via CameraRig.from_node) ---
        CameraRig.declare(self)
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
        # Ground-mask clipping: the VLM pixel trajectory is truncated at the first marker
        # that leaves the segmented ground (that marker and all after it are dropped, so we
        # never follow a path that runs off the floor). No fresh mask -> pass through.
        self.declare_parameter("mask_topic", "/camera/ground")
        self.declare_parameter("mask_timeout", 5.0)  # s; older mask -> skip clipping
        self.declare_parameter("server_timeout", 10.0)  # s to wait for controller_server
        self.declare_parameter("control_rate", 20.0)  # Hz feedback/poll loop
        # Smooth-densification spacing (m): the grounded VLM markers are resampled onto a
        # centripetal Catmull-Rom spline at this arc-length step before FollowPath, so MPPI's
        # path critics see a dense path (≈ costmap resolution). Live-adjustable.
        self.declare_parameter("path_resolution", 0.05)

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

        # --- Ground mask (image-space, for clipping the pixel trajectory) ---
        self._bridge = CvBridge()
        self._mask_buf = deque(maxlen=30)  # (header-stamp seconds, mask HxW uint8)
        self._last_mask_recv = None        # local receipt clock, for staleness
        self._mask_lock = threading.Lock()
        self.create_subscription(
            Image, str(self._p("mask_topic")), self._mask_cb, 5
        )

        # Latched debug publishers (RViz: add Path displays, fixed frame = path_frame).
        #   ~/path      — the ground-clipped path actually handed to MPPI
        #   ~/path_raw  — the FULL VLM trajectory as grounded (debug only, never followed),
        #                 so you can see what the VLM intended vs what survived the clip.
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, "~/path", _latched)
        self._raw_path_pub = self.create_publisher(Path, "~/path_raw", _latched)

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
    # Ground mask -> pixel-trajectory clipping

    def _mask_cb(self, msg):
        try:
            mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:
            self.get_logger().warn(f"Ground mask decode failed: {e}",
                                   throttle_duration_sec=5.0)
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._mask_lock:
            self._mask_buf.append((t, mask))
            self._last_mask_recv = self.get_clock().now().nanoseconds * 1e-9

    def _mask_at(self, stamp):
        """Ground mask (HxW uint8) nearest ``stamp`` (None/zero -> latest), or None if no
        mask has arrived or the freshest is older than ``mask_timeout``."""
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._mask_lock:
            if not self._mask_buf or self._last_mask_recv is None:
                return None
            if (now - self._last_mask_recv) > float(self._p("mask_timeout")):
                return None
            buf = list(self._mask_buf)
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            return buf[-1][1]
        key = stamp.sec + stamp.nanosec * 1e-9
        return min(buf, key=lambda e: abs(e[0] - key))[1]

    def _clip_to_ground(self, waypoints, stamp):
        """Keep the leading run of markers that lie on the segmented ground; drop the first
        off-ground marker and everything after it, so we never follow a broken path.

        Markers are normalized image coords (``x``/``y in [-1, 1]``, center 0), mapped into
        the mask's own pixel grid. No fresh mask -> pass the trajectory through unchanged.
        """
        mask = self._mask_at(stamp)
        if mask is None:
            self.get_logger().warn(
                "No fresh ground mask — following the VLM path unclipped.",
                throttle_duration_sec=5.0)
            return list(waypoints)
        h, w = mask.shape[:2]
        kept = []
        for p in waypoints:
            u = min(max(int(round((p.x + 1.0) * 0.5 * (w - 1))), 0), w - 1)
            v = min(max(int(round((p.y + 1.0) * 0.5 * (h - 1))), 0), h - 1)
            if mask[v, u] > 0:            # on ground
                kept.append(p)
            else:                         # off ground -> drop this and all subsequent
                break
        if len(kept) < len(waypoints):
            self.get_logger().info(
                f"Ground-clip: kept {len(kept)}/{len(waypoints)} waypoints "
                "(rest fell off the ground mask).")
        return kept

    # ------------------------------------------------------------------
    # Grounding: normalized markers -> ground (base_link) -> odom Path

    def _build_path(self, waypoints, stamp):
        """Ground ``waypoints`` (normalized image markers) into a ``nav_msgs/Path`` in
        ``path_frame``. Returns the Path, or None (no usable waypoints / no odom).
        """
        if not waypoints:
            return None
        ref = self._pose_at(stamp)
        if ref is None:
            self.get_logger().warn("No odometry yet — cannot ground the trajectory.")
            return None

        w = int(self._p("image_width"))
        h = int(self._p("image_height"))
        norm = np.array([[p.x, p.y] for p in waypoints], dtype=np.float64)
        pix = np.empty_like(norm)
        pix[:, 0] = (norm[:, 0] + 1.0) * 0.5 * (w - 1)
        pix[:, 1] = (norm[:, 1] + 1.0) * 0.5 * (h - 1)
        # (N,2) base_link (X fwd, Y left); snapshot the rig now (live-adjustable).
        ground = CameraRig.from_node(self).pixels_to_ground(pix, w, h)

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

        # Smooth + densify: fit a centripetal Catmull-Rom through the sparse markers and
        # resample at ~path_resolution m, so MPPI's path critics get a costmap-resolution
        # path instead of a few far-apart points (which stalled the optimizer mid-path).
        odom_pts = _smooth_resample(odom_pts, float(self._p("path_resolution")))

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

        # Ground the FULL VLM trajectory once — published on ~/path_raw as a debug so RViz
        # shows what the VLM intended, even though we only *drive* the ground-clipped path.
        raw_path = self._build_path(goal.waypoints, goal.stamp)

        # Clip the VLM pixel trajectory to the segmented ground: keep the leading run of
        # markers on the mask, drop the first off-ground one and everything after it.
        wps = self._clip_to_ground(goal.waypoints, goal.stamp)

        # Nothing left to follow — the planner sent no path (a turn-only move), or the whole
        # trajectory fell off the ground. Succeed immediately so the BT's SpinAction, which
        # runs next in the sequence, still performs the rotation. (A real grounding failure
        # with waypoints present still aborts below.)
        if not wps:
            if raw_path is not None:
                self._publish_path(raw_path, self._raw_path_pub)  # still show the intent
            result = FollowTrajectory.Result()
            result.success = True
            result.message = "No drivable path (turn-only or clipped off-ground)"
            goal_handle.succeed()
            return result

        # Reuse the raw grounding when nothing was clipped; else ground the surviving prefix.
        # (raw_path is None only if odom is missing, in which case the clipped build fails too.)
        path = raw_path if len(wps) == len(goal.waypoints) else self._build_path(wps, goal.stamp)
        if path is None:
            return self._abort(goal_handle, "Could not ground trajectory (no odometry)")
        self._publish_path(path, self._path_pub)         # RViz: the path handed to MPPI
        self._publish_path(raw_path, self._raw_path_pub)  # RViz: the full VLM intent

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
            self._publish_path(path, self._path_pub)
            self._publish_path(raw_path, self._raw_path_pub)
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
            self._publish_path(path, self._path_pub)
            self._publish_path(raw_path, self._raw_path_pub)
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

    def _publish_path(self, path, pub):
        """Publish an (odom-frame) path on ``pub`` with a CURRENT stamp.

        Re-stamping matters for ego (base_link) views: RViz transforms a Path at its
        header stamp, so a once-published, frozen-stamp path stays pinned to the plan-time
        robot pose. A fresh stamp each tick makes it slide as the robot moves — the same
        reason MPPI's transformed_global_plan tracks correctly.
        """
        path.header.stamp = self.get_clock().now().to_msg()
        pub.publish(path)

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
