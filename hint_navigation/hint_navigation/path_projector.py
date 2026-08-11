import math
import threading

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time as RclpyTime

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from tf2_ros import Buffer, TransformException, TransformListener

from nav2_msgs.action import FollowPath

from hint_interfaces.action import FollowVisualPath

from hint_navigation.camera_rig import CameraRig


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))  # (x, y, z, w)


def _clip_range(pts, max_range):
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 2 or max_range <= 0.0:
        return pts
    origin = pts[0]  # the robot; index 0 is its prepended pose
    dist = np.hypot(*(pts - origin).T)
    outside = np.nonzero(dist > max_range)[0]
    if len(outside) == 0:
        return pts
    # First point beyond the radius; interpolate the segment–circle crossing before it.
    i = int(outside[0])
    a, d, f = pts[i - 1], pts[i] - pts[i - 1], pts[i - 1] - origin
    A, B, C = d @ d, 2.0 * (f @ d), f @ f - max_range * max_range
    t = (-B + math.sqrt(max(B * B - 4.0 * A * C, 0.0))) / (2.0 * A)
    cross = a + t * d
    return np.vstack([pts[:i], cross])


def _smooth_resample(pts, spacing, samples_per_seg=24):
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

    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(curve, axis=0).T))])
    if s[-1] < spacing:
        return pts
    su = np.linspace(0.0, s[-1], int(s[-1] / spacing) + 1)
    return np.column_stack([np.interp(su, s, curve[:, 0]),
                            np.interp(su, s, curve[:, 1])])


class PathProjectorNode(Node):
    def __init__(self):
        super().__init__("path_projector_node")

        CameraRig.declare(self)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # Nav2 controller_server advertises follow_path at the root namespace, not the node.
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("tf_buffer_time", 90.0)
        self.declare_parameter("tf_lookup_timeout", 0.1)
        self.declare_parameter("server_timeout", 10.0)   # s
        self.declare_parameter("control_rate", 10.0)     # Hz
        self.declare_parameter("path_resolution", 0.05)
        self.declare_parameter("path_range", 2.0)         # m — max straight-line distance from the robot

        self._fp_client = ActionClient(
            self, FollowPath, str(self._p("follow_path_action")),
            callback_group=ReentrantCallbackGroup(),
        )

        self._tf_buffer = Buffer(
            cache_time=Duration(seconds=float(self._p("tf_buffer_time")))
        )
        self._tf_listener = TransformListener(self._tf_buffer, self)

        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, "~/path", _latched)

        self._last_path = None
        self.create_timer(
            1.0 / max(1.0, float(self._p("control_rate"))),
            self._republish_path,
            callback_group=ReentrantCallbackGroup(),
        )

        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            FollowVisualPath,
            "~/follow_visual_path",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            "Path projector ready — call ~/follow_visual_path (drives Nav2 "
            f"{self._p('follow_path_action')})."
        )

    def _p(self, name):
        return self.get_parameter(name).value

    def _goal_cb(self, goal_request):
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn("Rejecting goal — another path is running.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            self._goal_lock.release()

    def _pose_at(self, stamp):
        target = str(self._p("path_frame"))
        source = str(self._p("robot_frame"))
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            when = RclpyTime()  # latest available
        else:
            when = RclpyTime.from_msg(stamp)
        timeout = Duration(seconds=float(self._p("tf_lookup_timeout")))
        try:
            tf = self._tf_buffer.lookup_transform(target, source, when, timeout)
        except TransformException as e:
            self.get_logger().warn(
                f"TF {target}<-{source} at the frame stamp unavailable ({e}) — the "
                "buffer may be smaller than the VLM latency (raise tf_buffer_time).",
                throttle_duration_sec=5.0,
            )
            return None
        t = tf.transform.translation
        return (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    def _build_path(self, waypoints, stamp):
        if not waypoints:
            return None
        ref = self._pose_at(stamp)
        if ref is None:
            return None

        w = int(self._p("image_width"))
        h = int(self._p("image_height"))
        norm = np.array([[p.x, p.y] for p in waypoints], dtype=np.float64)
        pix = np.empty_like(norm)
        pix[:, 0] = (norm[:, 0] + 1.0) * 0.5 * (w - 1)
        pix[:, 1] = (norm[:, 1] + 1.0) * 0.5 * (h - 1)
        ground = CameraRig.from_node(self).pixels_to_ground(pix, w, h)

        # base_link -> odom; prepend the robot pose so the path starts at the robot.
        rx, ry, ryaw = ref
        c, s = math.cos(ryaw), math.sin(ryaw)
        odom_pts = [(rx, ry)]
        for gx, gy in ground:
            odom_pts.append((rx + c * gx - s * gy, ry + s * gx + c * gy))
        odom_pts = np.array(odom_pts, dtype=np.float64)

        # Drop any zero-length leading segment (ref coincident with the first waypoint).
        keep = [0]
        for i in range(1, len(odom_pts)):
            if math.hypot(*(odom_pts[i] - odom_pts[keep[-1]])) > 1e-3:
                keep.append(i)
        odom_pts = odom_pts[keep]
        if len(odom_pts) < 2:
            return None

        odom_pts = _clip_range(odom_pts, float(self._p("path_range")))
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

    def _run(self, goal_handle):
        goal = goal_handle.request

        # Empty waypoints = turn-only move: succeed so the BT's SpinAction still runs.
        if not goal.waypoints:
            result = FollowVisualPath.Result()
            result.message = "No drivable path (turn-only move)"
            goal_handle.succeed()
            return result

        path = self._build_path(goal.waypoints, goal.stamp)
        if path is None:
            return self._abort(goal_handle, "Could not ground path (no pose at the frame "
                                            "stamp — TF unavailable / tf_buffer_time too small)")
        self._last_path = path

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

        send_future = self._fp_client.send_goal_async(fp_goal)
        while rclpy.ok() and not send_future.done():
            self._publish_feedback(goal_handle, "IDLE")
            rate.sleep()
        fp_handle = send_future.result()
        if fp_handle is None or not fp_handle.accepted:
            return self._abort(goal_handle, "Nav2 rejected the path")

        result_future = fp_handle.get_result_async()
        canceling = False
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested and not canceling:
                canceling = True
                fp_handle.cancel_goal_async()
            self._publish_feedback(goal_handle, "WAITING" if canceling else "RUNNING")
            rate.sleep()

        wrapped = result_future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_UNKNOWN

        if goal_handle.is_cancel_requested or status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            result = FollowVisualPath.Result()
            result.message = "Cancelled by client"
            return result

        result = FollowVisualPath.Result()
        if status == GoalStatus.STATUS_SUCCEEDED:
            result.message = "Path complete (Nav2 FollowPath reached the goal)"
            goal_handle.succeed()
        else:
            result.message = (
                "Nav2 FollowPath did not reach the goal "
                f"(status {status}) — path blocked / no progress"
            )
            goal_handle.abort()   # ABORTED status is the failure signal
        return result

    def _abort(self, goal_handle, message):
        goal_handle.abort()
        result = FollowVisualPath.Result()
        result.message = message
        self.get_logger().warn(f"FollowVisualPath aborted: {message}")
        return result

    def _republish_path(self):
        path = self._last_path
        if path is not None:
            self._publish_path(path)

    def _publish_path(self, path):
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(path)

    def _publish_feedback(self, goal_handle, state):
        fb = FollowVisualPath.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = PathProjectorNode()
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
