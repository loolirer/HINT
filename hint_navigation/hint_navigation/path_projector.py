import math
import threading
from collections import deque

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

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image

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
        self.declare_parameter("mask_topic", "/camera/ground")
        self.declare_parameter("mask_timeout", 5.0)      # s
        self.declare_parameter("server_timeout", 10.0)   # s
        self.declare_parameter("control_rate", 20.0)     # Hz
        self.declare_parameter("path_resolution", 0.05)

        self._fp_client = ActionClient(
            self, FollowPath, str(self._p("follow_path_action")),
            callback_group=ReentrantCallbackGroup(),
        )

        self._tf_buffer = Buffer(
            cache_time=Duration(seconds=float(self._p("tf_buffer_time")))
        )
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._bridge = CvBridge()
        self._mask_buf = deque(maxlen=30)  # (stamp seconds, mask HxW uint8)
        self._last_mask_recv = None        # local receipt clock, for staleness
        self._mask_lock = threading.Lock()
        self.create_subscription(
            Image, str(self._p("mask_topic")), self._mask_cb, 5
        )

        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, "~/path", _latched)        # driven path
        self._raw_path_pub = self.create_publisher(Path, "~/path_raw", _latched)  # full VLM intent

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
            if mask[v, u] > 0:
                kept.append(p)
            else:                         # off ground -> drop this and all subsequent
                break
        if len(kept) < len(waypoints):
            self.get_logger().info(
                f"Ground-clip: kept {len(kept)}/{len(waypoints)} waypoints "
                "(rest fell off the ground mask).")
        return kept

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

        raw_path = self._build_path(goal.waypoints, goal.stamp)
        wps = self._clip_to_ground(goal.waypoints, goal.stamp)

        # No path left (turn-only move, or the whole path fell off the ground): succeed so
        # the BT's SpinAction still runs. A grounding failure with waypoints present aborts below.
        if not wps:
            if raw_path is not None:
                self._publish_path(raw_path, self._raw_path_pub)
            result = FollowVisualPath.Result()
            result.message = "No drivable path (turn-only or clipped off-ground)"
            goal_handle.succeed()
            return result

        path = raw_path if len(wps) == len(goal.waypoints) else self._build_path(wps, goal.stamp)
        if path is None:
            return self._abort(goal_handle, "Could not ground path (no pose at the frame "
                                            "stamp — TF unavailable / tf_buffer_time too small)")
        self._publish_path(path, self._path_pub)
        self._publish_path(raw_path, self._raw_path_pub)

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
            self._publish_path(path, self._path_pub)
            self._publish_path(raw_path, self._raw_path_pub)
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
            # Re-stamp each tick so RViz transforms the path against the current
            # odom->base_link and it slides with the robot instead of freezing.
            self._publish_path(path, self._path_pub)
            self._publish_path(raw_path, self._raw_path_pub)
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

    def _publish_path(self, path, pub):
        path.header.stamp = self.get_clock().now().to_msg()
        pub.publish(path)

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
