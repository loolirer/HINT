"""Pure-pursuit waypoint follower.

Sibling of ``visual_servo`` — same action-driven lifecycle and state dynamics, but
it drives off ``waypoint_tracker`` (the ground-trajectory tracker) instead of
``lk_tracker``, and steers with a pure-pursuit law instead of IBVS.

On a ``FollowTrajectory`` goal it hands the waypoints to the tracker via
``set_waypoints``, then runs a fixed-rate control loop that steers toward a lookahead
carrot from the ``/waypoint_tracking/points`` stream. It runs in **metric top-down
world space**: the tracker publishes every waypoint's live position in ``base_link``
(x forward, y left, metres) with a per-point ``in_front`` flag, and this follower
does plain metric pure pursuit — carrot by Euclidean distance, steering by the
carrot's bearing. The tracker only tracks (publishes the full set every frame, never
retires); this follower owns *reaching*: it advances a front over waypoints the robot
drives within ``reach_radius`` of, and declares arrival when the front reaches the
end. Only ``in_front`` waypoints steer; if the path continues behind, it rotates to
face it (turning back to a passed waypoint), which is why behind points stay on the
stream rather than being dropped.

State dynamics mirror ``visual_servo``: one goal at a time, ``IDLE`` while the
tracker initialises, ``RUNNING`` while following, ``WAITING`` while ``OCCLUDED``;
``init_timeout`` / ``occlusion_timeout`` failures; ``stop_tracking`` on result.
"""

import math
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

from hint_interfaces.action import FollowTrajectory
from hint_interfaces.msg import VisualWaypoints
from hint_interfaces.srv import SetWaypoints, StopTracking

STATUS_UNTRACKED = "UNTRACKED"
STATUS_TRACKING = "TRACKING"
STATUS_OCCLUDED = "OCCLUDED"

_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Robot reference point in metric top-down world space: the base_link origin — the
# robot itself. Waypoints arrive as metric base_link coords (x forward, y left), so
# lookahead / reach distances are plain Euclidean distances from here, in metres.
_ROBOT_REF = (0.0, 0.0)


class PursuitServoNode(Node):
    def __init__(self):
        super().__init__("pursuit_servo_node")

        # --- Parameters ---  (all distances metric, in metres — top-down world space)
        self.declare_parameter("k_yaw", 1.5)  # steering gain (rad/s per rad of bearing)
        self.declare_parameter("cruise_speed", 0.05)  # m/s forward when aligned
        self.declare_parameter("lookahead", 0.3)  # m — carrot distance from the robot
        self.declare_parameter("max_linear_vel", 0.26)  # m/s cap (Waffle Pi rated max)
        self.declare_parameter("max_angular_vel", 1.82)  # rad/s cap
        self.declare_parameter("init_timeout", 5.0)  # s to reach TRACKING before fail
        self.declare_parameter("occlusion_timeout", 5.0)  # s in OCCLUDED before abort
        self.declare_parameter("control_rate", 20.0)  # Hz
        # A waypoint is "reached" (driven over) — and the trajectory front advances
        # past it — when the robot comes within ``reach_radius`` metres of it (close to
        # the base_link origin). A waypoint the robot passes wide of is NOT reached;
        # pure pursuit keeps steering (and can turn back) to bring the robot onto it.
        # Retirement lives here, in the follower — the tracker only tracks.
        self.declare_parameter("reach_radius", 0.05)  # m — arrival radius per waypoint

        # --- Tracker service clients ---
        self._set_waypoints_cli = self.create_client(
            SetWaypoints, "/waypoint_tracker_node/set_waypoints"
        )
        self._stop_tracking_cli = self.create_client(
            StopTracking, "/waypoint_tracker_node/stop_tracking"
        )

        # --- Subscriptions ---
        self.create_subscription(
            VisualWaypoints, "/waypoint_tracking/points", self._points_cb, 10
        )
        self.create_subscription(
            String, "/waypoint_tracking/state", self._state_cb, _LATCHED_QOS
        )

        # --- Publisher ---
        self._cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # --- Shared state (written by subscriber threads, read by action loop) ---
        self._waypoints = None  # list of (x, y, in_front) — metric base_link
        self._tracker_state = STATUS_UNTRACKED
        # Trajectory progress: waypoints [0, _reached_count) have been driven over.
        # The tracker publishes the full ordered set every frame (it never retires);
        # the follower owns retirement by advancing this front as waypoints are
        # reached, and declares arrival when the front reaches the end.
        self._num_waypoints = 0
        self._reached_count = 0

        # Lock to serialise goal acceptance so only one goal runs at a time.
        self._goal_lock = threading.Lock()

        # --- Action server ---
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
            "Pursuit servo ready — call ~/follow_trajectory to start."
        )

    # ------------------------------------------------------------------
    # Action callbacks

    def _goal_cb(self, goal_request):
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn(
                "Rejecting goal — another trajectory is already running."
            )
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
    # Main control loop

    def _run(self, goal_handle):
        goal = goal_handle.request
        self._waypoints = None  # discard any stream from a previous run
        self._num_waypoints = len(goal.waypoints)
        self._reached_count = 0  # reset trajectory progress for this goal

        # Delegate initialisation to the tracker.
        resp = self._call_set_waypoints(goal.waypoints, goal.stamp)
        if resp is None or not resp.accepted:
            self._stop_robot()
            result = FollowTrajectory.Result()
            result.success = False
            result.message = "Tracker rejected waypoints"
            goal_handle.abort()
            return result

        rate = self.create_rate(float(self._p("control_rate")))
        init_timeout = float(self._p("init_timeout"))
        occlusion_timeout = float(self._p("occlusion_timeout"))
        deadline = self.get_clock().now()
        ever_tracking = False
        occlusion_start = None

        while rclpy.ok():
            # --- Cancellation ---
            if goal_handle.is_cancel_requested:
                self._call_stop_tracking()
                self._stop_robot()
                goal_handle.canceled()
                result = FollowTrajectory.Result()
                result.success = False
                result.message = "Cancelled by client"
                return result

            state = self._tracker_state

            # --- UNTRACKED: waiting for the tracker to initialise. Arrival is now
            # detected by the follower from waypoint progress (TRACKING branch below),
            # not by the tracker going UNTRACKED — the tracker never retires. So an
            # UNTRACKED *after* tracking means the tracker reset unexpectedly. ---
            if state == STATUS_UNTRACKED:
                self._stop_robot()
                if ever_tracking:
                    self._call_stop_tracking()
                    result = FollowTrajectory.Result()
                    result.success = False
                    result.message = "Tracker reset unexpectedly"
                    goal_handle.abort()
                    return result
                elapsed = (self.get_clock().now() - deadline).nanoseconds * 1e-9
                if elapsed > init_timeout:
                    self._call_stop_tracking()
                    result = FollowTrajectory.Result()
                    result.success = False
                    result.message = "Initialisation timeout"
                    goal_handle.abort()
                    return result
                self._publish_feedback(goal_handle, "IDLE")
                rate.sleep()
                continue

            # --- OCCLUDED: stop robot and wait for recovery ---
            if state == STATUS_OCCLUDED:
                self._stop_robot()
                if occlusion_start is None:
                    occlusion_start = self.get_clock().now()
                elapsed = (self.get_clock().now() - occlusion_start).nanoseconds * 1e-9
                if elapsed > occlusion_timeout:
                    self._call_stop_tracking()
                    result = FollowTrajectory.Result()
                    result.success = False
                    result.message = "Occlusion timeout"
                    goal_handle.abort()
                    return result
                self._publish_feedback(goal_handle, "WAITING")
                rate.sleep()
                continue

            # --- TRACKING ---
            ever_tracking = True
            occlusion_start = None  # reset occlusion timer on recovery
            deadline = self.get_clock().now()  # reset so brief UNTRACKED fails fast

            # Advance the trajectory front over any waypoints the robot has now driven
            # over — the retirement the tracker no longer does. When the front reaches
            # the end, the whole trajectory is consumed: arrival.
            self._update_reached(self._waypoints)
            if self._num_waypoints > 0 and self._reached_count >= self._num_waypoints:
                self._call_stop_tracking()
                self._stop_robot()
                result = FollowTrajectory.Result()
                result.success = True
                result.message = "Trajectory complete"
                goal_handle.succeed()
                return result

            # Steer only by waypoints ahead of the front (still to be reached).
            target = self._select_lookahead(self._unreached())
            if target is None:
                # No unreached waypoint to steer by this tick — hold still.
                self._stop_robot()
                self._publish_feedback(goal_handle, "RUNNING")
                rate.sleep()
                continue

            # --- Pure-pursuit control law (metric top-down) ---
            # Steer toward the carrot's bearing (angle off the robot's forward axis);
            # slow the forward speed as the bearing grows, turning in place when the
            # carrot is more than 90 deg off-axis (incl. a carrot that is behind — the
            # robot rotates to face it, which is how it turns back to a passed point).
            tx, ty = target
            bearing = math.atan2(ty, tx)  # +left; ±pi when behind
            w = float(self._p("k_yaw")) * bearing
            w = max(-float(self._p("max_angular_vel")),
                    min(float(self._p("max_angular_vel")), w))

            v = float(self._p("cruise_speed")) * max(
                0.0, 1.0 - abs(bearing) / (math.pi / 2.0)
            )
            v = min(v, float(self._p("max_linear_vel")))

            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = v
            cmd.twist.angular.z = w
            self._cmd_pub.publish(cmd)

            self._publish_feedback(goal_handle, "RUNNING")
            rate.sleep()

        # Node shutting down mid-action.
        self._stop_robot()
        result = FollowTrajectory.Result()
        result.success = False
        result.message = "Node shutdown"
        return result

    # ------------------------------------------------------------------
    # Trajectory progress (retirement lives in the follower, not the tracker)

    def _unreached(self):
        """The still-to-reach tail of the waypoint stream (front already driven over).

        The tracker publishes the full ordered set every frame; steering and the
        arrival test consider only waypoints from ``_reached_count`` onward.
        """
        if not self._waypoints:
            return None
        return self._waypoints[self._reached_count:]

    def _update_reached(self, waypoints):
        """Advance ``_reached_count`` over waypoints the robot has driven over.

        In-order, a waypoint is *reached* when the robot comes within
        ``reach_radius`` metres of it (close to the base_link origin). Consumes
        consecutively from the front and stops at the first not-yet-reached waypoint,
        so reaching is sequential along the trajectory. A waypoint the robot passes
        wide of is *not* reached (the robot keeps steering — and can turn back — to
        drive over it); actually driving over it is what retires it.
        """
        if not waypoints:
            return
        reach = float(self._p("reach_radius"))
        rx, ry = _ROBOT_REF
        n = len(waypoints)
        while self._reached_count < n:
            x, y, _in_front = waypoints[self._reached_count]
            if math.hypot(x - rx, y - ry) <= reach:
                self._reached_count += 1
            else:
                break

    # ------------------------------------------------------------------
    # Pure-pursuit lookahead selection

    def _select_lookahead(self, waypoints):
        """Pick the pure-pursuit carrot (metric base_link) from the unreached tail.

        Sequential along the trajectory: if the **next** unreached waypoint is behind
        the robot, target it directly so the control law rotates to face it — the
        robot turns back to a waypoint it passed wide of, rather than skipping ahead
        and abandoning it (which would stall completion). Otherwise walk the in-front
        prefix and return the first waypoint at least ``lookahead`` metres away (the
        farthest in-front one if none reach that), stopping at any behind waypoint so
        the path is never skipped over. ``None`` only when the tail is empty.
        """
        if not waypoints:
            return None
        fx, fy, front0 = waypoints[0]
        if not front0:
            return (fx, fy)  # next target is behind — turn back to it
        lookahead = float(self._p("lookahead"))
        rx, ry = _ROBOT_REF
        farthest = (fx, fy)
        for x, y, in_front in waypoints:  # trajectory order
            if not in_front:
                break  # don't skip past the path to a farther in-front waypoint
            farthest = (x, y)
            if math.hypot(x - rx, y - ry) >= lookahead:
                return (x, y)
        return farthest

    # ------------------------------------------------------------------
    # Subscriber callbacks

    def _points_cb(self, msg):
        # Metric base_link waypoints (x forward, y left) + per-point front/back flag.
        self._waypoints = [
            (p.x, p.y, bool(inf)) for p, inf in zip(msg.points, msg.in_front)
        ]

    def _state_cb(self, msg):
        self._tracker_state = msg.data

    # ------------------------------------------------------------------
    # Helpers

    def _p(self, name):
        return self.get_parameter(name).value

    def _publish_feedback(self, goal_handle, state):
        fb = FollowTrajectory.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self._cmd_pub.publish(cmd)

    def _call_set_waypoints(self, waypoints, stamp):
        req = SetWaypoints.Request()
        req.waypoints = waypoints
        req.stamp = stamp
        return self._call_sync(self._set_waypoints_cli, req)

    def _call_stop_tracking(self):
        self._call_sync(self._stop_tracking_cli, StopTracking.Request())

    def _call_sync(self, client, request, timeout=2.0):
        """Call a service synchronously from within the action execute thread.

        Uses ``threading.Event`` so the ``MultiThreadedExecutor`` can process the
        service response in a parallel thread while this thread waits.
        """
        event = threading.Event()
        result = [None]

        def _done(future):
            result[0] = future.result()
            event.set()

        client.call_async(request).add_done_callback(_done)
        event.wait(timeout=timeout)
        return result[0]


def main(args=None):
    rclpy.init(args=args)
    node = PursuitServoNode()
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
