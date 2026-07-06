"""Pure-pursuit waypoint follower.

Sibling of ``visual_servo`` — same action-driven lifecycle and state dynamics, but
it drives off ``waypoint_tracker`` (the ground-trajectory tracker) instead of
``lk_tracker``, and steers with a pure-pursuit law instead of IBVS.

On a ``FollowTrajectory`` goal it hands the waypoints to ``waypoint_tracker`` via
``set_waypoints``, then runs a fixed-rate control loop that steers the robot toward
a lookahead waypoint from the ``/waypoint_tracking/points`` stream. As the robot
advances, ``waypoint_tracker`` retires waypoints that pass under it and goes
``UNTRACKED`` once the whole trajectory is consumed — which is the arrival signal
here (mirroring how ``visual_servo`` treats the target filling the frame).

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

# Robot reference point in normalized image space (bottom-centre; just under the
# camera). The pure-pursuit lookahead distance is measured from here.
_ROBOT_REF = (0.0, 1.0)


class PursuitServoNode(Node):
    def __init__(self):
        super().__init__("pursuit_servo_node")

        # --- Parameters ---
        self.declare_parameter("k_yaw", 0.8)  # steering gain (rad/s per unit error)
        self.declare_parameter("cruise_speed", 0.15)  # m/s forward when aligned
        self.declare_parameter("lookahead", 0.6)  # normalized lookahead distance
        self.declare_parameter("max_linear_vel", 0.26)  # m/s cap (Waffle Pi rated max)
        self.declare_parameter("max_angular_vel", 1.82)  # rad/s cap
        self.declare_parameter("init_timeout", 5.0)  # s to reach TRACKING before fail
        self.declare_parameter("occlusion_timeout", 5.0)  # s in OCCLUDED before abort
        self.declare_parameter("control_rate", 20.0)  # Hz

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
        self._waypoints = None  # list of (x, y, tracked)
        self._tracker_state = STATUS_UNTRACKED

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

            # --- UNTRACKED: init pending, or (after tracking) trajectory consumed ---
            if state == STATUS_UNTRACKED:
                self._stop_robot()
                if ever_tracking:
                    # The tracker retires waypoints as they pass under the robot and
                    # goes UNTRACKED once the whole trajectory is consumed — arrival.
                    self._call_stop_tracking()
                    result = FollowTrajectory.Result()
                    result.success = True
                    result.message = "Trajectory complete"
                    goal_handle.succeed()
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

            target = self._select_lookahead(self._waypoints)
            if target is None:
                # No measured waypoint to steer by this tick — hold still.
                self._stop_robot()
                self._publish_feedback(goal_handle, "RUNNING")
                rate.sleep()
                continue

            # --- Pure-pursuit control law ---
            # Steer toward the lookahead waypoint's horizontal offset; slow the
            # forward speed as the heading error grows (turn in place when steep).
            tx, _ = target
            heading_error = -tx  # +tx = waypoint to the right -> turn right (w < 0)
            w = float(self._p("k_yaw")) * heading_error
            w = max(-float(self._p("max_angular_vel")),
                    min(float(self._p("max_angular_vel")), w))

            v = float(self._p("cruise_speed")) * max(0.0, 1.0 - abs(heading_error))
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
    # Pure-pursuit lookahead selection

    def _select_lookahead(self, waypoints):
        """Pick the pure-pursuit carrot from the tracked waypoint stream.

        Only **measured** (``tracked=true``) waypoints steer — a coasting point is
        a plane extrapolation, not a measurement. Walking the nearest-first list,
        return the first measured waypoint at least ``lookahead`` from the robot
        reference; if none reach it, the farthest measured one. ``None`` if there is
        no measured waypoint this tick.
        """
        if not waypoints:
            return None
        lookahead = float(self._p("lookahead"))
        rx, ry = _ROBOT_REF
        farthest = None
        for x, y, tracked in waypoints:  # nearest-first order
            if not tracked:
                continue
            farthest = (x, y)
            if math.hypot(x - rx, y - ry) >= lookahead:
                return (x, y)
        return farthest

    # ------------------------------------------------------------------
    # Subscriber callbacks

    def _points_cb(self, msg):
        self._waypoints = [
            (p.x, p.y, bool(tr)) for p, tr in zip(msg.points, msg.tracked)
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
