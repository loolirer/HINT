"""Control-Lyapunov path-following waypoint follower.

Sibling of ``visual_servo`` — same action-driven lifecycle and state dynamics, but
it drives off ``waypoint_tracker`` (the ground-trajectory tracker) instead of
``lk_tracker``, and steers with a Control-Lyapunov path-following law.

On a ``FollowTrajectory`` goal it hands the waypoints to the tracker via
``set_waypoints``, then runs a fixed-rate control loop in **metric top-down world
space**: the tracker publishes every waypoint's live position in ``base_link``
(x forward, y left, metres) every frame, and this follower treats the waypoint
polyline as the path Γ and applies the Control-Lyapunov law of Ebrahimi Toulkani et
al., "Reactive Safe Path Following for Differential Drive Mobile Robots Using Control
Barrier Functions" (ICCMA 2022), **Proposition 1**. A virtual target Q rides the path
at arc length ``s``; the law drives the along-track (``x_e``), cross-track (``y_e``)
and heading (``ψ_e``) errors to zero with guaranteed convergence at a constant cruise
speed ``v`` — the Lyapunov function ``V = ½(x_e² + y_e² + (ψ_e − σ)²)`` has
``V̇ ≤ 0`` for non-zero ``v``. Only the angular velocity ``ω`` is fed back; ``v`` is
held at cruise. It is structured so a Control-Barrier-Function QP can later wrap ``ω``
for safe obstacle avoidance (the paper's second contribution) — out of scope here.
Arrival = the robot reaching the last waypoint (within ``reach_radius``).

State dynamics mirror ``visual_servo``: one goal at a time, ``IDLE`` while the
tracker initialises, ``RUNNING`` while following, ``WAITING`` while ``OCCLUDED``;
``init_timeout`` / ``occlusion_timeout`` failures; ``stop_tracking`` on result.
"""

import math
import threading

import numpy as np
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

# The robot is the base_link origin (0, 0) heading +x; waypoints arrive as metric
# base_link coords (x forward, y left), so distances are plain Euclidean, in metres.


def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PursuitServoNode(Node):
    def __init__(self):
        super().__init__("pursuit_servo_node")

        # --- Parameters ---  (metric, top-down world space)
        self.declare_parameter("cruise_speed", 0.05)  # m/s — constant path speed v_ref
        self.declare_parameter("max_linear_vel", 0.26)  # m/s cap (Waffle Pi rated max)
        self.declare_parameter("max_angular_vel", 1.82)  # rad/s cap
        self.declare_parameter("init_timeout", 5.0)  # s to reach TRACKING before fail
        self.declare_parameter("occlusion_timeout", 5.0)  # s in OCCLUDED before abort
        self.declare_parameter("control_rate", 20.0)  # Hz
        self.declare_parameter("reach_radius", 0.15)  # m — arrival radius at last wp
        # Control-Lyapunov path-following gains (paper Proposition 1; their tuned
        # values k1=2, k2=1, k3=1, eps0=0.35). k1 damps the heading/approach error,
        # k2 (0..1) sets how hard cross-track error bends the approach angle, k3 pulls
        # the virtual target's along-track error to zero, eps0 softens the sigma law
        # near the path. ``curvature_window`` is the arc-length span (m) over which the
        # path tangent is differenced to estimate curvature C_c on the coarse polyline.
        self.declare_parameter("k1", 2.0)
        self.declare_parameter("k2", 1.0)
        self.declare_parameter("k3", 1.0)
        self.declare_parameter("eps0", 0.35)
        self.declare_parameter("curvature_window", 0.15)  # m

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
        # Virtual-target arc length along the path (paper's ``s``); ``None`` until the
        # first tracking tick seeds it at the robot's closest point on the path.
        self._s = None

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
        self._s = None  # reset the virtual-target arc length for this goal

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

            # Control-Lyapunov path-following step over the waypoint polyline.
            v, w, done = self._clf_step(self._waypoints)

            if done:  # robot reached the last waypoint — trajectory complete
                self._call_stop_tracking()
                self._stop_robot()
                result = FollowTrajectory.Result()
                result.success = True
                result.message = "Trajectory complete"
                goal_handle.succeed()
                return result

            if v is None:  # no usable path this tick — hold still
                self._stop_robot()
                self._publish_feedback(goal_handle, "RUNNING")
                rate.sleep()
                continue

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
    # Control-Lyapunov path following (paper Proposition 1)

    def _clf_step(self, waypoints):
        """One Control-Lyapunov path-following step over the waypoint polyline.

        Returns ``(v, w, done)``: linear/angular command and an arrival flag, or
        ``(None, None, False)`` when there is no usable path this tick. The polyline
        (all waypoints, in order, metric base_link) is the path Γ; a virtual target Q
        rides it at arc length ``self._s``. With the robot at the base_link origin
        heading +x, the tangent-normal error coordinates are ``x_e`` (along-track),
        ``y_e`` (cross-track) and ``ψ_e`` (heading), and the control is::

            σ    = -asin( clamp(k2·y_e / (|y_e|+ε0)) )          (6a, v>0)
            ṡ    = v·cos(ψ_e) + k3·x_e                           (6b)
            Δ    = (sin ψ_e − sin σ)/(ψ_e − σ)   (→ cos σ at ψ_e=σ)   (6c)
            ω    = C_c·ṡ + σ̇ − k1·(ψ_e − σ) − v·y_e·Δ

        driving ``V = ½(x_e²+y_e²+(ψ_e−σ)²) → 0``. ``v`` is held at cruise.
        """
        if not waypoints or len(waypoints) < 2:
            return None, None, False
        pts = np.array([(x, y) for x, y, _ in waypoints], dtype=float)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg)])  # cumulative arc length
        total = float(arc[-1])
        if total < 1e-6:
            return None, None, False

        # Arrival: robot (origin) within reach_radius of the last waypoint.
        if float(np.hypot(pts[-1, 0], pts[-1, 1])) <= float(self._p("reach_radius")):
            return 0.0, 0.0, True

        if self._s is None:  # seed the virtual target at the robot's closest point
            self._s = self._closest_s(pts, arc)
        self._s = min(max(self._s, 0.0), total)

        q, psi_t, kappa = self._locate(self._s, pts, arc)

        # Tangent-normal error coords (robot at origin, heading ψ_B = 0 in base_link):
        # [x_e; y_e] = R(-ψ_t)·(p_robot - q) with p_robot = 0.
        ct, st = math.cos(psi_t), math.sin(psi_t)
        x_e = -(q[0] * ct + q[1] * st)
        y_e = q[0] * st - q[1] * ct
        psi_e = _wrap(-psi_t)

        v = min(float(self._p("cruise_speed")), float(self._p("max_linear_vel")))
        k1, k2 = float(self._p("k1")), float(self._p("k2"))
        k3, eps0 = float(self._p("k3")), float(self._p("eps0"))

        # Approach angle σ and its analytic rate σ̇ (v > 0 so sign(v) = +1).
        u = max(-0.999, min(0.999, k2 * y_e / (abs(y_e) + eps0)))
        sigma = -math.asin(u)
        s_dot = v * math.cos(psi_e) + k3 * x_e                     # (6b)
        y_e_dot = x_e * kappa * s_dot + v * math.sin(psi_e)        # (4b)
        du_dye = k2 * eps0 / (abs(y_e) + eps0) ** 2
        dsigma_dye = -du_dye / max(math.sqrt(max(1.0 - u * u, 0.0)), 0.05)
        sigma_dot = dsigma_dye * y_e_dot

        # Δ (6c): (sin ψ_e − sin σ)/(ψ_e − σ), analytic limit cos σ at ψ_e = σ.
        dpe = psi_e - sigma
        delta = (math.cos(sigma) if abs(dpe) < 1e-6
                 else (math.sin(psi_e) - math.sin(sigma)) / dpe)

        w = kappa * s_dot + sigma_dot - k1 * dpe - v * y_e * delta

        # Advance the virtual target; complete if it (and the robot) passed the end.
        self._s = min(self._s + s_dot / float(self._p("control_rate")), total)
        if self._s >= total - 1e-3 and x_e <= 0.0:
            return 0.0, 0.0, True

        max_w = float(self._p("max_angular_vel"))
        w = max(-max_w, min(max_w, w))
        return v, w, False

    def _closest_s(self, pts, arc):
        """Arc length of the point on the polyline closest to the robot (origin)."""
        best_d, best_s = float("inf"), 0.0
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            ab = b - a
            ab2 = float(ab @ ab)
            t = 0.0 if ab2 < 1e-12 else min(1.0, max(0.0, float((-a) @ ab) / ab2))
            proj = a + t * ab
            d = float(np.hypot(proj[0], proj[1]))
            if d < best_d:
                best_d, best_s = d, float(arc[i] + t * math.sqrt(ab2))
        return best_s

    def _seg_index(self, s, arc):
        i = int(np.searchsorted(arc, s, side="right")) - 1
        return min(max(i, 0), len(arc) - 2)

    def _locate(self, s, pts, arc):
        """Return ``(q, ψ_t, C_c)`` at arc length ``s`` on the polyline."""
        i = self._seg_index(s, arc)
        seg = arc[i + 1] - arc[i]
        t = 0.0 if seg < 1e-9 else (s - arc[i]) / seg
        q = pts[i] + t * (pts[i + 1] - pts[i])
        psi_t = self._tangent_at(s, pts, arc)
        # Curvature C_c ≈ dψ_t/ds, tangent differenced over ``curvature_window`` so the
        # coarse polyline's per-vertex angle jumps read as a finite curvature.
        dw = float(self._p("curvature_window"))
        s_hi, s_lo = min(arc[-1], s + dw), max(0.0, s - dw)
        ds = s_hi - s_lo
        kappa = (_wrap(self._tangent_at(s_hi, pts, arc)
                       - self._tangent_at(s_lo, pts, arc)) / ds) if ds > 1e-6 else 0.0
        return q, psi_t, kappa

    def _tangent_at(self, s, pts, arc):
        i = self._seg_index(s, arc)
        d = pts[i + 1] - pts[i]
        return math.atan2(d[1], d[0])

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
