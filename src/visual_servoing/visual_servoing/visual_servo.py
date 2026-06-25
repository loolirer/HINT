import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import String

from hint_interfaces.action import ApproachTarget
from hint_interfaces.srv import SetTarget, StopTracking

STATUS_UNTRACKED = "UNTRACKED"
STATUS_TRACKING = "TRACKING"
STATUS_OCCLUDED = "OCCLUDED"

_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # --- Parameters ---
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("k_yaw", 0.20)
        self.declare_parameter("k_lin", 0.25)
        self.declare_parameter("max_linear_vel", 0.26)
        self.declare_parameter("max_angular_vel", 1.82)
        self.declare_parameter("stop_area_ratio", 0.75)
        self.declare_parameter("min_linear_vel", 0.05)  # m/s — robot dead zone floor
        self.declare_parameter("min_angular_vel", 0.05)  # rad/s — robot dead zone floor
        self.declare_parameter("init_timeout", 5.0)
        self.declare_parameter("control_rate", 20.0)

        # --- Tracker service clients ---
        self._set_target_cli = self.create_client(
            SetTarget, "/lk_tracker_node/set_target"
        )
        self._stop_tracking_cli = self.create_client(
            StopTracking, "/lk_tracker_node/stop_tracking"
        )

        # --- Subscriptions ---
        self.create_subscription(RegionOfInterest, "/tracking/bbox", self._bbox_cb, 10)
        self.create_subscription(
            String, "/tracking/state", self._state_cb, _LATCHED_QOS
        )

        # --- Publisher ---
        self._cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # --- Shared state (written by subscriber threads, read by action loop) ---
        self._bbox = None
        self._tracker_state = STATUS_UNTRACKED

        # Lock to serialise goal acceptance so only one goal runs at a time.
        self._goal_lock = threading.Lock()

        # --- Action server ---
        # ReentrantCallbackGroup allows the execute callback to run concurrently
        # with subscriber callbacks on the MultiThreadedExecutor.
        self._action_server = ActionServer(
            self,
            ApproachTarget,
            "~/approach_target",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            "Visual servoing ready — call ~/approach_target to start."
        )

    # ------------------------------------------------------------------
    # Action callbacks

    def _goal_cb(self, goal_request):
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn(
                "Rejecting goal — another approach is already running."
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
        self._bbox = None  # discard any bbox from a previous run

        # Delegate initialisation to the tracker.
        resp = self._call_set_target(goal.roi, goal.stamp)
        if resp is None or not resp.accepted:
            self._stop_robot()
            result = ApproachTarget.Result()
            result.success = False
            result.message = "Tracker rejected target"
            goal_handle.abort()
            return result

        rate = self.create_rate(float(self._p("control_rate")))
        init_timeout = float(self._p("init_timeout"))
        deadline = self.get_clock().now()
        ever_tracking = False

        while rclpy.ok():
            # --- Cancellation ---
            if goal_handle.is_cancel_requested:
                self._call_stop_tracking()
                self._stop_robot()
                goal_handle.canceled()
                result = ApproachTarget.Result()
                result.success = False
                result.message = "Cancelled by client"
                return result

            state = self._tracker_state

            # --- UNTRACKED: waiting for init or target completely lost ---
            if state == STATUS_UNTRACKED:
                self._stop_robot()
                elapsed = (self.get_clock().now() - deadline).nanoseconds * 1e-9
                if ever_tracking or elapsed > init_timeout:
                    self._call_stop_tracking()
                    result = ApproachTarget.Result()
                    result.success = False
                    result.message = (
                        "Target lost" if ever_tracking else "Initialisation timeout"
                    )
                    goal_handle.abort()
                    return result
                self._publish_feedback(goal_handle, "IDLE")
                rate.sleep()
                continue

            # --- OCCLUDED: stop robot and wait for recovery ---
            if state == STATUS_OCCLUDED:
                self._stop_robot()
                self._publish_feedback(goal_handle, "WAITING")
                rate.sleep()
                continue

            # --- TRACKING ---
            ever_tracking = True
            deadline = self.get_clock().now()  # reset so brief UNTRACKED fails fast

            bbox = self._bbox
            if bbox is None:
                rate.sleep()
                continue

            img_w = float(self._p("image_width"))
            img_h = float(self._p("image_height"))
            stop_ratio = float(self._p("stop_area_ratio"))
            area_ratio = (float(bbox.width) * float(bbox.height)) / (img_w * img_h)

            # --- Stopping condition ---
            if area_ratio >= stop_ratio:
                self._call_stop_tracking()
                self._stop_robot()
                result = ApproachTarget.Result()
                result.success = True
                result.message = "Reached target"
                goal_handle.succeed()
                return result

            # --- Control law (unchanged from original) ---
            cx_error = (img_w / 2.0 - (bbox.x_offset + bbox.width / 2.0)) / (
                img_w / 2.0
            )
            w = self._p("k_yaw") * cx_error
            w = max(-self._p("max_angular_vel"), min(self._p("max_angular_vel"), w))

            v = self._p("k_lin") * max(0.0, stop_ratio - area_ratio)
            v = min(v, float(self._p("max_linear_vel")))

            # If both outputs are below the robot's dead zone, declare arrived.
            if v < self._p("min_linear_vel") and abs(w) < self._p("min_angular_vel"):
                self._call_stop_tracking()
                self._stop_robot()
                result = ApproachTarget.Result()
                result.success = True
                result.message = "Reached target (velocity below minimum threshold)"
                goal_handle.succeed()
                return result

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
        result = ApproachTarget.Result()
        result.success = False
        result.message = "Node shutdown"
        return result

    # ------------------------------------------------------------------
    # Subscriber callbacks

    def _bbox_cb(self, msg):
        self._bbox = msg

    def _state_cb(self, msg):
        self._tracker_state = msg.data

    # ------------------------------------------------------------------
    # Helpers

    def _p(self, name):
        return self.get_parameter(name).value

    def _publish_feedback(self, goal_handle, state):
        fb = ApproachTarget.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self._cmd_pub.publish(cmd)

    def _call_set_target(self, roi, stamp):
        req = SetTarget.Request()
        req.roi = roi
        req.stamp = stamp
        return self._call_sync(self._set_target_cli, req)

    def _call_stop_tracking(self):
        self._call_sync(self._stop_tracking_cli, StopTracking.Request())

    def _call_sync(self, client, request, timeout=2.0):
        """Call a service synchronously from within the action execute thread.

        Uses threading.Event so the MultiThreadedExecutor can process the
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
    node = VisualServoingNode()
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
