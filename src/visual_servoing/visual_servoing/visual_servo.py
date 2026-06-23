import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import RegionOfInterest


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # --- Image geometry ---
        self.declare_parameter("image_width",  640)
        self.declare_parameter("image_height", 480)

        # --- Control gains ---
        # Angular: proportional on normalised centre error e ∈ [-1, 1] → rad/s
        self.declare_parameter("k_yaw", 1.5)
        # Linear: proportional on remaining area-ratio error → m/s
        self.declare_parameter("k_lin", 2.5)

        # --- Limits ---
        self.declare_parameter("max_linear_vel",  0.26)   # m/s  (Waffle Pi rated max)
        self.declare_parameter("max_angular_vel", 1.82)   # rad/s

        # --- Stopping condition ---
        # Robot stops when the bbox covers this fraction of the image area.
        self.declare_parameter("stop_area_ratio", 0.75)

        # --- Staleness guard ---
        # If no bbox arrives within this window (seconds), publish zero velocity.
        self.declare_parameter("bbox_timeout", 0.5)

        self.create_subscription(RegionOfInterest, "/tracking/bbox", self._bbox_cb, 10)
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # 20 Hz control loop — decoupled from bbox publication rate
        self.create_timer(0.05, self._control_loop)

        self._bbox      = None
        self._bbox_time = None

        self.get_logger().info("Visual servoing ready — listening on /tracking/bbox.")

    def _p(self, name):
        return self.get_parameter(name).value

    def _bbox_cb(self, msg):
        self._bbox      = msg
        self._bbox_time = self.get_clock().now()

    def _control_loop(self):
        now = self.get_clock().now()
        cmd = TwistStamped()
        cmd.header.stamp    = now.to_msg()
        cmd.header.frame_id = "base_link"

        # Publish zero and bail if bbox is absent or stale
        stale = self._bbox is None or (
            (now - self._bbox_time).nanoseconds * 1e-9 > self._p("bbox_timeout")
        )
        if stale:
            self._pub.publish(cmd)
            return

        img_w = float(self._p("image_width"))
        img_h = float(self._p("image_height"))
        bbox  = self._bbox

        # ----------------------------------------------------------------
        # Angular velocity — keep bbox centre aligned with image centre
        #
        # Normalised horizontal error: +1 = target fully left, -1 = fully right
        cx_error = (img_w / 2.0 - (bbox.x_offset + bbox.width / 2.0)) / (img_w / 2.0)
        w = self._p("k_yaw") * cx_error
        w = max(-self._p("max_angular_vel"), min(self._p("max_angular_vel"), w))

        # ----------------------------------------------------------------
        # Linear velocity — proportional approach; stops at stop_area_ratio
        #
        # As the robot closes in, bbox_area / image_area grows toward stop_area_ratio.
        # Velocity ramps down naturally and reaches zero exactly at the target.
        area_ratio = (float(bbox.width) * float(bbox.height)) / (img_w * img_h)
        stop_ratio = self._p("stop_area_ratio")
        v = self._p("k_lin") * max(0.0, stop_ratio - area_ratio)
        v = min(v, self._p("max_linear_vel"))

        cmd.twist.linear.x  = v
        cmd.twist.angular.z = w
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
