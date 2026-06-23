"""Gap detector node.

Finds the largest free-space gap along the horizontal bottom band of a mono8
depth image (black = farthest = safe) and publishes its normalised image-plane
coordinate as a PointStamped (u ∈ [-1,+1], v=0, z=0).
Sampling near the bottom of the frame captures ground-level obstacles for a
wheeled robot. A 2D depth graph is drawn above the sample row in the debug image.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class GapDetectorNode(Node):
    def __init__(self):
        super().__init__("gap_detector_node")
        self.bridge = CvBridge()

        # Fractional row position (0=top, 1=bottom) to sample; near the bottom
        # captures ground-level obstacles relevant to a wheeled robot.
        self.declare_parameter("sample_row_ratio", 0.85)
        # Fraction of image height sampled around the sample row.
        self.declare_parameter("band_height_ratio", 0.1)
        # Gaussian kernel width (pixels) for 1-D profile smoothing; must be odd.
        self.declare_parameter("smoothing_kernel", 7)

        self.create_subscription(Image, "/camera/depth/image_raw", self._cb, 1)
        self._pub_point = self.create_publisher(PointStamped, "/gap_detector/point", 10)
        self._pub_debug = self.create_publisher(Image, "/gap_detector/debug", 10)

        self.get_logger().info("Gap detector ready.")

    def _p(self, name):
        return self.get_parameter(name).value

    def _cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "mono8")  # H×W, black=farthest
        h, w = depth.shape
        sample_row = int(h * self._p("sample_row_ratio"))

        # --- 1-D depth profile from a horizontal band at ground level ---
        band_half = max(1, int(h * self._p("band_height_ratio") / 2))
        row_lo = max(0, sample_row - band_half)
        row_hi = min(h, sample_row + band_half)
        profile = depth[row_lo:row_hi, :].mean(axis=0).astype(np.float32)

        # Gaussian smoothing collapses per-pixel noise into a smooth envelope.
        k = int(self._p("smoothing_kernel")) | 1  # force odd
        profile_smooth = cv2.GaussianBlur(profile.reshape(1, -1), (k, 1), 0).reshape(-1)

        # Trough column = darkest = farthest = safest direction.
        gap_x = int(np.argmin(profile_smooth))

        # --- Publish normalised point ---
        # u ∈ [-1, +1]: -1 = left edge, 0 = centre, +1 = right edge
        u = (gap_x - w / 2.0) / (w / 2.0)
        pt = PointStamped()
        pt.header = msg.header
        pt.point.x = u
        pt.point.y = 0.0
        pt.point.z = 0.0
        self._pub_point.publish(pt)

        # --- Debug image ---
        debug = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)

        # Draw the depth graph above the sample row as a translucent red overlay.
        # Invert p_norm so that darker (farther) pixels produce taller bars.
        graph_h = h // 3
        p_norm = 1.0 - profile_smooth / (profile_smooth.max() + 1e-5)
        overlay = debug.copy()
        for x in range(w):
            bar_top = sample_row - int(p_norm[x] * graph_h)
            cv2.line(overlay, (x, sample_row), (x, bar_top), (0, 0, 200), 1)
        cv2.addWeighted(overlay, 0.4, debug, 0.6, 0, debug)

        # Highlight the gap column peak in the graph (opaque red).
        peak_top = sample_row - int(p_norm[gap_x] * graph_h)

        # Red cross at the gap position on the sample row.
        cv2.drawMarker(debug, (gap_x, sample_row), (0, 0, 255), cv2.MARKER_CROSS, 12, 1)

        self._pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = GapDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
