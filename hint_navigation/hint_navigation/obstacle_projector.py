import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from hint_navigation.camera_rig import CameraRig

# RELIABLE matches the segmenter's mask publisher (also serves best-effort subs).
_LATEST_MASK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)


class ObstacleProjectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_projector_node")
        self.bridge = CvBridge()

        CameraRig.declare(self)

        self.declare_parameter("bev_range", 3.0)        # m forward coverage
        self.declare_parameter("bev_half_width", 1.5)   # m lateral each side
        self.declare_parameter("bev_resolution", 0.05)  # m per cell (~ costmap res)
        self.declare_parameter("obstacle_frame", "base_link")
        self.declare_parameter("mask_topic", "/camera/ground")

        self.create_subscription(
            Image, str(self._p("mask_topic")), self.callback, _LATEST_MASK_QOS
        )
        self.pub_obstacles = self.create_publisher(PointCloud2, "/obstacles", 5)

        self.get_logger().info(
            "obstacle_projector ready — /camera/ground mask -> /obstacles cloud."
        )

    def _p(self, name):
        return self.get_parameter(name).value

    def _bev_geom(self):
        res = max(1e-3, float(self._p("bev_resolution")))
        rng = float(self._p("bev_range"))
        half = float(self._p("bev_half_width"))
        return (max(2, int(round(rng / res))), max(2, int(round(2 * half / res))),
                res, rng, half)

    def _metric_to_cell(self, x, y, res, rng, half):
        return (rng - x) / res - 0.5, (half - y) / res - 0.5  # (row, col)

    def _mask_to_obstacle_cells(self, rig, ground, w, h):
        rows, cols, res, rng, half = self._bev_geom()
        x_off = rig.forward_offset
        x_near = max(0.25 * rng, x_off + 0.2)
        gx = np.array([[rng, half], [rng, -half], [x_near, half], [x_near, -half]])
        img_pts = rig.ground_to_pixels(gx, w, h)[0].astype(np.float32)
        bev_pts = np.array(
            [self._metric_to_cell(x, y, res, rng, half)[::-1] for x, y in gx],
            dtype=np.float32,
        )
        m = cv2.getPerspectiveTransform(bev_pts, img_pts)

        flags = cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP
        g = cv2.warpPerspective(
            ground.astype(np.uint8), m, (cols, rows), flags=flags, borderValue=0) > 0
        known = cv2.warpPerspective(
            np.full((h, w), 255, np.uint8), m, (cols, rows), flags=flags, borderValue=0
        ) > 0
        # Rows whose ground points sit at/behind the image plane sample garbage.
        cell_x = rng - (np.arange(rows) + 0.5) * res
        behind = (math.cos(rig.tilt) * (cell_x - x_off)
                  + math.sin(rig.tilt) * rig.height) <= 1e-3
        known[behind, :] = False
        g[behind, :] = False

        ys, xs = np.nonzero(known & ~g)
        mx = rng - (ys + 0.5) * res            # metric x forward
        my = half - (xs + 0.5) * res           # metric y left
        return mx, my

    def callback(self, msg):
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:
            self.get_logger().warn(f"Skipping undecodable mask: {e}")
            return
        if mask is None:
            return

        rig = CameraRig.from_node(self)
        ground = mask > 0
        h, w = ground.shape[:2]
        mx, my = self._mask_to_obstacle_cells(rig, ground, w, h)

        header = Header()
        header.stamp = msg.header.stamp  # source-frame stamp -> Nav2 latency compensation
        header.frame_id = str(self._p("obstacle_frame"))
        pts = np.stack([mx, my, np.zeros_like(mx)], axis=1).astype(np.float32)
        self.pub_obstacles.publish(point_cloud2.create_cloud_xyz32(header, pts))


def main():
    rclpy.init()
    node = ObstacleProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
