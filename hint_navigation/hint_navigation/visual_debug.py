"""visual_debug — one composited /debug image for HINT live visualization.

Instead of every node shipping its own debug image, each node publishes only its real
output data and this node layers those outputs into a **single** `/debug` image:

- the camera frame as the backdrop,
- the binary ground mask as a green/red overlay,
- the trajectory navigator's followed (ground-clipped) and raw (full VLM-intent) paths,
  re-projected onto the frame via the current odometry pose so they track as the robot moves,
- the mission (BT) tree's live state as text, top-left.

Every layer toggles via a `show_*` parameter. Rendering is subscriber-gated: nothing is
composed or published unless something subscribes to `/debug`.

Lives in hint_navigation (not perception): it needs the camera rig to re-project the
navigator's odom paths, and most of what it draws (paths, BT state) is navigation state —
so it shares the rig (`camera_rig.CameraRig`) with the other navigation nodes.
"""

import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from hint_navigation.camera_rig import CameraRig

# Latest-wins for the streaming inputs; latched for the once-published state/paths.
_LATEST = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Muted palette (BGR) — deliberately no full green / full red.
_C_GROUND = (120, 190, 120)      # soft green
_C_NONGROUND = (110, 110, 215)   # soft coral
_C_PATH_RAW = (70, 170, 235)     # amber — the full VLM intent
_C_PATH = (190, 190, 90)         # teal — the followed (clipped) path
_C_TEXT = (240, 240, 240)


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class VisualDebugNode(Node):
    def __init__(self):
        super().__init__("visual_debug_node")
        self.bridge = CvBridge()

        # --- Camera rig (shared with the other navigation nodes; live-adjustable) ---
        CameraRig.declare(self)
        # --- Layer toggles ---
        self.declare_parameter("show_mask", True)
        self.declare_parameter("show_path", True)
        self.declare_parameter("show_path_raw", True)
        self.declare_parameter("show_bt_state", True)
        self.declare_parameter("overlay_alpha", 0.35)
        # --- Input topics ---
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("mask_topic", "/camera/ground")
        self.declare_parameter("path_topic", "/trajectory_navigator_node/path")
        self.declare_parameter("path_raw_topic", "/trajectory_navigator_node/path_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("bt_state_topic", "/hint_behavior_server/state")

        self._lock = threading.Lock()
        self._mask = None
        self._path = None          # list of (x, y) in odom
        self._path_raw = None
        self._pose = None          # (x, y, yaw) latest odom
        self._bt_state = "IDLE"

        self.create_subscription(Image, str(self._p("mask_topic")), self._mask_cb, _LATEST)
        self.create_subscription(Path, str(self._p("path_topic")), self._path_cb, _LATCHED)
        self.create_subscription(
            Path, str(self._p("path_raw_topic")), self._path_raw_cb, _LATCHED)
        self.create_subscription(Odometry, str(self._p("odom_topic")), self._odom_cb, 20)
        self.create_subscription(String, str(self._p("bt_state_topic")), self._bt_cb, _LATCHED)
        # The camera frame drives the render loop; subscribe last.
        self.create_subscription(
            CompressedImage, str(self._p("image_topic")), self._image_cb, _LATEST)

        self.pub_debug = self.create_publisher(Image, "/debug", 1)
        self.get_logger().info("visual_debug ready — composited /debug image.")

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Caching subscriptions

    def _mask_cb(self, msg):
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception:
            return
        with self._lock:
            self._mask = mask

    def _path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._path = pts

    def _path_raw_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._path_raw = pts

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._pose = (p.x, p.y, yaw)

    def _bt_cb(self, msg):
        with self._lock:
            self._bt_state = msg.data

    # ------------------------------------------------------------------
    # Projection: odom path -> current base_link -> image pixels

    def _project_path(self, rig, pts_odom, pose, w, h):
        """odom (x, y) points -> in-front image (u, v) via the current robot pose."""
        if not pts_odom or pose is None:
            return []
        rx, ry, ryaw = pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        a = np.array(pts_odom, dtype=float)
        dx, dy = a[:, 0] - rx, a[:, 1] - ry
        base = np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)  # base_link (X fwd, Y left)
        pix, infront = rig.ground_to_pixels(base, w, h)
        return [tuple(np.round(pix[i]).astype(int)) for i in range(len(pix)) if infront[i]]

    # ------------------------------------------------------------------
    # Render

    def _image_cb(self, msg):
        if self.pub_debug.get_subscription_count() == 0:
            return
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        if frame is None:  # truncated JPEG decodes to None without raising
            return
        h, w = frame.shape[:2]
        with self._lock:
            mask, path, path_raw = self._mask, self._path, self._path_raw
            pose, bt = self._pose, self._bt_state

        if bool(self._p("show_mask")) and mask is not None:
            m = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0
            tint = np.where(m[..., None], _C_GROUND, _C_NONGROUND).astype(np.float32)
            a = float(np.clip(self._p("overlay_alpha"), 0.0, 1.0))
            frame[:] = ((1.0 - a) * frame.astype(np.float32) + a * tint).astype(np.uint8)

        rig = CameraRig.from_node(self)  # snapshot current rig (live-adjustable)
        # Raw (intent) first, followed (clipped) on top.
        if bool(self._p("show_path_raw")) and path_raw:
            self._draw_polyline(frame, self._project_path(rig, path_raw, pose, w, h), _C_PATH_RAW)
        if bool(self._p("show_path")) and path:
            self._draw_polyline(
                frame, self._project_path(rig, path, pose, w, h), _C_PATH, dots=True)

        if bool(self._p("show_bt_state")):
            self._draw_state(frame, bt)

        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self.pub_debug.publish(out)

    @staticmethod
    def _draw_polyline(frame, px, color, dots=False):
        if len(px) >= 2:
            cv2.polylines(frame, [np.array(px, np.int32)], False, color, 2, cv2.LINE_AA)
        if dots:
            for u, v in px:
                cv2.circle(frame, (u, v), 4, color, -1, cv2.LINE_AA)

    @staticmethod
    def _draw_state(frame, text):
        label = f"BT: {text}"
        font = cv2.FONT_HERSHEY_DUPLEX
        scale, thick = 0.6, 1
        (tw, th), base = cv2.getTextSize(label, font, scale, thick)
        x, y = 12, 16 + th
        overlay = frame.copy()
        cv2.rectangle(overlay, (x - 7, y - th - 9), (x + tw + 7, y + base + 5),
                      (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        cv2.putText(frame, label, (x, y), font, scale, _C_TEXT, thick, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = VisualDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
