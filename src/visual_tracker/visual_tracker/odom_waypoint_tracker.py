"""Odometry waypoint tracker — dead-reckoned ground-plane re-projection.

A drop-in sibling of ``waypoint_tracker`` that keeps the **same tracking
dynamics** (state machine, per-waypoint measured/coasting flags, nearest-prefix
occlusion, publish-only-while-``TRACKING`` contract) but
swaps the *measurement backend*: instead of estimating where the waypoints moved
with DIS optical flow, it predicts their positions **purely from odometry** and a
pin-hole camera model. No image content is used to place the points.

The scene is assumed planar (every waypoint on the ground), so the pipeline is:

1. **Ground the waypoints once.** On ``set_waypoints`` each normalized image point
   is back-projected through the camera (``camera_height``, ``camera_tilt``,
   ``camera_hfov_deg``) onto the ground plane, giving a fixed 2-D ground point
   ``(X, Y)`` expressed in the **reference frame** — the robot's odometry pose at
   the instant the trajectory was received. This reference frame is re-anchored
   every time a new trajectory arrives.
2. **Dead-reckon the camera.** Each camera frame, the robot's current odometry
   pose is expressed relative to the reference pose (a planar rigid transform), so
   the fixed ground points are re-expressed in the *current* robot frame.
3. **Re-project.** The current-frame ground points are projected back through the
   same camera model to pixels, then published as the normalized waypoint stream —
   exactly the ``waypoint_tracker`` output contract, consumed unchanged by
   ``pursuit_servo``.

Because the placement is a geometric prediction rather than a visual measurement,
"occlusion" here means **odometry loss**: the near waypoints all leaving the frame
or projecting behind the camera, or ``/odom`` going stale. A waypoint is *measured*
(``tracked=true``) only while it projects in front of the camera **and** inside the
frame; otherwise it *coasts* (still placed by the prediction, flagged
``tracked=false``, clamped to the frame edge). The tracker **never retires**
waypoints — it re-projects and publishes the whole set every frame. Deciding when a
waypoint has been *reached* (driven over) and advancing through the trajectory is
the follower's job (``pursuit_servo``), since "reaching" is an act of the servo, not
the tracker.

Camera convention: OpenCV optical frame (x right, y down, z into scene); robot
frame REP-103 (x forward, y left, z up); the camera sits ``camera_height`` above
and ``camera_forward_offset`` ahead of the base origin, pitched ``camera_tilt``
radians **down** from horizontal. Principal point assumed at the image center.
"""

from collections import deque
from math import atan2, tan, radians

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point

from hint_interfaces.msg import VisualWaypoints
from hint_interfaces.srv import SetWaypoints, StopTracking
from visual_tracker.tracking_common import (
    STATUS_UNTRACKED,
    STATUS_TRACKING,
    STATUS_OCCLUDED,
    LATCHED_QOS,
)


def _yaw_from_quat(q):
    """Planar yaw (rad) from a ``geometry_msgs/Quaternion``."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny, cosy)


class OdomWaypointTrackerNode(Node):
    def __init__(self):
        super().__init__("odom_waypoint_tracker_node")

        # --- Camera / geometry parameters ---
        # Camera pose on the robot. Defaults are ballpark for a Waffle Pi front
        # camera — set them to the real rig for accurate grounding.
        self.declare_parameter("camera_height", 0.14)  # m above the ground plane
        self.declare_parameter("camera_forward_offset", 0.0)  # m ahead of base origin
        self.declare_parameter("camera_tilt", 0.0)  # rad, positive = pitched down
        self.declare_parameter("camera_hfov_deg", 62.2)  # horizontal FOV (Pi cam v2)

        # --- Tracking parameters (mirror waypoint_tracker where meaningful) ---
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_timeout", 0.5)  # s; older -> OCCLUDED
        # The nearest ``priority_count`` in-frame waypoints govern node state: the
        # node stays TRACKING while >=1 of them is measured, OCCLUDED when the near
        # end all leaves the frame / projects behind the camera.
        self.declare_parameter("priority_count", 2)

        # --- Publishers (identical contract to waypoint_tracker) ---
        self.pub_points = self.create_publisher(
            VisualWaypoints, "/waypoint_tracking/points", 10
        )
        self.pub_debug = self.create_publisher(
            Image, "/camera/waypoint_tracking", 10
        )
        self.pub_state = self.create_publisher(
            String, "/waypoint_tracking/state", LATCHED_QOS
        )

        # --- Services ---
        self.create_service(SetWaypoints, "~/set_waypoints", self._srv_set_waypoints)
        self.create_service(StopTracking, "~/stop_tracking", self._srv_stop_tracking)

        # --- Internal state ---
        self._pending_waypoints = None  # Nx2 normalized, awaiting init
        self._pending_stamp = None
        self._odom_buffer = deque(maxlen=200)  # (t_float, x, y, yaw) for stamp init
        self.bridge = CvBridge()

        self._reset(STATUS_UNTRACKED)

        # Re-publish state every second so late subscribers catch up.
        self.create_timer(1.0, lambda: self._publish_state(self.tracking_status))

        # Odometry first, then images — a frame needs a pose to project against.
        self.create_subscription(
            Odometry, str(self._p("odom_topic")), self._odom_callback, 20
        )
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10
        )

        self.get_logger().info(
            "Odometry waypoint tracker ready — call ~/set_waypoints to start."
        )

    # ------------------------------------------------------------------
    # Services

    def _srv_set_waypoints(self, request, response):
        self._reset(STATUS_UNTRACKED)
        if len(request.waypoints) == 0:
            response.accepted = False
            response.message = "No waypoints provided."
            return response
        wps = np.array([[p.x, p.y] for p in request.waypoints], dtype=np.float32)
        if not np.all((wps >= -1.0) & (wps <= 1.0)):
            response.accepted = False
            response.message = "Waypoints must be normalized to [-1, 1]."
            return response
        self._pending_waypoints = wps
        self._pending_stamp = request.stamp
        response.accepted = True
        response.message = f"Accepted {len(wps)} waypoints."
        self.get_logger().info(f"New trajectory received: {len(wps)} waypoints.")
        return response

    def _srv_stop_tracking(self, request, response):
        self._pending_waypoints = None
        self._pending_stamp = None
        self._reset(STATUS_UNTRACKED)
        self.get_logger().info("Waypoint tracking stopped by stop_tracking.")
        return response

    # ------------------------------------------------------------------
    # State

    def _reset(self, status):
        self.initialized = False
        self.pts = None  # (N,2) current waypoint pixel positions
        self.ground_ref = None  # (N,2) ground points in the reference frame
        self.ref_pose = None  # (x, y, yaw) odometry pose when trajectory received
        self.img_shape = None  # (h, w)
        self._ever_tracked = False  # a valid projection has landed since init
        # Debug/telemetry
        self._tracked_mask = None  # (N,) bool: per-waypoint measured vs coasting
        self._rel = (0.0, 0.0, 0.0)  # current pose relative to the reference
        self._set_status(status)

    def _set_status(self, status):
        self.tracking_status = status
        self._publish_state(status)

    def _publish_state(self, status):
        msg = String()
        msg.data = status
        self.pub_state.publish(msg)

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Camera model (derived each call so live param edits take effect)

    def _camera(self, w, h):
        """Return ``(f, cx, cy, cos_t, sin_t, cam_h, x_off)`` for the current frame.

        ``f`` is the focal length in pixels from the horizontal FOV (square pixels,
        principal point at the image center).
        """
        hfov = radians(float(self._p("camera_hfov_deg")))
        f = (w / 2.0) / tan(hfov / 2.0)
        tilt = float(self._p("camera_tilt"))
        return (
            f, (w - 1) / 2.0, (h - 1) / 2.0,
            np.cos(tilt), np.sin(tilt),
            float(self._p("camera_height")),
            float(self._p("camera_forward_offset")),
        )

    def _pixels_to_ground(self, pts, w, h):
        """Back-project pixel points (N,2) onto the ground plane (robot frame).

        Returns (N,2) ground coords ``(X forward, Y left)`` for the camera's current
        pose. Points on/above the horizon are clamped to a far ground distance
        rather than diverging.
        """
        f, cx, cy, cos_t, sin_t, cam_h, x_off = self._camera(w, h)
        xn = (pts[:, 0] - cx) / f
        yn = (pts[:, 1] - cy) / f
        # Ray hits the ground at depth t along it; denom>0 means below the horizon.
        denom = cos_t * yn + sin_t
        denom = np.where(denom > 1e-4, denom, 1e-4)  # clamp horizon/above to far
        t = cam_h / denom
        X = x_off + t * (cos_t - sin_t * yn)
        Y = t * (-xn)
        return np.stack([X, Y], axis=1).astype(np.float64)

    def _ground_to_pixels(self, gxy, w, h):
        """Project ground points (N,2, current robot frame) to pixels.

        Returns ``(pix, in_front)`` where ``pix`` is (N,2) ``(u, v)`` and
        ``in_front`` is a bool mask (point ahead of the image plane).
        """
        f, cx, cy, cos_t, sin_t, cam_h, x_off = self._camera(w, h)
        dx = gxy[:, 0] - x_off
        Y = gxy[:, 1]
        cam_z = cos_t * dx + sin_t * cam_h  # optical-axis depth
        cam_x = -Y
        cam_y = -sin_t * dx + cos_t * cam_h
        in_front = cam_z > 1e-6
        z = np.where(in_front, cam_z, 1.0)  # avoid div-by-zero; masked out anyway
        u = cx + f * cam_x / z
        v = cy + f * cam_y / z
        return np.stack([u, v], axis=1).astype(np.float32), in_front

    # ------------------------------------------------------------------
    # Odometry

    def _odom_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._odom_buffer.append((t, p.x, p.y, yaw))

    def _pose_at(self, stamp):
        """Odometry pose nearest ``stamp`` (``sec=nanosec=0`` -> latest), or None."""
        if not self._odom_buffer:
            return None
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            _, x, y, yaw = self._odom_buffer[-1]
            return (x, y, yaw)
        key = stamp.sec + stamp.nanosec * 1e-9
        t, x, y, yaw = min(self._odom_buffer, key=lambda s: abs(s[0] - key))
        return (x, y, yaw)

    def _odom_fresh(self):
        """True if the latest odometry is within ``odom_timeout`` of now."""
        if not self._odom_buffer:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._odom_buffer[-1][0]) <= float(self._p("odom_timeout"))

    def _relative_pose(self):
        """Current pose expressed in the reference frame: ``(rel_x, rel_y, rel_yaw)``."""
        x0, y0, yaw0 = self.ref_pose
        xc, yc, yawc = self._pose_at(None)
        dx, dy = xc - x0, yc - y0
        c0, s0 = np.cos(yaw0), np.sin(yaw0)
        return (c0 * dx + s0 * dy, -s0 * dx + c0 * dy, yawc - yaw0)

    # ------------------------------------------------------------------
    # Image callback

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if not self.initialized:
            if self._pending_waypoints is None:
                self._draw_idle(frame)
                self._publish_debug(frame, msg)
                return
            if self._pose_at(self._pending_stamp) is None:
                cv2.putText(
                    frame, "WAITING FOR ODOMETRY...",
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
                )
                self._publish_debug(frame, msg)
                return  # keep pending; retry when odom arrives
            self._initialise(frame.shape[:2])
            self._track(frame, msg)
            return

        self._track(frame, msg)

    def _initialise(self, shape):
        self.img_shape = shape
        h, w = shape

        # Seed the reference frame from the odometry pose at the selected frame.
        self.ref_pose = self._pose_at(self._pending_stamp)
        self._pending_stamp = None

        wps_norm = self._pending_waypoints
        self._pending_waypoints = None

        # Normalized [-1, 1] -> pixels.
        pix = np.empty((len(wps_norm), 2), dtype=np.float32)
        pix[:, 0] = (wps_norm[:, 0] + 1.0) * 0.5 * (w - 1)
        pix[:, 1] = (wps_norm[:, 1] + 1.0) * 0.5 * (h - 1)
        # Keep the waypoints in exactly the order they were sent — never re-sorted.
        # The caller owns the ordering (interface contract: nearest-first, index 0
        # nearest); the priority prefix and the published stream both walk this same
        # as-sent order.

        # Ground the waypoints once, in the reference frame.
        self.ground_ref = self._pixels_to_ground(pix, w, h)
        self.pts = pix

        self.initialized = True
        self._set_status(STATUS_TRACKING)
        self.get_logger().info(
            f"Tracking {len(self.pts)} waypoint(s) via odometry "
            f"(ref pose x={self.ref_pose[0]:.2f} y={self.ref_pose[1]:.2f} "
            f"yaw={self.ref_pose[2]:.2f})."
        )

    # ------------------------------------------------------------------
    # Occlusion handling (mirrors waypoint_tracker's TRACKING <-> OCCLUDED dynamics)

    def _mark_tracking(self):
        if self.tracking_status != STATUS_TRACKING:
            self.get_logger().info("Waypoint tracking recovered.")
            self._set_status(STATUS_TRACKING)
        self._ever_tracked = True

    def _handle_occlusion(self, frame, msg):
        """Near end lost / odom stale — hold, exactly like waypoint_tracker.

        No points are published while ``OCCLUDED``: the frozen prediction goes stale
        as the robot keeps moving, so the consumer must stop, not coast. If no valid
        projection ever landed after init the track never established -> ``UNTRACKED``.
        """
        if not self._ever_tracked:
            self._reset(STATUS_UNTRACKED)
            self._draw_idle(frame)
            self._publish_debug(frame, msg)
            return
        if self.tracking_status != STATUS_OCCLUDED:
            self.get_logger().warn("Waypoint tracking OCCLUDED — holding last position.")
            self._set_status(STATUS_OCCLUDED)
        self._draw_overlay(frame)
        self._publish_debug(frame, msg)

    # ------------------------------------------------------------------
    # Per-frame tracking (odometry re-projection)

    def _track(self, frame, msg):
        h, w = self.img_shape

        # Odometry is the only sensor — a stale pose is this tracker's "occlusion".
        if not self._odom_fresh():
            self._handle_occlusion(frame, msg)
            return

        # Re-express the fixed reference-frame ground points in the current robot
        # frame, then re-project them to pixels.
        rel_x, rel_y, rel_yaw = self._relative_pose()
        self._rel = (rel_x, rel_y, rel_yaw)
        cr, sr = np.cos(rel_yaw), np.sin(rel_yaw)
        ex = self.ground_ref[:, 0] - rel_x
        ey = self.ground_ref[:, 1] - rel_y
        cur = np.stack([cr * ex + sr * ey, -sr * ex + cr * ey], axis=1)

        pix, in_front = self._ground_to_pixels(cur, w, h)
        self.pts = pix

        # The tracker only *tracks* — it never retires waypoints. The full set is
        # re-projected and published every frame; deciding when a waypoint has been
        # *reached* (driven over) and advancing past it is the follower's job
        # (pursuit_servo owns that). A waypoint is *measured* only while it projects
        # in front of the camera AND inside the frame; behind-camera / off-frame
        # points coast (still published, clamped, flagged tracked=false).
        in_frame = (
            in_front
            & (self.pts[:, 0] >= 0) & (self.pts[:, 0] < w)
            & (self.pts[:, 1] >= 0) & (self.pts[:, 1] < h)
        )
        self._tracked_mask = in_frame

        # Node state follows the nearest ``priority_count`` in-frame waypoints,
        # recomputed each frame so forward progress that scrolls the near waypoints
        # off the bottom doesn't read as loss. No in-frame waypoint at all (the whole
        # trajectory has scrolled past / behind the camera) -> OCCLUDED.
        k = max(1, int(self._p("priority_count")))
        priority_idx = np.nonzero(in_frame)[0][:k]
        if len(priority_idx) == 0:
            self._handle_occlusion(frame, msg)
            return

        self._mark_tracking()
        self._draw_and_publish(frame, msg)

    # ------------------------------------------------------------------
    # Publishing

    def _draw_and_publish(self, frame, msg):
        self._publish_points(msg)
        self._draw_overlay(frame)
        self._publish_debug(frame, msg)

    def _publish_points(self, msg):
        """Publish the projected waypoints (normalized [-1, 1], ``z`` unused).

        Only reached while ``TRACKING`` (waypoint_tracker parity). ``tracked`` marks
        each waypoint *measured* (in front + in frame) vs *coasting* (off-frame /
        behind, clamped to the frame edge).
        """
        h, w = self.img_shape
        tracked = self._tracked_mask
        measuring = self.tracking_status == STATUS_TRACKING and tracked is not None
        out = VisualWaypoints()
        out.header = msg.header
        for i, pt in enumerate(self.pts):
            p = Point()
            p.x = float(np.clip(2.0 * pt[0] / (w - 1) - 1.0, -1.0, 1.0))
            p.y = float(np.clip(2.0 * pt[1] / (h - 1) - 1.0, -1.0, 1.0))
            out.points.append(p)
            out.tracked.append(bool(measuring and tracked[i]))
        self.pub_points.publish(out)

    def _draw_idle(self, frame):
        cv2.putText(
            frame,
            "UNTRACKED — call ~/set_waypoints to start",
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2,
        )

    def _draw_overlay(self, frame):
        # Amber while OCCLUDED (waypoints held/frozen), green while tracking.
        occluded = self.tracking_status == STATUS_OCCLUDED
        color = (0, 215, 255) if occluded else (0, 255, 0)

        tracked = self._tracked_mask
        measuring = (not occluded) and tracked is not None
        pts_int = [tuple(np.round(p).astype(int)) for p in self.pts]
        if len(pts_int) >= 2:
            cv2.polylines(
                frame, [np.array(pts_int, dtype=np.int32)], False, color, 2
            )
        for i, (px, py) in enumerate(pts_int):
            measured = bool(measuring and tracked[i])
            cv2.circle(frame, (px, py), 6, color, -1 if measured else 1)
            cv2.putText(
                frame, str(i), (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
            )
        n_meas = int(tracked.sum()) if measuring else 0
        rx, ry, ryaw = self._rel
        cv2.putText(
            frame,
            f"{self.tracking_status} [odom]  {len(self.pts)} pts  "
            f"meas={n_meas}/{len(self.pts)}  "
            f"d=({rx:.2f},{ry:.2f},{ryaw:.2f})",
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
        )

    def _publish_debug(self, frame, original_msg):
        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = original_msg.header
        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomWaypointTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
