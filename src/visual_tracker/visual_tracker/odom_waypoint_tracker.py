"""Odometry waypoint tracker — dead-reckoned ground-plane re-projection.

A **2D top-down** ground-trajectory tracker: it grounds the planner's waypoints
once and thereafter dead-reckons them **purely from odometry**, publishing their
live **metric positions in the robot body frame** (``base_link``) — the world-space
input a plain metric pure-pursuit follower (``pursuit_servo``) consumes. No image
content is used to place the points; the camera is only used to ground them once and
to decide what is currently *visible* for the debug overlay.

The scene is assumed planar (every waypoint on the ground), so the pipeline is:

1. **Ground the waypoints once.** On ``set_waypoints`` each normalized image point
   is back-projected through the camera (``camera_height``, ``camera_tilt``,
   ``camera_hfov_deg``) onto the ground plane, giving a fixed 2-D ground point
   ``(X, Y)`` expressed in the **reference frame** — the robot's odometry pose at
   the instant the trajectory was received. This reference frame is re-anchored
   every time a new trajectory arrives.
2. **Dead-reckon.** Each frame, the robot's current odometry pose is expressed
   relative to the reference pose (a planar rigid transform), so the fixed ground
   points are re-expressed in the *current* robot (``base_link``) frame — their 2D
   top-down world positions. The **full set is kept and re-expressed every frame**
   (never dropped), so a waypoint the robot has driven past re-appears ahead again
   once the robot turns back toward it.
3. **Publish (metric).** All waypoints are published as metric ``base_link``
   coordinates (x forward, y left, z=0), each flagged ``in_front`` (ahead of the
   camera) or behind. The follower steers by the in-front ones and owns *reaching* /
   completion — "reaching" is an act of the servo, not the tracker.

Because the placement is a dead-reckoned prediction, "occlusion" here means
**odometry loss**: ``/odom`` going stale (``odom_timeout``). Odometry always knows
where every waypoint is, so there is no visual occlusion — only the sensor dropping
out. A ``front/back`` flag (rather than a bogus reprojection) is what lets the
follower avoid chasing a point that is actually behind the camera.

Alongside the metric point stream, the tracked waypoints are also published as a
``visualization_msgs/MarkerArray`` on the **ground plane** (z=0) in ``marker_frame``
(the robot body / ``base_link``) — each a flat disk whose radius grows with the
waypoint's positional **uncertainty**. Since the waypoints are dead-reckoned from the
reference pose, that uncertainty is the odometry drift accrued since grounding: the
translational covariance growth plus its yaw component swung out to each waypoint's
range (so far waypoints, more sensitive to heading error, render larger).

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
from visualization_msgs.msg import Marker, MarkerArray

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

        # --- Ground-marker (uncertainty) parameters ---
        # The tracked waypoints are also published as a MarkerArray on the ground
        # plane (z=0) in ``marker_frame`` (the robot body). Each marker is a flat disk
        # whose radius grows with the waypoint's positional uncertainty — the
        # odometry drift accumulated since the trajectory was grounded, plus that
        # drift's yaw component scaled by the waypoint's range (a far waypoint is more
        # uncertain because a small heading error swings it further).
        self.declare_parameter("marker_frame", "base_link")
        self.declare_parameter("marker_base_size", 0.03)  # m; min disk diameter
        self.declare_parameter("uncertainty_scale", 1.0)  # disk-diameter gain per σ

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
        self.pub_markers = self.create_publisher(
            MarkerArray, "/waypoint_tracking/markers", 10
        )

        # --- Services ---
        self.create_service(SetWaypoints, "~/set_waypoints", self._srv_set_waypoints)
        self.create_service(StopTracking, "~/stop_tracking", self._srv_stop_tracking)

        # --- Internal state ---
        self._pending_waypoints = None  # Nx2 normalized, awaiting init
        self._pending_stamp = None
        # (t, x, y, yaw, var_x, var_y, var_yaw) — pose + odom covariance diagonal.
        self._odom_buffer = deque(maxlen=200)
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
        self.pts = None  # (N,2) current waypoint pixel positions (debug reprojection)
        self.cur = None  # (N,2) current waypoint metric base_link coords (x fwd, y left)
        self.ground_ref = None  # (N,2) ground points in the reference frame
        self.ref_pose = None  # (x, y, yaw) odometry pose when trajectory received
        self.ref_cov = None  # (var_x, var_y, var_yaw) odom covariance at that pose
        self.img_shape = None  # (h, w)
        self._ever_tracked = False  # a valid dead-reckon has landed since init
        # Debug/telemetry
        self._in_front = None  # (N,) bool: ahead of the camera vs behind it
        self._visible = None  # (N,) bool: in front AND inside the camera frame
        self._rel = (0.0, 0.0, 0.0)  # current pose relative to the reference
        self._set_status(status)
        self._clear_markers()  # drop any stale ground-uncertainty markers

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
        z = np.where(in_front, cam_z, 1.0)  # avoid div-by-zero; masked out via in_front
        u = cx + f * cam_x / z
        v = cy + f * cam_y / z
        return np.stack([u, v], axis=1).astype(np.float32), in_front

    # ------------------------------------------------------------------
    # Odometry

    def _odom_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        # pose.covariance is a row-major 6x6 (x, y, z, roll, pitch, yaw); keep the
        # x / y / yaw variances (diagonal 0, 7, 35) for the uncertainty markers.
        cov = msg.pose.covariance
        self._odom_buffer.append(
            (t, p.x, p.y, yaw, float(cov[0]), float(cov[7]), float(cov[35]))
        )

    def _sample_at(self, stamp):
        """Full odom sample nearest ``stamp`` (``sec=nanosec=0`` -> latest), or None."""
        if not self._odom_buffer:
            return None
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            return self._odom_buffer[-1]
        key = stamp.sec + stamp.nanosec * 1e-9
        return min(self._odom_buffer, key=lambda s: abs(s[0] - key))

    def _pose_at(self, stamp):
        """Odometry pose ``(x, y, yaw)`` nearest ``stamp``, or None."""
        s = self._sample_at(stamp)
        return None if s is None else (s[1], s[2], s[3])

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

        # Seed the reference frame from the odometry sample at the selected frame:
        # its pose anchors the ground points, its covariance is the uncertainty
        # baseline (drift is measured *relative* to here).
        ref = self._sample_at(self._pending_stamp)
        self.ref_pose = (ref[1], ref[2], ref[3])
        self.ref_cov = (ref[4], ref[5], ref[6])
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
            self._clear_markers()  # frozen prediction is stale — show nothing
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

        # Dead-reckon: express the saved ground points in the current robot
        # (base_link) frame — this is the 2D top-down world position of every
        # waypoint, and the tracker's primary output. The full set is kept and
        # re-expressed every frame (never dropped), so a waypoint the robot has driven
        # past re-appears ahead again once the robot turns back toward it.
        rel_x, rel_y, rel_yaw = self._relative_pose()
        self._rel = (rel_x, rel_y, rel_yaw)
        cr, sr = np.cos(rel_yaw), np.sin(rel_yaw)
        ex = self.ground_ref[:, 0] - rel_x
        ey = self.ground_ref[:, 1] - rel_y
        self.cur = np.stack([cr * ex + sr * ey, -sr * ex + cr * ey], axis=1)

        # Reproject to the image *only* to decide front/back and image-visibility —
        # the published stream is metric top-down (base_link), not image space. A
        # waypoint is *in front* when it projects ahead of the camera (cam_z>0), and
        # *visible* when it is additionally inside the frame (drawn on the overlay).
        pix, in_front = self._ground_to_pixels(self.cur, w, h)
        self.pts = pix
        self._in_front = in_front
        self._visible = (
            in_front
            & (pix[:, 0] >= 0) & (pix[:, 0] < w)
            & (pix[:, 1] >= 0) & (pix[:, 1] < h)
        )

        # Odometry always knows where every waypoint is, so the only failure mode is a
        # stale pose (handled above). A trusted dead-reckon this frame keeps TRACKING.
        self._mark_tracking()
        self._publish_markers(self.cur, in_front)  # ground uncertainty markers
        self._draw_and_publish(frame, msg)

    # ------------------------------------------------------------------
    # Ground-plane uncertainty markers

    def _drift_cov(self):
        """Odom-drift variances ``(var_x, var_y, var_yaw)`` accrued since the reference.

        The waypoints are dead-reckoned from ``ref_pose``, so what makes them
        uncertain is the odometry drift *since* that pose — the growth of the reported
        pose covariance, not its absolute value. Clamped at zero (covariance should
        grow, but guard against noise / a source that resets it).
        """
        s = self._sample_at(None)
        if s is None or self.ref_cov is None:
            return 0.0, 0.0, 0.0
        return (
            max(0.0, s[4] - self.ref_cov[0]),
            max(0.0, s[5] - self.ref_cov[1]),
            max(0.0, s[6] - self.ref_cov[2]),
        )

    def _publish_markers(self, cur, in_front):
        """Publish the waypoints as ground disks in ``marker_frame`` (z=0).

        ``cur`` is (N,2) in the current robot frame (x forward, y left) — already the
        ``marker_frame`` (base_link) coordinates. Only waypoints **in front of the
        camera** are shown; ones the robot has driven past (behind the camera) emit a
        ``DELETE`` for their id so consumed points don't leave stale disks behind (and
        we never render a point the robot has already gone by). Each disk's diameter
        grows with the waypoint's positional uncertainty
        ``σ_i = sqrt(σ_trans² + range_i²·σ_yaw²)``: the translational odom drift plus
        the yaw drift swung out to the waypoint's range (so far waypoints, more
        sensitive to heading error, read as larger). Measured waypoints (in frame)
        are green, coasting ones (in front but off-frame) amber.
        """
        frame = str(self._p("marker_frame"))
        base = float(self._p("marker_base_size"))
        gain = float(self._p("uncertainty_scale"))
        dvx, dvy, dvyaw = self._drift_cov()
        trans_var = dvx + dvy
        stamp = self.get_clock().now().to_msg()
        tracked = self._visible
        arr = MarkerArray()
        for i, (x, y) in enumerate(cur):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = "waypoints"
            m.id = i
            if not bool(in_front[i]):
                m.action = Marker.DELETE  # driven past — remove any prior disk
                arr.markers.append(m)
                continue
            rng = float(np.hypot(x, y))
            sigma = float(np.sqrt(max(0.0, trans_var + rng * rng * dvyaw)))
            diam = base + 2.0 * gain * sigma
            measured = bool(tracked is not None and i < len(tracked) and tracked[i])
            m.type = Marker.CYLINDER  # flat disk on the ground
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = 0.01  # thin — a ground footprint, not a pillar
            m.color.r = 0.0 if measured else 1.0
            m.color.g = 1.0 if measured else 0.65
            m.color.b = 0.0
            m.color.a = 0.6
            arr.markers.append(m)
        self.pub_markers.publish(arr)

    def _clear_markers(self):
        """Delete all published markers (on stop / new trajectory / occlusion)."""
        m = Marker()
        m.header.frame_id = str(self._p("marker_frame"))
        m.ns = "waypoints"
        m.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(m)
        self.pub_markers.publish(arr)

    # ------------------------------------------------------------------
    # Publishing

    def _draw_and_publish(self, frame, msg):
        self._publish_points(msg)
        self._draw_overlay(frame)
        self._publish_debug(frame, msg)

    def _publish_points(self, msg):
        """Publish all waypoints in metric top-down world space (base_link).

        ``points`` are the current base_link ground coordinates (x forward, y left,
        z=0, metres), in the order the planner sent them (index 0 first). ``in_front``
        flags each as ahead of the camera (true) or behind (false); **every** waypoint
        is published every frame, so a behind waypoint re-appears (in_front flips true)
        once the robot turns back toward it. ``tracked`` carries image-visibility (in
        front AND inside the frame). Published only while ``TRACKING`` (odom fresh).
        """
        out = VisualWaypoints()
        out.header = msg.header
        out.header.frame_id = str(self._p("marker_frame"))  # base_link
        for i, (x, y) in enumerate(self.cur):
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            out.points.append(p)
            out.in_front.append(bool(self._in_front[i]))
            out.tracked.append(bool(self._visible[i]))
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

        # Draw only waypoints that actually project into the image — in front of the
        # camera AND within frame bounds (the visible set). A waypoint the robot has
        # driven past is behind the camera and has no valid projection, so it is
        # skipped instead of shown at a bogus pixel. It is still tracked and published
        # (metric), and re-appears here if the robot turns back to face it. The
        # polyline is likewise built from the visible points only.
        tracked = self._visible
        total = len(self.pts) if self.pts is not None else 0
        vis = []
        if tracked is not None:
            for i, pt in enumerate(self.pts):
                if i < len(tracked) and tracked[i]:
                    vis.append((i, int(round(float(pt[0]))), int(round(float(pt[1])))))
        if len(vis) >= 2:
            cv2.polylines(
                frame,
                [np.array([(x, y) for _, x, y in vis], dtype=np.int32)],
                False, color, 2,
            )
        for i, px, py in vis:
            cv2.circle(frame, (px, py), 6, color, -1)
            cv2.putText(
                frame, str(i), (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
            )
        rx, ry, ryaw = self._rel
        cv2.putText(
            frame,
            f"{self.tracking_status} [odom]  {total} pts  "
            f"vis={len(vis)}/{total}  "
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
