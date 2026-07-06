"""Waypoint tracker — keyframe-anchored ground homography (DIS flow).

Every waypoint lies on the *same* ground plane, so one planar homography of the
ground describes how all of them move. Rather than advecting each waypoint by its
own (noisy, texture-dependent) flow, the node tracks the *plane* and warps the
whole waypoint set through it. Per frame, anchored to a **keyframe**:

1. Dense optical flow (DIS, preset via ``dis_preset``) carries a **grid** of
   points — masked to a band around the waypoint path (convex hull + tube), which
   keeps flanking walls and off-path movers out of the correspondences — from the
   keyframe to the current frame (both directions), giving plane correspondences
   even on blank floor.
2. Correspondences are forward-backward filtered (``fb_thresh``); a **RANSAC
   homography** (``findHomography``) is fitted when enough inliers support a
   stable perspective estimate, else a 4-DOF **affine** fallback.
3. The keyframe waypoints are warped through the transform
   (``perspectiveTransform``). Pooling over the grid + the plane constraint
   corrects the per-point flow noise that made pure advection drift; a covered or
   blank-floor waypoint is still placed correctly from texture elsewhere.
4. The keyframe **re-keys** (advances to the current frame) only when the
   keyframe->current baseline grows past ``rekey_flow_px`` or the flow breaks, so
   residual drift accrues at these infrequent events, not every frame.

When the RANSAC fit is unreliable (``inlier_ratio_thresh``) the waypoints hold
their last position rather than following a bad transform. Waypoints come in via
``set_waypoints`` (normalized [-1, 1] image space) and their positions are
re-published continuously on ``/waypoint_tracking/points``.
"""

from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
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

_DIS_PRESETS = {
    "ultrafast": cv2.DISOPTICAL_FLOW_PRESET_ULTRAFAST,
    "fast": cv2.DISOPTICAL_FLOW_PRESET_FAST,
    "medium": cv2.DISOPTICAL_FLOW_PRESET_MEDIUM,
}


class WaypointTrackerNode(Node):
    def __init__(self):
        super().__init__("waypoint_tracker_node")

        # --- Tuning parameters ---
        self.declare_parameter("dis_preset", "fast")  # ultrafast|fast|medium
        self.declare_parameter("fb_thresh", 1.0)  # px; forward-backward reject
        self.declare_parameter("rekey_flow_px", 30.0)  # re-key when kf baseline grows
        self.declare_parameter("grid_step", 10)  # px spacing of the ground flow grid
        self.declare_parameter("roi_margin", 0.10)  # padding around waypoint bbox
        self.declare_parameter("min_features", 10)  # min inlier grid points
        self.declare_parameter("min_homography_features", 20)  # homography vs affine
        self.declare_parameter("inlier_ratio_thresh", 0.80)  # hold below this
        self.declare_parameter("rekey_inlier_ratio", 0.90)  # min fit quality to re-key

        # --- Publishers ---
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
        self._frame_buffer = deque(maxlen=10)  # (stamp_key, gray) for stamp init
        self.bridge = CvBridge()

        # Dense optical flow estimator — low-texture friendly, CPU real-time.
        # (In the main cv2 module; contrib exposes cv2.optflow.createOptFlow_DIS.)
        # Preset is applied at startup (not live-adjustable).
        preset = _DIS_PRESETS.get(
            str(self.get_parameter("dis_preset").value).lower(),
            cv2.DISOPTICAL_FLOW_PRESET_FAST,
        )
        self._dis = cv2.DISOpticalFlow_create(preset)

        self._reset(STATUS_UNTRACKED)

        # Re-publish state every second so late subscribers catch up.
        self.create_timer(1.0, lambda: self._publish_state(self.tracking_status))

        # Subscribe last so the node is fully initialised before images arrive.
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10
        )

        self.get_logger().info(
            "Waypoint tracker (homography) ready — call ~/set_waypoints to start."
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
        self.kf_gray = None  # keyframe image (anchor)
        self.kf_grid = None  # (M,2) ground flow grid, in keyframe
        self.kf_pts = None  # (N,2) waypoint positions in the keyframe
        self.img_shape = None  # (h, w)
        self._ever_tracked = False  # a trusted fit has landed since init
        # Debug/telemetry
        self._n_rejected = 0
        self._rekeys = 0
        self._kf_disp = 0.0
        self._model = "none"
        self._inlier_ratio = 0.0
        self._grid_vis = None  # (K,2) current inlier grid positions, for overlay
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
    # Image callback

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._frame_buffer.append((stamp_key, gray.copy()))

        if not self.initialized:
            if self._pending_waypoints is None:
                self._draw_idle(frame)
                self._publish_debug(frame, msg)
                return
            if not self._initialise(gray):
                cv2.putText(
                    frame,
                    "GROUND ROI TOO SMALL — call set_waypoints again",
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                )
                self._publish_debug(frame, msg)
                return
            self._draw_and_publish(frame, msg)
            return

        self._track(gray, frame, msg)

    def _initialise(self, gray):
        self.img_shape = gray.shape[:2]

        # Select the requested frame from the buffer, else use the current one.
        init_gray = gray
        if self._pending_stamp is not None:
            key = (self._pending_stamp.sec, self._pending_stamp.nanosec)
            if key != (0, 0):
                for buf_key, buf_gray in self._frame_buffer:
                    if buf_key == key:
                        init_gray = buf_gray
                        break
        self._pending_stamp = None

        h, w = self.img_shape
        wps_norm = self._pending_waypoints
        self._pending_waypoints = None

        self.pts = np.empty((len(wps_norm), 2), dtype=np.float32)
        for i, (xn, yn) in enumerate(wps_norm):
            self.pts[i, 0] = (xn + 1.0) * 0.5 * (w - 1)
            self.pts[i, 1] = (yn + 1.0) * 0.5 * (h - 1)

        if not self._set_keyframe(init_gray):
            self.get_logger().warn("Ground ROI too small to seed a grid — call again.")
            self._reset(STATUS_UNTRACKED)
            return False

        self.initialized = True
        self._set_status(STATUS_TRACKING)
        self.get_logger().info(
            f"Tracking {len(self.pts)} waypoints "
            f"({len(self.kf_grid)} grid pts, homography)."
        )
        return True

    # ------------------------------------------------------------------
    # Occlusion handling (mirrors lk_tracker's TRACKING <-> OCCLUDED dynamics)

    def _mark_tracking(self):
        """A frame produced a trusted fit — (re)enter TRACKING, recovering if held."""
        if self.tracking_status != STATUS_TRACKING:
            self.get_logger().info("Waypoint tracking recovered.")
            self._set_status(STATUS_TRACKING)
        self._ever_tracked = True

    def _handle_occlusion(self, frame, msg):
        """A frame failed to produce a trusted fit — hold, like lk_tracker.

        ``TRACKING -> OCCLUDED`` immediately (the waypoints keep publishing, frozen
        at their last position — ``OCCLUDED`` on the state topic is the "stale, do
        not trust" signal for downstream consumers, exactly as ``visual_servo``
        already treats ``lk_tracker``'s ``OCCLUDED``). If no trusted fit was ever
        produced after init the track never established, so drop to ``UNTRACKED``.
        """
        if not self._ever_tracked:
            self._reset(STATUS_UNTRACKED)
            self._draw_idle(frame)
            self._publish_debug(frame, msg)
            return
        if self.tracking_status != STATUS_OCCLUDED:
            self.get_logger().warn("Waypoint tracking OCCLUDED — holding last position.")
            self._set_status(STATUS_OCCLUDED)
        self._draw_and_publish(frame, msg)  # republish the frozen waypoints

    # ------------------------------------------------------------------
    # Per-frame tracking

    def _track(self, gray, frame, msg):
        min_f = self._p("min_features")

        # Flow keyframe -> current at the grid, forward-backward filtered.
        flow_fwd = self._dis.calc(self.kf_gray, gray, None)
        g_fwd = self._sample_flow(flow_fwd, self.kf_grid)
        grid_cur = self.kf_grid + g_fwd
        flow_bwd = self._dis.calc(gray, self.kf_gray, None)
        g_bwd = self._sample_flow(flow_bwd, grid_cur)
        fb_err = np.linalg.norm(g_fwd + g_bwd, axis=1)
        good = fb_err < self._p("fb_thresh")
        self._n_rejected = int((~good).sum())
        src = self.kf_grid[good]
        dst = grid_cur[good]
        self._grid_vis = dst

        # Flow to the keyframe has broken (large baseline / occlusion) — re-anchor
        # to the current frame at the last-known positions to recover.
        if len(dst) < min_f:
            self._kf_disp = 0.0
            if self._set_keyframe(gray):  # re-anchor so the next frame can recover
                self._rekeys += 1
            self._handle_occlusion(frame, msg)  # flow broke — waypoints frozen
            return

        H, inliers, model = self._estimate_transform(src, dst)
        self._model = model
        if H is None:
            self._handle_occlusion(frame, msg)  # no transform — hold
            return

        self._inlier_ratio = float(inliers.sum()) / max(len(dst), 1)
        warped = self._warp(H)
        if warped is None or self._inlier_ratio < self._p("inlier_ratio_thresh"):
            self._handle_occlusion(frame, msg)  # unreliable fit -> hold
            return

        self.pts = warped
        self._mark_tracking()  # trusted measurement this frame

        # Re-key when the baseline grows enough that DIS accuracy degrades — but
        # only from a high-confidence frame. A re-key freezes the current warped
        # estimate in as the new anchor, so any error committed here becomes
        # permanent drift; a mediocre-fit frame is deferred until a clean one
        # (or, failing that, the broken-flow recovery re-anchor above).
        self._kf_disp = float(np.median(np.linalg.norm(g_fwd[good], axis=1)))
        if (
            self._kf_disp > self._p("rekey_flow_px")
            and self._inlier_ratio >= self._p("rekey_inlier_ratio")
        ):
            if self._set_keyframe(gray):
                self._rekeys += 1

        self._draw_and_publish(frame, msg)

    def _set_keyframe(self, gray):
        """Anchor to ``gray``: re-lay the ground grid around the current waypoints."""
        grid = self._make_grid(self.pts)
        if grid is None or len(grid) < self._p("min_features"):
            return False
        self.kf_gray = gray
        self.kf_grid = grid
        self.kf_pts = self.pts.copy()
        return True

    def _estimate_transform(self, src, dst):
        """Fit keyframe->current ground transform; homography when well-supported."""
        if len(src) >= self._p("min_homography_features"):
            H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            if H is not None and inliers is not None and self._is_sane_homography(H):
                return H.astype(np.float64), inliers, "homography"

        M, inliers = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0
        )
        if M is None or inliers is None:
            return None, None, "none"
        H = np.vstack([M, [0.0, 0.0, 1.0]]).astype(np.float64)
        return H, inliers, "affine"

    def _is_sane_homography(self, H):
        """Reject degenerate / reflected / wildly-skewed homographies.

        ``det(H[:2,:2]) > 1e-6`` alone still accepts near-singular, mirror-flipped
        or heavily-sheared fits — a bad homography that happens to clear the inlier
        gate poisons everything downstream (its warped waypoints can become the
        next re-key anchor). We bound the linear part's orientation, absolute
        scale and anisotropy, all generous enough not to touch honest per-frame
        keyframe->current motion (baseline capped at ``rekey_flow_px``).
        """
        if not np.all(np.isfinite(H)):
            return False
        A = H[:2, :2]
        # Orientation-preserving and not near-singular (rejects reflections too).
        if np.linalg.det(A) < 1e-3:
            return False
        # Singular values bound absolute scale; their ratio bounds shear/anisotropy.
        sv = np.linalg.svd(A, compute_uv=False)
        smax, smin = float(sv[0]), float(sv[-1])
        if smin < 0.25 or smax > 4.0:
            return False
        if smax / max(smin, 1e-6) > 4.0:
            return False
        return True

    def _warp(self, H):
        """Warp the keyframe waypoints by ``H``; ``None`` if the result is wild."""
        warped = cv2.perspectiveTransform(
            self.kf_pts.reshape(-1, 1, 2).astype(np.float32), H
        ).reshape(-1, 2)
        if not np.all(np.isfinite(warped)):
            return None
        h, w = self.img_shape
        limit = 3 * max(h, w)
        if warped.min() < -limit or warped.max() > limit:
            return None
        return warped.astype(np.float32)

    def _sample_flow(self, flow, pts):
        """Bilinearly-interpolated flow vectors at (fractional) pixel points, (N,2)."""
        h, w = flow.shape[:2]
        x = np.clip(pts[:, 0], 0, w - 1)
        y = np.clip(pts[:, 1], 0, h - 1)
        x0 = np.floor(x).astype(np.int32)
        y0 = np.floor(y).astype(np.int32)
        x1 = np.clip(x0 + 1, 0, w - 1)
        y1 = np.clip(y0 + 1, 0, h - 1)
        wx = (x - x0)[:, None]
        wy = (y - y0)[:, None]
        top = flow[y0, x0] * (1.0 - wx) + flow[y0, x1] * wx
        bot = flow[y1, x0] * (1.0 - wx) + flow[y1, x1] * wx
        return top * (1.0 - wy) + bot * wy

    def _make_grid(self, pts):
        """Regular grid (``grid_step`` px) inside a band around the waypoint path.

        The band is the filled convex hull of the waypoints plus a tube of
        half-width ``pad`` along the ordered path (``pad`` scales with the
        trajectory span via ``roi_margin``). Masking the grid to the ground path
        — instead of the full bounding box — keeps flanking walls and off-path
        moving objects out of the flow correspondences, which makes RANSAC much
        harder to "kidnap". Returns (M,2), or None if the band yields no grid.
        """
        h, w = self.img_shape
        step = float(self._p("grid_step"))
        xs = pts[:, 0]
        ys = pts[:, 1]
        span = float(max(xs.max() - xs.min(), ys.max() - ys.min()))
        pad = int(self._p("roi_margin") * span + 10)

        # Path band mask: filled hull + thick tube along the ordered waypoints +
        # a disc at each (robust for 1-2 / collinear waypoints).
        pts_i = np.round(pts).astype(np.int32).reshape(-1, 1, 2)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [cv2.convexHull(pts_i)], 255)
        cv2.polylines(mask, [pts_i], False, 255, thickness=max(1, 2 * pad))
        for p in pts_i.reshape(-1, 2):
            cv2.circle(mask, (int(p[0]), int(p[1])), max(1, pad), 255, -1)

        # Grid over the padded bbox, kept only where the band mask is set.
        x0 = int(max(0, xs.min() - pad))
        y0 = int(max(0, ys.min() - pad))
        x1 = int(min(w, xs.max() + pad))
        y1 = int(min(h, ys.max() + pad))
        gx = np.arange(x0 + step / 2.0, x1, step, dtype=np.float32)
        gy = np.arange(y0 + step / 2.0, y1, step, dtype=np.float32)
        if len(gx) == 0 or len(gy) == 0:
            return None
        mgx, mgy = np.meshgrid(gx, gy)
        grid = np.stack([mgx.ravel(), mgy.ravel()], axis=1)
        xi = np.clip(grid[:, 0].astype(np.int32), 0, w - 1)
        yi = np.clip(grid[:, 1].astype(np.int32), 0, h - 1)
        grid = grid[mask[yi, xi] > 0]
        if len(grid) == 0:
            return None
        return grid.astype(np.float32)

    # ------------------------------------------------------------------
    # Publishing

    def _draw_and_publish(self, frame, msg):
        self._publish_points(msg)
        self._draw_overlay(frame)
        self._publish_debug(frame, msg)

    def _publish_points(self, msg):
        """Publish the warped waypoints (normalized [-1, 1], `z` unused).

        `tracked` is `true` while the waypoint is inside the frame, `false` once
        the transform carries it off-image.
        """
        h, w = self.img_shape
        out = VisualWaypoints()
        out.header = msg.header
        for pt in self.pts:
            p = Point()
            p.x = float(2.0 * pt[0] / (w - 1) - 1.0)
            p.y = float(2.0 * pt[1] / (h - 1) - 1.0)
            out.points.append(p)
            in_frame = (0 <= pt[0] < w) and (0 <= pt[1] < h)
            out.tracked.append(bool(in_frame))
        self.pub_points.publish(out)

    def _draw_idle(self, frame):
        cv2.putText(
            frame,
            "UNTRACKED — call ~/set_waypoints to start",
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2,
        )

    def _draw_overlay(self, frame):
        # Amber while OCCLUDED (waypoints held/frozen), green while tracking —
        # same colour convention as lk_tracker's overlay.
        occluded = self.tracking_status == STATUS_OCCLUDED
        color = (0, 215, 255) if occluded else (0, 255, 0)

        # Inlier grid driving the homography.
        if self._grid_vis is not None:
            for gx, gy in self._grid_vis:
                cv2.circle(frame, (int(gx), int(gy)), 1, (255, 200, 0), -1)

        # Waypoint polyline + points.
        pts_int = [tuple(np.round(p).astype(int)) for p in self.pts]
        if len(pts_int) >= 2:
            cv2.polylines(
                frame, [np.array(pts_int, dtype=np.int32)], False, color, 2
            )
        for i, (px, py) in enumerate(pts_int):
            cv2.circle(frame, (px, py), 6, color, -1)
            cv2.putText(
                frame, str(i), (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
            )
        cv2.putText(
            frame,
            f"{self.tracking_status} ({self._model})  {len(self.pts)} pts  "
            f"inliers={self._inlier_ratio:.2f}  reject={self._n_rejected}  "
            f"rekeys={self._rekeys}  kf_disp={self._kf_disp:.1f}",
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
        )

    def _publish_debug(self, frame, original_msg):
        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = original_msg.header
        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
