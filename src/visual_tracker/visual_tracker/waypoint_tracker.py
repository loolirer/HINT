"""Waypoint tracker — keyframe-anchored ground homography (DIS flow).

Every waypoint lies on the *same* ground plane, so one planar homography of the
ground describes how all of them move. Rather than advecting each waypoint by its
own (noisy, texture-dependent) flow, the node tracks the *plane* and warps the
whole waypoint set through it. Per frame, anchored to a **keyframe**:

1. **Pre-warped dense flow.** The keyframe is first warped by the running estimate
   ``H_kf->prev`` so DIS only measures the small *residual* (the large FOE
   expansion of forward motion is absorbed by the prediction, keeping the flow in
   DIS's range and near-uniform — a raw keyframe->current flow tracked forward
   motion poorly). DIS (preset via ``dis_preset``) carries a **grid** of points —
   masked to a band around the waypoint path (convex hull + tube), keeping flanking
   walls and off-path movers out — both directions, giving plane correspondences
   even on blank floor.
2. Correspondences are forward-backward filtered (``fb_thresh``); a **RANSAC
   homography** (``findHomography``) is fitted — the motion model for a multi-point
   trajectory (a plane under any camera motion is a homography). A frame that can't
   support one holds rather than tracking through a weaker model that can't see
   perspective looming. **Single-point mode** — entered when the planner sends
   exactly one waypoint, or when a multi-point trajectory retires down to its last
   one — instead fits a well-conditioned local **translation**, since a homography
   can't be conditioned from the compact grid around one point.
3. The keyframe waypoints are warped through the transform
   (``perspectiveTransform``). Pooling over the grid + the plane constraint
   corrects the per-point flow noise that made pure advection drift; a covered or
   blank-floor waypoint is still placed correctly from texture elsewhere. The fit
   is re-anchored on the *near* support (``_refine_near``) so far-plane noise does
   not tilt what the near waypoints ride on.
4. Occlusion is judged **per waypoint**: each is *measured* when enough inlier grid
   points fall within ``support_radius`` of it, else *coasting* (still placed by the
   plane, flagged ``tracked=false``). The node stays ``TRACKING`` while the nearest
   ``priority_count`` *in-frame* waypoints keep support (recomputed each frame, so
   forward progress that scrolls the near waypoints off-screen doesn't read as
   loss), so a far occlusion only flips its own flag instead of forcing the whole
   set ``OCCLUDED``. Waypoints are published only while ``TRACKING``.
5. Waypoints that pass **under the robot** (any waypoint whose warped position
   crosses below the frame bottom) are **retired** — the set shrinks as the robot
   advances, so a consumed point can't be resurrected or flung off-screen by a later
   re-fit. All waypoints retired -> ``UNTRACKED`` (trajectory complete). Waypoints
   are sorted nearest-first at init so retirement/priority hold for any input order.
6. The keyframe **re-keys** (advances to the current frame) only when the
   keyframe->current baseline grows past ``rekey_flow_px`` or the flow breaks, so
   residual drift accrues at these infrequent events, not every frame.

Waypoints come in via ``set_waypoints`` (normalized [-1, 1] image space, nearest
first) and their positions are re-published continuously on
``/waypoint_tracking/points``.
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
        self.declare_parameter("fb_thresh", 2.0)  # px; forward-backward reject
        self.declare_parameter("rekey_flow_px", 30.0)  # re-key when kf baseline grows
        self.declare_parameter("grid_step", 5)  # px spacing of the ground flow grid
        self.declare_parameter("roi_margin", 0.10)  # padding around waypoint bbox
        self.declare_parameter("min_features", 10)  # min inlier grid points
        self.declare_parameter("min_homography_features", 15)  # min pts to fit homography
        self.declare_parameter("rekey_inlier_ratio", 0.90)  # min fit quality to re-key
        # Per-waypoint occlusion: the nearest ``priority_count`` waypoints govern the
        # node state; a waypoint counts as *measured* when at least ``min_support``
        # inlier grid points sit within ``support_radius`` px of it (else coasting).
        self.declare_parameter("priority_count", 2)  # nearest N drive node state
        self.declare_parameter("support_radius", 60.0)  # px; local support window
        self.declare_parameter("min_support", 3)  # inliers in window to be "measured"

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
        self._H_kf2prev = np.eye(3)  # running kf->prev estimate, prewarps the flow
        self.img_shape = None  # (h, w)
        self._single = False  # single-point mode (translation) vs homography
        self._ever_tracked = False  # a trusted fit has landed since init
        # Debug/telemetry
        self._n_rejected = 0
        self._rekeys = 0
        self._kf_disp = 0.0
        self._inlier_ratio = 0.0
        self._grid_vis = None  # (K,2) current inlier grid positions, for overlay
        self._tracked_mask = None  # (N,) bool: per-waypoint measured vs coasting
        self._support = None  # (N,) int: local inlier count per waypoint
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
        # Enforce nearest-first (largest image-y = lowest in frame = nearest) so the
        # retirement/priority front-logic and the published order hold regardless of
        # the order the waypoints arrived in. Assumes a forward ground path (depth
        # monotonic in image-y), which is the interface's nearest-first contract.
        self.pts = self.pts[np.argsort(-self.pts[:, 1], kind="stable")]

        # A lone waypoint is a single-point tracking task (e.g. one ground goal):
        # a homography can't be conditioned from the compact grid around one point,
        # so it tracks by local translation instead of the ground homography.
        self._single = len(self.pts) == 1

        if not self._set_keyframe(init_gray):
            self.get_logger().warn("Ground ROI too small to seed a grid — call again.")
            self._reset(STATUS_UNTRACKED)
            return False

        self.initialized = True
        self._set_status(STATUS_TRACKING)
        model = "translation" if self._single else "homography"
        self.get_logger().info(
            f"Tracking {len(self.pts)} waypoint(s) "
            f"({len(self.kf_grid)} grid pts, {model})."
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

        ``TRACKING -> OCCLUDED`` immediately. Following ``lk_tracker`` (which stops
        publishing ``/tracking/bbox`` while occluded), **no waypoints are published
        while OCCLUDED**: the frozen positions go stale as the robot keeps moving, so
        the consumer must gate on the state topic and stop, not coast on a held
        target. ``OCCLUDED`` on ``/waypoint_tracking/state`` is that signal, exactly
        as ``visual_servo`` treats ``lk_tracker``'s ``OCCLUDED``. If no trusted fit
        was ever produced after init the track never established, so drop to
        ``UNTRACKED``. Recovery to ``TRACKING`` is automatic when a trusted fit
        returns; the give-up timeout on a prolonged occlusion belongs to the IBVS.
        """
        if not self._ever_tracked:
            self._reset(STATUS_UNTRACKED)
            self._draw_idle(frame)
            self._publish_debug(frame, msg)
            return
        if self.tracking_status != STATUS_OCCLUDED:
            self.get_logger().warn("Waypoint tracking OCCLUDED — holding last position.")
            self._set_status(STATUS_OCCLUDED)
        # No points published while OCCLUDED — only state (already set) + debug.
        self._draw_overlay(frame)
        self._publish_debug(frame, msg)

    # ------------------------------------------------------------------
    # Per-frame tracking

    def _track(self, gray, frame, msg):
        min_f = self._p("min_features")

        h, w = self.img_shape

        # Pre-warped (compositional) flow. Warp the keyframe by the running estimate
        # H_kf->prev so DIS only measures the small *residual*. Under forward motion
        # the big FOE expansion (near-fast / far-slow) is absorbed by the prediction,
        # keeping the flow inside DIS's range and near-uniform — the raw
        # keyframe->current flow was what made forward motion track poorly.
        H_pred = self._H_kf2prev
        kf_pred = cv2.warpPerspective(self.kf_gray, H_pred, (w, h))
        grid_pred = cv2.perspectiveTransform(
            self.kf_grid.reshape(-1, 1, 2), H_pred
        ).reshape(-1, 2)

        flow_fwd = self._dis.calc(kf_pred, gray, None)
        g_fwd = self._sample_flow(flow_fwd, grid_pred)  # residual: prediction->current
        grid_cur = grid_pred + g_fwd
        flow_bwd = self._dis.calc(gray, kf_pred, None)
        g_bwd = self._sample_flow(flow_bwd, grid_cur)
        fb_err = np.linalg.norm(g_fwd + g_bwd, axis=1)
        # Keyframe grid points the prediction carries off-frame have no valid
        # correspondence in the warped keyframe — drop them with the FB failures.
        in_bounds = (
            (grid_pred[:, 0] >= 0) & (grid_pred[:, 0] < w)
            & (grid_pred[:, 1] >= 0) & (grid_pred[:, 1] < h)
        )
        good = (fb_err < self._p("fb_thresh")) & in_bounds
        self._n_rejected = int((~good).sum())
        src = self.kf_grid[good]  # keyframe coords
        dst = grid_cur[good].astype(np.float32)  # current coords
        self._grid_vis = dst

        # Flow to the keyframe has broken (large baseline / occlusion) — re-anchor
        # to the current frame at the last-known positions to recover.
        if len(dst) < min_f:
            self._kf_disp = 0.0
            if self._set_keyframe(gray):  # re-anchor so the next frame can recover
                self._rekeys += 1
            self._handle_occlusion(frame, msg)  # flow broke — waypoints frozen
            return

        # Single-point mode uses a well-conditioned local **translation**; multi-point
        # trajectories use the ground **homography** (a homography can't be
        # conditioned from the compact grid around a single point).
        if self._single:
            H, inliers = self._estimate_translation(src, dst)
        else:
            H, inliers = self._estimate_transform(src, dst)
        if H is None:
            self._handle_occlusion(frame, msg)  # no usable fit — hold
            return

        inlier_mask = inliers.ravel() == 1
        self._inlier_ratio = float(inlier_mask.sum()) / max(len(dst), 1)
        src_in = src[inlier_mask]
        inlier_dst = dst[inlier_mask]  # measured plane support this frame
        self._grid_vis = inlier_dst

        # Piece 1 — bias the plane to the near band (homography only; a translation
        # is already local so there is nothing to re-anchor). RANSAC already rejects
        # gross far occlusion as outliers, so H is robust; this re-anchors the plane
        # on the near support so residual far-plane noise doesn't tilt the near points.
        if not self._single:
            H = self._refine_near(H, src_in, inlier_dst)

        warped = self._warp(H)
        if warped is None:
            self._handle_occlusion(frame, msg)  # degenerate warp -> hold
            return

        # Piece 2 — per-waypoint support: a waypoint is *measured* when enough
        # inlier grid points fall within support_radius of its warped position;
        # otherwise it is still placed by the plane but flagged coasting.
        self._support = self._waypoint_support(warped, inlier_dst)
        self._tracked_mask = self._support >= int(self._p("min_support"))

        # Piece 3 — node state follows the nearest *in-frame* waypoints, recomputed
        # every frame. Keying on fixed indices 0..k would false-trip OCCLUDED on
        # normal progress: as the robot advances, the nearest-at-init waypoints leave
        # the frame bottom and lose support. Selecting the nearest *still-in-frame*
        # waypoints instead means a far occlusion only flips its own flag and forward
        # progress doesn't read as loss. No in-frame waypoint at all -> nothing
        # measurable -> OCCLUDED.
        in_frame = (
            (warped[:, 0] >= 0) & (warped[:, 0] < w)
            & (warped[:, 1] >= 0) & (warped[:, 1] < h)
        )
        k = max(1, int(self._p("priority_count")))
        priority_idx = np.nonzero(in_frame)[0][:k]  # nearest-first order preserved
        if len(priority_idx) == 0 or not bool(self._tracked_mask[priority_idx].any()):
            self._handle_occlusion(frame, msg)  # near end lost -> hold
            return

        self.pts = warped
        self._mark_tracking()  # trusted measurement this frame
        self._H_kf2prev = H  # predictor that pre-warps the next frame's flow

        # Retire consumed waypoints: *any* waypoint whose warped position has crossed
        # below the frame bottom has passed under the robot. Order-agnostic (a
        # mis-ordered waypoint set still retires the right points, unlike a
        # front-only scan), and a hard drop — a consumed point is gone from
        # ``kf_pts``, so no later re-fit can resurrect it or fling it off-screen.
        keep = self.pts[:, 1] < h
        if not keep.all():
            self.pts = self.pts[keep].copy()
            self.kf_pts = self.kf_pts[keep].copy()
            self._tracked_mask = self._tracked_mask[keep]
            self._support = self._support[keep]
            if len(self.pts) == 0:
                self.get_logger().info("Trajectory consumed — all waypoints passed.")
                self._pending_waypoints = None
                self._pending_stamp = None
                self._reset(STATUS_UNTRACKED)
                self._draw_idle(frame)
                self._publish_debug(frame, msg)
                return

        # Transition to single-point mode once a multi-point trajectory has consumed
        # down to its last waypoint — a homography can't be conditioned from one
        # point. Re-key so the grid re-lays as a single-point neighborhood (avoiding
        # a depth-averaged translation from the old wide grid), then publish + return.
        if len(self.pts) == 1 and not self._single:
            self._single = True
            self.get_logger().info("Last waypoint reached — single-point mode.")
            if self._set_keyframe(gray):
                self._rekeys += 1
            self._draw_and_publish(frame, msg)
            return

        # Re-key when the baseline grows enough that the pre-warp resampling / view
        # overlap degrades — but only from a high-confidence frame (a re-key freezes
        # the current warped estimate in as the new anchor, so error committed here
        # becomes permanent drift). ``kf_disp`` is the *total* keyframe->current
        # baseline (not the small residual DIS now sees), preserving the "re-key when
        # the anchor gets far" semantics; the pre-warp lets that baseline grow larger
        # before DIS breaks, so ``rekey_flow_px`` can be raised for less drift.
        self._kf_disp = float(
            np.median(np.linalg.norm((grid_cur - self.kf_grid)[good], axis=1))
        )
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
        # New keyframe == current frame, so the prediction resets to identity
        # (residual DIS on the next frame is just one frame of motion).
        self._H_kf2prev = np.eye(3)
        return True

    def _estimate_transform(self, src, dst):
        """Fit the keyframe->current ground **homography** (RANSAC), or ``None``.

        A homography is the correct model for a plane under any camera motion, so it
        is the only one. A frame that can't support one (too few points, or a
        degenerate/insane fit) is **held** rather than tracked through a weaker model
        that can't represent the perspective looming of forward motion.
        """
        if len(src) < self._p("min_homography_features"):
            return None, None
        H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
        if H is None or inliers is None or not self._is_sane_homography(H):
            return None, None
        return H.astype(np.float64), inliers

    def _estimate_translation(self, src, dst):
        """Robust pure **translation** (2 DOF) as an identity+shift homography.

        Well-conditioned from any support — even the compact disc of grid points
        around a single remaining waypoint, where a homography cannot be. The median
        shift is outlier-robust; inliers are the correspondences agreeing with it.
        """
        if len(src) < self._p("min_features"):
            return None, None
        d = dst - src
        t = np.median(d, axis=0)
        inliers = np.linalg.norm(d - t, axis=1) < 3.0
        if int(inliers.sum()) < self._p("min_features"):
            return None, None
        H = np.array([[1.0, 0.0, t[0]], [0.0, 1.0, t[1]], [0.0, 0.0, 1.0]])
        return H, inliers.reshape(-1, 1).astype(np.uint8)

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
        # Reject ill-conditioned *perspective*. The 2x2 checks above miss unconstrained
        # ``H[2,:]`` terms — which is what a fit from a small/compact support region
        # (few waypoints left) produces: fine near the cluster, flinging everything
        # far away off-screen. Map the frame corners and require they stay within a
        # frame-size margin of the frame.
        h, w = self.img_shape
        corners = np.array(
            [[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32
        ).reshape(-1, 1, 2)
        mapped = cv2.perspectiveTransform(corners, H.astype(np.float64)).reshape(-1, 2)
        if not np.all(np.isfinite(mapped)):
            return False
        m = float(max(h, w))
        if (
            mapped[:, 0].min() < -m or mapped[:, 0].max() > w + m
            or mapped[:, 1].min() < -m or mapped[:, 1].max() > h + m
        ):
            return False
        return True

    def _refine_near(self, H, src_in, dst_in):
        """Re-fit the plane on inlier grid near the priority (nearest) waypoints.

        ``src_in`` / ``dst_in`` are the RANSAC-inlier grid correspondences (keyframe
        -> current), already clean, so a plain least-squares homography suffices. We
        keep only inliers within ``support_radius`` of the nearest ``priority_count``
        keyframe waypoints, so the near end governs the plane the far (extrapolated)
        points ride on. Returns ``H`` unchanged when near support is too thin or the
        refit is degenerate.
        """
        k = max(1, int(self._p("priority_count")))
        near_kf = self.kf_pts[:k]
        r = float(self._p("support_radius"))
        d = np.linalg.norm(src_in[:, None, :] - near_kf[None, :, :], axis=2)
        near = d.min(axis=1) <= r
        if int(near.sum()) < self._p("min_homography_features"):
            return H  # not enough near support to trust a refit
        H2, _ = cv2.findHomography(src_in[near], dst_in[near], 0)  # LS, no RANSAC
        if H2 is None or not self._is_sane_homography(H2.astype(np.float64)):
            return H
        return H2.astype(np.float64)

    def _waypoint_support(self, warped, inlier_dst):
        """Per-waypoint count of inlier grid points within ``support_radius`` px."""
        n = len(warped)
        if len(inlier_dst) == 0:
            return np.zeros(n, dtype=np.int32)
        r = float(self._p("support_radius"))
        d = np.linalg.norm(warped[:, None, :] - inlier_dst[None, :, :], axis=2)
        return (d <= r).sum(axis=1).astype(np.int32)

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
        # A lone waypoint (single-point mode) has no path band; give it a real
        # neighborhood so the translation has enough flow support.
        if len(pts) == 1:
            pad = max(pad, int(self._p("support_radius")))

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

        Only reached while `TRACKING` (lk_tracker parity — no data is published while
        `OCCLUDED`/`UNTRACKED`). `tracked` marks each waypoint *measured* vs
        *coasting*: `true` when it has local inlier-grid support this frame, `false`
        for a far/occluded/off-image waypoint the plane is still placing.
        """
        h, w = self.img_shape
        tracked = self._tracked_mask
        measuring = self.tracking_status == STATUS_TRACKING and tracked is not None
        out = VisualWaypoints()
        out.header = msg.header
        for i, pt in enumerate(self.pts):
            p = Point()
            # Clamp to the published [-1, 1] contract — a coasting/off-frame waypoint
            # can extrapolate far outside the frame, and an unclamped value would map
            # to an enormous steering command in a naive consumer.
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
        # Amber while OCCLUDED (waypoints held/frozen), green while tracking —
        # same colour convention as lk_tracker's overlay.
        occluded = self.tracking_status == STATUS_OCCLUDED
        color = (0, 215, 255) if occluded else (0, 255, 0)

        # Inlier grid driving the homography.
        if self._grid_vis is not None:
            for gx, gy in self._grid_vis:
                cv2.circle(frame, (int(gx), int(gy)), 1, (255, 200, 0), -1)

        # Waypoint polyline + points: measured filled, coasting hollow.
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
        mode = "trans" if self._single else "homog"
        cv2.putText(
            frame,
            f"{self.tracking_status} [{mode}]  {len(self.pts)} pts  "
            f"meas={n_meas}/{len(self.pts)}  inliers={self._inlier_ratio:.2f}  "
            f"reject={self._n_rejected}  rekeys={self._rekeys}  "
            f"kf_disp={self._kf_disp:.1f}",
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
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
