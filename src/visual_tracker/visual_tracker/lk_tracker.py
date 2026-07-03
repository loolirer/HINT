import rclpy
import rclpy.parameter
from collections import deque
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, RegionOfInterest, CompressedImage
from std_msgs.msg import String
from hint_interfaces.srv import SetTarget, StopTracking
from cv_bridge import CvBridge
import cv2
import numpy as np

STATUS_UNTRACKED = "UNTRACKED"
STATUS_TRACKING = "TRACKING"
STATUS_OCCLUDED = "OCCLUDED"

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, 0.03)

_ORB = cv2.ORB_create(nfeatures=500)
_MATCHER = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

# Latched QoS so late-joining subscribers receive the current state immediately.
_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LKTrackerNode(Node):
    def __init__(self):
        super().__init__("lk_tracker_node")

        # --- Tuning parameters ---
        self.declare_parameter("fb_thresh", 2.0)
        self.declare_parameter("ema_alpha", 0.20)
        self.declare_parameter("ncc_thresh", 0.60)
        self.declare_parameter("ncc_redetect_thresh", 0.75)
        self.declare_parameter("inlier_ratio_thresh", 0.60)
        self.declare_parameter("max_features", 300)
        self.declare_parameter("min_features", 10)

        # --- Publishers ---
        self.pub_bbox = self.create_publisher(RegionOfInterest, "/tracking/bbox", 10)
        self.pub_debug = self.create_publisher(Image, "/camera/tracking", 10)
        self.pub_state = self.create_publisher(String, "/tracking/state", _LATCHED_QOS)

        # --- Services ---
        self.create_service(SetTarget, "~/set_target", self._srv_set_target)
        self.create_service(StopTracking, "~/stop_tracking", self._srv_stop_tracking)

        # --- Internal state ---
        self._pending_roi = None
        self._pending_stamp = None
        # Ring buffer: last 10 frames keyed by (sec, nanosec) for stamp-based init.
        self._frame_buffer = deque(maxlen=10)
        self.bridge = CvBridge()

        self._reset(STATUS_UNTRACKED)

        # Re-publish state every second so subscribers that join late catch up.
        self.create_timer(1.0, lambda: self._publish_state(self.tracking_status))

        # Subscribe last so the node is fully initialised before images arrive.
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10
        )

        self.get_logger().info(
            "LK tracker ready — call ~/set_target to start tracking."
        )

    # ------------------------------------------------------------------
    # Services

    def _srv_set_target(self, request, response):
        self._reset(STATUS_UNTRACKED)
        self._pending_roi = request.roi
        self._pending_stamp = request.stamp
        response.accepted = True
        response.message = "Target accepted — initialising on next matching frame."
        self.get_logger().info("New target received via set_target.")
        return response

    def _srv_stop_tracking(self, request, response):
        self._pending_roi = None
        self._pending_stamp = None
        self._reset(STATUS_UNTRACKED)
        self.get_logger().info("Tracking stopped by stop_tracking service call.")
        return response

    # ------------------------------------------------------------------
    # State management

    def _reset(self, status):
        self.initialized = False
        self.init_bbox = None
        self.prev_gray = None
        self.pts_init = None
        self.pts_prev = None
        self.bbox_corners_init = None
        self.smoothed_corners = None
        self.init_patch = None
        self.orb_kp_abs = None
        self.orb_des = None
        self._ever_tracked = False
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
        min_features = self._p("min_features")

        # Buffer every frame for stamp-based initialisation lookups.
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._frame_buffer.append((stamp_key, gray.copy()))

        # --- Initialisation ---
        if not self.initialized:
            if self._pending_roi is None:
                cv2.putText(
                    frame,
                    "UNTRACKED — call ~/set_target to start",
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (128, 128, 128),
                    2,
                )
                self._publish_debug(frame, msg)
                return

            roi = self._pending_roi
            self._pending_roi = None

            # Use the frame whose stamp matches the request, or fall back to current.
            init_gray = gray
            if self._pending_stamp is not None:
                target_key = (self._pending_stamp.sec, self._pending_stamp.nanosec)
                if target_key != (0, 0):
                    for buf_key, buf_gray in self._frame_buffer:
                        if buf_key == target_key:
                            init_gray = buf_gray
                            break
                self._pending_stamp = None

            init_bbox = (
                int(roi.x_offset),
                int(roi.y_offset),
                int(roi.width),
                int(roi.height),
            )
            pts = self._detect_features(init_gray, init_bbox)

            if pts is None or len(pts) < min_features:
                self.get_logger().warn(
                    "Too few features in target bbox — call set_target again."
                )
                cv2.putText(
                    frame,
                    "TOO FEW FEATURES — call set_target again",
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                self._publish_debug(frame, msg)
                return

            x, y, bw, bh = init_bbox
            self.init_bbox = init_bbox
            self.init_patch = init_gray[y : y + bh, x : x + bw].copy()
            self.prev_gray = init_gray
            self.pts_init = pts.copy()
            self.pts_prev = pts.copy()
            self.bbox_corners_init = self._bbox_corners(init_bbox)
            self.smoothed_corners = self.bbox_corners_init.copy()

            kp, des = _ORB.detectAndCompute(init_gray[y : y + bh, x : x + bw], None)
            if des is not None and len(kp) > 0:
                self.orb_kp_abs = np.array(
                    [[k.pt[0] + x, k.pt[1] + y] for k in kp], dtype=np.float32
                )
                self.orb_des = des
            else:
                self.orb_kp_abs = np.empty((0, 2), dtype=np.float32)
                self.orb_des = None
                self.get_logger().warn(
                    "ORB found no descriptors at init — re-detection disabled."
                )

            self.initialized = True
            self._set_status(STATUS_TRACKING)
            self.get_logger().info(f"Tracking initialised with {len(pts)} features.")
            self._publish_debug(frame, msg)
            return

        # --- LK with forward-backward filtering ---
        pts_cur, good = self._lk_fb(self.prev_gray, gray, self.pts_prev)

        good_init = self.pts_init[good]
        good_cur = pts_cur[good]

        if len(good_cur) < min_features:
            self._handle_occlusion(gray, frame, msg, reason="too few features")
            return

        M, inliers = cv2.estimateAffinePartial2D(
            good_init, good_cur, method=cv2.RANSAC, ransacReprojThreshold=3.0
        )
        if M is None or inliers is None:
            self._handle_occlusion(gray, frame, msg, reason="transform failed")
            return

        inlier_ratio = float(inliers.sum()) / max(len(good_cur), 1)
        score = self._appearance_score(gray, M)

        occluded = (score < self._p("ncc_thresh")) or (
            inlier_ratio < self._p("inlier_ratio_thresh")
        )

        if occluded:
            self._handle_occlusion(
                gray,
                frame,
                msg,
                reason="occluded",
                score=score,
                inlier_ratio=inlier_ratio,
            )
            return

        inlier_mask = inliers.ravel() == 1
        good_init = good_init[inlier_mask]
        good_cur = good_cur[inlier_mask]

        corners_cur = cv2.transform(self.bbox_corners_init, M)
        ema_alpha = self._p("ema_alpha")
        self.smoothed_corners = (
            ema_alpha * corners_cur + (1.0 - ema_alpha) * self.smoothed_corners
        )

        roi_out = self._corners_to_roi(self.smoothed_corners)
        self.pub_bbox.publish(roi_out)

        pts_poly = self.smoothed_corners.reshape(-1, 1, 2).astype(np.int32)
        cx = int(roi_out.x_offset + roi_out.width / 2)
        cy = int(roi_out.y_offset + roi_out.height / 2)
        cv2.polylines(frame, [pts_poly], True, (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        for p in good_cur:
            cv2.circle(frame, tuple(p.astype(int).ravel()), 2, (255, 200, 0), -1)
        cv2.putText(
            frame,
            f"TRACKING  ncc={score:.2f}  inliers={inlier_ratio:.2f}  pts={len(good_cur)}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        xs = self.smoothed_corners[:, 0, 0]
        ys = self.smoothed_corners[:, 0, 1]
        if score >= self._p("ncc_redetect_thresh") and len(good_cur) < min_features * 2:
            x0 = max(0, int(xs.min()))
            y0 = max(0, int(ys.min()))
            x1 = min(frame.shape[1], int(xs.max()))
            y1 = min(frame.shape[0], int(ys.max()))
            new_cur = self._detect_features(gray, (x0, y0, x1 - x0, y1 - y0))
            if new_cur is not None and len(new_cur) >= min_features:
                M_inv = cv2.invertAffineTransform(M)
                new_init = cv2.transform(new_cur, M_inv)
                good_init = np.vstack([good_init.reshape(-1, 1, 2), new_init])
                good_cur = np.vstack([good_cur.reshape(-1, 1, 2), new_cur])

        self.pts_init = good_init.reshape(-1, 1, 2)
        self.pts_prev = good_cur.reshape(-1, 1, 2)
        self.prev_gray = gray
        self._ever_tracked = True
        self._publish_debug(frame, msg)

    # ------------------------------------------------------------------
    # Helpers

    def _handle_occlusion(
        self, gray, frame, msg, reason, score=None, inlier_ratio=None
    ):
        if self._try_recover_orb(gray):
            roi_out = self._corners_to_roi(self.smoothed_corners)
            self.pub_bbox.publish(roi_out)
            pts_poly = self.smoothed_corners.reshape(-1, 1, 2).astype(np.int32)
            cv2.polylines(frame, [pts_poly], True, (0, 255, 128), 2)
            cv2.putText(
                frame,
                "RECOVERED",
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 128),
                2,
            )
        else:
            if self.smoothed_corners is not None:
                pts_poly = self.smoothed_corners.reshape(-1, 1, 2).astype(np.int32)
                cv2.polylines(frame, [pts_poly], True, (0, 215, 255), 2)
            label = f"OCCLUDED ({reason})"
            if score is not None:
                label += f"  ncc={score:.2f}"
            if inlier_ratio is not None:
                label += f"  inliers={inlier_ratio:.2f}"
            cv2.putText(
                frame, label, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 215, 255), 2
            )
            if not self._ever_tracked:
                self._reset(STATUS_UNTRACKED)
                self._publish_debug(frame, msg)
                return
            self._set_status(STATUS_OCCLUDED)
            empty = np.empty((0, 1, 2), dtype=np.float32)
            self.pts_init = empty
            self.pts_prev = empty
            self.prev_gray = gray
        self._publish_debug(frame, msg)

    def _try_recover_orb(self, gray):
        if (
            self.smoothed_corners is None
            or self.orb_des is None
            or len(self.orb_des) < 4
        ):
            return False

        xs = self.smoothed_corners[:, 0, 0]
        ys = self.smoothed_corners[:, 0, 1]
        cx, cy = float(xs.mean()), float(ys.mean())
        hw = max(float(xs.max() - xs.min()) * 0.75, 30.0)
        hh = max(float(ys.max() - ys.min()) * 0.75, 30.0)
        x0 = max(0, int(cx - hw))
        y0 = max(0, int(cy - hh))
        x1 = min(gray.shape[1], int(cx + hw))
        y1 = min(gray.shape[0], int(cy + hh))
        if x1 - x0 < 10 or y1 - y0 < 10:
            return False

        kp_cur, des_cur = _ORB.detectAndCompute(gray[y0:y1, x0:x1], None)
        if des_cur is None or len(kp_cur) < 4:
            return False

        matches = _MATCHER.knnMatch(self.orb_des, des_cur, k=2)
        good = [
            ms[0]
            for ms in matches
            if len(ms) == 2 and ms[0].distance < 0.75 * ms[1].distance
        ]
        if len(good) < 8:
            return False

        src = self.orb_kp_abs[[m.queryIdx for m in good]].reshape(-1, 1, 2)
        dst = np.array(
            [
                [kp_cur[m.trainIdx].pt[0] + x0, kp_cur[m.trainIdx].pt[1] + y0]
                for m in good
            ],
            dtype=np.float32,
        ).reshape(-1, 1, 2)

        M, inliers = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=5.0
        )
        if M is None or inliers is None or int(inliers.sum()) < 8:
            return False

        scale = np.sqrt(M[0, 0] ** 2 + M[0, 1] ** 2)
        if scale < 0.5 or scale > 2.0:
            return False

        mask = inliers.ravel() == 1
        self.pts_init = src[mask]
        self.pts_prev = dst[mask]
        self.smoothed_corners = cv2.transform(self.bbox_corners_init, M)
        self.prev_gray = gray
        self._set_status(STATUS_TRACKING)
        return True

    def _detect_features(self, gray, bbox):
        x, y, w, h = [int(v) for v in bbox]
        mask = np.zeros_like(gray)
        mask[y : y + h, x : x + w] = 255
        pts = cv2.goodFeaturesToTrack(
            gray,
            mask=mask,
            maxCorners=self._p("max_features"),
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7,
        )
        if pts is not None:
            pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), SUBPIX_CRITERIA)
        return pts

    def _bbox_corners(self, bbox):
        x, y, w, h = bbox
        return np.array(
            [
                [[float(x), float(y)]],
                [[float(x + w), float(y)]],
                [[float(x + w), float(y + h)]],
                [[float(x), float(y + h)]],
            ],
            dtype=np.float32,
        )

    def _corners_to_roi(self, corners):
        xs = corners[:, 0, 0]
        ys = corners[:, 0, 1]
        x0 = max(0, int(np.floor(xs.min())))
        y0 = max(0, int(np.floor(ys.min())))
        roi = RegionOfInterest()
        roi.x_offset = x0
        roi.y_offset = y0
        roi.width = max(0, int(np.ceil(xs.max())) - x0)
        roi.height = max(0, int(np.ceil(ys.max())) - y0)
        return roi

    def _lk_fb(self, prev_gray, gray, pts):
        if pts is None or len(pts) == 0:
            empty = np.empty((0, 1, 2), dtype=np.float32)
            return empty, np.zeros(0, dtype=bool)
        pts_fwd, st_fwd, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, gray, pts, None, **LK_PARAMS
        )
        pts_bwd, st_bwd, _ = cv2.calcOpticalFlowPyrLK(
            gray, prev_gray, pts_fwd, None, **LK_PARAMS
        )
        fb_error = np.abs(pts - pts_bwd).max(axis=2).ravel()
        good = (
            (st_fwd.ravel() == 1)
            & (st_bwd.ravel() == 1)
            & (fb_error < self._p("fb_thresh"))
        )
        return pts_fwd, good

    def _ncc(self, a, b):
        if a.shape != b.shape:
            b = cv2.resize(b, (a.shape[1], a.shape[0]))
        a = a.astype(np.float32) - a.mean()
        b = b.astype(np.float32) - b.mean()
        denom = np.sqrt((a**2).sum() * (b**2).sum())
        return float((a * b).sum() / denom) if denom > 1e-6 else 0.0

    def _appearance_score(self, gray, M):
        h, w = gray.shape
        M_inv = cv2.invertAffineTransform(M)
        warped = cv2.warpAffine(gray, M_inv, (w, h))
        x, y, bw, bh = self.init_bbox
        patch = warped[y : y + bh, x : x + bw]
        return self._ncc(self.init_patch, patch)

    def _publish_debug(self, frame, original_msg):
        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = original_msg.header
        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LKTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
