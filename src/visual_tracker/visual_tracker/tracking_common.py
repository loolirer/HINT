"""Shared classic-CV tracking primitives for the ``visual_tracker`` nodes.

Pure helpers (no ROS node state) reused by both ``lk_tracker`` and
``waypoint_tracker`` so the two trackers share a single implementation of LK
forward-backward flow, feature detection, NCC and bbox geometry. Keeping the
tracking backend in one place means a future swap (e.g. a learned point tracker
replacing the classic LK measurement) touches this module, not every node.
"""

import cv2
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import RegionOfInterest

# Shared tracking states, published on the per-node ``.../state`` topic.
STATUS_UNTRACKED = "UNTRACKED"
STATUS_TRACKING = "TRACKING"
STATUS_OCCLUDED = "OCCLUDED"

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, 0.03)

# Latched QoS so late-joining subscribers receive the current state immediately.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def detect_features(gray, bbox, max_features, quality_level=0.01):
    """Shi-Tomasi corners inside ``bbox`` (x, y, w, h), refined to sub-pixel."""
    x, y, w, h = [int(v) for v in bbox]
    mask = np.zeros_like(gray)
    mask[y : y + h, x : x + w] = 255
    pts = cv2.goodFeaturesToTrack(
        gray,
        mask=mask,
        maxCorners=max_features,
        qualityLevel=quality_level,
        minDistance=7,
        blockSize=7,
    )
    if pts is not None:
        pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), SUBPIX_CRITERIA)
    return pts


def lk_fb(prev_gray, gray, pts, fb_thresh):
    """Pyramidal LK with forward-backward consistency filtering.

    Returns ``(pts_fwd, good)`` where ``good`` is a boolean mask of points whose
    round-trip (forward then backward) error stays under ``fb_thresh`` pixels.
    """
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
        (st_fwd.ravel() == 1) & (st_bwd.ravel() == 1) & (fb_error < fb_thresh)
    )
    return pts_fwd, good


def ncc(a, b):
    """Zero-mean normalised cross-correlation of two equally-shaped patches."""
    if a.shape != b.shape:
        b = cv2.resize(b, (a.shape[1], a.shape[0]))
    a = a.astype(np.float32) - a.mean()
    b = b.astype(np.float32) - b.mean()
    denom = np.sqrt((a**2).sum() * (b**2).sum())
    return float((a * b).sum() / denom) if denom > 1e-6 else 0.0


def bbox_corners(bbox):
    """(x, y, w, h) -> 4x1x2 float32 corner array (TL, TR, BR, BL)."""
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


def corners_to_roi(corners):
    """Axis-aligned ``RegionOfInterest`` bounding the given 4x1x2 corners."""
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
