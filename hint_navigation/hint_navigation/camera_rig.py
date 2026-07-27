"""camera_rig.py — the camera rig (inverse-perspective-mapping), single source of truth.

hint_navigation owns the camera's physical placement (its *rig*: height above the ground,
forward offset, downward tilt) and the horizontal FOV that sets the focal length. This
module holds that rig **and** the ground-plane projection math shared by the navigation
nodes, so the two directions can never drift apart (they used to be hand-synced copies
across three nodes):

- ``trajectory_navigator`` : ``pixels_to_ground``  (VLM markers -> base_link metric path)
- ``obstacle_projector``   : ``ground_to_pixels``  (ground-mask -> BEV obstacle homography)
- ``visual_debug``         : ``ground_to_pixels``  (odom paths -> image overlay)

``pixels_to_ground`` and ``ground_to_pixels`` are exact inverses of one pinhole + tilt +
height model. Ground points are ``base_link`` metric (x forward, y left, z = 0 implied);
pixels are ``(u, v)`` in an image of size ``(w, h)``. The model is resolution-invariant
(focal length and principal point scale with ``w``/``h``), so callers pass whatever image
size their pixels are expressed in (full-res markers, or the coarse processing-grid mask).

Pure and stateless — no ROS, no node. ``CameraRig.from_node(node)`` reads the four rig
parameters (declaring them if absent) and returns a snapshot; call it at the point of use
to keep the rig live-adjustable (``ros2 param set`` while calibrating), or build one
directly with the constructor.
"""

import math

import numpy as np

# The four rig parameters + their defaults, declared once here so every consumer node
# shares the exact same parameter schema (bringup injects the real rig geometry).
RIG_PARAMS = {
    "camera_height": 0.14,          # m above the ground plane
    "camera_forward_offset": 0.0,   # m ahead of the base origin
    "camera_tilt": 0.0,             # rad, positive = pitched down
    "camera_hfov_deg": 62.2,        # horizontal FOV (deg) — sets the focal length
}


class CameraRig:
    """Pinhole + tilt + height ground-plane projector (both directions)."""

    def __init__(self, height, forward_offset, tilt, hfov_deg):
        self.height = float(height)
        self.forward_offset = float(forward_offset)
        self.tilt = float(tilt)
        self.hfov_deg = float(hfov_deg)

    @classmethod
    def declare(cls, node):
        """Declare the rig parameters on ``node`` (idempotent)."""
        for name, default in RIG_PARAMS.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, default)

    @classmethod
    def from_node(cls, node):
        """Snapshot the rig from ``node``'s current parameters (declaring if absent)."""
        cls.declare(node)
        g = node.get_parameter
        return cls(
            g("camera_height").value,
            g("camera_forward_offset").value,
            g("camera_tilt").value,
            g("camera_hfov_deg").value,
        )

    def _focal(self, w):
        """Focal length (px) implied by the horizontal FOV at image width ``w``."""
        return (w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    def pixels_to_ground(self, pts, w, h):
        """Back-project pixel points ``(N, 2)`` onto the ground plane in ``base_link``
        (X forward, Y left). Points on/above the horizon clamp to a far ground distance
        rather than diverging."""
        f = self._focal(w)
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        cos_t, sin_t = math.cos(self.tilt), math.sin(self.tilt)
        xn = (pts[:, 0] - cx) / f
        yn = (pts[:, 1] - cy) / f
        denom = cos_t * yn + sin_t
        denom = np.where(denom > 1e-4, denom, 1e-4)  # clamp horizon/above to far
        t = self.height / denom
        X = self.forward_offset + t * (cos_t - sin_t * yn)
        Y = t * (-xn)
        return np.stack([X, Y], axis=1).astype(np.float64)

    def ground_to_pixels(self, gxy, w, h):
        """Project ground points ``(N, 2)`` metric ``base_link`` to pixels; returns
        ``(pix (N, 2), in_front mask)``."""
        f = self._focal(w)
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        cos_t, sin_t = math.cos(self.tilt), math.sin(self.tilt)
        dx = gxy[:, 0] - self.forward_offset
        cam_z = cos_t * dx + sin_t * self.height
        cam_x = -gxy[:, 1]
        cam_y = -sin_t * dx + cos_t * self.height
        z = np.where(cam_z > 1e-6, cam_z, 1e-6)
        pix = np.stack([cx + f * cam_x / z, cy + f * cam_y / z], axis=1)
        return pix, cam_z > 1e-6
