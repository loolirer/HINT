import math

import numpy as np

RIG_PARAMS = {
    "camera_height": 0.14,          # m above the ground plane
    "camera_forward_offset": 0.0,   # m ahead of the base origin
    "camera_tilt": 0.0,             # rad, positive = pitched down
    "camera_hfov_deg": 62.2,        # horizontal FOV (deg) — sets the focal length
}


class CameraRig:
    def __init__(self, height, forward_offset, tilt, hfov_deg):
        self.height = float(height)
        self.forward_offset = float(forward_offset)
        self.tilt = float(tilt)
        self.hfov_deg = float(hfov_deg)

    @classmethod
    def declare(cls, node):
        for name, default in RIG_PARAMS.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, default)

    @classmethod
    def from_node(cls, node):
        cls.declare(node)
        g = node.get_parameter
        return cls(
            g("camera_height").value,
            g("camera_forward_offset").value,
            g("camera_tilt").value,
            g("camera_hfov_deg").value,
        )

    def _focal(self, w):
        return (w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    def pixels_to_ground(self, pts, w, h):
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
