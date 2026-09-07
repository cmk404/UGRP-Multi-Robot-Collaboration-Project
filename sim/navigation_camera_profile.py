"""Static, simulator-only pinhole calibration for forward navigation RGB.

The projection helpers depend only on pixels and this manufactured mount.  They
must not import or inspect MuJoCo state, object poses, depth, or segmentation.
Robot coordinates are x-forward, y-left, z-up; image coordinates are
u-right, v-down.
"""
from __future__ import annotations

import math

import numpy as np


NAV_CAMERA_NAME = "nav_cam"
NAV_IMAGE_WIDTH = 640
NAV_IMAGE_HEIGHT = 480
NAV_VERTICAL_FOVY_DEG = 70.0
NAV_MOUNT_XYZ_M = (0.050, 0.0, 0.320)
# The camera is attached to the robot root. At the nominal settled fixture pose
# that root is one wheel radius above the floor, so floor projection must use
# the optical centre's floor height rather than its body-local z coordinate.
NAV_NOMINAL_BASE_HEIGHT_M = 0.0325
NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M = NAV_NOMINAL_BASE_HEIGHT_M + NAV_MOUNT_XYZ_M[2]
NAV_PITCH_DOWN_DEG = 25.0
NAV_CALIBRATION_ID = "sim-nav-pinhole-640x480-fovy70-mount-v2"

NAV_FY_PX = (NAV_IMAGE_HEIGHT / 2.0) / math.tan(math.radians(NAV_VERTICAL_FOVY_DEG) / 2.0)
NAV_FX_PX = NAV_FY_PX
NAV_CX_PX = (NAV_IMAGE_WIDTH - 1.0) / 2.0
NAV_CY_PX = (NAV_IMAGE_HEIGHT - 1.0) / 2.0


def camera_intrinsics(width: int = NAV_IMAGE_WIDTH, height: int = NAV_IMAGE_HEIGHT) -> np.ndarray:
    """Return a square-pixel pinhole matrix scaled from the 640x480 profile."""
    width = int(width); height = int(height)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    sx = width / NAV_IMAGE_WIDTH
    sy = height / NAV_IMAGE_HEIGHT
    return np.asarray(((NAV_FX_PX * sx, 0.0, (width - 1.0) / 2.0),
                       (0.0, NAV_FY_PX * sy, (height - 1.0) / 2.0),
                       (0.0, 0.0, 1.0)), dtype=float)


def pixel_ray_camera(u: float, v: float, width: int = NAV_IMAGE_WIDTH,
                     height: int = NAV_IMAGE_HEIGHT) -> np.ndarray:
    """Return an unnormalised [right, down, forward] optical ray."""
    k = camera_intrinsics(width, height)
    return np.asarray(((float(u) - k[0, 2]) / k[0, 0],
                       (float(v) - k[1, 2]) / k[1, 1], 1.0), dtype=float)


def ground_point_robot(u: float, v: float, width: int = NAV_IMAGE_WIDTH,
                       height: int = NAV_IMAGE_HEIGHT) -> tuple[float, float] | None:
    """Intersect a pixel ray with z=0 and return (x-forward, y-left)."""
    right, down, forward = pixel_ray_camera(u, v, width, height)
    pitch = math.radians(NAV_PITCH_DOWN_DEG)
    # Optical basis in robot coordinates.
    ray = (np.asarray((0.0, -1.0, 0.0)) * right
           + np.asarray((-math.sin(pitch), 0.0, -math.cos(pitch))) * down
           + np.asarray((math.cos(pitch), 0.0, -math.sin(pitch))) * forward)
    if ray[2] >= -1e-12:
        return None
    scale = -NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M / float(ray[2])
    if scale <= 0.0:
        return None
    origin_above_floor = np.asarray((NAV_MOUNT_XYZ_M[0], NAV_MOUNT_XYZ_M[1],
                                     NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M), dtype=float)
    point = origin_above_floor + scale * ray
    return float(point[0]), float(point[1])


def calibration() -> dict[str, object]:
    """Serializable capture contract for an image-only navigation planner."""
    return {
        "calibration_id": NAV_CALIBRATION_ID,
        "camera": NAV_CAMERA_NAME,
        "model": "pinhole",
        "resolution": [NAV_IMAGE_WIDTH, NAV_IMAGE_HEIGHT],
        "fovy_deg": NAV_VERTICAL_FOVY_DEG,
        "fx_px": NAV_FX_PX, "fy_px": NAV_FY_PX,
        "cx_px": NAV_CX_PX, "cy_px": NAV_CY_PX,
        "mount_xyz_robot_m": list(NAV_MOUNT_XYZ_M),
        "nominal_base_height_above_floor_m": NAV_NOMINAL_BASE_HEIGHT_M,
        "camera_height_above_floor_m": NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M,
        "pitch_down_deg": NAV_PITCH_DOWN_DEG,
        "robot_frame": "x-forward y-left z-up",
        "image_frame": "u-right v-down",
        "distortion": "none",
    }


__all__ = [name for name in globals() if name.startswith("NAV_")] + [
    "camera_intrinsics", "pixel_ray_camera", "ground_point_robot", "calibration",
]
