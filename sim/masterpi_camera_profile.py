"""Operational camera profile for the physical ugrp1 MasterPi.

The simulator keeps the measured 640x480 fisheye intrinsics/distortion from the
physical icspring camera. The rigid mount is intentionally more conservative:
the current production mount is a mechanically centered structural model using
the existing successful-capture link/z/down-pitch anchors. It is NOT labelled as
fully calibrated because the planned 12-fit + 12-held-out hand-eye dataset has
not been collected.

A historical nine-point fit from one servo pose is preserved below as archival
pixel-parity evidence. It has low reprojection error, but no independent
held-out hand-eye set validates its lateral translation or yaw, so those terms
must not be promoted into the physical production mount.
"""
from __future__ import annotations

from functools import lru_cache
from typing import Final

import math
import numpy as np

CAMERA_MOUNT_STATUS: Final = "CENTERED_STRUCTURAL_MOUNT_UNVALIDATED"
CAMERA_CALIBRATION_ID: Final = "ugrp1-icspring-fisheye-centered-mount-provisional-20260831"
CAMERA_NATIVE_WIDTH: Final = 640
CAMERA_NATIVE_HEIGHT: Final = 480

# Measured physical intrinsics/distortion.
CAMERA_FX_PX: Final = 619.5194540517232
CAMERA_FY_PX: Final = 622.1654884818748
CAMERA_CX_PX: Final = 287.68910608896766
CAMERA_CY_PX: Final = 218.69686648455726
CAMERA_FISHEYE_D: Final[tuple[float, float, float, float]] = (
    -0.0199819517209417,
    -0.1707593617153504,
    -0.2568318201202376,
    0.9650933421665181,
)

# Angular span of the undistorted K-pinhole image. The raw fisheye image has no
# exact single FOV because distortion is nonlinear.
CAMERA_PINHOLE_HFOV_DEG: Final = math.degrees(
    math.atan(CAMERA_CX_PX / CAMERA_FX_PX)
    + math.atan((CAMERA_NATIVE_WIDTH - CAMERA_CX_PX) / CAMERA_FX_PX)
)
CAMERA_PINHOLE_VFOV_DEG: Final = math.degrees(
    math.atan(CAMERA_CY_PX / CAMERA_FY_PX)
    + math.atan((CAMERA_NATIVE_HEIGHT - CAMERA_CY_PX) / CAMERA_FY_PX)
)

# Production camera pose relative to the MuJoCo ``gripper`` body. The y=0 mount
# keeps the lens on the mechanical centerline. The 67 mm link and 13.6 mm local-z
# are existing successful-capture structural anchors. We retain only the pitch
# component (~7.46 deg) of the historical image fit because it preserves the
# measured range response, while explicitly removing its unsupported lateral
# translation/yaw/roll. This remains provisional until held-out validation.
# Quaternion is MuJoCo wxyz and gives zero optical yaw/roll.
CAMERA_LOCAL_POS_M: Final[tuple[float, float, float]] = (
    0.0670,
    0.0,
    0.0136,
)
CAMERA_LOCAL_QUAT_WXYZ: Final[tuple[float, float, float, float]] = (
    0.466417262018992,
    0.531464897891391,
    -0.531464897891391,
    -0.466417262018992,
)

# Archived one-pose nine-point fit. It reproduces that fixture well but contains
# an 8.1 mm lateral translation and about 5.5 degrees of optical yaw. Because no
# independent held-out set validates those terms, it is archival evidence only.
CAMERA_PARITY_FIT_LOCAL_POS_M: Final[tuple[float, float, float]] = (
    0.0391112236705376,
    0.0081199008231831,
    0.0151010908254302,
)
CAMERA_PARITY_FIT_LOCAL_QUAT_WXYZ: Final[tuple[float, float, float, float]] = (
    0.4970424680275261,
    0.5464191263921774,
    -0.5160775268254018,
    -0.4336345345976175,
)

CAMERA_HAND_EYE_SERVO_POSE: Final[dict[int, int]] = {
    3: 672,
    4: 2181,
    5: 1746,
    6: 1500,
}

# Old collection convention was [lateral-right, forward, up] cm. These points
# remain a historical parity fixture for the archived fit; they are not a gate
# that forces the centered production mount back to the one-pose 6-DoF result.
CAMERA_PARITY_POINTS: Final = (
    ((0.0, 18.0, 1.5), (370.98814005115577, 370.3440737977666)),
    ((-4.0, 18.0, 1.5), (231.4424962292609, 363.1670825313086)),
    ((4.0, 18.0, 1.5), (501.64998664071146, 374.23031100108307)),
    ((-6.0, 21.0, 1.5), (186.82904877602735, 282.94782211789254)),
    ((0.0, 21.0, 1.5), (370.99441861591356, 285.88745329257046)),
    ((6.0, 21.0, 1.5), (547.3971588393899, 289.2198490272582)),
    ((5.0, 24.0, 1.5), (510.3514065069297, 213.7296114573664)),
    ((0.0, 24.0, 1.5), (369.73027896058517, 207.93735084345892)),
    ((-5.0, 24.0, 1.5), (227.65017743383643, 206.4959787816564)),
)

# Historical evidence for CAMERA_PARITY_FIT_* only.
CAMERA_ANALYTICAL_REPROJECTION_MEAN_PX: Final = 2.3101735410050814
CAMERA_ANALYTICAL_REPROJECTION_MAX_PX: Final = 4.974092152136078
CAMERA_SIM_REPLAY_MEAN_PX: Final = 3.01586846173696
CAMERA_SIM_REPLAY_MAX_PX: Final = 7.898002504161762


def scaled_camera_matrix(width: int, height: int) -> np.ndarray:
    """Return the measured physical OpenCV K scaled from native 640x480."""
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        raise ValueError("camera dimensions must be positive")
    sx = float(width) / float(CAMERA_NATIVE_WIDTH)
    sy = float(height) / float(CAMERA_NATIVE_HEIGHT)
    return np.asarray(
        [
            [CAMERA_FX_PX * sx, 0.0, CAMERA_CX_PX * sx],
            [0.0, CAMERA_FY_PX * sy, CAMERA_CY_PX * sy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def mujoco_pixel_intrinsic(width: int, height: int) -> np.ndarray:
    """Return MuJoCo [fx,fy,principal-x,principal-y] pixel intrinsics."""
    k = scaled_camera_matrix(width, height)
    return np.asarray(
        [
            k[0, 0],
            k[1, 1],
            float(width) / 2.0 - k[0, 2],
            float(height) / 2.0 - k[1, 2],
        ],
        dtype=np.float64,
    )


@lru_cache(maxsize=8)
def raw_fisheye_remap(width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """Map a pinhole render into the raw fisheye pixel geometry seen by REAL."""
    import cv2

    k = scaled_camera_matrix(width, height)
    d = np.asarray(CAMERA_FISHEYE_D, dtype=np.float64).reshape(4, 1)
    ys, xs = np.indices((int(height), int(width)), dtype=np.float64)
    raw_pixels = np.stack((xs, ys), axis=-1).reshape(-1, 1, 2)
    ideal_pixels = cv2.fisheye.undistortPoints(
        raw_pixels,
        k,
        d,
        R=np.eye(3, dtype=np.float64),
        P=k,
    ).reshape(int(height), int(width), 2)
    return (
        ideal_pixels[..., 0].astype(np.float32),
        ideal_pixels[..., 1].astype(np.float32),
    )
