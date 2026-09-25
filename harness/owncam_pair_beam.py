"""Own-wrist-camera perception for a two-robot long_beam carry (FEASIBILITY STUDY).

Pure functions over one robot's own ``robot_cam`` JPEG and its own ISSUED
servo PWM. No simulator import, no truth, no partner image. This is a stub for
the team-cargo partial-visibility perception Kiro B2 is building (branch
``kiro/zone-own-perception-v2`` did not exist when this was written); only the
lime hue band of ``long_beam`` is shared knowledge with the catalogue.

* ``observe_beam``: lime pixels -> base-frame rays -> intersection with the beam
  top plane (z = 0.032 m) -> PCA axis and the near end (closest point along the
  axis). The grip point is the black band centre, 0.03 m inside the end
  (``sim.zone_cargo`` long_beam geometry: grip_x = 0.30 - 0.03).
* ``align_command``: mecanum command that puts that grip point at the calibrated
  grasp radius (0.155 m, centred) with the robot on the beam axis.
* ``held_signature`` / ``signature_iou``: the held bar's lime+band pixels in the
  lower image; while the handle stays in the jaws the camera (on the gripper)
  sees it at a fixed place, so a large change means slip or loss.
"""
from __future__ import annotations

import base64
import math
from typing import Any, Mapping

import cv2
import numpy as np

from harness.owncam_view import base_rays, posture, valid_pixel_mask

LIME_LO, LIME_HI = (36, 60, 40), (54, 255, 255)
BAND_V_MAX = 60                    # black grip band
BEAM_TOP_Z_M = .032
GRIP_INSET_M = .03                 # band centre from the beam end
GRASP_RADIUS_M = .155
MIN_POINTS = 60
RAY_STEP = 2
ALIGN_TOL_M, ALIGN_TOL_RAD = .008, .035
MIN_COMMAND = .035                 # smaller mecanum commands stall the chassis (zone skill dev 401)
BORDER_PX = 14
# Look postures by measured grip distance (dev 601: in SEARCH the near end leaves the valid
# fisheye region below ~0.30 m; the inspection pose sees band and end at the 0.155 m station).
LOOK_POSTURES = (
    ('search', .33, {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}),
    ('p45', .24, {**posture(.155, .14, -45.), 1: 2000}),
    ('inspect', 0., {1: 2000, 3: 508, 4: 2432, 5: 1320, 6: 1500}),
)


def _inner_valid():
    valid = valid_pixel_mask(1).astype(np.uint8)
    return cv2.erode(valid, np.ones((2 * BORDER_PX + 1,) * 2, np.uint8)).astype(bool)


_INNER = None


def look_posture(grip_distance_m: float | None) -> tuple[str, dict[int, int]]:
    if grip_distance_m is None:
        return LOOK_POSTURES[0][0], LOOK_POSTURES[0][2]
    for name, above, pose in LOOK_POSTURES:
        if grip_distance_m >= above:
            return name, pose
    return LOOK_POSTURES[-1][0], LOOK_POSTURES[-1][2]


def decode(image) -> np.ndarray:
    if isinstance(image, np.ndarray):
        return image
    data = base64.b64decode(image) if isinstance(image, str) else bytes(image)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('INVALID_JPEG')
    return frame


def lime_mask(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.asarray(LIME_LO), np.asarray(LIME_HI)) > 0


def observe_beam(image, pose: Mapping[int | str, int | float]) -> dict[str, Any]:
    """Beam axis, near end and grip point in the robot base frame (floor z = 0)."""
    frame = decode(image)
    mask = lime_mask(frame)
    origin, rays, xs, ys, valid = base_rays(pose, RAY_STEP)
    xi, yi = xs.astype(int), ys.astype(int)
    hit = valid & mask[yi, xi] & (rays[:, 2] < -1e-6)
    if hit.sum() < MIN_POINTS:
        return {'visible': False, 'reason': 'BEAM_NOT_VISIBLE', 'points': int(hit.sum())}
    s = (BEAM_TOP_Z_M - origin[2]) / rays[hit, 2]
    pts = (origin + s[:, None] * rays[hit])[:, :2]
    ok = (s > 0) & (np.linalg.norm(pts, axis=1) < 2.5)
    pts, px, py = pts[ok], xi[hit][ok], yi[hit][ok]
    if len(pts) < MIN_POINTS:
        return {'visible': False, 'reason': 'BEAM_NOT_ON_PLANE', 'points': int(len(pts))}
    mean = pts.mean(0)
    _, vecs = np.linalg.eigh(np.cov((pts - mean).T))
    u = vecs[:, -1]
    if u @ mean < 0:                    # axis points away from the robot
        u = -u
    along = (pts - mean) @ u
    across = (pts - mean) @ np.array([-u[1], u[0]])
    near = float(np.percentile(along, 1))
    far = float(np.percentile(along, 99))
    end = mean + near * u
    grip = end + GRIP_INSET_M * u
    # The near end is only trustworthy if lime pixels there are not at the image border.
    near_px = along <= near + .01
    global _INNER
    if _INNER is None:
        _INNER = _inner_valid()
    clipped = bool(np.any(~_INNER[py[near_px], px[near_px]]))
    return {'visible': True, 'reason': 'END_CLIPPED' if clipped else 'BEAM_END_VISIBLE',
            'end_visible': not clipped, 'points': int(len(pts)),
            'axis_heading_rad': float(math.atan2(u[1], u[0])),
            'near_end_base_m': [float(end[0]), float(end[1])], 'grip_base_m': [float(grip[0]), float(grip[1])],
            'visible_length_m': float(far - near), 'lateral_spread_m': float(np.std(across)),
            'provenance': 'own_rgb_lime_hue+issued_pwm_camera_fk+beam_top_plane'}


def align_errors(obs: Mapping[str, Any]) -> tuple[float, float, float]:
    gx, gy = obs['grip_base_m']
    return gx - GRASP_RADIUS_M, gy, obs['axis_heading_rad']


def _floor(value: float, tol: float) -> float:
    if abs(value) <= 1e-9:
        return 0.
    return math.copysign(max(abs(value), MIN_COMMAND), value) if abs(value) > tol * .5 else value


def align_command(obs: Mapping[str, Any]) -> dict[str, Any] | None:
    """None when aligned; otherwise one short mecanum command toward the station."""
    ex, ey, ea = align_errors(obs)
    if abs(ex) <= ALIGN_TOL_M and abs(ey) <= ALIGN_TOL_M and abs(ea) <= ALIGN_TOL_RAD:
        return None
    fwd = float(np.clip(.9 * ex, -.05, .08)) if abs(ex) > ALIGN_TOL_M else 0.
    left = float(np.clip(.9 * ey, -.06, .06)) if abs(ey) > ALIGN_TOL_M else 0.
    turn = float(np.clip(.8 * ea, -.10, .10)) if abs(ea) > ALIGN_TOL_RAD else 0.
    return {'kind': 'mecanum', 'forward': _floor(fwd, ALIGN_TOL_M), 'left': _floor(left, ALIGN_TOL_M),
            'turn': _floor(turn, ALIGN_TOL_RAD), 'duration': .3}


def held_signature(image) -> np.ndarray:
    """Lime or black-band pixels in the lower 60 % of the frame (the handle between the jaws)."""
    frame = decode(image)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lime = cv2.inRange(hsv, np.asarray(LIME_LO), np.asarray(LIME_HI)) > 0
    band = (hsv[..., 2] <= BAND_V_MAX) & (hsv[..., 1] < 90)
    h = frame.shape[0]
    sig = np.zeros(frame.shape[:2], bool)
    sig[int(.4 * h):] = (lime | band)[int(.4 * h):]
    # drop the black fisheye margin: keep only band pixels adjacent to lime
    near_lime = cv2.dilate(lime.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    return sig & (lime | near_lime)


def signature_iou(a: np.ndarray, b: np.ndarray) -> float:
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.


def signature_fraction(sig: np.ndarray) -> float:
    return float(sig.mean())
