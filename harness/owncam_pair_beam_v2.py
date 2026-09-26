"""Own-wrist-camera long_beam perception v2: the black grip band is the grip measurement.

v1 (``harness.owncam_pair_beam``, used by pair cohort 31d16b0) took the near edge
of the lime pixels as the beam end and put the grip 0.03 m inside it. Recorded
failure (cohort 31d16b0, 5/6 runs; replayed against evaluation-only GT):

* In the SEARCH posture at a true grip distance of 0.258 m the lime stops where
  the 36 mm black grip band begins; the band runs to the bottom of the valid
  fisheye view and the true end is below the frame. v1 read the band's far edge
  as the beam end -> grip 0.317 m (+59 mm) and "end visible".
* In the steeper p45 view the band is fully visible (grip read 0.252 m, -6 mm),
  but lime pixels of the vertical END FACE continue below the frame, so v1
  flagged END_CLIPPED and fell back to SEARCH -> endless SEARCH<->p45 switching.
  (The end face is lit like the top, V ~164 in both, so it is not separable by
  brightness.) The end-face hypothesis is therefore only half right: it causes
  the false clip in p45, not the large misread, which is the band in SEARCH.

v2: find the band (dark pixels in the beam corridor, projected to the top
plane) nearest to the robot with lime beyond it. If the whole band lies inside
the eroded valid view and its length is plausible (20-60 mm), the grip is the
band centre (``BAND_VISIBLE``); if any band pixel touches the invalid border,
``BAND_CLIPPED`` (the grip is nearer than this view can show). No band -> the
v1 end estimate, labelled ``NO_BAND_END_ESTIMATE``. Same inputs as v1: own JPEG
and own issued PWM only.
"""
from __future__ import annotations

import math
from typing import Any, Mapping

import cv2
import numpy as np

from harness import owncam_pair_beam as v1
from harness.owncam_view import base_rays

PROFILE = 'owncam_pair_beam_perception_v2'
BAND_V_MAX, BAND_S_MAX = 60, 90
CORRIDOR_HALF_M = .03
BAND_GAP_M = .008
BAND_LEN_M = (.020, .060)          # catalogue 36 mm
MIN_BAND_POINTS = 25
# Look postures (v1 poses). Switch-nearer thresholds on the v2 band reading: replay of 1110 recorded
# align frames (cohort 31d16b0 + dev) gave band-centre biases +20 mm (search), +9 mm (p45), +6 mm
# (inspect); inspect shows the whole band only up to a true grip of ~0.26 m, so inspect starts at a
# read 0.255 m (v1: 0.275, where 65 inspect frames lost the band).
LOOK_POSTURES = (('search', .34, v1.LOOK_POSTURES[0][2]),
                 ('p45', .255, v1.LOOK_POSTURES[1][2]),
                 ('inspect', 0., v1.LOOK_POSTURES[2][2]))


def order() -> list[str]:
    return [name for name, _, _ in LOOK_POSTURES]


def pose_of(name: str) -> dict[int, int]:
    return dict(next(pose for n, _, pose in LOOK_POSTURES if n == name))


def look_posture(grip_distance_m: float | None) -> tuple[str, dict[int, int]]:
    if grip_distance_m is None:
        return LOOK_POSTURES[0][0], pose_of(LOOK_POSTURES[0][0])
    for name, above, pose in LOOK_POSTURES:
        if grip_distance_m >= above:
            return name, dict(pose)
    return LOOK_POSTURES[-1][0], pose_of(LOOK_POSTURES[-1][0])


def _runs(values: np.ndarray, gap: float):
    order = np.argsort(values)
    v = values[order]
    cuts = np.flatnonzero(np.diff(v) > gap) + 1
    return [order[s] for s in np.split(np.arange(len(v)), cuts)]


def observe_beam(image, pose: Mapping[int | str, int | float]) -> dict[str, Any]:
    frame = v1.decode(image)
    base = v1.observe_beam(frame, pose)
    if not base.get('visible'):
        return {**base, 'grip_source': None, 'perception': PROFILE}
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lime = v1.lime_mask(frame)
    dark = (hsv[..., 2] <= BAND_V_MAX) & (hsv[..., 1] < BAND_S_MAX)
    origin, rays, xs, ys, valid = base_rays(pose, v1.RAY_STEP)
    xi, yi = xs.astype(int), ys.astype(int)
    down = rays[:, 2] < -1e-6
    s = np.where(down, (v1.BEAM_TOP_Z_M - origin[2]) / np.where(down, rays[:, 2], -1.), np.nan)
    pts = origin[:2] + s[:, None] * rays[:, :2]
    heading = base['axis_heading_rad']
    u = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-u[1], u[0]])
    grip_v1 = np.asarray(base['grip_base_m'])
    anchor = grip_v1 - v1.GRIP_INSET_M * u                      # v1 near end: a point on the beam axis
    along = (pts - anchor) @ u
    across = (pts - anchor) @ n
    corridor = down & np.isfinite(along) & (np.abs(across) <= CORRIDOR_HALF_M)
    if v1._INNER is None:
        v1._INNER = v1._inner_valid()
    inner = v1._INNER[yi, xi]
    band_px = corridor & dark[yi, xi] & valid
    lime_px = corridor & lime[yi, xi] & valid
    result = {**base, 'perception': PROFILE}
    if band_px.sum() >= MIN_BAND_POINTS:
        idx = np.flatnonzero(band_px)
        lime_along = along[lime_px]
        for run in _runs(along[idx], BAND_GAP_M):
            members = idx[run]
            if len(members) < MIN_BAND_POINTS:
                continue
            lo, hi = float(np.percentile(along[members], 2)), float(np.percentile(along[members], 98))
            if not np.any((lime_along > hi) & (lime_along < hi + .03)):
                continue                                         # the grip band has lime beyond it
            clipped = bool(np.any(~inner[members]))
            length = hi - lo
            band = {'near_m': round(lo, 4), 'far_m': round(hi, 4), 'length_m': round(length, 4),
                    'points': int(len(members)), 'touches_invalid_border': clipped}
            if clipped:
                return {**result, 'reason': 'BAND_CLIPPED', 'end_visible': False, 'band': band,
                        'grip_source': 'band_clipped'}
            if not BAND_LEN_M[0] <= length <= BAND_LEN_M[1]:
                band['rejected'] = 'implausible_length'
                result['band'] = band
                break
            centre = anchor + (lo + hi) / 2 * u + float(np.median(across[members])) * n
            end = centre - v1.GRIP_INSET_M * u
            return {**result, 'reason': 'BAND_VISIBLE', 'end_visible': True, 'band': band,
                    'grip_base_m': [float(centre[0]), float(centre[1])],
                    'near_end_base_m': [float(end[0]), float(end[1])], 'grip_source': 'band_centre',
                    'provenance': base['provenance'] + '+black_grip_band_centre'}
    reason = 'NO_BAND_END_ESTIMATE' if base['end_visible'] else base['reason']
    return {**result, 'reason': reason, 'grip_source': 'end_plus_inset_v1'}


# ---------------- grip / hold views (v2) ----------------
# dev 613 (v2 run 95923c1): at grasp range the beam top renders yellow (hue ~30, outside the v1 lime
# band 36-54), so the v1 lime-based grip signature was 0.0047 while GT fingers held 5.9 N. Grip views
# recorded at the band centre: dark band 0.60-0.96 of the valid view, beam colour in the bottom third
# 0.067-0.144 (13 grip views, dev + cohort 31d16b0).
BEAM_HUE = (25, 54)
BEAM_S_MIN, BEAM_V_MIN = 100, 120
GRIP_MIN_DARK = .40
GRIP_MIN_BOTTOM_BEAM = .04
_VALID = None


def _valid():
    global _VALID
    if _VALID is None:
        from harness.owncam_view import valid_pixel_mask
        _VALID = valid_pixel_mask(1).astype(bool)
    return _VALID


def beam_colour_mask(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return ((hsv[..., 0] >= BEAM_HUE[0]) & (hsv[..., 0] <= BEAM_HUE[1]) & (hsv[..., 1] >= BEAM_S_MIN)
            & (hsv[..., 2] >= BEAM_V_MIN))


def grip_view(image) -> dict[str, Any]:
    """Grasp-height own view: the band fills the view and beam colour is between the jaws."""
    frame = v1.decode(image)
    valid = _valid()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dark = (hsv[..., 2] <= BAND_V_MAX) & (hsv[..., 1] < BAND_S_MAX) & valid
    beam = beam_colour_mask(frame) & valid
    h = frame.shape[0]
    dark_frac = float(dark.sum() / valid.sum())
    bottom = float(beam[2 * h // 3:].sum() / max(valid[2 * h // 3:].sum(), 1))
    ok = dark_frac >= GRIP_MIN_DARK and bottom >= GRIP_MIN_BOTTOM_BEAM
    return {'seen': ok, 'dark_fraction': round(dark_frac, 4), 'bottom_beam_fraction': round(bottom, 4),
            'reason': 'BAND_BETWEEN_JAWS' if ok else 'GRIP_VIEW_NOT_BAND'}


# Grasp -> lift co-motion uses the widened beam hue: v1 lime IoU was 0.077 in dev 613 (a892890; GT
# lifted, 5.7 N) because the grasp-range view is yellow; widened IoU 0.69-0.90 on all 11 recorded real
# lifts. The CARRY hold check keeps v1's lime signature anchored on the lift view: replaying the
# widened signature on recorded carries gave ratios down to 0.41 without slip, v1's stayed >= 0.99.


def co_motion_signature(image) -> np.ndarray:
    frame = v1.decode(image)
    beam = beam_colour_mask(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    band = (hsv[..., 2] <= BAND_V_MAX) & (hsv[..., 1] < BAND_S_MAX)
    near = cv2.dilate(beam.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    h = frame.shape[0]
    sig = np.zeros(frame.shape[:2], bool)
    sig[int(.4 * h):] = ((beam | band) & (beam | near))[int(.4 * h):]
    return sig
