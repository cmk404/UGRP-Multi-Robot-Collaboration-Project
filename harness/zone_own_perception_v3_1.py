"""Wrist-RGB-only zone judgments, version 3.1: no information, no answer.

v3 (``harness/zone_own_perception_v3.py``) is imported and **never modified**; its
test split was scored on v3 and stays that record. v3.1 closes the P1 defect of
the Codex review of PR #193: an information-free frame (all black, BGR 10/10/10,
any brightness 0-39) with the CARRY pulses and ``expected_kind='heavy_crate'``
made v3 answer ``yes`` / 0.70 / ``EXPECTED_KIND_SHAPE_AT_GRIP`` - v3 crops the
dark pixels to the predicted grip band before the IoU, and in CARRY the crate's
silhouette *is* the whole usable frame, so a black frame matches it with IoU 1.0 -
and the v1 tracker confirmed it after two frames.

v3.1 adds three checks and changes nothing else:

1. **Image information gate, before every judgment.** A frame must show lit
   pixels, a brightness spread, sharp edges and not be a blurred copy of a scene
   (``image_information``). Otherwise every judgment - held item, carried team
   item, handle, grasp stage (kind *and* handle) - is ``unknown`` with reason
   ``INSUFFICIENT_IMAGE_INFORMATION``. Black, near-black (V 0-39), uniform grey,
   heavily blurred and lens-covered frames all fail it.
2. **A shape answer needs the shape's boundary in the frame.** v3's shape cue is
   only kept when the pixels just outside the reported kind's at-grip silhouette
   are lit floor and the pixels just inside it are dark, over a minimum length.
   A silhouette that covers the whole frame (the carried crate in CARRY) has no
   boundary to see, so its shape is never evidence.
3. **Absence needs a lit band.** v3's "ordered kind absent from the grip band"
   is kept only when that band is mostly lit (V above v3's dark ceiling), so a
   dark cover over the grip region is not read as "nothing held".

Allowed inputs are v3's: one own wrist fisheye frame, the robot's own *issued*
arm pulses, the static catalogue/map and the scenario order sheet. ``import
mujoco`` never appears (the tests assert it).

Thresholds marked ``PREREGISTERED`` were fixed on the v3 dev frames and the
synthetic adversarial set before the new v3.1 test split was rendered
(``experiments/2026-09-26-zone-own-perception-v3-1``).
"""
from __future__ import annotations

import hashlib
from collections import OrderedDict
from collections.abc import Mapping, Sequence
from functools import lru_cache
from typing import Any

import cv2
import numpy as np

from harness import zone_own_perception as _v1
from harness import zone_own_perception_v2 as _v2
from harness import zone_own_perception_v3 as _v3

SCHEMA = 'ugrp.zone_own_perception.v3_1'
PROFILE = 'zone_own_perception_v3_1'
ANSWERS = _v3.ANSWERS
JUDGMENTS = _v3.JUDGMENTS            # same judgment names: the v3 tracker applies unchanged
PROVENANCE = _v3.PROVENANCE
TEAM_KINDS = _v3.TEAM_KINDS
SOLO_KINDS = _v3.SOLO_KINDS
GRASP_LOOK_POSTURE = _v3.GRASP_LOOK_POSTURE
UNKNOWN_REASON = 'INSUFFICIENT_IMAGE_INFORMATION'

# --- image information gate (PREREGISTERED on the v3 dev frames) ---------------
# Measured on the 204 v3 dev frames of the v3 postures that show a scene
# (CARRY, held-check with a can, grasp-look), against synthetic black /
# near-black / grey / blurred / lens-covered versions of the same frames:
#   brightness spread P95-P5 of V   dev min 34        flat frames <= 5
#   lit share (V > 40)              dev min 0.061     V 0-39 frames 0
#   edge share (Sobel >= 40)        dev min 0.005     flat / covered frames 0
#   sharpness (edge Sobel over the  dev min 1.406     Gaussian blur sigma >= 4: max 1.19
#     same after a sigma-1.5 blur)
# The 36 v3 dev frames of the held-check posture with *no* can at the grip are
# a uniform V 37 (spread 8, edges 0): exactly the near-black class, and v3
# answered "can absent" on them. v3.1 answers ``unknown`` there by design.
INFO_LIT_VALUE = _v3.DARK_MAX_VALUE          # V above this is lit (v3's dark ceiling, 40)
INFO_LIT_SHARE_MIN = .03
INFO_VALUE_RANGE_MIN = 24.
INFO_VALUE_PERCENTILES = (5., 95.)
INFO_EDGE_MAGNITUDE = 40.
INFO_EDGE_SHARE_MIN = .003
INFO_EDGE_MIN_PX = 50
INFO_SHARPNESS_MIN = 1.30
INFO_SHARPNESS_SIGMA = 1.5
INFO_INNER_ERODE_PX = 41                     # edges only away from the fisheye mask border
# --- shape boundary (PREREGISTERED) ----------------------------------------------
SHAPE_RING_PX = 9
SHAPE_RING_OUT_MIN_PX = 2000
SHAPE_RING_OUT_LIT_MIN = .80
SHAPE_RING_IN_DARK_MIN = .80
# --- absence needs a lit band (PREREGISTERED) ---------------------------------------
ABSENT_BAND_LIT_MIN = .80


@lru_cache(maxsize=4)
def _inner_region(width: int, height: int) -> np.ndarray:
    usable = _v1._usable_region(width, height)
    inner = cv2.erode(usable.astype(np.uint8), np.ones((INFO_INNER_ERODE_PX, INFO_INNER_ERODE_PX), np.uint8))
    inner = inner.astype(bool)
    inner.setflags(write=False)
    return inner


def _sobel(gray):
    return np.hypot(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))


def image_information(image) -> dict[str, Any]:
    """Does this own wrist frame carry enough information to judge anything?

    Four measures on the usable fisheye region (edges on its interior, away from
    the mask border): lit share, brightness spread, edge share and sharpness.
    ``sufficient`` is True only when all four pass; ``failed`` names the ones that
    did not. The measures read only the frame.
    """
    frame = _v1._frame(image)
    key = hashlib.blake2b(frame.tobytes(), digest_size=16).hexdigest()+str(frame.shape)
    if key in _INFO_CACHE:
        _INFO_CACHE.move_to_end(key)
        return dict(_INFO_CACHE[key])
    row = _measure_information(frame)
    _INFO_CACHE[key] = row
    while len(_INFO_CACHE) > _INFO_CACHE_SIZE:
        _INFO_CACHE.popitem(last=False)
    return dict(row)


_INFO_CACHE: OrderedDict = OrderedDict()
_INFO_CACHE_SIZE = 64


def _measure_information(frame):
    height, width = frame.shape[:2]
    usable = _v1._usable_region(width, height)
    inner = _inner_region(width, height)
    value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[..., 2][usable].astype(np.float64)
    lit_share = float((value > INFO_LIT_VALUE).mean()) if value.size else 0.
    low, high = np.percentile(value, INFO_VALUE_PERCENTILES) if value.size else (0., 0.)
    spread = float(high-low)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    magnitude = _sobel(gray)[inner]
    reblurred = _sobel(cv2.GaussianBlur(gray, (0, 0), INFO_SHARPNESS_SIGMA))[inner]
    strong = magnitude >= INFO_EDGE_MAGNITUDE
    edge_px = int(strong.sum())
    edge_share = edge_px/max(int(inner.sum()), 1)
    sharpness = (float(magnitude[strong].mean())/max(float(reblurred[strong].mean()), 1e-6)
                 if edge_px >= INFO_EDGE_MIN_PX else 0.)
    failed = []
    if lit_share < INFO_LIT_SHARE_MIN:
        failed.append('too_dark')
    if spread < INFO_VALUE_RANGE_MIN:
        failed.append('low_brightness_range')
    if edge_share < INFO_EDGE_SHARE_MIN or edge_px < INFO_EDGE_MIN_PX:
        failed.append('low_texture')
    if sharpness < INFO_SHARPNESS_MIN:
        failed.append('blurred_or_covered')
    return {'sufficient': not failed, 'failed': failed, 'lit_share': round(lit_share, 4),
            'value_range': round(spread, 1), 'edge_share': round(edge_share, 5), 'edge_px': edge_px,
            'sharpness': round(sharpness, 3)}


# --------------------------------------------------------------------------- shared

def _rebrand(row: Mapping[str, Any], info: Mapping[str, Any], **extra) -> dict[str, Any]:
    out = {**row, 'schema': SCHEMA, 'profile': PROFILE, 'v3_answer': row['answer'], 'v3_reason': row['reason'],
           'image_information': dict(info)}
    out.update(extra)
    return out


def _unknown(judgment: str, reason: str, info: Mapping[str, Any], **extra) -> dict[str, Any]:
    row = {'schema': SCHEMA, 'profile': PROFILE, 'judgment': judgment, 'answer': 'unknown', 'confidence': 0.,
           'reason': reason, 'provenance': PROVENANCE, 'observed': 'not_observed',
           'image_information': dict(info)}
    row.update(extra)
    return row


def _downgrade(row: Mapping[str, Any], reason: str, **extra) -> dict[str, Any]:
    out = {**row, 'answer': 'unknown', 'confidence': 0., 'reason': reason, 'observed': 'not_observed'}
    out.update(extra)
    return out


def _value(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[..., 2]


def shape_boundary(frame, kind: str, servo_pose, *, grasp_pose=None) -> dict[str, Any]:
    """Is the boundary of ``kind``'s at-grip silhouette visible as dark-in / lit-out?"""
    height, width = frame.shape[:2]
    usable = _v1._usable_region(width, height)
    nominal = _v3.held_silhouette(kind, _v1._pose(servo_pose), (width, height), grasp_pose=grasp_pose)[0]
    kernel = np.ones((2*SHAPE_RING_PX+1, 2*SHAPE_RING_PX+1), np.uint8)
    grown = cv2.dilate(nominal.astype(np.uint8), kernel).astype(bool)
    shrunk = cv2.erode(nominal.astype(np.uint8), kernel).astype(bool)
    ring_out = grown & ~nominal & usable
    ring_in = nominal & ~shrunk & usable
    value = _value(frame)
    out_px, in_px = int(ring_out.sum()), int(ring_in.sum())
    out_lit = float((value[ring_out] > INFO_LIT_VALUE).mean()) if out_px else 0.
    in_dark = float((value[ring_in] <= INFO_LIT_VALUE).mean()) if in_px else 0.
    ok = out_px >= SHAPE_RING_OUT_MIN_PX and out_lit >= SHAPE_RING_OUT_LIT_MIN and in_dark >= SHAPE_RING_IN_DARK_MIN
    return {'kind': kind, 'ring_out_px': out_px, 'ring_in_px': in_px, 'ring_out_lit_share': round(out_lit, 4),
            'ring_in_dark_share': round(in_dark, 4), 'evidenced': bool(ok)}


def band_lit_share(frame, kind: str, servo_pose, *, grasp_pose=None) -> float:
    height, width = frame.shape[:2]
    usable = _v1._usable_region(width, height)
    band = _v3.held_band(kind, _v1._pose(servo_pose), (width, height), grasp_pose=grasp_pose)[1] & usable
    if not band.any():
        return 0.
    return float((_value(frame)[band] > INFO_LIT_VALUE).mean())


# --------------------------------------------------------------------------- judgments

def judge_held_item(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                    candidate_kinds: Sequence[str] = SOLO_KINDS,
                    grasp_pose: Mapping[int | str, int | float] | None = None) -> dict[str, Any]:
    """v3's ``judge_held_item`` behind the information gate; absence needs a lit band."""
    if expected_kind not in _v1.APPEARANCE:
        raise ValueError(f'unknown cargo kind: {expected_kind}')
    frame = _v1._frame(image)
    _v1._pose(servo_pose)
    info = image_information(frame)
    if not info['sufficient']:
        return _unknown('held_item_at_grip', UNKNOWN_REASON, info, expected_kind=expected_kind)
    row = _rebrand(_v3.judge_held_item(frame, servo_pose, expected_kind=expected_kind,
                                       candidate_kinds=candidate_kinds, grasp_pose=grasp_pose), info)
    if row['v3_reason'] == 'EXPECTED_KIND_ABSENT_FROM_AT_GRIP_BAND':
        lit = band_lit_share(frame, expected_kind, servo_pose, grasp_pose=grasp_pose)
        row['expected_band_lit_share'] = round(lit, 4)
        if lit < ABSENT_BAND_LIT_MIN:
            return _downgrade(row, 'AT_GRIP_BAND_NOT_LIT')
    return row


def judge_team_cargo_at_grip(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                             grasp_pose: Mapping[int | str, int | float] | None = None,
                             candidate_kinds: Sequence[str] = TEAM_KINDS) -> dict[str, Any]:
    """v3's ``judge_team_cargo_at_grip`` behind the gate; a shape answer needs its boundary."""
    if expected_kind not in _v2.TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame = _v1._frame(image)
    _v1._pose(servo_pose)
    info = image_information(frame)
    if not info['sufficient']:
        return _unknown('team_cargo_at_grip', UNKNOWN_REASON, info, expected_kind=expected_kind)
    row = _rebrand(_v3.judge_team_cargo_at_grip(frame, servo_pose, expected_kind=expected_kind,
                                                grasp_pose=grasp_pose, candidate_kinds=candidate_kinds), info)
    if row['v3_reason'] in ('EXPECTED_KIND_SHAPE_AT_GRIP', 'OTHER_KIND_SHAPE_AT_GRIP'):
        boundary = shape_boundary(frame, row['observed'], servo_pose, grasp_pose=grasp_pose)
        row['shape_boundary'] = boundary
        if not boundary['evidenced']:
            return _downgrade(row, 'SHAPE_BOUNDARY_NOT_IN_FRAME')
    return row


def judge_team_cargo_handle(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                            reach_band_m: tuple[float, float] = _v2.HANDLE_REACH_BAND_M,
                            lateral_m: float = _v2.HANDLE_LATERAL_M) -> dict[str, Any]:
    """v3's handle judgment behind the gate."""
    if expected_kind not in _v2.TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame = _v1._frame(image)
    _v1._pose(servo_pose)
    info = image_information(frame)
    if not info['sufficient']:
        return _unknown('team_cargo_handle', UNKNOWN_REASON, info, expected_kind=expected_kind)
    return _rebrand(_v3.judge_team_cargo_handle(frame, servo_pose, expected_kind=expected_kind,
                                                reach_band_m=reach_band_m, lateral_m=lateral_m), info)


def judge_team_cargo_grasp_stage(image, servo_pose: Mapping[int | str, int | float], *,
                                 expected_kind: str) -> dict[str, Any]:
    """v3's grasp-stage judgment behind the gate: kind *and* handle are unknown without information."""
    if expected_kind not in _v2.TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame = _v1._frame(image)
    _v1._pose(servo_pose)
    info = image_information(frame)
    if not info['sufficient']:
        sub = {'answer': 'unknown', 'confidence': 0., 'reason': UNKNOWN_REASON, 'observed': 'not_observed'}
        return _unknown('team_cargo_grasp_stage', UNKNOWN_REASON, info, expected_kind=expected_kind,
                        identity={**sub, 'judgment': 'team_cargo_identity'},
                        handle={**sub, 'judgment': 'team_cargo_handle'})
    row = _v3.judge_team_cargo_grasp_stage(frame, servo_pose, expected_kind=expected_kind)
    handle = row['handle']
    if 'v3_answer' not in handle and handle.get('schema') == _v3.SCHEMA:
        handle = _rebrand(handle, info)
    return _rebrand(row, info, handle=handle)


__all__ = ['SCHEMA', 'PROFILE', 'ANSWERS', 'JUDGMENTS', 'PROVENANCE', 'TEAM_KINDS', 'SOLO_KINDS',
           'GRASP_LOOK_POSTURE', 'UNKNOWN_REASON', 'image_information', 'shape_boundary', 'band_lit_share',
           'judge_held_item', 'judge_team_cargo_at_grip', 'judge_team_cargo_handle',
           'judge_team_cargo_grasp_stage']
