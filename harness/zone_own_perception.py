"""Wrist-RGB-only recognition judgments for the Korean-dialogue zone study.

2026-09-26 study decisions: the TOP cameras are evaluation-only, so every
robot-facing recognition must come from that robot's own wrist fisheye camera.
This module answers the four judgments the study needs beyond the cyan box
grasp/place skill (``harness/wrist_zone_skill_v*.py``, owned elsewhere and
imported nowhere here):

1. ``judge_slot_item``     which cargo kind sits at a pickup-bay slot from the
                           order sheet - or ``empty``, or ``not_observed``.
2. ``judge_holding_item``  after a grasp, in the agreed CARRY posture, whether
                           the held thing is the ordered item.
3. ``judge_placed_in_slot`` look-back after release: is the item in the zone
                           slot? Answers the recorded 518 false negative (a cyan
                           box lit on zone B blue paint renders dark and
                           desaturated, so fixed HSV gates miss it).
4. ``judge_route_blockage`` an obstruction ahead that the static map does not
                           declare (an object left in a corridor or door).

Allowed inputs only
    * one own ``robot_cam`` JPEG/array in raw fisheye geometry,
    * the robot's own *issued* arm pulses (servos 3-6; never measured joints),
    * the versioned static map dict (walls, passages, zones, zone slots),
    * the scenario order sheet (kind, destination zone, coarse initial slot),
    * an injected own-pose belief for map-relative questions. The belief is the
      caller's estimate (own-camera localization package); this module never
      improves it and records which belief it used.

Never read here: TOP frames or anything derived from them, simulator poses,
segmentation, contacts, measured joints, teacher phases or outcomes, other
robots' frames or command logs. ``import mujoco`` never appears, directly or
transitively (``tests/test_zone_own_perception.py`` asserts this).

Every judgment returns ``yes`` / ``no`` / ``unknown`` with a confidence.
``unknown`` is a first-class answer: no gate ever converts "cannot see it" into
``no``. A ``no`` always names the positive evidence it stands on (a different
kind seen, bare floor/paint proven, the lane proven clear).

Illumination policy (the 518 lesson): colour gates are *relative* to the frame
and to the local floor/paint around the target, and the geometric gates carry
the discrimination. An absolute saturation/value floor is only used to reject
near-black pixels.

Thresholds carry a ``PREREGISTERED`` marker and were fixed on the dev split
before the held-out split was scored
(``experiments/2026-09-26-zone-own-perception``).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from functools import lru_cache
from typing import Any

import cv2
import numpy as np

from harness import markerless_box as _mb
from harness import zone_color_boxes as _zcb

SCHEMA = 'ugrp.zone_own_perception.v1'
ANSWERS = ('yes', 'no', 'unknown')
JUDGMENTS = ('slot_item', 'holding_item', 'placed_in_slot', 'route_blockage')
PROVENANCE = ('own wrist RGB (raw fisheye) + own issued arm pulses + static map + scenario order sheet'
              ' + injected own-pose belief; no TOP, no simulator state, no teacher result')

# Agreed observation postures (issued servo pulses), 2026-09-26 package brief.
CARRY_POSTURE = {3: 777, 4: 2053, 5: 1646}
LOOK_P20_POSTURE = {3: 1072, 4: 2400, 5: 1482}
PAN_PULSES = (1230, 1500, 1770)
POSTURE_TOLERANCE_PWM = 90       # PREREGISTERED: issued pulses this close count as the posture

# Solo cargo appearance from own RGB. ``hue`` is the OpenCV H (0..179) band of
# the unlit paint; the bands are wide because the zone ceiling light both
# darkens and desaturates a face. ``footprint_m`` is the catalogue silhouette
# (sim/zone_cargo.py, sim/zone_arena.py) used for the apparent-size gate.
BOX_KINDS = _zcb.KINDS                     # cyan, green, red, yellow
CARGO_KINDS = ('can', 'tile')
TEAM_KINDS = ('long_beam', 'heavy_crate', 'tri_frame')   # stretch goal, reported separately
KNOWN_KINDS = BOX_KINDS + CARGO_KINDS + TEAM_KINDS
APPEARANCE: dict[str, dict[str, Any]] = {
    'cyan':        {'hue': ((84, 100),), 'footprint_m': (.034, .040), 'height_m': .032, 'shape': 'cuboid'},
    'green':       {'hue': ((54, 72),), 'footprint_m': (.034, .040), 'height_m': .032, 'shape': 'cuboid'},
    'red':         {'hue': ((0, 8), (170, 179)), 'footprint_m': (.034, .040), 'height_m': .032, 'shape': 'cuboid'},
    'yellow':      {'hue': ((21, 34),), 'footprint_m': (.034, .040), 'height_m': .032, 'shape': 'cuboid'},
    'can':         {'hue': ((124, 139),), 'footprint_m': (.038, .038), 'height_m': .050, 'shape': 'cylinder'},
    'tile':        {'hue': ((150, 170),), 'footprint_m': (.060, .040), 'height_m': .012, 'shape': 'slab'},
    'long_beam':   {'hue': ((36, 54),), 'footprint_m': (.600, .040), 'height_m': .032, 'shape': 'bar'},
    'heavy_crate': {'hue': ((160, 176),), 'footprint_m': (.240, .100), 'height_m': .060, 'shape': 'cuboid'},
    'tri_frame':   {'hue': ((14, 26),), 'footprint_m': (.400, .400), 'height_m': .024, 'shape': 'frame'},
}
# Zone floor paint hues, converted from ``regions.zone_*.rgba`` in
# maps/zones/*.json: A orange (H 12), B blue (H 112), C violet (H 143). The
# bands below are those centres widened for the ceiling light. A kind whose band
# overlaps the paint of the zone it stands on can only be *confirmed* by the
# local contrast test and can never be proven absent there.
PAINT_HUE = {'A': (8, 18), 'B': (104, 120), 'C': (138, 150)}

# --- colour gates (PREREGISTERED) -------------------------------------------
ABS_MIN_VALUE = 28          # near-black pixels carry no hue
ABS_MIN_SAT = 45            # unpainted floor/wall grey stays below this
REL_VALUE_FLOOR = .34       # ... and at least this share of the frame's 75th-percentile V
REL_SAT_FLOOR = .40         # ... and this share of the frame's 75th-percentile S
FRAME_PERCENTILE = 75

# --- slot observation (PREREGISTERED) --------------------------------------
SLOT_MAX_RANGE_M = 1.20     # the production near-range floor fit refuses farther contacts
SLOT_MATCH_M = .10          # detection accepted as "at the slot" within this of the slot centre
SLOT_INNER_M = .030         # slot centre disc (box half-diagonal is .026)
SLOT_OUTER_M = .075         # floor/paint reference ring around the slot
FRAME_MARGIN_PX = 6
MIN_RING_PIXELS = 120
MIN_CENTRE_PIXELS = 40
BARE_LIKE_DIST = 26         # max-channel distance to the ring median counted as "same surface"
BARE_SHARE_MIN = .82        # share of centre pixels like the ring -> the slot is bare
PAINT_BARE_SHARE_MIN = .88  # ... stricter on zone paint (dev: bare 0.97 against 0.72-0.76 occupied)
OCCLUSION_SHARE_MAX = .55   # below this, something is there but unidentified -> unknown
RING_FLOOR_TOLERANCE = 34   # the ring must match the floor of its own image row, or the slot is covered

# --- placement look-back (PREREGISTERED) -----------------------------------
# Dev measurement: bare zone paint reaches a max-channel delta of 37 across the
# slot disc from its own lighting gradient, while an item on the paint reaches
# 63-84. The contrast gate therefore sits above the paint gradient, well clear of
# ``BARE_LIKE_DIST``; otherwise the same pixels counted as "bare" and "differing".
CONTRAST_SHARE_MIN = .25    # share of centre pixels differing from the paint ring
CONTRAST_HUE_TOL_DEG = 16   # ... whose hue is within this of the kind's band (OpenCV H*2)
CONTRAST_MIN_DELTA = 38     # ... with this max-channel BGR difference from the ring median
CONTRAST_MAX_DELTA_MIN = 55  # ... and the disc must reach this delta somewhere
PAINT_RING_HUE_TOL = 12     # the reference ring must be the zone's own paint (OpenCV H)

# --- holding (PREREGISTERED) -----------------------------------------------
CARRY_ROI = (.18, .10, .82, .92)        # fractional x0, y0, x1, y1 of the held-item region
HOLD_MIN_COVERAGE = {'cuboid': .30, 'cylinder': .14, 'slab': .10, 'bar': .20, 'frame': .12}
HOLD_MARGIN = 1.6           # best kind coverage over the runner-up
HOLD_EMPTY_MAX = .05        # no kind above this and the ROI is one surface -> nothing held
HOLD_EDGE_MIN = .006        # cargo colour outside the ROI -> something is held but not framed
# Dev finding (experiments/2026-09-26-zone-own-perception): in the agreed CARRY
# posture a held box or tile fills the held-item region, but a held 50 mm can
# shows only its top rim, entirely outside that region (3,017 px at the frame
# edge, 0 px inside). Absence can therefore only be claimed for kinds the posture
# actually frames; for the others this judgment stays ``unknown`` and the study
# needs another look posture before a robot can verify a held can.
CARRY_FRAMED_KINDS = BOX_KINDS + ('tile',)

# --- route blockage (PREREGISTERED) ----------------------------------------
LANE_HALF_WIDTH_M = .22     # forward lane half width when no passage is named
BLOCK_MAX_RANGE_M = 2.00
CLEAR_MIN_RANGE_M = .90     # floor must be proven this far ahead to answer "not blocked"
LANE_SWEEP_START_M = .45    # the wrist floor view starts here in both agreed postures
MAP_MATCH_M = .18           # obstruction within this of a mapped wall footprint is expected
MIN_BLOCK_AREA_PX = 260
# The arena floor is lit from the ceiling, so its colour changes strongly with
# distance (dev frames: BGR 72,46,29 at 0.45 m against 29,26,24 at 3.5 m). A
# single near-field reference flagged about 28 % of a clear frame. The floor is
# therefore modelled per image row band over the rows that project onto the
# floor in range, which is also what makes the height gate below meaningful.
FLOOR_ROW_BAND_PX = 8
FLOOR_MODEL_RANGE_M = (.30, 3.00)
BLOCK_LIKE_DIST = 34        # max-channel distance from the row floor model counted as floor
MIN_OBSTACLE_HEIGHT_M = .06  # ... and it must stand at least this tall above its floor contact
HEIGHT_GATE_FRACTION = .55   # measured pixel height over the height that much would project to
# The raw fisheye remap leaves black wedges at the frame edges, and the robot's
# own fingers sit at the bottom. On dev these formed one 8,600 px component whose
# lowest row projected to 0.42 m and passed the height gate, so every clear view
# was reported blocked. Components that touch the invalid region or the frame
# border are therefore not obstruction candidates (the same border policy as
# ``harness.markerless_box``).
INVALID_DILATE_PX = 7
BORDER_REJECT_PX = 3
POSE_BELIEF_SLACK_M = {'high': .06, 'medium': .12, 'low': .25}

CONFIDENCE = {
    'slot_kind_matched': .90,
    'slot_kind_conflict': .85,
    'slot_bare': .80,
    'slot_unknown': .0,
    'hold_kind_matched': .88,
    'hold_kind_conflict': .82,
    'hold_empty': .75,
    'hold_unknown': .0,
    'placed_detector_and_contrast': .95,
    'placed_detector_only': .85,
    'placed_contrast_only': .70,
    'placed_bare_paint': .80,
    'placed_unknown': .0,
    'block_unmapped': .80,
    'block_unmapped_weak': .60,
    'lane_clear': .75,
    'block_unknown': .0,
}


# --------------------------------------------------------------------------- frames

def _frame(image) -> np.ndarray:
    return _zcb._frame(image)


def _pose(servo_pose: Mapping[int | str, int | float]) -> dict[int, int]:
    out = {}
    for key, value in servo_pose.items():
        out[int(key)] = int(value)
    for servo in (3, 4, 5, 6):
        if servo not in out:
            raise ValueError(f'issued arm pulses must include servo {servo}')
    return out


def _optics(frame: np.ndarray, pose: Mapping[int, int]):
    height, width = frame.shape[:2]
    origin, axes = _mb.camera_extrinsics(pose)
    origin = np.asarray(origin, np.float64)
    axes = np.asarray(axes, np.float64)
    k = _mb.scaled_camera_matrix(width, height)
    d = np.asarray(_mb.CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    return origin, axes, k, d, (width, height)


def _project(points_base_xy, z, origin, axes, k, d):
    flat = np.asarray(points_base_xy, np.float64).reshape(-1, 2)
    points = np.column_stack((flat, np.full(len(flat), float(z))))
    return _mb._project_points(points, origin, axes, k, d)


def _ground(pixel, origin, axes, k, d):
    return _mb._pixel_ground_point(pixel, origin, axes, k, d)


def _inside(pixel, size, margin=FRAME_MARGIN_PX):
    return margin <= pixel[0] <= size[0]-1-margin and margin <= pixel[1] <= size[1]-1-margin


# --------------------------------------------------------------------------- colour

def _frame_reference(hsv: np.ndarray) -> tuple[float, float]:
    """Frame-relative saturation/value scale: the 75th percentile of each channel."""
    return (float(np.percentile(hsv[..., 1], FRAME_PERCENTILE)),
            float(np.percentile(hsv[..., 2], FRAME_PERCENTILE)))


def hue_mask(hsv: np.ndarray, kind: str, *, reference: tuple[float, float] | None = None) -> np.ndarray:
    """Illumination-relative mask for one cargo kind.

    The hue band is fixed; the saturation/value floors are the larger of a small
    absolute floor and a share of this frame's own 75th percentile. A cyan box
    lit on zone B paint (recorded failure 518) keeps its hue but loses about
    half of its saturation and value, which the relative floors accept and a
    fixed absolute gate rejects.
    """
    if kind not in APPEARANCE:
        raise ValueError(f'unknown cargo kind: {kind}')
    sat_ref, val_ref = reference if reference is not None else _frame_reference(hsv)
    min_sat = max(ABS_MIN_SAT, REL_SAT_FLOOR*sat_ref)
    min_val = max(ABS_MIN_VALUE, REL_VALUE_FLOOR*val_ref)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in APPEARANCE[kind]['hue']:
        mask |= cv2.inRange(hsv, np.asarray((low, min_sat, min_val), np.uint8),
                            np.asarray((high, 255, 255), np.uint8))
    return mask


def _components(mask: np.ndarray, min_area: float):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        out.append({'contour': contour, 'area': area, 'bbox': tuple(int(v) for v in cv2.boundingRect(contour))})
    return out, mask


# --------------------------------------------------------------------------- own-RGB cargo detection

def detect_cargo_own(image, servo_pose: Mapping[int | str, int | float],
                     kinds: Sequence[str] = CARGO_KINDS) -> dict[str, Any]:
    """Catalogue cargo (can, tile, ... ) in one own-RGB frame, floor hypothesis.

    ``harness.zone_color_boxes.detect_own`` covers the four painted boxes with
    the marker-free cuboid fit; it is imported, not modified. This adds the
    non-box catalogue silhouettes, which are not cuboids: the lowest contour
    band is projected to the floor and the apparent size is checked against the
    catalogue footprint at that range. Positions are in the robot base frame
    and are conditional on the floor hypothesis.
    """
    frame = _frame(image)
    pose = _pose(servo_pose)
    origin, axes, k, d, size = _optics(frame, pose)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    reference = _frame_reference(hsv)
    detections = []
    for kind in kinds:
        if kind not in APPEARANCE:
            raise ValueError(f'unknown cargo kind: {kind}')
        comps, _ = _components(hue_mask(hsv, kind, reference=reference), 70.)
        for comp in comps:
            x, y, w, h = comp['bbox']
            if x <= 2 or y <= 2 or x+w >= size[0]-2 or y+h >= size[1]-2:
                continue
            points = comp['contour'].reshape(-1, 2)
            band = points[points[:, 1] >= float(np.max(points[:, 1]))-max(2., h*.12)]
            contacts = [_ground(p, origin, axes, k, d) for p in band]
            contacts = np.asarray([c for c in contacts if c is not None])
            if not len(contacts):
                continue
            contact = np.median(contacts[:, :2], axis=0)
            rng = float(np.linalg.norm(contact-origin[:2]))
            if not .05 < rng <= SLOT_MAX_RANGE_M*2:
                continue
            spec = APPEARANCE[kind]
            expected = _expected_pixel_extent(contact, spec, origin, axes, k, d)
            if expected is None:
                continue
            ratio = max(w, h)/max(expected, 1e-6)
            if not .45 <= ratio <= 2.4:
                continue
            m = cv2.moments(comp['contour'])
            detections.append({
                'kind': kind, 'range_class': 'near' if rng <= SLOT_MAX_RANGE_M else 'far_coarse',
                'pixel_centroid': [m['m10']/m['m00'], m['m01']/m['m00']],
                'pixel_bbox': [int(v) for v in comp['bbox']], 'area_px': comp['area'],
                'estimated_base_m': [float(contact[0]), float(contact[1])],
                'estimated_range_m': rng, 'apparent_size_ratio': float(ratio)})
    return {'detections': detections, 'image_size_px': list(size), 'profile': 'own_cargo_v1',
            'provenance': PROVENANCE,
            'identity_source': 'colour band + catalogue silhouette size; individual item identity is not decoded'}


def _expected_pixel_extent(contact_xy, spec, origin, axes, k, d):
    """Longest expected pixel extent of a kind standing at ``contact_xy``."""
    a, b = spec['footprint_m']
    z = spec['height_m']
    corners = []
    for sx in (-1., 1.):
        for sy in (-1., 1.):
            for sz in (0., z):
                corners.append((contact_xy[0]+sx*a/2, contact_xy[1]+sy*b/2, sz))
    pixels = _mb._project_points(np.asarray(corners, np.float64), origin, axes, k, d)
    if pixels is None:
        return None
    return float(max(np.ptp(pixels[:, 0]), np.ptp(pixels[:, 1])))


def _all_detections(frame, pose, kinds):
    """Box-colour detections (zone_color_boxes) plus catalogue detections."""
    rows = []
    box_kinds = [k for k in kinds if k in BOX_KINDS]
    other = [k for k in kinds if k not in BOX_KINDS]
    if box_kinds:
        found = _zcb.detect_own(frame, pose, box_kinds, profile=_zcb.OWN_PROFILE_ZONE)
        for det in found['detections']:
            rows.append({'kind': det['kind'], 'source': 'zone_color_boxes.detect_own',
                         'estimated_base_m': det['estimated_box_center_base_m'][:2],
                         'pixel_centroid': det['pixel_centroid'], 'area_px': det['area_px'],
                         'range_class': det['range_class'],
                         'floor_hypothesis_projection_iou': det.get('floor_hypothesis_projection_iou')})
    if other:
        for det in detect_cargo_own(frame, pose, other)['detections']:
            rows.append({'kind': det['kind'], 'source': 'zone_own_perception.detect_cargo_own',
                         'estimated_base_m': det['estimated_base_m'],
                         'pixel_centroid': det['pixel_centroid'], 'area_px': det['area_px'],
                         'range_class': det['range_class'],
                         'apparent_size_ratio': det['apparent_size_ratio']})
    return rows


# --------------------------------------------------------------------------- ring / centre statistics

def _disc_stats(frame, centre_base_xy, inner_m, outer_m, origin, axes, k, d, size):
    """Median colour of the ring around a floor point and the share of the inner
    disc that matches it. Self-referential: no reference frame, no TOP."""
    centre_px = _project([centre_base_xy], 0., origin, axes, k, d)
    if centre_px is None or not _inside(centre_px[0], size):
        return None
    cu, cv_ = float(centre_px[0][0]), float(centre_px[0][1])
    radii = {}
    for name, metres in (('inner', inner_m), ('outer', outer_m)):
        ring = [(centre_base_xy[0]+metres*math.cos(t), centre_base_xy[1]+metres*math.sin(t))
                for t in np.linspace(0., 2*math.pi, 16, endpoint=False)]
        pixels = _project(ring, 0., origin, axes, k, d)
        if pixels is None:
            return None
        radii[name] = float(np.median(np.hypot(pixels[:, 0]-cu, pixels[:, 1]-cv_)))
    if radii['inner'] < 3. or radii['outer'] <= radii['inner']+2.:
        return None
    r_out = radii['outer']
    x0, x1 = int(cu-r_out)-1, int(cu+r_out)+2
    y0, y1 = int(cv_-r_out)-1, int(cv_+r_out)+2
    if x0 < 0 or y0 < 0 or x1 > size[0] or y1 > size[1]:
        return None
    patch = frame[y0:y1, x0:x1].astype(np.int16)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xx-cu, yy-cv_)
    ring_mask = (dist >= radii['inner']*1.25) & (dist <= r_out)
    centre_mask = dist <= radii['inner']
    if int(ring_mask.sum()) < MIN_RING_PIXELS or int(centre_mask.sum()) < MIN_CENTRE_PIXELS:
        return None
    ring_median = np.median(patch[ring_mask], axis=0)
    centre = patch[centre_mask]
    delta = np.max(np.abs(centre-ring_median), axis=1)
    hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    return {'centre_px': [cu, cv_], 'inner_px': radii['inner'], 'outer_px': r_out,
            'ring_median_bgr': [float(v) for v in ring_median],
            'ring_pixels': int(ring_mask.sum()), 'centre_pixels': int(centre_mask.sum()),
            'bare_share': float(np.mean(delta <= BARE_LIKE_DIST)),
            'differing_share': float(np.mean(delta > CONTRAST_MIN_DELTA)),
            'max_delta': float(delta.max()), 'centre_hsv': hsv[centre_mask],
            'centre_delta': delta}


def _answer(judgment, answer, confidence, reason, **extra):
    if answer not in ANSWERS:
        raise ValueError(f'answer must be one of {ANSWERS}')
    row = {'schema': SCHEMA, 'judgment': judgment, 'answer': answer,
           'confidence': round(float(confidence), 3), 'reason': reason,
           'provenance': PROVENANCE}
    row.update(extra)
    return row


# --------------------------------------------------------------------------- 1. item at a pickup slot

def judge_slot_item(image, servo_pose: Mapping[int | str, int | float], *,
                    slot_base_xy: Sequence[float], expected_kind: str,
                    slot_id: str | None = None,
                    candidate_kinds: Sequence[str] = BOX_KINDS+CARGO_KINDS) -> dict[str, Any]:
    """Is the order sheet's item at this pickup-bay slot?

    ``slot_base_xy`` is the slot centre in the robot's own base frame (+x
    forward, +y left); the caller derives it from the static map slot and its
    own pose belief. ``observed`` reports what was seen: a cargo kind,
    ``empty`` (bare floor proven at the slot) or ``not_observed``.

    ``no`` needs positive evidence - a different kind detected at the slot, or
    the slot surface proven bare. A slot that is out of view, out of range,
    occluded or too dark stays ``unknown``.
    """
    if expected_kind not in APPEARANCE:
        raise ValueError(f'unknown cargo kind: {expected_kind}')
    frame = _frame(image)
    pose = _pose(servo_pose)
    origin, axes, k, d, size = _optics(frame, pose)
    slot = (float(slot_base_xy[0]), float(slot_base_xy[1]))
    rng = math.hypot(slot[0]-origin[0], slot[1]-origin[1])
    common = {'slot_id': slot_id, 'expected_kind': expected_kind,
              'slot_base_m': list(slot), 'slot_range_m': round(rng, 4)}
    pixel = _project([slot], 0., origin, axes, k, d)
    if pixel is None or not _inside(pixel[0], size):
        return _answer('slot_item', 'unknown', CONFIDENCE['slot_unknown'], 'SLOT_NOT_IN_VIEW',
                       observed='not_observed', **common)
    if rng > SLOT_MAX_RANGE_M:
        return _answer('slot_item', 'unknown', CONFIDENCE['slot_unknown'], 'SLOT_BEYOND_FIT_RANGE',
                       observed='not_observed', **common)
    kinds = [kind for kind in candidate_kinds if kind in APPEARANCE]
    if expected_kind not in kinds:
        kinds.append(expected_kind)
    matches = [det for det in _all_detections(frame, pose, kinds)
               if math.dist(det['estimated_base_m'], slot) <= SLOT_MATCH_M]
    matches.sort(key=lambda det: (-det['area_px'],))
    common['detections_at_slot'] = [{'kind': det['kind'], 'source': det['source'],
                                     'estimated_base_m': [round(v, 4) for v in det['estimated_base_m']],
                                     'area_px': det['area_px']} for det in matches]
    kinds_seen = {det['kind'] for det in matches}
    if expected_kind in kinds_seen:
        other = sorted(kinds_seen-{expected_kind})
        return _answer('slot_item', 'yes', CONFIDENCE['slot_kind_matched'],
                       'EXPECTED_KIND_AT_SLOT' if not other else 'EXPECTED_KIND_AT_SLOT_WITH_NEIGHBOURS',
                       observed=expected_kind, other_kinds=other, **common)
    if matches:
        return _answer('slot_item', 'no', CONFIDENCE['slot_kind_conflict'], 'OTHER_KIND_AT_SLOT',
                       observed=matches[0]['kind'], other_kinds=sorted(kinds_seen), **common)
    stats = _disc_stats(frame, slot, SLOT_INNER_M, SLOT_OUTER_M, origin, axes, k, d, size)
    if stats is None:
        return _answer('slot_item', 'unknown', CONFIDENCE['slot_unknown'], 'SLOT_SURFACE_UNMEASURABLE',
                       observed='not_observed', **common)
    common['bare_share'] = round(stats['bare_share'], 4)
    model, valid = _floor_row_model(frame, origin, axes, k, d, size)
    row = int(round(stats['centre_px'][1]))
    ring_matches_floor = bool(0 <= row < len(valid) and valid[row]
                              and np.max(np.abs(np.asarray(stats['ring_median_bgr'])
                                                - model[row, 0])) <= RING_FLOOR_TOLERANCE)
    common['reference_ring_matches_row_floor'] = ring_matches_floor
    if not ring_matches_floor:
        # Something (a peer chassis, a shadowed body) covers the slot surround:
        # an unbroken surface there is not evidence of an empty slot.
        return _answer('slot_item', 'unknown', CONFIDENCE['slot_unknown'], 'SLOT_SURROUND_NOT_FLOOR',
                       observed='not_observed', **common)
    if stats['bare_share'] >= BARE_SHARE_MIN:
        return _answer('slot_item', 'no', CONFIDENCE['slot_bare'], 'SLOT_SURFACE_PROVEN_BARE',
                       observed='empty', **common)
    return _answer('slot_item', 'unknown', CONFIDENCE['slot_unknown'],
                   'SLOT_OCCUPIED_BY_UNIDENTIFIED_SURFACE' if stats['bare_share'] < OCCLUSION_SHARE_MAX
                   else 'SLOT_SURFACE_AMBIGUOUS', observed='not_observed', **common)


# --------------------------------------------------------------------------- 2. holding the right item

def judge_holding_item(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                       candidate_kinds: Sequence[str] = BOX_KINDS+CARGO_KINDS,
                       posture: Mapping[int, int] = CARRY_POSTURE) -> dict[str, Any]:
    """In the CARRY posture, is the held thing the ordered item?

    The held item fills most of the wrist frame (about 63 % for a box), so this
    uses what is visible rather than a floor projection: per-kind coverage of
    the held-item region, illumination-relative, with a margin over the
    runner-up. The gripper command is never evidence of a grasp; only the issued
    arm pulses are used, to check the posture the frame was taken in.
    """
    if expected_kind not in APPEARANCE:
        raise ValueError(f'unknown cargo kind: {expected_kind}')
    frame = _frame(image)
    pose = _pose(servo_pose)
    off = {servo: abs(pose[servo]-value) for servo, value in posture.items() if servo in pose}
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = (int(round(CARRY_ROI[0]*width)), int(round(CARRY_ROI[1]*height)),
                      int(round(CARRY_ROI[2]*width)), int(round(CARRY_ROI[3]*height)))
    roi = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    reference = _frame_reference(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    kinds = [kind for kind in candidate_kinds if kind in APPEARANCE]
    if expected_kind not in kinds:
        kinds.append(expected_kind)
    coverage = {}
    for kind in kinds:
        mask = hue_mask(hsv, kind, reference=reference)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        coverage[kind] = float(mask.astype(bool).mean())
    ranked = sorted(coverage.items(), key=lambda item: -item[1])
    best, best_cover = ranked[0]
    runner = ranked[1][1] if len(ranked) > 1 else 0.
    common = {'expected_kind': expected_kind, 'coverage': {k: round(v, 4) for k, v in coverage.items()},
              'roi_fraction': list(CARRY_ROI), 'posture_offset_pwm': off,
              'posture_ok': all(v <= POSTURE_TOLERANCE_PWM for v in off.values())}
    if not common['posture_ok']:
        return _answer('holding_item', 'unknown', CONFIDENCE['hold_unknown'], 'NOT_IN_CARRY_POSTURE',
                       observed='not_observed', **common)
    required = HOLD_MIN_COVERAGE[APPEARANCE[best]['shape']]
    decisive = best_cover >= required and best_cover >= HOLD_MARGIN*max(runner, 1e-6)
    if decisive and best == expected_kind:
        return _answer('holding_item', 'yes', CONFIDENCE['hold_kind_matched'], 'EXPECTED_KIND_FILLS_VIEW',
                       observed=expected_kind, **common)
    if decisive:
        return _answer('holding_item', 'no', CONFIDENCE['hold_kind_conflict'], 'OTHER_KIND_FILLS_VIEW',
                       observed=best, **common)
    if expected_kind not in CARRY_FRAMED_KINDS:
        return _answer('holding_item', 'unknown', CONFIDENCE['hold_unknown'],
                       'KIND_NOT_FRAMED_BY_CARRY_POSTURE', observed='not_observed', **common)
    if best_cover <= HOLD_EMPTY_MAX:
        # A sliver of cargo colour at the frame edge means "held but not framed",
        # never "nothing held".
        edge = _edge_coverage(frame, (x0, y0, x1, y1), kinds, reference)
        common['edge_coverage'] = {k: round(v, 4) for k, v in edge.items()}
        if max(edge.values(), default=0.) >= HOLD_EDGE_MIN:
            return _answer('holding_item', 'unknown', CONFIDENCE['hold_unknown'],
                           'CARGO_COLOUR_ONLY_AT_FRAME_EDGE', observed='not_observed', **common)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        uniform = float(np.std(gray.astype(np.float32))) <= 34.
        if uniform:
            return _answer('holding_item', 'no', CONFIDENCE['hold_empty'], 'NO_CARGO_COLOUR_IN_VIEW',
                           observed='empty', **common)
        return _answer('holding_item', 'unknown', CONFIDENCE['hold_unknown'], 'VIEW_CLUTTERED_NO_CARGO_COLOUR',
                       observed='not_observed', **common)
    return _answer('holding_item', 'unknown', CONFIDENCE['hold_unknown'], 'HELD_ITEM_AMBIGUOUS',
                   observed='not_observed', **common)


def _edge_coverage(frame, roi_box, kinds, reference):
    """Cargo-colour coverage of the band outside the held-item region."""
    x0, y0, x1, y1 = roi_box
    outside = np.ones(frame.shape[:2], bool)
    outside[y0:y1, x0:x1] = False
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    total = float(outside.sum()) or 1.
    out = {}
    for kind in kinds:
        mask = hue_mask(hsv, kind, reference=reference).astype(bool) & outside
        out[kind] = float(mask.sum())/total
    return out


# --------------------------------------------------------------------------- 3. placed in the zone slot

def judge_placed_in_slot(image, servo_pose: Mapping[int | str, int | float], *,
                         slot_base_xy: Sequence[float], kind: str, zone: str,
                         slot_id: str | None = None) -> dict[str, Any]:
    """Look-back after release: is the item in this zone slot?

    Two independent tests, because the recorded 518 failure was a *false
    negative* of the colour detector alone: a cyan box lit on zone B blue paint
    is dark and desaturated, and the fixed own-RGB HSV range missed it in both
    look-back frames although the cuboid fit on the same frame succeeded.

    A. detector: ``zone_color_boxes.detect_own`` / ``detect_cargo_own`` reports
       the kind within ``SLOT_MATCH_M`` of the slot centre.
    B. local contrast: the slot disc differs from the paint ring around it and
       the differing pixels carry the kind's hue. This needs no absolute
       saturation and so survives the 518 lighting.

    ``no`` is returned only when the slot disc is proven to be bare paint, and
    then only if the paint hue does not overlap the kind's own hue band (on an
    overlapping pair a bare-looking disc is not evidence of absence).
    """
    if kind not in APPEARANCE:
        raise ValueError(f'unknown cargo kind: {kind}')
    if zone not in PAINT_HUE:
        raise ValueError(f'unknown zone: {zone}')
    frame = _frame(image)
    pose = _pose(servo_pose)
    origin, axes, k, d, size = _optics(frame, pose)
    slot = (float(slot_base_xy[0]), float(slot_base_xy[1]))
    rng = math.hypot(slot[0]-origin[0], slot[1]-origin[1])
    overlap = _hue_bands_overlap(kind, zone)
    common = {'slot_id': slot_id, 'kind': kind, 'zone': zone, 'slot_base_m': list(slot),
              'slot_range_m': round(rng, 4), 'paint_hue_overlaps_kind': overlap}
    pixel = _project([slot], 0., origin, axes, k, d)
    if pixel is None or not _inside(pixel[0], size):
        return _answer('placed_in_slot', 'unknown', CONFIDENCE['placed_unknown'], 'SLOT_NOT_IN_VIEW', **common)
    if rng > SLOT_MAX_RANGE_M:
        return _answer('placed_in_slot', 'unknown', CONFIDENCE['placed_unknown'], 'SLOT_BEYOND_FIT_RANGE', **common)
    detected = [det for det in _all_detections(frame, pose, [kind])
                if math.dist(det['estimated_base_m'], slot) <= SLOT_MATCH_M]
    stats = _disc_stats(frame, slot, SLOT_INNER_M, SLOT_OUTER_M, origin, axes, k, d, size)
    contrast = None
    if stats is not None:
        contrast = _contrast_test(stats, kind)
        common.update(bare_share=round(stats['bare_share'], 4),
                      contrast=None if contrast is None else {key: round(value, 4)
                                                              for key, value in contrast.items()})
    common['detections_at_slot'] = [{'kind': det['kind'], 'source': det['source'],
                                     'estimated_base_m': [round(v, 4) for v in det['estimated_base_m']]}
                                    for det in detected]
    ring_is_paint = stats is not None and _ring_is_zone_paint(stats, zone)
    common['reference_ring_is_zone_paint'] = ring_is_paint
    hit = (contrast is not None and contrast['kind_share'] >= CONTRAST_SHARE_MIN
           and contrast['max_delta'] >= CONTRAST_MAX_DELTA_MIN)
    if detected and hit:
        return _answer('placed_in_slot', 'yes', CONFIDENCE['placed_detector_and_contrast'],
                       'DETECTOR_AND_PAINT_CONTRAST_AGREE', **common)
    if detected:
        return _answer('placed_in_slot', 'yes', CONFIDENCE['placed_detector_only'], 'DETECTOR_ONLY', **common)
    if stats is None:
        return _answer('placed_in_slot', 'unknown', CONFIDENCE['placed_unknown'],
                       'SLOT_SURFACE_UNMEASURABLE', **common)
    if not ring_is_paint:
        # The disc's own reference is not the zone paint: the slot is covered or
        # the belief points somewhere else. Nothing can be proven either way.
        return _answer('placed_in_slot', 'unknown', CONFIDENCE['placed_unknown'],
                       'REFERENCE_RING_NOT_ZONE_PAINT', **common)
    if stats['bare_share'] >= PAINT_BARE_SHARE_MIN and not overlap:
        return _answer('placed_in_slot', 'no', CONFIDENCE['placed_bare_paint'], 'SLOT_PROVEN_BARE_PAINT', **common)
    if hit:
        return _answer('placed_in_slot', 'yes', CONFIDENCE['placed_contrast_only'],
                       'PAINT_CONTRAST_ONLY', **common)
    return _answer('placed_in_slot', 'unknown', CONFIDENCE['placed_unknown'],
                   'PAINT_HUE_OVERLAPS_KIND' if overlap else 'SLOT_SURFACE_AMBIGUOUS', **common)


def _ring_is_zone_paint(stats, zone):
    """Is the reference ring the zone's own floor paint (static-map colour)?"""
    bgr = np.asarray([[stats['ring_median_bgr']]], np.uint8)
    hue = float(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0, 0, 0])
    low, high = PAINT_HUE[zone]
    return bool(low-PAINT_RING_HUE_TOL <= hue <= high+PAINT_RING_HUE_TOL)


def _hue_bands_overlap(kind: str, zone: str) -> bool:
    low, high = PAINT_HUE[zone]
    for klow, khigh in APPEARANCE[kind]['hue']:
        if klow <= high and low <= khigh:
            return True
    return False


def _contrast_test(stats, kind):
    """Share of the slot disc that differs from the paint ring *and* carries the
    kind's hue. Independent of any absolute saturation gate."""
    differing = stats['centre_delta'] > CONTRAST_MIN_DELTA
    total = int(differing.size)
    if not total:
        return None
    hsv = stats['centre_hsv']
    if not len(hsv):
        return None
    hue = hsv[:, 0].astype(np.float64)
    close = np.zeros(hue.shape, bool)
    for low, high in APPEARANCE[kind]['hue']:
        tol = CONTRAST_HUE_TOL_DEG/2.
        close |= (hue >= low-tol) & (hue <= high+tol)
    bright = hsv[:, 2].astype(np.float64) >= ABS_MIN_VALUE
    return {'differing_share': float(differing.mean()),
            'kind_share': float((differing & close & bright).mean()),
            'max_delta': float(stats['max_delta'])}


# --------------------------------------------------------------------------- 4. blockage ahead

def judge_route_blockage(image, servo_pose: Mapping[int | str, int | float], *,
                         static_map: Mapping[str, Any], pose_belief: Mapping[str, Any],
                         passage_id: str | None = None,
                         lane_half_width_m: float = LANE_HALF_WIDTH_M) -> dict[str, Any]:
    """Is something the static map does not declare blocking the lane ahead?

    Uses the floor measurement idea of ``harness/visual_floor.py``: pixels that
    do not look like the near-field floor are grouped, their lowest band is
    projected onto the floor plane and the contact is turned into a world point
    with the caller's own pose belief. A contact that coincides with a mapped
    wall footprint (within the belief's slack) is *expected*, not a blockage.

    ``pose_belief`` is ``{'x_m', 'y_m', 'yaw_rad', 'confidence': 'low'|'medium'
    |'high'}``. A ``low`` belief widens the map-matching slack, so an
    obstruction near a wall stays ``unknown`` instead of being reported.
    """
    frame = _frame(image)
    pose = _pose(servo_pose)
    origin, axes, k, d, size = _optics(frame, pose)
    slack = POSE_BELIEF_SLACK_M.get(str(pose_belief.get('confidence', 'low')), POSE_BELIEF_SLACK_M['low'])
    to_world = _belief_transform(pose_belief)
    lane = _lane(static_map, passage_id, lane_half_width_m)
    model, valid = _floor_row_model(frame, origin, axes, k, d, size)
    usable = _usable_region(size[0], size[1])
    common = {'passage_id': passage_id, 'lane_half_width_m': round(lane['half_width_m'], 4),
              'pose_belief': {'x_m': float(pose_belief['x_m']), 'y_m': float(pose_belief['y_m']),
                              'yaw_rad': float(pose_belief['yaw_rad']),
                              'confidence': str(pose_belief.get('confidence', 'low'))},
              'map_slack_m': slack, 'floor_model_rows_px': int(valid.sum())}
    diff = np.max(np.abs(frame.astype(np.int16)-model.astype(np.int16)), axis=2)
    nonfloor = ((diff > BLOCK_LIKE_DIST) & valid[:, None] & usable).astype(np.uint8)
    nonfloor = cv2.morphologyEx(nonfloor, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats_cc, _ = cv2.connectedComponentsWithStats(nonfloor)
    obstructions, mapped, flat, unknown_reasons, border = [], [], [], [], 0
    for index in range(1, count):
        x, y, w, h, area = (int(v) for v in stats_cc[index])
        if area < MIN_BLOCK_AREA_PX:
            continue
        if not _within_usable(x, y, w, h, usable, size):
            # own fingers, the fisheye wedges and anything clipped by them
            border += 1
            continue
        mask = labels[y:y+h, x:x+w] == index
        rows = np.nonzero(mask.any(axis=1))[0]
        bottom = y+int(rows[-1])
        cols = np.nonzero(mask[int(rows[-1])])[0]
        contact_px = (x+float(np.median(cols)), float(bottom))
        contact = _ground(contact_px, origin, axes, k, d)
        if contact is None:
            unknown_reasons.append('NO_FLOOR_CONTACT')
            continue
        base_xy = (float(contact[0]), float(contact[1]))
        rng = math.hypot(base_xy[0], base_xy[1])
        if rng > BLOCK_MAX_RANGE_M or base_xy[0] <= .05:
            continue
        lateral = abs(base_xy[1])
        world = to_world(base_xy)
        wall = _nearest_mapped(static_map, world, slack)
        stands = _height_gate(base_xy, contact_px, y, origin, axes, k, d)
        row = {'contact_px': [round(v, 1) for v in contact_px], 'estimated_base_m': [round(v, 4) for v in base_xy],
               'estimated_world_m': [round(v, 4) for v in world], 'range_m': round(rng, 4),
               'lateral_m': round(lateral, 4), 'area_px': area, 'pixel_bbox': [x, y, w, h],
               'mapped_obstacle': wall, 'height_px': stands['height_px'],
               'height_px_needed': stands['needed_px']}
        if not stands['stands_up']:
            # A flat floor feature (paint edge, shadow, lit patch): not an obstacle.
            flat.append(row)
            continue
        if wall is not None:
            mapped.append(row)
            continue
        if lateral > lane['half_width_m'] + slack:
            continue
        near_wall = _nearest_mapped(static_map, world, MAP_MATCH_M)
        row['near_mapped_obstacle'] = near_wall
        obstructions.append(row)
    mapped_in_lane = [row for row in mapped if row['lateral_m'] <= lane['half_width_m']+slack]
    common.update(mapped_obstacles=mapped[:6], candidates=obstructions[:6], flat_features=len(flat),
                  border_components=border,
                  mapped_in_lane=[row['mapped_obstacle'] for row in mapped_in_lane])
    confident = [row for row in obstructions if row['near_mapped_obstacle'] is None]
    if obstructions and not confident:
        # Every candidate sits within the map-matching margin of a declared wall:
        # with this belief it cannot be told from the wall itself.
        return _answer('route_blockage', 'unknown', CONFIDENCE['block_unknown'],
                       'OBSTRUCTION_NEAR_MAPPED_WALL', observed='not_observed',
                       nearest=min(obstructions, key=lambda row: row['range_m']), **common)
    if confident:
        obstructions = confident
        nearest = min(obstructions, key=lambda row: row['range_m'])
        weak = nearest['area_px'] < 3*MIN_BLOCK_AREA_PX or nearest['range_m'] > 1.4
        return _answer('route_blockage', 'yes',
                       CONFIDENCE['block_unmapped_weak'] if weak else CONFIDENCE['block_unmapped'],
                       'UNMAPPED_OBSTRUCTION_IN_LANE', observed='unmapped_obstruction',
                       nearest=nearest, **common)
    # The static map itself limits how far "clear" can ever be claimed.
    map_cap = _mapped_range_along_lane(static_map, pose_belief, BLOCK_MAX_RANGE_M)
    common['mapped_range_along_lane_m'] = None if map_cap is None else round(map_cap, 4)
    clear_m = _lane_clear_range(frame, nonfloor, origin, axes, k, d, size, lane['half_width_m'],
                                cap_m=map_cap)
    common['lane_clear_range_m'] = None if clear_m is None else round(clear_m, 4)
    if clear_m is not None and clear_m >= CLEAR_MIN_RANGE_M:
        return _answer('route_blockage', 'no', CONFIDENCE['lane_clear'], 'LANE_PROVEN_CLEAR',
                       observed='clear', **common)
    reachable = [r for r in (map_cap, min((row['range_m'] for row in mapped_in_lane), default=None))
                 if r is not None]
    if reachable and clear_m is not None and clear_m >= min(reachable)-.20:
        # A mapped wall filling the lane is the map being right, not a blockage.
        return _answer('route_blockage', 'no', CONFIDENCE['lane_clear'],
                       'ONLY_MAPPED_OBSTACLE_IN_LANE', observed='clear', **common)
    return _answer('route_blockage', 'unknown', CONFIDENCE['block_unknown'],
                   'LANE_VISIBILITY_TOO_SHORT' if not unknown_reasons else unknown_reasons[0],
                   observed='not_observed', **common)


@lru_cache(maxsize=4)
def _usable_region(width: int, height: int) -> np.ndarray:
    """Pixels that the raw fisheye remap actually fills, minus a border margin.

    Uses the fixed camera calibration (``sim.masterpi_camera_profile``), which is
    allowed static knowledge, not simulator state.
    """
    from sim.masterpi_camera_profile import raw_fisheye_remap
    map_x, map_y = raw_fisheye_remap(int(width), int(height))
    valid = ((map_x >= 0) & (map_x <= width-1) & (map_y >= 0) & (map_y <= height-1))
    invalid = cv2.dilate((~valid).astype(np.uint8),
                         np.ones((INVALID_DILATE_PX, INVALID_DILATE_PX), np.uint8)).astype(bool)
    usable = valid & ~invalid
    usable[:BORDER_REJECT_PX, :] = usable[-BORDER_REJECT_PX:, :] = False
    usable[:, :BORDER_REJECT_PX] = usable[:, -BORDER_REJECT_PX:] = False
    return usable


def _within_usable(x, y, w, h, usable, size):
    """True when the component's bounding box keeps clear of the unusable border."""
    if x <= BORDER_REJECT_PX or y <= BORDER_REJECT_PX:
        return False
    if x+w >= size[0]-BORDER_REJECT_PX or y+h >= size[1]-BORDER_REJECT_PX:
        return False
    frame_box = usable[max(0, y-2):y+h+2, max(0, x-2):x+w+2]
    return bool(frame_box.all())


def _height_gate(base_xy, contact_px, top_row, origin, axes, k, d):
    """Does the component stand above its floor contact, or is it a flat feature?"""
    raised = _mb._project_points(np.asarray([[base_xy[0], base_xy[1], MIN_OBSTACLE_HEIGHT_M]], np.float64),
                                 origin, axes, k, d)
    if raised is None:
        return {'stands_up': False, 'height_px': None, 'needed_px': None}
    needed = abs(float(contact_px[1])-float(raised[0][1]))
    height = float(contact_px[1])-float(top_row)
    return {'stands_up': bool(needed > .5 and height >= HEIGHT_GATE_FRACTION*needed),
            'height_px': round(height, 1), 'needed_px': round(needed, 1)}


def _floor_row_model(frame, origin, axes, k, d, size):
    """Median floor colour per image row band, for the rows that see floor in range."""
    height = size[1]
    model = np.zeros((height, 3), np.float32)
    valid = np.zeros(height, bool)
    for y0 in range(0, height, FLOOR_ROW_BAND_PX):
        y1 = min(height, y0+FLOOR_ROW_BAND_PX)
        ground = _ground((size[0]/2., (y0+y1)/2.), origin, axes, k, d)
        if ground is None:
            continue
        rng = math.hypot(float(ground[0]), float(ground[1]))
        if not FLOOR_MODEL_RANGE_M[0] <= rng <= FLOOR_MODEL_RANGE_M[1]:
            continue
        model[y0:y1] = np.median(frame[y0:y1].reshape(-1, 3), axis=0)
        valid[y0:y1] = True
    return model[:, None, :], valid


def _mapped_range_along_lane(static_map, pose_belief, max_m):
    """Range to the first mapped obstacle along the lane centre, from the belief."""
    to_world = _belief_transform(pose_belief)
    for metres in np.arange(.20, max_m+.001, .05):
        if _nearest_mapped(static_map, to_world((float(metres), 0.)), 0.) is not None:
            return float(metres)
    return None


def _belief_transform(pose_belief):
    x, y = float(pose_belief['x_m']), float(pose_belief['y_m'])
    yaw = float(pose_belief['yaw_rad'])
    c, s = math.cos(yaw), math.sin(yaw)

    def to_world(base_xy):
        return (x + c*base_xy[0] - s*base_xy[1], y + s*base_xy[0] + c*base_xy[1])
    return to_world


def _lane(static_map, passage_id, half_width_m):
    for passage in static_map.get('passages', []) or ():
        if passage.get('id') == passage_id:
            width = float(passage.get('width_m') or 2*half_width_m)
            return {'half_width_m': max(.10, width/2-.02), 'passage': passage}
    return {'half_width_m': float(half_width_m), 'passage': None}


def _nearest_mapped(static_map, world_xy, slack):
    """Id of a mapped obstacle whose footprint contains ``world_xy`` within ``slack``."""
    for obstacle in static_map.get('obstacles', []) or ():
        centre = obstacle.get('center_m')
        half = obstacle.get('half_extents_m')
        if not centre or not half:
            continue
        if (abs(world_xy[0]-centre[0]) <= half[0]+slack and abs(world_xy[1]-centre[1]) <= half[1]+slack):
            return obstacle.get('id')
    return None


def _lane_clear_range(frame, nonfloor, origin, axes, k, d, size, half_width_m, *, cap_m=None):
    """How far ahead the lane is proven floor-like, in metres (None if unmeasurable).

    Lateral samples that fall outside the frame are *not measured* rather than
    "not clear": close to the robot the fisheye sees only the middle of the lane.
    The centre sample must be in frame and floor-like at every step. ``cap_m``
    is the range at which the static map itself puts a wall: "clear" is never
    claimed through a mapped obstacle.
    """
    reached = None
    limit = CLEAR_MIN_RANGE_M+.61 if cap_m is None else min(CLEAR_MIN_RANGE_M+.61, cap_m)
    for metres in np.arange(LANE_SWEEP_START_M, limit, .10):
        offsets = np.linspace(-half_width_m*.8, half_width_m*.8, 5)
        pixels = _project([(float(metres), float(off)) for off in offsets], 0., origin, axes, k, d)
        if pixels is None:
            break
        centre = pixels[len(offsets)//2]
        if not _inside(centre, size, 2):
            break
        ok = True
        for pixel in pixels:
            if not _inside(pixel, size, 2):
                continue
            if nonfloor[int(round(pixel[1])), int(round(pixel[0]))]:
                ok = False
                break
        if not ok:
            break
        reached = float(metres)
    return reached


__all__ = ['SCHEMA', 'ANSWERS', 'JUDGMENTS', 'PROVENANCE', 'APPEARANCE', 'PAINT_HUE',
           'CARRY_POSTURE', 'LOOK_P20_POSTURE', 'PAN_PULSES', 'BOX_KINDS', 'CARGO_KINDS', 'TEAM_KINDS',
           'KNOWN_KINDS', 'hue_mask', 'detect_cargo_own', 'judge_slot_item', 'judge_holding_item',
           'judge_placed_in_slot', 'judge_route_blockage']
