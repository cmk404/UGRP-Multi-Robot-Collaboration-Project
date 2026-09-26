"""Wrist-RGB-only zone judgments, version 2 (package B follow-up).

``harness/zone_own_perception.py`` (v1, PR #193) is imported and **never
modified**: its four judgments keep their thresholds, reasons and confidences
byte for byte, and v2 reuses its helpers. This module adds the three judgments
the Korean-dialogue study asked for next:

5. ``judge_peer_in_lane``        a peer MasterPi ahead, or an unmapped object?
                                 "a robot is standing in the door" and "the door
                                 is blocked by an object" must become different
                                 Korean reports, so the discrimination has to
                                 exist before the message does. Appearance only
                                 (the robot's own paint and geometry); no peer
                                 pose, no peer command log, no TOP.
6. ``judge_team_cargo_identity`` which team item (``long_beam``,
                                 ``heavy_crate``, ``tri_frame``) is in front of
                                 me, from partial views: at a pickup slot, while
                                 approaching a handle, and while carrying.
7. ``judge_team_cargo_handle``   "my handle is here": is a graspable handle of
                                 that item within my own reach, in front of me?
8. ``judge_held_item``           what am I holding, in the **held-check
                                 posture**. v1 could not answer this for a
                                 ``can``: in the wrist skill's CARRY posture a
                                 held can shows nothing inside the held-item
                                 region. The posture below was measured with
                                 ``scripts/probe_held_can_posture.py``.

Allowed inputs are exactly v1's: one own wrist fisheye frame, the robot's own
*issued* arm pulses, the versioned static map, the scenario order sheet and an
injected own-pose belief. No TOP frame, no simulator pose/segmentation/contact,
no measured joints, no teacher phase, no other robot's data. ``import mujoco``
never appears (``tests/test_zone_own_perception_v2.py`` asserts it).

Every judgment returns ``yes`` / ``no`` / ``unknown`` with a confidence.
``unknown`` stays a first-class answer and no gate turns "cannot see it" into
``no``; a ``no`` always names the positive evidence it stands on.

Thresholds marked ``PREREGISTERED`` were fixed on the dev split before the
held-out split was scored (``experiments/2026-09-26-zone-own-perception-v2``).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness import zone_own_perception as _v1

SCHEMA = 'ugrp.zone_own_perception.v2'
PROFILE = 'zone_own_perception_v2'
ANSWERS = _v1.ANSWERS
JUDGMENTS = ('peer_in_lane', 'team_cargo_identity', 'team_cargo_handle', 'held_item')
PROVENANCE = _v1.PROVENANCE
TEAM_KINDS = _v1.TEAM_KINDS
SOLO_KINDS = _v1.BOX_KINDS + _v1.CARGO_KINDS

# --- held-check observation posture (PREREGISTERED) --------------------------
# ``robot_cam`` sits at gripper (.067, 0, .0136) and ``grip_site`` at (.100, 0,
# 0): camera and held item are one rigid body, so no posture can move the item
# in the tool frame. What a posture changes is the angle between the tool axis
# and the item's world-vertical axis (plus the background and the lighting).
# Measured with scripts/probe_held_can_posture.py (segmentation, eval only), a
# held 50 mm can in the own wrist frame / inside the held-item region:
#   carry_p30 (wrist skill CARRY, tool pitch -30.1 deg) 12,972 px /      0 px
#   look_p20  (package B look posture, -20.0 deg)          971 px /      0 px
#   tilt_m20  (+27.7 deg)                               72,446 px / 32,560 px
#   high_m45  (+43.9 deg)                              107,098 px / 73,657 px
#   tilt_m70  (+78.1 deg)                                6,595 px /  6,489 px
# high_m45 is the posture below: grip site 0.143 m ahead and 0.299 m high, tool
# pitched 43.9 deg up, so the can stands in front of the lens instead of at the
# top edge. Retention through CARRY -> posture -> hold -> CARRY with a ground
# truth teacher grasp and weld OFF: 0.044 mm of slip under ``cargo_noslip_v1``
# (0.043 mm over a 20 s hold) and 0.485 mm under ``local_contact_fine``
# (1.254 mm over 20 s); finger contact was never lost in either.
HELD_CHECK_POSTURE = {1: 1500, 3: 1500, 4: 1700, 5: 1900, 6: 1500}
HELD_CHECK_ROI = _v1.CARRY_ROI          # the same rectangle; the posture is what changed
HELD_POSTURES = {'held_check': HELD_CHECK_POSTURE, 'carry': _v1.CARRY_POSTURE}
# Dev measurement (2026-09-26-zone-own-perception-v2): in the held-check posture
# a held can covers 26,064 px of the held-item region while a held box or tile
# shows **0 px anywhere in the frame** - the two postures are complementary, so
# each may only claim absence for the kinds it frames. CARRY frames the four
# boxes and the tile (v1's list); the held-check posture frames the can.
HELD_CHECK_FRAMED_KINDS = ('can',)
HELD_ROI_MIN_VALUE = .30        # ... and the region must be lit before absence is claimed

# --- pre-grasp look posture (PREREGISTERED) --------------------------------
# ``judge_team_cargo_handle`` projects a handle's floor contact, so the frame has
# to contain the floor of the reach band. The travel look posture
# (``zone_own_perception.LOOK_P20_POSTURE``, tool pitch -20 deg) only starts
# seeing the floor at about 0.45 m: dev measured five confident "the handle is
# somewhere else" errors with it, every one of them a near handle at 0.24-0.32 m
# that was simply below the frame. This is the calibrated grasp IK for the grip
# site 0.155 m ahead at 0.095 m (``visual_arm.solve_grip_ik(.155, 0, .095, -55)``,
# the wrist skill's hover height), tool pitch -54.9 deg, which does see it.
APPROACH_LOOK_POSTURE = {1: 2000, 3: 973, 4: 2212, 5: 1959, 6: 1500}
HANDLE_POSTURES = {'approach_look': APPROACH_LOOK_POSTURE, 'travel_look': _v1.LOOK_P20_POSTURE}

# --- peer robot appearance (PREREGISTERED) ---------------------------------
# From the robot model (sim/masterpi_scene_v2.xml), unchanged: the MasterPi is
# painted in exactly three materials that matter at wrist range.
#   orange   .95 .60 .03  -> OpenCV H 19, S 247  (arm plates, shoulder and elbow
#                            horns, wrist plates, finger pads, 32 mecanum
#                            rollers) - saturated, and spread over the whole body
#   aluminum .48 .50 .52  -> H 105, S 20, V 133  (side/front/rear panels,
#                            electronics covers, jaw links, camera bracket)
#   dark     .055 .06 .068 / black .018 .02 .024 (base top, servo bodies, hubs)
# Dev measurement of the candidate boxes: a peer gives orange 0.29-0.52, grey
# 0.12-0.38, dark 0.10-0.34 and 0.83-0.91 of the box explained by those three;
# the brown ``unmapped_block`` gives orange 0.000, grey 0.018 and body 0.018, and
# wall/shadow candidates give orange 0.000, body 0.03-0.12. The saturation floor
# is what keeps zone A floor paint (H 8-18), the pale ``tri_frame`` (H 14-26) and
# the brown block (H 14, S 106) out of the orange mask.
PEER_ORANGE_HUE = (12, 26)
PEER_ORANGE_MIN_SAT = 120       # the confusable surfaces stay below this
PEER_ORANGE_REL_SAT = .55       # ... or below this share of the frame's 75th percentile
PEER_GREY_MAX_SAT = 70
# Share of the frame's 75th-percentile V. The upper end is above 1.0 on purpose:
# an aluminium panel can be brighter than the floor it stands on. The absolute cap
# is what keeps blown-out white out of the grey mask.
PEER_GREY_VALUE_BAND = (.22, 1.35)
PEER_GREY_MAX_VALUE = 245
PEER_DARK_MAX_VALUE = .38           # ... share for the near-black body
PEER_MIN_PATCH_PX = 12              # an orange patch this small is noise
PEER_ORANGE_SHARE_MIN = .06         # share of the candidate that is robot orange
PEER_GREY_SHARE_MIN = .05           # ... with an aluminium body next to it
PEER_STRONG_ORANGE_SHARE = .30      # this much saturated orange is a robot on its own
PEER_OBJECT_ORANGE_MAX = .010       # below this the candidate carries no robot paint
PEER_OBJECT_BODY_MAX = .35          # ... and the robot materials do not explain it
PEER_MIN_CANDIDATE_PX = 320         # smaller than this and the appearance is unreadable
# One peer arrives as several components: the floor-difference mask cuts a
# MasterPi into an orange arm, a grey panel and a dark chassis (dev: fragments of
# orange 0.63-0.91 with grey 0.01-0.05 each, against 0.21/0.24 for the whole
# robot in one piece). The signature is therefore also evaluated over the union of
# the unmapped candidates in the lane, because a robot is one object.
# v2 candidate extraction. v1's ``judge_route_blockage`` drops every component
# that touches the fisheye border, because the robot's own fingers sit there and
# a false *blockage* report is expensive. A peer standing close fills the frame
# and touches that border too (dev: a peer of 14,534 px produced no v1
# candidate at all), so the peer judgment runs its own extraction with a range
# floor instead of a border rule: the own gripper is 0.03-0.15 m from the lens
# and never projects a floor contact beyond ``PEER_MIN_RANGE_M``.
PEER_MIN_RANGE_M = .28
PEER_MAX_RANGE_M = 2.50
PEER_MIN_AREA_PX = 300
PEER_LANE_SLACK_M = .05

# --- team cargo appearance (PREREGISTERED) ---------------------------------
# Catalogue colours and geometry (sim/zone_cargo.py), unchanged:
#   long_beam    lime  .55 .78 .12 -> H 40, S 216; 0.60 x 0.04 x 0.032 m bar,
#                                    black grip bands 36 mm wide at x = +-0.27
#   heavy_crate  pink  1.0 .58 .72 -> H 170, S 107; 0.14 x 0.10 x 0.06 body with
#                                    black lugs at x = +-0.09 (0.24 m overall)
#   tri_frame    cream .95 .88 .66 -> H 23, S 78; open triangle outline, side
#                                    0.35 m, black lug at each vertex (r 0.22)
# Dev medians inside the item silhouette: beam H 40 / S 212 / V 190, crate H 169
# / S 99 / V 166, frame H 23 / S 43 / V 141. ``tri_frame`` is pale, and its hue
# band overlaps the robot orange (S 247) and the yellow box paint (S 241), so its
# mask needs a saturation *window*, not only a floor. v1's ``hue_mask`` has floors
# only and is not modified, so the team masks below are v2's own.
# Dev measurement: the zone ceiling light shifts the lit lime beam from H 40 to
# H 30 with S 173-190 and V 253-255, so the beam band has to reach down to H 28.
# That overlaps the pale ``tri_frame`` (H 12-32) and the yellow box paint
# (H 21-34), and the saturation windows are what separate them: the beam never
# drops below S 150 in any dev view, while ``tri_frame`` never rises above 140.
TEAM_MASK = {
    'long_beam':   {'hue': (28, 58), 'sat': (150, 255), 'value': (40, 255), 'rel_value': .20},
    'heavy_crate': {'hue': (156, 178), 'sat': (45, 210), 'value': (60, 255), 'rel_value': .35},
    'tri_frame':   {'hue': (12, 32), 'sat': (22, 140), 'value': (80, 255), 'rel_value': .45},
}
TEAM_MIN_COVERAGE = .010        # share of the usable frame carrying the kind's colour
TEAM_MARGIN = 1.8               # best kind coverage over the runner-up
TEAM_MIN_COMPONENT_PX = 260
# Silhouette guards, from the catalogue shapes. They only stop a stray blob of
# the right colour from being called cargo; the three hues already separate the
# three kinds from each other. Dev: beam aspect 12.7, crate 2.46, frame 3.46 in
# 4 fragments (the wrist view cuts the open triangle into edge bars).
TEAM_MIN_ASPECT = {'long_beam': 2.6}
TEAM_MAX_ASPECT = {'heavy_crate': 5.0}
TEAM_FRAGMENTED = ('tri_frame',)     # accepted as several components of one outline
# A carried item is 2-5 cm from the lens and its silhouette is cut by the frame:
# dev measured the carried beam at aspect 1.43 over 65 % of the usable region,
# against 8 % at a pickup slot and 13 % at the approach standoff. Above the share
# below the silhouette is a partial view, the elongation guard is skipped, and the
# answer is reported with the weaker confidence because colour alone carried it.
TEAM_CROP_AREA_SHARE = .25
HANDLE_MAX_VALUE = .34          # share of the frame's 75th-percentile V
HANDLE_MAX_SAT = 90
HANDLE_MIN_PX = 90
HANDLE_NEAR_CARGO_PX = 14       # a handle must touch the kind's colour mask
# Pre-grasp reach band: "the handle I would take is in front of me, within one
# approach step". The calibrated grasp radius is 0.155 m from the base centre and
# the wrist skill starts its RGB approach at a 0.40 m standoff, so the band spans
# both. It is not a claim that the handle is already between the jaws.
HANDLE_REACH_BAND_M = (.18, .45)
HANDLE_LATERAL_M = .10
HANDLE_MAX_RANGE_M = 1.20            # v1's near-range floor-fit limit


CONFIDENCE = {
    'peer_signature': .88,
    'peer_object': .80,
    'peer_lane_clear': .75,
    'peer_unknown': .0,
    'team_kind_matched': .88,
    'team_kind_matched_weak': .70,
    'team_kind_conflict': .82,
    'team_unknown': .0,
    'handle_in_reach': .85,
    'handle_out_of_reach': .75,
    'handle_unknown': .0,
    'held_kind_matched': .88,
    'held_kind_conflict': .82,
    'held_empty': .75,
    'held_unknown': .0,
}


# --------------------------------------------------------------------------- shared

def _prepare(image, servo_pose):
    frame = _v1._frame(image)
    pose = _v1._pose(servo_pose)
    height, width = frame.shape[:2]
    usable = _v1._usable_region(width, height)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    reference = _v1._frame_reference(hsv)
    return frame, pose, hsv, reference, usable


def _answer(judgment, answer, confidence, reason, **extra):
    if answer not in ANSWERS:
        raise ValueError(f'answer must be one of {ANSWERS}')
    row = {'schema': SCHEMA, 'profile': PROFILE, 'judgment': judgment, 'answer': answer,
           'confidence': round(float(confidence), 3), 'reason': reason, 'provenance': PROVENANCE}
    row.update(extra)
    return row


def _clip_box(box, size, pad=0):
    x, y, w, h = (int(v) for v in box)
    x0, y0 = max(0, x-pad), max(0, y-pad)
    x1, y1 = min(size[0], x+w+pad), min(size[1], y+h+pad)
    return x0, y0, x1, y1


# --------------------------------------------------------------------------- 5. peer or object

def robot_appearance_stats(hsv: np.ndarray, reference: tuple[float, float], region: np.ndarray
                           ) -> dict[str, Any]:
    """Robot-paint signature of one image region (appearance only, no pose).

    ``region`` is a boolean mask. Returns the share of the region that is the
    robot's saturated orange, how many separate orange patches it forms, and the
    shares that look like the aluminium panels and the near-black chassis.
    """
    total = int(region.sum())
    if not total:
        return {'pixels': 0, 'orange_share': 0., 'orange_parts': 0, 'grey_share': 0., 'dark_share': 0.,
                'body_share': 0., 'dominant_share': 0., 'dominant_hue': None}
    sat_ref, val_ref = reference
    hue, sat, val = hsv[..., 0].astype(np.int16), hsv[..., 1].astype(np.int16), hsv[..., 2].astype(np.int16)
    min_sat = max(PEER_ORANGE_MIN_SAT, PEER_ORANGE_REL_SAT*sat_ref)
    orange = ((hue >= PEER_ORANGE_HUE[0]) & (hue <= PEER_ORANGE_HUE[1]) & (sat >= min_sat)
              & (val >= _v1.ABS_MIN_VALUE) & region)
    grey = ((sat <= PEER_GREY_MAX_SAT) & (val >= PEER_GREY_VALUE_BAND[0]*val_ref)
            & (val <= min(PEER_GREY_VALUE_BAND[1]*val_ref, PEER_GREY_MAX_VALUE)) & region)
    dark = (val <= PEER_DARK_MAX_VALUE*val_ref) & region
    patches = 0
    if orange.any():
        count, _, stats, _ = cv2.connectedComponentsWithStats(orange.astype(np.uint8))
        patches = sum(1 for i in range(1, count) if int(stats[i][4]) >= PEER_MIN_PATCH_PX)
    # Dominant non-robot colour: the largest saturated hue bin that is not the
    # robot orange. A left-behind object is one such colour over most of the box.
    bins = np.zeros(18, np.int64)
    coloured = region & (sat >= _v1.ABS_MIN_SAT) & (val >= _v1.ABS_MIN_VALUE)
    if coloured.any():
        idx = (hue[coloured]//10).clip(0, 17)
        for value in idx:
            bins[value] += 1
    dominant = int(bins.argmax()) if bins.any() else None
    dominant_share = float(bins.max())/total if bins.any() else 0.
    return {'pixels': total, 'orange_share': float(orange.sum())/total, 'orange_parts': int(patches),
            'grey_share': float(grey.sum())/total, 'dark_share': float(dark.sum())/total,
            'body_share': float((orange | grey | dark).sum())/total,
            'dominant_share': dominant_share,
            'dominant_hue': None if dominant is None else dominant*10+5}


def _classify_candidate(hsv, reference, usable, box, size, component_mask=None):
    """Appearance verdict for one candidate.

    The statistics are taken over the component's own pixels when they are known,
    not over its bounding box: a narrow robot in a wide box would otherwise be
    diluted by the floor behind it.
    """
    x0, y0, x1, y1 = _clip_box(box, size)
    region = np.zeros(hsv.shape[:2], bool)
    if component_mask is None:
        region[y0:y1, x0:x1] = True
    else:
        h, w = component_mask.shape
        region[y0:y0+h, x0:x0+w] = component_mask[:min(h, size[1]-y0), :min(w, size[0]-x0)]
    region &= usable
    stats = robot_appearance_stats(hsv, reference, region)
    return {'verdict': _verdict(stats), 'pixel_bbox': [x0, y0, x1-x0, y1-y0],
            **{k: (round(v, 4) if isinstance(v, float) else v) for k, v in stats.items()}}, region


def _verdict(stats):
    """Appearance verdict from the robot-paint signature of one region."""
    if stats['orange_share'] >= PEER_STRONG_ORANGE_SHARE:
        return 'peer_robot'
    if stats['orange_share'] >= PEER_ORANGE_SHARE_MIN and stats['grey_share'] >= PEER_GREY_SHARE_MIN:
        return 'peer_robot'
    if stats['orange_share'] <= PEER_OBJECT_ORANGE_MAX and stats['body_share'] <= PEER_OBJECT_BODY_MAX:
        return 'unmapped_object'
    return 'unreadable'


def _peer_candidates(frame, pose, static_map, pose_belief, lane_half_width_m, usable):
    """Standing non-floor components in the lane ahead (v2 border policy).

    The floor model, the floor projection, the height gate and the map matching
    are v1's helpers, unchanged. What differs from v1's blockage candidates is
    that a component touching the fisheye border is kept: it is rejected by a
    range floor instead, because the robot's own gripper never projects a floor
    contact past ``PEER_MIN_RANGE_M``.
    """
    origin, axes, k, d, size = _v1._optics(frame, pose)
    slack = _v1.POSE_BELIEF_SLACK_M.get(str(pose_belief.get('confidence', 'low')),
                                        _v1.POSE_BELIEF_SLACK_M['low'])
    to_world = _v1._belief_transform(pose_belief)
    model, valid = _v1._floor_row_model(frame, origin, axes, k, d, size)
    diff = np.max(np.abs(frame.astype(np.int16)-model.astype(np.int16)), axis=2)
    nonfloor = ((diff > _v1.BLOCK_LIKE_DIST) & valid[:, None] & usable).astype(np.uint8)
    nonfloor = cv2.morphologyEx(nonfloor, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats_cc, _ = cv2.connectedComponentsWithStats(nonfloor)
    rows, masks = [], []
    dropped = {'small': 0, 'no_contact': 0, 'too_near': 0, 'too_far': 0, 'off_lane': 0, 'flat': 0}
    for index in range(1, count):
        x, y, w, h, area = (int(v) for v in stats_cc[index])
        if area < PEER_MIN_AREA_PX:
            dropped['small'] += 1
            continue
        mask = labels[y:y+h, x:x+w] == index
        image_rows = np.nonzero(mask.any(axis=1))[0]
        bottom = y+int(image_rows[-1])
        cols = np.nonzero(mask[int(image_rows[-1])])[0]
        contact_px = (x+float(np.median(cols)), float(bottom))
        contact = _v1._ground(contact_px, origin, axes, k, d)
        if contact is None:
            dropped['no_contact'] += 1
            continue
        base_xy = (float(contact[0]), float(contact[1]))
        rng = math.hypot(base_xy[0], base_xy[1])
        if rng < PEER_MIN_RANGE_M:
            dropped['too_near'] += 1
            continue
        if rng > PEER_MAX_RANGE_M:
            dropped['too_far'] += 1
            continue
        if abs(base_xy[1]) > lane_half_width_m + slack + PEER_LANE_SLACK_M:
            dropped['off_lane'] += 1
            continue
        stands = _v1._height_gate(base_xy, contact_px, y, origin, axes, k, d)
        if not stands['stands_up']:
            dropped['flat'] += 1
            continue
        world = to_world(base_xy)
        rows.append({'pixel_bbox': [x, y, w, h], 'area_px': area, 'range_m': round(rng, 4),
                     'estimated_base_m': [round(v, 4) for v in base_xy],
                     'estimated_world_m': [round(v, 4) for v in world],
                     'mapped_obstacle': _v1._nearest_mapped(static_map, world, slack),
                     'height_px': stands['height_px'], 'height_px_needed': stands['needed_px']})
        masks.append(mask)
    return rows, masks, dropped, slack


def judge_peer_in_lane(image, servo_pose: Mapping[int | str, int | float], *,
                       static_map: Mapping[str, Any], pose_belief: Mapping[str, Any],
                       passage_id: str | None = None,
                       lane_half_width_m: float = _v1.LANE_HALF_WIDTH_M) -> dict[str, Any]:
    """Is the thing ahead in my lane a peer MasterPi, or an object?

    The lane geometry, the floor model, the height gate and the map matching are
    v1's, and v1's ``judge_route_blockage`` answer is recorded next to this one so
    the two reports can be read together ("a robot is in the door" against "the
    door is blocked"). Answers:

    * ``yes``  / ``peer_robot``       the robot paint signature is on a standing
      candidate in the lane -> report "a robot is in the way", which the peer can
      be asked to move.
    * ``no``   / ``unmapped_object``  a standing candidate in the lane carries
      none of the robot's three materials -> "an object is in the way".
    * ``no``   / ``clear``            nothing stands in the lane and v1 proved the
      floor clear ahead, so no peer is there either.
    * ``unknown``                     the candidate is too small or too mixed to
      read, or standing candidates disagree with each other.
    """
    frame, pose, hsv, reference, usable = _prepare(image, servo_pose)
    size = (frame.shape[1], frame.shape[0])
    blockage = _v1.judge_route_blockage(frame, pose, static_map=static_map, pose_belief=pose_belief,
                                        passage_id=passage_id, lane_half_width_m=lane_half_width_m)
    half = blockage['lane_half_width_m']
    rows, masks, dropped, slack = _peer_candidates(frame, pose, static_map, pose_belief, half, usable)
    common = {'route_blockage': {key: blockage[key] for key in ('answer', 'confidence', 'reason',
                                                                'observed')},
              'lane_half_width_m': half, 'map_slack_m': slack,
              'pose_belief': blockage['pose_belief'], 'dropped_components': dropped}
    classified, regions = [], []
    for row, mask in zip(rows, masks):
        info, region = _classify_candidate(hsv, reference, usable, row['pixel_bbox'], size, mask)
        info.update(estimated_base_m=row['estimated_base_m'], range_m=row['range_m'],
                    area_px=row['area_px'], mapped_obstacle=row['mapped_obstacle'])
        classified.append(info)
        regions.append(region)
    unmapped = [row for row in classified if row['mapped_obstacle'] is None]
    union = None
    if classified:
        merged = np.zeros(hsv.shape[:2], bool)
        for region in regions:
            merged |= region
        stats = robot_appearance_stats(hsv, reference, merged)
        union = {'verdict': _verdict(stats),
                 **{k: (round(v, 4) if isinstance(v, float) else v) for k, v in stats.items()}}
    common['candidates'] = classified[:6]
    common['union'] = union
    common['mapped_candidates'] = [row['mapped_obstacle'] for row in classified
                                   if row['mapped_obstacle'] is not None][:6]
    if not unmapped:
        if blockage['answer'] == 'no' and blockage.get('observed') == 'clear':
            return _answer('peer_in_lane', 'no', CONFIDENCE['peer_lane_clear'],
                           'LANE_PROVEN_CLEAR_NO_PEER', observed='clear', **common)
        return _answer('peer_in_lane', 'unknown', CONFIDENCE['peer_unknown'],
                       'NO_UNMAPPED_CANDIDATE_IN_LANE', observed='not_observed', **common)
    readable = [row for row in unmapped if row['pixels'] >= PEER_MIN_CANDIDATE_PX]
    if not readable:
        return _answer('peer_in_lane', 'unknown', CONFIDENCE['peer_unknown'],
                       'CANDIDATE_TOO_SMALL_TO_READ', observed='not_observed', **common)
    peers = [row for row in readable if row['verdict'] == 'peer_robot']
    if peers or (union is not None and union['verdict'] == 'peer_robot'):
        nearest = min(peers or readable, key=lambda row: row['range_m'])
        return _answer('peer_in_lane', 'yes', CONFIDENCE['peer_signature'],
                       'ROBOT_PAINT_SIGNATURE_IN_LANE', observed='peer_robot', nearest=nearest, **common)
    objects = [row for row in readable if row['verdict'] == 'unmapped_object']
    if objects and len(objects) == len(readable):
        nearest = min(objects, key=lambda row: row['range_m'])
        return _answer('peer_in_lane', 'no', CONFIDENCE['peer_object'], 'NO_ROBOT_PAINT_ON_CANDIDATE',
                       observed='unmapped_object', nearest=nearest, **common)
    return _answer('peer_in_lane', 'unknown', CONFIDENCE['peer_unknown'], 'CANDIDATE_APPEARANCE_MIXED',
                   observed='not_observed', **common)


# --------------------------------------------------------------------------- 6. team cargo identity

def team_mask(hsv: np.ndarray, kind: str, reference: tuple[float, float]) -> np.ndarray:
    """Illumination-relative mask for one team cargo kind (v2's own windows).

    Unlike v1's ``hue_mask`` this uses a saturation *window*: ``tri_frame`` is a
    pale cream whose hue band also contains the robot's saturated orange and the
    yellow box paint, so an upper saturation bound is what separates them.
    """
    if kind not in TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {kind}')
    spec = TEAM_MASK[kind]
    _, val_ref = reference
    low_v = max(spec['value'][0], spec['rel_value']*val_ref)
    low = np.asarray((spec['hue'][0], spec['sat'][0], low_v), np.uint8)
    high = np.asarray((spec['hue'][1], spec['sat'][1], spec['value'][1]), np.uint8)
    return cv2.inRange(hsv, low, high)


def _kind_components(hsv, reference, usable, kind):
    mask = team_mask(hsv, kind, reference).astype(bool) & usable
    comps, cleaned = _v1._components(mask.astype(np.uint8)*255, TEAM_MIN_COMPONENT_PX)
    return comps, cleaned.astype(bool), float(mask.sum())/max(int(usable.sum()), 1)


def _shape_stats(comp, usable_px):
    x, y, w, h = comp['bbox']
    aspect = max(w, h)/max(1., min(w, h))
    hull = cv2.convexHull(comp['contour'])
    hull_area = float(cv2.contourArea(hull)) or 1.
    share = comp['area']/max(float(usable_px), 1.)
    return {'aspect': round(float(aspect), 3), 'fill_of_hull': round(comp['area']/hull_area, 3),
            'area_px': comp['area'], 'bbox': [int(v) for v in comp['bbox']],
            'usable_share': round(share, 4), 'silhouette_cropped': bool(share >= TEAM_CROP_AREA_SHARE)}


def _shape_agrees(kind, shape, components):
    """Catalogue silhouette guard: only stops a stray blob being called cargo."""
    if shape['silhouette_cropped']:
        return True, 'CROPPED_SILHOUETTE_NOT_TESTED'
    if kind in TEAM_MIN_ASPECT and shape['aspect'] < TEAM_MIN_ASPECT[kind]:
        return False, 'NOT_ELONGATED'
    if kind in TEAM_MAX_ASPECT and shape['aspect'] > TEAM_MAX_ASPECT[kind]:
        return False, 'TOO_ELONGATED'
    if kind in TEAM_FRAGMENTED and components < 2 and shape['aspect'] > 5.0:
        # The open triangle reaches the wrist view as two or three edge bars; one
        # very thin bar on its own is not the outline.
        return False, 'SINGLE_THIN_BAR'
    return True, None


def judge_team_cargo_identity(image, servo_pose: Mapping[int | str, int | float], *,
                              expected_kind: str,
                              candidate_kinds: Sequence[str] = TEAM_KINDS) -> dict[str, Any]:
    """Is the order sheet's team item the one in front of me (partial views allowed)?

    Only appearance is used: the catalogue colour window of each team kind, the
    largest component of that colour, and a silhouette guard from the catalogue
    shape. No range, no pose and no slot are needed, so the same judgment covers a
    full view at a pickup slot, a partial view while approaching a handle and a
    partial view while carrying.

    ``no`` requires another *known* team kind to win decisively. A view that only
    shows a sliver of colour, or whose best colour does not match its own
    silhouette, stays ``unknown``.
    """
    if expected_kind not in _v1.APPEARANCE:
        raise ValueError(f'unknown cargo kind: {expected_kind}')
    if expected_kind not in TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame, pose, hsv, reference, usable = _prepare(image, servo_pose)
    usable_px = int(usable.sum())
    kinds = [k for k in candidate_kinds if k in TEAM_MASK]
    if expected_kind not in kinds:
        kinds.append(expected_kind)
    per = {}
    for kind in kinds:
        comps, _, share = _kind_components(hsv, reference, usable, kind)
        best = max(comps, key=lambda c: c['area'], default=None)
        shape = None if best is None else _shape_stats(best, usable_px)
        agrees, why = ((False, 'NO_COMPONENT') if shape is None
                       else _shape_agrees(kind, shape, len(comps)))
        per[kind] = {'colour_share': round(share, 5), 'components': len(comps), 'component': shape,
                     'shape_agrees': bool(agrees), 'shape_reason': why}
    ranked = sorted(per.items(), key=lambda item: -item[1]['colour_share'])
    best_kind, best_row = ranked[0]
    runner = ranked[1][1]['colour_share'] if len(ranked) > 1 else 0.
    common = {'expected_kind': expected_kind, 'per_kind': per, 'best_kind': best_kind,
              'margin': round(best_row['colour_share']/max(runner, 1e-6), 3)}
    if best_row['colour_share'] < TEAM_MIN_COVERAGE or best_row['component'] is None:
        return _answer('team_cargo_identity', 'unknown', CONFIDENCE['team_unknown'],
                       'NO_TEAM_CARGO_COLOUR_IN_VIEW', observed='not_observed', **common)
    decisive = best_row['colour_share'] >= TEAM_MARGIN*max(runner, 1e-6)
    if not decisive:
        return _answer('team_cargo_identity', 'unknown', CONFIDENCE['team_unknown'],
                       'TWO_TEAM_COLOURS_WITHOUT_MARGIN', observed='not_observed', **common)
    if not best_row['shape_agrees']:
        # The colour says one kind and its own silhouette disagrees: something is
        # there, but this is not an identification.
        return _answer('team_cargo_identity', 'unknown', CONFIDENCE['team_unknown'],
                       'SHAPE_DISAGREES_' + str(best_row['shape_reason']), observed='not_observed',
                       **common)
    cropped = bool(best_row['component']['silhouette_cropped'])
    common['silhouette_cropped'] = cropped
    if best_kind == expected_kind:
        return _answer('team_cargo_identity', 'yes',
                       CONFIDENCE['team_kind_matched_weak'] if cropped
                       else CONFIDENCE['team_kind_matched'],
                       'EXPECTED_TEAM_KIND_SEEN_CROPPED' if cropped else 'EXPECTED_TEAM_KIND_SEEN',
                       observed=expected_kind, **common)
    return _answer('team_cargo_identity', 'no', CONFIDENCE['team_kind_conflict'],
                   'OTHER_TEAM_KIND_SEEN', observed=best_kind, **common)


# --------------------------------------------------------------------------- 7. my handle is here

def judge_team_cargo_handle(image, servo_pose: Mapping[int | str, int | float], *,
                            expected_kind: str,
                            reach_band_m: tuple[float, float] = HANDLE_REACH_BAND_M,
                            lateral_m: float = HANDLE_LATERAL_M) -> dict[str, Any]:
    """"My handle is here": a graspable handle of this item within my own reach.

    Every catalogue handle is the same unpainted near-black lug or grip band on
    a coloured body (``sim/zone_cargo.py``), so a handle is a dark component that
    *touches* the kind's colour mask. Its lowest row is projected onto the floor
    plane with the fixed camera calibration, which puts it in the robot's own
    base frame; the answer is whether that point lies in the calibrated grasp
    band ahead of the robot.

    * ``yes``      a handle sits in the reach band in front of me.
    * ``no``       handles of this item were found and all of them are outside
      the band (positive evidence: I am at the wrong end and must move).
    * ``unknown``  the item is not identified here, no handle feature is visible,
      or the contact cannot be projected.
    """
    if expected_kind not in TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame, pose, hsv, reference, usable = _prepare(image, servo_pose)
    origin, axes, k, d, size = _v1._optics(frame, pose)
    comps, colour_mask, share = _kind_components(hsv, reference, usable, expected_kind)
    sat_ref, val_ref = reference
    dark = ((hsv[..., 2] <= HANDLE_MAX_VALUE*val_ref) & (hsv[..., 1] <= HANDLE_MAX_SAT) & usable)
    near_cargo = cv2.dilate(colour_mask.astype(np.uint8),
                            np.ones((HANDLE_NEAR_CARGO_PX, HANDLE_NEAR_CARGO_PX), np.uint8)).astype(bool)
    handle_mask = (dark & near_cargo).astype(np.uint8)
    handle_mask = cv2.morphologyEx(handle_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats_cc, _ = cv2.connectedComponentsWithStats(handle_mask)
    common = {'expected_kind': expected_kind, 'colour_share': round(share, 5),
              'reach_band_m': [round(v, 4) for v in reach_band_m], 'lateral_m': round(lateral_m, 4),
              'cargo_components': len(comps)}
    if not comps:
        return _answer('team_cargo_handle', 'unknown', CONFIDENCE['handle_unknown'],
                       'KIND_NOT_IDENTIFIED_HERE', observed='not_observed', **common)
    found = []
    for index in range(1, count):
        x, y, w, h, area = (int(v) for v in stats_cc[index])
        if area < HANDLE_MIN_PX:
            continue
        mask = labels[y:y+h, x:x+w] == index
        rows = np.nonzero(mask.any(axis=1))[0]
        bottom = y+int(rows[-1])
        cols = np.nonzero(mask[int(rows[-1])])[0]
        contact_px = (x+float(np.median(cols)), float(bottom))
        ground = _v1._ground(contact_px, origin, axes, k, d)
        if ground is None:
            continue
        base_xy = (float(ground[0]), float(ground[1]))
        rng = math.hypot(base_xy[0], base_xy[1])
        if rng > HANDLE_MAX_RANGE_M or base_xy[0] <= .02:
            continue
        reach = reach_band_m[0] <= base_xy[0] <= reach_band_m[1] and abs(base_xy[1]) <= lateral_m
        found.append({'contact_px': [round(v, 1) for v in contact_px], 'area_px': area,
                      'estimated_base_m': [round(v, 4) for v in base_xy], 'range_m': round(rng, 4),
                      'in_reach': bool(reach), 'pixel_bbox': [x, y, w, h]})
    common['handles'] = sorted(found, key=lambda row: -row['area_px'])[:6]
    if not found:
        return _answer('team_cargo_handle', 'unknown', CONFIDENCE['handle_unknown'],
                       'NO_HANDLE_FEATURE_ON_THE_ITEM', observed='not_observed', **common)
    in_reach = [row for row in found if row['in_reach']]
    if in_reach:
        nearest = min(in_reach, key=lambda row: abs(row['estimated_base_m'][1]))
        return _answer('team_cargo_handle', 'yes', CONFIDENCE['handle_in_reach'], 'HANDLE_IN_REACH_BAND',
                       observed='handle_here', nearest=nearest, **common)
    nearest = min(found, key=lambda row: row['range_m'])
    return _answer('team_cargo_handle', 'no', CONFIDENCE['handle_out_of_reach'],
                   'HANDLES_FOUND_ALL_OUTSIDE_REACH', observed='handle_elsewhere', nearest=nearest, **common)


# --------------------------------------------------------------------------- 8. held item, held-check posture

def judge_held_item(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                    candidate_kinds: Sequence[str] = SOLO_KINDS,
                    posture_name: str = 'held_check') -> dict[str, Any]:
    """What am I holding, in the held-check posture?

    Same coverage test as v1's ``judge_holding_item`` (illumination-relative
    per-kind coverage of the held-item region, with a margin over the runner-up);
    what changes is the **posture** the frame must be taken in, and therefore
    which kinds may be reported absent. In the wrist skill's CARRY posture a
    held can is not framed at all, so v1 can only answer ``unknown`` for a can;
    in the held-check posture the can fills the region and both ``yes`` and
    ``no`` become available.

    The gripper command is never evidence of a grasp: only the issued arm pulses
    are read, to check which posture the frame belongs to.
    """
    if expected_kind not in _v1.APPEARANCE:
        raise ValueError(f'unknown cargo kind: {expected_kind}')
    if posture_name not in HELD_POSTURES:
        raise ValueError(f'posture_name must be one of {tuple(HELD_POSTURES)}')
    posture = HELD_POSTURES[posture_name]
    framed = HELD_CHECK_FRAMED_KINDS if posture_name == 'held_check' else _v1.CARRY_FRAMED_KINDS
    frame = _v1._frame(image)
    pose = _v1._pose(servo_pose)
    off = {servo: abs(pose[servo]-value) for servo, value in posture.items() if servo in pose}
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = (int(round(HELD_CHECK_ROI[0]*width)), int(round(HELD_CHECK_ROI[1]*height)),
                      int(round(HELD_CHECK_ROI[2]*width)), int(round(HELD_CHECK_ROI[3]*height)))
    roi = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    reference = _v1._frame_reference(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    kinds = [kind for kind in candidate_kinds if kind in _v1.APPEARANCE]
    if expected_kind not in kinds:
        kinds.append(expected_kind)
    coverage = {}
    for kind in kinds:
        mask = _v1.hue_mask(hsv, kind, reference=reference)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        coverage[kind] = float(mask.astype(bool).mean())
    ranked = sorted(coverage.items(), key=lambda item: -item[1])
    best, best_cover = ranked[0]
    runner = ranked[1][1] if len(ranked) > 1 else 0.
    common = {'expected_kind': expected_kind, 'posture_name': posture_name,
              'posture_pwm': {str(k): int(v) for k, v in posture.items()},
              'coverage': {k: round(v, 4) for k, v in coverage.items()},
              'roi_fraction': list(HELD_CHECK_ROI), 'posture_offset_pwm': off,
              'posture_ok': all(v <= _v1.POSTURE_TOLERANCE_PWM for v in off.values()),
              'framed_kinds': list(framed)}
    if not common['posture_ok']:
        return _answer('held_item', 'unknown', CONFIDENCE['held_unknown'], 'NOT_IN_' + posture_name.upper()
                       + '_POSTURE', observed='not_observed', **common)
    required = _v1.HOLD_MIN_COVERAGE[_v1.APPEARANCE[best]['shape']]
    decisive = best_cover >= required and best_cover >= _v1.HOLD_MARGIN*max(runner, 1e-6)
    if decisive and best == expected_kind:
        return _answer('held_item', 'yes', CONFIDENCE['held_kind_matched'], 'EXPECTED_KIND_FILLS_VIEW',
                       observed=expected_kind, **common)
    if decisive:
        return _answer('held_item', 'no', CONFIDENCE['held_kind_conflict'], 'OTHER_KIND_FILLS_VIEW',
                       observed=best, **common)
    if expected_kind not in framed:
        return _answer('held_item', 'unknown', CONFIDENCE['held_unknown'],
                       'KIND_NOT_FRAMED_BY_POSTURE', observed='not_observed', **common)
    if best_cover <= _v1.HOLD_EMPTY_MAX:
        # Absence of the *expected* kind, not "nothing held": in the held-check
        # posture a held can covers about a quarter of the region, while a held box
        # or tile shows nothing at all, so an empty region proves only that the
        # framed kind is not there. The region must be lit before that is claimed
        # (a black frame is not evidence), and v1's whole-frame edge test is not
        # used here because the raised posture also sees the arena behind the
        # item, where cargo colour is normal.
        roi_value = float(np.percentile(hsv[..., 2], _v1.FRAME_PERCENTILE))
        common['roi_value_percentile'] = round(roi_value, 1)
        common['roi_value_floor'] = round(HELD_ROI_MIN_VALUE*reference[1], 1)
        if roi_value < max(_v1.ABS_MIN_VALUE, HELD_ROI_MIN_VALUE*reference[1]):
            return _answer('held_item', 'unknown', CONFIDENCE['held_unknown'], 'HELD_REGION_TOO_DARK',
                           observed='not_observed', **common)
        if posture_name == 'carry':
            edge = _v1._edge_coverage(frame, (x0, y0, x1, y1), kinds, reference)
            common['edge_coverage'] = {k: round(v, 4) for k, v in edge.items()}
            if max(edge.values(), default=0.) >= _v1.HOLD_EDGE_MIN:
                return _answer('held_item', 'unknown', CONFIDENCE['held_unknown'],
                               'CARGO_COLOUR_ONLY_AT_FRAME_EDGE', observed='not_observed', **common)
        return _answer('held_item', 'no', CONFIDENCE['held_empty'], 'EXPECTED_KIND_ABSENT_FROM_REGION',
                       observed='expected_kind_absent', **common)
    return _answer('held_item', 'unknown', CONFIDENCE['held_unknown'], 'HELD_ITEM_AMBIGUOUS',
                   observed='not_observed', **common)


__all__ = ['SCHEMA', 'PROFILE', 'ANSWERS', 'JUDGMENTS', 'PROVENANCE', 'TEAM_KINDS', 'SOLO_KINDS',
           'HELD_CHECK_POSTURE', 'HELD_CHECK_ROI', 'HELD_POSTURES', 'HELD_CHECK_FRAMED_KINDS',
           'APPROACH_LOOK_POSTURE', 'HANDLE_POSTURES',
           'PEER_ORANGE_HUE', 'TEAM_MASK', 'HANDLE_REACH_BAND_M', 'team_mask',
           'robot_appearance_stats', 'judge_peer_in_lane', 'judge_team_cargo_identity',
           'judge_team_cargo_handle', 'judge_held_item']
