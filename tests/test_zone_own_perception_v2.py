"""Tests for the v2 wrist-RGB-only zone judgments and their decision layer.

Synthetic frames only: these check the input boundary, the answer contract, the
``unknown`` policy, the appearance discriminations and that v1 is untouched.
Detector accuracy is measured offline against rendered frames
(``scripts/eval_zone_own_perception_v2.py``,
``experiments/2026-09-26-zone-own-perception-v2``), not here.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_own_outcome as outcome_v1
from harness import zone_own_outcome_v2 as outcome
from harness import zone_own_perception as v1
from harness import zone_own_perception_v2 as v2

LOOK = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
CARRY = {1: 1500, 6: 1500, **v1.CARRY_POSTURE}   # the issued pose, servos 1 and 3-6
HELD_CHECK = dict(v2.HELD_CHECK_POSTURE)
SIZE = (640, 480)
STATIC_MAP = {
    'map_id': 'zone_wide_door', 'version': 1, 'bounds_m': [-1.05, 5.4, -3.15, 1.45],
    'obstacles': [{'id': 'wall_divider_1', 'center_m': [2.0, -1.5], 'half_extents_m': [0.025, 1.0],
                   'height_m': 0.10, 'kind': 'wall'}],
    'passages': [{'id': 'door_1', 'kind': 'door', 'center_m': [2.0, 0.05], 'half_extents_m': [0.065, 0.28],
                  'width_m': 0.56, 'lanes': 1, 'axis': 'x',
                  'connects': ['pickup side (west)', 'zone side (east)']}],
    'regions': {'zone_B': {'center_m': [4.6, -2.1], 'half_extents_m': [0.3, 0.7]}},
    'zone_slots': {'B': [{'slot_id': 'B2', 'center_m': [4.6, -2.1], 'half_extents_m': [0.06, 0.06]}]},
}
BELIEF = {'x_m': 1.40, 'y_m': 0.05, 'yaw_rad': 0.0, 'confidence': 'high'}
# Catalogue paints as BGR (sim/zone_cargo.py rgba x 255), and the robot materials.
LIME = (31, 199, 140)           # long_beam  .55 .78 .12
PINK = (184, 148, 255)          # heavy_crate 1.0 .58 .72
CREAM = (168, 224, 242)         # tri_frame  .95 .88 .66
BLACK_LUG = (15, 15, 15)        # HANDLE_RGBA .06 .06 .06
ROBOT_ORANGE = (8, 153, 242)    # material orange .95 .60 .03
ROBOT_GREY = (133, 128, 122)    # material aluminum .48 .50 .52
ROBOT_DARK = (17, 15, 14)       # material dark .055 .060 .068
BROWN_BLOCK = (54, 71, 92)      # unmapped_block .36 .28 .21
# The arena floor is the dark checker of the scene (rgb1 .20 .22 .24 / rgb2 .27
# .29 .31); the peer tests need it, because an aluminium panel against a floor of
# nearly the same brightness is not a non-floor component at all.
ARENA_FLOOR = (58, 56, 52)


# --------------------------------------------------------------------- helpers

def _optics(pose):
    from harness import markerless_box as mb
    origin, axes = mb.camera_extrinsics(pose)
    k = mb.scaled_camera_matrix(*SIZE)
    d = np.asarray(mb.CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    return np.asarray(origin, np.float64), np.asarray(axes, np.float64), k, d


def _floor_frame(pose, floor_bgr=(120, 118, 115)):
    frame = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    frame[:, :] = (35, 33, 32)
    origin, axes, k, d = _optics(pose)
    ys, xs = np.mgrid[0:SIZE[1], 0:SIZE[0]]
    pixels = np.column_stack((xs.ravel(), ys.ravel())).astype(np.float64)
    undistorted = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    optical = np.column_stack((undistorted, np.ones(len(undistorted))))
    direction = optical @ axes
    hit = direction[:, 2] < -0.02
    flat = frame.reshape(-1, 3)
    flat[hit] = floor_bgr
    return flat.reshape(SIZE[1], SIZE[0], 3)


def _floor_rect(frame, pose, base_xy, half_x, half_y, bgr):
    """Paint an axis-aligned floor rectangle centred on a base-frame point."""
    from harness import markerless_box as mb
    origin, axes, k, d = _optics(pose)
    corners = [(base_xy[0]-half_x, base_xy[1]-half_y, 0.), (base_xy[0]+half_x, base_xy[1]-half_y, 0.),
               (base_xy[0]+half_x, base_xy[1]+half_y, 0.), (base_xy[0]-half_x, base_xy[1]+half_y, 0.)]
    pixels = mb._project_points(np.asarray(corners, np.float64), origin, axes, k, d)
    assert pixels is not None, 'test geometry must project'
    cv2.fillPoly(frame, [pixels.round().astype(np.int32)], bgr)
    return frame


def _jpeg(frame):
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    assert ok
    return buf.tobytes()


def _observation(judgment, answer, confidence=0.9):
    return {'judgment': judgment, 'answer': answer, 'confidence': confidence,
            'reason': 'TEST', 'observed': 'x'}


def _standing_patch(frame, pose, base_xy, width_m, z_from, z_to, bgr):
    """A vertical slab band standing on the floor, so v1's height gate accepts it."""
    from harness import markerless_box as mb
    origin, axes, k, d = _optics(pose)
    corners = [(base_xy[0], base_xy[1]-width_m/2, z_from), (base_xy[0], base_xy[1]+width_m/2, z_from),
               (base_xy[0], base_xy[1]+width_m/2, z_to), (base_xy[0], base_xy[1]-width_m/2, z_to)]
    pixels = mb._project_points(np.asarray(corners, np.float64), origin, axes, k, d)
    assert pixels is not None, 'test geometry must project'
    cv2.fillPoly(frame, [pixels.round().astype(np.int32)], bgr)
    return pixels


# --------------------------------------------------------------------- contract

def test_v2_answer_contract_and_schema():
    frame = _jpeg(_floor_frame(LOOK))
    rows = [
        v2.judge_peer_in_lane(frame, CARRY, static_map=STATIC_MAP, pose_belief=BELIEF,
                              passage_id='door_1'),
        v2.judge_team_cargo_identity(frame, LOOK, expected_kind='long_beam'),
        v2.judge_team_cargo_handle(frame, LOOK, expected_kind='long_beam'),
        v2.judge_held_item(frame, HELD_CHECK, expected_kind='can'),
    ]
    for row in rows:
        assert row['schema'] == v2.SCHEMA and row['profile'] == v2.PROFILE
        assert row['judgment'] in v2.JUDGMENTS
        assert row['answer'] in v2.ANSWERS
        assert 0. <= row['confidence'] <= 1.
        assert row['reason'] and row['provenance'] == v2.PROVENANCE
    assert {row['judgment'] for row in rows} == set(v2.JUDGMENTS)


def test_unknown_kinds_and_postures_are_refused():
    frame = _jpeg(_floor_frame(LOOK))
    with pytest.raises(ValueError):
        v2.judge_team_cargo_identity(frame, LOOK, expected_kind='banana')
    with pytest.raises(ValueError):
        v2.judge_team_cargo_identity(frame, LOOK, expected_kind='can')      # a solo kind
    with pytest.raises(ValueError):
        v2.judge_team_cargo_handle(frame, LOOK, expected_kind='tile')
    with pytest.raises(ValueError):
        v2.judge_held_item(frame, HELD_CHECK, expected_kind='can', posture_name='somewhere')
    with pytest.raises(ValueError):
        v2.judge_held_item(frame, {3: 1500, 4: 1700}, expected_kind='can')  # servos 3-6 required


# --------------------------------------------------------------------- 5. peer or object

def test_robot_paint_signature_separates_a_peer_from_a_brown_block():
    """The appearance signature alone, on two synthetic regions of equal size."""
    peer = np.zeros((120, 120, 3), np.uint8)
    peer[:, :] = ROBOT_DARK
    peer[10:40, 10:110] = ROBOT_GREY
    peer[45:70, 20:100] = ROBOT_ORANGE
    peer[80:100, 30:90] = ROBOT_ORANGE
    block = np.zeros((120, 120, 3), np.uint8)
    block[:, :] = BROWN_BLOCK
    region = np.ones((120, 120), bool)
    for image, expect_peer in ((peer, True), (block, False)):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        stats = v2.robot_appearance_stats(hsv, v1._frame_reference(hsv), region)
        if expect_peer:
            assert stats['orange_share'] >= v2.PEER_ORANGE_SHARE_MIN
            assert stats['orange_parts'] >= 1
            assert stats['grey_share'] >= v2.PEER_GREY_SHARE_MIN
            assert v2._verdict(stats) == 'peer_robot'
        else:
            assert stats['orange_share'] <= v2.PEER_OBJECT_ORANGE_MAX
            assert stats['body_share'] <= v2.PEER_OBJECT_BODY_MAX
            assert v2._verdict(stats) == 'unmapped_object'


def test_peer_in_lane_reports_a_robot_and_an_object_differently():
    lane = dict(STATIC_MAP)
    peer_frame = _floor_frame(CARRY, floor_bgr=ARENA_FLOOR)
    # A MasterPi silhouette in the three materials it is painted in, in roughly
    # the proportions the model gives: a dark chassis base, the orange rollers and
    # arm plates above it, and the aluminium side panel and electronics cover over
    # most of the body.
    _standing_patch(peer_frame, CARRY, (.75, 0.), .16, 0., .020, ROBOT_DARK)
    _standing_patch(peer_frame, CARRY, (.75, 0.), .16, .020, .045, ROBOT_ORANGE)
    _standing_patch(peer_frame, CARRY, (.75, 0.), .16, .045, .115, ROBOT_GREY)
    peer = v2.judge_peer_in_lane(_jpeg(peer_frame), CARRY, static_map=lane, pose_belief=BELIEF,
                                 passage_id='door_1')
    assert (peer['answer'], peer['observed']) == ('yes', 'peer_robot')
    assert peer['reason'] == 'ROBOT_PAINT_SIGNATURE_IN_LANE'

    object_frame = _floor_frame(CARRY, floor_bgr=ARENA_FLOOR)
    _standing_patch(object_frame, CARRY, (.75, 0.), .16, 0., .120, BROWN_BLOCK)
    obj = v2.judge_peer_in_lane(_jpeg(object_frame), CARRY, static_map=lane, pose_belief=BELIEF,
                                passage_id='door_1')
    assert (obj['answer'], obj['observed']) == ('no', 'unmapped_object')
    assert obj['reason'] == 'NO_ROBOT_PAINT_ON_CANDIDATE'


def test_peer_in_lane_on_a_clear_lane_is_no_peer_not_unknown():
    row = v2.judge_peer_in_lane(_jpeg(_floor_frame(CARRY)), CARRY, static_map=STATIC_MAP,
                                pose_belief=BELIEF, passage_id='door_1')
    assert (row['answer'], row['observed']) == ('no', 'clear')
    assert row['route_blockage']['answer'] == 'no'


def test_peer_in_lane_keeps_the_v1_blockage_answer_next_to_its_own():
    row = v2.judge_peer_in_lane(_jpeg(_floor_frame(CARRY)), CARRY, static_map=STATIC_MAP,
                                pose_belief=BELIEF, passage_id='door_1')
    assert set(row['route_blockage']) == {'answer', 'confidence', 'reason', 'observed'}
    assert 'dropped_components' in row and 'candidates' in row


# --------------------------------------------------------------------- 6. team cargo identity

def test_team_masks_separate_the_three_kinds():
    for kind, bgr in (('long_beam', LIME), ('heavy_crate', PINK), ('tri_frame', CREAM)):
        patch = np.zeros((40, 40, 3), np.uint8)
        patch[:, :] = bgr
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        reference = v1._frame_reference(hsv)
        for other in v2.TEAM_MASK:
            share = float(v2.team_mask(hsv, other, reference).astype(bool).mean())
            if other == kind:
                assert share > .9, (kind, other, share)
            else:
                assert share < .05, (kind, other, share)


def test_tri_frame_mask_rejects_the_robot_orange_and_the_yellow_box():
    """The pale cream shares its hue band with two saturated paints; the
    saturation window is what separates them."""
    for bgr in (ROBOT_ORANGE, (13, 184, 242)):        # orange material, yellow box paint
        patch = np.zeros((40, 40, 3), np.uint8)
        patch[:, :] = bgr
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        share = float(v2.team_mask(hsv, 'tri_frame', v1._frame_reference(hsv)).astype(bool).mean())
        assert share < .05, (bgr, share)


def test_team_cargo_identity_reports_the_kind_and_a_conflict():
    frame = _floor_frame(LOOK)
    _floor_rect(frame, LOOK, (.60, 0.), .30, .02, LIME)          # a 0.60 m bar
    row = v2.judge_team_cargo_identity(_jpeg(frame), LOOK, expected_kind='long_beam')
    assert (row['answer'], row['observed']) == ('yes', 'long_beam')
    other = v2.judge_team_cargo_identity(_jpeg(frame), LOOK, expected_kind='heavy_crate')
    assert (other['answer'], other['observed']) == ('no', 'long_beam')
    assert other['reason'] == 'OTHER_TEAM_KIND_SEEN'


def test_team_cargo_identity_is_unknown_without_colour_and_on_a_wrong_silhouette():
    empty = v2.judge_team_cargo_identity(_jpeg(_floor_frame(LOOK)), LOOK, expected_kind='long_beam')
    assert (empty['answer'], empty['reason']) == ('unknown', 'NO_TEAM_CARGO_COLOUR_IN_VIEW')
    # A long thin bar in the crate's pink: the colour says heavy_crate and the
    # silhouette says otherwise, so nothing is identified.
    thin = _floor_frame(LOOK)
    _floor_rect(thin, LOOK, (.70, 0.), .32, .012, PINK)
    row = v2.judge_team_cargo_identity(_jpeg(thin), LOOK, expected_kind='heavy_crate')
    assert row['answer'] == 'unknown' and row['reason'] == 'SHAPE_DISAGREES_TOO_ELONGATED'


# --------------------------------------------------------------------- 7. my handle is here

def test_team_cargo_handle_in_reach_and_out_of_reach():
    near = _floor_frame(LOOK)
    _floor_rect(near, LOOK, (.62, 0.), .22, .02, LIME)           # a beam reaching towards the robot
    _floor_rect(near, LOOK, (.43, 0.), .02, .021, BLACK_LUG)     # grip band inside the reach band
    row = v2.judge_team_cargo_handle(_jpeg(near), LOOK, expected_kind='long_beam')
    assert (row['answer'], row['observed']) == ('yes', 'handle_here')
    assert v2.HANDLE_REACH_BAND_M[0] <= row['nearest']['estimated_base_m'][0] <= v2.HANDLE_REACH_BAND_M[1]

    far = _floor_frame(LOOK)
    _floor_rect(far, LOOK, (.85, 0.), .25, .02, LIME)
    _floor_rect(far, LOOK, (.85, 0.), .02, .021, BLACK_LUG)      # the same band, out of reach
    out = v2.judge_team_cargo_handle(_jpeg(far), LOOK, expected_kind='long_beam')
    assert (out['answer'], out['observed']) == ('no', 'handle_elsewhere')
    assert out['reason'] == 'HANDLES_FOUND_ALL_OUTSIDE_REACH'


def test_team_cargo_handle_is_unknown_without_the_item_or_without_a_handle():
    nothing = v2.judge_team_cargo_handle(_jpeg(_floor_frame(LOOK)), LOOK, expected_kind='heavy_crate')
    assert (nothing['answer'], nothing['reason']) == ('unknown', 'KIND_NOT_IDENTIFIED_HERE')
    bare = _floor_frame(LOOK)
    _floor_rect(bare, LOOK, (.60, 0.), .07, .05, PINK)           # a crate body with no lug in view
    row = v2.judge_team_cargo_handle(_jpeg(bare), LOOK, expected_kind='heavy_crate')
    assert row['answer'] == 'unknown'
    assert row['reason'] in ('NO_HANDLE_FEATURE_ON_THE_ITEM', 'KIND_NOT_IDENTIFIED_HERE')


# --------------------------------------------------------------------- 8. held item

def test_held_check_posture_matches_its_recorded_geometry():
    """The posture is a measured artefact: keep its own forward kinematics honest."""
    from harness.visual_arm import forward_grip, tool_pose
    x, y, z = forward_grip(v2.HELD_CHECK_POSTURE)
    pitch = tool_pose(v2.HELD_CHECK_POSTURE).pitch_deg
    assert (round(x, 3), round(y, 3), round(z, 3)) == (.143, 0., .299)
    assert round(pitch, 1) == 43.9
    assert z - .024 > .012, 'the held can must clear the floor'
    # ... and the CARRY posture it replaces looks 30 degrees the other way.
    assert round(tool_pose(CARRY).pitch_deg, 1) == -30.1


def test_held_item_answers_a_can_in_the_held_check_posture():
    filled = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    filled[:, :] = (60, 58, 55)
    filled[60:420, 120:520] = (235, 56, 133)                     # can violet .52 .22 .92
    held = v2.judge_held_item(_jpeg(filled), HELD_CHECK, expected_kind='can')
    assert (held['answer'], held['observed']) == ('yes', 'can')
    assert held['posture_ok'] is True and held['posture_name'] == 'held_check'


def test_held_item_claims_absence_only_for_the_framed_kind():
    empty = np.full((SIZE[1], SIZE[0], 3), 108, np.uint8)
    absent = v2.judge_held_item(_jpeg(empty), HELD_CHECK, expected_kind='can')
    assert (absent['answer'], absent['observed']) == ('no', 'expected_kind_absent')
    assert absent['reason'] == 'EXPECTED_KIND_ABSENT_FROM_REGION'
    # A box is not framed by this posture (dev: 0 px anywhere), so no claim is made.
    box = v2.judge_held_item(_jpeg(empty), HELD_CHECK, expected_kind='cyan')
    assert (box['answer'], box['reason']) == ('unknown', 'KIND_NOT_FRAMED_BY_POSTURE')
    assert v2.HELD_CHECK_FRAMED_KINDS == ('can',)
    dark = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    unlit = v2.judge_held_item(_jpeg(dark), HELD_CHECK, expected_kind='can')
    assert (unlit['answer'], unlit['reason']) == ('unknown', 'HELD_REGION_TOO_DARK')


def test_held_item_posture_gate_and_the_carry_comparison():
    filled = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    filled[:, :] = (60, 58, 55)
    filled[60:420, 120:520] = (190, 150, 30)                     # cyan box filling the region
    off = v2.judge_held_item(_jpeg(filled), LOOK, expected_kind='cyan')
    assert (off['answer'], off['reason']) == ('unknown', 'NOT_IN_HELD_CHECK_POSTURE')
    carry = v2.judge_held_item(_jpeg(filled), CARRY, expected_kind='cyan', posture_name='carry')
    assert (carry['answer'], carry['observed']) == ('yes', 'cyan')
    # In CARRY a can is still not framed - the v1 finding this posture answers.
    empty = np.full((SIZE[1], SIZE[0], 3), 108, np.uint8)
    can_in_carry = v2.judge_held_item(_jpeg(empty), CARRY, expected_kind='can', posture_name='carry')
    assert (can_in_carry['answer'], can_in_carry['reason']) == ('unknown', 'KIND_NOT_FRAMED_BY_POSTURE')
    assert 'can' not in v1.CARRY_FRAMED_KINDS


# --------------------------------------------------------------------- tracker

def test_tracker_v2_accepts_the_v2_names_and_keeps_the_v1_policy():
    for judgment in v2.JUDGMENTS:
        tracker = outcome.JudgmentTrackerV2(judgment)
        assert tracker.judgment == judgment
        assert (tracker.cadence_s, tracker.stable_ticks, tracker.commit_confidence) == (
            outcome_v1.CADENCE_S, outcome_v1.STABLE_TICKS, outcome_v1.COMMIT_CONFIDENCE)
        first = tracker.feed(_observation(judgment, 'yes'), 0.)
        assert first['status'] == outcome.UNCONFIRMED and first['answer'] == 'unknown'
        second = tracker.feed(_observation(judgment, 'yes'), 1.)
        assert second['status'] == outcome.CONFIRMED and second['answer'] == 'yes'
        assert second['schema'] == outcome.SCHEMA
    with pytest.raises(ValueError):
        outcome.JudgmentTrackerV2('not_a_judgment')
    for judgment in outcome_v1.JUDGMENTS:
        assert outcome.JudgmentTrackerV2(judgment).judgment == judgment


def test_tracker_v2_never_turns_unknown_into_no_and_keeps_the_cadence():
    decision = outcome.track('peer_in_lane', [_observation('peer_in_lane', 'unknown', 0.)]*6)
    assert decision['status'] == outcome.UNCONFIRMED and decision['answer'] == 'unknown'
    assert decision['unknown_ticks'] == 6
    tracker = outcome.JudgmentTrackerV2('held_item')
    tracker.feed(_observation('held_item', 'yes'), 0.)
    with pytest.raises(ValueError):
        tracker.feed(_observation('held_item', 'yes'), 0.3)
    with pytest.raises(ValueError):
        tracker.feed(_observation('peer_in_lane', 'yes'), 2.)


def test_tracker_v2_low_confidence_does_not_commit():
    decision = outcome.track('team_cargo_identity',
                             [_observation('team_cargo_identity', 'yes', .4)]*4)
    assert decision['status'] == outcome.UNCONFIRMED and decision['answer'] == 'unknown'


# --------------------------------------------------------------------- v1 untouched

V1_PREREGISTERED = {
    'CARRY_POSTURE': {3: 777, 4: 2053, 5: 1646}, 'POSTURE_TOLERANCE_PWM': 90,
    'SLOT_MAX_RANGE_M': 1.20, 'SLOT_MATCH_M': .10, 'BARE_SHARE_MIN': .82,
    'PAINT_BARE_SHARE_MIN': .88, 'CONTRAST_SHARE_MIN': .25, 'CONTRAST_MIN_DELTA': 38,
    'CONTRAST_MAX_DELTA_MIN': 55, 'CARRY_ROI': (.18, .10, .82, .92), 'HOLD_MARGIN': 1.6,
    'HOLD_EMPTY_MAX': .05, 'LANE_HALF_WIDTH_M': .22, 'CLEAR_MIN_RANGE_M': .90,
    'MIN_BLOCK_AREA_PX': 260, 'MIN_OBSTACLE_HEIGHT_M': .06, 'HEIGHT_GATE_FRACTION': .55,
    'MAP_MATCH_M': .18,
}


def test_v1_preregistered_constants_are_unchanged_by_v2():
    """v2 adds judgments; it must not move a single v1 threshold."""
    for name, value in V1_PREREGISTERED.items():
        assert getattr(v1, name) == value, name
    assert v1.JUDGMENTS == ('slot_item', 'holding_item', 'placed_in_slot', 'route_blockage')
    assert v1.SCHEMA == 'ugrp.zone_own_perception.v1'
    assert outcome_v1.SCHEMA == 'ugrp.zone_own_outcome.v1'
    assert v1.CARRY_FRAMED_KINDS == v1.BOX_KINDS + ('tile',)


def test_v1_judgments_answer_the_same_with_and_without_v2_imported():
    """The same frame through v1 alone and through an interpreter that also
    imported v2: identical answer, reason and confidence."""
    frame = _floor_frame(LOOK)
    _floor_rect(frame, LOOK, (.52, 0.), .035, .035, (40, 40, 220))
    jpeg = _jpeg(frame)
    direct = v1.judge_slot_item(jpeg, LOOK, slot_base_xy=(.52, 0.), expected_kind='cyan')
    script = '''
import json, sys
sys.path.insert(0, %r)
import numpy as np, cv2
from harness import zone_own_perception_v2 as v2      # imported first on purpose
from harness import zone_own_perception as v1
data = np.frombuffer(open(%r, 'rb').read(), np.uint8)
row = v1.judge_slot_item(bytes(data.tobytes()), {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500},
                         slot_base_xy=(.52, 0.), expected_kind='cyan')
print(json.dumps({k: row[k] for k in ('answer', 'reason', 'confidence', 'observed')}))
'''
    tmp = ROOT/'outputs'/'test_v1_unchanged.jpg'
    tmp.parent.mkdir(exist_ok=True)
    tmp.write_bytes(jpeg)
    try:
        proc = subprocess.run([sys.executable, '-c', script % (str(ROOT), str(tmp))],
                              capture_output=True, text=True, cwd=str(ROOT))
        assert proc.returncode == 0, proc.stderr
        got = json.loads(proc.stdout.strip().splitlines()[-1])
    finally:
        tmp.unlink(missing_ok=True)
    assert got == {k: direct[k] for k in ('answer', 'reason', 'confidence', 'observed')}


# --------------------------------------------------------------------- input boundary

RUNTIME_MODULES = ('harness.zone_own_perception_v2', 'harness.zone_own_outcome_v2')


def test_v2_runtime_judgments_never_import_mujoco_or_read_sim_state():
    """A fresh interpreter: import the v2 runtime modules, run every judgment, and
    assert that no simulator module was pulled in."""
    script = '''
import sys, json
import numpy as np, cv2
sys.path.insert(0, %r)
from harness import zone_own_outcome_v2 as outcome
from harness import zone_own_perception_v2 as perception
frame = np.full((480, 640, 3), 110, np.uint8)
ok, buf = cv2.imencode('.jpg', frame)
jpeg = buf.tobytes()
look = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
carry = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
held = dict(perception.HELD_CHECK_POSTURE)
static = {'obstacles': [], 'passages': []}
belief = {'x_m': 0., 'y_m': 0., 'yaw_rad': 0., 'confidence': 'high'}
perception.judge_peer_in_lane(jpeg, carry, static_map=static, pose_belief=belief)
perception.judge_team_cargo_identity(jpeg, look, expected_kind='long_beam')
perception.judge_team_cargo_handle(jpeg, look, expected_kind='heavy_crate')
perception.judge_held_item(jpeg, held, expected_kind='can')
perception.judge_held_item(jpeg, carry, expected_kind='cyan', posture_name='carry')
outcome.track('peer_in_lane', [{'judgment': 'peer_in_lane', 'answer': 'yes', 'confidence': .9}]*2)
banned = sorted(m for m in sys.modules if m == 'mujoco' or m.startswith(('mujoco.', 'sim.zone_scene',
                'sim.zone_cargo_scene', 'sim.multi_masterpi', 'sim.masterpi_production',
                'sim.session_scenes')))
print(json.dumps(banned))
''' % str(ROOT)
    proc = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout.strip().splitlines()[-1]) == []


def test_v2_runtime_sources_do_not_mention_sim_state_or_top_inputs():
    banned = ('import mujoco', 'mj_forward', 'detect_top', '.xpos', '.qpos', 'top_camera',
              'segmentation', 'world.data', 'world.model')
    for name in RUNTIME_MODULES:
        source = (ROOT/(name.replace('.', '/')+'.py')).read_text()
        body = '\n'.join(line for line in source.splitlines() if not line.lstrip().startswith('#'))
        _, _, rest = body.partition('"""')
        _, _, code = rest.partition('"""')
        for token in banned:
            assert token not in code, f'{name} must not reference {token} in code'


def test_appearance_tables_agree_with_the_catalogue():
    from sim import zone_cargo
    for kind in v2.TEAM_MASK:
        spec = zone_cargo.kind(kind)
        assert spec.required_carriers >= 2, kind
        box = zone_cargo.bounding_box(spec)
        declared = v1.APPEARANCE[kind]['footprint_m']
        assert math.isclose(max(declared), max(box[0], box[1]), rel_tol=.35), kind
        # every kind keeps at least one black handle part for the handle judgment
        assert any(part.rgba == zone_cargo.HANDLE_RGBA for part in spec.parts), kind
    assert v2.HELD_CHECK_ROI == v1.CARRY_ROI
    assert set(v2.HELD_POSTURES) == {'held_check', 'carry'}
