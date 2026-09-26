"""Tests for the wrist-RGB-only zone judgments and their decision layer.

Synthetic frames only: these check the input boundary, the answer contract and
the ``unknown`` policy. Detector accuracy is measured offline against rendered
frames (``scripts/eval_zone_own_perception.py``,
``experiments/2026-09-26-zone-own-perception``), not here.
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

from harness import zone_own_outcome as outcome
from harness import zone_own_perception as perception

LOOK = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
CARRY = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
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


# --------------------------------------------------------------------- helpers

def _optics(pose):
    from harness import markerless_box as mb
    origin, axes = mb.camera_extrinsics(pose)
    k = mb.scaled_camera_matrix(*SIZE)
    d = np.asarray(mb.CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    return np.asarray(origin, np.float64), np.asarray(axes, np.float64), k, d


def _floor_frame(pose, floor_bgr=(120, 118, 115)):
    """A frame where every pixel that maps to the floor carries ``floor_bgr``."""
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


def _paint_disc(frame, pose, base_xy, radius_m, bgr):
    """Paint a filled floor disc of ``radius_m`` centred on a base-frame point."""
    origin, axes, k, d = _optics(pose)
    ring = [(base_xy[0]+radius_m*math.cos(t), base_xy[1]+radius_m*math.sin(t), 0.)
            for t in np.linspace(0., 2*math.pi, 48, endpoint=False)]
    from harness import markerless_box as mb
    pixels = mb._project_points(np.asarray(ring, np.float64), origin, axes, k, d)
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


# --------------------------------------------------------------------- contract

def test_answer_contract_and_schema():
    frame = _floor_frame(LOOK)
    row = perception.judge_slot_item(_jpeg(frame), LOOK, slot_base_xy=(.52, 0.),
                                     expected_kind='cyan', slot_id='P1-1')
    assert row['schema'] == perception.SCHEMA
    assert row['judgment'] == 'slot_item'
    assert row['answer'] in perception.ANSWERS
    assert 0. <= row['confidence'] <= 1.
    assert row['reason'] and row['provenance'] == perception.PROVENANCE
    assert row['observed'] in ('empty', 'not_observed') + perception.KNOWN_KINDS


def test_unknown_is_first_class_out_of_view_and_out_of_range():
    frame = _jpeg(_floor_frame(LOOK))
    behind = perception.judge_slot_item(frame, LOOK, slot_base_xy=(-.60, 0.), expected_kind='cyan')
    assert behind['answer'] == 'unknown' and behind['observed'] == 'not_observed'
    far = perception.judge_slot_item(frame, LOOK, slot_base_xy=(2.40, 0.), expected_kind='cyan')
    assert far['answer'] == 'unknown'
    assert far['reason'] in ('SLOT_NOT_IN_VIEW', 'SLOT_BEYOND_FIT_RANGE')


def test_empty_slot_needs_positive_bare_evidence():
    """Bare floor at the slot answers ``no``/``empty``; a dark unreadable frame does not."""
    bare = perception.judge_slot_item(_jpeg(_floor_frame(LOOK)), LOOK, slot_base_xy=(.52, 0.),
                                      expected_kind='cyan')
    assert (bare['answer'], bare['observed']) == ('no', 'empty')
    dark = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    unreadable = perception.judge_slot_item(_jpeg(dark), LOOK, slot_base_xy=(.52, 0.), expected_kind='cyan')
    assert unreadable['answer'] in ('unknown', 'no')
    if unreadable['answer'] == 'no':
        assert unreadable['observed'] == 'empty'   # a uniform black floor is still one surface


def test_slot_item_reports_a_conflicting_kind():
    frame = _floor_frame(LOOK)
    _paint_disc(frame, LOOK, (.52, 0.), .035, (40, 40, 220))       # red box paint
    row = perception.judge_slot_item(_jpeg(frame), LOOK, slot_base_xy=(.52, 0.), expected_kind='cyan')
    assert row['answer'] in ('no', 'unknown')
    if row['answer'] == 'no':
        assert row['observed'] != 'cyan'


def test_placed_in_slot_survives_desaturated_cyan_on_blue_paint():
    """The recorded 518 false negative: a dark, desaturated cyan box on zone B paint.

    Both colours are the medians measured on the dev render (ring BGR 48,26,16;
    an occupied disc reaches a max-channel delta of 63 against it).
    """
    frame = _floor_frame(LOOK, floor_bgr=(120, 118, 115))
    _paint_disc(frame, LOOK, (.52, 0.), .20, (48, 26, 16))         # lit zone B blue paint
    _paint_disc(frame, LOOK, (.52, 0.), .030, (110, 84, 44))       # dark, low-sat cyan box face
    row = perception.judge_placed_in_slot(_jpeg(frame), LOOK, slot_base_xy=(.52, 0.),
                                         kind='cyan', zone='B', slot_id='B2')
    assert row['answer'] == 'yes'
    assert row['reason'] in ('DETECTOR_AND_PAINT_CONTRAST_AGREE', 'PAINT_CONTRAST_ONLY', 'DETECTOR_ONLY')
    assert row['reference_ring_is_zone_paint'] is True


def test_placed_in_slot_bare_paint_is_no_and_overlapping_hue_is_unknown(monkeypatch):
    frame = _floor_frame(LOOK)
    _paint_disc(frame, LOOK, (.52, 0.), .20, (48, 26, 16))         # bare zone B paint
    absent = perception.judge_placed_in_slot(_jpeg(frame), LOOK, slot_base_xy=(.52, 0.),
                                            kind='cyan', zone='B')
    assert (absent['answer'], absent['reason']) == ('no', 'SLOT_PROVEN_BARE_PAINT')
    # The measured bands separate cyan (H 84-100) from zone B paint (H 104-120);
    # when a pair *does* overlap, absence must stop being provable.
    assert perception._hue_bands_overlap('cyan', 'B') is False
    monkeypatch.setitem(perception.PAINT_HUE, 'B', (84, 120))
    overlapping = perception.judge_placed_in_slot(_jpeg(frame), LOOK, slot_base_xy=(.52, 0.),
                                                 kind='cyan', zone='B')
    assert (overlapping['answer'], overlapping['reason']) == ('unknown', 'PAINT_HUE_OVERLAPS_KIND')
    assert overlapping['paint_hue_overlaps_kind'] is True


def test_holding_item_posture_gate_and_answers():
    filled = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    filled[:, :] = (60, 58, 55)
    filled[40:440, 100:540] = (190, 150, 30)                       # cyan-ish box filling the view
    held = perception.judge_holding_item(_jpeg(filled), CARRY, expected_kind='cyan')
    assert held['answer'] == 'yes' and held['observed'] == 'cyan'
    assert held['posture_ok'] is True
    wrong = perception.judge_holding_item(_jpeg(filled), CARRY, expected_kind='red')
    assert (wrong['answer'], wrong['observed']) == ('no', 'cyan')
    off_posture = perception.judge_holding_item(_jpeg(filled), LOOK, expected_kind='cyan')
    assert (off_posture['answer'], off_posture['reason']) == ('unknown', 'NOT_IN_CARRY_POSTURE')
    empty = np.full((SIZE[1], SIZE[0], 3), 108, np.uint8)
    nothing = perception.judge_holding_item(_jpeg(empty), CARRY, expected_kind='cyan')
    assert (nothing['answer'], nothing['observed']) == ('no', 'empty')


def test_route_blockage_reports_unmapped_object_and_clears_open_floor():
    belief = {'x_m': 1.40, 'y_m': 0.05, 'yaw_rad': 0.0, 'confidence': 'high'}
    clear = perception.judge_route_blockage(_jpeg(_floor_frame(CARRY)), CARRY, static_map=STATIC_MAP,
                                           pose_belief=belief, passage_id='door_1')
    assert (clear['answer'], clear['observed']) == ('no', 'clear')
    assert clear['lane_clear_range_m'] >= perception.CLEAR_MIN_RANGE_M
    blocked = _floor_frame(CARRY)
    _paint_disc(blocked, CARRY, (.62, 0.), .10, (54, 72, 92))      # unmapped brown crate
    row = perception.judge_route_blockage(_jpeg(blocked), CARRY, static_map=STATIC_MAP,
                                         pose_belief=belief, passage_id='door_1')
    assert (row['answer'], row['observed']) == ('yes', 'unmapped_obstruction')
    assert row['nearest']['mapped_obstacle'] is None


def test_route_blockage_does_not_report_a_mapped_wall():
    """The same patch, but the belief puts its measured contact inside the mapped wall."""
    frame = _floor_frame(CARRY)
    _paint_disc(frame, CARRY, (.62, 0.), .10, (54, 72, 92))
    jpeg = _jpeg(frame)
    probe = perception.judge_route_blockage(jpeg, CARRY, static_map=STATIC_MAP,
                                            pose_belief={'x_m': 0., 'y_m': 0., 'yaw_rad': 0.,
                                                         'confidence': 'high'})
    contact = probe['nearest']['estimated_base_m']
    belief = {'x_m': 2.0-contact[0], 'y_m': -1.5-contact[1], 'yaw_rad': 0.0, 'confidence': 'high'}
    row = perception.judge_route_blockage(jpeg, CARRY, static_map=STATIC_MAP, pose_belief=belief)
    assert row['answer'] != 'yes'
    assert 'wall_divider_1' in row['mapped_in_lane']


def test_low_pose_belief_widens_the_map_slack():
    high = perception.judge_route_blockage(_jpeg(_floor_frame(CARRY)), CARRY, static_map=STATIC_MAP,
                                           pose_belief={'x_m': 0., 'y_m': 0., 'yaw_rad': 0.,
                                                        'confidence': 'high'})
    low = perception.judge_route_blockage(_jpeg(_floor_frame(CARRY)), CARRY, static_map=STATIC_MAP,
                                          pose_belief={'x_m': 0., 'y_m': 0., 'yaw_rad': 0.,
                                                       'confidence': 'low'})
    assert low['map_slack_m'] > high['map_slack_m']


def test_unknown_cargo_kind_and_zone_are_refused():
    frame = _jpeg(_floor_frame(LOOK))
    with pytest.raises(ValueError):
        perception.judge_slot_item(frame, LOOK, slot_base_xy=(.52, 0.), expected_kind='banana')
    with pytest.raises(ValueError):
        perception.judge_placed_in_slot(frame, LOOK, slot_base_xy=(.52, 0.), kind='cyan', zone='D')
    with pytest.raises(ValueError):
        perception.judge_slot_item(frame, {3: 1072, 4: 2400}, slot_base_xy=(.52, 0.), expected_kind='cyan')


def test_hue_mask_is_relative_to_the_frame():
    """Halving brightness and saturation keeps a cyan face inside its own mask."""
    bright = np.zeros((60, 60, 3), np.uint8)
    bright[:, :] = (200, 160, 40)
    dark = (bright.astype(np.float32)*.45).astype(np.uint8)
    for frame in (bright, dark):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        assert perception.hue_mask(hsv, 'cyan').astype(bool).mean() > .9


# --------------------------------------------------------------------- tracker

def test_tracker_commits_only_on_repeated_confident_answers():
    tracker = outcome.JudgmentTracker('placed_in_slot', question={'slot_id': 'B2'})
    first = tracker.feed(_observation('placed_in_slot', 'yes'), 0.)
    assert first['status'] == outcome.UNCONFIRMED and first['answer'] == 'unknown'
    second = tracker.feed(_observation('placed_in_slot', 'yes'), 1.)
    assert second['status'] == outcome.CONFIRMED and second['answer'] == 'yes'
    assert second['committed_at_sim_s'] == 1.


def test_tracker_never_turns_unknown_into_no():
    decision = outcome.track('placed_in_slot', [_observation('placed_in_slot', 'unknown', 0.)]*6)
    assert decision['status'] == outcome.UNCONFIRMED
    assert decision['answer'] == 'unknown' and decision['unknown_ticks'] == 6
    assert decision['reason'] == 'OBSERVATION_BUDGET_SPENT' or decision['exhausted'] is False


def test_tracker_rejects_a_flipped_streak_and_a_broken_cadence():
    tracker = outcome.JudgmentTracker('slot_item')
    tracker.feed(_observation('slot_item', 'yes'), 0.)
    tracker.feed(_observation('slot_item', 'no'), 1.)
    assert tracker.decision is None
    with pytest.raises(ValueError):
        tracker.feed(_observation('slot_item', 'no'), 1.2)
    with pytest.raises(ValueError):
        tracker.feed(_observation('placed_in_slot', 'no'), 3.)


def test_tracker_low_confidence_does_not_commit():
    decision = outcome.track('holding_item', [_observation('holding_item', 'yes', .4)]*4)
    assert decision['status'] == outcome.UNCONFIRMED and decision['answer'] == 'unknown'


def test_tracker_offers_the_agreed_pan_plan():
    assert sorted(outcome.PAN_PLAN) == sorted(perception.PAN_PULSES)
    tracker = outcome.JudgmentTracker('placed_in_slot')
    assert tracker.next_pan_pwm() == 1500          # straight ahead first
    tracker.feed(_observation('placed_in_slot', 'unknown', 0.), 0.)
    assert tracker.next_pan_pwm() == outcome.PAN_PLAN[1]
    for sim_s in (1., 2.):
        tracker.feed(_observation('placed_in_slot', 'unknown', 0.), sim_s)
    assert tracker.next_pan_pwm() is None          # the plan is spent


# --------------------------------------------------------------------- input boundary

RUNTIME_MODULES = ('harness.zone_own_perception', 'harness.zone_own_outcome')


def test_runtime_judgments_never_import_mujoco_or_read_sim_state():
    """A fresh interpreter: import the runtime modules, run every judgment, and
    assert that no simulator module was pulled in."""
    script = '''
import sys, math, json
import numpy as np, cv2
sys.path.insert(0, %r)
from harness import zone_own_outcome as outcome
from harness import zone_own_perception as perception
frame = np.full((480, 640, 3), 110, np.uint8)
ok, buf = cv2.imencode('.jpg', frame)
jpeg = buf.tobytes()
pose = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
carry = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
static = {'obstacles': [], 'passages': []}
perception.judge_slot_item(jpeg, pose, slot_base_xy=(.52, 0.), expected_kind='cyan')
perception.judge_holding_item(jpeg, carry, expected_kind='cyan')
perception.judge_placed_in_slot(jpeg, pose, slot_base_xy=(.52, 0.), kind='cyan', zone='B')
perception.judge_route_blockage(jpeg, carry, static_map=static,
                               pose_belief={'x_m': 0., 'y_m': 0., 'yaw_rad': 0., 'confidence': 'high'})
perception.detect_cargo_own(jpeg, pose)
outcome.track('slot_item', [{'judgment': 'slot_item', 'answer': 'yes', 'confidence': .9}]*2)
banned = sorted(m for m in sys.modules if m == 'mujoco' or m.startswith(('mujoco.', 'sim.zone_scene',
                'sim.multi_masterpi', 'sim.masterpi_production', 'sim.session_scenes')))
print(json.dumps(banned))
''' % str(ROOT)
    proc = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout.strip().splitlines()[-1]) == []


def test_runtime_sources_do_not_mention_sim_state_or_top_inputs():
    banned = ('import mujoco', 'mj_forward', 'detect_top', '.xpos', '.qpos', 'top_camera', 'segmentation')
    for name in RUNTIME_MODULES:
        source = (ROOT/(name.replace('.', '/')+'.py')).read_text()
        body = '\n'.join(line for line in source.splitlines()
                         if not line.lstrip().startswith('#'))
        _, _, rest = body.partition('"""')
        _, _, code = rest.partition('"""')
        for token in banned:
            assert token not in code, f'{name} must not reference {token} in code'


def test_appearance_table_covers_the_solo_cargo_and_the_catalogue_agrees():
    from sim import zone_cargo
    for kind in ('cyan', 'green', 'red', 'yellow', 'can', 'tile'):
        assert kind in perception.APPEARANCE
    for kind in ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame'):
        box = zone_cargo.bounding_box(zone_cargo.kind(kind))
        declared = perception.APPEARANCE[kind]['footprint_m']
        assert math.isclose(max(declared), max(box[0], box[1]), rel_tol=.35), kind
