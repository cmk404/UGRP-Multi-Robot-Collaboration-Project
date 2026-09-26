"""Tests for the v3 wrist-RGB-only judgments (at-grip silhouette, grasp-stage posture).

Synthetic frames only: the input boundary, the answer contract, the ``unknown``
policy, the at-grip geometry and the W7 regression are checked here, and that v1
and v2 stay byte-identical. Detector accuracy is measured offline on rendered
frames (``scripts/eval_zone_own_perception_v3.py``,
``experiments/2026-09-26-zone-own-perception-v3``).
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_own_outcome_v3 as outcome
from harness import zone_own_perception as v1
from harness import zone_own_perception_v2 as v2
from harness import zone_own_perception_v3 as v3

SIZE = (640, 480)
CARRY = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
HELD_CHECK = {1: 1500, **v2.HELD_CHECK_POSTURE}
LOOK = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
# Catalogue paints as BGR, and the measured render colours of the W7 case.
CAN_VIOLET = (235, 56, 133)          # can .52 .22 .92
CYAN_BOX = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[93, 182, 190]]]), cv2.COLOR_HSV2BGR)[0, 0])
PICKUP_PAINT = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[100, 100, 160]]]), cv2.COLOR_HSV2BGR)[0, 0])
TILE_MAGENTA = (148, 41, 219)        # tile .86 .16 .58
PINK = (184, 148, 255)               # heavy_crate 1.0 .58 .72
LIME = (31, 199, 140)                # long_beam .55 .78 .12
LIT_FLOOR = (150, 128, 104)          # bluish arena floor under the ceiling light

# v1/v2 files as committed in fb44c2c (PR #193, v2 test split recorded). v3 must
# not touch a byte of them.
PINNED_SHA256 = {
    'harness/zone_own_perception.py': '93e941da731818dca5ccb6e44273dd1df03ffdd9e42542e262fcc182c34f780e',
    'harness/zone_own_perception_v2.py': 'ef3b1bc2215666c248d259176b7d1d5586cdf1e09f2e615cb85ad7ef9d3fb2b8',
    'harness/zone_own_outcome.py': '49d566e766a423d8f561130f288b83edba756f131187e519926e2b1b65725fb6',
    'harness/zone_own_outcome_v2.py': '0ca83e213d080b8200537e0fce17c498f51505f2575e82f67cc76181aa51cd63',
    'scripts/eval_zone_own_perception.py': '9b0b857ad49692b379955236b4af4f0631ccab5da40a1aee5e82d735efa13080',
    'scripts/eval_zone_own_perception_v2.py': '612c089d5d1dafa74e57a63b9ceb78e5f3e43ac8f40a420772a7bc55547c1c79',
}


def _frame(bgr=LIT_FLOOR):
    frame = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    frame[:, :] = bgr
    return frame


def _paint(frame, mask, bgr):
    frame[mask] = bgr
    return frame


def _jpeg(frame):
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    assert ok
    return buf.tobytes()


def _sil(kind, pose, **kw):
    return v3.held_silhouette(kind, pose, SIZE, **kw)[0]


# --------------------------------------------------------------------- v1/v2 untouched

@pytest.mark.parametrize('path', sorted(PINNED_SHA256))
def test_v1_and_v2_files_are_byte_identical(path):
    digest = hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    assert digest == PINNED_SHA256[path], f'{path} changed; v3 must add, never edit'


def test_v1_and_v2_answer_the_same_with_and_without_v3_imported():
    frame = _paint(_frame(), _sil('can', HELD_CHECK), CAN_VIOLET)
    jpeg = _jpeg(frame)
    direct = [v2.judge_held_item(jpeg, HELD_CHECK, expected_kind='can'),
              v1.judge_holding_item(jpeg, CARRY, expected_kind='cyan')]
    script = '''
import json, sys
sys.path.insert(0, %r)
from harness import zone_own_perception_v3 as v3     # imported first on purpose
from harness import zone_own_perception_v2 as v2
from harness import zone_own_perception as v1
data = open(%r, 'rb').read()
rows = [v2.judge_held_item(data, %r, expected_kind='can'), v1.judge_holding_item(data, %r, expected_kind='cyan')]
print(json.dumps([{k: r[k] for k in ('answer', 'reason', 'confidence', 'observed')} for r in rows]))
'''
    tmp = ROOT/'outputs'/'test_v3_v1v2_unchanged.jpg'
    tmp.parent.mkdir(exist_ok=True)
    tmp.write_bytes(jpeg)
    try:
        proc = subprocess.run([sys.executable, '-c', script % (str(ROOT), str(tmp), HELD_CHECK, CARRY)],
                              capture_output=True, text=True, cwd=str(ROOT))
        assert proc.returncode == 0, proc.stderr
        got = json.loads(proc.stdout.strip().splitlines()[-1])
    finally:
        tmp.unlink(missing_ok=True)
    assert got == [{k: r[k] for k in ('answer', 'reason', 'confidence', 'observed')} for r in direct]


# --------------------------------------------------------------------- geometry

def test_at_grip_silhouette_reproduces_the_measured_framing():
    """The two v2 posture findings follow from FK + near clip alone.

    Dev check against segmentation (eval only): IoU 0.96 for a box in CARRY and
    0.83 for a can in the held-check posture; a box in the held-check posture and
    a tile there are 0 px in both the prediction and the render.
    """
    assert int(_sil('cyan', CARRY).sum()) > 60000          # a held box fills the CARRY view
    assert int(_sil('can', HELD_CHECK).sum()) > 30000      # the held-check posture frames a can
    assert int(_sil('cyan', HELD_CHECK).sum()) == 0        # ... and no box
    assert int(_sil('tile', HELD_CHECK).sum()) == 0
    assert int(_sil('can', CARRY).sum()) < v3.HELD_MIN_SILHOUETTE_PX   # CARRY: a sliver at most
    # without the render near clip the can would appear to fill the CARRY frame
    assert int(_sil('can', CARRY, near_clip_m=0.).sum()) > 200000


def test_a_physical_hold_is_rigid_in_the_gripper():
    """With a grasp pose the silhouette is the one at the grasp, whatever the arm does next."""
    at_grasp = _sil('cyan', CARRY)
    moved = _sil('cyan', HELD_CHECK, grasp_pose=CARRY)
    assert np.array_equal(at_grasp, moved)
    assert not np.array_equal(at_grasp, _sil('cyan', HELD_CHECK))


def test_team_silhouettes_follow_the_grasp_role():
    parts, grip, approach = v3.held_parts('heavy_crate')
    assert grip[0] == pytest.approx(-.10) and approach == 0.     # held by the west lug, not the centroid
    # the lens (0.033 m behind the grip) stays outside the crate body
    body = [p for p in parts if p[1] == (0., 0., .03)][0]
    lens_x = grip[0]-.033
    assert lens_x < body[1][0]-body[2][0]
    for kind in v3.TEAM_KINDS:
        assert int(_sil(kind, CARRY).sum()) >= v3.TEAM_MIN_SILHOUETTE_PX, kind


def test_grasp_look_posture_geometry():
    from harness.visual_arm import forward_grip, tool_pose
    x, y, z = forward_grip(v3.GRASP_LOOK_POSTURE)
    assert (round(x, 2), round(y, 2), round(z, 2)) == (.10, 0., .16)
    assert round(tool_pose(v3.GRASP_LOOK_POSTURE).pitch_deg) == -39


# --------------------------------------------------------------------- held item at grip

def test_held_can_in_the_held_check_posture():
    frame = _paint(_frame((40, 36, 34)), _sil('can', HELD_CHECK), CAN_VIOLET)
    row = v3.judge_held_item(_jpeg(frame), HELD_CHECK, expected_kind='can')
    assert (row['answer'], row['observed'], row['reason']) == ('yes', 'can', 'EXPECTED_KIND_IN_AT_GRIP_BAND')
    assert row['schema'] == v3.SCHEMA and row['confidence'] >= outcome.COMMIT_CONFIDENCE


def test_absence_needs_a_lit_band_and_no_colour():
    absent = v3.judge_held_item(_jpeg(_frame()), HELD_CHECK, expected_kind='can')
    assert (absent['answer'], absent['observed']) == ('no', 'expected_kind_absent')
    dark = v3.judge_held_item(_jpeg(_frame((0, 0, 0))), HELD_CHECK, expected_kind='can')
    assert (dark['answer'], dark['reason']) == ('unknown', 'AT_GRIP_BAND_TOO_DARK')


def test_w7_regression_a_can_out_of_frame_over_pickup_paint_is_unknown():
    """The v2 test failure: CARRY posture, held can not framed, pickup paint in view."""
    frame = _frame(PICKUP_PAINT)
    jpeg = _jpeg(frame)
    old = v2.judge_held_item(jpeg, CARRY, expected_kind='can', posture_name='carry')
    assert (old['answer'], old['observed']) == ('no', 'cyan')      # the recorded v2 confident error
    new = v3.judge_held_item(jpeg, CARRY, expected_kind='can')
    assert (new['answer'], new['reason']) == ('unknown', 'EXPECTED_KIND_NOT_FRAMED_AT_GRIP')


def test_pickup_paint_is_not_a_held_cyan_box():
    """Nothing held, expected cyan, the whole view is lit pickup paint (H 100)."""
    row = v3.judge_held_item(_jpeg(_frame(PICKUP_PAINT)), CARRY, expected_kind='cyan')
    assert row['answer'] != 'yes'
    held = v3.judge_held_item(_jpeg(_paint(_frame(PICKUP_PAINT), _sil('cyan', CARRY), CYAN_BOX)), CARRY,
                              expected_kind='cyan')
    assert (held['answer'], held['observed']) == ('yes', 'cyan')


def test_colour_that_runs_past_the_band_is_not_held():
    """A cyan surface covering the whole view (not the box silhouette) fails containment."""
    row = v3.judge_held_item(_jpeg(_frame(CYAN_BOX)), CARRY, expected_kind='cyan')
    assert row['answer'] == 'unknown'
    assert row['per_kind']['cyan']['containment'] < v3.HELD_CONTAINMENT_MIN


def test_another_kind_at_the_grip():
    frame = _paint(_frame(), _sil('tile', CARRY), TILE_MAGENTA)
    row = v3.judge_held_item(_jpeg(frame), CARRY, expected_kind='cyan')
    assert (row['answer'], row['observed'], row['reason']) == ('no', 'tile', 'OTHER_KIND_IN_ITS_AT_GRIP_BAND')


def test_a_box_in_the_held_check_posture_is_not_framed():
    row = v3.judge_held_item(_jpeg(_frame()), HELD_CHECK, expected_kind='cyan')
    assert (row['answer'], row['reason']) == ('unknown', 'EXPECTED_KIND_NOT_FRAMED_AT_GRIP')


# --------------------------------------------------------------------- team item at grip

def test_team_item_at_grip_by_colour():
    crate = v3.judge_team_cargo_at_grip(_jpeg(_paint(_frame(), _sil('heavy_crate', CARRY), PINK)), CARRY,
                                        expected_kind='heavy_crate')
    assert (crate['answer'], crate['cue'], crate['observed']) == ('yes', 'colour', 'heavy_crate')
    beam = v3.judge_team_cargo_at_grip(_jpeg(_paint(_frame(), _sil('long_beam', CARRY), LIME)), CARRY,
                                       expected_kind='heavy_crate')
    assert (beam['answer'], beam['observed']) == ('no', 'long_beam')


def test_unlit_crate_at_grip_by_shape_and_nothing_at_grip():
    dark = _paint(_frame(), _sil('heavy_crate', CARRY), (10, 9, 9))
    shape = v3.judge_team_cargo_at_grip(_jpeg(dark), CARRY, expected_kind='heavy_crate')
    assert (shape['answer'], shape['cue'], shape['reason']) == ('yes', 'shape', 'EXPECTED_KIND_SHAPE_AT_GRIP')
    assert shape['confidence'] == v3.CONFIDENCE['team_shape_matched']
    other = v3.judge_team_cargo_at_grip(_jpeg(dark), CARRY, expected_kind='long_beam')
    assert (other['answer'], other['observed']) == ('no', 'heavy_crate')
    empty = v3.judge_team_cargo_at_grip(_jpeg(_frame()), CARRY, expected_kind='heavy_crate')
    assert (empty['answer'], empty['observed']) == ('no', 'empty')


def test_team_item_not_framed_and_mixed_evidence_is_unknown():
    row = v3.judge_team_cargo_at_grip(_jpeg(_frame()), HELD_CHECK, expected_kind='heavy_crate')
    assert row['answer'] in ('unknown', 'no')
    if row['answer'] == 'no':
        assert row['observed'] == 'empty'
    half = _frame()
    half[:240] = (10, 9, 9)                                   # a dark band that is no kind's silhouette
    mixed = v3.judge_team_cargo_at_grip(_jpeg(half), CARRY, expected_kind='heavy_crate')
    assert mixed['answer'] == 'unknown'


# --------------------------------------------------------------------- grasp stage

def test_grasp_stage_requires_its_posture_and_wraps_v2():
    frame = _jpeg(_frame())
    off = v3.judge_team_cargo_grasp_stage(frame, LOOK, expected_kind='tri_frame')
    assert (off['answer'], off['reason']) == ('unknown', 'NOT_IN_GRASP_LOOK_POSTURE')
    assert off['identity']['answer'] == off['handle']['answer'] == 'unknown'
    on = v3.judge_team_cargo_grasp_stage(frame, v3.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
    assert on['posture_ok'] is True
    direct = v2.judge_team_cargo_identity(frame, v3.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
    assert on['identity'] == direct


def test_unknown_kinds_are_refused():
    frame = _jpeg(_frame())
    with pytest.raises(ValueError):
        v3.judge_held_item(frame, CARRY, expected_kind='anvil')
    with pytest.raises(ValueError):
        v3.judge_team_cargo_at_grip(frame, CARRY, expected_kind='cyan')
    with pytest.raises(ValueError):
        v3.judge_team_cargo_grasp_stage(frame, CARRY, expected_kind='can')


# --------------------------------------------------------------------- tracker

def test_tracker_v3_accepts_the_v3_names_and_keeps_the_v1_policy():
    obs = [{'judgment': 'held_item_at_grip', 'answer': 'yes', 'confidence': .9, 'reason': 'T', 'observed': 'can'}]
    row = outcome.track('held_item_at_grip', obs*2)
    assert row['status'] == outcome.CONFIRMED and row['answer'] == 'yes'
    assert row['schema'] == outcome.SCHEMA
    unknown = [{'judgment': 'team_cargo_at_grip', 'answer': 'unknown', 'confidence': 0., 'reason': 'T',
                'observed': 'not_observed'}]
    assert outcome.track('team_cargo_at_grip', unknown*3)['answer'] == 'unknown'
    with pytest.raises(ValueError):
        outcome.JudgmentTrackerV3('not_a_judgment')


# --------------------------------------------------------------------- input boundary

RUNTIME_MODULES = ('harness.zone_own_perception_v3', 'harness.zone_own_outcome_v3')


def test_v3_runtime_judgments_never_import_mujoco_or_read_sim_state():
    script = '''
import sys, json
import numpy as np, cv2
sys.path.insert(0, %r)
from harness import zone_own_outcome_v3 as outcome
from harness import zone_own_perception_v3 as perception
frame = np.full((480, 640, 3), 110, np.uint8)
ok, buf = cv2.imencode('.jpg', frame)
jpeg = buf.tobytes()
carry = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
held = {1: 1500, 3: 1500, 4: 1700, 5: 1900, 6: 1500}
perception.judge_held_item(jpeg, held, expected_kind='can')
perception.judge_held_item(jpeg, carry, expected_kind='cyan', grasp_pose=carry)
perception.judge_team_cargo_at_grip(jpeg, carry, expected_kind='heavy_crate')
perception.judge_team_cargo_grasp_stage(jpeg, perception.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
outcome.track('held_item_at_grip', [{'judgment': 'held_item_at_grip', 'answer': 'yes', 'confidence': .9}]*2)
banned = sorted(m for m in sys.modules if m == 'mujoco' or m.startswith(('mujoco.', 'sim.zone_scene',
                'sim.zone_cargo_scene', 'sim.multi_masterpi', 'sim.masterpi_production',
                'sim.session_scenes', 'sim.zone_arena')))
print(json.dumps(banned))
''' % str(ROOT)
    proc = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout.strip().splitlines()[-1]) == []


def test_v3_runtime_sources_do_not_mention_sim_state_or_top_inputs():
    banned = ('import mujoco', 'mj_forward', 'detect_top', '.xpos', '.qpos', 'top_camera',
              'segmentation', 'world.data', 'world.model', 'site_xyz')
    for name in RUNTIME_MODULES:
        source = (ROOT/(name.replace('.', '/')+'.py')).read_text()
        body = '\n'.join(line for line in source.splitlines() if not line.lstrip().startswith('#'))
        _, _, rest = body.partition('"""')
        _, _, code = rest.partition('"""')
        for token in banned:
            assert token not in code, f'{name} must not reference {token} in code'
