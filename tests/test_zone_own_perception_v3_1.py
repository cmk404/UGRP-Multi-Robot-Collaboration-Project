"""Tests for v3.1: no image information, no answer (PR #193 Codex review P1).

Synthetic frames only. Checks that v1, v2 and v3 stay byte-identical, that the
recorded P1 failure of v3 (a BGR 10/10/10 frame in CARRY read as a held
``heavy_crate``) is ``unknown`` in v3.1 and never confirmed by the tracker, that
every v3.1 judgment in every posture answers ``unknown`` on black, near-black
(V 0-39), uniform grey, heavily blurred and lens-covered frames, and that on
informative frames v3.1 answers exactly as v3. Accuracy on rendered frames is
measured offline (``scripts/eval_zone_own_perception_v3_1.py``,
``experiments/2026-09-26-zone-own-perception-v3-1``).
"""
from __future__ import annotations

import hashlib
import json
import random
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
from harness import zone_own_perception_v2 as v2
from harness import zone_own_perception_v3 as v3
from harness import zone_own_perception_v3_1 as v31
from scripts import eval_zone_own_perception_v3_1 as ev31

SIZE = (640, 480)
CARRY = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
HELD_CHECK = {1: 1500, **v2.HELD_CHECK_POSTURE}
LOOK = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
POSES = {'carry': CARRY, 'held_check': HELD_CHECK, 'grasp_look_v3': dict(v3.GRASP_LOOK_POSTURE),
         'approach_look_v2': dict(v2.APPROACH_LOOK_POSTURE), 'look': LOOK}
CAN_VIOLET = (235, 56, 133)
CYAN_BOX = tuple(int(v) for v in cv2.cvtColor(np.uint8([[[93, 182, 190]]]), cv2.COLOR_HSV2BGR)[0, 0])
TILE_MAGENTA = (148, 41, 219)
PINK = (184, 148, 255)
LIME = (31, 199, 140)
LIT_FLOOR = (150, 128, 104)
FLOOR_LINE = (112, 96, 78)

# v1/v2/v3 files as committed in 16d464e (PR #193, v3 test split recorded).
PINNED_SHA256 = {
    'harness/zone_own_perception.py': '93e941da731818dca5ccb6e44273dd1df03ffdd9e42542e262fcc182c34f780e',
    'harness/zone_own_perception_v2.py': 'ef3b1bc2215666c248d259176b7d1d5586cdf1e09f2e615cb85ad7ef9d3fb2b8',
    'harness/zone_own_perception_v3.py': 'fbc29b63ed04446b53a27e4e87cb4d2e0a3f0bd648106c2675a1356d82f6f936',
    'harness/zone_own_outcome.py': '49d566e766a423d8f561130f288b83edba756f131187e519926e2b1b65725fb6',
    'harness/zone_own_outcome_v2.py': '0ca83e213d080b8200537e0fce17c498f51505f2575e82f67cc76181aa51cd63',
    'harness/zone_own_outcome_v3.py': 'aca334b9726259438033b71a1142b7f9fdc2f3a7c5d9bec6e777925bc8a44f91',
    'scripts/eval_zone_own_perception.py': '9b0b857ad49692b379955236b4af4f0631ccab5da40a1aee5e82d735efa13080',
    'scripts/eval_zone_own_perception_v2.py': '612c089d5d1dafa74e57a63b9ceb78e5f3e43ac8f40a420772a7bc55547c1c79',
    'scripts/eval_zone_own_perception_v3.py': '050c0d517ca515014e4d419d2c896d80b79807705bd5bb7745646762d5ae6647',
}


def _floor():
    """Lit arena floor with sharp tile seams: an informative background."""
    frame = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    frame[:, :] = LIT_FLOOR
    frame[::40, :] = FLOOR_LINE
    frame[1::40, :] = FLOOR_LINE
    frame[2::40, :] = FLOOR_LINE
    frame[:, ::40] = FLOOR_LINE
    frame[:, 1::40] = FLOOR_LINE
    frame[:, 2::40] = FLOOR_LINE
    return frame


def _paint(frame, mask, bgr):
    frame[mask] = bgr
    return frame


def _jpeg(frame):
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    assert ok
    return buf.tobytes()


def _sil(kind, pose):
    return v3.held_silhouette(kind, pose, SIZE)[0]


def _informative_scenes():
    return {
        'floor': _floor(),
        'held_cyan_carry': _paint(_floor(), _sil('cyan', CARRY), CYAN_BOX),
        'held_can_check': _paint(_floor(), _sil('can', HELD_CHECK), CAN_VIOLET),
        'carry_crate_pink': _carried_crate(),
    }


def _carried_crate():
    """A lit carried crate fills the CARRY view: pink faces, a shaded face and the black lug bar."""
    frame = _paint(_floor(), _sil('heavy_crate', CARRY), PINK)
    frame[140:, 130:510] = (130, 90, 140)          # the shaded face below the lug
    frame[65:140, :] = (12, 12, 12)                # the lug bar across the view
    return frame


# --------------------------------------------------------------------- v1/v2/v3 untouched

@pytest.mark.parametrize('path', sorted(PINNED_SHA256))
def test_v1_v2_v3_files_are_byte_identical(path):
    digest = hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    assert digest == PINNED_SHA256[path], f'{path} changed; v3.1 must add, never edit'


# --------------------------------------------------------------------- P1 regression

def test_p1_black_frame_carry_crate_is_unknown_and_never_confirmed():
    """The Codex review repro: 640x480 BGR (10,10,10), CARRY pulses, expected heavy_crate."""
    jpeg = _jpeg(np.full((SIZE[1], SIZE[0], 3), 10, np.uint8))
    old = v3.judge_team_cargo_at_grip(jpeg, CARRY, expected_kind='heavy_crate')
    assert (old['answer'], old['confidence'], old['reason']) == ('yes', .7, 'EXPECTED_KIND_SHAPE_AT_GRIP')
    assert outcome.track('team_cargo_at_grip', [old]*2)['status'] == outcome.CONFIRMED   # the recorded defect
    new = v31.judge_team_cargo_at_grip(jpeg, CARRY, expected_kind='heavy_crate')
    assert (new['answer'], new['confidence'], new['reason']) == ('unknown', 0., v31.UNKNOWN_REASON)
    assert new['schema'] == v31.SCHEMA and new['profile'] == v31.PROFILE
    tracked = outcome.track('team_cargo_at_grip', [new]*3)
    assert tracked['status'] != outcome.CONFIRMED and tracked['answer'] == 'unknown'


@pytest.mark.parametrize('level', range(0, 40))
def test_every_near_black_level_is_unknown_for_the_crate_at_grip(level):
    jpeg = _jpeg(np.full((SIZE[1], SIZE[0], 3), level, np.uint8))
    for kind in v31.TEAM_KINDS:
        row = v31.judge_team_cargo_at_grip(jpeg, CARRY, expected_kind=kind)
        assert row['answer'] == 'unknown', (level, kind, row['reason'])


def test_a_crate_that_fills_the_frame_has_no_shape_boundary():
    boundary = v31.shape_boundary(_floor(), 'heavy_crate', CARRY)
    assert boundary['ring_out_px'] == 0 and boundary['evidenced'] is False


# --------------------------------------------------------------------- adversarial set

ADVERSARIAL_BASES = ('floor', 'held_cyan_carry', 'held_can_check', 'carry_crate_pink')


@pytest.mark.parametrize('transform', ev31.INFO_FREE)
@pytest.mark.parametrize('posture', sorted(POSES))
def test_information_free_frames_are_unknown_for_every_judgment_and_posture(transform, posture):
    scenes = _informative_scenes()
    for base in ADVERSARIAL_BASES:
        frame, params = ev31.adversarial_frame(transform, scenes[base], random.Random(f'{transform}:{base}'))
        jpeg = _jpeg(frame)
        info = v31.image_information(jpeg)
        assert info['sufficient'] is False, (transform, base, params, info)
        for judgment, kind, row in ev31._all_judgments(v31, jpeg, POSES[posture]):
            assert row['answer'] == 'unknown', (transform, base, posture, judgment, kind, row['reason'])
            assert row['confidence'] == 0.


def test_information_free_series_is_never_confirmed_by_the_tracker():
    scenes = _informative_scenes()
    for transform in ev31.INFO_FREE:
        rows = []
        for tick in range(3):
            frame, _ = ev31.adversarial_frame(transform, scenes['held_cyan_carry'],
                                              random.Random(f'{transform}:{tick}'))
            rows.append(v31.judge_held_item(_jpeg(frame), CARRY, expected_kind='cyan'))
        assert outcome.track('held_item_at_grip', rows)['status'] != outcome.CONFIRMED, transform


def test_adversarial_frames_are_what_they_claim():
    base = _informative_scenes()['held_cyan_carry']
    rng = random.Random(0)
    value = lambda f: cv2.cvtColor(f, cv2.COLOR_BGR2HSV)[..., 2]  # noqa: E731
    assert value(ev31.adversarial_frame('black', base, rng)[0]).max() == 0
    assert set(np.unique(ev31.adversarial_frame('black_10', base, rng)[0])) == {10}
    for name in ('near_black_uniform', 'near_black_noise', 'dimmed_scene'):
        assert value(ev31.adversarial_frame(name, base, random.Random(name))[0]).max() <= 39, name
    grey, params = ev31.adversarial_frame('uniform_grey', base, rng)
    assert 40 <= params['level'] <= 250 and len(np.unique(grey)) == 1


def test_partial_dark_cover_over_the_grip_is_not_absence():
    """A dark cover over the lower frame hides the CARRY grip band: not "nothing held"."""
    for level in (20, 30, 35, 39):
        frame = _floor()
        frame[150:] = level
        row = v31.judge_held_item(_jpeg(frame), CARRY, expected_kind='cyan')
        assert row['answer'] == 'unknown', (level, row['reason'])
        team = v31.judge_team_cargo_at_grip(_jpeg(frame), CARRY, expected_kind='long_beam')
        assert team['answer'] == 'unknown', (level, team['reason'])


def test_half_dark_band_is_unknown():
    frame = _floor()
    frame[:240] = (10, 9, 9)
    row = v31.judge_team_cargo_at_grip(_jpeg(frame), CARRY, expected_kind='heavy_crate')
    assert row['answer'] == 'unknown'


# --------------------------------------------------------------------- informative frames: same as v3

def test_informative_scene_passes_the_gate():
    for name, frame in _informative_scenes().items():
        info = v31.image_information(_jpeg(frame))
        assert info['sufficient'] is True, (name, info)


@pytest.mark.parametrize('case', [
    ('held', 'held_cyan_carry', CARRY, 'cyan', ('yes', 'EXPECTED_KIND_IN_AT_GRIP_BAND')),
    ('held', 'held_can_check', HELD_CHECK, 'can', ('yes', 'EXPECTED_KIND_IN_AT_GRIP_BAND')),
    ('held', 'floor', CARRY, 'cyan', ('no', 'EXPECTED_KIND_ABSENT_FROM_AT_GRIP_BAND')),
    ('held', 'floor', HELD_CHECK, 'can', ('no', 'EXPECTED_KIND_ABSENT_FROM_AT_GRIP_BAND')),
    ('held', 'floor', HELD_CHECK, 'cyan', ('unknown', 'EXPECTED_KIND_NOT_FRAMED_AT_GRIP')),
    ('team', 'carry_crate_pink', CARRY, 'heavy_crate', ('yes', 'EXPECTED_KIND_COLOUR_IN_AT_GRIP_BAND')),
    ('team', 'carry_crate_pink', CARRY, 'long_beam', ('no', 'OTHER_KIND_COLOUR_IN_AT_GRIP_BAND')),
    ('team', 'floor', CARRY, 'heavy_crate', ('no', 'NOTHING_AT_GRIP_BANDS_ARE_LIT_FLOOR')),
])
def test_informative_frames_answer_exactly_as_v3(case):
    family, scene, pose, kind, expected = case
    jpeg = _jpeg(_informative_scenes()[scene])
    if family == 'held':
        old = v3.judge_held_item(jpeg, pose, expected_kind=kind)
        new = v31.judge_held_item(jpeg, pose, expected_kind=kind)
    else:
        old = v3.judge_team_cargo_at_grip(jpeg, pose, expected_kind=kind)
        new = v31.judge_team_cargo_at_grip(jpeg, pose, expected_kind=kind)
    assert (old['answer'], old['reason']) == expected
    assert (new['answer'], new['reason'], new['confidence']) == (old['answer'], old['reason'], old['confidence'])
    assert (new['v3_answer'], new['v3_reason']) == (old['answer'], old['reason'])


def test_other_kind_at_grip_matches_v3():
    jpeg = _jpeg(_paint(_floor(), _sil('tile', CARRY), TILE_MAGENTA))
    old = v3.judge_held_item(jpeg, CARRY, expected_kind='cyan')
    new = v31.judge_held_item(jpeg, CARRY, expected_kind='cyan')
    assert (new['answer'], new['observed']) == (old['answer'], old['observed']) == ('no', 'tile')


def test_shape_answer_is_kept_only_with_its_boundary_in_view():
    """A dark long_beam on lit floor shows its outline, so v3's shape answer stands."""
    jpeg = _jpeg(_paint(_floor(), _sil('long_beam', CARRY), (10, 9, 9)))
    old = v3.judge_team_cargo_at_grip(jpeg, CARRY, expected_kind='long_beam')
    new = v31.judge_team_cargo_at_grip(jpeg, CARRY, expected_kind='long_beam')
    assert old['cue'] == 'shape' and old['answer'] != 'unknown'
    assert new['shape_boundary']['evidenced'] is True
    assert (new['answer'], new['observed']) == (old['answer'], old['observed'])


def test_grasp_stage_black_frame_is_unknown_for_kind_and_handle():
    jpeg = _jpeg(np.zeros((SIZE[1], SIZE[0], 3), np.uint8))
    row = v31.judge_team_cargo_grasp_stage(jpeg, v3.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
    assert row['answer'] == row['identity']['answer'] == row['handle']['answer'] == 'unknown'
    handle = v31.judge_team_cargo_handle(jpeg, v3.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
    assert handle['answer'] == 'unknown'


def test_grasp_stage_informative_frame_wraps_v3():
    jpeg = _jpeg(_floor())
    old = v3.judge_team_cargo_grasp_stage(jpeg, v3.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
    new = v31.judge_team_cargo_grasp_stage(jpeg, v3.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
    assert new['identity'] == old['identity']
    assert new['handle']['answer'] == old['handle']['answer']


def test_unknown_kinds_are_refused():
    jpeg = _jpeg(_floor())
    with pytest.raises(ValueError):
        v31.judge_held_item(jpeg, CARRY, expected_kind='anvil')
    with pytest.raises(ValueError):
        v31.judge_team_cargo_at_grip(jpeg, CARRY, expected_kind='cyan')
    with pytest.raises(ValueError):
        v31.judge_team_cargo_grasp_stage(jpeg, CARRY, expected_kind='can')
    with pytest.raises(ValueError):
        v31.judge_held_item(jpeg, {1: 1500}, expected_kind='can')      # issued pulses must be complete


# --------------------------------------------------------------------- input boundary

def test_v3_1_runtime_never_imports_mujoco_or_scene_modules():
    script = '''
import sys, json
import numpy as np, cv2
sys.path.insert(0, %r)
from harness import zone_own_perception_v3_1 as p
jpeg = cv2.imencode('.jpg', np.full((480, 640, 3), 110, np.uint8))[1].tobytes()
carry = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
p.judge_held_item(jpeg, carry, expected_kind='cyan')
p.judge_team_cargo_at_grip(jpeg, carry, expected_kind='heavy_crate')
p.judge_team_cargo_handle(jpeg, p.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
p.judge_team_cargo_grasp_stage(jpeg, p.GRASP_LOOK_POSTURE, expected_kind='tri_frame')
banned = sorted(m for m in sys.modules if m == 'mujoco' or m.startswith(('mujoco.', 'sim.zone_scene',
                'sim.zone_cargo_scene', 'sim.multi_masterpi', 'sim.masterpi_production',
                'sim.session_scenes', 'sim.zone_arena')))
print(json.dumps(banned))
''' % str(ROOT)
    proc = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout.strip().splitlines()[-1]) == []


def test_v3_1_runtime_source_does_not_mention_sim_state_or_top_inputs():
    banned = ('import mujoco', 'mj_forward', 'detect_top', '.xpos', '.qpos', 'top_camera',
              'segmentation', 'world.data', 'world.model', 'site_xyz')
    source = (ROOT/'harness/zone_own_perception_v3_1.py').read_text()
    body = '\n'.join(line for line in source.splitlines() if not line.lstrip().startswith('#'))
    _, _, rest = body.partition('"""')
    _, _, code = rest.partition('"""')
    for token in banned:
        assert token not in code, token
