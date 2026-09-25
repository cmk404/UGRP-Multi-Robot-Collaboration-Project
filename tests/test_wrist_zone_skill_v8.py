"""wrist_zone_skill_v8: frame-relative approach cyan gate (M1 dev-a6 west pickup floor)."""
import base64
import json
import math
import subprocess
from pathlib import Path

import cv2

from harness import visual_box_skill as n7
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v5 as v5
from harness import wrist_zone_skill_v6 as v6
from harness import wrist_zone_skill_v8 as v8
from harness.markerless_box import observe_ground_box

ROOT = Path(__file__).resolve().parents[1]
FIX = ROOT / 'tests' / 'fixtures' / 'wrist_zone_skill_v8'
WEST = {f['file']: f for f in json.loads((FIX / 'frames.json').read_text())['frames']}
EAST = json.loads((ROOT / 'tests' / 'fixtures' / 'wrist_zone_skill_v6' / 'frames.json').read_text())['frames']


def _b64(path):
    return base64.b64encode(Path(path).read_bytes()).decode()


def _detect(path, pose, sat):
    box = observe_ground_box(_b64(path), pose, 'small_box_01', min_saturation=sat)
    return box['estimated_box_center_base_m'][:2] if box.get('visible') else None


def test_frozen_versions_are_unchanged():
    for rev, path in (('ac34651', 'harness/wrist_zone_skill_v6.py'), ('57e2fd1', 'harness/wrist_zone_skill_v7.py'),
                      ('57e2fd1', 'scripts/run_zone_owncam_skill_v7.py')):
        assert subprocess.check_output(['git', 'show', f'{rev}:{path}'], cwd=ROOT) == (ROOT / path).read_bytes(), path


def test_west_floor_raises_the_gate_and_recovers_the_box():
    for name in ('m1dev-s93_00603.jpg', 'm1dev-s93_00608.jpg'):
        frame = cv2.imread(str(FIX / name))
        gate = v8.floor_relative_min_saturation(frame)
        assert gate['floor_kind'] == 'coloured_floor' and gate['min_saturation'] >= 130
        pose = WEST[name]['own_servo_pwm']
        assert _detect(FIX / name, pose, 65) is None                  # v6: missed on the blue floor
        found = _detect(FIX / name, pose, gate['min_saturation'])
        assert found is not None and .40 < math.hypot(*found) < .60


def test_west_floor_false_box_is_rejected():
    name = 'm1devdiag-s95_00371.jpg'
    pose = WEST[name]['own_servo_pwm']
    wrong = _detect(FIX / name, pose, 65)
    assert wrong is not None and math.hypot(*wrong) > .9               # v6: the floor taken for the box (0.98, -0.32)
    gate = v8.floor_relative_min_saturation(cv2.imread(str(FIX / name)))
    found = _detect(FIX / name, pose, gate['min_saturation'])
    assert found is None or math.hypot(*found) < .6


def test_grey_east_floor_keeps_a_low_gate_and_the_same_detection():
    for frame in EAST[::4]:
        path = ROOT / 'tests' / 'fixtures' / 'wrist_zone_skill_v6' / frame['file']
        gate = v8.floor_relative_min_saturation(cv2.imread(str(path)))
        assert gate['min_saturation'] <= 110
        a = _detect(path, frame['own_servo_pwm'], 65)
        b = _detect(path, frame['own_servo_pwm'], gate['min_saturation'])
        assert (a is None) == (b is None)
        if a is not None:
            assert math.hypot(a[0] - b[0], a[1] - b[1]) < .01


def test_box_skill_gate_is_scoped_to_its_own_decide(monkeypatch):
    box = v8.WristOnlyBoxSkillV8(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    original = n7.observe_ground_box
    seen = {}

    def fake_parent_decide(self, observation):
        seen['min_saturation'] = n7.observe_ground_box.__closure__ is not None
        seen['patched'] = n7.observe_ground_box is not original
        return {'kind': 'wait', 'duration': .1}
    monkeypatch.setattr(v6.WristOnlyBoxSkillV6, 'decide', fake_parent_decide)
    name = 'm1dev-s93_00608.jpg'
    obs = {'robot_id': 'r1', 'frame_id': 1, 'sim_time': 1., 'camera': 'robot_cam', 'image': _b64(FIX / name),
           'sha256': WEST[name]['sha256'], 'actuator_state': {'servo_pulses': WEST[name]['own_servo_pwm']}}
    box.decide(obs)
    assert seen['patched'] and n7.observe_ground_box is original
    assert box.last_floor_gate['min_saturation'] >= 130


def test_v8_keeps_the_v6_v7_api():
    order = v5.CoarseOrderSheet('cyan', 'W2', (.60, -.85), (.15, .25), 'B2', (4.6, -2.1))
    skill = v8.WristZoneDeliveryV8(order, mode='m1', robot_id='r2', static_keepouts=[], static_bounds_m=(-1.05, 5.4, -3.15, 1.45))
    assert isinstance(skill.box, v8.WristOnlyBoxSkillV8) and skill.box.robot_id == 'r2'
    for name in ('approach_point', 'reanchor_after_probe', 'decide', 'confirm_placement', 'summary', 'planner_discs'):
        assert callable(getattr(skill, name))
    assert skill.approach_point()['goal_xy_m'] == [round(.60 - .15 - .25, 4), -.85]
