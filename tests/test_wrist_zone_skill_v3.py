"""wrist_zone_skill_v3: explicit grip check on N7's hand-off, ground-fit look-back (pure)."""
import hashlib
from pathlib import Path

import pytest

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v3 as v3
from tests.test_wrist_zone_skill import ORDER, _obs

ROOT = Path(__file__).resolve().parents[1]
V1_SHA256 = '964e337eccdaef0bad17fec204b54234875bbb6a9e6e3300847e4c967d5312cd'  # d016c04, cohort 501-505
V2_SHA256 = '70f9f9a2b0d81e0477a87cc9fa1a07f90eae0c2365e76244b5e28636be90ae6a'  # edd075d, cohort 511-520
CARRY = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
EST = v1.PoseEstimate(3.0, -1.0, 0., 'stub')


def test_v1_and_v2_modules_are_unchanged_by_v3():
    assert hashlib.sha256((ROOT / 'harness/wrist_zone_skill.py').read_bytes()).hexdigest() == V1_SHA256
    assert hashlib.sha256((ROOT / 'harness/wrist_zone_skill_v2.py').read_bytes()).hexdigest() == V2_SHA256


def _held_handoff():
    s = v3.WristOnlyBoxSkillV3(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    s.phase, s.reason, s.held = 'finished', v3.GRIP_CHECK_REASON, True
    return s


def test_check_grip_only_on_the_n7_handoff_with_a_held_box():
    s = v3.WristOnlyBoxSkillV3(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    with pytest.raises(RuntimeError):
        s.begin_check_grip(_obs(1, pose=CARRY))
    s.phase, s.reason, s.held = 'finished', 'VISUAL_GRASP_DRIFT', True
    with pytest.raises(RuntimeError):
        s.begin_check_grip(_obs(1, pose=CARRY))


def test_check_grip_runs_n7_carry_probe_from_a_fresh_anchor():
    s = _held_handoff()
    obs = _obs(7, pose=CARRY)
    action = s.begin_check_grip(obs)
    assert action == {'kind': 'pose', 'pulses': {6: 1560, 1: 1500}}
    assert s.phase == 'carry_probe_left' and s.reason == 'RUNNING'
    assert s._attachment_image == obs['image'] and s._probe_origin_phase == 'surface_low_height'


class _StubBox:
    def __init__(self, first, then=None):
        self.first, self.then, self.held, self.phase = first, then, True, 'carry'
        self.last_attachment = None

    def decide(self, obs):
        action, self.first = self.first, self.then
        return action

    def begin_check_grip(self, obs):
        self.phase = 'carry_probe_left'
        return {'kind': 'pose', 'pulses': {6: 1560, 1: 1500}}


def test_delivery_selects_check_grip_instead_of_ending_the_run():
    d = v3.WristZoneDeliveryV3(ORDER)
    d.phase = 'nav_preplace'
    d.box = _StubBox({'kind': 'finish', 'reason': v3.GRIP_CHECK_REASON})
    action = d.decide(_obs(1, pose=CARRY), EST)
    assert d.phase == 'grip_check' and action['kind'] == 'pose' and len(d.grip_checks) == 1


def test_grip_check_pass_resumes_carry_and_fail_ends_honestly():
    d = v3.WristZoneDeliveryV3(ORDER)
    d.phase, d.grip_checks = 'grip_check', [{'result': None}]
    box = _StubBox({'kind': 'wait', 'duration': .1})
    d.box = box
    d.decide(_obs(1, pose=CARRY), EST)
    assert d.phase == 'nav_preplace' and d.grip_checks[-1]['result'] == 'ATTACHED'
    d.phase, d.grip_checks = 'grip_check', [{'result': None}]
    box.phase = 'carry_probe_home'
    d.box = _StubBox({'kind': 'finish', 'reason': 'VISUAL_LOAD_DROPPED_OR_OCCLUDED'})
    d.box.phase = 'finished'
    action = d.decide(_obs(2, pose=CARRY), EST)
    assert action == {'kind': 'finish', 'reason': 'GRIP_CHECK_VISUAL_LOAD_DROPPED_OR_OCCLUDED'}


def test_grip_checks_are_capped():
    d = v3.WristZoneDeliveryV3(ORDER)
    d.phase, d.grip_checks = 'nav_preplace', [{}] * v3.MAX_GRIP_CHECKS
    d.box = _StubBox({'kind': 'finish', 'reason': v3.GRIP_CHECK_REASON})
    assert d.decide(_obs(1, pose=CARRY), EST) == {'kind': 'finish', 'reason': 'CARRY_' + v3.GRIP_CHECK_REASON}


def test_look_back_prefers_the_n7_ground_fit(monkeypatch):
    import harness.markerless_box as mb
    monkeypatch.setattr(mb, 'observe_ground_box', lambda *a, **k: {
        'reason': 'FLOOR_CUBOID_HYPOTHESIS_VALIDATED', 'estimated_box_center_base_m': [.168, -.009, .016]})
    d = v3.WristZoneDeliveryV3(ORDER)
    sx, sy = ORDER.slot_xy_m
    result = d.confirm_placement(_obs(1), v1.PoseEstimate(sx - .168, sy + .009, 0., 'stub'))
    assert result['detector'] == 'n7_floor_cuboid_fit' and result['reason'] == 'IN_SLOT'
    far = d.confirm_placement(_obs(1), v1.PoseEstimate(sx - .40, sy, 0., 'stub'))
    assert far['reason'] == 'OUTSIDE_SLOT'


def test_look_back_falls_back_to_detect_own(monkeypatch):
    import harness.markerless_box as mb
    monkeypatch.setattr(mb, 'observe_ground_box', lambda *a, **k: {'reason': 'NO_CYAN_SILHOUETTE'})
    result = v3.WristZoneDeliveryV3(ORDER).confirm_placement(_obs(1), EST)
    assert result['detector'] == 'zone_color_boxes.detect_own' and result['reason'] == 'NO_UNIQUE_CYAN_BOX'
    assert result['ground_fit_reason'] == 'NO_CYAN_SILHOUETTE'


def test_runner_registers_v3_seeds_and_explicit_contact_profiles():
    from scripts import run_zone_owncam_skill as run
    assert run.PROFILES['v3'] == 'wrist_zone_skill_v3'
    assert run.CONTACT_PROFILE == 'local_contact_fine'          # default unchanged
    assert run.CONTACT_PROFILES == ('local_contact_fine', 'cargo_noslip_v1')
    assert set(run.V3_TEST_SEEDS) == set(range(521, 531)) <= set(run.SCENARIOS)
    earlier = set(run.DEV_SEEDS + run.V2_DEV_SEEDS + run.V2_TEST_SEEDS) | set(range(501, 506))
    assert not set(run.V3_TEST_SEEDS) & earlier
