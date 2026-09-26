"""wrist_zone_skill_v4: self-occluded low-top rejection without weakening drop detection."""
import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v3 as v3
from harness import wrist_zone_skill_v4 as v4
from harness.visual_box_surface import observe_known_box_top
from tests.test_wrist_zone_skill import ORDER, _obs

ROOT = Path(__file__).resolve().parents[1]
FIX = ROOT / 'tests/fixtures/wrist_zone_v4'
FRAMES = json.loads((FIX / 'frames.json').read_text())

PINNED = {'harness/wrist_zone_skill.py': '964e337eccdaef0bad17fec204b54234875bbb6a9e6e3300847e4c967d5312cd',
          'harness/wrist_zone_skill_v2.py': '70f9f9a2b0d81e0477a87cc9fa1a07f90eae0c2365e76244b5e28636be90ae6a',
          'harness/wrist_zone_skill_v3.py': '81114b8053b0d7f687c205f55d036e8a33fb5d66f50c846450b934654d8a8689'}
EST = v1.PoseEstimate(2.95, -0.73, 1.75, 'stub')


def _b64(name):
    return base64.b64encode((FIX / name).read_bytes()).decode()


def _pose(name):
    return {int(k): v for k, v in FRAMES[name]['own_pose_commands'].items()}


def test_earlier_versions_are_unchanged():
    for path, digest in PINNED.items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path


def test_fixtures_match_recorded_raw_frames():
    for name, meta in FRAMES.items():
        assert hashlib.sha256((FIX / name).read_bytes()).hexdigest() == meta['sha256']


@pytest.mark.parametrize('alarm, anchor', [('alarm_v3P529_0211.jpg', 'anchor_v3P529_0118.jpg'),
                                           ('alarm_v3P523_0305.jpg', 'anchor_v3P523_0199.jpg')])
def test_recorded_false_alarms_are_strips_of_the_held_box(alarm, anchor):
    surface = observe_known_box_top(_b64(alarm), _pose(alarm))
    # N7's surface fit reports a floor-height "top" while GT (eval only) says the box is held.
    assert surface['visible'] and surface['estimated_box_center_height_m'] < .035
    assert FRAMES[alarm]['gt_box_z_m_eval_only'] > .15
    evidence = v4.self_occlusion_check(surface, _b64(alarm), _b64(anchor))
    assert evidence['self_occluded'] and evidence['reason'] == 'QUAD_INSIDE_HELD_SILHOUETTE'


def _floor_box_frame():
    """Synthetic post-drop view: no held box; a small cyan top on grey floor near the bottom."""
    img = np.full((480, 640, 3), (70, 70, 70), np.uint8)
    cv2.rectangle(img, (280, 360), (360, 430), (200, 190, 40), -1)     # BGR cyan-ish top
    return base64.b64encode(cv2.imencode('.jpg', img)[1].tobytes()).decode()


def test_a_real_drop_is_not_rejected_no_held_silhouette():
    quad = {'pixel_corners': [[280, 360], [360, 360], [360, 430], [280, 430]], 'area_px': 5600}
    evidence = v4.self_occlusion_check(quad, _floor_box_frame(), _b64('anchor_v3P529_0118.jpg'))
    assert not evidence['self_occluded']
    assert evidence['reason'] in ('NO_HELD_SILHOUETTE', 'HELD_SILHOUETTE_CHANGED')


def test_quad_outside_the_held_silhouette_is_not_rejected():
    anchor = _b64('anchor_v3P529_0118.jpg')
    quad = {'pixel_corners': [[300, 40], [340, 40], [340, 60], [300, 60]]}      # upper band, above the held box
    evidence = v4.self_occlusion_check(quad, anchor, anchor)
    assert not evidence['self_occluded'] and evidence['reason'] == 'QUAD_OUTSIDE_HELD_SILHOUETTE'


def test_missing_anchor_or_quad_never_rejects():
    img = _b64('alarm_v3P529_0211.jpg')
    assert not v4.self_occlusion_check(None, img, img)['self_occluded']
    assert not v4.self_occlusion_check({'pixel_corners': [[1, 1]] * 4}, img, None)['self_occluded']


class _StubBox:
    def __init__(self, first, surface=None, image=None):
        self.first, self.held, self.phase, self.reason = first, True, 'finished', first.get('reason')
        self.last_surface, self._attachment_image, self.last_attachment = surface, image, None
        self.resumed = False

    def decide(self, obs):
        return self.first

    def grasp_offset_base_m(self):
        return (.15, 0.)

    def resume_carry_after_self_occluded_top(self):
        self.resumed, self.phase = True, 'carry'

    def begin_check_grip(self, obs):
        self.phase = 'carry_probe_left'
        return {'kind': 'pose', 'pulses': {6: 1560, 1: 1500}}


def _alarm_obs():
    return {**_obs(1, pose=_pose('alarm_v3P529_0211.jpg')), 'image': _b64('alarm_v3P529_0211.jpg')}


def test_delivery_resumes_carry_on_a_self_occluded_strip():
    obs = _alarm_obs()
    surface = observe_known_box_top(obs['image'], _pose('alarm_v3P529_0211.jpg'))
    d = v4.WristZoneDeliveryV4(ORDER)
    d.phase = 'nav_preplace'
    d.box = _StubBox({'kind': 'finish', 'reason': v3.GRIP_CHECK_REASON}, surface, _b64('anchor_v3P529_0118.jpg'))
    action = d.decide(obs, EST)
    assert d.box.resumed and d.phase == 'nav_preplace' and not d.grip_checks
    assert len(d.self_occluded_rejections) == 1 and action['kind'] != 'finish'


def test_delivery_still_probes_when_the_held_silhouette_is_gone():
    d = v4.WristZoneDeliveryV4(ORDER)
    d.phase = 'nav_preplace'
    quad = {'pixel_corners': [[280, 360], [360, 360], [360, 430], [280, 430]]}
    d.box = _StubBox({'kind': 'finish', 'reason': v3.GRIP_CHECK_REASON}, quad, _b64('anchor_v3P529_0118.jpg'))
    obs = {**_obs(1, pose=_pose('alarm_v3P529_0211.jpg')), 'image': _floor_box_frame()}
    action = d.decide(obs, EST)
    assert d.phase == 'grip_check' and action['kind'] == 'pose' and not d.self_occluded_rejections


def test_n7_drop_reasons_still_end_the_run():
    d = v4.WristZoneDeliveryV4(ORDER)
    d.phase = 'nav_preplace'
    d.box = _StubBox({'kind': 'finish', 'reason': 'VISUAL_LOAD_DROPPED_OR_OCCLUDED'})
    assert d.decide(_alarm_obs(), EST) == {'kind': 'finish', 'reason': 'CARRY_VISUAL_LOAD_DROPPED_OR_OCCLUDED'}


def test_resume_requires_the_handoff_with_a_held_box():
    s = v4.WristOnlyBoxSkillV4(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    with pytest.raises(RuntimeError):
        s.resume_carry_after_self_occluded_top()
    s.phase, s.reason, s.held = 'finished', v3.GRIP_CHECK_REASON, False
    with pytest.raises(RuntimeError):
        s.resume_carry_after_self_occluded_top()
    s.held = True
    s.resume_carry_after_self_occluded_top()
    assert s.phase == 'carry' and s.reason == 'RUNNING'


def test_runner_registers_v4_seeds_and_zone_c_destinations():
    from scripts import run_zone_owncam_skill as run
    assert run.PROFILES['v4'] == 'wrist_zone_skill_v4' and run.V4_TEST_SEEDS == tuple(range(531, 541))
    zone_c = [s for s in run.V4_TEST_SEEDS if run.SCENARIOS[s]['slot'].startswith('C')]
    assert zone_c == [531, 532, 533, 534, 538, 540]
    earlier = set(run.DEV_SEEDS + run.V2_DEV_SEEDS + run.V2_TEST_SEEDS + run.V3_TEST_SEEDS) | set(range(501, 506))
    assert not set(run.V4_TEST_SEEDS) & earlier


def test_recorded_real_drop_frame_is_never_classified_self_occluded():
    """Deliberate drop (dev fault injection, a8b9ee2 408): the held silhouette vanished."""
    drop = 'drop_v4dev408_0156.jpg'
    assert FRAMES[drop]['gt_box_z_m_eval_only'] < .03
    anchor = _b64('anchor_v3P529_0118.jpg')
    quad = {'pixel_corners': [[300, 300], [340, 300], [340, 330], [300, 330]]}   # where the held face used to be
    evidence = v4.self_occlusion_check(quad, _b64(drop), anchor)
    assert not evidence['self_occluded']
    assert evidence['reason'] in ('NO_HELD_SILHOUETTE', 'HELD_SILHOUETTE_CHANGED')
