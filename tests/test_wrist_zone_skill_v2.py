"""wrist_zone_skill_v2: fused approach, progress guarantee, retreat and re-seat (pure)."""
import hashlib
from pathlib import Path

import pytest

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v2 as v2
from tests.test_wrist_zone_skill import ORDER, _obs

ROOT = Path(__file__).resolve().parents[1]
POSE = {'1': 2000, '3': 600, '4': 2320, '5': 1320, '6': 1500}


def _box(x, y, iou=.9, cy=218.7):
    return {'visible': True, 'pixel_centroid': [287., cy], 'floor_hypothesis_projection_iou': iou,
            'estimated_box_center_base_m': [x, y, .016]}


def _skill():
    s = v2.WristOnlyBoxSkillV2(robot_id='r1', cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
    s._face_inspection_reached = True
    return s


def test_v1_module_is_unchanged_by_v2():
    digest = hashlib.sha256((ROOT / 'harness/wrist_zone_skill.py').read_bytes()).hexdigest()
    assert digest == V1_SHA256


V1_SHA256 = '964e337eccdaef0bad17fec204b54234875bbb6a9e6e3300847e4c967d5312cd'  # harness/wrist_zone_skill.py at d016c04 (v1 cohort 501-505)


def test_fusion_prefers_good_fits_and_resets_on_base_motion():
    s = _skill()
    s._fuse(_box(.28, .011, .6), (.28, .011))
    s._fuse(_box(.28, -.001, .9), (.28, -.001))
    fx, fy = s._fuse(_box(.28, .000, .92), (.28, .000))
    assert fy == pytest.approx(-.0005) and s.last_fused['good'] == 2
    s._drive_macro(.08, 0., .3)
    assert not s._fused


def test_alternating_lateral_noise_no_longer_flips_the_pan():
    s = _skill()
    actions = []
    for i, y in enumerate([.011, -.003] * 4):
        pose = dict(POSE, **{'6': 1500})
        actions.append(s._approach(_box(.28, y, .9 if i % 2 else .7), (.28, y, .016), pose))
    assert not any(a['kind'] == 'pose' and 6 in a['pulses'] for a in actions)


def test_progress_is_forced_after_repeated_corrections():
    s = _skill()
    kinds = []
    for i in range(8):
        cy = 218.7 + (60 if i % 2 else -60)          # vertical error alternates beyond tolerance
        kinds.append(s._approach(_box(.30, 0., .9, cy), (.30, 0., .016), dict(POSE))['kind'])
    assert 'drive' in kinds and s.approach_stats['forced_forward'] >= 1


def test_combined_correction_is_one_macro():
    s = _skill()
    action = s._approach(_box(.30, .03, .9, 218.7 + 60), (.30, .03, .016), dict(POSE))
    assert action['kind'] == 'pose' and 6 in action['pulses'] and 3 in action['pulses']


def test_transient_miss_uses_fused_estimate_not_search():
    s = _skill()
    s._approach(_box(.30, 0., .9), (.30, 0., .016), dict(POSE))
    action = s._approach({'visible': False}, None, dict(POSE))
    assert s._fused_only == 1 and action['kind'] != 'drive' or action.get('fwd', 0) > 0


def test_no_progress_requests_retreat():
    s = _skill()
    for _ in range(v2.NO_PROGRESS_FRAMES + 3):
        a = s._approach(_box(.30, .05, .9, 218.7 + 60), (.30, .05, .016), dict(POSE, **{'6': 1500}))
    assert s.needs_retreat and a == {'kind': 'wait', 'duration': .05}


def test_reseat_only_with_a_held_box():
    s = _skill()
    with pytest.raises(RuntimeError):
        s.begin_reseat_release()
    s.phase, s.reason, s.held = 'finished', 'VISUAL_GRASP_DRIFT', True
    s.begin_reseat_release()
    assert s.phase == 'release'


def test_delivery_v2_backoff_uses_pose_estimate_then_regrasps():
    d = v2.WristZoneDeliveryV2(ORDER)
    est = v1.PoseEstimate(3.0, -1.8, 0., 'stub')
    assert d._start_backoff(est, .18, 'regrasp')['kind'] == 'pose'
    first = d.decide(_obs(1), est)
    assert first['kind'] == 'mecanum' and first['forward'] < 0
    done = d.decide(_obs(2), v1.PoseEstimate(2.80, -1.8, 0., 'stub'))
    assert done['kind'] == 'wait' and d.phase == 'grasp' and isinstance(d.box, v2.WristOnlyBoxSkillV2)


def test_carry_speed_cap_raised_but_planner_keeps_loaded_clearance():
    seen = []
    d = v2.WristZoneDeliveryV2(ORDER, planner=lambda s, g, loaded: seen.append(loaded) or [g])
    action = d._navigate(v1.PoseEstimate(2.0, -1.25, 0., 'stub'), (2.84, -1.25), 0., carrying=True)
    assert seen == [True] and action['forward'] == pytest.approx(.12)
