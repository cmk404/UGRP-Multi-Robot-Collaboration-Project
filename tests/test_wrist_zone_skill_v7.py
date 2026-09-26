"""wrist_zone_skill_v7: re-plan to another clear box face after a static keep-out guard stop."""
import math
import subprocess
from pathlib import Path

import pytest

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v5 as v5
from harness import wrist_zone_skill_v6 as v6
from harness import wrist_zone_skill_v7 as v7

ROOT = Path(__file__).resolve().parents[1]
ORDER = v5.CoarseOrderSheet('cyan', 'E2', (3.70, -1.50), (.25, .25), 'A2', (4.6, -2.1))
BOUNDS = (-1.05, 5.4, -3.15, 1.45)


def _est(x, y, yaw=0.):
    return v1.PoseEstimate(x, y, yaw, 'own_rgb_apriltag_ekf')


def _skill(keepouts=(), box=(3.60, -1.42), yaw=0.):
    skill = v7.WristZoneDeliveryV7(ORDER, static_keepouts=keepouts, static_bounds_m=BOUNDS)
    skill.phase = 'grasp'
    skill.box_map_estimate = {'xy_m': list(box), 'yaw_rad': yaw, 'source': 'own_rgb_ground_fit+own_pose_estimate'}
    return skill


def _peer(xy, pid='parking_P1'):
    return v6.StaticKeepout(pid, xy, .17, 'static_map_parking')


def test_v6_is_unchanged_for_the_m1_integration():
    for rev, path in (('ac34651', 'harness/wrist_zone_skill_v6.py'), ('ac34651', 'scripts/run_zone_owncam_skill_v6.py')):
        assert subprocess.check_output(['git', 'show', f'{rev}:{path}'], cwd=ROOT) == (ROOT / path).read_bytes(), path


def test_v6_552_layout_blocks_the_west_face_and_offers_others():
    skill = _skill([_peer((3.2, -1.2))])               # v6 cohort 552: r2 parked at the v6 lateral spot
    faces = skill.face_candidates((3.28, -1.55))
    usable = {round(math.degrees(f['normal_rad'])) for f in faces if f['usable']}
    blocked = {round(math.degrees(f['normal_rad'])) for f in faces if not f['usable']}
    assert 180 in blocked or -180 in blocked
    assert usable and usable <= {0, -90, 90}


def test_guard_in_grasp_replans_backs_off_then_drives_to_a_clear_face():
    skill = _skill([_peer((3.2, -1.2))])
    est = _est(3.28, -1.55, .04)                         # r1 west of the box, squaring toward the peer
    action = skill._replan_after_guard(est, 'parking_P1', {'kind': 'mecanum', 'forward': 0., 'left': .08,
                                                          'turn': 0., 'duration': 1.})
    assert action['kind'] == 'wait' and skill.phase == 'keepout_backoff' and skill.keepout_replans == 1
    choice = skill.replan['choice']
    assert choice['usable'] and abs(v7._wrap(choice['normal_rad'] - math.pi)) > math.radians(45)
    back = skill._keepout_backoff(None, est)
    assert back['kind'] == 'mecanum' and skill._guard(back, est) is None
    # moves away from the box (box is ahead/+x of r1)
    assert back['forward'] < 0
    skill.replan['origin'] = (est.x_m + .2, est.y_m)     # pretend the retreat is done
    skill._keepout_backoff(None, est)
    assert skill.phase == 'replan_nav'
    assert skill.planner_discs() == [(3.60, -1.42, v7.BOX_PLANNER_DISC_M)]
    gx, gy = choice['approach_xy_m']
    old_box = skill.box
    arrived = skill._replan_nav(None, _est(gx, gy, choice['heading_rad']))
    assert arrived['kind'] == 'wait' and skill.phase == 'grasp' and skill.box is not old_box
    assert any(e['event'] == 'replan_approach_reached' for e in skill.events)


def test_no_clear_face_aborts_cleanly():
    peers = [_peer((3.60 + .30 * math.cos(a), -1.42 + .30 * math.sin(a)), f'p{i}')
             for i, a in enumerate((math.pi, math.pi / 2, -math.pi / 2, 0.))]
    skill = _skill(peers[:3] + [_peer((3.95, -1.42), 'p3')])
    action = skill._replan_after_guard(_est(3.28, -1.42), 'p0', {'kind': 'drive', 'fwd': .05, 'turn': 0., 'duration': .5})
    assert action == {'kind': 'finish', 'reason': 'NO_GRASPABLE_FACE_CLEAR_OF_KEEPOUTS'}
    assert skill.keepout_replans == 0


def test_replan_limit_and_unknown_box():
    skill = _skill([_peer((3.2, -1.2))])
    skill.keepout_replans = v7.MAX_KEEPOUT_REPLANS
    assert skill._replan_after_guard(_est(3.28, -1.55), 'parking_P1', {})['reason'] == 'KEEPOUT_REPLAN_LIMIT'
    unknown = v7.WristZoneDeliveryV7(ORDER, static_keepouts=[_peer((3.2, -1.2))])
    unknown.phase = 'grasp'
    assert unknown._replan_after_guard(_est(3.28, -1.55), 'parking_P1', {})['reason'] == 'STATIC_KEEPOUT_GUARD'


def test_blocked_faces_are_not_offered_again():
    skill = _skill([_peer((3.2, -1.2))])
    skill.blocked_normals = [math.pi, -math.pi / 2]
    normals = {round(math.degrees(f['normal_rad'])) for f in skill.face_candidates((3.3, -1.4))}
    assert normals == {0, 90}


def test_box_map_estimate_comes_from_own_rgb_target_and_own_pose():
    skill = v7.WristZoneDeliveryV7(ORDER)
    skill.box.last_target = (.40, .10, .016)
    skill.box.last_face_alignment = {'ready': True, 'normal_source': 'own_rgb_markerless_top_edge_vote',
                                     'evidence': {'yaw_mod90_deg': 10.}}
    skill._track_box(_est(3.0, -1.5, math.pi / 2))
    x, y = skill.box_map_estimate['xy_m']
    assert (round(x, 4), round(y, 4)) == (2.9, -1.1)
    assert round(math.degrees(skill.box_map_estimate['yaw_rad']), 3) == 100.
    assert skill.box_map_estimate['source'].startswith('own_rgb')


def test_east_side_is_tried_last_and_v6_points_are_unchanged():
    free = v7.WristZoneDeliveryV7(ORDER, static_bounds_m=BOUNDS).approach_point()
    assert free['side'] == 'west' and free['is_v5_point'] and free['goal_xy_m'] == [3.2, -1.5]
    ring = [_peer((3.70 + .45 * math.cos(a), -1.50 + .45 * math.sin(a)), f'p{i}')
            for i, a in enumerate((math.pi, math.pi / 2, -math.pi / 2))]
    east = v7.WristZoneDeliveryV7(ORDER, static_keepouts=ring, static_bounds_m=BOUNDS).approach_point()
    assert east['side'] == 'east' and east['heading_rad'] == pytest.approx(math.pi)
    assert v6.WristZoneDeliveryV6(ORDER, static_keepouts=ring, static_bounds_m=BOUNDS).approach_point()['blocked']


def test_decide_replans_instead_of_finishing(monkeypatch):
    skill = _skill([_peer((3.2, -1.2))])
    move = {'kind': 'mecanum', 'forward': 0., 'left': .08, 'turn': 0., 'duration': 1.}
    monkeypatch.setattr(v5.WristZoneDeliveryV5, 'decide', lambda self, obs, est: move)
    action = skill.decide({}, _est(3.28, -1.55, .04))
    assert skill.phase == 'keepout_backoff' and action['kind'] == 'wait'
    assert [e['event'] for e in skill.events][-2:] == ['static_keepout_guard', 'keepout_replan_candidates']
