"""Package F input boundaries, corrupted frames, near-range truncation fixture and CI collection (issue #221).

Wrong types, empty input, 0, None, NaN/Inf and corrupted frames are REFUSED (a rejected ack or a
contract error), never an uncaught IndexError/TypeError (earlier audit: goto([]), goto(None),
hold(None) raised). Every rejected ack is still a valid package A action record.
"""
from __future__ import annotations

import base64
import fnmatch
import hashlib
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import m1_owncam_contract  # noqa: E402
from harness import zone_own_executor as zox  # noqa: E402
from harness import zone_study_contract as A  # noqa: E402
from tests.test_zone_own_executor import CALIB, MAP, SEARCH_POSE, Driver, make, obs, rgb_of  # noqa: E402

NAN, INF = float('nan'), float('inf')
FIXTURE = ROOT / 'tests' / 'fixtures' / 'zone_own_executor_near_clip'


@pytest.mark.parametrize('target', [[], None, [NAN, 0.], [0., INF], [1., 2., 3.], [1.], True, 0, 1.5, {'x': 1.},
                                    ['a', 'b'], [True, False], (None, None), b'A2'])
def test_goto_refuses_bad_targets(target):
    ex = make()
    ack = ex.goto(target)
    assert not ack['accepted'] and ack['rejected_reason'] == 'BAD_TARGET' and ex.job is None
    assert A.action_record_violations(zox.action_record(ack, run_id='t', condition='no_comm', seed=0,
                                                        request_id='q')) == []
    json.dumps(ack)


@pytest.mark.parametrize('target,reason', [('', 'UNKNOWN_TARGET'), ('Q7', 'UNKNOWN_TARGET'),
                                           ([99., 0.], 'TARGET_OUTSIDE_MAP'), ([0, 0], None), ('A', None),
                                           ('door_1', None), ('P1-2', None), ('C2', None)])
def test_goto_map_vocabulary(target, reason):
    ack = make().goto(target)
    assert ack['accepted'] is (reason is None) and ack['rejected_reason'] == reason


@pytest.mark.parametrize('duration', [None, True, False, -1, -1e-9, NAN, INF, -INF, '5', [1], zox.MAX_HOLD_S + 1])
def test_hold_refuses_bad_durations(duration):
    ex = make()
    for api in (ex.hold, ex.wait):
        ack = api(duration)
        assert not ack['accepted'] and ack['rejected_reason'] == 'BAD_DURATION' and ex.job is None
        json.dumps(ack)


def test_hold_zero_is_a_valid_duration():
    ex = make()
    d = Driver(ex)
    assert ex.hold(0)['accepted']                        # 0 is a duration, not "missing" (no `x or default`)
    d.run(.3)
    ev = ex.drain_events()
    assert [e['event'] for e in ev] == ['job_started', 'job_done']


@pytest.mark.parametrize('item,slot,reason', [(None, 'A2', 'UNKNOWN_ORDER'), (['o1'], 'A2', 'UNKNOWN_ORDER'),
                                              ('', 'A2', 'UNKNOWN_ORDER'), (0, 'A2', 'UNKNOWN_ORDER'),
                                              ('o1', None, 'UNKNOWN_ZONE_SLOT'), ('o1', '', 'UNKNOWN_ZONE_SLOT'),
                                              ('o1', ['A2'], 'UNKNOWN_ZONE_SLOT'), ('o1', 'Z9', 'UNKNOWN_ZONE_SLOT'),
                                              ('o1', NAN, 'UNKNOWN_ZONE_SLOT')])
def test_deliver_refuses_bad_arguments(item, slot, reason):
    ex = make()
    ack = ex.deliver(item, slot)
    assert not ack['accepted'] and ack['rejected_reason'] == reason and ex.job is None
    json.dumps(ack)


@pytest.mark.parametrize('code', [None, '', 7, 'x' * 65, ['a']])
def test_abort_refuses_bad_reason_codes(code):
    ex = make()
    ex.hold(5.)
    ack = ex.abort(code)
    assert not ack['accepted'] and ack['rejected_reason'] == 'BAD_REASON' and ex.job is not None


def test_corrupted_or_foreign_frames_are_refused_without_state_change():
    ex = make()
    d = Driver(ex)
    d.frame()
    last = ex.last_frame_id
    good = obs('r1', 5, 0., SEARCH_POSE)
    bad_hash = {**good, 'sha256': '0' * 64}
    bad_jpeg = {**good, 'image': base64.b64encode(b'not a jpeg').decode()}
    bad_jpeg['sha256'] = hashlib.sha256(b'not a jpeg').hexdigest()
    for frame in (bad_hash, {**good, 'camera': 'nav_cam'}, {**good, 'camera': 'top'}, {**good, 'robot_id': 'r2'},
                  {**good, 'frame_id': last}, {**good, 'sim_time': -5.}, None, [], 'frame'):
        with pytest.raises((zox.ExecutorContractError, m1_owncam_contract.M1ContractError)):
            ex.on_frame(0., frame, rgb_of(good))
        assert ex.last_frame_id == last
    # a hash-consistent but undecodable JPEG reaches the estimator as the caller's decoded frame only;
    # the near-range check on it degrades to 0 instead of raising
    from harness.zone_own_deliver import bottom_clipped_cyan_px
    assert bottom_clipped_cyan_px(bad_jpeg['image']) == 0 and bottom_clipped_cyan_px('%%%') == 0


def test_executor_constructor_refuses_bad_limits_and_non_static_keepouts():
    for bad in (0, -1, NAN, INF, None, True):
        with pytest.raises((ValueError, TypeError)):
            make(job_sim_limit_s=bad)
    with pytest.raises(zox.ExecutorContractError):
        make(static_keepouts=[{'id': 'peer', 'center_m': [0, 0], 'radius_m': .2, 'source': 'live_peer_pose'}])


def test_executor_has_no_condition_input():
    """The executor (and so the pair status it could publish) is identical across study conditions."""
    import inspect
    params = inspect.signature(zox.ZoneOwnExecutor.__init__).parameters
    assert not any('condition' in p or 'leader' in p or 'channel' in p for p in params)


# ---------------------------------------------------------------- near-range truncation (Codex P2-6)
def test_near_clip_fixture_real_frames():
    labels = json.loads((FIXTURE / 'labels.json').read_text())
    from harness.zone_own_deliver import NEAR_CLIP_MIN_PX, bottom_clipped_cyan_px
    assert {f['label'] for f in labels['frames']} == {'clipped_near', 'no_cyan', 'visible_not_clipped'}
    for f in labels['frames']:
        raw = (FIXTURE / f['file']).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == f['sha256'] == f['raw_sha256_in_frames_jsonl']
        px = bottom_clipped_cyan_px(base64.b64encode(raw).decode())
        assert (px >= NEAR_CLIP_MIN_PX) is (f['label'] == 'clipped_near'), (f['file'], px)


def test_near_clip_leads_to_one_retreat_relook_then_the_next_viewpoint():
    """s700 r3 / s701 r3 (smoke v1): the box 0.27 m ahead was cut off and the search ended SEARCH_NOT_FOUND."""
    from harness import zone_own_guards as guards
    from harness.owncam_pose_source import OwnCamPoseSource
    slots = zox.pickup_slots(MAP)
    slot = slots['P1-3']
    discs = [{'id': f'spawn_row_{i}', 'center_m': [-.85, y], 'radius_m': .17, 'source': 'static_layout_idle_spawn'}
             for i, y in enumerate((-2.25, -.85, .55))]
    ctl = zox._DeliverController(MAP, CALIB['params'], box_kind='cyan', slot_id='A2', slot_xy=[4.6, .4],
                                 skill_factory=lambda order: None, pose_estimate_cls=tuple, search_rows_y=[-.05, .75],
                                 robot_id='r1', seed=0, order_kind='own_rgb_bay',
                                 shared_pose=OwnCamPoseSource(MAP, CALIB['params']), servo=dict(SEARCH_POSE),
                                 slot_rect=(tuple(slot['x_range_m']), tuple(slot['y_range_m'])),
                                 gate=guards.UncertaintyGate(), guard=guards.SweepGuard(MAP), static_keepouts=discs)
    ctl.viewpoints, ctl.view_index, ctl.phase = [(-.47, .75), (-.47, -.05)], 0, 'search_sweep'
    ctl.near_clipped = [{'t': 1., 'frame_id': 1, 'px': 337}, {'t': 1.2, 'frame_id': 2, 'px': 311}]
    ctl._after_sweep(10.)
    assert ctl.phase == 'search_leg' and ctl.leg.goal == [-.47, .35]    # west retreat blocked by the spawn disc
    assert ctl.view_index == 0 and ctl.events[-1]['event'] == 'near_clipped'
    ctl.phase, ctl.near_clipped = 'search_sweep', [{'t': 20., 'frame_id': 9, 'px': 300}] * 2
    ctl._after_sweep(20.)                                               # one retreat per viewpoint
    assert ctl.view_index == 1 and ctl.leg.goal == [-.47, -.05]
    free = zox._DeliverController(MAP, CALIB['params'], box_kind='cyan', slot_id='A2', slot_xy=[4.6, .4],
                                  skill_factory=lambda order: None, pose_estimate_cls=tuple, search_rows_y=[-1.65],
                                  robot_id='r1', seed=0, order_kind='own_rgb_bay',
                                  shared_pose=OwnCamPoseSource(MAP, CALIB['params']), servo=dict(SEARCH_POSE),
                                  slot_rect=(tuple(slots['P1-1']['x_range_m']), tuple(slots['P1-1']['y_range_m'])),
                                  gate=guards.UncertaintyGate(), guard=guards.SweepGuard(MAP), static_keepouts=discs)
    assert free._retreat_goal(-.47, -1.65) == (-.47 - .20, -1.65)       # clear of every spawn disc: back off west


# ---------------------------------------------------------------- CI collection
def test_executor_tests_are_collected_by_ci():
    sys.path.insert(0, str(ROOT / 'scripts'))
    import run_ci_tests
    mine = sorted(str(p.relative_to(ROOT)) for p in (ROOT / 'tests').glob('test_zone_own_executor*.py'))
    assert len(mine) == 4
    for path in mine:
        assert any(fnmatch.fnmatch(path, pat) for pat in run_ci_tests.TEST_PATTERNS), path
