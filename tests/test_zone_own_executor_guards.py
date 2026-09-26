"""Package F guards (issue #221; Codex review 2 of PR #206): uncertainty gate, look-sweep collision guard,
progress monitor with bounded recovery, per-location blockage streak.

Simulator-free. The P1 regressions drive the real executor API with a scripted own-estimate fixture
(``mode='diagnostic'``, never M1): they failed on the pre-fix executor (c8a2355a) and pass now.
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_own_executor as zox  # noqa: E402
from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE, WIDE_LOOK_PANS  # noqa: E402
from harness.owncam_pose_source import PoseReport  # noqa: E402
from tests.test_zone_own_executor import MAP, Driver, make  # noqa: E402

try:                                    # imported lazily so the P1 regressions also run (and fail) on c8a2355a
    from harness import zone_own_guards as guards
except ImportError:                     # pragma: no cover - pre-fix source only
    guards = None

CALIBRATION = ROOT / 'experiments' / '2026-09-26-zone-own-executor' / 'body_model_calibration.json'


def v3_like(static):
    """The v2 tagged map with the env-v3 wall height (0.40 m, PR #208) and no door posts."""
    m = copy.deepcopy(static)
    for o in m['obstacles']:
        o['height_m'] = .40
    m['landmarks'].pop('door_posts', None)
    return m


class ScriptedLoc:
    """Localizer stand-in whose own estimate the test scripts over SIM time (diagnostic fixtures only)."""

    def __init__(self, fn):
        self.fn, self.t, self.commands, self.stats = fn, 0., [], {}

    def command(self, row):
        self.commands.append(row)

    def predict_to(self, t):
        self.t = max(self.t, float(t))

    def estimate(self):
        e = self.fn(self.t)
        return {'t': self.t, 'initialized': True, 'x': e[0], 'y': e[1], 'yaw': e[2], 'std_xy_m': e[3],
                'std_yaw_rad': e[4], 'since_tag_s': e[5], 'n_eff': 100., 'cov': []}


class ScriptedPose:
    source = 'diagnostic_scripted_estimate'

    def __init__(self, fn):
        self.loc = ScriptedLoc(fn)

    def on_command(self, row):
        self.loc.command(row)

    def on_frame(self, now, rgb):
        self.loc.predict_to(now)
        return self.report(now)

    def report(self, now):
        self.loc.predict_to(now)
        e = self.loc.estimate()
        return PoseReport(now, True, e['x'], e['y'], e['yaw'], (), e['std_xy_m'], e['std_yaw_rad'], e['since_tag_s'],
                          source=self.source)

    def set_motion_profile(self, t, name):
        pass


def scripted(fn, **kw):
    kw.setdefault('judgments', False)
    return make(mode='diagnostic', pose_source=ScriptedPose(fn), **kw)


def motion_commands(driver):
    out = []
    for t, d in driver.decisions:
        for c in json.loads(d).get('commands', ()):
            if c['kind'] in ('mecanum', 'drive'):
                out.append((t, c))
    return out


def pans_commanded(driver):
    return [c['pan_pulse'] for _, d in driver.decisions for c in json.loads(d).get('commands', ()) if c['kind'] == 'look']


# ---------------------------------------------------------------- 1. uncertainty gate
def test_gate_two_thresholds_and_dwell():
    g = guards.UncertaintyGate(guards.GATE_UNLOADED)
    assert g.state == 'uncertain'                       # nothing confirmed yet
    t = 0.
    for _ in range(2):                                  # 0.2 s below LOW: dwell (0.4 s) not reached
        assert g.update(t, True, .02, .01) is None
        t += .2
    assert g.update(t, True, .02, .01) == 'exited' and g.ok
    for std in (.079, .081) * 20:                       # oscillating around HIGH (0.08): never 0.6 s above
        t += .2
        assert g.update(t, True, std, .01) is None and g.ok
    changes = []
    for _ in range(5):
        t += .2
        changes.append(g.update(t, True, .09, .01))
    assert changes.count('entered') == 1 and not g.ok
    for std in (.06, .07, .04, .06) * 5:               # band / single low frames: no exit without dwell below LOW
        t += .2
        assert g.update(t, True, std, .01) is None and not g.ok


@pytest.mark.parametrize('bad', [None, float('nan'), float('inf'), -float('inf'), 'x', True])
def test_gate_bad_sigma_counts_as_high_and_bad_time_is_ignored(bad):
    g = guards.UncertaintyGate()
    for i in range(4):
        g.update(i * .2, True, .01, .01)
    assert g.ok
    assert g.classify(True, bad, .01) == 'high' and g.classify(False, .01, .01) == 'high'
    assert g.update(float('nan'), True, .5, .5) is None and g.update(-1., True, .5, .5) is None and g.ok


def test_p1_forward_motion_and_arrival_blocked_under_large_uncertainty():
    """Codex P1-1 fixture: sigma_xy 0.20 m, sigma_yaw 0.15 rad, box held (loaded driver).

    Pre-fix: forward mecanum commands and ``ARRIVED / own_camera_confirmed``.
    """
    for goal_dx in (.5, .02):
        ex = scripted(lambda t: (1.0, -0.85, 0., .20, .15, .1))
        ex._holding_after = {'answer': 'unknown', 'source': 'fixture: previous job ended with the gripper closed'}
        d = Driver(ex)
        assert ex.goto([1.0 + goal_dx, -0.85])['accepted']
        d.run(200., stop=lambda: ex.job is None)
        ev = ex.drain_events()
        assert not [c for _, c in motion_commands(d) if abs(c['forward']) > 1e-9 or abs(c['left']) > 1e-9], goal_dx
        done = [e for e in ev if e['event'] in ('job_done', 'job_failed')]
        assert len(done) == 1 and done[0]['event'] == 'job_failed'
        assert done[0]['detail']['reason'] in ('GOTO_pose_uncertain', 'GOTO_lost')      # both: own sigma never fixed
        assert not any(e['event'] == 'job_done' and e['detail'].get('confirmation') == 'own_camera_confirmed' for e in ev)
        unc = [e for e in ev if e['event'] == 'pose_uncertain']
        assert unc and all(e['scheduler_trigger'] == 'failure' for e in unc)


def test_p1_pose_uncertain_fires_once_per_entry_not_per_frame():
    """Codex P1-3: sigma oscillating 0.079/0.081 m re-armed on one medium frame and fired per frame."""
    def fn(t):
        if t < 2.:
            return (0., -0.85, 0., .02, .01, .1)
        if t < 12.:
            return (0., -0.85, 0., .079 if int(t * 5) % 2 else .081, .01, .1)
        if t < 14.:
            return (0., -0.85, 0., .09, .01, .1)
        return (0., -0.85, 0., .079 if int(t * 5) % 2 else .081, .01, .1)
    ex = scripted(fn)
    d = Driver(ex)
    ex.hold(30.)
    d.run(25.)
    unc = [e for e in ex.drain_events() if e['event'] == 'pose_uncertain']
    assert len(unc) == 1 and unc[0]['scheduler_trigger'] == 'failure' and unc[0]['detail']['ends_job'] is False


def test_look_around_confirms_only_with_an_ok_gate():
    ex = scripted(lambda t: (0., -0.85, 0., .20, .15, .1))
    d = Driver(ex)
    ex.look_around()
    d.run(20., stop=lambda: ex.job is None)
    last = [e for e in ex.drain_events() if e['event'] == 'job_done'][-1]
    assert last['detail']['confirmation'] == 'unconfirmed' and last['detail']['outcome'] == 'LOOKED_POSE_UNCERTAIN'
    ok = scripted(lambda t: (0., -0.85, 0., .02, .01, .1))
    d2 = Driver(ok)
    ok.look_around()
    d2.run(20., stop=lambda: ok.job is None)
    last = [e for e in ok.drain_events() if e['event'] == 'job_done'][-1]
    assert last['detail']['confirmation'] == 'own_camera_confirmed'


# ---------------------------------------------------------------- 2. sweep guard
def test_body_model_matches_the_calibration_record():
    rec = json.loads(CALIBRATION.read_text())
    assert tuple(rec['mount_xyz_m']) == guards.BODY_MOUNT_XYZ_M
    assert rec['coverage_residual_max_m'] <= guards.BODY_COVERAGE_RESIDUAL_M
    val = rec['sweep_validation_0p40_walls']
    # every physical arm-wall contact of the v1 full sweep happened at a pan the guard marks unsafe,
    # and the guard's own plan made no contact at any validation pose
    assert all(v['contact_pans_predicted_unsafe'] for v in val)
    assert any(v['policy'] == 'full_sweep_v1' and v['contact_steps'] > 0 for v in val)
    assert all(v['contact_steps'] == 0 for v in val if v['policy'] == 'guard_plan')


def test_sweep_guard_restricts_pans_near_a_tall_wall_and_backs_off():
    guard = guards.SweepGuard(v3_like(MAP))
    pose = guards.OwnPose(2.08, 0.18, 0., .01, .01)        # validation pose: v1 sweep hit the 0.40 m wall at 1890-2030
    plan = guard.plan({**SEARCH_POSE, 6: 1500}, LOOK_P20, WIDE_LOOK_PANS, pose, loaded=False, allow_backoff=True)
    assert plan['reason'] == 'restricted' and 2030 in plan['dropped'] and 1770 in plan['dropped']
    assert plan['backoff'] is not None and len(plan['backoff']['pans_after']) > len(plan['pans'])
    assert guard.chassis_clearance(pose.moved(plan['backoff']['dx_base_m'] * guards.BACKOFF_GAIN_MAX,
                                              plan['backoff']['dy_base_m'] * guards.BACKOFF_GAIN_MAX))[0] >= 0
    assert guards.SweepGuard(MAP).plan({**SEARCH_POSE, 6: 1500}, LOOK_P20, WIDE_LOOK_PANS, pose,
                                       loaded=False)['reason'] == 'restricted'   # v2: 0.30 m door post
    open_floor = guard.plan({**SEARCH_POSE, 6: 1500}, LOOK_P20, WIDE_LOOK_PANS, guards.OwnPose(1., -1., 0., .02, .02),
                            loaded=False)
    assert open_floor['reason'] == 'clear' and open_floor['pans'] == list(WIDE_LOOK_PANS)
    bigger = guard.plan({**SEARCH_POSE, 6: 1500}, LOOK_P20, WIDE_LOOK_PANS, guards.OwnPose(2.2, .05, 0., .10, .05),
                        loaded=True)
    assert bigger['reason'] != 'clear'                  # uncertainty inflates the margin: in the door, fewer pans
    assert guard.plan({**SEARCH_POSE, 6: 1500}, LOOK_P20, WIDE_LOOK_PANS, None, loaded=False)['reason'] == 'no_own_estimate'


def test_p1_look_around_near_a_tall_wall_never_commands_a_colliding_pan():
    """Codex P1-2: the +-48 deg look inside a 0.40 m-wall door hit the jamb. Pre-fix: every WIDE pan."""
    ex = zox.ZoneOwnExecutor('r1', v3_like(MAP), make().params, {'orders': []}, skill_factory=lambda o, robot_id: None,
                             pose_estimate_cls=tuple, search_rows_y=(), mode='diagnostic', judgments=False,
                             pose_source=ScriptedPose(lambda t: (2.08, 0.18, 0., .01, .01, .1)))
    d = Driver(ex)
    assert ex.look_around()['accepted']
    d.run(40., stop=lambda: ex.job is None)
    assert max(pans_commanded(d)) < 1890                # the v1 sweep's physical contact pans (validation record)
    strict = guards.SweepGuard(v3_like(MAP), residual_m=0.)
    pose = guards.OwnPose(2.08, 0.18, 0., 0., 0.)
    for pan in pans_commanded(d):                       # no commanded pan intersects the wall (zero margin)
        assert strict.arm_clearance({**SEARCH_POSE, **LOOK_P20, 6: pan}, pose, loaded=False)[0] + guards.BASE_MARGIN_M >= 0, pan
    moves = motion_commands(d)
    assert moves and moves[0][1]['kind'] == 'mecanum'  # backed off first (the scripted estimate does not move)
    assert [e['event'] for e in ex.drain_events()][-1] == 'job_done'


# ---------------------------------------------------------------- 3. progress monitor
def test_progress_monitor_is_nav2_style():
    m = guards.ProgressMonitor()
    m.trusted((0., 0.), 1.)
    m.drove(guards.MOVEMENT_TIME_ALLOWANCE_S - .1)
    assert not m.stalled()
    m.trusted((.05, 0.), .95)
    m.drove(.2)
    assert m.stalled()
    m.trusted((.2, 0.), .8)                             # moved > REQUIRED_MOVEMENT_M: new baseline
    assert not m.stalled()
    m2 = guards.ProgressMonitor()
    m2.trusted((0., 0.), .06)                          # near the goal: half the remaining distance is progress
    m2.drove(guards.MOVEMENT_TIME_ALLOWANCE_S - 1.)
    m2.trusted((.035, 0.), .025)
    m2.drove(2.)
    assert not m2.stalled()


def test_p1_blocked_leg_recovers_a_bounded_number_of_times_then_fails():
    """Codex P1-3: an unseen box stopped the leg for ~660 SIM s until the 720 s job limit.

    Scripted: the own estimate (fresh tags, low sigma) never moves while the robot drives. Pre-fix:
    LOCAL_TIMEOUT at the job limit. Now: MAX_RECOVERIES back-off + keep-out recoveries, then GOTO_blocked.
    """
    ex = scripted(lambda t: (0.33, -1.60, 0., .02, .01, .1), job_sim_limit_s=300.)
    d = Driver(ex)
    assert ex.goto([0.9, -1.6])['accepted']
    d.run(300., stop=lambda: ex.job is None)
    ev = ex.drain_events()
    failed = [e for e in ev if e['event'] == 'job_failed']
    assert len(failed) == 1 and failed[0]['detail']['reason'] == 'GOTO_blocked', failed
    assert failed[0]['sim_s'] < 150.
    blocked = [e for e in ev if e['event'] == 'blockage_seen']
    assert len(blocked) == 1 and blocked[0]['detail']['source'] == 'own_progress_stall'
    assert len(blocked[0]['detail']['stall_keepouts']) == guards.MAX_RECOVERIES
    summary = next(s for s in ex._summaries if 'driver_log' in s)
    recoveries = [e for e in summary['driver_log'] if e['event'] == 'stall_recovery']
    assert len(recoveries) == guards.MAX_RECOVERIES and all(r['backoff'] for r in recoveries)
    reverse = [c for _, c in motion_commands(d) if c['forward'] < 0]
    assert reverse                                     # the back-off (Nav2 BackUp) was commanded


# ---------------------------------------------------------------- 4. blockage streak per location
def test_blockage_streak_is_per_location_and_gap_limited():
    s = guards.BlockageStreak(2)
    a = guards.BlockageStreak.key_of(None, guards.OwnPose(1.0, -1.0, 0., .02, .02))
    b = guards.BlockageStreak.key_of('door_1', guards.OwnPose(1.9, 0.05, 0., .02, .02))
    assert not s.observe(0., 'yes', True, a) and not s.observe(1., 'yes', True, b)   # two places: no streak
    assert s.observe(2., 'yes', True, b)                                             # same place twice
    assert not s.observe(3., 'yes', True, b)                                         # disarmed at that place
    assert not s.observe(4., 'no', False, b) and not s.observe(5., 'yes', True, b)
    assert s.observe(6., 'yes', True, b)                                             # re-armed by a 'no'
    s2 = guards.BlockageStreak(2)
    assert not s2.observe(0., 'yes', True, a) and not s2.observe(5., 'yes', True, a)  # gap > max: reset
    assert not s2.observe(1., 'yes', True, None)


def test_p2_blockage_needs_two_judgments_at_the_same_place(monkeypatch):
    """Codex P2-7: a commit with passage None, then one at door_1, made 'blockage_seen(door_1)'."""
    from harness import zone_own_perception
    monkeypatch.setattr(zone_own_perception, 'judge_route_blockage',
                        lambda *a, **k: {'answer': 'yes', 'confidence': .9, 'reason': 'fixture'})
    ex = zox.ZoneOwnExecutor('r1', MAP, make().params, {'orders': []}, skill_factory=lambda o, robot_id: None,
                             pose_estimate_cls=tuple, search_rows_y=(), mode='diagnostic',
                             pose_source=ScriptedPose(lambda t: (0.5 if t < .5 else 1.9, 0.05, 0., .02, .01, .1)))
    d = Driver(ex)
    d.servo.update(LOOK_P20)                           # agreed posture, pan 1500, standing still
    ex.hold(10.)
    d.run(2.1)                                         # judgments at t = 0 (x 0.5), 1 and 2 (door)
    seen = [e for e in ex.drain_events() if e['event'] == 'blockage_seen']
    assert [j['passage_id'] for j in ex.judgment_log if j['judgment'] == 'route_blockage'] == [None, 'door_1', 'door_1']
    assert len(seen) == 1 and seen[0]['sim_s'] >= 1.9 and seen[0]['detail']['passage_id'] == 'door_1'
