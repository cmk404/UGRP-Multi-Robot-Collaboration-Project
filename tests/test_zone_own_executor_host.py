"""Package F host cancel path (issue #221; Codex review 2 of PR #206, P1-4 / P1-5, earlier audit P1).

abort, a job's local deadline, the episode horizon and a controller exception must each (1) drop the
robot's scheduled macro commands, (2) hold it at once and (3) end the job with exactly one terminal
event; a stopped robot refuses new jobs.

``FakeHost`` runs the REAL ``OwnCamTeamHost`` scheduling code (run loop, macro timeline, call, cancel)
on a sim-free clock and ports; ``test_team_host_*`` builds the MuJoCo world (skipped without MuJoCo;
CI has no simulator by design, requirements-test.txt).
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_own_executor as zox  # noqa: E402
from tests.test_zone_own_executor import CALIB, MAP, ROWS_Y, SEARCH_POSE, SHEET, _reachable, obs, rgb_of  # noqa: E402

OwnCamTeamHost = zox.OwnCamTeamHost
_RobotSlot = sys.modules[OwnCamTeamHost.__module__]._RobotSlot


class FakePort:
    """Own-port stand-in: records every issued command and hold with its SIM time."""

    def __init__(self, rid):
        self.rid, self.log, self.servo, self.fid = rid, [], dict(SEARCH_POSE), 0

    def apply(self, action, now):
        self.log.append((round(now, 4), action['kind'], dict(action)))
        if action['kind'] == 'arm':
            self.servo[int(action['servo_id'])] = int(action['pulse'])
        elif action['kind'] == 'look':
            self.servo[6] = int(action['pan_pulse'])

    def hold(self, now):
        self.log.append((round(now, 4), 'hold', {}))

    def tick(self, now):
        pass


class MacroExecutor(zox.ZoneOwnExecutor):
    """Test double: a hold job whose first decision is a wrist ``pose`` macro (servo 3 by 660 PWM, ~1.1 s)."""

    def _step_hold(self, now, job):
        if not getattr(job, 'macro_sent', False):
            job.macro_sent = True
            return {'mode': 'macro', 'action': {'kind': 'pose', 'pulses': {3: 1400}}}
        return super()._step_hold(now, job)


class RaisingExecutor(zox.ZoneOwnExecutor):
    """Test double: the controller raises at t >= 1 s."""

    def _step_hold(self, now, job):
        if now >= 1.:
            raise RuntimeError('fixture controller fault')
        return super()._step_hold(now, job)


def executor(cls=zox.ZoneOwnExecutor, rid='r1', **kw):
    kw.setdefault('judgments', False)
    return cls(rid, MAP, CALIB['params'], SHEET, skill_factory=lambda order, robot_id: None, pose_estimate_cls=tuple,
               search_rows_y=ROWS_Y, **kw)


class FakeHost(OwnCamTeamHost):
    """The real host scheduling code on a sim-free clock (0.05 s physics chunks) and fake own ports."""

    def __init__(self, executors, layer, hooks=()):
        self.world = types.SimpleNamespace(data=types.SimpleNamespace(time=0.),
                                           model=types.SimpleNamespace(opt=types.SimpleNamespace(timestep=.05)),
                                           close=lambda: None)
        self.robots = {rid: _RobotSlot(rid, FakePort(rid), ex) for rid, ex in executors.items()}
        self.study_layer, self.api_calls, self.event_log = layer, [], []
        self.closed, self.frames_dir, self.assigned_box, self._last_contact = False, None, {}, {}
        self.eval_only = {'frames_eval': []}
        self.hooks = sorted(hooks, key=lambda h: h[0])
        for rid, slot in self.robots.items():
            self._sink(rid, {'t': 0., 'kind': 'initial_servo_command', 'pulses': dict(slot.port.servo)})

    def _physics_until(self, t_end):
        d = self.world.data
        while d.time < t_end - 1e-9:
            d.time = round(d.time + .05, 6)
            while self.hooks and d.time + 1e-9 >= self.hooks[0][0]:
                self.hooks.pop(0)[1](self)
            for rid, s in self.robots.items():
                if not s.dead and d.time + 1e-9 >= s.next_frame:
                    self._capture(rid, d.time)

    def _capture_raw(self, rid, now):
        slot = self.robots[rid]
        slot.port.fid += 1
        o = obs(rid, slot.port.fid, now, slot.port.servo)
        slot.executor.on_frame(now, o, rgb_of(o))
        slot.frames.append({'t': now, 'robot_id': rid, 'camera': 'robot_cam'})
        slot.next_frame = now + self.FRAME_S


def team(r1, **others):
    exs = {'r1': r1, 'r2': others.get('r2') or executor(rid='r2'), 'r3': others.get('r3') or executor(rid='r3')}
    return exs


def terminal(host, rid):
    return [e for e in host.event_log if e['robot_id'] == rid and e['event'] in ('job_done', 'job_failed')]


def issued_after(port, t, kinds=('arm', 'look', 'mecanum', 'drive')):
    return [row for row in port.log if row[0] > t + 1e-9 and row[1] in kinds]


def start_hold(duration):
    def layer(host, kind, event, now):
        if kind == 'start':
            for rid in host.robots:
                host.call(rid, 'hold', duration)
    return layer


def test_p1_abort_drops_scheduled_macro_commands_and_holds_at_once():
    """Earlier audit P1: abort() ended the job but the host still issued the scheduled arm commands."""
    acks = []
    host = FakeHost(team(executor(MacroExecutor)), start_hold(10.),
                    hooks=[(.9, lambda h: acks.append(h.call('r1', 'abort', 'peer_asked')))])
    host.run(3.)
    port = host.robots['r1'].port
    assert any(row[1] == 'arm' and row[0] < .9 for row in port.log)          # the macro was running
    assert acks[0]['accepted']
    assert not issued_after(port, .9), issued_after(port, .9)[:3]           # nothing scheduled survives the abort
    assert any(row[1] == 'hold' and abs(row[0] - .9) < 1e-6 for row in port.log)
    ends = terminal(host, 'r1')
    assert len(ends) == 1 and ends[0]['detail']['reason'] == 'ABORTED:peer_asked'


def test_p1_local_deadline_is_enforced_while_a_macro_runs():
    """Codex P1-5: with deadline 1.0 s the scheduled gripper/arm command was still issued at 1.2 s."""
    host = FakeHost(team(executor(MacroExecutor, job_sim_limit_s=.8)), start_hold(10.))
    host.run(3.)
    port = host.robots['r1'].port
    deadline = .5 + .8
    ends = terminal(host, 'r1')
    assert len(ends) == 1 and ends[0]['detail']['reason'] == 'LOCAL_TIMEOUT'
    assert ends[0]['sim_s'] <= deadline + .06 and ends[0]['scheduler_trigger'] == 'timeout'
    assert not issued_after(port, deadline + .06)
    assert any(row[1] == 'hold' and deadline <= row[0] <= deadline + .06 for row in port.log)


def test_p1_episode_horizon_ends_every_active_job_once_and_refuses_later_calls():
    """Codex P1-5: at the episode SIM limit a job stayed alive with only 'job_started'."""
    host = FakeHost(team(executor(MacroExecutor)), start_hold(100.))
    out = host.run(2.)
    assert out['outcome'] == 'SIM_LIMIT'
    for rid in host.robots:
        ends = terminal(host, rid)
        assert len(ends) == 1 and ends[0]['detail']['reason'] == 'EPISODE_END:SIM_LIMIT', (rid, ends)
        assert host.robots[rid].executor.job is None
    late = host.call('r2', 'hold', 1.)
    assert not late['accepted'] and late['rejected_reason'] == 'EPISODE_ENDED'


def test_p1_a_stopped_robot_refuses_new_jobs():
    """Codex P1-4: after a controller exception hold(2) returned accepted=True and never ended."""
    acks = []
    host = FakeHost(team(executor(RaisingExecutor)), start_hold(5.),
                    hooks=[(1.5, lambda h: acks.append(h.call('r1', 'hold', 2.)))])
    host.run(3.)
    slot = host.robots['r1']
    assert slot.dead and slot.exception['type'] == 'RuntimeError'
    assert not acks[0]['accepted'] and acks[0]['rejected_reason'] == 'ROBOT_STOPPED'
    ends = terminal(host, 'r1')
    assert len(ends) == 1 and ends[0]['detail']['reason'] == 'EXCEPTION:RuntimeError'
    assert not issued_after(slot.port, 1.)
    for api, args in (('goto', ('A',)), ('look_around', ()), ('deliver', ('o1', 'A2')), ('abort', ())):
        ack = host.call('r1', api, *args)
        assert not ack['accepted']


def test_every_accepted_job_has_exactly_one_terminal_event():
    host = FakeHost(team(executor(MacroExecutor, job_sim_limit_s=.8)), start_hold(.3),
                    hooks=[(1.6, lambda h: h.call('r2', 'hold', 5.)), (1.8, lambda h: h.call('r2', 'abort')),
                           (1.9, lambda h: h.call('r3', 'look_around'))])
    host.run(2.5)
    for rid in host.robots:
        started = [e['job_id'] for e in host.event_log if e['robot_id'] == rid and e['event'] == 'job_started']
        ended = [e['job_id'] for e in terminal(host, rid)]
        assert sorted(started) == sorted(ended) and len(set(ended)) == len(ended), (rid, started, ended)


def test_contact_log_dedupes_per_robot_and_kind():
    """Earlier audit: dedupe compared with the global last row, so interleaved robots were never de-duplicated."""
    host = object.__new__(OwnCamTeamHost)
    host.eval_only = {'kind_steps': {r: {} for r in ('r1', 'r2', 'r3')}, 'contacts': []}
    host._last_contact = {}
    for i in range(10):                                  # r1 and r2 touch walls on alternating steps
        host._record_contacts(i * .01, {'r1': {'wall'} if i % 2 == 0 else set(), 'r2': {'wall'} if i % 2 else set(),
                                        'r3': set()})
    rows = [(c['robot_id'], c['kind']) for c in host.eval_only['contacts']]
    assert rows == [('r1', 'wall'), ('r2', 'wall')]
    assert host.eval_only['kind_steps']['r1']['wall'] == 5
    host._record_contacts(.15, {'r1': {'wall'}, 'r2': set(), 'r3': set()})
    assert len(host.eval_only['contacts']) == 2          # continuous contact (last 0.07 s ago) stays one row
    host._record_contacts(.5, {'r1': {'wall'}, 'r2': set(), 'r3': set()})
    assert len(host.eval_only['contacts']) == 3


# ---------------------------------------------------------------- MuJoCo host (not in CI: no simulator there)
def test_team_host_isolation_abort_and_horizon_on_the_real_world():
    pytest.importorskip('mujoco')
    spec = {'map': 'zone_wide_door_tags_v2', 'seed': 703, 'goal': {'A': {'cyan': 1}, 'B': {'cyan': 1}, 'C': {'cyan': 1}},
            'extra_boxes': {'red': 2, 'green': 1}, 'contact_profile': 'cargo_noslip_v1', 'order_sheet': SHEET}
    student = {'mode': 'm1', 'calibration': 'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json',
               'skill_module': 'harness.wrist_zone_skill_v9', 'skill_class': 'WristZoneDeliveryV9'}
    seen, abort = [], {}

    def layer(host, kind, event, now):
        if kind == 'start':
            for rid in zox.ROBOTS:
                host.call(rid, 'look_around')
        else:
            seen.append(event)

    host = OwnCamTeamHost(spec, student, root=ROOT, study_layer=layer)
    physics = host._physics_until

    def physics_with_abort(t_end):
        if 'ack' not in abort and float(host.world.data.time) >= 1.5:
            abort['t'] = float(host.world.data.time)
            abort['ack'] = host.call('r1', 'abort', 'test')
            port = host.robots['r1'].port
            abort['port'] = {'servo_targets': dict(port._servo_targets), 'motors': list(port._motor_commands)}
        physics(t_end)
    host._physics_until = physics_with_abort
    try:
        out = host.run(3.)
        assert out['outcome'] == 'SIM_LIMIT'
        assert host.eval_only['max_eq_active'] == 0 and host.contact_record['noslip_iterations'] > 0
        assert host.contact_record['user_decision'].startswith('approved by the user 2026-09-26')
        for rid, slot in host.robots.items():
            ex = slot.executor
            assert slot.frames and all(f['robot_id'] == rid and f['camera'] == 'robot_cam' for f in slot.frames)
            assert ex.cameras_seen == {'robot_cam'} and all(s.startswith('owncam_pf_v2:') for s in ex.pose_sources_seen)
            reach = _reachable(ex, limit=200_000)
            assert id(host) not in reach and id(host.world) not in reach
            assert not any(id(s.port) in reach or id(s.executor) in reach for r, s in host.robots.items() if r != rid)
            assert slot.commands[0]['kind'] == 'initial_servo_command'
            ends = terminal(host, rid)
            assert len(ends) == 1, (rid, ends)
        assert abort['ack']['accepted'] and abort['port'] == {'servo_targets': {}, 'motors': [0., 0., 0., 0.]}
        assert terminal(host, 'r1')[0]['detail']['reason'] == 'ABORTED:test'
        r1_after = [c for c in host.robots['r1'].commands if c['t'] > abort['t'] + 1e-6 and c['kind'] != 'hold']
        assert not r1_after, r1_after[:3]
        assert all(terminal(host, r)[0]['detail']['reason'] == 'EPISODE_END:SIM_LIMIT' for r in ('r2', 'r3'))
        assert {e['robot_id'] for e in seen if e['event'] == 'job_started'} == set(zox.ROBOTS)
        assert not host.call('r2', 'look_around')['accepted']
    finally:
        host.close()
