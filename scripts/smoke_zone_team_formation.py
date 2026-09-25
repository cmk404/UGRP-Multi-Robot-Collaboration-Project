#!/usr/bin/env python3
"""Synchronous-SIM smoke of the A1 team-job pieces on an open floor patch (GT TEACHER, weld OFF).

Phase A1 of the zone team-carry work (2026-09-25). This is NOT the zone
dispatch executor (that is A2). It checks, in physics, that the new pure
pieces compose into one carry:

  role claims -> RendezvousRule (stations physically occupied) -> TeamJobLedger
  commit -> TeamJob states behind PhaseBarriers -> FormationPlan targets
  (approach, IK, virtual-structure carry) -> lower, release, retreat.

The scene, fixture placement, truth metrics, video and the base result are
``scripts.probe_zone_cargo.Probe`` (PR #164); only the teacher is replaced.
Robots start at staggered stand-offs behind their stations so they arrive at
different times and the rendezvous really waits. Every physics step asserts
``eq_active`` is all zero (Probe.run). Teacher truth (poses, contact forces)
drives every decision: this is a teacher-condition result, never a student or
RGB success.

  .venv-sim/bin/python -m scripts.smoke_zone_team_formation --probe pair_beam --output outputs/zone-team-jobs/pair_beam
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.zone_team_formation import FormationPlan, reference  # noqa: E402
from harness.zone_team_jobs import RendezvousRule, RoleClaim, TeamJobLedger  # noqa: E402
from scripts.cargo_formation_teacher import CLOSED, FINISH_ERR_M, FINISH_ERR_RAD, FINISH_SETTLE_S, OPEN  # noqa: E402
from scripts.cargo_formation_teacher import FormationTeacher  # noqa: E402
from scripts.probe_zone_cargo import BASE_Z, PROBES, Probe  # noqa: E402
from scripts.zone_teacher import FOLDED  # noqa: E402

SMOKES = ('pair_beam', 'trio_frame')
# Stand-off behind each station at the start (m), by formation order: robots
# arrive at different SIM times, so the first ones wait at the rendezvous.
STANDOFF_M = (.30, .55, .80)
ITEM = 'probe'
ZONE = 'A'            # symbolic destination for the spec hash (open patch, no zone here)
STATE_LIMIT_S = {'COMMITTED': 5., 'RENDEZVOUS': 40., 'PREGRASP': 10., 'CLOSE': 10., 'LIFT': 15.,
                 'LOWER': 10., 'RELEASE': 10., 'RETREAT': 15.}
DRIVE_LIMIT = {'forward': (-.05, .15), 'left': (-.10, .10), 'turn': (-.15, .15)}


class TeamJobTeacher(FormationTeacher):
    """Drives one TeamJob through its states with the A1 pure pieces.

    Inherits only truth readers and command plumbing from the PR #164 teacher
    (robot_pose, cargo_pose, finger_forces, bilateral, ArmSequence per robot).
    ``phase`` keeps the PR #164 names so Probe's metrics and summary apply.
    """
    def __init__(self, world, ports, instance, roles, *, legs, log, hold_s=1.5):
        super().__init__(world, ports, instance, roles, legs=legs, log=log, hold_s=hold_s)
        self.rule = RendezvousRule()
        self.claims = {rid: RoleClaim(rid, ITEM, instance.kind, ZONE, role) for rid, role in self.roles.items()}
        self.ledger = TeamJobLedger({ZONE: {instance.kind: 1}})
        self.plan = FormationPlan(instance.kind, self.roles)
        self.job = None
        self.phase = 'to_station'
        self.arrived = {}
        self.station_log = []
        self.barriers = []
        self.state_t = 0.
        self.arm_plans = {}
        self.lift_ok_at = None
        self.release_at = None

    # --- helpers ---
    def poses(self):
        return {rid: self.robot_pose(rid) for rid in ('r1', 'r2', 'r3')}

    def _arms_idle(self, rid, now):
        return self.arms[rid].tick(now) and not self.arms[rid].events

    def _report(self, rid, now):
        if self.job.report_ready(rid, now):
            pass

    def _advance(self, now):
        state = self.job.state
        ready = dict(self.job.barrier.ready)
        nxt = self.job.advance(now)
        if nxt:
            self.barriers.append({'state': state, 'ready_t': {r: round(t, 3) for r, t in ready.items()},
                                  'spread_s': round(max(ready.values()) - min(ready.values()), 3) if ready else 0.,
                                  'advanced_t': round(now, 3), 'next': nxt})
            self.log('team_state', now, state=nxt, job=self.job.job_id)
            self.state_t = now
        return nxt

    def _fail(self, rid, reason, now):
        r = self.job.fail(rid, reason, now)
        self.log('team_failure', now, robot=rid, reason=reason, action=r['action'], state=self.job.state)
        self.outcome = self.outcome or f'aborted:{reason}'
        if r['action'] == 'hold_lower':
            for p in self.roles:
                self.ports[p].hold(now)
            self._queue_lower(now)
        elif r['action'] == 'cancel_retreat':
            for p in self.roles:
                self.ports[p].hold(now)
            self._queue_retreat(now)
        self.state_t = now
        return r

    def _queue_lower(self, now):
        for rid, arm in self.arms.items():
            arm.queue({**self.arm_plans[rid]['descent'][-1], 1: CLOSED}, now, duration=1.2, settle=.4)
        self._set('lower', now)

    def _queue_retreat(self, now):
        for rid, arm in self.arms.items():
            hover = self.arm_plans[rid]['hover'] if rid in self.arm_plans else {k: v for k, v in FOLDED.items() if k != 1}
            arm.queue({**hover, 1: OPEN}, now, duration=.6)
            arm.queue(FOLDED, now)
        self._set('retract', now)
        self.release_at = None

    # --- main tick ---
    def tick(self, now, metrics):
        for arm in self.arms.values():
            arm.tick(now)
        if self.phase == 'done':
            return
        if now + 1e-9 < self.next_tick:
            return
        self.next_tick = now + .1
        item = self.cargo_pose()
        if self.job is None:
            self._to_station(now, item)
            return
        state, el = self.job.state, now - self.state_t
        if state in STATE_LIMIT_S and el > STATE_LIMIT_S[state] and not self.job.aborting:
            late = self.job.barrier.missing or list(self.roles)
            self._fail(late[0], f'{state.lower()}_timeout', now)
            return
        getattr(self, '_' + state.lower())(now, item, metrics)

    def _to_station(self, now, item):
        poses = self.poses()
        occ = self.rule.occupancy(self.claims, poses, {ITEM: item})
        stations = self.plan.stations(item)
        for rid in self.roles:
            if occ[rid] == 'at_station':
                self.arrived.setdefault(rid, now)
                self.ports[rid].hold(now)
                continue
            x, y, a = poses[rid]
            sx, sy, sa = stations[rid]
            c, s = math.cos(a), math.sin(a)
            ex, ey = c*(sx-x) + s*(sy-y), -s*(sx-x) + c*(sy-y)
            self.ports[rid].apply({'kind': 'mecanum', 'duration_s': .15,
                                   'forward': max(-.05, min(.08, .9*ex)), 'left': max(-.06, min(.06, .9*ey)),
                                   'turn': max(-.10, min(.10, .8*math.remainder(sa-a, 2*math.pi)))}, now)
        self.station_log.append({'t': round(now, 2), 'occupancy': dict(occ)})
        for rid, t0 in self.arrived.items():
            if self.rule.expired(t0, now):
                self.outcome = 'rendezvous_timeout'
                self.log('rendezvous_timeout', now, robot=rid)
                self._set('done', now)
                return
        teams = self.rule.teams(self.claims, occ)
        if teams:
            self.job = self.ledger.commit(teams[0].values(), now)
            self.state_t = now
            self.log('team_commit', now, job=self.job.job_id, role_by_robot=self.job.role_by_robot,
                     arrived={r: round(t, 2) for r, t in self.arrived.items()})
            self._set('approach', now)

    def _committed(self, now, item, metrics):
        for rid in self.roles:
            self._report(rid, now)
        self._advance(now)

    def _rendezvous(self, now, item, metrics):
        poses = self.poses()
        for rid in self.roles:
            cmd, _ = self.plan.approach_command(rid, poses[rid], item)
            if cmd is None:
                self.ports[rid].hold(now)
                self._report(rid, now)
            elif rid not in self.job.barrier.ready:
                self.ports[rid].apply({'kind': 'mecanum', 'duration_s': .12, **cmd}, now)
        if self._advance(now) == 'PREGRASP':
            # One synchronized arm start for everyone: plans from the aligned poses.
            poses = self.poses()
            for rid, arm in self.arms.items():
                self.arm_plans[rid] = self.plan.arm_plan(rid, poses[rid], item)
                arm.queue({**self.arm_plans[rid]['hover'], 1: OPEN}, now, duration=1.0)
            self._set('grasp', now)

    def _pregrasp(self, now, item, metrics):
        for rid in self.roles:
            if self._arms_idle(rid, now):
                self._report(rid, now)
        if self._advance(now) == 'CLOSE':
            for rid, arm in self.arms.items():
                for pose in self.arm_plans[rid]['descent']:
                    arm.queue(pose, now, duration=.12, settle=0.)
                arm.queue({1: CLOSED}, now, duration=.5, settle=.4)

    def _close(self, now, item, metrics):
        forces = self.finger_forces()
        for rid in self.roles:
            if self._arms_idle(rid, now) and min(forces[rid]) >= .5:
                self._report(rid, now)
        if self._advance(now) == 'LIFT':
            self.detail['grasp_forces_n'] = {r: [round(x, 2) for x in v] for r, v in forces.items()}
            for rid, arm in self.arms.items():
                arm.queue({**self.arm_plans[rid]['lift'], 1: CLOSED}, now, duration=1.2, settle=.3)
            self._set('lift', now)
            self.lift_ok_at = None
        elif all(self._arms_idle(r, now) for r in self.roles) and now - self.state_t > 3.:
            missing = [r for r in self.roles if min(forces[r]) < .5]
            if missing:
                self._fail(missing[0], 'grasp_contact_missing', now)

    def _lift(self, now, item, metrics):
        if self.phase == 'lift' and all(self._arms_idle(r, now) for r in self.roles):
            self._set('hold', now)
        if self.phase != 'hold':
            return
        clear = metrics['cargo_min_z'] > .012 and all(self.bilateral().values())
        self.lift_ok_at = (self.lift_ok_at or now) if clear else None
        if self.lift_ok_at is not None and now - self.lift_ok_at >= self.hold_s:
            self.lifted_clear = True
            self.detail['hold_min_z_m'] = round(metrics['cargo_min_z'], 4)
            for rid in self.roles:
                self._report(rid, now)
        if self._advance(now) == 'CARRY':
            self.ref = reference(item, self.legs)
            self.ref_total_s = self.ref.total_s
            self.ref_done_at = None
            self.lost_since = None
            self._set('carry', now)
        elif now - self.phase_t > 6.:
            self.detail['hold_min_z_m'] = round(metrics['cargo_min_z'], 4)
            self._fail(self.roles and sorted(self.roles)[0], 'not_lifted_clear', now)

    def _carry(self, now, item, metrics):
        if not isinstance(metrics, dict) or self.job.state != 'CARRY':
            return
        lost = [rid for rid, ok in self.bilateral().items() if not ok]
        if lost:
            self.lost_since = self.lost_since or now
            if now - self.lost_since > .3:
                self._fail(lost[0], 'grip_lost_in_transit', now)
                return
        else:
            self.lost_since = None
        step = self.plan.carry_step(self.ref.pose, self.ref.velocity(), self.poses())
        metrics['track_err_m'] = step['track_err_m']
        if step['advance']:
            self.ref.advance(.1)
        if self.ref.finished and self.ref_done_at is None:
            self.ref_done_at = now
        if self.ref.finished and ((step['track_err_m'] < FINISH_ERR_M and step['track_err_rad'] < FINISH_ERR_RAD)
                                  or now - self.ref_done_at >= FINISH_SETTLE_S):
            self.detail['finish_track_err_m'] = round(step['track_err_m'], 4)
            for rid in self.roles:
                self.ports[rid].hold(now)
                self._report(rid, now)
            if self._advance(now) == 'LOWER':
                self._queue_lower(now)
            return
        if now - self.state_t > 3*self.ref_total_s + 30:
            self._fail(sorted(self.roles)[0], 'carry_timeout', now)
            return
        for rid, cmd in step['commands'].items():
            self.ports[rid].apply({'kind': 'mecanum', 'duration_s': .15, **cmd}, now)

    def _lower(self, now, item, metrics):
        for rid in self.roles:
            if self._arms_idle(rid, now):
                self._report(rid, now)
        if self._advance(now) == 'RELEASE':
            for arm in self.arms.values():
                arm.queue({1: OPEN}, now, duration=.4, settle=.5)
            self._set('release', now)

    def _release(self, now, item, metrics):
        for rid in self.roles:
            if self._arms_idle(rid, now):
                self._report(rid, now)
        if self._advance(now) == 'RETREAT':
            self._queue_retreat(now)

    def _retreat(self, now, item, metrics):
        if self.phase == 'retract':
            if all(self._arms_idle(r, now) for r in self.roles):
                self._set('back_off', now)
            return
        if self.phase == 'back_off':
            if now - self.phase_t < 2.5:
                for rid in self.roles:
                    self.ports[rid].apply({'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0.,
                                           'duration_s': .15}, now)
                return
            for rid in self.roles:
                self.ports[rid].hold(now)
                self._report(rid, now)
            nxt = self._advance(now)
            if nxt:
                if nxt == 'FINISHED' and self.outcome is None:
                    self.outcome = 'completed_sequence'
                self._set('done', now)


def run(name, output, *, video=True):
    spec = dict(PROBES[name])
    probe = Probe(name, spec, output, video=video)
    # Re-place the fixture with staggered stand-offs behind the stations (setup only).
    from sim.zone_cargo import world_grasps
    grasps = world_grasps(probe.inst)
    for k, (rid, role) in enumerate(sorted(spec['roles'].items())):
        x, y, a = grasps[role]['base_xyyaw']
        d = STANDOFF_M[k]
        probe.world.robot(rid).set_base_pose_for_test((x - d*math.cos(a), y - d*math.sin(a), BASE_Z), a)
    import mujoco
    mujoco.mj_forward(probe.m, probe.d)
    probe.teacher = TeamJobTeacher(probe.world, {r: probe.ports[r] for r in probe.roles}, probe.inst, probe.roles,
                                   legs=spec['legs'], log=probe._log)
    probe.teacher.start_pose = probe.teacher.cargo_pose()
    result = probe.run()
    t = probe.teacher
    job = t.job.record() if t.job else None
    result.update({
        'schema': 'ugrp.zone_team_job_smoke.v1', 'smoke': name,
        'executor': 'scripts.smoke_zone_team_formation.TeamJobTeacher (A1 pure pieces; GT teacher)',
        'standoff_m': {rid: STANDOFF_M[k] for k, rid in enumerate(sorted(spec['roles']))},
        'rendezvous_rule': t.rule.record(), 'arrived_sim_s': {r: round(v, 2) for r, v in t.arrived.items()},
        'team_job': job, 'ledger': t.ledger.record(), 'barriers': t.barriers,
        'weld_path_used': False,
    })
    (Path(output)/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    (Path(output)/'station-log.json').write_text(json.dumps(t.station_log) + '\n')
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--probe', choices=SMOKES, required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--no-video', action='store_true')
    a = p.parse_args(argv)
    r = run(a.probe, a.output, video=not a.no_video)
    print(json.dumps({k: r.get(k) for k in ('smoke', 'success', 'teacher_outcome', 'lifted_clear', 'placement',
                                            'eq_active_max', 'arrived_sim_s', 'max_slip_mm', 'max_cargo_tilt_deg',
                                            'sim_time_s', 'wall_s', 'load_avg_1m')}, indent=2))
    print('team_job state:', (r.get('team_job') or {}).get('state'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
