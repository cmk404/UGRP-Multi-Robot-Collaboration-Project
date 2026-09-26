"""GT TEACHER fixes for PR #169 feasibility blockers B3 and B4 (zone teacher fix, 2026-09-26).

TEACHER-ONLY. The teacher reads simulator truth (poses, contact forces) and
exists for demonstrations, training targets and scenario feasibility. It is
never the study executor, and a teacher delivery is never a robot success.

This module subclasses the PR #169 team teacher (``scripts.zone_team_teacher``,
not edited) and is selected only by ``scripts/run_zone_teacher_fix.py``. It
does not change ``harness.zone_protocol_v2.CONDITIONS``, the ``ModeLoop``
subclasses, the rendezvous rule, the claim checks or anything a robot's model
reads; robots still get only the two receipts.

B3 (claim-holding robot on a carrying team's route; record: r3 waited at the
heavy_crate west station on the beam route, the carry paused 3 times and ended
``carry_blocked``). Root cause: ``carry_blockers`` asked only IDLE robots to
yield, and a robot with a live claim drove back to (and waited at) a station
inside the team's route. Fix (switch ``b3_claim_yield``):
  1. a robot with a live claim that is not yet in a job (``to_station`` /
     ``waiting``) inside the team's lookahead hull is asked to yield like an
     idle robot;
  2. while its claimed station lies inside a carrying team's hull over
     ``STAGE_LOOKAHEAD_S``, the robot holds where it is (``station_staged``)
     instead of driving to the station; afterwards it re-plans to the station;
  3. leaving the station this way resets its own arrival time, so the 60 SIM s
     rendezvous wait counts from its next physical arrival (the rule itself is
     unchanged). Inputs: the carrying team's held item and its planned route
     (a physical team in motion), never a claim, goal or robot id.

B4 (``hold_lower`` set a beam down where it blocked another item's station).
Root cause: a post-contact failure lowered the item wherever the team stood.
Fix (switch ``b4_return_setdown``): a team failure in CARRY that is only about
motion (``carry_blocked``, ``carry_timeout``) with both grips intact first
carries the item back along the part of its route already travelled to the
item's start pose (clear of stations and lanes by the scenario placement), then
lowers there (``hold_lower`` as before). Grip loss, a failed return or a
second failure lowers in place as before (recorded).
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

from harness.static_keepouts import polygons_overlap
from harness.zone_team_footprint import circle
from harness.zone_team_route import PoseReference
from scripts.cargo_formation_teacher import FINGER_MIN_N
from scripts.zone_team_teacher import CARRY_LOOKAHEAD_S, ROBOT_DISC_M, TeamCarry, TeamRobot, ZoneTeamExecutor

PROFILE = 'team_teacher_fix_v1'
DEFAULT_SWITCHES = {'b3_claim_yield': True, 'b4_return_setdown': True}
STAGE_LOOKAHEAD_S = 2*CARRY_LOOKAHEAD_S
RETURN_REASONS = ('carry_blocked', 'carry_timeout')
PRE_JOB = ('to_station', 'waiting')


def source_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _pre_job(claim):
    return claim is not None and claim.live and claim.state in PRE_JOB


def return_poses(ref, here):
    """Item poses back to the route start: the current pose, then the passed waypoints in reverse."""
    passed = list(ref.poses[:ref.leg + 1])
    poses = [tuple(here)]
    for p in reversed(passed):
        if math.dist(p[:2], poses[-1][:2]) > 1e-4 or abs(p[2] - poses[-1][2]) > 1e-4:
            poses.append(tuple(p))
    if len(poses) == 1:
        poses.append(tuple(passed[0]))
    return poses


def station_in_hulls(station_xy, hulls, radius=ROBOT_DISC_M):
    disc = circle(station_xy[0], station_xy[1], radius)
    return any(polygons_overlap(h, disc) for h in hulls)


class FixTeamRobot(TeamRobot):
    """TeamRobot that stays off a carrying team's route while holding a claim (B3)."""
    ex = None
    staged = None

    def tick(self, now, discs_for):
        c = self.claim
        pre = _pre_job(c)
        if self.ex is not None and self.ex.switches['b3_claim_yield'] and pre and not self.yield_req \
                and not self.passage_yield:
            team = self.ex.station_in_team_path(self)
            if team:
                if self.staged is None:
                    self.staged = (team, now)
                    self.ex.fix_events['station_staged'] += 1
                    self.log('station_staged', self.rid, now, team=team, own_phase=self.phase)
                self.arm.tick(now)
                self.port.hold(now)
                return
            if self.staged is not None:
                self.log('station_unstaged', self.rid, now, team=self.staged[0],
                         held_s=round(now - self.staged[1], 2))
                self.staged = None
                self._set('to_box', now, after='staged')
        was_yielding = self.yield_req is not None
        super().tick(now, discs_for)
        if was_yielding and self.yield_req is None and pre and self.claim is c and self.phase == 'align_box':
            # the straight station hold is only for short range: re-plan from the yield spot
            self._set('to_box', now, after='yield')


class FixTeamCarry(TeamCarry):
    """TeamCarry that returns the item to its start pose before a motion-only hold_lower (B4)."""
    returning = None

    def _grips_ok(self):
        forces = self.ex.finger_forces(self.item_id, self.parts)
        return all(min(forces[r]) >= FINGER_MIN_N for r in self.parts)

    def fail(self, rid, reason, now, **detail):
        ex = self.ex
        if (ex.switches['b4_return_setdown'] and self.n > 1 and self.job.state == 'CARRY' and self.returning is None
                and reason in RETURN_REASONS and self.ref is not None and self._grips_ok()):
            return self._start_return(rid, reason, now, detail)
        if self.returning is not None and 'outcome' not in self.returning:
            self.returning.update(outcome='lowered_in_place_after_return_failure', end_reason=reason,
                                  t1=round(now, 2))
            ex.fix_events['return_failed'] += 1
        return super().fail(rid, reason, now, **detail)

    def _start_return(self, rid, reason, now, detail):
        here = self.ex.item_pose(self.item_id)
        poses = return_poses(self.ref, here)
        if self.pause is not None:
            self.pause['t1'] = round(now, 2)
            self.pause['s'] = round(now - self.pause['t0'], 2)
            self.record['pauses'].append(self.pause)
            self.pause = None
        self.ref = PoseReference(poses)
        self.ref_done_at = None
        self.carry_t, self.pause_total = now, 0.
        self.returning = {'robot': rid, 'reason': reason, 't0': round(now, 2),
                          'detail': {k: v for k, v in detail.items() if k != 'by'} | (
                              {'by': sorted(detail['by'])} if 'by' in detail else {}),
                          'from_pose': [round(v, 4) for v in here], 'to_pose': [round(v, 4) for v in poses[-1]],
                          'legs': len(poses) - 1}
        self.record.setdefault('returns', []).append(self.returning)
        self.ex.fix_events['return_started'] += 1
        self.ex.log('team_return', None, now, job=self.job.job_id, reason=reason, legs=len(poses) - 1)
        return {'action': 'return_to_start', 'robots': list(self.parts), 'state': self.job.state}

    def advance(self, now):
        if self.returning is not None and self.job.state == 'CARRY':
            if self.job.barrier.complete and 'outcome' not in self.returning:
                pose = self.ex.item_pose(self.item_id)
                self.returning.update(outcome='returned_to_start', t1=round(now, 2),
                                      end_pose=[round(v, 4) for v in pose],
                                      end_err_m=round(math.dist(pose[:2], self.returning['to_pose'][:2]), 4))
                self.ex.fix_events['return_finished'] += 1
                r = self.returning
                TeamCarry.fail(self, r['robot'], r['reason'], now, returned_to_start=True)
            return None
        return super().advance(now)


class FixZoneTeamExecutor(ZoneTeamExecutor):
    """ZoneTeamExecutor with the B3/B4 teacher fixes (per-switch, recorded in ``record()``)."""
    def __init__(self, world, ports, static_map, items, goal, log, *, robot_event=None, inject=None, switches=None):
        super().__init__(world, ports, static_map, items, goal, log, robot_event=robot_event, inject=inject)
        unknown = set(switches or {}) - set(DEFAULT_SWITCHES)
        if unknown:
            raise ValueError(f'unknown teacher fix switches: {sorted(unknown)}')
        self.switches = {**DEFAULT_SWITCHES, **(switches or {})}
        self.robots = {rid: FixTeamRobot(rid, world, port, static_map, log) for rid, port in ports.items()}
        for robot in self.robots.values():
            robot.team = self.robots
            robot.ex = self
        self.fix_events = {k: 0 for k in ('claim_yield', 'station_staged', 'arrival_reset', 'return_started',
                                          'return_finished', 'return_failed')}

    def _stage_hulls(self, carry):
        return [carry.footprint.hull_at(p) for p in carry.ref.lookahead(STAGE_LOOKAHEAD_S)]

    def station_in_team_path(self, robot):
        """Job id of a carrying team whose route over STAGE_LOOKAHEAD_S covers this robot's claimed station."""
        if robot.station is None:
            return None
        for jid, carry in self.carries.items():
            if (carry.n < 2 or carry.job.terminal or carry.job.state != 'CARRY' or carry.ref is None
                    or robot.rid in carry.parts or not self.held(carry.item_id)):
                continue
            if station_in_hulls(robot.station[:2], self._stage_hulls(carry)):
                return jid
        return None

    def carry_blockers(self, carry, now):
        if not self.switches['b3_claim_yield']:
            return super().carry_blockers(carry, now)
        hulls = [carry.footprint.hull_at(p) for p in carry.ref.lookahead(CARRY_LOOKAHEAD_S)]
        out = set()
        for rid, robot in self.robots.items():
            if rid in carry.parts:
                continue
            x, y, _ = robot.pose()
            disc = circle(x, y, ROBOT_DISC_M)
            if any(polygons_overlap(h, disc) for h in hulls):
                out.add(rid)
                pre = _pre_job(robot.claim)
                if (not robot.busy or pre) and not robot.yield_req:
                    avoid = [p[:2] for p in carry.ref.lookahead(CARRY_LOOKAHEAD_S*2)]
                    robot.request_yield(carry.proxy, avoid, now)
                    if pre:
                        self.fix_events['claim_yield'] += 1
                        self.log('claim_yield', rid, now, team=carry.job.job_id, item=robot.claim.item_id)
        for jid, other in self.carries.items():
            if other is carry or other.job.terminal or other.n == 1 or not self.held(other.item_id):
                continue
            theirs = other.footprint.hull_at(self.item_pose(other.item_id))
            if any(polygons_overlap(h, theirs) for h in hulls):
                out.add(jid)
        return out

    def _claims_tick(self, now):
        if self.switches['b3_claim_yield']:
            for rid, c in self.claims.items():
                robot = self.robots[rid]
                if _pre_job(c) and c.arrived_at is not None and (robot.yield_req or robot.staged):
                    self.fix_events['arrival_reset'] += 1
                    self.log('arrival_reset', rid, now, item=c.item_id, arrived_at=round(c.arrived_at, 2),
                             cause='yield' if robot.yield_req else 'staged')
                    c.arrived_at, c.state = None, 'to_station'
        super()._claims_tick(now)

    def _commit(self, claims, now):
        before = set(self.carries)
        super()._commit(claims, now)
        for jid in set(self.carries) - before:
            # same object, fixed methods (B4); the base commit already ran its route check
            self.carries[jid].__class__ = FixTeamCarry

    def record(self):
        return super().record() | {'teacher_fix': {
            'profile': PROFILE, 'switches': dict(self.switches), 'events': dict(self.fix_events),
            'source': 'scripts/zone_team_teacher_fix.py', 'source_sha256': source_sha256(),
            'scope': 'GT TEACHER only (demonstrations, training targets, scenario feasibility); '
                     'never the study executor; teacher success is never robot success'}}


def fix_run_class(switches=None):
    """A ZoneRunV2 whose executor is FixZoneTeamExecutor (used by scripts/run_zone_teacher_fix.py)."""
    from scripts.zone_dispatch_v2 import ZoneRunV2

    class FixZoneRunV2(ZoneRunV2):
        def __init__(self, config, output, *, contact_profile, record_replay=False, inject=None):
            super().__init__(config, output, contact_profile=contact_profile, record_replay=record_replay,
                             inject=inject)
            self.executor = FixZoneTeamExecutor(self.world, self.ports, config['static_map'], self.items,
                                                config['goal'], self._log, inject=inject, switches=switches)

    return FixZoneRunV2


__all__ = ['PROFILE', 'DEFAULT_SWITCHES', 'FixZoneTeamExecutor', 'FixTeamCarry', 'FixTeamRobot', 'fix_run_class',
           'return_poses', 'station_in_hulls', 'source_sha256']
