"""Ground-truth TEACHER executor for zone team jobs (zone team A2, 2026-09-25).

Never a student result. Every v2 zone job is a ``harness.zone_team_jobs.TeamJob``:
colour boxes, can and tile are one-robot teams; long_beam and heavy_crate need
two robots, tri_frame three. One rule forms every team in every coordination
mode (``RendezvousRule``): a robot with a claim drives to ITS claimed station
and waits there; the team commits only when every station of the item is
physically occupied by robots whose claims share one spec (item, kind, zone,
formation). A robot waits at most 60 SIM s after its own arrival.

A robot's claim is stopped only by what it could see at the item (audit L1):
another body stands physically nearer its station (station blocking), or the
item is already being grasped or was delivered. A peer's claim alone never
stops it; the host never picks teammates.

After commit: ``FormationPlan`` fine approach, one synchronized arm plan for
all participants, closing with a finger-contact check, lift with a clear-hold
check, carry, lower, release, retreat; each state behind a ``PhaseBarrier`` of
all participants. Failures follow ``TeamJob.fail``: ``cancel_retreat`` before
contact, ``hold_lower`` after it (set down together, release, retreat); the job
ends ABORTED and every participant gets the "stopped" receipt.

Carry: teams follow a route planned once before contact in item-pose space
(``harness.zone_team_route``: straight moves, in-place 90 deg turns, the
static_keepouts swept footprint test); a route that does not pass is rejected
before contact. One-robot teams keep the disc planner of the solo teacher.
Not carried over from the solo teacher: the 12 s forced grasp after a failed
alignment (an alignment timeout is a failure before contact) and opening the
gripper when a carry is blocked (now hold_lower).

Other teams are obstacles with their full footprint (item + every carrier
chassis and arm). Teacher truth (poses, finger contact forces, item heights)
drives every decision. Robots only ever receive the two receipts. No weld and
no equality constraint anywhere.
"""
from __future__ import annotations

import math

import numpy as np

from harness.static_keepouts import inside_rect, polygons_overlap
from harness.zone_goal_v2 import claim_roles, formation, landing_layout, required_carriers
from harness.zone_team_formation import FormationPlan
from harness.zone_team_footprint import TeamFootprint, circle, item_polygons, station_offset, transform
from harness.zone_team_jobs import (CONTACT, SETTING_DOWN, TERMINAL, CommitRejected, RendezvousRule, RoleClaim,
                                    TeamJobLedger, station_pose)
from harness.zone_team_route import PoseReference, plan_team_route
from scripts.cargo_formation_teacher import FINGER_MIN_N, FINISH_ERR_M, FINISH_ERR_RAD, FINISH_SETTLE_S
from scripts.zone_teacher import (BOX_CLEARANCE_M, CLOSED, CONTROL_S, DRIVE_PHASE_LIMIT_S, FOLDED, OPEN,
                                  ROUTE_DRIVE_PHASE_LIMIT_S, PeerDisc, TeacherRobot, ZoneTeacherExecutor)

SCHEMA = 'ugrp.zone_team_executor.v1'
# Pre-station: the disc planner drives to the station backed off along its heading,
# then a straight fine approach reaches the station (team items keep the robot
# disc clear of the item until then).
PRESTATION_BACKOFF_M = {1: .10, 2: .20, 3: .20}
STATION_DRIVE_TOL_M = .05
# Per-state limits (SIM s; smoke_zone_team_formation values). A limit is a failure, never readiness.
STATE_LIMIT_S = {'COMMITTED': 5., 'RENDEZVOUS': 40., 'PREGRASP': 10., 'CLOSE': 10., 'LIFT': 15.,
                 'LOWER': 10., 'RELEASE': 10., 'RETREAT': 15.}
CLOSE_CONTACT_WAIT_S = 3.
LIFT_CLEAR_Z_M, LIFT_HOLD_S, LIFT_WAIT_S = .012, 1.5, 6.
GRIP_LOST_S = .3
BACKOFF_S = {1: 4., 2: 2.5, 3: 2.5}
# Team carry: look this far ahead along the reference for robots/teams in the way.
CARRY_LOOKAHEAD_S = 6.
CARRY_PAUSE_LIMIT_S = 180.
ROBOT_DISC_M = .17
# Solo carry: drive to the landing station backed off by this much, then align.
LANDING_BACKOFF_M = .08
SOLO_ALIGN_LIMIT_S = 20.
# Single-lane entry gate (v2 only; A2 smoke 2026-09-25: two team members that
# finish a carry together leave for the next shared item together and wedged
# side by side inside a 0.5 m door, where neither could back out). A driving
# robot inside a passage zone whose path enters the opening holds while another
# robot stands in the opening, or while a moving robot is nearer the opening.
# A held robot is still, so two robots never hold for each other. Physical
# inputs only: peer positions and stillness, the static passage and the robot's
# own path; never a peer's claim, job, goal or id. The hold is capped.
GATE_PATH_M = 1.0
GATE_LIMIT_S = 60.
GATE_MOVING_S = 1.


def _wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def landing_pose_for_role(kind, area, role):
    """Landing item pose for the claimed role. A one-robot item with alternative
    sides (tile: west or east, 180 deg symmetric) is turned so the claimed side's
    station lands where the landing layout put the default role's station."""
    pose = tuple(area['item_pose'])
    default = formation(kind)[0]
    if required_carriers(kind) != 1 or role == default:
        return pose
    dyaw = station_offset(kind, default)[2] - station_offset(kind, role)[2]
    return (pose[0], pose[1], _wrap(pose[2] + dyaw))


def poly_rect(poly):
    """Oriented rectangle (cx, cy, hx, hy, yaw) of a 4-vertex rectangle polygon; a
    square around any other polygon (planner keep-out)."""
    cx = sum(x for x, _ in poly)/len(poly)
    cy = sum(y for _, y in poly)/len(poly)
    if len(poly) == 4:
        (x0, y0), (x1, y1), (x2, y2) = poly[0], poly[1], poly[2]
        yaw = math.atan2(y1-y0, x1-x0)
        return (cx, cy, math.hypot(x1-x0, y1-y0)/2, math.hypot(x2-x1, y2-y1)/2, yaw)
    r = max(math.hypot(x-cx, y-cy) for x, y in poly)
    return (cx, cy, r, r, 0.)


class Claim:
    """One robot's live claim (executor side; the robot's model sees only receipts)."""
    def __init__(self, rid, role_claim, item_id, kind, role_phys, now, label_role):
        self.rid, self.role_claim = rid, role_claim
        self.item_id, self.kind, self.role_phys = item_id, kind, role_phys
        self.label_role = label_role
        self.issued_at, self.arrived_at = float(now), None
        self.state, self.outcome, self.job_id, self.receipt = 'to_station', None, None, None
        self.ended_at = None

    @property
    def phys(self):
        return RoleClaim(self.rid, self.item_id, self.kind, self.role_claim.zone, self.role_phys)

    @property
    def live(self):
        return self.state != 'ended'

    def record(self):
        return {'robot': self.rid, 'claim': self.role_claim.record(), 'item_id': self.item_id,
                'role_phys': self.role_phys, 'issued_at_sim_s': round(self.issued_at, 2),
                'arrived_at_sim_s': None if self.arrived_at is None else round(self.arrived_at, 2),
                'state': self.state, 'outcome': self.outcome, 'job_id': self.job_id, 'receipt': self.receipt,
                'ended_at_sim_s': None if self.ended_at is None else round(self.ended_at, 2)}


class TeamRobot(TeacherRobot):
    """A zone robot for v2 claims: station drive, station hold, solo carry legs."""
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.dyn_rects = ()
        self.claim = None
        self.station = None          # (x, y, yaw) of the claimed station (updated each tick)
        self.backoff = .10
        self.flag = None             # (reason, detail) a solo carry leg could not finish
        self.landed = False
        self.landing = None          # {'pose', 'plan'} for a solo carry
        self.gate = None             # (passage_id, since) while held at a single-lane entry
        self.gate_passed = set()     # passages whose gate cap ran out on this leg
        self.peer_moved = []         # [(xy, since)] peer positions for the stillness test

    def planning_rects(self):
        return tuple(self.rects) + tuple(self.dyn_rects)

    def _limit(self):
        return ROUTE_DRIVE_PHASE_LIMIT_S if self.rects else DRIVE_PHASE_LIMIT_S

    def _set(self, phase, now, **detail):
        super()._set(phase, now, **detail)
        self.gate, self.gate_passed = None, set()

    def _path_enters(self, core):
        """Does the own planned path cross the opening within GATE_PATH_M?"""
        prev, run = self.pose()[:2], 0.
        for q in (self.path or ())[:40]:
            seg = math.dist(prev, q[:2])
            n = max(1, int(seg/.05))
            if any(inside_rect((prev[0]+(q[0]-prev[0])*i/n, prev[1]+(q[1]-prev[1])*i/n), core) for i in range(n+1)):
                return True
            run += seg
            prev = q[:2]
            if run >= GATE_PATH_M:
                break
        return False

    def _moving_peers(self, peers, now):
        """Peers seen displaced by more than 2 cm within the last GATE_MOVING_S."""
        seen, moving = [], {}
        for p in peers:
            old = next((e for e in self.peer_moved if math.dist(e[0], p) < .02), None)
            entry = old or (p, now)
            seen.append(entry)
            moving[p] = now - entry[1] < GATE_MOVING_S
        self.peer_moved = seen
        return moving

    def passage_gate(self, now, discs):
        """True while this robot holds before a single-lane opening (see GATE_*)."""
        x, y, _ = self.pose()
        peers = [(d[0], d[1]) for d in discs if isinstance(d, PeerDisc)]
        moving = self._moving_peers(peers, now)
        for pid, core, zone in self.passages:
            if pid in self.gate_passed or not inside_rect((x, y), zone) or inside_rect((x, y), core, grow=.10):
                continue
            if not self._path_enters(core):
                continue
            mine = math.dist((x, y), core[:2])
            reason = None
            for p in peers:
                if inside_rect(p, core, grow=.10):
                    reason = 'opening_occupied'
                elif inside_rect(p, zone) and moving[p] and math.dist(p, core[:2]) < mine - .02:
                    reason = 'nearer_robot_entering'
                if reason:
                    break
            if reason is None:
                continue
            if self.gate is None or self.gate[0] != pid:
                self.gate = (pid, now)
                self.log('passage_gate', self.rid, now, passage=pid, reason=reason, own_phase=self.phase)
            if now - self.gate[1] >= GATE_LIMIT_S:
                self.gate_passed.add(pid)
                self.log('passage_gate_end', self.rid, now, passage=pid, reason='limit',
                         wait_s=round(now-self.gate[1], 2))
                self.gate = None
                return False
            self.port.hold(now)
            return True
        if self.gate is not None:
            self.log('passage_gate_end', self.rid, now, passage=self.gate[0], reason='clear',
                     wait_s=round(now-self.gate[1], 2))
            self.gate = None
        return False

    def station_hold(self, station, now):
        """Straight fine approach to a station pose (base frame P control)."""
        x, y, a = self.pose()
        sx, sy, sa = station
        c, s = math.cos(a), math.sin(a)
        ex, ey = c*(sx-x) + s*(sy-y), -s*(sx-x) + c*(sy-y)
        self.port.apply({'kind': 'mecanum', 'duration_s': .15,
                         'forward': float(np.clip(.9*ex, -.05, .08)), 'left': float(np.clip(.9*ey, -.06, .06)),
                         'turn': float(np.clip(.8*_wrap(sa-a), -.10, .10))}, now)

    def tick(self, now, discs_for):
        self.arm.tick(now)
        if self.passage_yield and self._passage_yield(now, discs_for):
            return
        if self.yield_req and self._yield(now, discs_for):
            return
        if not self.busy or self.claim is None:
            return
        if self.phase == 'to_box':
            if self.goal_occupied or now - self.phase_started > self._limit():
                self.port.hold(now)
                self.flag = ('teacher_path_blocked', {'goal_occupied': bool(self.goal_occupied)})
                return
            sx, sy, sa = self.station
            pre = (sx - self.backoff*math.cos(sa), sy - self.backoff*math.sin(sa))
            discs = discs_for(self, carrying=False)
            if self.passages and self.passage_gate(now, discs):
                return
            if self._drive_to(pre, sa, now, discs, carrying=False,
                              tol=STATION_DRIVE_TOL_M):
                self._set('align_box', now)
        elif self.phase == 'align_box':
            self.station_hold(self.station, now)
        elif self.phase == 'carry':
            if self.goal_occupied or now - self.phase_started > self._limit():
                self.port.hold(now)
                self.flag = ('teacher_path_blocked', {'goal_occupied': bool(self.goal_occupied), 'carrying': True})
                return
            sx, sy, sa = self.landing['station']
            goal = (sx - LANDING_BACKOFF_M*math.cos(sa), sy - LANDING_BACKOFF_M*math.sin(sa))
            discs = discs_for(self, exclude=self.job['box_body'], carrying=True)
            if self.passages and self.passage_gate(now, discs):
                return
            if self._drive_to(goal, sa, now, discs, carrying=True, tol=.04):
                self._set('align_slot', now)
        elif self.phase == 'align_slot':
            if now - self.phase_started > SOLO_ALIGN_LIMIT_S:
                self.port.hold(now)
                self.flag = ('landing_align_timeout', {})
                return
            cmd, _ = self.landing['plan'].approach_command(self.rid, self.pose(), self.landing['pose'])
            if cmd is None:
                self.port.hold(now)
                self.landed = True
            else:
                self.port.apply({'kind': 'mecanum', 'duration_s': .12, **cmd}, now)


class TeamProxy:
    """Stands for a carrying team in the solo teacher's yield protocol (TeacherRobot._yield)."""
    def __init__(self, carry):
        self.carry = carry
        self.rid = 'team:' + carry.job.job_id
        self.job = carry.job

    @property
    def phase(self):
        return 'carry' if self.carry.job.state == 'CARRY' else self.carry.job.state.lower()

    def radius(self, carrying):
        return self.carry.footprint.radius()


class TeamCarry:
    """Drives one committed TeamJob (1..3 carriers) through its states."""
    def __init__(self, ex, job, claims, area, now):
        self.ex, self.job = ex, job
        self.claims = {c.rid: c for c in claims}
        self.parts = tuple(job.participants)
        self.item_id, self.kind = claims[0].item_id, job.kind
        self.body = ex.items[self.item_id]['body_name']
        self.area = area
        self.plan = FormationPlan(self.kind, job.role_by_robot)
        self.footprint = TeamFootprint(self.kind, tuple(sorted(job.role_by_robot.values())))
        self.n = len(self.parts)
        self.seen, self.state_t = None, float(now)
        self.arm_plans, self.arms_idle_t = {}, None
        self.lift_ok_at, self.lost_since = None, None
        self.ref, self.ref_done_at, self.carry_t = None, None, None
        self.route = None
        self.pause = None
        self.pause_total = 0.
        self.backoff_t = None
        self.inject = None
        self.slip_ref, self.slip_max = {}, {r: 0. for r in self.parts}
        self.proxy = TeamProxy(self)
        self.record = {'job_id': job.job_id, 'item_id': self.item_id, 'kind': self.kind, 'zone': job.zone,
                       'participants': list(self.parts), 'role_by_robot': dict(job.role_by_robot),
                       'claimed_roles': {c.rid: c.label_role for c in claims},
                       'landing_area': area and {k: area[k] for k in ('item_pose', 'landing_center_m',
                                                                       'landing_half_extents_m')},
                       'arrived_sim_s': {c.rid: round(c.arrived_at, 2) for c in claims},
                       'commit_sim_s': round(now, 2),
                       'formation_wait_s': {c.rid: round(now - c.arrived_at, 2) for c in claims},
                       'barriers': [], 'pauses': [], 'failures': [], 'route': None, 'grip_lost': [],
                       'drops': 0}

    # --- helpers ---
    def robots(self):
        return {r: self.ex.robots[r] for r in self.parts}

    def arm_idle(self, rid, now):
        arm = self.ex.robots[rid].arm
        return now >= arm.until and not arm.events

    def grip(self, rid):
        return OPEN if self.inject and self.inject['robot'] == rid else CLOSED

    def report(self, rid, now):
        self.job.report_ready(rid, now)

    def advance(self, now):
        state = self.job.state
        ready = dict(self.job.barrier.ready)
        nxt = self.job.advance(now)
        if nxt:
            self.record['barriers'].append({'state': state, 'ready_t': {r: round(t, 2) for r, t in ready.items()},
                                            'spread_s': round(max(ready.values()) - min(ready.values()), 3),
                                            'advanced_t': round(now, 2), 'next': nxt})
        return nxt

    def hold_all(self, now):
        for r in self.parts:
            self.ex.robots[r].port.hold(now)

    def fail(self, rid, reason, now, **detail):
        r = self.job.fail(rid, reason, now)
        entry = {'robot': rid, 'reason': reason, 'state_before': self.seen, 'action': r['action'],
                 't': round(now, 2), **detail}
        self.record['failures'].append(entry)
        self.ex.log('team_failure', rid, now, job=self.job.job_id, reason=reason, action=r['action'])
        self.hold_all(now)
        for p in self.parts:
            robot = self.ex.robots[p]
            if robot.phase in ('carry', 'align_slot'):
                robot._set('team', now)
        self.ex.robot_event(rid, 'team_failure', now, job=self.job.job_id, reason=reason)
        return r

    # --- state entry actions ---
    def _enter(self, state, now):
        self.seen, self.state_t, self.arms_idle_t = state, now, None
        self.ex.log('team_state', None, now, job=self.job.job_id, state=state)
        robots = self.robots()
        if state == 'PREGRASP':
            item = self.ex.station_item_pose(self.item_id)
            for rid, robot in robots.items():
                self.arm_plans[rid] = self.plan.arm_plan(rid, robot.pose(), item)
                robot.arm.queue({**self.arm_plans[rid]['hover'], 1: OPEN}, now, duration=1.0)
        elif state == 'CLOSE':
            for rid, robot in robots.items():
                for pose in self.arm_plans[rid]['descent']:
                    robot.arm.queue(pose, now, duration=.12, settle=0.)
                robot.arm.queue({1: self.grip(rid)}, now, duration=.5, settle=.4)
        elif state == 'LIFT':
            for rid, robot in robots.items():
                robot.arm.queue({**self.arm_plans[rid]['lift'], 1: self.grip(rid)}, now, duration=1.2, settle=.3)
            self.lift_ok_at = None
        elif state == 'CARRY':
            self.carry_t = now
            if self.n == 1:
                rid = self.parts[0]
                robot = robots[rid]
                role = self.job.role_by_robot[rid]
                pose = landing_pose_for_role(self.kind, self.area, role)
                robot.landing = {'station': tuple(station_pose(pose, self.kind, role)), 'pose': pose,
                                 'plan': self.plan}
                robot.landed, robot.flag = False, None
                robot._set('carry', now)
            else:
                item = self.ex.item_pose(self.item_id)
                poses = [item] + [tuple(p) for p in self.route['poses'][1:]]
                self.ref = PoseReference(poses)
                self.ref_done_at = None
        elif state == 'LOWER':
            self.hold_all(now)
            for rid, robot in robots.items():
                if robot.phase != 'team':
                    robot._set('team', now)
                if rid in self.arm_plans:
                    robot.arm.queue({**self.arm_plans[rid]['descent'][-1], 1: self.grip(rid)}, now,
                                    duration=1.2, settle=.4)
        elif state == 'RELEASE':
            for robot in robots.values():
                robot.arm.queue({1: OPEN}, now, duration=.4, settle=.5)
        elif state == 'RETREAT':
            self.hold_all(now)
            for rid, robot in robots.items():
                if robot.phase != 'team':
                    robot._set('team', now)
                hover = self.arm_plans.get(rid, {}).get('hover')
                if hover is not None:
                    robot.arm.queue({**hover, 1: OPEN}, now, duration=.6)
                robot.arm.queue({**FOLDED, 1: OPEN}, now)
                robot.arm.queue(FOLDED, now)
            self.backoff_t = None
        elif state in TERMINAL:
            self.hold_all(now)
            self.ex.finish_job(self, now)

    # --- per-state ticks ---
    def tick(self, now):
        job = self.job
        if job.state != self.seen:
            self._enter(job.state, now)
        if job.terminal:
            return
        state = job.state
        limit = STATE_LIMIT_S.get(state)
        if limit and now - self.state_t > limit + (self.pause_total if state == 'CARRY' else 0.):
            late = job.barrier.missing or list(self.parts)
            self.fail(late[0], f'{state.lower()}_timeout', now)
        else:
            getattr(self, '_' + state.lower())(now)
        if job.state != self.seen:
            self._enter(job.state, now)

    def _committed(self, now):
        for r in self.parts:
            self.report(r, now)
        self.advance(now)

    def _rendezvous(self, now):
        item = self.ex.station_item_pose(self.item_id)
        for rid, robot in self.robots().items():
            cmd, _ = self.plan.approach_command(rid, robot.pose(), item)
            if cmd is None:
                robot.port.hold(now)
                self.report(rid, now)
            elif rid not in self.job.barrier.ready:
                robot.port.apply({'kind': 'mecanum', 'duration_s': .12, **cmd}, now)
        self.advance(now)

    def _pregrasp(self, now):
        for rid in self.parts:
            if self.arm_idle(rid, now):
                self.report(rid, now)
        self.advance(now)

    def _close(self, now):
        forces = self.ex.finger_forces(self.item_id, self.parts)
        idle = all(self.arm_idle(r, now) for r in self.parts)
        for rid in self.parts:
            if self.arm_idle(rid, now) and min(forces[rid]) >= FINGER_MIN_N:
                self.report(rid, now)
        if self.advance(now):
            self.record['grasp_forces_n'] = {r: [round(v, 2) for v in f] for r, f in forces.items()}
            return
        if idle:
            self.arms_idle_t = self.arms_idle_t or now
            if now - self.arms_idle_t > CLOSE_CONTACT_WAIT_S:
                missing = [r for r in self.parts if min(forces[r]) < FINGER_MIN_N]
                if missing:
                    self.fail(missing[0], 'grasp_contact_missing', now,
                              forces_n={r: [round(v, 2) for v in forces[r]] for r in self.parts})
                    self.ex.robot_event(missing[0], 'grasp_outcome_unexpected', now, job=self.job.job_id)

    def _lift(self, now):
        if not all(self.arm_idle(r, now) for r in self.parts):
            return
        self.arms_idle_t = self.arms_idle_t or now
        min_z = self.ex.item_min_z(self.item_id)
        forces = self.ex.finger_forces(self.item_id, self.parts)
        bilateral = all(min(forces[r]) >= FINGER_MIN_N for r in self.parts)
        clear = min_z > LIFT_CLEAR_Z_M and bilateral
        self.lift_ok_at = (self.lift_ok_at or now) if clear else None
        if self.lift_ok_at is not None and now - self.lift_ok_at >= LIFT_HOLD_S:
            self.record['hold_min_z_m'] = round(min_z, 4)
            for r in self.parts:
                self.report(r, now)
            self.advance(now)
        elif now - self.arms_idle_t > LIFT_WAIT_S:
            self.record['hold_min_z_m'] = round(min_z, 4)
            missing = [r for r in self.parts if min(forces[r]) < FINGER_MIN_N] or list(self.parts)
            self.fail(missing[0], 'not_lifted_clear', now, min_z_m=round(min_z, 4))
            self.ex.robot_event(missing[0], 'grasp_outcome_unexpected', now, job=self.job.job_id)

    def _grip_check(self, now):
        forces = self.ex.finger_forces(self.item_id, self.parts)
        lost = [r for r in self.parts if min(forces[r]) < FINGER_MIN_N]
        self.sample_slip()
        if lost:
            self.lost_since = self.lost_since or now
            if now - self.lost_since > GRIP_LOST_S:
                z = self.ex.item_min_z(self.item_id)
                self.record['grip_lost'].append({'robot': lost[0], 't': round(now, 2), 'min_z_m': round(z, 4)})
                if z < .005:
                    self.record['drops'] += 1
                self.fail(lost[0], 'grip_lost_in_transit', now, min_z_m=round(z, 4))
                return False
        else:
            self.lost_since = None
        return True

    def sample_slip(self):
        mids = self.ex.finger_midpoints_in_item(self.item_id, self.parts)
        for rid, p in mids.items():
            ref = self.slip_ref.setdefault(rid, p.copy())
            self.slip_max[rid] = max(self.slip_max[rid], float(np.linalg.norm(p - ref))*1000)

    def _carry(self, now):
        if not self._grip_check(now):
            return
        if self.n == 1:
            rid = self.parts[0]
            robot = self.ex.robots[rid]
            if robot.flag:
                reason, detail = robot.flag
                robot.flag = None
                self.fail(rid, reason, now, **detail)
                return
            if robot.landed:
                self.report(rid, now)
                self.advance(now)
            return
        robots = self.robots()
        poses = {r: robot.pose() for r, robot in robots.items()}
        blockers = self.ex.carry_blockers(self, now)
        if blockers:
            if self.pause is None:
                self.pause = {'t0': round(now, 2), 'by': sorted(blockers),
                              'near_passage': self.ex.near_passage(self.ex.item_pose(self.item_id))}
                self.ex.log('team_pause', None, now, job=self.job.job_id, by=sorted(blockers))
        elif self.pause is not None:
            self.pause['t1'] = round(now, 2)
            self.pause['s'] = round(now - self.pause['t0'], 2)
            self.pause_total += self.pause['s']
            self.record['pauses'].append(self.pause)
            self.pause = None
        paused = self.pause is not None
        if paused and now - self.pause['t0'] > CARRY_PAUSE_LIMIT_S:
            self.fail(self.parts[0], 'carry_blocked', now, by=self.pause['by'])
            return
        velocity = (0., 0., 0.) if paused else self.ref.velocity()
        step = self.plan.carry_step(self.ref.pose, velocity, poses)
        self.record['max_track_err_m'] = max(self.record.get('max_track_err_m', 0.), step['track_err_m'])
        if step['advance'] and not paused:
            self.ref.advance(CONTROL_S)
        if self.ref.finished and self.ref_done_at is None:
            self.ref_done_at = now
        if self.ref.finished and ((step['track_err_m'] < FINISH_ERR_M and step['track_err_rad'] < FINISH_ERR_RAD)
                                  or now - self.ref_done_at >= FINISH_SETTLE_S):
            self.record['finish_track_err_m'] = round(step['track_err_m'], 4)
            self.hold_all(now)
            for r in self.parts:
                self.report(r, now)
            self.advance(now)
            return
        if now - self.carry_t > 3*self.ref.total_s + 30 + self.pause_total + (now - self.pause['t0'] if paused else 0.):
            self.fail(self.parts[0], 'carry_timeout', now)
            return
        for rid, cmd in step['commands'].items():
            robots[rid].port.apply({'kind': 'mecanum', 'duration_s': .15, **cmd}, now)

    def _lower(self, now):
        for r in self.parts:
            if self.arm_idle(r, now):
                self.report(r, now)
        self.advance(now)

    def _release(self, now):
        for r in self.parts:
            if self.arm_idle(r, now):
                self.report(r, now)
        self.advance(now)

    def _retreat(self, now):
        if not all(self.arm_idle(r, now) for r in self.parts):
            return
        if self.backoff_t is None:
            self.backoff_t = now
        if now - self.backoff_t < BACKOFF_S[self.n]:
            for r in self.parts:
                self.ex.robots[r].port.apply({'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0.,
                                              'duration_s': .15}, now)
            return
        self.hold_all(now)
        for r in self.parts:
            self.report(r, now)
        self.advance(now)


class ZoneTeamExecutor(ZoneTeacherExecutor):
    """All v2 claims and team jobs on the one physics clock (CONTROL_S ticks)."""
    def __init__(self, world, ports, static_map, items, goal, log, *, robot_event=None, inject=None):
        # items: {item_id: {'kind', 'body_name', 'carriers'}} (boxes + cargo; teacher truth)
        self.world, self.items, self.log = world, items, log
        self.objects = items          # legacy helpers (lifted_box_near) read body_name
        self.static = static_map
        self.robots = {rid: TeamRobot(rid, world, port, static_map, log) for rid, port in ports.items()}
        for robot in self.robots.values():
            robot.team = self.robots
        self.next_tick = 0.
        self.rule = RendezvousRule()
        self.ledger = TeamJobLedger(goal)
        self.goal = goal
        self.areas = {}
        for zone, areas in landing_layout(goal, static_map=static_map).items():
            for a in areas:
                self.areas.setdefault((zone, a['kind']), []).append(a)
        self.claims = {}
        self.ended = []
        self.carries = {}
        self.history = []
        self.robot_event_hook = robot_event
        self.robot_events = []
        self.inject = dict(inject) if inject else None
        self.injected = None
        self.route_log = []
        self.claim_log = []
        self._geoms = {}
        self._finger_ids = None
        self.metrics = {k: 0 for k in ('placed_by_teacher', 'team_aborted', 'rendezvous_timeout', 'station_blocked',
                                       'item_taken', 'path_blocked', 'no_landing_area', 'commit_rejected',
                                       'route_rejected')}

    # --- teacher truth ---
    def item_pose(self, iid):
        d = self.world.data.body(self.items[iid]['body_name'])
        q = d.xquat
        yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        return (float(d.xpos[0]), float(d.xpos[1]), float(yaw))

    def station_item_pose(self, iid):
        """Item pose for station geometry: boxes and cans by the east-facing convention,
        tiles folded to their 180 deg symmetry, team items as they lie."""
        x, y, yaw = self.item_pose(iid)
        kind = self.items[iid]['kind']
        if required_carriers(kind) > 1:
            return (x, y, yaw)
        if kind == 'tile':
            return (x, y, _wrap(yaw) if abs(_wrap(yaw)) <= math.pi/2 else _wrap(yaw + math.pi))
        return (x, y, 0.)

    def geoms(self, iid):
        if iid not in self._geoms:
            m = self.world.model
            bid = self.world.data.body(self.items[iid]['body_name']).id
            self._geoms[iid] = [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == bid
                                and (int(m.geom_contype[g]) or int(m.geom_conaffinity[g]))]
        return self._geoms[iid]

    def item_min_z(self, iid):
        m, d = self.world.model, self.world.data
        import mujoco
        lo = math.inf
        for g in self.geoms(iid):
            pos, mat, size = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            lo = min(lo, float((pos + mat @ (np.array([sx, sy, sz])*size))[2]))
            else:
                r, h = size[0], size[1]
                for k in range(8):
                    a = k*math.pi/4
                    for sz in (-1, 1):
                        lo = min(lo, float((pos + mat @ np.array([r*math.cos(a), r*math.sin(a), sz*h]))[2]))
        return lo

    def item_tilt_deg(self, iid):
        mat = self.world.data.body(self.items[iid]['body_name']).xmat.reshape(3, 3)
        return math.degrees(math.acos(max(-1., min(1., float(mat[2, 2])))))

    def _fingers(self):
        if self._finger_ids is None:
            import mujoco
            m = self.world.model
            self._finger_ids = {}
            for rid in self.robots:
                for k, side in enumerate(('left', 'right')):
                    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f'{rid}__{side}_finger')
                    if gid >= 0:
                        self._finger_ids[gid] = (rid, k)
        return self._finger_ids

    def finger_forces(self, iid, robots):
        """{rid: (left N, right N)} finger normal force on this item (truth)."""
        import mujoco
        m, d = self.world.model, self.world.data
        item = set(self.geoms(iid))
        ids = self._fingers()
        out = {r: [0., 0.] for r in robots}
        f = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in ids and b in item and ids[a][0] in out:
                    mujoco.mj_contactForce(m, d, i, f)
                    rid, k = ids[a]
                    out[rid][k] += abs(float(f[0]))
        return {r: tuple(v) for r, v in out.items()}

    def item_held(self, iid):
        forces = self.finger_forces(iid, tuple(self.robots))
        return any(sum(v) > 0. for v in forces.values())

    def finger_midpoints_in_item(self, iid, robots):
        d = self.world.data
        body = d.body(self.items[iid]['body_name'])
        mat = body.xmat.reshape(3, 3)
        return {r: mat.T @ ((d.geom(f'{r}__left_finger').xpos + d.geom(f'{r}__right_finger').xpos)/2 - body.xpos)
                for r in robots}

    # --- items and obstacles ---
    def live_job(self, iid):
        jid = self.ledger.item_job.get(iid)
        return self.ledger.jobs[jid] if jid and self.ledger.live(jid) else None

    def held(self, iid):
        job = self.live_job(iid)
        return job is not None and (job.state in CONTACT or job.state in SETTING_DOWN)

    def taken(self, iid):
        """Visible at the item: another job is grasping/holding it, or it was delivered."""
        job = self.live_job(iid)
        return iid in self.ledger.delivered_items or (job is not None and job.contact_made)

    def discs_for(self, robot, *, exclude=None, carrying=False, peers=True):
        discs = []
        for iid, it in self.items.items():
            if it['carriers'] > 1 or it['body_name'] == exclude or self.held(iid):
                continue
            p = self.world.data.body(it['body_name']).xpos
            discs.append((float(p[0]), float(p[1]), BOX_CLEARANCE_M))
        for other in self.robots.values():
            if other is robot or not peers:
                continue
            x, y, _ = other.pose()
            discs.append(PeerDisc((x, y, .14)))
        return discs

    def _item_rects(self, iid):
        return [poly_rect(p) for p in (transform(q, self.item_pose(iid)) for q in item_polygons(self.items[iid]['kind']))]

    def update_rects(self):
        """Per robot: team cargo on the floor, and other teams' full footprints."""
        floor = {}
        for iid, it in self.items.items():
            if it['carriers'] > 1 and not self.held(iid):
                floor[iid] = self._item_rects(iid)
        teams = {}
        for carry in self.carries.values():
            if carry.n > 1 and not carry.job.terminal and self.held(carry.item_id):
                teams[carry.job.job_id] = [poly_rect(p) for p in carry.footprint.at(self.item_pose(carry.item_id))]
        for rid, robot in self.robots.items():
            rects = [r for rs in floor.values() for r in rs]
            for jid, rs in teams.items():
                if rid not in self.carries[jid].parts:
                    rects += rs
            robot.dyn_rects = tuple(rects)

    def item_obstacles(self, exclude):
        """World polygons of every other item on the floor (route planning, before contact)."""
        out = []
        for iid, it in self.items.items():
            if iid == exclude or self.held(iid):
                continue
            pose = self.item_pose(iid)
            if it['carriers'] == 1:
                x, y, _ = pose
                out.append(circle(x, y, .035))
            else:
                out += [transform(q, pose) for q in item_polygons(it['kind'])]
        return out

    def near_passage(self, pose):
        from harness.static_keepouts import inside_rect
        return next((p[0] for p in self.robots[next(iter(self.robots))].passages
                     if inside_rect(pose[:2], p[2], grow=.6)), None)

    def carry_blockers(self, carry, now):
        """Robots (not in the team) and other held teams inside the team's hull over the lookahead."""
        hulls = [carry.footprint.hull_at(p) for p in carry.ref.lookahead(CARRY_LOOKAHEAD_S)]
        out = set()
        for rid, robot in self.robots.items():
            if rid in carry.parts:
                continue
            x, y, _ = robot.pose()
            disc = circle(x, y, ROBOT_DISC_M)
            if any(polygons_overlap(h, disc) for h in hulls):
                out.add(rid)
                if not robot.busy and not robot.yield_req:
                    avoid = [p[:2] for p in carry.ref.lookahead(CARRY_LOOKAHEAD_S*2)]
                    robot.request_yield(carry.proxy, avoid, now)
        for jid, other in self.carries.items():
            if other is carry or other.job.terminal or other.n == 1 or not self.held(other.item_id):
                continue
            theirs = other.footprint.hull_at(self.item_pose(other.item_id))
            if any(polygons_overlap(h, theirs) for h in hulls):
                out.add(jid)
        return out

    # --- claims ---
    def bind(self, rid, role_claim, labels):
        """Label -> nearest physical item of the same kind; role -> the physical role whose approach
        (base station pose from the RGB handle, else its grip point) is nearest the claimed RGB role."""
        label = labels[role_claim.item]
        cands = [iid for iid, it in self.items.items()
                 if it['kind'] == role_claim.kind and iid not in self.ledger.delivered_items]
        if not cands:
            raise ValueError(f'no {role_claim.kind} item')
        iid = min(cands, key=lambda i: math.dist(self.item_pose(i)[:2], label['floor_xy_m']))
        kind = role_claim.kind
        roles = claim_roles(kind)
        if len(roles) == 1:
            return iid, roles[0]
        handle = (label.get('handles') or {}).get(role_claim.role, {})
        base, rgb = handle.get('approach_base_xyyaw'), handle.get('grip_xy_m')
        pose = self.station_item_pose(iid) if required_carriers(kind) == 1 else self.item_pose(iid)
        if base is not None:
            def cost(r):
                st = station_pose(pose, kind, r)
                return math.dist(st[:2], base[:2]) + .1*abs(_wrap(st[2] - base[2]))
            return iid, min(roles, key=cost)
        if rgb is None:
            return iid, role_claim.role
        from sim.zone_cargo import CATALOGUE
        grips = {g.role: transform([g.grip_xyz[:2]], pose)[0] for g in CATALOGUE[kind].grasps if g.role in roles}
        return iid, min(roles, key=lambda r: math.dist(grips[r], rgb))

    def claim(self, rid, role_claim, labels, now):
        if rid in self.claims and self.claims[rid].live:
            raise RuntimeError(f'{rid} already has a live claim')
        robot = self.robots[rid]
        if robot.busy:
            raise RuntimeError(f'{rid} is busy')
        iid, role_phys = self.bind(rid, role_claim, labels)
        c = Claim(rid, role_claim, iid, role_claim.kind, role_phys, now, role_claim.role)
        self.claims[rid] = c
        n = required_carriers(role_claim.kind)
        robot.claim = c
        robot.job = {'job_id': f'{rid}-claim-{len(self.claim_log)+1}', 'box_body': self.items[iid]['body_name']}
        robot.backoff = PRESTATION_BACKOFF_M[n]
        robot.station = station_pose(self.station_item_pose(iid), c.kind, role_phys)
        robot.flag, robot.landed, robot.outcome = None, False, None
        robot.goal_occupied = False
        robot.phase, robot.phase_started, robot.path = 'to_box', now, None
        robot.assigned_at = now
        self.claim_log.append(c)
        self.log('claim', rid, now, item=iid, label=role_claim.item, zone=role_claim.zone,
                 claimed_role=role_claim.role, role_phys=role_phys)
        return c

    def end_claim(self, c, outcome, receipt, now):
        c.state, c.outcome, c.receipt, c.ended_at = 'ended', outcome, receipt, now
        robot = self.robots[c.rid]
        robot.port.hold(now)
        robot.claim = None
        robot.outcome = outcome
        robot._set('done' if receipt == 'issued sequence finished' else 'failed', now, outcome=outcome)
        self.ended.append(c)
        if outcome in self.metrics:
            self.metrics[outcome] += 1
        if outcome in ('station_blocked', 'rendezvous_timeout'):
            self.robot_event(c.rid, outcome, now, item=c.item_id)

    def pop_ended(self):
        out, self.ended = self.ended, []
        return out

    def robot_event(self, rid, kind, now, **detail):
        """Hook: a robot-local event where a later design may ask that robot's model
        mid-motion. A2 records it only; nothing is asked."""
        entry = {'robot': rid, 'event': kind, 'sim_time_s': round(now, 2), **detail}
        self.robot_events.append(entry)
        if self.robot_event_hook:
            self.robot_event_hook(entry)

    def _claims_tick(self, now):
        waiting = {rid: c for rid, c in self.claims.items() if c.live and c.state in ('to_station', 'waiting')}
        if not waiting:
            return
        for rid, c in list(waiting.items()):
            robot = self.robots[rid]
            robot.station = station_pose(self.station_item_pose(c.item_id), c.kind, c.role_phys)
            if self.taken(c.item_id):
                self.end_claim(c, 'item_taken', 'executor stopped before finishing', now)
                del waiting[rid]
            elif robot.flag:
                reason, _ = robot.flag
                robot.flag = None
                self.end_claim(c, 'path_blocked' if reason == 'teacher_path_blocked' else reason,
                               'executor stopped before finishing', now)
                del waiting[rid]
        if not waiting:
            return
        poses = {rid: robot.pose() for rid, robot in self.robots.items()}
        items = {c.item_id: self.station_item_pose(c.item_id) for c in waiting.values()}
        occ = self.rule.occupancy({rid: c.phys for rid, c in waiting.items()}, poses, items)
        for rid, c in list(waiting.items()):
            if occ[rid] == 'station_blocked':
                self.end_claim(c, 'station_blocked', 'executor stopped before finishing', now)
                del waiting[rid]
                continue
            if occ[rid] == 'at_station' and c.arrived_at is None:
                c.arrived_at, c.state = now, 'waiting'
                self.log('at_station', rid, now, item=c.item_id, role=c.role_phys)
            if c.arrived_at is not None and self.rule.expired(c.arrived_at, now):
                self.end_claim(c, 'rendezvous_timeout', 'executor stopped before finishing', now)
                del waiting[rid]
        teams = self.rule.teams({rid: c.phys for rid, c in waiting.items()}, occ)
        for members in teams:
            self._commit([waiting[r] for r in members], now)

    def _commit(self, claims, now):
        first = claims[0]
        zone, kind = first.role_claim.zone, first.kind
        pool = self.areas.get((zone, kind), [])
        if not pool:
            for c in claims:
                self.end_claim(c, 'no_landing_area', 'executor stopped before finishing', now)
            self.log('no_landing_area', None, now, item=first.item_id, zone=zone, kind=kind)
            return
        try:
            job = self.ledger.commit([c.phys for c in claims], now)
        except CommitRejected as exc:
            for c in claims:
                self.end_claim(c, 'commit_rejected', 'executor stopped before finishing', now)
            self.log('commit_rejected', None, now, item=first.item_id, reason=str(exc))
            return
        area = pool.pop(0)
        for c in claims:
            c.state, c.job_id = 'job', job.job_id
            self.robots[c.rid]._set('team', now)
        carry = TeamCarry(self, job, claims, area, now)
        self.carries[job.job_id] = carry
        self.log('team_commit', None, now, job=job.job_id, role_by_robot=job.role_by_robot,
                 waits=carry.record['formation_wait_s'])
        if self.inject and self.injected is None and carry.n >= self.inject.get('min_carriers', 2) and (
                self.inject.get('kind') in (None, kind)):
            victim = sorted(job.role_by_robot, key=lambda r: job.role_by_robot[r])[0]
            carry.inject = {'failure': 'grasp_stays_open', 'robot': victim}
            self.injected = {'failure': 'grasp_stays_open', 'job_id': job.job_id, 'robot': victim,
                             'role': job.role_by_robot[victim], 'sim_time_s': round(now, 2)}
            self.log('injected', victim, now, failure='grasp_stays_open', job=job.job_id)
        if carry.n > 1:
            import time as _time
            started = _time.monotonic()
            goal_pose = tuple(area['item_pose'])
            sym = {'long_beam': math.pi, 'heavy_crate': math.pi, 'tri_frame': 2*math.pi/3}[kind]
            yaws = [goal_pose[2] + k*sym for k in range(int(round(2*math.pi/sym)))]
            route = plan_team_route(carry.footprint, self.item_pose(first.item_id), goal_pose,
                                    rects=self.robots[first.rid].rects, obstacles=self.item_obstacles(first.item_id),
                                    bounds=self.static['bounds_m'], goal_yaws=yaws)
            route['wall_s'] = round(_time.monotonic() - started, 2)
            carry.route = route
            carry.record['route'] = {k: route[k] for k in ('ok', 'reason', 'expansions', 'wall_s')} | {
                'poses': [[round(v, 4) for v in p] for p in route['poses']], 'checks': route['checks'],
                'legs': max(0, len(route['poses'])-1)}
            if route['ok']:
                carry.record['route']['reference_s'] = round(PoseReference(route['poses']).total_s, 1)
            self.route_log.append({'job_id': job.job_id, **carry.record['route']})
            if not route['ok']:
                self.metrics['route_rejected'] += 1
                carry.fail(first.rid, 'no_team_route', now, detail=route['reason'])

    def finish_job(self, carry, now):
        job = carry.job
        if job.state == 'ABORTED' and carry.area is not None:
            self.areas.setdefault((job.zone, job.kind), []).insert(0, carry.area)
        carry.record.update({'state': job.state, 'end_sim_s': round(now, 2),
                             'slip_mm': {r: round(v, 1) for r, v in carry.slip_max.items()},
                             'job': job.record(), 'inject': carry.inject})
        self.history.append(carry.record)
        for rid in carry.parts:
            c = carry.claims[rid]
            self.end_claim(c, 'placed_by_teacher' if job.state == 'FINISHED' else 'team_aborted', job.receipt, now)

    # --- clock ---
    def tick(self, now):
        if now + 1e-9 < self.next_tick:
            for robot in self.robots.values():
                robot.arm.tick(now)
            return
        self.next_tick = now + CONTROL_S
        self.update_rects()
        routes = any(r.passages for r in self.robots.values())
        if routes:
            self.track_motion(now)
        for robot in self.robots.values():
            robot.tick(now, self.discs_for)
        self._claims_tick(now)
        for carry in list(self.carries.values()):
            if not carry.job.terminal:
                carry.tick(now)
        if routes:
            self.resolve_passages(now)
        self.resolve_blocks(now)

    def record(self):
        return {'schema': SCHEMA, 'rendezvous_rule': self.rule.record(), 'ledger': self.ledger.record(),
                'jobs': self.history + [c.record for c in self.carries.values() if not c.job.terminal],
                'claims': [c.record() for c in self.claim_log], 'routes': self.route_log,
                'robot_events': self.robot_events, 'injection': self.injected, 'metrics': dict(self.metrics),
                'free_landing_areas': {f'{z}/{k}': len(v) for (z, k), v in self.areas.items()}}


__all__ = ['ZoneTeamExecutor', 'TeamCarry', 'TeamRobot', 'Claim', 'poly_rect', 'SCHEMA']
