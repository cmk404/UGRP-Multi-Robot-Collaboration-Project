"""Team jobs for the zone benchmark: one item, N carriers, one transactional commit.

Phase A1 of the zone team-carry work (2026-09-25). Pure bookkeeping and rules;
the executor (A2) calls these from the one SIM clock.

A robot never joins a team by being picked. Every robot states its own
``RoleClaim`` (item, zone, grasp role). How claims are made differs by
coordination mode (independent / plan_first / dynamic; validators below). How
a team FORMS is one rule for every mode (``RendezvousRule``):

  A team for an item commits when every grasp station of the item's formation
  is physically occupied by a robot whose own claim names that station's role
  and the same (item, kind, zone, formation) spec.

Station occupancy is a physical fact (the robot's base at its station) that the
teacher reads from simulator truth, like every other teacher motion decision.
No robot's claim is ever changed, completed or merged by the host, and a
robot's own outcome depends on a peer's intention only once that peer is
physically holding a station of the same item (see ``RendezvousRule``).

TeamJob states (one per item):
COMMITTED -> RENDEZVOUS -> PREGRASP -> CLOSE -> LIFT -> CARRY -> LOWER ->
RELEASE -> RETREAT -> FINISHED | ABORTED

Partial failure: before contact (COMMITTED, RENDEZVOUS, PREGRASP) the job is
cancelled and every participant retreats. From CLOSE on, every participant
holds, then the team lowers and releases together, then retreats; the job ends
ABORTED. Participants never change: no automatic replacement, no N-1 carry.
Every state change waits for a barrier of all participants; a timeout is never
readiness.
"""
from __future__ import annotations

import copy
import itertools
import math
from dataclasses import dataclass

from harness.three_robot_plan import digest
from harness.zone_goal_v2 import ALL_KINDS, formation, formation_id, required_carriers
from harness.zone_team_footprint import station_offset

SCHEMA = 'ugrp.zone_team_job.v1'
STATES = ('COMMITTED', 'RENDEZVOUS', 'PREGRASP', 'CLOSE', 'LIFT', 'CARRY', 'LOWER', 'RELEASE', 'RETREAT',
          'FINISHED', 'ABORTED')
PRE_CONTACT = frozenset({'COMMITTED', 'RENDEZVOUS', 'PREGRASP'})
CONTACT = frozenset({'CLOSE', 'LIFT', 'CARRY'})
SETTING_DOWN = frozenset({'LOWER', 'RELEASE'})
TERMINAL = frozenset({'FINISHED', 'ABORTED'})
NEXT = dict(zip(STATES[:9], STATES[1:10]))
# Executor receipts (the only job result a robot's model reads, as today).
RECEIPT_FINISHED = 'issued sequence finished'
RECEIPT_STOPPED = 'executor stopped before finishing'


class CommitRejected(ValueError):
    """A team commit that would break a job invariant; nothing was changed."""


def spec_hash(item, kind, zone):
    """What every participant must agree on: item, kind, destination zone, formation."""
    return digest({'item': item, 'kind': kind, 'zone': zone, 'formation_id': formation_id(kind)})


@dataclass(frozen=True)
class RoleClaim:
    robot: str
    item: str
    kind: str
    zone: str
    role: str

    def __post_init__(self):
        if self.kind not in ALL_KINDS:
            raise ValueError(f'unknown item kind: {self.kind}')
        if self.role not in formation(self.kind):
            raise ValueError(f'{self.kind} has no grasp role {self.role!r}')

    @property
    def spec_hash(self):
        return spec_hash(self.item, self.kind, self.zone)

    def record(self):
        return {'robot': self.robot, 'item': self.item, 'kind': self.kind, 'zone': self.zone, 'role': self.role,
                'spec_hash': self.spec_hash}


# ---------------------------------------------------------------------------
# Barrier

class PhaseBarrier:
    """All participants must report ready for the same state and generation.

    Stale reports (other state or older generation) are ignored and counted.
    There is no timeout path to readiness.
    """
    def __init__(self, participants, state, generation):
        self.participants = frozenset(participants)
        self.state, self.generation = state, generation
        self.ready = {}
        self.stale = 0

    def report(self, rid, state, generation, now):
        if rid not in self.participants:
            raise ValueError(f'{rid} is not a participant')
        if state != self.state or generation != self.generation:
            self.stale += 1
            return False
        self.ready.setdefault(rid, float(now))
        return True

    @property
    def complete(self):
        return set(self.ready) == self.participants

    @property
    def missing(self):
        return sorted(self.participants - set(self.ready))


# ---------------------------------------------------------------------------
# TeamJob

class TeamJob:
    def __init__(self, *, job_id, generation, item_label, kind, zone, role_by_robot, now, on_terminal=None):
        self.job_id, self.generation = job_id, int(generation)
        self.item_label, self.kind, self.zone = item_label, kind, zone
        self.role_by_robot = dict(sorted(role_by_robot.items()))
        self.participants = tuple(self.role_by_robot)
        self.formation_id = formation_id(kind)
        self.spec_hash = spec_hash(item_label, kind, zone)
        self.required_carriers = required_carriers(kind)
        self.state = 'COMMITTED'
        self.aborting = None            # {'robot', 'reason', 'state', 't'} of the first failure
        self.failures = []
        self.contact_made = False
        self.history = [{'t': round(float(now), 3), 'state': 'COMMITTED'}]
        self.barrier = PhaseBarrier(self.participants, self.state, self.generation)
        self._on_terminal = on_terminal
        self._terminal_seen = False

    @property
    def terminal(self):
        return self.state in TERMINAL

    def report_ready(self, rid, now, *, state=None, generation=None):
        """A participant finished its part of the current state (its own evidence)."""
        if self.terminal:
            return False
        return self.barrier.report(rid, self.state if state is None else state,
                                   self.generation if generation is None else generation, now)

    def _enter(self, state, now, **detail):
        self.state = state
        if state in CONTACT:
            self.contact_made = True
        self.history.append({'t': round(float(now), 3), 'state': state, **detail})
        self.barrier = PhaseBarrier(self.participants, state, self.generation)
        if state in TERMINAL and not self._terminal_seen:
            self._terminal_seen = True
            if self._on_terminal:
                self._on_terminal(self)

    def advance(self, now):
        """Move to the next state once the barrier is complete; returns the new state or None."""
        if self.terminal or not self.barrier.complete:
            return None
        nxt = NEXT[self.state]
        if nxt == 'FINISHED' and self.aborting:
            nxt = 'ABORTED'
        self._enter(nxt, now)
        return nxt

    def fail(self, rid, reason, now):
        """One participant cannot go on. Returns the team action for the executor.

        before contact -> 'cancel_retreat' (everyone retreats, job ABORTED after)
        CLOSE/LIFT/CARRY -> 'hold_lower' (everyone holds, lowers, releases, retreats)
        LOWER/RELEASE    -> 'continue_release' (keep setting down together)
        RETREAT          -> 'end' (job ABORTED now; the item is already released)
        """
        if rid not in self.participants:
            raise ValueError(f'{rid} is not a participant of {self.job_id}')
        if self.terminal:
            return {'action': 'none', 'state': self.state}
        entry = {'robot': rid, 'reason': str(reason), 'state': self.state, 't': round(float(now), 3)}
        self.failures.append(entry)
        if self.aborting is None:
            self.aborting = entry
        if self.state in PRE_CONTACT:
            self._enter('RETREAT', now, cancelled=True, cause=entry)
            action = 'cancel_retreat'
        elif self.state in CONTACT:
            self._enter('LOWER', now, hold_first=True, cause=entry)
            action = 'hold_lower'
        elif self.state in SETTING_DOWN:
            action = 'continue_release'
        else:
            self._enter('ABORTED', now, cause=entry)
            action = 'end'
        return {'action': action, 'robots': list(self.participants), 'state': self.state}

    @property
    def receipt(self):
        """The receipt a participant's model may read when the job ends."""
        if not self.terminal:
            return None
        return RECEIPT_FINISHED if self.state == 'FINISHED' else RECEIPT_STOPPED

    def record(self):
        return {'schema': SCHEMA, 'job_id': self.job_id, 'generation': self.generation,
                'item_label': self.item_label, 'kind': self.kind, 'zone': self.zone,
                'participants': list(self.participants), 'role_by_robot': dict(self.role_by_robot),
                'formation_id': self.formation_id, 'spec_hash': self.spec_hash,
                'required_carriers': self.required_carriers, 'state': self.state,
                'contact_made': self.contact_made, 'aborting': copy.deepcopy(self.aborting),
                'failures': copy.deepcopy(self.failures), 'history': copy.deepcopy(self.history)}


# ---------------------------------------------------------------------------
# Ledger

class TeamJobLedger:
    """Every team job, the robot -> job index and the goal bookkeeping.

    ``jobs[job_id]`` is the only job record; ``robot_job[rid]`` is a reference.
    The goal is decremented once per item, when its job FINISHES.
    """
    def __init__(self, goal):
        self.goal = copy.deepcopy(goal)
        self.jobs, self.robot_job, self.item_job = {}, {}, {}
        self.generations = {}
        self.delivered = {}
        self.delivered_items = {}
        self.events = []

    def live(self, job_id):
        return job_id in self.jobs and not self.jobs[job_id].terminal

    def commit(self, claims, now):
        """Transactional: all claims agree on one spec and exactly fill its formation."""
        claims = list(claims)
        if not claims:
            raise CommitRejected('no claims')
        specs = {c.spec_hash for c in claims}
        if len(specs) != 1:
            raise CommitRejected('participants disagree on item, zone or formation')
        first = claims[0]
        robots = [c.robot for c in claims]
        if len(set(robots)) != len(robots):
            raise CommitRejected('a robot appears twice in one team')
        roles = [c.role for c in claims]
        if len(set(roles)) != len(roles):
            raise CommitRejected('duplicate role in one team')
        if sorted(roles) != sorted(formation(first.kind)):
            raise CommitRejected(f'roles {sorted(roles)} do not fill formation {formation_id(first.kind)}')
        if len(claims) != required_carriers(first.kind):
            raise CommitRejected('participant count differs from required_carriers')
        busy = [r for r in robots if r in self.robot_job and self.live(self.robot_job[r])]
        if busy:
            raise CommitRejected(f'robots already in a live job: {sorted(busy)}')
        if first.item in self.item_job and self.live(self.item_job[first.item]):
            raise CommitRejected(f'item {first.item} already has a live job')
        if first.item in self.delivered_items:
            raise CommitRejected(f'item {first.item} was already delivered')
        generation = self.generations.get(first.item, 0) + 1
        self.generations[first.item] = generation
        job = TeamJob(job_id=f'{first.item}@g{generation}', generation=generation, item_label=first.item,
                      kind=first.kind, zone=first.zone, role_by_robot={c.robot: c.role for c in claims},
                      now=now, on_terminal=self._terminal)
        self.jobs[job.job_id] = job
        self.item_job[first.item] = job.job_id
        for r in robots:
            self.robot_job[r] = job.job_id
        self.events.append({'event': 'commit', 't': round(float(now), 3), 'job_id': job.job_id,
                            'role_by_robot': dict(job.role_by_robot)})
        return job

    def _terminal(self, job):
        if job.state == 'FINISHED':
            if job.item_label in self.delivered_items:
                raise RuntimeError(f'item {job.item_label} finished twice')
            self.delivered_items[job.item_label] = job.job_id
            self.delivered.setdefault(job.zone, {}).setdefault(job.kind, 0)
            self.delivered[job.zone][job.kind] += 1
        for r in job.participants:
            if self.robot_job.get(r) == job.job_id:
                del self.robot_job[r]
        self.events.append({'event': job.state.lower(), 'job_id': job.job_id})

    def remaining(self):
        """Goal minus finished deliveries (never below zero)."""
        out = {}
        for zone, kinds in self.goal.items():
            for kind, count in kinds.items():
                left = count - self.delivered.get(zone, {}).get(kind, 0)
                if left > 0:
                    out.setdefault(zone, {})[kind] = left
        return out

    def over_delivered(self):
        out = {}
        for zone, kinds in self.delivered.items():
            for kind, n in kinds.items():
                extra = n - self.goal.get(zone, {}).get(kind, 0)
                if extra > 0:
                    out.setdefault(zone, {})[kind] = extra
        return out

    def record(self):
        return {'goal': copy.deepcopy(self.goal), 'delivered': copy.deepcopy(self.delivered),
                'remaining': self.remaining(), 'over_delivered': self.over_delivered(),
                'jobs': {k: j.record() for k, j in self.jobs.items()}, 'events': copy.deepcopy(self.events)}


# ---------------------------------------------------------------------------
# Team formation: one physical rule for every coordination mode

def _wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def station_pose(item_pose, kind, role):
    """World base pose of a grasp station for an item pose (teacher geometry)."""
    x, y, yaw = item_pose
    sx, sy, syaw = station_offset(kind, role)
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c*sx - s*sy, y + s*sx + c*sy, _wrap(yaw + syaw))


class RendezvousRule:
    """Communication-free, symmetric team formation (same in all modes).

    - Each robot with a claim drives to ITS claimed station and waits there.
    - ``occupancy``: a robot is ``at_station`` when its base is within the
      station tolerance and no other robot is physically nearer that station;
      ``station_blocked`` when another robot (any robot; its claim is not read)
      physically stands nearer while this robot is itself at the item (within
      ``near_item_m`` of the station: a visible event, not an intention);
      else ``en_route``. Equal distances block both.
    - ``teams``: robots at_station on the same item whose claims share one spec
      and exactly fill the formation form a team. Nothing else forms a team.
    - A robot waits at most ``wait_s`` SIM seconds after its own arrival; then
      its claim ends with the plain "stopped" receipt, whether or not a peer
      with another spec stood at another station of the item.
    Robot ids are never used to decide; outputs are only sorted for records.
    """
    def __init__(self, *, tol_m=.10, tol_rad=.35, wait_s=60., near_item_m=.50):
        self.tol_m, self.tol_rad, self.wait_s = float(tol_m), float(tol_rad), float(wait_s)
        self.near_item_m = float(near_item_m)

    def occupancy(self, claims, robot_poses, item_poses):
        """{rid: 'at_station' | 'station_blocked' | 'en_route'} for robots with a claim."""
        out = {}
        for rid, claim in claims.items():
            st = station_pose(item_poses[claim.item], claim.kind, claim.role)
            mine = math.dist(robot_poses[rid][:2], st[:2])
            nearer = [r for r, p in robot_poses.items() if r != rid and math.dist(p[:2], st[:2]) <= self.tol_m
                      and math.dist(p[:2], st[:2]) <= mine]
            if nearer and mine <= self.near_item_m:
                out[rid] = 'station_blocked'
            elif mine <= self.tol_m and abs(_wrap(robot_poses[rid][2] - st[2])) <= self.tol_rad:
                out[rid] = 'at_station'
            else:
                out[rid] = 'en_route'
        return out

    def teams(self, claims, occupancy):
        """[{rid: RoleClaim}] of complete teams among robots at their stations."""
        groups = {}
        for rid, claim in claims.items():
            if occupancy.get(rid) == 'at_station':
                groups.setdefault(claim.spec_hash, {})[rid] = claim
        out = []
        for _, members in sorted(groups.items()):
            roles = sorted(c.role for c in members.values())
            kind = next(iter(members.values())).kind
            if roles == sorted(formation(kind)):
                out.append(dict(sorted(members.items())))
        return out

    def expired(self, arrived_at_s, now):
        return arrived_at_s is not None and now - arrived_at_s >= self.wait_s

    def record(self):
        return {'rule': 'all formation stations physically occupied by robots whose own claims share one spec',
                'tol_m': self.tol_m, 'tol_rad': self.tol_rad, 'wait_s': self.wait_s,
                'near_item_m': self.near_item_m,
                'uses_robot_ids': False, 'reads_peer_claims_of_robots_not_at_station': False}


# ---------------------------------------------------------------------------
# Claims per coordination mode

def normalize_claim(rid, raw, labels):
    """Model reply claim -> RoleClaim or None (idle). Accepts the old {'box', 'zone'} form.

    Raises ValueError with a reason for an invalid claim.
    """
    if raw is None:
        raise ValueError('no valid reply')
    if not isinstance(raw, dict):
        raise ValueError('claim must be an object')
    if set(raw) == {'box', 'zone'}:
        raw = {'item': raw['box'], 'zone': raw['zone'], 'role': None}
    if set(raw) != {'item', 'zone', 'role'}:
        raise ValueError('claim requires item, zone and role')
    if raw['item'] is None:
        if raw['zone'] is not None or raw['role'] is not None:
            raise ValueError('an idle claim has item, zone and role all null')
        return None
    if raw['item'] not in labels:
        raise ValueError(f"unknown item label {raw['item']}")
    kind = labels[raw['item']]['kind']
    role = raw['role']
    if role is None:
        if required_carriers(kind) != 1:
            raise ValueError(f'{kind} needs {required_carriers(kind)} carriers: name your role')
        role = formation(kind)[0]
    if role not in formation(kind):
        raise ValueError(f'{kind} roles are {list(formation(kind))}')
    if raw['zone'] not in ('A', 'B', 'C'):
        raise ValueError('zone must be A, B or C')
    return RoleClaim(rid, raw['item'], kind, raw['zone'], role)


def _visible(view):
    return view.get('pickup_items_still_visible', view.get('pickup_boxes_still_visible', []))


def check_one_independent(rid, raw, *, goal, labels, view):
    """One robot's own claim against what that robot can know: goal and its own RGB view."""
    try:
        claim = normalize_claim(rid, raw, labels)
    except ValueError as exc:
        return {'status': 'invalid', 'reason': str(exc)}
    if claim is None:
        return {'status': 'idle'}
    if claim.item not in _visible(view):
        return {'status': 'invalid', 'reason': f'{claim.item} is not an item still visible in pickup'}
    need = goal.get(claim.zone, {}).get(claim.kind, 0) - view['zone_counts_seen'].get(claim.zone, {}).get(claim.kind, 0)
    if need <= 0:
        return {'status': 'invalid', 'reason': f'zone {claim.zone} needs no more {claim.kind}'}
    return {'status': 'accepted', 'claim': claim}


def check_independent_claims(claims, *, goal, labels, view):
    """No communication: each claim is checked alone; claims are never compared.

    Two robots may claim the same station or different zones for one item; the
    physical rendezvous rule then decides, identically in every mode.
    """
    out = {'accepted': {}, 'invalid': {}, 'idle': []}
    for rid in claims:
        r = check_one_independent(rid, claims[rid], goal=goal, labels=labels, view=view)
        if r['status'] == 'accepted':
            out['accepted'][rid] = r['claim']
        elif r['status'] == 'idle':
            out['idle'].append(rid)
        else:
            out['invalid'][rid] = r['reason']
    out['idle'].sort()
    return out


def remaining_need_items(goal, view, active, finished=None):
    """Goal minus RGB zone counts minus active claims, each ITEM counted once.

    active: {rid: RoleClaim}. finished: [{'zone', 'kind'}] of finished jobs
    (one per item). As ``zone_coordination.remaining_need``: items seen in a
    zone beyond the finished deliveries are attributed to active jobs first.
    """
    items = {}
    for c in active.values():
        items.setdefault(c.item, (c.zone, c.kind))
    need = {}
    for zone, kinds in goal.items():
        for kind, count in kinds.items():
            seen = view['zone_counts_seen'].get(zone, {}).get(kind, 0)
            claimed = sum(1 for z, k in items.values() if z == zone and k == kind)
            if finished is not None:
                delivered = sum(1 for j in finished if j['zone'] == zone and j['kind'] == kind)
                claimed -= min(claimed, max(0, seen - delivered))
            if count - seen - claimed > 0:
                need.setdefault(zone, {})[kind] = count - seen - claimed
    return need


def check_dynamic_claims(claims, *, goal, labels, view, active, finished=None):
    """Host checks against goal, RGB view and active peer claims; it never picks for anyone.

    - same item + same role by two robots -> collision 'same_role' (they talk)
    - same item with different zones      -> collision 'zone_mismatch' (they talk)
    - a free role on an item that already has active claims to the same zone is
      accepted (the robot chose to join; the item is not counted twice)
    - a new item needs remaining need (items counted once)
    """
    out = {'accepted': {}, 'invalid': {}, 'idle': [], 'collisions': []}
    parsed = {}
    for rid in sorted(claims):
        try:
            c = normalize_claim(rid, claims[rid], labels)
        except ValueError as exc:
            out['invalid'][rid] = str(exc)
            continue
        if c is None:
            out['idle'].append(rid)
        else:
            parsed[rid] = c
    pending = dict(active)
    by_station, by_item = {}, {}
    for rid, c in parsed.items():
        by_station.setdefault((c.item, c.role), []).append(rid)
        by_item.setdefault(c.item, []).append(rid)
    colliding = set()
    for (item, role), rids in sorted(by_station.items()):
        holders = sorted(r for r, c in pending.items() if c.item == item and c.role == role and r not in parsed)
        if len(rids) + len(holders) > 1:
            out['collisions'].append({'kind': 'same_role', 'item': item, 'role': role,
                                      'robots': sorted(rids + holders)})
            colliding.update(rids)
    for item, rids in sorted(by_item.items()):
        zones = {parsed[r].zone for r in rids} | {c.zone for r, c in pending.items() if c.item == item}
        if len(zones) > 1:
            others = sorted(r for r, c in pending.items() if c.item == item and r not in parsed)
            out['collisions'].append({'kind': 'zone_mismatch', 'item': item, 'zones': sorted(zones),
                                      'robots': sorted(rids + others)})
            colliding.update(rids)
    for rid, c in parsed.items():
        if rid in colliding:
            continue
        if c.item not in _visible(view):
            out['invalid'][rid] = f'{c.item} is not an item still visible in pickup'
            continue
        joining = any(p.item == c.item for r, p in pending.items() if r != rid)
        if not joining:
            need = remaining_need_items(goal, view, {r: p for r, p in pending.items() if r != rid},
                                        finished).get(c.zone, {}).get(c.kind, 0)
            if need <= 0:
                out['invalid'][rid] = f'zone {c.zone} needs no more {c.kind}'
                continue
        out['accepted'][rid] = c
        pending[rid] = c
    out['idle'].sort()
    return out


def validate_team_plan(plan, goal, labels, robots=('r1', 'r2', 'r3')):
    """plan_first v2: {'assignments': {rid: [{'item', 'zone', 'role'} | {'box', 'zone'}]}}.

    Every item's entries across robots share one zone and fill its formation
    exactly (no duplicate role, no robot twice); deliveries meet the goal
    exactly; and the per-robot orders admit one global order (no cyclic wait:
    robot lists that meet on two team items must meet them in the same order).
    Returns the normalized plan (every entry with item, zone and role).
    """
    if not isinstance(plan, dict) or set(plan) != {'assignments'}:
        raise ValueError('plan requires assignments only')
    assignments = plan['assignments']
    if not isinstance(assignments, dict) or set(assignments) != set(robots):
        raise ValueError('assignments need ' + ', '.join(robots) + ' (empty lists allowed)')
    items, norm = {}, {}
    for rid in robots:
        jobs = assignments[rid]
        if not isinstance(jobs, list):
            raise ValueError('each robot needs a job list')
        norm[rid] = []
        for job in jobs:
            if not isinstance(job, dict):
                raise ValueError('each job is an object')
            try:
                c = normalize_claim(rid, job, labels)
            except ValueError as exc:
                raise ValueError(f'{rid}: {exc}') from None
            if c is None:
                raise ValueError('a plan job needs an item')
            if any(x.item == c.item for x in norm[rid]):
                raise ValueError(f'{rid} lists item {c.item} twice')
            norm[rid].append(c)
            items.setdefault(c.item, []).append(c)
    delivered = {}
    for item, cs in items.items():
        if len({c.zone for c in cs}) != 1:
            raise ValueError(f'item {item} goes to different zones')
        roles = sorted(c.role for c in cs)
        if roles != sorted(formation(cs[0].kind)):
            raise ValueError(f'item {item} roles {roles} do not fill {formation_id(cs[0].kind)}')
        delivered.setdefault(cs[0].zone, {}).setdefault(cs[0].kind, 0)
        delivered[cs[0].zone][cs[0].kind] += 1
    if delivered != goal:
        raise ValueError(f'plan delivers {delivered}, goal is {goal}')
    _check_no_cyclic_wait(norm)
    return {'assignments': {rid: [{'item': c.item, 'zone': c.zone, 'role': c.role} for c in norm[rid]]
                            for rid in robots}}


def _check_no_cyclic_wait(norm):
    """Precedence graph over items from each robot's order; reject any cycle."""
    edges = {}
    for cs in norm.values():
        for a, b in zip(cs, cs[1:]):
            edges.setdefault(a.item, set()).add(b.item)
    state = {}

    def visit(node, stack):
        state[node] = 1
        for nxt in sorted(edges.get(node, ())):
            if state.get(nxt) == 1:
                raise ValueError('plan orders would make robots wait for each other: '
                                 + ' -> '.join(stack + [node, nxt]))
            if state.get(nxt) is None:
                visit(nxt, stack + [node])
        state[node] = 2
    for node in sorted(edges):
        if state.get(node) is None:
            visit(node, [])


def relabel(mapping, value):
    """Apply a robot relabelling to nested dict keys / RoleClaim robots (tests, audits)."""
    if isinstance(value, RoleClaim):
        return RoleClaim(mapping.get(value.robot, value.robot), value.item, value.kind, value.zone, value.role)
    if isinstance(value, dict):
        return {mapping.get(k, k): relabel(mapping, v) for k, v in value.items()}
    if isinstance(value, list):
        return [relabel(mapping, v) for v in value]
    if isinstance(value, str):
        return mapping.get(value, value)
    return value


def robot_permutations(robots=('r1', 'r2', 'r3')):
    return [dict(zip(robots, p)) for p in itertools.permutations(robots)]


__all__ = ['SCHEMA', 'STATES', 'PRE_CONTACT', 'CONTACT', 'TERMINAL', 'RECEIPT_FINISHED', 'RECEIPT_STOPPED',
           'CommitRejected', 'RoleClaim', 'PhaseBarrier', 'TeamJob', 'TeamJobLedger', 'RendezvousRule',
           'station_pose', 'spec_hash', 'normalize_claim', 'check_one_independent', 'check_independent_claims',
           'check_dynamic_claims', 'remaining_need_items', 'validate_team_plan', 'relabel', 'robot_permutations']
