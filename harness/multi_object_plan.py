"""Variable-cargo planning and task leases for OFFLINE protocol validation.

There is deliberately no motor adapter here. Slots/routes are logical resources,
not geometric safety certificates. READY/DONE are peer visual claims, never
physical success labels. A physical adapter must supply image identity tracking,
the stage synchronizer and an admitted map before using this contract.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import re

from harness.three_robot_plan import ROBOTS, TeamAgreement, digest


def _fields(value, names):
    if not isinstance(value, dict) or set(value) != set(names.split()):
        raise ValueError('exact contract fields required')


def _id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', value):
        raise ValueError('invalid identifier')
    return value


def _ids(values):
    if not isinstance(values, list) or any(not isinstance(v, str) for v in values):
        raise ValueError('identifier list required')
    if len(values) != len(set(values)):
        raise ValueError('duplicate identifier')
    for value in values:
        _id(value)
    return values


def _index(rows, key):
    if not isinstance(rows, list) or not rows:
        raise ValueError('nonempty row list required')
    if any(not isinstance(row, dict) or key not in row for row in rows):
        raise ValueError('missing row identifier')
    ids = _ids([row[key] for row in rows])
    return dict(zip(ids, rows))


def _ancestors(tasks):
    result, visiting = {}, set()

    def visit(tid):
        if tid in visiting:
            raise ValueError('task dependency cycle')
        if tid in result:
            return result[tid]
        visiting.add(tid)
        deps = _ids(tasks[tid]['after'])
        if any(dep not in tasks for dep in deps):
            raise ValueError('unknown dependency')
        ancestors = set(deps)
        for dep in deps:
            ancestors.update(visit(dep))
        visiting.remove(tid)
        result[tid] = ancestors
        return ancestors

    for tid in tasks:
        visit(tid)
    return result


def validate_mission(value):
    """Allowlist authored requirements; no setup, identity poses or referee data."""
    _fields(value, 'schema objects destinations routes tasks')
    if value['schema'] != 'ugrp.multi_object_mission.v1':
        raise ValueError('unknown mission schema')
    objects = _index(value['objects'], 'object_id')
    destinations = _index(value['destinations'], 'destination_id')
    routes = _index(value['routes'], 'route_id')
    tasks = _index(value['tasks'], 'task_id')
    for obj in objects.values():
        _fields(obj, 'object_id kind')
        if obj['kind'] not in ('beam', 'box'):
            raise ValueError('only unchanged beam and box replicas are planned')
    for dest in destinations.values():
        _fields(dest, 'destination_id kind')
        if dest['kind'] not in ('final', 'staging'):
            raise ValueError('unknown destination kind')
    for route in routes.values():
        _fields(route, 'route_id resources')
        _ids(route['resources'])
    for task in tasks.values():
        _fields(task, 'task_id object_id destination_id after')
        _id(task['object_id'])
        _id(task['destination_id'])
        if task['object_id'] not in objects or task['destination_id'] not in destinations:
            raise ValueError('unknown object or destination')
    ancestors = _ancestors(tasks)
    finals = []
    for oid in objects:
        jobs = [tid for tid, task in tasks.items() if task['object_id'] == oid]
        final = [tid for tid in jobs if destinations[tasks[tid]['destination_id']]['kind'] == 'final']
        if len(final) != 1 or any(tid != final[0] and tid not in ancestors[final[0]] for tid in jobs):
            raise ValueError('each object needs exactly one final task after every earlier task')
        if any(a != b and a not in ancestors[b] and b not in ancestors[a] for a in jobs for b in jobs):
            raise ValueError('tasks on one object must be totally ordered')
        finals.append(tasks[final[0]]['destination_id'])
    if len(finals) != len(set(finals)):
        raise ValueError('final slots have capacity one; distinct object slots required')
    return copy.deepcopy(value)


def validate_plan(value, mission):
    mission = validate_mission(mission)
    _fields(value, 'schema mission_sha256 tasks')
    if value['schema'] != 'ugrp.multi_object_plan.v1' or value['mission_sha256'] != digest(mission):
        raise ValueError('wrong plan schema or mission hash')
    jobs = _index(mission['tasks'], 'task_id')
    objects = _index(mission['objects'], 'object_id')
    routes = _index(mission['routes'], 'route_id')
    tasks = _index(value['tasks'], 'task_id')
    if set(tasks) != set(jobs):
        raise ValueError('plan must include every task exactly once')
    for tid, task in tasks.items():
        _fields(task, 'task_id participants route_id after')
        people = task['participants']
        if not isinstance(people, dict) or not set(people) <= set(ROBOTS):
            raise ValueError('unknown participant')
        roles = ['end_a', 'end_b'] if objects[jobs[tid]['object_id']]['kind'] == 'beam' else ['solo']
        if any(not isinstance(role, str) for role in people.values()) or sorted(people.values()) != roles:
            raise ValueError('beam needs two different end roles; box needs one solo role')
        if _id(task['route_id']) not in routes:
            raise ValueError('unknown logical route')
        if not set(jobs[tid]['after']) <= set(_ids(task['after'])):
            raise ValueError('plan removed a required precedence edge')
    _ancestors(tasks)
    # Repeated robot assignments across jobs are intentional. The lease gate
    # arbitrates concurrent use instead of globally forbidding robot reuse.
    return copy.deepcopy(value)


@dataclass(frozen=True)
class Ticket:
    proposal_id: str
    plan_hash: str
    task_id: str
    object_id: str
    serial: int
    issued_at_s: float


class MissionProtocol:
    """Unanimous plan -> resource leases -> all-carrier completion claims.

    Logical end-to-end fixture only: does not issue motion, verify images or
    establish collision freedom. Invalidation blocks transitions and retains
    leases. No automatic reset/replan may discard occupied resources.
    """
    def __init__(self, mission, agreement, *, max_age_s=2.):
        self.mission = validate_mission(mission)
        if not isinstance(agreement, TeamAgreement) or agreement.committed is None:
            raise ValueError('unanimously committed TeamAgreement required')
        self.authority = agreement
        self.committed = copy.deepcopy(agreement.committed)
        self.plan = validate_plan(self.committed['plan'], self.mission)
        if digest(self.plan) != self.committed['plan_hash']:
            raise ValueError('committed hash mismatch')
        self.max_age_s = self._time(max_age_s)
        if self.max_age_s <= 0:
            raise ValueError('positive report age required')
        self.jobs = _index(self.mission['tasks'], 'task_id')
        self.tasks = _index(self.plan['tasks'], 'task_id')
        self.routes = _index(self.mission['routes'], 'route_id')
        self.ready, self.active, self.done_claims = {}, {}, {}
        self.completed, self.locks, self.residents = set(), {}, {}
        self.sequences, self.events = {}, []
        self.serial, self.clock = 0, 0.

    @staticmethod
    def _time(value):
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError('finite nonnegative monotonic time required')
        return float(value)

    def _check(self, now_s):
        now_s = self._time(now_s)
        if now_s < self.clock:
            raise ValueError('clock moved backwards')
        if not self.authority.authorize(self.committed['proposal_id'], self.committed['plan_hash']):
            raise ValueError('plan revoked; leases retained until external recovery')
        self.clock = now_s
        return now_s

    def _report(self, tid, rid, *, proposal_id, plan_hash, object_id, sequence,
                observed_at_s, now_s, own_rgb_ref, top_rgb_ref):
        now_s = self._check(now_s)
        if (proposal_id, plan_hash) != (self.committed['proposal_id'], self.committed['plan_hash']):
            raise ValueError('stale proposal or hash')
        if tid not in self.tasks or rid not in self.tasks[tid]['participants']:
            raise ValueError('wrong task or participant')
        if object_id != self.jobs[tid]['object_id']:
            raise ValueError('wrong object identity claim')
        if type(sequence) is not int or sequence <= self.sequences.get(rid, -1):
            raise ValueError('stale report sequence')
        observed_at_s = self._time(observed_at_s)
        if not 0 <= now_s - observed_at_s <= self.max_age_s:
            raise ValueError('stale or future observation')
        if any(not isinstance(ref, str) or not ref.strip() or len(ref) > 512 for ref in (own_rgb_ref, top_rgb_ref)):
            raise ValueError('own and TOP frame references required')
        return {'observed_at_s': observed_at_s, 'received_at_s': now_s, 'sequence': sequence,
                'own_rgb_ref': own_rgb_ref, 'top_rgb_ref': top_rgb_ref}

    def report_ready(self, tid, rid, **report):
        row = self._report(tid, rid, **report)
        if tid in self.active or tid in self.completed:
            raise ValueError('task is not pending')
        self.ready[tid, rid] = row
        self.sequences[rid] = row['sequence']
        self.events.append({'event': 'READY_CLAIM', 'task_id': tid, 'robot_id': rid, **row})

    def _resources(self, tid):
        job, task = self.jobs[tid], self.tasks[tid]
        return {f'robot:{r}' for r in task['participants']} | {
            'object:' + job['object_id'], 'slot:' + job['destination_id'],
        } | {'route:' + r for r in self.routes[task['route_id']]['resources']}

    def grant_ready(self, now_s):
        """Plan list order is the agreed tie-break priority, never an allocation fallback."""
        now_s = self._check(now_s)
        granted = []
        for tid, task in self.tasks.items():
            if tid in self.active or tid in self.completed or not set(task['after']) <= self.completed:
                continue
            reports = [self.ready.get((tid, rid)) for rid in task['participants']]
            if any(row is None or now_s - row['observed_at_s'] > self.max_age_s for row in reports):
                continue
            resources = self._resources(tid)
            job = self.jobs[tid]
            resident = self.residents.get(job['destination_id'])
            if resources & self.locks.keys() or resident not in (None, job['object_id']):
                continue
            self.serial += 1
            ticket = Ticket(self.committed['proposal_id'], self.committed['plan_hash'], tid,
                            job['object_id'], self.serial, now_s)
            self.active[tid] = ticket
            self.done_claims[tid] = {}
            self.locks.update({key: tid for key in resources})
            for rid in task['participants']:
                self.ready.pop((tid, rid))
            granted.append(ticket)
            self.events.append({'event': 'TASK_GRANTED', 'task_id': tid, 'serial': ticket.serial,
                                'now_s': now_s, 'resources': sorted(resources)})
        return granted

    def report_done(self, ticket, rid, **report):
        if not isinstance(ticket, Ticket) or self.active.get(ticket.task_id) != ticket:
            raise ValueError('old or unknown task ticket')
        tid = ticket.task_id
        row = self._report(tid, rid, **report)
        if row['observed_at_s'] < ticket.issued_at_s:
            raise ValueError('pre-grant completion claim')
        self.sequences[rid] = row['sequence']
        self.done_claims[tid][rid] = row
        self.events.append({'event': 'DONE_CLAIM', 'task_id': tid, 'robot_id': rid, **row})
        if set(self.done_claims[tid]) != set(self.tasks[tid]['participants']) or any(
                row['received_at_s'] - peer['observed_at_s'] > self.max_age_s
                for peer in self.done_claims[tid].values()):
            return False
        self.completed.add(tid)
        self.active.pop(tid)
        for resource in self._resources(tid):
            del self.locks[resource]
        oid, dest = self.jobs[tid]['object_id'], self.jobs[tid]['destination_id']
        # Occupied staging remains reserved until the object's next task ends.
        self.residents = {slot: obj for slot, obj in self.residents.items() if obj != oid}
        self.residents[dest] = oid
        self.events.append({'event': 'TASK_COMPLETE_CLAIM', 'task_id': tid, 'now_s': row['received_at_s']})
        return True

    def summary(self):
        finals = [t for t in self.mission['tasks'] if next(
            d for d in self.mission['destinations'] if d['destination_id'] == t['destination_id'])['kind'] == 'final']
        return {'objects': len(self.mission['objects']), 'tasks': len(self.tasks),
                'completed_task_claims': len(self.completed),
                'final_object_claims': sum(t['task_id'] in self.completed for t in finals),
                'whole_mission_claimed': len(self.completed) == len(self.tasks),
                'physical_success': 'not_evaluated', 'motor_execution': 'unsupported'}
