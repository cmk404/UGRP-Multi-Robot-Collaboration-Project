"""Conservative central traffic coordination over RGB-derived reports only.

The first integration is for unloaded circular swept bounds. Reserving the
entire route is deliberately more conservative than time-slot scheduling. ETA
and GO expiry never clear a reservation. Loaded teams are rejected here until
their full articulated footprint and physical adapter have been validated.
"""
from __future__ import annotations

import copy
import math


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    denominator = dx * dx + dy * dy
    t = 0.0 if denominator == 0 else max(0.0, min(1.0,
        ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denominator))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def segments(path):
    return list(zip(path, path[1:])) or [(path[0], path[0])]


def path_distance(point, path):
    return min(point_segment_distance(point, a, b) for a, b in segments(path))


def segment_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])
    if (cross(a, b, c) * cross(a, b, d) <= 0 and
            cross(c, d, a) * cross(c, d, b) <= 0 and
            max(min(a[0], b[0]), min(c[0], d[0])) <= min(max(a[0], b[0]), max(c[0], d[0])) and
            max(min(a[1], b[1]), min(c[1], d[1])) <= min(max(a[1], b[1]), max(c[1], d[1]))):
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def paths_conflict(first, second, separation):
    """Continuous segment capsules, including head-on swaps between samples."""
    return any(segment_distance(a, b, c, d) <= separation
               for a, b in segments(first) for c, d in segments(second))


def _number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _point(value):
    return isinstance(value, (list, tuple)) and len(value) == 2 and all(_number(x) for x in value)


class TrafficCoordinator:
    """Atomic all-peer snapshots, FIFO grants, durable possible occupancy.

    One experiment-owned monotonic command clock; no simulator pose, contact,
    joint or evaluation API. Caller must persist snapshot() before dispatching GO.
    """
    def __init__(self, participants, *, radius_m=.18, uncertainty_m=.035,
                 stopping_margin_m=.08, max_wait_s=120.):
        self.participants = tuple(sorted(participants))
        if len(self.participants) < 2 or len(set(self.participants)) != len(self.participants):
            raise ValueError('distinct participants required')
        self.radius_m = radius_m
        self.uncertainty_m = uncertainty_m
        self.stopping_margin_m = stopping_margin_m
        self.max_wait_s = max_wait_s
        if not all(_number(x) and x > 0 for x in
                   (radius_m, uncertainty_m, stopping_margin_m, max_wait_s)):
            raise ValueError('positive finite bounds required')
        self.epoch = 1
        self.generation = 0
        self.last_now = -1.
        self.reports = {}
        self.reservations = {}
        self.waiting = {}
        self.cleared = {}
        self.events = []
        self.last_reason = {}

    @property
    def envelope_radius(self):
        return self.radius_m + self.uncertainty_m + self.stopping_margin_m

    def _event(self, kind, unit, now, **fields):
        self.events.append({'event': kind, 'unit_id': unit, 'clock_s': now,
                            'coordinator_epoch': self.epoch, **fields})

    @staticmethod
    def identity(report):
        return tuple(report[k] for k in ('task_id', 'plan_version', 'route_version'))

    def _valid(self, report, now):
        fields = {'unit_id', 'participants', 'task_id', 'plan_version', 'route_version',
                  'accepted_by', 'sequence', 'clock_s', 'clock_id', 'source',
                  'top_sha256', 'position_m', 'heading_rad', 'uncertainty_m',
                  'footprint', 'route', 'arrived', 'visual_ok'}
        if not isinstance(report, dict) or set(report) != fields:
            return False
        unit = report['unit_id']
        if unit not in self.participants or report['participants'] != [unit] or report['accepted_by'] != [unit]:
            return False
        if report['footprint'] != {'kind': 'unloaded_circle', 'radius_m': self.radius_m}:
            return False
        if report['source'] != 'top_rgb' or report['clock_id'] != 'issued_command_clock_v1':
            return False
        if not isinstance(report['task_id'], str) or not report['task_id']:
            return False
        if not all(isinstance(report[k], int) and not isinstance(report[k], bool) and report[k] >= 0
                   for k in ('plan_version', 'route_version', 'sequence')):
            return False
        if not _number(report['clock_s']) or abs(now - report['clock_s']) > 1e-6:
            return False
        if (not _point(report['position_m']) or not _number(report['uncertainty_m']) or
                not 0 <= report['uncertainty_m'] <= self.uncertainty_m):
            return False
        if report['heading_rad'] is not None and not _number(report['heading_rad']):
            return False
        if not isinstance(report['top_sha256'], str) or len(report['top_sha256']) != 64:
            return False
        if not isinstance(report['route'], list) or not report['route'] or not all(_point(p) for p in report['route']):
            return False
        if report['visual_ok'] is not True or not isinstance(report['arrived'], bool):
            return False
        previous = self.reports.get(unit)
        if previous and (report['sequence'] <= previous['sequence'] or
                         report['plan_version'] < previous['plan_version'] or
                         report['route_version'] < previous['route_version']):
            return False
        return True

    def step(self, reports, now, *, lease_s=.6):
        if not _number(now) or now <= self.last_now or not _number(lease_s) or not 0 < lease_s <= .6:
            raise ValueError('increasing clock and bounded GO lease required')
        self.last_now = now
        valid = (isinstance(reports, dict) and set(reports) == set(self.participants) and
                 all(isinstance(reports[u], dict) and reports[u].get('unit_id') == u and self._valid(reports[u], now)
                     for u in self.participants))
        if not valid:
            # Do not partly update a snapshot: both actors lose GO together.
            return self._permissions(now, lease_s, {}, 'fresh_all_peer_rgb_required')
        self.reports = copy.deepcopy(reports)
        separation = 2 * self.envelope_radius

        # Replan cannot erase a corridor a previous command might still occupy.
        for unit, reservation in self.reservations.items():
            report = reports[unit]
            identity = list(self.identity(report))
            if identity[:2] != reservation['identity'][:2]:
                reservation['replan_blocked'] = True
            elif identity[2] > reservation['identity'][2]:
                # Atomic route extension retains every old possibly occupied
                # capsule, and invalidates the old generation before dispatch.
                blocked = any(paths_conflict(report['route'], path, separation)
                    for peer, r in self.reservations.items() if peer != unit for path in r['routes'])
                blocked |= any(path_distance(r['position_m'], report['route']) <= separation
                               for peer, r in reports.items() if peer != unit)
                reservation['revision_blocked'] = blocked
                if not blocked:
                    self.generation += 1
                    reservation['routes'].append(copy.deepcopy(report['route']))
                    reservation['identity'] = identity
                    reservation['generation'] = self.generation
                    self._event('EXTEND', unit, now, generation=self.generation,
                                identity=identity, retained_route_count=len(reservation['routes']))
        for unit, report in reports.items():
            key = list(self.identity(report))
            if not report['arrived'] and self.cleared.get(unit) != key:
                if unit not in self.waiting:
                    self.waiting[unit] = now
                    self._event('REQUEST', unit, now, identity=key, route=report['route'])

        for unit, reservation in list(self.reservations.items()):
            report = reports[unit]
            if reservation.get('replan_blocked') or reservation.get('revision_blocked'):
                continue
            if not reservation['entered'] and math.dist(report['position_m'], reservation['start_m']) > .01:
                reservation['entered'] = True
                self._event('ENTER', unit, now, generation=reservation['generation'],
                            evidence='RGB displacement; not command acknowledgement')
            others = [r for u, r in reports.items() if u != unit and not r['arrived']]
            clear = report['arrived'] and all(path_distance(report['position_m'], r['route']) > separation for r in others)
            stationary = math.dist(report['position_m'], reservation['last_m']) <= .004
            reservation['clear_frames'] = reservation['clear_frames'] + 1 if clear and stationary else 0
            reservation['last_m'] = report['position_m']
            if reservation['clear_frames'] >= 2:
                self._event('CLEAR', unit, now, generation=reservation['generation'],
                            sequence=report['sequence'], top_sha256=report['top_sha256'])
                self._event('RELEASE', unit, now, generation=reservation['generation'])
                self.cleared[unit] = list(self.identity(report))
                self.waiting.pop(unit, None)
                del self.reservations[unit]

        reasons = {}
        for unit in sorted(self.waiting, key=lambda u: (self.waiting[u], u)):
            report = reports[unit]
            if unit in self.reservations or report['arrived']:
                continue
            blockers = [u for u, r in self.reservations.items()
                        if any(paths_conflict(report['route'], path, separation) for path in r['routes'])]
            if blockers:
                reasons[unit] = 'reserved_route:' + ','.join(sorted(blockers))
                continue
            # An unassigned, waiting or completed body does not disappear.
            parked = [u for u, r in reports.items() if u != unit and
                      path_distance(r['position_m'], report['route']) <= separation]
            if parked:
                reasons[unit] = 'peer_occupies_route:' + ','.join(sorted(parked))
                continue
            self.generation += 1
            self.reservations[unit] = {'identity': list(self.identity(report)),
                'generation': self.generation, 'route': copy.deepcopy(report['route']),
                'routes': [copy.deepcopy(report['route'])],
                'start_m': report['position_m'], 'last_m': report['position_m'],
                'clear_frames': 0, 'entered': False, 'replan_blocked': False}
            self._event('GRANT', unit, now, generation=self.generation,
                        identity=list(self.identity(report)),
                        occupancy='possibly occupied until RGB CLEAR, without TTL')

        go = {}
        for unit, reservation in self.reservations.items():
            report = reports[unit]
            if reservation.get('replan_blocked') or reservation.get('revision_blocked'):
                reasons[unit] = 'occupied_plan_changed_replan_required'
                continue
            # The route can be refined inside its original conservative tube;
            # a different corridor must obtain a new plan/version handshake.
            deviation = max(min(path_distance(p, path) for path in reservation['routes'])
                            for p in [report['position_m'], *report['route']])
            if deviation > self.stopping_margin_m:
                reasons[unit] = 'route_left_reserved_tube_replan_required'
                continue
            too_close = any(math.dist(report['position_m'], r['position_m']) <= separation
                            for u, r in reports.items() if u != unit)
            if too_close:
                reasons[unit] = 'peer_stop_margin'
                continue
            go[unit] = reservation
        for unit, start in self.waiting.items():
            if unit not in go and now - start >= self.max_wait_s:
                reasons[unit] = 'wait_budget_replan_required'
        return self._permissions(now, lease_s, go, reasons)

    def _permissions(self, now, lease_s, go, reasons):
        result = {}
        for unit in self.participants:
            reservation = go.get(unit)
            reason = 'reserved_route_ready' if reservation else (
                reasons if isinstance(reasons, str) else reasons.get(unit, 'completed_or_waiting'))
            report = self.reports.get(unit)
            result[unit] = {'phase': 'GO' if reservation else 'HOLD', 'reason': reason,
                'unit_id': unit, 'coordinator_epoch': self.epoch,
                'generation': reservation['generation'] if reservation else None,
                'identity': list(self.identity(report)) if report else None,
                'sequence': report['sequence'] if report else None,
                'issued_at_s': now, 'expires_at_s': now + lease_s}
            if self.last_reason.get(unit) != reason:
                self._event('GO' if reservation else 'HOLD', unit, now, reason=reason,
                            generation=result[unit]['generation'])
                self.last_reason[unit] = reason
        return result

    def snapshot(self):
        return copy.deepcopy({'schema': 'ugrp.traffic_coordinator.v1',
            'participants': list(self.participants), 'radius_m': self.radius_m,
            'uncertainty_m': self.uncertainty_m, 'stopping_margin_m': self.stopping_margin_m,
            'max_wait_s': self.max_wait_s, 'epoch': self.epoch, 'generation': self.generation,
            'last_now': self.last_now, 'reports': self.reports,
            'reservations': self.reservations, 'waiting': self.waiting, 'cleared': self.cleared})

    @classmethod
    def restore(cls, state):
        if state['schema'] != 'ugrp.traffic_coordinator.v1':
            raise ValueError('unsupported traffic snapshot')
        obj = cls(state['participants'], **{k: state[k] for k in
            ('radius_m', 'uncertainty_m', 'stopping_margin_m', 'max_wait_s')})
        for key in ('generation', 'last_now', 'reports', 'reservations', 'waiting', 'cleared'):
            setattr(obj, key, copy.deepcopy(state[key]))
        obj.epoch = state['epoch'] + 1
        obj._event('RESTART', '*', obj.last_now, retained_generations={
            u: r['generation'] for u, r in obj.reservations.items()})
        return obj


class TrafficCommandGate:
    """Robot-local protection against old, replayed, mismatched or expired GO."""
    def __init__(self, unit_id):
        self.unit_id = unit_id
        self.last_sequence = -1

    def accept(self, permission, report, *, now, epoch, current_generation, duration_s):
        valid = (permission['phase'] == 'GO' and permission['unit_id'] == self.unit_id and
                 permission['coordinator_epoch'] == epoch and
                 permission['generation'] == current_generation and current_generation is not None and
                 permission['identity'] == list(TrafficCoordinator.identity(report)) and
                 permission['sequence'] == report['sequence'] and report['sequence'] > self.last_sequence and
                 permission['issued_at_s'] <= now < permission['expires_at_s'] and
                 now + duration_s <= permission['expires_at_s'] + 1e-9)
        if valid:
            self.last_sequence = report['sequence']
        return valid
