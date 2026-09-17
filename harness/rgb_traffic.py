"""Connect the existing heading navigator to atomic RGB traffic reservations."""
from __future__ import annotations

import copy
import hashlib

from harness.heading_map_navigation import HeadingMapNavigator
from harness.traffic_reservations import TrafficCoordinator, TrafficCommandGate, path_distance


STOP = {'kind': 'mecanum', 'forward': 0., 'left': 0., 'turn': 0., 'duration_s': .25}


def moving(action):
    return any(abs(action[k]) > 1e-12 for k in ('forward', 'left', 'turn'))


class RGBTrafficRuntime:
    """Own RGB + public RGB + static maps; no access to the physics owner.

    Proposals are transactional: a denied command does not advance the
    navigator's calibration or turn state and is never recorded as issued.
    """
    def __init__(self, maps, *, coordination=True):
        self.maps = copy.deepcopy(maps)
        self.actors = {u: HeadingMapNavigator(data, u) for u, data in maps.items()}
        self.coordinator = TrafficCoordinator(maps)
        self.gates = {u: TrafficCommandGate(u) for u in maps}
        self.issued = {u: [] for u in maps}
        self.coordination = coordination
        self.route_versions = {u: 1 for u in maps}
        self.published_routes = {}

    def step(self, frames, sequence, now, *, missing=(), restart=False):
        if restart:
            self.coordinator = TrafficCoordinator.restore(self.coordinator.snapshot())
        proposals, candidates, reports = {}, {}, {}
        for unit, actor in self.actors.items():
            candidates[unit] = copy.deepcopy(actor)
            own, top = frames[unit]
            decision = candidates[unit].decide(own, top, sequence)
            proposals[unit] = decision
            diagnostic = decision['diagnostics']
            route = diagnostic['path']
            if route:
                previous = self.published_routes.get(unit)
                if previous and max(path_distance(p, previous) for p in route) > .04:
                    self.route_versions[unit] += 1
                    self.published_routes[unit] = copy.deepcopy(route)
                elif previous is None:
                    self.published_routes[unit] = copy.deepcopy(route)
            report = {'unit_id': unit, 'participants': [unit], 'accepted_by': [unit],
                'task_id': self.maps[unit]['map_id'] + ':' + unit, 'plan_version': 1,
                'route_version': self.route_versions[unit],
                'sequence': sequence, 'clock_s': now, 'clock_id': 'issued_command_clock_v1',
                'source': 'top_rgb', 'top_sha256': hashlib.sha256(top).hexdigest(),
                'position_m': diagnostic['position_estimate_m'],
                'heading_rad': diagnostic['heading']['estimate_rad'], 'uncertainty_m': .035,
                'footprint': {'kind': 'unloaded_circle', 'radius_m': .18},
                'route': diagnostic['path'], 'arrived': decision['status'] == 'arrived',
                'visual_ok': diagnostic['localization']['ok']}
            reports[unit] = report
        sent = {u: report for u, report in reports.items() if u not in missing}
        permissions = self.coordinator.step(sent, now) if self.coordination else {
            u: {'phase': 'UNCOORDINATED', 'reason': 'independent RGB baseline'} for u in self.maps}
        actions, accepted = {}, {}
        for unit, decision in proposals.items():
            action = decision['action']
            reservation = self.coordinator.reservations.get(unit)
            allowed = not self.coordination or self.gates[unit].accept(permissions[unit], reports[unit], now=now,
                epoch=self.coordinator.epoch,
                current_generation=reservation['generation'] if reservation else None,
                duration_s=action['duration_s'])
            # A zero command is safe and may commit processing of a previously
            # issued probe/turn; a rejected movement proposal never commits.
            accepted[unit] = allowed or not moving(action)
            if accepted[unit]:
                self.actors[unit] = candidates[unit]
                actions[unit] = copy.deepcopy(action)
            else:
                actions[unit] = dict(STOP)
                self.actors[unit]._issued.append(dict(STOP))
            self.issued[unit].append(copy.deepcopy(actions[unit]))
        return {'sequence': sequence, 'clock_s': now, 'proposals': proposals, 'reports': reports,
                'missing_reports': list(missing), 'restart': restart,
                'permissions': permissions, 'accepted_proposals': accepted, 'issued_actions': actions,
                'reservation_state': self.coordinator.snapshot()}
