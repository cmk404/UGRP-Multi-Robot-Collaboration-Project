"""Authored unloaded courses, fixed private starts, and scheduled test faults."""
from __future__ import annotations

import copy
import json
from pathlib import Path

from sim.authored_navigation_map import validate_map


SCENARIOS = ('crossing', 'head_on', 'following', 'delayed_owner', 'report_gap', 'restart', 'blocked_exit')


def scenario(name):
    if name not in SCENARIOS:
        raise ValueError('unknown traffic scenario')
    template = json.loads((Path(__file__).resolve().parents[1]/'maps/navigation/open.json').read_text())
    template['map_id'] = 'traffic_' + name
    template['bounds_m'] = [-.95, 2.05, -3.1, -.9]
    if name == 'head_on':
        starts = {'r1': [-.60, -2.45], 'r3': [1.65, -1.55]}
        goals = {'r1': [1.65, -2.45], 'r3': [-.60, -1.55]}
        yaws = {'r1': 0., 'r3': 180.}
        template['obstacles'] = [{
            'id': side, 'kind': 'wall', 'center_m': [.55, y],
            'half_extents_m': [.35, .36], 'height_m': .22,
            'traversable': False, 'cost_multiplier': 1.}
            for side, y in [('north', -1.26), ('south', -2.74)]]
    elif name == 'following':
        starts = {'r1': [.25, -2.], 'r3': [-.65, -2.]}
        goals = {'r1': [1.70, -2.], 'r3': [.95, -2.]}
        yaws = {'r1': 0., 'r3': 0.}
    else:
        starts = {'r1': [-.60, -2.], 'r3': [.55, -2.80]}
        goals = {'r1': [1.70, -2.], 'r3': [.55, -1.20]}
        yaws = {'r1': 0., 'r3': 90.}
        if name == 'blocked_exit':
            starts['r3'] = [1.45, -2.]
            goals['r3'] = [1.45, -2.]
            # An occupied exit; no synthesized backwards escape command.
            yaws['r3'] = 0.
    maps, setup = {}, {}
    for unit in ('r1', 'r3'):
        data = copy.deepcopy(template)
        data['zones'] = {'start': {'center_m': starts[unit], 'radius_m': .14},
                         'goal': {'center_m': goals[unit], 'radius_m': .10}}
        maps[unit] = validate_map(data)
        setup[unit] = {'start_xy_m': starts[unit], 'start_yaw_deg': yaws[unit]}
    return maps, setup


def faults(name, sequence):
    return {'missing': ['r1'] if name == 'report_gap' and 24 <= sequence < 29 else [],
            'drive_blocked': ['r1'] if name == 'delayed_owner' and 24 <= sequence < 36 else [],
            'restart': name == 'restart' and sequence == 24}
