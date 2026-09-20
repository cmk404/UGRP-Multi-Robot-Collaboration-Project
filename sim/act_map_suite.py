"""Reproducible map/protocol preparation, never a student controller.

Pair maps are the common static representation. A separate adapter uses the
production dispatch scene builder for inspection. Setup, split labels and
geometric search results are deliberately absent from the student task.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import random
import re

from harness.pair_navigation import validate_map, footprint_clear, plan_route, swept_clear
from sim.research_dispatch_arena import FIXED_TOP, digest

ROOT = Path(__file__).resolve().parents[1]
SPEC = ROOT / 'maps/act_generalization/suite_v1.json'
FOOTPRINT = {'half_forward_m': .20, 'half_lateral_m': .51, 'margin_m': .025}
TOPOLOGIES = {'open': 'room', 'door': 'room-door-room',
              'corner': 'entry-left-corner-exit',
              'double_door': 'room-door-offset-room-door-room',
              'too_narrow': 'room-impassable-door-room', 'blocked': 'disconnected-rooms'}
SPLITS = {'train', 'dev', 'test_a', 'test_b', 'control', 'regression'}


def read_json(path):
    return json.loads(Path(path).read_text())


def layout_digest(data):
    """Count physical layouts separately from goal/start repetitions."""
    return digest({k: data[k] for k in ('bounds_m', 'obstacles', 'top_camera')})


def _wall(name, x, lo, hi):
    # Avoid round-off outside strict map bounds; 10um gaps cannot pass the load.
    lo, hi = lo+.00001, hi-.00001
    return {'id': name, 'center_m': [round(x, 6), round((lo+hi)/2, 6)],
            'half_extents_m': [.035, round((hi-lo)/2, 6)], 'height_m': .16}


def generate_case(row):
    """All randomness is local to map_seed; physics/training seeds are separate."""
    required = {'id', 'split', 'family', 'map_seed', 'goal_yaw_deg'}
    if set(row) != required or row['split'] not in SPLITS:
        raise ValueError('invalid case specification')
    if not isinstance(row['id'], str) or not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,79}', row['id']):
        raise ValueError('invalid case id')
    if type(row['map_seed']) is not int or row['map_seed'] < 0:
        raise ValueError('nonnegative map seed required')
    if row['goal_yaw_deg'] not in (0, 90):
        raise ValueError('v1 goal headings are 0 or 90 degrees')
    family = row['family']
    if family not in TOPOLOGIES:
        raise ValueError('unknown map family')
    rng = random.Random(row['map_seed'])
    sample = lambda a, b: round(rng.uniform(a, b), 6)
    parameters = {}
    if family == 'corner':
        source = ROOT / 'maps/pair_navigation/l-corner.json'
        data = read_json(source)
        # A changed inside-corner length is geometry, not a relabelled seed.
        shift = sample(-.018, .018)
        data['obstacles'][0]['center_m'][1] = round(-2.4395+shift/2, 6)
        data['obstacles'][0]['half_extents_m'][1] = round(.4595+shift/2, 6)
        data['goal']['center_m'][0] = sample(-.545, -.495)
        data['goal']['relative_yaw_deg'] = row['goal_yaw_deg']
        nominal = [.62, -2., 0.]
        parameters = {'inside_corner_top_shift_m': shift,
                      'source': str(source.relative_to(ROOT)), 'source_sha256': digest(read_json(source))}
    else:
        # Small boundary changes produce distinct layouts even for open rooms.
        east = sample(2.02, 2.10)
        bounds = [-1.02, east, -3.14, -.86]
        start_y, goal_y = sample(-2.04, -1.96), sample(-2.08, -1.92)
        goal_x = sample(1.40, 1.47)
        nominal = [-.60, start_y, 0.]
        data = {'schema': 'ugrp.pair_navigation_map.v1', 'map_id': 'pending',
                'top_camera': copy.deepcopy(FIXED_TOP), 'bounds_m': bounds,
                'start_zone': {'center_m': nominal[:2], 'radius_m': .15},
                'goal': {'center_m': [goal_x, goal_y], 'relative_yaw_deg': row['goal_yaw_deg']},
                'obstacles': [], 'footprint': dict(FOOTPRINT), 'grid_m': .06}
        parameters = {'east_bound_m': east, 'doors': []}
        if family != 'open':
            xs = [.0, .95] if family == 'double_door' else [sample(.40, .55)]
            for index, x in enumerate(xs):
                width = (sample(1.18, 1.26) if family == 'double_door' else
                         .40 if family == 'too_narrow' else
                         0. if family == 'blocked' else sample(.62, .82))
                center = -2. + ((-.12 if index == 0 else .12) if family == 'double_door'
                               else sample(-.06, .06))
                if width == 0:
                    data['obstacles'].append(_wall('divider', x, bounds[2], bounds[3]))
                else:
                    data['obstacles'] += [_wall(f'gate_{index}_lower', x, bounds[2], center-width/2),
                                          _wall(f'gate_{index}_upper', x, center+width/2, bounds[3])]
                parameters['doors'].append({'x_m': x, 'center_y_m': center, 'width_m': width})
    # Opaque static identity: no holdout label, seed or expected outcome in input.
    data['map_id'] = 'map_' + digest({k: v for k, v in data.items() if k != 'map_id'})[:16]
    validate_map(data)
    return {'id': row['id'], 'split': row['split'], 'family': family,
            'topology_id': TOPOLOGIES[family], 'map_seed': row['map_seed'],
            'parameters': parameters, 'map': data,
            'evaluation_only': {'nominal_start_m_rad': nominal,
                                'design_intent': 'impossible_control' if family in ('too_narrow', 'blocked')
                                else 'passage_candidate', 'teacher_passage': 'not_run'}}


def student_task(data):
    """Only a validated authored map enters the task, never the case envelope."""
    data = copy.deepcopy(validate_map(data))
    return {'schema': 'ugrp.local_pair_map_task.v1',
            'instruction': 'Carry the plain beam with a peer into the authored goal region and heading. '
                           'Estimate current pose and progress from own RGB, fixed TOP RGB and own issued history.',
            'static_map': data, 'static_map_sha256': digest(data),
            'runtime_support': 'map inspection only; generalized dispatch/ACT route contract not implemented'}


def geometry_check(case):
    data = case['map']
    start = case['evaluation_only']['nominal_start_m_rad']
    goal = [*data['goal']['center_m'], math.radians(data['goal']['relative_yaw_deg'])]
    start_clear, goal_clear = footprint_clear(start, data), footprint_clear(goal, data)
    route = plan_route(start, goal, data)
    if route and not all(swept_clear(a, b, data) for a, b in zip(route, route[1:])):
        raise ValueError('planner returned an invalid swept footprint path')
    # A failed discretized search alone is NOT a proof of physical impossibility.
    doors = case['parameters'].get('doors', [])
    minimum_width = 2 * min(FOOTPRINT['half_forward_m']+FOOTPRINT['margin_m'],
                            FOOTPRINT['half_lateral_m']+FOOTPRINT['margin_m'])
    barrier_proof = any(d['width_m'] < minimum_width for d in doors)
    return {'scope': 'authored nominal pose and conservative complete-load geometry only',
            'start_clear': start_clear, 'goal_clear': goal_clear,
            'route_found': route is not None, 'route_m_rad': route,
            'classification': ('geometric_candidate' if route else 'geometric_impossible_control'
                               if barrier_proof else 'no_lattice_path_unresolved'),
            'barrier_width_proof': barrier_proof, 'minimum_loaded_width_m': minimum_width,
            'teacher_passage': 'not_run', 'student_passage': 'not_run'}


def scene_config(case, *, physics_seed=11, event='none'):
    """Setup owner only. Do not pass this configuration to a student.

    Events remain an independent axis. Non-baseline events are rejected until
    the event scheduler, observation audit and paired baseline are implemented.
    """
    if event != 'none':
        raise ValueError('event scheduling is not implemented in map preparation v1')
    if type(physics_seed) is not int or physics_seed < 0:
        raise ValueError('nonnegative physics seed required')
    data = validate_map(case['map'])
    xmin, xmax, ymin, ymax = data['bounds_m']
    # Bounds have matching physical walls, on the OUTSIDE of free-space bounds.
    boundary = [
        {'id': 'bound_w', 'center_m': [xmin-.02, (ymin+ymax)/2], 'half_extents_m': [.02, (ymax-ymin)/2]},
        {'id': 'bound_e', 'center_m': [xmax+.02, (ymin+ymax)/2], 'half_extents_m': [.02, (ymax-ymin)/2]},
        {'id': 'bound_s', 'center_m': [(xmin+xmax)/2, ymin-.02], 'half_extents_m': [(xmax-xmin)/2, .02]},
        {'id': 'bound_n', 'center_m': [(xmin+xmax)/2, ymax+.02], 'half_extents_m': [(xmax-xmin)/2, .02]}]
    obstacles = [{**copy.deepcopy(o), 'kind': 'wall'} for o in data['obstacles']]
    obstacles += [{**o, 'height_m': .10, 'kind': 'wall'} for o in boundary]
    gx, gy = data['goal']['center_m']
    yaw = data['goal']['relative_yaw_deg']
    # Goal paint expresses the actual final beam axis, not a changed payload.
    slot_half = [.27, .14] if yaw == 90 else [.14, .27]
    static = {'schema': 'ugrp.dispatch_map_preview.v1', 'map_id': data['map_id'],
              'version': 1, 'frame': 'warehouse_xy_m', 'bounds_m': copy.deepcopy(data['bounds_m']),
              'top_camera': copy.deepcopy(data['top_camera']), 'obstacles': obstacles, 'terrain': [],
              'regions': {}, 'routes': {}, 'resource_rules': {},
              'docks': {'local_goal': {'slots': {'beam': {'center_m': [gx, gy], 'half_extents_m': slot_half}}}}}
    x, y = data['start_zone']['center_m']
    # Static preview only, folded arms and beam on floor. No loaded state reset.
    setup = {'spawns': {'r1': [x-.06, y+.34, .032355118817659255, 0.],
                        'r3': [x-.06, y-.34, .032355118817659255, 0.],
                        'r2': [xmax-.24, ymin+.24, .032355118817659255, 0.]},
             'cargo': {'beam': [x+.18, y, .020], 'box': [xmax-.24, ymin+.54, .016]},
             'unexpected_obstacles': []}
    return {'schema': 'ugrp.dispatch_episode.v1', 'seed': physics_seed,
            'variant': 'local_map_preview', 'static_map': static, 'static_map_sha256': digest(static),
            'source_pair_map_sha256': digest(data), 'setup_only': setup,
            'event_schedule_only': {'event_id': event, 'events': []}}


def load_suite(path=SPEC):
    spec = read_json(path)
    if spec.get('schema') != 'ugrp.act_map_suite.v1':
        raise ValueError('unknown suite version')
    cases = [generate_case(row) for row in spec['cases']]
    if spec['include_legacy_regression']:
        catalog = read_json(ROOT / 'maps/pair_navigation/catalog.json')
        for row in catalog['entries']:
            data = validate_map(read_json(ROOT / 'maps/pair_navigation' / row['map']))
            cases.append({'id': 'regression_'+row['id'], 'split': 'regression', 'family': row['id'],
                          'topology_id': 'legacy_'+row['id'], 'map_seed': None,
                          'parameters': {'source': 'maps/pair_navigation/'+row['map']}, 'map': data,
                          'evaluation_only': {'nominal_start_m_rad': catalog['nominal_planning_start_m_rad'],
                                              'design_intent': 'historical_regression', 'teacher_passage': 'not_run'}})
    validate_splits(cases)
    return spec, cases


def validate_splits(cases):
    """Fail closed on repeated layouts, aliases, seed overlap or topology leaks."""
    ids, seeds, layouts = set(), set(), set()
    topologies = {s: set() for s in SPLITS}
    for case in cases:
        if case['id'] in ids or case['split'] not in SPLITS:
            raise ValueError('duplicate case or invalid split')
        ids.add(case['id'])
        validate_map(case['map'])
        topologies[case['split']].add(case['topology_id'])
        if case['map_seed'] is not None:
            if case['map_seed'] in seeds:
                raise ValueError('map generation seed reused across instances')
            seeds.add(case['map_seed'])
        fingerprint = layout_digest(case['map'])
        if fingerprint in layouts:
            raise ValueError('physical layout repeated; goal/start repeats are not new maps')
        layouts.add(fingerprint)
    familiar = topologies['train']
    if not topologies['dev'] <= familiar or not topologies['test_a'] <= familiar:
        raise ValueError('development and test A require familiar topology families')
    if topologies['test_b'] & (familiar | topologies['dev'] | topologies['test_a'] | topologies['regression']):
        raise ValueError('test B topology leaked into development or regression')
    if not all(topologies[s] for s in ('train', 'dev', 'test_a', 'test_b', 'control')):
        raise ValueError('all protocol splits must be populated')


def manifest(spec, cases):
    validate_splits(cases)
    return {'schema': 'ugrp.act_map_manifest.v1', 'suite_id': spec['suite_id'],
            'spec_sha256': digest(spec), 'status': 'map design frozen; final evaluation NOT authorized by this manifest',
            'physics_initialization_seeds': spec['physics_initialization_seeds'],
            'model_training_seeds': None, 'event_axis': ['none'],
            'cases': [{'id': c['id'], 'split': c['split'], 'family': c['family'],
                       'topology_id': c['topology_id'], 'map_seed': c['map_seed'],
                       'map_sha256': digest(c['map']), 'layout_sha256': layout_digest(c['map']),
                       'task_sha256': digest(student_task(c['map'])),
                       'parameters': c['parameters']} for c in cases]}


def load_prepared_map(directory):
    """Verify an exported map/task against its frozen parent manifest."""
    directory = Path(directory)
    record = read_json(directory.parent/'manifest.json')
    matches = [r for r in record['cases'] if r['id'] == directory.name]
    if len(matches) != 1:
        raise ValueError('map is not uniquely registered in manifest')
    row = matches[0]
    data = validate_map(read_json(directory/'map.json'))
    task = student_task(data)
    if (digest(data) != row['map_sha256'] or layout_digest(data) != row['layout_sha256']
            or digest(task) != row['task_sha256'] or read_json(directory/'student-task.json') != task):
        raise ValueError('map/task hash mismatch')
    return data, task


def require_frozen_protocol(protocol):
    """A future final cohort must call this before consuming holdout results."""
    required = ('map_manifest_sha256', 'source_sha', 'policy_hashes', 'model_training_seeds',
                'physics_initialization_seeds', 'max_sim_seconds', 'max_actions',
                'max_api_cost_usd', 'success_tolerance_drop', 'required_time_improvement',
                'required_cost_improvement', 'collision_rule', 'drop_rule', 'sample_period_s')
    if protocol.get('status') != 'frozen_final' or any(protocol.get(k) is None for k in required):
        raise ValueError('final protocol is not frozen; complete pilot and preregister all budgets/criteria')
    if any(not protocol[k] for k in ('policy_hashes', 'model_training_seeds', 'physics_initialization_seeds')):
        raise ValueError('final protocol requires policies and independent seeds')
    for key in ('model_training_seeds', 'physics_initialization_seeds'):
        values = protocol[key]
        if (not isinstance(values, list) or len(values) != len(set(values))
                or any(type(v) is not int or v < 0 for v in values)):
            raise ValueError('invalid independent seeds')
    for key in ('max_sim_seconds', 'max_actions', 'sample_period_s'):
        if not isinstance(protocol[key], (int, float)) or isinstance(protocol[key], bool) or not math.isfinite(protocol[key]) or protocol[key] <= 0:
            raise ValueError('invalid final protocol budget: '+key)
    for key in ('max_api_cost_usd', 'success_tolerance_drop', 'required_time_improvement', 'required_cost_improvement'):
        v = protocol[key]
        if type(v) not in (int, float) or not math.isfinite(v) or v < 0 or (key != 'max_api_cost_usd' and v > 1):
            raise ValueError('invalid final protocol threshold: '+key)
    if type(protocol['max_actions']) is not int:
        raise ValueError('max_actions must be an integer')
    for key, size in (('source_sha', 40), ('map_manifest_sha256', 64)):
        if not isinstance(protocol[key], str) or not re.fullmatch('[0-9a-f]{'+str(size)+'}', protocol[key]):
            raise ValueError('invalid protocol hash: '+key)
    if (not isinstance(protocol['policy_hashes'], dict)
            or any(not isinstance(v, str) or not re.fullmatch('[0-9a-f]{64}', v) for v in protocol['policy_hashes'].values())
            or not all(isinstance(protocol[k], str) and protocol[k].strip() for k in ('collision_rule', 'drop_rule'))):
        raise ValueError('policy hashes and operational evaluation rules required')
    return copy.deepcopy(protocol)
