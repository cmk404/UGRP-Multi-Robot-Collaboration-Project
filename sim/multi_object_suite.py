"""Authored logical mission templates, not a physics scene or a model policy."""
from __future__ import annotations

import json
from pathlib import Path

from harness.multi_object_plan import validate_mission, validate_plan
from harness.three_robot_plan import ROBOTS, digest

SPEC = Path(__file__).resolve().parents[1] / 'maps/act_generalization/multi_object_pilot_v1.json'
PATTERNS = {'independent', 'shared_gate', 'clear_first', 'stage_then_deliver'}


def make_mission(row):
    if set(row) != {'id', 'beams', 'boxes', 'pattern'} or row['pattern'] not in PATTERNS:
        raise ValueError('invalid pilot case')
    if any(type(row[k]) is not int or row[k] < 0 for k in ('beams', 'boxes')):
        raise ValueError('nonnegative object counts required')
    if not 1 <= row['beams'] + row['boxes'] <= 8:
        raise ValueError('pilot supports one to eight objects')
    if row['pattern'] == 'stage_then_deliver' and (row['beams'], row['boxes']) != (1, 2):
        raise ValueError('staging pilot requires one beam and two boxes')
    if row['pattern'] == 'clear_first' and (not row['beams'] or not row['boxes']):
        raise ValueError('clear-first requires a beam and a box')
    objects, destinations, tasks = [], [], []
    kinds = ['beam'] * row['beams'] + ['box'] * row['boxes']
    for i, kind in enumerate(kinds, 1):
        oid = f'object_{i:02}'
        # IDs are symbolic task references, not visible labels or live poses.
        objects.append({'object_id': oid, 'kind': kind})
        dest = f'goal_{"a" if i % 2 else "b"}_{i:02}'
        destinations.append({'destination_id': dest, 'kind': 'final'})
        tasks.append({'task_id': f'deliver_{i:02}', 'object_id': oid, 'destination_id': dest, 'after': []})
    if row['pattern'] == 'clear_first':
        clear = tasks[row['beams']]['task_id']
        for task in tasks[:row['beams']]:
            task['after'] = [clear]
    if row['pattern'] == 'stage_then_deliver':
        staged = []
        for i in (2, 3):
            dest, tid = f'staging_{i:02}', f'stage_{i:02}'
            destinations.append({'destination_id': dest, 'kind': 'staging'})
            staged.append({'task_id': tid, 'object_id': f'object_{i:02}', 'destination_id': dest, 'after': []})
            tasks[i-1]['after'] = [tid, 'deliver_01']
        tasks[0]['after'] = [t['task_id'] for t in staged]
        tasks = staged + tasks
    routes = [{'route_id': f'lane_{x}', 'resources': [f'lane_{x}'] + (
        ['shared_gate'] if row['pattern'] == 'shared_gate' else [])} for x in ('a', 'b')]
    return validate_mission({'schema': 'ugrp.multi_object_mission.v1', 'objects': objects,
                             'destinations': destinations, 'routes': routes, 'tasks': tasks})


def load_pilot(path=SPEC):
    spec = json.loads(Path(path).read_text())
    if spec['schema'] != 'ugrp.multi_object_pilot.v1':
        raise ValueError('unknown pilot schema')
    ids = [row['id'] for row in spec['cases']]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate pilot id')
    return spec, [{'id': row['id'], 'configuration': row, 'mission': make_mission(row)} for row in spec['cases']]


def fixture_plan(mission, *, rotation=0):
    """Deterministic TEST proposal; never use as an LLM fallback or a transport plan."""
    mission = validate_mission(mission)
    kinds = {o['object_id']: o['kind'] for o in mission['objects']}
    tasks = []
    for i, job in enumerate(mission['tasks']):
        lead = (i + rotation) % len(ROBOTS)
        participants = ({ROBOTS[lead]: 'end_a', ROBOTS[(lead+1) % 3]: 'end_b'}
                        if kinds[job['object_id']] == 'beam' else {ROBOTS[(lead+1) % 3]: 'solo'})
        tasks.append({'task_id': job['task_id'], 'participants': participants,
                      'route_id': 'lane_a' if i % 2 == 0 else 'lane_b', 'after': list(job['after'])})
    return validate_plan({'schema': 'ugrp.multi_object_plan.v1', 'mission_sha256': digest(mission),
                          'tasks': tasks}, mission)
