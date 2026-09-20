"""Multi-cargo scene construction and setup-only geometric admission.

No student controller consumes the placement ledger, geometric paths, masks or
simulator identifiers. Scene IDs are not RGB identity tracking.
"""
from __future__ import annotations

import copy
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from harness.multi_object_plan import validate_mission
from harness.pair_navigation import footprint_clear, swept_clear
from sim.act_map_suite import generate_case, scene_config, camera_coverage
from sim.research_dispatch_arena import build_scene_xml, digest

SPEC = Path(__file__).resolve().parents[1] / 'maps/act_generalization/multi_object_layout_v1.json'
HALF = {'beam': [.025, .225, .020], 'box': [.017, .020, .016]}


def configuration(case, layout=None):
    layout = copy.deepcopy(layout or json.loads(SPEC.read_text()))
    if layout['schema'] != 'ugrp.multi_object_layout.v1' or layout['source_case']['split'] != 'train':
        raise ValueError('v1 admission is train-only')
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', case['id']):
        raise ValueError('unsafe case id')
    mission = validate_mission(case['mission'])
    base = generate_case(layout['source_case'])
    config = scene_config(base, physics_seed=layout['physics_seed'])
    config.update(schema='ugrp.multi_object_scene.v1', case_id=case['id'], split='train',
                  mission=mission, mission_sha256=digest(mission), layout_spec_sha256=digest(layout),
                  source_map_id=base['map']['map_id'], source_map_sha256=digest(base['map']),
                  geometry_parameters=layout['geometry'], route_binding_status='not_executable')
    config['setup_only']['spawns'] = layout['spawns']
    config['static_map']['docks'] = {}
    inventory, counters, goals = {}, {'beam': 0, 'box': 0}, {}
    for obj in mission['objects']:
        oid, kind = obj['object_id'], obj['kind']
        i = counters[kind]; counters[kind] += 1
        start = layout[kind+'_starts_m'][i]
        inventory[oid] = {'kind': kind, 'body_name': 'cargo_'+oid, 'joint_name': 'cargo_'+oid+'_free',
                          'position_m': [*start, HALF[kind][2]], 'half_extents_m': list(HALF[kind])}
        goals[oid] = layout[kind+'_goals_m'][i]
    destinations, staged = {}, 0
    for task in mission['tasks']:
        did, oid = task['destination_id'], task['object_id']
        kind = inventory[oid]['kind']
        stage = next(d for d in mission['destinations'] if d['destination_id'] == did)['kind'] == 'staging'
        center = layout['staging_centers_m'][staged] if stage else goals[oid]
        staged += int(stage)
        half = [.07, .27] if kind == 'beam' else [.055, .055]
        destinations[did] = {'center_m': list(center), 'half_extents_m': half, 'relative_yaw_deg': 0,
                             'kind': 'staging' if stage else 'final'}
        config['static_map']['docks'][did] = {'slots': {kind: {'center_m': list(center), 'half_extents_m': half}}}
    config['static_map']['destinations'] = destinations
    config['static_map_sha256'] = digest(config['static_map'])
    config['setup_only']['objects'] = inventory
    # Legacy names are invisible inert API placeholders, never declared cargo.
    config['setup_only']['cargo'] = {'beam': [0., 0., .02], 'box': [0., 0., .016]}
    return config


def static_task(config):
    """Detached authored content only. No initial poses, body names or paths."""
    if config['mission_sha256'] != digest(validate_mission(config['mission'])):
        raise ValueError('mission hash mismatch')
    if config['static_map_sha256'] != digest(config['static_map']):
        raise ValueError('static map hash mismatch')
    static = config['static_map']
    def fields(value, names):
        if not isinstance(value,dict) or set(value) != set(names.split()):
            raise ValueError('unexpected static map field')
    fields(static,'schema map_id version frame bounds_m top_camera obstacles terrain regions routes resource_rules docks destinations')
    from sim.research_dispatch_arena import FIXED_TOP
    if static['top_camera'] != FIXED_TOP or any(static[k] for k in ('terrain','regions','routes','resource_rules')):
        raise ValueError('only the fixed-camera open preview map is supported')
    for box in static['obstacles']:
        fields(box,'id center_m half_extents_m height_m kind')
    required = {d['destination_id'] for d in config['mission']['destinations']}
    if set(static['destinations']) != required or set(static['docks']) != required:
        raise ValueError('mission destinations not bound one-to-one')
    for did,slot in static['destinations'].items():
        fields(slot,'center_m half_extents_m relative_yaw_deg kind')
        fields(static['docks'][did],'slots')
        if len(static['docks'][did]['slots']) != 1 or not set(static['docks'][did]['slots']) <= {'beam','box'}:
            raise ValueError('invalid physical cargo slot')
        for paint in static['docks'][did]['slots'].values():
            fields(paint,'center_m half_extents_m')
            if any(paint[k] != slot[k] for k in paint): raise ValueError('destination paint mismatch')
    return {'schema': 'ugrp.multi_object_static_task.v1', 'mission': copy.deepcopy(config['mission']),
            'mission_sha256': config['mission_sha256'], 'static_map': copy.deepcopy(config['static_map']),
            'static_map_sha256': config['static_map_sha256'],
            'identity_binding': 'symbolic object IDs require RGB tracking; no live identity poses supplied',
            'runtime_support': 'static inspection only; routes and motor adapter not connected'}


def shape_signature(body):
    """Canonical complete subtree, excluding only root pose and instance names."""
    value = copy.deepcopy(body)
    value.attrib.pop('pos', None); value.attrib.pop('quat', None)
    for node in value.iter():
        node.attrib.pop('name', None)
    return hashlib.sha256(ET.tostring(value)).hexdigest()


def build_multi_object_xml(source, config):
    xml, record = build_scene_xml(source, config)
    root = ET.fromstring(xml); world = root.find('worldbody')
    prototypes = {kind: world.find(f"body[@name='{name}']") for kind, name in
                  (('beam', 'team_beam'), ('box', 'dispatch_box'))}
    signatures = {kind: shape_signature(body) for kind, body in prototypes.items()}
    clones = {}
    for oid, item in config['setup_only']['objects'].items():
        prototype = prototypes[item['kind']]
        body = copy.deepcopy(prototype)
        prefix = prototype.get('name')
        for node in body.iter():
            if node.get('name'):
                node.set('name', node.get('name').replace(prefix, item['body_name'], 1))
        body.set('pos', ' '.join(map(str, item['position_m'])))
        joint = body.find('freejoint')
        if joint is None: joint = body.find('joint')
        if joint is None or joint.get('name') != item['joint_name']:
            raise ValueError('cargo joint naming mismatch')
        if shape_signature(body) != signatures[item['kind']]:
            raise ValueError('cargo replica changed shape, appearance or dynamics')
        world.append(body)
        clones[oid] = {'body_name': item['body_name'], 'shape_sha256': shape_signature(body)}
    for body in prototypes.values():
        for node in body.iter('body'): node.set('gravcomp', '1')
        for geom in body.iter('geom'):
            geom.attrib.update(contype='0', conaffinity='0', rgba='0 0 0 0', group='5')
    if any(eq.get('active') != 'false' for eq in root.findall('equality/weld')):
        raise ValueError('weld assistance must be OFF')
    xml = ET.tostring(root, encoding='unicode')
    return xml, {**record, 'scene_xml_sha256': hashlib.sha256(xml.encode()).hexdigest(),
                 'mission_sha256': config['mission_sha256'], 'source_map_sha256': config['source_map_sha256'],
                 'prototype_shape_sha256': signatures, 'cargo_replicas': clones,
                 'inert_legacy_placeholders': ['team_beam', 'dispatch_box']}


def _map(config, obstacles, footprint):
    return {'bounds_m': config['static_map']['bounds_m'], 'obstacles': obstacles, 'footprint': footprint}


def translation_path(start, goal, data, *, grid=.06, budget=2400):
    """Fixed-yaw conservative sweep search, not a dynamic robot trajectory."""
    start, goal = [*start[:2], 0.], [*goal[:2], 0.]
    if not footprint_clear(start, data) or not footprint_clear(goal, data): return None
    if swept_clear(start, goal, data): return [start, goal]
    def pose(node): return [start[0]+grid*node[0], start[1]+grid*node[1], 0.]
    costs, parents, closed = {(0, 0): 0.}, {}, set()
    queue = [(math.dist(start, goal), 0., (0, 0))]
    moves = [(x,y) for x in (-1,0,1) for y in (-1,0,1) if x or y]
    while queue and len(closed) < budget:
        _, cost, node = heapq.heappop(queue)
        if node in closed: continue
        closed.add(node); p = pose(node)
        if math.dist(p, goal) <= grid*1.5 and swept_clear(p, goal, data):
            result = [goal, p]
            while node in parents:
                node = parents[node]; result.append(pose(node))
            return list(reversed(result))
        for dx, dy in moves:
            nxt = (node[0]+dx, node[1]+dy); distance = cost + grid*math.hypot(dx,dy)
            if nxt in closed or distance >= costs.get(nxt, math.inf): continue
            q = pose(nxt)
            if not swept_clear(p, q, data): continue
            costs[nxt] = distance; parents[nxt] = node
            heapq.heappush(queue, (distance+math.dist(q,goal), distance,nxt))
    return None


def geometry_admission(config):
    """Search placement order with remaining AND delivered cargo as obstacles.

    The ledger updates authored XY coordinates only, never the simulator. Robot
    parking-to-grasp paths are independent reachability probes; no reset or
    complete joint robot motion is executed or certified.
    """
    inventory = config['setup_only']['objects']; g = config['geometry_parameters']
    tasks = {t['task_id']: t for t in config['mission']['tasks']}
    slots = config['static_map']['destinations']
    base_obstacles = [o for o in config['static_map']['obstacles'] if not o['id'].startswith('bound_')]
    initial = {oid: item['position_m'][:2] for oid,item in inventory.items()}
    blocked, visited, best = {}, set(), []
    exhausted = False

    def search(done, positions, trace):
        nonlocal best, exhausted
        if len(trace) > len(best): best = trace
        if len(done) == len(tasks): return trace
        key = tuple(sorted(done))
        if key in visited: return None
        if len(visited) >= g['order_state_budget']:
            exhausted = True; return None
        visited.add(key)
        for tid, task in tasks.items():
            if tid in done or not set(task['after']) <= done: continue
            oid = task['object_id']; kind = inventory[oid]['kind']
            obstacles = list(base_obstacles) + [
                {'id': other, 'center_m': xy, 'half_extents_m': inventory[other]['half_extents_m'][:2]}
                for other,xy in positions.items() if other != oid]
            start = [positions[oid][0]-g['grasp_offset_x_m'], positions[oid][1]]
            target = slots[task['destination_id']]['center_m']
            goal = [target[0]-g['grasp_offset_x_m'], target[1]]
            data = _map(config, obstacles, g['pair_footprint' if kind == 'beam' else 'solo_footprint'])
            route = translation_path(start, goal, data, grid=g['grid_m'], budget=g['route_node_budget'])
            if route is None:
                blocked[','.join(key)+'/'+tid] = 'no_fixed_yaw_loaded_path'; continue
            # Include the target cargo when probing unloaded approach clearance.
            approach_obstacles = obstacles + [{'id': oid, 'center_m': positions[oid],
                                               'half_extents_m': inventory[oid]['half_extents_m'][:2]}]
            targets = [[start[0]-.06, start[1]+dy] for dy in ((-.34,.34) if kind == 'beam' else (0.,))]
            paths = {}
            for end, point in enumerate(targets):
                for rid, spawn in config['setup_only']['spawns'].items():
                    f = g['unloaded_footprint']
                    peers = [{'id': 'parked_'+other, 'center_m': pose[:2],
                              'half_extents_m': [f['half_forward_m'], f['half_lateral_m']]}
                             for other,pose in config['setup_only']['spawns'].items() if other != rid]
                    unloaded = _map(config, approach_obstacles+peers, f)
                    path = translation_path(spawn[:2], point, unloaded, grid=g['grid_m'], budget=g['route_node_budget'])
                    if path is not None: paths[end,rid] = path
            assignment = next((robots for robots in itertools.permutations(config['setup_only']['spawns'], len(targets))
                               if all((end,rid) in paths for end,rid in enumerate(robots))), None)
            if assignment is None:
                blocked[','.join(key)+'/'+tid] = 'no_distinct_carrier_approach_paths'; continue
            row = {'task_id': tid, 'object_id': oid, 'obstacle_object_ids': sorted(set(positions)-{oid}),
                   'loaded_path_m_rad': route, 'nominal_carrier_assignment': list(assignment),
                   'independent_approach_paths': {rid: paths[end,rid] for end,rid in enumerate(assignment)},
                   'approach_peer_model': 'all other robots stationary at authored parking poses',
                   'start_quarter_turn_clear': swept_clear([*start,0.], [*start,math.pi/2], data),
                   'goal_quarter_turn_clear': swept_clear([*goal,0.], [*goal,math.pi/2], data)}
            result = search(done|{tid}, {**positions, oid: list(target)}, trace+[row])
            if result is not None: return result
        return None

    trace = search(set(), initial, [])
    # Slots are physical cargo footprints plus margin; overlap is never fixed
    # by silently reducing the number of objects or treating two slots as one.
    slot_overlaps = []
    for a,b in itertools.combinations(slots,2):
        aa,bb = slots[a],slots[b]
        if all(abs(aa['center_m'][k]-bb['center_m'][k]) < aa['half_extents_m'][k]+bb['half_extents_m'][k] for k in (0,1)):
            slot_overlaps.append([a,b])
    coverage_config = copy.deepcopy(config)
    coverage_config['static_map']['obstacles'] += [
        {'id': oid, 'center_m': item['position_m'][:2], 'half_extents_m': item['half_extents_m'][:2],
         'height_m': item['half_extents_m'][2]*2} for oid,item in inventory.items()]
    coverage_config['static_map']['obstacles'] += [
        {'id': did, 'center_m': slot['center_m'], 'half_extents_m': slot['half_extents_m'], 'height_m': .002}
        for did,slot in slots.items()]
    coverage = camera_coverage(coverage_config)
    candidate = trace is not None and not slot_overlaps and all(c['inside_top_frustum'] for c in coverage)
    return {'scope': 'setup-only geometry; fixed-yaw translation and independent parking reachability probes',
            'classification': 'geometric_candidate' if candidate else 'unresolved_layout',
            'complete_placement_order_found': trace is not None, 'trace': trace if trace is not None else best,
            'states_visited': len(visited), 'order_budget_exhausted': exhausted, 'blocked_attempts': blocked,
            'slot_overlaps': slot_overlaps, 'fov_checks': coverage,
            'joint_robot_trajectory': 'not_planned', 'grasp_and_transport': 'not_run',
            'failure_interpretation': 'failed bounded fixed-yaw search is not proof of physical impossibility'}
