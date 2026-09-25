"""Zone-goal delivery arena: several coloured boxes, three goal zones, two TOPs.

A coordination benchmark scene. It widens the dispatch laboratory eastwards and
keeps the approved TOP camera; CCTVs of the identical specification cover the
rest (zone_open: one east; zone_wide: east plus a northern row of two). Box replicas keep the production cyan box shape, mass and
friction; only the paint colour differs so robots can tell kinds apart in RGB.
Setup poses and simulator IDs are setup-only and never reach the robots.
"""
from __future__ import annotations

import copy
import hashlib
import json
import random
import xml.etree.ElementTree as ET

from pathlib import Path

from sim.research_dispatch_arena import FIXED_TOP, ROBOTS, digest

SCHEMA = 'ugrp.zone_arena.v1'
MAP_DIR = Path(__file__).resolve().parents[1] / 'maps' / 'zones'
VARIANTS = ('zone_open', 'zone_wide')


def _same_top(name, dx, dy):
    """A CCTV of the approved TOP specification, shifted on the floor plane."""
    x, y, z = FIXED_TOP['position_m']
    return {**copy.deepcopy(FIXED_TOP), 'name': name, 'position_m': [x+dx, y+dy, z]}


EAST_TOP = _same_top('cctv_top_east', 3.3, 0.)
# zone_wide doubles the arena northwards: one more row of the same two CCTVs,
# 2.3 m north, so the four views still overlap by about 0.3 m on the floor.
NORTH_TOP = _same_top('cctv_top_north', 0., 2.3)
NORTH_EAST_TOP = _same_top('cctv_top_north_east', 3.3, 2.3)
# Kind -> paint. Cyan is the production box colour; the others are new kinds.
COLORS = {'cyan': '.20 .65 .70 1', 'red': '.80 .12 .10 1', 'green': '.15 .62 .20 1',
          'yellow': '.92 .78 .10 1'}
BOX_HALF = (.017, .020, .016)
ZONE_IDS = ('A', 'B', 'C')
SLOTS_PER_ZONE = 3
# Reference goal when a scene is opened without one (catalog/preview).
DEFAULT_GOAL = {'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}
SLOT_SPACING_M = .24
# Pickup grid: columns along +x, rows along y. Robots grasp facing +x, so each
# column leaves the chassis room west of every box. The 0.6 m spacing keeps
# lanes that a robot carrying a box can drive through, plus a corridor between
# the spawn line and the first column: with 8 boxes on a denser grid
# (Z1-G8-dyn) no teacher path existed and the robots never left their spawns.
PICKUP_COLUMNS_X = (-.20, .40, 1.00, 1.60)
PICKUP_ROWS_Y = (-1.45, -2.05, -2.65)
SPAWN_X = -.85
SPAWN_ROWS_Y = (-1.35, -2.0, -2.65)
# Per-variant geometry. zone_open is map v2 exactly as used by Z1-Z3.
# views: (camera, image label shown to robots, frame key, file suffix, role).
LAYOUTS = {
    'zone_open': {
        'version': 2, 'bounds': [-1.05, 5.40, -3.15, -.85], 'cameras': (FIXED_TOP, EAST_TOP),
        'views': (('cctv_top', 'TOP_WEST', 'top_west', 'top-west', 'pickup'),
                  ('cctv_top_east', 'TOP_EAST', 'top_east', 'top-east', 'zones')),
        'cameras_text': 'TOP_WEST covers the west half (pickup); TOP_EAST covers the east half (zones)',
        'zones': {'A': [4.70, -1.35], 'B': [4.70, -2.65], 'C': [3.30, -2.00]},
        'zone_half': [.16, .40], 'slot_spacing': SLOT_SPACING_M,
        'pickup': {'center_m': [.70, -2.05], 'half_extents_m': [1.15, .85]},
        'pickup_columns_x': PICKUP_COLUMNS_X, 'pickup_rows_y': PICKUP_ROWS_Y,
        'spawn_x': SPAWN_X, 'spawn_rows_y': SPAWN_ROWS_Y,
        'observer': {'position_m': (2.2, -6.6, 4.6), 'target_m': (2.2, -2., .03)}},
    # 2026-09-25 user request: the same benchmark on a wider (north-south) floor.
    # 6.45 x 4.6 m; zones are twice as large with 0.40 m between slots, the
    # pickup grid keeps 0.6 m between columns and spaces rows 0.8 m apart.
    'zone_wide': {
        'version': 1, 'bounds': [-1.05, 5.40, -3.15, 1.45],
        'cameras': (FIXED_TOP, EAST_TOP, NORTH_TOP, NORTH_EAST_TOP),
        'views': (('cctv_top', 'TOP_SW', 'top_sw', 'top-sw', 'pickup, south'),
                  ('cctv_top_north', 'TOP_NW', 'top_nw', 'top-nw', 'pickup, north'),
                  ('cctv_top_east', 'TOP_SE', 'top_se', 'top-se', 'zones, south'),
                  ('cctv_top_north_east', 'TOP_NE', 'top_ne', 'top-ne', 'zones, north')),
        'cameras_text': ('four TOP views of one size: TOP_SW and TOP_NW cover the west half (pickup), '
                         'TOP_SE and TOP_NE the east half (zones); neighbouring views overlap slightly'),
        'zones': {'A': [4.60, .40], 'B': [4.60, -2.10], 'C': [3.00, -.85]},
        'zone_half': [.30, .70], 'slot_spacing': .40,
        'pickup': {'center_m': [.70, -.85], 'half_extents_m': [1.15, 1.95]},
        'pickup_columns_x': PICKUP_COLUMNS_X, 'pickup_rows_y': (-2.45, -1.65, -.85, -.05, .75),
        'spawn_x': SPAWN_X, 'spawn_rows_y': (-2.25, -.85, .55),
        'observer': {'position_m': (2.2, -6.2, 5.8), 'target_m': (2.2, -.85, .03)}},
}


def layout(variant):
    if variant not in VARIANTS:
        raise ValueError('unknown zone arena variant')
    return LAYOUTS[variant]


def top_views(static):
    """(camera, label, frame key, file suffix, role) for the map's TOP images."""
    return LAYOUTS[static['map_id']]['views']


def build_authored_map(variant='zone_open'):
    """Author the static map (used once to write maps/zones/<variant>.json)."""
    spec = layout(variant)
    bounds = list(spec['bounds'])
    cx, cy = (bounds[0]+bounds[1])/2, (bounds[2]+bounds[3])/2
    hx, hy = (bounds[1]-bounds[0])/2, (bounds[3]-bounds[2])/2
    walls = [
        {'id': 'wall_north', 'center_m': [cx, bounds[3]], 'half_extents_m': [hx+.025, .025]},
        {'id': 'wall_south', 'center_m': [cx, bounds[2]], 'half_extents_m': [hx+.025, .025]},
        {'id': 'wall_west', 'center_m': [bounds[0], cy], 'half_extents_m': [.025, hy]},
        {'id': 'wall_east', 'center_m': [bounds[1], cy], 'half_extents_m': [.025, hy]}]
    for wall in walls:
        wall.update(height_m=.10, kind='wall')
    zone_rgba = {'A': '.95 .45 .10 .30', 'B': '.20 .40 .95 .30', 'C': '.70 .20 .85 .30'}
    regions = {'pickup': {**copy.deepcopy(spec['pickup']), 'rgba': '.12 .36 .70 .14'}}
    slots = {}
    for zone, (x, y) in spec['zones'].items():
        regions['zone_'+zone] = {'center_m': [x, y], 'half_extents_m': list(spec['zone_half']),
                                 'rgba': zone_rgba[zone]}
        slots[zone] = [{'slot_id': f'{zone}{i+1}', 'center_m': [x, y+(i-1)*spec['slot_spacing']],
                        'half_extents_m': [.06, .06]} for i in range(SLOTS_PER_ZONE)]
    return {'schema': SCHEMA, 'map_id': 'zone_'+variant.split('_', 1)[1], 'version': spec['version'],
            'frame': 'world metres; x east, y north',
            'bounds_m': bounds, 'top_cameras': [copy.deepcopy(c) for c in spec['cameras']],
            'obstacles': walls, 'terrain': [], 'regions': regions, 'zone_slots': slots,
            'box_kinds': sorted(COLORS),
            'approach_convention': 'boxes are grasped and placed with the robot facing east (+x)'}


def authored_map(variant='zone_open'):
    """Static map the robots may know: the versioned JSON under maps/zones/."""
    layout(variant)
    value = json.loads((MAP_DIR/(variant+'.json')).read_text())
    if value != build_authored_map(variant):
        raise ValueError('zone map file differs from its authored definition')
    return value


def goal_counts(goal):
    """Validate {zone: {kind: count}}; returns a normalized copy."""
    if not isinstance(goal, dict) or not goal or not set(goal) <= set(ZONE_IDS):
        raise ValueError('goal needs zones among A, B, C')
    out = {}
    for zone, kinds in goal.items():
        if not isinstance(kinds, dict) or not kinds or not set(kinds) <= set(COLORS):
            raise ValueError('each zone needs box kinds among ' + ', '.join(sorted(COLORS)))
        for kind, count in kinds.items():
            if isinstance(count, bool) or not isinstance(count, int) or count < 1:
                raise ValueError('counts must be positive integers')
        if sum(kinds.values()) > SLOTS_PER_ZONE:
            raise ValueError(f'a zone holds at most {SLOTS_PER_ZONE} boxes')
        out[zone] = {k: int(kinds[k]) for k in sorted(kinds)}
    return {z: out[z] for z in sorted(out)}


def episode(variant='zone_open', seed=11, *, goal, extra_boxes=None):
    """Setup-only placement: enough boxes of each kind (plus optional spares)."""
    static = authored_map(variant)
    goal = goal_counts(goal)
    need = {}
    for kinds in goal.values():
        for kind, count in kinds.items():
            need[kind] = need.get(kind, 0) + count
    for kind, count in (extra_boxes or {}).items():
        if kind not in COLORS or isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError('extra boxes need known kinds and non-negative counts')
        need[kind] = need.get(kind, 0) + count
    spec = layout(variant)
    cells = [(x, y) for x in spec['pickup_columns_x'] for y in spec['pickup_rows_y']]
    total = sum(need.values())
    if total > len(cells):
        raise ValueError(f'at most {len(cells)} boxes fit the pickup grid')
    rng = random.Random(seed)
    rng.shuffle(cells)
    kinds = [k for k in sorted(need) for _ in range(need[k])]
    rng.shuffle(kinds)
    objects = {}
    for index, (kind, (x, y)) in enumerate(zip(kinds, cells)):
        oid = f'box_{index:02d}'
        objects[oid] = {'kind': kind, 'body_name': 'cargo_'+oid, 'joint_name': 'cargo_'+oid+'_free',
                        'position_m': [x, y, BOX_HALF[2]], 'half_extents_m': list(BOX_HALF)}
    order = list(ROBOTS)
    rng.shuffle(order)
    spawns = {rid: [spec['spawn_x'], y, .032355118817659255, 0.] for rid, y in zip(order, spec['spawn_rows_y'])}
    return {'schema': 'ugrp.zone_episode.v1', 'variant': variant, 'seed': seed,
            'goal': goal, 'goal_sha256': digest(goal),
            'static_map': static, 'static_map_sha256': digest(static),
            'setup_only': {'spawns': spawns, 'objects': objects, 'unexpected_obstacles': []}}


def actor_task(static, goal):
    """What every robot is told: the goal and the authored map, no setup poses."""
    return {'schema': 'ugrp.zone_task.v1', 'goal': goal_counts(goal),
            'instruction': ('Deliver boxes so that each zone ends with exactly the requested number of '
                            'boxes of each colour. Any box of the right colour counts. Boxes start in '
                            'the pickup area (west). Zones A, B and C are painted floor areas (east). '
                            'Each robot carries one box at a time.'),
            'zones': {z: {'center_m': static['regions']['zone_'+z]['center_m'],
                          'slots': [s['slot_id'] for s in static['zone_slots'][z]]} for z in ZONE_IDS},
            'box_kinds': static['box_kinds'], 'bounds_m': static['bounds_m'],
            'cameras': LAYOUTS[static['map_id']]['cameras_text']}


def _geom(world, name, center, half, height, rgba, *, collision=True, z=None):
    return ET.SubElement(world, 'geom', name='zone_'+name, type='box',
        pos=f'{center[0]} {center[1]} {height/2 if z is None else z}',
        size=f'{half[0]} {half[1]} {height/2}', rgba=rgba,
        contype='1' if collision else '0', conaffinity='3' if collision else '0',
        group='0', mass='0')


def build_zone_xml(source, config):
    """Walls, floor paint, coloured box replicas and the east CCTV; robots unchanged."""
    from sim.research_dispatch_arena import build_scene_xml
    from sim.multi_object_scene import shape_signature
    # Start from the dispatch builder so robots, cameras and weld-off stay identical.
    base = {'static_map': {**_dispatch_compatible(config['static_map'])},
            'setup_only': {'unexpected_obstacles': [], 'cargo': {'beam': [0, 0, 0], 'box': [0, 0, .016]}}}
    xml, record = build_scene_xml(source, base)
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    for node in list(world):
        if node.tag == 'geom' and (node.get('name') or '').startswith('dispatch_'):
            world.remove(node)
    floor = world.find("geom[@name='floor']")
    if floor is not None:
        floor.set('size', '8 8 .1')
    static = config['static_map']
    for wall in static['obstacles']:
        _geom(world, wall['id'], wall['center_m'], wall['half_extents_m'], wall['height_m'], '.23 .28 .33 1')
    for rid, region in static['regions'].items():
        _geom(world, rid, region['center_m'], region['half_extents_m'], .001, region['rgba'],
              collision=False, z=.0006)
    for zone, slots in static['zone_slots'].items():
        for slot in slots:
            _geom(world, 'slot_'+slot['slot_id'], slot['center_m'], slot['half_extents_m'], .001,
                  '.95 .95 .95 .35', collision=False, z=.0013)
    prototype = world.find("body[@name='dispatch_box']")
    signature = shape_signature(prototype)
    replicas = {}
    for oid, item in config['setup_only']['objects'].items():
        body = copy.deepcopy(prototype)
        for node in body.iter():
            if node.get('name'):
                node.set('name', node.get('name').replace('dispatch_box', item['body_name'], 1))
        body.set('pos', ' '.join(map(str, item['position_m'])))
        body.find("geom[@name='%s_geom']" % item['body_name']).set('rgba', COLORS[item['kind']])
        world.append(body)
        replicas[oid] = {'body_name': item['body_name'], 'kind': item['kind']}
    for body in (prototype, world.find("body[@name='team_beam']")):
        for node in body.iter('body'):
            node.set('gravcomp', '1')
        for geom in body.iter('geom'):
            geom.attrib.update(contype='0', conaffinity='0', rgba='0 0 0 0', group='5')
    top = world.find("camera[@name='cctv_top']")
    for spec in static['top_cameras'][1:]:
        extra = copy.deepcopy(top)
        extra.set('name', spec['name'])
        extra.set('pos', ' '.join(map(str, spec['position_m'])))
        world.append(extra)
    light_y = -2 if static['map_id'] == 'zone_open' else (static['bounds_m'][2]+static['bounds_m'][3])/2
    for light in world.findall('light'):
        if light.get('name') == 'dispatch_ceiling':
            light.set('pos', f"{(static['bounds_m'][0]+static['bounds_m'][1])/2} {light_y} 4.5")
    if any(eq.get('active') != 'false' for eq in root.findall('equality/weld')):
        raise ValueError('weld assistance must be OFF')
    xml = ET.tostring(root, encoding='unicode')
    return xml, {'scene_xml_sha256': hashlib.sha256(xml.encode()).hexdigest(),
                 'robot_xml_sha256': record['robot_xml_sha256'],
                 'static_map_sha256': digest(static), 'box_prototype_signature': signature,
                 'box_replicas': replicas, 'weld': 'off',
                 'appearance_change': 'box paint colour only (kinds); shape, mass, friction unchanged'}


def _dispatch_compatible(static):
    """The dispatch builder needs its own keys; zone geometry is added afterwards."""
    return {'obstacles': [], 'terrain': [], 'regions': {}, 'docks': {}}


def summary(config):
    """Readable, setup-free description for records."""
    return json.dumps({'variant': config['variant'], 'seed': config['seed'], 'goal': config['goal'],
                       'boxes': {k: sum(o['kind'] == k for o in config['setup_only']['objects'].values())
                                 for k in COLORS}}, ensure_ascii=False)
