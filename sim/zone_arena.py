"""Zone-goal delivery arena: several coloured boxes, three goal zones, two TOPs.

A coordination benchmark scene. It widens the dispatch laboratory eastwards and
keeps the approved TOP camera; CCTVs of the identical specification cover the
rest (retired zone_open: one east; zone_wide: east plus a northern row of two). Box replicas keep the production cyan box shape, mass and
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
VARIANTS = ('zone_open', 'zone_wide', 'zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor')
# 2026-09-25 user request ("옛날의 작은 맵은 없애주라"): the small zone_open floor
# is retired. Its geometry and map file stay byte-identical so the Z1-Z3
# records and replays can be reproduced; new runs refuse it unless the runner
# is asked explicitly (scripts/run_zone_dispatch.py --allow-retired-variant).
RETIRED_VARIANTS = ('zone_open',)
DEFAULT_VARIANT = 'zone_wide'


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
# Per-variant geometry. zone_open (retired) is map v2 exactly as used by Z1-Z3.
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


# 2026-09-25 hard-route variants: the zone_wide floor, TOPs, pickup grid,
# spawns and zones unchanged, plus interior walls with narrow passages where
# robots must take turns. A wall at x = 2.20 (on the 0.05 m planner grid and
# inside the 0.13 m strip both TOP columns see) separates pickup (west, boxes
# up to x = 1.60) from the zones (east, C slots approached from x = 2.77).
# Walls match the perimeter: 0.05 m thick, 0.10 m high, same paint, so the
# fixed TOPs still see the floor next to them (tests) and the robot camera
# (0.21 m high) sees over them.
DIVIDER_X = 2.20
WALL_HALF = .025
# One-robot door: the loaded robot's turning envelope (front reach 0.195 m,
# diameter 0.39 m) plus 0.11 m. The teacher plans a disc of 0.21 m when
# carrying, so a loaded robot has a 0.08 m band on the door axis and two robots
# (peer clearance 0.14 m) can never be in it together.
NARROW_DOOR_M = .50
# Two-lane door: two loaded robots side by side (0.21 + 0.35 + 0.21 m) plus margin.
WIDE_DOOR_M = 1.00


def _wall(wid, x0, x1, y0, y1):
    return {'id': wid, 'center_m': [round((x0+x1)/2, 4), round((y0+y1)/2, 4)],
            'half_extents_m': [round((x1-x0)/2, 4), round((y1-y0)/2, 4)]}


def _divider(doors):
    """N-S wall segments at DIVIDER_X leaving the given (centre_y, width) openings."""
    edges, y = [], -3.15
    for cy, width in sorted(doors):
        edges.append((y, cy-width/2))
        y = cy+width/2
    edges.append((y, 1.45))
    return [_wall(f'wall_divider_{i+1}', DIVIDER_X-WALL_HALF, DIVIDER_X+WALL_HALF, lo, hi)
            for i, (lo, hi) in enumerate((lo, hi) for lo, hi in edges if hi-lo > 2*WALL_HALF)]


def _door(pid, cy, width, lanes):
    return {'id': pid, 'kind': 'door', 'center_m': [DIVIDER_X, cy], 'half_extents_m': [WALL_HALF, width/2],
            'width_m': width, 'lanes': lanes, 'axis': 'x', 'connects': ['pickup side (west)', 'zone side (east)']}


# Corridor: the divider stops at the corridor wall y = 0.90; the only way east
# is a 0.50 m single lane along the north wall (y 0.925..1.425) from x = 2.20
# to x = 3.90, with one passing bay south of it (x 2.80..3.40, down to y 0.35).
CORRIDOR = {'x0': DIVIDER_X, 'x1': 3.90, 'wall_y': .90, 'bay_x': (2.80, 3.40), 'bay_y0': .35}


def _corridor_walls():
    c = CORRIDOR
    (bx0, bx1), wy, by0 = c['bay_x'], c['wall_y'], c['bay_y0']
    return [_wall('wall_divider_1', DIVIDER_X-WALL_HALF, DIVIDER_X+WALL_HALF, -3.15, wy+WALL_HALF),
            _wall('wall_corridor_1', DIVIDER_X-WALL_HALF, bx0+WALL_HALF, wy-WALL_HALF, wy+WALL_HALF),
            _wall('wall_corridor_2', bx1-WALL_HALF, c['x1']+WALL_HALF, wy-WALL_HALF, wy+WALL_HALF),
            _wall('wall_bay_west', bx0-WALL_HALF, bx0+WALL_HALF, by0-WALL_HALF, wy),
            _wall('wall_bay_east', bx1-WALL_HALF, bx1+WALL_HALF, by0-WALL_HALF, wy),
            _wall('wall_bay_south', bx0-WALL_HALF, bx1+WALL_HALF, by0-WALL_HALF, by0+WALL_HALF)]


def _corridor_passages():
    c = CORRIDOR
    lane_lo, lane_hi = c['wall_y']+WALL_HALF, 1.45-WALL_HALF
    (bx0, bx1), by0 = c['bay_x'], c['bay_y0']
    return [{'id': 'corridor_1', 'kind': 'corridor', 'center_m': [round((c['x0']+c['x1']+WALL_HALF)/2, 4),
             round((lane_lo+lane_hi)/2, 4)], 'half_extents_m': [round((c['x1']+WALL_HALF-c['x0'])/2, 4),
             round((lane_hi-lane_lo)/2, 4)], 'width_m': round(lane_hi-lane_lo, 4), 'lanes': 1, 'axis': 'x',
             'connects': ['pickup side (west)', 'zone side (east)']},
            {'id': 'bay_1', 'kind': 'passing_bay', 'center_m': [round((bx0+bx1)/2, 4),
             round((by0+WALL_HALF+c['wall_y']-WALL_HALF)/2, 4)],
             'half_extents_m': [round((bx1-bx0)/2-WALL_HALF, 4), round((c['wall_y']-by0)/2-WALL_HALF, 4)],
             'axis': 'y', 'opens_to': 'corridor_1',
             'note': 'one robot can wait here while another passes in the corridor'}]


for _name, _walls, _passages, _text in (
        ('zone_wide_door', _divider([(.05, NARROW_DOOR_M)]), [_door('door_1', .05, NARROW_DOOR_M, 1)],
         'one narrow door'),
        ('zone_wide_two_doors', _divider([(.05, NARROW_DOOR_M), (-2.625, WIDE_DOOR_M)]),
         [_door('door_narrow', .05, NARROW_DOOR_M, 1), _door('door_wide', -2.625, WIDE_DOOR_M, 2)],
         'a narrow short door and a wide door at the south end'),
        ('zone_wide_corridor', _corridor_walls(), _corridor_passages(), 'a single-lane corridor with a bay')):
    LAYOUTS[_name] = {**copy.deepcopy(LAYOUTS['zone_wide']), 'version': 1, 'interior_walls': _walls,
                      'passages': _passages, 'route_text': _text}


# 2026-09-26 environment v3 (user request "벽이 너무 낮은 거 같아, 벽 너머가 보여야
# 하나"): a versioned wall-height profile applied on top of an authored base map.
# ``walls_v1`` is the 0.10 m wall every base map above is authored with (their
# JSON files and every map up to *_tags_v2 stay byte-identical). ``walls_v3``
# raises every wall (perimeter and interior) to 0.40 m; footprints, doors,
# passages, zones, TOP cameras and spawns are unchanged. The height is the
# smallest 0.05 m step above both (a) the highest wrist-camera position any
# commandable arm posture reaches plus 0.05 m (0.321 m: servo 3/4/5 PWM
# 500..2500 through the simulator's own pulse map and MuJoCo joint ranges, arm
# straight up; the named work postures reach at most 0.239 m) and (b) the highest
# robot geom plus 0.03 m (0.367 m, finger pad with the arm straight up). So no
# wrist camera sees over a wall into another room (only through doors and
# corridors) and no raised arm or held box shows above one.
# Evidence: experiments/2026-09-26-zone-env-v3/camera_height.json.
WALL_PROFILES = {
    'walls_v1': {'version': 1, 'height_m': .10,
                 'scope': 'authored base maps and every tagged map up to *_tags_v2 (unchanged)'},
    'walls_v3': {'version': 3, 'height_m': .40,
                 'rule': ('smallest 0.05 m step >= max(kinematic max wrist-camera z + 0.05 m, '
                          'highest robot geom z + 0.03 m)'),
                 'kinematic_max_camera_z_m': .321, 'max_work_posture_camera_z_m': .239,
                 'robot_top_max_z_m': .367,
                 'evidence': 'experiments/2026-09-26-zone-env-v3/camera_height.json'},
}


def wall_profile_record(profile_id):
    """The profile as recorded in a map file (id, parameters, parameter hash)."""
    if profile_id not in WALL_PROFILES:
        raise ValueError(f'unknown wall profile: {profile_id}')
    value = {'id': profile_id, **copy.deepcopy(WALL_PROFILES[profile_id])}
    value['sha256'] = digest(value)
    return value


def apply_wall_profile(static, profile_id):
    """Copy of a static map with every wall at the profile height (footprints unchanged)."""
    record = wall_profile_record(profile_id)
    value = copy.deepcopy(static)
    for obstacle in value['obstacles']:
        if obstacle.get('kind') == 'wall':
            obstacle['height_m'] = record['height_m']
    value['wall_profile'] = record
    return value


def layout(variant):
    if variant not in VARIANTS:
        raise ValueError('unknown zone arena variant')
    return LAYOUTS[variant]


def top_views(static):
    """(camera, label, frame key, file suffix, role) for the map's TOP images."""
    return LAYOUTS[static['map_id']]['views']


def build_authored_map(variant=DEFAULT_VARIANT):
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
    walls += copy.deepcopy(spec.get('interior_walls', []))
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
    value = {'schema': SCHEMA, 'map_id': 'zone_'+variant.split('_', 1)[1], 'version': spec['version'],
             'frame': 'world metres; x east, y north',
             'bounds_m': bounds, 'top_cameras': [copy.deepcopy(c) for c in spec['cameras']],
             'obstacles': walls, 'terrain': [], 'regions': regions, 'zone_slots': slots,
             'box_kinds': sorted(COLORS),
             'approach_convention': 'boxes are grasped and placed with the robot facing east (+x)'}
    if 'passages' in spec:
        value['passages'] = copy.deepcopy(spec['passages'])
    return value


def static_map_text(static):
    """Walls and passages of the authored map as prompt text (static, no live state).

    None for maps with only the perimeter (their prompts stay unchanged).
    """
    interior = [o for o in static['obstacles'] if not o['id'].startswith(('wall_north', 'wall_south',
                                                                          'wall_west', 'wall_east'))]
    if not interior:
        return None
    def seg(o):
        (x, y), (hx, hy) = o['center_m'], o['half_extents_m']
        a, b = ((x-hx, y), (x+hx, y)) if hx >= hy else ((x, y-hy), (x, y+hy))
        return f"{o['id']} ({a[0]:.2f}, {a[1]:.2f})-({b[0]:.2f}, {b[1]:.2f})"
    # Wall height from the map itself (0.10 m on every authored map, so their text is unchanged;
    # 0.40 m under wall profile walls_v3).
    heights = sorted({float(o.get('height_m', .10)) for o in interior})
    high = '/'.join(f'{h:.2f}' for h in heights) + ' m high'
    lines = [f'Fixed interior walls ({high}; x east, y north, metres): ' + '; '.join(seg(o) for o in interior) + '.']
    for p in static.get('passages', []):
        (x, y), (hx, hy) = p['center_m'], p['half_extents_m']
        if p['kind'] == 'passing_bay':
            lines.append(f"{p['id']}: passing bay x {x-hx:.2f}..{x+hx:.2f}, y {y-hy:.2f}..{y+hy:.2f}, "
                         f"opening onto {p['opens_to']}; one robot can wait there while another passes.")
            continue
        span = (f'x {x-hx:.2f}..{x+hx:.2f}, y {y-hy:.2f}..{y+hy:.2f}')
        lanes = ('one robot at a time: robots cannot pass each other inside it'
                 if p['lanes'] == 1 else f"{p['lanes']} robots side by side")
        lines.append(f"{p['id']} ({p['kind']}, {p['width_m']:.2f} m wide, {span}) connects "
                     f"{' and '.join(p['connects'])}; {lanes}.")
    lines.append('The pickup area and the zones are on opposite sides of these walls; every trip passes a door '
                 'or corridor listed here.')
    return ' '.join(lines)


def authored_map(variant=DEFAULT_VARIANT):
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


def episode(variant=DEFAULT_VARIANT, seed=11, *, goal, extra_boxes=None):
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
    task = {'schema': 'ugrp.zone_task.v1', 'goal': goal_counts(goal),
            'instruction': ('Deliver boxes so that each zone ends with exactly the requested number of '
                            'boxes of each colour. Any box of the right colour counts. Boxes start in '
                            'the pickup area (west). Zones A, B and C are painted floor areas (east). '
                            'Each robot carries one box at a time.'),
            'zones': {z: {'center_m': static['regions']['zone_'+z]['center_m'],
                          'slots': [s['slot_id'] for s in static['zone_slots'][z]]} for z in ZONE_IDS},
            'box_kinds': static['box_kinds'], 'bounds_m': static['bounds_m'],
            'cameras': LAYOUTS[static['map_id']]['cameras_text']}
    text = static_map_text(static)
    if text:
        # Static map only (walls, doors): allowed robot knowledge; no live state.
        task['static_map_text'] = text
    return task


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
        geom = _geom(world, wall['id'], wall['center_m'], wall['half_extents_m'], wall['height_m'], '.23 .28 .33 1')
        if wall.get('yaw_rad'):
            geom.set('euler', f"0 0 {wall['yaw_rad']}")
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
