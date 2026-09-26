"""Zone goal format v2, per-kind landing areas and the full-footprint referee.

Phase A1 of the zone team-carry work (2026-09-25).

Goal v2 is ``{zone: {kind: count}}`` where a kind is one of the four box
colours (``sim.zone_arena.COLORS``) or a catalogue kind (``sim.zone_cargo``:
can, tile, long_beam, heavy_crate, tri_frame). Colour-only goals are the old
format and go through ``sim.zone_arena.goal_counts`` unchanged (same result,
same errors). Catalogue kinds need the ``zone_wide`` arena.

Landing areas are static task information derived from the goal and the
authored map (like the old zone slots): one area per goal unit, sized from the
catalogue footprint, packed along each zone's long axis. A bay reserves the
whole team envelope (item + every carrier chassis/arm at its grasp station).

``referee_v2`` counts kinds per zone from simulator poses with the item's full
footprint (every collision part inside the zone) instead of its centre point.
It is evaluation-only output and never robot input.
"""
from __future__ import annotations

import copy
import math

from harness.zone_team_footprint import (TeamFootprint, bounds, item_polygons, polygons_inside_rect,
                                         station_offset, transform)
from sim.zone_arena import COLORS, SLOTS_PER_ZONE, ZONE_IDS, authored_map, goal_counts
from sim.zone_cargo import CATALOGUE

SCHEMA = 'ugrp.zone_goal.v2'
BOX_KINDS = tuple(sorted(COLORS))
CARGO_KINDS = ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')
ALL_KINDS = BOX_KINDS + CARGO_KINDS
# Placement tolerance around the item footprint inside its landing area.
LANDING_TOL_M = .04
# Free space between the team envelopes of neighbouring bays.
BAY_GAP_M = .06
# A bay is never shorter than this (one chassis plus planner slack).
MIN_BAY_M = .30
# Envelope distance to the inner face of a perimeter wall and to interior walls.
WALL_CLEARANCE_M = .05
PERIMETER_WALLS = frozenset({'wall_north', 'wall_south', 'wall_west', 'wall_east'})


def _interior_boxes(static):
    """Axis-aligned bounds of interior obstacles (maps with walls/doors, e.g. zone_wide_door)."""
    out = []
    for o in static.get('obstacles', ()):
        if o['id'] in PERIMETER_WALLS:
            continue
        (cx, cy), (hx, hy), yaw = o['center_m'], o['half_extents_m'], float(o.get('yaw_rad', 0.))
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        ax, ay = c*hx + s*hy, s*hx + c*hy
        out.append((o['id'], cx-ax, cx+ax, cy-ay, cy+ay))
    return out
TEAM_YAWS = (0., math.pi/2, math.pi, -math.pi/2)
_REFEREE_Z_M, _REFEREE_TILT_DEG = .012, 10.

if set(CARGO_KINDS) != set(CATALOGUE):
    raise RuntimeError('zone goal v2 kinds and the cargo catalogue differ')


def required_carriers(kind):
    if kind in COLORS:
        return 1
    if kind in CATALOGUE:
        return CATALOGUE[kind].required_carriers
    raise ValueError(f'unknown item kind: {kind}')


def formation(kind):
    """Default formation (the catalogue's first): footprints, landing layout, fixtures.
    Colour boxes: ('west',). Claims accept any formation (see formations/claim_roles)."""
    return formations(kind)[0]


def formations(kind):
    """Every catalogue formation of a kind (alternative role sets; tile: west or east)."""
    if kind in COLORS:
        return (('west',),)
    if kind in CATALOGUE:
        return tuple(tuple(f) for f in CATALOGUE[kind].formations)
    raise ValueError(f'unknown item kind: {kind}')


def claim_roles(kind):
    """Roles a robot may claim for a kind: the union of its formations, catalogue order."""
    out = []
    for f in formations(kind):
        out += [r for r in f if r not in out]
    return tuple(out)


def fills_formation(kind, roles):
    """Do these roles (one per robot) fill exactly one formation of the kind?"""
    return sorted(roles) in [sorted(f) for f in formations(kind)]


def formation_id(kind):
    return kind + '/' + '+'.join(sorted(formation(kind)))


def is_legacy_goal(goal):
    """Old colour-only format (also any malformed goal without catalogue kinds)."""
    if not isinstance(goal, dict):
        return True
    return not any(isinstance(kinds, dict) and any(k in CATALOGUE for k in kinds) for kinds in goal.values())


def goal_counts_v2(goal, variant='zone_wide'):
    """Validate a v2 goal; returns a normalized copy (sorted zones and kinds)."""
    if is_legacy_goal(goal):
        return goal_counts(goal)
    if not str(variant).startswith('zone_wide'):
        raise ValueError('catalogue kinds need a zone_wide arena')
    if not goal or not set(goal) <= set(ZONE_IDS):
        raise ValueError('goal needs zones among A, B, C')
    out = {}
    for zone, kinds in goal.items():
        if not isinstance(kinds, dict) or not kinds or not set(kinds) <= set(ALL_KINDS):
            raise ValueError('each zone needs kinds among ' + ', '.join(ALL_KINDS))
        for count in kinds.values():
            if isinstance(count, bool) or not isinstance(count, int) or count < 1:
                raise ValueError('counts must be positive integers')
        colours = sum(v for k, v in kinds.items() if k in COLORS)
        if colours > SLOTS_PER_ZONE:
            raise ValueError(f'a zone holds at most {SLOTS_PER_ZONE} colour boxes')
        out[zone] = {k: int(kinds[k]) for k in sorted(kinds)}
    out = {z: out[z] for z in sorted(out)}
    landing_layout(out, variant)          # raises when the zone cannot hold the kinds
    return out


def goal_units(goal):
    """[(zone, kind)] once per requested item, in a fixed order."""
    return [(zone, kind) for zone in sorted(goal) for kind in sorted(goal[zone]) for _ in range(goal[zone][kind])]


# ---------------------------------------------------------------------------
# Landing areas

def _landing_yaw_candidates(kind):
    if required_carriers(kind) == 1:
        # Solo items are placed with the robot facing east (zone teacher convention).
        role = formation(kind)[0]
        return (-station_offset(kind, role)[2],)
    return TEAM_YAWS


def _bay(kind, yaw, zone_half):
    """Envelope and item extents (item origin at 0) for one landing yaw; None if the item is too wide."""
    team = TeamFootprint(kind, formation(kind), margin=0.)
    env = bounds([transform(p, (0., 0., yaw)) for p in team.parts])
    item = bounds([transform(p, (0., 0., yaw)) for p in item_polygons(kind)])
    long_axis = 1 if zone_half[1] >= zone_half[0] else 0
    short_half = zone_half[1-long_axis]
    across = (item[1]-item[0], item[3]-item[2])[1-long_axis]
    if across/2 + LANDING_TOL_M > short_half:
        return None
    along = (env[1]-env[0], env[3]-env[2])[long_axis]
    return {'yaw': yaw, 'envelope': env, 'item': item, 'length': max(along + BAY_GAP_M, MIN_BAY_M),
            'long_axis': long_axis}


def landing_spec(kind, zone_half):
    """Chosen landing yaw and bay for a kind in a zone of the given half extents."""
    options = [b for b in (_bay(kind, yaw, zone_half) for yaw in _landing_yaw_candidates(kind)) if b]
    if not options:
        raise ValueError(f'{kind} does not fit across a zone')
    return min(options, key=lambda b: (round(b['length'], 9), TEAM_YAWS.index(b['yaw'])
                                       if b['yaw'] in TEAM_YAWS else 0))


def landing_layout(goal, variant='zone_wide', static_map=None):
    """{zone: [landing area]} for a normalized goal; raises if a zone cannot hold its kinds.

    Order inside a zone: team items first (more carriers first), then solo
    items, by kind name. Bays are centred on the zone's long axis.
    """
    static = static_map or authored_map(variant)
    x0, x1, y0, y1 = static['bounds_m']
    walls = _interior_boxes(static)
    inner = (x0 + WALL_CLEARANCE_M, x1 - WALL_CLEARANCE_M, y0 + WALL_CLEARANCE_M, y1 - WALL_CLEARANCE_M)
    out = {}
    for zone in sorted(goal):
        region = static['regions']['zone_'+zone]
        (cx, cy), half = region['center_m'], region['half_extents_m']
        units = sorted(((kind, i) for kind, n in goal[zone].items() for i in range(n)),
                       key=lambda u: (-required_carriers(u[0]), u[0], u[1]))
        bays = [(kind, i, landing_spec(kind, half)) for kind, i in units]
        axis = bays[0][2]['long_axis'] if bays else 1
        total = sum(b['length'] for _, _, b in bays)
        if total > 2*half[axis] + 1e-9:
            raise ValueError(f'zone {zone} cannot hold {goal[zone]}: bays need {total:.2f} m, '
                             f'zone is {2*half[axis]:.2f} m long')
        cursor = (cy if axis == 1 else cx) - total/2
        areas = []
        for kind, i, b in bays:
            env, item = b['envelope'], b['item']
            if axis == 1:
                oy = cursor + (b['length'] - (env[3]-env[2]))/2 - env[2]
                ox = cx - (item[0]+item[1])/2
            else:
                ox = cursor + (b['length'] - (env[1]-env[0]))/2 - env[0]
                oy = cy - (item[2]+item[3])/2
            cursor += b['length']
            pose = (ox, oy, b['yaw'])
            env_w = (env[0]+ox, env[1]+ox, env[2]+oy, env[3]+oy)
            if not (inner[0] <= env_w[0] and env_w[1] <= inner[1] and inner[2] <= env_w[2] and env_w[3] <= inner[3]):
                raise ValueError(f'zone {zone}: the {kind} team envelope leaves the arena')
            for wid, wx0, wx1, wy0, wy1 in walls:
                if (env_w[0] < wx1 + WALL_CLEARANCE_M and wx0 - WALL_CLEARANCE_M < env_w[1]
                        and env_w[2] < wy1 + WALL_CLEARANCE_M and wy0 - WALL_CLEARANCE_M < env_w[3]):
                    raise ValueError(f'zone {zone}: the {kind} team envelope meets interior wall {wid}')
            item_w = (item[0]+ox, item[1]+ox, item[2]+oy, item[3]+oy)
            landing_c = ((item_w[0]+item_w[1])/2, (item_w[2]+item_w[3])/2)
            landing_h = ((item_w[1]-item_w[0])/2 + LANDING_TOL_M, (item_w[3]-item_w[2])/2 + LANDING_TOL_M)
            if not polygons_inside_rect([[(landing_c[0]+sx*landing_h[0], landing_c[1]+sy*landing_h[1])
                                          for sx in (-1, 1) for sy in (-1, 1)]], (cx, cy), half):
                raise ValueError(f'zone {zone}: the {kind} landing area leaves the zone')
            stations = {}
            for role in formation(kind):
                sx, sy, syaw = station_offset(kind, role)
                wx, wy = transform([(sx, sy)], pose)[0]
                stations[role] = [round(wx, 6), round(wy, 6), round(math.atan2(math.sin(syaw+pose[2]),
                                                                                 math.cos(syaw+pose[2])), 6)]
            areas.append({'area_id': f'{zone}-{kind}-{i+1}', 'kind': kind,
                          'required_carriers': required_carriers(kind),
                          'item_pose': [round(ox, 6), round(oy, 6), round(b['yaw'], 6)],
                          'landing_center_m': [round(v, 6) for v in landing_c],
                          'landing_half_extents_m': [round(v, 6) for v in landing_h],
                          'envelope_m': [round(v, 6) for v in env_w],
                          'bay_length_m': round(b['length'], 6), 'stations': stations})
        out[zone] = areas
    return out


def landing_capacity_table(variant='zone_wide'):
    """Bay length per kind and how many of one kind a zone of this arena holds (record)."""
    static = authored_map(variant)
    half = static['regions']['zone_A']['half_extents_m']
    axis_len = 2*max(half)
    table = {}
    for kind in ALL_KINDS:
        b = landing_spec(kind, half)
        table[kind] = {'required_carriers': required_carriers(kind), 'landing_yaw_deg': round(math.degrees(b['yaw']), 1),
                       'bay_length_m': round(b['length'], 4),
                       'max_per_zone_by_length': int((axis_len + 1e-9)//b['length'])}
    return table


# ---------------------------------------------------------------------------
# Referee v2 (evaluation only)

def referee_v2(goal, static_map, items, *, z_tol_m=_REFEREE_Z_M, tilt_tol_deg=_REFEREE_TILT_DEG):
    """Exact final counts per zone from simulator poses, full footprint.

    items: {item_id: {'kind', 'pose': [x, y, yaw], 'min_z_m', 'tilt_deg', 'held'}}
    (``held``: any finger still touches it; computed by the caller from the
    simulator). An item counts for a zone only when it rests on the floor
    (lowest point <= z_tol_m, tilt <= tilt_tol_deg, not held) and every
    collision part lies inside the zone rectangle. ``centre_point_counts`` is
    the old centre-point rule, for comparison only.
    """
    counts, centre, per_item = {}, {}, {}
    zones = {z: static_map['regions']['zone_'+z] for z in ZONE_IDS if 'zone_'+z in static_map['regions']}
    for iid, item in sorted(items.items()):
        kind, pose = item['kind'], tuple(float(v) for v in item['pose'])
        polys = [transform(p, pose) for p in item_polygons(kind)]
        resting = (float(item['min_z_m']) <= z_tol_m and float(item['tilt_deg']) <= tilt_tol_deg
                   and not item.get('held', False))
        status, where = 'outside', None
        for zone, region in zones.items():
            (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
            if abs(pose[0]-cx) <= hx and abs(pose[1]-cy) <= hy and resting:
                centre.setdefault(zone, {}).setdefault(kind, 0)
                centre[zone][kind] += 1
            if polygons_inside_rect(polys, (cx, cy), (hx, hy)):
                status, where = 'inside', zone
                break
            if any(abs(x-cx) <= hx and abs(y-cy) <= hy for p in polys for x, y in p):
                status, where = 'straddling', zone
        if status == 'inside' and resting:
            counts.setdefault(where, {}).setdefault(kind, 0)
            counts[where][kind] += 1
        per_item[iid] = {'kind': kind, 'zone': where, 'footprint': status, 'resting': resting}
    per_zone = {zone: counts.get(zone, {}) == kinds for zone, kinds in goal.items()}
    return {'schema': 'ugrp.zone_referee.v2', 'zone_counts': counts, 'per_zone_exact': per_zone,
            'goal_met': all(per_zone.values()), 'items': per_item, 'centre_point_counts': centre,
            'rule': {'footprint': 'every collision part inside the zone rectangle', 'z_tol_m': z_tol_m,
                     'tilt_tol_deg': tilt_tol_deg, 'held_items_do_not_count': True},
            'scope': 'referee-only simulator poses after control ends; never robot input'}


def task_static_info(goal, variant='zone_wide'):
    """Static task information robots may be told in A2: kinds, carriers, roles, landing areas.

    Built only from the catalogue, the goal and the authored map (no setup poses).
    """
    goal = goal_counts_v2(goal, variant)
    kinds = sorted({k for z in goal.values() for k in z})
    return {'schema': SCHEMA, 'goal': copy.deepcopy(goal),
            'kinds': {k: {'required_carriers': required_carriers(k), 'roles': list(claim_roles(k)),
                          'formations': [list(f) for f in formations(k)],
                          'formation_id': formation_id(k)} for k in kinds},
            'landing_areas': landing_layout(goal, variant) if not is_legacy_goal(goal) else None}


__all__ = ['SCHEMA', 'BOX_KINDS', 'CARGO_KINDS', 'ALL_KINDS', 'required_carriers', 'formation', 'formations',
           'claim_roles', 'fills_formation', 'formation_id',
           'is_legacy_goal', 'goal_counts_v2', 'goal_units', 'landing_spec', 'landing_layout',
           'landing_capacity_table', 'referee_v2', 'task_static_info']
