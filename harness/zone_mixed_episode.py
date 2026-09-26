"""Mixed-cargo zone episodes for goal v2 (zone team A2, 2026-09-25). Setup only.

A v2 goal names colour boxes and catalogue cargo (``harness.zone_goal_v2``).
The colour part goes through the unchanged ``sim.zone_arena.episode`` (same
boxes, same spawns as a colour-only goal with that part); the cargo items are
placed here, deterministically per seed, on the pickup side where every grasp
station and the approach behind it are free. Nothing here reaches a robot:
cargo poses are setup-only like box poses (the TOP RGB detector finds them).

The scene is ``sim.zone_cargo_scene.CargoZoneScene`` (PR #164 cargo path).
"""
from __future__ import annotations

import copy
import math
import random

from harness.static_keepouts import keepout_rects, polygons_overlap
from harness.zone_goal_v2 import CARGO_KINDS, formation, goal_counts_v2, is_legacy_goal, required_carriers
from harness.zone_team_footprint import TeamFootprint, circle, item_polygons, transform
from sim.zone_arena import BOX_HALF, COLORS, episode, layout
from sim.zone_cargo import CATALOGUE

SCHEMA = 'ugrp.zone_mixed_episode.v1'
# Candidate item centres on the pickup side (x, y step) and the extra margin
# kept around a team footprint (robots reach the stations from behind).
SITE_STEP_M = (.30, .40)
APPROACH_BACKOFF_M = .25
FOOTPRINT_CLEAR_M = .10
BOX_CLEAR_M = .08
SPAWN_CLEAR_M = .30
# Passage approach lanes kept free of cargo (along the passage axis / across it).
LANE_LENGTH_M, LANE_SIDE_M = 1.20, .20
# Item yaws tried in order (setup only): beams along y fit between pickup rows.
SITE_YAWS = {'long_beam': (math.pi/2, 0.), 'heavy_crate': (0., math.pi/2), 'tri_frame': (0., math.pi/2),
             'can': (0.,), 'tile': (0.,)}
# Larger teams first, so the big footprints get room.
PLACE_ORDER = ('tri_frame', 'long_beam', 'heavy_crate', 'tile', 'can')


def split_goal(goal):
    """({zone: {colour: n}}, {cargo kind: n}) of a normalized v2 goal."""
    boxes, cargo = {}, {}
    for zone, kinds in goal.items():
        for kind, n in kinds.items():
            if kind in COLORS:
                boxes.setdefault(zone, {})[kind] = n
            else:
                cargo[kind] = cargo.get(kind, 0) + n
    return boxes, cargo


def _box_polys(objects):
    return [circle(o['position_m'][0], o['position_m'][1], math.hypot(*BOX_HALF[:2]) + BOX_CLEAR_M)
            for o in objects.values()]


def _occupied(kind, pose):
    """Team footprint (all roles, grown) plus a disc behind every station."""
    team = TeamFootprint(kind, formation(kind), margin=FOOTPRINT_CLEAR_M)
    polys = team.at(pose)
    for role, (sx, sy, syaw) in team.stations.items():
        bx, by = sx - APPROACH_BACKOFF_M*math.cos(syaw), sy - APPROACH_BACKOFF_M*math.sin(syaw)
        (wx, wy), = transform([(bx, by)], pose)
        polys.append(circle(wx, wy, .17 + .03))
    return polys


def _inside(polys, rect):
    x0, x1, y0, y1 = rect
    return all(x0 <= x <= x1 and y0 <= y <= y1 for p in polys for x, y in p)


def cargo_sites(variant, seed, counts, objects, spawns, static):
    """Deterministic setup poses for {kind: n} cargo items, clear of boxes and each other."""
    spec = layout(variant)
    (pcx, pcy), (phx, phy) = spec['pickup']['center_m'], spec['pickup']['half_extents_m']
    x0, x1, y0, y1 = static['bounds_m']
    # Pickup side of the map: inside the arena, west of any interior wall.
    walls = keepout_rects(static)
    east = min((cx - hx for cx, _, hx, _, _ in walls), default=x1) - .05
    area = (x0 + .05, min(x1 - .05, east), y0 + .05, y1 - .05)
    xs = [pcx - phx + .25 + i*SITE_STEP_M[0] for i in range(int((2*phx - .5)/SITE_STEP_M[0]) + 1)]
    ys = [pcy - phy + .25 + j*SITE_STEP_M[1] for j in range(int((2*phy - .5)/SITE_STEP_M[1]) + 1)]
    cells = [(round(x, 4), round(y, 4)) for x in xs for y in ys]
    rng = random.Random(seed*7919 + 17)
    rng.shuffle(cells)
    blocked = _box_polys(objects)
    blocked += [circle(s[0], s[1], .17 + SPAWN_CLEAR_M) for s in spawns.values()]
    wall_polys = [[(cx + math.cos(a)*u - math.sin(a)*v, cy + math.sin(a)*u + math.cos(a)*v)
                   for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))] for cx, cy, hx, hy, a in walls]
    # Keep the approach lanes of every passage (both sides of a door or
    # corridor mouth, door width + robot) free of cargo: a team must line up
    # with the opening (zone_wide_door seed 11 put a beam across the door lane).
    for p in static.get('passages', ()):
        if p.get('kind') == 'passing_bay':
            continue
        (cx, cy), (hx, hy) = p['center_m'], p['half_extents_m']
        if p['axis'] == 'x':
            lane = (cx, cy, hx + LANE_LENGTH_M, hy + LANE_SIDE_M, 0.)
        else:
            lane = (cx, cy, hx + LANE_SIDE_M, hy + LANE_LENGTH_M, 0.)
        wall_polys.append([(lane[0] + sx*lane[2], lane[1] + sy*lane[3]) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
    placed, items = [], []
    for kind in PLACE_ORDER:
        for i in range(counts.get(kind, 0)):
            for (x, y) in cells:
                pose = None
                for yaw in SITE_YAWS[kind]:
                    polys = _occupied(kind, (x, y, yaw))
                    if not _inside(polys, area):
                        continue
                    if any(polygons_overlap(p, q) for p in polys for q in blocked + placed + wall_polys):
                        continue
                    pose = (x, y, yaw)
                    break
                if pose:
                    placed += _occupied(kind, pose)
                    items.append({'item_id': f'{kind}_{i}', 'kind': kind,
                                  'pose': [pose[0], pose[1], round(pose[2], 6)]})
                    break
            else:
                raise ValueError(f'no free pickup site for {kind} #{i+1} (seed {seed}, {variant})')
    return items


def mixed_episode(variant, seed, *, goal, extra_boxes=None, extra_cargo=None, colour_only_ok=False):
    """Setup-only episode for a v2 goal: boxes via sim.zone_arena.episode + placed cargo.

    extra_cargo: spare cargo items per kind (distractors), like extra_boxes.
    colour_only_ok: a colour-only goal on the v2 protocol (explicit --protocol v2);
    its boxes are exactly those of sim.zone_arena.episode.
    """
    goal = goal_counts_v2(goal, variant)
    if is_legacy_goal(goal) and not colour_only_ok:
        raise ValueError('colour-only goals use sim.zone_arena.episode')
    box_goal, cargo = split_goal(goal)
    if not box_goal:
        # The zone scene path always places the boxes of a colour goal (an
        # empty one falls back to the default goal's boxes); A2 needs >= 1 box.
        raise ValueError('mixed goals need at least one colour box in this A2 scene path')
    for kind, n in (extra_cargo or {}).items():
        if kind not in CARGO_KINDS or isinstance(n, bool) or not isinstance(n, int) or n < 0:
            raise ValueError('extra cargo needs catalogue kinds and non-negative counts')
        cargo[kind] = cargo.get(kind, 0) + n
    base = episode(variant, seed, goal=box_goal, extra_boxes=extra_boxes)
    items = cargo_sites(variant, seed, cargo, base['setup_only']['objects'], base['setup_only']['spawns'],
                        base['static_map'])
    config = copy.deepcopy(base)
    config.update({'schema': SCHEMA, 'goal': goal, 'box_goal': box_goal,
                   'extra_boxes': extra_boxes or {}, 'extra_cargo': extra_cargo or {},
                   'cargo_items': items})
    return config


def scene_for(config, contact_profile='cargo_noslip_v1'):
    """The standard cargo zone scene for a mixed episode (checked against it)."""
    from sim.zone_cargo_scene import CargoZoneScene
    scene = CargoZoneScene.from_cargo_config(config['variant'], config['seed'], cargo=config['cargo_items'],
                                             goal=config['box_goal'], extra_boxes=config['extra_boxes'],
                                             contact_profile=contact_profile)
    if (scene.config['setup_only'] != config['setup_only'] or scene.config['static_map'] != config['static_map']
            or scene.config['cargo_set']['items'] != config['cargo_items']):
        raise ValueError('mixed episode differs from the selected cargo scene')
    return scene


def item_table(config):
    """Every physical item (boxes + cargo): {item_id: {'kind', 'body_name', 'solo'}}. Teacher/referee only."""
    out = {}
    for oid, o in config['setup_only']['objects'].items():
        out[oid] = {'kind': o['kind'], 'body_name': o['body_name'], 'carriers': 1}
    for item in config.get('cargo_items', ()):
        out[item['item_id']] = {'kind': item['kind'], 'body_name': 'cargo_' + item['item_id'],
                                'carriers': required_carriers(item['kind'])}
    return out


__all__ = ['SCHEMA', 'split_goal', 'cargo_sites', 'mixed_episode', 'scene_for', 'item_table',
           'item_polygons', 'CATALOGUE']
