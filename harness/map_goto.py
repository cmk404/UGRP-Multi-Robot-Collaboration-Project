"""Model-chosen destinations reached by A* over the authored static map.

The robot LLM chooses WHERE to go: an authored place name or a map
coordinate.  This module chooses HOW: an 8-connected A* path whose every cell
keeps the declared fixed-heading body envelope clear of authored obstacles,
unvalidated terrain and caller-supplied RGB obstacle estimates.

It never reads simulator state.  The start position must be an allowed RGB
estimate supplied by the caller.  A planned path is not physical passage proof:
the executor still follows it with fresh RGB and fails closed.
"""
from __future__ import annotations

import copy
import hashlib
import heapq
import json
import math
from typing import Any, Iterable, Mapping, Sequence

import cv2
import numpy as np

SCHEMA = 'ugrp.map_goto.v1'
GRID_M = .025
MARGIN_M = .02
# Nominal plane of the lifted cyan cargo feature seen from the fixed TOP camera.
# The same authored 9 cm plane is used for the carried beam. Calibration value,
# never a live height measurement.
CARRY_PLANE_M = .09
# Fixed east-facing mecanum carrier, relative to the tracked cargo feature.
# The chassis centre sits about 0.16 m behind (west of) the held box and the
# chassis plus wheels span about +-0.13 m; 0.32 m lateral width matches the
# existing solo corridor check. Conservative authored envelope, not a pose.
SOLO_CARRY_ENVELOPE = {'x_m': [-.30, .06], 'y_m': [-.16, .16]}
# Folded-arm chassis relative to its RGB wheel-geometry centre.
UNLOADED_ENVELOPE = {'x_m': [-.15, .15], 'y_m': [-.15, .15]}
# Demonstrated loaded pair: both carriers stand behind (west of) the shaft and
# span its length. Relative to the beam centre; used only as keep-out space.
BEAM_TEAM_ENVELOPE = {'x_m': [-.36, .06], 'y_m': [-.59, .59]}
BEAM_PLACEMENT_OFFSET_M = -.06  # the pair leaves room toward the box slot


def digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _finite_pair(value: Any) -> bool:
    if isinstance(value, np.ndarray):
        value = value.tolist()
    return (isinstance(value, (list, tuple)) and len(value) == 2
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in value))


def parse_destination(value: Any) -> dict:
    """Accept a place name or {"xy_m": [x, y]}; reject anything else."""
    if isinstance(value, str) and value:
        return {'place': value}
    if isinstance(value, Mapping) and set(value) == {'place'} and isinstance(value['place'], str) and value['place']:
        return {'place': value['place']}
    if isinstance(value, Mapping) and set(value) == {'xy_m'} and _finite_pair(value['xy_m']):
        return {'xy_m': [float(value['xy_m'][0]), float(value['xy_m'][1])]}
    raise ValueError('destination must be an authored place name or {"xy_m":[x,y]}')


def places(static_map: Mapping[str, Any]) -> dict:
    """Named authored destinations: every region plus every dock slot."""
    result = {name: {'center_m': list(map(float, region['center_m'])),
                     'half_extents_m': list(map(float, region['half_extents_m'])), 'kind': 'region'}
              for name, region in static_map.get('regions', {}).items()}
    for dock, value in static_map.get('docks', {}).items():
        for obj, slot in value.get('slots', {}).items():
            result[dock + '.' + obj] = {'center_m': list(map(float, slot['center_m'])),
                                        'half_extents_m': list(map(float, slot['half_extents_m'])),
                                        'kind': 'dock_slot'}
    return result


def interior_bounds(static_map: Mapping[str, Any]) -> list[float]:
    """Map bounds are wall centres; return the free side of the boundary walls."""
    xmin, xmax, ymin, ymax = map(float, static_map['bounds_m'])
    walls = {o['id']: o for o in static_map.get('obstacles', [])}
    def face(name, axis, sign, default):
        wall = walls.get(name)
        if wall is None:
            return default
        return float(wall['center_m'][axis]) + sign * float(wall['half_extents_m'][axis])
    return [face('wall_west', 0, 1, xmin), face('wall_east', 0, -1, xmax),
            face('wall_south', 1, 1, ymin), face('wall_north', 1, -1, ymax)]


def resolve_destination(static_map: Mapping[str, Any], value: Any) -> dict:
    parsed = parse_destination(value)
    if 'place' in parsed:
        catalog = places(static_map)
        if parsed['place'] not in catalog:
            raise ValueError('unknown authored place ' + repr(parsed['place']) + '; choose one of '
                             + ', '.join(sorted(catalog)))
        xy = catalog[parsed['place']]['center_m']
        source = 'authored_place:' + parsed['place']
    else:
        xy = parsed['xy_m']
        source = 'model_coordinate'
    xmin, xmax, ymin, ymax = interior_bounds(static_map)
    if not (xmin < xy[0] < xmax and ymin < xy[1] < ymax):
        raise ValueError('destination ' + json.dumps(xy) + ' is outside the walled map interior '
                         + json.dumps([xmin, xmax, ymin, ymax]))
    return {'destination': parsed, 'xy_m': [float(xy[0]), float(xy[1])], 'source': source,
            'frame': static_map.get('frame', 'warehouse_xy_m')}


def rect(identifier: str, center: Sequence[float], half: Sequence[float], source: str) -> dict:
    return {'id': identifier, 'center_m': [float(center[0]), float(center[1])],
            'half_extents_m': [float(half[0]), float(half[1])], 'source': source}


def authored_obstacles(static_map: Mapping[str, Any]) -> list[dict]:
    """Static obstacles plus terrain. Terrain passage is unvalidated: avoid it."""
    rows = [rect(o['id'], o['center_m'], o['half_extents_m'], 'authored static obstacle')
            for o in static_map.get('obstacles', [])]
    rows += [rect('unvalidated_' + t['id'], t['center_m'], t['half_extents_m'],
                  'authored terrain; loaded passage unvalidated')
             for t in static_map.get('terrain', [])]
    return rows


def _envelope_bounds(envelope: Mapping[str, Sequence[float]]) -> tuple[float, float, float, float]:
    ex0, ex1 = map(float, envelope['x_m'])
    ey0, ey1 = map(float, envelope['y_m'])
    if not (ex0 < ex1 and ey0 < ey1):
        raise ValueError('invalid body envelope')
    return ex0, ex1, ey0, ey1


def envelope_overlaps(point: Sequence[float], envelope, box: Mapping[str, Any], margin: float = 0.) -> bool:
    ex0, ex1, ey0, ey1 = _envelope_bounds(envelope)
    cx, cy = box['center_m']
    hx, hy = box['half_extents_m']
    return (point[0] + ex1 > cx - hx - margin and point[0] + ex0 < cx + hx + margin
            and point[1] + ey1 > cy - hy - margin and point[1] + ey0 < cy + hy + margin)


class _Grid:
    def __init__(self, bounds: Sequence[float], resolution: float):
        xmin, xmax, ymin, ymax = bounds
        self.resolution = resolution
        self.xs = np.arange(xmin + resolution / 2, xmax, resolution)
        self.ys = np.arange(ymin + resolution / 2, ymax, resolution)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)

    @property
    def shape(self):
        return self.X.shape

    def cell(self, point: Sequence[float]) -> tuple[int, int]:
        i = int(round((point[0] - self.xs[0]) / self.resolution))
        j = int(round((point[1] - self.ys[0]) / self.resolution))
        return min(max(i, 0), len(self.xs) - 1), min(max(j, 0), len(self.ys) - 1)

    def point(self, cell: tuple[int, int]) -> list[float]:
        return [float(self.xs[cell[0]]), float(self.ys[cell[1]])]

    def footprint_mask(self, box: Mapping[str, Any], envelope, margin: float) -> np.ndarray:
        """Cells whose envelope would overlap the (margin-inflated) rectangle."""
        ex0, ex1, ey0, ey1 = _envelope_bounds(envelope)
        cx, cy = box['center_m']
        hx, hy = box['half_extents_m']
        return ((self.X + ex1 > cx - hx - margin) & (self.X + ex0 < cx + hx + margin)
                & (self.Y + ey1 > cy - hy - margin) & (self.Y + ey0 < cy + hy + margin))


def _segment_samples(a: Sequence[float], b: Sequence[float], step: float) -> list[list[float]]:
    count = max(1, int(math.ceil(math.dist(a, b) / step)))
    return [[a[0] + (b[0] - a[0]) * k / count, a[1] + (b[1] - a[1]) * k / count] for k in range(count + 1)]


def path_regions(static_map: Mapping[str, Any], waypoints: Sequence[Sequence[float]], envelope) -> list[str]:
    """Authored regions touched by the body envelope anywhere along the path."""
    touched = set()
    regions = static_map.get('regions', {})
    for a, b in zip(waypoints, waypoints[1:]):
        for sample in _segment_samples(a, b, GRID_M / 2):
            for name, region in regions.items():
                if name not in touched and envelope_overlaps(sample, envelope, region):
                    touched.add(name)
    if len(waypoints) == 1:
        touched |= {n for n, r in regions.items() if envelope_overlaps(waypoints[0], envelope, r)}
    return sorted(touched)


def resource_regions(static_map: Mapping[str, Any]) -> list[str]:
    return sorted(name for name in static_map.get('resource_rules', {}) if name in static_map.get('regions', {}))


def plan_path(static_map: Mapping[str, Any], start: Sequence[float], goal: Sequence[float], envelope, *,
              obstacles: Iterable[Mapping[str, Any]] = (), blocked_regions: Iterable[str] = (),
              penalized_regions: Mapping[str, float] | None = None, escape_start_m: float = 0.,
              point_keepouts: Iterable[Mapping[str, Any]] = (),
              margin_m: float = MARGIN_M, resolution_m: float = GRID_M) -> dict | None:
    """A* for a fixed-heading translating body; None when no clear path exists.

    `obstacles` are axis-aligned rectangles (authored or RGB-estimated).
    `blocked_regions` names authored regions the body may not touch.
    `penalized_regions` multiplies step cost inside named regions.
    `escape_start_m` lets the body leave a start that already touches an
    obstacle (e.g. just-released cargo) but never enter one elsewhere.
    `point_keepouts` are rectangles the tracked reference point itself must
    avoid (not the whole envelope), e.g. where the RGB feature is hidden.
    """
    if not (_finite_pair(start) and _finite_pair(goal)):
        raise ValueError('finite start and goal required')
    envelope = {'x_m': list(map(float, envelope['x_m'])), 'y_m': list(map(float, envelope['y_m']))}
    obstacles = [dict(o) for o in authored_obstacles(static_map)] + [dict(o) for o in obstacles]
    bounds = interior_bounds(static_map)
    grid = _Grid(bounds, resolution_m)
    ex0, ex1, ey0, ey1 = _envelope_bounds(envelope)
    blocked = ~((grid.X + ex0 >= bounds[0]) & (grid.X + ex1 <= bounds[1])
                & (grid.Y + ey0 >= bounds[2]) & (grid.Y + ey1 <= bounds[3]))
    for obstacle in obstacles:
        blocked |= grid.footprint_mask(obstacle, envelope, margin_m)
    point_keepouts = [dict(o) for o in point_keepouts]
    for keepout in point_keepouts:
        cx, cy = keepout['center_m']
        hx, hy = keepout['half_extents_m']
        blocked |= (np.abs(grid.X - cx) < hx) & (np.abs(grid.Y - cy) < hy)
    regions = static_map.get('regions', {})
    blocked_regions = sorted(set(blocked_regions))
    for name in blocked_regions:
        if name not in regions:
            raise ValueError('unknown blocked region ' + name)
        blocked |= grid.footprint_mask(regions[name], envelope, 0.)
    penalty = np.ones(grid.shape)
    penalized = dict(penalized_regions or {})
    for name, factor in penalized.items():
        if name not in regions or not (isinstance(factor, (int, float)) and factor >= 1):
            raise ValueError('invalid region penalty for ' + str(name))
        penalty[grid.footprint_mask(regions[name], envelope, 0.)] *= float(factor)
    source, target = grid.cell(start), grid.cell(goal)
    escape = np.hypot(grid.X - start[0], grid.Y - start[1]) <= escape_start_m
    passable = ~blocked | escape
    if blocked[target[1], target[0]] or not passable[source[1], source[0]]:
        return None
    clearance = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5) * resolution_m
    # Prefer room for RGB projection noise and motor transients without
    # changing the hard envelope exclusion (same idea as known-map A*).
    proximity = 1. + 4. * np.exp(-clearance / .06)
    step_cost = penalty * proximity * np.where(blocked, 4., 1.)
    moves = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    width, height = len(grid.xs), len(grid.ys)
    frontier = [(0., source)]
    cost = {source: 0.}
    parent: dict = {}
    closed = set()
    while frontier:
        _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        if current == target:
            break
        for dx, dy in moves:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < height) or not passable[nxt[1], nxt[0]]:
                continue
            if dx and dy and not (passable[current[1], nxt[0]] and passable[nxt[1], current[0]]):
                continue
            new = cost[current] + math.hypot(dx, dy) * float(step_cost[nxt[1], nxt[0]])
            if new < cost.get(nxt, math.inf):
                cost[nxt], parent[nxt] = new, current
                heapq.heappush(frontier, (new + math.hypot(target[0] - nxt[0], target[1] - nxt[1]), nxt))
    if target not in cost:
        return None
    cells = [target]
    while cells[-1] != source:
        cells.append(parent[cells[-1]])
    cells.reverse()

    def clear_segment(a, b, minimum, max_penalty):
        for p in _segment_samples(grid.point(a), grid.point(b), resolution_m / 2):
            c = grid.cell(p)
            if not passable[c[1], c[0]] or penalty[c[1], c[0]] > max_penalty:
                return False
            if not escape[c[1], c[0]] and clearance[c[1], c[0]] < minimum:
                return False
        return True

    simple, anchor = [cells[0]], 0
    while anchor < len(cells) - 1:
        candidate = len(cells) - 1
        while candidate > anchor + 1:
            span = cells[anchor:candidate + 1]
            minimum = min(float(clearance[y, x]) for x, y in span if not escape[y, x]) * .95 \
                if any(not escape[y, x] for x, y in span) else 0.
            if clear_segment(cells[anchor], cells[candidate], minimum,
                             max(float(penalty[y, x]) for x, y in span)):
                break
            candidate -= 1
        simple.append(cells[candidate])
        anchor = candidate
    waypoints = [[float(start[0]), float(start[1])]] + [grid.point(c) for c in simple[1:-1]] \
        + [[float(goal[0]), float(goal[1])]]
    touched = path_regions(static_map, waypoints, envelope)
    result = {'schema': SCHEMA, 'planner': 'grid A* (8-connected, no corner cutting, clearance-weighted)',
              'start_m': waypoints[0], 'goal_m': waypoints[-1], 'waypoints_m': waypoints,
              'length_m': float(sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))),
              'regions_touched': touched,
              'resources': [r for r in resource_regions(static_map) if r in touched],
              'envelope': envelope, 'margin_m': margin_m, 'grid_m': resolution_m,
              'obstacles': [{k: o[k] for k in ('id', 'center_m', 'half_extents_m', 'source') if k in o}
                            for o in obstacles],
              'point_keepouts': [{k: o[k] for k in ('id', 'center_m', 'half_extents_m', 'source') if k in o}
                                 for o in point_keepouts],
              'blocked_regions': blocked_regions, 'penalized_regions': penalized,
              'escape_start_m': escape_start_m, 'map_sha256': digest(static_map),
              'scope': 'authored static map + supplied RGB obstacle estimates; not physical passage proof'}
    result['plan_sha256'] = digest({k: v for k, v in result.items() if k != 'plan_sha256'})
    return result


def box_slot_goal(static_map: Mapping[str, Any], dock: str) -> list[float]:
    """Final cargo target: 4 cm past the slot centre, away from the beam slot.

    This is the same authored target the fixed-route skill used; the existing
    RGB slot-containment check still decides the release.
    """
    slots = static_map['docks'][dock]['slots']
    direction = math.copysign(1., slots['box']['center_m'][0] - slots['beam']['center_m'][0])
    return [float(slots['box']['center_m'][0]) + .04 * direction, float(slots['box']['center_m'][1])]


def beam_delivered_keepout(static_map: Mapping[str, Any], dock: str) -> dict:
    """Space a delivered beam and its two carriers may occupy in the dock."""
    slots = static_map['docks'][dock]['slots']
    beam = list(slots['beam']['center_m'])
    direction = math.copysign(1., slots['box']['center_m'][0] - beam[0])
    beam[0] += direction * BEAM_PLACEMENT_OFFSET_M
    return beam_team_keepout(beam, 'planned beam delivery and parked carriers (authored slot + agreed dock)')


def beam_team_keepout(center: Sequence[float], source: str, extra_m: float = 0.) -> dict:
    lo = [center[0] + BEAM_TEAM_ENVELOPE['x_m'][0] - extra_m, center[1] + BEAM_TEAM_ENVELOPE['y_m'][0] - extra_m]
    hi = [center[0] + BEAM_TEAM_ENVELOPE['x_m'][1] + extra_m, center[1] + BEAM_TEAM_ENVELOPE['y_m'][1] + extra_m]
    return rect('beam_team_keepout', [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2],
                [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2], source)


def open_beam_corridor(static_map: Mapping[str, Any], dock: str, route: str, beam_center: Sequence[float]) -> list[dict]:
    """Conservative swept space of the fixed open-map beam route (ImageRoute).

    Mirrors the authored polyline the pair follows: to the gate lane, east to
    x=1.12, then along the lane to the dock. Keep-out only; never a command.
    """
    gate = list(static_map['regions'][route + '_gate']['center_m'])
    ymin, ymax = static_map['bounds_m'][2:]
    gate[1] = max(ymin + .48, min(ymax - .48, gate[1]))
    goal = list(static_map['docks'][dock]['slots']['beam']['center_m'])
    goal[0] += BEAM_PLACEMENT_OFFSET_M * math.copysign(
        1., static_map['docks'][dock]['slots']['box']['center_m'][0] - goal[0])
    points = [list(beam_center), [beam_center[0], gate[1]], [1.12, gate[1]], [1.12, goal[1]], goal]
    rows = []
    for index, (a, b) in enumerate(zip(points, points[1:])):
        lo = [min(a[0], b[0]) + BEAM_TEAM_ENVELOPE['x_m'][0], min(a[1], b[1]) + BEAM_TEAM_ENVELOPE['y_m'][0]]
        hi = [max(a[0], b[0]) + BEAM_TEAM_ENVELOPE['x_m'][1], max(a[1], b[1]) + BEAM_TEAM_ENVELOPE['y_m'][1]]
        rows.append(rect('beam_corridor_' + str(index), [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2],
                         [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2],
                         'agreed beam route swept by the authored pair envelope'))
    return rows


def pose_route_corridor(route: Sequence[Sequence[float]], radius_m: float = .55) -> list[dict]:
    """Squares bounding any rotation of the loaded-pair footprint along an SE(2) route.

    0.55 m covers the half diagonal of the 0.225 x 0.495 m inflated pair
    footprint used by the dispatch pair planner.
    """
    rows = []
    for index, (a, b) in enumerate(zip(route, route[1:])):
        for k, p in enumerate(_segment_samples(a[:2], b[:2], .10)):
            rows.append(rect('beam_corridor_' + str(index) + '_' + str(k), p, [radius_m, radius_m],
                             'feasibility SE2 beam route swept by its rotation-free radius'))
    return rows


def check_park(static_map: Mapping[str, Any], value: Any, *, keepouts: Sequence[Mapping[str, Any]] = ()) -> dict:
    """Validate a model-chosen waiting place for the unloaded solo robot.

    Rejected when its folded-arm envelope would touch a static obstacle, a
    shared resource region (gates/apron), a dock or a supplied keep-out such
    as the beam team's future path. The reason goes back to the models.
    """
    resolved = resolve_destination(static_map, value)
    xy = resolved['xy_m']
    problems = []
    for o in authored_obstacles(static_map):
        if envelope_overlaps(xy, UNLOADED_ENVELOPE, o, MARGIN_M):
            problems.append('touches ' + o['id'])
    xmin, xmax, ymin, ymax = interior_bounds(static_map)
    if not (xmin <= xy[0] + UNLOADED_ENVELOPE['x_m'][0] and xy[0] + UNLOADED_ENVELOPE['x_m'][1] <= xmax
            and ymin <= xy[1] + UNLOADED_ENVELOPE['y_m'][0] and xy[1] + UNLOADED_ENVELOPE['y_m'][1] <= ymax):
        problems.append('body leaves the walled interior')
    regions = static_map.get('regions', {})
    for name in resource_regions(static_map) + sorted(n for n in static_map.get('docks', {}) if n in regions):
        if envelope_overlaps(xy, UNLOADED_ENVELOPE, regions[name]):
            problems.append('blocks shared ' + name)
    hits: dict[str, list[str]] = {}
    for keepout in keepouts:
        if envelope_overlaps(xy, UNLOADED_ENVELOPE, keepout, MARGIN_M):
            hits.setdefault(keepout.get('source', 'keep-out'), []).append(keepout['id'])
    for source, ids in hits.items():
        problems.append('inside ' + source + (' (' + ids[0] + ')' if len(ids) == 1
                                              else ' (' + str(len(ids)) + ' samples from ' + ids[0] + ')'))
    return {**resolved, 'feasible': not problems, 'problems': problems,
            'envelope': copy.deepcopy(UNLOADED_ENVELOPE),
            'keepouts': [dict(k) for k in keepouts],
            'scope': 'authored map and agreed plan only; physical arrival still needs fresh RGB'}


def pixel_to_map(pixel: Sequence[float], static_map: Mapping[str, Any], shape: Sequence[int], *,
                 height: float = 0.) -> list[float]:
    """Inverse of the fixed TOP projection on a nominal feature plane."""
    cam = static_map['top_camera']
    if list(cam['quaternion_wxyz']) != [1, 0, 0, 0]:
        raise ValueError('unsupported static camera')
    h, w = shape[:2]
    cx, cy, cz = cam['position_m']
    scale = h / (2 * (cz - height) * math.tan(math.radians(cam['fov_y_deg']) / 2))
    return [float(cx + (pixel[0] - (w - 1) / 2) / scale), float(cy - (pixel[1] - (h - 1) / 2) / scale)]


def map_to_pixel(xy: Sequence[float], static_map: Mapping[str, Any], shape: Sequence[int], *,
                 height: float = 0.) -> np.ndarray:
    cam = static_map['top_camera']
    if list(cam['quaternion_wxyz']) != [1, 0, 0, 0]:
        raise ValueError('unsupported static camera')
    h, w = shape[:2]
    cx, cy, cz = cam['position_m']
    scale = h / (2 * (cz - height) * math.tan(math.radians(cam['fov_y_deg']) / 2))
    return np.array([(w - 1) / 2 + (xy[0] - cx) * scale, (h - 1) / 2 - (xy[1] - cy) * scale])


def direction_preserving(vector: Sequence[float], limits: Sequence[Sequence[float]]) -> np.ndarray:
    """Scale a command uniformly so every axis fits [low, high]; keeps heading of travel."""
    v = np.asarray(vector, dtype=float)
    factor = 1.
    for value, (low, high) in zip(v, limits):
        if value > high > 0:
            factor = min(factor, high / value)
        elif value < low < 0:
            factor = min(factor, low / value)
    return v * factor
