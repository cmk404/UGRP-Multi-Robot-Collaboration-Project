"""Offline feasibility checker for zone study scenarios: is the scenario physically solvable?

Motivation (2026-09-26): the teacher-feasibility smoke of PR #169 lost two runs
to scenarios that no executor could have solved.

* **B2** ``tri_frame`` × ``zone_wide_door``: the trio formation is 0.889 m wide
  and the only door opening is 0.50 m. The team route planner refused 332 times
  before any contact and the trial burned its whole 1800 SIM-second budget.
* **B8** ``tri_frame`` × ``zone_wide_two_doors``: a colour box parked in front of
  the wide door blocked the trio's only remaining route, and all three robots
  were committed to team items, so nobody was left to clear the box.

Both are properties of the *scenario configuration* and the *static map*, so both
can be decided before a scenario ever ships. This module does that, offline:

1. ``carry_route`` — can an item's carrying formation (item + every carrier
   chassis/arm, ``harness.zone_team_footprint``) sweep from its setup pose to a
   pose where the item lands inside its destination zone? The search is a
   breadth-first search in the item's (x, y, yaw) pose space, so turns are part
   of the route; the accepted route is then re-checked segment by segment with
   ``harness.static_keepouts.swept_clear``.
2. ``check_placement_blocking`` — if the route only fails once the *other* items'
   setup placements are in the way, which placements are the blockers, can they
   be cleared, and can that happen while the blocked item's team is formed?
3. ``check_robot_count`` — simultaneous carrier demand against the three robots.
4. ``check_hidden_events`` — after a private ``passage_blocked`` event fires,
   does every item still have a route (or is the scenario declared unsolvable)?

Everything is read-only static geometry: the scenario config, the authored map
JSON under ``maps/zones/`` and the cargo catalogue (``sim.zone_cargo``). No
MuJoCo, no scene build, no model call, no simulator state. A verdict here is a
*necessary* condition, never a claim that a trial succeeds: contact, grasping,
localisation from the robot's own camera and the executor's own choices are out
of scope, and so is anything a robot is told (this module reads the private
``eval`` section, so its output is evaluation-only and must not reach a robot).

Verdicts per item route:

``feasible``
    A route exists for the planning footprint (``TEAM_MARGIN_M`` outward growth)
    and every segment of it passes the exact swept check.
``tight``
    A route exists for the bare footprint but not with the planning margin, or
    the margin route failed its swept re-check. Physically possible, but the
    executor's own clearance would refuse it: treat as needing a SIM check.
``infeasible``
    No route even for the bare footprint on the pose lattice. Sound down to the
    lattice resolution (``GRID_M``, ``YAW_STEPS``), which is the documented limit.
"""
from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from harness.static_keepouts import (MOUTH_M, disc_footprint, inside_rect, keepout_rects, passage_zones,
                                     polygons_overlap, rect_corners, rect_distance)
from harness.static_keepouts import pose_clear as keepout_pose_clear
from harness.zone_team_footprint import TEAM_MARGIN_M, team_footprint, transform
from harness.zone_team_footprint import swept_clear as team_swept_clear
from sim.zone_arena import BOX_HALF, COLORS, MAP_DIR, ZONE_IDS
from sim.zone_cargo import CATALOGUE

SCHEMA = 'ugrp.zone_scenario_feasibility.v1'
# Pose lattice of the route search. 0.05 m matches the team route planner of
# PR #169; 12 yaw steps (30 deg) plus every item's own setup yaw.
GRID_M = .05
YAW_STEPS = 12
# Unloaded / loaded single-robot disc radii (scripts.zone_teacher: ROBOT_RADIUS_M,
# CARRY_RADIUS_M). Used only for the "is a free robot able to reach it" check.
ROBOT_RADIUS_M = .17
CARRY_RADIUS_M = .21
TEAM_SIZE = 3
ROBOTS = ('r1', 'r2', 'r3')
VERDICTS = ('feasible', 'tight', 'infeasible')
SEVERITIES = ('info', 'warning', 'infeasible')


class ScenarioInfeasible(ValueError):
    """A scenario that no executor could solve (raised only by ``raise_for_verdict``)."""


# ---------------------------------------------------------------------------
# Scenario items

@dataclass(frozen=True)
class ScenarioItem:
    """One item a scenario asks the robots to move, from the config alone."""

    item_id: str
    kind: str
    pose: tuple                     # (x, y, yaw) setup pose, world metres/radians
    destination_zone: str | None
    required_robots: int
    order_id: str | None = None

    def footprint(self, *, margin=TEAM_MARGIN_M):
        return team_footprint(self.kind, margin=margin)

    def record(self):
        return {'item_id': self.item_id, 'kind': self.kind, 'pose_m': [round(v, 6) for v in self.pose],
                'destination_zone': self.destination_zone, 'required_robots': self.required_robots,
                'order_id': self.order_id}


def required_robots(kind: str) -> int:
    """Carriers a kind needs, from the SIM catalogue (colour boxes: one)."""
    return CATALOGUE[kind].required_carriers if kind in CATALOGUE else 1


def scenario_items(scenario: Mapping) -> tuple[ScenarioItem, ...]:
    """Items of a scenario config.

    Accepts the study config layout (public ``orders`` + private
    ``eval.setup.placements``, package E) and the plain layout
    ``{'items': [{'item_id', 'kind', 'pose_m', 'destination_zone'}]}``.
    """
    orders = {o['order_id']: o for o in scenario.get('orders') or () if isinstance(o, Mapping)}
    raw = list(scenario.get('items') or ())
    if not raw:
        raw = list(((scenario.get('eval') or {}).get('setup') or {}).get('placements') or ())
    out = []
    for entry in raw:
        kind = entry['kind']
        order = orders.get(entry.get('order_id')) or {}
        zone = entry.get('destination_zone', order.get('destination_zone'))
        if zone is not None and zone not in ZONE_IDS:
            raise ValueError(f'{entry.get("item_id")}: unknown destination zone {zone!r}')
        out.append(ScenarioItem(str(entry['item_id']), kind, tuple(float(v) for v in entry['pose_m']),
                                zone, int(entry.get('required_robots', order.get('required_robots')
                                                    or required_robots(kind))),
                                entry.get('order_id')))
    return tuple(out)


def load_static_map(map_id: str, *, maps_dir: Path | str = MAP_DIR) -> dict:
    """Read one authored zone map. Read-only: this module never writes a map."""
    return json.loads((Path(maps_dir) / (str(map_id) + '.json')).read_text())


# ---------------------------------------------------------------------------
# Static geometry helpers

def min_width(polygon) -> float:
    """Smallest gap a convex polygon fits through: its minimum supporting width."""
    best = math.inf
    count = len(polygon)
    for index in range(count):
        (x0, y0), (x1, y1) = polygon[index], polygon[(index + 1) % count]
        ex, ey = x1 - x0, y1 - y0
        norm = math.hypot(ex, ey)
        if norm < 1e-12:
            continue
        nx, ny = -ey / norm, ex / norm
        best = min(best, max(abs(nx * (x - x0) + ny * (y - y0)) for x, y in polygon))
    return best


def item_rects(kind: str, pose: Sequence[float], *, optimistic: bool = False) -> tuple:
    """Keep-out rectangles of an item resting at ``pose`` (oriented, world frame).

    Boxes are exact. A cylinder becomes its circumscribing square, or -- when
    ``optimistic`` -- its inscribed square, so that a route declared *infeasible*
    is never blamed on an over-sized round obstacle.
    """
    x, y, yaw = (float(v) for v in pose)
    if kind in COLORS:
        return ((x, y, BOX_HALF[0], BOX_HALF[1], yaw),)
    out = []
    for part in CATALOGUE[kind].parts:
        if not part.collision:
            continue
        cx, cy = part.center[0], part.center[1]
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * cx - s * cy, y + s * cx + c * cy
        if part.shape == 'box':
            out.append((wx, wy, part.size[0], part.size[1], yaw + part.yaw))
        else:
            half = part.size[0] * (math.cos(math.pi / 4) if optimistic else 1.)
            out.append((wx, wy, half, half, yaw))
    return tuple(out)


def landing_half_extents(kind: str) -> tuple:
    """Object-frame rectangle that must lie inside the destination zone."""
    if kind in COLORS:
        return (BOX_HALF[0], BOX_HALF[1])
    return tuple(CATALOGUE[kind].landing_half_extents_m)


def landing_fits(kind: str, pose: Sequence[float], zone_rect: Sequence[float]) -> bool:
    """Does the item's landing rectangle at ``pose`` lie inside ``(cx, cy, hx, hy)``?"""
    hx, hy = landing_half_extents(kind)
    c, s = abs(math.cos(pose[2])), abs(math.sin(pose[2]))
    ex, ey = c * hx + s * hy, s * hx + c * hy
    cx, cy, zhx, zhy = zone_rect
    return abs(pose[0] - cx) <= zhx - ex + 1e-9 and abs(pose[1] - cy) <= zhy - ey + 1e-9


def zone_rect(static_map: Mapping, zone: str) -> tuple:
    region = static_map['regions']['zone_' + zone]
    return (float(region['center_m'][0]), float(region['center_m'][1]),
            float(region['half_extents_m'][0]), float(region['half_extents_m'][1]))


def passage_fit(static_map: Mapping, footprint) -> dict:
    """Per passage: does the footprint's minimum width fit its opening?

    ``min_width`` is a convex-hull property, so it is a hard lower bound on the
    gap any orientation of the formation needs. This is what rejects B2.
    """
    width = min_width(footprint.hull)
    out = {}
    for passage in static_map.get('passages', ()):
        if passage.get('kind') == 'passing_bay':
            continue
        opening = float(passage['width_m'])
        out[passage['id']] = {'opening_m': round(opening, 4), 'min_footprint_width_m': round(width, 4),
                              'fits': width <= opening, 'lanes': passage.get('lanes'),
                              'mouth_m': MOUTH_M.get(passage['kind'])}
    return out


# ---------------------------------------------------------------------------
# Pose lattice and clearance

@dataclass(frozen=True)
class Lattice:
    """Discrete (x, y, yaw) poses of the route search, anchored on the map bounds."""

    x0: float
    y0: float
    nx: int
    ny: int
    grid: float
    yaws: tuple

    @classmethod
    def build(cls, static_map: Mapping, extra_yaws=(), *, grid: float = GRID_M,
              yaw_steps: int = YAW_STEPS) -> Lattice:
        bx0, bx1, by0, by1 = (float(v) for v in static_map['bounds_m'])
        nx = int(math.floor((bx1 - bx0) / grid)) + 1
        ny = int(math.floor((by1 - by0) / grid)) + 1
        step = 2 * math.pi / yaw_steps
        yaws = {round(_wrap(index * step), 9) for index in range(yaw_steps)}
        yaws |= {round(_wrap(float(yaw)), 9) for yaw in extra_yaws}
        return cls(bx0, by0, nx, ny, float(grid), tuple(sorted(yaws)))

    @property
    def size(self):
        return self.nx * self.ny * len(self.yaws)

    def pose(self, index):
        i, j, k = index
        return (self.x0 + i * self.grid, self.y0 + j * self.grid, self.yaws[k])

    def nearest(self, pose):
        """Lattice index closest to ``pose``; its yaw must be on the lattice."""
        i = min(self.nx - 1, max(0, int(round((pose[0] - self.x0) / self.grid))))
        j = min(self.ny - 1, max(0, int(round((pose[1] - self.y0) / self.grid))))
        yaw = round(_wrap(pose[2]), 9)
        if yaw not in self.yaws:
            raise ValueError(f'yaw {pose[2]} is not on the lattice; pass it as an extra yaw')
        return (i, j, self.yaws.index(yaw))

    def flat(self, index):
        i, j, k = index
        return (k * self.ny + j) * self.nx + i


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Clearance:
    """Memoised footprint-vs-rectangle test over a lattice.

    Same decision as ``harness.zone_team_footprint.pose_clear`` (every convex
    part against every grown rectangle, plus the map bounds), with a bounding
    circle and convex-hull pre-filter so a whole lattice is affordable offline.
    ``tests/test_zone_scenario_feasibility.py`` pins the agreement.
    """

    def __init__(self, lattice: Lattice, footprint, rects, bounds, *, margin: float = 0.):
        self.lattice, self.footprint, self.margin = lattice, footprint, float(margin)
        self.rects = tuple(rects)
        self.grown = tuple(rect_corners(rect, margin=self.margin) for rect in self.rects)
        self.bounds = tuple(float(v) for v in bounds) if bounds is not None else None
        self.radius = footprint.radius()
        self.cache = bytearray(lattice.size)     # 0 unknown, 1 free, 2 blocked
        self.tests = 0

    def free(self, index) -> bool:
        flat = self.lattice.flat(index)
        cached = self.cache[flat]
        if cached:
            return cached == 1
        value = 1 if self.clear(self.lattice.pose(index)) else 2
        self.cache[flat] = value
        return value == 1

    def clear(self, pose) -> bool:
        self.tests += 1
        hull = transform(self.footprint.hull, pose)
        if self.bounds is not None:
            x0, x1, y0, y1 = self.bounds
            if any(not (x0 + self.margin <= x <= x1 - self.margin
                        and y0 + self.margin <= y <= y1 - self.margin) for x, y in hull):
                return False
        reach = self.radius + self.margin + 1e-9
        near = [grown for rect, grown in zip(self.rects, self.grown)
                if rect_distance(pose[:2], rect) <= reach]
        if not near or not any(polygons_overlap(hull, grown) for grown in near):
            return True
        return not any(polygons_overlap(transform(part, pose), grown)
                       for part in self.footprint.parts for grown in near)


# ---------------------------------------------------------------------------
# Route search

@dataclass
class CarryRoute:
    """Result of one item's carry-route search."""

    item_id: str
    kind: str
    verdict: str
    reason_ko: str
    start_pose: tuple = ()
    goal_pose: tuple | None = None
    path: list = field(default_factory=list)
    passages_used: tuple = ()
    passage_fit: dict = field(default_factory=dict)
    swept_ok: bool | None = None
    margin_m: float | None = None
    poses_tested: int = 0
    proof: str = ''

    @property
    def ok(self):
        return self.verdict != 'infeasible'

    def record(self):
        return {'item_id': self.item_id, 'kind': self.kind, 'verdict': self.verdict,
                'reason_ko': self.reason_ko, 'goal_pose_m': (None if self.goal_pose is None
                                                             else [round(v, 4) for v in self.goal_pose]),
                'path_poses': len(self.path), 'passages_used': list(self.passages_used),
                'passage_fit': self.passage_fit, 'swept_ok': self.swept_ok, 'margin_m': self.margin_m,
                'poses_tested': self.poses_tested, 'proof': self.proof}


def _search(lattice: Lattice, clearance: Clearance, start, goal_pred):
    """Breadth-first search over lattice poses. Returns (goal index, parents) or (None, parents)."""
    if not clearance.free(start):
        return None, {}
    layers = len(lattice.yaws)
    parents = {start: None}
    queue, head = [start], 0
    while head < len(queue):
        index = queue[head]
        head += 1
        if goal_pred(lattice.pose(index)):
            return index, parents
        i, j, k = index
        for nxt in ((i + 1, j, k), (i - 1, j, k), (i, j + 1, k), (i, j - 1, k),
                    (i, j, (k + 1) % layers), (i, j, (k - 1) % layers)):
            if not (0 <= nxt[0] < lattice.nx and 0 <= nxt[1] < lattice.ny) or nxt in parents:
                continue
            if clearance.free(nxt):
                parents[nxt] = index
                queue.append(nxt)
    return None, parents


def _path(lattice, goal, parents):
    out, index = [], goal
    while index is not None:
        out.append(lattice.pose(index))
        index = parents[index]
    return out[::-1]


def crossing_zones(static_map: Mapping, *, side_m: float = .3) -> tuple:
    """``(passage_id, zone_rect, single_lane)`` for every door/corridor of the map.

    ``harness.static_keepouts.passage_zones`` only yields the single-lane
    passages a robot can deadlock in; a route may also use a two-lane door, so
    the mouths are grown the same way here and the single-lane flag is taken
    straight from that function.
    """
    single = {pid for pid, _core, _zone in passage_zones(static_map, side_m=side_m)}
    out = []
    for passage in static_map.get('passages', ()):
        if passage.get('kind') == 'passing_bay':
            continue
        (cx, cy), (hx, hy) = passage['center_m'], passage['half_extents_m']
        mouth = MOUTH_M[passage['kind']]
        grow = (mouth, side_m) if passage['axis'] == 'x' else (side_m, mouth)
        out.append((passage['id'], (cx, cy, hx + grow[0], hy + grow[1], 0.), passage['id'] in single))
    return tuple(out)


def _passages_on(static_map, path):
    return tuple(pid for pid, zone, _single in crossing_zones(static_map)
                 if any(inside_rect(pose[:2], zone) for pose in path))


def carry_route(static_map: Mapping, item: ScenarioItem, *, obstacles=(), margin: float = TEAM_MARGIN_M,
                grid: float = GRID_M, yaw_steps: int = YAW_STEPS, verify_swept: bool = True,
                confirm_infeasible: bool = True) -> CarryRoute:
    """Can ``item``'s formation sweep from its setup pose into its destination zone?

    ``obstacles``: extra keep-out rectangles (other items' setup placements, a
    hidden-event obstacle). The search runs twice: with ``margin`` (the planning
    footprint) and, if that fails, with a bare footprint, so a failure is
    reported as ``tight`` or ``infeasible`` rather than a single boolean.

    ``confirm_infeasible``: before reporting ``infeasible``, repeat the bare
    search once on a lattice of half the step and twice the yaw resolution, so
    the verdict is not an artefact of the lattice. The repeat is skipped when no
    passage of the map admits the formation at all (``min_width`` is a hull
    property, so no resolution can squeeze a 0.889 m formation through a 0.50 m
    door): that verdict is recorded as ``proof='passage_width'``.
    """
    fit = passage_fit(static_map, item.footprint(margin=margin))
    if item.destination_zone is None:
        return CarryRoute(item.item_id, item.kind, 'feasible', '목적 구역이 없어 운반 경로를 검사하지 않았다.',
                          item.pose, passage_fit=fit)
    target = zone_rect(static_map, item.destination_zone)
    rects = tuple(keepout_rects(static_map, perimeter=True)) + tuple(obstacles)
    lattice = Lattice.build(static_map, (item.pose[2],), grid=grid, yaw_steps=yaw_steps)
    start = lattice.nearest(item.pose)

    def goal_pred(pose):
        return landing_fits(item.kind, pose, target)

    tested = 0
    attempts = (margin, 0.) if margin > 0. else (0.,)
    for attempt_margin in attempts:
        footprint = item.footprint(margin=attempt_margin)
        clearance = Clearance(lattice, footprint, rects, static_map['bounds_m'], margin=0.)
        goal, parents = _search(lattice, clearance, start, goal_pred)
        tested += clearance.tests
        if goal is None:
            if attempt_margin != attempts[-1]:
                continue
            bare_fit = passage_fit(static_map, footprint)
            start_blocked = not clearance.free(start)
            proof = 'start_pose' if start_blocked \
                else 'passage_width' if bare_fit and not any(v['fits'] for v in bare_fit.values()) \
                else 'lattice_search'
            if proof == 'lattice_search' and confirm_infeasible:
                refined = carry_route(static_map, item, obstacles=obstacles, margin=0.,
                                      grid=grid / 2., yaw_steps=yaw_steps * 2,
                                      verify_swept=False, confirm_infeasible=False)
                tested += refined.poses_tested
                if refined.verdict != 'infeasible':
                    return CarryRoute(item.item_id, item.kind, 'tight',
                                      '격자 %.3f m / yaw %d에서는 경로가 없고 절반 격자에서만 있다: '
                                      '실기 여유가 부족한 좁은 경로다.' % (grid, yaw_steps), item.pose,
                                      refined.goal_pose, refined.path, refined.passages_used, fit,
                                      None, 0., tested, 'refined_lattice')
            reason = ('출발 자세에서 편대 발자국이 이미 막혀 있다.' if start_blocked
                      else _no_route_reason(item, bare_fit, target))
            return CarryRoute(item.item_id, item.kind, 'infeasible', reason, item.pose,
                              passage_fit=fit, margin_m=attempt_margin, poses_tested=tested, proof=proof)
        path = _path(lattice, goal, parents)
        swept = None
        if verify_swept:
            swept = all(team_swept_clear(a, b, footprint, rects, bounds=static_map['bounds_m'])
                        for a, b in zip(path, path[1:]))
        verdict = 'feasible' if attempt_margin == margin and swept is not False else 'tight'
        reason = ('여유 %.2f m 발자국으로 경로가 있고 구간 sweep 검사를 통과했다.' % attempt_margin
                  if verdict == 'feasible' else
                  '맨 발자국으로만 경로가 있다(계획 여유 %.2f m로는 없음): 실기 여유가 부족하다.' % margin
                  if attempt_margin == 0. else
                  '경로는 있으나 구간 sweep 검사에서 벽에 닿는 구간이 있다.')
        return CarryRoute(item.item_id, item.kind, verdict, reason, item.pose, path[-1], path,
                          _passages_on(static_map, path), fit, swept, attempt_margin, tested,
                          'lattice_route')
    raise AssertionError('unreachable')


def _no_route_reason(item: ScenarioItem, fit: Mapping, target) -> str:
    blocked = [pid for pid, value in fit.items() if not value['fits']]
    if blocked and not any(value['fits'] for value in fit.values()):
        worst = fit[blocked[0]]
        return ('편대 최소 폭 %.3f m가 모든 통로 개구부보다 넓다(%s). 접촉 전에 경로가 없다.'
                % (worst['min_footprint_width_m'],
                   ', '.join('%s %.2f m' % (pid, fit[pid]['opening_m']) for pid in blocked)))
    if blocked:
        return ('구역 %s까지 남은 통로(%s)로 가는 경로가 없다. 통과 불가 통로: %s.'
                % (item.destination_zone,
                   ', '.join(pid for pid, value in fit.items() if value['fits']), ', '.join(blocked)))
    return '구역 %s 안에 내려놓을 수 있는 자세까지 가는 경로가 없다.' % item.destination_zone


# ---------------------------------------------------------------------------
# Findings

@dataclass(frozen=True)
class Finding:
    code: str
    severity: str
    reason_ko: str
    item_id: str | None = None
    detail: dict = field(default_factory=dict)

    def record(self):
        return {'code': self.code, 'severity': self.severity, 'item_id': self.item_id,
                'reason_ko': self.reason_ko, 'detail': self.detail}


def _other_rects(items, item_id, *, optimistic=False, skip=()):
    return tuple(rect for other in items if other.item_id != item_id and other.item_id not in skip
                 for rect in item_rects(other.kind, other.pose, optimistic=optimistic))


def check_carry_routes(static_map, items, *, margin=TEAM_MARGIN_M, **kw) -> tuple:
    """Every item's formation route against the walls alone (no other placements)."""
    routes, findings = {}, []
    for item in items:
        route = carry_route(static_map, item, margin=margin, **kw)
        routes[item.item_id] = route
        if route.verdict == 'infeasible':
            findings.append(Finding('no_carry_route', 'infeasible', route.reason_ko, item.item_id,
                                    {'kind': item.kind, 'destination_zone': item.destination_zone,
                                     'passage_fit': route.passage_fit}))
        elif route.verdict == 'tight':
            findings.append(Finding('tight_carry_route', 'warning', route.reason_ko, item.item_id,
                                    {'kind': item.kind, 'margin_m': route.margin_m,
                                     'swept_ok': route.swept_ok}))
    return routes, tuple(findings)


def check_placement_blocking(static_map, items, routes, *, margin=TEAM_MARGIN_M, **kw) -> tuple:
    """Does another item's setup placement close an item's only route?

    Run with every other placement present. When the route drops below
    ``feasible`` although the walls-only route was feasible, drop one placement at
    a time to name the blockers, then decide whether the blockers can be cleared
    *while* the blocked item's team stands:
    ``required_robots(blocked) + required_robots(cheapest blocker) <= 3``. Above
    three, the blockers have to be delivered first -- a hard ordering constraint
    the executor must honour, which is the B8 failure of PR #169.
    """
    blocking, findings = {}, []
    by_id = {item.item_id: item for item in items}
    for item in items:
        if item.destination_zone is None or routes[item.item_id].verdict != 'feasible':
            continue
        loaded = carry_route(static_map, item, obstacles=_other_rects(items, item.item_id),
                             margin=margin, **kw)
        entry = {'verdict': loaded.verdict, 'blockers': [], 'reason_ko': loaded.reason_ko}
        blocking[item.item_id] = entry
        if loaded.verdict == 'feasible':
            continue
        blockers = []
        for other in items:
            if other.item_id == item.item_id:
                continue
            probe = carry_route(static_map, item, obstacles=_other_rects(
                items, item.item_id, optimistic=True, skip=(other.item_id,)), margin=margin, **kw)
            if probe.verdict == 'feasible':
                blockers.append(other.item_id)
        entry['blockers'] = blockers
        severity = 'infeasible' if loaded.verdict == 'infeasible' else 'warning'
        if not blockers:
            findings.append(Finding(
                'blocked_by_placements', severity,
                '다른 물건들의 초기 배치가 %s의 경로를 %s 상태로 만들고, 하나만 치워도 풀리지 않는다(%s).'
                % (item.item_id, ROUTE_KO[loaded.verdict], loaded.reason_ko), item.item_id,
                {'loaded_verdict': loaded.verdict, 'reason_ko': loaded.reason_ko}))
            continue
        movable = [bid for bid in blockers if by_id[bid].destination_zone is not None
                   and routes[bid].ok]
        stuck = [bid for bid in blockers if bid not in movable]
        if not movable:
            findings.append(Finding(
                'blocker_cannot_be_cleared', 'infeasible',
                '%s의 경로를 막는 %s를 옮길 수 없다(목적 구역이 없거나 그 물건의 경로가 없다).'
                % (item.item_id, ', '.join(stuck)), item.item_id,
                {'blockers': blockers, 'unclearable': stuck, 'loaded_verdict': loaded.verdict}))
            continue
        need = item.required_robots + min(by_id[bid].required_robots for bid in movable)
        entry['concurrent_demand'] = need
        detail = {'blockers': blockers, 'movable': movable, 'unclearable': stuck,
                  'loaded_verdict': loaded.verdict, 'required_robots': item.required_robots,
                  'blocker_required_robots': {bid: by_id[bid].required_robots for bid in movable},
                  'concurrent_demand': need, 'team_size': TEAM_SIZE}
        if need > TEAM_SIZE:
            findings.append(Finding(
                'clearing_needs_precedence', 'warning',
                '%s(%d대)의 경로를 %s의 초기 배치가 막는다(막힌 상태: %s). 치울 로봇까지 동시에 %d대가 '
                '필요해 로봇 %d대로는 동시에 못 한다: %s를 먼저 배달해야 열린다(순서 강제, PR #169 B8와 '
                '같은 모양).'
                % (item.item_id, item.required_robots, ', '.join(blockers), ROUTE_KO[loaded.verdict],
                   need, TEAM_SIZE, '/'.join(movable)), item.item_id, detail))
        else:
            findings.append(Finding(
                'clearing_robot_available', 'info',
                '%s의 경로는 %s를 치운 뒤 열린다. 동시 필요 로봇 %d대 ≤ %d대이므로 남는 로봇이 치울 수 있다.'
                % (item.item_id, ', '.join(blockers), need, TEAM_SIZE), item.item_id, detail))
    return blocking, tuple(findings)


def check_robot_count(items, blocking) -> tuple:
    """Carrier demand against the three robots, including forced-precedence pairs."""
    findings = []
    per_item = {item.item_id: item.required_robots for item in items}
    for item in items:
        if item.required_robots > TEAM_SIZE:
            findings.append(Finding('too_many_carriers', 'infeasible',
                                    '%s(%s)는 %d대가 필요하지만 팀은 %d대다.'
                                    % (item.item_id, item.kind, item.required_robots, TEAM_SIZE),
                                    item.item_id, {'required_robots': item.required_robots}))
    peak = max([entry.get('concurrent_demand', 0) for entry in blocking.values()] or [0])
    solo_only = [iid for iid, need in per_item.items() if need == 1]
    team_items = {iid: need for iid, need in per_item.items() if need > 1}
    if sum(team_items.values()) > TEAM_SIZE and not solo_only:
        findings.append(Finding('no_free_robot', 'warning',
                                '팀 물건 %s의 필요 인원 합 %d대가 로봇 %d대를 넘고 단독 물건이 없다: '
                                '한 팀이 막히면 치울 로봇이 남지 않는다.'
                                % (', '.join(sorted(team_items)), sum(team_items.values()), TEAM_SIZE),
                                None, {'team_items': team_items}))
    return {'per_item': per_item, 'team_size': TEAM_SIZE, 'peak_concurrent_demand': peak,
            'solo_items': sorted(solo_only), 'team_items': team_items}, tuple(findings)


def _event_rects(event: Mapping, *, optimistic=False) -> tuple:
    obstacle = ((event.get('target') or {}).get('obstacle')) or {}
    if not obstacle:
        return ()
    cx, cy = (float(v) for v in obstacle['center_m'])
    hx, hy = (float(v) for v in obstacle['half_extents_m'])
    if optimistic:
        hx, hy = max(0., hx - 1e-6), max(0., hy - 1e-6)
    return ((cx, cy, hx, hy, 0.),)


def declared_unsolvable(scenario: Mapping) -> dict:
    """Optional ``eval.feasibility`` declaration: what the scenario means to be unsolvable.

    ``{'intentionally_unsolvable': bool, 'items': [item_id], 'events': [event_id],
    'notes_ko': str}``. Absent means "every item must stay solvable".
    """
    value = dict(((scenario.get('eval') or {}).get('feasibility')) or {})
    return {'intentionally_unsolvable': bool(value.get('intentionally_unsolvable')),
            'items': tuple(value.get('items') or ()), 'events': tuple(value.get('events') or ()),
            'notes_ko': value.get('notes_ko', '')}


BLOCKING_EVENT_KINDS = ('passage_blocked', 'obstruction_added')


def check_hidden_events(static_map, items, scenario, routes, *, margin=TEAM_MARGIN_M, **kw) -> tuple:
    """Does every item still have a route after a private event moves the geometry?

    Checked kinds: ``passage_blocked`` / ``obstruction_added`` (the obstacle is
    added to the walls) and ``item_moved`` (the item is re-routed from its new
    pose, and the new pose becomes an obstacle for everybody else). Only the
    walls plus the event change are used, so the verdict isolates the event: the
    effect of the other setup placements is ``check_placement_blocking``'s
    finding. Obstacle rectangles are shrunk by a micron so a lost route is never
    an artefact of a touching rectangle.

    ``item_dropped`` and ``robot_hold`` change no static geometry and are
    recorded as not checked; a drop lands wherever the physics puts it, which is
    a SIM question this module must not answer.
    """
    events = ((scenario.get('eval') or {}).get('hidden_events')) or ()
    declared = declared_unsolvable(scenario)
    by_id = {item.item_id: item for item in items}
    results, findings = [], []
    for event in events:
        if not isinstance(event, Mapping):
            continue
        kind, target = event.get('kind'), event.get('target') or {}
        entry = {'event_id': event.get('event_id'), 'kind': kind,
                 'at_sim_s': (event.get('trigger') or {}).get('at_sim_s'),
                 'passage': target.get('passage'), 'lost': [], 'kept': [], 'checked': False}
        rects = _event_rects(event, optimistic=True)
        moved = by_id.get(target.get('item_id')) if kind == 'item_moved' else None
        if kind in BLOCKING_EVENT_KINDS and rects:
            entry['checked'] = True
            entry['obstacle_rects'] = [[round(v, 6) for v in rect] for rect in rects]
            for item in items:
                if item.destination_zone is None or routes[item.item_id].verdict != 'feasible':
                    continue
                after = carry_route(static_map, item, obstacles=rects, margin=margin, **kw)
                (entry['kept'] if after.verdict == 'feasible' else entry['lost']).append(item.item_id)
        elif moved is not None and target.get('to_pose_m'):
            entry['checked'] = True
            pose = tuple(float(v) for v in target['to_pose_m'])
            entry['to_pose_m'] = [round(v, 6) for v in pose]
            relocated = ScenarioItem(moved.item_id, moved.kind, pose, moved.destination_zone,
                                     moved.required_robots, moved.order_id)
            after = carry_route(static_map, relocated, margin=margin, **kw)
            (entry['kept'] if after.verdict == 'feasible' else entry['lost']).append(moved.item_id)
            entry['moved_reason_ko'] = after.reason_ko
            rects = item_rects(moved.kind, pose, optimistic=True)
            entry['obstacle_rects'] = [[round(v, 6) for v in rect] for rect in rects]
            for item in items:
                if item.item_id == moved.item_id or item.destination_zone is None \
                        or routes[item.item_id].verdict != 'feasible':
                    continue
                other = carry_route(static_map, item, obstacles=rects, margin=margin, **kw)
                (entry['kept'] if other.verdict == 'feasible' else entry['lost']).append(item.item_id)
        if not entry['checked']:
            results.append(entry)
            continue
        allowed = declared['intentionally_unsolvable'] or event.get('event_id') in declared['events']
        for item_id in entry['lost']:
            if allowed or item_id in declared['items']:
                findings.append(Finding('declared_unsolvable_after_event', 'info',
                                        '사건 %s 뒤 %s의 경로가 사라지지만 시나리오가 의도된 불가능으로 '
                                        '선언했다.' % (entry['event_id'], item_id), item_id, dict(entry)))
            else:
                findings.append(Finding(
                    'no_route_after_event', 'infeasible',
                    '숨은 사건 %s(%s, %s SIM초, 대상 %s)이 일어나면 %s의 남은 경로가 없다. 의도된 '
                    '불가능이면 eval.feasibility에 선언해야 한다.'
                    % (entry['event_id'], entry['kind'], entry['at_sim_s'],
                       entry.get('passage') or target.get('item_id') or '—', item_id), item_id,
                    dict(entry)))
        results.append(entry)
    return tuple(results), tuple(findings)


def check_free_robot_access(static_map, items, *, obstacles=(), clearance_m: float = GRID_M) -> tuple:
    """Can an unloaded robot disc stand at every carrier station of every item?

    A blocker is only clearable if a robot can take its grasp station. The disc
    footprint and radius come from the teacher planner
    (``scripts.zone_teacher.ROBOT_RADIUS_M``) through
    ``harness.static_keepouts.disc_footprint``; other items' placements count as
    obstacles, because a station inside another item is not a station.

    The measured gap at each station is reported too. A station that is clear by
    less than ``clearance_m`` (the teacher's planning grid) is flagged: the disc
    planner cannot reliably drive into it, and a scenario whose *intent* depends
    on that gap being open or closed is resting on millimetres.
    """
    footprint = disc_footprint(ROBOT_RADIUS_M)
    disc_radius = ROBOT_RADIUS_M / math.cos(math.pi / 16)
    walls = tuple(keepout_rects(static_map, perimeter=True)) + tuple(obstacles)
    bounds = static_map['bounds_m']
    findings, stations = [], {}
    for item in items:
        rects = walls + _other_rects(items, item.item_id)
        team = item.footprint(margin=0.)
        blocked, gaps = [], {}
        for role, offset in team.stations.items():
            x, y, yaw = item.pose
            c, s = math.cos(yaw), math.sin(yaw)
            pose = (x + c * offset[0] - s * offset[1], y + s * offset[0] + c * offset[1],
                    _wrap(yaw + offset[2]))
            gaps[role] = round(min([rect_distance(pose[:2], rect) - disc_radius for rect in rects]
                                   or [math.inf]), 5)
            if not keepout_pose_clear(pose, footprint, rects, bounds=bounds):
                blocked.append((role, pose))
        tight = {role: gap for role, gap in gaps.items()
                 if 0. <= gap < clearance_m and role not in [r for r, _ in blocked]}
        stations[item.item_id] = {'roles': list(team.stations), 'gap_m': gaps,
                                  'blocked_roles': [r for r, _ in blocked]}
        if blocked:
            findings.append(Finding(
                'station_not_standable', 'warning',
                '%s의 파지 자리 %s에 빈 로봇(반지름 %.2f m)이 설 수 없다(벽 또는 다른 물건의 초기 배치).'
                % (item.item_id, ', '.join(role for role, _ in blocked), ROBOT_RADIUS_M), item.item_id,
                {'blocked': [{'role': role, 'pose_m': [round(v, 4) for v in pose]}
                             for role, pose in blocked], 'gap_m': gaps}))
        if tight:
            findings.append(Finding(
                'station_clearance_tight', 'warning',
                '%s의 파지 자리 여유가 %s로 교사 계획 격자 %.2f m보다 작다: 설 수는 있지만 주행으로 들어가는 '
                '것은 보장되지 않는다(설계 의도가 이 틈에 걸려 있으면 다시 정해야 한다).'
                % (item.item_id, ', '.join('%s %.4f m' % pair for pair in sorted(tight.items())),
                   clearance_m), item.item_id, {'gap_m': gaps, 'threshold_m': clearance_m}))
    return stations, tuple(findings)


# ---------------------------------------------------------------------------
# Scenario report

@dataclass
class FeasibilityReport:
    scenario_id: str
    map_id: str
    items: tuple = ()
    routes: dict = field(default_factory=dict)
    blocking: dict = field(default_factory=dict)
    robots: dict = field(default_factory=dict)
    events: tuple = ()
    findings: tuple = ()
    manifest: dict = field(default_factory=dict)

    @property
    def verdict(self):
        if any(f.severity == 'infeasible' for f in self.findings):
            return 'infeasible'
        return 'conditional' if any(f.severity == 'warning' for f in self.findings) else 'feasible'

    @property
    def ok(self):
        return self.verdict != 'infeasible'

    def raise_for_verdict(self):
        if not self.ok:
            raise ScenarioInfeasible('%s: %s' % (self.scenario_id, '; '.join(
                f.reason_ko for f in self.findings if f.severity == 'infeasible')))
        return self

    def record(self):
        return {'schema': SCHEMA, 'scenario_id': self.scenario_id, 'map_id': self.map_id,
                'verdict': self.verdict, 'items': [item.record() for item in self.items],
                'routes': {iid: route.record() for iid, route in self.routes.items()},
                'blocking': self.blocking, 'robots': self.robots, 'hidden_events': list(self.events),
                'findings': [f.record() for f in self.findings], 'manifest': self.manifest}


VERDICT_KO = {'feasible': '실현 가능', 'conditional': '조건부 실현 가능', 'infeasible': '실현 불가능'}
ROUTE_KO = {'feasible': '가능', 'tight': '여유 부족', 'infeasible': '불가능'}


def evaluate(scenario: Mapping, *, maps_dir: Path | str = MAP_DIR, static_map: Mapping | None = None,
             margin: float = TEAM_MARGIN_M, grid: float = GRID_M, yaw_steps: int = YAW_STEPS,
             verify_swept: bool = True) -> FeasibilityReport:
    """Run every feasibility check over one scenario config. Never raises on a bad scenario."""
    map_id = scenario.get('map_id') or ((scenario.get('eval') or {}).get('setup') or {}).get('arena_variant')
    file_sha = None
    if static_map is None:
        raw = (Path(maps_dir) / (str(map_id) + '.json')).read_bytes()
        data, file_sha = json.loads(raw), hashlib.sha256(raw).hexdigest()
    else:
        data = dict(static_map)
    items = scenario_items(scenario)
    kw = {'grid': grid, 'yaw_steps': yaw_steps, 'verify_swept': verify_swept}
    routes, findings = check_carry_routes(data, items, margin=margin, **kw)
    blocking, blocking_findings = check_placement_blocking(data, items, routes, margin=margin, **kw)
    robots, robot_findings = check_robot_count(items, blocking)
    events, event_findings = check_hidden_events(data, items, scenario, routes, margin=margin, **kw)
    stations, access_findings = check_free_robot_access(data, items)
    robots['carrier_stations'] = stations
    report = FeasibilityReport(
        str(scenario.get('scenario_id') or map_id), str(map_id), items, routes, blocking, robots, events,
        findings + blocking_findings + robot_findings + event_findings + access_findings)
    report.manifest = {
        'schema': SCHEMA, 'map_id': str(map_id), 'map_version': data.get('version'),
        'map_file_sha256': file_sha, 'map_canonical_sha256': _digest(data),
        'bounds_m': list(data['bounds_m']),
        'grid_m': grid, 'yaw_steps': yaw_steps, 'planning_margin_m': margin,
        'robots': list(ROBOTS), 'footprint_source': 'harness.zone_team_footprint',
        'robot_radius_m': ROBOT_RADIUS_M, 'carry_radius_m': CARRY_RADIUS_M,
        'geometry_source': 'harness.static_keepouts', 'cargo_source': 'sim.zone_cargo',
        'declared_unsolvable': declared_unsolvable(scenario),
        'verified': ['운반 편대 발자국의 (x, y, yaw) 경로 존재', '구간 sweep 재검사', '초기 배치 막힘과 최소 원인',
                     '동시 필요 로봇 수', 'passage_blocked 사건 뒤 경로'],
        'not_verified': ['파지·접촉 성공', '자기 카메라 위치 추정', '실행기의 선언·순서 정책',
                         '격자(%.2f m, %d yaw)보다 좁은 틈' % (grid, yaw_steps), '실제 SIM 완주']}
    return report


def _digest(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def report_ko(report: FeasibilityReport) -> str:
    """Korean per-scenario report with the reason for every verdict."""
    lines = ['# %s — %s' % (report.scenario_id, VERDICT_KO[report.verdict]),
             '',
             '- 지도: `%s` v%s (파일 sha256 `%s`, 정규화 sha256 `%s`)'
             % (report.map_id, report.manifest.get('map_version'),
                (report.manifest.get('map_file_sha256') or '—')[:16],
                report.manifest.get('map_canonical_sha256', '')[:16]),
             '- 격자 %.2f m / yaw %d단계, 계획 여유 %.2f m, 로봇 %d대'
             % (report.manifest.get('grid_m', GRID_M), report.manifest.get('yaw_steps', YAW_STEPS),
                report.manifest.get('planning_margin_m', TEAM_MARGIN_M), TEAM_SIZE),
             '- 판정 근거: 정적 지도·설정 배치·화물 목록만 사용한 오프라인 기하 검사다. 파지·인식·실행기 정책은 '
             '검사하지 않는다.', '',
             '## 물건별 운반 경로', '',
             '| 물건 | 종류 | 필요 로봇 | 목적 구역 | 판정 | 통과 통로 | 이유 |',
             '|---|---|---|---|---|---|---|']
    by_id = {item.item_id: item for item in report.items}
    for item_id, route in report.routes.items():
        item = by_id[item_id]
        lines.append('| `%s` | %s | %d | %s | %s | %s | %s |'
                     % (item_id, item.kind, item.required_robots, item.destination_zone or '—',
                        ROUTE_KO[route.verdict], ', '.join(route.passages_used) or '—', route.reason_ko))
    lines += ['', '## 초기 배치 막힘과 순서 제약', '']
    blocked = {iid: entry for iid, entry in report.blocking.items() if entry['verdict'] != 'feasible'}
    if not blocked:
        lines.append('- 다른 물건의 초기 배치가 어떤 물건의 경로도 좁히지 않는다.')
    for item_id, entry in blocked.items():
        lines.append('- `%s`: 다른 배치가 있으면 경로 %s. 원인 배치: %s. 동시 필요 로봇 %s대. %s'
                     % (item_id, ROUTE_KO[entry['verdict']],
                        ', '.join('`%s`' % b for b in entry['blockers']) or '(단일 원인 없음)',
                        entry.get('concurrent_demand', '—'), entry['reason_ko']))
    lines += ['', '## 로봇 수와 파지 자리', '',
              '- 물건별 필요 인원: %s' % ', '.join('%s %d대' % (iid, need)
                                                for iid, need in report.robots['per_item'].items()),
              '- 팀 물건 필요 인원 합 %d대, 단독 물건 %d개, 막힘 해소 동시 필요 최대 %d대 (보유 %d대)'
              % (sum(report.robots['team_items'].values()), len(report.robots['solo_items']),
                 report.robots['peak_concurrent_demand'], TEAM_SIZE)]
    for item_id, entry in (report.robots.get('carrier_stations') or {}).items():
        lines.append('- `%s` 파지 자리 여유(빈 로봇 반지름 %.2f m 기준): %s%s'
                     % (item_id, ROBOT_RADIUS_M,
                        ', '.join('%s %s' % (role, '무한' if gap == math.inf else '%.4f m' % gap)
                                  for role, gap in entry['gap_m'].items()),
                        '; 설 수 없는 자리: ' + ', '.join(entry['blocked_roles'])
                        if entry['blocked_roles'] else ''))
    lines += ['', '## 숨은 사건', '']
    if not report.events:
        lines.append('- 숨은 사건이 없다.')
    for entry in report.events:
        lines.append('- `%s` (%s, %s SIM초): %s'
                     % (entry['event_id'], entry['kind'], entry['at_sim_s'],
                        ('사건 뒤 경로 유지 %s / 경로 상실 %s'
                         % (', '.join(entry['kept']) or '없음', ', '.join(entry['lost']) or '없음')
                         if entry.get('checked') else
                         '정적 기하를 바꾸지 않아(%s) 경로 재검사 대상이 아니다.' % entry['kind'])))
    lines += ['', '## 발견 사항', '']
    if not report.findings:
        lines.append('- 없다.')
    for finding in report.findings:
        lines.append('- **%s** [%s]%s %s' % (finding.code, finding.severity,
                                             ' `%s`:' % finding.item_id if finding.item_id else '',
                                             finding.reason_ko))
    lines += ['', '## 검사 범위', '',
              '- 확인: %s' % '; '.join(report.manifest.get('verified', ())),
              '- 미확인: %s' % '; '.join(report.manifest.get('not_verified', ()))]
    return '\n'.join(lines) + '\n'


__all__ = ['SCHEMA', 'GRID_M', 'YAW_STEPS', 'ROBOT_RADIUS_M', 'CARRY_RADIUS_M', 'TEAM_SIZE', 'ROBOTS',
           'VERDICTS', 'SEVERITIES', 'ScenarioInfeasible', 'ScenarioItem', 'required_robots',
           'scenario_items', 'load_static_map', 'min_width', 'item_rects', 'landing_half_extents',
           'landing_fits', 'zone_rect', 'passage_fit', 'crossing_zones', 'Lattice', 'Clearance',
           'CarryRoute', 'carry_route', 'Finding', 'check_carry_routes', 'check_placement_blocking',
           'check_robot_count', 'check_hidden_events', 'check_free_robot_access', 'declared_unsolvable',
           'FeasibilityReport', 'evaluate', 'report_ko', 'VERDICT_KO', 'ROUTE_KO']
