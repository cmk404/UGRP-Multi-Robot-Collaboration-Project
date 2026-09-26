"""Team carry routes in item-pose space (zone team A2, 2026-09-25). Pure geometry.

A carrying team moves as one rigid footprint (``harness.zone_team_footprint``:
item + every carrier chassis and arm). Its route is planned once, before
contact, on the item pose (x, y, yaw) with straight translations along the
world axes and in-place turns of 90 degrees only:

- translations: exact swept test (the swept area of a translating convex part
  is the convex hull of its start and end placements), against the map's
  interior walls, the arena bounds and the other items on the floor;
- turns: only where the disc swept by the whole footprint is clear, so a team
  never turns inside a door or corridor;
- the final route is re-checked leg by leg with the sampled swept test of
  ``harness.static_keepouts`` (``zone_team_footprint.swept_clear``) and the
  result is recorded; a route that fails is rejected before contact.

``PoseReference`` plays a route as the reference item pose for the
virtual-structure carry (``harness.zone_team_formation.FormationPlan.carry_step``),
with the PR #164 speeds (0.05 m/s, 0.2 rad/s).
"""
from __future__ import annotations

import heapq
import math

from harness.static_keepouts import polygons_overlap, rect_corners
from harness.zone_team_footprint import circle, convex_hull, swept_clear, transform

GRID_M = .05
TURN_COST_M = .60
# A change of translation direction costs this much: fewer, longer straight legs.
BEND_COST_M = .15
MAX_EXPANSIONS = 60000
REF_SPEED_M_S, REF_YAW_RATE = .05, .20


def _wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def _circumradius(poly):
    cx = sum(x for x, _ in poly)/len(poly)
    cy = sum(y for _, y in poly)/len(poly)
    return (cx, cy), max(math.hypot(x-cx, y-cy) for x, y in poly)


class Obstacles:
    """World convex polygons (walls as rects, items), with a bounding-circle prefilter."""
    def __init__(self, rects=(), polygons=(), bounds=None):
        self.polys = [rect_corners(r) for r in rects] + [list(p) for p in polygons]
        self.circles = [_circumradius(p) for p in self.polys]
        self.bounds = bounds

    def hit(self, poly):
        (cx, cy), r = _circumradius(poly)
        if self.bounds is not None:
            x0, x1, y0, y1 = self.bounds
            if any(not (x0 <= x <= x1 and y0 <= y <= y1) for x, y in poly):
                return True
        for poly_o, ((ox, oy), orad) in zip(self.polys, self.circles):
            if math.hypot(cx-ox, cy-oy) > r + orad:
                continue
            if polygons_overlap(poly, poly_o):
                return True
        return False


def _snap_yaw(yaw, base):
    """Index k of base + k*90deg nearest to yaw."""
    return round(_wrap(yaw - base)/(math.pi/2)) % 4


def plan_team_route(footprint, start, goal, *, rects=(), obstacles=(), bounds=None, grid=GRID_M,
                    goal_yaws=None, max_expansions=MAX_EXPANSIONS):
    """Route of item poses from start to goal for a rigid team footprint.

    footprint: harness.zone_team_footprint.TeamFootprint (parts in the item frame).
    rects: static keep-out rectangles (cx, cy, hx, hy, yaw); obstacles: world convex
    polygons (items on the floor). goal_yaws: acceptable goal yaws (item symmetry);
    default (goal[2],). Returns {'ok', 'poses', 'reason', 'expansions', 'checks'}.
    """
    obs = Obstacles(rects, obstacles, bounds)
    parts = footprint.parts
    reach = max(math.hypot(x, y) for p in parts for x, y in p)
    x0, y0, a0 = (float(v) for v in start)
    yaws = [_wrap(a0 + k*math.pi/2) for k in range(4)]
    goal_yaws = list(goal_yaws or (goal[2],))

    def placed(x, y, k):
        return [transform(p, (x, y, yaws[k])) for p in parts]

    pose_cache, turn_cache = {}, {}

    def pose_ok(x, y, k, cache=pose_cache):
        key = (round(x, 4), round(y, 4), k)
        if key not in cache:
            cache[key] = not any(obs.hit(p) for p in placed(x, y, k))
        return cache[key]

    def move_ok(p, q, k):
        a, b = placed(p[0], p[1], k), placed(q[0], q[1], k)
        return all(not obs.hit(convex_hull(pa + pb)) for pa, pb in zip(a, b))

    def turn_ok(x, y, cache=turn_cache):
        key = (round(x, 4), round(y, 4))
        if key not in cache:
            cache[key] = not obs.hit(circle(x, y, reach))
        return cache[key]

    # goal: the acceptable yaw nearest a lattice heading; a small final turn if needed
    best = min(goal_yaws, key=lambda g: min(abs(_wrap(g - y)) for y in yaws))
    gk = _snap_yaw(best, a0)
    gx, gy = float(goal[0]), float(goal[1])
    final_turn = _wrap(best - yaws[gk])

    def cell(x, y):
        return (round((x-x0)/grid), round((y-y0)/grid))

    def point(c):
        return (x0 + c[0]*grid, y0 + c[1]*grid)

    # State: (i, j, heading k, last translation direction d); d = 4 after a turn or at the start.
    DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))
    start_node = (0, 0, 0, 4)
    if not pose_ok(x0, y0, 0):
        return {'ok': False, 'reason': 'start pose not clear', 'poses': [], 'expansions': 0, 'checks': []}
    gc = cell(gx, gy)
    goal_c = point(gc)

    def h(node):
        x, y = point(node[:2])
        turns = min((node[2]-gk) % 4, (gk-node[2]) % 4)
        return abs(x-goal_c[0]) + abs(y-goal_c[1]) + turns*TURN_COST_M

    def search(bend, budget):
        """A*; bend=False drops the direction from the state (fast, proves infeasibility)."""
        root = start_node if bend else start_node[:3] + (4,)
        frontier = [(h(root), 0., root)]
        came, cost = {root: None}, {root: 0.}
        expansions = 0
        while frontier and expansions < budget:
            _, g, node = heapq.heappop(frontier)
            if g > cost.get(node, math.inf):
                continue
            expansions += 1
            if node[:2] == gc and node[2] == gk:
                return node, came, expansions, 'found'
            x, y = point(node[:2])
            for d, (dx, dy) in enumerate(DIRS):
                nxt = (node[0]+dx, node[1]+dy, node[2], d if bend else 4)
                q = point(nxt[:2])
                new = g + grid + (BEND_COST_M if bend and node[3] not in (4, d) else 0.)
                if (new >= cost.get(nxt, math.inf) or not pose_ok(q[0], q[1], node[2])
                        or not move_ok((x, y), q, node[2])):
                    continue
                cost[nxt], came[nxt] = new, node
                heapq.heappush(frontier, (new + h(nxt), new, nxt))
            for dk in (1, 3):
                nxt = (node[0], node[1], (node[2]+dk) % 4, 4)
                new = g + TURN_COST_M
                if new >= cost.get(nxt, math.inf) or not turn_ok(x, y) or not pose_ok(x, y, nxt[2]):
                    continue
                cost[nxt], came[nxt] = new, node
                heapq.heappush(frontier, (new + h(nxt), new, nxt))
        return None, came, expansions, ('exhausted' if not frontier else 'budget')

    found, came, expansions, status = search(False, max_expansions)
    if found is None:
        return {'ok': False, 'reason': ('no collision-free route (search exhausted)' if status == 'exhausted'
                                        else 'route search budget exceeded'),
                'poses': [], 'expansions': expansions, 'checks': []}
    straight, came2, expansions2, _ = search(True, max_expansions)
    expansions += expansions2
    if straight is not None:
        found, came = straight, came2
    nodes = []
    n = found
    while n is not None:
        nodes.append(n)
        n = came[n]
    nodes.reverse()
    poses = [(*point(nd[:2]), yaws[nd[2]]) for nd in nodes]
    poses[0] = (x0, y0, a0)
    # lattice goal cell -> exact goal (x then y, both checked), then the final small turn
    last = poses[-1]
    tail = [(gx, last[1], last[2]), (gx, gy, last[2])]
    for p, q in zip([last] + tail[:1], tail):
        if not move_ok(p[:2], q[:2], gk):
            return {'ok': False, 'reason': 'last approach to the landing pose is blocked', 'poses': [],
                    'expansions': expansions, 'checks': []}
    poses += tail
    if abs(final_turn) > 1e-6:
        if not turn_ok(gx, gy):
            return {'ok': False, 'reason': 'final turn at the landing pose is blocked', 'poses': [],
                    'expansions': expansions, 'checks': []}
        poses.append((gx, gy, _wrap(yaws[gk] + final_turn)))
    poses = compress(poses)
    checks = verify_route(poses, footprint, rects, obstacles, bounds)
    ok = all(c['clear'] for c in checks)
    return {'ok': ok, 'reason': None if ok else 'sampled swept check failed on a leg', 'poses': poses,
            'expansions': expansions, 'checks': checks}


def compress(poses):
    """Merge collinear translations; drop zero-length legs."""
    out = [tuple(poses[0])]
    for p in poses[1:]:
        p = tuple(p)
        if math.dist(p[:2], out[-1][:2]) < 1e-9 and abs(_wrap(p[2]-out[-1][2])) < 1e-9:
            continue
        if len(out) >= 2:
            a, b = out[-2], out[-1]
            same_yaw = abs(_wrap(a[2]-b[2])) < 1e-9 and abs(_wrap(b[2]-p[2])) < 1e-9
            d1 = (b[0]-a[0], b[1]-a[1])
            d2 = (p[0]-b[0], p[1]-b[1])
            cross = d1[0]*d2[1] - d1[1]*d2[0]
            dot = d1[0]*d2[0] + d1[1]*d2[1]
            if same_yaw and abs(cross) < 1e-9 and dot > 0 and math.hypot(*d1) > 0:
                out[-1] = p
                continue
        out.append(p)
    return out


def verify_route(poses, footprint, rects, obstacles=(), bounds=None):
    """Sampled swept check (static_keepouts) of every leg: walls/bounds, and item obstacles."""
    item_rects = []
    for poly in obstacles:
        (cx, cy), r = _circumradius(poly)
        item_rects.append((cx, cy, r, r, 0.))          # conservative square around each item part
    checks = []
    for a, b in zip(poses, poses[1:]):
        walls = swept_clear(a, b, footprint, rects, bounds=bounds)
        items = swept_clear(a, b, footprint, item_rects) if item_rects else True
        checks.append({'from': [round(v, 4) for v in a], 'to': [round(v, 4) for v in b],
                       'kind': 'turn' if math.dist(a[:2], b[:2]) < 1e-9 else 'move',
                       'clear_walls_bounds': bool(walls), 'clear_items_conservative': bool(items),
                       'clear': bool(walls)})
    return checks


class PoseReference:
    """Reference item pose along a route of poses (straight moves, in-place turns)."""
    def __init__(self, poses, *, speed=REF_SPEED_M_S, yaw_rate=REF_YAW_RATE):
        self.poses = [tuple(p) for p in poses]
        self.speed, self.yaw_rate = speed, yaw_rate
        self.leg, self.u = 0, 0.
        self.pose = self.poses[0]

    @property
    def finished(self):
        return self.leg >= len(self.poses) - 1

    def _leg_len(self, i=None):
        i = self.leg if i is None else i
        a, b = self.poses[i], self.poses[i+1]
        d = math.dist(a[:2], b[:2])
        if d > 1e-9:
            return max(d/self.speed, abs(_wrap(b[2]-a[2]))/self.yaw_rate)
        return max(abs(_wrap(b[2]-a[2]))/self.yaw_rate, 1e-6)

    def velocity(self):
        if self.finished:
            return 0., 0., 0.
        a, b = self.poses[self.leg], self.poses[self.leg+1]
        T = self._leg_len()
        return (b[0]-a[0])/T, (b[1]-a[1])/T, _wrap(b[2]-a[2])/T

    def advance(self, dt):
        while dt > 1e-12 and not self.finished:
            T = self._leg_len()
            step = min(dt, (1-self.u)*T)
            self.u += step/T
            dt -= step
            if self.u >= 1-1e-9:
                self.leg, self.u = self.leg+1, 0.
        if self.finished:
            self.pose = self.poses[-1]
        else:
            a, b = self.poses[self.leg], self.poses[self.leg+1]
            self.pose = (a[0]+(b[0]-a[0])*self.u, a[1]+(b[1]-a[1])*self.u, _wrap(a[2]+_wrap(b[2]-a[2])*self.u))

    def lookahead(self, seconds, step=.5):
        """Reference poses over the next seconds (for runtime clearance checks)."""
        saved = (self.leg, self.u, self.pose)
        out = [self.pose]
        t = 0.
        while t < seconds and not self.finished:
            self.advance(step)
            t += step
            out.append(self.pose)
        self.leg, self.u, self.pose = saved
        return out

    @property
    def total_s(self):
        return sum(self._leg_len(i) for i in range(len(self.poses)-1))


__all__ = ['plan_team_route', 'verify_route', 'compress', 'PoseReference', 'Obstacles', 'GRID_M']
