"""Static footprints of zone items and carrying teams (convex polygons).

Phase A1 of the zone team-carry work (2026-09-25). Everything here is static
geometry from the cargo catalogue (``sim.zone_cargo``) and the measured robot
collision envelope: no simulator state and no live poses of other robots.

Output form: convex polygons in a body frame (x forward, y left), exactly what
``harness.static_keepouts.pose_clear`` / ``swept_clear`` consume (that module
comes from branch ``claude/zone-hard-routes``, commit c6cd334, not yet on main
when this was written; it is imported lazily). A team footprint is kept as a
list of convex parts (item parts, each carrier's chassis and arm) whose union
is the exact, generally non-convex shape, plus their convex hull as a single
conservative polygon for callers that accept one polygon only.

The footprint frame of a team is the item's body frame: plan with one pose
(x, y, yaw) of the item and every chassis follows (virtual structure).
"""
from __future__ import annotations

import math

from sim.zone_arena import BOX_HALF, COLORS
from sim.zone_cargo import CATALOGUE, GRASP_RADIUS_M, Grasp

# Robot collision envelope in its base frame, measured on the zone_wide scene
# (all r1 collision geoms, arm folded): x -0.096..0.119, y -0.1015..0.1015 m.
# Rounded outwards to the next millimetre (the front stays 5 mm short of a
# beam end at the beam grasp station, as in the probe).
CHASSIS_X_M = (-.100, .120)
CHASSIS_Y_M = (-.105, .105)
# At the grasp the arm reaches over the item to the grip point; the forearm
# and fingers stay within a narrow strip in front of the chassis.
ARM_X_M = (0., GRASP_RADIUS_M + .030)
ARM_HALF_W_M = .035
# Default outward growth of every part for planning (walls, peers).
TEAM_MARGIN_M = .03
CIRCLE_SIDES = 16

BOX_KINDS = tuple(sorted(COLORS))


def box_grasps():
    """Colour boxes: one role, robot facing east (the zone teacher convention)."""
    return (Grasp('west', (0., 0., .024), 0., 'box'),)


def rect(x0, x1, y0, y1):
    """Counter-clockwise axis-aligned rectangle."""
    return [(x1, y1), (x0, y1), (x0, y0), (x1, y0)]


def centred_rect(cx, cy, hx, hy, yaw=0.):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c*x - s*y, cy + s*x + c*y) for x, y in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def circle(cx, cy, r, sides=CIRCLE_SIDES):
    """Circumscribing polygon of a disc (conservative)."""
    rr = r/math.cos(math.pi/sides)
    return [(cx + rr*math.cos(2*math.pi*(i+.5)/sides), cy + rr*math.sin(2*math.pi*(i+.5)/sides))
            for i in range(sides)]


def transform(poly, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c*px - s*py, y + s*px + c*py) for px, py in poly]


def convex_hull(points):
    """Andrew's monotone chain; counter-clockwise, no repeated end point."""
    pts = sorted(set((round(float(x), 12), round(float(y), 12)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def bounds(polys):
    xs = [x for p in polys for x, _ in p]
    ys = [y for p in polys for _, y in p]
    return min(xs), max(xs), min(ys), max(ys)


def item_polygons(kind, *, grow=0.):
    """Collision parts of an item (or a colour box) as convex polygons, item frame."""
    if kind in COLORS:
        return [centred_rect(0., 0., BOX_HALF[0]+grow, BOX_HALF[1]+grow)]
    if kind not in CATALOGUE:
        raise ValueError(f'unknown item kind: {kind}')
    out = []
    for part in CATALOGUE[kind].parts:
        if not part.collision:
            continue
        cx, cy, _ = part.center
        if part.shape == 'box':
            out.append(centred_rect(cx, cy, part.size[0]+grow, part.size[1]+grow, part.yaw))
        else:
            out.append(circle(cx, cy, part.size[0]+grow))
    return out


def grasps(kind):
    if kind in COLORS:
        return box_grasps()
    if kind not in CATALOGUE:
        raise ValueError(f'unknown item kind: {kind}')
    return CATALOGUE[kind].grasps


def station_offset(kind, role):
    """Carrier base pose (x, y, yaw) in the item frame for a grasp role."""
    for g in grasps(kind):
        if g.role == role:
            return g.approach_base()
    raise ValueError(f'{kind} has no grasp role {role!r}')


def carrier_polygons(base_in_item, *, grow=0., arm=True):
    """One carrier's chassis (and grasping arm strip) placed at its base pose, item frame."""
    parts = [rect(CHASSIS_X_M[0]-grow, CHASSIS_X_M[1]+grow, CHASSIS_Y_M[0]-grow, CHASSIS_Y_M[1]+grow)]
    if arm:
        parts.append(rect(ARM_X_M[0], ARM_X_M[1]+grow, -ARM_HALF_W_M-grow, ARM_HALF_W_M+grow))
    return [transform(p, base_in_item) for p in parts]


class TeamFootprint:
    """Item + every carrier chassis/arm at its grasp station, grown by a margin.

    ``parts``: convex polygons whose union is the team shape (item frame).
    ``hull``: their convex hull, one conservative convex polygon.
    """
    def __init__(self, kind, roles, *, margin=TEAM_MARGIN_M):
        roles = tuple(roles)
        if len(set(roles)) != len(roles):
            raise ValueError('duplicate roles in a team footprint')
        self.kind, self.roles, self.margin = kind, roles, float(margin)
        self.stations = {role: station_offset(kind, role) for role in roles}
        self.item_parts = item_polygons(kind, grow=margin)
        self.carrier_parts = {role: carrier_polygons(self.stations[role], grow=margin) for role in roles}
        self.parts = self.item_parts + [p for role in roles for p in self.carrier_parts[role]]
        self.hull = convex_hull([pt for p in self.parts for pt in p])

    def at(self, pose):
        """World polygons (parts) for an item pose."""
        return [transform(p, pose) for p in self.parts]

    def hull_at(self, pose):
        return transform(self.hull, pose)

    def radius(self):
        return max(math.hypot(x, y) for x, y in self.hull)

    def record(self):
        return {'kind': self.kind, 'roles': list(self.roles), 'margin_m': self.margin,
                'frame': 'item body frame, metres (x forward, y left)',
                'stations': {r: [round(v, 6) for v in s] for r, s in self.stations.items()},
                'parts': [[[round(x, 6), round(y, 6)] for x, y in p] for p in self.parts],
                'hull': [[round(x, 6), round(y, 6)] for x, y in self.hull],
                'bounds_m': [round(v, 6) for v in bounds(self.parts)]}


def team_footprint(kind, roles=None, *, margin=TEAM_MARGIN_M):
    """The catalogue formation's footprint (all roles) unless roles are given."""
    if roles is None:
        roles = CATALOGUE[kind].formations[0] if kind in CATALOGUE else ('west',)
    return TeamFootprint(kind, roles, margin=margin)


def _keepouts(module):
    if module is not None:
        return module
    from harness import static_keepouts  # branch claude/zone-hard-routes (A2 dependency)
    return static_keepouts


def pose_clear(pose, footprint, rects, *, bounds=None, margin=0., exact=True, keepouts=None):
    """Team footprint vs static keep-out rectangles, through static_keepouts.pose_clear.

    exact: test each convex part (the union); otherwise the convex hull.
    """
    ko = _keepouts(keepouts)
    polys = footprint.parts if exact else [footprint.hull]
    return all(ko.pose_clear(pose, p, rects, bounds=bounds, margin=margin) for p in polys)


def swept_clear(start, end, footprint, rects, *, bounds=None, margin=0., exact=True, keepouts=None, **kw):
    ko = _keepouts(keepouts)
    polys = footprint.parts if exact else [footprint.hull]
    return all(ko.swept_clear(start, end, p, rects, bounds=bounds, margin=margin, **kw) for p in polys)


def point_in_convex(point, poly, *, eps=1e-9):
    """Inside or on the boundary of a convex polygon of either orientation."""
    sign = 0
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i+1) % n]
        c = (x1-x0)*(point[1]-y0) - (y1-y0)*(point[0]-x0)
        if abs(c) <= eps:
            continue
        s = 1 if c > 0 else -1
        if sign and s != sign:
            return False
        sign = s
    return True


def polygons_inside_rect(polys, center, half, *, margin=0.):
    """Every vertex of every (convex) polygon inside an axis-aligned rectangle."""
    (cx, cy), (hx, hy) = center, half
    return all(abs(x-cx) <= hx-margin+1e-12 and abs(y-cy) <= hy-margin+1e-12 for p in polys for x, y in p)


__all__ = ['TeamFootprint', 'team_footprint', 'item_polygons', 'carrier_polygons', 'station_offset', 'grasps',
           'convex_hull', 'transform', 'bounds', 'pose_clear', 'swept_clear', 'point_in_convex',
           'polygons_inside_rect', 'rect', 'centred_rect', 'circle', 'BOX_KINDS', 'CHASSIS_X_M', 'CHASSIS_Y_M',
           'ARM_X_M', 'ARM_HALF_W_M', 'TEAM_MARGIN_M']
