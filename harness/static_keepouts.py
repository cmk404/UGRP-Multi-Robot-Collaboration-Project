"""Static rectangle keep-outs of an authored map, for any planner or checker.

One definition of the fixed walls serves both the single-robot disc planner
(``rect_distance``) and footprint checks of larger loads (``pose_clear`` /
``swept_clear`` with a convex polygon footprint), so a later team-carry
executor does not keep a second wall list. Everything here is static-map
geometry: no simulator state, no live poses of other robots.

Rectangles are ``(cx, cy, hx, hy, yaw)`` in world metres/radians. Footprints
are convex polygons in the body frame (x forward, y left), counter-clockwise
or clockwise. The separating-axis test generalises
``harness.pair_navigation.footprint_clear`` (oriented load rectangle vs
axis-aligned walls) to any convex footprint and rotated walls.
"""
from __future__ import annotations

import math

# The four perimeter walls every zone map has. Their inner faces are the map
# bounds that the disc planner already respects, so they are not interior
# keep-outs (keeping v1 plans byte-identical).
PERIMETER_WALLS = frozenset({'wall_north', 'wall_south', 'wall_west', 'wall_east'})


def keepout_rects(static_map, *, perimeter=False):
    """Oriented rectangles of the map's fixed obstacles (interior only by default)."""
    return tuple((float(o['center_m'][0]), float(o['center_m'][1]), float(o['half_extents_m'][0]),
                  float(o['half_extents_m'][1]), float(o.get('yaw_rad', 0.)))
                 for o in static_map.get('obstacles', ()) if perimeter or o['id'] not in PERIMETER_WALLS)


def _local(point, rect):
    cx, cy, _, _, yaw = rect
    dx, dy = point[0]-cx, point[1]-cy
    if yaw:
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = c*dx + s*dy, -s*dx + c*dy
    return dx, dy


def rect_distance(point, rect):
    """Euclidean distance from a point to a (filled) oriented rectangle; 0 inside."""
    dx, dy = _local(point, rect)
    return math.hypot(max(abs(dx)-rect[2], 0.), max(abs(dy)-rect[3], 0.))


def inside_rect(point, rect, *, grow=0.):
    dx, dy = _local(point, rect)
    return abs(dx) <= rect[2]+grow and abs(dy) <= rect[3]+grow


def rect_corners(rect, *, margin=0.):
    cx, cy, hx, hy, yaw = rect
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = hx+margin, hy+margin
    return [(cx + c*x - s*y, cy + s*x + c*y) for x, y in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def polygon_at(pose, footprint):
    """Body-frame convex polygon placed at pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c*px - s*py, y + s*px + c*py) for px, py in footprint]


def _axes(polygon):
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        ex, ey = x1-x0, y1-y0
        norm = math.hypot(ex, ey)
        if norm > 1e-12:
            yield (-ey/norm, ex/norm)


def polygons_overlap(a, b):
    """Separating-axis test for two convex polygons (touching counts as overlap)."""
    for ax, ay in (*_axes(a), *_axes(b)):
        pa = [ax*x + ay*y for x, y in a]
        pb = [ax*x + ay*y for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def pose_clear(pose, footprint, rects, *, bounds=None, margin=0.):
    """True when the footprint at pose overlaps no rectangle (grown by margin)
    and, with bounds [x0, x1, y0, y1], stays inside them by margin."""
    polygon = polygon_at(pose, footprint)
    if bounds is not None:
        x0, x1, y0, y1 = bounds
        if any(not (x0+margin <= x <= x1-margin and y0+margin <= y <= y1-margin) for x, y in polygon):
            return False
    return not any(polygons_overlap(polygon, rect_corners(r, margin=margin)) for r in rects)


def _wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def swept_clear(start, end, footprint, rects, *, bounds=None, margin=0., step_m=.012,
                step_rad=math.radians(2)):
    """Sampled swept check of a straight translation plus shortest rotation.

    As in ``pair_navigation.swept_clear``, the margin must also cover the
    motion between samples (at most step_m of translation and step_rad of
    rotation times the footprint radius).
    """
    delta = _wrap(end[2]-start[2])
    count = max(1, math.ceil(math.dist(start[:2], end[:2])/step_m), math.ceil(abs(delta)/step_rad))
    return all(pose_clear((start[0]+(end[0]-start[0])*t/count, start[1]+(end[1]-start[1])*t/count,
                           start[2]+delta*t/count), footprint, rects, bounds=bounds, margin=margin)
               for t in range(count+1))


def disc_footprint(radius, sides=16):
    """Circumscribing polygon of a disc (conservative for SAT checks)."""
    r = radius/math.cos(math.pi/sides)
    return [(r*math.cos(2*math.pi*(i+.5)/sides), r*math.sin(2*math.pi*(i+.5)/sides)) for i in range(sides)]


MOUTH_M = {'door': .50, 'corridor': .35}


def passage_zones(static_map, *, side_m=.3):
    """Single-lane passages from the static map as (passage_id, core_rect, zone_rect).

    core: the passage opening itself. zone: the core grown along the travel
    axis by the mouth length of its kind (a door's wall is thin, so its mouths
    are longer; a corridor's mouths stay short of the slots beyond it) and by
    side_m across it: where two robots can block each other.
    """
    out = []
    for p in static_map.get('passages', ()):
        if p.get('lanes') != 1 or p.get('kind') == 'passing_bay':
            continue
        (cx, cy), (hx, hy) = p['center_m'], p['half_extents_m']
        mouth_m = MOUTH_M[p['kind']]
        grow = (mouth_m, side_m) if p['axis'] == 'x' else (side_m, mouth_m)
        out.append((p['id'], (cx, cy, hx, hy, 0.), (cx, cy, hx+grow[0], hy+grow[1], 0.)))
    return out


__all__ = ['PERIMETER_WALLS', 'keepout_rects', 'rect_distance', 'inside_rect', 'rect_corners', 'polygon_at',
           'polygons_overlap', 'pose_clear', 'swept_clear', 'disc_footprint', 'passage_zones']
