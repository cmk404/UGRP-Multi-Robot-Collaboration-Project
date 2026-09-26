"""Environment v3 AprilTag placement RULE ``ugrp.zone_tag_rule.v3`` (sparse, site based).

2026-09-26 user request ("벽에 뭐가 저렇게 많아야 하나"): the tags_v1/v2 maps put a
72 mm tag every 0.3-0.5 m on both faces of every wall (70-87 tags). v3 keeps
tags only where a robot needs a fix: door frames, pickup bays, zones (entrance
and slots) and, on the corridor map, the corridor entrance, the lane axis and
the passing bay. The rule below was written and committed BEFORE any v3
localization run (experiments/2026-09-26-zone-env-v3/tag_rule_v3.md); later
changes are only allowed as recorded amendments that name where and why.

A *site* is one vertical column of three tag36h11 tags (black square 0.072 m,
white plate 0.090 m, both unchanged from v1/v2 so the detector calibration
still applies) centred at z = 0.05, 0.15 and 0.25 m on one wall face. These are
the v1 wall height plus the two v2 door-post heights: the validated postures see
at least one of them (PR #176 carry-view probe): SEARCH_POSE and level carry
drive see 0.05/0.15, LOOK_P20 with a box held sees 0.25 (0.15 beyond ~2 m).

Sites, in id order (all positions are functions of the static map only):

S1 door frames. For every ``door`` passage edge that ends an interior wall
   (edges on a perimeter wall have no free wall end and get none, as the v2
   posts), one site on EACH long face of that wall, centred 0.05 m from the edge
   (the v2 door-post position): 2 sites per interior door edge.
S5 corridors. For every ``corridor`` passage: (a) entrance frame: every wall
   face that ends at a corner of a lane opening and faces away from the lane
   (the side a robot approaches from) gets a site 0.05 m from that end; (b) lane
   axis: from each lane end, the first wall face straight ahead along the lane
   axis at the lane centre line, if it is at most 2.0 m away. For every
   ``passing_bay``: one site on its back wall (the face opposite its opening),
   at the bay centre.
S2 pickup bays. The pickup region is split into 2 bays (P1 west, P2 east) x 3
   slots (south to north), as the study map projection does
   (harness/zone_map_schematic.pickup_bays, PR #190). For every bay slot, one
   site at the nearest point of the nearest wall face that the slot centre faces
   with a clear 2-D line of sight, excluding the west perimeter wall (by the
   approach convention robots face east in the pickup area; spawns stand with
   their backs to that wall). The point is kept 0.075 m from face ends.
S3 zones. For every zone A/B/C, one site on the first wall face straight ahead
   (+x, the placement direction of the approach convention) of the zone's middle
   slot, at that slot's y. It serves the zone entrance and its three slots.

Merging: a new site within 0.30 m of an earlier site on the same wall face is
not added (it is served by that site; the record keeps ``served_by``).

Nothing here reads simulator state.
"""
from __future__ import annotations

import copy
import math

RULE_ID = 'ugrp.zone_tag_rule.v3'
COLUMN_HEIGHTS_M = (.05, .15, .25)
FRAME_OFFSET_M = .05          # v2 door-post centre: 0.05 m outside the door edge
END_MARGIN_M = .03            # plate half + this from the end of a face (v1 value)
MERGE_RADIUS_M = .30
PICKUP_BAY_COLUMNS, PICKUP_BAY_ROWS = 2, 3
LANE_AXIS_MAX_M = 2.0
CORNER_TOL_M = .03
EXCLUDED_PICKUP_FACES = ('wall_west',)
PLACEMENT_V3 = {
    'rule': RULE_ID, 'size_m': .072, 'plate_m': .090, 'plate_thickness_m': .001, 'cell_thickness_m': .001,
    'column_heights_m': list(COLUMN_HEIGHTS_M), 'frame_offset_from_edge_m': FRAME_OFFSET_M,
    'end_margin_m': END_MARGIN_M, 'merge_radius_m': MERGE_RADIUS_M,
    'pickup_bay_grid': [PICKUP_BAY_COLUMNS, PICKUP_BAY_ROWS], 'lane_axis_max_m': LANE_AXIS_MAX_M,
    'excluded_pickup_faces': list(EXCLUDED_PICKUP_FACES),
    'sites': ('S1 door frames (both faces of every interior door edge), S5 corridor entrance/lane axis/passing '
              'bay, S2 pickup bay slots (nearest faced wall, not the west wall), S3 zones (wall ahead of the '
              'middle slot); one column of 3 tags per site'),
    'rule_doc': 'experiments/2026-09-26-zone-env-v3/tag_rule_v3.md',
}


def _round(v):
    return round(float(v), 4)


def walls(static):
    return [o for o in static['obstacles'] if o.get('kind') == 'wall' and not o.get('yaw_rad')]


def _box(wall):
    (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
    return cx - hx, cx + hx, cy - hy, cy + hy


def faces(static):
    """Every long wall face whose front is inside the map bounds, with its free intervals.

    Same geometry as sim.zone_landmarks._faces/_free_intervals: 'x' faces lie on
    y = coord and run along x (normal (0, +-1)); 'y' faces lie on x = coord.
    """
    x0, x1, y0, y1 = static['bounds_m']
    out = []
    for wall in walls(static):
        (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
        if hx >= hy:
            for sign in (-1, 1):
                y = cy + sign*hy
                if y0 < y + sign*.05 < y1:
                    out.append({'wall': wall['id'], 'axis': 'x', 'coord': round(y, 4), 'normal': (0, sign),
                                'lo': cx - hx, 'hi': cx + hx})
        else:
            for sign in (-1, 1):
                x = cx + sign*hx
                if x0 < x + sign*.05 < x1:
                    out.append({'wall': wall['id'], 'axis': 'y', 'coord': round(x, 4), 'normal': (sign, 0),
                                'lo': cy - hy, 'hi': cy + hy})
    for face in out:
        face['free'] = _free(static, face)
    return out


def _free(static, face):
    free = [(face['lo'], face['hi'])]
    for wall in walls(static):
        if wall['id'] == face['wall']:
            continue
        bx0, bx1, by0, by1 = _box(wall)
        if face['axis'] == 'x':
            s0, s1 = sorted((face['coord'], face['coord'] + face['normal'][1]*.01))
            touches, a, b = by0 < s1 and by1 > s0, bx0, bx1
        else:
            s0, s1 = sorted((face['coord'], face['coord'] + face['normal'][0]*.01))
            touches, a, b = bx0 < s1 and bx1 > s0, by0, by1
        if touches:
            free = [p for lo, hi in free for p in ((lo, min(hi, a)), (max(lo, b), hi)) if p[1] - p[0] > 1e-9]
    return free


def _point(face, s):
    return (s, face['coord']) if face['axis'] == 'x' else (face['coord'], s)


def _along(face, xy):
    return xy[0] if face['axis'] == 'x' else xy[1]


def _segment_hits_box(p, q, box, eps=1e-9):
    """True if the open segment p->q passes through the interior of an axis-aligned box."""
    (x0, x1, y0, y1) = box
    t0, t1 = 0., 1.
    for a, d, lo, hi in ((p[0], q[0] - p[0], x0, x1), (p[1], q[1] - p[1], y0, y1)):
        if abs(d) < eps:
            if not lo + eps < a < hi - eps:
                return False
            continue
        ta, tb = sorted(((lo - a)/d, (hi - a)/d))
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 >= t1 - eps:
            return False
    return t1 > eps and t0 < 1 - eps


def line_of_sight(static, p, q):
    return not any(_segment_hits_box(p, q, _box(w)) for w in walls(static))


def pickup_bays(static):
    """P1..Pn west to east, slots P<n>-1..P<n>-m south to north over regions.pickup."""
    (cx, cy), (hx, hy) = static['regions']['pickup']['center_m'], static['regions']['pickup']['half_extents_m']
    bay_w, slot_h = 2*hx/PICKUP_BAY_COLUMNS, 2*hy/PICKUP_BAY_ROWS
    bays = []
    for col in range(PICKUP_BAY_COLUMNS):
        bx = cx - hx + bay_w*(col + .5)
        bay = {'bay_id': f'P{col + 1}', 'center_m': [_round(bx), _round(cy)],
               'half_extents_m': [_round(bay_w/2), _round(hy)], 'slots': []}
        for row in range(PICKUP_BAY_ROWS):
            sy = cy - hy + slot_h*(row + .5)
            bay['slots'].append({'slot_id': f'P{col + 1}-{row + 1}', 'center_m': [_round(bx), _round(sy)],
                                 'half_extents_m': [_round(bay_w/2), _round(slot_h/2)]})
        bays.append(bay)
    return bays


class _Sites:
    def __init__(self, plate):
        self.sites, self.merged, self.plate = [], [], plate

    def add(self, kind, face, s, purpose, **extra):
        xy = _point(face, s)
        for site in self.sites:
            if site['wall'] == face['wall'] and tuple(site['normal_xy']) == tuple(face['normal']) and \
                    abs(_along(face, site['center_m']) - s) < MERGE_RADIUS_M - 1e-9:
                self.merged.append({'kind': kind, 'purpose': purpose, 'wall': face['wall'],
                                    'normal_xy': list(face['normal']), 'center_m': [_round(xy[0]), _round(xy[1])],
                                    'served_by': site['id'], **extra})
                return site
        site = {'id': f'site_{len(self.sites):02d}', 'kind': kind, 'purpose': purpose, 'wall': face['wall'],
                'normal_xy': list(face['normal']), 'center_m': [_round(xy[0]), _round(xy[1])], **extra}
        self.sites.append(site)
        return site


def _face_at(faces_, wall_id, normal):
    return next(f for f in faces_ if f['wall'] == wall_id and tuple(f['normal']) == tuple(normal))


def _fits(face, s, half):
    return any(lo + half - 1e-9 <= s <= hi - half + 1e-9 for lo, hi in face['free'])


def door_frame_sites(static, faces_, sites):
    half = sites.plate/2
    for door in (p for p in static.get('passages', []) if p['kind'] == 'door'):
        (cx, cy), width = door['center_m'], door['width_m']
        across = 1 if door['axis'] == 'x' else 0          # coordinate index along the opening
        centre = (cx, cy)[across]
        for k, edge in enumerate((centre - width/2, centre + width/2)):
            direction = -1 if k == 0 else 1
            s = edge + direction*FRAME_OFFSET_M
            probe = (cx, s) if across == 1 else (s, cy)
            wall = next((w for w in walls(static) if w['id'] not in ('wall_north', 'wall_south', 'wall_west',
                                                                        'wall_east')
                         and _box(w)[0] < probe[0] < _box(w)[1] and _box(w)[2] < probe[1] < _box(w)[3]), None)
            if wall is None:
                continue          # edge on a perimeter wall: no free wall end
            normals = ((-1, 0), (1, 0)) if door['axis'] == 'x' else ((0, -1), (0, 1))
            for normal in normals:
                face = _face_at(faces_, wall['id'], normal)
                if not _fits(face, s, half):
                    raise ValueError(f"door frame site does not fit on {wall['id']}")
                sites.add('door_frame', face, s, f"{door['id']} {'low' if k == 0 else 'high'} edge",
                          passage=door['id'])


def corridor_sites(static, faces_, sites):
    half = sites.plate/2
    for lane in (p for p in static.get('passages', []) if p['kind'] == 'corridor'):
        if lane['axis'] != 'x':
            raise ValueError('corridor rule implemented for x-axis lanes')
        (cx, cy), (hx, hy) = lane['center_m'], lane['half_extents_m']
        ends, sides = (cx - hx, cx + hx), (cy - hy, cy + hy)
        for x_end in ends:
            for y_side in sides:
                for face in faces_:
                    if face['wall'] in ('wall_north', 'wall_south', 'wall_west', 'wall_east'):
                        continue
                    for lo, hi in face['free']:
                        if face['axis'] == 'y' and abs(face['coord'] - x_end) <= CORNER_TOL_M:
                            end = hi if abs(hi - y_side) <= CORNER_TOL_M else lo if abs(lo - y_side) <= CORNER_TOL_M else None
                            point = (face['coord'], end)
                        elif face['axis'] == 'x' and abs(face['coord'] - y_side) <= CORNER_TOL_M:
                            end = hi if abs(hi - x_end) <= CORNER_TOL_M else lo if abs(lo - x_end) <= CORNER_TOL_M else None
                            point = (end, face['coord'])
                        else:
                            continue
                        if end is None:
                            continue
                        towards_lane = (cx - point[0])*face['normal'][0] + (cy - point[1])*face['normal'][1]
                        if towards_lane >= 0:
                            continue      # faces into the lane: not seen when approaching the opening
                        s = end - FRAME_OFFSET_M if end == hi else end + FRAME_OFFSET_M
                        if _fits(face, s, half):
                            sites.add('corridor_entrance', face, s, f"{lane['id']} opening at x={x_end:.3f}",
                                      passage=lane['id'])
        for x_end, direction in ((ends[0], -1), (ends[1], 1)):
            ahead = [f for f in faces_ if f['axis'] == 'y' and f['normal'][0] == -direction
                     and (f['coord'] - x_end)*direction > 0 and _fits(f, cy, half + END_MARGIN_M)
                     and line_of_sight(static, (x_end, cy), (f['coord'] - direction*.01, cy))]
            if not ahead:
                continue
            face = min(ahead, key=lambda f: abs(f['coord'] - x_end))
            if abs(face['coord'] - x_end) <= LANE_AXIS_MAX_M:
                sites.add('corridor_axis', face, cy, f"{lane['id']} lane axis beyond x={x_end:.3f}",
                          passage=lane['id'], distance_m=_round(abs(face['coord'] - x_end)))
    for bay in (p for p in static.get('passages', []) if p['kind'] == 'passing_bay'):
        target = next(p for p in static['passages'] if p['id'] == bay['opens_to'])
        (bx, by), (hx, hy) = bay['center_m'], bay['half_extents_m']
        if bay['axis'] != 'y':
            raise ValueError('passing-bay rule implemented for y-axis bays')
        back_y, normal = (by - hy, (0, 1)) if target['center_m'][1] > by else (by + hy, (0, -1))
        face = min((f for f in faces_ if f['axis'] == 'x' and tuple(f['normal']) == normal
                    and _fits(f, bx, half + END_MARGIN_M)), key=lambda f: abs(f['coord'] - back_y))
        if abs(face['coord'] - back_y) > CORNER_TOL_M:
            raise ValueError(f"no back wall for {bay['id']}")
        sites.add('passing_bay', face, bx, f"{bay['id']} back wall", passage=bay['id'])


def _nearest_on_face(face, xy, half):
    best = None
    for lo, hi in face['free']:
        a, b = lo + half + END_MARGIN_M, hi - half - END_MARGIN_M
        if b < a:
            continue
        s = min(max(_along(face, xy), a), b)
        p = _point(face, s)
        d = math.dist(p, xy)
        if best is None or d < best[0]:
            best = (d, s, p)
    return best


def pickup_sites(static, faces_, sites):
    half = sites.plate/2
    for bay in pickup_bays(static):
        for slot in bay['slots']:
            xy = tuple(slot['center_m'])
            candidates = []
            for face in faces_:
                if face['wall'] in EXCLUDED_PICKUP_FACES:
                    continue
                front = (xy[0] - _point(face, _along(face, xy))[0])*face['normal'][0] + \
                        (xy[1] - _point(face, _along(face, xy))[1])*face['normal'][1]
                if front <= 0:
                    continue              # the slot is behind this face
                best = _nearest_on_face(face, xy, half)
                if best is None:
                    continue
                d, s, p = best
                eye = (p[0] + face['normal'][0]*.01, p[1] + face['normal'][1]*.01)
                if line_of_sight(static, xy, eye):
                    candidates.append((round(d, 6), face['wall'], face['normal'], s, face))
            if not candidates:
                raise ValueError(f"no faced wall for pickup slot {slot['slot_id']}")
            d, _, _, s, face = min(candidates, key=lambda c: (c[0], c[1], c[2]))
            sites.add('pickup_bay', face, s, f"pickup bay slot {slot['slot_id']}", bay_slot=slot['slot_id'],
                      distance_m=_round(d))


def zone_sites(static, faces_, sites):
    half = sites.plate/2
    for zone in ('A', 'B', 'C'):
        slots = sorted(static['zone_slots'][zone], key=lambda s: s['center_m'][1])
        x, y = slots[len(slots)//2]['center_m']
        ahead = [f for f in faces_ if f['axis'] == 'y' and f['normal'][0] == -1 and f['coord'] > x
                 and _fits(f, y, half + END_MARGIN_M) and line_of_sight(static, (x, y), (f['coord'] - .01, y))]
        face = min(ahead, key=lambda f: f['coord'])
        sites.add('zone', face, y, f'zone {zone} entrance and slots (wall ahead of the middle slot)', zone=zone,
                  distance_m=_round(face['coord'] - x))


# --------------------------------------------------------------------------- amendment A1
# 2026-09-26, written AFTER the v3 loop test failed (experiments/2026-09-26-zone-env-v3/
# prereg_amendments.json A1) and BEFORE any run on the amended maps. It only ADDS sites,
# after every v3 site (v3 tag ids 0..n-1 keep their poses), with the same column and merge
# rule. Why (post hoc evidence, loop/failure_analysis.json and st6_candidates.json):
#  S6 the loaded robot reaches a door along the pickup-side face of the wall that ends at
#     the door edge facing the pickup region; there v2 fixes came from tags seen OVER the
#     0.10 m wall (1149 of 1257 sightings), which the 0.40 m wall hides, and a tag on that
#     face is readable only 0.4-0.9 m along it (wide pans), so sites every 0.30 m.
#  S7 in the door the only v3 tags are the far east-wall sites (2.7-3.2 m, all near the
#     optical axis, 0.2-0.4 m PnP range error): y and yaw couple. One site on the nearer
#     lateral perimeter wall, 45 deg from the door centre on the zone side, is seen from
#     the whole door region at 1.4-1.9 m.
RULE_ID_A1 = 'ugrp.zone_tag_rule.v3a1'
APPROACH_STEP_M = .30
APPROACH_REACH_M = 1.20
FLANK_BEARING_DEG = 45.
PLACEMENT_V3A1 = {
    **PLACEMENT_V3, 'rule': RULE_ID_A1, 'base_rule': RULE_ID, 'amendment': 'A1',
    'approach_step_m': APPROACH_STEP_M, 'approach_reach_m': APPROACH_REACH_M, 'flank_bearing_deg': FLANK_BEARING_DEG,
    'sites': PLACEMENT_V3['sites'] + (
        '; amendment A1 adds S6 door approach lane (pickup-side face of the wall at the door edge that faces '
        'the pickup region, every 0.30 m up to 1.20 m from the door-frame site) and S7 door exit flank (nearer '
        'lateral perimeter wall, 45 deg from the door centre on the zone side)'),
    'rule_doc': 'experiments/2026-09-26-zone-env-v3/prereg_amendments.json',
}


def _pickup_side(static, door):
    """(normal of the pickup-side face, centre line of the pickup region across the door axis)."""
    (px, py) = static['regions']['pickup']['center_m']
    if door['axis'] != 'x':
        raise ValueError('door approach rule implemented for x-axis doors')
    return ((-1, 0) if px < door['center_m'][0] else (1, 0)), py


def door_approach_sites(static, faces_, sites):
    half = sites.plate/2
    for door in (p for p in static.get('passages', []) if p['kind'] == 'door'):
        normal, centre_line = _pickup_side(static, door)
        cy, width = door['center_m'][1], door['width_m']
        if abs(centre_line - cy) <= width/2:
            continue                                   # the pickup centre line runs through the opening
        direction = -1 if centre_line < cy else 1
        edge = cy + direction*width/2
        s0 = edge + direction*FRAME_OFFSET_M
        probe = (door['center_m'][0], s0)
        wall = next((w for w in walls(static) if w['id'] not in ('wall_north', 'wall_south', 'wall_west', 'wall_east')
                     and _box(w)[0] < probe[0] < _box(w)[1] and _box(w)[2] < probe[1] < _box(w)[3]), None)
        if wall is None:
            continue                                   # that edge is a perimeter wall
        face = _face_at(faces_, wall['id'], normal)
        k = 1
        while k*APPROACH_STEP_M <= APPROACH_REACH_M + 1e-9:
            s = s0 + direction*k*APPROACH_STEP_M
            if not _fits(face, s, half + END_MARGIN_M):
                break
            sites.add('door_approach', face, s, f"{door['id']} approach lane {k*APPROACH_STEP_M:.2f} m from the frame",
                      passage=door['id'])
            k += 1


def door_flank_sites(static, faces_, sites):
    half = sites.plate/2
    for door in (p for p in static.get('passages', []) if p['kind'] == 'door'):
        normal, _ = _pickup_side(static, door)
        zone_dir = -normal[0]                          # +1: the zone side is east of the door
        cx, cy = door['center_m']
        lateral = [f for f in faces_ if f['axis'] == 'x' and f['wall'] in ('wall_north', 'wall_south')]
        face = min(lateral, key=lambda f: abs(f['coord'] - cy))
        x = cx + zone_dir*abs(face['coord'] - cy)*math.tan(math.radians(FLANK_BEARING_DEG))
        eye = (x, face['coord'] + face['normal'][1]*.01)
        if not (_fits(face, x, half + END_MARGIN_M) and line_of_sight(static, (cx + zone_dir*.05, cy), eye)):
            raise ValueError(f"no door flank site for {door['id']}")
        sites.add('door_flank', face, x, f"{door['id']} exit flank ({FLANK_BEARING_DEG:.0f} deg, zone side)",
                  passage=door['id'], distance_m=_round(math.dist((cx, cy), (x, face['coord']))))


def place_sites(static, placement=PLACEMENT_V3):
    """(sites, merged) of the v3 rule (or v3 + amendment A1) for a static map (deterministic, map only)."""
    faces_ = faces(static)
    sites = _Sites(placement['plate_m'])
    door_frame_sites(static, faces_, sites)
    corridor_sites(static, faces_, sites)
    pickup_sites(static, faces_, sites)
    zone_sites(static, faces_, sites)
    if placement.get('rule') == RULE_ID_A1:
        door_approach_sites(static, faces_, sites)
        door_flank_sites(static, faces_, sites)
    return sites.sites, sites.merged


def place_tags_v3(static, placement=PLACEMENT_V3):
    """(tags, sites, merged): one tag per (site, column height), ids in site order."""
    if placement.get('rule') not in (RULE_ID, RULE_ID_A1):
        raise ValueError('not a v3 placement')
    plate, size = placement['plate_m'], placement['size_m']
    heights = placement['column_heights_m']
    top = min(float(o['height_m']) for o in walls(static))
    if plate < size or heights[0] - plate/2 < 0 or heights[-1] + plate/2 > top:
        raise ValueError('every tag plate must stay between the floor and the wall top')
    for a, b in zip(heights, heights[1:]):
        if b - a < plate:
            raise ValueError('column plates must not overlap')
    sites, merged = place_sites(static, placement)
    tags = []
    for site in sites:
        nx, ny = site['normal_xy']
        for z in heights:
            tags.append({'id': len(tags), 'wall': site['wall'], 'site': site['id'], 'normal_xy': [nx, ny],
                         'center_m': [site['center_m'][0], site['center_m'][1], z],
                         'yaw_rad': round(math.atan2(ny, nx), 6), 'size_m': size})
        site['tag_ids'] = [t['id'] for t in tags if t['site'] == site['id']]
    return tags, copy.deepcopy(sites), copy.deepcopy(merged)
