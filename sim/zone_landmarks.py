"""Wall AprilTag landmarks as versioned static-map features of the zone maps.

2026-09-25 user decision: tag36h11 markers may be placed on walls and door
posts as static-map features (never on cargo or robots). A tagged map is a NEW
map version (``maps/zones/<base>_tags_v1.json``) that copies its base map
byte-for-byte in content, links the base map hash, and adds one ``landmarks``
block. The base maps stay unchanged.

Placement is parameterized (size, centre height, spacing, margins) because the
recommended values depend on what the wrist camera can see; the defaults keep
the whole white plate inside the 0.10 m wall face so nothing sticks out above
the wall. Tags are drawn in MuJoCo from the same JSON as thin visual-only box
geoms (a white plate plus merged black cell runs, contype=conaffinity=0,
mass 0): the physics model is unchanged (tests compare trajectories).

The pose of every tag is map knowledge (fixed, surveyed once); it is allowed
robot input. Nothing here reads simulator state.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from sim.research_dispatch_arena import digest
from sim.session_scenes import ROOT
from sim.zone_arena import MAP_DIR, authored_map, episode
from sim.zone_scene import ZoneScene

LANDMARK_SCHEMA = 'ugrp.zone_landmarks.v1'
FAMILY = 'tag36h11'
OPENCV_DICTIONARY = 'DICT_APRILTAG_36h11'
# Defaults (2026-09-25, before the wrist-visibility study): the 8-cell black
# square is 0.072 m (9 mm cells), the white plate adds one cell on each side
# (0.090 m) and spans z 0.005..0.095 on the 0.10 m wall. The wrist camera sits
# 0.21-0.23 m high in the search and level-carry postures and sees such a tag
# from about 0.4 m out. Every free wall face gets a tag at both ends (next to
# door edges and corners) and at most spacing_m apart in between.
DEFAULT_PLACEMENT = {'size_m': .072, 'plate_m': .090, 'center_height_m': .050, 'spacing_m': .50,
                     'end_margin_m': .03, 'plate_thickness_m': .001, 'cell_thickness_m': .001,
                     'faces': 'interior walls: both faces; perimeter walls: inner face'}
# 2026-09-25 v2 (PR #176 wrist carry-view study): with a box held, the wrist
# camera sees only a ~10 deg band above the box, so near doors the robot needs
# tags higher than the 0.10 m wall. Door posts: a 0.09 m wide, 0.30 m high
# visual plate on the wall end 0.05 m outside each door edge, tags on both
# faces at 0.15 m and 0.25 m. Wall-face tags stay (0.05 m) but are 0.30 m
# apart within 1 m of a door. Posts are visual only in SIM; they stand on the
# wall footprint, which every planner keeps out of anyway.
PLACEMENT_V2 = {**DEFAULT_PLACEMENT, 'near_door_spacing_m': .30, 'near_door_radius_m': 1.0,
                'door_posts': {'offset_from_edge_m': .05, 'width_m': .09, 'height_m': .30,
                               'tag_center_heights_m': [.15, .25]}}
POST_RGBA = '.23 .28 .33 1'
TAGGED_MAPS = {
    'zone_wide_door_tags_v1': {'base': 'zone_wide_door', 'placement': DEFAULT_PLACEMENT},
    'zone_wide_two_doors_tags_v1': {'base': 'zone_wide_two_doors', 'placement': DEFAULT_PLACEMENT},
    'zone_wide_corridor_tags_v1': {'base': 'zone_wide_corridor', 'placement': DEFAULT_PLACEMENT},
    'zone_wide_door_tags_v2': {'base': 'zone_wide_door', 'placement': PLACEMENT_V2},
    'zone_wide_two_doors_tags_v2': {'base': 'zone_wide_two_doors', 'placement': PLACEMENT_V2},
}
PLATE_RGBA = '.95 .95 .95 1'
CELL_RGBA = '.02 .02 .02 1'


def tag_bits(tag_id):
    """8x8 cells (outer black border included) of one tag36h11 id; 1 = black.

    Row 0 is the top of the tag, column 0 its left edge seen from the front.
    Taken from the OpenCV dictionary that the detector uses.
    """
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, OPENCV_DICTIONARY))
    image = cv2.aruco.generateImageMarker(dictionary, int(tag_id), 8, borderBits=1)
    return [[1 if int(v) < 128 else 0 for v in row] for row in image]


def dictionary_digest(ids):
    return hashlib.sha256(json.dumps([tag_bits(i) for i in ids]).encode()).hexdigest()


def _faces(static):
    """(wall_id, axis, coordinate, normal, lo, hi) for every candidate wall face.

    axis 'x': face lies on y = coordinate and runs along x; normal is (0, +-1).
    axis 'y': face lies on x = coordinate and runs along y; normal is (+-1, 0).
    Faces whose front is outside the map bounds (outer side of the perimeter)
    are skipped.
    """
    x0, x1, y0, y1 = static['bounds_m']
    faces = []
    for wall in static['obstacles']:
        if wall.get('kind') != 'wall' or wall.get('yaw_rad'):
            continue
        (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
        if hx >= hy:
            for sign in (-1, 1):
                y = cy + sign*hy
                if y0 < y + sign*.05 < y1:
                    faces.append((wall['id'], 'x', round(y, 4), (0, sign), cx-hx, cx+hx))
        else:
            for sign in (-1, 1):
                x = cx + sign*hx
                if x0 < x + sign*.05 < x1:
                    faces.append((wall['id'], 'y', round(x, 4), (sign, 0), cy-hy, cy+hy))
    return faces


def _free_intervals(static, face):
    """Parts of a face not covered by another wall standing against it."""
    wid, axis, coord, normal, lo, hi = face
    free = [(lo, hi)]
    for wall in static['obstacles']:
        if wall['id'] == wid or wall.get('kind') != 'wall':
            continue
        (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
        # Strip 0.01 m in front of the face.
        if axis == 'x':
            s0, s1 = sorted((coord, coord + normal[1]*.01))
            touches = cy-hy < s1 and cy+hy > s0
            a, b = cx-hx, cx+hx
        else:
            s0, s1 = sorted((coord, coord + normal[0]*.01))
            touches = cx-hx < s1 and cx+hx > s0
            a, b = cy-hy, cy+hy
        if not touches:
            continue
        free = [piece for lo_, hi_ in free for piece in ((lo_, min(hi_, a)), (max(lo_, b), hi_))
                if piece[1] - piece[0] > 1e-9]
    return free


def place_tags(static, placement=DEFAULT_PLACEMENT):
    """Deterministic tag list for a static map (ids in wall/face/position order)."""
    size, plate = placement['size_m'], placement['plate_m']
    if plate < size or placement['center_height_m'] - plate/2 < 0:
        raise ValueError('plate must hold the tag and stay above the floor')
    margin = plate/2 + placement['end_margin_m']
    doors = [p for p in static.get('passages', []) if p['kind'] == 'door']
    tags = []
    for face in _faces(static):
        wid, axis, coord, normal, _, _ = face
        for lo, hi in _free_intervals(static, face):
            a, b = lo + margin, hi - margin
            if b < a:
                continue
            for s in _positions(a, b, placement, lambda v: (v, coord) if axis == 'x' else (coord, v),
                                doors):
                x, y = (s, coord) if axis == 'x' else (coord, s)
                tags.append({'id': len(tags), 'wall': wid, 'normal_xy': list(normal),
                             'center_m': [round(x, 4), round(y, 4), placement['center_height_m']],
                             'yaw_rad': round(math.atan2(normal[1], normal[0]), 6), 'size_m': size})
    for post in door_posts(static, placement):
        for normal in ((-1, 0), (1, 0)) if post['axis'] == 'x' else ((0, -1), (0, 1)):
            (cx, cy), (hx, hy) = post['center_m'], post['half_extents_m']
            fx, fy = cx + normal[0]*hx, cy + normal[1]*hy
            for z in placement['door_posts']['tag_center_heights_m']:
                tags.append({'id': len(tags), 'wall': post['wall'], 'mount': 'door_post', 'post': post['id'],
                             'normal_xy': list(normal), 'center_m': [round(fx, 4), round(fy, 4), z],
                             'yaw_rad': round(math.atan2(normal[1], normal[0]), 6), 'size_m': size})
    return tags


def _door_edges(door):
    (cx, cy), half = door['center_m'], door['width_m']/2
    return [(cx, cy - half), (cx, cy + half)] if door['axis'] == 'x' else [(cx - half, cy), (cx + half, cy)]


def _positions(a, b, placement, point, doors):
    """Tag positions along [a, b]: v1 uniform spacing, or v2 denser near doors."""
    if 'near_door_spacing_m' not in placement:
        count = 1 if b - a < 1e-9 else math.ceil((b - a)/placement['spacing_m']) + 1
        return [(a + b)/2 if count == 1 else a + (b - a)*i/(count - 1) for i in range(count)]
    if b - a < 1e-9:
        return [(a + b)/2]
    near = placement['near_door_radius_m']

    def spacing(v):
        xy = point(v)
        close = any(math.dist(xy, e) <= near for d in doors for e in _door_edges(d))
        return placement['near_door_spacing_m'] if close else placement['spacing_m']
    # walk from the end nearer a door so the dense spacing starts at the door
    forward = min((math.dist(point(a), e) for d in doors for e in _door_edges(d)), default=0.) <= \
        min((math.dist(point(b), e) for d in doors for e in _door_edges(d)), default=0.)
    out, v = [], a if forward else b
    while (v <= b + 1e-9) if forward else (v >= a - 1e-9):
        out.append(round(v, 6))
        v = v + spacing(v) if forward else v - spacing(v)
    end = b if forward else a
    if abs(out[-1] - end) > .10:
        out.append(end)
    elif len(out) > 1:
        out[-1] = end
    return sorted(out)


def door_posts(static, placement):
    """Door-post plates (v2 placement only) on wall ends next to door edges."""
    spec = placement.get('door_posts')
    if not spec:
        return []
    posts = []
    walls = [o for o in static['obstacles'] if o.get('kind') == 'wall']
    for door in (p for p in static.get('passages', []) if p['kind'] == 'door'):
        cx, cy = door['center_m']
        half = door['width_m']/2
        for sign in (-1, 1):
            if door['axis'] == 'x':
                center = (cx, cy + sign*(half + spec['offset_from_edge_m']))
            else:
                center = (cx + sign*(half + spec['offset_from_edge_m']), cy)
            wall = next((w for w in walls if abs(center[0] - w['center_m'][0]) < w['half_extents_m'][0] - 1e-9
                         and abs(center[1] - w['center_m'][1]) < w['half_extents_m'][1] - 1e-9), None)
            if wall is None:
                continue  # the opening reaches a perimeter wall: no post there
            thick = wall['half_extents_m'][0] if door['axis'] == 'x' else wall['half_extents_m'][1]
            half_xy = [thick, spec['width_m']/2] if door['axis'] == 'x' else [spec['width_m']/2, thick]
            posts.append({'id': f"post_{door['id']}_{'lo' if sign < 0 else 'hi'}", 'door': door['id'],
                          'wall': wall['id'], 'axis': door['axis'],
                          'center_m': [round(center[0], 4), round(center[1], 4)],
                          'half_extents_m': [round(v, 4) for v in half_xy], 'height_m': spec['height_m']})
    return posts


def build_tagged_map(name):
    spec = TAGGED_MAPS[name]
    base = authored_map(spec['base'])
    tags = place_tags(base, spec['placement'])
    value = copy.deepcopy(base)
    value.update(map_id=name, version=1,
                 base_map={'map_id': base['map_id'], 'version': base['version'],
                           'static_map_sha256': digest(base)})
    value['landmarks'] = {
        'schema': LANDMARK_SCHEMA, 'family': FAMILY, 'opencv_dictionary': OPENCV_DICTIONARY,
        'dictionary_bits_sha256': dictionary_digest([t['id'] for t in tags]),
        'placement': copy.deepcopy(spec['placement']),
        'tag_frame': ('tag centre on the wall face; normal_xy points out of the wall into the room '
                      '(the side a camera sees it from); tag x = image right seen from the front, '
                      'tag y = up; size_m = outer black square'),
        'tags': tags}
    posts = door_posts(base, spec['placement'])
    if posts:
        value['landmarks']['door_posts'] = posts
    return value


def tagged_map(name):
    """The committed tagged map; refuses drift from its definition or base map."""
    if name not in TAGGED_MAPS:
        raise ValueError(f'unknown tagged zone map: {name}')
    value = json.loads((MAP_DIR/(name+'.json')).read_text())
    if value != build_tagged_map(name):
        raise ValueError('tagged zone map file differs from its authored definition or base map')
    return value


def write_tagged_maps():
    """Author the JSON files once (used when a new version is created)."""
    for name in TAGGED_MAPS:
        path = MAP_DIR/(name+'.json')
        if path.exists():
            continue  # never overwrite a published version (tagged_map() verifies it)
        path.write_text(json.dumps(build_tagged_map(name), indent=2) + '\n')


def landmarks_digest(static):
    return digest(static['landmarks'])


def _box(world, name, center, half, rgba):
    ET.SubElement(world, 'geom', name=name, type='box', pos=' '.join(f'{v:.5f}' for v in center),
                  size=' '.join(f'{v:.5f}' for v in half), rgba=rgba, contype='0', conaffinity='0',
                  group='0', mass='0')


def add_tag_geoms(xml, static):
    """Append the visual-only tag geoms of ``static['landmarks']`` to a scene XML."""
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    block = static['landmarks']
    placement = block['placement']
    count = 0
    for post in block.get('door_posts', []):
        (cx, cy), (hx, hy) = post['center_m'], post['half_extents_m']
        _box(world, f"tag_{post['id']}", (cx, cy, post['height_m']/2), (hx, hy, post['height_m']/2), POST_RGBA)
        count += 1
    for tag in block['tags']:
        nx, ny = tag['normal_xy']
        cx, cy, cz = tag['center_m']
        along_x = abs(ny) > 0          # face runs along x when the normal is along y
        right = (-ny, nx)              # tag x axis seen from the front
        cell = tag['size_m']/8
        tp, tc = placement['plate_thickness_m'], placement['cell_thickness_m']

        def half(width, height, thick):
            return (width/2, thick/2, height/2) if along_x else (thick/2, width/2, height/2)

        def at(offset_right, z, depth):
            return (cx + right[0]*offset_right + nx*depth, cy + right[1]*offset_right + ny*depth, z)

        plate = placement['plate_m']
        _box(world, f"tag_{tag['id']:03d}_plate", at(0., cz, tp/2), half(plate, plate, tp), PLATE_RGBA)
        count += 1
        for r, row in enumerate(tag_bits(tag['id'])):
            c = 0
            while c < 8:
                if not row[c]:
                    c += 1
                    continue
                start = c
                while c < 8 and row[c]:
                    c += 1
                width = (c - start)*cell
                offset = ((start + c)/2 - 4)*cell
                z = cz + (3.5 - r)*cell
                _box(world, f"tag_{tag['id']:03d}_r{r}c{start}", at(offset, z, tp + tc/2),
                     half(width, cell, tc), CELL_RGBA)
                count += 1
    return ET.tostring(root, encoding='unicode'), count


class TaggedZoneScene(ZoneScene):
    """ZoneScene on a tagged map: base-map episode, same robots and physics,
    plus the tag geoms. Selection id: ``zones/<tagged map id>``."""

    @classmethod
    def from_tagged(cls, name, seed, goal, extra_boxes=None, contact_profile=None, base_dir=ROOT):
        selected = {'layout': 'zones/' + name, 'seed': seed, 'map_file': None, 'cargo_ids': None,
                    'robots': {}, 'objects': [], 'builder': None, 'contact_profile': contact_profile,
                    'params': {'goal': goal, 'extra_boxes': extra_boxes or {}}}
        return cls(selected, base_dir)

    def _resolve(self):
        family, name = self.selection.split('/', 1)
        if family != 'zones' or name not in TAGGED_MAPS:
            raise ValueError(f'unknown tagged zone scene: {self.selection}')
        self._read(MAP_DIR/(name+'.json'))
        base = TAGGED_MAPS[name]['base']
        self._read(MAP_DIR/(base+'.json'))
        params = self.scene.get('params') or {}
        config = episode(base, self.scene['seed'], goal=params['goal'], extra_boxes=params.get('extra_boxes'))
        static = tagged_map(name)
        config.update(static_map=static, static_map_sha256=digest(static), tagged_map_id=name)
        config['extra_boxes'] = params.get('extra_boxes') or {}
        self.config = config
        self.bounds = static['bounds_m']
        self.inventory = list(config['setup_only']['objects'])
        self._verify_camera()

    def transform(self, xml):
        xml = super().transform(xml)
        xml, count = add_tag_geoms(xml, self.config['static_map'])
        static = self.config['static_map']
        self.manifest.update(
            scene_xml_sha256=hashlib.sha256(xml.encode()).hexdigest(),
            static_map_sha256=digest(static),
            base_static_map_sha256=static['base_map']['static_map_sha256'],
            landmarks_sha256=landmarks_digest(static),
            tag_count=len(static['landmarks']['tags']), tag_geoms=count,
            tag_geom_physics='visual only: contype=0 conaffinity=0 mass=0')
        return xml
