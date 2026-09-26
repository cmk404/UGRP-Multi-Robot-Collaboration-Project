"""Static-map serializer and schematic renderer for the zone dialogue study (package A).

Read-only over ``maps/zones/*.json`` (the authored maps and the ``*_tags_v1``
AprilTag variants). Nothing here imports a simulator, opens MuJoCo or reads any
live state: the projection and the PNG are functions of the map FILE alone, so
they are identical on every call of a run.

What a robot may receive from this module:

* ``public_map(data)`` -- a compact JSON projection: perimeter/interior walls,
  doors and corridors, the pickup region with derived coarse bays/slots, zones
  ``A``/``B``/``C`` with their landing slots, static terrain, the item kinds the
  map paints and (when the map file has them) the surveyed AprilTag landmarks.
* ``render_schematic(data)`` -- a floor plan drawn FROM THE MAP FILE with PIL.
  It is not a camera image: no robot, no cargo, no occupancy, no discovered
  obstacle is ever drawn.

Deliberately dropped: ``top_cameras``. The TOP views are evaluation-only, so
their calibration never reaches a robot payload
(``harness.zone_study_contract`` rejects the key).

Hashes: ``map_bundle`` records the map file SHA-256, the projection SHA-256, the
schematic PNG/pixel SHA-256 and the renderer version, and carries the tag
variant's ``base_map.static_map_sha256`` through, so a run bundle can pin the
prompt input to the physical map file (docs/execution_versioning.md).

Frames: map JSON is ``world metres; x east, y north``. The schematic is drawn
with +x right and +y up (north up), origin at the map bounds.
"""
from __future__ import annotations

import hashlib
import io
import json
from collections.abc import Mapping, Sequence
from pathlib import Path

import PIL
from PIL import Image, ImageDraw, ImageFont

MAP_DIR = Path(__file__).resolve().parents[1] / 'maps' / 'zones'
PUBLIC_MAP_SCHEMA = 'ugrp.zone_public_map.v1'
MAP_BUNDLE_SCHEMA = 'ugrp.zone_map_bundle.v1'
ARENA_SCHEMA = 'ugrp.zone_arena.v1'
LANDMARK_SCHEMA = 'ugrp.zone_landmarks.v1'
ZONE_IDS = ('A', 'B', 'C')
PERIMETER_WALLS = ('wall_north', 'wall_south', 'wall_west', 'wall_east')
LANDMARK_DETAIL = ('full', 'summary', 'none')
# Coarse pickup grid derived from the map's pickup region only: bays P1..Pn west
# to east, each split into slots P<n>-1..P<n>-m south to north. Coarse on
# purpose -- an order sheet names where an item was PUT AT SETUP, never where it
# is now, and the robot still has to find it with its own camera.
PICKUP_BAY_COLUMNS = 2
PICKUP_BAY_ROWS = 3
# Every 4th tag id is labelled in the schematic; all of them are in the JSON.
TAG_LABEL_STRIDE = 4
ROUND_M = 4


def digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def digest_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _round(value):
    """Round every float to ROUND_M decimals (kills 2.1750000000000003 noise)."""
    if isinstance(value, bool):
        return value
    if isinstance(value, float):
        return round(value, ROUND_M)
    if isinstance(value, Mapping):
        return {k: _round(v) for k, v in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [_round(v) for v in value]
    return value


def map_path(map_id: str, *, maps_dir: Path | str = MAP_DIR) -> Path:
    if not isinstance(map_id, str) or not map_id or '/' in map_id or map_id.startswith('.'):
        raise ValueError(f'bad map_id: {map_id!r}')
    return Path(maps_dir) / (map_id + '.json')


def load_map(map_id: str, *, maps_dir: Path | str = MAP_DIR) -> tuple[dict, str]:
    """(map file JSON, file SHA-256). Read-only; the file is never rewritten."""
    path = map_path(map_id, maps_dir=maps_dir)
    raw = path.read_bytes()
    data = json.loads(raw)
    if data.get('schema') != ARENA_SCHEMA:
        raise ValueError(f'{path.name}: expected {ARENA_SCHEMA}, got {data.get("schema")!r}')
    if data.get('map_id') != map_id:
        raise ValueError(f'{path.name}: map_id {data.get("map_id")!r} differs from the file name')
    return data, digest_bytes(raw)


def pickup_bays(data: Mapping) -> list[dict]:
    """Coarse bays/slots derived from ``regions.pickup`` (deterministic, map-only)."""
    region = data['regions']['pickup']
    (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
    bay_w, slot_h = 2. * hx / PICKUP_BAY_COLUMNS, 2. * hy / PICKUP_BAY_ROWS
    bays = []
    for col in range(PICKUP_BAY_COLUMNS):
        bx = cx - hx + bay_w * (col + .5)
        bay = {'bay_id': f'P{col + 1}', 'center_m': [bx, cy], 'half_extents_m': [bay_w / 2., hy], 'slots': []}
        for row in range(PICKUP_BAY_ROWS):
            sy = cy - hy + slot_h * (row + .5)
            bay['slots'].append({'slot_id': f'P{col + 1}-{row + 1}', 'center_m': [bx, sy],
                                 'half_extents_m': [bay_w / 2., slot_h / 2.]})
        bays.append(bay)
    return _round(bays)


def bay_ids(data: Mapping) -> tuple[frozenset[str], frozenset[str]]:
    """(bay ids, slot ids) a scenario config may name."""
    bays = pickup_bays(data)
    return frozenset(b['bay_id'] for b in bays), frozenset(s['slot_id'] for b in bays for s in b['slots'])


def _walls(data: Mapping) -> list[dict]:
    out = []
    for obstacle in data.get('obstacles', ()):
        wall = {'id': obstacle['id'], 'center_m': obstacle['center_m'],
                'half_extents_m': obstacle['half_extents_m'], 'height_m': obstacle['height_m'],
                'perimeter': obstacle['id'] in PERIMETER_WALLS}
        if 'yaw_rad' in obstacle:
            wall['yaw_rad'] = obstacle['yaw_rad']
        out.append(wall)
    return out


def _passages(data: Mapping) -> list[dict]:
    keys = ('id', 'kind', 'center_m', 'half_extents_m', 'width_m', 'lanes', 'axis', 'connects', 'opens_to')
    return [{k: p[k] for k in keys if k in p} for p in data.get('passages', ())]


def _regions(data: Mapping) -> dict:
    out = {}
    for key, region in data['regions'].items():
        name = key[5:] if key.startswith('zone_') else key
        out[name] = {'map_key': key, 'center_m': region['center_m'],
                     'half_extents_m': region['half_extents_m'], 'paint_rgba': region['rgba']}
    return out


def _landmarks(data: Mapping, detail: str) -> dict | None:
    marks = data.get('landmarks')
    if marks is None or detail == 'none':
        return None
    if marks.get('schema') != LANDMARK_SCHEMA:
        raise ValueError(f'unexpected landmark schema: {marks.get("schema")!r}')
    head = {'schema': marks['schema'], 'family': marks['family'], 'tag_frame': marks['tag_frame'],
            'placement': marks['placement'], 'tag_count': len(marks['tags'])}
    if detail == 'summary':
        head['walls'] = sorted({t['wall'] for t in marks['tags']})
        head['tag_ids'] = sorted(int(t['id']) for t in marks['tags'])
        return head
    head['tags'] = [{'id': int(t['id']), 'wall': t['wall'], 'normal_xy': t['normal_xy'],
                     'center_m': t['center_m'], 'yaw_rad': t['yaw_rad'], 'size_m': t['size_m']}
                    for t in sorted(marks['tags'], key=lambda t: int(t['id']))]
    return head


def public_map(data: Mapping, *, landmark_detail: str = 'full') -> dict:
    """The compact JSON projection a robot may receive every call (no TOP, no live state)."""
    if landmark_detail not in LANDMARK_DETAIL:
        raise ValueError(f'landmark_detail must be one of {LANDMARK_DETAIL}')
    value = {'schema': PUBLIC_MAP_SCHEMA, 'map_id': data['map_id'], 'version': data['version'],
             'frame': data['frame'], 'bounds_m': data['bounds_m'], 'walls': _walls(data),
             'terrain': list(data.get('terrain', ())), 'passages': _passages(data),
             'regions': _regions(data), 'zone_slots': {z: data['zone_slots'][z] for z in ZONE_IDS},
             'pickup_bays': pickup_bays(data), 'item_kinds_painted': list(data['box_kinds']),
             'approach_convention': data['approach_convention'], 'landmark_detail': landmark_detail}
    marks = _landmarks(data, landmark_detail)
    if marks is not None:
        value['landmarks'] = marks
    if 'base_map' in data:
        value['base_map_id'] = data['base_map']['map_id']
    return _round(value)


# ---------------------------------------------------------------------------
# Schematic PNG (drawn from the map file; never a camera capture)

FLOOR = (246, 246, 244)
WALL = (58, 58, 62)
DOOR = (32, 132, 64)
CORRIDOR = (140, 110, 40)
PICKUP = (150, 178, 214)
ZONE_PAINT = {'A': (242, 190, 140), 'B': (168, 190, 240), 'C': (208, 172, 226)}
SLOT = (120, 120, 126)
TAG = (16, 16, 16)
INK = (24, 24, 28)
MARGIN_PX = 30
FOOTER_PX = 42


def _font():
    return ImageFont.load_default()


class _Plan:
    """World metres -> pixels, north up."""

    def __init__(self, bounds, px_per_m, margin=MARGIN_PX):
        self.x0, self.x1, self.y0, self.y1 = [float(v) for v in bounds]
        self.px_per_m, self.margin = int(px_per_m), int(margin)

    def px(self, x, y):
        return (self.margin + int(round((float(x) - self.x0) * self.px_per_m)),
                self.margin + int(round((self.y1 - float(y)) * self.px_per_m)))

    def box(self, center, half):
        (cx, cy), (hx, hy) = center, half
        left, top = self.px(cx - hx, cy + hy)
        right, bottom = self.px(cx + hx, cy - hy)
        return [left, top, max(right, left + 1), max(bottom, top + 1)]

    @property
    def size(self):
        return (2 * self.margin + int(round((self.x1 - self.x0) * self.px_per_m)),
                2 * self.margin + int(round((self.y1 - self.y0) * self.px_per_m)) + FOOTER_PX)


def render_schematic(data: Mapping, *, width_px: int = 760, path: Path | str | None = None,
                     landmark_detail: str = 'full') -> tuple[bytes, dict]:
    """(PNG bytes, metadata) of a floor plan drawn from the map file only.

    Deterministic for a map file and a PIL version: integer pixel geometry, the
    bundled default bitmap font and ASCII labels only (ids and zone letters stay
    literal). Robots, cargo, occupancy and discovered obstacles are never drawn.
    """
    bounds = data['bounds_m']
    px_per_m = max(int((int(width_px) - 2 * MARGIN_PX) / (float(bounds[1]) - float(bounds[0]))), 20)
    plan = _Plan(bounds, px_per_m)
    image = Image.new('RGB', plan.size, (255, 255, 255))
    draw = ImageDraw.Draw(image)
    font = _font()
    floor = [*plan.px(plan.x0, plan.y1), *plan.px(plan.x1, plan.y0)]
    draw.rectangle(floor, fill=FLOOR, outline=INK)

    region = _regions(data)
    pickup = region['pickup']
    draw.rectangle(plan.box(pickup['center_m'], pickup['half_extents_m']), outline=PICKUP, width=2)
    for bay in pickup_bays(data):
        draw.rectangle(plan.box(bay['center_m'], bay['half_extents_m']), outline=PICKUP)
        for slot in bay['slots']:
            left, top, right, bottom = plan.box(slot['center_m'], slot['half_extents_m'])
            draw.rectangle([left, top, right, bottom], outline=PICKUP)
            draw.text((left + 3, top + 2), slot['slot_id'], fill=INK, font=font)
    draw.text(plan.px(pickup['center_m'][0] - pickup['half_extents_m'][0],
                      pickup['center_m'][1] - pickup['half_extents_m'][1]), 'pickup', fill=INK, font=font)

    for zone in ZONE_IDS:
        spec = region[zone]
        left, top, right, bottom = plan.box(spec['center_m'], spec['half_extents_m'])
        draw.rectangle([left, top, right, bottom], fill=ZONE_PAINT[zone], outline=INK)
        draw.text((left + 4, top + 3), zone, fill=INK, font=font)
        for slot in data['zone_slots'][zone]:
            draw.rectangle(plan.box(slot['center_m'], slot['half_extents_m']), outline=SLOT)

    for wall in _walls(data):
        draw.rectangle(plan.box(wall['center_m'], wall['half_extents_m']), fill=WALL)
    for passage in _passages(data):
        colour = DOOR if passage['kind'] == 'door' else CORRIDOR
        left, top, right, bottom = plan.box(passage['center_m'], passage['half_extents_m'])
        draw.rectangle([left - 1, top, right + 1, bottom], fill=FLOOR, outline=colour, width=2)
        label = f"{passage['id']} {passage['width_m']:.2f}m L{passage['lanes']}" if 'width_m' in passage \
            else passage['id']
        draw.text((right + 4, top), label, fill=colour, font=font)

    tags = data.get('landmarks', {}).get('tags', ()) if landmark_detail != 'none' else ()
    for tag in tags:
        x, y = float(tag['center_m'][0]), float(tag['center_m'][1])
        cx, cy = plan.px(x, y)
        draw.rectangle([cx - 2, cy - 2, cx + 2, cy + 2], fill=TAG)
        if int(tag['id']) % TAG_LABEL_STRIDE == 0:
            draw.text((cx + 4, cy - 5), str(int(tag['id'])), fill=TAG, font=font)

    width, height = plan.size
    draw.text((MARGIN_PX, 8), f"{data['map_id']} v{data['version']} floor plan (drawn from the map file; "
                              'not a camera image)', fill=INK, font=font)
    draw.text((MARGIN_PX, height - 34), f"x east, y north; {len(tags)} AprilTag landmarks marked",
              fill=INK, font=font)
    bar_x, bar_y = MARGIN_PX, height - 12
    draw.line([bar_x, bar_y, bar_x + px_per_m, bar_y], fill=INK, width=2)
    draw.text((bar_x + px_per_m + 6, bar_y - 6), '1 m', fill=INK, font=font)
    arrow_x, arrow_y = width - MARGIN_PX - 8, MARGIN_PX + 6
    draw.line([arrow_x, arrow_y + 18, arrow_x, arrow_y], fill=INK, width=2)
    draw.polygon([(arrow_x - 4, arrow_y + 5), (arrow_x + 4, arrow_y + 5), (arrow_x, arrow_y - 3)], fill=INK)
    draw.text((arrow_x - 3, arrow_y + 20), 'N', fill=INK, font=font)

    buffer = io.BytesIO()
    image.save(buffer, format='PNG', optimize=False)
    png = buffer.getvalue()
    meta = {'width_px': width, 'height_px': height, 'px_per_m': px_per_m,
            'png_sha256': digest_bytes(png), 'pixels_sha256': digest_bytes(image.tobytes()),
            'renderer': {'library': 'PIL', 'version': PIL.__version__, 'font': 'PIL default bitmap'},
            'draws': 'walls, doors/corridors, pickup bays, zones A/B/C and their slots, map AprilTags',
            'excludes': 'robots, cargo, occupancy, discovered obstacles, camera frames'}
    if path is not None:
        Path(path).write_bytes(png)
        meta['path'] = str(path)
    return png, meta


def schematic_ref_id(map_id: str) -> str:
    return f'map-{map_id}-schematic'


def map_bundle(map_id: str, *, maps_dir: Path | str = MAP_DIR, landmark_detail: str = 'full',
               schematic: bool = True, width_px: int = 760, schematic_path: Path | str | None = None) -> dict:
    """Everything a run must pin: map file hash, JSON projection hash, schematic hash."""
    data, file_sha = load_map(map_id, maps_dir=maps_dir)
    projection = public_map(data, landmark_detail=landmark_detail)
    bundle = {'schema': MAP_BUNDLE_SCHEMA, 'map_id': map_id,
              'map_file': str(map_path(map_id, maps_dir=maps_dir).relative_to(Path(maps_dir).parents[1])),
              'map_file_sha256': file_sha, 'landmark_detail': landmark_detail,
              'public_map': projection, 'public_map_sha256': digest(projection),
              'base_map': dict(data['base_map']) if 'base_map' in data else None,
              'has_landmarks': 'landmarks' in data}
    if schematic:
        png, meta = render_schematic(data, width_px=width_px, path=schematic_path,
                                     landmark_detail=landmark_detail)
        bundle['schematic'] = {'ref': schematic_ref_id(map_id), 'bytes': len(png), **meta}
    return bundle


def static_map_section(bundle: Mapping) -> dict:
    """The robot-facing ``static_map`` value of a per-call payload."""
    if bundle.get('schema') != MAP_BUNDLE_SCHEMA:
        raise ValueError('static_map_section needs a map_bundle')
    section = {'map_id': bundle['map_id'], 'map_file_sha256': bundle['map_file_sha256'],
               'public_map': bundle['public_map'], 'public_map_sha256': bundle['public_map_sha256']}
    schematic = bundle.get('schematic')
    if schematic:
        section['schematic_ref'] = {'ref': schematic['ref'], 'kind': 'map_schematic',
                                    'png_sha256': schematic['png_sha256'],
                                    'width_px': schematic['width_px'], 'height_px': schematic['height_px'],
                                    'px_per_m': schematic['px_per_m']}
    return section


def verify_static_map_section(section: Mapping, *, maps_dir: Path | str = MAP_DIR) -> dict:
    """Re-derive the projection from the map file and compare hashes (audit path)."""
    data, file_sha = load_map(section['map_id'], maps_dir=maps_dir)
    detail = section['public_map'].get('landmark_detail', 'full')
    projection = public_map(data, landmark_detail=detail)
    report = {'map_id': section['map_id'], 'map_file_sha256_matches': file_sha == section['map_file_sha256'],
              'public_map_sha256_matches': digest(projection) == section['public_map_sha256'],
              'public_map_matches': projection == section['public_map']}
    report['ok'] = all(report[k] for k in ('map_file_sha256_matches', 'public_map_sha256_matches',
                                           'public_map_matches'))
    return report
