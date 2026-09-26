"""Tag-free environment-v3 zone scene: base map + wall profile ``walls_v3`` + ZERO landmarks.

User decisions (2026-09-26): the final study environment has no AprilTags at all
(no wall, door-frame, corridor-entrance, bay or zone tags). It is the environment
v3 wall profile (every wall 0.40 m, ``sim.zone_arena.WALL_PROFILES['walls_v3']``,
branch ``kiro/zone-map-v3``) applied to an authored base map
(``maps/zones/zone_wide_door.json`` ...), with an explicitly empty landmark list.

This module is additive glue. It does not modify ``sim/zone_scene.py``,
``sim/zone_arena.py`` or any file under ``maps/zones/`` (owned by other work);
it imports them from the pinned source tree the renders run in
(``kiro/zone-map-v3`` at ``V3_SOURCE_SHA``) and builds:

* ``tagfree_map(base)``: the static map the student receives (walls, door,
  passages, regions, zone slots, bounds, TOP camera spec; ``landmarks.tags`` = []);
* ``TagFreeZoneScene``: a ``ZoneScene`` whose static map is that map, so the
  walls are built 0.40 m high and no tag geoms are added. Physics and every other
  scene element are the base map's (tag geoms are visual-only anyway).

The empty ``landmarks`` block exists only so that code written for tagged maps
(``tags_by_id``, ``TagDetector.for_map``) runs unchanged; it contains no tags.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

V3_SOURCE_BRANCH = 'kiro/zone-map-v3'
V3_SOURCE_SHA = '7cedb0490b3802088df5b5f789566657aebd82d0'
WALL_PROFILE = 'walls_v3'
SCHEMA = 'ugrp.zone_tagfree_map.v1'
HERE = Path(__file__).resolve().parent
MAP_OUT_DIR = HERE/'maps'


def map_id(base: str) -> str:
    return f'{base}_{WALL_PROFILE}_notags'


def tagfree_map(base: str = 'zone_wide_door') -> dict:
    """Static map of the tag-free environment (deterministic, from the pinned v3 source)."""
    from sim.research_dispatch_arena import digest
    from sim.zone_arena import apply_wall_profile, authored_map
    source = authored_map(base)
    value = apply_wall_profile(source, WALL_PROFILE)
    value.update(map_id=map_id(base), version=3,
                 base_map={'map_id': source['map_id'], 'version': source['version'],
                           'static_map_sha256': digest(source)})
    value['landmarks'] = {'schema': SCHEMA, 'tags': [],
                          'note': ('environment v3 without any AprilTag (user decision 2026-09-26): no wall, '
                                   'door-frame, corridor, bay or zone tags; the empty list only keeps code written '
                                   'for tagged maps runnable')}
    return value


def map_sha256(value: dict) -> str:
    """Hash as recorded by the zone code (``sim.research_dispatch_arena.digest``)."""
    from sim.research_dispatch_arena import digest
    return digest(value)


def file_sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_map(base: str = 'zone_wide_door') -> Path:
    """Write the student map JSON once (never overwrite a different published file)."""
    MAP_OUT_DIR.mkdir(exist_ok=True)
    path = MAP_OUT_DIR/(map_id(base) + '.json')
    value = tagfree_map(base)
    text = json.dumps(value, indent=2) + '\n'
    if path.exists():
        if json.loads(path.read_text()) != value:
            raise ValueError(f'{path} differs from its definition; create a new version instead')
        return path
    path.write_text(text)
    return path


def load_map(base: str = 'zone_wide_door') -> dict:
    """The committed student map; refuses drift from the definition (needs the v3 source on sys.path)."""
    path = MAP_OUT_DIR/(map_id(base) + '.json')
    value = json.loads(path.read_text())
    if value != tagfree_map(base):
        raise ValueError('tag-free map file differs from base map + walls_v3 + empty landmarks')
    return value


def make_scene_class():
    """``TagFreeZoneScene`` (import-time dependency on the v3 source tree)."""
    from sim.research_dispatch_arena import digest
    from sim.zone_arena import MAP_DIR, episode
    from sim.zone_scene import ZoneScene

    class TagFreeZoneScene(ZoneScene):
        """ZoneScene on the tag-free walls_v3 map. Selection id: ``zones/<base>_walls_v3_notags``."""

        @classmethod
        def from_tagfree(cls, base, seed, goal, extra_boxes=None, contact_profile=None):
            from sim.session_scenes import ROOT
            selected = {'layout': 'zones/' + map_id(base), 'seed': seed, 'map_file': None, 'cargo_ids': None,
                        'robots': {}, 'objects': [], 'builder': None, 'contact_profile': contact_profile,
                        'params': {'goal': goal, 'extra_boxes': extra_boxes or {}, 'base': base}}
            return cls(selected, ROOT)

        def _resolve(self):
            family, name = self.selection.split('/', 1)
            params = self.scene.get('params') or {}
            base = params.get('base')
            if family != 'zones' or base is None or name != map_id(base):
                raise ValueError(f'unknown tag-free zone scene: {self.selection}')
            self._read(MAP_DIR/(base + '.json'))
            config = episode(base, self.scene['seed'], goal=params['goal'], extra_boxes=params.get('extra_boxes'))
            static = load_map(base)
            config.update(static_map=static, static_map_sha256=digest(static), tagfree_map_id=name)
            config['extra_boxes'] = params.get('extra_boxes') or {}
            self.config = config
            self.bounds = static['bounds_m']
            self.inventory = list(config['setup_only']['objects'])
            self._verify_camera()

        def transform(self, xml):
            xml = super().transform(xml)
            static = self.config['static_map']
            self.manifest.update(
                scene_xml_sha256=hashlib.sha256(xml.encode()).hexdigest(),
                static_map_sha256=digest(static),
                base_static_map_sha256=static['base_map']['static_map_sha256'],
                landmarks_sha256=digest(static['landmarks']), tag_count=0, tag_geoms=0,
                wall_profile=copy.deepcopy(static['wall_profile']),
                environment='environment v3 walls (0.40 m) without AprilTags')
            return xml

    return TagFreeZoneScene
