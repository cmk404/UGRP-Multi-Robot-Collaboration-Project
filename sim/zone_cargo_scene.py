"""Opt-in cargo profile for zone scenes (``cargo_set``).

``CargoZoneScene`` is the standard ``ZoneScene`` plus catalogue cargo items from
``sim.zone_cargo``. Without cargo items it produces exactly the ZoneScene XML,
so ``zone_open`` v2 and ``zone_wide`` v1 stay byte-identical (tested). Cargo
poses are setup-only, like box poses, and never reach robots.
"""
from __future__ import annotations

import hashlib

from sim.session_scenes import ROOT
from sim.zone_arena import DEFAULT_GOAL, VARIANTS
from sim.zone_cargo import add_cargo_xml, catalogue_record, instances, place
from sim.zone_scene import ZoneScene

CARGO_SET_SCHEMA = 'ugrp.zone_cargo_set.v1'


class CargoZoneScene(ZoneScene):
    @classmethod
    def from_cargo_config(cls, variant, seed, *, cargo, goal=None, extra_boxes=None,
                          contact_profile='local_contact_fine', base_dir=ROOT):
        if variant not in VARIANTS:
            raise ValueError('unknown zone arena variant')
        if contact_profile is not None:
            from sim.dispatch_contact_profile import PROFILES
            if contact_profile not in PROFILES:
                raise ValueError(f'contact profile: choose {PROFILES}')
        selected = {'layout': 'zones/' + variant, 'seed': seed, 'map_file': None, 'cargo_ids': None,
                    'robots': {}, 'objects': [], 'builder': None, 'contact_profile': contact_profile,
                    'params': {'goal': goal or DEFAULT_GOAL, 'extra_boxes': extra_boxes or {},
                               'cargo_set': {'schema': CARGO_SET_SCHEMA, 'items': list(cargo or [])}}}
        return cls(selected, base_dir)

    def _resolve(self):
        super()._resolve()
        cargo_set = (self.scene.get('params') or {}).get('cargo_set') or {'items': []}
        self.cargo = instances(cargo_set['items'])
        self.config['cargo_set'] = {'schema': CARGO_SET_SCHEMA,
                                    'catalogue_sha256': catalogue_record()['sha256'],
                                    'items': [{'item_id': c.item_id, 'kind': c.kind, 'pose': list(c.pose),
                                               **({'mass_kg': c.mass_kg} if c.mass_kg is not None else {})}
                                              for c in self.cargo]}

    def transform(self, xml):
        xml = super().transform(xml)
        if not self.cargo:
            return xml
        xml, record = add_cargo_xml(xml, self.cargo, mirror_pairs=bool(self.scene['contact_profile']))
        self.manifest['scene_xml_sha256'] = hashlib.sha256(xml.encode()).hexdigest()
        self.manifest['cargo'] = record
        return xml

    def setup(self, world):
        super().setup(world)
        if self.cargo:
            place(world, self.cargo)
