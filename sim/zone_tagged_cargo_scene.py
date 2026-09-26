"""Tagged zone map + catalogue cargo (M2 pair long_beam carry without runtime GT).

``TaggedCargoZoneScene`` = ``sim.zone_landmarks.TaggedZoneScene`` (tagged static
map: wall tags / door posts as static-map features, visual-only geoms) plus the
catalogue cargo items of ``sim.zone_cargo_scene.CargoZoneScene`` (same
``instances``/``add_cargo_xml``/``place`` calls and the same cargo contact
profile handling). Neither parent is modified. Without cargo items the XML is
exactly the ``TaggedZoneScene`` XML (tested). Cargo poses are setup-only.
"""
from __future__ import annotations

import hashlib

from sim.session_scenes import ROOT
from sim.zone_cargo import add_cargo_xml, catalogue_record, instances, place
from sim.zone_cargo_scene import CARGO_SET_SCHEMA
from sim.zone_landmarks import TaggedZoneScene


class TaggedCargoZoneScene(TaggedZoneScene):
    @classmethod
    def from_tagged_cargo(cls, name, seed, *, cargo, goal, contact_profile='local_contact_fine', base_dir=ROOT):
        from sim.zone_cargo_contact import CARGO_PROFILES, base_profile
        cargo_profile = contact_profile if contact_profile in CARGO_PROFILES else None
        base = base_profile(contact_profile) if contact_profile is not None else None
        selected = {'layout': 'zones/' + name, 'seed': seed, 'map_file': None, 'cargo_ids': None,
                    'robots': {}, 'objects': [], 'builder': None, 'contact_profile': base,
                    'cargo_contact_profile': cargo_profile,
                    'params': {'goal': goal, 'extra_boxes': {},
                               'cargo_set': {'schema': CARGO_SET_SCHEMA, 'items': list(cargo or [])}}}
        return cls(selected, base_dir)

    def _resolve(self):
        super()._resolve()
        cargo_set = (self.scene.get('params') or {}).get('cargo_set') or {'items': []}
        self.cargo = instances(cargo_set['items'])
        self.config['cargo_set'] = {'schema': CARGO_SET_SCHEMA, 'catalogue_sha256': catalogue_record()['sha256'],
                                    'items': [{'item_id': c.item_id, 'kind': c.kind, 'pose': list(c.pose),
                                               **({'mass_kg': c.mass_kg} if c.mass_kg is not None else {})}
                                              for c in self.cargo]}

    def transform(self, xml):
        xml = super().transform(xml)
        if self.cargo:
            xml, record = add_cargo_xml(xml, self.cargo, mirror_pairs=bool(self.scene['contact_profile']))
            self.manifest['cargo'] = record
        profile = self.scene.get('cargo_contact_profile')
        if profile:
            from sim.zone_cargo_contact import apply, profile_record
            xml = apply(xml, profile)
            self.manifest['cargo_contact_profile'] = profile_record(profile)
        if self.cargo or profile:
            self.manifest['scene_xml_sha256'] = hashlib.sha256(xml.encode()).hexdigest()
        return xml

    def setup(self, world):
        super().setup(world)
        if self.cargo:
            place(world, self.cargo)
