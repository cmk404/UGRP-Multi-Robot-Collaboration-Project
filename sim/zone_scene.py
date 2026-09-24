"""Standard-scene path for the zone benchmark.

``ZoneScene`` subclasses the standard ``sim.session_scenes.Scene`` and reuses
its source recording, spawn/object reset and record format. It lives outside
``sim/session_scenes.py`` so the pinned RGB dispatch bundle sources stay
unchanged; the zone family is added to the shared catalog together with the
next dispatch bundle registration.
"""
from __future__ import annotations

import copy
import hashlib

from sim.session_scenes import ROOT, Scene
from sim.zone_arena import DEFAULT_GOAL, EAST_TOP, MAP_DIR, VARIANTS, build_zone_xml, episode

OBSERVER_POSITION_M = (2.2, -6.6, 4.6)
OBSERVER_TARGET_M = (2.2, -2., .03)


def mirror_box_contact_pairs(xml, bodies):
    """Give every painted replica the finger contact pairs of the dispatch box.

    The contact profile declares finger/cargo pairs by geom name for the
    original ``dispatch_box_geom`` only. Replicas are the same box with other
    paint, so they receive identical copies of those pairs (same friction,
    solref, solimp, margin); without them the fingers fall back to default
    contact and the box slides out of the grip during transport.
    """
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml)
    contact = root.find('contact')
    source = [pair for pair in (contact.findall('pair') if contact is not None else [])
              if pair.get('geom2') == 'dispatch_box_geom']
    for body in bodies:
        for pair in source:
            contact.append(ET.Element('pair', {**pair.attrib, 'geom2': body + '_geom'}))
    return ET.tostring(root, encoding='unicode'), len(source)*len(bodies)


class ZoneScene(Scene):
    @classmethod
    def from_zone_config(cls, config, base_dir=ROOT):
        """Wrap a selected zone episode (goal and spares) in the standard scene path."""
        profile = config.get('contact_solver_profile')
        if profile is not None:
            from sim.dispatch_contact_profile import PROFILES
            if profile not in PROFILES:
                raise ValueError(f'contact profile: choose {PROFILES}')
        selected = {'layout': 'zones/' + config['variant'], 'seed': config['seed'],
                    'map_file': None, 'cargo_ids': None, 'robots': {}, 'objects': [],
                    'builder': None, 'contact_profile': profile,
                    'params': {'goal': config['goal'], 'extra_boxes': config.get('extra_boxes') or {}}}
        scene = cls(selected, base_dir)
        if scene.config['setup_only'] != config['setup_only'] or scene.config['static_map'] != config['static_map']:
            raise ValueError('zone runner episode differs from the selected scene')
        return scene

    def _resolve(self):
        family, name = self.selection.split('/', 1)
        if family != 'zones' or name not in VARIANTS:
            raise ValueError(f'unknown zone scene: {self.selection}')
        self._read(MAP_DIR/(name+'.json'))
        params = self.scene.get('params') or {}
        self.config = episode(name, self.scene['seed'], goal=params.get('goal') or DEFAULT_GOAL,
                              extra_boxes=params.get('extra_boxes'))
        self.config['extra_boxes'] = params.get('extra_boxes') or {}
        self.bounds = self.config['static_map']['bounds_m']
        self.inventory = list(self.config['setup_only']['objects'])
        self._verify_camera()

    def _verify_camera(self):
        from sim.research_dispatch_arena import FIXED_TOP
        if self.config['static_map']['top_cameras'] != [FIXED_TOP, EAST_TOP]:
            raise ValueError('zone scene must keep the approved TOP plus one identical east CCTV')

    def transform(self, xml):
        from sim.research_scene_xml import plain_beam_xml
        xml, self.manifest = build_zone_xml(plain_beam_xml(xml), self.config)
        if self.scene['contact_profile']:
            from sim.dispatch_contact_profile import contact_profile
            xml = contact_profile(xml, self.scene['contact_profile'])
            xml, mirrored = mirror_box_contact_pairs(xml, [r['body_name'] for r in self.manifest['box_replicas'].values()])
            self.manifest['scene_xml_sha256'] = hashlib.sha256(xml.encode()).hexdigest()
            self.manifest['contact_solver_profile'] = self.scene['contact_profile']
            self.manifest['replica_finger_contact_pairs'] = mirrored
        return xml

    def setup(self, world):
        # The standard reset reads one top camera; give it the approved west
        # TOP, then place the identical east TOP and frame the wider arena.
        import mujoco
        import numpy as np
        static = self.config['static_map']
        self.config['static_map'] = {**static, 'top_camera': copy.deepcopy(static['top_cameras'][0])}
        try:
            super().setup(world)
        finally:
            self.config['static_map'] = static
        for spec in static['top_cameras']:
            top = world.model.camera(spec['name'])
            top.pos[:] = spec['position_m']
            top.quat[:] = spec['quaternion_wxyz']
            top.fovy[:] = spec['fov_y_deg']
        position, target = np.array(OBSERVER_POSITION_M), np.array(OBSERVER_TARGET_M)
        forward = target-position; forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.column_stack((right, up, -forward)).ravel())
        observer = world.model.camera('cctv_warehouse')
        observer.pos[:] = position; observer.quat[:] = quat
        mujoco.mj_forward(world.model, world.data)


def catalog():
    return [{'id': 'zones/'+name, 'family': 'zones',
             'scope': 'zone-goal coordination benchmark (ZoneScene); goal via scene params'} for name in VARIANTS]
