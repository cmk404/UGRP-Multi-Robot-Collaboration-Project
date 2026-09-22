"""Local scene catalog and setup adapters. Privileged setup never enters actors.

Use the existing research XML builders, with per-instance injection instead of
process-global monkeypatches. This module's catalog/resolve path needs no MuJoCo.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ENGINE_LAYOUTS = ('standard', 'mixed', 'arena', 'camera_team')
DEFAULT_SCENE = 'dispatch/shared_crossing'
CUSTOM_MAPS = ('navigation/file', 'pair_navigation/file')


def catalog():
    from sim.research_dispatch_arena import VARIANTS
    rows = [{'id': name, 'family': 'legacy', 'scope': 'engine example'} for name in ENGINE_LAYOUTS]
    rows += [{'id': 'dispatch/'+name, 'family': 'dispatch', 'scope': 'research scene; controller selected separately'} for name in VARIANTS]
    for family in ('navigation', 'pair_navigation'):
        rows += [{'id': family+'/'+p.stem, 'family': family,
                  'scope': 'unloaded scene' if family == 'navigation' else 'floor-load preview; no pre-grasped state'}
                 for p in sorted((ROOT/'maps'/family).glob('*.json')) if p.name != 'catalog.json']
    suite = json.loads((ROOT/'maps/act_generalization/suite_v1.json').read_text())
    cases = [c['id'] for c in suite['cases']]
    if suite['include_legacy_regression']:
        cases += ['regression_'+c['id'] for c in json.loads((ROOT/'maps/pair_navigation/catalog.json').read_text())['entries']]
    rows += [{'id': 'act/'+name, 'family': 'act', 'scope': 'map scene; no automatic ACT policy'} for name in cases]
    pilot = json.loads((ROOT/'maps/act_generalization/multi_object_pilot_v1.json').read_text())
    rows += [{'id': 'multi_object/'+c['id'], 'family': 'multi_object',
              'scope': 'physical replicas; research task execution uses its own runner'} for c in pilot['cases']]
    return rows


def validate_selection(scene):
    selection = scene['layout']
    if not isinstance(selection, str) or selection not in {row['id'] for row in catalog()} | set(CUSTOM_MAPS):
        raise ValueError('scene.layout: unknown scene; list available scenes with sim_cli scenes')
    path = scene['map_file']
    if selection in CUSTOM_MAPS:
        if not isinstance(path, str) or not path:
            raise ValueError('scene.map_file: required for navigation/file or pair_navigation/file')
    elif path is not None:
        raise ValueError('scene.map_file requires navigation/file or pair_navigation/file')
    if selection not in ENGINE_LAYOUTS and scene['cargo_ids'] is not None:
        raise ValueError('scene.cargo_ids only filters legacy engine layouts; research inventories are authored')
    profile = scene['contact_profile']
    if profile is not None:
        from sim.dispatch_contact_profile import PROFILES
        if not selection.startswith('dispatch/') or profile not in PROFILES:
            raise ValueError(f'scene.contact_profile: dispatch scenes only, choose {PROFILES}')


class Scene:
    def __init__(self, scene, base_dir):
        self.scene = scene
        self.selection = scene['layout']
        self.family = self.selection.split('/')[0] if '/' in self.selection else 'legacy'
        self.sources = {}
        self.config = None
        self.map = None
        self.manifest = {}
        self.xml = None
        self.bounds = None
        self.inventory = []
        self.engine_layout = self.selection if self.family == 'legacy' else 'standard'
        self.base_dir = Path(base_dir).resolve()
        self._resolve()

    def _read(self, path):
        path = Path(path).resolve()
        data = path.read_bytes()
        self.sources[str(path)] = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': data}
        return json.loads(data)

    def _resolve(self):
        from sim.research_dispatch_arena import episode, MAP_PATH
        if self.family == 'legacy':
            return
        name = self.selection.split('/', 1)[1]
        seed = self.scene['seed']
        if self.family == 'dispatch':
            self._read(MAP_PATH)
            self.config = episode(name, seed)
        elif self.family in ('navigation', 'pair_navigation'):
            path = self.base_dir/self.scene['map_file'] if name == 'file' else ROOT/'maps'/self.family/(name+'.json')
            data = self._read(path)
            if self.family == 'navigation':
                from sim.authored_navigation_map import validate_map
                self.map = validate_map(data)
            else:
                from harness.pair_navigation import validate_map
                from sim.act_map_suite import scene_config
                self.map = validate_map(data)
                self.config = scene_config({'map': self.map}, physics_seed=seed)
        elif self.family == 'act':
            from sim.act_map_suite import load_suite, scene_config, SPEC
            self._read(SPEC)
            self._read(ROOT/'maps/act_generalization/protocol_v1.json')
            suite, cases = load_suite()
            # Resolution generates and validates the whole suite, not only the
            # selected case. Preserve every map read by that path, including
            # the authored corner template used by non-regression cases.
            if suite['include_legacy_regression']:
                self._read(ROOT/'maps/pair_navigation/catalog.json')
            for source in sorted({c['parameters']['source'] for c in cases
                                  if 'source' in c['parameters']}):
                self._read(ROOT/source)
            case = next(c for c in cases if c['id'] == name)
            self.map = copy.deepcopy(case['map'])
            self.config = scene_config(case, physics_seed=seed)
        elif self.family == 'multi_object':
            from sim.multi_object_suite import load_pilot, SPEC as PILOT
            from sim.multi_object_scene import configuration, SPEC
            self._read(PILOT)
            layout = self._read(SPEC)
            layout['physics_seed'] = seed
            _, cases = load_pilot()
            self.config = configuration(next(c for c in cases if c['id'] == name), layout)
        self.bounds = (self.config['static_map'] if self.config else self.map)['bounds_m']
        if self.config:
            setup = self.config['setup_only']
            self.inventory = (list(setup['objects']) if 'objects' in setup else list(setup['cargo']))
        self._verify_camera()

    def _verify_camera(self):
        from sim.research_dispatch_arena import FIXED_TOP
        data = self.config['static_map'] if self.config else self.map
        if data and data['top_camera'] != FIXED_TOP:
            raise ValueError('research scene must preserve the approved fixed TOP camera')

    def transform(self, xml):
        from sim.research_scene_xml import plain_beam_xml, course_xml
        from sim.research_dispatch_arena import build_scene_xml
        if self.family == 'navigation':
            xml, robots = course_xml(xml, self.map)
            self.manifest = {'robot_xml_sha256': robots}
        elif self.family != 'legacy':
            xml = plain_beam_xml(xml)
            if self.family == 'multi_object':
                from sim.multi_object_scene import build_multi_object_xml
                xml, self.manifest = build_multi_object_xml(xml, self.config)
            else:
                xml, self.manifest = build_scene_xml(xml, self.config)
            if self.scene['contact_profile']:
                from sim.dispatch_contact_profile import contact_profile
                xml = contact_profile(xml, self.scene['contact_profile'])
        return xml

    def setup(self, world):
        if self.family == 'legacy':
            self.inventory = [s.cargo_id for s in getattr(world, 'warehouse_specs', ())]
            return
        import mujoco
        import numpy as np
        if self.family == 'navigation':
            for i, rid in enumerate(world.robot_ids):
                world.robot(rid).set_base_pose_for_test((-1.2+1.2*i, .6, .0325), 0.)
            x, y = self.map['zones']['start']['center_m']
            world.robot('r1').set_base_pose_for_test((x, y, .0325), 0.)
            folded = ('r1',)
        else:
            for rid, pose in self.config['setup_only']['spawns'].items():
                world.robot(rid).set_base_pose_for_test(pose[:3], pose[3])
            setup = self.config['setup_only']
            objects = ([(o['joint_name'], o['position_m']) for o in setup['objects'].values()]
                       if 'objects' in setup else [('team_beam_free', setup['cargo']['beam']),
                                                   ('dispatch_box_free', setup['cargo']['box'])])
            for joint, position in objects:
                jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                if jid < 0:
                    raise ValueError(f'missing cargo joint: {joint}')
                q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
                world.data.qpos[q:q+7] = [*position, 1, 0, 0, 0]
                world.data.qvel[v:v+6] = 0
            folded = world.robot_ids
        world.data.eq_active[:] = 0
        mujoco.mj_forward(world.model, world.data)
        world._team_joint_move_servos({r: {1:2000,3:740,4:2320,5:1320,6:1500} for r in folded},
                                     .6, settle_s=.5 if self.family == 'navigation' else .4)
        data = self.config['static_map'] if self.config else self.map
        top = world.model.camera('cctv_top')
        top.pos[:] = data['top_camera']['position_m']
        top.quat[:] = data['top_camera']['quaternion_wxyz']
        top.fovy[:] = data['top_camera']['fov_y_deg']
        observer = world.model.camera('cctv_warehouse')
        position = np.array([.55, -4.3, 3.]) if self.family == 'navigation' else np.array([2.6, -4.8, 3.4])
        target = np.array([.55, -2., 0. if self.family == 'navigation' else .03])
        forward = target-position; forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0.,0.,1.]); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.column_stack((right,up,-forward)).ravel())
        observer.pos[:] = position; observer.quat[:] = quat
        observer.fovy[:] = 55. if self.family == 'navigation' else 46.
        mujoco.mj_forward(world.model, world.data)

    def record(self):
        from sim.research_dispatch_arena import digest
        return {'selection': self.selection, 'family': self.family, 'inventory': self.inventory,
                'bounds_m': self.bounds, 'contact_profile': self.scene['contact_profile'],
                'setup_only': self.config, 'authored_map': self.map,
                'resolved_sha256': digest({'config': self.config, 'map': self.map}),
                'geometry_extension_configured': bool(self.scene['objects'] or self.scene['builder']),
                'geometry_modified': None if self.scene['builder'] else bool(self.scene['objects']),
                'overridden_robot_poses': self.scene['robots'], **self.manifest}
