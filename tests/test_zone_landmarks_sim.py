"""Tagged zone maps in MuJoCo: tags are visual only and visible to the wrist camera."""
from __future__ import annotations

import importlib.util
import math
import unittest

import numpy as np

GOAL = {'A': {'cyan': 1}}
EXTRA = {'red': 1}


def build(tagged, render, name='zone_wide_door_tags_v1'):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.session_scenes import ROOT
    from sim.zone_landmarks import TaggedZoneScene
    from sim.zone_scene import ZoneScene
    if tagged:
        scene = TaggedZoneScene.from_tagged(name, 11, GOAL, EXTRA,
                                            contact_profile='local_contact_fine')
    else:
        scene = ZoneScene({'layout': 'zones/zone_wide_door', 'seed': 11, 'map_file': None, 'cargo_ids': None,
                           'robots': {}, 'objects': [], 'builder': None, 'contact_profile': 'local_contact_fine',
                           'params': {'goal': GOAL, 'extra_boxes': EXTRA}}, ROOT)
    world = MultiMasterPiProductionV2(seed=11, width=640, height=480, render=render,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    return scene, world


def drive(world, seconds=1.5):
    from sim.camera_robot_port import CameraRobotPort
    ports = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2', 'r3')}
    t0 = float(world.data.time)
    ports['r1'].apply({'kind': 'mecanum', 'forward': .12, 'left': .03, 'turn': .05, 'duration_s': 1.}, t0)
    ports['r2'].apply({'kind': 'arm', 'servo_id': 3, 'pulse': 900}, t0)
    trace = []
    dt = float(world.model.opt.timestep)
    for _ in range(int(seconds/dt)):
        now = float(world.data.time)
        for port in ports.values():
            port.tick(now)
        world._physics_step_for(world.controllers['r1'])
        trace.append(world.data.qpos.copy())
    return np.array(trace)


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'mujoco is not installed')
class TaggedScenePhysicsTests(unittest.TestCase):
    def test_tags_do_not_change_physics(self):
        for name in ('zone_wide_door_tags_v1', 'zone_wide_door_tags_v2'):
            with self.subTest(map=name):
                self._same_physics(name)

    def _same_physics(self, name):
        base_scene, base = build(False, False)
        tag_scene, tagged = build(True, False, name)
        try:
            self.assertEqual(base_scene.config['setup_only'], tag_scene.config['setup_only'])
            self.assertEqual((base.model.nbody, base.model.njnt, base.model.nq),
                             (tagged.model.nbody, tagged.model.njnt, tagged.model.nq))
            self.assertEqual(tagged.model.ngeom - base.model.ngeom, tag_scene.manifest['tag_geoms'])
            import mujoco
            tag_ids = [g for g in range(tagged.model.ngeom)
                       if (mujoco.mj_id2name(tagged.model, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith('tag_')]
            self.assertEqual(len(tag_ids), tag_scene.manifest['tag_geoms'])
            for gid in tag_ids:
                self.assertEqual((int(tagged.model.geom_contype[gid]), int(tagged.model.geom_conaffinity[gid])), (0, 0))
            a, b = drive(base), drive(tagged)
            self.assertTrue(np.array_equal(a, b), float(np.abs(a - b).max()))
            self.assertGreater(float(np.abs(a[-1] - a[0]).max()), .05)  # the robot really moved
        finally:
            base.close()
            tagged.close()

    def test_wrist_camera_sees_wall_tags_with_consistent_pnp(self):
        from harness.wall_tags import TagDetector, observed_tag_in_camera, predicted_tag_in_camera, tags_by_id
        from sim.masterpi_production_v2 import SEARCH_POSE
        from sim.camera_robot_port import CameraRobotPort
        scene, world = build(True, True)
        try:
            port = CameraRobotPort(world, 'r1')
            for servo, pulse in SEARCH_POSE.items():
                port.apply({'kind': 'look', 'pan_pulse': pulse} if servo == 6 else
                           {'kind': 'arm', 'servo_id': servo, 'pulse': pulse}, float(world.data.time))
            dt = float(world.model.opt.timestep)
            for _ in range(int(1./dt)):
                port.tick(float(world.data.time))
                world._physics_step_for(world.controllers['r1'])
            static = scene.config['static_map']
            dets = TagDetector.for_map(static).detect(world.render_rgb(robot_id='r1', camera='robot_cam'))
            self.assertGreaterEqual(len(dets), 2)
            robot = world.robot('r1')   # test-only truth to check the geometry
            pose = np.array([[*robot.base_xyz()[:2], robot.base_rpy()[2]]])
            tags = tags_by_id(static)
            for det in dets:
                t_obs, _ = observed_tag_in_camera(det)
                p_c, _ = predicted_tag_in_camera(pose, tags[det['id']], SEARCH_POSE)
                az = math.degrees(math.atan2(t_obs[0], t_obs[2]) - math.atan2(p_c[0, 0], p_c[0, 2]))
                self.assertLess(abs(az), .5, det['id'])
                self.assertLess(abs(math.log(np.linalg.norm(t_obs)/np.linalg.norm(p_c[0]))), .1, det['id'])
        finally:
            world.close()


if __name__ == '__main__':
    unittest.main()
