"""Evaluation-only TOP profiles (sim/zone_eval_top.py, #218).

Pins both profiles, checks the overlay contract on every zone map (only the
corridor family changes under v2, the input map is never modified), boundary
inputs, that no robot-side module references the profiles, and with MuJoCo that
applying a profile moves only the TOP cameras: physics trace and the robot
wrist RGB are byte-identical, and the moved TOP sees the whole corridor lane
and bay (env v3 ST5 measurement).
"""
from __future__ import annotations

import copy
import importlib.util
import math
import unittest
from pathlib import Path

from sim import zone_eval_top as zet
from sim.zone_eval_top import EvalTopError

ROOT = Path(__file__).resolve().parents[1]
X = ROOT/'experiments'/'2026-09-26-zone-eval-topcam'
# Frozen parameter hashes of the registered profiles (records cite these).
RECORD_SHA256 = {
    'zone_eval_top_v1': '070d159a0badcefd9dafd1bbc9ed32d16dc9077a9f16e72a70f6dbabefe63439',
    'zone_eval_top_v2': '03ad04df6be078f7bae15b9e10543324f3fa7c58849ec73b1561f237c8afa571',
}
MOVED = 'cctv_top_north_east'
V2_POSITION = [3.40, 0.80, 3.00]
CORRIDOR_MAPS = ('zone_wide_corridor', 'zone_wide_corridor_tags_v1', 'zone_wide_corridor_tags_v3')
OTHER_MAPS = ('zone_wide', 'zone_wide_door', 'zone_wide_two_doors', 'zone_wide_door_tags_v1',
              'zone_wide_door_tags_v2', 'zone_wide_door_tags_v3', 'zone_wide_door_tags_v3a1',
              'zone_wide_two_doors_tags_v1', 'zone_wide_two_doors_tags_v2', 'zone_wide_two_doors_tags_v3',
              'zone_wide_two_doors_tags_v3a1')


def static_of(name):
    from sim.zone_arena import VARIANTS, authored_map
    from sim.zone_landmarks import tagged_map
    return authored_map(name) if name in VARIANTS else tagged_map(name)


class ProfileTests(unittest.TestCase):
    def test_registered_profiles_are_frozen(self):
        self.assertEqual(set(zet.PROFILES), set(RECORD_SHA256))
        for pid, digest in RECORD_SHA256.items():
            record = zet.profile_record(pid)
            self.assertEqual(record['id'], pid)
            self.assertEqual(record['sha256'], digest, pid)

    def test_v1_is_the_authored_cameras_on_every_map(self):
        for name in CORRIDOR_MAPS + OTHER_MAPS:
            static = static_of(name)
            cameras = zet.eval_top_cameras(static, 'zone_eval_top_v1')
            self.assertEqual(cameras, static['top_cameras'], name)
            self.assertIsNot(cameras, static['top_cameras'])

    def test_v2_moves_one_corridor_camera_and_nothing_else(self):
        for name in OTHER_MAPS:
            static = static_of(name)
            self.assertEqual(zet.eval_top_cameras(static, 'zone_eval_top_v2'), static['top_cameras'], name)
        for name in CORRIDOR_MAPS:
            static = static_of(name)
            before = copy.deepcopy(static)
            cameras = zet.eval_top_cameras(static, 'zone_eval_top_v2')
            self.assertEqual(static, before, 'input map must not change')
            by_name = {c['name']: c for c in cameras}
            authored = {c['name']: c for c in static['top_cameras']}
            self.assertEqual(list(by_name), list(authored))
            for cname, cam in by_name.items():
                if cname != MOVED:
                    self.assertEqual(cam, authored[cname], (name, cname))
            moved = by_name[MOVED]
            self.assertEqual(moved['position_m'], V2_POSITION)
            self.assertEqual(moved['quaternion_wxyz'], [1., 0., 0., 0.])
            self.assertEqual(moved['fov_y_deg'], authored[MOVED]['fov_y_deg'])
            self.assertNotEqual(moved['position_m'], authored[MOVED]['position_m'])

    def test_eval_static_map_is_a_recorded_copy(self):
        static = static_of('zone_wide_corridor_tags_v3')
        before = copy.deepcopy(static)
        value = zet.eval_static_map(static, 'zone_eval_top_v2')
        self.assertEqual(static, before)
        self.assertEqual(value['top_cameras'], zet.eval_top_cameras(static, 'zone_eval_top_v2'))
        self.assertEqual(value['eval_top_profile'], zet.profile_record('zone_eval_top_v2'))
        self.assertEqual({k: v for k, v in value.items() if k not in ('top_cameras', 'eval_top_profile')},
                         {k: v for k, v in before.items() if k != 'top_cameras'})
        with_top = {**before, 'top_camera': copy.deepcopy(before['top_cameras'][3])}
        self.assertEqual(zet.eval_static_map(with_top, 'zone_eval_top_v2')['top_camera']['position_m'], V2_POSITION)
        with_top['top_camera'] = {**with_top['top_camera'], 'name': 'nope'}
        with self.assertRaises(EvalTopError):
            zet.eval_static_map(with_top, 'zone_eval_top_v2')

    def test_moved_camera_keeps_the_evaluation_floor_projection(self):
        from harness.zone_perception import pixel_to_floor
        moved = [c for c in zet.eval_top_cameras(static_of('zone_wide_corridor_tags_v3'), 'zone_eval_top_v2')
                 if c['name'] == MOVED][0]
        x, y = pixel_to_floor(479.5, 359.5, moved, (720, 960))
        self.assertAlmostEqual(x, 3.40, places=9)
        self.assertAlmostEqual(y, 0.80, places=9)

    def test_changed_corridor_geometry_is_refused(self):
        static = static_of('zone_wide_corridor')
        moved_wall = copy.deepcopy(static)
        moved_wall['obstacles'][-1]['center_m'][0] += .05
        self.assertEqual(zet.eval_top_cameras(moved_wall, 'zone_eval_top_v1'), static['top_cameras'])
        with self.assertRaises(EvalTopError):
            zet.eval_top_cameras(moved_wall, 'zone_eval_top_v2')
        tagged = copy.deepcopy(static_of('zone_wide_corridor_tags_v3'))
        tagged['base_map']['static_map_sha256'] = '0'*64
        with self.assertRaises(EvalTopError):
            zet.eval_top_cameras(tagged, 'zone_eval_top_v2')


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.static = static_of('zone_wide_corridor_tags_v3')

    def test_bad_profile_ids(self):
        for bad in (None, '', 'zone_eval_top_v3', 'ZONE_EVAL_TOP_V2', 0, 1.0, float('nan'), True,
                    ['zone_eval_top_v2'], {'id': 'zone_eval_top_v2'}):
            with self.assertRaises(EvalTopError, msg=repr(bad)):
                zet.eval_top_cameras(self.static, bad)
            with self.assertRaises(EvalTopError, msg=repr(bad)):
                zet.profile_record(bad)

    def test_bad_maps(self):
        for bad in (None, {}, [], 'zone_wide_corridor', 0, {'map_id': 'zone_wide_corridor'},
                    {'map_id': 'zone_wide_corridor', 'top_cameras': []},
                    {'map_id': 'zone_wide_corridor', 'top_cameras': None},
                    {**self.static, 'base_map': {'map_id': 'zone_wide_corridor'}},
                    {**self.static, 'base_map': 'x'},
                    {**self.static, 'map_id': None}):
            with self.assertRaises(EvalTopError, msg=repr(bad)[:80]):
                zet.eval_top_cameras(bad, 'zone_eval_top_v1')

    def test_bad_camera_values(self):
        def with_cam(**change):
            value = copy.deepcopy(self.static)
            value['top_cameras'][0].update(change)
            return value
        cases = [dict(position_m=[float('nan'), 0., 2.5]), dict(position_m=[0., float('inf'), 2.5]),
                 dict(position_m=[0., 0.]), dict(position_m='0 0 2.5'), dict(position_m=None),
                 dict(position_m=[True, 0., 2.5]), dict(position_m=['0', 0., 2.5]),
                 dict(fov_y_deg=0.), dict(fov_y_deg=180.), dict(fov_y_deg=-5.), dict(fov_y_deg=float('nan')),
                 dict(fov_y_deg=None), dict(fov_y_deg='55'), dict(quaternion_wxyz=[.99, .1, 0., 0.]),
                 dict(quaternion_wxyz=[]), dict(name=''), dict(name=None)]
        for change in cases:
            for pid in zet.PROFILES:
                with self.assertRaises(EvalTopError, msg=f'{change} {pid}'):
                    zet.eval_top_cameras(with_cam(**change), pid)
        dup = copy.deepcopy(self.static)
        dup['top_cameras'][1]['name'] = dup['top_cameras'][0]['name']
        with self.assertRaises(EvalTopError):
            zet.eval_top_cameras(dup, 'zone_eval_top_v1')
        missing = copy.deepcopy(self.static)
        missing['top_cameras'] = [c for c in missing['top_cameras'] if c['name'] != MOVED]
        self.assertEqual(len(zet.eval_top_cameras(missing, 'zone_eval_top_v1')), 3)
        with self.assertRaises(EvalTopError):
            zet.eval_top_cameras(missing, 'zone_eval_top_v2')

    def test_zero_coordinates_are_kept_not_defaulted(self):
        value = copy.deepcopy(static_of('zone_wide_door'))
        value['top_cameras'][0]['position_m'] = [0., 0, 2.5]
        cams = zet.eval_top_cameras(value, 'zone_eval_top_v2')
        self.assertEqual(cams[0]['position_m'], [0., 0, 2.5])


class RobotInputBoundaryTests(unittest.TestCase):
    """The profiles are evaluation-only: no robot-side code may name or import them."""

    def test_no_robot_side_module_references_the_profiles(self):
        allowed = {ROOT/'sim'/'zone_eval_top.py'}
        needles = ('sim.zone_eval_top', 'import zone_eval_top', 'zone_eval_top_v', 'eval_static_map',
                   'eval_top_cameras')
        hits = []
        for folder in ('harness', 'scripts', 'sim', 'configs', 'maps'):
            for path in (ROOT/folder).rglob('*'):
                if not path.is_file() or path in allowed or path.suffix not in ('.py', '.json', '.md', '.txt', '.yaml'):
                    continue
                text = path.read_text(errors='ignore')
                if any(n in text for n in needles):
                    hits.append(str(path.relative_to(ROOT)))
        self.assertEqual(hits, [])

    def test_module_itself_reads_no_robot_camera(self):
        text = (ROOT/'sim'/'zone_eval_top.py').read_text()
        for word in ('robot_cam', 'render_rgb', 'render_jpeg', 'nav_cam', 'qpos', 'xpos'):
            self.assertNotIn(word, text)


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'mujoco is not installed')
class WorldTests(unittest.TestCase):
    GOAL, EXTRA = {'A': {'cyan': 1}}, {'red': 1}

    def world(self, render=False, name='zone_wide_corridor_tags_v3'):
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_landmarks import TaggedZoneScene
        scene = TaggedZoneScene.from_tagged(name, 11, self.GOAL, self.EXTRA, contact_profile='cargo_noslip_v1')
        world = MultiMasterPiProductionV2(seed=11, width=320, height=240, render=render,
                                          warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                          xml_transform=scene.transform)
        scene.setup(world)
        return scene, world

    @staticmethod
    def cameras(world):
        import numpy as np
        m = world.model
        return {m.camera(i).name: (np.array(m.cam_pos[i]), np.array(m.cam_quat[i]), float(m.cam_fovy[i]))
                for i in range(m.ncam)}

    def test_apply_moves_only_the_profile_camera_and_detects_a_reset(self):
        import numpy as np
        scene, world = self.world()
        try:
            static = scene.config['static_map']
            before = self.cameras(world)
            record = zet.apply_to_world(world, static, 'zone_eval_top_v2')
            after = self.cameras(world)
            self.assertEqual(set(before), set(after))
            changed = [n for n in before if not all(np.array_equal(a, b) for a, b in zip(before[n][:2], after[n][:2]))
                       or before[n][2] != after[n][2]]
            self.assertEqual(changed, [MOVED])
            for spec, row in zip(record['cameras'], record['applied']):
                self.assertEqual(spec['name'], row['name'])
                for a, b in zip(spec['position_m'], row['position_m']):
                    self.assertAlmostEqual(a, b, places=6)
                self.assertEqual(row['fov_y_deg'], spec['fov_y_deg'])
            self.assertEqual([r['position_m'] for r in record['applied'] if r['name'] == MOVED], [V2_POSITION])
            self.assertEqual(record['profile'], zet.profile_record('zone_eval_top_v2'))
            scene.setup(world)   # the scene puts the authored TOPs back
            with self.assertRaises(EvalTopError):
                zet.verify_world(world, static, 'zone_eval_top_v2')
            zet.verify_world(world, static, 'zone_eval_top_v1')
            zet.apply_to_world(world, static, 'zone_eval_top_v2')
            zet.verify_world(world, static, 'zone_eval_top_v2')
            with self.assertRaises(EvalTopError):
                zet.apply_to_world(world, {**static, 'top_cameras': static['top_cameras'] + [
                    {**static['top_cameras'][0], 'name': 'cctv_missing'}]}, 'zone_eval_top_v1')
        finally:
            world.close()

    def test_physics_trace_is_identical_with_and_without_the_profile(self):
        import numpy as np
        from sim.camera_robot_port import CameraRobotPort
        traces = []
        for profile in (None, 'zone_eval_top_v2'):
            scene, world = self.world()
            try:
                if profile:
                    zet.apply_to_world(world, scene.config['static_map'], profile)
                port = CameraRobotPort(world, 'r1', allow_reverse=True, allow_mecanum=True)
                port.apply({'kind': 'mecanum', 'forward': .12, 'left': .03, 'turn': .05, 'duration_s': .5},
                           float(world.data.time))
                trace = []
                for _ in range(int(.6/float(world.model.opt.timestep))):
                    port.tick(float(world.data.time))
                    world._physics_step_for(world.controllers['r1'])
                    trace.append(world.data.qpos.copy())
                traces.append(np.array(trace))
            finally:
                world.close()
        self.assertTrue(np.array_equal(traces[0], traces[1]))
        self.assertGreater(float(np.abs(traces[0][-1] - traces[0][0]).max()), .01)

    def test_robot_wrist_rgb_is_byte_identical_and_the_moved_top_differs(self):
        import numpy as np
        frames = []
        for profile in (None, 'zone_eval_top_v2'):
            scene, world = self.world(render=True)
            try:
                if profile:
                    zet.apply_to_world(world, scene.config['static_map'], profile)
                wrist = {rid: world.render_rgb(robot_id=rid, camera='robot_cam') for rid in sorted(world.controllers)}
                top = world.render_team_jpeg(camera=MOVED, quality=95)
                frames.append((wrist, top))
            finally:
                world.close()
        for rid in frames[0][0]:
            self.assertTrue(np.array_equal(frames[0][0][rid], frames[1][0][rid]), rid)
        self.assertNotEqual(frames[0][1], frames[1][1])

    def test_moved_top_sees_the_whole_lane_and_bay_with_the_env_v3_measurement(self):
        spec = importlib.util.spec_from_file_location('eval_topcam', X/'eval_topcam.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        sc, _ = mod.static_checks()
        floor = mod.Floor(sc, 'zone_wide_corridor_tags_v3')
        try:
            authored = floor.fractions(floor.authored)
            moved = floor.fractions(V2_POSITION)
        finally:
            floor.close()
        # env v3 static_results.json (source f843d392), reproduced by the same code path
        self.assertEqual(authored['z0.003'][mod.LANE], .86896)
        self.assertEqual(authored['z0.003'][mod.BAY], .81778)
        for level in moved:
            self.assertEqual(moved[level][mod.LANE], 1.)
            self.assertEqual(moved[level][mod.BAY], 1.)
            for region, value in authored[level].items():
                self.assertGreaterEqual(moved[level][region], value, (level, region))
        self.assertTrue(math.isclose(mod.box_area_px(V2_POSITION[2]), 73.8, abs_tol=.05))


if __name__ == '__main__':
    unittest.main()
