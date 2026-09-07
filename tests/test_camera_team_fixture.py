from __future__ import annotations

import math
import unittest

import mujoco
import numpy as np

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.navigation_camera_profile import (
    NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M, NAV_CAMERA_NAME, NAV_CX_PX, NAV_CY_PX,
    NAV_FX_PX, NAV_FY_PX, NAV_IMAGE_HEIGHT, NAV_IMAGE_WIDTH, NAV_MOUNT_XYZ_M,
    calibration, ground_point_robot,
)
from sim.warehouse_arena import camera_team_arena_for_seed


class CameraTeamFixtureTests(unittest.TestCase):
    def test_fixture_has_three_independent_cyclic_box_jobs(self):
        arena = camera_team_arena_for_seed(41)
        self.assertEqual({s.cargo_id for s in arena.cargo_specs},
                         {"small_box_01", "small_box_02", "small_box_03"})
        self.assertTrue(all(s.required_carriers == 1 for s in arena.cargo_specs))
        self.assertTrue(all(s.label_color == "cyan" for s in arena.cargo_specs))
        self.assertEqual(next(s for s in arena.cargo_specs if s.cargo_id == "small_box_01").dimensions_m,
                         next(s for s in arena.cargo_specs if s.cargo_id == "small_box_03").dimensions_m)
        next_zone = {"A": "B", "B": "C", "C": "A"}
        occupied = set()
        for spec in arena.cargo_specs:
            source = next(z for z, zone in arena.zones.items()
                          if math.dist(spec.start_xyz[:2], zone.center_xy) < zone.half_extents_xy[0])
            destination = next(z for z, zone in arena.zones.items()
                               if math.dist(spec.goal_xyz[:2], zone.center_xy) < zone.half_extents_xy[0])
            occupied.add(source)
            self.assertEqual(destination, next_zone[source])
        self.assertEqual(occupied, {"A", "B", "C"})
        starts = list(arena.robot_start_poses.values())
        self.assertGreater(min(math.dist(a[:2], b[:2]) for i, a in enumerate(starts)
                               for b in starts[i + 1:]), 0.8)

    def test_nav_camera_is_fixed_to_each_robot_and_cosmetic_band_is_nonphysical(self):
        world = MultiMasterPiProductionV2(warehouse_layout="camera_team", seed=41, render=False)
        try:
            for rid in world.robot_ids:
                cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                        f"{rid}__{NAV_CAMERA_NAME}")
                robot_bid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, f"{rid}__robot")
                self.assertEqual(int(world.model.cam_bodyid[cid]), robot_bid)
                self.assertTrue(np.allclose(world.model.cam_pos[cid], NAV_MOUNT_XYZ_M, atol=1e-8))
                gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM,
                                        f"{rid}__peer_visibility_band")
                self.assertEqual(int(world.model.geom_contype[gid]), 0)
                self.assertEqual(int(world.model.geom_conaffinity[gid]), 0)
                self.assertEqual(float(world.model.body_mass[int(world.model.geom_bodyid[gid])]),
                                 float(world.model.body_mass[robot_bid]))
        finally:
            world.close()

    def test_camera_team_makes_legacy_demo_props_nonphysical_and_invisible(self):
        world = MultiMasterPiProductionV2(warehouse_layout="camera_team", seed=41, render=False)
        try:
            demo_geom_names = ("red_block_geom", "blue_block_geom",
                               "yellow_block_geom", "team_beam_geom",
                               "team_beam_r1_endpoint_geom", "team_beam_r3_endpoint_geom")
            for name in demo_geom_names:
                gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                self.assertGreaterEqual(gid, 0, name)
                self.assertEqual(int(world.model.geom_contype[gid]), 0, name)
                self.assertEqual(int(world.model.geom_conaffinity[gid]), 0, name)
                self.assertEqual(float(world.model.geom_rgba[gid, 3]), 0.0, name)
                self.assertEqual(float(world.model.body_gravcomp[
                    int(world.model.geom_bodyid[gid])]), 1.0, name)
            for cargo_id in ("small_box_01", "small_box_02", "small_box_03"):
                body_name = world.warehouse_spec_by_id[cargo_id].body_name
                self.assertGreaterEqual(mujoco.mj_name2id(
                    world.model, mujoco.mjtObj.mjOBJ_BODY, body_name), 0)
            for terrain in world.warehouse_terrain_specs:
                self.assertGreaterEqual(mujoco.mj_name2id(
                    world.model, mujoco.mjtObj.mjOBJ_GEOM, terrain.terrain_id), 0)
        finally:
            world.close()

    def test_non_camera_layout_preserves_legacy_demo_blocks(self):
        world = MultiMasterPiProductionV2(warehouse_layout="mixed", seed=41, render=False)
        try:
            for color in ("red", "blue", "yellow"):
                self.assertGreaterEqual(mujoco.mj_name2id(
                    world.model, mujoco.mjtObj.mjOBJ_BODY, f"{color}_block"), 0)
                self.assertGreaterEqual(mujoco.mj_name2id(
                    world.model, mujoco.mjtObj.mjOBJ_GEOM, f"{color}_block_geom"), 0)
                self.assertGreaterEqual(mujoco.mj_name2id(
                    world.model, mujoco.mjtObj.mjOBJ_JOINT, f"{color}_free"), 0)
                for rid in world.robot_ids:
                    self.assertGreaterEqual(mujoco.mj_name2id(
                        world.model, mujoco.mjtObj.mjOBJ_EQUALITY,
                        f"{rid}__grasp_{color}"), 0)
            beam_gid = mujoco.mj_name2id(
                world.model, mujoco.mjtObj.mjOBJ_GEOM, "team_beam_geom")
            self.assertNotEqual(int(world.model.geom_contype[beam_gid]), 0)
            self.assertGreater(float(world.model.geom_rgba[beam_gid, 3]), 0.0)
        finally:
            world.close()

    def test_static_profile_projects_lower_image_to_ground(self):
        self.assertEqual(calibration()["resolution"], [NAV_IMAGE_WIDTH, NAV_IMAGE_HEIGHT])
        centre_bottom = ground_point_robot((NAV_IMAGE_WIDTH - 1) / 2, NAV_IMAGE_HEIGHT - 1)
        self.assertIsNotNone(centre_bottom)
        self.assertGreater(centre_bottom[0], 0.05)
        self.assertAlmostEqual(centre_bottom[1], 0.0, places=8)
        self.assertIsNone(ground_point_robot((NAV_IMAGE_WIDTH - 1) / 2, 0))

    def test_profile_inverts_actual_mujoco_camera_projection_on_nominal_floor(self):
        world = MultiMasterPiProductionV2(warehouse_layout="camera_team", seed=41, render=False)
        try:
            rid = "r1"
            robot = world.robot(rid)
            cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                    f"{rid}__{NAV_CAMERA_NAME}")
            bid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, f"{rid}__robot")
            camera_origin = world.data.cam_xpos[cid].copy()
            axes = world.data.cam_xmat[cid].reshape(3, 3)
            base_xy = world.data.xpos[bid, :2].copy()
            yaw = robot.base_rpy()[2]
            expected_robot = np.asarray((0.62, 0.09))
            world_xy = base_xy + np.asarray((
                math.cos(yaw) * expected_robot[0] - math.sin(yaw) * expected_robot[1],
                math.sin(yaw) * expected_robot[0] + math.cos(yaw) * expected_robot[1],
            ))
            delta = np.asarray((world_xy[0], world_xy[1], 0.0)) - camera_origin
            cam_right, cam_up, cam_backward = axes[:, 0], axes[:, 1], axes[:, 2]
            forward = -float(np.dot(delta, cam_backward))
            u = NAV_CX_PX + NAV_FX_PX * float(np.dot(delta, cam_right)) / forward
            v = NAV_CY_PX + NAV_FY_PX * float(np.dot(delta, -cam_up)) / forward
            recovered = ground_point_robot(u, v)
            # Wheel/contact settling lowers the free base by about 0.15 mm;
            # the static nominal calibration stays within sub-millimetre error.
            self.assertTrue(np.allclose(recovered, expected_robot, atol=5e-4),
                            (recovered, expected_robot, camera_origin))
            self.assertAlmostEqual(float(camera_origin[2]), NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M, places=3)
            for zone_id in ("a", "b", "c"):
                gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM,
                                        f"warehouse_zone_{zone_id}")
                self.assertEqual(int(world.model.geom_group[gid]), 0)
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
