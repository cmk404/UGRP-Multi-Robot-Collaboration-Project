from __future__ import annotations

import unittest

import numpy as np
import cv2

from sim.warehouse_observation import WarehouseObservationAdapter, WarehouseSensorConfig, camera_intrinsics_from_vertical_fovy


class _World:
    def __init__(self, rgb, depth):
        self.rgb = rgb
        self.depth = depth
        self.warehouse_state_calls = 0

    def render_rgb(self, *, robot_id, camera):
        self.last_render = (robot_id, camera)
        return self.rgb

    def warehouse_state(self):
        self.warehouse_state_calls += 1
        return {"cargo": {"oak_plank_01": {"position": (99, 99, 99)}}}


class WarehouseObservationTests(unittest.TestCase):
    def _sample(self):
        rgb = np.zeros((32, 48, 3), dtype=np.uint8)
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 11, 16)
        rgb[7:25, 15:33] = 255
        rgb[8:24, 16:32] = np.repeat(marker[..., None], 3, axis=2)
        depth = np.full((32, 48), 1.2, dtype=np.float32)
        return rgb, depth

    def test_rgbd_observation_contains_camera_local_geometry_and_provenance(self):
        rgb, depth = self._sample()
        world = _World(rgb, depth)
        obs = WarehouseObservationAdapter(
            WarehouseSensorConfig(fx_px=24, fy_px=24, min_pixels=3),
            render_depth=lambda **_: depth,
        ).observe(world, "r2")
        cargo = obs["cargos"]["oak_plank_01"]
        self.assertTrue(cargo["visible"])
        self.assertEqual(cargo["geometry"]["frame"], "robot_camera")
        self.assertAlmostEqual(cargo["geometry"]["depth_m"], 1.2, places=5)
        self.assertIn("depth", cargo["provenance"])
        self.assertEqual(obs["peers"], [])
        self.assertFalse(obs["provenance"]["privileged_state_used"])
        self.assertEqual(world.last_render, ("r2", "robot_cam"))
        self.assertEqual(world.warehouse_state_calls, 0)

    def test_isolated_yellow_tower_cube_is_not_crate(self):
        rgb = np.zeros((32, 48, 3), dtype=np.uint8)
        rgb[10:15, 20:25] = (242, 183, 13)
        obs = WarehouseObservationAdapter(WarehouseSensorConfig(min_pixels=3)).observe(_World(rgb, None), "r1")
        self.assertNotIn("wood_crate_01", obs["cargos"])
        self.assertEqual(obs["detection_status"]["wood_crate_01"]["reason"], "ARUCO_NOT_DECODED")

    def test_depth_uses_label_component_only_and_does_not_mutate_input(self):
        rgb, _ = self._sample()
        depth = np.full((32, 48), 3.0, dtype=np.float32)
        depth[8:24, 16:32] = 1.0
        original = depth.copy()
        obs = WarehouseObservationAdapter(
            WarehouseSensorConfig(fx_px=24, fy_px=24, min_pixels=3),
            render_depth=lambda **_: depth,
        ).observe(_World(rgb, depth), "r1")
        self.assertAlmostEqual(obs["cargos"]["oak_plank_01"]["geometry"]["depth_m"], 1.0)
        np.testing.assert_array_equal(depth, original)

    def test_vertical_fovy_uses_height_for_focal_length(self):
        intrinsics = camera_intrinsics_from_vertical_fovy(640, 480, 60.0)
        expected = 480.0 / (2.0 * np.tan(np.deg2rad(60.0) / 2.0))
        self.assertAlmostEqual(intrinsics["fx_px"], expected)
        self.assertAlmostEqual(intrinsics["fy_px"], expected)

    def test_missing_depth_stays_missing_instead_of_inventing_pose(self):
        rgb, _ = self._sample()
        obs = WarehouseObservationAdapter(WarehouseSensorConfig(min_pixels=3)).observe(_World(rgb, None), "r1")
        cargo = obs["detection_status"]["oak_plank_01"]
        self.assertIsNone(cargo["geometry"])
        self.assertEqual(cargo["reason"], "DEPTH_UNAVAILABLE")

    def test_occluded_depth_stays_missing(self):
        rgb, _ = self._sample()
        depth = np.full((32, 48), np.nan, dtype=np.float32)
        obs = WarehouseObservationAdapter(
            WarehouseSensorConfig(min_pixels=3), render_depth=lambda **_: depth,
        ).observe(_World(rgb, depth), "r1")
        cargo = obs["detection_status"]["oak_plank_01"]
        self.assertIsNone(cargo["geometry"])
        self.assertEqual(cargo["reason"], "DEPTH_OCCLUDED_OR_INVALID")

    def test_privileged_metadata_perturbation_does_not_change_observation(self):
        rgb, depth = self._sample()
        adapter = WarehouseObservationAdapter(
            WarehouseSensorConfig(fx_px=24, fy_px=24, min_pixels=3),
            render_depth=lambda **_: depth,
        )
        first = adapter.observe(_World(rgb, depth), "r1")
        second_world = _World(rgb, depth)
        second_world.secret = {"cargo_pose": (1234, -987, 55), "peer_poses": {"r2": (88, 77)}}
        second = adapter.observe(second_world, "r1")
        self.assertEqual(first, second)
        self.assertNotIn("secret", second)


if __name__ == "__main__":
    unittest.main()
