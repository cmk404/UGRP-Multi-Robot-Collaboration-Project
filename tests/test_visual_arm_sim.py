"""Self-robot calibration checks against settled MuJoCo joints and sites."""

import math
import unittest

import mujoco
import numpy as np

from harness.visual_arm import camera_extrinsics, forward_grip, solve_grip_ik
from sim.masterpi_production_v2 import HOVER_POSE, MasterPiProductionV2, SEARCH_POSE


class VisualArmSimCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = MasterPiProductionV2(
            seed=1, render=False, use_calibration_manifest=False
        )

    @classmethod
    def tearDownClass(cls):
        cls.world.close()

    def _settle(self, pose):
        self.world.set_servo_pulses(pose)
        for _ in range(500):
            self.world._physics_step()

    def _world_to_robot_floor(self, point):
        base = np.asarray(self.world.base_xyz(), dtype=float)
        yaw = float(self.world.base_rpy()[2])
        delta = np.asarray(point, dtype=float) - base
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.asarray(
            (cy * delta[0] + sy * delta[1], -sy * delta[0] + cy * delta[1], point[2]),
            dtype=float,
        )

    def _actual_camera(self):
        origin = self._world_to_robot_floor(self.world.data.cam_xpos[self.world.robot_cam_cid])
        rotation_world = np.asarray(
            self.world.data.cam_xmat[self.world.robot_cam_cid], dtype=float
        ).reshape(3, 3)
        yaw = float(self.world.base_rpy()[2])
        world_to_base = np.asarray(
            ((math.cos(yaw), math.sin(yaw), 0.0),
             (-math.sin(yaw), math.cos(yaw), 0.0),
             (0.0, 0.0, 1.0)),
            dtype=float,
        )
        # MuJoCo camera columns are right, up, backward. OpenCV uses right,
        # down, forward.
        axes = np.column_stack(
            (rotation_world[:, 0], -rotation_world[:, 1], -rotation_world[:, 2])
        )
        return origin, world_to_base @ axes

    def test_settled_grip_fk_at_search_hover_and_low_grasp(self):
        low = {1: 2000, **solve_grip_ik(0.165, 0.0, 0.016, -90.0)}
        cases = (SEARCH_POSE, HOVER_POSE, low)
        for pose in cases:
            with self.subTest(pose=pose):
                self._settle(pose)
                expected = np.asarray(forward_grip(pose), dtype=float)
                actual = self._world_to_robot_floor(self.world.site_xyz("grip_site"))
                # Position actuators settle close to their targets but retain
                # gravity/contact compliance; this is the practical command-FK
                # rather than an exact encoder-FK equality.
                np.testing.assert_allclose(actual, expected, atol=0.006)

    def test_settled_camera_extrinsics_match_own_camera_transform(self):
        for pose in (SEARCH_POSE, HOVER_POSE):
            with self.subTest(pose=pose):
                self._settle(pose)
                expected_origin, expected_axes = camera_extrinsics(pose)
                actual_origin, actual_axes = self._actual_camera()
                np.testing.assert_allclose(actual_origin, expected_origin, atol=0.006)
                np.testing.assert_allclose(
                    actual_axes, np.asarray(expected_axes, dtype=float).T, atol=0.035
                )


if __name__ == "__main__":
    unittest.main()
