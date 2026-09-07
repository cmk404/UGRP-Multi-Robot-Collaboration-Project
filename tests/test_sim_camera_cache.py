from __future__ import annotations

import unittest

import numpy as np

from sim.real_stack_adapter import SimCamera


class _Clock:
    def sleep(self, seconds: float) -> None:
        del seconds


class _Data:
    def __init__(self) -> None:
        self.qpos = np.zeros(12, dtype=float)
        self.ctrl = np.zeros(6, dtype=float)
        self.eq_active = np.zeros(2, dtype=np.uint8)


class _World:
    def __init__(self) -> None:
        self.data = _Data()
        self.render_calls = 0
        self._latest_robot_bgr = None
        self._latest_robot_frame_seq = 0
        self._latest_robot_jpeg = None
        self._latest_robot_jpeg_seq = None
        self._latest_robot_jpeg_quality = None
        self._presentation_dirty = True

    def render_rgb(self, camera: str = "robot_cam") -> np.ndarray:
        self.assert_camera(camera)
        self.render_calls += 1
        # A deterministic RGB frame is enough to verify that the detector sees
        # the same actor-visible pixels on a cache hit.
        return np.full((4, 6, 3), self.render_calls, dtype=np.uint8)

    @staticmethod
    def assert_camera(camera: str) -> None:
        if camera != "robot_cam":
            raise AssertionError(camera)


class SimCameraSceneCacheTests(unittest.TestCase):
    def setUp(self) -> None:
        self.world = _World()
        self.camera = SimCamera(self.world, _Clock())

    def test_static_confirmation_reuses_actor_visible_frame(self) -> None:
        first = self.camera.read()
        second = self.camera.read()

        self.assertEqual(self.world.render_calls, 1)
        self.assertEqual(self.camera.sequence, 2)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(self.world._latest_robot_frame_seq, 2)

    def test_base_motion_invalidates_cache(self) -> None:
        self.camera.read()
        self.world.data.qpos[0] += SimCamera._QPOS_TOLERANCE * 2

        self.camera.read()

        self.assertEqual(self.world.render_calls, 2)

    def test_object_and_peer_motion_invalidate_shared_scene_cache(self) -> None:
        self.camera.read()
        # The signature covers the complete shared qpos, including free-body
        # object and peer state, rather than only this camera's robot joints.
        self.world.data.qpos[7] += SimCamera._QPOS_TOLERANCE * 2
        self.camera.read()
        self.world.data.qpos[10] += SimCamera._QPOS_TOLERANCE * 2
        self.camera.read()

        self.assertEqual(self.world.render_calls, 3)

    def test_servo_command_change_invalidates_even_before_joint_moves(self) -> None:
        self.camera.read()
        self.world.data.ctrl[2] = 0.25

        self.camera.read()

        self.assertEqual(self.world.render_calls, 2)

    def test_subpixel_settling_noise_within_tolerance_reuses_frame(self) -> None:
        self.camera.read()
        self.world.data.qpos[0] += SimCamera._QPOS_TOLERANCE * 0.25

        self.camera.read()

        self.assertEqual(self.world.render_calls, 1)


if __name__ == "__main__":
    unittest.main()
