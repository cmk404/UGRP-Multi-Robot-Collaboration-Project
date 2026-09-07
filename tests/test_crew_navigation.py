from __future__ import annotations

import math
import unittest

import numpy as np

from sim.crew_navigation import ForwardPathController
FORWARD_PATTERN = np.ones(4, dtype=float)
YAW_LEFT_PATTERN = np.array([-1.0, 1.0, -1.0, 1.0], dtype=float)


class ForwardPathControllerTests(unittest.TestCase):
    def test_target_behind_rotates_before_forward(self):
        controller = ForwardPathController([( -1.0, 0.0)])
        command = controller.update((0.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.state, "ROTATE")
        self.assertGreater(abs(float(np.dot(command, YAW_LEFT_PATTERN))), 0.0)
        self.assertAlmostEqual(float(np.dot(command, FORWARD_PATTERN)), 0.0)

    def test_forward_projection_is_never_negative(self):
        controller = ForwardPathController([(1.0, 0.0)])
        for _ in range(20):
            command = controller.update((0.0, 0.0), 0.0, 0.02)
            self.assertGreaterEqual(float(np.dot(command, FORWARD_PATTERN)), -1e-9)

    def test_instances_complete_independently(self):
        first = ForwardPathController([(0.0, 0.0)])
        second = ForwardPathController([(1.0, 0.0)])
        first.update((0.0, 0.0), 0.0, 0.02)
        self.assertTrue(first.done)
        self.assertFalse(second.done)
        self.assertEqual(second.waypoint_index, 0)
        self.assertEqual(second.target, (1.0, 0.0))

    def test_final_yaw_is_only_applied_after_path(self):
        controller = ForwardPathController([(0.0, 0.0)], final_yaw=math.pi / 2)
        command = controller.update((0.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.state, "BRAKE")
        for _ in range(40):
            command = controller.update((0.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.state, "FINAL_YAW")
        self.assertAlmostEqual(float(np.dot(command, FORWARD_PATTERN)), 0.0)
        self.assertGreater(float(np.dot(command, YAW_LEFT_PATTERN)), 0.0)

    def test_l_turn_reorients_before_forward(self):
        controller = ForwardPathController([(1.0, 0.0), (1.0, 1.0)])
        controller.update((0.0, 0.0), 0.0, 0.02)
        command = controller.update((1.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.waypoint_index, 1)
        self.assertEqual(controller.state, "BRAKE")
        self.assertAlmostEqual(float(np.dot(command, FORWARD_PATTERN)), 0.0)

    def test_reverse_after_waypoint_turns_without_retreat(self):
        controller = ForwardPathController([(1.0, 0.0), (0.0, 0.0)])
        controller.update((0.0, 0.0), 0.0, 0.02)
        command = controller.update((1.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.waypoint_index, 1)
        self.assertEqual(controller.state, "BRAKE")
        self.assertAlmostEqual(float(np.dot(command, FORWARD_PATTERN)), 0.0)

    def test_intermediate_waypoint_has_wider_acceptance_than_final(self):
        controller = ForwardPathController(
            [(1.0, 0.0), (2.0, 0.0)], tolerance_m=0.018,
            intermediate_tolerance_m=0.065,
        )
        controller.update((0.0, 0.0), 0.0, 0.02)
        controller.update((1.0, 0.055), 0.0, 0.02)
        self.assertEqual(controller.waypoint_index, 1)
        self.assertFalse(controller.done)

    def test_final_yaw_keeps_position_anchor(self):
        controller = ForwardPathController(
            [(1.0, 0.0)], final_yaw=math.pi / 2, tolerance_m=0.035,
        )
        command = controller.update((1.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.state, "BRAKE")
        for _ in range(40):
            command = controller.update((1.0, 0.0), 0.0, 0.02)
        self.assertEqual(controller.state, "FINAL_YAW")
        self.assertGreater(abs(float(np.dot(command, YAW_LEFT_PATTERN))), 0.0)
        # A displaced chassis receives a translation component while turning.
        displaced = controller.update((0.8, 0.0), 0.2, 0.02)
        self.assertGreater(float(np.dot(displaced, FORWARD_PATTERN)), 0.0)
        self.assertFalse(controller.done)


if __name__ == "__main__":
    unittest.main()
