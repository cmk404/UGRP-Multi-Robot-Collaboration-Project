from __future__ import annotations

import math
import unittest
from unittest import mock

import numpy as np

from sim.masterpi_production_v2 import MasterPiProductionV2, WHEEL_RADIUS_M
from sim.masterpi_dynamics_v2 import TARGET_BLOCK_HALF_M, TARGET_BLOCK_LIFT_CENTER_M
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.real_stack_adapter import MigratedRealStack, _patched_modules, precision_mod


class RealStackSimServoTests(unittest.TestCase):
    def test_namespaced_clock_fast_path_advances_without_full_state_per_chunk(self):
        world = MultiMasterPiProductionV2(seed=2, render=False)
        try:
            stack = MigratedRealStack(world.controllers["r1"])
            start = float(world.controllers["r1"].data.time)
            with mock.patch.object(world.controllers["r1"], "state", wraps=world.controllers["r1"].state) as state:
                stack.clock.sleep(0.10)
            self.assertGreater(float(world.controllers["r1"].data.time), start)
            state.assert_not_called()
        finally:
            world.close()

    def test_nudge_duration_is_interpolated_in_sim_not_applied_instantly(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = MigratedRealStack(world)
            start = int(world.servo_command_pulses[5])
            target = min(2500, start + 600)
            stack.robot.nudge_servos({5: target}, duration=1.0)
            current = int(world.servo_command_pulses[5])
            self.assertGreater(current, start)
            self.assertLess(current, target)
            # Robot.pose mirrors the hardware command target, while MuJoCo's
            # actuator target is still physically traversing toward it.
            self.assertEqual(stack.robot.pose[5], target)
            self.assertIn(5, stack.robot._servo_motions)
            stack.clock.sleep(1.0)
            self.assertEqual(int(world.servo_command_pulses[5]), target)
            self.assertNotIn(5, stack.robot._servo_motions)
        finally:
            world.close()

    def test_closed_grasp_lifts_continuously_through_real_arm_helper(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            world.set_base_pose_for_test((0.0, 0.0, WHEEL_RADIUS_M), 0.0)
            # Use a floor-contact pose from the corrected nominal kinematics.
            # The old 650/2230/1920 fixture depended on the legacy SIM arm being
            # 32.5 mm too low and must not be preserved as fake REAL agreement.
            grasp = {1: 2000, 3: 1100, 4: 2250, 5: 2300, 6: 1500}
            hover = {1: 1500, 3: 1100, 4: 2250, 5: 2100, 6: 1500}
            world.set_servo_pulses(grasp, forward_only=True)
            # Place the 30-mm fixture at the actual MasterPi pad/TCP centre, not
            # at the legacy oversized-finger contact proxy.
            grip = world.site_xyz("grip_site")
            world.set_free_body_pose_for_reset("red_block", (float(grip[0]), float(grip[1]), TARGET_BLOCK_HALF_M), 0.0)
            for _ in range(100):
                world._physics_step(np.zeros(4))

            stack = MigratedRealStack(world)
            published: list[tuple[float, float, float]] = []
            world.frame_callback = lambda: published.append(tuple(float(v) for v in world.body_xyz("red_block")))
            # Keep the test quick; the same interpolation must hold at all UI speeds.
            world.set_speed_multiplier(3.0)
            stack.robot.move_servo(1, 1500, 0.85)
            self.assertTrue(world.finger_block_contact("red")["bilateral"])
            with _patched_modules(stack):
                precision_mod.move_arm_together(
                    stack.robot,
                    {3: hover[3], 4: hover[4], 5: hover[5]},
                    duration=1.20,
                )

            contact = world.finger_block_contact("red")
            self.assertTrue(contact["bilateral"])
            self.assertGreater(float(world.body_xyz("red_block")[2]), TARGET_BLOCK_LIFT_CENTER_M)
            self.assertGreaterEqual(len(published), 12)
            jumps = [math.dist(a, b) for a, b in zip(published, published[1:])]
            # A 1--10 cm state jump is the old teleport-like failure. With the
            # servo-board duration emulation, each published step stays small.
            self.assertLess(max(jumps), 0.015)
            lift_heights = [p[2] for p in published if p[2] > 0.030]
            self.assertGreaterEqual(len(lift_heights), 6)
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
