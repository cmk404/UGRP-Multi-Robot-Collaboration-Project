import math
import unittest

from harness.visual_arm import (
    CAMERA_LOCAL_X_CM,
    CAMERA_LOCAL_Z_CM,
    camera_extrinsics,
    camera_to_base,
    camera_optical_to_robot_base,
    forward_grip,
    solve_grip_ik,
    solve_grip_site_ik,
    tool_pose,
)


POSE = {3: 672, 4: 2181, 5: 1746, 6: 1500}


class VisualArmTests(unittest.TestCase):
    def test_camera_origin_uses_static_mount_and_owned_servo_fk(self):
        wrist = tool_pose(POSE, tool_length_cm=0.0)
        camera = camera_optical_to_robot_base(POSE, (0.0, 0.0, 0.0))
        pitch = math.radians(wrist.pitch_deg)
        self.assertAlmostEqual(camera[0], wrist.x_m + (CAMERA_LOCAL_X_CM * math.cos(pitch) - CAMERA_LOCAL_Z_CM * math.sin(pitch)) / 100.0)
        self.assertAlmostEqual(camera[1], 0.0)
        self.assertAlmostEqual(camera[2], wrist.z_m + (CAMERA_LOCAL_X_CM * math.sin(pitch) + CAMERA_LOCAL_Z_CM * math.cos(pitch)) / 100.0)

    def test_optical_right_rotates_to_robot_right_at_center_pan(self):
        origin = camera_optical_to_robot_base(POSE, (0.0, 0.0, 0.0))
        point = camera_optical_to_robot_base(POSE, (0.01, 0.0, 0.0))
        self.assertAlmostEqual(point[0], origin[0])
        self.assertAlmostEqual(point[1], origin[1] - 0.01)
        self.assertAlmostEqual(point[2], origin[2])
        alias = camera_to_base((0.01, 0.0, 0.0), POSE)
        self.assertEqual(alias, point)
        extrinsic_origin, axes = camera_extrinsics(POSE)
        self.assertEqual(extrinsic_origin, origin)
        self.assertAlmostEqual(sum(value * value for value in axes[2]), 1.0)

    def test_grip_ik_round_trips_floor_frame_target(self):
        target = (0.165, 0.0, 0.0525)
        solved = solve_grip_site_ik(target, preferred_pitch_deg=-66.0)
        actual = tool_pose(solved)
        self.assertAlmostEqual(actual.x_m, target[0], delta=0.0015)
        self.assertAlmostEqual(actual.y_m, target[1], delta=0.0015)
        self.assertAlmostEqual(actual.z_m, target[2], delta=0.0015)
        self.assertEqual(set(solved), {3, 4, 5, 6})
        self.assertTrue(all(500 <= pulse <= 2500 for pulse in solved.values()))
        self.assertEqual(forward_grip(solved), (actual.x_m, actual.y_m, actual.z_m))

    def test_runner_grip_interface_supports_floor_cube_center(self):
        solved = solve_grip_ik(0.165, 0.0, 0.016, -90.0)
        actual = forward_grip(solved)
        self.assertAlmostEqual(actual[0], 0.165, delta=0.0015)
        self.assertAlmostEqual(actual[2], 0.016, delta=0.0015)

    def test_grip_ik_yaw_is_radial_and_bounded(self):
        angle = math.radians(10.0)
        solved = solve_grip_site_ik((0.165 * math.cos(angle), 0.165 * math.sin(angle), 0.0525))
        self.assertAlmostEqual(solved[6], 1500 + 10.0 * (2000.0 / 180.0), delta=1.0)
        with self.assertRaisesRegex(ValueError, "grasp envelope"):
            solve_grip_site_ik((0.25, 0.0, 0.05))
        with self.assertRaisesRegex(ValueError, "grasp sector"):
            solve_grip_site_ik((0.14, 0.10, 0.05))

    def test_rejects_missing_nonfinite_and_unsupported_inputs(self):
        with self.assertRaises(ValueError):
            camera_optical_to_robot_base({3: 1500}, (0.0, 0.0, 0.2))
        with self.assertRaises(ValueError):
            camera_optical_to_robot_base(POSE, (0.0, math.nan, 0.2))
        with self.assertRaises(ValueError):
            solve_grip_site_ik((0.165, 0.0, math.inf))


if __name__ == "__main__":
    unittest.main()
