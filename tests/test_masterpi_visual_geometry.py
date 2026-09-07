import unittest

import mujoco

from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2
from sim.masterpi_geometry import (
    ARM_PLATE_THICKNESS_M,
    ARM_PLATE_SIDE_OFFSET_M,
    ARM_LINK_RAIL_Z_OFFSET_M,
    GRIPPER_PAD_HALF_LENGTH_M,
    GRIPPER_PAD_HALF_WIDTH_M,
    GRIPPER_PAD_HALF_HEIGHT_M,
    MICRO_SERVO_BODY_HALF,
    SERVO_BODY_HALF,
)


class MasterPiVisualGeometryTests(unittest.TestCase):
    def setUp(self):
        self.world = MasterPiDynamicsV2(seed=1, render=False, use_calibration_manifest=False)

    def tearDown(self):
        self.world.close()

    def _geom(self, name):
        gid = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        self.assertGreaterEqual(gid, 0)
        return gid

    def test_standard_servo_visual_uses_official_40x20x40p5_envelope(self):
        for name in ("base_yaw_servo_visual", "shoulder_servo_visual", "elbow_servo_visual", "wrist_servo_visual"):
            gid = self._geom(name)
            got = tuple(float(x) for x in self.world.model.geom_size[gid])
            for actual, expected in zip(got, SERVO_BODY_HALF):
                self.assertAlmostEqual(actual, expected, places=6)

    def test_arm_side_plates_are_two_mm_not_old_six_mm(self):
        for name in ("upper_arm_plate_left", "upper_arm_plate_right", "forearm_plate_left", "forearm_plate_right", "wrist_plate_left", "wrist_plate_right"):
            gid = self._geom(name)
            self.assertAlmostEqual(float(self.world.model.geom_size[gid][1]), ARM_PLATE_THICKNESS_M, places=6)
        self.assertAlmostEqual(2.0 * ARM_PLATE_THICKNESS_M, 0.002, places=6)

    def test_arm_visual_width_is_not_the_old_bulky_44mm(self):
        left = self._geom("upper_arm_plate_left")
        right = self._geom("upper_arm_plate_right")
        outer = (
            float(self.world.model.geom_pos[left][1]) + float(self.world.model.geom_size[left][1])
            - (float(self.world.model.geom_pos[right][1]) - float(self.world.model.geom_size[right][1]))
        )
        self.assertAlmostEqual(outer, 2.0 * (ARM_PLATE_SIDE_OFFSET_M + ARM_PLATE_THICKNESS_M), places=6)
        self.assertLess(outer, 0.030)

    def test_gripper_servo_uses_lfd01m_micro_envelope(self):
        gid = self._geom("gripper_servo_visual")
        got = tuple(float(x) for x in self.world.model.geom_size[gid])
        for actual, expected in zip(got, MICRO_SERVO_BODY_HALF):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_arm_links_are_open_two_rail_frames(self):
        for upper, lower in (
            ("upper_arm_plate_left", "upper_arm_plate_left_lower"),
            ("upper_arm_plate_right", "upper_arm_plate_right_lower"),
            ("forearm_plate_left", "forearm_plate_left_lower"),
            ("forearm_plate_right", "forearm_plate_right_lower"),
        ):
            u = self._geom(upper)
            l = self._geom(lower)
            self.assertAlmostEqual(float(self.world.model.geom_pos[u][2]), ARM_LINK_RAIL_Z_OFFSET_M, places=6)
            self.assertAlmostEqual(float(self.world.model.geom_pos[l][2]), -ARM_LINK_RAIL_Z_OFFSET_M, places=6)
            self.assertGreater(
                float(self.world.model.geom_pos[u][2]) - float(self.world.model.geom_size[u][2]),
                float(self.world.model.geom_pos[l][2]) + float(self.world.model.geom_size[l][2]),
            )

    def test_gripper_visual_pads_are_separate_from_contact_proxies(self):
        expected_pad = (
            GRIPPER_PAD_HALF_LENGTH_M,
            GRIPPER_PAD_HALF_WIDTH_M,
            GRIPPER_PAD_HALF_HEIGHT_M,
        )
        for side in ("left", "right"):
            pad = self._geom(f"{side}_finger_pad_visual")
            contact = self._geom(f"{side}_finger")
            got = tuple(float(x) for x in self.world.model.geom_size[pad])
            for actual, expected in zip(got, expected_pad):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertEqual(int(self.world.model.geom_contype[pad]), 0)
            self.assertEqual(int(self.world.model.geom_conaffinity[pad]), 0)
            self.assertEqual(float(self.world.model.geom_rgba[contact][3]), 0.0)
            self.assertGreater(int(self.world.model.geom_contype[contact]), 0)
            self._geom(f"{side}_jaw_link_visual")
            self._geom(f"{side}_jaw_link_upper_visual")
            self._geom(f"{side}_jaw_pivot_visual")


if __name__ == "__main__":
    unittest.main()
