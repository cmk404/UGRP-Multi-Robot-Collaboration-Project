from __future__ import annotations

import unittest

import mujoco
import numpy as np

from harness.real_geometry import SERVO_DEVIATION

from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2
from sim.masterpi_camera_profile import CAMERA_LOCAL_POS_M, CAMERA_LOCAL_QUAT_WXYZ
from sim.masterpi_geometry import (
    OFFICIAL_TOTAL_LENGTH_M,
    OFFICIAL_TOTAL_WIDTH_M,
    OFFICIAL_WHEEL_DIAMETER_M,
    NOMINAL_DECK_TOP_FROM_FLOOR_M,
    NOMINAL_BODY_TOP_FROM_FLOOR_M,
    NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M,
    NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M,
    NOMINAL_LOWER_BODY_HEIGHT_M,
    NOMINAL_CAGE_STANDOFF_HEIGHT_M,
    NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M,
    NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M,
    NOMINAL_DECK_LENGTH_M,
    NOMINAL_DECK_WIDTH_M,
    NOMINAL_WHEEL_WIDTH_M,
    NOMINAL_WHEELBASE_M,
    NOMINAL_TRACK_M,
    NOMINAL_PI_CAGE_CENTER_X_M,
    NOMINAL_PI_CAGE_HALF_LENGTH_M,
    NOMINAL_PI_CAGE_HALF_WIDTH_M,
    NOMINAL_PI_BOARD_HALF_LENGTH_M,
    NOMINAL_PI_BOARD_HALF_WIDTH_M,
    NOMINAL_SHOULDER_FLOOR_M,
    NOMINAL_ELBOW_FLOOR_M,
    NOMINAL_WRIST_FLOOR_M,
    NOMINAL_YAW_TO_SHOULDER_M,
    OFFICIAL_REFERENCE_PX_PER_MM,
    OFFICIAL_REFERENCE_JOINT_Y_PX,
    OFFICIAL_REFERENCE_WHEEL_BOTTOM_Y_PX,
    CONTROLLER_TOOL_M,
    ARM_LINK_HALF_HEIGHT_M,
    ARM_PLATE_HALF_WIDTH_M,
)


class MasterPiPhysicalGeometryTests(unittest.TestCase):
    def setUp(self):
        self.world = MasterPiDynamicsV2(seed=8, render=False, use_calibration_manifest=False)

    def tearDown(self):
        self.world.close()

    def _body(self, name: str) -> int:
        return mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def _geom(self, name: str) -> int:
        return mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, name)

    def test_wheel_envelope_matches_manufacturer_dimensions(self):
        model = self.world.model
        fl = self._body("wheel_fl_body")
        rr = self._body("wheel_rr_body")
        wheel = self._geom("wheel_fl")
        radius = float(model.geom_size[wheel][0])
        half_width = float(model.geom_size[wheel][1])
        length = abs(float(model.body_pos[fl][0]) - float(model.body_pos[rr][0])) + 2.0 * radius
        width = abs(float(model.body_pos[fl][1]) - float(model.body_pos[rr][1])) + 2.0 * half_width
        self.assertAlmostEqual(2.0 * radius, OFFICIAL_WHEEL_DIAMETER_M, places=6)
        self.assertAlmostEqual(length, OFFICIAL_TOTAL_LENGTH_M, places=6)
        self.assertAlmostEqual(width, OFFICIAL_TOTAL_WIDTH_M, places=6)

    def test_masterpi_uses_official_65_by_31mm_mecanum_wheels(self):
        self.assertAlmostEqual(NOMINAL_WHEEL_WIDTH_M, 0.031, places=6)
        self.assertAlmostEqual(NOMINAL_TRACK_M, 0.131, places=6)

    def test_lower_body_ends_at_front_and_rear_wheel_axes(self):
        self.assertAlmostEqual(NOMINAL_DECK_LENGTH_M, NOMINAL_WHEELBASE_M, places=6)
        model = self.world.model
        deck = self._geom("base_top")
        self.assertAlmostEqual(2.0 * float(model.geom_size[deck][0]), NOMINAL_WHEELBASE_M, places=6)

    def test_pi_stack_orientation_and_cage_fit_inside_axle_body(self):
        # Raspberry Pi 4/5 full-size boards are about 85 x 56 mm. Hiwonder's
        # assembly image mounts the long board dimension laterally across MasterPi.
        self.assertAlmostEqual(2.0 * NOMINAL_PI_BOARD_HALF_WIDTH_M, 0.085, places=6)
        self.assertAlmostEqual(2.0 * NOMINAL_PI_BOARD_HALF_LENGTH_M, 0.056, places=6)
        cage_lo = NOMINAL_PI_CAGE_CENTER_X_M - NOMINAL_PI_CAGE_HALF_LENGTH_M
        cage_hi = NOMINAL_PI_CAGE_CENTER_X_M + NOMINAL_PI_CAGE_HALF_LENGTH_M
        self.assertGreaterEqual(cage_lo, -NOMINAL_DECK_LENGTH_M / 2.0 - 1e-9)
        self.assertLessEqual(cage_hi, NOMINAL_DECK_LENGTH_M / 2.0 + 1e-9)
        self.assertLessEqual(2.0 * NOMINAL_PI_CAGE_HALF_WIDTH_M, NOMINAL_DECK_WIDTH_M)
        board = self._geom("pi_board_visual")
        self.assertAlmostEqual(float(self.world.model.geom_size[board][0]), NOMINAL_PI_BOARD_HALF_LENGTH_M, places=6)
        self.assertAlmostEqual(float(self.world.model.geom_size[board][1]), NOMINAL_PI_BOARD_HALF_WIDTH_M, places=6)

    def test_deck_top_matches_nominal_dimension_drawing(self):
        model = self.world.model
        robot = self._body("robot")
        deck = self._geom("base_top")
        top_z = float(model.body_pos[robot][2]) + float(model.geom_pos[deck][2]) + float(model.geom_size[deck][2])
        self.assertAlmostEqual(top_z, NOMINAL_DECK_TOP_FROM_FLOOR_M, places=6)

    def test_lower_chassis_is_open_inverted_u_not_closed_box(self):
        model = self.world.model
        self.assertEqual(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "base_bottom_visual"),
            -1,
        )
        self.assertAlmostEqual(NOMINAL_LOWER_BODY_HEIGHT_M, 0.030, places=6)
        self.assertLess(NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M, OFFICIAL_WHEEL_DIAMETER_M / 2.0)
        self.assertAlmostEqual(NOMINAL_DECK_TOP_FROM_FLOOR_M, 0.051, places=6)
        for name in (
            "motor_fl_gearbox_visual", "motor_fr_gearbox_visual",
            "motor_rl_gearbox_visual", "motor_rr_gearbox_visual",
        ):
            self.assertGreaterEqual(self._geom(name), 0)

    def test_cage_uses_50mm_standoff_span_to_official_101mm_top(self):
        self.assertAlmostEqual(NOMINAL_CAGE_STANDOFF_HEIGHT_M, 0.050, places=6)
        self.assertAlmostEqual(
            NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M - NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M,
            NOMINAL_CAGE_STANDOFF_HEIGHT_M,
            places=6,
        )
        self.assertAlmostEqual(NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M, NOMINAL_DECK_TOP_FROM_FLOOR_M, places=6)
        self.assertAlmostEqual(NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M, NOMINAL_BODY_TOP_FROM_FLOOR_M, places=6)

    def test_body_silhouette_stays_inside_official_101mm_chassis_height(self):
        model = self.world.model
        robot = self._body("robot")
        robot_z = float(model.body_pos[robot][2])
        # All non-arm chassis/electronics visual boxes are deliberately named
        # and bounded by the manufacturer's 101 mm side-drawing dimension.
        top = 0.0
        for name in (
            "base_top", "left_side_panel_visual",
            "right_side_panel_visual", "front_plate", "rear_plate_visual",
            "pi_board_visual", "pi_heatsink_visual", "expansion_board_visual",
            "electronics_left_side_visual", "electronics_right_side_visual",
            "electronics_top_cover_visual", "ultrasonic_bracket",
        ):
            gid = self._geom(name)
            if int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_BOX):
                top = max(top, robot_z + float(model.geom_pos[gid][2]) + float(model.geom_size[gid][2]))
        self.assertLessEqual(top, NOMINAL_BODY_TOP_FROM_FLOOR_M + 1e-9)
        self.assertGreater(top, 0.095)

    def test_body_width_fits_between_mecanum_wheels_instead_of_filling_robot_width(self):
        inner_gap = NOMINAL_TRACK_M - NOMINAL_WHEEL_WIDTH_M
        self.assertLess(NOMINAL_DECK_WIDTH_M, inner_gap)
        self.assertAlmostEqual(NOMINAL_DECK_WIDTH_M, 0.096, places=6)
        self.assertGreaterEqual(inner_gap - NOMINAL_DECK_WIDTH_M, 0.004 - 1e-9)
        self.assertLess(NOMINAL_DECK_WIDTH_M, OFFICIAL_TOTAL_WIDTH_M * 0.65)

    def test_electronics_enclosure_matches_official_perforated_cover_silhouette(self):
        model = self.world.model
        collision = self._geom("rear_cage_collision")
        self.assertEqual(float(model.geom_rgba[collision][3]), 0.0)

        # Official assembly imagery shows thin side sheets and a broad slotted
        # top cover. The previous rail-only cage was a visual interpretation
        # error and must not return.
        for name in (
            "electronics_left_side_visual", "electronics_right_side_visual",
            "electronics_top_cover_visual",
            "cage_front_left_standoff", "cage_front_right_standoff",
            "cage_mid_left_standoff", "cage_mid_right_standoff",
            "cage_rear_left_standoff", "cage_rear_right_standoff",
            "pi_board_visual", "expansion_board_visual",
        ):
            self.assertGreaterEqual(self._geom(name), 0)
        for removed in (
            "cage_top_visual", "cage_top_rear_visual", "cage_top_left_visual",
            "cage_top_right_visual", "cage_top_cross_visual",
            "cage_left_brace_visual", "cage_right_brace_visual",
        ):
            self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, removed), -1)

        top = self._geom("electronics_top_cover_visual")
        self.assertAlmostEqual(float(model.geom_size[top][0]), NOMINAL_PI_CAGE_HALF_LENGTH_M, places=6)
        self.assertAlmostEqual(float(model.geom_size[top][1]), NOMINAL_PI_CAGE_HALF_WIDTH_M, places=6)
        for idx in range(1, 5):
            slot = self._geom(f"top_cover_slot_{idx}")
            self.assertEqual(int(model.geom_contype[slot]), 0)
            self.assertEqual(int(model.geom_conaffinity[slot]), 0)

    def test_each_wheel_has_visible_mecanum_rollers(self):
        model = self.world.model
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            for gid in range(model.ngeom)
        }
        for wheel in ("fl", "fr", "rl", "rr"):
            rollers = [name for name in names if name and name.startswith(f"wheel_{wheel}_roller_")]
            spokes = [name for name in names if name and name.startswith(f"wheel_{wheel}_hub_spoke_")]
            self.assertEqual(len(rollers), 8)
            self.assertEqual(len(spokes), 8)

    def test_arm_topology_and_tool_envelope_match_masterpi(self):
        model = self.world.model
        joint_names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            for jid in range(model.njnt)
        }
        for name in ("arm_yaw", "shoulder", "elbow", "wrist_pitch", "left_gripper_close", "right_gripper_close"):
            self.assertIn(name, joint_names)

        finger = self._geom("left_finger")
        finger_lo = float(model.geom_pos[finger][0]) - float(model.geom_size[finger][0])
        finger_hi = float(model.geom_pos[finger][0]) + float(model.geom_size[finger][0])
        # CONTROLLER_TOOL_M is the grasp centre/TCP, not the physical tip.
        self.assertLess(finger_lo, CONTROLLER_TOOL_M)
        self.assertGreater(finger_hi, CONTROLLER_TOOL_M)
        grip_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
        self.assertAlmostEqual(float(model.site_pos[grip_site][0]), CONTROLLER_TOOL_M, places=6)

    def test_straight_reference_axis_heights_match_official_side_photo(self):
        pose = {
            1: 2000,
            3: 1500 + SERVO_DEVIATION[3],
            4: 1500 + SERVO_DEVIATION[4],
            5: 1500 + SERVO_DEVIATION[5],
            6: 1500,
        }
        self.world.set_base_pose_for_test()
        self.world.set_servo_pulses(pose, forward_only=True)
        mujoco.mj_forward(self.world.model, self.world.data)

        expected = {
            "shoulder": NOMINAL_SHOULDER_FLOOR_M,
            "elbow": NOMINAL_ELBOW_FLOOR_M,
            "wrist": NOMINAL_WRIST_FLOOR_M,
        }
        bodies = {"shoulder": "shoulder_link", "elbow": "elbow_link", "wrist": "wrist_link"}
        for name, body_name in bodies.items():
            z_m = float(self.world.data.xpos[self._body(body_name)][2])
            self.assertAlmostEqual(z_m, expected[name], places=6)
            photo_mm = (
                OFFICIAL_REFERENCE_WHEEL_BOTTOM_Y_PX - OFFICIAL_REFERENCE_JOINT_Y_PX[name]
            ) / OFFICIAL_REFERENCE_PX_PER_MM
            # Pixel picking on a product photograph is only a reference check,
            # not metrology.  The nominal CAD-like geometry should nevertheless
            # land within a few millimetres of all three visible pitch axes.
            self.assertLess(abs(z_m * 1000.0 - photo_mm), 4.0, name)

        yaw_z = float(self.world.data.xpos[self._body("arm_base")][2])
        shoulder_z = float(self.world.data.xpos[self._body("shoulder_link")][2])
        self.assertAlmostEqual(shoulder_z - yaw_z, NOMINAL_YAW_TO_SHOULDER_M, places=6)
        self.assertGreater(shoulder_z, yaw_z)

    def test_eye_in_hand_camera_is_rigid_to_gripper_with_measured_hand_eye(self):
        cid = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, "robot_cam")
        gripper = self._body("gripper")
        poses = [
            {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500},
            {1: 1500, 3: 650, 4: 2230, 5: 1920, 6: 1500},
            {1: 2000, 3: 960, 4: 2410, 5: 1215, 6: 1700},
        ]
        expected_local = np.asarray(CAMERA_LOCAL_POS_M, dtype=float)
        expected_rot = np.zeros(9, dtype=float)
        mujoco.mju_quat2Mat(expected_rot, np.asarray(CAMERA_LOCAL_QUAT_WXYZ, dtype=float))
        expected_rot = expected_rot.reshape(3, 3)
        for pose in poses:
            self.world.set_base_pose_for_test()
            self.world.set_servo_pulses(pose, forward_only=True)
            self.world._sync_real_camera_mount()
            body_rot = self.world.data.xmat[gripper].reshape(3, 3)
            local = body_rot.T @ (self.world.data.cam_xpos[cid] - self.world.data.xpos[gripper])
            local_rot = body_rot.T @ self.world.data.cam_xmat[cid].reshape(3, 3)
            np.testing.assert_allclose(local, expected_local, atol=1e-9)
            np.testing.assert_allclose(local_rot, expected_rot, atol=1e-8)

    def test_arm_visual_links_are_narrower_than_joint_cheeks(self):
        # Side-photo proportions: joint cheeks are broad, but the inter-joint
        # members are thin.  This prevents a return to the old orange-plank look.
        self.assertLess(ARM_LINK_HALF_HEIGHT_M, ARM_PLATE_HALF_WIDTH_M * 0.5)
        upper = self._geom("upper_arm_plate_left")
        forearm = self._geom("forearm_plate_left")
        self.assertAlmostEqual(float(self.world.model.geom_size[upper][2]), ARM_LINK_HALF_HEIGHT_M, places=6)
        self.assertAlmostEqual(float(self.world.model.geom_size[forearm][2]), ARM_LINK_HALF_HEIGHT_M, places=6)

    def test_v2_scene_contains_no_decorative_nonreal_workcell(self):
        model = self.world.model
        geom_names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            for gid in range(model.ngeom)
        }
        for forbidden in ("wall_back", "rack_post_a", "workbench", "delivery_blue", "crate_yellow"):
            self.assertNotIn(forbidden, geom_names)
        self.assertIn("floor", geom_names)
        for block in ("red_block_geom", "blue_block_geom", "yellow_block_geom"):
            self.assertIn(block, geom_names)

    def test_visual_servo_and_camera_parts_exist_separately_from_collision_proxies(self):
        model = self.world.model
        geom_names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            for gid in range(model.ngeom)
        }
        required = {
            "shoulder_servo_visual",
            "elbow_servo_visual",
            "wrist_servo_visual",
            "upper_arm_plate_left",
            "upper_arm_plate_right",
            "camera_body",
            "camera_lens_visual",
            "gripper_crossbar_visual",
            "upper_arm_collision",
            "forearm_collision",
        }
        self.assertTrue(required <= geom_names)


if __name__ == "__main__":
    unittest.main()
