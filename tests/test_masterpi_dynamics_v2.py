import math
import unittest

import mujoco
import numpy as np

from harness.real_geometry import forward_kinematics
from sim.masterpi_camera_profile import mujoco_pixel_intrinsic
from sim.masterpi_dynamics_v2 import (
    MasterPiDynamicsV2,
    TARGET_BLOCK_HALF_M,
    OFFICIAL_TOTAL_MASS_KG,
    WHEEL_RADIUS_M,
    REDUCED_MECANUM_SUPPORT_FRICTION,
)


class MasterPiDynamicsV2Tests(unittest.TestCase):
    def setUp(self):
        self.world = MasterPiDynamicsV2(seed=3)

    def tearDown(self):
        self.world.close()

    def test_robot_mass_and_free_base_match_structural_contract(self):
        w = self.world
        self.assertAlmostEqual(w.robot_mass_kg, OFFICIAL_TOTAL_MASS_KG, places=6)
        joint_names = {
            mujoco.mj_id2name(w.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(w.model.njnt)
        }
        self.assertIn('base_free', joint_names)
        self.assertNotIn('base_x', joint_names)
        self.assertNotIn('base_y', joint_names)
        self.assertNotIn('base_yaw', joint_names)
        for name in ('wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint'):
            self.assertIn(name, joint_names)

    def test_idle_robot_settles_on_all_four_wheels(self):
        w = self.world
        x, y, z = w.base_xyz()
        roll, pitch, _ = w.base_rpy()
        self.assertLess(abs(x), .001)
        self.assertLess(abs(y), .001)
        self.assertLess(abs(z - .034), .003)
        self.assertLess(abs(roll), .005)
        self.assertLess(abs(pitch), .005)
        self.assertGreaterEqual(w.contact_summary()['wheel_floor_contacts'], 4)

    def test_pwm_mapping_preserves_legacy_fk_shape_with_explicit_floor_offset(self):
        """Servo-angle mapping stays compatible while the nominal frame is corrected.

        The deployed REAL controller historically treated LINK_1=9.3 cm as a
        floor-to-shoulder height.  The official MasterPi geometry shows it is an
        axle/base-to-shoulder offset, so the corrected nominal twin is exactly one
        65 mm wheel radius higher in z.  Keeping that offset explicit prevents a
        future regression from lowering the SIM arm merely to satisfy legacy FK.
        """
        poses = [
            {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500},
            {1: 1500, 3: 650, 4: 2230, 5: 1920, 6: 1500},
            {1: 2000, 3: 960, 4: 2410, 5: 1215, 6: 1500},
        ]
        legacy_floor_offset_cm = WHEEL_RADIUS_M * 100.0
        for pose in poses:
            self.world.set_base_pose_for_test()
            self.world.set_servo_pulses(pose, forward_only=True)
            cam = self.world.site_xyz('camera_axis_site')
            grip = self.world.site_xyz('grip_site')
            real_cam = forward_kinematics(pose, tool_length_cm=7.0)
            real_grip = forward_kinematics(pose, tool_length_cm=10.0)
            self.assertAlmostEqual(cam[0] * 100.0, real_cam.radius_cm, places=6)
            self.assertAlmostEqual(cam[2] * 100.0, real_cam.height_cm + legacy_floor_offset_cm, places=6)
            self.assertAlmostEqual(grip[0] * 100.0, real_grip.radius_cm, places=6)
            self.assertAlmostEqual(grip[2] * 100.0, real_grip.height_cm + legacy_floor_offset_cm, places=6)

    def test_camera_intrinsics_match_real_calibration(self):
        cid = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, 'robot_cam')
        np.testing.assert_allclose(
            np.asarray(self.world.model.cam_intrinsic[cid], dtype=float),
            mujoco_pixel_intrinsic(self.world.width, self.world.height),
            atol=1e-5,
        )

    def test_motor_patterns_have_correct_omnidirectional_axes(self):
        cases = [
            ([1, 1, 1, 1], 'forward'),
            ([-1, 1, 1, -1], 'left'),
            ([-1, 1, -1, 1], 'yaw'),
        ]
        results = {}
        for command, name in cases:
            w = MasterPiDynamicsV2(seed=4)
            try:
                p0 = w.base_xyz().copy()
                yaw0 = w.base_rpy()[2]
                w.step(motor_commands=command, duration_s=.6)
                d = w.base_xyz() - p0
                results[name] = (d, w.base_rpy()[2] - yaw0)
            finally:
                w.close()

        forward, fyaw = results['forward']
        self.assertGreater(forward[0], .05)
        self.assertLess(abs(forward[1]), .02)
        self.assertLess(abs(fyaw), .05)

        left, lyaw = results['left']
        self.assertGreater(left[1], .03)
        self.assertLess(abs(left[0]), .02)
        self.assertLess(abs(lyaw), .08)

        rotate, ryaw = results['yaw']
        self.assertGreater(ryaw, .10)
        self.assertLess(np.linalg.norm(rotate[:2]), .02)

    def test_reduced_mecanum_support_proxy_does_not_side_scrub_lateral_motion(self):
        # Traction is already represented by the explicit reduced mecanum body
        # wrench.  The invisible wheel cylinders must therefore remain almost
        # frictionless support proxies; otherwise their cylindrical contact adds
        # a second tyre model and suppresses strafe motion.
        wheel_ids = [
            mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, f"wheel_{name}")
            for name in self.world.wheel_names
        ]
        for gid in wheel_ids:
            self.assertLessEqual(
                float(self.world.model.geom_friction[gid, 0]),
                REDUCED_MECANUM_SUPPORT_FRICTION + 1e-12,
            )

        def displacement(command):
            w = MasterPiDynamicsV2(seed=4, render=False, use_calibration_manifest=False)
            try:
                p0 = w.base_xyz().copy()
                w.step(motor_commands=command, duration_s=.20)
                return w.base_xyz() - p0
            finally:
                w.close()

        forward = displacement([.875, .875, .875, .875])
        left = displacement([-.875, .875, .875, -.875])
        forward_m = abs(float(forward[0]))
        lateral_m = abs(float(left[1]))
        self.assertGreater(forward_m, .005)
        self.assertGreater(lateral_m, .005)
        # The provisional wrench itself uses 1.65/2.2 = 0.75 lateral/forward
        # authority.  Support contact must not collapse that to the old ~0.26.
        self.assertGreater(lateral_m / forward_m, .65)
        self.assertLess(lateral_m / forward_m, .90)

    def test_real_task_collisions_are_enabled_without_fabricated_workcell(self):
        for geom_name in ('floor', 'red_block_geom', 'base_lower_collision', 'upper_arm_collision', 'left_finger'):
            gid = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            self.assertGreaterEqual(gid, 0, geom_name)
            self.assertNotEqual(int(self.world.model.geom_contype[gid]), 0, geom_name)
            self.assertNotEqual(int(self.world.model.geom_conaffinity[gid]), 0, geom_name)
        for fake_name in ('wall_back', 'workbench', 'rack_post_a', 'delivery_blue'):
            self.assertEqual(
                mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, fake_name),
                -1,
            )

    def test_open_floor_long_drive_remains_supported_and_upright(self):
        w = MasterPiDynamicsV2(seed=8)
        try:
            # Keep task objects out of the chassis path: this test isolates the
            # open-floor drivetrain, not collision-induced deflection.
            for i, body_name in enumerate(('red_block', 'blue_block', 'yellow_block')):
                w.set_free_body_pose_for_reset(body_name, (0.8, 0.8 + 0.12 * i, TARGET_BLOCK_HALF_M), 0.0)
            w.step(motor_commands=[1, 1, 1, 1], duration_s=2.0)
            x, y, z = w.base_xyz()
            roll, pitch, _ = w.base_rpy()
            self.assertGreater(float(x), .15)
            self.assertLess(abs(float(y)), .04)
            self.assertLess(abs(float(z) - float(WHEEL_RADIUS_M)), .006)
            self.assertLess(abs(float(roll)), .04)
            self.assertLess(abs(float(pitch)), .04)
        finally:
            w.close()

    def test_chassis_contact_physically_pushes_block(self):
        w = MasterPiDynamicsV2(seed=9)
        try:
            bid = mujoco.mj_name2id(w.model, mujoco.mjtObj.mjOBJ_BODY, 'red_block')
            jid = int(w.model.body_jntadr[bid])
            qa = int(w.model.jnt_qposadr[jid])
            da = int(w.model.jnt_dofadr[jid])
            # A 30 mm task cube fits below the chassis centreline in the current
            # nominal geometry. Place it in the front-wheel path to verify genuine
            # robot/block contact rather than relying on the old 50 mm centre hit.
            w.data.qpos[qa:qa + 3] = [.22, .055, TARGET_BLOCK_HALF_M]
            w.data.qpos[qa + 3:qa + 7] = [1, 0, 0, 0]
            w.data.qvel[da:da + 6] = 0
            mujoco.mj_forward(w.model, w.data)
            red0 = np.array(w.data.qpos[qa:qa + 3], dtype=float)
            w.step(motor_commands=[.6, .6, .6, .6], duration_s=1.2)
            red1 = np.array(w.data.qpos[qa:qa + 3], dtype=float)
            dx = float(red1[0] - red0[0])
            dy = float(red1[1] - red0[1])
            self.assertGreater(dx, .015)
            # This is a structural contact test, not a calibrated collision
            # trajectory.  A lightweight foam target hit by the edge of a
            # mecanum wheel can travel substantially after separation, and we
            # have no REAL metrology for that post-impact distance yet.  Do not
            # turn an arbitrary SIM displacement cap into fake calibration.
            # Require only the structural invariants we can defend here: the
            # contact pushes predominantly forward and the cube stays on the
            # floor instead of tunnelling or being launched vertically.
            self.assertLess(abs(dy), dx)
            # Sample floor stability after contact has had time to settle. A
            # lightweight foam cube can be briefly airborne at the exact end of
            # the drive pulse without representing an unstable/launched model.
            w.step(motor_commands=[0, 0, 0, 0], duration_s=.8)
            red2 = np.array(w.data.qpos[qa:qa + 3], dtype=float)
            self.assertLess(abs(float(red2[2]) - TARGET_BLOCK_HALF_M), .004)
        finally:
            w.close()


if __name__ == '__main__':
    unittest.main()
