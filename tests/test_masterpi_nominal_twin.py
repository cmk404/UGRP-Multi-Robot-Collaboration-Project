from __future__ import annotations

from pathlib import Path
import unittest

import mujoco
import numpy as np

from sim.masterpi_dynamics_v2 import XML, GRIPPER_MAX_CLOSE_M, TARGET_BLOCK_HALF_M, TARGET_BLOCK_SIDE_M
from sim.masterpi_production_v2 import MasterPiProductionV2, _detect_color
from harness.real_geometry import project_detection
from sim.masterpi_camera_profile import (
    CAMERA_CALIBRATION_ID,
    CAMERA_MOUNT_STATUS,
    CAMERA_LOCAL_POS_M,
    CAMERA_LOCAL_QUAT_WXYZ,
    CAMERA_PARITY_FIT_LOCAL_POS_M,
    CAMERA_PARITY_FIT_LOCAL_QUAT_WXYZ,
    mujoco_pixel_intrinsic,
)
from sim.real_stack_adapter import (
    MigratedRealStack,
    SimMotorTransport,
    _patched_modules,
    physical_motor_api_to_v2,
    precision_mod,
)


ROOT = Path(__file__).resolve().parents[1]


class _MotorWorld:
    def __init__(self):
        self.commands = None

    def set_motor_commands(self, values):
        self.commands = np.asarray(values, dtype=float).copy()


class MasterPiNominalTwinTests(unittest.TestCase):
    def test_observer_renderer_is_higher_resolution_without_changing_robot_sensor_contract(self):
        world = MasterPiProductionV2(seed=1, width=640, height=480, render=False)
        self.assertEqual((world.width, world.height), (640, 480))
        self.assertGreaterEqual(world.observer_width, 1280)
        self.assertGreaterEqual(world.observer_height, 720)
        self.assertGreater(world.observer_width * world.observer_height, world.width * world.height)

    def test_static_scene_is_exact_runtime_v2_scene(self):
        self.assertEqual((ROOT / "sim/masterpi_scene_v2.xml").read_text(encoding="utf-8"), XML)

    def test_v2_task_object_and_gripper_match_real_nominal_geometry(self):
        world = MasterPiProductionV2(seed=1, render=False)
        try:
            gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, "red_block_geom")
            self.assertGreaterEqual(gid, 0)
            self.assertAlmostEqual(float(world.model.geom_size[gid][0]) * 2.0, TARGET_BLOCK_SIDE_M, places=6)
            self.assertAlmostEqual(float(world.body_xyz("red_block")[2]), TARGET_BLOCK_HALF_M, delta=0.001)
            for jid in world.gripper_joint:
                self.assertAlmostEqual(float(world.model.jnt_range[jid][1]), GRIPPER_MAX_CLOSE_M, places=6)
        finally:
            world.close()

    def test_robot_sensor_uses_centered_provisional_mount_and_measured_intrinsics(self):
        world = MasterPiProductionV2(seed=1, render=False)
        try:
            np.testing.assert_allclose(
                np.asarray(world.model.cam_pos[world.robot_cam_cid], dtype=float),
                np.asarray(CAMERA_LOCAL_POS_M, dtype=float),
                atol=1e-10,
            )
            # q and -q represent the same rotation; compare rotation matrices.
            expected_r = np.zeros(9, dtype=float)
            actual_r = np.zeros(9, dtype=float)
            mujoco.mju_quat2Mat(expected_r, np.asarray(CAMERA_LOCAL_QUAT_WXYZ, dtype=float))
            mujoco.mju_quat2Mat(actual_r, np.asarray(world.model.cam_quat[world.robot_cam_cid], dtype=float))
            np.testing.assert_allclose(actual_r, expected_r, atol=1e-9)
            np.testing.assert_allclose(
                np.asarray(world.model.cam_intrinsic[world.robot_cam_cid], dtype=float),
                mujoco_pixel_intrinsic(world.width, world.height),
                atol=1e-5,
            )
            self.assertEqual(world.state()["camera_calibration_id"], CAMERA_CALIBRATION_ID)
            self.assertEqual(world.state()["camera_mount_status"], CAMERA_MOUNT_STATUS)
            self.assertTrue(world.state()["camera_raw_fisheye"])
            for name in ("camera_bracket", "camera_body", "camera_lens_visual"):
                gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                self.assertEqual(int(world.model.geom_group[gid]), 5)
        finally:
            world.close()

    def test_robot_sensor_mount_is_centered_and_old_single_pose_fit_is_archival_only(self):
        # Production mount is mechanically centered; the historical one-pose fit
        # is preserved for provenance but must not silently become the mount.
        self.assertAlmostEqual(float(CAMERA_LOCAL_POS_M[1]), 0.0, places=12)
        self.assertGreater(abs(float(CAMERA_PARITY_FIT_LOCAL_POS_M[1])), 0.005)
        self.assertFalse(np.allclose(CAMERA_LOCAL_POS_M, CAMERA_PARITY_FIT_LOCAL_POS_M))
        self.assertFalse(np.allclose(CAMERA_LOCAL_QUAT_WXYZ, CAMERA_PARITY_FIT_LOCAL_QUAT_WXYZ))

        rot = np.zeros(9, dtype=float)
        mujoco.mju_quat2Mat(rot, np.asarray(CAMERA_LOCAL_QUAT_WXYZ, dtype=float))
        rot = rot.reshape(3, 3)
        optical_forward = -rot[:, 2]
        # gripper +X is forward; centered mount has no lateral optical yaw.
        self.assertAlmostEqual(float(optical_forward[1]), 0.0, delta=1e-9)
        self.assertGreater(float(optical_forward[0]), 0.98)
        self.assertGreater(float(optical_forward[2]), 0.10)

    def test_robot_sensor_range_change_is_not_exaggerated_by_camera_geometry(self):
        world = MasterPiProductionV2(seed=11, render=True, use_calibration_manifest=False)
        try:
            ranges = []
            z = float(world.data.xpos[world.robot_bid][2])
            for x in (0.0, 0.06):
                world.set_free_body_pose_for_reset("robot", [x, 0.0, z], yaw=0.0)
                detection = _detect_color(world.render_rgb("robot_cam"), "red")
                self.assertTrue(detection.get("visible"))
                projection = project_detection(world.servo_command_pulses, detection)
                self.assertIsNotNone(projection)
                ranges.append(float(projection.range_m))
            estimated_drop_cm = (ranges[0] - ranges[1]) * 100.0
            self.assertGreater(estimated_drop_cm, 3.5)
            self.assertLess(estimated_drop_cm, 7.52)
        finally:
            world.close()

    def test_front_third_person_camera_follows_robot_from_ahead(self):
        world = MasterPiProductionV2(seed=1, render=False)
        try:
            world.set_base_pose_for_test()
            world._sync_front_follow_camera()
            start = np.asarray(world.model.cam_pos[world.front_cam_cid], dtype=float).copy()
            world.set_free_body_pose_for_reset("robot", [0.30, -0.20, float(world.data.xpos[world.robot_bid][2])], yaw=np.pi / 2.0)
            world._sync_front_follow_camera()
            moved = np.asarray(world.model.cam_pos[world.front_cam_cid], dtype=float).copy()
            robot = np.asarray(world.data.xpos[world.robot_bid], dtype=float)
            # yaw=+90 deg => robot forward is +Y, so the viewer sits ahead in +Y.
            self.assertGreater(float(moved[1] - robot[1]), 0.85)
            self.assertAlmostEqual(float(moved[0] - robot[0]), 0.0, delta=0.03)
            self.assertGreater(float(np.linalg.norm(moved - start)), 0.20)
        finally:
            world.close()

    def test_nominal_profile_has_no_hidden_sim_adjustments(self):
        world = MasterPiProductionV2(seed=1, render=False)
        try:
            state = world.state()
            self.assertEqual(state["digital_twin_profile"], "nominal_real_contract_v1")
            self.assertEqual(state["sim_only_adjustments"], [])
        finally:
            world.close()

    def test_motor_transport_is_exact_physical_api_to_v2_mapping(self):
        world = _MotorWorld()
        transport = SimMotorTransport(world)
        raw = np.zeros(4, dtype=float)
        sequence = [(1, -35), (2, 35), (3, 35), (4, -35)]
        for motor, speed in sequence:
            raw[motor - 1] = speed / 40.0
            transport.write_motor(motor, speed)
            np.testing.assert_allclose(world.commands, physical_motor_api_to_v2(raw), atol=1e-12)

    def test_sim_patch_does_not_widen_real_fingertip_envelope(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = MigratedRealStack(world)
            before = float(precision_mod.CALIBRATED_FINGERTIP_RADIUS_MAX_CM)
            with _patched_modules(stack):
                self.assertEqual(float(precision_mod.CALIBRATED_FINGERTIP_RADIUS_MAX_CM), before)
            self.assertEqual(float(precision_mod.CALIBRATED_FINGERTIP_RADIUS_MAX_CM), before)
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
