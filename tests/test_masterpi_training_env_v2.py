from __future__ import annotations

import unittest
import numpy as np

from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2, TARGET_BLOCK_HALF_M, TARGET_BLOCK_LIFT_CENTER_M, WHEEL_RADIUS_M
from sim.masterpi_training_env_v2 import MasterPiTrainingEnvV2, SERVO_IDS, SERVO_DELTA_PER_STEP
from sim.masterpi_production_v2 import MasterPiProductionV2


class MasterPiTrainingEnvV2Tests(unittest.TestCase):
    def test_policy_observation_is_transfer_constrained(self):
        env = MasterPiTrainingEnvV2(seed=3, width=96, height=72)
        try:
            obs, info = env.reset(seed=3)
            self.assertEqual(obs.shape, (18,))
            self.assertEqual(env.action_space.shape, (9,))
            self.assertEqual(info["observation_contract"], "RGB_TARGET_FEATURES+CAMERA_COMMANDED_FK+COMMANDED_PWM+COMMANDED_MOTORS_ONLY")
            self.assertNotIn("block_xyz", info)
            self.assertNotIn("grip_xyz", info)
            self.assertTrue(np.all(np.isfinite(obs)))
            self.assertTrue(np.all(obs >= -1.0) and np.all(obs <= 1.0))
        finally:
            env.close()

    def test_camera_target_visible_across_near_reset_seeds(self):
        env = MasterPiTrainingEnvV2(seed=0, width=96, height=72)
        try:
            hits = 0
            for seed in range(8):
                _obs, info = env.reset(seed=seed)
                hits += int(info["camera_observation"]["visible"])
            self.assertGreaterEqual(hits, 7)
        finally:
            env.close()

    def test_nominal_pwm_grasp_sequence_is_physically_feasible(self):
        # Physics regression for the corrected nominal mechanism.  The legacy
        # 650/2230/1920 fixture depended on the arm being one wheel radius too
        # low; this trajectory reaches a 15 cm floor cube with healthy servo
        # margin and uses the same timed PWM command boundary as the robot.
        w = MasterPiDynamicsV2(seed=2)
        try:
            w.set_base_pose_for_test((0.0, 0.0, WHEEL_RADIUS_M), 0.0)
            w.set_free_body_pose_for_reset("red_block", (0.150, 0.0, TARGET_BLOCK_HALF_M), 0.0)
            hover = {1: 2000, 3: 1143, 4: 2406, 5: 1936, 6: 1500}
            grasp = {1: 2000, 3: 1447, 4: 2352, 5: 2350, 6: 1500}
            closed = {**grasp, 1: 1500}
            lift = {**hover, 1: 1500}
            w.move_pose_timed(hover, lowering=False)
            w.move_pose_timed(grasp, lowering=True)
            w.move_pose_timed(closed, lowering=True)
            w.move_pose_timed(lift, lowering=False)
            contact = w.finger_block_contact("red")
            self.assertTrue(contact["bilateral"])
            self.assertGreater(float(w.body_xyz("red_block")[2]), TARGET_BLOCK_LIFT_CENTER_M)
        finally:
            w.close()

    def test_grasp_ready_can_physically_grasp_using_env_step_only(self):
        # Regression for the actual RL action boundary. No world teleport, direct
        # servo API, contact injection, weld, or hidden-state motion is allowed
        # after reset: every movement below goes through env.step(action).
        env = MasterPiTrainingEnvV2(
            seed=0, width=96, height=72, max_steps=120, curriculum="grasp_ready"
        )
        try:
            obs, reset_info = env.reset(seed=0)
            self.assertTrue(reset_info["camera_observation"]["visible"])
            sequence = (
                {1: 2000, 3: 1143, 4: 2406, 5: 1936, 6: 1500},  # hover
                {1: 2000, 3: 1447, 4: 2352, 5: 2350, 6: 1500},  # descend
                {1: 1500, 3: 1447, 4: 2352, 5: 2350, 6: 1500},  # close jaws
                {1: 1500, 3: 1143, 4: 2406, 5: 1936, 6: 1500},  # lift
            )
            success = False
            info = reset_info
            for target in sequence:
                for _ in range(80):
                    action = np.zeros(9, dtype=np.float32)
                    done = True
                    # Reconstruct current commanded PWM strictly from the policy
                    # observation rather than peeking at MuJoCo joint state.
                    for j, servo_id in enumerate(SERVO_IDS):
                        current = 1500.0 + float(obs[9 + j]) * 1000.0
                        delta = float(target[servo_id]) - current
                        if abs(delta) > 1.0:
                            done = False
                        action[4 + j] = np.clip(
                            delta / float(SERVO_DELTA_PER_STEP[j]), -1.0, 1.0
                        )
                    if done:
                        break
                    obs, _reward, terminated, truncated, info = env.step(action)
                    success = bool(info["is_success"])
                    if terminated or truncated:
                        break
                if success or info.get("lost") or info.get("tipped"):
                    break
            if not success:
                for _ in range(8):
                    obs, _reward, terminated, truncated, info = env.step(
                        np.zeros(9, dtype=np.float32)
                    )
                    success = bool(info["is_success"])
                    if terminated or truncated:
                        break
            self.assertTrue(success, info)
            truth = info["privileged_eval"]
            self.assertTrue(truth["bilateral_contact"])
            self.assertGreater(truth["block_height_m"], TARGET_BLOCK_LIFT_CENTER_M)
            self.assertGreater(truth["left_force_n"], 0.0)
            self.assertGreater(truth["right_force_n"], 0.0)
        finally:
            env.close()

    def test_camera_housing_is_not_a_fake_grasp_obstacle(self):
        w = MasterPiDynamicsV2(seed=2)
        try:
            import mujoco
            gid = mujoco.mj_name2id(w.model, mujoco.mjtObj.mjOBJ_GEOM, "camera_body")
            self.assertEqual(int(w.model.geom_contype[gid]), 0)
            self.assertEqual(int(w.model.geom_conaffinity[gid]), 0)
        finally:
            w.close()


    def test_stale_logical_grasp_is_cleared_when_gripper_is_open_and_block_is_on_floor(self):
        world = MasterPiProductionV2(seed=11, width=96, height=72, render=False)
        try:
            world.grasp_color = "red"
            world.spatial_memory["red"] = {"relation": "HELD"}
            world.set_servo_pulses({1: 2000}, forward_only=True)
            state = world.state()
            self.assertIsNone(state["grasp_color"])
            self.assertFalse(state["bilateral_contact"])
            self.assertFalse(state["lifted"])
            self.assertEqual(world.spatial_memory["red"]["relation"], "GROUND")
        finally:
            world.close()

if __name__ == "__main__":
    unittest.main()
