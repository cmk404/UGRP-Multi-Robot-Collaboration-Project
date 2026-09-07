from __future__ import annotations

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from sim.grasp_physics import GROUND_POSE, PhysicsGraspWorld


class MasterPiGraspEnv(gym.Env):
    """Continuous-control RL environment for contact-based floor grasping.

    Action (7): six arm target deltas + jaw closure scalar, all normalized [-1, 1].
    Observation: arm/finger state, block pose/velocity relative to gripper, contacts/forces.

    The base is placed in grasp range by the episode initializer. This first
    curriculum stage deliberately learns only the hard part: physical grasp + lift.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(self, max_steps: int = 260, frame_skip: int = 8, seed: int = 0):
        super().__init__()
        self.max_steps = int(max_steps)
        self.frame_skip = int(frame_skip)
        self.world = PhysicsGraspWorld(seed=seed, render=False)
        obs = self.world.observation()
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=obs.shape, dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
        self._step_count = 0
        self._prev_z = .075
        self._prev_distance = 1.0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is None:
            seed = int(self.np_random.integers(0, 2**31 - 1))
        rng = np.random.default_rng(seed)
        # Domain randomization: target pose and approach standoff vary each episode.
        bx = 1.15 + float(rng.uniform(-.06, .06))
        by = float(rng.uniform(-.30, .30))
        self.world.reset(seed=seed, block_xy=(bx, by))
        self.world.move_base_to_grasp_range(float(rng.uniform(.455, .495)))
        self.world.step_physics(25)
        self.world.open_gripper()

        # Curriculum stage 1 starts near a viable pre-grasp pose but not exactly on it.
        pose = GROUND_POSE.copy()
        pose += rng.normal(0.0, [0.035, 0.04, 0.05, 0.05, 0.04, 0.035])
        self.world._set_arm_target(pose)
        self.world.step_physics(180)
        self._step_count = 0
        self._prev_z = float(self.world._body_pos("red_block")[2])
        self._prev_distance = float(np.linalg.norm(self.world._body_pos("red_block") - self.world._site_pos("grip_site")))
        info = {"physics_state": self.world.state(), "curriculum": "grasp_lift_stage1"}
        return self.world.observation(), info

    def step(self, action):
        self._step_count += 1
        self.world.apply_delta_action(action, frame_skip=self.frame_skip)
        reward, info = self.world.grasp_reward(prev_block_z=self._prev_z, prev_distance=self._prev_distance)
        self._prev_z = float(self.world._body_pos("red_block")[2])
        self._prev_distance = float(np.linalg.norm(self.world._body_pos("red_block") - self.world._site_pos("grip_site")))

        # Small action regularization: prefer smooth, economical corrections.
        action = np.asarray(action, dtype=np.float32)
        reward -= 0.006 * float(np.square(action[:6]).mean())

        success = bool(info["success"])
        block = self.world._body_pos("red_block")
        robot = self.world._body_pos("robot")
        lost = bool(np.linalg.norm(block[:2] - robot[:2]) > .95 or block[2] < .02)
        terminated = success or lost
        truncated = self._step_count >= self.max_steps
        info.update({
            "is_success": success,
            "lost": lost,
            "step": self._step_count,
            "physics_state": self.world.state(),
        })
        return self.world.observation(), float(reward), terminated, truncated, info

    def render(self):
        # Rendering is intentionally omitted from training workers for speed.
        # Use PhysicsGraspWorld(render=True) for evaluation videos.
        return None

    def close(self):
        self.world.close()
        super().close()
