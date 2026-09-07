"""Transfer-constrained Gymnasium environment for MasterPiDynamicsV2.

The policy observation intentionally contains only signals available on the
physical MasterPi:

* red-target features extracted from robot-camera RGB;
* the last commanded PWM values for servos 1/3/4/5/6;
* the last four chassis motor commands.

MuJoCo body positions, velocities and contact forces are *privileged*.  They are
used only to compute reward, termination and evaluation info, never observation.
This makes the environment suitable for simulator research before calibration
and structurally suitable for sim-to-real after the v2 fidelity gate is passed.
"""
from __future__ import annotations

import math
from typing import Mapping

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2, TARGET_BLOCK_HALF_M, TARGET_BLOCK_LIFT_CENTER_M, WHEEL_RADIUS_M
from harness.real_geometry import project_detection

SERVO_IDS = (1, 3, 4, 5, 6)
SERVO_MIN = 500
SERVO_MAX = 2500
SEARCH_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
# Observed REAL pre-capture command from physical_state_machine_reference.py.
# It is retained as a transferable camera-start command, not as evidence that
# the legacy REAL FK height is the nominal mechanism geometry.  The corrected
# nominal twin keeps the explicit +32.5 mm floor-frame offset and therefore has
# its own physically feasible grasp-ready radius/arm trajectory below.
PRECAPTURE_POSE = {1: 2000, 3: 500, 4: 2320, 5: 1320, 6: 1500}
NOMINAL_GRASP_READY_DISTANCE_M = 0.150
NOMINAL_GRASP_READY_JITTER_M = 0.002
# One 100-ms policy step should not imply an impossible instantaneous PWM jump.
# Conservative command-rate envelope: 40--50 PWM per 100 ms. This stays
# below the ~800 PWM/s long-move rate used by the physical move_duration law
# while still allowing 0.1-s visual-servo corrections like REAL nudge_servos.
SERVO_DELTA_PER_STEP = np.array([50.0, 40.0, 40.0, 40.0, 50.0], dtype=np.float32)
# Require bottom clearance of at least half a block side while both jaws
# remain in physical contact.
GRASP_SUCCESS_HEIGHT_M = TARGET_BLOCK_LIFT_CENTER_M


def detect_red_from_rgb(rgb: np.ndarray) -> dict:
    """Cheap RGB detector whose output has a direct physical-camera analogue."""
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("rgb image must be HxWx3")
    h, w = rgb.shape[:2]
    r = rgb[..., 0].astype(np.int16)
    g = rgb[..., 1].astype(np.int16)
    b = rgb[..., 2].astype(np.int16)
    mask = (r > 55) & (r > 3.2 * g) & (r > 3.2 * b) & ((r - g) > 35)
    ys, xs = np.where(mask)
    min_pixels = max(5, int(h * w * 0.00035))
    if xs.size < min_pixels:
        return {"visible": False, "cx": 0.5, "cy": 0.5, "area_ratio": 0.0, "bbox_w": 0.0, "bbox_h": 0.0, "bbox": None}
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    # Reject obvious full-frame/large red artifacts; a 30-mm task cube cannot occupy
    # this much of the image in the supported manipulation envelope.
    area_ratio = float(xs.size) / float(h * w)
    if area_ratio > 0.20:
        return {"visible": False, "cx": 0.5, "cy": 0.5, "area_ratio": 0.0, "bbox_w": 0.0, "bbox_h": 0.0, "bbox": None}
    return {
        "visible": True,
        "cx": float(xs.mean()) / float(w),
        "cy": float(ys.mean()) / float(h),
        "area_ratio": area_ratio,
        "bbox_w": float(x1 - x0 + 1) / float(w),
        "bbox_h": float(y1 - y0 + 1) / float(h),
        "bbox": [float(x0) / float(w), float(y0) / float(h), float(x1 + 1) / float(w), float(y1 + 1) / float(h)],
    }


class MasterPiTrainingEnvV2(gym.Env):
    """Low-level pick curriculum sharing the real robot's command boundary.

    Action (9 floats in [-1,1]):
      0:4   physical motor commands FL/FR/RL/RR, where magnitude 1 == speed 40
      4:9   bounded PWM deltas for servos [1,3,4,5,6]

    Observation (18 floats):
      0:6   camera-derived [visible,cx,cy,area,bbox_w,bbox_h]
      6:9   transferable camera+commanded-FK [metric_valid,range,lateral]
      9:14  last commanded PWM, normalized around 1500
      14:18 last four motor commands

    The default curriculum starts with the block in the forward manipulation
    envelope.  It does not teleport during an episode.  Reset randomization is
    allowed because reset state is not a policy action.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(
        self,
        *,
        seed: int = 0,
        max_steps: int = 240,
        control_period_s: float = 0.10,
        width: int = 160,
        height: int = 120,
        curriculum: str = "grasp_near",
        dynamics: Mapping[str, float] | None = None,
        dynamics_randomization: float = 0.0,
    ) -> None:
        super().__init__()
        if curriculum not in {"grasp_ready", "grasp_near", "pick_full"}:
            raise ValueError("curriculum must be grasp_ready, grasp_near or pick_full")
        if not 0.05 <= control_period_s <= 0.25:
            raise ValueError("control_period_s must be in [0.05, 0.25]")
        if not 0.0 <= dynamics_randomization <= 0.30:
            raise ValueError("dynamics_randomization must be in [0, 0.30]")
        self.max_steps = int(max_steps)
        self.control_period_s = float(control_period_s)
        self.curriculum = curriculum
        self._base_dynamics = None if dynamics is None else dict(dynamics)
        self.dynamics_randomization = float(dynamics_randomization)
        self._seed0 = int(seed)
        self.world = MasterPiDynamicsV2(seed=seed, dynamics=dynamics, render=True, width=width, height=height)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(9,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        self._servo = dict(SEARCH_POSE)
        self._motor = np.zeros(4, dtype=np.float32)
        self._step_count = 0
        self._stable_steps = 0
        self._prev_distance = 1.0
        self._prev_height = 0.025
        self._prev_visual_error = 1.0
        self._last_detection: dict = {}

    def close(self) -> None:
        self.world.close()
        super().close()

    def render(self) -> np.ndarray:
        return self.world.render_rgb("robot_cam")

    def _randomized_dynamics(self, rng: np.random.Generator) -> dict | None:
        if self.dynamics_randomization <= 0:
            return self._base_dynamics
        base = dict(self.world.dynamics if self._base_dynamics is None else self._base_dynamics)
        spread = self.dynamics_randomization
        for key, value in list(base.items()):
            base[key] = float(value) * float(rng.uniform(1.0 - spread, 1.0 + spread))
        return base

    def _replace_world_if_needed(self, rng: np.random.Generator, seed: int) -> None:
        if self.dynamics_randomization <= 0:
            return
        width, height = self.world.width, self.world.height
        self.world.close()
        self.world = MasterPiDynamicsV2(
            seed=seed,
            dynamics=self._randomized_dynamics(rng),
            render=True,
            width=width,
            height=height,
        )

    def _place_reset_scene(self, rng: np.random.Generator) -> None:
        self.world.set_base_pose_for_test((0.0, 0.0, WHEEL_RADIUS_M), yaw=float(rng.uniform(-0.06, 0.06)))
        if self.curriculum == "grasp_ready":
            # Stage 0 of manipulation learning: the chassis is already aligned
            # and the target is inside the corrected nominal arm's floor-grasp
            # envelope with useful servo-limit margin.  The old 0.172--0.178 m
            # fixture only worked while the SIM arm was 32.5 mm too low and now
            # pushes servo 5 against its upper limit, so it is not valid nominal
            # geometry evidence.  No teleport occurs after reset.
            distance = float(rng.uniform(
                NOMINAL_GRASP_READY_DISTANCE_M - NOMINAL_GRASP_READY_JITTER_M,
                NOMINAL_GRASP_READY_DISTANCE_M + NOMINAL_GRASP_READY_JITTER_M,
            ))
            lateral = float(rng.uniform(-0.002, 0.002))
            block_yaw = float(rng.uniform(-math.radians(4), math.radians(4)))
        elif self.curriculum == "grasp_near":
            distance = float(rng.uniform(0.22, 0.34))
            lateral = float(rng.uniform(-0.055, 0.055))
            block_yaw = float(rng.uniform(-math.pi, math.pi))
        else:
            distance = float(rng.uniform(0.28, 0.60))
            lateral = float(rng.uniform(-0.16, 0.16))
            block_yaw = float(rng.uniform(-math.pi, math.pi))
        self.world.set_free_body_pose_for_reset("red_block", (distance, lateral, TARGET_BLOCK_HALF_M), yaw=block_yaw)
        # Keep distractor blocks physical but outside the initial pickup corridor.
        self.world.set_free_body_pose_for_reset("blue_block", (0.78, -0.42, TARGET_BLOCK_HALF_M), yaw=0.0)
        self.world.set_free_body_pose_for_reset("yellow_block", (0.86, 0.42, TARGET_BLOCK_HALF_M), yaw=0.0)
        self._servo = dict(PRECAPTURE_POSE if self.curriculum == "grasp_ready" else SEARCH_POSE)
        # Small camera-pose variation models commanded-pose differences without
        # exposing unobservable joint truth to the policy. Keep grasp-ready
        # variation tighter because REAL deliberately enters a trusted fixed
        # pre-capture pose before the final visual handoff.
        tilt_jitter = 12 if self.curriculum == "grasp_ready" else 35
        pan_jitter = 20 if self.curriculum == "grasp_ready" else 50
        self._servo[3] = int(np.clip(self._servo[3] + rng.integers(-tilt_jitter, tilt_jitter + 1), SERVO_MIN, SERVO_MAX))
        self._servo[6] = int(np.clip(self._servo[6] + rng.integers(-pan_jitter, pan_jitter + 1), SERVO_MIN, SERVO_MAX))
        self.world.set_servo_pulses(self._servo, forward_only=True)
        self.world.set_motor_commands(np.zeros(4))
        # Settle all reset contacts. No teleports occur after this point.
        for _ in range(int(round(0.18 / float(self.world.model.opt.timestep)))):
            self.world._physics_step(np.zeros(4))

    def _camera_observation(self) -> tuple[np.ndarray, dict]:
        detection = detect_red_from_rgb(self.world.render_rgb("robot_cam"))
        visible = 1.0 if detection["visible"] else 0.0
        # Missing target is represented by visible=0 and neutral geometry rather
        # than fake coordinates; all values remain bounded for PPO.
        visual = np.array([
            visible,
            np.clip(2.0 * float(detection["cx"]) - 1.0, -1.0, 1.0) if visible else 0.0,
            np.clip(2.0 * float(detection["cy"]) - 1.0, -1.0, 1.0) if visible else 0.0,
            np.clip(float(detection["area_ratio"]) / 0.08, 0.0, 1.0) if visible else 0.0,
            np.clip(float(detection["bbox_w"]) / 0.45, 0.0, 1.0) if visible else 0.0,
            np.clip(float(detection["bbox_h"]) / 0.45, 0.0, 1.0) if visible else 0.0,
        ], dtype=np.float32)
        return visual, detection

    def _observation(self) -> np.ndarray:
        visual, detection = self._camera_observation()
        self._last_detection = detection
        projection = project_detection(self._servo, detection)
        if projection is None:
            metric = np.zeros(3, dtype=np.float32)
        else:
            metric = np.array([
                1.0,
                np.clip(float(projection.range_m) / 0.80, 0.0, 1.0),
                np.clip(float(projection.y_left_m) / 0.40, -1.0, 1.0),
            ], dtype=np.float32)
        pwm = np.array([(self._servo[i] - 1500.0) / 1000.0 for i in SERVO_IDS], dtype=np.float32)
        obs = np.concatenate([visual, metric, np.clip(pwm, -1, 1), np.clip(self._motor, -1, 1)]).astype(np.float32)
        return obs

    def _privileged_truth(self) -> dict:
        """Reward/evaluation only. Never concatenate this into observation."""
        block = self.world.body_xyz("red_block")
        grip = self.world.site_xyz("grip_site")
        contact = self.world.finger_block_contact("red")
        base = self.world.base_xyz()
        roll, pitch, yaw = self.world.base_rpy()
        return {
            "block_xyz": block,
            "grip_xyz": grip,
            "distance_m": float(np.linalg.norm(block - grip)),
            "block_height_m": float(block[2]),
            "contact": contact,
            "impact_n": float(self.world.nonfinger_block_contact_force("red")),
            "base_xyz": base,
            "base_rpy": np.array([roll, pitch, yaw], dtype=float),
        }

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        resolved_seed = int(self.np_random.integers(0, 2**31 - 1)) if seed is None else int(seed)
        rng = np.random.default_rng(resolved_seed)
        self._replace_world_if_needed(rng, resolved_seed)
        self.world.reset()
        self._place_reset_scene(rng)
        self._motor[:] = 0.0
        self._step_count = 0
        self._stable_steps = 0
        truth = self._privileged_truth()
        self._prev_distance = float(truth["distance_m"])
        self._prev_height = float(truth["block_height_m"])
        obs = self._observation()
        self._prev_visual_error = float(abs(obs[1]) + 0.5 * abs(obs[2])) if obs[0] > 0.5 else 1.0
        return obs, {
            "curriculum": self.curriculum,
            "camera_observation": dict(self._last_detection),
            "privileged_eval": self._public_truth(truth),
            "observation_contract": "RGB_TARGET_FEATURES+CAMERA_COMMANDED_FK+COMMANDED_PWM+COMMANDED_MOTORS_ONLY",
        }

    @staticmethod
    def _public_truth(truth: dict) -> dict:
        contact = truth["contact"]
        return {
            "distance_m": round(float(truth["distance_m"]), 4),
            "block_height_m": round(float(truth["block_height_m"]), 4),
            "bilateral_contact": bool(contact["bilateral"]),
            "left_force_n": round(float(contact["left_force_n"]), 3),
            "right_force_n": round(float(contact["right_force_n"]), 3),
            "impact_n": round(float(truth["impact_n"]), 3),
        }

    def step(self, action):
        self._step_count += 1
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(9), -1.0, 1.0)
        self._motor[:] = a[:4]
        for idx, servo_id in enumerate(SERVO_IDS):
            delta = float(a[4 + idx]) * float(SERVO_DELTA_PER_STEP[idx])
            self._servo[servo_id] = int(np.clip(round(self._servo[servo_id] + delta), SERVO_MIN, SERVO_MAX))
        self.world.set_motor_commands(self._motor)
        # Real control sends a PWM target with a duration. Interpolate command
        # targets across that duration instead of teleporting joint setpoints.
        self.world.move_servos_timed(self._servo, self.control_period_s)

        truth = self._privileged_truth()
        obs = self._observation()
        contact = truth["contact"]
        distance = float(truth["distance_m"])
        height = float(truth["block_height_m"])

        # Dense shaping uses simulator truth only in reward. It does not create a
        # sensor channel the deployed policy would depend on.
        reward = -0.025 - 0.45 * min(distance, 1.0)
        reward += 4.0 * float(np.clip(self._prev_distance - distance, -0.03, 0.03))
        reward += 90.0 * float(np.clip(height - self._prev_height, -0.015, 0.020))
        reward += 0.18 * float(contact["left"]) + 0.18 * float(contact["right"])
        reward += 0.35 * float(contact["bilateral"])

        # Transferable visual shaping prevents the easiest learned solution from
        # ignoring RGB and memorizing open-loop PWM sequences.
        if obs[0] > 0.5:
            visual_error = float(abs(obs[1]) + 0.5 * abs(obs[2]))
            reward += 0.06
            reward += 0.18 * float(np.clip(self._prev_visual_error - visual_error, -0.20, 0.20))
        else:
            visual_error = 1.0
            reward -= 0.04
        self._prev_visual_error = visual_error

        impact = float(truth["impact_n"])
        reward -= 0.012 * min(impact, 25.0)
        reward -= 0.002 * float(np.square(a).mean())
        roll, pitch, _ = truth["base_rpy"]
        tipped = bool(abs(float(roll)) > math.radians(25) or abs(float(pitch)) > math.radians(25))

        lifted_with_contact = bool(contact["bilateral"] and height > GRASP_SUCCESS_HEIGHT_M)
        self._stable_steps = self._stable_steps + 1 if lifted_with_contact else 0
        success = self._stable_steps >= 4
        if success:
            reward += 20.0

        block = truth["block_xyz"]
        base = truth["base_xyz"]
        lost = bool(np.linalg.norm(block[:2] - base[:2]) > 1.15 or block[2] < -0.005)
        terminated = bool(success or lost or tipped)
        truncated = bool(self._step_count >= self.max_steps)

        self._prev_distance = distance
        self._prev_height = height
        info = {
            "is_success": success,
            "lost": lost,
            "tipped": tipped,
            "step": self._step_count,
            "camera_observation": dict(self._last_detection),
            "commanded_pwm": dict(self._servo),
            "motor_commands": [round(float(v), 4) for v in self._motor],
            "privileged_eval": self._public_truth(truth),
            "observation_contract": "RGB_TARGET_FEATURES+CAMERA_COMMANDED_FK+COMMANDED_PWM+COMMANDED_MOTORS_ONLY",
        }
        return obs, float(reward), terminated, truncated, info
