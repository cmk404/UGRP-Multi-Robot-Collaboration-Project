from __future__ import annotations

import json
from urllib.request import Request, urlopen

import numpy as np

from sim.grasp_physics import GROUND_POSE, PhysicsGraspWorld
from sim.mujoco_world import ActionResult


def grasp_completion(state: dict) -> bool:
    try:
        z = float((state.get("red_xyz") or [0.0, 0.0, 0.0])[2])
    except (TypeError, ValueError, IndexError):
        z = 0.0
    return bool(
        state.get("stable")
        and state.get("bilateral_contact")
        and state.get("lifted")
        and z > 0.16
    )


class GraspPolicySkill:
    """Low-level learned grasp using a separate PPO inference process.

    Keeping Torch outside the MuJoCo render worker avoids native runtime
    conflicts on the Oracle ARM host while preserving the same learned policy.
    """

    def __init__(
        self,
        policy_url: str = "http://127.0.0.1:8093",
        *,
        max_steps: int = 260,
        frame_skip: int = 8,
    ) -> None:
        self.policy_url = policy_url.rstrip("/")
        self.max_steps = int(max_steps)
        self.frame_skip = int(frame_skip)

    def _predict(self, observation: np.ndarray) -> np.ndarray:
        data = json.dumps({"observation": np.asarray(observation, dtype=float).tolist()}).encode()
        req = Request(
            self.policy_url + "/predict",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(req, timeout=5) as response:
            obj = json.loads(response.read().decode())
        if obj.get("ok") is not True:
            raise RuntimeError(obj.get("error") or "policy inference failed")
        return np.asarray(obj["action"], dtype=np.float32).reshape(7)

    def run(self, world: PhysicsGraspWorld) -> ActionResult:
        initial = world.state()
        if grasp_completion(initial):
            return ActionResult(True, "grasp_rl", "already in verified stable grasp; no motion needed", initial)

        try:
            world.move_base_to_grasp_range(.475)
            world.step_physics(25)
            world.open_gripper()
            pose = GROUND_POSE.copy()
            pose[0] = 0.0
            world._set_arm_target(pose)
            world.step_physics(180)

            max_z = float(world._body_pos("red_block")[2])
            for step in range(1, self.max_steps + 1):
                action = self._predict(world.observation())
                world.apply_delta_action(action, frame_skip=self.frame_skip)
                state = world.state()
                max_z = max(max_z, float(state["red_xyz"][2]))
                if grasp_completion(state):
                    world.step_physics(80)
                    state = world.state()
                    if grasp_completion(state):
                        return ActionResult(
                            True,
                            "grasp_rl",
                            f"PPO physical grasp verified after {step} policy steps; max_z={max_z:.3f}m",
                            state,
                        )

                block = world._body_pos("red_block")
                robot = world._body_pos("robot")
                if np.linalg.norm(block[:2] - robot[:2]) > .95 or block[2] < .02:
                    break

            return ActionResult(
                False,
                "grasp_rl",
                f"PPO grasp did not reach stable completion within {self.max_steps} steps; max_z={max_z:.3f}m",
                world.state(),
            )
        except Exception as exc:
            return ActionResult(False, "grasp_rl", f"PPO policy unavailable: {exc}", world.state())
