#!/usr/bin/env python3
"""Transfer-only expert sanity check for MasterPiTrainingEnvV2.

This controller uses the same deployable observation channels as a learned
policy. It never reads MuJoCo position/contact truth to choose an action; truth
is consumed only after each episode for evaluation/reporting.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.masterpi_training_env_v2 import MasterPiTrainingEnvV2, SERVO_IDS, SERVO_DELTA_PER_STEP

FORWARD = np.array([1, 1, 1, 1], dtype=np.float32)
LEFT = np.array([-1, 1, 1, -1], dtype=np.float32)
RIGHT = -LEFT
POSES = [
    {1: 2000, 3: 650, 4: 2230, 5: 1500, 6: 1500},  # hover
    {1: 2000, 3: 650, 4: 2230, 5: 1920, 6: 1500},  # grasp open
    {1: 1500, 3: 650, 4: 2230, 5: 1920, 6: 1500},  # close
    {1: 1500, 3: 650, 4: 2230, 5: 1500, 6: 1500},  # lift to hover
    {1: 1500, 3: 960, 4: 2410, 5: 1215, 6: 1500},  # carry
]


def pwm_from_obs(obs: np.ndarray) -> dict[int, int]:
    # Observation layout: 0:9 vision/FK, 9:14 commanded PWM.
    return {sid: int(round(float(obs[9 + j]) * 1000.0 + 1500.0)) for j, sid in enumerate(SERVO_IDS)}


def servo_action(obs: np.ndarray, target: dict[int, int]) -> tuple[np.ndarray, bool]:
    current = pwm_from_obs(obs)
    action = np.zeros(9, dtype=np.float32)
    done = True
    for j, sid in enumerate(SERVO_IDS):
        delta = float(target[sid] - current[sid])
        if abs(delta) > 2.0:
            done = False
        action[4 + j] = np.clip(delta / float(SERVO_DELTA_PER_STEP[j]), -1.0, 1.0)
    return action, done


def run_episode(env: MasterPiTrainingEnvV2, seed: int) -> dict:
    obs, _ = env.reset(seed=seed)
    stage = -1  # -1 == visual approach, 0..4 == arm sequence
    last_range: float | None = None
    blind_creep = 0
    steps = 0
    for steps in range(1, env.max_steps + 1):
        action = np.zeros(9, dtype=np.float32)
        motion_only = False
        if stage < 0:
            visible = bool(obs[0] > 0.5)
            metric_valid = bool(obs[6] > 0.5)
            if blind_creep > 0:
                action[:4] = FORWARD
                motion_only = True
                blind_creep -= 1
                if blind_creep == 0:
                    stage = 0
            elif metric_valid:
                rng = float(obs[7]) * 0.80
                lateral = float(obs[8]) * 0.40
                last_range = rng
                if rng <= 0.19:
                    stage = 0
                elif abs(lateral) > 0.022:
                    action[:4] = LEFT if lateral > 0 else RIGHT
                else:
                    action[:4] = FORWARD
            elif not visible and last_range is not None and last_range <= 0.26:
                # At the low search pitch a correctly approached cube leaves the
                # bottom of the image before it reaches the ~10-cm grasp corridor.
                # Two 100-ms physical forward commands bridge that known blind
                # strip without using simulator coordinates.
                blind_creep = 2
                action[:4] = FORWARD
                motion_only = True
                blind_creep -= 1
            else:
                # Current curriculum begins visible. If perception is unexpectedly
                # lost before a usable near estimate, back up one physical step.
                action[:4] = -FORWARD
        if stage >= 0 and not motion_only:
            action, pose_done = servo_action(obs, POSES[stage])
            if pose_done and stage < len(POSES) - 1:
                stage += 1
                action, _ = servo_action(obs, POSES[stage])
        obs, reward, terminated, truncated, info = env.step(action)
        if info.get("is_success") or terminated or truncated:
            break
    return {
        "seed": seed,
        "success": bool(info.get("is_success")),
        "steps": steps,
        "stage": stage,
        "last_visual_range_m": None if last_range is None else round(last_range, 4),
        "eval": info.get("privileged_eval"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--width", type=int, default=160)
    ap.add_argument("--height", type=int, default=120)
    ap.add_argument("--curriculum", choices=("grasp_near", "pick_full"), default="grasp_near")
    args = ap.parse_args()
    env = MasterPiTrainingEnvV2(seed=args.seed, width=args.width, height=args.height, curriculum=args.curriculum)
    rows = []
    try:
        for i in range(args.episodes):
            rows.append(run_episode(env, args.seed + i))
    finally:
        env.close()
    successes = sum(int(row["success"]) for row in rows)
    print(json.dumps({
        "episodes": len(rows),
        "successes": successes,
        "success_rate": successes / max(1, len(rows)),
        "mean_steps": sum(r["steps"] for r in rows) / max(1, len(rows)),
        "failures": [r for r in rows if not r["success"]][:10],
    }, ensure_ascii=False))
    return 0 if successes == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
