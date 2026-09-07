#!/usr/bin/env python3
"""Smoke-test physical grasping strictly through MasterPiTrainingEnvV2.step()."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from sim.masterpi_training_env_v2 import MasterPiTrainingEnvV2, SERVO_IDS, SERVO_DELTA_PER_STEP

SEQUENCE = (
    {1: 2000, 3: 762, 4: 1715, 5: 2180, 6: 1500},
    {1: 2000, 3: 951, 4: 1762, 5: 2443, 6: 1500},
    {1: 1500, 3: 951, 4: 1762, 5: 2443, 6: 1500},
    {1: 1500, 3: 722, 4: 1652, 5: 2136, 6: 1500},
)

def run(seed: int) -> dict:
    env = MasterPiTrainingEnvV2(seed=seed, width=96, height=72, max_steps=120, curriculum="grasp_ready")
    try:
        obs, reset_info = env.reset(seed=seed)
        info = reset_info
        success = False
        for target in SEQUENCE:
            for _ in range(80):
                action = np.zeros(9, dtype=np.float32)
                done = True
                for j, servo_id in enumerate(SERVO_IDS):
                    current = 1500.0 + float(obs[9 + j]) * 1000.0
                    delta = float(target[servo_id]) - current
                    done = done and abs(delta) <= 1.0
                    action[4 + j] = np.clip(delta / float(SERVO_DELTA_PER_STEP[j]), -1.0, 1.0)
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
                obs, _reward, terminated, truncated, info = env.step(np.zeros(9, dtype=np.float32))
                success = bool(info["is_success"])
                if terminated or truncated:
                    break
        truth = info.get("privileged_eval", {})
        return {
            "ok": bool(success),
            "seed": seed,
            "camera_visible_at_reset": bool(reset_info["camera_observation"]["visible"]),
            "steps": int(info.get("step", 0)),
            "bilateral_contact": bool(truth.get("bilateral_contact")),
            "left_force_n": truth.get("left_force_n"),
            "right_force_n": truth.get("right_force_n"),
            "block_height_m": truth.get("block_height_m"),
            "lost": bool(info.get("lost", False)),
            "tipped": bool(info.get("tipped", False)),
        }
    finally:
        env.close()

def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument('--seed',type=int,default=0); args=ap.parse_args()
    result=run(args.seed); print(json.dumps(result,ensure_ascii=False)); return 0 if result["ok"] else 1
if __name__ == '__main__': raise SystemExit(main())
