#!/usr/bin/env python3
"""Train PPO on the transfer-constrained MasterPi v2 environment.

By default this refuses to run while physical dynamics calibration is not
validated. --allow-uncalibrated is explicitly simulator-only research and must
not be described as a deployable sim-to-real policy.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor

from sim.masterpi_training_env_v2 import MasterPiTrainingEnvV2

CALIBRATION = ROOT / "sim/masterpi_dynamics_calibration.json"


def calibration_status() -> tuple[bool, dict, dict]:
    manifest = json.loads(CALIBRATION.read_text())
    validated = manifest.get("validated") is True
    params = manifest.get("parameters") if isinstance(manifest.get("parameters"), dict) else {}
    dynamics_keys = {
        "motor_time_constant_s", "max_forward_force_n", "max_lateral_force_n",
        "max_yaw_torque_nm", "linear_damping_n_per_mps", "yaw_damping_nm_per_radps",
    }
    usable = {k: float(v) for k, v in params.items() if k in dynamics_keys and isinstance(v, (int, float))}
    return validated, manifest, usable


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=100_000)
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--out", default="outputs/rl/masterpi_v2_transfer_ppo")
    ap.add_argument("--curriculum", choices=("grasp_near", "pick_full"), default="grasp_near")
    ap.add_argument("--allow-uncalibrated", action="store_true")
    ap.add_argument("--domain-randomization", type=float, default=0.0)
    ap.add_argument("--smoke", action="store_true", help="run environment/check_env only; do not train")
    args = ap.parse_args()

    validated, manifest, fitted = calibration_status()
    if not validated and not args.allow_uncalibrated:
        print("TRAINING BLOCKED: MasterPiDynamicsV2 has not passed real-robot dynamics calibration.")
        print("Run physical calibration/held-out validation first, or pass --allow-uncalibrated for SIM-ONLY algorithm research.")
        print(f"fit_trials={manifest.get('results', {}).get('fit_trials', 0)} held_out_trials={manifest.get('results', {}).get('held_out_trials', 0)}")
        return 2

    # Use fitted values if they exist. Before calibration, provisional v2 defaults
    # are allowed only behind the explicit simulator-research flag above.
    dynamics = fitted or None
    env_kwargs = dict(
        seed=args.seed,
        curriculum=args.curriculum,
        dynamics=dynamics,
        dynamics_randomization=args.domain_randomization,
    )
    probe = MasterPiTrainingEnvV2(**env_kwargs)
    check_env(probe, warn=True)
    obs, info = probe.reset(seed=args.seed)
    print(json.dumps({
        "mode": "CALIBRATED_SIM_TO_REAL" if validated else "UNCALIBRATED_SIM_ONLY",
        "observation_shape": list(obs.shape),
        "action_shape": list(probe.action_space.shape),
        "observation_contract": info["observation_contract"],
        "camera_observation": info["camera_observation"],
    }, ensure_ascii=False))
    probe.close()
    if args.smoke:
        return 0

    env = Monitor(MasterPiTrainingEnvV2(**env_kwargs))
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=2.5e-4,
        n_steps=1024,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=0.003,
        policy_kwargs=dict(net_arch=[192, 192]),
        seed=args.seed,
        verbose=1,
        device="cpu",
    )
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cb = CheckpointCallback(
        save_freq=max(5_000, args.steps // 10),
        save_path=str(out.parent),
        name_prefix=out.name,
    )
    model.learn(total_timesteps=args.steps, callback=cb, progress_bar=False)
    model.save(str(out))
    env.close()
    print(f"saved {out}.zip")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
