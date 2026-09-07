#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

LEGACY_WARNING = (
    "BLOCKED: sim/grasp_physics.py is a legacy non-transfer physics world "
    "(mocap base, 15 cm blocks, 6-axis arm, simulator-truth observations). "
    "It must not be used for sim-to-real training. Pass --allow-legacy-physics "
    "only to reproduce historical PPO experiments."
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=100_000)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--out', default='outputs/rl/grasp_ppo')
    ap.add_argument('--check-only', action='store_true')
    ap.add_argument(
        '--allow-legacy-physics',
        action='store_true',
        help='explicitly opt into the historical non-transfer PPO environment',
    )
    a = ap.parse_args()

    # Keep this gate before stable-baselines3 / legacy environment imports so a
    # machine cannot accidentally begin or even initialize the obsolete world.
    if not a.allow_legacy_physics:
        raise SystemExit(LEGACY_WARNING)

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3.common.monitor import Monitor
    from sim.grasp_env import MasterPiGraspEnv

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = MasterPiGraspEnv(seed=a.seed)
    check_env(raw, warn=True)
    if a.check_only:
        obs, info = raw.reset(seed=a.seed)
        print('LEGACY REPRODUCTION ONLY')
        print('env_ok observation_shape=', obs.shape, 'action_shape=', raw.action_space.shape)
        print('initial=', info['physics_state'])
        raw.close()
        return
    raw.close()

    env = Monitor(MasterPiGraspEnv(seed=a.seed))
    checkpoint = CheckpointCallback(save_freq=max(5_000, a.steps // 10), save_path=str(out.parent), name_prefix=out.name)
    model = PPO(
        'MlpPolicy', env,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=256,
        n_epochs=10,
        gamma=.985,
        gae_lambda=.95,
        ent_coef=.002,
        policy_kwargs=dict(net_arch=[128, 128]),
        seed=a.seed,
        verbose=1,
        device='cpu',
    )
    model.learn(total_timesteps=a.steps, callback=checkpoint, progress_bar=False)
    model.save(str(out))
    print('saved LEGACY reproduction model', str(out) + '.zip')
    env.close()


if __name__ == '__main__':
    main()
