#!/usr/bin/env python3
from __future__ import annotations

import argparse

LEGACY_WARNING = (
    "BLOCKED: this evaluator uses the historical sim/grasp_physics.py world, "
    "not the production MasterPi physics. Pass --allow-legacy-physics only to "
    "reproduce historical PPO results; do not interpret them as sim-to-real performance."
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='outputs/rl/grasp_ppo.zip')
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--seed', type=int, default=1000)
    ap.add_argument('--allow-legacy-physics', action='store_true')
    a = ap.parse_args()
    if not a.allow_legacy_physics:
        raise SystemExit(LEGACY_WARNING)

    import numpy as np
    from stable_baselines3 import PPO
    from sim.grasp_env import MasterPiGraspEnv

    model = PPO.load(a.model, device='cpu')
    env = MasterPiGraspEnv(seed=a.seed)
    successes = 0
    returns = []
    lengths = []
    for ep in range(a.episodes):
        obs, info = env.reset(seed=a.seed + ep)
        total = 0.0
        for t in range(env.max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total += r
            if term or trunc:
                break
        ok = bool(info.get('is_success'))
        successes += int(ok)
        returns.append(total)
        lengths.append(t + 1)
        print(f'LEGACY ep={ep:02d} success={ok} return={total:.2f} steps={t+1} z={info["physics_state"]["red_xyz"][2]}')
    print({
        'legacy_reproduction_only': True,
        'episodes': a.episodes,
        'successes': successes,
        'success_rate': successes / a.episodes,
        'mean_return': float(np.mean(returns)),
        'mean_length': float(np.mean(lengths)),
    })
    env.close()


if __name__ == '__main__':
    main()
