#!/usr/bin/env python3
import argparse, numpy as np
from stable_baselines3 import PPO
from sim.realistic_grasp_env import RealisticGraspEnv

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--model',default='outputs/rl/grasp_ppo_realistic.zip'); ap.add_argument('--episodes',type=int,default=50); ap.add_argument('--seed',type=int,default=5000); a=ap.parse_args(); model=PPO.load(a.model,device='cpu'); env=RealisticGraspEnv(seed=a.seed)
    succ=0; zs=[]; impacts=[]; lens=[]
    for ep in range(a.episodes):
        obs,info=env.reset(seed=a.seed+ep)
        for t in range(env.max_steps):
            act,_=model.predict(obs,deterministic=True); obs,r,term,trunc,info=env.step(act)
            if term or trunc: break
        ok=bool(info['is_success']); succ+=ok; zs.append(info['physics_state']['red_xyz'][2]); impacts.append(info['physics_state']['nonfinger_impact_peak_N']); lens.append(t+1)
        print(f'ep={ep:02d} ok={ok} steps={t+1} z={zs[-1]:.3f} L={info["physics_state"]["left_normal_N"]:.2f} R={info["physics_state"]["right_normal_N"]:.2f} impact={impacts[-1]:.2f}')
    print({'episodes':a.episodes,'successes':int(succ),'success_rate':float(succ/a.episodes),'mean_steps':float(np.mean(lens)),'mean_final_z':float(np.mean(zs)),'mean_peak_nonfinger_impact_N':float(np.mean(impacts))})
    env.close()
if __name__=='__main__': main()
