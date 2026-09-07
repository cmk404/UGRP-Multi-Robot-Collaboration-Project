#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from stable_baselines3 import PPO
from sim.continuous_physics import ContinuousPhysicsWorld, GROUND_POSE

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--seed',type=int,default=13025); ap.add_argument('--model',default='outputs/rl/grasp_ppo_realistic_v4_37500_steps.zip'); ap.add_argument('--out',default='outputs/realistic_trial.json'); ap.add_argument('--raw-approach',action='store_true'); a=ap.parse_args()
    m=PPO.load(a.model,device='cpu'); w=ContinuousPhysicsWorld(seed=a.seed,render=False); w.reset(seed=a.seed,domain_randomize=True)
    initial=w.state(); nav = w.navigate_to_pregrasp(stop=.47,max_seconds=8) if a.raw_approach else w.navigate_to_face_aligned_pregrasp(stop=.47,max_seconds=9)
    post_nav=w.state(); w.move_arm_continuous(GROUND_POSE,1.6); pre_policy=w.state(); actions=[]; states=[]
    for t in range(220):
        obs=w.observation(noise_std=0); act,_=m.predict(obs,deterministic=True); act=[float(x) for x in act]
        actions.append(act); w.apply_delta_action(act,frame_skip=8,motor_noise=0); st=w.state(); states.append(st)
        if st['stable']: break
    final=w.state(); payload={'seed':a.seed,'model':a.model,'face_aligned':not a.raw_approach,'nav_ok':nav,'initial':initial,'post_nav':post_nav,'pre_policy':pre_policy,'actions':actions,'states':states,'final':final,'physics_timestep':float(w.model.opt.timestep),'policy_frame_skip':8}
    Path(a.out).parent.mkdir(parents=True,exist_ok=True); Path(a.out).write_text(json.dumps(payload,indent=2)); print(json.dumps({'out':a.out,'actions':len(actions),'stable':final['stable'],'z':final['red_xyz'][2],'L':final['left_normal_N'],'R':final['right_normal_N'],'impact':final['nonfinger_impact_peak_N']},indent=2)); w.close()
if __name__=='__main__': main()
