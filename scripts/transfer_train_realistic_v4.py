#!/usr/bin/env python3
from pathlib import Path
import argparse, torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from sim.realistic_grasp_env import RealisticGraspEnv

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--steps',type=int,default=45000); ap.add_argument('--seed',type=int,default=41); ap.add_argument('--src',default='outputs/rl/grasp_ppo_realistic.zip'); ap.add_argument('--out',default='outputs/rl/grasp_ppo_realistic_v4'); a=ap.parse_args()
 old=PPO.load(a.src,device='cpu'); env=Monitor(RealisticGraspEnv(seed=a.seed))
 new=PPO('MlpPolicy',env,learning_rate=1.5e-4,n_steps=1024,batch_size=256,n_epochs=8,gamma=.987,gae_lambda=.95,ent_coef=.0015,policy_kwargs=dict(net_arch=[128,128]),seed=a.seed,verbose=1,device='cpu')
 osd=old.policy.state_dict(); nsd=new.policy.state_dict()
 for k in nsd:
  if k in osd and nsd[k].shape==osd[k].shape: nsd[k].copy_(osd[k])
 for k in ('mlp_extractor.policy_net.0.weight','mlp_extractor.value_net.0.weight'):
  nsd[k].zero_(); nsd[k][:,:17].copy_(osd[k][:,:17]); nsd[k][:,19:23].copy_(osd[k][:,17:21])
 new.policy.load_state_dict(nsd); Path(a.out).parent.mkdir(parents=True,exist_ok=True); new.save(a.out+'_warm')
 cb=CheckpointCallback(save_freq=max(5000,a.steps//6),save_path=str(Path(a.out).parent),name_prefix=Path(a.out).name)
 new.learn(total_timesteps=a.steps,callback=cb,progress_bar=False); new.save(a.out); env.close(); print('saved',a.out+'.zip')
if __name__=='__main__': main()
