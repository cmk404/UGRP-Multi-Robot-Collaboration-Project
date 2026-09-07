#!/usr/bin/env python3
from pathlib import Path
import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from sim.realistic_grasp_env import RealisticGraspEnv

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--allow-legacy-physics',action='store_true'); ap.add_argument('--steps',type=int,default=80000); ap.add_argument('--seed',type=int,default=17); ap.add_argument('--out',default='outputs/rl/grasp_ppo_realistic'); ap.add_argument('--init',default='outputs/rl/grasp_ppo_v2.zip'); a=ap.parse_args()
    if not a.allow_legacy_physics:
        raise SystemExit('BLOCKED: RealisticGraspEnv uses legacy non-transfer physics/privileged observations. Use scripts/train_masterpi_v2.py, or pass --allow-legacy-physics only for historical reproduction.')
    raw=RealisticGraspEnv(seed=a.seed); check_env(raw,warn=True); raw.close(); env=Monitor(RealisticGraspEnv(seed=a.seed)); out=Path(a.out); out.parent.mkdir(parents=True,exist_ok=True)
    if a.init and Path(a.init).exists():
        model=PPO.load(a.init,env=env,device='cpu'); model.learning_rate=2e-4; print('fine_tune_from',a.init)
    else:
        model=PPO('MlpPolicy',env,learning_rate=2.5e-4,n_steps=1024,batch_size=256,n_epochs=10,gamma=.987,gae_lambda=.95,ent_coef=.002,policy_kwargs=dict(net_arch=[128,128]),seed=a.seed,verbose=1,device='cpu')
    cb=CheckpointCallback(save_freq=max(5000,a.steps//8),save_path=str(out.parent),name_prefix=out.name)
    model.learn(total_timesteps=a.steps,callback=cb,progress_bar=False,reset_num_timesteps=False); model.save(str(out)); env.close(); print('saved',str(out)+'.zip')
if __name__=='__main__': main()
