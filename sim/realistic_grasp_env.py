from __future__ import annotations
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from sim.continuous_physics import ContinuousPhysicsWorld, GROUND_POSE

class RealisticGraspEnv(gym.Env):
    metadata={'render_modes':['rgb_array'],'render_fps':30}
    def __init__(self,max_steps=320,frame_skip=8,seed=0):
        super().__init__(); self.max_steps=int(max_steps); self.frame_skip=int(frame_skip); self.world=ContinuousPhysicsWorld(seed=seed,render=False)
        obs=self.world.observation(); self.observation_space=spaces.Box(-np.inf,np.inf,shape=obs.shape,dtype=np.float32); self.action_space=spaces.Box(-1,1,shape=(7,),dtype=np.float32)
        self._step=0; self._prev_z=.075; self._prev_dist=1.0; self._prev_action=np.zeros(7,np.float32)
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed); seed=int(self.np_random.integers(0,2**31-1)) if seed is None else int(seed); rng=np.random.default_rng(seed)
        bx=1.05+float(rng.uniform(-.07,.07)); by=float(rng.uniform(-.24,.24)); yaw=float(rng.uniform(-.40,.40))
        self.world.reset(seed=seed,block_xy=(bx,by),block_yaw=yaw,domain_randomize=True)
        # Reset can choose initial conditions; episode dynamics themselves never teleport.
        heading=float(rng.uniform(-.42,.42)); standoff=float(rng.uniform(.455,.50)); robot_target=np.array([bx,by])-np.array([np.cos(heading),np.sin(heading)])*standoff
        self.world._set_joint_state('base_x',float(robot_target[0])); self.world._set_joint_state('base_y',float(robot_target[1])); self.world._set_joint_state('base_yaw',heading)
        pose=GROUND_POSE.copy()+rng.normal(0,[.045,.055,.07,.07,.055,.045]);
        for n,v in zip(('arm_yaw','shoulder','elbow','wrist_pitch','wrist_roll','gripper_pitch'),pose): self.world._set_joint_state(n,float(v))
        self.world._set_arm_target(pose); self.world.open_gripper(); self.world.stop_base(); self.world.enable_base_hold(); import mujoco; mujoco.mj_forward(self.world.model,self.world.data); self.world.step_physics(120)
        self._step=0; self._prev_z=float(self.world._body_pos('red_block')[2]); self._prev_dist=float(np.linalg.norm(self.world._body_pos('red_block')-self.world._site_pos('grip_site'))); self._prev_action=np.zeros(7,np.float32)
        return self.world.observation(noise_std=.0015), {'physics_state':self.world.state(),'curriculum':'realistic_grasp_stage2'}
    def step(self,action):
        self._step+=1; action=np.asarray(action,np.float32); self.world.apply_delta_action(action,frame_skip=self.frame_skip,motor_noise=.012)
        b=self.world._body_pos('red_block'); g=self.world._site_pos('grip_site'); cs=self.world.contact_state(); dist=float(np.linalg.norm(b-g)); z=float(b[2])
        reward=-.045-.32*dist
        reward += .08*float(cs.left)+.08*float(cs.right)+.12*float(cs.bilateral)
        reward += 7.0*max(0,z-.075)
        reward += 110*np.clip(z-self._prev_z,-.012,.018)+3.0*np.clip(self._prev_dist-dist,-.025,.025)
        if z>.12: reward+=.20
        if z>.16 and cs.bilateral: reward+=.45
        # Realistic penalties: impacts, violent/swingy control, excessive squeeze asymmetry.
        impact=self.world.nonfinger_block_contact_force(); reward-=.018*min(impact,20)
        reward-=.012*float(np.square(action[:6]-self._prev_action[:6]).mean())
        reward-=.003*float(np.square(action[:6]).mean())
        if cs.left_force>8.5 or cs.right_force>8.5: reward-=.25
        success=bool(self.world.state()['stable'])
        if success: reward+=24.0
        self._prev_z=z; self._prev_dist=dist; self._prev_action=action.copy()
        block=self.world._body_pos('red_block'); robot=self.world._body_pos('robot'); lost=bool(np.linalg.norm(block[:2]-robot[:2])>.95 or block[2]<.025)
        term=success or lost; trunc=self._step>=self.max_steps
        info={'is_success':success,'lost':lost,'step':self._step,'distance':dist,'impact_N':impact,'physics_state':self.world.state()}
        return self.world.observation(noise_std=.0015),float(reward),term,trunc,info
    def close(self): self.world.close(); super().close()
