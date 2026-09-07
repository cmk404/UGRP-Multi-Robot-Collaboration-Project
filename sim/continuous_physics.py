from __future__ import annotations

import io, math, os
from dataclasses import dataclass
from typing import Iterable

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from PIL import Image

# Experiment-oriented simulation: no mocap base and no object attach/weld/teleport after reset.
# Base motion is a bounded-force planar omnidirectional dynamics proxy. Arm/gripper/object
# are ordinary MuJoCo rigid-body/contact dynamics.
XML = r'''
<mujoco model="ugrp_masterpi_continuous_experiment">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" iterations="100" tolerance="1e-9"/>
  <visual><global offwidth="960" offheight="540"/></visual>
  <default>
    <joint armature="0.003" damping="1.4"/>
    <geom solref="0.010 1" solimp="0.90 0.98 0.004"/>
  </default>
  <asset>
    <texture name="ground" type="2d" builtin="checker" width="256" height="256" rgb1=".20 .22 .24" rgb2=".29 .31 .33"/>
    <material name="groundmat" texture="ground" texrepeat="12 12" reflectance=".03"/>
    <material name="metal" rgba=".34 .37 .40 1"/><material name="dark" rgba=".055 .065 .075 1"/>
    <material name="orange" rgba=".95 .34 .06 1"/><material name="rubber" rgba=".08 .08 .08 1"/>
    <material name="redmat" rgba=".88 .04 .04 1"/><material name="blumat" rgba=".04 .18 .88 1"/>
    <material name="yellowmat" rgba=".95 .72 .05 1"/>
  </asset>
  <worldbody>
    <light pos="0 -1.8 4" dir="0 .3 -1" diffuse="1 1 1"/>
    <light pos="-1.5 2 2.8" dir=".35 -.35 -1" diffuse=".42 .42 .42"/>
    <geom name="floor" type="plane" size="5 5 .1" material="groundmat" friction=".92 .018 .0015" condim="4"/>
    <geom name="wall_back" type="box" pos="2.8 0 1" size=".05 3 1" rgba=".56 .58 .60 1"/>

    <body name="robot" pos="0 0 .11">
      <joint name="base_x" type="slide" axis="1 0 0" range="-1.2 2.2" damping="32" armature=".25"/>
      <joint name="base_y" type="slide" axis="0 1 0" range="-1.2 1.2" damping="32" armature=".25"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1" range="-3.14 3.14" damping="24" armature=".18"/>
      <geom name="base_lower" type="box" pos="0 0 -.005" size=".30 .22 .085" material="dark" mass="5.0" contype="0" conaffinity="0"/>
      <geom name="base_top" type="box" pos="-.02 0 .105" size=".24 .18 .035" material="metal" mass="1.0" contype="0" conaffinity="0"/>
      <geom name="front_plate" type="box" pos=".305 0 .015" size=".018 .17 .075" material="metal" mass=".20" contype="0" conaffinity="0"/>

      <body name="wheel_fl" pos=".20 .255 -.045"><joint name="wheel_fl_j" axis="0 1 0" damping=".03"/><geom type="cylinder" size=".075 .035" euler="1.5708 0 0" material="orange" mass=".12" contype="0" conaffinity="0"/></body>
      <body name="wheel_fr" pos=".20 -.255 -.045"><joint name="wheel_fr_j" axis="0 1 0" damping=".03"/><geom type="cylinder" size=".075 .035" euler="1.5708 0 0" material="orange" mass=".12" contype="0" conaffinity="0"/></body>
      <body name="wheel_rl" pos="-.20 .255 -.045"><joint name="wheel_rl_j" axis="0 1 0" damping=".03"/><geom type="cylinder" size=".075 .035" euler="1.5708 0 0" material="orange" mass=".12" contype="0" conaffinity="0"/></body>
      <body name="wheel_rr" pos="-.20 -.255 -.045"><joint name="wheel_rr_j" axis="0 1 0" damping=".03"/><geom type="cylinder" size=".075 .035" euler="1.5708 0 0" material="orange" mass=".12" contype="0" conaffinity="0"/></body>

      <!-- Camera is on a fixed mast on the base, not on the moving gripper. -->
      <body name="camera_mast" pos=".15 0 .22">
        <geom type="box" pos="0 0 .04" size=".035 .035 .08" material="dark" mass=".08" contype="0" conaffinity="0"/>
        <camera name="robot_cam" pos=".05 0 .135" xyaxes="0 -1 0 .342 0 .940" fovy="61"/>
      </body>

      <body name="arm_base" pos=".03 0 .145">
        <joint name="arm_yaw" type="hinge" axis="0 0 1" range="-1.55 1.55" damping="2.4"/>
        <geom type="cylinder" size=".075 .035" pos="0 0 .025" material="dark" mass=".25" contype="0" conaffinity="0"/>
        <geom type="box" size=".065 .075 .07" pos="0 0 .10" material="orange" mass=".25" contype="0" conaffinity="0"/>
        <body name="shoulder_link" pos="0 0 .15">
          <joint name="shoulder" type="hinge" axis="0 1 0" range="-1.45 1.35" damping="2.6"/>
          <geom type="box" size=".055 .065 .055" material="dark" mass=".12" contype="1" conaffinity="1" friction=".5 .01 .001"/>
          <geom name="upper_arm_geom" type="capsule" fromto="0 0 0 .205 0 .105" size=".032" material="orange" mass=".35" contype="1" conaffinity="1" friction=".45 .01 .001"/>
          <body name="elbow_link" pos=".205 0 .105">
            <joint name="elbow" type="hinge" axis="0 1 0" range="-1.7 1.5" damping="2.3"/>
            <geom type="box" size=".052 .062 .052" material="dark" mass=".10" contype="1" conaffinity="1" friction=".5 .01 .001"/>
            <geom name="forearm_geom" type="capsule" fromto="0 0 0 .19 0 -.015" size=".029" material="orange" mass=".28" contype="1" conaffinity="1" friction=".45 .01 .001"/>
            <body name="wrist_pitch_link" pos=".19 0 -.015">
              <joint name="wrist_pitch" type="hinge" axis="0 1 0" range="-1.8 1.8" damping="1.5"/>
              <geom type="box" size=".045 .055 .045" material="dark" mass=".08" contype="1" conaffinity="1" friction=".5 .01 .001"/>
              <geom name="wrist_link_geom" type="capsule" fromto="0 0 0 .11 0 0" size=".024" material="orange" mass=".13" contype="1" conaffinity="1" friction=".45 .01 .001"/>
              <body name="wrist_roll_link" pos=".11 0 0">
                <joint name="wrist_roll" type="hinge" axis="1 0 0" range="-1.57 1.57" damping="1.0"/>
                <geom type="cylinder" size=".038 .035" euler="0 1.5708 0" material="dark" mass=".08" contype="1" conaffinity="1" friction=".5 .01 .001"/>
                <body name="gripper" pos=".055 0 0">
                  <joint name="gripper_pitch" type="hinge" axis="0 1 0" range="-.9 .9" damping="1.0"/>
                  <geom name="gripper_palm" type="box" size=".04 .075 .035" material="metal" mass=".10" contype="1" conaffinity="1" friction=".55 .01 .001"/>
                  <body name="left_finger" pos=".075 .125 0">
                    <joint name="left_finger_slide" type="slide" axis="0 1 0" range="-.085 0" damping="3.0"/>
                    <geom type="box" pos=".060 0 0" size=".075 .016 .018" material="orange" mass=".035" contype="0" conaffinity="0"/>
                    <geom name="left_finger_geom" type="box" pos=".132 0 -.006" size=".022 .020 .052" material="rubber" mass=".020" contype="1" conaffinity="1" condim="6" friction="1.15 .035 .003" solref=".018 1" solimp=".72 .94 .012"/>
                  </body>
                  <body name="right_finger" pos=".075 -.125 0">
                    <joint name="right_finger_slide" type="slide" axis="0 1 0" range="0 .085" damping="3.0"/>
                    <geom type="box" pos=".060 0 0" size=".075 .016 .018" material="orange" mass=".035" contype="0" conaffinity="0"/>
                    <geom name="right_finger_geom" type="box" pos=".132 0 -.006" size=".022 .020 .052" material="rubber" mass=".020" contype="1" conaffinity="1" condim="6" friction="1.15 .035 .003" solref=".018 1" solimp=".72 .94 .012"/>
                  </body>
                  <site name="grip_site" pos=".135 0 0" size=".012" rgba="1 1 .2 .45"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <body name="red_block" pos="1.05 .12 .075">
      <freejoint name="red_free"/>
      <geom name="red_block_geom" type="box" size=".075 .075 .075" material="redmat" mass=".18" contype="1" conaffinity="1" condim="6" friction=".72 .018 .0015" solref=".012 1" solimp=".90 .98 .006"/>
    </body>
    <body name="blue_block" pos="1.40 -.42 .075"><freejoint/><geom type="box" size=".075 .075 .075" material="blumat" mass=".18" contype="1" conaffinity="1" condim="4" friction=".8 .01 .001"/></body>
    <body name="yellow_block" pos="1.55 .46 .075"><freejoint/><geom type="box" size=".075 .075 .075" material="yellowmat" mass=".18" contype="1" conaffinity="1" condim="4" friction=".8 .01 .001"/></body>
    <!-- Fixed lab camera: shows the entire approach path and grasp workspace. -->
    <camera name="overview" pos=".60 -2.40 1.55" xyaxes=".999 -.0416 0 .0204 .489 .871" fovy="49"/>
  </worldbody>
  <actuator>
    <!-- Bounded-force planar base proxy. Commands are target world-frame velocities. -->
    <velocity name="base_vx" joint="base_x" kv="80" ctrllimited="true" ctrlrange="-.30 .30" forcelimited="true" forcerange="-45 45"/>
    <velocity name="base_vy" joint="base_y" kv="80" ctrllimited="true" ctrlrange="-.30 .30" forcelimited="true" forcerange="-45 45"/>
    <velocity name="base_wz" joint="base_yaw" kv="55" ctrllimited="true" ctrlrange="-.85 .85" forcelimited="true" forcerange="-18 18"/>
    <velocity name="wheel_fl" joint="wheel_fl_j" kv=".15" ctrllimited="true" ctrlrange="-25 25" forcelimited="true" forcerange="-1.0 1.0"/>
    <velocity name="wheel_fr" joint="wheel_fr_j" kv=".15" ctrllimited="true" ctrlrange="-25 25" forcelimited="true" forcerange="-1.0 1.0"/>
    <velocity name="wheel_rl" joint="wheel_rl_j" kv=".15" ctrllimited="true" ctrlrange="-25 25" forcelimited="true" forcerange="-1.0 1.0"/>
    <velocity name="wheel_rr" joint="wheel_rr_j" kv=".15" ctrllimited="true" ctrlrange="-25 25" forcelimited="true" forcerange="-1.0 1.0"/>
    <position name="a_yaw" joint="arm_yaw" kp="36" ctrllimited="true" ctrlrange="-1.55 1.55" forcelimited="true" forcerange="-7 7"/>
    <position name="a_shoulder" joint="shoulder" kp="52" ctrllimited="true" ctrlrange="-1.45 1.35" forcelimited="true" forcerange="-13 13"/>
    <position name="a_elbow" joint="elbow" kp="48" ctrllimited="true" ctrlrange="-1.7 1.5" forcelimited="true" forcerange="-11 11"/>
    <position name="a_wrist_pitch" joint="wrist_pitch" kp="34" ctrllimited="true" ctrlrange="-1.8 1.8" forcelimited="true" forcerange="-5.5 5.5"/>
    <position name="a_wrist_roll" joint="wrist_roll" kp="24" ctrllimited="true" ctrlrange="-1.57 1.57" forcelimited="true" forcerange="-3.2 3.2"/>
    <position name="a_gripper_pitch" joint="gripper_pitch" kp="28" ctrllimited="true" ctrlrange="-.9 .9" forcelimited="true" forcerange="-3.8 3.8"/>
    <!-- 7 N/jaw cap + compliant contact makes squeeze/alignment non-trivial. -->
    <position name="a_left_finger" joint="left_finger_slide" kp="150" ctrllimited="true" ctrlrange="-.085 0" forcelimited="true" forcerange="-7 7"/>
    <position name="a_right_finger" joint="right_finger_slide" kp="150" ctrllimited="true" ctrlrange="0 .085" forcelimited="true" forcerange="-7 7"/>
  </actuator>
</mujoco>
'''

ARM_JOINTS = ('arm_yaw','shoulder','elbow','wrist_pitch','wrist_roll','gripper_pitch')
ARM_ACTUATORS = ('a_yaw','a_shoulder','a_elbow','a_wrist_pitch','a_wrist_roll','a_gripper_pitch')
STOW_POSE = np.array([0.0, .15, -.55, .35, 0.0, .05], dtype=float)
GROUND_POSE = np.array([0.0, .04, .67, .33, 0.0, .36], dtype=float)

@dataclass
class ContactState:
    left: bool; right: bool; left_force: float; right_force: float
    @property
    def bilateral(self): return self.left and self.right

class ContinuousPhysicsWorld:
    def __init__(self, seed=0, width=960, height=540, render=True):
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(XML)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        self.width, self.height = width, height
        self._stable_steps = 0
        self._impact_peak = 0.0
        self._cmd_v = np.zeros(3, float)
        self._base_hold_target = None
        self.reset(seed=seed)

    def close(self):
        if self.renderer is not None:
            self.renderer.close(); self.renderer=None

    def _id(self, typ, name): return mujoco.mj_name2id(self.model, typ, name)
    def _joint_qpos(self, name):
        j=self._id(mujoco.mjtObj.mjOBJ_JOINT,name); return float(self.data.qpos[self.model.jnt_qposadr[j]])
    def _joint_qvel(self, name):
        j=self._id(mujoco.mjtObj.mjOBJ_JOINT,name); return float(self.data.qvel[self.model.jnt_dofadr[j]])
    def _set_joint_state(self,name,q,v=0.0):
        j=self._id(mujoco.mjtObj.mjOBJ_JOINT,name); self.data.qpos[self.model.jnt_qposadr[j]]=q; self.data.qvel[self.model.jnt_dofadr[j]]=v
    def _set_actuator(self,name,val):
        a=self._id(mujoco.mjtObj.mjOBJ_ACTUATOR,name); lo,hi=self.model.actuator_ctrlrange[a]; self.data.ctrl[a]=float(np.clip(val,lo,hi))
    def _body_pos(self,name): return self.data.xpos[self._id(mujoco.mjtObj.mjOBJ_BODY,name)].copy()
    def _site_pos(self,name): return self.data.site_xpos[self._id(mujoco.mjtObj.mjOBJ_SITE,name)].copy()
    def _set_arm_target(self, pose:Iterable[float]):
        for n,v in zip(ARM_ACTUATORS,pose): self._set_actuator(n,float(v))
    def arm_targets(self): return np.array([self.data.ctrl[self._id(mujoco.mjtObj.mjOBJ_ACTUATOR,n)] for n in ARM_ACTUATORS],float)
    def open_gripper(self): self._set_actuator('a_left_finger',0); self._set_actuator('a_right_finger',0)
    def close_gripper(self,amount):
        c=float(np.clip(amount,0,1)); self._set_actuator('a_left_finger',-.085*c); self._set_actuator('a_right_finger',.085*c)

    def reset(self, seed=None, block_xy=None, block_yaw=None, domain_randomize=False):
        if seed is not None: self.rng=np.random.default_rng(seed)
        mujoco.mj_resetData(self.model,self.data)
        if block_xy is None: block_xy=(1.05+float(self.rng.uniform(-.08,.08)), float(self.rng.uniform(-.22,.22)))
        if block_yaw is None: block_yaw=float(self.rng.uniform(-.45,.45))
        self._set_joint_state('base_x',0.0); self._set_joint_state('base_y',0.0); self._set_joint_state('base_yaw',0.0)
        for n,v in zip(ARM_JOINTS,STOW_POSE): self._set_joint_state(n,float(v))
        self._set_joint_state('left_finger_slide',0); self._set_joint_state('right_finger_slide',0)
        self._set_arm_target(STOW_POSE); self.open_gripper(); self.stop_base()
        jid=self._id(mujoco.mjtObj.mjOBJ_JOINT,'red_free'); qa=self.model.jnt_qposadr[jid]
        bx,by=map(float,block_xy); self.data.qpos[qa:qa+3]=[bx,by,.075]
        self.data.qpos[qa+3:qa+7]=[math.cos(block_yaw/2),0,0,math.sin(block_yaw/2)]
        if domain_randomize: self.randomize_domain()
        self._stable_steps=0; self._impact_peak=0; self._base_hold_target=None
        mujoco.mj_forward(self.model,self.data); self.step_physics(80)
        return self.state()

    def randomize_domain(self):
        block_gid=self._id(mujoco.mjtObj.mjOBJ_GEOM,'red_block_geom')
        l=self._id(mujoco.mjtObj.mjOBJ_GEOM,'left_finger_geom'); r=self._id(mujoco.mjtObj.mjOBJ_GEOM,'right_finger_geom')
        self.model.geom_friction[block_gid,:]=[self.rng.uniform(.50,.95), self.rng.uniform(.010,.030), self.rng.uniform(.001,.0025)]
        grip=self.rng.uniform(.85,1.35)
        self.model.geom_friction[l,:]=[grip,.035,.003]; self.model.geom_friction[r,:]=[grip,.035,.003]
        bid=self._id(mujoco.mjtObj.mjOBJ_BODY,'red_block'); self.model.body_mass[bid]=self.rng.uniform(.14,.24)
        # Randomize jaw force cap to represent servo/battery variation.
        for name in ('a_left_finger','a_right_finger'):
            aid=self._id(mujoco.mjtObj.mjOBJ_ACTUATOR,name); f=self.rng.uniform(5.5,7.5); self.model.actuator_forcerange[aid]=[-f,f]

    def base_pose(self): return np.array([self._joint_qpos('base_x'),self._joint_qpos('base_y'),self._joint_qpos('base_yaw')],float)
    def base_velocity(self): return np.array([self._joint_qvel('base_x'),self._joint_qvel('base_y'),self._joint_qvel('base_yaw')],float)

    def command_base(self, vx,vy,wz, accel_limit=.55, angular_accel=1.6):
        # Rate-limit commands so a new skill cannot instantaneously change velocity.
        dt=self.model.opt.timestep
        target=np.array([np.clip(vx,-.28,.28),np.clip(vy,-.28,.28),np.clip(wz,-.8,.8)],float)
        max_delta=np.array([accel_limit*dt,accel_limit*dt,angular_accel*dt])
        self._cmd_v += np.clip(target-self._cmd_v,-max_delta,max_delta)
        self._set_actuator('base_vx',self._cmd_v[0]); self._set_actuator('base_vy',self._cmd_v[1]); self._set_actuator('base_wz',self._cmd_v[2])
        # Wheel animation follows body-frame command; it never drives qpos directly.
        yaw=self._joint_qpos('base_yaw'); c,s=math.cos(yaw),math.sin(yaw)
        body_vx=c*self._cmd_v[0]+s*self._cmd_v[1]
        body_vy=-s*self._cmd_v[0]+c*self._cmd_v[1]
        L=.22; R=.075
        w=np.array([body_vx-body_vy-L*self._cmd_v[2], body_vx+body_vy+L*self._cmd_v[2], body_vx+body_vy-L*self._cmd_v[2], body_vx-body_vy+L*self._cmd_v[2]])/R
        for name,val in zip(('wheel_fl','wheel_fr','wheel_rl','wheel_rr'),w): self._set_actuator(name,val)

    def stop_base(self):
        self._cmd_v[:] = 0
        for n in ('base_vx','base_vy','base_wz','wheel_fl','wheel_fr','wheel_rl','wheel_rr'): self._set_actuator(n,0)

    def enable_base_hold(self, pose=None):
        self._base_hold_target = self.base_pose().copy() if pose is None else np.asarray(pose,float).copy()

    def disable_base_hold(self):
        self._base_hold_target = None

    def _update_base_hold(self):
        if self._base_hold_target is None: return
        cur=self.base_pose(); err=self._base_hold_target-cur; err[2]=(err[2]+math.pi)%(2*math.pi)-math.pi
        self.command_base(np.clip(3.0*err[0],-.16,.16),np.clip(3.0*err[1],-.16,.16),np.clip(3.5*err[2],-.55,.55),accel_limit=.85,angular_accel=2.4)

    def step_physics(self,n=1):
        for _ in range(max(1,int(n))):
            self._update_base_hold()
            mujoco.mj_step(self.model,self.data)
            cs=self.contact_state(); bz=float(self._body_pos('red_block')[2])
            if cs.bilateral and bz>.16 and max(cs.left_force,cs.right_force)<12.0: self._stable_steps+=1
            else: self._stable_steps=0
            self._impact_peak=max(self._impact_peak,self.nonfinger_block_contact_force())

    def nonfinger_block_contact_force(self):
        block=self._id(mujoco.mjtObj.mjOBJ_GEOM,'red_block_geom'); lg=self._id(mujoco.mjtObj.mjOBJ_GEOM,'left_finger_geom'); rg=self._id(mujoco.mjtObj.mjOBJ_GEOM,'right_finger_geom')
        peak=0.0
        for i in range(self.data.ncon):
            c=self.data.contact[i]; g1,g2=int(c.geom1),int(c.geom2)
            if block not in (g1,g2): continue
            other=g2 if g1==block else g1
            if other in (lg,rg): continue
            # Floor contact is ordinary support, not an impact penalty.
            if other==self._id(mujoco.mjtObj.mjOBJ_GEOM,'floor'): continue
            force=np.zeros(6); mujoco.mj_contactForce(self.model,self.data,i,force); peak=max(peak,abs(float(force[0])))
        return peak

    def contact_state(self):
        block=self._id(mujoco.mjtObj.mjOBJ_GEOM,'red_block_geom'); lg=self._id(mujoco.mjtObj.mjOBJ_GEOM,'left_finger_geom'); rg=self._id(mujoco.mjtObj.mjOBJ_GEOM,'right_finger_geom')
        left=right=False; lf=rf=0.0
        for i in range(self.data.ncon):
            c=self.data.contact[i]; g1,g2=int(c.geom1),int(c.geom2)
            if block not in (g1,g2): continue
            other=g2 if g1==block else g1
            if other not in (lg,rg): continue
            force=np.zeros(6); mujoco.mj_contactForce(self.model,self.data,i,force); n=abs(float(force[0]))
            if other==lg: left=True; lf+=n
            if other==rg: right=True; rf+=n
        return ContactState(left,right,lf,rf)

    def state(self):
        b=self._body_pos('red_block'); g=self._site_pos('grip_site'); cs=self.contact_state(); bp=self.base_pose(); bv=self.base_velocity()
        return {'red_xyz':[round(float(x),4) for x in b], 'grip_xyz':[round(float(x),4) for x in g], 'grip_error_m':round(float(np.linalg.norm(b-g)),4),
                'left_contact':cs.left,'right_contact':cs.right,'left_normal_N':round(cs.left_force,3),'right_normal_N':round(cs.right_force,3),'bilateral_contact':cs.bilateral,
                'lifted':bool(b[2]>.105),'stable':bool(self._stable_steps>=125),'stable_steps':int(self._stable_steps),'nonfinger_impact_peak_N':round(self._impact_peak,3),
                'base_xyyaw':[round(float(x),4) for x in bp],'base_vxyw':[round(float(x),4) for x in bv],
                'finger_qpos':[round(self._joint_qpos('left_finger_slide'),4),round(self._joint_qpos('right_finger_slide'),4)],
                'arm_qpos':[round(self._joint_qpos(n),4) for n in ARM_JOINTS]}

    def observation(self, noise_std=0.0):
        block=self._body_pos('red_block'); grip=self._site_pos('grip_site'); jid=self._id(mujoco.mjtObj.mjOBJ_JOINT,'red_free'); va=self.model.jnt_dofadr[jid]; qa=self.model.jnt_qposadr[jid]
        vel=self.data.qvel[va:va+6].copy(); cs=self.contact_state(); arm=np.array([self._joint_qpos(n) for n in ARM_JOINTS],np.float32); fingers=np.array([self._joint_qpos('left_finger_slide'),self._joint_qpos('right_finger_slide')],np.float32)
        # Rotation-invariant policy input: positions/velocities are expressed in the mobile-base frame.
        yaw=self._joint_qpos('base_yaw'); c,s=math.cos(yaw),math.sin(yaw); R=np.array([[c,s],[-s,c]],float)
        rel=block-grip; rel_local=np.array([*(R@rel[:2]),rel[2]],np.float32)
        lin_local=np.array([*(R@vel[:2]),vel[2]],np.float32); ang_local=np.array([*(R@vel[3:5]),vel[5]],np.float32)
        qw,qx,qy,qz=self.data.qpos[qa+3:qa+7]; block_yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)); rel_yaw=(block_yaw-yaw+math.pi)%(2*math.pi)-math.pi
        orient=np.array([math.sin(rel_yaw),math.cos(rel_yaw)],np.float32)
        obs=np.concatenate([arm,fingers,rel_local,lin_local,ang_local,orient,np.array([float(cs.left),float(cs.right),min(cs.left_force,12)/12,min(cs.right_force,12)/12],np.float32)]).astype(np.float32)
        if noise_std: obs += self.rng.normal(0,noise_std,obs.shape).astype(np.float32)
        return obs

    def apply_delta_action(self, action, frame_skip=8, motor_noise=0.0):
        a=np.clip(np.asarray(action,float).reshape(7),-1,1)
        if motor_noise: a[:6]+=self.rng.normal(0,motor_noise,6); a=np.clip(a,-1,1)
        current=self.arm_targets(); scales=np.array([.022,.022,.030,.030,.035,.022]); target=current+a[:6]*scales
        # Bounded target slew rate models finite hobby-servo speed.
        self._set_arm_target(target); closure=(a[6]+1)*.5; self.close_gripper(closure); self.step_physics(frame_skip)

    def move_arm_continuous(self,target,seconds=1.5,callback=None):
        start=self.arm_targets().copy(); n=max(1,int(seconds/self.model.opt.timestep));
        for i in range(n):
            u=(i+1)/n; u=u*u*(3-2*u); self._set_arm_target((1-u)*start+u*np.asarray(target,float)); self.step_physics(1)
            if callback and i%4==0: callback(self)

    def navigate_to_pregrasp(self, stop=.47, max_seconds=8.0, callback=None):
        self.disable_base_hold()
        n=int(max_seconds/self.model.opt.timestep)
        reached=False
        for k in range(n):
            robot=self._body_pos('robot'); block=self._body_pos('red_block'); dxy=block[:2]-robot[:2]; dist=float(np.linalg.norm(dxy)); yaw=self._joint_qpos('base_yaw'); desired=math.atan2(dxy[1],dxy[0]); err=(desired-yaw+math.pi)%(2*math.pi)-math.pi
            # Slow down near target. While misaligned, rotate more than translate.
            speed=np.clip(1.1*(dist-stop),0,.22)*max(0.15,1-min(abs(err)/.9,1.0))
            vx=speed*math.cos(yaw); vy=speed*math.sin(yaw); wz=np.clip(2.0*err,-.7,.7)
            self.command_base(vx,vy,wz)
            self.step_physics(1)
            if callback and k%4==0: callback(self)
            if dist<stop+.012 and abs(err)<.035 and np.linalg.norm(self.base_velocity()[:2])<.035:
                reached=True; break
        # Smooth deceleration, not a hard velocity reset.
        for k in range(int(.55/self.model.opt.timestep)):
            self.command_base(0,0,0); self.step_physics(1)
            if callback and k%4==0: callback(self)
        self.stop_base(); self.enable_base_hold()
        return reached

    def block_yaw(self):
        jid=self._id(mujoco.mjtObj.mjOBJ_JOINT,'red_free'); qa=self.model.jnt_qposadr[jid]
        qw,qx,qy,qz=self.data.qpos[qa+3:qa+7]
        return math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))

    def navigate_to_face_aligned_pregrasp(self, stop=.47, max_seconds=9.0, callback=None):
        """Continuous omni-base approach to a grasp pose normal to a cube face.

        The target face normal is chosen modulo 90 degrees to minimize rotation from
        the current target bearing. Base position/velocity are never assigned here;
        all motion goes through bounded acceleration/force actuators.
        """
        self.disable_base_hold()
        block=self._body_pos('red_block'); robot=self._body_pos('robot')
        bearing=math.atan2(block[1]-robot[1],block[0]-robot[0]); byaw=self.block_yaw()
        candidates=[byaw+k*math.pi/2 for k in range(-2,3)]
        def wrap(x): return (x+math.pi)%(2*math.pi)-math.pi
        desired=min(candidates,key=lambda h: abs(wrap(h-bearing))); desired=wrap(desired)
        target=block[:2]-stop*np.array([math.cos(desired),math.sin(desired)])
        n=int(max_seconds/self.model.opt.timestep); reached=False
        for k in range(n):
            pos=self.base_pose()[:2]; yaw=self._joint_qpos('base_yaw'); epos=target-pos; eyaw=wrap(desired-yaw)
            dist=float(np.linalg.norm(epos))
            # Omnidirectional body: world-frame translation can occur while rotating.
            gain=min(1.0,max(.18,dist/.20)); v=np.clip(1.25*epos,-.22,.22)*gain
            if dist<.06: v=np.clip(1.7*epos,-.10,.10)
            wz=float(np.clip(2.2*eyaw,-.65,.65))
            self.command_base(float(v[0]),float(v[1]),wz)
            self.step_physics(1)
            if callback and k%4==0: callback(self)
            if dist<.008 and abs(eyaw)<.022 and np.linalg.norm(self.base_velocity()[:2])<.03:
                reached=True; break
        for k in range(int(.65/self.model.opt.timestep)):
            self.command_base(0,0,0); self.step_physics(1)
            if callback and k%4==0: callback(self)
        self.stop_base(); self.enable_base_hold()
        return reached

    def render_rgb(self,camera='overview'):
        if self.renderer is None: raise RuntimeError('render=False')
        self.renderer.update_scene(self.data,camera=camera); return self.renderer.render().copy()
    def render_jpeg(self,camera='robot_cam',quality=88):
        img=self.render_rgb(camera); buf=io.BytesIO(); Image.fromarray(img).save(buf,format='JPEG',quality=quality); return buf.getvalue()
