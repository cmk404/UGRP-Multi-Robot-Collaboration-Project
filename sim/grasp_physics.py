from __future__ import annotations

import io
import math
import os
from dataclasses import dataclass
from typing import Iterable

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

# Contact-physics version of the MasterPi-like simulator.
# Important design rule: the red block is NEVER teleported/attached to the gripper.
# It can only move because MuJoCo contact forces move it.
XML = r'''
<mujoco model="ugrp_masterpi_contact_grasp">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" iterations="80"/>
  <visual><global offwidth="640" offheight="480"/></visual>
  <default>
    <joint armature="0.002" damping="1.2"/>
    <geom solref="0.006 1" solimp="0.95 0.99 0.002"/>
  </default>
  <asset>
    <texture name="ground" type="2d" builtin="checker" width="256" height="256"
             rgb1=".20 .22 .24" rgb2=".27 .29 .31"/>
    <material name="groundmat" texture="ground" texrepeat="10 10" reflectance=".04"/>
    <material name="metal" rgba=".30 .33 .36 1"/>
    <material name="dark" rgba=".055 .065 .075 1"/>
    <material name="orange" rgba=".95 .34 .06 1"/>
    <material name="rubber" rgba=".10 .10 .10 1"/>
    <material name="redmat" rgba=".88 .04 .04 1"/>
    <material name="blumat" rgba=".04 .18 .88 1"/>
    <material name="yellowmat" rgba=".95 .72 .05 1"/>
  </asset>

  <worldbody>
    <light pos="0 -1.5 4" dir="0 .25 -1" diffuse="1 1 1"/>
    <light pos="-1 2 2.5" dir=".3 -.4 -1" diffuse=".45 .45 .45"/>
    <geom name="floor" type="plane" size="5 5 .1" material="groundmat"
          friction="1.0 .01 .001" condim="4"/>
    <geom name="wall_back" type="box" pos="2.8 0 1" size=".05 3 1" rgba=".54 .56 .58 1"/>

    <!-- Base remains kinematic at this stage; learned grasp control starts at the arm/gripper. -->
    <body name="robot" pos="0 0 .11" mocap="true">
      <geom name="base_lower" type="box" pos="0 0 -.005" size=".30 .22 .085"
            material="dark" mass="5" contype="0" conaffinity="0"/>
      <geom name="base_top" type="box" pos="-.02 0 .105" size=".24 .18 .035"
            material="metal" contype="0" conaffinity="0"/>
      <geom name="front_plate" type="box" pos=".305 0 .015" size=".018 .17 .075"
            material="metal" contype="0" conaffinity="0"/>
      <!-- Four mecanum-looking wheels. -->
      <geom type="cylinder" pos=".20 .255 -.045" size=".075 .035" euler="1.5708 0 0"
            material="orange" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".20 -.255 -.045" size=".075 .035" euler="1.5708 0 0"
            material="orange" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="-.20 .255 -.045" size=".075 .035" euler="1.5708 0 0"
            material="orange" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="-.20 -.255 -.045" size=".075 .035" euler="1.5708 0 0"
            material="orange" contype="0" conaffinity="0"/>

      <body name="arm_base" pos=".03 0 .145">
        <joint name="arm_yaw" type="hinge" axis="0 0 1" range="-1.55 1.55" damping="2.0"/>
        <geom type="cylinder" size=".075 .035" pos="0 0 .025" material="dark"
              mass=".25" contype="0" conaffinity="0"/>
        <geom type="box" size=".065 .075 .07" pos="0 0 .10" material="orange"
              mass=".25" contype="0" conaffinity="0"/>

        <body name="shoulder_link" pos="0 0 .15">
          <joint name="shoulder" type="hinge" axis="0 1 0" range="-1.45 1.35" damping="2.2"/>
          <geom type="box" size=".055 .065 .055" material="dark" mass=".12"
                contype="0" conaffinity="0"/>
          <geom type="capsule" fromto="0 0 0 .205 0 .105" size=".032" material="orange"
                mass=".35" contype="0" conaffinity="0"/>

          <body name="elbow_link" pos=".205 0 .105">
            <joint name="elbow" type="hinge" axis="0 1 0" range="-1.7 1.5" damping="2.0"/>
            <geom type="box" size=".052 .062 .052" material="dark" mass=".10"
                  contype="0" conaffinity="0"/>
            <geom type="capsule" fromto="0 0 0 .19 0 -.015" size=".029" material="orange"
                  mass=".28" contype="0" conaffinity="0"/>

            <body name="wrist_pitch_link" pos=".19 0 -.015">
              <joint name="wrist_pitch" type="hinge" axis="0 1 0" range="-1.8 1.8" damping="1.2"/>
              <geom type="box" size=".045 .055 .045" material="dark" mass=".08"
                    contype="0" conaffinity="0"/>
              <geom type="capsule" fromto="0 0 0 .11 0 0" size=".024" material="orange"
                    mass=".13" contype="0" conaffinity="0"/>

              <body name="wrist_roll_link" pos=".11 0 0">
                <joint name="wrist_roll" type="hinge" axis="1 0 0" range="-1.57 1.57" damping=".8"/>
                <geom type="cylinder" size=".038 .035" euler="0 1.5708 0" material="dark"
                      mass=".08" contype="0" conaffinity="0"/>

                <body name="gripper" pos=".055 0 0">
                  <joint name="gripper_pitch" type="hinge" axis="0 1 0" range="-.9 .9" damping=".8"/>
                  <geom name="gripper_palm" type="box" size=".04 .075 .035" material="metal"
                        mass=".10" contype="0" conaffinity="0"/>

                  <!-- Parallel jaw fingers. Their rigid contacts with the block do the grasping. -->
                  <body name="left_finger" pos=".075 .125 0">
                    <joint name="left_finger_slide" type="slide" axis="0 1 0"
                           range="-.085 0" damping="2"/>
                    <geom type="box" pos=".060 0 0" size=".075 .016 .018"
                          material="orange" mass=".035" contype="0" conaffinity="0"/>
                    <geom name="left_finger_geom" type="box" pos=".132 0 -.006" size=".022 .020 .052"
                          material="rubber" mass=".020" contype="1" conaffinity="1" condim="4"
                          friction="2.6 .04 .002"/>
                  </body>
                  <body name="right_finger" pos=".075 -.125 0">
                    <joint name="right_finger_slide" type="slide" axis="0 1 0"
                           range="0 .085" damping="2"/>
                    <geom type="box" pos=".060 0 0" size=".075 .016 .018"
                          material="orange" mass=".035" contype="0" conaffinity="0"/>
                    <geom name="right_finger_geom" type="box" pos=".132 0 -.006" size=".022 .020 .052"
                          material="rubber" mass=".020" contype="1" conaffinity="1" condim="4"
                          friction="2.6 .04 .002"/>
                  </body>

                  <site name="grip_site" pos=".135 0 0" size=".014" rgba="1 1 .2 .45"/>
                  <geom type="box" pos=".015 0 .070" size=".035 .045 .025" material="dark"
                        contype="0" conaffinity="0"/>
                  <camera name="robot_cam" pos=".06 0 .075" xyaxes="0 -1 0 0 0 1" fovy="62"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <body name="red_block" pos="1.15 .18 .075">
      <freejoint name="red_free"/>
      <geom name="red_block_geom" type="box" size=".075 .075 .075" material="redmat"
            mass=".18" contype="1" conaffinity="1" condim="4" friction="1.5 .02 .001"/>
    </body>
    <body name="blue_block" pos="1.45 -.42 .075">
      <freejoint/>
      <geom type="box" size=".075 .075 .075" material="blumat" mass=".18"
            contype="1" conaffinity="1" condim="4" friction="1.1 .01 .001"/>
    </body>
    <body name="yellow_block" pos="1.72 .46 .075">
      <freejoint/>
      <geom type="box" size=".075 .075 .075" material="yellowmat" mass=".18"
            contype="1" conaffinity="1" condim="4" friction="1.1 .01 .001"/>
    </body>
    <camera name="overview" pos="-1.6 -2.3 1.45" xyaxes=".82 -.57 0 .27 .39 .88" fovy="48"/>
  </worldbody>

  <actuator>
    <position name="a_yaw" joint="arm_yaw" kp="45" ctrllimited="true" ctrlrange="-1.55 1.55" forcelimited="true" forcerange="-10 10"/>
    <position name="a_shoulder" joint="shoulder" kp="70" ctrllimited="true" ctrlrange="-1.45 1.35" forcelimited="true" forcerange="-18 18"/>
    <position name="a_elbow" joint="elbow" kp="65" ctrllimited="true" ctrlrange="-1.7 1.5" forcelimited="true" forcerange="-15 15"/>
    <position name="a_wrist_pitch" joint="wrist_pitch" kp="45" ctrllimited="true" ctrlrange="-1.8 1.8" forcelimited="true" forcerange="-8 8"/>
    <position name="a_wrist_roll" joint="wrist_roll" kp="30" ctrllimited="true" ctrlrange="-1.57 1.57" forcelimited="true" forcerange="-5 5"/>
    <position name="a_gripper_pitch" joint="gripper_pitch" kp="35" ctrllimited="true" ctrlrange="-.9 .9" forcelimited="true" forcerange="-5 5"/>
    <position name="a_left_finger" joint="left_finger_slide" kp="500" ctrllimited="true" ctrlrange="-.085 0" forcelimited="true" forcerange="-20 20"/>
    <position name="a_right_finger" joint="right_finger_slide" kp="500" ctrllimited="true" ctrlrange="0 .085" forcelimited="true" forcerange="-20 20"/>
  </actuator>
</mujoco>
'''

ARM_JOINTS = ("arm_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_pitch")
ARM_ACTUATORS = ("a_yaw", "a_shoulder", "a_elbow", "a_wrist_pitch", "a_wrist_roll", "a_gripper_pitch")
CARRY_POSE = np.array([0.0, 0.15, -0.55, 0.35, 0.0, 0.05], dtype=float)
GROUND_POSE = np.array([0.0, 0.04, 0.67, 0.33, 0.0, 0.36], dtype=float)


@dataclass
class ContactState:
    left: bool
    right: bool
    left_force: float
    right_force: float

    @property
    def bilateral(self) -> bool:
        return self.left and self.right


class PhysicsGraspWorld:
    """MasterPi-like MuJoCo world with a real contact/friction grasp.

    The mobile base is still kinematic for now. The arm, fingers and block use
    MuJoCo dynamics. There is intentionally no attach/weld/teleport helper.
    """

    def __init__(self, seed: int = 0, width: int = 640, height: int = 480, render: bool = True):
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(XML)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        self.width, self.height = width, height
        self.base_yaw = 0.0
        self._stable_steps = 0
        self.reset(seed)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def _id(self, typ, name: str) -> int:
        return mujoco.mj_name2id(self.model, typ, name)

    def _joint_qpos(self, name: str) -> float:
        jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(self.data.qpos[self.model.jnt_qposadr[jid]])

    def _set_joint_qpos(self, name: str, value: float):
        jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
        self.data.qpos[self.model.jnt_qposadr[jid]] = float(value)
        self.data.qvel[self.model.jnt_dofadr[jid]] = 0.0

    def _set_actuator(self, name: str, value: float):
        aid = self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        lo, hi = self.model.actuator_ctrlrange[aid]
        self.data.ctrl[aid] = float(np.clip(value, lo, hi))

    def _set_arm_target(self, pose: Iterable[float]):
        for actuator, value in zip(ARM_ACTUATORS, pose):
            self._set_actuator(actuator, float(value))

    def open_gripper(self):
        self._set_actuator("a_left_finger", 0.0)
        self._set_actuator("a_right_finger", 0.0)

    def close_gripper(self, amount: float = 1.0):
        amount = float(np.clip(amount, 0.0, 1.0))
        self._set_actuator("a_left_finger", -.085 * amount)
        self._set_actuator("a_right_finger", .085 * amount)

    def _body_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self._id(mujoco.mjtObj.mjOBJ_BODY, name)].copy()

    def _site_pos(self, name: str) -> np.ndarray:
        return self.data.site_xpos[self._id(mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    def reset(self, seed: int | None = None, block_xy: tuple[float, float] | None = None) -> dict:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        if block_xy is None:
            bx = 1.15 + float(self.rng.uniform(-.06, .06))
            by = float(self.rng.uniform(-.28, .28))
        else:
            bx, by = map(float, block_xy)
        red_jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, "red_free")
        qa = self.model.jnt_qposadr[red_jid]
        self.data.qpos[qa:qa+3] = [bx, by, .075]
        self.data.qpos[qa+3:qa+7] = [1, 0, 0, 0]
        self.data.mocap_pos[0] = [0.0, 0.0, .11]
        self.data.mocap_quat[0] = [1.0, 0.0, 0.0, 0.0]
        self.base_yaw = 0.0
        for name, value in zip(ARM_JOINTS, CARRY_POSE):
            self._set_joint_qpos(name, float(value))
        self._set_joint_qpos("left_finger_slide", 0.0)
        self._set_joint_qpos("right_finger_slide", 0.0)
        self._set_arm_target(CARRY_POSE)
        self.open_gripper()
        self._stable_steps = 0
        mujoco.mj_forward(self.model, self.data)
        # Let free blocks settle on the ground before an episode begins.
        self.step_physics(60)
        return self.state()

    def step_physics(self, n: int = 1):
        for _ in range(max(1, int(n))):
            mujoco.mj_step(self.model, self.data)
            cs = self.contact_state()
            block_z = float(self._body_pos("red_block")[2])
            if cs.bilateral and block_z > .16:
                self._stable_steps += 1
            else:
                self._stable_steps = 0

    def move_base_to_grasp_range(self, stop: float = .47):
        robot = self._body_pos("robot")
        block = self._body_pos("red_block")
        dx, dy = block[0]-robot[0], block[1]-robot[1]
        d = max(float(math.hypot(dx, dy)), 1e-8)
        target = block[:2] - np.array([dx, dy]) / d * stop
        self.base_yaw = math.atan2(dy, dx)
        self.data.mocap_pos[0] = [float(target[0]), float(target[1]), .11]
        self.data.mocap_quat[0] = [math.cos(self.base_yaw/2), 0, 0, math.sin(self.base_yaw/2)]
        self._set_actuator("a_yaw", 0.0)
        mujoco.mj_forward(self.model, self.data)

    def set_arm_pose(self, pose: Iterable[float], steps: int = 250):
        self._set_arm_target(pose)
        self.step_physics(steps)

    def contact_state(self) -> ContactState:
        block_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, "red_block_geom")
        left_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, "left_finger_geom")
        right_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, "right_finger_geom")
        # Fingertip child geoms are unnamed, so body membership is also checked.
        left_bid = self._id(mujoco.mjtObj.mjOBJ_BODY, "left_finger")
        right_bid = self._id(mujoco.mjtObj.mjOBJ_BODY, "right_finger")
        left = right = False
        lf = rf = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if block_gid not in (g1, g2):
                continue
            other = g2 if g1 == block_gid else g1
            body = int(self.model.geom_bodyid[other])
            force = np.zeros(6, dtype=float)
            if c.efc_address >= 0:
                mujoco.mj_contactForce(self.model, self.data, i, force)
            normal = abs(float(force[0]))
            if other == left_gid or body == left_bid:
                left = True; lf += normal
            if other == right_gid or body == right_bid:
                right = True; rf += normal
        return ContactState(left, right, lf, rf)

    def state(self) -> dict:
        block = self._body_pos("red_block")
        grip = self._site_pos("grip_site")
        cs = self.contact_state()
        return {
            "red_xyz": [round(float(x), 4) for x in block],
            "grip_xyz": [round(float(x), 4) for x in grip],
            "grip_error_m": round(float(np.linalg.norm(block - grip)), 4),
            "left_contact": cs.left,
            "right_contact": cs.right,
            "left_normal_N": round(cs.left_force, 3),
            "right_normal_N": round(cs.right_force, 3),
            "bilateral_contact": cs.bilateral,
            "lifted": bool(block[2] > .105),
            "stable": bool(self._stable_steps >= 50),
            "stable_steps": int(self._stable_steps),
            "finger_qpos": [round(self._joint_qpos("left_finger_slide"), 4), round(self._joint_qpos("right_finger_slide"), 4)],
            "arm_qpos": [round(self._joint_qpos(n), 4) for n in ARM_JOINTS],
        }

    def observation(self) -> np.ndarray:
        block = self._body_pos("red_block")
        grip = self._site_pos("grip_site")
        red_jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, "red_free")
        va = self.model.jnt_dofadr[red_jid]
        block_vel = self.data.qvel[va:va+6].copy()
        cs = self.contact_state()
        arm_q = np.array([self._joint_qpos(n) for n in ARM_JOINTS], dtype=np.float32)
        fingers = np.array([
            self._joint_qpos("left_finger_slide"), self._joint_qpos("right_finger_slide")
        ], dtype=np.float32)
        return np.concatenate([
            arm_q,
            fingers,
            (block-grip).astype(np.float32),
            block_vel.astype(np.float32),
            np.array([float(cs.left), float(cs.right), min(cs.left_force, 30)/30, min(cs.right_force, 30)/30], dtype=np.float32),
        ]).astype(np.float32)

    def apply_delta_action(self, action: np.ndarray, frame_skip: int = 8):
        """Low-level RL action: 6 arm target deltas + gripper close/open scalar.

        action is normalized to [-1, 1]. The last element controls jaw closure:
        -1=open, +1=fully closed. Arm components are incremental target changes.
        """
        a = np.asarray(action, dtype=float).reshape(7)
        a = np.clip(a, -1.0, 1.0)
        current_targets = np.array([self.data.ctrl[self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, n)] for n in ARM_ACTUATORS])
        scales = np.array([.035, .035, .045, .045, .05, .035])
        new_targets = current_targets + a[:6] * scales
        self._set_arm_target(new_targets)
        closure = (a[6] + 1.0) * .5
        self.close_gripper(closure)
        self.step_physics(frame_skip)

    def grasp_reward(self, prev_block_z: float | None = None, prev_distance: float | None = None) -> tuple[float, dict]:
        block = self._body_pos("red_block")
        grip = self._site_pos("grip_site")
        cs = self.contact_state()
        distance = float(np.linalg.norm(block-grip))
        # Potential-style shaping: ground contact alone should not be profitable to farm.
        reward = -0.06 - 0.40 * distance
        reward += 0.04 * float(cs.left) + 0.04 * float(cs.right) + 0.04 * float(cs.bilateral)
        lift = max(0.0, float(block[2] - .075))
        reward += 10.0 * lift
        if prev_block_z is not None:
            reward += 120.0 * float(np.clip(block[2] - prev_block_z, -.01, .02))
        if prev_distance is not None:
            reward += 4.0 * float(np.clip(prev_distance - distance, -.03, .03))
        if block[2] > .12:
            reward += .25
        if block[2] > .16:
            reward += .55
        success = bool(self._stable_steps >= 50)
        if success:
            reward += 20.0
        info = {
            "distance": distance,
            "bilateral": cs.bilateral,
            "block_z": float(block[2]),
            "success": success,
        }
        return float(reward), info

    def act(self, action: str):
        # Import lazily to avoid coupling this physics module to the legacy world.
        from sim.mujoco_world import ActionResult
        action = str(action).strip().lower()
        if action == "reset":
            return ActionResult(True, action, "contact-physics world reset", self.reset())
        current = self.state()
        if current["stable"] and current["bilateral_contact"] and current["red_xyz"][2] > .16:
            if action == "carry":
                return ActionResult(True, action, "already stably holding the lifted block; maintained current pose", current)
            if action in {"approach", "track", "pick", "fetch"}:
                return ActionResult(False, action, "refused disruptive action because the block is already stably grasped and lifted", current)
        if action == "approach":
            self.move_base_to_grasp_range(.47)
            self.step_physics(35)
            return ActionResult(True, action, "moved base to physical grasp range", self.state())
        if action == "track":
            robot = self._body_pos("robot")
            block = self._body_pos("red_block")
            desired = math.atan2(block[1]-robot[1], block[0]-robot[0])
            rel = (desired-self.base_yaw+math.pi)%(2*math.pi)-math.pi
            pose = np.array([self.data.ctrl[self._id(mujoco.mjtObj.mjOBJ_ACTUATOR,n)] for n in ARM_ACTUATORS], dtype=float)
            pose[0] = float(np.clip(rel, -1.45, 1.45))
            self._set_arm_target(pose); self.step_physics(80)
            return ActionResult(True, action, "arm/camera yaw aligned to target", self.state())
        if action == "pick":
            self.open_gripper(); self.step_physics(30)
            # Keep current yaw while moving remaining joints into the floor pre-grasp.
            pose = GROUND_POSE.copy()
            pose[0] = self.data.ctrl[self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, "a_yaw")]
            self.set_arm_pose(pose, 300)
            for c in np.linspace(0.0, 1.0, 18):
                self.close_gripper(float(c)); self.step_physics(16)
            cs = self.contact_state()
            ok = cs.bilateral
            reason = (f"bilateral physical grasp established L={cs.left_force:.1f}N R={cs.right_force:.1f}N"
                      if ok else f"physical grasp failed left={cs.left} right={cs.right}")
            return ActionResult(ok, action, reason, self.state())
        if action == "carry":
            if not self.contact_state().bilateral:
                return ActionResult(False, action, "no bilateral finger contact", self.state())
            start = np.array([self.data.ctrl[self._id(mujoco.mjtObj.mjOBJ_ACTUATOR,n)] for n in ARM_ACTUATORS], dtype=float)
            target = CARRY_POSE.copy(); target[0] = start[0]
            for t in np.linspace(0.0, 1.0, 50):
                u=t*t*(3-2*t)
                self._set_arm_target((1-u)*start+u*target)
                self.close_gripper(1.0); self.step_physics(8)
            self.step_physics(60)
            st=self.state(); ok=bool(st["stable"])
            reason=("block lifted and stable under contact forces" if ok else "block was not stably retained during lift")
            return ActionResult(ok, action, reason, st)
        if action == "fetch":
            for name in ("approach","track","pick","carry"):
                r=self.act(name)
                if not r.ok: return r
            return r
        return ActionResult(False, action, "unknown action", self.state())

    def render_jpeg(self, camera: str = "robot_cam", quality: int = 85) -> bytes:
        if self.renderer is None:
            raise RuntimeError("world was created with render=False")
        self.renderer.update_scene(self.data, camera=camera)
        rgb = self.renderer.render()
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()


def scripted_contact_grasp(seed: int = 0, verbose: bool = False) -> tuple[PhysicsGraspWorld, list[dict]]:
    """A calibration controller proving that contact physics can perform a grasp.

    It does not set block qpos after reset. Every block displacement comes from
    MuJoCo constraints/contact forces. This is a baseline for later RL training.
    """
    w = PhysicsGraspWorld(seed=seed, render=False)
    trace = []
    def snap(label):
        s = {"phase": label, **w.state()}
        trace.append(s)
        if verbose: print(s)

    snap("initial")
    w.move_base_to_grasp_range(.47)
    # Let the base transform propagate and the block re-settle.
    w.step_physics(40)
    snap("approach")
    w.open_gripper(); w.step_physics(40)
    w.set_arm_pose(GROUND_POSE, 320)
    snap("ground_pose_open")
    # Close gradually so solver contacts can stop the jaws around the rigid cube.
    for c in np.linspace(0.0, 1.0, 18):
        w.close_gripper(float(c)); w.step_physics(18)
    snap("closed")
    # Lift slowly, maintaining finger force.
    start = np.array([w.data.ctrl[w._id(mujoco.mjtObj.mjOBJ_ACTUATOR, n)] for n in ARM_ACTUATORS], dtype=float)
    for t in np.linspace(0.0, 1.0, 55):
        smooth = t*t*(3-2*t)
        w._set_arm_target((1-smooth)*start + smooth*CARRY_POSE)
        w.close_gripper(1.0)
        w.step_physics(8)
    w.step_physics(80)
    snap("lifted")
    return w, trace


if __name__ == "__main__":
    world, trace = scripted_contact_grasp(seed=4, verbose=True)
    print("final_success=", world.state()["stable"])
    world.close()
