from __future__ import annotations

import io
import math
import os
from dataclasses import dataclass

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

XML = r'''
<mujoco model="ugrp_masterpi_sim">
  <compiler angle="radian"/>
  <option timestep="0.01" gravity="0 0 -9.81" integrator="RK4"/>
  <visual><global offwidth="640" offheight="480"/></visual>
  <asset>
    <texture name="ground" type="2d" builtin="checker" width="256" height="256" rgb1=".20 .22 .24" rgb2=".27 .29 .31"/>
    <material name="groundmat" texture="ground" texrepeat="14 14" reflectance=".035"/>
    <material name="aluminum" rgba=".48 .50 .52 1"/>
    <material name="dark" rgba=".055 .060 .068 1"/>
    <material name="black" rgba=".018 .020 .024 1"/>
    <material name="orange" rgba=".96 .31 .035 1"/>
    <material name="hub" rgba=".28 .30 .32 1"/>
    <material name="redmat" rgba=".88 .04 .04 1"/>
    <material name="blumat" rgba=".04 .18 .88 1"/>
    <material name="yellowmat" rgba=".95 .72 .05 1"/>
  </asset>
  <worldbody>
    <light pos="0 -1.3 2.4" dir="0 .25 -1" diffuse="1 1 1"/>
    <light pos="-1 1.7 1.7" dir=".3 -.4 -1" diffuse=".42 .42 .42"/>
    <geom name="floor" type="plane" size="3 3 .1" material="groundmat"/>
    <geom name="wall_back" type="box" pos="1.8 0 .65" size=".025 1.8 .65" rgba=".54 .56 .58 1"/>

    <!-- MasterPi: ~185 x 162 x 343 mm overall, four mecanum wheels, compact metal chassis. -->
    <body name="robot" pos="0 0 .040" mocap="true">
      <geom name="base_lower" type="box" pos="0 0 .018" size=".064 .054 .022" material="dark" mass="1.1" contype="0" conaffinity="0"/>
      <geom name="base_top" type="box" pos="-.004 0 .052" size=".059 .048 .010" material="aluminum" contype="0" conaffinity="0"/>
      <geom name="front_plate" type="box" pos=".068 0 .021" size=".006 .048 .022" material="aluminum" contype="0" conaffinity="0"/>
      <geom name="rear_cage" type="box" pos="-.044 0 .078" size=".031 .043 .018" material="aluminum" contype="0" conaffinity="0"/>
      <geom name="pi_board" type="box" pos="-.040 0 .066" size=".032 .038 .006" rgba=".07 .26 .12 1" contype="0" conaffinity="0"/>
      <geom type="box" pos="-.044 0 .079" size=".022 .044 .014" material="dark" contype="0" conaffinity="0"/>

      <!-- front ultrasonic sensor pair -->
      <geom type="box" pos=".076 0 .050" size=".009 .032 .015" material="black" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".086 .017 .050" size=".010 .006" euler="0 1.5708 0" rgba=".72 .76 .80 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".086 -.017 .050" size=".010 .006" euler="0 1.5708 0" rgba=".72 .76 .80 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".092 .017 .050" size=".006 .007" euler="0 1.5708 0" rgba=".28 .63 .95 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".092 -.017 .050" size=".006 .007" euler="0 1.5708 0" rgba=".28 .63 .95 1" contype="0" conaffinity="0"/>

      <!-- four mecanum wheels -->
      <geom name="wheel_fl" type="cylinder" pos=".052 .067 0" size=".034 .014" euler="1.5708 0 0" material="hub" contype="0" conaffinity="0"/>
      <geom name="wheel_fr" type="cylinder" pos=".052 -.067 0" size=".034 .014" euler="1.5708 0 0" material="hub" contype="0" conaffinity="0"/>
      <geom name="wheel_rl" type="cylinder" pos="-.052 .067 0" size=".034 .014" euler="1.5708 0 0" material="hub" contype="0" conaffinity="0"/>
      <geom name="wheel_rr" type="cylinder" pos="-.052 -.067 0" size=".034 .014" euler="1.5708 0 0" material="hub" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".052 .082 0" size=".029 .004" euler="1.5708 0 0" material="orange" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos=".052 -.082 0" size=".029 .004" euler="1.5708 0 0" material="orange" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="-.052 .082 0" size=".029 .004" euler="1.5708 0 0" material="orange" contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="-.052 -.082 0" size=".029 .004" euler="1.5708 0 0" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto=".031 .087 -.022 .047 .087 .027" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto=".057 .087 -.027 .073 .087 .022" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto=".031 -.087 .022 .047 -.087 -.027" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto=".057 -.087 .027 .073 -.087 -.022" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-.073 .087 -.022 -.057 .087 .027" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-.047 .087 -.027 -.031 .087 .022" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-.073 -.087 .022 -.057 -.087 -.027" size=".005" material="orange" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-.047 -.087 .027 -.031 -.087 -.022" size=".005" material="orange" contype="0" conaffinity="0"/>

      <!-- Actual MasterPi topology: 4 arm DOF (yaw + shoulder + elbow + wrist pitch) + gripper. -->
      <body name="arm_base" pos="0 0 .064">
        <joint name="arm_yaw" type="hinge" axis="0 0 1" range="-1.55 1.55" damping="2"/>
        <geom type="cylinder" size=".026 .012" pos="0 0 .010" material="black" contype="0" conaffinity="0"/>
        <geom type="box" size=".024 .026 .025" pos="0 0 .037" material="orange" contype="0" conaffinity="0"/>
        <body name="shoulder_link" pos="0 0 .062">
          <joint name="shoulder" type="hinge" axis="0 1 0" range="-1.45 1.35" damping="3"/>
          <geom type="box" size=".021 .024 .021" material="black" contype="0" conaffinity="0"/>
          <geom type="capsule" fromto="0 .025 0 .072 .025 .060" size=".008" material="orange" mass=".10" contype="0" conaffinity="0"/>
          <geom type="capsule" fromto="0 -.025 0 .072 -.025 .060" size=".008" material="orange" contype="0" conaffinity="0"/>
          <body name="elbow_link" pos=".072 0 .060">
            <joint name="elbow" type="hinge" axis="0 1 0" range="-1.65 1.45" damping="3"/>
            <geom type="box" size=".020 .023 .020" material="black" contype="0" conaffinity="0"/>
            <geom type="capsule" fromto="0 .024 0 .075 .024 .035" size=".0075" material="orange" mass=".08" contype="0" conaffinity="0"/>
            <geom type="capsule" fromto="0 -.024 0 .075 -.024 .035" size=".0075" material="orange" contype="0" conaffinity="0"/>
            <body name="wrist_link" pos=".075 0 .035">
              <joint name="wrist_pitch" type="hinge" axis="0 1 0" range="-1.65 1.65" damping="2"/>
              <geom type="box" size=".018 .022 .018" material="black" contype="0" conaffinity="0"/>
              <geom type="capsule" fromto="0 .022 0 .045 .022 -.002" size=".0065" material="orange" mass=".05" contype="0" conaffinity="0"/>
              <geom type="capsule" fromto="0 -.022 0 .045 -.022 -.002" size=".0065" material="orange" contype="0" conaffinity="0"/>
              <body name="gripper" pos=".045 0 -.002">
                <geom type="box" pos=".004 0 .025" size=".018 .025 .014" material="black" contype="0" conaffinity="0"/>
                <geom type="cylinder" pos=".024 0 .025" size=".010 .008" euler="0 1.5708 0" rgba=".08 .08 .09 1" contype="0" conaffinity="0"/>
                <camera name="robot_cam" pos=".030 0 .026" xyaxes="0 -1 0 0 0 1" fovy="62"/>
                <geom type="box" pos=".012 .022 0" size=".018 .006 .012" material="aluminum" contype="0" conaffinity="0"/>
                <geom type="box" pos=".012 -.022 0" size=".018 .006 .012" material="aluminum" contype="0" conaffinity="0"/>
                <geom name="left_finger" type="box" pos=".047 .035 -.004" size=".034 .006 .006" material="orange" contype="0" conaffinity="0"/>
                <body name="right_jaw" pos="0 -.035 0">
                  <joint name="gripper_open" type="slide" axis="0 1 0" range="0 .012" damping="1"/>
                  <geom name="right_finger" type="box" pos=".047 0 -.004" size=".034 .006 .006" material="orange" contype="0" conaffinity="0"/>
                </body>
                <site name="grip_site" pos=".075 0 -.004" size=".008" rgba="1 1 .2 .45"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- 50 mm cubes, directly on the same z=0 floor; no table. -->
    <body name="red_block" pos=".55 .07 .025">
      <freejoint name="red_free"/>
      <geom name="red_block_geom" type="box" size=".025 .025 .025" material="redmat" mass=".05" friction="1 .01 .001"/>
    </body>
    <body name="blue_block" pos=".70 -.20 .025">
      <freejoint/>
      <geom type="box" size=".025 .025 .025" material="blumat" mass=".05" friction="1 .01 .001"/>
    </body>
    <body name="yellow_block" pos=".82 .23 .025">
      <freejoint/>
      <geom type="box" size=".025 .025 .025" material="yellowmat" mass=".05" friction="1 .01 .001"/>
    </body>
    <camera name="overview" pos="-.58 -.88 .52" xyaxes=".83 -.55 0 .26 .39 .88" fovy="47"/>
  </worldbody>
</mujoco>
'''



@dataclass
class ActionResult:
    ok: bool
    action: str
    reason: str
    state: dict


class MasterPiWorld:
    def __init__(self, seed: int = 0, width: int = 640, height: int = 480):
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(XML)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.width, self.height = width, height
        self.held = False
        self.stable = False
        self.pick_attempts = 0
        self.reset(seed)

    def reset(self, seed: int | None = None) -> dict:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        bx = .55 + float(self.rng.uniform(-.045, .045))
        by = float(self.rng.uniform(-.20, .20))
        qadr = self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "red_free")]
        self.data.qpos[qadr:qadr+3] = [bx, by, .025]
        self.data.qpos[qadr+3:qadr+7] = [1, 0, 0, 0]
        self.base_yaw = 0.0
        self.data.mocap_pos[0] = [0.0, 0.0, 0.040]
        self.data.mocap_quat[0] = [1.0, 0.0, 0.0, 0.0]
        self._set_joint("arm_yaw", 0.0)
        self._set_joint("shoulder", -0.30)
        self._set_joint("elbow", 0.55)
        self._set_joint("wrist_pitch", -0.20)
        self._set_joint("gripper_open", 0.0)
        self.held = False
        self.stable = False
        self.pick_attempts = 0
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def _id(self, typ, name: str) -> int:
        return mujoco.mj_name2id(self.model, typ, name)

    def _set_joint(self, name: str, value: float) -> None:
        jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
        qadr = self.model.jnt_qposadr[jid]
        dadr = self.model.jnt_dofadr[jid]
        self.data.qpos[qadr] = value
        self.data.qvel[dadr] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _body_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self._id(mujoco.mjtObj.mjOBJ_BODY, name)].copy()

    def _site_pos(self, name: str) -> np.ndarray:
        return self.data.site_xpos[self._id(mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    def _step(self, n: int = 1) -> None:
        # High-level agent skills are kinematic. MuJoCo dynamics remain available
        # for future learned low-level policies without letting scripted skills
        # inject unstable joint forces.
        if self.held:
            self._pin_block_to_gripper()
        mujoco.mj_forward(self.model, self.data)

    def physics_step(self, n: int = 1) -> None:
        for _ in range(max(1, n)):
            if self.held:
                self._pin_block_to_gripper()
            mujoco.mj_step(self.model, self.data)

    def _pin_block_to_gripper(self) -> None:
        qadr = self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, "red_free")]
        p = self._site_pos("grip_site")
        self.data.qpos[qadr:qadr+3] = p
        self.data.qpos[qadr+3:qadr+7] = [1, 0, 0, 0]
        vadr = self.model.jnt_dofadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, "red_free")]
        self.data.qvel[vadr:vadr+6] = 0

    def state(self) -> dict:
        robot = self._body_pos("robot")
        block = self._body_pos("red_block")
        dx, dy = block[0] - robot[0], block[1] - robot[1]
        dist = float(math.hypot(dx, dy))
        return {
            "robot_xy": [round(float(robot[0]), 3), round(float(robot[1]), 3)],
            "red_xy": [round(float(block[0]), 3), round(float(block[1]), 3)],
            "distance": round(dist, 3),
            "held": bool(self.held),
            "stable": bool(self.stable),
            "pick_attempts": int(self.pick_attempts),
            "arm_yaw": round(float(self.data.qpos[self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, "arm_yaw")]]), 3),
        }

    def render_jpeg(self, quality: int = 85) -> bytes:
        self.renderer.update_scene(self.data, camera="robot_cam")
        rgb = self.renderer.render()
        im = Image.fromarray(rgb)
        buf = io.BytesIO()
        im.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

    def render_overview_jpeg(self, quality: int = 85) -> bytes:
        self.renderer.update_scene(self.data, camera="overview")
        rgb = self.renderer.render()
        im = Image.fromarray(rgb)
        buf = io.BytesIO()
        im.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

    def act(self, action: str) -> ActionResult:
        action = action.strip().lower()
        if action == "approach":
            return self._approach()
        if action == "track":
            return self._track()
        if action == "pick":
            return self._pick()
        if action == "carry":
            return self._carry()
        if action == "fetch":
            a = self._approach()
            if not a.ok:
                return a
            self._track()
            return self._pick()
        if action == "reset":
            return ActionResult(True, action, "world reset", self.reset())
        return ActionResult(False, action, "unknown action", self.state())

    def _approach(self) -> ActionResult:
        robot = self._body_pos("robot")
        block = self._body_pos("red_block")
        dx, dy = block[0] - robot[0], block[1] - robot[1]
        d = max(math.hypot(dx, dy), 1e-6)
        stop = .155
        target = block[:2] - np.array([dx, dy]) / d * stop
        tx = float(np.clip(target[0], -2, 2))
        ty = float(np.clip(target[1], -2, 2))
        self.base_yaw = math.atan2(dy, dx)
        self.data.mocap_pos[0] = [tx, ty, 0.040]
        self.data.mocap_quat[0] = [math.cos(self.base_yaw/2), 0.0, 0.0, math.sin(self.base_yaw/2)]
        mujoco.mj_forward(self.model, self.data)
        return ActionResult(True, "approach", "moved to grasp range", self.state())

    def _track(self) -> ActionResult:
        robot = self._body_pos("robot")
        block = self._body_pos("red_block")
        desired = math.atan2(block[1] - robot[1], block[0] - robot[0])
        base_yaw = float(self.base_yaw)
        head = (desired - base_yaw + math.pi) % (2 * math.pi) - math.pi
        self._set_joint("arm_yaw", float(np.clip(head, -1.45, 1.45)))
        mujoco.mj_forward(self.model, self.data)
        return ActionResult(True, "track", "camera centered toward target", self.state())

    def _pick(self) -> ActionResult:
        self.pick_attempts += 1
        robot = self._body_pos("robot")
        block = self._body_pos("red_block")
        dist = float(np.linalg.norm(block[:2] - robot[:2]))
        # MasterPi 4DOF ground-grasp pose: yaw + shoulder + elbow + wrist pitch.
        self._set_joint("shoulder", 0.30)
        self._set_joint("elbow", 0.90)
        self._set_joint("wrist_pitch", 0.25)
        self._set_joint("gripper_open", 0.012)
        mujoco.mj_forward(self.model, self.data)
        grip = self._site_pos("grip_site")
        planar = float(np.linalg.norm(block[:2] - grip[:2]))
        if dist > .22 or planar > .045:
            return ActionResult(False, "pick", f"target not aligned (base={dist:.2f}, grip={planar:.2f})", self.state())
        self.held = True
        self._pin_block_to_gripper()
        self._step(20)
        return ActionResult(True, "pick", "block attached after valid grasp geometry", self.state())

    def _carry(self) -> ActionResult:
        if not self.held:
            return ActionResult(False, "carry", "nothing is grasped", self.state())
        self._set_joint("shoulder", -0.28)
        self._set_joint("elbow", 0.42)
        self._set_joint("wrist_pitch", -0.15)
        self._set_joint("gripper_open", 0.012)
        self._step(20)
        self.stable = True
        return ActionResult(True, "carry", "held block stabilized in carry pose", self.state())
