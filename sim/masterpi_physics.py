from __future__ import annotations

import io
import math
import os
from dataclasses import dataclass
from typing import Callable, Iterable

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

from sim.mujoco_world import ActionResult, XML as VISUAL_XML

# Keep the same geometry as the visual MasterPi world, but enable real finger/block
# contact and position actuators for the four arm axes plus one gripper axis.
# The block is never welded, teleported, or pinned after grasping.
_PHYSICS_XML = VISUAL_XML
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<compiler angle="radian"/>',
    '<compiler angle="radian" autolimits="true"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<option timestep="0.01" gravity="0 0 -9.81" integrator="RK4"/>',
    '<option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" iterations="80"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<body name="robot" pos="0 0 .040" mocap="true">',
    '''<body name="robot" pos="0 0 .040">
      <!-- Dynamic planar chassis.  X/Y/yaw are real generalized coordinates;
           z/roll/pitch stay constrained by the planar mechanism. -->
      <joint name="base_x" type="slide" axis="1 0 0" damping="8"/>
      <joint name="base_y" type="slide" axis="0 1 0" damping="8"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1" damping="1.5"/>''',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<geom name="left_finger" type="box" pos=".047 .035 -.004" size=".034 .006 .006" material="orange" contype="0" conaffinity="0"/>',
    '<geom name="left_finger" type="box" pos=".047 .035 -.025" size=".034 .006 .014" material="orange" mass=".018" contype="1" conaffinity="1" condim="4" friction="3.4 .05 .003"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<geom name="right_finger" type="box" pos=".047 0 -.004" size=".034 .006 .006" material="orange" contype="0" conaffinity="0"/>',
    '<geom name="right_finger" type="box" pos=".047 0 -.025" size=".034 .006 .014" material="orange" mass=".018" contype="1" conaffinity="1" condim="4" friction="3.4 .05 .003"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<geom name="red_block_geom" type="box" size=".025 .025 .025" material="redmat" mass=".05" friction="1 .01 .001"/>',
    '<geom name="red_block_geom" type="box" size=".025 .025 .025" material="redmat" mass=".05" contype="1" conaffinity="1" condim="4" friction="1.5 .02 .001"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<geom name="blue_block_geom" type="box" size=".025 .025 .025" material="blumat" mass=".05" friction="1 .01 .001"/>',
    '<geom name="blue_block_geom" type="box" size=".025 .025 .025" material="blumat" mass=".05" contype="1" conaffinity="1" condim="4" friction="1.5 .02 .001"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '<geom name="yellow_block_geom" type="box" size=".025 .025 .025" material="yellowmat" mass=".05" friction="1 .01 .001"/>',
    '<geom name="yellow_block_geom" type="box" size=".025 .025 .025" material="yellowmat" mass=".05" contype="1" conaffinity="1" condim="4" friction="1.5 .02 .001"/>',
)
_PHYSICS_XML = _PHYSICS_XML.replace(
    '</mujoco>',
    '''  <actuator>
    <position name="p_base_x" joint="base_x" kp="120" ctrllimited="true" ctrlrange="-3 3" forcelimited="true" forcerange="-30 30"/>
    <position name="p_base_y" joint="base_y" kp="120" ctrllimited="true" ctrlrange="-3 3" forcelimited="true" forcerange="-30 30"/>
    <position name="p_base_yaw" joint="base_yaw" kp="35" ctrllimited="true" ctrlrange="-100 100" forcelimited="true" forcerange="-8 8"/>
    <velocity name="a_base_x" joint="base_x" kv="55" ctrllimited="true" ctrlrange="-.55 .55" forcelimited="true" forcerange="-22 22"/>
    <velocity name="a_base_y" joint="base_y" kv="55" ctrllimited="true" ctrlrange="-.55 .55" forcelimited="true" forcerange="-22 22"/>
    <velocity name="a_base_yaw" joint="base_yaw" kv="18" ctrllimited="true" ctrlrange="-2.2 2.2" forcelimited="true" forcerange="-5 5"/>
    <position name="a_yaw" joint="arm_yaw" kp="35" ctrllimited="true" ctrlrange="-1.55 1.55" forcelimited="true" forcerange="-4 4"/>
    <position name="a_shoulder" joint="shoulder" kp="48" ctrllimited="true" ctrlrange="-1.45 1.35" forcelimited="true" forcerange="-8 8"/>
    <position name="a_elbow" joint="elbow" kp="45" ctrllimited="true" ctrlrange="-1.65 1.45" forcelimited="true" forcerange="-7 7"/>
    <position name="a_wrist_pitch" joint="wrist_pitch" kp="30" ctrllimited="true" ctrlrange="-1.65 1.65" forcelimited="true" forcerange="-4 4"/>
    <position name="a_gripper" joint="gripper_open" kp="260" ctrllimited="true" ctrlrange="0 .012" forcelimited="true" forcerange="-12 12"/>
  </actuator>
</mujoco>''',
)

XML = _PHYSICS_XML
ARM_JOINTS = ("arm_yaw", "shoulder", "elbow", "wrist_pitch")
ARM_ACTUATORS = ("a_yaw", "a_shoulder", "a_elbow", "a_wrist_pitch")
GROUND_POSE = np.array([0.0, 0.30, 0.90, 0.25], dtype=float)
GRASP_HOVER_POSE = np.array([0.0, -0.195926, 0.873160, 0.714279], dtype=float)
CARRY_POSE = np.array([0.0, -0.28, 0.42, -0.15], dtype=float)
LIFT_POSE = np.array([0.0, 0.10, 0.72, 0.10], dtype=float)
SEARCH_POSE = np.array([0.0, 0.0, 0.50, 0.20], dtype=float)
# Dedicated visual-servo pose. SEARCH_POSE points too far downward/forward for
# distant targets and can push a target from the bottom of a survey view to the
# top edge in one pose transition. This midpoint keeps typical 0.5-0.9 m floor
# targets near the vertical image center while leaving the gripper clear.
TRACK_POSE = 0.5 * (CARRY_POSE + SEARCH_POSE)
STACK_POSE = np.array([0.0, 0.20, 0.60, 0.05], dtype=float)

# Semantic destinations are known landmarks in the workcell map. They are not
# object truth: these correspond to the visible delivery pads in the scene and
# can later be replaced by user-defined/SLAM landmarks on the real robot.
DROP_ZONES = {
    "BLUE DELIVERY": {"center_xy": [0.72, -0.82], "size_xy": [0.44, 0.36], "kind": "drop", "accepts": ["red", "blue"]},
    "YELLOW DELIVERY": {"center_xy": [0.72, 0.82], "size_xy": [0.44, 0.36], "kind": "drop", "accepts": ["red", "yellow"]},
}


@dataclass
class ContactState:
    left: bool
    right: bool
    left_force: float
    right_force: float

    @property
    def bilateral(self) -> bool:
        return self.left and self.right


class MasterPiPhysicsWorld:
    """4DOF+gripper MasterPi world with real MuJoCo finger contact/friction."""

    def __init__(self, seed: int = 0, width: int = 640, height: int = 480, render: bool = True):
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)
        self.model = mujoco.MjModel.from_xml_string(XML)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        self.width, self.height = width, height
        self._base_width, self._base_height = int(width), int(height)
        self.base_yaw = 0.0
        self.odom_xy = np.zeros(2, dtype=float)
        self.yellow_map_xy: np.ndarray | None = None
        self.blue_map_xy: np.ndarray | None = None
        # Object-centric spatial memory. Every coordinate here is inferred from
        # camera pixels + calibrated camera pose + robot odometry; never object truth.
        self.spatial_memory: dict[str, dict] = {}
        self._memory_observation_id = 0
        # Sparse camera-depth occupancy memory. Keys are 8 cm world cells; no
        # simulator body IDs/poses are used to populate this map.
        self.obstacle_cells: dict[tuple[int, int], dict] = {}
        self._obstacle_scan_interval = max(1, int(os.environ.get("UGRP_OBSTACLE_SCAN_INTERVAL", "3")))
        self._stable_steps = 0
        self.grasp_color: str | None = None
        self.speed_multiplier = 1.0
        self.frame_callback: Callable[[], None] | None = None
        self.reset(seed)


    def set_speed_multiplier(self, value: float) -> None:
        value = float(value)
        if value not in (1.0, 2.0, 3.0):
            raise ValueError("speed multiplier must be 1, 2, or 3")
        self.speed_multiplier = value
        if self.renderer is not None:
            scale = {1.0: 1.0, 2.0: 0.75, 3.0: 0.5}[value]
            target_w = max(240, int(round(self._base_width * scale)))
            target_h = max(180, int(round(self._base_height * scale)))
            if (target_w, target_h) != (self.width, self.height):
                self.renderer.close()
                self.renderer = mujoco.Renderer(self.model, height=target_h, width=target_w)
                self.width, self.height = target_w, target_h

    def _fast_samples(self, nominal: int, *, minimum: int = 2) -> int:
        return max(minimum, int(math.ceil(float(nominal) / self.speed_multiplier)))

    def _fast_steps(self, nominal: int) -> int:
        # Preserve approximately the same simulated physics time while reducing
        # Python interpolation/render iterations at 2x/3x wall-clock speed.
        return max(1, int(round(float(nominal) * self.speed_multiplier)))

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def _id(self, typ, name: str) -> int:
        return mujoco.mj_name2id(self.model, typ, name)

    def _joint_qpos(self, name: str) -> float:
        jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(self.data.qpos[self.model.jnt_qposadr[jid]])

    def _set_joint_qpos(self, name: str, value: float) -> None:
        jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
        self.data.qpos[self.model.jnt_qposadr[jid]] = float(value)
        self.data.qvel[self.model.jnt_dofadr[jid]] = 0.0

    def _set_actuator(self, name: str, value: float) -> None:
        aid = self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        lo, hi = self.model.actuator_ctrlrange[aid]
        self.data.ctrl[aid] = float(np.clip(value, lo, hi))

    def _arm_targets(self) -> np.ndarray:
        return np.array([
            self.data.ctrl[self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)]
            for name in ARM_ACTUATORS
        ], dtype=float)

    def _set_arm_target(self, pose: Iterable[float]) -> None:
        for actuator, value in zip(ARM_ACTUATORS, pose):
            self._set_actuator(actuator, float(value))

    def _body_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self._id(mujoco.mjtObj.mjOBJ_BODY, name)].copy()

    def _site_pos(self, name: str) -> np.ndarray:
        return self.data.site_xpos[self._id(mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    def open_gripper(self) -> None:
        self._set_actuator("a_gripper", 0.0)

    def close_gripper(self, amount: float = 1.0) -> None:
        self._set_actuator("a_gripper", 0.012 * float(np.clip(amount, 0.0, 1.0)))

    def reset(self, seed: int | None = None, block_xy: tuple[float, float] | None = None) -> dict:
        if seed is None:
            seed = self.seed
        seed = int(seed)
        if seed < 0 or seed > 2**31 - 1:
            raise ValueError("seed must be between 0 and 2147483647")
        self.seed = seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        if block_xy is None:
            bx = .55 + float(self.rng.uniform(-.045, .045))
            by = float(self.rng.uniform(-.20, .20))
        else:
            bx, by = map(float, block_xy)
        red_jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, "red_free")
        qa = self.model.jnt_qposadr[red_jid]
        self.data.qpos[qa:qa+3] = [bx, by, .025]
        self.data.qpos[qa+3:qa+7] = [1, 0, 0, 0]
        for name in ("base_x", "base_y", "base_yaw"):
            self._set_joint_qpos(name, 0.0)
        for name in ("a_base_x", "a_base_y", "a_base_yaw", "p_base_x", "p_base_y", "p_base_yaw"):
            self._set_actuator(name, 0.0)
        self.base_yaw = 0.0
        self.odom_xy[:] = 0.0
        self.yellow_map_xy = None
        self.blue_map_xy = None
        self.spatial_memory.clear()
        self._memory_observation_id = 0
        self.obstacle_cells.clear()
        for name, value in zip(ARM_JOINTS, CARRY_POSE):
            self._set_joint_qpos(name, float(value))
        self._set_joint_qpos("gripper_open", 0.0)
        self._set_arm_target(CARRY_POSE)
        self.open_gripper()
        self._stable_steps = 0
        self.grasp_color = None
        mujoco.mj_forward(self.model, self.data)
        self.step_physics(80)
        return self.state()

    def contact_state(self, color: str | None = None) -> ContactState:
        color = color or self.grasp_color or "red"
        if color not in {"red", "blue", "yellow"}:
            color = "red"
        block_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, f"{color}_block_geom")
        left_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, "left_finger")
        right_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, "right_finger")
        left = right = False
        lf = rf = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if block_gid not in (g1, g2):
                continue
            other = g2 if g1 == block_gid else g1
            force = np.zeros(6, dtype=float)
            if c.efc_address >= 0:
                mujoco.mj_contactForce(self.model, self.data, i, force)
            normal = abs(float(force[0]))
            if other == left_gid:
                left = True; lf += normal
            if other == right_gid:
                right = True; rf += normal
        return ContactState(left, right, lf, rf)

    def step_physics(self, n: int = 1) -> None:
        for _ in range(max(1, int(n))):
            mujoco.mj_step(self.model, self.data)
            color = self.grasp_color or "red"
            cs = self.contact_state(color)
            block_z = float(self._body_pos(f"{color}_block")[2])
            if cs.bilateral and block_z > .060:
                self._stable_steps += 1
            else:
                self._stable_steps = 0

    def _emit_frame(self) -> None:
        callback = self.frame_callback
        if callback is not None:
            callback()

    def _base_joint_value(self, name: str) -> float:
        return self._joint_qpos(name)

    def _base_velocity(self, name: str) -> float:
        jid = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(self.data.qvel[self.model.jnt_dofadr[jid]])

    def _set_base_hold(self, x: float | None = None, y: float | None = None, yaw: float | None = None) -> None:
        if x is None:
            x = self._base_joint_value("base_x")
        if y is None:
            y = self._base_joint_value("base_y")
        if yaw is None:
            yaw = self._base_joint_value("base_yaw")
        self._set_actuator("p_base_x", float(x))
        self._set_actuator("p_base_y", float(y))
        self._set_actuator("p_base_yaw", float(yaw))

    def _stop_base(self, settle_steps: int = 18) -> None:
        self._set_actuator("a_base_x", 0.0)
        self._set_actuator("a_base_y", 0.0)
        self._set_actuator("a_base_yaw", 0.0)
        # Latch the current wheel-odometry pose. This models the real drive
        # controller holding the chassis while the arm moves, without mocap.
        self._set_base_hold()
        self.step_physics(settle_steps)
        # Re-latch after the short settling transient.
        self._set_base_hold()
        self.base_yaw = self._base_joint_value("base_yaw")

    def move_base_relative(
        self, forward: float = 0.0, yaw_delta: float = 0.0, frames: int = 8,
        *, max_v: float = 0.12, max_w: float = 0.65,
        a_lin: float = 0.75, a_ang: float = 3.0,
    ) -> None:
        """Drive the planar chassis dynamically using velocity actuators.

        The controller uses only commanded odometry.  Unlike the old mocap
        implementation, the chassis, arm, fingers and grasped object all evolve
        in the same MuJoCo dynamics, so carrying an object does not leave it
        behind when the base moves.
        """
        forward = float(forward)
        yaw_delta = float(yaw_delta)
        start_xy = np.array([
            self._base_joint_value("base_x"),
            self._base_joint_value("base_y"),
        ], dtype=float)
        start_yaw = self._base_joint_value("base_yaw")
        self.base_yaw = start_yaw
        heading = np.array([math.cos(start_yaw), math.sin(start_yaw)], dtype=float)

        # Simulation speed changes wall-clock rendering cadence/resolution, not
        # robot physics.  The same command must have the same dynamics at 1x,
        # 2x and 3x so accelerated viewing does not make grasping less stable.
        max_v = float(max_v)
        max_w = float(max_w)
        a_lin = float(a_lin)
        a_ang = float(a_ang)
        dt = float(self.model.opt.timestep)
        emit_budget = max(2, self._fast_samples(max(2, int(frames))))

        linear_done = abs(forward) < 1e-6
        angular_done = abs(yaw_delta) < 1e-6
        target_xy = start_xy + forward * heading
        target_yaw = start_yaw + yaw_delta
        max_steps = int(max(1.0, abs(forward) / max(0.02, max_v), abs(yaw_delta) / max(0.15, max_w)) / dt * 8.0) + 350
        emit_every = max(1, max_steps // max(2, emit_budget * 4))

        # Acceleration limits are what preserve a friction grasp during base
        # motion; speed multiplier does not scale these as aggressively.
        cmd_v = 0.0
        cmd_w = 0.0
        for i in range(max_steps):
            xy = np.array([
                self._base_joint_value("base_x"),
                self._base_joint_value("base_y"),
            ])
            yaw = self._base_joint_value("base_yaw")
            self.base_yaw = yaw

            if not linear_done:
                remaining = float(np.dot(target_xy - xy, heading))
                sign = 1.0 if forward >= 0 else -1.0
                if sign * remaining <= 0.0025:
                    desired_v = 0.0
                    linear_done = True
                else:
                    brake_v = math.sqrt(max(0.0, 2.0 * a_lin * abs(remaining)))
                    desired_v = sign * min(max_v, brake_v)
                dv = float(np.clip(desired_v - cmd_v, -a_lin * dt, a_lin * dt))
                cmd_v += dv
            else:
                dv = float(np.clip(-cmd_v, -a_lin * dt, a_lin * dt))
                cmd_v += dv

            if not angular_done:
                remaining_yaw = target_yaw - yaw
                sign_w = 1.0 if yaw_delta >= 0 else -1.0
                if sign_w * remaining_yaw <= math.radians(0.4):
                    desired_w = 0.0
                    angular_done = True
                else:
                    brake_w = math.sqrt(max(0.0, 2.0 * a_ang * abs(remaining_yaw)))
                    desired_w = sign_w * min(max_w, brake_w)
                dw = float(np.clip(desired_w - cmd_w, -a_ang * dt, a_ang * dt))
                cmd_w += dw
            else:
                dw = float(np.clip(-cmd_w, -a_ang * dt, a_ang * dt))
                cmd_w += dw

            # base_x/base_y actuators use world-frame generalized velocities.
            self._set_base_hold(float(xy[0]), float(xy[1]), float(yaw))
            self._set_actuator("a_base_x", cmd_v * heading[0])
            self._set_actuator("a_base_y", cmd_v * heading[1])
            self._set_actuator("a_base_yaw", cmd_w)
            self.step_physics(1)

            if i % emit_every == 0:
                self._emit_frame()

            stopped = abs(cmd_v) < 0.006 and abs(cmd_w) < 0.025
            if linear_done and angular_done and stopped:
                break

        self._stop_base(settle_steps=12)
        end_xy = np.array([
            self._base_joint_value("base_x"),
            self._base_joint_value("base_y"),
        ])
        # Odometry is measured from the robot's own joint encoders, not target
        # object/world truth.  This is the same abstraction available on real hardware.
        self.odom_xy += end_xy - start_xy
        self.base_yaw = self._base_joint_value("base_yaw")
        self._emit_frame()

    def render_rgb(self, camera: str = "robot_cam") -> np.ndarray:
        if self.renderer is None:
            raise RuntimeError("world was created with render=False")
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render().copy()

    @staticmethod
    def _compact_component(mask: np.ndarray, *, min_pixels: int, max_area_ratio: float = 0.12) -> tuple[np.ndarray, np.ndarray] | None:
        """Return the largest compact 4-connected component from a color mask."""
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=np.uint8)
        best = None
        best_n = 0
        ys0, xs0 = np.where(mask)
        for sy, sx in zip(ys0.tolist(), xs0.tolist()):
            if visited[sy, sx]:
                continue
            stack = [(sy, sx)]
            visited[sy, sx] = 1
            ys = []
            xs = []
            while stack:
                y, x = stack.pop()
                ys.append(y); xs.append(x)
                if y > 0 and mask[y-1, x] and not visited[y-1, x]:
                    visited[y-1, x] = 1; stack.append((y-1, x))
                if y+1 < h and mask[y+1, x] and not visited[y+1, x]:
                    visited[y+1, x] = 1; stack.append((y+1, x))
                if x > 0 and mask[y, x-1] and not visited[y, x-1]:
                    visited[y, x-1] = 1; stack.append((y, x-1))
                if x+1 < w and mask[y, x+1] and not visited[y, x+1]:
                    visited[y, x+1] = 1; stack.append((y, x+1))
            n = len(xs)
            if n < min_pixels or n > int(h*w*max_area_ratio):
                continue
            bw = max(xs)-min(xs)+1; bh = max(ys)-min(ys)+1
            aspect = bw/max(1, bh)
            fill = n/max(1, bw*bh)
            if not (0.45 <= aspect <= 2.2) or fill < 0.18:
                continue
            if n > best_n:
                best_n = n
                best = (np.asarray(ys), np.asarray(xs))
        return best

    @staticmethod
    def _compact_components(mask: np.ndarray, *, min_pixels: int, max_area_ratio: float = 0.12) -> list[tuple[np.ndarray, np.ndarray]]:
        """Return every compact candidate, not just the largest same-color region."""
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=np.uint8)
        out: list[tuple[np.ndarray, np.ndarray]] = []
        ys0, xs0 = np.where(mask)
        for sy, sx in zip(ys0.tolist(), xs0.tolist()):
            if visited[sy, sx]:
                continue
            stack=[(sy,sx)]; visited[sy,sx]=1; ys=[]; xs=[]
            while stack:
                y,x=stack.pop(); ys.append(y); xs.append(x)
                if y>0 and mask[y-1,x] and not visited[y-1,x]: visited[y-1,x]=1; stack.append((y-1,x))
                if y+1<h and mask[y+1,x] and not visited[y+1,x]: visited[y+1,x]=1; stack.append((y+1,x))
                if x>0 and mask[y,x-1] and not visited[y,x-1]: visited[y,x-1]=1; stack.append((y,x-1))
                if x+1<w and mask[y,x+1] and not visited[y,x+1]: visited[y,x+1]=1; stack.append((y,x+1))
            n=len(xs)
            if n<min_pixels or n>int(h*w*max_area_ratio):
                continue
            bw=max(xs)-min(xs)+1; bh=max(ys)-min(ys)+1
            aspect=bw/max(1,bh); fill=n/max(1,bw*bh)
            if 0.40 <= aspect <= 2.35 and fill >= 0.16:
                out.append((np.asarray(ys),np.asarray(xs)))
        return out

    def _projected_component_width_m(self, comp) -> float | None:
        """Estimate same-height horizontal footprint from camera geometry only."""
        if comp is None:
            return None
        ys,xs=comp; cy=float(np.median(ys))/max(1,self.height)
        x0=float(xs.min())/max(1,self.width); x1=float(xs.max())/max(1,self.width)
        left=self._camera_pixel_ground_xy({'visible':True,'cx':x0,'cy':cy},plane_z=.025)
        right=self._camera_pixel_ground_xy({'visible':True,'cx':x1,'cy':cy},plane_z=.025)
        if left is None or right is None:
            return None
        return float(np.linalg.norm(right-left))

    def _select_block_component(self, mask: np.ndarray, *, min_pixels: int):
        """Select a 50 mm cube candidate among same-color pads/crates.

        Color alone is ambiguous in this scene. Candidate image blobs are projected
        through calibrated camera geometry and scored by their inferred horizontal
        footprint. This remains camera/FK based and does not use simulator object IDs.
        """
        candidates=self._compact_components(mask,min_pixels=min_pixels)
        best=None; best_score=float('inf')
        for comp in candidates:
            ys,xs=comp
            # A clipped same-color pad/crate can masquerade as a 5 cm object
            # because only a narrow fragment is visible. Do not identify an
            # object from a component truncated by the left/right/top image edge.
            # Bottom clipping remains allowed for a genuinely close block.
            if int(xs.min()) <= 1 or int(xs.max()) >= self.width-2 or int(ys.min()) <= 1:
                continue
            width=self._projected_component_width_m(comp)
            if width is None or not (0.018 <= width <= 0.115):
                continue
            ys,xs=comp; bw=int(xs.max()-xs.min()+1); bh=int(ys.max()-ys.min()+1)
            aspect=bw/max(1,bh); fill=len(xs)/max(1,bw*bh)
            # 50 mm physical width dominates; near-square/high-fill components are tie-breakers.
            score=abs(width-.050)/.050 + .20*abs(math.log(max(.25,aspect))) + .08*(1.0-min(1.0,fill))
            if score<best_score:
                best_score=score; best=comp
        return best

    @staticmethod
    def _detection_from_component(comp, width: int, height: int) -> dict:
        if comp is None:
            return {"visible": False, "pixels": 0}
        ys, xs = comp
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        return {
            "visible": True,
            "cx": round(float(xs.mean()) / width, 4),
            "cy": round(float(ys.mean()) / height, 4),
            "bbox": [round(x0/width,4), round(y0/height,4), round(x1/width,4), round(y1/height,4)],
            "area_ratio": round(float(xs.size)/(width*height),5),
            "pixels": int(xs.size),
        }

    def scene_detections(self, rgb: np.ndarray | None = None) -> dict[str, dict]:
        """Detect all known colored objects from one robot-camera render."""
        if rgb is None:
            rgb = self.render_rgb("robot_cam")
        r = rgb[..., 0].astype(np.int16)
        g = rgb[..., 1].astype(np.int16)
        b = rgb[..., 2].astype(np.int16)
        masks = {
            "red": (r > 50) & (r > 4*g) & (r > 4*b) & ((r-g) > 30),
            "yellow": (r > 100) & (g > 65) & (g > 0.50*r) & (g < 0.90*r) & (b < 85) & ((g-b) > 45),
            "blue": (b > 70) & (b > 1.8*r) & (b > 1.6*g) & ((b-r) > 45) & ((b-g) > 35),
        }
        out = {}
        for color, mask in masks.items():
            min_pixels = max(
                18 if color == "red" else 16,
                int(self.width*self.height*(0.0008 if color == "red" else 0.00045)),
            )
            # Red has no large same-color workcell landmark today, but using the
            # same geometry-aware selector prevents future semantic color clashes.
            comp = self._select_block_component(mask, min_pixels=min_pixels)
            out[color] = self._detection_from_component(comp, self.width, self.height)
        return out

    def _update_spatial_memory(self, detections: dict[str, dict]) -> None:
        self._memory_observation_id += 1
        obs_id = self._memory_observation_id
        for entry in self.spatial_memory.values():
            entry["visible"] = False
        for color, det in detections.items():
            if not det.get("visible"):
                continue
            # Verified manipulation relations are stronger than a generic floor
            # ray. Do not demote ON_BLUE/ON_YELLOW merely because a later RGB
            # frame sees the red cube; only another verified manipulation should
            # change that semantic relation.
            entry = self.spatial_memory.get(color)
            if color == "red" and entry and str(entry.get("relation", "")).startswith("ON_") and not self.contact_state().bilateral:
                entry["visible"] = True
                entry["last_seen_observation"] = obs_id
                continue
            # A red block held between the fingers is not on the floor, so do not
            # reinterpret its image ray as a new floor coordinate.
            if color == "red" and self.contact_state().bilateral:
                if entry:
                    entry["visible"] = True
                    entry["relation"] = "HELD"
                    entry["last_seen_observation"] = obs_id
                continue
            # Ground-plane intersection becomes ill-conditioned close to the
            # image horizon/top and at extreme horizontal edges. Preserve the
            # landmark as visible, but do not let such a frame move its 3D pose.
            cx = float(det.get("cx", 0.5))
            cy = float(det.get("cy", 0.5))
            if not (0.06 <= cx <= 0.94 and 0.12 <= cy <= 0.90):
                entry = self.spatial_memory.get(color)
                if entry:
                    entry["visible"] = True
                    entry["last_seen_observation"] = obs_id
                continue
            xy = self._camera_pixel_ground_xy(det, plane_z=0.025)
            if xy is None:
                continue
            old = self.spatial_memory.get(color)
            candidate_xy = None
            candidate_count = 0
            if old and old.get("xy") is not None:
                previous = np.asarray(old["xy"], dtype=float)
                innovation = float(np.linalg.norm(xy - previous))
                seen_count = int(old.get("seen_count", 0)) + 1
                if innovation <= 0.030:
                    # Consistent observation: fuse into the established landmark.
                    xy = 0.55 * previous + 0.45 * xy
                else:
                    # Large jumps are common when a floor ray is nearly grazing.
                    # Do not destroy a stable landmark from one oblique view. A
                    # genuinely moved object must form a repeatable new cluster.
                    prior_candidate = old.get("candidate_xy")
                    prior_count = int(old.get("candidate_count", 0))
                    if prior_candidate is not None and float(np.linalg.norm(xy - np.asarray(prior_candidate, dtype=float))) <= 0.025:
                        candidate_xy = xy
                        candidate_count = prior_count + 1
                    else:
                        candidate_xy = xy
                        candidate_count = 1
                    if candidate_count >= 3:
                        xy = np.asarray(candidate_xy, dtype=float)
                        candidate_xy = None
                        candidate_count = 0
                    else:
                        xy = previous
            else:
                seen_count = 1
            confidence = min(0.99, 0.70 + 0.06 * min(seen_count, 4))
            self.spatial_memory[color] = {
                "xy": [float(xy[0]), float(xy[1])],
                "z": 0.025,
                "confidence": confidence,
                "seen_count": seen_count,
                "last_seen_observation": obs_id,
                "visible": True,
                "relation": "GROUND",
                "source": "robot_camera_ray+fk+odometry",
                "candidate_xy": None if candidate_xy is None else [float(candidate_xy[0]), float(candidate_xy[1])],
                "candidate_count": candidate_count,
            }

    def _depth_obstacle_scan(self, rgb: np.ndarray | None = None, *, force: bool = False) -> None:
        """Fuse a sparse depth pass into a camera-derived 2.5D occupancy map.

        MuJoCo depth is used only as the SIM perception provider. The output
        contract is generic (occupied world cells with height/confidence), so REAL
        can later supply metric depth from Depth Anything/stereo/depth hardware.
        """
        if self.renderer is None:
            return
        # Keep control loops cheap: obstacle geometry changes much more slowly
        # than target tracking, so scan only every few semantic observations.
        if not force and self._memory_observation_id % self._obstacle_scan_interval != 0:
            return
        try:
            self.renderer.update_scene(self.data, camera="robot_cam")
            self.renderer.enable_depth_rendering()
            depth = self.renderer.render().copy()
        finally:
            try:
                self.renderer.disable_depth_rendering()
            except Exception:
                pass
        if depth.ndim != 2:
            return
        h, w = depth.shape
        cid = self._id(mujoco.mjtObj.mjOBJ_CAMERA, "robot_cam")
        cam_pos = self.data.cam_xpos[cid].copy()
        cam_rot = self.data.cam_xmat[cid].reshape(3, 3).copy()
        fovy = math.radians(float(self.model.cam_fovy[cid]))
        tan_y = math.tan(fovy / 2.0)
        tan_x = tan_y * (float(w) / max(1.0, float(h)))
        cell = 0.08
        obs_id = self._memory_observation_id
        seen_cells: dict[tuple[int,int], tuple[np.ndarray, float]] = {}
        # Sparse sampling is enough for a sonar/occupancy representation and
        # avoids flooding state/WebSocket with thousands of points.
        step = max(8, int(round(min(w, h) / 24)))
        for py in range(step//2, h, step):
            cy = (py + 0.5) / h
            for px in range(step//2, w, step):
                d = float(depth[py, px])
                if not math.isfinite(d) or d <= 0.10 or d > 1.65:
                    continue
                cx = (px + 0.5) / w
                ray = np.array([2.0*(cx-0.5)*tan_x, 2.0*(0.5-cy)*tan_y, -1.0], dtype=float)
                # MuJoCo depth is optical-axis depth, hence use the unnormalized
                # pinhole ray [x/z,y/z,-1] rather than Euclidean range.
                point = cam_pos + d * (cam_rot @ ray)
                rel = point[:2] - self.odom_xy
                planar = float(np.linalg.norm(rel))
                # Above-floor surfaces are occupied. Ignore near-camera/self
                # geometry and tiny block-height surfaces handled semantically.
                if planar < 0.20 or planar > 1.55 or float(point[2]) < 0.065 or float(point[2]) > 0.75:
                    continue
                key = (int(round(float(point[0])/cell)), int(round(float(point[1])/cell)))
                prev = seen_cells.get(key)
                if prev is None or float(point[2]) > prev[1]:
                    seen_cells[key] = (point, float(point[2]))
        for key, (point, height) in seen_cells.items():
            old = self.obstacle_cells.get(key, {})
            hits = min(20, int(old.get("hits", 0)) + 1)
            old_xy = np.asarray(old.get("xy") or point[:2], dtype=float)
            xy = .70*old_xy + .30*point[:2]
            self.obstacle_cells[key] = {
                "xy": [float(xy[0]), float(xy[1])],
                "height": max(float(old.get("height", 0.0))*0.85, height),
                "hits": hits,
                "last_seen_observation": obs_id,
                "source": "robot_camera_depth",
            }
        # Bound memory and let stale dynamic obstacles fade away.
        stale = [k for k,v in self.obstacle_cells.items() if obs_id-int(v.get("last_seen_observation",0)) > 90]
        for k in stale:
            self.obstacle_cells.pop(k, None)
        if len(self.obstacle_cells) > 160:
            keep = sorted(self.obstacle_cells.items(), key=lambda kv: (kv[1].get("last_seen_observation",0), kv[1].get("hits",0)), reverse=True)[:160]
            self.obstacle_cells = dict(keep)

    def semantic_map_public(self) -> dict:
        obs_id = self._memory_observation_id
        obstacles = []
        for entry in self.obstacle_cells.values():
            age = max(0, obs_id-int(entry.get("last_seen_observation",0)))
            hits = int(entry.get("hits",0))
            confidence = min(.98, .40 + .08*min(hits,7)) * max(.25, 1.0-age/100.0)
            obstacles.append({
                "position_xy": [round(float(entry["xy"][0]),3), round(float(entry["xy"][1]),3)],
                "size_m": .08,
                "height_m": round(float(entry.get("height",.1)),3),
                "confidence": round(float(confidence),3),
                "age_observations": age,
                "source": str(entry.get("source","robot_camera_depth")),
            })
        zones = []
        for name,z in DROP_ZONES.items():
            zones.append({"name":name, **z, "source":"configured_workcell_landmark"})
        arm_yaw = self._joint_qpos("arm_yaw")
        return {
            "obstacles": obstacles,
            "drop_zones": zones,
            "scan": {
                "heading_rad": round(float(self.base_yaw + arm_yaw),4),
                "head_yaw_rad": round(float(arm_yaw),4),
                "horizontal_fov_deg": 74.0,
                "range_m": 1.55,
                "observation_id": int(obs_id),
                "source": "robot_camera",
            },
        }

    def observe_scene(self, *, force_obstacles: bool = False) -> dict[str, dict]:
        """One camera observation updates object and obstacle spatial memory."""
        rgb = self.render_rgb("robot_cam")
        detections = self.scene_detections(rgb)
        self._update_spatial_memory(detections)
        self._depth_obstacle_scan(rgb, force=force_obstacles)
        return detections

    def spatial_memory_public(self) -> dict[str, dict]:
        out: dict[str, dict] = {}
        for color, entry in self.spatial_memory.items():
            xy = np.asarray(entry.get("xy") or [0.0, 0.0], dtype=float)
            delta = xy - self.odom_xy
            bearing = math.atan2(float(delta[1]), float(delta[0])) - self.base_yaw
            bearing = (bearing + math.pi) % (2.0*math.pi) - math.pi
            out[color] = {
                "position_xy": [round(float(xy[0]), 4), round(float(xy[1]), 4)],
                "height_m": round(float(entry.get("z", 0.025)), 4),
                "distance_m": round(float(np.linalg.norm(delta)), 3),
                "bearing_deg": round(math.degrees(bearing), 1),
                "confidence": round(float(entry.get("confidence", 0.0)), 3),
                "visible": bool(entry.get("visible", False)),
                "seen_count": int(entry.get("seen_count", 0)),
                "age_observations": max(0, self._memory_observation_id - int(entry.get("last_seen_observation", 0))),
                "relation": str(entry.get("relation", "GROUND")),
                "source": str(entry.get("source", "robot_camera")),
            }
        return out

    def _set_search_head_yaw(self, yaw: float, duration_steps: int = 70) -> None:
        """Move the eye-in-hand search head from measured joints, not stale ctrl targets."""
        target = SEARCH_POSE.copy()
        target[0] = float(np.clip(yaw, -1.35, 1.35))
        start = np.array([self._joint_qpos(n) for n in ARM_JOINTS], dtype=float)
        samples = self._fast_samples(max(10, int(duration_steps // 3)), minimum=10)
        emit_every = max(1, samples // 8)
        for i, t in enumerate(np.linspace(0.0, 1.0, samples)):
            u = t*t*(3.0 - 2.0*t)
            self._set_arm_target((1.0-u)*start + u*target)
            self.step_physics(self._fast_steps(5))
            if i % emit_every == 0 or i == samples-1:
                self._emit_frame()
        # Let the position servos settle close enough that the next visual
        # correction starts from the pose the camera actually occupies.
        self._set_arm_target(target)
        self.step_physics(18)

    @staticmethod
    def _tracking_blend_for_range(range_m: float) -> float:
        """Blend CARRY->SEARCH to keep a floor target vertically observable."""
        # Empirically/calibrationally from camera geometry: ~0.5 at 0.65 m,
        # ~0.65 at 0.55 m, ~0.8 at 0.45 m, and near SEARCH at 0.35 m.
        return float(np.clip(1.40 - 1.40*float(range_m), 0.48, 1.0))

    def _set_track_head_yaw(self, yaw: float, duration_steps: int = 55, *, blend: float = 0.5) -> None:
        """Distance-adaptive camera pose for visual tracking and approach."""
        a=float(np.clip(blend,0.0,1.0))
        target=(1.0-a)*CARRY_POSE+a*SEARCH_POSE
        target[0] = float(np.clip(yaw, -1.35, 1.35))
        start = np.array([self._joint_qpos(n) for n in ARM_JOINTS], dtype=float)
        samples = self._fast_samples(max(10, int(duration_steps // 3)), minimum=10)
        emit_every=max(1,samples//8)
        for i,t in enumerate(np.linspace(0.0,1.0,samples)):
            u=t*t*(3.0-2.0*t)
            self._set_arm_target((1.0-u)*start+u*target)
            self.step_physics(self._fast_steps(5))
            if i % emit_every == 0 or i == samples-1:
                self._emit_frame()
        self._set_arm_target(target)
        self.step_physics(18)

    def _set_survey_head_yaw(self, yaw: float, duration_steps: int = 70) -> None:
        """High camera pose for mapping obstacles and semantic landmarks."""
        target = CARRY_POSE.copy()
        target[0] = float(np.clip(yaw, -1.35, 1.35))
        start = np.array([self._joint_qpos(n) for n in ARM_JOINTS], dtype=float)
        samples = self._fast_samples(max(10, int(duration_steps // 3)), minimum=10)
        for t in np.linspace(0.0, 1.0, samples):
            u=t*t*(3.0-2.0*t)
            self._set_arm_target((1.0-u)*start+u*target)
            self.step_physics(self._fast_steps(5))
        self._set_arm_target(target)
        self.step_physics(18)

    def _look_at_memory(self, color: str) -> dict | None:
        entry = self.spatial_memory.get(color)
        if not entry or entry.get("xy") is None:
            return None
        xy = np.asarray(entry["xy"], dtype=float)
        delta = xy - self.odom_xy
        desired = math.atan2(float(delta[1]), float(delta[0])) - self.base_yaw
        desired = (desired + math.pi) % (2.0*math.pi) - math.pi
        if abs(desired) > 1.35:
            return None
        self._set_search_head_yaw(desired, 75)
        return self.observe_scene().get(color)

    def _look_at_memory_track(self, color: str) -> dict | None:
        entry=self.spatial_memory.get(color)
        if not entry or entry.get("xy") is None:
            return None
        xy=np.asarray(entry["xy"],dtype=float); delta=xy-self.odom_xy
        desired=math.atan2(float(delta[1]),float(delta[0]))-self.base_yaw
        desired=(desired+math.pi)%(2.0*math.pi)-math.pi
        if abs(desired)>1.35:
            return None
        blend=self._tracking_blend_for_range(float(np.linalg.norm(delta)))
        self._set_track_head_yaw(desired,58,blend=blend)
        return self.observe_scene().get(color)

    def _head_scan_for(self, color: str, *, allow_base_fallback: bool = True) -> dict:
        """2D head search over yaw and camera pitch before any chassis turn."""
        det = self._look_at_memory_track(color) if color == "red" else self._look_at_memory(color)
        if det and det.get("visible"):
            return det
        yaws=(-1.20,-0.80,-0.40,0.0,0.40,0.80,1.20)
        # Mid pitch has large vertical margin for 0.5-0.8 m floor targets and is
        # tolerant to realistic servo settling error. A near-search pitch covers
        # the closer end of the workcell.
        for blend in (0.65,0.93):
            for yaw in yaws:
                self._set_track_head_yaw(yaw,58,blend=blend)
                det=self.observe_scene().get(color,{"visible":False})
                if det.get("visible"):
                    return det
        if not allow_base_fallback:
            return {"visible":False,"pixels":0}
        # Only after yaw x pitch head coverage is exhausted expose the rear side.
        self._set_track_head_yaw(0.0,35,blend=0.70)
        self.move_base_relative(yaw_delta=math.radians(155.0),frames=10,max_w=0.48,a_ang=1.6)
        for blend in (0.65,0.93):
            for yaw in (-0.85,-0.40,0.0,0.40,0.85):
                self._set_track_head_yaw(yaw,58,blend=blend)
                det=self.observe_scene().get(color,{"visible":False})
                if det.get("visible"):
                    return det
        return {"visible":False,"pixels":0}

    def _survey_scan_for(self, color: str, *, allow_base_fallback: bool = True) -> dict:
        """Map a floor landmark with vertical coverage before moving the base."""
        yaws=(-1.20,-0.80,-0.40,0.0,0.40,0.80,1.20)
        for yaw in yaws:
            self._set_survey_head_yaw(yaw,58)
            det=self.observe_scene().get(color,{"visible":False})
            if det.get("visible") and color in self.spatial_memory:
                return det
        # A close landmark may sit too low in the pure survey view. Mid-pitch
        # moves it away from the lower quality gate while preserving clearance.
        for yaw in yaws:
            self._set_track_head_yaw(yaw,58,blend=0.65)
            det=self.observe_scene().get(color,{"visible":False})
            if det.get("visible") and color in self.spatial_memory:
                return det
        if not allow_base_fallback:
            return {"visible":False,"pixels":0}
        self._set_track_head_yaw(0.0,35,blend=0.65)
        self.move_base_relative(yaw_delta=math.radians(155.0),frames=10,max_w=0.48,a_ang=1.6)
        for yaw in (-0.85,-0.40,0.0,0.40,0.85):
            self._set_track_head_yaw(yaw,58,blend=0.65)
            det=self.observe_scene().get(color,{"visible":False})
            if det.get("visible") and color in self.spatial_memory:
                return det
        return {"visible":False,"pixels":0}

    def _transfer_head_yaw_to_chassis(self, head_yaw: float, color: str = "red") -> dict:
        """Transfer eye-in-hand yaw to chassis while preserving the selected target."""
        total=float(head_yaw)
        if abs(total) < math.radians(1.0):
            return self.observe_scene().get(color, {"visible": False})
        entry=self.spatial_memory.get(color) or {}
        if entry.get("xy") is not None:
            rng=float(np.linalg.norm(np.asarray(entry["xy"],dtype=float)-self.odom_xy))
            blend=self._tracking_blend_for_range(rng)
        else:
            blend=0.5
        n=max(3, int(math.ceil(abs(total)/math.radians(3.0))))
        step=total/n
        for i in range(n):
            remaining=total-step*(i+1)
            self._set_track_head_yaw(remaining, 28, blend=blend)
            self.move_base_relative(yaw_delta=step, frames=4, max_w=0.42, a_ang=1.4)
        self._set_track_head_yaw(0.0, 30, blend=blend)
        return self.observe_scene().get(color, {"visible": False})

    def camera_yellow_detection(self) -> dict:
        """Detect the yellow object from current robot-camera RGB."""
        return self.scene_detections().get("yellow", {"visible": False, "pixels": 0})
    def camera_blue_detection(self) -> dict:
        """Detect the blue object from current robot-camera RGB."""
        return self.scene_detections().get("blue", {"visible": False, "pixels": 0})
    def _camera_pixel_ground_xy(
        self, det: dict, *, camera: str = "robot_cam", plane_z: float = 0.025
    ) -> np.ndarray | None:
        """Project a detected pixel through calibrated camera intrinsics onto a ground plane.

        This uses only robot kinematics, camera calibration, and image pixels. It does
        not read the target body's simulator position, so the same geometry transfers
        to the physical robot when its FK supplies the camera pose.
        """
        if not det.get("visible"):
            return None
        cid = self._id(mujoco.mjtObj.mjOBJ_CAMERA, camera)
        cam_pos = self.data.cam_xpos[cid].copy()
        cam_rot = self.data.cam_xmat[cid].reshape(3, 3).copy()
        fovy = math.radians(float(self.model.cam_fovy[cid]))
        tan_y = math.tan(fovy / 2.0)
        tan_x = tan_y * (float(self.width) / max(1.0, float(self.height)))
        cx = float(det.get("cx", 0.5))
        cy = float(det.get("cy", 0.5))
        ray_local = np.array([
            2.0 * (cx - 0.5) * tan_x,
            2.0 * (0.5 - cy) * tan_y,
            -1.0,
        ], dtype=float)
        ray_local /= max(1e-9, float(np.linalg.norm(ray_local)))
        ray_world = cam_rot @ ray_local
        if float(ray_world[2]) >= -1e-5:
            return None
        distance = (float(plane_z) - float(cam_pos[2])) / float(ray_world[2])
        if distance <= 0.0:
            return None
        return (cam_pos + distance * ray_world)[:2]

    def _center_visible_yellow(self, max_iters: int = 6) -> tuple[bool, dict]:
        det = self.camera_yellow_detection()
        if not det.get("visible"):
            return False, det
        for _ in range(max_iters):
            err = 0.5 - float(det["cx"])
            if abs(err) < 0.006:
                return True, det
            correction = float(np.clip(err * math.radians(65.0), -math.radians(10), math.radians(10)))
            self.move_base_relative(yaw_delta=correction, frames=4)
            det = self.camera_yellow_detection()
            if not det.get("visible"):
                return False, det
        return bool(det.get("visible")), det

    def camera_detection(self, color: str) -> dict:
        return self.scene_detections().get(color, {"visible": False, "pixels": 0})

    def camera_red_detection(self) -> dict:
        return self.camera_detection("red")

    def camera_yellow_detection(self) -> dict:
        return self.camera_detection("yellow")

    def camera_blue_detection(self) -> dict:
        return self.camera_detection("blue")

    def _center_visible_target(self, color: str = "red", max_iters: int = 5, det: dict | None = None) -> tuple[bool, dict]:
        det = self.camera_detection(color) if det is None else det
        if not det.get("visible"):
            return False, det
        for _ in range(max_iters):
            err = 0.5 - float(det["cx"])
            if abs(err) < 0.012:
                return True, det
            correction = float(np.clip(err * math.radians(65.0), -math.radians(10), math.radians(10)))
            self.move_base_relative(yaw_delta=correction, frames=4)
            det = self.camera_detection(color)
            if not det.get("visible"):
                return False, det
        return bool(det.get("visible")), det

    def set_arm_pose(self, pose: Iterable[float], duration_steps: int = 180) -> None:
        start = self._arm_targets()
        target = np.asarray(tuple(pose), dtype=float)
        nominal_samples = max(2, int(duration_steps // 4))
        samples = self._fast_samples(nominal_samples)
        emit_every = max(1, samples // 14)
        for i, t in enumerate(np.linspace(0.0, 1.0, samples)):
            u = t*t*(3.0 - 2.0*t)
            self._set_arm_target((1.0-u)*start + u*target)
            self.step_physics(self._fast_steps(4))
            if i % emit_every == 0 or i == samples-1:
                self._emit_frame()

    def state(self) -> dict:
        red = self._body_pos("red_block")
        blue = self._body_pos("blue_block")
        yellow = self._body_pos("yellow_block")
        color = self.grasp_color or "red"
        block = {"red": red, "blue": blue, "yellow": yellow}[color]
        grip = self._site_pos("grip_site")
        cs = self.contact_state(color) if self.grasp_color is not None else ContactState(False, False, 0.0, 0.0)
        return {
            "seed": int(self.seed),
            "robot_xy": [round(float(x), 4) for x in self._body_pos("robot")[:2]],
            "base_yaw": round(self._joint_qpos("base_yaw"), 4),
            "red_xyz": [round(float(x), 4) for x in red],
            "blue_xyz": [round(float(x), 4) for x in blue],
            "yellow_xyz": [round(float(x), 4) for x in yellow],
            "grasp_color": self.grasp_color,
            "grip_xyz": [round(float(x), 4) for x in grip],
            "grip_error_m": round(float(np.linalg.norm(block - grip)), 4),
            "left_contact": cs.left,
            "right_contact": cs.right,
            "left_normal_N": round(cs.left_force, 3),
            "right_normal_N": round(cs.right_force, 3),
            "bilateral_contact": cs.bilateral,
            "lifted": bool(self.grasp_color is not None and block[2] > .055),
            "stable": bool(self.grasp_color is not None and self._stable_steps >= 40),
            "stable_steps": int(self._stable_steps),
            "gripper_qpos": round(self._joint_qpos("gripper_open"), 4),
            "arm_qpos": [round(self._joint_qpos(n), 4) for n in ARM_JOINTS],
            "spatial_memory": self.spatial_memory_public(),
            "semantic_map": self.semantic_map_public(),
            "memory_observation_id": int(self._memory_observation_id),
            "arm_dof": 4,
            "gripper_dof": 1,
        }

    def act(self, action: str, *, seed: int | None = None) -> ActionResult:
        action = str(action).strip().lower()
        if action == "reset":
            st = self.reset(seed)
            self._emit_frame()
            return ActionResult(True, action, "MasterPi camera-only physics world reset", st)

        if action == "scan_world":
            # Deliberate two-pass semantic scan while the chassis stays fixed.
            # 1) High survey pose: sparse depth occupancy for furniture/walls.
            # 2) Low object pose: 50 mm colored blocks, separated from same-color
            #    delivery pads/crates by geometry-aware component selection.
            start_yaw = self.base_yaw
            seen: set[str] = set()
            # Hints preserve edge sightings that the spatial quality gate correctly
            # refuses to localize. After the coarse sweep we actively re-center
            # those objects instead of silently forgetting them.
            edge_hints: dict[str, tuple[float, float, bool]] = {}
            survey_yaws = (-1.20, -0.80, -0.40, 0.0, 0.40, 0.80, 1.20)
            object_yaws = (-1.20, -0.75, -0.35, 0.0, 0.35, 0.75, 1.20)
            def remember_hints(dets: dict[str, dict], yaw: float, survey: bool) -> None:
                for color, det in dets.items():
                    if not det.get("visible"):
                        continue
                    seen.add(color)
                    cx=float(det.get("cx", .5))
                    # Keep the sighting closest to image center; it needs the
                    # smallest corrective head movement and is least distorted.
                    old=edge_hints.get(color)
                    if old is None or abs(cx-.5) < abs(old[1]-.5):
                        edge_hints[color]=(float(yaw),cx,bool(survey))
            for yaw in survey_yaws:
                self._set_survey_head_yaw(yaw, 62)
                dets = self.observe_scene(force_obstacles=True)
                remember_hints(dets,yaw,True)
            for yaw in object_yaws:
                self._set_track_head_yaw(yaw, 62, blend=0.68)
                dets = self.observe_scene(force_obstacles=False)
                remember_hints(dets,yaw,False)
            # Active-perception refinement for objects seen only at a low-quality
            # edge/horizon pixel. Never rotate the chassis for this refinement.
            for color in ("red", "blue", "yellow"):
                if color in self.spatial_memory or color not in edge_hints:
                    continue
                yaw,cx,survey=edge_hints[color]
                desired=float(np.clip(yaw + (0.5-cx)*math.radians(65.0), -1.32, 1.32))
                for _ in range(2):
                    if survey:
                        self._set_survey_head_yaw(desired,48)
                    else:
                        self._set_track_head_yaw(desired,48,blend=0.68)
                    dets=self.observe_scene(force_obstacles=False)
                    det=dets.get(color,{"visible":False})
                    if color in self.spatial_memory:
                        break
                    if not det.get("visible"):
                        break
                    desired=float(np.clip(desired + (0.5-float(det.get("cx",.5)))*math.radians(65.0), -1.32, 1.32))
            self._set_search_head_yaw(0.0, 45)
            self.observe_scene(force_obstacles=False)
            base_delta = (self.base_yaw-start_yaw+math.pi)%(2*math.pi)-math.pi
            return ActionResult(
                True, action,
                f"head-only semantic scan completed; objects={sorted(seen)}, obstacles={len(self.obstacle_cells)}, chassis_delta_deg={math.degrees(base_delta):.2f}",
                self.state(),
            )

        if action in {"map_red", "map_yellow", "map_blue"}:
            color = action.removeprefix("map_")
            # Mapping a placement landmark uses the high survey camera first.
            # The low grasp/search pose can completely miss distant floor blocks.
            det = self.observe_scene().get(color, {"visible": False})
            entry = self.spatial_memory.get(color)
            if not entry or entry.get("xy") is None:
                det = self._survey_scan_for(color, allow_base_fallback=True)
            entry = self.spatial_memory.get(color)
            if not entry or entry.get("xy") is None:
                return ActionResult(False, action, f"{color} placement target was not localized by visual spatial memory", self.state())
            mapped = np.asarray(entry["xy"], dtype=float)
            if color == "yellow": self.yellow_map_xy = mapped.copy()
            elif color == "blue": self.blue_map_xy = mapped.copy()
            return ActionResult(
                True, action,
                f"{color} target recalled/localized by spatial memory at odom ({mapped[0]:.2f}, {mapped[1]:.2f})",
                self.state(),
            )

        if action in {"search", "search_red", "search_blue", "search_yellow"}:
            color = "red" if action in {"search", "search_red"} else action.removeprefix("search_")
            # Current view -> remembered bearing -> head sweep -> chassis fallback.
            det = self.observe_scene().get(color, {"visible": False})
            if not det.get("visible") and color == "red" and color in self.spatial_memory:
                det = self._look_at_memory_track(color) or {"visible": False}
            if not det.get("visible"):
                det = self._head_scan_for(color, allow_base_fallback=True)
            if not det.get("visible"):
                # Backing up creates camera clearance; do it only after memory +
                # full head sweep fail. Do not rotate the chassis merely because
                # the fixed low camera pose could not see a remembered target.
                self._set_search_head_yaw(0.0, 40)
                self.move_base_relative(forward=-0.10, frames=7)
                det = self._head_scan_for(color, allow_base_fallback=False)
            if not det.get("visible"):
                return ActionResult(False, action, f"{color} target not found by spatial-memory/head-first visual search", self.state())
            return ActionResult(True, action, f"{color} target found/recalled with head-first spatial search", self.state())

        if action in {"track", "track_red", "track_blue", "track_yellow"}:
            color = "red" if action == "track" else action.removeprefix("track_")
            det = self.observe_scene().get(color, {"visible": False})
            if not det.get("visible"):
                det = self._look_at_memory_track(color) or {"visible": False}
            if not det.get("visible"):
                return ActionResult(False, action, f"{color} target is not visible and could not be recalled from spatial memory", self.state())
            # Whatever pose search/survey ended in, enter the dedicated tracking
            # pose while preserving the measured horizontal bearing.
            current_yaw=self._joint_qpos("arm_yaw")
            projected=self._camera_pixel_ground_xy(det,plane_z=0.025)
            if projected is not None:
                initial_range=float(np.linalg.norm(np.asarray(projected,dtype=float)-self.odom_xy))
                blend=self._tracking_blend_for_range(initial_range)
            else:
                blend=0.5
            self._set_track_head_yaw(current_yaw,45,blend=blend)
            det=self.observe_scene().get(color,{"visible":False})
            if not det.get("visible"):
                det=self._look_at_memory_track(color) or {"visible":False}
            for _ in range(7):
                if not det.get("visible"):
                    break
                err = 0.5 - float(det.get("cx", 0.5))
                if abs(err) < 0.018:
                    self.observe_scene()
                    return ActionResult(True, action, f"tracking pose centered the visible {color} target", self.state())
                current_yaw = self._joint_qpos("arm_yaw")
                desired = float(np.clip(current_yaw + err * math.radians(65.0), -1.35, 1.35))
                projected=self._camera_pixel_ground_xy(det,plane_z=0.025)
                rng=float(np.linalg.norm(np.asarray(projected)-self.odom_xy)) if projected is not None else 0.60
                self._set_track_head_yaw(desired, 38, blend=self._tracking_blend_for_range(rng))
                det = self.observe_scene().get(color, {"visible": False})
            return ActionResult(False, action, f"tracking pose lost the {color} target", self.state())

        if action in {"approach", "approach_red", "approach_blue", "approach_yellow"}:
            color = "red" if action == "approach" else action.removeprefix("approach_")
            det = self.observe_scene().get(color, {"visible": False})
            if not det.get("visible"):
                det = self._look_at_memory_track(color) or {"visible": False}
            if not det.get("visible"):
                return ActionResult(False, action, f"{color} target is not visible and is absent from usable spatial memory", self.state())
            # Tracking is head-only. Transfer that bearing into chassis yaw in
            # counter-rotated increments so the camera keeps looking at the same
            # world direction instead of snapping away from the target.
            head_yaw = self._joint_qpos("arm_yaw")
            if abs(head_yaw) > math.radians(2.0):
                det = self._transfer_head_yaw_to_chassis(head_yaw, color)
                if not det.get("visible"):
                    # Spatial memory is allowed to reacquire the target; do not
                    # fail merely because one transition frame clipped the cube.
                    det = self._look_at_memory_track(color) or {"visible": False}
            centered, det = self._center_visible_target(color, det=det)
            if not centered:
                return ActionResult(False, action, f"could not keep the {color} target visible while aligning chassis for approach", self.state())
            # Visual servo: use metric ray-ground range, not image cy. Image height
            # changes dramatically with camera pitch, so a cy threshold is not a
            # transferable distance criterion. Keep roughly 31 cm before pick(),
            # which then performs the final FK-calibrated creep to gripper reach.
            approach_step = {1.0: 0.03, 2.0: 0.045, 3.0: 0.06}[self.speed_multiplier]
            pregrasp_range = float(os.environ.get("UGRP_PREGRASP_RANGE", "0.350"))
            approach_iters = int(math.ceil(0.72 / approach_step))
            for _ in range(approach_iters):
                det = self.camera_detection(color)
                if not det.get("visible"):
                    det = self._look_at_memory_track(color) or {"visible": False}
                if not det.get("visible"):
                    return ActionResult(False, action, f"lost the {color} target during visual approach", self.state())
                centered, det = self._center_visible_target(color, max_iters=5, det=det)
                if not centered:
                    return ActionResult(False, action, f"lost the {color} target during visual approach", self.state())
                target_xy = self._camera_pixel_ground_xy(det, plane_z=0.025)
                if target_xy is None:
                    return ActionResult(False, action, f"visual approach could not project the centered {color} target onto the floor", self.state())
                heading=np.array([math.cos(self.base_yaw),math.sin(self.base_yaw)],dtype=float)
                delta=np.asarray(target_xy,dtype=float)-self.odom_xy
                forward_range=float(np.dot(delta,heading))
                lateral=float(heading[0]*delta[1]-heading[1]*delta[0])
                if abs(lateral)>0.022:
                    continue
                remaining=max(0.0,forward_range-pregrasp_range)
                if remaining<=0.006:
                    return ActionResult(True, action, f"metric visual approach reached pre-grasp range {forward_range:.3f} m", self.state())
                travel=min(approach_step,remaining)
                # Tilt toward SEARCH before getting closer so the target moves up
                # rather than falling out of the bottom of the camera image.
                predicted=max(pregrasp_range,forward_range-travel)
                current_head=self._joint_qpos("arm_yaw")
                self._set_track_head_yaw(current_head,28,blend=self._tracking_blend_for_range(predicted))
                self.move_base_relative(forward=travel, frames=4)
            return ActionResult(False, action, "visual approach exhausted its metric range budget before reaching pre-grasp distance", self.state())

        if action in {"pick", "pick_red", "pick_blue", "pick_yellow"}:
            color = "red" if action == "pick" else action.removeprefix("pick_")
            det = self.camera_detection(color)
            if not det.get("visible"):
                return ActionResult(False, action, f"{color} target is not visible in the robot camera; approach first", self.state())
            centered, det = self._center_visible_target(color, max_iters=8, det=det)
            if not centered:
                return ActionResult(False, action, "could not center the target before grasp", self.state())
            # Guarded geometric final approach, adapted from the real-robot
            # implementation: stop, take several settled measurements, reject
            # unstable/lateral observations, then project the target pixel ray
            # onto the floor using calibrated camera FK. This replaces the old
            # empirical image-cy -> fixed creep formula.
            sample_count = max(1, int(os.environ.get("UGRP_GRASP_RAY_SAMPLES", "3")))
            target_samples = []
            cx_samples = []
            for _ in range(sample_count):
                sample = self.camera_detection(color)
                if not sample.get("visible"):
                    return ActionResult(False, action, "target disappeared during settled pre-grasp measurement", self.state())
                xy = self._camera_pixel_ground_xy(sample, plane_z=0.025)
                if xy is None:
                    return ActionResult(False, action, "pre-grasp pixel ray did not intersect the floor", self.state())
                target_samples.append(np.asarray(xy, dtype=float))
                cx_samples.append(float(sample.get("cx", 0.5)))
            targets = np.asarray(target_samples)
            target_xy = np.median(targets, axis=0)
            radial_mad = float(np.median(np.linalg.norm(targets - target_xy, axis=1))) if len(targets) > 1 else 0.0
            if radial_mad > 0.012:
                return ActionResult(False, action, "pre-grasp target estimate was unstable across camera frames", self.state())
            heading = np.array([math.cos(self.base_yaw), math.sin(self.base_yaw)], dtype=float)
            delta = target_xy - self.odom_xy
            forward_range = float(np.dot(delta, heading))
            lateral = float(heading[0] * delta[1] - heading[1] * delta[0])
            # Ground-pose grip-site reach measured from robot FK. A few mm of
            # residual compression is supplied by closing the fingers, not by
            # ramming the chassis forward.
            desired_reach = float(os.environ.get("UGRP_GROUND_GRIP_REACH", "0.1572"))
            final_creep = forward_range - desired_reach
            if abs(lateral) > 0.018:
                return ActionResult(False, action, "pre-grasp target remained laterally outside the gripper corridor", self.state())
            if final_creep < -0.012 or final_creep > 0.205:
                return ActionResult(False, action, "geometry-based final grasp travel was outside the safe range", self.state())
            if final_creep > 0.002:
                self.move_base_relative(forward=final_creep, frames=12)
            # Only now commit target identity: all camera/geometry preconditions
            # have passed and the physical grasp sequence is about to begin.
            self.grasp_color = color
            self._stable_steps = 0
            self.open_gripper()
            self.step_physics(20)
            self._emit_frame()
            # Friend-code style hover->grasp descent. The two FK-calibrated
            # endpoints keep nearly the same planar grip-site reach, avoiding
            # the large SEARCH->GROUND sweep that can shove a corner of a cube.
            self.set_arm_pose(GRASP_HOVER_POSE, 110)
            self.set_arm_pose(GROUND_POSE, 110)
            close_samples = self._fast_samples(24, minimum=8)
            for i, c in enumerate(np.linspace(0.0, 1.0, close_samples)):
                self.close_gripper(float(c))
                self.step_physics(self._fast_steps(10))
                if i % max(1, close_samples // 12) == 0 or i == close_samples - 1:
                    self._emit_frame()
            if os.environ.get("UGRP_PICK_REQUIRE_BILATERAL", "1") == "1":
                self.step_physics(24)
                if not self.contact_state().bilateral:
                    # Friend implementation's guarded-recovery principle: do not
                    # carry a bad grasp forward. Release, create visual clearance,
                    # and let the executive reacquire from a known-safe state.
                    self.open_gripper()
                    self.grasp_color = None
                    self.step_physics(20)
                    self.set_arm_pose(SEARCH_POSE, 80)
                    self.move_base_relative(forward=-0.07, frames=6)
                    return ActionResult(False, action, "grasp postcondition failed: bilateral finger contact was not acquired", self.state())
            # Execution success means the motor sequence completed. In the
            # default transferable mode, external sensing still owns grasp proof.
            return ActionResult(True, action, "grasp motion completed; verify the result from the robot camera", self.state())

        if action == "carry":
            color = self.grasp_color
            if color not in {"red", "blue", "yellow"}:
                return ActionResult(False, action, "no selected grasp target to carry", self.state())
            # Two-stage lift: first raise the wrist with very little rotation, then
            # fold into the carry pose. The old single arc applied a large torque
            # to a friction-only cube immediately after closing the jaws.
            for target, nominal in ((LIFT_POSE, 90), (CARRY_POSE, 110)):
                start = self._arm_targets()
                samples = self._fast_samples(nominal, minimum=20)
                for i, t in enumerate(np.linspace(0.0, 1.0, samples)):
                    u = t*t*(3.0 - 2.0*t)
                    self._set_arm_target((1.0-u)*start + u*target)
                    self.close_gripper(1.0)
                    self.step_physics(self._fast_steps(4))
                    if i % max(1, samples // 12) == 0 or i == samples - 1:
                        self._emit_frame()
                self.step_physics(20)
                if not self.contact_state().bilateral:
                    # Finish the commanded motion only if the physical grasp is
                    # still present; otherwise report a failed carry immediately.
                    self.open_gripper()
                    self.step_physics(12)
                    # Create camera clearance before the executive starts its
                    # reacquisition sweep. A slipped cube is often directly
                    # below/behind the eye-in-hand camera at this point.
                    self.set_arm_pose(SEARCH_POSE, 70)
                    self.move_base_relative(forward=-0.10, frames=8)
                    self.grasp_color = None
                    return ActionResult(False, action, f"{color} block slipped from the gripper during lift", self.state())
            self.step_physics(30)
            # The robot knows the object moved with its gripper because the carry
            # postcondition verified a bilateral grasp. Update memory from gripper
            # FK rather than leaving the landmark at its old floor position.
            grip = self._site_pos("grip_site")
            old = self.spatial_memory.get(color) or {}
            self.spatial_memory[color] = {
                **old,
                "xy": [float(grip[0]), float(grip[1])],
                "z": float(grip[2]),
                "confidence": max(0.95, float(old.get("confidence", 0.0))),
                "visible": False,
                "relation": "HELD",
                "last_seen_observation": self._memory_observation_id,
                "source": "gripper_fk+verified_grasp",
            }
            self._emit_frame()
            return ActionResult(True, action, "lift motion completed with bilateral finger contact", self.state())

        if action in {"place_on_red", "place_on_yellow", "place_on_blue"}:
            target_color = action.removeprefix("place_on_")
            held_color = self.grasp_color
            if held_color not in {"red", "blue", "yellow"}:
                return ActionResult(False, action, "no verified held block for placement", self.state())
            if held_color == target_color:
                return ActionResult(False, action, "held block and placement target cannot be the same object", self.state())
            remembered = self.spatial_memory.get(target_color)
            fallback = self.yellow_map_xy if target_color == "yellow" else self.blue_map_xy if target_color == "blue" else None
            target_map = np.asarray(remembered["xy"], dtype=float) if remembered and remembered.get("xy") is not None else fallback
            if target_map is None:
                return ActionResult(False, action, f"{target_color} target has not been mapped; map_{target_color} first", self.state())
            self.set_arm_pose(CARRY_POSE, 100)
            self.close_gripper(1.0)
            vec = target_map - self.odom_xy
            dist = float(np.linalg.norm(vec))
            target_yaw = self.base_yaw if dist < 1e-4 else math.atan2(float(vec[1]), float(vec[0]))
            yaw_delta = (target_yaw - self.base_yaw + math.pi) % (2.0 * math.pi) - math.pi
            gentle = os.environ.get("UGRP_GENTLE_PLACE_TRANSPORT", "1") == "1"
            if gentle:
                self.move_base_relative(yaw_delta=yaw_delta, frames=18, max_w=0.30, a_ang=0.70)
                self.close_gripper(1.0)
                if not self.contact_state(held_color).bilateral:
                    return ActionResult(False, action, "carried block slipped during target-facing rotation", self.state())
            else:
                self.move_base_relative(yaw_delta=yaw_delta, frames=10)
            forward = max(0.0, dist - 0.238)
            if forward > 1e-6:
                if gentle:
                    self.move_base_relative(forward=forward, frames=30, max_v=0.085, a_lin=0.34)
                    self.close_gripper(1.0)
                    if not self.contact_state(held_color).bilateral:
                        return ActionResult(False, action, "carried block slipped during target approach", self.state())
                else:
                    self.move_base_relative(forward=forward, frames=24)
                    self.close_gripper(1.0)
            self.set_arm_pose(STACK_POSE, 240)
            self.step_physics(50)
            for i, c in enumerate(np.linspace(1.0, 0.0, 10)):
                self.close_gripper(float(c)); self.step_physics(20)
                if i % 2 == 0 or i == 9: self._emit_frame()
            self.open_gripper(); self.step_physics(160)
            self.set_arm_pose(CARRY_POSE, 180); self._emit_frame()
            held = self._body_pos(f"{held_color}_block")
            target = self._body_pos(f"{target_color}_block")
            planar_error = float(np.linalg.norm(held[:2] - target[:2]))
            vertical_gap = float(held[2] - target[2])
            placed = planar_error <= 0.040 and 0.040 <= vertical_gap <= 0.065
            if placed:
                old = self.spatial_memory.get(held_color) or {}
                self.spatial_memory[held_color] = {
                    **old,
                    "xy": [float(target_map[0]), float(target_map[1])],
                    "z": 0.075,
                    "confidence": 0.97,
                    "visible": False,
                    "relation": f"ON_{target_color.upper()}",
                    "last_seen_observation": self._memory_observation_id,
                    "source": "verified_place_postcondition+target_memory",
                }
                self.grasp_color = None
            return ActionResult(
                placed, action,
                f"{held_color} placement verified on the {target_color} target" if placed else "placement motion completed but stack postcondition was not achieved",
                self.state(),
            )

        return ActionResult(False, action, "unknown action", self.state())

    def render_jpeg(self, camera: str = "robot_cam", quality: int = 85) -> bytes:
        if self.renderer is None:
            raise RuntimeError("world was created with render=False")
        rgb = self.render_rgb(camera)
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

    def render_overview_jpeg(self, quality: int = 88) -> bytes:
        return self.render_jpeg(camera="cctv_front_left", quality=quality)

    def render_observer_jpegs(self, quality: int = 82) -> dict[str, bytes]:
        return {
            name: self.render_jpeg(camera=name, quality=quality)
            for name in ("cctv_front_left", "cctv_front_right", "cctv_rear_left", "cctv_top")
        }
