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
CARRY_POSE = np.array([0.0, -0.28, 0.42, -0.15], dtype=float)
LIFT_POSE = np.array([0.0, 0.10, 0.72, 0.10], dtype=float)
SEARCH_POSE = np.array([0.0, 0.0, 0.50, 0.20], dtype=float)
STACK_POSE = np.array([0.0, 0.20, 0.60, 0.05], dtype=float)


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
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(XML)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        self.width, self.height = width, height
        self._base_width, self._base_height = int(width), int(height)
        self.base_yaw = 0.0
        self.odom_xy = np.zeros(2, dtype=float)
        self.yellow_map_xy: np.ndarray | None = None
        self._stable_steps = 0
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
        if seed is not None:
            self.rng = np.random.default_rng(seed)
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
        for name, value in zip(ARM_JOINTS, CARRY_POSE):
            self._set_joint_qpos(name, float(value))
        self._set_joint_qpos("gripper_open", 0.0)
        self._set_arm_target(CARRY_POSE)
        self.open_gripper()
        self._stable_steps = 0
        mujoco.mj_forward(self.model, self.data)
        self.step_physics(80)
        return self.state()

    def contact_state(self) -> ContactState:
        block_gid = self._id(mujoco.mjtObj.mjOBJ_GEOM, "red_block_geom")
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
                left = True
                lf += normal
            if other == right_gid:
                right = True
                rf += normal
        return ContactState(left, right, lf, rf)

    def step_physics(self, n: int = 1) -> None:
        for _ in range(max(1, int(n))):
            mujoco.mj_step(self.model, self.data)
            cs = self.contact_state()
            block_z = float(self._body_pos("red_block")[2])
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

    def move_base_relative(self, forward: float = 0.0, yaw_delta: float = 0.0, frames: int = 8) -> None:
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
        max_v = 0.12
        max_w = 0.65
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
        a_lin = 0.75
        a_ang = 3.0
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

    def camera_yellow_detection(self) -> dict:
        """Detect a compact yellow cube/target from robot-camera RGB only."""
        rgb = self.render_rgb("robot_cam")
        r = rgb[..., 0].astype(np.int16)
        g = rgb[..., 1].astype(np.int16)
        b = rgb[..., 2].astype(np.int16)
        # Yellow has much more green than MasterPi orange accents.
        mask = (r > 100) & (g > 65) & (g > 0.50*r) & (g < 0.90*r) & (b < 85) & ((g-b) > 45)
        min_pixels = max(16, int(self.width*self.height*0.00045))
        comp = self._compact_component(mask, min_pixels=min_pixels)
        if comp is None:
            return {"visible": False, "pixels": 0}
        ys, xs = comp
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        return {
            "visible": True,
            "cx": round(float(xs.mean()) / self.width, 4),
            "cy": round(float(ys.mean()) / self.height, 4),
            "bbox": [round(x0/self.width,4), round(y0/self.height,4), round(x1/self.width,4), round(y1/self.height,4)],
            "area_ratio": round(float(xs.size)/(self.width*self.height),5),
            "pixels": int(xs.size),
        }

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

    def camera_red_detection(self) -> dict:
        """Detect the red cube from robot-camera RGB only; no MuJoCo object pose is used."""
        rgb = self.render_rgb("robot_cam")
        r = rgb[..., 0].astype(np.int16)
        g = rgb[..., 1].astype(np.int16)
        b = rgb[..., 2].astype(np.int16)
        # The cube is deep red while MasterPi accents are orange.  Requiring a
        # strong red/green ratio rejects the robot's own orange arm/fingers.
        mask = (r > 50) & (r > 4*g) & (r > 4*b) & ((r-g) > 30)
        ys, xs = np.where(mask)
        # Reject tiny red/orange self-reflections from the gripper. Keep this
        # threshold aligned with harness/perception.py so executive and skill
        # agree about whether a target actually exists.
        if xs.size < max(18, int(self.width*self.height*0.0008)):
            return {"visible": False, "pixels": int(xs.size)}
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        return {
            "visible": True,
            "cx": round(float(xs.mean()) / self.width, 4),
            "cy": round(float(ys.mean()) / self.height, 4),
            "bbox": [round(x0/self.width, 4), round(y0/self.height, 4), round(x1/self.width, 4), round(y1/self.height, 4)],
            "area_ratio": round(float(xs.size) / (self.width*self.height), 5),
            "pixels": int(xs.size),
        }

    def _center_visible_target(self, max_iters: int = 5) -> tuple[bool, dict]:
        det = self.camera_red_detection()
        if not det.get("visible"):
            return False, det
        for _ in range(max_iters):
            err = 0.5 - float(det["cx"])
            if abs(err) < 0.012:
                return True, det
            # Approximate horizontal FOV from the camera's 62-degree vertical FOV.
            # A tighter camera-center tolerance is important because the 5 cm cube
            # leaves little lateral margin between the parallel fingers.
            correction = float(np.clip(err * math.radians(65.0), -math.radians(10), math.radians(10)))
            self.move_base_relative(yaw_delta=correction, frames=4)
            det = self.camera_red_detection()
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
        block = self._body_pos("red_block")
        blue = self._body_pos("blue_block")
        yellow = self._body_pos("yellow_block")
        grip = self._site_pos("grip_site")
        cs = self.contact_state()
        return {
            "robot_xy": [round(float(x), 4) for x in self._body_pos("robot")[:2]],
            "base_yaw": round(self._joint_qpos("base_yaw"), 4),
            "red_xyz": [round(float(x), 4) for x in block],
            "blue_xyz": [round(float(x), 4) for x in blue],
            "yellow_xyz": [round(float(x), 4) for x in yellow],
            "grip_xyz": [round(float(x), 4) for x in grip],
            "grip_error_m": round(float(np.linalg.norm(block - grip)), 4),
            "left_contact": cs.left,
            "right_contact": cs.right,
            "left_normal_N": round(cs.left_force, 3),
            "right_normal_N": round(cs.right_force, 3),
            "bilateral_contact": cs.bilateral,
            "lifted": bool(block[2] > .055),
            "stable": bool(self._stable_steps >= 40),
            "stable_steps": int(self._stable_steps),
            "gripper_qpos": round(self._joint_qpos("gripper_open"), 4),
            "arm_qpos": [round(self._joint_qpos(n), 4) for n in ARM_JOINTS],
            "arm_dof": 4,
            "gripper_dof": 1,
        }

    def act(self, action: str) -> ActionResult:
        action = str(action).strip().lower()
        if action == "reset":
            st = self.reset()
            self._emit_frame()
            return ActionResult(True, action, "MasterPi camera-only physics world reset", st)

        if action == "map_yellow":
            # Prefer the current view: changing arm pose before mapping can push an
            # already-visible target to the image edge and throw away good geometry.
            det = self.camera_yellow_detection()
            if not det.get("visible"):
                self.set_arm_pose(SEARCH_POSE, 120)
                det = self.camera_yellow_detection()
            if not det.get("visible"):
                sweep_deg = {1.0: 20.0, 2.0: 30.0, 3.0: 40.0}[self.speed_multiplier]
                for _ in range(int(math.ceil(360.0 / sweep_deg))):
                    self.move_base_relative(yaw_delta=math.radians(sweep_deg), frames=3)
                    det = self.camera_yellow_detection()
                    if det.get("visible"):
                        break
            if not det.get("visible"):
                return ActionResult(False, action, "yellow placement target was not found by the robot camera", self.state())
            mapped = self._camera_pixel_ground_xy(det, plane_z=0.025)
            if mapped is None:
                return ActionResult(False, action, "yellow target pixel ray did not intersect the floor in front of the camera", self.state())
            self.yellow_map_xy = mapped.copy()
            return ActionResult(
                True, action,
                f"yellow target mapped from calibrated camera ray at odom ({mapped[0]:.2f}, {mapped[1]:.2f})",
                self.state(),
            )

        if action == "search":
            # Put the camera in a forward/down search pose, then sweep in place.
            self.set_arm_pose(SEARCH_POSE, 120)
            det = self.camera_red_detection()
            if not det.get("visible"):
                # At accelerated simulation speeds, scan the same 360-degree
                # field with fewer camera renders. The camera HFOV is wide
                # enough that 30/40-degree checkpoints retain overlap.
                sweep_deg = {1.0: 20.0, 2.0: 30.0, 3.0: 40.0}[self.speed_multiplier]
                sweep_count = int(math.ceil(360.0 / sweep_deg))
                for _ in range(sweep_count):
                    self.move_base_relative(yaw_delta=math.radians(sweep_deg), frames=3)
                    det = self.camera_red_detection()
                    if det.get("visible"):
                        break
            if not det.get("visible"):
                # A failed grasp can leave the cube directly below/behind the
                # eye-in-hand camera. Backing away is a transferable recovery:
                # regain field of view, then perform one more visual sweep.
                self.move_base_relative(forward=-0.12, frames=8)
                for _ in range(sweep_count):
                    self.move_base_relative(yaw_delta=math.radians(sweep_deg), frames=3)
                    det = self.camera_red_detection()
                    if det.get("visible"):
                        break
            if not det.get("visible"):
                return ActionResult(False, action, "red target not found after visual sweep and camera-backoff recovery", self.state())
            centered, det = self._center_visible_target()
            return ActionResult(bool(centered), action, "red target found from robot-camera pixels" if centered else "red target was seen but lost while centering", self.state())

        if action == "track":
            det = self.camera_red_detection()
            if not det.get("visible"):
                return ActionResult(False, action, "red target is not visible in the robot camera; search first", self.state())
            centered, det = self._center_visible_target()
            return ActionResult(bool(centered), action, "camera-centered the visible red target" if centered else "lost the red target while camera-centering", self.state())

        if action == "approach":
            det = self.camera_red_detection()
            if not det.get("visible"):
                return ActionResult(False, action, "red target is not visible in the robot camera; search first", self.state())
            centered, det = self._center_visible_target()
            if not centered:
                return ActionResult(False, action, "could not keep the red target visible while aligning", self.state())
            # Visual servo: advance in small odometric steps while the cube stays
            # centered. Stop while it is still fully useful in the image; the
            # final short approach is a calibrated dead-reckoned creep in pick().
            approach_step = {1.0: 0.03, 2.0: 0.045, 3.0: 0.06}[self.speed_multiplier]
            approach_iters = int(math.ceil(0.36 / approach_step))
            for _ in range(approach_iters):
                det = self.camera_red_detection()
                if not det.get("visible"):
                    return ActionResult(False, action, "lost the red target during visual approach", self.state())
                if float(det.get("cy", 0.0)) >= 0.66:
                    centered, det = self._center_visible_target(max_iters=8)
                    if not centered:
                        return ActionResult(False, action, "target reached pre-grasp size but could not be centered", self.state())
                    return ActionResult(True, action, "visually approached and tightly centered the red target at pre-grasp range", self.state())
                centered, det = self._center_visible_target(max_iters=4)
                if not centered:
                    return ActionResult(False, action, "lost the red target during visual approach", self.state())
                self.move_base_relative(forward=approach_step, frames=4)
            return ActionResult(True, action, "visual approach reached its odometry limit; verify the camera before picking", self.state())

        if action == "pick":
            det = self.camera_red_detection()
            if not det.get("visible"):
                return ActionResult(False, action, "red target is not visible in the robot camera; approach first", self.state())
            if float(det.get("cy", 0.0)) < 0.60:
                return ActionResult(False, action, "red target is visible but still too far away in the camera; approach first", self.state())
            centered, det = self._center_visible_target(max_iters=8)
            if not centered:
                return ActionResult(False, action, "could not center the target before grasp", self.state())
            # From the visually calibrated pre-grasp image, estimate the
            # remaining base travel from image cy. Offline calibration against
            # the 50 mm cube gives ~1.3 mm RMS range error. A 10 mm short-side
            # bias avoids overshooting the narrow friction-grasp window.
            # Runtime uses camera pixels only; no simulator target pose is read.
            cy = float(det.get("cy", 0.72))
            final_creep = float(np.clip(0.4083 - 0.3311 * cy, 0.135, 0.195))
            self.move_base_relative(forward=final_creep, frames=12)
            self.open_gripper()
            self.step_physics(20)
            self._emit_frame()
            pose = GROUND_POSE.copy()
            pose[0] = 0.0
            self.set_arm_pose(pose, 220)
            close_samples = self._fast_samples(24, minimum=8)
            for i, c in enumerate(np.linspace(0.0, 1.0, close_samples)):
                self.close_gripper(float(c))
                self.step_physics(self._fast_steps(10))
                if i % max(1, close_samples // 12) == 0 or i == close_samples - 1:
                    self._emit_frame()
            # Execution success means the motor sequence completed. Whether a
            # block was actually acquired is deliberately left to the next camera frame.
            return ActionResult(True, action, "grasp motion completed; verify the result from the robot camera", self.state())

        if action == "carry":
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
                    return ActionResult(False, action, "red block slipped from the gripper during lift", self.state())
            self.step_physics(30)
            self._emit_frame()
            return ActionResult(True, action, "lift motion completed with bilateral finger contact", self.state())

        if action == "place_on_yellow":
            if self.yellow_map_xy is None:
                return ActionResult(False, action, "yellow target has not been mapped; map_yellow first", self.state())
            self.set_arm_pose(CARRY_POSE, 100)
            self.close_gripper(1.0)
            vec = self.yellow_map_xy - self.odom_xy
            dist = float(np.linalg.norm(vec))
            target_yaw = self.base_yaw if dist < 1e-4 else math.atan2(float(vec[1]), float(vec[0]))
            yaw_delta = (target_yaw - self.base_yaw + math.pi) % (2.0 * math.pi) - math.pi
            self.move_base_relative(yaw_delta=yaw_delta, frames=10)
            forward = max(0.0, dist - 0.238)
            if forward > 1e-6:
                # Transport in one smooth profile. Repeated stop-start segments
                # accumulated enough tangential disturbance to let a friction
                # grasp creep out of the jaws.
                self.move_base_relative(forward=forward, frames=24)
                self.close_gripper(1.0)
            self.set_arm_pose(STACK_POSE, 240)
            self.step_physics(50)
            # Lower until the carried cube is almost supported by the yellow
            # cube, then release gradually. Keep physical release time invariant
            # across UI speed modes; speed affects rendering, not contact physics.
            open_samples = 10
            for i, c in enumerate(np.linspace(1.0, 0.0, open_samples)):
                self.close_gripper(float(c))
                self.step_physics(20)
                if i % 2 == 0 or i == open_samples - 1:
                    self._emit_frame()
            self.open_gripper()
            self.step_physics(160)
            self.set_arm_pose(CARRY_POSE, 180)
            self._emit_frame()
            # Driver-level postcondition verification. Raw simulator geometry never
            # leaves this skill boundary; the planner only receives achieved/not-achieved.
            red = self._body_pos("red_block")
            yellow = self._body_pos("yellow_block")
            planar_error = float(np.linalg.norm(red[:2] - yellow[:2]))
            vertical_gap = float(red[2] - yellow[2])
            placed = planar_error <= 0.040 and 0.040 <= vertical_gap <= 0.065
            return ActionResult(
                placed, action,
                "placement verified on the yellow target" if placed else "placement motion completed but stack postcondition was not achieved",
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
