from __future__ import annotations

import io
import math
import os
from pathlib import Path
from dataclasses import dataclass

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

XML = (Path(__file__).with_name("masterpi_scene.xml")).read_text(encoding="utf-8")



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

    def render_camera_jpeg(self, camera: str, quality: int = 85) -> bytes:
        self.renderer.update_scene(self.data, camera=camera)
        rgb = self.renderer.render()
        im = Image.fromarray(rgb)
        buf = io.BytesIO()
        im.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

    def render_overview_jpeg(self, quality: int = 85) -> bytes:
        return self.render_camera_jpeg("cctv_front_left", quality=quality)

    def render_observer_jpegs(self, quality: int = 82) -> dict[str, bytes]:
        return {
            name: self.render_camera_jpeg(name, quality=quality)
            for name in ("cctv_front_left", "cctv_front_right", "cctv_rear_left", "cctv_top")
        }

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
