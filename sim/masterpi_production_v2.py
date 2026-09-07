from __future__ import annotations

import io
import math
import time as wall_time
from dataclasses import dataclass
from typing import Mapping

import mujoco
import numpy as np
from PIL import Image

from harness.real_geometry import project_detection
from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2, TARGET_BLOCK_HALF_M, TARGET_BLOCK_SIDE_M, TARGET_BLOCK_LIFT_CENTER_M, WHEEL_RADIUS_M
from sim.masterpi_camera_profile import CAMERA_CALIBRATION_ID, CAMERA_MOUNT_STATUS

SERVO_IDS = (1, 3, 4, 5, 6)
SEARCH_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
HOVER_POSE = {1: 2000, 3: 650, 4: 2230, 5: 1500, 6: 1500}
GRASP_POSE = {1: 2000, 3: 650, 4: 2230, 5: 1920, 6: 1500}
CLOSED_GRASP_POSE = {**GRASP_POSE, 1: 1500}
LIFT_POSE = {1: 1500, 3: 650, 4: 2230, 5: 1500, 6: 1500}
CARRY_POSE = {1: 1500, 3: 960, 4: 2410, 5: 1215, 6: 1500}
STACK_POSE = {1: 1500, 3: 650, 4: 2230, 5: 1750, 6: 1500}
OPEN_STACK_POSE = {**STACK_POSE, 1: 2000}

FORWARD = np.array([1.0, 1.0, 1.0, 1.0])
BACKWARD = -FORWARD
LEFT = np.array([-1.0, 1.0, 1.0, -1.0])
RIGHT = -LEFT
ROTATE_LEFT = np.array([-1.0, 1.0, -1.0, 1.0])
ROTATE_RIGHT = -ROTATE_LEFT
STOP = np.zeros(4)


@dataclass
class ActionResult:
    ok: bool
    action: str
    reason: str
    state: dict


def _detect_color(rgb: np.ndarray, color: str) -> dict:
    h, w = rgb.shape[:2]
    r = rgb[..., 0].astype(np.int16)
    g = rgb[..., 1].astype(np.int16)
    b = rgb[..., 2].astype(np.int16)
    if color == "red":
        mask = (r > 55) & (r > 3.0 * g) & (r > 3.0 * b) & ((r - g) > 35)
    elif color == "blue":
        mask = (b > 65) & (b > 1.65 * r) & (b > 1.55 * g) & ((b - r) > 40)
    elif color == "yellow":
        mask = (r > 95) & (g > 80) & (b < 80) & ((r - b) > 65) & ((g - b) > 55) & (np.abs(r - g) < 105)
    else:
        raise ValueError(color)
    ys, xs = np.where(mask)
    min_pixels = max(8, int(h * w * 0.00025))
    if xs.size < min_pixels:
        return {"visible": False, "cx": 0.5, "cy": 0.5, "area_ratio": 0.0, "bbox_w": 0.0, "bbox_h": 0.0, "bbox": None}
    x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
    area = float(xs.size) / float(h * w)
    # Reject robot/body-colored spill. A 30-mm task target should not fill a large fraction.
    if area > 0.24:
        return {"visible": False, "cx": 0.5, "cy": 0.5, "area_ratio": 0.0, "bbox_w": 0.0, "bbox_h": 0.0, "bbox": None}
    return {
        "visible": True,
        "cx": float(xs.mean()) / float(w),
        "cy": float(ys.mean()) / float(h),
        "area_ratio": area,
        "bbox_w": float(x1 - x0 + 1) / float(w),
        "bbox_h": float(y1 - y0 + 1) / float(h),
        "bbox": [float(x0) / w, float(y0) / h, float(x1 + 1) / w, float(y1 + 1) / h],
        "source": "robot_camera_rgb",
    }


class MasterPiProductionV2(MasterPiDynamicsV2):
    """Production high-level skill adapter on the same v2 core used for training."""

    def __init__(self, seed: int = 0, width: int = 640, height: int = 480, render: bool = True, *, dynamics=None, hardware=None, use_calibration_manifest: bool = True):
        self.speed_multiplier = 1.0
        self.frame_callback = None
        self._frame_step = 0
        self.grasp_color: str | None = None
        self.pregrasp_color: str | None = None
        self.spatial_memory: dict[str, dict] = {}
        self._observation_id = 0
        self._real_stack = None
        self._latest_robot_bgr = None
        self._latest_robot_frame_seq = 0
        self._latest_robot_jpeg = None
        self._latest_robot_jpeg_seq = None
        self._latest_robot_jpeg_quality = None
        self._presentation_dirty = True
        super().__init__(
            seed=seed, render=render, width=width, height=height,
            dynamics=dynamics, hardware=hardware, use_calibration_manifest=use_calibration_manifest,
        )
        self.reset(seed)

    def _physics_step(self, commands: np.ndarray | None = None) -> None:
        super()._physics_step(commands)
        self._presentation_dirty = True
        self._frame_step += 1
        # 20 Hz presentation target at 1x. The worker still applies its wall-clock
        # stream gate, so faster SIM speeds do not flood the browser.
        cadence = max(10, int(round(25 / self.speed_multiplier)))
        if self.frame_callback is not None and self._frame_step % cadence == 0:
            try:
                self.frame_callback()
            except Exception:
                pass

    def set_speed_multiplier(self, value: float) -> None:
        value = float(value)
        if value not in (1.0, 2.0, 3.0):
            raise ValueError("speed multiplier must be 1, 2, or 3")
        self.speed_multiplier = value

    def step_realtime(self, duration_s: float) -> dict:
        """Advance production MuJoCo at the UI-selected wall-clock speed.

        The old SIM advanced a 1.2 s servo motion as fast as the GPU could
        calculate it. The worker's 10 Hz publisher is wall-clock throttled, so
        browsers received only the pre/post states and the held block appeared
        to teleport to the gripper. When a live frame callback is attached,
        advance in short chunks and pace them to 1x/2x/3x wall time. Headless
        unit tests keep the fast path because they have no live callback.
        """
        duration_s = float(duration_s)
        if duration_s <= 0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        if self.frame_callback is None:
            return self.step(duration_s=duration_s)
        speed = float(self.speed_multiplier)
        chunk_s = 0.020
        simulated = 0.0
        started = wall_time.perf_counter()
        while simulated + 1e-12 < duration_s:
            chunk = min(chunk_s, duration_s - simulated)
            self.step(duration_s=chunk)
            simulated += chunk
            deadline = started + simulated / speed
            delay = deadline - wall_time.perf_counter()
            if delay > 0:
                wall_time.sleep(delay)
        return self.state()

    def _real_controller(self):
        """Lazily bind the physical REAL stack to this persistent MuJoCo world."""
        if self._real_stack is None:
            from sim.real_stack_adapter import MigratedRealStack
            self._real_stack = MigratedRealStack(self)
        return self._real_stack

    def _real_action(self, action: str, **params) -> ActionResult:
        """Run the shared REAL task/controller source against the MuJoCo I/O boundary."""
        stack = self._real_controller()
        target_color = str(params.get("target_color") or "red")
        if target_color not in {"red", "blue", "yellow"}:
            return ActionResult(False, action, f"unsupported target_color={target_color}", self.state())
        try:
            if action == "search":
                reason = stack.search(
                    target_color=target_color,
                    seconds=float(params.get("seconds") or 12.0),
                )
            elif action == "track":
                reason = stack.track(
                    target_color=target_color,
                    seconds=float(params.get("seconds") or 4.0),
                )
            elif action == "approach":
                reason = stack.approach(target_color=target_color)
                self.pregrasp_color = target_color
            elif action == "pick":
                reason = stack.pick(target_color=target_color)
                self.pregrasp_color = None
            elif action == "put_down":
                reason = stack.put_down()
            elif action == "search_destination":
                destination_color = str(params.get("destination_color") or "")
                if destination_color not in {"red", "blue", "yellow"}:
                    raise ValueError("search_destination requires destination_color=red|blue|yellow")
                reason = stack.search_destination(target_color=target_color, destination_color=destination_color)
            elif action == "place":
                destination_color = str(params.get("destination_color") or "")
                if destination_color not in {"red", "blue", "yellow"}:
                    raise ValueError("place requires destination_color=red|blue|yellow")
                reason = stack.place(
                    target_color=target_color,
                    destination_color=destination_color,
                )
            elif action in {
                "move_forward", "move_backward",
                "turn_left", "turn_right", "stop_motion",
            }:
                from scripts.robot_actions import PRIMITIVE_MOTIONS
                reason = stack.primitive(
                    PRIMITIVE_MOTIONS[action],
                    speed=int(params.get("speed") or 35),
                    duration=float(params.get("duration") or 0.30),
                )
            elif action == "observe_scene":
                # Observation is an environment boundary, not a control policy.
                # Use only robot-camera RGB + commanded FK/odometry, matching the
                # transferable REAL observation contract.
                detections = self.scene_detections()
                visible = sorted(c for c, d in detections.items() if d.get("visible"))
                reason = f"read-only MuJoCo robot-camera observation completed; visible={visible}"
            else:
                raise ValueError(action)
            return ActionResult(True, action, reason, self.state())
        except Exception as exc:
            try:
                stack.robot.stop()
            except Exception:
                pass
            return ActionResult(False, action, str(exc), self.state())

    def reset(self, seed: int | None = None, block_xy=None) -> dict:
        if seed is not None:
            self.seed = int(seed)
            self.rng = np.random.default_rng(self.seed)
        super().reset()
        # Production reset preserves the canonical workcell but randomizes the pickup target.
        if block_xy is None:
            bx = 0.54 + float(self.rng.uniform(-0.04, 0.04))
            by = 0.07 + float(self.rng.uniform(-0.16, 0.16))
        else:
            bx, by = map(float, block_xy)
        self.set_free_body_pose_for_reset("red_block", (bx, by, TARGET_BLOCK_HALF_M), float(self.rng.uniform(-math.pi, math.pi)))
        self.set_free_body_pose_for_reset("blue_block", (0.70, -0.20, TARGET_BLOCK_HALF_M), 0.0)
        self.set_free_body_pose_for_reset("yellow_block", (0.82, 0.23, TARGET_BLOCK_HALF_M), 0.0)
        self.grasp_color = None
        self.pregrasp_color = None
        self.spatial_memory.clear()
        self._observation_id = 0
        self._frame_step = 0
        self._latest_robot_bgr = None
        self._latest_robot_frame_seq = 0
        self._latest_robot_jpeg = None
        self._latest_robot_jpeg_seq = None
        self._latest_robot_jpeg_quality = None
        self._presentation_dirty = True
        self.set_servo_pulses(SEARCH_POSE, forward_only=True)
        for _ in range(int(round(0.20 / self.model.opt.timestep))):
            super()._physics_step(STOP)
        if self._real_stack is not None:
            self._real_stack.reset()
        return self.state()

    def render_jpeg(self, camera: str = "robot_cam", quality: int = 82) -> bytes:
        rgb = self.render_rgb(camera)
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=int(quality))
        return buf.getvalue()

    def _arm_qpos(self) -> list[float]:
        out = []
        for name in ("arm_yaw", "shoulder", "elbow", "wrist_pitch"):
            jid = self._joint(name)
            out.append(float(self.data.qpos[int(self.model.jnt_qposadr[jid])]))
        return out

    def _gripper_close_qpos(self) -> float:
        vals = []
        for jid in self.gripper_joint:
            vals.append(float(self.data.qpos[int(self.model.jnt_qposadr[jid])]))
        return float(sum(vals) / max(1, len(vals)))

    def _world_from_relative(self, x_forward: float, y_left: float) -> tuple[float, float]:
        base = self.base_xyz()
        _, _, yaw = self.base_rpy()
        c, s = math.cos(yaw), math.sin(yaw)
        return float(base[0] + c*x_forward - s*y_left), float(base[1] + s*x_forward + c*y_left)

    def _remember(self, color: str, det: Mapping) -> None:
        proj = project_detection(self.servo_command_pulses, det)
        if proj is None:
            return
        # Nominal digital-twin profile: consume the shared camera/FK projection
        # directly. Measured range error belongs in calibrated geometry, not a
        # hidden production-only offset.
        wx, wy = self._world_from_relative(float(proj.x_forward_m), float(proj.y_left_m))
        self._observation_id += 1
        old = self.spatial_memory.get(color) or {}
        self.spatial_memory[color] = {
            "position_xy": [round(wx, 4), round(wy, 4)],
            "height_m": TARGET_BLOCK_SIDE_M,
            "distance_m": round(float(proj.range_m), 4),
            "bearing_deg": round(float(proj.bearing_deg), 2),
            "confidence": min(0.98, 0.80 + 0.02 * int(old.get("seen_count", 0))),
            "visible": True,
            "seen_count": int(old.get("seen_count", 0)) + 1,
            "age_observations": 0,
            "relation": old.get("relation", "GROUND"),
            "source": "robot_camera_rgb+commanded_fk+base_odometry",
        }

    def cache_robot_rgb(self, rgb: np.ndarray, *, sensor_sequence: int | None = None) -> None:
        """Publish one actor-visible sensor sample for zero-render UI reuse."""
        self._latest_robot_bgr = np.ascontiguousarray(rgb[..., ::-1])
        self._latest_robot_frame_seq = (
            int(sensor_sequence)
            if sensor_sequence is not None
            else int(getattr(self, "_latest_robot_frame_seq", 0) or 0) + 1
        )
        self._latest_robot_jpeg = None
        self._latest_robot_jpeg_seq = None
        self._latest_robot_jpeg_quality = None
        self._presentation_dirty = False

    def scene_detections(self, rgb: np.ndarray | None = None) -> dict[str, dict]:
        if rgb is None:
            rgb = self.render_rgb("robot_cam")
        self.cache_robot_rgb(rgb)
        out = {c: _detect_color(rgb, c) for c in ("red", "blue", "yellow")}
        for color, det in out.items():
            if det.get("visible"):
                self._remember(color, det)
            elif color in self.spatial_memory:
                self.spatial_memory[color]["visible"] = False
                self.spatial_memory[color]["age_observations"] = int(self.spatial_memory[color].get("age_observations", 0)) + 1
        return out

    def camera_detection(self, color: str) -> dict:
        return self.scene_detections().get(color, {"visible": False})

    def spatial_memory_public(self) -> dict:
        return {k: dict(v) for k, v in self.spatial_memory.items()}

    def semantic_map_public(self) -> dict:
        base = self.base_xyz(); _, _, yaw = self.base_rpy()
        return {
            "robot_xy": [round(float(base[0]), 4), round(float(base[1]), 4)],
            "objects": self.spatial_memory_public(),
            "obstacles": [],
            "zones": [],
            "scan": {"heading_rad": round(float(yaw + self._arm_qpos()[0]), 4)},
            "source": "v2_rgb+fk+odometry",
        }

    def _reconcile_grasp_state(self) -> None:
        """Keep logical grasp state subordinate to MuJoCo physics.

        A color may only remain marked HELD while the jaws are physically closed
        around a lifted block.  This prevents stale executive state from claiming
        a grasp after a later search/track/servo motion opened the gripper or the
        block fell back to the floor.
        """
        color = self.grasp_color
        if color not in {"red", "blue", "yellow"}:
            return
        block_z = float(self.body_xyz(f"{color}_block")[2])
        contact = self.finger_block_contact(color)
        gripper_pulse = int(self.servo_command_pulses.get(1, 2000))
        clearly_open = gripper_pulse >= 1900 or self._gripper_close_qpos() <= 0.001
        clearly_dropped = block_z <= 0.040 and not bool(contact.get("bilateral"))
        if clearly_open or clearly_dropped:
            self.grasp_color = None
            entry = self.spatial_memory.get(color)
            if isinstance(entry, dict) and entry.get("relation") == "HELD":
                entry["relation"] = "GROUND"

    def state(self) -> dict:
        self._reconcile_grasp_state()
        base = self.base_xyz(); roll, pitch, yaw = self.base_rpy()
        red = self.body_xyz("red_block"); blue = self.body_xyz("blue_block"); yellow = self.body_xyz("yellow_block")
        color = self.grasp_color or "red"
        contact = self.finger_block_contact(color)
        block = {"red": red, "blue": blue, "yellow": yellow}[color]
        grip = self.site_xyz("grip_site")
        left_finger_gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger")
        right_finger_gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger")
        left_finger_xyz = self.data.geom_xpos[left_finger_gid].copy()
        right_finger_xyz = self.data.geom_xpos[right_finger_gid].copy()
        return {
            "seed": int(self.seed),
            "model": "masterpi_dynamics_v2_reduced_mecanum",
            "physics_fidelity": (
                "V2_REAL_CALIBRATED_VALIDATED"
                if self.calibration_status == "REAL_CALIBRATED_VALIDATED"
                else "V2_REAL_FITTED_UNVALIDATED"
                if self.calibration_status == "REAL_FITTED_UNVALIDATED"
                else "V2_STRUCTURAL_UNCALIBRATED"
            ),
            "training_ready": self.calibration_status == "REAL_CALIBRATED_VALIDATED",
            "calibration_status": self.calibration_status,
            "calibration_parameters": dict(self.calibration_parameters),
            "physical_parameters": dict(self.physical_params),
            "camera_calibration_id": CAMERA_CALIBRATION_ID,
            "camera_mount_status": CAMERA_MOUNT_STATUS,
            "camera_raw_fisheye": True,
            "digital_twin_profile": (
                "calibrated_task_twin_v1"
                if self.calibration_status == "REAL_CALIBRATED_VALIDATED"
                else "fitted_real_contract_v1"
                if self.calibration_status == "REAL_FITTED_UNVALIDATED"
                else "nominal_real_contract_v1"
            ),
            "sim_only_adjustments": [],
            "gripper_symmetric": True,
            "robot_xy": [round(float(base[0]), 4), round(float(base[1]), 4)],
            "base_xyz": [round(float(v), 4) for v in base],
            "base_rpy": [round(float(v), 4) for v in (roll, pitch, yaw)],
            "base_yaw": round(float(yaw), 4),
            "red_xyz": [round(float(v), 4) for v in red],
            "blue_xyz": [round(float(v), 4) for v in blue],
            "yellow_xyz": [round(float(v), 4) for v in yellow],
            "grasp_color": self.grasp_color,
            "grip_xyz": [round(float(v), 4) for v in grip],
            "left_finger_xyz": [round(float(v), 4) for v in left_finger_xyz],
            "right_finger_xyz": [round(float(v), 4) for v in right_finger_xyz],
            "grip_error_m": round(float(np.linalg.norm(block - grip)), 4),
            "left_contact": bool(contact["left"]),
            "right_contact": bool(contact["right"]),
            "left_normal_N": round(float(contact["left_force_n"]), 3),
            "right_normal_N": round(float(contact["right_force_n"]), 3),
            "bilateral_contact": bool(contact["bilateral"]),
            "lifted": bool(self.grasp_color is not None and block[2] > TARGET_BLOCK_LIFT_CENTER_M),
            # Require bottom clearance of at least half a block side plus true
            # bilateral MuJoCo contact.
            "stable": bool(self.grasp_color is not None and contact["bilateral"] and block[2] > TARGET_BLOCK_LIFT_CENTER_M),
            "stable_steps": 0,
            "gripper_qpos": round(self._gripper_close_qpos(), 4),
            "arm_qpos": [round(float(v), 4) for v in self._arm_qpos()],
            "commanded_pwm": dict(self.servo_command_pulses),
            "motor_commands": [round(float(v), 4) for v in self.motor_command],
            "spatial_memory": self.spatial_memory_public(),
            "semantic_map": self.semantic_map_public(),
            "memory_observation_id": int(self._observation_id),
            "arm_dof": 4,
            "gripper_dof": 1,
        }

    def _drive(self, pattern: np.ndarray, duration_s: float, settle_s: float = 0.08) -> None:
        self.step(motor_commands=pattern, duration_s=float(duration_s))
        self.step(motor_commands=STOP, duration_s=float(settle_s))

    def _search_color(self, color: str) -> dict:
        self.move_pose_timed(SEARCH_POSE, lowering=False, settle_s=0.02)
        det = self.camera_detection(color)
        if det.get("visible"):
            return det
        for pulse in (1300, 1700, 1100, 1900, 1500):
            self.move_servo_timed(6, pulse, 0.22, settle_s=0.03)
            det = self.camera_detection(color)
            if det.get("visible"):
                return det
        return {"visible": False}

    def act(self, action: str, *, seed: int | None = None, **params) -> ActionResult:
        action = str(action).strip().lower()
        if action == "reset":
            return ActionResult(True, action, "v2 simulation reset", self.reset(seed))
        if action in {
            "move_forward", "move_backward",
            "turn_left", "turn_right", "stop_motion", "observe_scene",
            "search", "track", "approach", "pick", "put_down", "search_destination", "place",
        }:
            return self._real_action(action, **params)
        # REAL compatibility aliases contain no independent policy.
        if action in {"place_on_blue", "place_on_yellow"}:
            destination_color = action.removeprefix("place_on_")
            target_color = str(params.get("target_color") or self.grasp_color or "red")
            return self._real_action(
                "place", target_color=target_color, destination_color=destination_color
            )
        if action == "scan_world":
            seen = []
            for pulse in (1100, 1300, 1500, 1700, 1900, 1500):
                self.move_servo_timed(6, pulse, 0.18, settle_s=0.02)
                dets = self.scene_detections()
                seen.extend([c for c, d in dets.items() if d.get("visible")])
            return ActionResult(True, action, f"v2 head scan completed; objects={sorted(set(seen))}", self.state())
        if action.startswith("map_"):
            color = action.removeprefix("map_")
            det = self._search_color(color)
            ok = bool(det.get("visible") and color in self.spatial_memory)
            return ActionResult(ok, action, f"{color} mapped from v2 RGB/FK" if ok else f"{color} target not localized", self.state())
        if action in {"search_red", "track_red", "approach_red", "pick_red"}:
            base = action.removesuffix("_red")
            return self._real_action(base, target_color="red")
        if action in {
            "search_blue", "search_yellow",
            "track_blue", "track_yellow",
            "approach_blue", "approach_yellow",
            "pick_blue", "pick_yellow",
        }:
            base, color = action.split("_", 1)
            return self._real_action(base, target_color=color)
        if action == "carry":
            return ActionResult(
                False, action,
                "standalone carry is disabled; shared REAL pick already ends in carry pose",
                self.state(),
            )
        if action == "place_on_red":
            target_color = str(params.get("target_color") or self.grasp_color or "blue")
            return self._real_action(
                "place", target_color=target_color, destination_color="red"
            )
        return ActionResult(False, action, "unknown v2 action", self.state())
