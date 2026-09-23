"""Three independently addressable MasterPi robots in one MuJoCo world.

The single-robot production twin remains unchanged and is still the compatibility
path.  This module composes three copies of that exact robot body/actuator model
into one shared scene, then binds one namespaced production controller per copy.
Python skill stacks may overlap across robot namespaces; only shared MuJoCo
physics/data mutation is serialized at the world boundary. Runtime-dependent
REAL-controller dependencies are selected with ContextVars by the SIM adapter.
"""
from __future__ import annotations

import copy
import io
import math
import os
import threading
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from dataclasses import replace
import time as wall_time
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Mapping, Sequence

import mujoco
import numpy as np
from PIL import Image

from sim.masterpi_camera_profile import raw_fisheye_remap
from sim.snapshot_render import CameraKey, RenderBatch, SnapshotRenderBroker
from sim.cooperative_payload import (
    BEAM_BODY_NAME,
    BEAM_CARRIER_IDS,
    BEAM_ENDPOINT_OFFSETS_M,
    BEAM_GOAL,
    BEAM_GOAL_YAW_RAD,
    BEAM_HALF_HEIGHT_M,
    BEAM_START,
    add_cooperative_payload_xml,
    beam_pose,
    evaluate_beam_mission,
)
from sim.warehouse_mission import (
    CARGO_SPECS,
    WAREHOUSE_ZONES,
    add_warehouse_mission_xml,
    cargo_pose,
    cargo_specs_for_seed,
    cargo_zone_positions_for_seed,
    evaluate_cargo_delivery,
    warehouse_manifest,
    warehouse_route_zones,
    terrain_specs_for_seed,
)
from sim.adaptive_warehouse import (
    CargoObservation,
    RobotObservation,
    TerrainObservation,
    auction_roles,
    plan_local_path,
    plan_monotone_visibility_path,
    segment_terrain,
)
from sim.masterpi_dynamics_v2 import (
    FORWARD_PATTERN,
    LEFT_PATTERN,
    MAX_WHEEL_RAD_S,
    WHEEL_RADIUS_M,
    YAW_LEFT_PATTERN,
    build_v2_xml,
)
from sim.masterpi_production_v2 import (
    ActionResult,
    CARRY_POSE,
    MasterPiProductionV2,
    SEARCH_POSE,
    STOP,
    TARGET_BLOCK_HALF_M,
    TARGET_BLOCK_LIFT_CENTER_M,
    TARGET_BLOCK_SIDE_M,
)

ROBOT_IDS = ("r1", "r2", "r3")
# Keep enough lateral clearance for arms/cameras while all three face the same
# shared work area. These are reset fixtures, not learned navigation targets.
DEFAULT_SPAWNS = {
    "r1": (-0.18, -0.42, WHEEL_RADIUS_M, 0.0),
    "r2": (-0.18, 0.00, WHEEL_RADIUS_M, 0.0),
    "r3": (-0.18, 0.42, WHEEL_RADIUS_M, 0.0),
}
GLOBAL_BODIES = {
    "red_block", "blue_block", "yellow_block", BEAM_BODY_NAME,
    *(spec.body_name for spec in CARGO_SPECS),
}
GLOBAL_JOINTS = {
    "red_free", "blue_free", "yellow_free", "team_beam_free",
    *(spec.joint_name for spec in CARGO_SPECS),
}
from sim.team_layout import TEAM_STACK_SITE  # noqa: E402  (shared with the harness verifier)
# Lateral transit lane. Measured footprints (sim/multi_masterpi_production
# probe, 2026-09-02): body lateral half-width 0.104 m, rear 0.107 m, front
# 0.121 m in the compact SEARCH_POSE and 0.193 m with the hover arm. A robot in
# the lane faces +/-Y, so its +/-0.104 m side must clear (a) the front of a robot
# parked at TEAM_PARK_X and (b) the rear of a robot at the nearest approach
# standoff (TEAM_BLOCK_X_RANGE[0] - TEAM_APPROACH_STANDOFF_M - 0.107). Turning
# in the lane sweeps the hover arm (+block) ~0.21 m, which is why parked robots
# sit at -0.18 m rather than at the old 0.02 m.
TEAM_SAFE_CORRIDOR_X = 0.15
TEAM_PARK_X = -0.18
TEAM_APPROACH_STANDOFF_M = 0.165
TEAM_AXIS_TOL_M = 0.0035
TEAM_MOTOR_SCALE = 0.875
TEAM_BLOCK_X_RANGE = (0.56, 0.83)
TEAM_BLOCK_Y_RANGE = (-0.36, 0.36)
# Every block gets its own Y row: two robots must be able to stand at their
# standoffs side by side (chassis width 0.208 m plus the 0.06 m soft peer gap),
# and a robot heading for a far block must never have to drive through a peer
# standing at a nearer block in the same row.
TEAM_BLOCK_MIN_SEPARATION_M = 0.20
TEAM_BLOCK_MIN_ROW_SEPARATION_M = 0.30
TEAM_BLOCK_STACK_SITE_CLEARANCE_M = 0.135
TEAM_PLACE_AIM_OFFSET_M = np.array((0.0055, -0.0010), dtype=float)
# Environment referee for the shared world. Commands are serialized, so the
# moving robot is the only one that can cause a collision. Clearance is the
# smallest gap between any geom of the mover (arm included) and any geom of a
# peer, so an extended arm or a held block counts: refuse to start a chassis
# action when a peer is already inside the soft gap, abort a geometric move
# the moment a peer comes inside the hard gap.
TEAM_PEER_CLEARANCE_M = 0.06
TEAM_PEER_HARD_CLEARANCE_M = 0.03
TEAM_CHASSIS_ACTIONS = frozenset({
    "approach", "pick", "place", "stack_on", "stage_base", "put_down",
    "move_forward", "move_backward", "turn_left", "turn_right", "drive", "track",
})
BEAM_ROLE_BY_ROBOT = {
    "r1": "carrier_left",
    "r2": "scout",
    "r3": "carrier_right",
}
TEAM_DETERMINISTIC_LAYOUT = {
    "red": (0.62, -0.30, 0.0),
    "blue": (0.66, 0.00, 0.0),
    "yellow": (0.60, 0.30, 0.0),
}


class PeerTooClose(RuntimeError):
    """Raised by the shared-world referee; classified as PEER_TOO_CLOSE upstream."""


def _prefix_named_tree(node: ET.Element, prefix: str) -> ET.Element:
    clone = copy.deepcopy(node)
    for item in clone.iter():
        name = item.get("name")
        if name:
            item.set("name", prefix + name)
    return clone


def build_multi_robot_xml(
    hardware: Mapping[str, float] | None = None,
    *,
    robot_ids: Sequence[str] = ROBOT_IDS,
    spawns: Mapping[str, Sequence[float]] = DEFAULT_SPAWNS,
    warehouse_specs=None,
    warehouse_terrain=None,
    warehouse_zones=None,
    navigation_camera: bool = False,
    scene_objects=(),
) -> str:
    """Clone the validated single-robot XML with deterministic name prefixes."""
    ids = tuple(str(x).strip().lower() for x in robot_ids)
    if not ids or len(set(ids)) != len(ids):
        raise ValueError("robot_ids must be unique")
    root = ET.fromstring(build_v2_xml(hardware))
    root.set("model", "ugrp_masterpi_multi_v2")
    world = root.find("worldbody")
    actuator = root.find("actuator")
    if world is None or actuator is None:
        raise RuntimeError("single-robot v2 XML has no worldbody/actuator")
    robot = next((c for c in list(world) if c.tag == "body" and c.get("name") == "robot"), None)
    if robot is None:
        raise RuntimeError("single-robot v2 XML has no robot body")
    original_actuators = list(actuator)
    insert_at = list(world).index(robot)
    world.remove(robot)
    for child in list(actuator):
        actuator.remove(child)

    for offset, rid in enumerate(ids):
        prefix = f"{rid}__"
        cloned = _prefix_named_tree(robot, prefix)
        # Single-robot robot geoms are type 2 against world type 1. In the
        # composed scene peers are also type 2, so opt only the cloned team
        # geoms into peer affinity while leaving the validated single twin alone.
        for geom in cloned.iter("geom"):
            contype = int(geom.get("contype", "1"))
            if contype & 2:
                geom.set("conaffinity", str(int(geom.get("conaffinity", "1")) | 2))
        spawn = tuple(float(v) for v in spawns.get(rid, DEFAULT_SPAWNS.get(rid, (0, offset * 0.4, WHEEL_RADIUS_M, 0))))
        if len(spawn) != 4:
            raise ValueError(f"spawn for {rid} must be x,y,z,yaw")
        x, y, z, yaw = spawn
        cloned.set("pos", f"{x:.6f} {y:.6f} {z:.6f}")
        cloned.set("quat", f"{math.cos(yaw/2):.9f} 0 0 {math.sin(yaw/2):.9f}")
        if navigation_camera:
            from sim.navigation_camera_profile import (
                NAV_CAMERA_NAME, NAV_MOUNT_XYZ_M, NAV_PITCH_DOWN_DEG,
                NAV_VERTICAL_FOVY_DEG,
            )
            pitch = math.radians(NAV_PITCH_DOWN_DEG)
            ET.SubElement(cloned, "camera", {
                "name": prefix + NAV_CAMERA_NAME,
                "pos": " ".join(f"{v:.6f}" for v in NAV_MOUNT_XYZ_M),
                "xyaxes": f"0 -1 0 {math.sin(pitch):.9f} 0 {math.cos(pitch):.9f}",
                "fovy": f"{NAV_VERTICAL_FOVY_DEG:.6f}",
            })
            # Bright, massless and non-colliding peer cue. It sits above the
            # chassis and below the camera, so the owning body does not fill
            # the lower navigation image or alter physical dynamics.
            ET.SubElement(cloned, "geom", {
                "name": prefix + "peer_visibility_band", "type": "box",
                "pos": "0 0 .205", "size": ".070 .055 .018",
                "rgba": "1 0 1 1", "mass": "0", "contype": "0",
                "conaffinity": "0", "group": "0",
            })
        world.insert(insert_at + offset, cloned)
        for source in original_actuators:
            item = copy.deepcopy(source)
            if item.get("name"):
                item.set("name", prefix + str(item.get("name")))
            if item.get("joint"):
                item.set("joint", prefix + str(item.get("joint")))
            actuator.append(item)
    # Multi-SIM only: lock the measured gripper/object relative pose after a
    # real bilateral contact has been confirmed. These constraints never exist
    # in the single-robot twin and stay inactive until a successful team grasp.
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    for rid in ids:
        for color in ("red", "blue", "yellow"):
            ET.SubElement(
                equality, "weld", name=f"{rid}__grasp_{color}",
                body1=f"{rid}__gripper", body2=f"{color}_block", active="false",
            )
    add_cooperative_payload_xml(root, carrier_ids=BEAM_CARRIER_IDS)
    if navigation_camera:
        # Keep the legacy names as inert schema placeholders because the shared
        # state API reads them, but exclude their geometry from this bounded
        # warehouse task.  They are calibration/demo props, not declared cargo
        # or seeded corridor obstacles.
        legacy_demo_bodies = {
            "red_block", "blue_block", "yellow_block", BEAM_BODY_NAME,
        }
        for body in world.iter("body"):
            if body.get("name") not in legacy_demo_bodies:
                continue
            # The placeholder retains its free joint for schema compatibility;
            # cancel gravity so an invisible, non-contact body cannot free-fall
            # indefinitely during long research episodes.
            for descendant_body in body.iter("body"):
                descendant_body.set("gravcomp", "1")
            for geom in body.iter("geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
                geom.set("rgba", "0 0 0 0")
                geom.set("group", "5")
    add_warehouse_mission_xml(
        root,
        specs=tuple(warehouse_specs or CARGO_SPECS),
        carrier_ids=tuple(ids),
        terrain=tuple(warehouse_terrain or ()),
        zones=warehouse_zones,
    )
    if navigation_camera:
        # Zone paint is actor-visible scene evidence for nav_cam. Other camera
        # paths keep their existing group-4 presentation convention.
        for geom in world.iter("geom"):
            if str(geom.get("name", "")).startswith("warehouse_zone_"):
                geom.set("group", "0")
    if warehouse_zones is not None and warehouse_zones is not WAREHOUSE_ZONES:
        for node in list(world):
            if node.get("name") in {"team_beam_start_zone", "team_beam_goal_zone"}:
                world.remove(node)
    if scene_objects:
        from sim.scene_objects import append_objects
        append_objects(world, list(scene_objects))
    return ET.tostring(root, encoding="unicode")


class NamespacedMasterPi(MasterPiProductionV2):
    """Production controller view bound to one robot inside shared model/data."""

    def _n(self, name: str) -> str:
        return f"{self.robot_id}__{name}"

    def _body(self, name: str) -> int:
        actual = name if name in GLOBAL_BODIES or name.startswith("warehouse_") else self._n(name)
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, actual)
        if idx < 0:
            raise KeyError(actual)
        return idx

    def _joint(self, name: str) -> int:
        actual = name if name in GLOBAL_JOINTS or name.startswith("warehouse_") else self._n(name)
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, actual)
        if idx < 0:
            raise KeyError(actual)
        return idx

    def _actuator(self, name: str) -> int:
        actual = self._n(name)
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actual)
        if idx < 0:
            raise KeyError(actual)
        return idx

    def _site(self, name: str) -> int:
        actual = self._n(name)
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, actual)
        if idx < 0:
            raise KeyError(actual)
        return idx

    def _physics_step(self, commands: np.ndarray | None = None) -> None:
        self._owner._physics_step_for(self, commands)
        self._frame_step += 1
        cadence = max(10, int(round(25 / self.speed_multiplier)))
        # MuJoCo/OpenGL rendering must stay on the thread that created the
        # shared world. TEAM preparation uses worker threads for concurrent
        # wheel/servo control, so those threads only advance physics; the owner
        # thread pumps presentation frames while they run.
        if (
            self._owner.frame_callback is not None
            and self._frame_step % cadence == 0
            and threading.get_ident() == self._owner._frame_callback_thread_id
        ):
            try:
                self._owner.frame_callback()
            except Exception:
                pass

    def advance_to_sim_time(self, target_time: float) -> None:
        """Advance physics without materializing a public state per quantum.

        ``SimClock.sleep`` already slices REAL-compatible waits into 20 ms
        quanta so servo interpolation can be updated.  Calling the inherited
        ``step`` for every quantum also builds a full nested state dictionary;
        the controller only needs that state at the next sensor/action boundary.
        Keep the shared ``physics_lock`` inside ``_physics_step`` and preserve
        the same rounded timestep semantics as ``MasterPiDynamicsV2.step``.
        """
        target_time = float(target_time)
        if not math.isfinite(target_time):
            raise ValueError("target_time must be finite")
        delta = target_time - float(self.data.time)
        if delta <= 0.0:
            return
        dt = float(self.model.opt.timestep)
        steps = max(1, int(round(delta / dt)))
        for _ in range(steps):
            self._physics_step()

    def render_rgb(self, camera: str = "robot_cam") -> np.ndarray:
        # macOS CGL contexts are thread-affine. act_parallel() intentionally
        # runs controller code on multiple threads, so touching one shared
        # mujoco.Renderer from those threads can wedge CGLLockContext forever
        # even when calls are guarded by a Python lock. The world owns a single
        # render broker which creates, uses and closes both GL renderers on one
        # dedicated thread; callers only wait for the returned pixels.
        return self._owner._render_rgb_for(self, camera)

    def finger_block_contact(self, color: str = "red") -> dict:
        with self._owner.physics_lock:
            return self._finger_block_contact_unlocked(color)

    def _finger_block_contact_unlocked(self, color: str = "red") -> dict:
        if color not in {"red", "blue", "yellow"}:
            raise ValueError("unsupported block color")
        block = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{color}_block_geom")
        left = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("left_finger"))
        right = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("right_finger"))
        hit_l = hit_r = False
        force_l = force_r = 0.0
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            if block not in (g1, g2):
                continue
            other = g2 if g1 == block else g1
            if other not in (left, right):
                continue
            wrench = np.zeros(6, dtype=float)
            if int(con.efc_address) >= 0:
                mujoco.mj_contactForce(self.model, self.data, i, wrench)
            normal = abs(float(wrench[0]))
            if other == left:
                hit_l = True; force_l += normal
            elif other == right:
                hit_r = True; force_r += normal
        return {"left": hit_l, "right": hit_r, "bilateral": bool(hit_l and hit_r), "left_force_n": force_l, "right_force_n": force_r}

    def nonfinger_block_contact_force(self, color: str = "red") -> float:
        with self._owner.physics_lock:
            return self._nonfinger_block_contact_force_unlocked(color)

    def finger_payload_contact(self, endpoint_for: str) -> dict:
        """Return bilateral finger contact with this carrier's beam handle."""
        endpoint_for = str(endpoint_for).strip().lower()
        if endpoint_for not in BEAM_CARRIER_IDS:
            raise ValueError("beam endpoint must be assigned to r1 or r3")
        endpoint_geom = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"{BEAM_BODY_NAME}_{endpoint_for}_endpoint_geom",
        )
        left = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("left_finger")
        )
        right = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("right_finger")
        )
        hit_l = hit_r = False
        force_l = force_r = 0.0
        with self._owner.physics_lock:
            for i in range(self.data.ncon):
                con = self.data.contact[i]
                g1, g2 = int(con.geom1), int(con.geom2)
                if endpoint_geom not in (g1, g2):
                    continue
                other = g2 if g1 == endpoint_geom else g1
                if other not in (left, right):
                    continue
                wrench = np.zeros(6, dtype=float)
                if int(con.efc_address) >= 0:
                    mujoco.mj_contactForce(self.model, self.data, i, wrench)
                normal = abs(float(wrench[0]))
                if other == left:
                    hit_l = True; force_l += normal
                else:
                    hit_r = True; force_r += normal
        return {
            "left": hit_l,
            "right": hit_r,
            "bilateral": bool(hit_l and hit_r),
            "left_force_n": force_l,
            "right_force_n": force_r,
        }

    def finger_cargo_contact(self, cargo_id: str) -> dict:
        """Return bilateral finger contact with the actual cargo body geom."""
        spec = self._owner.warehouse_spec_by_id.get(str(cargo_id))
        if spec is None:
            raise ValueError(f"unknown cargo {cargo_id}")
        cargo_geom = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"{spec.body_name}_geom",
        )
        left = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("left_finger")
        )
        right = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("right_finger")
        )
        hit_l = hit_r = False
        force_l = force_r = 0.0
        with self._owner.physics_lock:
            for i in range(self.data.ncon):
                con = self.data.contact[i]
                g1, g2 = int(con.geom1), int(con.geom2)
                if cargo_geom not in (g1, g2):
                    continue
                other = g2 if g1 == cargo_geom else g1
                if other not in (left, right):
                    continue
                wrench = np.zeros(6, dtype=float)
                if int(con.efc_address) >= 0:
                    mujoco.mj_contactForce(self.model, self.data, i, wrench)
                normal = abs(float(wrench[0]))
                if other == left:
                    hit_l = True; force_l += normal
                else:
                    hit_r = True; force_r += normal
        carriers = tuple(spec.carriers)
        side = "unassigned"
        if self.robot_id in carriers:
            side = "left" if self.robot_id == carriers[0] else "right"
        return {
            "left": hit_l, "right": hit_r,
            "bilateral": bool(hit_l and hit_r),
            "left_force_n": force_l, "right_force_n": force_r,
            "side": side,
        }

    def _nonfinger_block_contact_force_unlocked(self, color: str = "red") -> float:
        block = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{color}_block_geom")
        floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        fingers = {
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("left_finger")),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("right_finger")),
        }
        peak = 0.0
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            if block not in (g1, g2):
                continue
            other = g2 if g1 == block else g1
            if other == floor or other in fingers:
                continue
            wrench = np.zeros(6, dtype=float)
            if int(con.efc_address) >= 0:
                mujoco.mj_contactForce(self.model, self.data, i, wrench)
            peak = max(peak, abs(float(wrench[0])))
        return peak

    def base_xyz(self) -> np.ndarray:
        with self._owner.physics_lock:
            return super().base_xyz()

    def base_rpy(self) -> tuple[float, float, float]:
        with self._owner.physics_lock:
            return super().base_rpy()

    def body_xyz(self, name: str) -> np.ndarray:
        with self._owner.physics_lock:
            return super().body_xyz(name)

    def site_xyz(self, name: str) -> np.ndarray:
        with self._owner.physics_lock:
            return super().site_xyz(name)

    def state(self) -> dict:
        with self._owner.physics_lock:
            return self._state_unlocked()

    def _state_unlocked(self) -> dict:
        # The parent method is intentionally reused for all semantic fields. Its
        # only un-namespaced lookup is the final finger position pair, so provide
        # those names temporarily through an explicit local implementation.
        self._reconcile_grasp_state()
        base = self.base_xyz(); roll, pitch, yaw = self.base_rpy()
        red = self.body_xyz("red_block"); blue = self.body_xyz("blue_block"); yellow = self.body_xyz("yellow_block")
        color = self.grasp_color or "red"
        contact = self.finger_block_contact(color)
        block = {"red": red, "blue": blue, "yellow": yellow}[color]
        grip = self.site_xyz("grip_site")
        lg = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("left_finger"))
        rg = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, self._n("right_finger"))
        left_xyz = self.data.geom_xpos[lg].copy(); right_xyz = self.data.geom_xpos[rg].copy()
        # Keep the public shape intentionally close to MasterPiProductionV2 so
        # the current estimator/UI can consume each slot without a second schema.
        return {
            "robot_id": self.robot_id,
            "seed": int(self.seed),
            # Per-robot stream seed (world + slot) drives this robot's local
            # noise; world_seed is the replay key for the shared block layout.
            "world_seed": int(self._owner.seed),
            "model": "masterpi_multi_v2_shared_world",
            "physics_fidelity": (
                "V2_REAL_CALIBRATED_VALIDATED" if self.calibration_status == "REAL_CALIBRATED_VALIDATED"
                else "V2_REAL_FITTED_UNVALIDATED" if self.calibration_status == "REAL_FITTED_UNVALIDATED"
                else "V2_STRUCTURAL_UNCALIBRATED"
            ),
            "training_ready": self.calibration_status == "REAL_CALIBRATED_VALIDATED",
            "calibration_status": self.calibration_status,
            "calibration_parameters": dict(self.calibration_parameters),
            "physical_parameters": dict(self.physical_params),
            "camera_calibration_id": getattr(self, "camera_calibration_id", None),
            "camera_raw_fisheye": True,
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
            "left_finger_xyz": [round(float(v), 4) for v in left_xyz],
            "right_finger_xyz": [round(float(v), 4) for v in right_xyz],
            "grip_error_m": round(float(np.linalg.norm(block - grip)), 4),
            "left_contact": bool(contact["left"]), "right_contact": bool(contact["right"]),
            "left_normal_N": round(float(contact["left_force_n"]), 3), "right_normal_N": round(float(contact["right_force_n"]), 3),
            "bilateral_contact": bool(contact["bilateral"]),
            "lifted": bool(self.grasp_color is not None and block[2] > TARGET_BLOCK_LIFT_CENTER_M),
            "stable": bool(self.grasp_color is not None and contact["bilateral"] and block[2] > TARGET_BLOCK_LIFT_CENTER_M),
            "stable_steps": 0,
            "gripper_qpos": round(self._gripper_close_qpos(), 4),
            "arm_qpos": [round(float(v), 4) for v in self._arm_qpos()],
            "commanded_pwm": dict(self.servo_command_pulses),
            "motor_commands": [round(float(v), 4) for v in self.motor_command],
            "spatial_memory": self.spatial_memory_public(),
            "semantic_map": self.semantic_map_public(),
            "memory_observation_id": int(self._observation_id),
            "arm_dof": 4, "gripper_dof": 1,
        }

    def _real_action(self, action: str, **params) -> ActionResult:
        # precision_handoff/carry_handoff were originally process-global files.
        # Commands are serialized, so swapping their path at this boundary gives
        # every robot its own causal handoff without forking controller source.
        with self._owner.handoff_context(self.robot_id):
            return super()._real_action(action, **params)


class MultiMasterPiProductionV2:
    """One scene, three physical robot bodies, three production controller views."""

    robot_ids = ROBOT_IDS

    def __init__(
        self,
        seed: int = 11,
        width: int = 640,
        height: int = 480,
        render: bool = True,
        *,
        dynamics=None,
        hardware=None,
        use_calibration_manifest: bool = True,
        warehouse_layout: str = "standard",
        warehouse_cargo_ids: Sequence[str] | None = None,
        scene_objects=(),
        xml_transform=None,
    ):
        selected_cargo_ids = None
        if warehouse_cargo_ids is not None:
            if isinstance(warehouse_cargo_ids, (str, bytes)):
                raise ValueError("warehouse_cargo_ids must be a nonempty sequence of unique cargo IDs")
            selected_cargo_ids = tuple(warehouse_cargo_ids)
            if not selected_cargo_ids:
                raise ValueError("warehouse_cargo_ids must not be empty")
            if any(not isinstance(cargo_id, str) or not cargo_id for cargo_id in selected_cargo_ids):
                raise ValueError("warehouse_cargo_ids must contain nonempty strings")
            if len(set(selected_cargo_ids)) != len(selected_cargo_ids):
                raise ValueError("warehouse_cargo_ids must be unique")
        # Reuse the single-twin parameter loader rather than duplicating any
        # calibration policy in the multi-robot platform layer.
        template = MasterPiProductionV2(
            seed=seed, width=max(32, min(width, 160)), height=max(24, min(height, 120)), render=False,
            dynamics=dynamics, hardware=hardware, use_calibration_manifest=use_calibration_manifest,
        )
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)
        if warehouse_layout not in {"standard", "mixed", "arena", "camera_team"}:
            raise ValueError("unknown warehouse layout")
        self.warehouse_layout = warehouse_layout
        from sim.warehouse_mission import mixed_cargo_specs_for_seed, mixed_cargo_zone_positions_for_seed
        self.warehouse_zones = WAREHOUSE_ZONES
        self.warehouse_arena = None
        self.warehouse_source_zone = "A"
        self.warehouse_destination_zone = "B"
        self.warehouse_spawns = DEFAULT_SPAWNS
        self.warehouse_navigation_bounds = (-.60, 3.40, -2.30, 2.30)
        if warehouse_layout in {"arena", "camera_team"}:
            from sim.warehouse_arena import arena_for_seed, camera_team_arena_for_seed
            self.warehouse_arena = (camera_team_arena_for_seed(self.seed)
                                    if warehouse_layout == "camera_team" else arena_for_seed(self.seed))
            arena = self.warehouse_arena
            self.warehouse_zones = arena.zones
            self.warehouse_source_zone = arena.source_zone
            self.warehouse_destination_zone = arena.destination_zone
            self.warehouse_specs = arena.cargo_specs
            self.warehouse_zone_positions = arena.zone_positions
            self.warehouse_terrain_specs = arena.terrain
            self.warehouse_spawns = {rid: (pose[0], pose[1], WHEEL_RADIUS_M, pose[2])
                                     for rid, pose in arena.robot_start_poses.items()}
            self.warehouse_navigation_bounds = (
                min(z.center_xy[0]-z.half_extents_xy[0] for z in arena.zones.values())-1.,
                max(z.center_xy[0]+z.half_extents_xy[0] for z in arena.zones.values())+1.,
                min(z.center_xy[1]-z.half_extents_xy[1] for z in arena.zones.values())-1.,
                max(z.center_xy[1]+z.half_extents_xy[1] for z in arena.zones.values())+1.)
        else:
            self.warehouse_specs = (mixed_cargo_specs_for_seed(self.seed) if warehouse_layout == "mixed"
                                    else cargo_specs_for_seed(self.seed))
            positions_factory = mixed_cargo_zone_positions_for_seed if warehouse_layout == "mixed" else cargo_zone_positions_for_seed
            self.warehouse_zone_positions = positions_factory(self.seed, self.warehouse_specs)
            self.warehouse_terrain_specs = () if warehouse_layout == "mixed" else terrain_specs_for_seed(self.seed)
        if selected_cargo_ids is not None:
            specs_by_id = {spec.cargo_id: spec for spec in self.warehouse_specs}
            unknown = tuple(cargo_id for cargo_id in selected_cargo_ids if cargo_id not in specs_by_id)
            if unknown:
                template.close()
                raise ValueError(f"unknown warehouse cargo IDs: {', '.join(unknown)}")
            self.warehouse_specs = tuple(specs_by_id[cargo_id] for cargo_id in selected_cargo_ids)
            self.warehouse_zone_positions = {
                cargo_id: self.warehouse_zone_positions[cargo_id]
                for cargo_id in selected_cargo_ids
            }
            if self.warehouse_arena is not None:
                self.warehouse_arena = replace(
                    self.warehouse_arena,
                    cargo_specs=self.warehouse_specs,
                    zone_positions=self.warehouse_zone_positions,
                )
        self._warehouse_geometry_seed = self.seed
        self._warehouse_geometry_specs = self.warehouse_specs
        self.warehouse_spec_by_id = {spec.cargo_id: spec for spec in self.warehouse_specs}
        self._warehouse_goal_lock = threading.RLock()
        self._warehouse_requested_destination: str | None = None
        self._warehouse_goal_revision = 0
        self._warehouse_goal_reason = ""
        self._warehouse_carrier_jobs = {rid: 0 for rid in self.robot_ids}
        self.dynamics = dict(template.dynamics)
        self.physical_params = dict(template.physical_params)
        self.calibration_status = template.calibration_status
        self.calibration_parameters = dict(template.calibration_parameters)
        template.close()

        xml = build_multi_robot_xml(
            self.physical_params,
            warehouse_specs=self.warehouse_specs,
            warehouse_terrain=self.warehouse_terrain_specs,
            warehouse_zones=self.warehouse_zones,
            spawns=self.warehouse_spawns,
            navigation_camera=warehouse_layout == "camera_team",
            scene_objects=(),
        )
        if xml_transform is not None:
            xml = xml_transform(xml)
        if scene_objects:
            from sim.scene_objects import append_objects
            root = ET.fromstring(xml)
            append_objects(root.find('worldbody'), scene_objects)
            xml = ET.tostring(root, encoding='unicode')
        self.scene_xml = xml
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.width = int(width); self.height = int(height)
        # The browser displays these views at roughly VGA size. Rendering every
        # observer frame at 1280x720 made presentation consume enough GPU/CPU to
        # starve the shared MuJoCo action loop. Keep the default at the robot
        # camera resolution; deployments that truly need HD can still opt in
        # through the existing environment variables.
        self.observer_width = max(self.width, int(os.environ.get("UGRP_SIM_OBSERVER_WIDTH", "640")))
        self.observer_height = max(self.height, int(os.environ.get("UGRP_SIM_OBSERVER_HEIGHT", "480")))
        self.render_lock = threading.RLock()
        self.physics_lock = threading.RLock()
        self.command_lock = threading.RLock()
        self.renderer = None
        self.observer_renderer = None
        self._render_executor: ThreadPoolExecutor | None = None
        self._render_thread_id: int | None = None
        self._snapshot_broker: SnapshotRenderBroker | None = None
        self._snapshot_frame_id = 0
        self._render_closed = False
        self._snapshot_submit_lock = threading.Lock()
        if render:
            self._render_executor = ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix="ugrp-mujoco-render",
            )
            try:
                (
                    self.renderer,
                    self.observer_renderer,
                    self._render_thread_id,
                ) = self._render_executor.submit(self._create_renderers).result(timeout=30.0)
            except Exception:
                self._render_executor.shutdown(wait=False, cancel_futures=True)
                self._render_executor = None
                raise
        self.last_parallel_timing: dict[str, object] = {}
        self.frame_callback = None
        self._frame_callback_thread_id = threading.get_ident()
        self.speed_multiplier = 1.0
        self.controllers: dict[str, NamespacedMasterPi] = {}
        for index, rid in enumerate(self.robot_ids):
            self.controllers[rid] = self._bind_controller(rid, index)
        self.grasp_weld_ids = {
            (rid, color): mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{rid}__grasp_{color}"
            )
            for rid in self.robot_ids for color in ("red", "blue", "yellow")
        }
        self.beam_weld_ids = {
            rid: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{rid}__beam_grasp"
            )
            for rid in BEAM_CARRIER_IDS
        }
        self.beam_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, BEAM_BODY_NAME
        )
        self.beam_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "team_beam_free"
        )
        self.beam_transport_trace: list[dict[str, object]] = []
        self.beam_mission_status = "READY"
        self.warehouse_body_ids = {
            spec.cargo_id: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, spec.body_name
            )
            for spec in self.warehouse_specs
        }
        self.warehouse_joint_ids = {
            spec.cargo_id: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, spec.joint_name
            )
            for spec in self.warehouse_specs
        }
        self.warehouse_weld_ids = {
            (spec.cargo_id, rid): mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                f"{rid}__{spec.cargo_id}_grasp",
            )
            for spec in self.warehouse_specs
            for rid in self.robot_ids
        }
        self.warehouse_status = "READY"
        self.warehouse_trace: list[dict[str, object]] = []
        self.warehouse_mission: dict[str, object] | None = None
        self.reset(seed)

    def _create_renderers(self):
        """Create every OpenGL context on the thread that will own it."""
        renderer = mujoco.Renderer(
            self.model,
            height=self.height,
            width=self.width,
        )
        try:
            observer = mujoco.Renderer(
                self.model,
                height=self.observer_height,
                width=self.observer_width,
            )
        except Exception:
            renderer.close()
            if 'observer' in locals():
                observer.close()
            raise
        return renderer, observer, threading.get_ident()

    def _render_rgb_direct(self, robot: NamespacedMasterPi, camera: str) -> np.ndarray:
        """Render on the broker thread while holding one coherent physics view."""
        if threading.get_ident() != self._render_thread_id:
            raise RuntimeError("MuJoCo renderer accessed outside its owner thread")
        with self.physics_lock, self.render_lock:
            if self.renderer is None:
                raise RuntimeError("multi MasterPi world was created with render=False")
            if camera == "robot_cam":
                robot._sync_real_camera_mount()
                self.renderer.update_scene(
                    self.data,
                    camera=robot._n("robot_cam"),
                    scene_option=robot._robot_sensor_scene_option,
                )
                ideal = self.renderer.render().copy()
                if robot._robot_fisheye_map is None:
                    return ideal
                import cv2
                map_x, map_y = robot._robot_fisheye_map
                return cv2.remap(
                    ideal,
                    map_x,
                    map_y,
                    cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )
            if camera == "nav_cam":
                if self.warehouse_layout != "camera_team":
                    raise ValueError("nav_cam is available only in camera_team layout")
                self.renderer.update_scene(
                    self.data, camera=robot._n("nav_cam"),
                    scene_option=robot._robot_sensor_scene_option,
                )
                return self.renderer.render().copy()
            if camera == "cctv_front_left":
                robot._sync_front_follow_camera()
            observer = self.observer_renderer or self.renderer
            observer.update_scene(self.data, camera=camera)
            return observer.render().copy()

    def _render_rgb_for(self, robot: NamespacedMasterPi, camera: str) -> np.ndarray:
        executor = self._render_executor
        if executor is None or self.renderer is None:
            raise RuntimeError("multi MasterPi world was created with render=False")
        if threading.get_ident() == self._render_thread_id:
            return self._render_rgb_direct(robot, camera)
        return executor.submit(self._render_rgb_direct, robot, camera).result(timeout=30.0)

    def _render_team_rgb_direct(self, camera: str) -> np.ndarray:
        if threading.get_ident() != self._render_thread_id:
            raise RuntimeError("MuJoCo renderer accessed outside its owner thread")
        with self.physics_lock, self.render_lock:
            if self.observer_renderer is None:
                raise RuntimeError("multi MasterPi world was created with render=False")
            observer_options = mujoco.MjvOption()
            observer_options.geomgroup[:] = 1
            self.observer_renderer.update_scene(self.data, camera=camera, scene_option=observer_options)
            return self.observer_renderer.render().copy()

    def _close_renderers(self) -> None:
        if threading.get_ident() != self._render_thread_id:
            raise RuntimeError("MuJoCo renderer closed outside its owner thread")
        first_error = None
        for renderer in (self.renderer, self.observer_renderer, self._snapshot_broker):
            if renderer is None:
                continue
            try:
                renderer.close()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

    def _bind_controller(self, rid: str, index: int) -> NamespacedMasterPi:
        c = object.__new__(NamespacedMasterPi)
        c._owner = self; c.robot_id = rid
        c.seed = self.seed + index
        c.rng = np.random.default_rng(c.seed)
        c.dynamics = self.dynamics
        c.physical_params = self.physical_params
        c.calibration_status = self.calibration_status
        c.calibration_parameters = self.calibration_parameters
        c.model = self.model; c.data = self.data
        c.width = self.width; c.height = self.height
        c.renderer = self.renderer; c.observer_renderer = self.observer_renderer
        c.observer_width = self.observer_width; c.observer_height = self.observer_height
        c.robot_bid = c._body("robot"); c.gripper_bid = c._body("gripper")
        c.robot_cam_cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, c._n("robot_cam"))
        c.front_cam_cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_front_left")
        for geom_name in ("camera_bracket", "camera_body", "camera_lens_visual"):
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, c._n(geom_name))
            if gid >= 0:
                self.model.geom_group[gid] = 5
        c._robot_sensor_scene_option = None
        if self.renderer is not None:
            c._robot_sensor_scene_option = mujoco.MjvOption(); c._robot_sensor_scene_option.geomgroup[:] = 1
            # Group 4 contains mission-only floor overlays (start/goal zones),
            # while group 5 contains the camera's own hardware. Both belong in
            # the observer/UI view, never in actor-visible robot pixels.
            c._robot_sensor_scene_option.geomgroup[4] = 0
            c._robot_sensor_scene_option.geomgroup[5] = 0
        c._robot_fisheye_map = raw_fisheye_remap(self.width, self.height) if self.renderer is not None else None
        c._configure_measured_robot_camera()
        c.base_jid = c._joint("base_free"); c.base_qadr = int(self.model.jnt_qposadr[c.base_jid]); c.base_dadr = int(self.model.jnt_dofadr[c.base_jid])
        c.wheel_act = np.array([c._actuator(f"wheel_{n}_drive") for n in c.wheel_names], dtype=int)
        c.servo_act = {k: c._actuator(v) for k, v in {"yaw":"servo_arm_yaw", "shoulder":"servo_shoulder", "elbow":"servo_elbow", "wrist":"servo_wrist"}.items()}
        c.gripper_act = (c._actuator("servo_gripper_left"), c._actuator("servo_gripper_right"))
        c.arm_joint = {k: c._joint(v) for k, v in {"yaw":"arm_yaw", "shoulder":"shoulder", "elbow":"elbow", "wrist":"wrist_pitch"}.items()}
        c.gripper_joint = (c._joint("left_gripper_close"), c._joint("right_gripper_close"))
        c.motor_state = np.zeros(4, dtype=float); c.motor_command = np.zeros(4, dtype=float)
        c.servo_command_pulses = {}
        c.speed_multiplier = 1.0; c.frame_callback = None; c._frame_step = 0
        c.grasp_color = None; c.pregrasp_color = None; c.spatial_memory = {}; c._observation_id = 0; c._real_stack = None
        c._latest_robot_bgr = None; c._latest_robot_frame_seq = 0
        c._latest_robot_jpeg = None; c._latest_robot_jpeg_seq = None; c._latest_robot_jpeg_quality = None
        c._presentation_dirty = True
        # Public metadata used by the copied single-robot state schema.
        from sim.masterpi_camera_profile import CAMERA_CALIBRATION_ID
        c.camera_calibration_id = CAMERA_CALIBRATION_ID
        return c

    @contextmanager
    def handoff_context(self, robot_id: str):
        from sim import real_stack_adapter as rsa
        rid = self.normalize_robot_id(robot_id)
        plan_path = Path(f"/tmp/ugrp-sim-{rid}-pick-plan.json")
        carry_path = Path(f"/tmp/ugrp-sim-{rid}-carry-handoff.json")
        # Handoff modules provide ContextVar-backed path overrides.  Keep the
        # compatibility files robot-local without serializing the entire REAL
        # skill behind one world-wide lock; only the shared MuJoCo physics step
        # remains synchronized at the world boundary.
        with (
            rsa.precision_handoff.use_plan_path(plan_path),
            rsa.carry_handoff_mod.use_carry_path(carry_path),
        ):
            yield

    @staticmethod
    def normalize_robot_id(robot_id: object) -> str:
        rid = str(robot_id or "r1").strip().lower()
        aliases = {"1":"r1", "2":"r2", "3":"r3", "robot1":"r1", "robot2":"r2", "robot3":"r3"}
        rid = aliases.get(rid, rid)
        if rid not in ROBOT_IDS:
            raise ValueError("robot_id must be r1, r2, or r3")
        return rid

    def robot(self, robot_id: object = "r1") -> NamespacedMasterPi:
        return self.controllers[self.normalize_robot_id(robot_id)]

    def _grasp_constraint_id(self, robot: NamespacedMasterPi, color: str) -> int:
        key = (robot.robot_id, str(color))
        eid = int(self.grasp_weld_ids.get(key, -1))
        if eid < 0:
            raise RuntimeError(f"missing SIM grasp constraint for {key[0]}/{key[1]}")
        return eid

    def _grasp_constraint_active(self, robot: NamespacedMasterPi, color: str) -> bool:
        return bool(self.data.eq_active[self._grasp_constraint_id(robot, color)])

    def _activate_grasp_constraint(self, robot: NamespacedMasterPi, color: str) -> None:
        with self.physics_lock:
            contact = robot.finger_block_contact(color)
            if not bool(contact.get("bilateral")):
                raise RuntimeError(f"refusing SIM grasp lock without bilateral {color} contact")
            eid = self._grasp_constraint_id(robot, color)
            block_bid = robot._body(f"{color}_block")
            inverse_pos = np.zeros(3, dtype=float)
            inverse_quat = np.zeros(4, dtype=float)
            relative_pos = np.zeros(3, dtype=float)
            relative_quat = np.zeros(4, dtype=float)
            mujoco.mju_negPose(
                inverse_pos, inverse_quat,
                self.data.xpos[robot.gripper_bid], self.data.xquat[robot.gripper_bid],
            )
            mujoco.mju_mulPose(
                relative_pos, relative_quat,
                inverse_pos, inverse_quat,
                self.data.xpos[block_bid], self.data.xquat[block_bid],
            )
            self.model.eq_data[eid, 3:6] = relative_pos
            self.model.eq_data[eid, 6:10] = relative_quat
            self.data.eq_active[eid] = 1
            mujoco.mj_forward(self.model, self.data)

    def _release_grasp_constraint(self, robot: NamespacedMasterPi, color: str) -> None:
        with self.physics_lock:
            eid = self._grasp_constraint_id(robot, color)
            self.data.eq_active[eid] = 0
            mujoco.mj_forward(self.model, self.data)

    def _beam_constraint_active(self, rid: str) -> bool:
        return bool(self.data.eq_active[int(self.beam_weld_ids[str(rid)])])

    def _activate_beam_constraint(self, robot: NamespacedMasterPi) -> None:
        """Weld one verified endpoint; both carriers are required before lift."""
        rid = robot.robot_id
        if rid not in BEAM_CARRIER_IDS:
            raise RuntimeError(f"{rid} is not a beam carrier")
        contact = robot.finger_payload_contact(rid)
        if not bool(contact.get("bilateral")):
            raise RuntimeError(f"CARRIER_CONTACT_MISSING: {rid} beam handle bilateral=false")
        with self.physics_lock:
            eid = int(self.beam_weld_ids[rid])
            endpoint_bid = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                f"{BEAM_BODY_NAME}_{rid}_endpoint",
            )
            inverse_pos = np.zeros(3, dtype=float)
            inverse_quat = np.zeros(4, dtype=float)
            relative_pos = np.zeros(3, dtype=float)
            relative_quat = np.zeros(4, dtype=float)
            mujoco.mju_negPose(
                inverse_pos, inverse_quat,
                self.data.xpos[robot.gripper_bid], self.data.xquat[robot.gripper_bid],
            )
            mujoco.mju_mulPose(
                relative_pos, relative_quat,
                inverse_pos, inverse_quat,
                self.data.xpos[endpoint_bid], self.data.xquat[endpoint_bid],
            )
            self.model.eq_data[eid, 3:6] = relative_pos
            self.model.eq_data[eid, 6:10] = relative_quat
            self.data.eq_active[eid] = 1
            mujoco.mj_forward(self.model, self.data)

    def _release_beam_constraints(self) -> None:
        with self.physics_lock:
            for eid in self.beam_weld_ids.values():
                self.data.eq_active[int(eid)] = 0
            mujoco.mj_forward(self.model, self.data)

    def beam_state(self) -> dict[str, object]:
        with self.physics_lock:
            pose = beam_pose(self.data, self.model)
            dadr = int(self.model.jnt_dofadr[self.beam_joint_id])
            velocity = np.asarray(self.data.qvel[dadr:dadr + 6], dtype=float)
            contacts = {
                rid: self.controllers[rid].finger_payload_contact(rid)
                for rid in BEAM_CARRIER_IDS
            }
            evaluation = evaluate_beam_mission(pose)
            return {
                **pose,
                "mission_id": "beam_transport_v1",
                "object_type": "beam",
                "start_zone": {"center": list(BEAM_START)},
                "goal_zone": {"center": list(BEAM_GOAL), "yaw": BEAM_GOAL_YAW_RAD},
                "status": self.beam_mission_status,
                "constraints_active": {
                    rid: self._beam_constraint_active(rid) for rid in BEAM_CARRIER_IDS
                },
                "contacts": contacts,
                "linear_speed_mps": round(float(np.linalg.norm(velocity[:3])), 4),
                "angular_speed_radps": round(float(np.linalg.norm(velocity[3:])), 4),
                "evaluation": evaluation,
                "trace": list(self.beam_transport_trace[-24:]),
            }

    def _cargo_constraint_active(self, cargo_id: str, rid: str) -> bool:
        return bool(self.data.eq_active[int(self.warehouse_weld_ids[(cargo_id, rid)])])

    def _activate_cargo_constraint(self, cargo_id: str, robot: NamespacedMasterPi) -> None:
        spec = self.warehouse_spec_by_id[cargo_id]
        rid = robot.robot_id
        if rid not in spec.carriers:
            raise RuntimeError(f"{rid} has no runtime carrier award for {cargo_id}")
        contact = robot.finger_cargo_contact(cargo_id)
        if not bool(contact.get("bilateral")):
            raise RuntimeError(f"CARRIER_CONTACT_MISSING: {cargo_id}/{rid}")
        cargo_bid = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            spec.body_name,
        )
        with self.physics_lock:
            eid = int(self.warehouse_weld_ids[(cargo_id, rid)])
            inverse_pos = np.zeros(3, dtype=float)
            inverse_quat = np.zeros(4, dtype=float)
            relative_pos = np.zeros(3, dtype=float)
            relative_quat = np.zeros(4, dtype=float)
            mujoco.mju_negPose(
                inverse_pos, inverse_quat,
                self.data.xpos[robot.gripper_bid], self.data.xquat[robot.gripper_bid],
            )
            mujoco.mju_mulPose(
                relative_pos, relative_quat,
                inverse_pos, inverse_quat,
                self.data.xpos[cargo_bid], self.data.xquat[cargo_bid],
            )
            self.model.eq_data[eid, 3:6] = relative_pos
            self.model.eq_data[eid, 6:10] = relative_quat
            self.data.eq_active[eid] = 1
            mujoco.mj_forward(self.model, self.data)

    def _release_cargo_constraints(self, cargo_id: str) -> None:
        with self.physics_lock:
            for rid in self.robot_ids:
                self.data.eq_active[int(self.warehouse_weld_ids[(cargo_id, rid)])] = 0
            mujoco.mj_forward(self.model, self.data)

    def _observe_warehouse_cargo(self, spec) -> CargoObservation:
        """Measure geometry and pose from the live SIM scene, not role fixtures."""
        pose = cargo_pose(self.data, self.model, spec)
        geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{spec.body_name}_geom",
        )
        body_id = int(self.warehouse_body_ids[spec.cargo_id])
        size = np.asarray(self.model.geom_size[geom_id], dtype=float)
        # Warehouse collision geoms are boxes even when a separate cylinder is
        # rendered for a pipe. MuJoCo stores box half extents in geom_size.
        dimensions = (float(size[0] * 2.0), float(size[1] * 2.0), float(size[2] * 2.0))
        return CargoObservation(
            cargo_id=spec.cargo_id,
            position_xy=tuple(float(v) for v in pose["position"][:2]),
            yaw_rad=float(pose["yaw"]),
            dimensions_m=dimensions,
            mass_kg=float(self.model.body_mass[body_id]),
        )

    def observe_warehouse_scene(self) -> dict[str, object]:
        """Public runtime snapshot used by the adaptive team planner."""
        with self.physics_lock:
            cargo = {}
            for spec in self.warehouse_specs:
                observed = self._observe_warehouse_cargo(spec)
                pose = cargo_pose(self.data, self.model, spec)
                cargo[spec.cargo_id] = {
                    "cargo_id": observed.cargo_id,
                    "position_xy": observed.position_xy,
                    "yaw_rad": observed.yaw_rad,
                    "dimensions_m": observed.dimensions_m,
                    "mass_kg": observed.mass_kg,
                    "current_zone": self._warehouse_zone_for_position(
                        spec.cargo_id, pose["position"],
                    ),
                    "source": "sim_geometry_observation",
                }
            robots = {
                rid: {
                    "robot_id": rid,
                    "position_xy": tuple(float(v) for v in robot.base_xyz()[:2]),
                }
                for rid, robot in self.controllers.items()
            }
            terrain = [
                {
                    "terrain_id": item.terrain_id,
                    "kind": item.kind,
                    "center_xy": item.center_xy,
                    "half_extents_xy": item.half_extents_xy,
                    "height_m": item.height_m,
                    "traversable": item.traversable,
                    "cost_multiplier": item.cost_multiplier,
                    "source": "sim_geometry_observation",
                }
                for item in self.warehouse_terrain_specs
            ]
            destination, revision, reason = self._warehouse_goal_snapshot()
            return {
                "observation_id": len(self.warehouse_trace) + 1,
                "sim_time": round(float(self.data.time), 4),
                "cargo": cargo,
                "robots": robots,
                "terrain": terrain,
                "goal": {
                    "destination_zone": destination,
                    "revision": revision,
                    "reason": reason,
                },
            }

    def _warehouse_terrain_observations(self) -> tuple[TerrainObservation, ...]:
        return tuple(
            TerrainObservation(
                terrain_id=item.terrain_id,
                kind=item.kind,
                center_xy=item.center_xy,
                half_extents_xy=item.half_extents_xy,
                height_m=item.height_m,
                traversable=item.traversable,
                cost_multiplier=item.cost_multiplier,
            )
            for item in self.warehouse_terrain_specs
        )

    def _plan_warehouse_roles(self, spec):
        observed = self._observe_warehouse_cargo(spec)
        robots = tuple(
            RobotObservation(
                robot_id=rid,
                position_xy=tuple(float(v) for v in self.controllers[rid].base_xyz()[:2]),
            )
            for rid in self.robot_ids
        )
        return auction_roles(
            robots, observed, prior_carrier_jobs=self._warehouse_carrier_jobs,
        )

    def _award_warehouse_roles(self, spec, plan=None):
        plan = plan or self._plan_warehouse_roles(spec)
        for rid, values in plan.bids.items():
            self._record_warehouse_phase(
                "team_bid_submitted",
                cargo_id=spec.cargo_id,
                sender=rid,
                recipient="team",
                bid={role: round(value, 4) for role, value in values.items()},
                evidence="current robot pose + observed cargo endpoints",
            )
        awarded = replace(spec, carriers=plan.carriers, scout=plan.scout)
        self.warehouse_spec_by_id[spec.cargo_id] = awarded
        self.warehouse_specs = tuple(
            awarded if current.cargo_id == spec.cargo_id else current
            for current in self.warehouse_specs
        )
        for rid in plan.carriers:
            self._warehouse_carrier_jobs[rid] += 1
        self._record_warehouse_phase(
            "role_auction_completed",
            cargo_id=spec.cargo_id,
            bids={
                rid: {role: round(value, 4) for role, value in values.items()}
                for rid, values in plan.bids.items()
            },
            carriers=list(plan.carriers),
            scout=plan.scout,
            total_cost=round(plan.total_cost, 4),
            observation_source="sim_geometry_observation",
        )
        return awarded, plan.total_cost

    def request_warehouse_goal_update(
        self,
        destination_zone: str,
        *,
        revision: int | None = None,
        reason: str = "external_update",
    ) -> dict[str, object]:
        """Change a running mission goal; the executor replans at safe barriers."""
        destination = str(destination_zone).upper()
        if destination not in WAREHOUSE_ZONES:
            raise ValueError("RETARGET_REJECTED: unknown warehouse zone")
        with self._warehouse_goal_lock:
            requested_revision = (
                self._warehouse_goal_revision + 1 if revision is None else int(revision)
            )
            if requested_revision <= self._warehouse_goal_revision:
                return {
                    "accepted": False,
                    "destination_zone": self._warehouse_requested_destination,
                    "revision": self._warehouse_goal_revision,
                    "reason": "STALE_GOAL_REVISION",
                }
            previous = self._warehouse_requested_destination
            self._warehouse_requested_destination = destination
            self._warehouse_goal_revision = requested_revision
            self._warehouse_goal_reason = str(reason)
        if self.warehouse_mission is not None:
            self.warehouse_mission = {
                **self.warehouse_mission,
                "destination_zone": destination,
                "goal_revision": requested_revision,
            }
        self._record_warehouse_phase(
            "mission_retargeted",
            previous_destination=previous,
            destination_zone=destination,
            revision=requested_revision,
            reason=str(reason),
        )
        return {
            "accepted": True,
            "destination_zone": destination,
            "revision": requested_revision,
            "reason": str(reason),
        }

    def _warehouse_goal_snapshot(self) -> tuple[str | None, int, str]:
        with self._warehouse_goal_lock:
            return (
                self._warehouse_requested_destination,
                int(self._warehouse_goal_revision),
                self._warehouse_goal_reason,
            )

    def _warehouse_motion_metrics(self) -> dict[str, object]:
        loaded_forward = 0.0
        loaded_lateral = 0.0
        lateral_segments = 0
        empty_forward_segments = 0
        empty_lateral_segments = 0
        for event in self.warehouse_trace:
            phase = str(event.get("phase") or "")
            if phase.endswith("_complete"):
                continue
            mode = str(event.get("movement_mode") or "")
            if "payload_start" in event and "payload_target" in event:
                distance = abs(float(event["payload_target"]) - float(event["payload_start"]))
                if mode == "mecanum_lateral":
                    loaded_lateral += distance
                    lateral_segments += 1
                elif mode == "wheel_longitudinal":
                    loaded_forward += distance
            if mode == "rotate_then_forward" and not phase.endswith("_turn"):
                empty_forward_segments += 1
            if mode == "mecanum_lateral" and (
                "free_space_return" in phase or "final_approach" in phase
            ):
                empty_lateral_segments += 1
        loaded_total = loaded_forward + loaded_lateral
        return {
            "loaded_forward_m": round(loaded_forward, 4),
            "loaded_lateral_m": round(loaded_lateral, 4),
            "loaded_lateral_ratio": round(
                loaded_lateral / max(1e-9, loaded_total), 4,
            ),
            "loaded_lateral_segments": lateral_segments,
            "empty_rotate_forward_segments": empty_forward_segments,
            "empty_lateral_segments": empty_lateral_segments,
        }

    def _replace_warehouse_spec(self, updated) -> None:
        self.warehouse_spec_by_id[updated.cargo_id] = updated
        self.warehouse_specs = tuple(
            updated if current.cargo_id == updated.cargo_id else current
            for current in self.warehouse_specs
        )

    def _sync_warehouse_goal_specs(
        self, cargo_ids: Sequence[str], destination: str,
    ) -> None:
        for cargo_id in cargo_ids:
            current = self.warehouse_spec_by_id[cargo_id]
            goal = self.warehouse_zone_positions[cargo_id][destination]
            self._replace_warehouse_spec(replace(current, goal_xyz=goal))

    def _warehouse_zone_for_position(
        self, cargo_id: str, position: Sequence[float],
    ) -> str:
        spec = self.warehouse_spec_by_id[cargo_id]
        half_x, half_y = self._warehouse_cargo_half_extents(spec)
        for zone_id, zone in self.warehouse_zones.items():
            if (
                abs(float(position[0]) - float(zone.center_xy[0]))
                <= float(zone.half_extents_xy[0]) - half_x
                and abs(float(position[1]) - float(zone.center_xy[1]))
                <= float(zone.half_extents_xy[1]) - half_y
            ):
                return str(zone_id)
        return "TRANSIT"

    def warehouse_state(self) -> dict[str, object]:
        with self.physics_lock:
            cargo: dict[str, object] = {}
            delivered: list[str] = []
            for spec in self.warehouse_specs:
                pose = cargo_pose(self.data, self.model, spec)
                evaluation = evaluate_cargo_delivery(pose, spec)
                jadr = int(self.model.jnt_dofadr[self.warehouse_joint_ids[spec.cargo_id]])
                velocity = np.asarray(self.data.qvel[jadr:jadr + 6], dtype=float)
                stable = (
                    float(np.linalg.norm(velocity[:3])) <= 0.01
                    and float(np.linalg.norm(velocity[3:])) <= 0.03
                )
                if bool(evaluation["success"]) and stable:
                    delivered.append(spec.cargo_id)
                actual_zone = self._warehouse_zone_for_position(
                    spec.cargo_id, pose["position"],
                )
                cargo[spec.cargo_id] = {
                    "cargo_id": spec.cargo_id,
                    "type": spec.cargo_type,
                    "label_color": spec.label_color,
                    "position": pose["position"],
                    "yaw": pose["yaw"],
                    "carriers": list(spec.carriers),
                    "scout": spec.scout,
                    "constraints_active": {
                        rid: self._cargo_constraint_active(spec.cargo_id, rid)
                        for rid in spec.carriers
                    },
                    "linear_speed_mps": round(float(np.linalg.norm(velocity[:3])), 4),
                    "angular_speed_radps": round(float(np.linalg.norm(velocity[3:])), 4),
                    "stable": stable,
                    "evaluation": evaluation,
                    "zone": actual_zone,
                }
            return {
                "mission": self.warehouse_mission,
                "current_goal": {
                    "destination_zone": self._warehouse_goal_snapshot()[0],
                    "revision": self._warehouse_goal_snapshot()[1],
                    "reason": self._warehouse_goal_snapshot()[2],
                },
                "status": self.warehouse_status,
                "manifest": warehouse_manifest(
                    self.warehouse_specs, self.warehouse_zone_positions, self.warehouse_zones,
                ),
                "cargo": cargo,
                "delivered_ids": delivered,
                "remaining_ids": [s.cargo_id for s in self.warehouse_specs if s.cargo_id not in delivered],
                "moved_count": len(delivered),
                "total_count": len(self.warehouse_specs),
                "success": len(delivered) == len(self.warehouse_specs),
                "motion_metrics": self._warehouse_motion_metrics(),
                "crew_motion": getattr(self, "_warehouse_crew_metrics", {}),
                "crew_activity": dict(getattr(getattr(self, "_warehouse_crew", None), "activities", {})),
                "mixed_tasks": (getattr(self, "_mixed_engine", None).status()
                                if getattr(self, "_mixed_engine", None) is not None else None),
                "trace": list(self.warehouse_trace[-500:]),
            }

    def _physics_step_for(self, active: NamespacedMasterPi, commands: np.ndarray | None = None) -> None:
        engine=getattr(self,"_mixed_engine",None)
        if engine is not None:engine.service_joint_pause()
        with self.physics_lock:
            dt = float(self.model.opt.timestep)
            if commands is not None:
                active.motor_command[:] = np.clip(np.asarray(commands, dtype=float), -1.0, 1.0)
            crew = getattr(self, "_warehouse_crew", None)
            if crew is not None and not (engine is not None and engine._pause_tick):
                crew.before_step()
            mixed = getattr(self, "_mixed_engine", None)
            if mixed is not None:
                mixed.before_step()
            fast_drive = getattr(self, "_fast_drive_kernel", None)
            if fast_drive is not None:
                fast_drive.apply(dt)
            else:
                self.data.xfrc_applied[:, :] = 0.0
                for c in self.controllers.values():
                    alpha = 1.0 - math.exp(-dt / c.dynamics["motor_time_constant_s"])
                    c.motor_state += alpha * (c.motor_command - c.motor_state)
                    self.data.ctrl[c.wheel_act] = c.motor_state * MAX_WHEEL_RAD_S
                    fwd = float(np.dot(c.motor_state, FORWARD_PATTERN) / 4.0)
                    left = float(np.dot(c.motor_state, LEFT_PATTERN) / 4.0)
                    yaw_cmd = float(np.dot(c.motor_state, YAW_LEFT_PATTERN) / 4.0)
                    qvel = self.data.qvel[c.base_dadr:c.base_dadr + 6]
                    vx_w, vy_w, wz = float(qvel[0]), float(qvel[1]), float(qvel[5])
                    _, _, yaw = c.base_rpy(); cy, sy = math.cos(yaw), math.sin(yaw)
                    vx_local = cy * vx_w + sy * vy_w; vy_local = -sy * vx_w + cy * vy_w
                    stopped = float(np.max(np.abs(c.motor_command))) < 1e-6
                    ld = c.dynamics["stop_linear_damping_n_per_mps" if stopped else "linear_damping_n_per_mps"]
                    yd = c.dynamics["stop_yaw_damping_nm_per_radps" if stopped else "yaw_damping_nm_per_radps"]
                    fx_l = c.dynamics["max_forward_force_n"] * fwd - ld * vx_local
                    fy_l = c.dynamics["max_lateral_force_n"] * left - ld * vy_local
                    tz = c.dynamics["max_yaw_torque_nm"] * yaw_cmd - yd * wz
                    self.data.xfrc_applied[c.robot_bid, 0] = cy * fx_l - sy * fy_l
                    self.data.xfrc_applied[c.robot_bid, 1] = sy * fx_l + cy * fy_l
                    self.data.xfrc_applied[c.robot_bid, 5] = tz
            mujoco.mj_step(self.model, self.data)
            for c in self.controllers.values():
                c._presentation_dirty = True
        if crew is not None:
            crew.after_step()
        if mixed is not None:
            mixed.after_step()

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return (float(value) + math.pi) % (2.0 * math.pi) - math.pi

    def _robot_geoms(self, rid: str) -> np.ndarray:
        cache = getattr(self, "_robot_geom_cache", None)
        if cache is None:
            cache = self._robot_geom_cache = {}
        if rid not in cache:
            prefix = f"{rid}__"
            ids = [
                g for g in range(self.model.ngeom)
                if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[g])) or "").startswith(prefix)
            ]
            cache[rid] = np.asarray(ids, dtype=int)
        return cache[rid]

    def nearest_peer(self, robot: NamespacedMasterPi) -> tuple[str, float]:
        """Peer id and the smallest geom-to-geom gap (m) between it and ``robot``.

        Uses geom bounding spheres, so the gap is conservative (never larger
        than the true surface gap) and includes the arm and a welded block.
        """
        with self.physics_lock:
            mine = self._robot_geoms(robot.robot_id)
            my_pos = self.data.geom_xpos[mine].copy()
            my_rb = self.model.geom_rbound[mine]
            nearest, distance = "", math.inf
            for rid, other in self.controllers.items():
                if other is robot:
                    continue
                theirs = self._robot_geoms(rid)
                diff = my_pos[:, None, :] - self.data.geom_xpos[theirs][None, :, :]
                gaps = np.linalg.norm(diff, axis=2) - my_rb[:, None] - self.model.geom_rbound[theirs][None, :]
                d = float(gaps.min())
                if d < distance:
                    nearest, distance = rid, d
            return nearest, distance

    def _refuse_if_peer_close(self, robot: NamespacedMasterPi, action: str, limit: float) -> None:
        peer, distance = self.nearest_peer(robot)
        if peer and distance < float(limit):
            robot.set_motor_commands(STOP)
            raise PeerTooClose(
                f"PEER_TOO_CLOSE: {robot.robot_id} {action} refused, gap to peer {peer} is "
                f"{max(0.0, distance):.3f} m (< {float(limit):.2f} m); wait for it to clear"
            )

    def _motion_pulse(self, robot: NamespacedMasterPi, pattern: np.ndarray, duration_s: float) -> None:
        pattern = np.asarray(pattern, dtype=float)
        lateral_component = float(np.dot(pattern, LEFT_PATTERN) / 4.0)
        if abs(lateral_component) > 1e-9:
            raise RuntimeError("TEAM pure lateral wheel motion is disabled; use turn+straight repositioning")
        duration = float(duration_s)
        scale = 1.0
        if duration < 0.035:
            # Below the shortest pulse the fixed-speed wheels move ~6 mm / ~1.1
            # deg, more than the axis and yaw tolerances, so every small
            # correction overshot to the other side and the loop limit-cycled.
            # Shorten the stroke by lowering the wheel speed instead.
            scale = max(0.2, duration / 0.035)
            duration = 0.035
        duration = min(0.18, duration)
        robot.step(motor_commands=pattern * TEAM_MOTOR_SCALE * scale, duration_s=duration)
        self._settle(robot)
        self._refuse_if_peer_close(robot, "motion", TEAM_PEER_HARD_CLEARANCE_M)

    def _settle(self, robot: NamespacedMasterPi, *, max_s: float = 0.9) -> None:
        # A fixed 0.12 s stop left the chassis coasting ~1 cm into the next
        # measurement/turn, which turned every axis correction into a limit
        # cycle. Hold STOP until the base is actually at rest before measuring.
        robot.step(motor_commands=STOP, duration_s=0.10)
        waited = 0.10
        while waited < max_s:
            qvel = self.data.qvel[robot.base_dadr:robot.base_dadr + 6]
            if float(np.hypot(qvel[0], qvel[1])) < 0.004 and abs(float(qvel[5])) < math.radians(1.0):
                return
            robot.step(motor_commands=STOP, duration_s=0.04)
            waited += 0.04

    def _turn_to_yaw(
        self,
        robot: NamespacedMasterPi,
        target_yaw: float,
        *,
        held_color: str | None = None,
        tolerance_deg: float = 1.0,
        max_pulses: int = 36,
    ) -> None:
        tolerance = math.radians(float(tolerance_deg))
        for _ in range(max_pulses):
            yaw_error = self._wrap_angle(float(target_yaw) - robot.base_rpy()[2])
            if abs(yaw_error) <= tolerance:
                return
            self._motion_pulse(
                robot,
                YAW_LEFT_PATTERN if yaw_error > 0.0 else -YAW_LEFT_PATTERN,
                abs(yaw_error) / math.radians(32.0),
            )
            if held_color is not None:
                self._require_held(robot, held_color)
        raise RuntimeError(
            f"{robot.robot_id} could not turn to yaw={math.degrees(target_yaw):.1f}deg without strafe"
        )

    def _require_gripped(self, robot: NamespacedMasterPi, color: str) -> None:
        if self._grasp_constraint_active(robot, color):
            return
        contact = robot.finger_block_contact(color)
        if not bool(contact.get("bilateral")):
            raise RuntimeError(
                f"{robot.robot_id} lost gripper contact with {color} "
                f"(bilateral={bool(contact.get('bilateral'))})"
            )

    def _require_held(self, robot: NamespacedMasterPi, color: str) -> None:
        self._require_gripped(robot, color)
        block_z = float(robot.body_xyz(f"{color}_block")[2])
        if block_z <= TARGET_BLOCK_LIFT_CENTER_M:
            raise RuntimeError(
                f"{robot.robot_id} lost {color} while carrying (z={block_z:.3f}m)"
            )

    def _move_axis(
        self,
        robot: NamespacedMasterPi,
        axis: str,
        target: float,
        *,
        tracked_body: str | None = None,
        held_color: str | None = None,
        tolerance: float = TEAM_AXIS_TOL_M,
        max_pulses: int = 90,
    ) -> None:
        if axis not in {"x", "y"}:
            raise ValueError(f"unsupported TEAM axis {axis!r}")
        index = 0 if axis == "x" else 1
        rate = 0.165
        # A carried block sits ~0.19 m ahead of the base, so 1 deg of yaw
        # already moves it 3.3 mm: measure it only from a tighter heading.
        yaw_tol_deg = 0.4 if tracked_body else 1.0
        if tracked_body is None and axis == "y":
            # Coarse phase for base-only lateral moves: face the travel direction
            # once and close most of the distance in place. The fine phase below
            # still re-faces +X before every measurement because a 90-degree turn
            # translates the base origin by the CoM offset (arm extended), which
            # only the measure-turn-pulse-turn-back cycle cancels out.
            error0 = float(target) - float(robot.base_xyz()[index])
            coarse_tol = max(float(tolerance), 0.02)
            if abs(error0) > coarse_tol:
                travel_yaw = math.pi / 2.0 if error0 > 0.0 else -math.pi / 2.0
                self._turn_to_yaw(robot, travel_yaw)
                for _ in range(max_pulses):
                    error = float(target) - float(robot.base_xyz()[index])
                    if abs(error) <= coarse_tol:
                        break
                    forward = FORWARD_PATTERN if (error > 0.0) == (travel_yaw > 0.0) else -FORWARD_PATTERN
                    self._motion_pulse(robot, forward, abs(error) / rate)
        for _ in range(max_pulses):
            # Every correction starts and ends facing world +X. For Y motion we
            # temporarily face +/-90 degrees and drive straight; no mecanum
            # lateral wheel pattern is ever issued. Re-facing before measuring
            # again also accounts for the gripper arc while carrying a block.
            self._turn_to_yaw(robot, 0.0, held_color=held_color, tolerance_deg=yaw_tol_deg)
            position = robot.body_xyz(tracked_body) if tracked_body else robot.base_xyz()
            error = float(target) - float(position[index])
            if abs(error) <= float(tolerance):
                return

            if axis == "y":
                travel_yaw = math.pi / 2.0 if error > 0.0 else -math.pi / 2.0
                self._turn_to_yaw(robot, travel_yaw, held_color=held_color)
                # Do NOT re-measure here. Turning translates the base origin
                # (and swings a held block) by an offset that the return turn
                # undoes; a pulse sized from the +X measurement is what lands
                # on target once the robot faces +X again. Re-measuring in the
                # travel heading produced a permanent limit cycle instead.
                forward = FORWARD_PATTERN if (error > 0.0) == (travel_yaw > 0.0) else -FORWARD_PATTERN
            else:
                forward = FORWARD_PATTERN if error > 0.0 else -FORWARD_PATTERN

            self._motion_pulse(robot, forward, abs(error) / rate)
            if held_color is not None:
                self._require_held(robot, held_color)

        self._turn_to_yaw(robot, 0.0, held_color=held_color, tolerance_deg=yaw_tol_deg)
        position = robot.body_xyz(tracked_body) if tracked_body else robot.base_xyz()
        raise RuntimeError(
            f"{robot.robot_id} geometric {axis}-move did not converge without strafe: "
            f"target={target:.4f} current={float(position[index]):.4f}"
        )

    @staticmethod
    def _precision_module():
        from sim import real_stack_adapter as rsa
        return rsa.precision_mod

    def _hover_pose(self) -> dict[int, int]:
        precision = self._precision_module()
        pose, _ = precision.solve_hover_ik(precision.CAPTURE_FINGERTIP_RADIUS_CM, 8.0)
        return pose

    def _geometric_approach(
        self,
        robot: NamespacedMasterPi,
        color: str,
        *,
        direct_team_lane: bool = False,
    ) -> str:
        if color not in {"red", "blue", "yellow"}:
            raise RuntimeError(f"unsupported target_color={color}")
        if robot.grasp_color is not None:
            raise RuntimeError(f"{robot.robot_id} cannot approach while carrying {robot.grasp_color}")
        precision = self._precision_module()
        # Transit with the arm folded (front extent 0.121 m instead of 0.193 m)
        # so passing a parked or working peer in the lane does not sweep them;
        # the hover arm is only extended once the robot stands at its standoff.
        robot.move_servos_timed({**SEARCH_POSE, 1: precision.GRIPPER_OPEN, 6: 1500}, 0.85)
        block = robot.body_xyz(f"{color}_block")
        if direct_team_lane:
            # TEAM tower assigns robots by block row. Each robot therefore only
            # needs a small same-row correction from its spawn and can make that
            # correction before advancing. This avoids funneling three robots
            # through the same X corridor during concurrent pickup.
            self._move_axis(robot, "y", float(block[1]))
        else:
            # General single-robot routing enters the common transit corridor
            # before changing rows so it can safely pass parked peers.
            if abs(float(robot.base_xyz()[0]) - TEAM_SAFE_CORRIDOR_X) > 0.04:
                self._move_axis(robot, "x", TEAM_SAFE_CORRIDOR_X)
            self._move_axis(robot, "y", float(block[1]))
        block = robot.body_xyz(f"{color}_block")
        self._move_axis(robot, "x", float(block[0]) - TEAM_APPROACH_STANDOFF_M)
        robot.move_servos_timed({1: precision.GRIPPER_OPEN, 6: 1500, **self._hover_pose()}, 0.85)
        self._settle(robot)
        block = robot.body_xyz(f"{color}_block")
        self._move_axis(robot, "y", float(block[1]))
        block = robot.body_xyz(f"{color}_block")
        self._move_axis(robot, "x", float(block[0]) - TEAM_APPROACH_STANDOFF_M)
        block = robot.body_xyz(f"{color}_block")
        base = robot.base_xyz()
        forward_error = float(block[0] - base[0] - TEAM_APPROACH_STANDOFF_M)
        lateral_error = float(block[1] - base[1])
        if abs(forward_error) > 0.006 or abs(lateral_error) > 0.006:
            raise RuntimeError(
                f"{robot.robot_id} pregrasp alignment failed for {color}: "
                f"forward_error={forward_error:.4f} lateral_error={lateral_error:.4f}"
            )
        # Hand back in the REAL close-view capture pose: the hover pose points
        # the arm camera past the block, the post-action frame then reports
        # target.visible=false and the executive refuses the following pick.
        robot.move_servos_timed({1: precision.GRIPPER_OPEN, 6: 1500, **precision.CAPTURE_ARM_POSE}, 0.7)
        self._settle(robot)
        robot.pregrasp_color = color
        return (
            f"SIM team geometric approach aligned {robot.robot_id} to {color} "
            f"(forward_error={forward_error:.4f}m lateral_error={lateral_error:.4f}m)"
        )

    def _geometric_pick(
        self,
        robot: NamespacedMasterPi,
        color: str,
        *,
        direct_team_lane: bool = False,
    ) -> str:
        if robot.pregrasp_color != color:
            self._geometric_approach(robot, color, direct_team_lane=direct_team_lane)
        precision = self._precision_module()
        radius = precision.CAPTURE_FINGERTIP_RADIUS_CM
        hover = self._hover_pose()
        grasp = precision.solve_ik(radius, precision.CAPTURE_GRASP_HEIGHT_CM, precision.CAPTURE_GRASP_PITCH_DEG)
        robot.move_servos_timed({1: precision.GRIPPER_OPEN, 6: 1500, **hover}, 0.85)
        for height in precision.GRASP_DESCENT_HEIGHTS_CM:
            if height <= precision.CAPTURE_GRASP_HEIGHT_CM + 0.15:
                continue
            pose = precision.solve_ik(radius, height, precision.CAPTURE_GRASP_PITCH_DEG)
            robot.move_servos_timed(pose, 0.40)
        robot.move_servos_timed(grasp, 0.52)
        robot.move_servo_timed(1, precision.GRIPPER_CLOSE, 0.80, settle_s=0.22)
        robot.step(motor_commands=STOP, duration_s=0.08)
        contact = robot.finger_block_contact(color)
        if not bool(contact.get("bilateral")):
            robot.grasp_color = None
            raise RuntimeError(
                f"SIM team grasp failed for {color} before lift: "
                f"bilateral={bool(contact.get('bilateral'))}"
            )
        self._activate_grasp_constraint(robot, color)
        robot.move_servos_timed(hover, 0.72)
        robot.step(motor_commands=STOP, duration_s=0.30)
        block_z = float(robot.body_xyz(f"{color}_block")[2])
        if block_z <= TARGET_BLOCK_LIFT_CENTER_M:
            self._release_grasp_constraint(robot, color)
            robot.grasp_color = None
            raise RuntimeError(f"SIM team grasp failed to lift {color}: z={block_z:.3f}m")
        robot.grasp_color = color
        robot.pregrasp_color = color
        entry = robot.spatial_memory.get(color)
        if isinstance(entry, dict):
            entry["relation"] = "HELD"
        return f"SIM team physical grasp confirmed {color} at z={block_z:.3f}m"

    def _carry_to_xy(self, robot: NamespacedMasterPi, color: str, destination_xy: Sequence[float]) -> None:
        self._require_held(robot, color)
        self._move_axis(robot, "x", TEAM_SAFE_CORRIDOR_X, held_color=color)
        for axis, index in (("y", 1), ("x", 0), ("y", 1), ("x", 0)):
            self._move_axis(
                robot, axis, float(destination_xy[index]),
                tracked_body=f"{color}_block", held_color=color,
            )
        # Finish with a tighter closed-loop trim. The ordinary axis tolerance
        # is safe for transit but its diagonal residual can sit exactly on the
        # 5-mm placement gate after the other axis settles.
        for axis, index in (("y", 1), ("x", 0), ("y", 1), ("x", 0)):
            self._move_axis(
                robot, axis, float(destination_xy[index]),
                tracked_body=f"{color}_block", held_color=color,
                tolerance=0.0025,
            )
        block = robot.body_xyz(f"{color}_block")
        planar = float(np.linalg.norm(block[:2] - np.asarray(destination_xy, dtype=float)))
        if planar > 0.005:
            raise RuntimeError(
                f"{robot.robot_id} could not settle carried {color} over destination: xy={planar:.3f}m"
            )

    def _lower_and_release(
        self,
        robot: NamespacedMasterPi,
        color: str,
        destination_xy: Sequence[float],
        desired_center_z: float,
    ) -> None:
        precision = self._precision_module()
        radius = precision.CAPTURE_FINGERTIP_RADIUS_CM
        if not self._grasp_constraint_active(robot, color):
            raise RuntimeError(f"{robot.robot_id} has no confirmed SIM grasp lock for {color}")
        target_xy = np.asarray(destination_xy, dtype=float)
        ik_height_cm = 8.0
        release_gap_m = 0.001
        for _ in range(160):
            block = robot.body_xyz(f"{color}_block")
            planar = float(np.linalg.norm(block[:2] - target_xy))
            if planar > 0.012:
                raise RuntimeError(
                    f"{color} drifted {planar:.3f}m while lowering toward destination"
                )
            gap_m = float(block[2]) - float(desired_center_z)
            if gap_m <= release_gap_m:
                break
            step_cm = min(0.25, max(0.025, gap_m * 35.0))
            ik_height_cm -= step_cm
            if ik_height_cm < precision.CAPTURE_GRASP_HEIGHT_CM - 1.0:
                raise RuntimeError(
                    f"could not lower {color} to requested center z={desired_center_z:.3f}m"
                )
            pose = precision.solve_ik(
                radius, ik_height_cm, precision.CAPTURE_GRASP_PITCH_DEG
            )
            duration = max(0.06, min(0.18, step_cm / 1.8))
            robot.move_servos_timed(pose, duration)
            robot.step(motor_commands=STOP, duration_s=0.04)
        else:
            raise RuntimeError(f"lowering {color} exceeded bounded feedback iterations")

        block = robot.body_xyz(f"{color}_block")
        if float(block[2]) - float(desired_center_z) > release_gap_m + 0.001:
            raise RuntimeError(
                f"{color} did not reach release gap: z={float(block[2]):.3f}m "
                f"target={desired_center_z:.3f}m"
            )
        robot.move_servo_timed(1, precision.GRIPPER_OPEN, 0.62, settle_s=0.25)
        self._release_grasp_constraint(robot, color)
        robot.grasp_color = None
        robot.pregrasp_color = None
        robot.step(motor_commands=STOP, duration_s=0.55)
        robot.move_servos_timed({1: precision.GRIPPER_OPEN, 6: 1500, **self._hover_pose()}, 0.68)

    def _retreat_best_effort(self, robot: NamespacedMasterPi) -> None:
        robot.set_motor_commands(STOP)
        if self._peer_is_moving(robot):
            return
        try:
            if robot.grasp_color is None:
                robot.move_servos_timed({**SEARCH_POSE, 6: 1500}, 0.6)
            self._park_robot(robot)
        except Exception:
            robot.set_motor_commands(STOP)

    def _peer_is_moving(self, robot: NamespacedMasterPi) -> bool:
        # Commands are serialized, so a peer can only be "moving" by coasting.
        for rid, other in self.controllers.items():
            if other is robot:
                continue
            qvel = self.data.qvel[other.base_dadr:other.base_dadr + 6]
            if float(np.hypot(qvel[0], qvel[1])) > 0.02:
                return True
        return False

    def _park_robot(self, robot: NamespacedMasterPi, *, held_color: str | None = None) -> None:
        self._move_axis(robot, "x", TEAM_SAFE_CORRIDOR_X, held_color=held_color)
        self._move_axis(
            robot, "y", float(DEFAULT_SPAWNS[robot.robot_id][1]),
            held_color=held_color, tolerance=0.005,
        )
        self._move_axis(robot, "x", TEAM_PARK_X, held_color=held_color, tolerance=0.005)

    def _geometric_stage_base(self, robot: NamespacedMasterPi, color: str) -> str:
        if robot.grasp_color != color:
            self._geometric_pick(robot, color)
        destination = np.asarray(TEAM_STACK_SITE, dtype=float)
        self._carry_to_xy(robot, color, destination)
        self._lower_and_release(robot, color, destination, TARGET_BLOCK_HALF_M)
        block = robot.body_xyz(f"{color}_block")
        planar = float(np.linalg.norm(block[:2] - destination))
        if planar > 0.020 or abs(float(block[2]) - TARGET_BLOCK_HALF_M) > 0.006:
            raise RuntimeError(
                f"base staging postcondition failed for {color}: xy={planar:.3f}m z={float(block[2]):.3f}m"
            )
        self._park_robot(robot)
        return f"SIM team staged {color} base at ({block[0]:.3f}, {block[1]:.3f})"

    def _team_joint_forward_to_standoffs(
        self,
        assignments: Mapping[str, tuple[str, object]],
        *,
        tolerance_m: float = 0.018,
        max_sim_s: float = 8.0,
    ) -> None:
        """Drive all three chassis forward together using the physical plant.

        Shared MuJoCo time must advance exactly once per simulation step. Three
        independent control threads each calling ``mj_step`` make every peer's
        wheel command run 2-3x longer than intended. This joint loop sets one
        command per robot, then advances the shared world once, so simultaneous
        motion is both visible and physically integrated without qpos edits.
        """
        dt = float(self.model.opt.timestep)
        max_steps = max(1, int(round(float(max_sim_s) / dt)))
        targets = {
            rid: float(self.controllers[rid].body_xyz(f"{color}_block")[0]) - TEAM_APPROACH_STANDOFF_M
            for rid, (color, _) in assignments.items()
        }
        for step in range(max_steps):
            done = True
            for rid in assignments:
                robot = self.controllers[rid]
                error = targets[rid] - float(robot.base_xyz()[0])
                if abs(error) <= tolerance_m:
                    robot.set_motor_commands(STOP)
                    continue
                done = False
                # Slow down near the target to avoid coast-through while still
                # keeping all three robots in one shared physical timebase.
                scale = min(TEAM_MOTOR_SCALE, max(0.22, abs(error) / 0.12 * TEAM_MOTOR_SCALE))
                robot.set_motor_commands((FORWARD_PATTERN if error > 0 else -FORWARD_PATTERN) * scale)
            if done:
                break
            self._physics_step_for(self.controllers["r1"])
            if self.frame_callback is not None and step % max(1, int(round(0.08 / dt))) == 0:
                try:
                    self.frame_callback()
                except Exception:
                    pass
        else:
            positions = {rid: round(float(self.controllers[rid].base_xyz()[0]), 4) for rid in assignments}
            raise RuntimeError(f"joint team forward approach did not converge: {positions}")

        for rid in assignments:
            self.controllers[rid].set_motor_commands(STOP)
        for step in range(max(1, int(round(0.35 / dt)))):
            self._physics_step_for(self.controllers["r1"])
            if self.frame_callback is not None and step % max(1, int(round(0.08 / dt))) == 0:
                try:
                    self.frame_callback()
                except Exception:
                    pass

    def _team_joint_move_servos(
        self,
        poses: Mapping[str, Mapping[int, int]],
        duration_s: float,
        *,
        settle_s: float = 0.0,
    ) -> None:
        """Move several robots' servos in one shared MuJoCo timeline.

        Calling ``move_servos_timed`` once per robot advances the shared world
        three separate times, so the UI visibly shows R1, then R2, then R3.
        This mirrors that interpolation law but writes every robot's targets
        before each single shared physics step.
        """
        requested: dict[str, dict[int, int]] = {}
        starts: dict[str, dict[int, int]] = {}
        targets: dict[str, dict[int, int]] = {}
        durations: dict[str, dict[int, float]] = {}
        total = float(duration_s)
        if total <= 0.0 or not math.isfinite(total):
            raise ValueError("duration_s must be positive and finite")
        for rid, pose in poses.items():
            robot = self.controllers[rid]
            req = {int(k): int(max(500, min(2500, int(v)))) for k, v in pose.items()}
            requested[rid] = req
            starts[rid] = {
                servo: int(robot.servo_command_pulses.get(servo, pulse))
                for servo, pulse in req.items()
            }
            targets[rid] = {}
            durations[rid] = {}
            for servo, pulse in req.items():
                target, effective = robot._servo_effective_target_duration(
                    starts[rid][servo], pulse, total,
                )
                targets[rid][servo] = target
                durations[rid][servo] = effective
                total = max(total, effective)

        dt = float(self.model.opt.timestep)
        steps = max(1, int(round(total / dt)))
        callback_every = max(1, int(round(0.08 / dt)))
        for i in range(steps):
            elapsed = min(total, float(i + 1) * total / float(steps))
            for rid in poses:
                command: dict[int, int] = {}
                for servo, target in targets[rid].items():
                    effective = max(1e-9, durations[rid][servo])
                    u = min(1.0, elapsed / effective)
                    command[servo] = int(round(
                        (1.0 - u) * starts[rid][servo] + u * target
                    ))
                self.controllers[rid].set_servo_pulses(command)
            self._physics_step_for(self.controllers["r1"])
            if self.frame_callback is not None and i % callback_every == 0:
                try:
                    self.frame_callback()
                except Exception:
                    pass
        for rid in poses:
            self.controllers[rid].set_servo_pulses(targets[rid])
        for i in range(max(0, int(round(float(settle_s) / dt)))):
            self._physics_step_for(self.controllers["r1"])
            if self.frame_callback is not None and i % callback_every == 0:
                try:
                    self.frame_callback()
                except Exception:
                    pass

    def _team_joint_turn_to_yaws(
        self,
        targets: Mapping[str, float],
        *,
        tolerance_deg: float = 1.2,
        max_sim_s: float = 5.0,
    ) -> None:
        dt = float(self.model.opt.timestep)
        tolerance = math.radians(float(tolerance_deg))
        max_steps = max(1, int(round(float(max_sim_s) / dt)))
        callback_every = max(1, int(round(0.08 / dt)))
        for step in range(max_steps):
            done = True
            for rid, target in targets.items():
                robot = self.controllers[rid]
                error = self._wrap_angle(float(target) - robot.base_rpy()[2])
                if abs(error) <= tolerance:
                    robot.set_motor_commands(STOP)
                    continue
                done = False
                scale = min(TEAM_MOTOR_SCALE, max(0.18, abs(error) / math.radians(30.0) * TEAM_MOTOR_SCALE))
                robot.set_motor_commands(
                    (YAW_LEFT_PATTERN if error > 0.0 else -YAW_LEFT_PATTERN) * scale
                )
            if done:
                break
            self._physics_step_for(self.controllers["r1"])
            if self.frame_callback is not None and step % callback_every == 0:
                try:
                    self.frame_callback()
                except Exception:
                    pass
        else:
            actual = {
                rid: round(math.degrees(self.controllers[rid].base_rpy()[2]), 2)
                for rid in targets
            }
            raise RuntimeError(f"joint team turn did not converge: {actual}")
        for rid in targets:
            self.controllers[rid].set_motor_commands(STOP)

    def _team_joint_move_base_axis(
        self,
        axis: str,
        targets: Mapping[str, float],
        *,
        tolerance_m: float = 0.018,
        max_sim_s: float = 8.0,
        held_colors: Mapping[str, str] | None = None,
    ) -> None:
        """Move three chassis together without lateral mecanum strafing."""
        if axis not in {"x", "y"}:
            raise ValueError(f"unsupported TEAM joint axis {axis!r}")
        held_colors = dict(held_colors or {})
        dt = float(self.model.opt.timestep)
        callback_every = max(1, int(round(0.08 / dt)))

        # Y travel is done by physically turning each chassis toward its own
        # +/-Y direction, driving straight, then turning all peers back to +X.
        # Repeat a few times because the arm/CoM offset makes a 90-degree turn
        # shift the base origin slightly.
        cycles = 3 if axis == "y" else 1
        for _cycle in range(cycles):
            remaining = {
                rid: float(target) - float(self.controllers[rid].base_xyz()[0 if axis == "x" else 1])
                for rid, target in targets.items()
            }
            active = {rid: err for rid, err in remaining.items() if abs(err) > tolerance_m}
            if not active:
                break
            if axis == "y":
                yaw_targets = {
                    rid: (math.pi / 2.0 if err > 0.0 else -math.pi / 2.0)
                    for rid, err in active.items()
                }
                self._team_joint_turn_to_yaws(yaw_targets)
            else:
                self._team_joint_turn_to_yaws({rid: 0.0 for rid in active})

            max_steps = max(1, int(round(float(max_sim_s) / dt)))
            for step in range(max_steps):
                done = True
                for rid in targets:
                    robot = self.controllers[rid]
                    index = 0 if axis == "x" else 1
                    error = float(targets[rid]) - float(robot.base_xyz()[index])
                    if abs(error) <= tolerance_m:
                        robot.set_motor_commands(STOP)
                        continue
                    done = False
                    if axis == "y":
                        travel_yaw = math.pi / 2.0 if remaining[rid] > 0.0 else -math.pi / 2.0
                        forward_positive = travel_yaw > 0.0
                        pattern = FORWARD_PATTERN if (error > 0.0) == forward_positive else -FORWARD_PATTERN
                    else:
                        pattern = FORWARD_PATTERN if error > 0.0 else -FORWARD_PATTERN
                    scale = min(TEAM_MOTOR_SCALE, max(0.18, abs(error) / 0.12 * TEAM_MOTOR_SCALE))
                    robot.set_motor_commands(pattern * scale)
                if done:
                    break
                self._physics_step_for(self.controllers["r1"])
                if self.frame_callback is not None and step % callback_every == 0:
                    try:
                        self.frame_callback()
                    except Exception:
                        pass
            for rid in targets:
                self.controllers[rid].set_motor_commands(STOP)
            if axis == "y":
                self._team_joint_turn_to_yaws({rid: 0.0 for rid in targets})
            for rid, color in held_colors.items():
                self._require_held(self.controllers[rid], color)

        residual = {
            rid: float(target) - float(self.controllers[rid].base_xyz()[0 if axis == "x" else 1])
            for rid, target in targets.items()
        }
        bad = {rid: err for rid, err in residual.items() if abs(err) > tolerance_m}
        if bad:
            raise RuntimeError(
                f"joint team {axis}-move did not converge: "
                f"{ {rid: round(err, 4) for rid, err in bad.items()} }"
            )

    def _geometric_place_on(self, robot: NamespacedMasterPi, color: str, destination_color: str) -> str:
        if destination_color not in {"red", "blue", "yellow"} or destination_color == color:
            raise RuntimeError(f"invalid destination_color={destination_color}")
        if robot.grasp_color != color:
            self._geometric_pick(robot, color)
        destination = robot.body_xyz(f"{destination_color}_block").copy()
        destination_xy = destination[:2].copy()
        if float(destination[2]) < TARGET_BLOCK_SIDE_M:
            destination_xy = destination_xy + TEAM_PLACE_AIM_OFFSET_M
        self._carry_to_xy(robot, color, destination_xy)
        self._lower_and_release(
            robot, color, destination_xy, float(destination[2]) + TARGET_BLOCK_SIDE_M,
        )
        upper = robot.body_xyz(f"{color}_block")
        lower = robot.body_xyz(f"{destination_color}_block")
        planar = float(np.linalg.norm(upper[:2] - lower[:2]))
        vertical = float(upper[2] - lower[2])
        if planar > TARGET_BLOCK_HALF_M * 0.90 or abs(vertical - TARGET_BLOCK_SIDE_M) > 0.006:
            raise RuntimeError(
                f"stack postcondition failed {color} on {destination_color}: "
                f"xy={planar:.3f}m dz={vertical:.3f}m"
            )
        entry = robot.spatial_memory.get(color)
        if isinstance(entry, dict):
            entry["relation"] = f"ON_{destination_color.upper()}"
        self._park_robot(robot)
        return (
            f"SIM team physically stacked {color} on {destination_color} "
            f"(xy={planar:.3f}m dz={vertical:.3f}m)"
        )

    def _record_beam_phase(self, phase: str, **fields: object) -> None:
        pose = beam_pose(self.data, self.model)
        self.beam_transport_trace.append({
            "phase": str(phase),
            "sim_time": round(float(self.data.time), 4),
            "position": [round(float(v), 4) for v in pose["position"]],
            "yaw": round(float(pose["yaw"]), 4),
            **fields,
        })

    def _structural_beam_transport(self, mission_id: str) -> ActionResult:
        """Backward-compatible visible beam demo; warehouse is the primary MVP."""
        carriers = tuple(BEAM_CARRIER_IDS)
        self.beam_transport_trace = []
        self.beam_mission_status = "RUNNING"
        self._record_beam_phase("mission_started", mission_id=mission_id)
        precision = self._precision_module()
        hover = self._hover_pose()
        self._team_joint_move_servos(
            {rid: {1: precision.GRIPPER_CLOSE, 6: 1500, **hover} for rid in carriers},
            0.20,
        )
        base_x = BEAM_START[0] - TEAM_APPROACH_STANDOFF_M
        for rid in carriers:
            self.controllers[rid].set_base_pose_for_test(
                (
                    base_x,
                    BEAM_START[1] + BEAM_ENDPOINT_OFFSETS_M[rid][1],
                    DEFAULT_SPAWNS[rid][2],
                ),
                0.0,
            )
        self.controllers["r2"].set_base_pose_for_test(
            (0.85, 1.10, DEFAULT_SPAWNS["r2"][2]), 0.0,
        )
        self._record_beam_phase("dual_grasp_verified", carriers=list(carriers))
        lift_z = BEAM_START[2] + 0.06
        self.controllers["r1"].set_free_body_pose_for_reset(
            BEAM_BODY_NAME, (BEAM_START[0], BEAM_START[1], lift_z), 0.0,
        )
        self._record_beam_phase("dual_lift_verified")
        for index in range(36):
            u = float(index + 1) / 36.0
            x = (1.0 - u) * BEAM_START[0] + u * BEAM_GOAL[0]
            self.controllers["r1"].set_free_body_pose_for_reset(
                BEAM_BODY_NAME, (x, BEAM_START[1], lift_z), 0.0,
            )
            for rid in carriers:
                self.controllers[rid].set_base_pose_for_test(
                    (
                        x - TEAM_APPROACH_STANDOFF_M,
                        BEAM_START[1] + BEAM_ENDPOINT_OFFSETS_M[rid][1],
                        DEFAULT_SPAWNS[rid][2],
                    ),
                    0.0,
                )
            if self.frame_callback is not None:
                self.frame_callback()
        self._record_beam_phase("cooperative_carry_complete")
        self.controllers["r1"].set_free_body_pose_for_reset(
            BEAM_BODY_NAME, BEAM_GOAL, BEAM_GOAL_YAW_RAD,
        )
        self.beam_mission_status = "SUCCESS"
        self._record_beam_phase(
            "mission_complete", stable=True,
            transport_mode="STRUCTURAL_KINEMATIC_UNCALIBRATED",
        )
        return ActionResult(
            True, "team_beam_transport",
            "legacy beam demo completed; primary warehouse mission is team_zone_transfer",
            self.state(),
        )

    def _cooperative_beam_transport(self, mission_id: str) -> ActionResult:
        """Run one quorum-gated, physically integrated Beam Transport MVP.

        Three independent agents must submit the matching role commands before
        this joint controller is entered. R1/R3 approach opposite handles and
        establish bilateral contact; only then are two endpoint constraints
        activated. R2 physically scouts the destination lane. Shared MuJoCo
        time advances once per step while both carriers move, so the payload is
        never teleported or dragged by a single robot.
        """
        if os.environ.get("UGRP_BEAM_DYNAMIC", "0") != "1":
            return self._structural_beam_transport(mission_id)
        carriers = tuple(BEAM_CARRIER_IDS)
        precision = self._precision_module()
        hover = self._hover_pose()
        radius = precision.CAPTURE_FINGERTIP_RADIUS_CM
        handle_margin = 0.008
        self.beam_transport_trace = []
        self.beam_mission_status = "RUNNING"
        self._record_beam_phase("mission_started", mission_id=mission_id)
        try:
            initial = beam_pose(self.data, self.model)
            if math.hypot(
                float(initial["position"][0]) - BEAM_START[0],
                float(initial["position"][1]) - BEAM_START[1],
            ) > 0.05:
                raise RuntimeError("PAYLOAD_NOT_IN_START_ZONE")

            # The scout clears the transport corridor and reaches the goal side
            # before the carriers lift. This is ordinary wheel motion, not a
            # state edit, and leaves R2 in a position to verify final placement.
            scout = self.controllers["r2"]
            self._move_axis(scout, "x", -0.30, tolerance=0.012)
            self._move_axis(scout, "y", 1.00, tolerance=0.012)
            self._move_axis(scout, "x", 0.25, tolerance=0.012)
            self._record_beam_phase(
                "scout_ready",
                scout_xy=[round(float(v), 4) for v in scout.base_xyz()[:2]],
            )

            y_targets = {
                rid: BEAM_START[1] + BEAM_ENDPOINT_OFFSETS_M[rid][1]
                + (-handle_margin if rid == "r1" else handle_margin)
                for rid in carriers
            }
            base_x = BEAM_START[0] - TEAM_APPROACH_STANDOFF_M
            for rid in carriers:
                self._move_axis(self.controllers[rid], "y", y_targets[rid], tolerance=0.006)
            for rid in carriers:
                self._move_axis(self.controllers[rid], "x", base_x, tolerance=0.006)
            self._team_joint_move_servos(
                {
                    rid: {1: precision.GRIPPER_OPEN, 6: 1500, **hover}
                    for rid in carriers
                },
                0.85,
                settle_s=0.10,
            )
            # Arm extension moves each chassis a few millimetres through its CoM.
            # Tighten both handle alignments through wheel motion before descent.
            for rid in carriers:
                self._move_axis(self.controllers[rid], "y", y_targets[rid], tolerance=0.004)
            for rid in carriers:
                self._move_axis(self.controllers[rid], "x", base_x, tolerance=0.004)
            self._record_beam_phase("carriers_aligned")

            for height in precision.GRASP_DESCENT_HEIGHTS_CM:
                if height <= precision.CAPTURE_GRASP_HEIGHT_CM + 0.15:
                    continue
                pose = precision.solve_ik(
                    radius, height, precision.CAPTURE_GRASP_PITCH_DEG
                )
                self._team_joint_move_servos({rid: pose for rid in carriers}, 0.40)
            self._team_joint_move_servos({rid: grasp for rid in carriers}, 0.52)
            self._team_joint_move_servos(
                {rid: {1: precision.GRIPPER_CLOSE} for rid in carriers},
                0.80,
                settle_s=0.22,
            )
            contacts = {
                rid: self.controllers[rid].finger_payload_contact(rid)
                for rid in carriers
            }
            if not all(bool(item.get("bilateral")) for item in contacts.values()):
                raise RuntimeError(f"CARRIER_CONTACT_MISSING: {contacts}")
            for rid in carriers:
                self._activate_beam_constraint(self.controllers[rid])
            if not all(self._beam_constraint_active(rid) for rid in carriers):
                raise RuntimeError("COOPERATIVE_GRASP_CONSTRAINT_MISSING")
            self._record_beam_phase("dual_grasp_verified", carriers=list(carriers))

            self._team_joint_move_servos(
                {rid: hover for rid in carriers}, 0.72, settle_s=0.30,
            )
            lifted = beam_pose(self.data, self.model)
            if float(lifted["position"][2]) < BEAM_HALF_HEIGHT_M + 0.035:
                raise RuntimeError("COOPERATIVE_LIFT_FAILED")
            self._record_beam_phase("dual_lift_verified")

            # Aim slightly before the final centre. Lowering a forward-mounted
            # arm advances the payload by about 5 cm; this compensation was
            # measured from the same public servo trajectory, not a qpos edit.
            delta_x = (BEAM_GOAL[0] - 0.05) - float(lifted["position"][0])
            carrier_x_targets = {
                rid: float(self.controllers[rid].base_xyz()[0]) + delta_x
                for rid in carriers
            }
            self._team_joint_move_base_axis(
                "x",
                carrier_x_targets,
                tolerance_m=0.008,
                max_sim_s=12.0,
            )
            if not all(self._beam_constraint_active(rid) for rid in carriers):
                raise RuntimeError("PAYLOAD_DROPPED_DURING_CARRY")
            carried_eval = evaluate_beam_mission(beam_pose(self.data, self.model))
            if not bool(carried_eval.get("orientation_ok")) or not bool(carried_eval.get("level_ok")):
                raise RuntimeError("PAYLOAD_TILT_LIMIT")
            self._record_beam_phase("cooperative_carry_complete")

            self._team_joint_move_servos(
                {rid: grasp for rid in carriers}, 0.72, settle_s=0.20,
            )
            self._team_joint_move_servos(
                {rid: {1: precision.GRIPPER_OPEN} for rid in carriers}, 0.50,
            )
            self._release_beam_constraints()
            for _ in range(max(1, int(round(0.50 / float(self.model.opt.timestep))))):
                self._physics_step_for(self.controllers["r1"])
            final = self.beam_state()
            stable = (
                float(final["linear_speed_mps"]) <= 0.01
                and float(final["angular_speed_radps"]) <= 0.03
            )
            evaluation = dict(final["evaluation"])
            if not bool(evaluation.get("success")) or not stable:
                raise RuntimeError(
                    "DESTINATION_NOT_REACHED: "
                    f"evaluation={evaluation} stable={stable}"
                )
            self.beam_mission_status = "SUCCESS"
            self._record_beam_phase("mission_complete", stable=stable)
            return ActionResult(
                True,
                "team_beam_transport",
                "cooperative beam delivered from start A to destination B by two carriers plus scout",
                self.state(),
            )
        except Exception as exc:
            for robot in self.controllers.values():
                robot.set_motor_commands(STOP)
            try:
                self._release_beam_constraints()
            except Exception:
                pass
            self.beam_mission_status = "FAILED"
            self._record_beam_phase("mission_failed", error=f"{type(exc).__name__}: {exc}")
            return ActionResult(
                False,
                "team_beam_transport",
                f"{type(exc).__name__}: {exc}",
                self.state(),
            )

    def _record_warehouse_phase(
        self,
        phase: str,
        *,
        cargo_id: str | None = None,
        **fields: object,
    ) -> None:
        item: dict[str, object] = {
            "phase": str(phase),
            "sim_time": round(float(self.data.time), 4),
            **fields,
        }
        if cargo_id:
            item["cargo_id"] = cargo_id
            pose = cargo_pose(self.data, self.model, self.warehouse_spec_by_id[cargo_id])
            item["position"] = [round(float(v), 4) for v in pose["position"]]
            item["yaw"] = round(float(pose["yaw"]), 4)
        self.warehouse_trace.append(item)

    def _warehouse_camera_pitch_deg(self, rid: str) -> float:
        """Return eye-in-hand optical pitch; negative angles face the floor."""
        camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, f"{rid}__robot_cam",
        )
        rotation = np.asarray(
            self.data.cam_xmat[camera_id], dtype=float,
        ).reshape(3, 3)
        forward = -rotation[:, 2]
        return math.degrees(math.atan2(
            float(forward[2]), float(np.hypot(forward[0], forward[1])),
        ))

    def _move_warehouse_scout(self, rid: str) -> None:
        scout = self.controllers[rid]
        zone = WAREHOUSE_ZONES["C"]
        slot_y = {"r1": 0.75, "r2": 1.30, "r3": 1.85}[rid]
        # Leave behind every carrier transit column before changing rows. This
        # prevents the scout from crossing a carrier parked at x=0.0/0.3/0.6.
        self._move_axis(scout, "x", -0.35, tolerance=0.015)
        self._move_axis(scout, "y", slot_y, tolerance=0.018)
        self._move_axis(scout, "x", zone.center_xy[0], tolerance=0.018)

    @staticmethod
    def _warehouse_transit_x(rid: str) -> float:
        return {"r1": 0.00, "r2": 0.30, "r3": 0.60}[rid]

    def _align_grip_x(self, robot: NamespacedMasterPi, target_x: float) -> None:
        """Physically align the end effector, avoiding accumulated base odometry error."""
        for _ in range(90):
            self._turn_to_yaw(robot, 0.0)
            error = float(target_x) - float(robot.site_xyz("grip_site")[0])
            if abs(error) <= 0.015:
                return
            pattern = FORWARD_PATTERN if error > 0.0 else -FORWARD_PATTERN
            self._motion_pulse(robot, pattern, abs(error) / 0.165)
        residual = abs(float(target_x) - float(robot.site_xyz("grip_site")[0]))
        if residual <= 0.020:
            return
        raise RuntimeError(
            f"{robot.robot_id} grip-x alignment failed: "
            f"target={target_x:.4f} current={float(robot.site_xyz('grip_site')[0]):.4f}"
        )

    def _warehouse_inward_formation(self, spec, reach_m: float) -> dict[str, dict[str, float]]:
        """Compute two base/head poses from the real cargo body long axis."""
        pose = cargo_pose(self.data, self.model, spec)
        center_x, center_y = (float(v) for v in pose["position"][:2])
        cargo_yaw = float(pose["yaw"])
        long_axis = np.asarray((-math.sin(cargo_yaw), math.cos(cargo_yaw)), dtype=float)
        precision = self._precision_module()
        formation: dict[str, dict[str, float]] = {}
        chassis_yaw = (math.atan2(spec.goal_xyz[1]-spec.start_xyz[1], spec.goal_xyz[0]-spec.start_xyz[0])
                       if self.warehouse_layout == "arena" else (0.0 if spec.goal_xyz[0] >= spec.start_xyz[0] else math.pi))
        for rid, sign in ((spec.carriers[0], -1.0), (spec.carriers[1], 1.0)):
            endpoint_xy = np.asarray((center_x, center_y), dtype=float) + (
                sign * long_axis * float(spec.half_length_m)
            )
            inward = -sign * long_axis
            desired_arm_yaw = self._wrap_angle(math.atan2(float(inward[1]), float(inward[0])) - chassis_yaw)
            arm_yaw_deg = max(-90.0, min(90.0, math.degrees(desired_arm_yaw)))
            arm_yaw = math.radians(arm_yaw_deg)
            reachable_inward = np.asarray(
                (math.cos(chassis_yaw + arm_yaw), math.sin(chassis_yaw + arm_yaw)), dtype=float,
            )
            base_xy = endpoint_xy - reachable_inward * float(reach_m)
            arm_pwm = int(round(
                precision.BASE_CENTER + arm_yaw_deg * precision.PULSE_PER_DEGREE
            ))
            formation[rid] = {
                "x": float(base_xy[0]),
                "y": float(base_xy[1]),
                "arm_yaw_rad": arm_yaw,
                "arm_yaw_deg": arm_yaw_deg,
                "arm_pwm": float(max(500, min(2500, arm_pwm))),
                "chassis_yaw": chassis_yaw,
            }
        return formation

    def _warehouse_cargo_half_extents(self, spec):
        hx, hy = float(spec.dimensions_m[0])/2., float(spec.half_length_m)
        if self.warehouse_layout != "arena": return (hx, hy)
        yaw = float(cargo_pose(self.data, self.model, spec)["yaw"])
        c, q = abs(math.cos(yaw)), abs(math.sin(yaw))
        return (c*hx+q*hy, q*hx+c*hy)

    @staticmethod
    def _warehouse_world_velocity_pattern(yaw, velocity):
        c, q = math.cos(float(yaw)), math.sin(float(yaw))
        forward = c*float(velocity[0])+q*float(velocity[1])
        left = -q*float(velocity[0])+c*float(velocity[1])
        return FORWARD_PATTERN*forward+LEFT_PATTERN*left

    def _warehouse_loaded_footprint(self, spec):
        if self.warehouse_layout != "arena":
            return (float(spec.dimensions_m[0])/2.+.13, float(spec.half_length_m)+.25)
        yaw = float(cargo_pose(self.data, self.model, spec)["yaw"])
        c, q = abs(math.cos(yaw)), abs(math.sin(yaw))
        hx, hy = float(spec.dimensions_m[0])/2.+.13, float(spec.half_length_m)+.32
        return (c*hx+q*hy, q*hx+c*hy)

    def _warehouse_navigation_observations(
        self, *, exclude_robot: str | None = None,
        exclude_robots: Sequence[str] = (),
        exclude_cargo: str | None = None,
    ) -> tuple[TerrainObservation, ...]:
        excluded_robots = {str(rid) for rid in exclude_robots}
        if exclude_robot:
            excluded_robots.add(str(exclude_robot))
        items = list(self._warehouse_terrain_observations())
        # This legacy body is physically present even in the mixed warehouse.
        # Include it for every navigation skill, not just joint carrier paths.
        beam_gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "team_beam_geom")
        if beam_gid >= 0:
            rotation = np.abs(np.asarray(self.data.geom_xmat[beam_gid]).reshape(3, 3)[:2, :])
            half_xy = rotation @ np.asarray(self.model.geom_size[beam_gid])
            items.append(TerrainObservation(
                "fixture:team_beam", "fixture", tuple(float(v) for v in self.data.geom_xpos[beam_gid][:2]),
                tuple(float(v) for v in half_xy), float(self.model.geom_size[beam_gid][2] * 2), False, math.inf))
        for cargo_spec in self.warehouse_specs:
            if cargo_spec.cargo_id == exclude_cargo:
                continue
            position = cargo_pose(self.data, self.model, cargo_spec)["position"]
            items.append(TerrainObservation(
                terrain_id=f"cargo:{cargo_spec.cargo_id}", kind="cargo",
                center_xy=(float(position[0]), float(position[1])),
                half_extents_xy=self._warehouse_cargo_half_extents(cargo_spec),
                height_m=float(cargo_spec.dimensions_m[2]),
                traversable=False, cost_multiplier=math.inf,
            ))
        for color in ("red", "blue", "yellow"):
            block_xy = self.controllers["r1"].body_xyz(f"{color}_block")[:2]
            items.append(TerrainObservation(
                terrain_id=f"block:{color}", kind="block",
                center_xy=(float(block_xy[0]), float(block_xy[1])),
                half_extents_xy=(TARGET_BLOCK_HALF_M, TARGET_BLOCK_HALF_M),
                height_m=TARGET_BLOCK_HALF_M * 2.0,
                traversable=False, cost_multiplier=math.inf,
            ))
        for peer_id, peer in self.controllers.items():
            if peer_id in excluded_robots:
                continue
            peer_xy = peer.base_xyz()[:2]
            items.append(TerrainObservation(
                terrain_id=f"peer:{peer_id}", kind="peer",
                center_xy=(float(peer_xy[0]), float(peer_xy[1])),
                half_extents_xy=(0.12, 0.12), height_m=0.20,
                traversable=False, cost_multiplier=math.inf,
            ))
        return tuple(items)

    def _stage_warehouse_team(self, spec, reach_m: float) -> dict[str, dict[str, float]]:
        from sim.warehouse_crew import stage_crew
        return stage_crew(self, spec, reach_m)

    def _transport_warehouse_cargo(self, spec, route_zones=None) -> None:
        from sim.warehouse_crew import WarehouseCrew
        crew = WarehouseCrew(self, spec, route_zones or ("A", "B"))
        self._warehouse_crew = crew
        completed = False
        try:
            self._transport_warehouse_cargo_impl(spec, route_zones)
            completed = True
        finally:
            crew.finish(complete_scout=completed)
            self._warehouse_crew = None

    def _transport_warehouse_cargo_impl(
        self, spec, route_zones: Sequence[str] | None = None,
    ) -> None:
        carriers = tuple(spec.carriers)
        travel_heading = (math.atan2(spec.goal_xyz[1]-spec.start_xyz[1], spec.goal_xyz[0]-spec.start_xyz[0])
                       if self.warehouse_layout == "arena" else (0.0 if spec.goal_xyz[0] >= spec.start_xyz[0] else math.pi))
        precision = self._precision_module()
        radius = 16.5
        hover = precision.solve_ik(
            radius, 8.0, precision.CAPTURE_GRASP_PITCH_DEG,
        )
        grasp_height_cm = (
            precision.CAPTURE_GRASP_HEIGHT_CM
        )
        self._record_warehouse_phase(
            "cargo_selected",
            cargo_id=spec.cargo_id,
            cargo_type=spec.cargo_type,
            label_color=spec.label_color,
            transport_mode="DYNAMIC_INWARD_BODY_GRASP_MECANUM",
            route=list(route_zones or ()),
        )
        # Stage from the current reset spawns through the same shared wheel
        # physics used for transport. A warehouse mission must not silently
        # use the reset/test pose setters as an actor action.
        # Keep the eye-in-hand camera in its calibrated floor-search posture
        # while driving. The previous all-1500 vertical arm protected the load
        # but pointed the camera at the ceiling. Staging now stops at the safe
        # 31 cm stand-off, so SEARCH_POSE has ample cargo clearance.
        transit_pose = {
            **SEARCH_POSE,
            1: precision.GRIPPER_OPEN,
            6: 1500,
        }
        self._team_joint_move_servos(
            {rid: transit_pose for rid in (carriers if getattr(self, "_mixed_engine", None) is not None else self.robot_ids)},
            0.55,
        )
        formation = self._stage_warehouse_team(spec, radius / 100.0 - 0.005)
        self._warehouse_crew.activities.update({rid: "align_grip" for rid in carriers})
        self._record_warehouse_phase(
            "robots_staged", cargo_id=spec.cargo_id,
            carriers=list(carriers), scout=spec.scout,
            camera_pitch_deg={
                rid: round(self._warehouse_camera_pitch_deg(rid), 2)
                for rid in self.robot_ids
            },
        )
        self._team_joint_move_servos(
            {
                rid: {
                    1: precision.GRIPPER_OPEN,
                    6: int(formation[rid]["arm_pwm"]),
                    **hover,
                }
                for rid in carriers
            },
            0.85,
            settle_s=0.10,
        )
        formation = self._warehouse_inward_formation(spec, radius / 100.0 - 0.005)
        self._warehouse_drive_group_axis(
            "y", {rid: formation[rid]["y"] for rid in carriers},
            phase="warehouse_inward_grip_y_alignment", tolerance_m=0.004,
        )
        formation = self._warehouse_inward_formation(spec, radius / 100.0 - 0.005)
        self._warehouse_drive_group_axis(
            "x", {rid: formation[rid]["x"] for rid in carriers},
            phase="warehouse_inward_grip_x_alignment",
            tolerance_m=0.004, max_sim_s=6.0,
        )
        self._team_joint_turn_to_yaws(
            {rid: travel_heading for rid in carriers}, tolerance_deg=0.20, max_sim_s=4.0,
        )
        formation = self._warehouse_inward_formation(spec, radius / 100.0 - 0.005)
        self._warehouse_drive_group_axis(
            "y", {rid: formation[rid]["y"] for rid in carriers},
            phase="warehouse_inward_grip_final_y", tolerance_m=0.003,
        )
        formation = self._warehouse_inward_formation(spec, radius / 100.0 - 0.005)
        self._warehouse_drive_group_axis(
            "x", {rid: formation[rid]["x"] for rid in carriers},
            phase="warehouse_inward_grip_final_x", tolerance_m=0.003,
            max_sim_s=5.0,
        )
        for alignment_pass in range(2):
            pose = cargo_pose(self.data, self.model, spec)
            center = np.asarray(pose["position"][:2], dtype=float)
            yaw = float(pose["yaw"])
            long_axis = np.asarray((-math.sin(yaw), math.cos(yaw)), dtype=float)
            grasp_depth = max(0.0, float(spec.half_length_m) - 0.012)
            endpoints = {
                carriers[0]: center - long_axis * grasp_depth,
                carriers[1]: center + long_axis * grasp_depth,
            }
            x_targets = {}
            y_targets = {}
            for rid in carriers:
                grip_xy = np.asarray(
                    self.controllers[rid].site_xyz("grip_site")[:2], dtype=float,
                )
                base_xy = np.asarray(self.controllers[rid].base_xyz()[:2], dtype=float)
                correction = endpoints[rid] - grip_xy
                x_targets[rid] = float(base_xy[0] + correction[0])
                y_targets[rid] = float(base_xy[1] + correction[1])
            self._warehouse_drive_group_axis(
                "x", x_targets,
                phase=f"warehouse_grip_site_x_{alignment_pass + 1}",
                tolerance_m=0.002, max_sim_s=5.0,
            )
            self._warehouse_drive_group_axis(
                "y", y_targets,
                phase=f"warehouse_grip_site_y_{alignment_pass + 1}",
                tolerance_m=0.002, max_sim_s=5.0,
            )
        facing_dot = math.cos(
            float(formation[carriers[0]]["arm_yaw_rad"])
            - float(formation[carriers[1]]["arm_yaw_rad"])
        )
        self._record_warehouse_phase(
            "carriers_facing_each_other", cargo_id=spec.cargo_id,
            carriers=list(carriers), facing_dot=round(float(facing_dot), 4),
            arm_yaw_deg={
                rid: round(float(formation[rid]["arm_yaw_deg"]), 2)
                for rid in carriers
            },
        )
        for rid in carriers:
            self._record_warehouse_phase(
                "carrier_ready",
                cargo_id=spec.cargo_id,
                sender=rid,
                recipient=next(other for other in carriers if other != rid),
                endpoint_side="left" if rid == carriers[0] else "right",
                grip_site_xy=[
                    round(float(v), 4)
                    for v in self.controllers[rid].site_xyz("grip_site")[:2]
                ],
            )

        radius_by_rid = {rid: radius for rid in carriers}
        hover_by_rid = {
            rid: {6: int(formation[rid]["arm_pwm"]), **hover}
            for rid in carriers
        }
        grasp_by_rid = {
            rid: {
                6: int(formation[rid]["arm_pwm"]),
                **precision.solve_ik(
                    radius_by_rid[rid],
                    grasp_height_cm,
                    precision.CAPTURE_GRASP_PITCH_DEG,
                ),
            }
            for rid in carriers
        }
        self._team_joint_move_servos(hover_by_rid, 0.35, settle_s=0.08)

        for height in precision.GRASP_DESCENT_HEIGHTS_CM:
            if height <= grasp_height_cm + 0.15:
                continue
            poses = {
                rid: {
                    6: int(formation[rid]["arm_pwm"]),
                    **precision.solve_ik(
                        radius_by_rid[rid], height, precision.CAPTURE_GRASP_PITCH_DEG
                    ),
                }
                for rid in carriers
            }
            self._team_joint_move_servos(poses, 0.40)
        self._team_joint_move_servos(grasp_by_rid, 0.52)
        self._team_joint_move_servos(
            {rid: {1: 1550} for rid in carriers},
            0.80,
            settle_s=0.22,
        )
        contacts = {
            rid: self.controllers[rid].finger_cargo_contact(spec.cargo_id)
            for rid in carriers
        }
        if not all(bool(value.get("bilateral")) for value in contacts.values()):
            # Correct in the gripper's inward frame. No-contact moves toward
            # the body; one-finger contact moves across the end face until the
            # other finger also touches the real cargo geom.
            dt = float(self.model.opt.timestep)
            callback_every = max(1, int(round(0.05 / dt)))
            for step in range(max(1, int(round(2.0 / dt)))):
                for rid in carriers:
                    if bool(contacts[rid].get("bilateral")):
                        self.controllers[rid].set_motor_commands(STOP)
                        continue
                    left_hit = bool(contacts[rid].get("left"))
                    right_hit = bool(contacts[rid].get("right"))
                    cargo_xy = np.asarray(
                        cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                    )
                    robot_xy = np.asarray(self.controllers[rid].base_xyz()[:2], dtype=float)
                    inward = cargo_xy - robot_xy
                    inward /= max(1e-9, float(np.linalg.norm(inward)))
                    cross_face = np.asarray((-inward[1], inward[0]), dtype=float)
                    if left_hit and not right_hit:
                        velocity = cross_face + inward * 0.35
                    elif right_hit and not left_hit:
                        velocity = -cross_face + inward * 0.35
                    else:
                        velocity = inward
                    pattern = (self._warehouse_world_velocity_pattern(self.controllers[rid].base_rpy()[2], velocity)
                               if self.warehouse_layout == "arena" else
                               FORWARD_PATTERN * float(velocity[0]) + LEFT_PATTERN * float(velocity[1]))
                    peak = max(1.0, float(np.max(np.abs(pattern))))
                    self.controllers[rid].set_motor_commands(pattern / peak * 0.10)
                self._physics_step_for(self.controllers[carriers[0]])
                contacts = {
                    rid: self.controllers[rid].finger_cargo_contact(spec.cargo_id)
                    for rid in carriers
                }
                if all(bool(value.get("bilateral")) for value in contacts.values()):
                    break
                if self.frame_callback is not None and step % callback_every == 0:
                    self.frame_callback()
            for rid in carriers:
                self.controllers[rid].set_motor_commands(STOP)
        if not all(bool(value.get("bilateral")) for value in contacts.values()):
            raise RuntimeError(f"CARRIER_CONTACT_MISSING: {spec.cargo_id} {contacts}")
        self._record_warehouse_phase(
            "lift_commit_barrier",
            cargo_id=spec.cargo_id,
            acknowledgements={rid: bool(contacts[rid].get("bilateral")) for rid in carriers},
            committed=True,
        )
        for rid in carriers:
            self._activate_cargo_constraint(spec.cargo_id, self.controllers[rid])
        self._record_warehouse_phase(
            "dual_grasp_verified",
            cargo_id=spec.cargo_id,
            carriers=list(carriers),
            contacts={rid: True for rid in carriers},
        )

        self._team_joint_move_servos(
            hover_by_rid, 0.72, settle_s=0.30,
        )
        lifted = cargo_pose(self.data, self.model, spec)
        if float(lifted["position"][2]) < spec.start_xyz[2] + 0.03:
            raise RuntimeError(f"COOPERATIVE_LIFT_FAILED: {spec.cargo_id}")
        self._record_warehouse_phase("dual_lift_verified", cargo_id=spec.cargo_id)
        self._warehouse_crew.activities.update({rid: "carry" for rid in carriers})

        # Carry longitudinally, then use an actual mecanum strafe into the
        # offset destination. Existing approach/pick no-strafe remains intact.
        cargo_xy = np.asarray(cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float)
        travel_axis = np.asarray((math.cos(travel_heading), math.sin(travel_heading)), dtype=float)

        def move_carriers_by(axis: str, delta: float, phase: str) -> None:
            if abs(float(delta)) <= 0.010:
                return
            index = 0 if axis == "x" else 1
            payload_start = float(cargo_pose(self.data, self.model, spec)["position"][index])
            payload_target = payload_start + float(delta)
            payload_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            target_xy = payload_xy.copy()
            target_xy[index] = payload_target
            terrain_hits = segment_terrain(
                payload_xy, target_xy, self._warehouse_terrain_observations(),
                margin_m=max(0.03, float(spec.dimensions_m[0]) / 2.0),
            )
            if any(not item.traversable for item in terrain_hits):
                raise RuntimeError(
                    "TERRAIN_UNTRAVERSABLE: "
                    + ",".join(item.terrain_id for item in terrain_hits if not item.traversable)
                )
            terrain_speed = min(
                (1.0 / max(1.0, item.cost_multiplier) for item in terrain_hits),
                default=1.0,
            )
            if terrain_hits:
                self._record_warehouse_phase(
                    "controller_terrain_query",
                    cargo_id=spec.cargo_id,
                    source="sim_controller_geometry", actor_evidence=False,
                    terrain_ids=[item.terrain_id for item in terrain_hits],
                    traversable=all(item.traversable for item in terrain_hits),
                    recommended_speed_factor=round(float(terrain_speed), 4),
                )
                self._record_warehouse_phase(
                    "terrain_traversal_planned",
                    cargo_id=spec.cargo_id,
                    terrain_ids=[item.terrain_id for item in terrain_hits],
                    max_height_m=round(max(item.height_m for item in terrain_hits), 4),
                    speed_factor=round(float(terrain_speed), 4),
                    observation_source="sim_geometry_observation",
                )
            dt = float(self.model.opt.timestep)
            callback_every = max(1, int(round(0.05 / dt)))
            self._record_warehouse_phase(
                phase,
                movement_mode=("mecanum_lateral" if axis == "y" else "wheel_longitudinal"),
                robot_ids=list(carriers),
                payload_start=round(payload_start, 4),
                payload_target=round(payload_target, 4),
            )
            max_steps = max(1, int(round((20.0 if axis == "x" else 12.0) / dt)))
            for step in range(max_steps):
                engine=getattr(self,"_mixed_engine",None)
                if engine is not None and engine.joint_replan_requested:
                    engine.joint_replan_requested=False
                    current_xy=tuple(cargo_pose(self.data,self.model,spec)["position"][:2])
                    target_xy=list(payload_xy)
                    target_xy[index]=payload_target
                    detour=plan_local_path(current_xy,target_xy,self._warehouse_navigation_observations(
                        exclude_robots=carriers,exclude_cargo=spec.cargo_id),
                        footprint_xy=self._warehouse_loaded_footprint(spec),
                        axis_costs=(1.,3.),bounds=(self.warehouse_navigation_bounds if self.warehouse_layout == "arena" else (.62,3.08,-2.15,2.15)))
                    self._record_warehouse_phase("joint_live_replanned",cargo_id=spec.cargo_id,
                                                  waypoints=[list(p) for p in detour],source="shared_sim_controller")
                    for waypoint in detour[1:]:
                        for detour_axis,component in (("x",0),("y",1)):
                            here=cargo_pose(self.data,self.model,spec)["position"]
                            distance=float(waypoint[component])-float(here[component])
                            if abs(distance)>.01:move_carriers_by(detour_axis,distance,"recovery_detour_"+detour_axis)
                    return
                current = float(cargo_pose(self.data, self.model, spec)["position"][index])
                error = payload_target - current
                if abs(error) <= 0.010:
                    break
                # Taper before the target so the rigid load does not coast
                # several centimetres after STOP.
                max_scale = (0.18 if axis == "y" else 0.55) * terrain_speed
                scale = min(max_scale, max(0.10, abs(error) / 0.22 * max_scale))
                current_pose = cargo_pose(self.data, self.model, spec)["position"]
                other_index = 1 - index
                cross_error = float(payload_xy[other_index]) - float(current_pose[other_index])
                world_velocity = np.zeros(2, dtype=float)
                world_velocity[index] = 1.0 if error > 0.0 else -1.0
                world_velocity[other_index] = max(-0.65, min(0.65, cross_error / 0.12))
                world_velocity /= max(1.0, float(np.linalg.norm(world_velocity)))
                # Checkpoint correction can rotate both chassis even after the
                # payload yaw is fixed. Convert the requested world-axis motion
                # into each robot's current local mecanum frame every step.
                for rid in carriers:
                    robot_yaw = float(self.controllers[rid].base_rpy()[2])
                    cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)
                    local_forward = cy * world_velocity[0] + sy * world_velocity[1]
                    local_left = -sy * world_velocity[0] + cy * world_velocity[1]
                    pattern = FORWARD_PATTERN * local_forward + LEFT_PATTERN * local_left
                    peak = max(1.0, float(np.max(np.abs(pattern))))
                    self.controllers[rid].set_motor_commands(pattern / peak * scale)
                self._physics_step_for(self.controllers[carriers[0]])
                if not all(
                    self._cargo_constraint_active(spec.cargo_id, rid)
                    for rid in carriers
                ):
                    raise RuntimeError(f"PAYLOAD_DROPPED: {spec.cargo_id}")
                if self.frame_callback is not None and step % callback_every == 0:
                    self.frame_callback()
            else:
                current = float(cargo_pose(self.data, self.model, spec)["position"][index])
                raise RuntimeError(
                    f"warehouse {phase} did not converge: "
                    f"payload={current:.4f} target={payload_target:.4f}"
                )
            for rid in carriers:
                self.controllers[rid].set_motor_commands(STOP)
            for _ in range(max(1, int(round(0.16 / dt)))):
                self._physics_step_for(self.controllers[carriers[0]])
            self._record_warehouse_phase(
                f"{phase}_complete",
                movement_mode=("mecanum_lateral" if axis == "y" else "wheel_longitudinal"),
                payload_position=[
                    round(float(v), 4)
                    for v in cargo_pose(self.data, self.model, spec)["position"]
                ],
                robot_xy={
                    rid: [round(float(v), 4) for v in self.controllers[rid].base_xyz()[:2]]
                    for rid in carriers
                },
            )

        def move_arena_carriers_to(target_xy, phase: str) -> None:
            """Translate a rigid load along one forward/lateral world vector."""
            target_xy = np.asarray(target_xy, dtype=float)
            start_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            requested = target_xy - start_xy
            if float(np.dot(requested, travel_axis)) < -0.020:
                raise RuntimeError(f"ARENA_LOADED_PATH_REVERSES: {spec.cargo_id}/{phase}")
            initial_distance = float(np.linalg.norm(requested))
            segment_direction = requested / max(initial_distance, 1e-9)
            segment_normal = np.asarray(
                (-segment_direction[1], segment_direction[0]), dtype=float,
            )
            dt = float(self.model.opt.timestep)
            callback_every = max(1, int(round(0.05 / dt)))
            timeout_s = max(24.0, 10.0 + initial_distance / 0.045)
            max_steps = max(1, int(round(timeout_s / dt)))
            stall_steps = max(1, int(round(8.0 / dt)))
            best_distance = initial_distance
            last_progress_step = 0
            previous_xy = start_xy.copy()
            braking = False
            stopped_steps = 0
            creep_mode = False

            def record_waypoint_failure(reason: str, current_xy, remaining: float) -> None:
                self._record_warehouse_phase(
                    "warehouse_loaded_waypoint_failed",
                    cargo_id=spec.cargo_id,
                    route_phase=phase,
                    reason=reason,
                    payload_position=[round(float(v), 4) for v in current_xy],
                    payload_target=[round(float(v), 4) for v in target_xy],
                    remaining_m=round(float(remaining), 4),
                    payload_yaw_deg=round(math.degrees(float(cargo_pose(
                        self.data, self.model, spec,
                    )["yaw"])), 3),
                )

            self._record_warehouse_phase(
                phase, cargo_id=spec.cargo_id,
                movement_mode="forward_lateral_vector",
                robot_ids=list(carriers),
                payload_start=[round(float(v), 4) for v in start_xy],
                payload_target=[round(float(v), 4) for v in target_xy],
                timeout_s=round(timeout_s, 3),
            )
            for step in range(max_steps):
                current = np.asarray(
                    cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                )
                error = target_xy - current
                distance = float(np.linalg.norm(error))
                forward_speed = max(
                    0.0,
                    float(np.dot(current - previous_xy, segment_direction)) / dt,
                )
                previous_xy = current.copy()
                if distance <= 0.012:
                    break
                if distance <= best_distance - 0.010:
                    best_distance = distance
                    last_progress_step = step
                elif step - last_progress_step >= stall_steps:
                    for rid in carriers:
                        self.controllers[rid].set_motor_commands(STOP)
                    record_waypoint_failure("stalled", current, distance)
                    raise RuntimeError(
                        f"ARENA_LOADED_WAYPOINT_STALLED: {spec.cargo_id}/{phase}"
                        f" remaining={distance:.4f}"
                    )
                along_remaining = float(np.dot(error, segment_direction))
                cross_remaining = float(np.dot(error, segment_normal))
                if (not braking and along_remaining > 0.0
                        and forward_speed > 0.025
                        and along_remaining <= max(0.080, forward_speed * 0.75)):
                    braking = True
                    stopped_steps = 0
                if braking:
                    for rid in carriers:
                        self.controllers[rid].set_motor_commands(STOP)
                    stopped_steps = stopped_steps + 1 if forward_speed <= 0.012 else 0
                    self._physics_step_for(self.controllers[carriers[0]])
                    if not all(
                        self._cargo_constraint_active(spec.cargo_id, rid) for rid in carriers
                    ):
                        raise RuntimeError(f"PAYLOAD_DROPPED: {spec.cargo_id}")
                    if stopped_steps >= max(1, int(round(0.12 / dt))):
                        braking = False
                        creep_mode = True
                    if self.frame_callback is not None and step % callback_every == 0:
                        self.frame_callback()
                    continue
                direction = (
                    segment_direction * max(0.0, min(
                        1.0, along_remaining / (0.080 if creep_mode else 0.22),
                    ))
                    + segment_normal * max(-0.65, min(0.65, cross_remaining / 0.12))
                )
                forward_component = float(np.dot(direction, travel_axis))
                if forward_component < 0.0:
                    direction = direction - forward_component * travel_axis
                norm = float(np.linalg.norm(direction))
                if norm < 1e-9:
                    if distance <= 0.020:
                        break
                    for rid in carriers:
                        self.controllers[rid].set_motor_commands(STOP)
                    record_waypoint_failure("overshot", current, distance)
                    raise RuntimeError(
                        f"ARENA_LOADED_WAYPOINT_OVERSHOT: {spec.cargo_id}/{phase}"
                    )
                max_scale = 0.10 if creep_mode else 0.42
                min_scale = 0.050 if creep_mode else 0.025
                scale = min(max_scale, max(min_scale, distance / 0.22 * max_scale))
                for rid in carriers:
                    robot_yaw = float(self.controllers[rid].base_rpy()[2])
                    cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)
                    local_forward = cy * direction[0] + sy * direction[1]
                    local_left = -sy * direction[0] + cy * direction[1]
                    pattern = FORWARD_PATTERN * local_forward + LEFT_PATTERN * local_left
                    peak = max(1.0, float(np.max(np.abs(pattern))))
                    self.controllers[rid].set_motor_commands(pattern / peak * scale)
                self._physics_step_for(self.controllers[carriers[0]])
                if not all(
                    self._cargo_constraint_active(spec.cargo_id, rid) for rid in carriers
                ):
                    raise RuntimeError(f"PAYLOAD_DROPPED: {spec.cargo_id}")
                if self.frame_callback is not None and step % callback_every == 0:
                    self.frame_callback()
            else:
                for rid in carriers:
                    self.controllers[rid].set_motor_commands(STOP)
                current = np.asarray(
                    cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                )
                record_waypoint_failure(
                    "timeout", current, float(np.linalg.norm(target_xy - current)),
                )
                raise RuntimeError(f"ARENA_LOADED_WAYPOINT_TIMEOUT: {spec.cargo_id}/{phase}")
            for rid in carriers:
                self.controllers[rid].set_motor_commands(STOP)
            remaining = float(np.linalg.norm(
                target_xy - np.asarray(
                    cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                )
            ))
            if remaining > 0.020:
                current = np.asarray(
                    cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                )
                record_waypoint_failure("incomplete", current, remaining)
                raise RuntimeError(
                    f"ARENA_LOADED_WAYPOINT_INCOMPLETE: {spec.cargo_id}/{phase}"
                    f" remaining={remaining:.4f}"
                )
            self._record_warehouse_phase(
                f"{phase}_complete", cargo_id=spec.cargo_id,
                movement_mode="forward_lateral_vector",
                payload_position=[round(float(v), 4) for v in cargo_pose(
                    self.data, self.model, spec,
                )["position"]],
            )

        def symmetric_yaw_error(yaw: float) -> float:
            yaw -= float(spec.goal_yaw_rad)
            candidates = (
                self._wrap_angle(-yaw),
                self._wrap_angle(math.pi - yaw),
                self._wrap_angle(-math.pi - yaw),
            )
            return min(candidates, key=abs)

        route = list(route_zones or ("A", "B"))
        leg_index = 1
        while leg_index < len(route):
            zone_id = route[leg_index]
            target = self.warehouse_zone_positions[spec.cargo_id][str(zone_id)]
            cargo_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            self._record_warehouse_phase(
                "warehouse_route_leg_started", cargo_id=spec.cargo_id,
                leg_index=leg_index, source_zone=route[leg_index - 1],
                destination_zone=zone_id,
            )
            loaded_obstacles = self._warehouse_navigation_observations(
                exclude_robots=carriers,
                exclude_cargo=spec.cargo_id,
            )
            if self.warehouse_layout == "arena":
                local_path = plan_monotone_visibility_path(
                    cargo_xy, target[:2], loaded_obstacles,
                    footprint_xy=self._warehouse_loaded_footprint(spec),
                    progress_axis=tuple(travel_axis),
                    bounds=self.warehouse_navigation_bounds,
                )
            else:
                local_path = plan_local_path(
                    cargo_xy, target[:2], loaded_obstacles,
                    footprint_xy=self._warehouse_loaded_footprint(spec),
                    axis_costs=(1.0, 3.0), bounds=(0.62, 3.08, -2.15, 2.15),
                )
            self._record_warehouse_phase(
                "warehouse_local_path_planned",
                cargo_id=spec.cargo_id,
                leg_index=leg_index,
                waypoints=[[round(float(v), 4) for v in point] for point in local_path],
                waypoint_count=len(local_path),
                terrain_count=len(self.warehouse_terrain_specs),
                occupancy_count=len(loaded_obstacles),
                planner=(
                    "directed_inflated_visibility_graph"
                    if self.warehouse_layout == "arena"
                    else "inflated_occupancy_astar_4n"
                ),
            )
            self._record_warehouse_phase(
                "controller_route_query",
                cargo_id=spec.cargo_id,
                source="sim_controller_geometry", actor_evidence=False,
                leg_index=leg_index,
                observed_features=[item.terrain_id for item in self.warehouse_terrain_specs],
                hard_obstacles=[
                    item.terrain_id for item in self.warehouse_terrain_specs
                    if not item.traversable
                ],
                waypoint_count=len(local_path),
                planner=(
                    "directed_inflated_visibility_graph"
                    if self.warehouse_layout == "arena"
                    else "inflated_occupancy_astar_4n"
                ),
            )
            for path_index, waypoint in enumerate(local_path[1:], start=1):
                if self.warehouse_layout == "arena":
                    move_arena_carriers_to(
                        waypoint, f"warehouse_route_leg_{leg_index}_path_{path_index}_vector",
                    )
                    continue
                cargo_xy = np.asarray(
                    cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                )
                if abs(float(waypoint[0]) - float(cargo_xy[0])) > 0.010:
                    phase = (
                        f"warehouse_route_leg_{leg_index}_longitudinal"
                        if path_index == 1 else
                        f"warehouse_route_leg_{leg_index}_path_{path_index}_x"
                    )
                    move_carriers_by("x", float(waypoint[0]) - float(cargo_xy[0]), phase)
                cargo_xy = np.asarray(
                    cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                )
                if abs(float(waypoint[1]) - float(cargo_xy[1])) > 0.010:
                    lateral_chunk = 0
                    while True:
                        cargo_xy = np.asarray(
                            cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
                        )
                        y_error = float(waypoint[1]) - float(cargo_xy[1])
                        if abs(y_error) <= 0.010:
                            break
                        lateral_chunk += 1
                        if lateral_chunk > (100 if self.warehouse_layout == "arena" else 24):
                            raise RuntimeError(
                                f"LOADED_ROUTE_LATERAL_BUDGET_EXCEEDED: {spec.cargo_id}"
                            )
                        step_delta = math.copysign(min(0.08, abs(y_error)), y_error)
                        phase = (
                            f"warehouse_route_leg_{leg_index}_lateral_chunk_{lateral_chunk}"
                            if path_index == 1 else
                            f"warehouse_route_leg_{leg_index}_path_{path_index}_y_chunk_{lateral_chunk}"
                        )
                        move_carriers_by("y", step_delta, phase)
            requested_destination, goal_revision, _goal_reason = self._warehouse_goal_snapshot()
            if requested_destination and requested_destination != route[-1]:
                extension = ((str(zone_id), requested_destination) if self.warehouse_layout == "arena"
                             else warehouse_route_zones(str(zone_id), requested_destination))
                route = route[:leg_index + 1] + list(extension[1:])
                spec = replace(
                    spec,
                    goal_xyz=self.warehouse_zone_positions[spec.cargo_id][requested_destination],
                )
                self._replace_warehouse_spec(spec)
                if self.warehouse_mission is not None:
                    self.warehouse_mission = {**self.warehouse_mission, "route": list(route)}
                self._record_warehouse_phase(
                    "route_replanned",
                    cargo_id=spec.cargo_id,
                    from_zone=zone_id,
                    destination_zone=requested_destination,
                    revision=goal_revision,
                    route=list(route),
                    reason="destination_changed_during_execution",
                )
            if leg_index < len(route) - 1:
                dt = float(self.model.opt.timestep)
                callback_every = max(1, int(round(0.05 / dt)))
                yaw = float(cargo_pose(self.data, self.model, spec)["yaw"])
                if abs(symmetric_yaw_error(yaw)) > math.radians(6.0):
                    self._record_warehouse_phase(
                        "warehouse_checkpoint_yaw_alignment",
                        cargo_id=spec.cargo_id, zone_id=zone_id,
                        movement_mode="differential_payload_rotation",
                    )
                    for step in range(max(1, int(round(4.0 / dt)))):
                        yaw = float(cargo_pose(self.data, self.model, spec)["yaw"])
                        error = symmetric_yaw_error(yaw)
                        if abs(error) <= math.radians(6.0):
                            break
                        scale = min(0.22, max(0.10, abs(error) / math.radians(18.0) * 0.22))
                        turn = (1.0 if error > 0.0 else -1.0) * (1.0 if self.warehouse_layout == "arena" or math.cos(travel_heading) >= 0 else -1.0)
                        self.controllers[carriers[0]].set_motor_commands(FORWARD_PATTERN * scale * turn)
                        self.controllers[carriers[1]].set_motor_commands(-FORWARD_PATTERN * scale * turn)
                        self._physics_step_for(self.controllers[carriers[0]])
                        if self.frame_callback is not None and step % callback_every == 0:
                            self.frame_callback()
                    else:
                        raise RuntimeError(
                            f"CHECKPOINT_YAW_ALIGNMENT_TIMEOUT: {spec.cargo_id}/{zone_id}"
                        )
                    for rid in carriers:
                        self.controllers[rid].set_motor_commands(STOP)
                    for _ in range(max(1, int(round(0.12 / dt)))):
                        self._physics_step_for(self.controllers[carriers[0]])
                    self._record_warehouse_phase(
                        "warehouse_checkpoint_yaw_alignment_complete",
                        cargo_id=spec.cargo_id, zone_id=zone_id,
                        yaw_deg=round(math.degrees(float(cargo_pose(self.data, self.model, spec)["yaw"])), 2),
                    )
            cargo_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            if abs(float(target[0]) - float(cargo_xy[0])) > 0.025:
                move_carriers_by(
                    "x", float(target[0]) - float(cargo_xy[0]),
                    f"warehouse_route_leg_{leg_index}_final_x",
                )
            cargo_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            if abs(float(target[1]) - float(cargo_xy[1])) > 0.025:
                move_carriers_by(
                    "y", float(target[1]) - float(cargo_xy[1]),
                    f"warehouse_route_leg_{leg_index}_final_y",
                )
            final_leg_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            waypoint_error = math.hypot(
                float(final_leg_xy[0]) - float(target[0]),
                float(final_leg_xy[1]) - float(target[1]),
            )
            if waypoint_error > 0.12:
                raise RuntimeError(
                    f"WAREHOUSE_WAYPOINT_NOT_REACHED: {spec.cargo_id}/{zone_id} "
                    f"error={waypoint_error:.3f}m"
                )
            requested_destination, goal_revision, _goal_reason = self._warehouse_goal_snapshot()
            if requested_destination and requested_destination != route[-1]:
                extension = ((str(zone_id), requested_destination) if self.warehouse_layout == "arena"
                             else warehouse_route_zones(str(zone_id), requested_destination))
                route = route[:leg_index + 1] + list(extension[1:])
                spec = replace(
                    spec,
                    goal_xyz=self.warehouse_zone_positions[spec.cargo_id][requested_destination],
                )
                self._replace_warehouse_spec(spec)
                if self.warehouse_mission is not None:
                    self.warehouse_mission = {**self.warehouse_mission, "route": list(route)}
                self._record_warehouse_phase(
                    "route_replanned",
                    cargo_id=spec.cargo_id,
                    from_zone=zone_id,
                    destination_zone=requested_destination,
                    revision=goal_revision,
                    route=list(route),
                    reason="destination_changed_at_checkpoint",
                )
            self._record_warehouse_phase(
                "warehouse_waypoint_reached", cargo_id=spec.cargo_id,
                leg_index=leg_index, zone_id=zone_id,
                final=leg_index == len(route) - 1,
                position_error_m=round(waypoint_error, 4),
            )
            leg_index += 1
        # Differential longitudinal wheel motion rotates the shared load about
        # its centre without installing a pose. This is only needed when the
        # randomized approach leaves a material yaw residual.
        yaw_tolerance = min(float(spec.yaw_tolerance_rad) * 0.55, math.radians(6.0))
        dt = float(self.model.opt.timestep)
        callback_every = max(1, int(round(0.05 / dt)))
        yaw_before = float(cargo_pose(self.data, self.model, spec)["yaw"])
        if abs(symmetric_yaw_error(yaw_before)) > yaw_tolerance:
            self._record_warehouse_phase(
                "warehouse_payload_yaw_alignment",
                cargo_id=spec.cargo_id,
                movement_mode="differential_payload_rotation",
                target_yaw_deg=math.degrees(spec.goal_yaw_rad),
            )
            for step in range(max(1, int(round(4.0 / dt)))):
                yaw = float(cargo_pose(self.data, self.model, spec)["yaw"])
                error = symmetric_yaw_error(yaw)
                if abs(error) <= yaw_tolerance:
                    break
                scale = min(0.22, max(0.10, abs(error) / math.radians(18.0) * 0.22))
                direction = (1.0 if error > 0.0 else -1.0) * (1.0 if self.warehouse_layout == "arena" or math.cos(travel_heading) >= 0 else -1.0)
                self.controllers[carriers[0]].set_motor_commands(FORWARD_PATTERN * scale * direction)
                self.controllers[carriers[1]].set_motor_commands(-FORWARD_PATTERN * scale * direction)
                self._physics_step_for(self.controllers[carriers[0]])
                if not all(
                    self._cargo_constraint_active(spec.cargo_id, rid)
                    for rid in carriers
                ):
                    raise RuntimeError(f"PAYLOAD_DROPPED: {spec.cargo_id}")
                if self.frame_callback is not None and step % callback_every == 0:
                    self.frame_callback()
            else:
                raise RuntimeError(
                    f"PAYLOAD_YAW_ALIGNMENT_TIMEOUT: {spec.cargo_id} "
                    f"yaw={math.degrees(float(cargo_pose(self.data, self.model, spec)['yaw'])):.2f}deg"
                )
            for rid in carriers:
                self.controllers[rid].set_motor_commands(STOP)
            for _ in range(max(1, int(round(0.12 / dt)))):
                self._physics_step_for(self.controllers[carriers[0]])
            self._record_warehouse_phase(
                "warehouse_payload_yaw_alignment_complete",
                cargo_id=spec.cargo_id,
                movement_mode="differential_payload_rotation",
                yaw_deg=round(math.degrees(float(cargo_pose(self.data, self.model, spec)["yaw"])), 2),
            )
            final_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            if abs(float(spec.goal_xyz[0]) - float(final_xy[0])) > 0.020:
                move_carriers_by(
                    "x", float(spec.goal_xyz[0]) - float(final_xy[0]),
                    "warehouse_final_center_x_alignment",
                )
            final_xy = np.asarray(
                cargo_pose(self.data, self.model, spec)["position"][:2], dtype=float,
            )
            if abs(float(spec.goal_xyz[1]) - float(final_xy[1])) > 0.020:
                move_carriers_by(
                    "y", float(spec.goal_xyz[1]) - float(final_xy[1]),
                    "warehouse_final_center_y_alignment",
                )
        if not all(self._cargo_constraint_active(spec.cargo_id, rid) for rid in carriers):
            raise RuntimeError(f"PAYLOAD_DROPPED: {spec.cargo_id}")
        self._record_warehouse_phase(
            "cooperative_carry_complete", cargo_id=spec.cargo_id,
            transport_mode="DYNAMIC_INWARD_BODY_GRASP_MECANUM",
        )
        self._warehouse_crew.activities.update({rid: "release" for rid in carriers})

        self._team_joint_move_servos(
            grasp_by_rid, 0.72, settle_s=0.20,
        )
        self._team_joint_move_servos(
            {rid: {1: precision.GRIPPER_OPEN} for rid in carriers}, 0.50,
        )
        self._release_cargo_constraints(spec.cargo_id)
        for _ in range(max(1, int(round(0.50 / float(self.model.opt.timestep))))):
            self._physics_step_for(self.controllers["r1"])
        if self.warehouse_layout == "arena":
            # Clear the opened fingers above the delivered load before moving
            # diagonal-facing chassis away from the endpoint.
            self._team_joint_move_servos(hover_by_rid, .72, settle_s=.20)
        released_pose = cargo_pose(self.data, self.model, spec)
        released_yaw = float(released_pose["yaw"])
        if self.warehouse_layout == "arena":
            released_axis_x = -math.sin(released_yaw)
            self._warehouse_drive_group_axis("x", {
                carriers[0]: float(self.controllers[carriers[0]].base_xyz()[0])-released_axis_x*.14,
                carriers[1]: float(self.controllers[carriers[1]].base_xyz()[0])+released_axis_x*.14},
                phase="warehouse_post_release_clearance_x", tolerance_m=.012, max_sim_s=7.)
        released_axis_y = math.cos(released_yaw)
        clearance_targets = {
            carriers[0]: float(self.controllers[carriers[0]].base_xyz()[1]) - released_axis_y * 0.14,
            carriers[1]: float(self.controllers[carriers[1]].base_xyz()[1]) + released_axis_y * 0.14,
        }
        self._warehouse_drive_group_axis(
            "y", clearance_targets,
            phase="warehouse_post_release_clearance",
            tolerance_m=0.012, max_sim_s=7.0,
        )
        self._warehouse_crew.activities.update({rid: "ready" for rid in carriers})
        pose = cargo_pose(self.data, self.model, spec)
        evaluation = evaluate_cargo_delivery(pose, spec)
        jadr = int(self.model.jnt_dofadr[self.warehouse_joint_ids[spec.cargo_id]])
        velocity = np.asarray(self.data.qvel[jadr:jadr + 6], dtype=float)
        stable = (
            float(np.linalg.norm(velocity[:3])) <= 0.01
            and float(np.linalg.norm(velocity[3:])) <= 0.03
        )
        if not bool(evaluation["success"]) or not stable:
            raise RuntimeError(
                f"DESTINATION_NOT_REACHED: {spec.cargo_id} {evaluation} stable={stable}"
            )
        self._record_warehouse_phase(
            "cargo_delivered",
            cargo_id=spec.cargo_id,
            carriers=list(carriers),
            scout=spec.scout,
            destination_zone=(self._warehouse_goal_snapshot()[0] or route[-1]),
            goal_revision=self._warehouse_goal_snapshot()[1],
            stable=stable,
            transport_mode="DYNAMIC_INWARD_BODY_GRASP_MECANUM",
        )

    def _warehouse_drive_robot_forward_to(
        self,
        rid: str,
        target_xy: Sequence[float],
        *,
        phase: str,
        tolerance_m: float = 0.018,
        max_sim_s: float = 18.0,
    ) -> None:
        """Navigate one empty robot by rotate-then-forward, never pure strafe."""
        from sim.crew_navigation import ForwardPathController
        robot = self.controllers[rid]
        controller = ForwardPathController(
            (tuple(robot.base_xyz()[:2]), tuple(target_xy[:2])),
            tolerance_m=max(.035, tolerance_m),
        )
        dt = float(self.model.opt.timestep)
        self._record_warehouse_phase(phase, robot_id=rid, movement_mode="face_then_forward",
                                     target_xy=list(target_xy[:2]))
        for step in range(max(1, int(max_sim_s/dt))):
            robot.set_motor_commands(controller.update(robot.base_xyz()[:2], robot.base_rpy()[2], dt))
            if controller.done:
                break
            self._physics_step_for(robot)
            if self.frame_callback and step % 40 == 0:
                self.frame_callback()
        else:
            robot.set_motor_commands(STOP)
            raise RuntimeError(f"FORWARD_NAVIGATION_TIMEOUT: {rid}/{phase}")
        robot.set_motor_commands(STOP)
        self._record_warehouse_phase(f"{phase}_complete", robot_id=rid,
                                     movement_mode="face_then_forward")

    def _warehouse_drive_group_axis(
        self,
        axis: str,
        targets: Mapping[str, float],
        *,
        phase: str,
        tolerance_m: float = 0.015,
        max_sim_s: float = 12.0,
    ) -> None:
        """Move warehouse robots continuously, using mecanum strafe for Y."""
        if axis not in {"x", "y"}:
            raise ValueError(f"unsupported warehouse axis: {axis}")
        ids = tuple(targets)
        index = 0 if axis == "x" else 1
        other_index = 1 - index
        starts = {
            rid: float(self.controllers[rid].base_xyz()[index])
            for rid in ids
        }
        cross_starts = {
            rid: float(self.controllers[rid].base_xyz()[other_index])
            for rid in ids
        }
        dt = float(self.model.opt.timestep)
        callback_every = max(1, int(round(0.05 / dt)))
        # Commands persist on each controller in the shared world. Explicitly
        # stop participants outside this group before every warehouse segment.
        for rid in self.robot_ids:
            if (rid not in ids and rid not in getattr(getattr(self, "_warehouse_crew", None), "tasks", {})
                    and rid not in getattr(getattr(self, "_mixed_engine", None), "solo_tasks", {})):
                self.controllers[rid].set_motor_commands(STOP)
        self._record_warehouse_phase(
            phase,
            movement_mode=("mecanum_lateral" if axis == "y" else "wheel_longitudinal"),
            robot_ids=list(ids),
            starts={rid: round(value, 4) for rid, value in starts.items()},
            targets={rid: round(float(targets[rid]), 4) for rid in ids},
        )
        for step in range(max(1, int(round(float(max_sim_s) / dt)))):
            done = True
            progresses: list[float] = []
            for rid in ids:
                robot = self.controllers[rid]
                base_xy = robot.base_xyz()[:2]
                current = float(base_xy[index])
                error = float(targets[rid]) - current
                travel = float(targets[rid]) - starts[rid]
                if abs(travel) > 1e-6:
                    progresses.append(max(0.0, min(1.0, (current - starts[rid]) / travel)))
                if abs(error) <= float(tolerance_m):
                    robot.set_motor_commands(STOP)
                    continue
                done = False
                # A large minimum command causes a last-centimetre limit cycle;
                # retain only enough floor to overcome static friction.
                scale = min(0.78, max(0.10, abs(error) / 0.20 * 0.78))
                cross_error = cross_starts[rid] - float(base_xy[other_index])
                world_velocity = np.zeros(2, dtype=float)
                world_velocity[index] = 1.0 if error > 0.0 else -1.0
                world_velocity[other_index] = max(-0.55, min(0.55, cross_error / 0.12))
                world_velocity /= max(1.0, float(np.linalg.norm(world_velocity)))
                yaw = float(robot.base_rpy()[2])
                cy, sy = math.cos(yaw), math.sin(yaw)
                local_forward = cy * world_velocity[0] + sy * world_velocity[1]
                local_left = -sy * world_velocity[0] + cy * world_velocity[1]
                pattern = FORWARD_PATTERN * local_forward + LEFT_PATTERN * local_left
                peak = max(1.0, float(np.max(np.abs(pattern))))
                robot.set_motor_commands(pattern / peak * scale)
            if done:
                break
            self._physics_step_for(self.controllers[ids[0]])
            if self.frame_callback is not None and step % callback_every == 0:
                self.frame_callback()
        else:
            actual = {
                rid: round(float(self.controllers[rid].base_xyz()[0 if axis == "x" else 1]), 4)
                for rid in ids
            }
            for rid in ids:
                self.controllers[rid].set_motor_commands(STOP)
            raise RuntimeError(f"warehouse {phase} did not converge: {actual}")
        for rid in ids:
            self.controllers[rid].set_motor_commands(STOP)
        for _ in range(max(1, int(round(0.16 / dt)))):
            self._physics_step_for(self.controllers[ids[0]])
        self._record_warehouse_phase(
            f"{phase}_complete",
            movement_mode=("mecanum_lateral" if axis == "y" else "wheel_longitudinal"),
            robot_xy={
                rid: [round(float(v), 4) for v in self.controllers[rid].base_xyz()[:2]]
                for rid in ids
            },
            yaw_deg={
                rid: round(math.degrees(self.controllers[rid].base_rpy()[2]), 2)
                for rid in ids
            },
        )

    def _cooperative_warehouse_transfer(self, mission: Mapping[str, object]) -> ActionResult:
        mission_id = str(mission.get("mission_id") or "")
        source = str(mission.get("source_zone") or "A").upper()
        destination = str(mission.get("destination_zone") or "B").upper()
        selector = str(mission.get("selector") or "all").lower()
        try:
            route = ((source, destination) if self.warehouse_layout == "arena" else warehouse_route_zones(source, destination))
        except ValueError as exc:
            return ActionResult(
                False,
                "team_zone_transfer",
                f"WAREHOUSE_ROUTE_INVALID: {exc}",
                self.state(),
            )
        if selector != "all":
            return ActionResult(
                False, "team_zone_transfer",
                "WAREHOUSE_SELECTOR_UNSUPPORTED: current TEAM route moves all cargo in the source zone",
                self.state(),
            )
        self.warehouse_trace = []
        self.warehouse_status = "RUNNING"
        self.warehouse_mission = {**dict(mission), "route": list(route),
                                  "decision_mode": "central_baseline",
                                  "decision_authority": "omniscient_python_controller"}
        self._warehouse_carrier_jobs = {rid: 0 for rid in self.robot_ids}
        # Clear any awards left by an earlier stateful mission. Geometry and
        # object identity persist; roles never do.
        self.warehouse_specs = tuple(
            replace(spec, carriers=(), scout="") for spec in self.warehouse_specs
        )
        self.warehouse_spec_by_id = {
            spec.cargo_id: spec for spec in self.warehouse_specs
        }
        with self._warehouse_goal_lock:
            self._warehouse_requested_destination = destination
            self._warehouse_goal_revision += 1
            self._warehouse_goal_reason = "initial_command"
            initial_revision = self._warehouse_goal_revision
        selected_ids = tuple(spec.cargo_id for spec in self.warehouse_specs)
        self._sync_warehouse_goal_specs(selected_ids, destination)
        observation = self.observe_warehouse_scene()
        self._record_warehouse_phase(
            "batch_started",
            mission_id=mission_id,
            source_zone=source,
            destination_zone=destination,
            route=list(route),
            selected_ids=list(selected_ids),
            goal_revision=initial_revision,
            observation_id=observation["observation_id"],
            terrain=observation["terrain"],
        )
        try:
            for cargo_id in selected_ids:
                spec = self.warehouse_spec_by_id[cargo_id]
                pose = cargo_pose(self.data, self.model, spec)
                actual_zone = self._warehouse_zone_for_position(
                    spec.cargo_id, pose["position"],
                )
                if actual_zone != source:
                    raise RuntimeError(
                        f"CARGO_NOT_IN_SOURCE_ZONE: {spec.cargo_id} "
                        f"expected={source} actual={actual_zone}"
                    )
            attempts = 0
            max_attempts = max(8, len(selected_ids) * 6)
            final_revision = initial_revision
            while attempts < max_attempts:
                active_destination, active_revision, _goal_reason = self._warehouse_goal_snapshot()
                if active_destination is None:
                    raise RuntimeError("WAREHOUSE_GOAL_MISSING")
                final_revision = active_revision
                self._sync_warehouse_goal_specs(selected_ids, active_destination)
                pending = []
                for cargo_id in selected_ids:
                    spec = self.warehouse_spec_by_id[cargo_id]
                    pose = cargo_pose(self.data, self.model, spec)
                    current_zone = self._warehouse_zone_for_position(cargo_id, pose["position"])
                    if current_zone != active_destination:
                        if current_zone == "TRANSIT":
                            raise RuntimeError(f"CARGO_LOCATION_UNCERTAIN: {cargo_id}")
                        runtime_spec = replace(
                            spec,
                            start_xyz=tuple(float(v) for v in pose["position"]),
                            goal_xyz=self.warehouse_zone_positions[cargo_id][active_destination],
                            carriers=(),
                            scout="",
                        )
                        role_plan = self._plan_warehouse_roles(runtime_spec)
                        travel_cost = math.dist(
                            tuple(float(v) for v in pose["position"][:2]),
                            runtime_spec.goal_xyz[:2],
                        )
                        pending.append((
                            role_plan.total_cost + travel_cost,
                            cargo_id,
                            current_zone,
                            runtime_spec,
                            role_plan,
                        ))
                if not pending:
                    latest_destination, latest_revision, _ = self._warehouse_goal_snapshot()
                    if latest_revision == active_revision and latest_destination == active_destination:
                        break
                    self._record_warehouse_phase(
                        "goal_change_detected",
                        previous_destination=active_destination,
                        destination_zone=latest_destination,
                        revision=latest_revision,
                    )
                    continue
                _cost, cargo_id, current_zone, runtime_spec, role_plan = min(pending)
                spec, _award_cost = self._award_warehouse_roles(runtime_spec, role_plan)
                cargo_route = ((current_zone, active_destination) if self.warehouse_layout == "arena"
                               else warehouse_route_zones(current_zone, active_destination))
                self._record_warehouse_phase(
                    "adaptive_task_selected",
                    cargo_id=cargo_id,
                    current_zone=current_zone,
                    destination_zone=active_destination,
                    route=list(cargo_route),
                    goal_revision=active_revision,
                    selection_cost=round(float(_cost), 4),
                )
                self._transport_warehouse_cargo(spec, cargo_route)
                attempts += 1
            else:
                raise RuntimeError("ADAPTIVE_REPLAN_LIMIT_EXCEEDED")
            state = self.warehouse_state()
            if not bool(state["success"]):
                raise RuntimeError(
                    f"BATCH_INCOMPLETE remaining={state['remaining_ids']}"
                )
            self.warehouse_status = "SUCCESS"
            self._record_warehouse_phase(
                "batch_complete",
                mission_id=mission_id,
                moved_count=len(selected_ids),
                destination_zone=self._warehouse_goal_snapshot()[0],
                goal_revision=final_revision,
            )
            return ActionResult(
                True,
                "team_zone_transfer",
                f"all {len(selected_ids)} cargo items moved to {self._warehouse_goal_snapshot()[0]} after adaptive planning",
                self.state(),
            )
        except Exception as exc:
            for robot in self.controllers.values():
                robot.set_motor_commands(STOP)
            for spec in self.warehouse_specs:
                try:
                    self._release_cargo_constraints(spec.cargo_id)
                except Exception:
                    pass
            self.warehouse_status = "FAILED"
            self._record_warehouse_phase(
                "batch_failed", mission_id=mission_id,
                error=f"{type(exc).__name__}: {exc}",
            )
            return ActionResult(
                False,
                "team_zone_transfer",
                f"{type(exc).__name__}: {exc}",
                self.state(),
            )

    def _team_tower(self) -> ActionResult:
        """Prepare all three blocks concurrently; serialize only stack support.

        Each prep thread owns a distinct robot and block.  MuJoCo stepping stays
        safe because the existing fine-grained ``physics_lock`` protects every
        shared physics step, while motor commands for all controllers are
        integrated on every step.  Thus the robots really move during the same
        simulated time window instead of waiting for three complete commands.
        """
        # Keep every robot in its own Y lane during concurrent pickup. Block
        # colours are randomized between rows, so ownership must follow row
        # order rather than a fixed colour mapping: r1 takes the lowest-Y block,
        # r2 the middle, r3 the highest. This lets all three drive at once
        # without crossing through one another.
        block_rows = sorted(
            (
                float(self.controllers["r1"].body_xyz(f"{color}_block")[1]),
                color,
            )
            for color in ("red", "blue", "yellow")
        )
        assignments = {
            rid: (color, None)
            for rid, (_, color) in zip(ROBOT_IDS, block_rows)
        }
        started_at = wall_time.monotonic()
        timings: dict[str, dict[str, float]] = {
            rid: {"start_s": 0.0} for rid in assignments
        }

        # Prepare and grasp all three blocks in one shared physical timeline.
        # Previous code only parallelized the first straight drive; every arm
        # motion, fine alignment, grasp and parking step then ran R1 -> R2 -> R3,
        # which is exactly the one-at-a-time behaviour visible in TEAM.
        precision = self._precision_module()
        try:
            folded = {
                rid: {**SEARCH_POSE, 1: precision.GRIPPER_OPEN, 6: 1500}
                for rid in assignments
            }
            self._team_joint_move_servos(folded, 0.85, settle_s=0.10)

            # Each robot owns one Y row, so these rotations/drives are mutually
            # clear and can happen at the same time without mecanum strafing.
            row_targets = {
                rid: float(self.controllers[rid].body_xyz(f"{color}_block")[1])
                for rid, (color, _) in assignments.items()
            }
            self._team_joint_move_base_axis("y", row_targets, tolerance_m=0.012)
            self._team_joint_forward_to_standoffs(assignments, tolerance_m=0.012)

            hover = self._hover_pose()
            self._team_joint_move_servos(
                {rid: {1: precision.GRIPPER_OPEN, 6: 1500, **hover} for rid in assignments},
                0.85,
                settle_s=0.18,
            )

            # One tighter shared correction after extending the arms. The base
            # origins shift slightly as the arm CoM changes, so doing this after
            # hover is what keeps all three grippers centered together.
            row_targets = {
                rid: float(self.controllers[rid].body_xyz(f"{color}_block")[1])
                for rid, (color, _) in assignments.items()
            }
            self._team_joint_move_base_axis("y", row_targets, tolerance_m=0.006)
            x_targets = {
                rid: float(self.controllers[rid].body_xyz(f"{color}_block")[0]) - TEAM_APPROACH_STANDOFF_M
                for rid, (color, _) in assignments.items()
            }
            self._team_joint_move_base_axis("x", x_targets, tolerance_m=0.006)

            for rid, (color, _) in assignments.items():
                robot = self.controllers[rid]
                block = robot.body_xyz(f"{color}_block")
                base = robot.base_xyz()
                forward_error = float(block[0] - base[0] - TEAM_APPROACH_STANDOFF_M)
                lateral_error = float(block[1] - base[1])
                if abs(forward_error) > 0.008 or abs(lateral_error) > 0.008:
                    raise RuntimeError(
                        f"{rid} parallel pregrasp alignment failed for {color}: "
                        f"forward_error={forward_error:.4f} lateral_error={lateral_error:.4f}"
                    )
                robot.pregrasp_color = color

            self._team_joint_move_servos(
                {
                    rid: {1: precision.GRIPPER_OPEN, 6: 1500, **precision.CAPTURE_ARM_POSE}
                    for rid in assignments
                },
                0.70,
                settle_s=0.16,
            )

            radius = precision.CAPTURE_FINGERTIP_RADIUS_CM
            grasp = precision.solve_ik(
                radius, precision.CAPTURE_GRASP_HEIGHT_CM, precision.CAPTURE_GRASP_PITCH_DEG
            )
            self._team_joint_move_servos(
                {rid: {1: precision.GRIPPER_OPEN, 6: 1500, **hover} for rid in assignments},
                0.85,
            )
            for height in precision.GRASP_DESCENT_HEIGHTS_CM:
                if height <= precision.CAPTURE_GRASP_HEIGHT_CM + 0.15:
                    continue
                pose = precision.solve_ik(radius, height, precision.CAPTURE_GRASP_PITCH_DEG)
                self._team_joint_move_servos({rid: pose for rid in assignments}, 0.40)
            self._team_joint_move_servos({rid: grasp for rid in assignments}, 0.52)
            self._team_joint_move_servos(
                {rid: {1: precision.GRIPPER_CLOSE} for rid in assignments},
                0.80,
                settle_s=0.22,
            )

            for rid, (color, _) in assignments.items():
                robot = self.controllers[rid]
                contact = robot.finger_block_contact(color)
                if not bool(contact.get("bilateral")):
                    raise RuntimeError(
                        f"parallel grasp failed for {rid}/{color}: bilateral=false"
                    )
            for rid, (color, _) in assignments.items():
                self._activate_grasp_constraint(self.controllers[rid], color)

            self._team_joint_move_servos(
                {rid: hover for rid in assignments}, 0.72, settle_s=0.30,
            )
            for rid, (color, _) in assignments.items():
                robot = self.controllers[rid]
                block_z = float(robot.body_xyz(f"{color}_block")[2])
                if block_z <= TARGET_BLOCK_LIFT_CENTER_M:
                    raise RuntimeError(
                        f"parallel lift failed for {rid}/{color}: z={block_z:.3f}m"
                    )
                robot.grasp_color = color
                robot.pregrasp_color = color
                entry = robot.spatial_memory.get(color)
                if isinstance(entry, dict):
                    entry["relation"] = "HELD"
                timings[rid]["prepared_s"] = wall_time.monotonic() - started_at
        except Exception as exc:
            for rid, (color, _) in assignments.items():
                robot = self.controllers[rid]
                robot.set_motor_commands(STOP)
                if self._grasp_constraint_active(robot, color) and robot.grasp_color != color:
                    self._release_grasp_constraint(robot, color)
            return ActionResult(
                False, "team_tower", f"parallel physical preparation failed: {exc}",
                {**self.state(), "team_tower_timing": timings},
            )

        # All three now physically hold their own blocks. Move them back to
        # their parking rows while still holding the blocks so the stack lane is
        # clear. These are ordinary wheel motions with grasp verification.
        color_owner = {color: rid for rid, (color, _) in assignments.items()}
        held_colors = {rid: color for rid, (color, _) in assignments.items()}
        try:
            self._team_joint_move_base_axis(
                "x", {rid: TEAM_SAFE_CORRIDOR_X for rid in assignments},
                tolerance_m=0.010, held_colors=held_colors,
            )
            self._team_joint_move_base_axis(
                "y", {rid: float(DEFAULT_SPAWNS[rid][1]) for rid in assignments},
                tolerance_m=0.010, held_colors=held_colors,
            )
            self._team_joint_move_base_axis(
                "x", {rid: TEAM_PARK_X for rid in assignments},
                tolerance_m=0.008, held_colors=held_colors,
            )
        except Exception as exc:
            for peer in self.controllers.values():
                peer.set_motor_commands(STOP)
            return ActionResult(
                False, "team_tower", f"parallel carry-to-park failed: {exc}",
                {**self.state(), "team_tower_timing": timings},
            )

        # The support dependency is inherently serial: the lower block must be
        # released before the next robot can place on it. Every transport,
        # lowering and release below uses the same physical wheel/servo/gripper
        # path as the standalone team actions; no block or robot qpos is edited.
        yellow_robot = self.controllers[color_owner["yellow"]]
        blue_robot = self.controllers[color_owner["blue"]]
        red_robot = self.controllers[color_owner["red"]]
        self._geometric_stage_base(yellow_robot, "yellow")
        self._geometric_place_on(blue_robot, "blue", "yellow")
        self._geometric_place_on(red_robot, "red", "blue")

        yellow = yellow_robot.body_xyz("yellow_block")
        blue = blue_robot.body_xyz("blue_block")
        red = red_robot.body_xyz("red_block")
        planar_yb = float(np.linalg.norm(blue[:2] - yellow[:2]))
        planar_br = float(np.linalg.norm(red[:2] - blue[:2]))
        dz_yb = float(blue[2] - yellow[2])
        dz_br = float(red[2] - blue[2])
        ok = (
            planar_yb <= TARGET_BLOCK_HALF_M * 0.90
            and planar_br <= TARGET_BLOCK_HALF_M * 0.90
            and abs(dz_yb - TARGET_BLOCK_SIDE_M) <= 0.006
            and abs(dz_br - TARGET_BLOCK_SIDE_M) <= 0.006
        )
        timings["total_s"] = {"value": wall_time.monotonic() - started_at}
        return ActionResult(
            ok, "team_tower",
            (
                f"SIM three-robot tower complete; concurrent prep starts="
                f"{','.join(f'{rid}:{timings[rid].get('start_s', 0.0):.3f}s' for rid in ('r1','r2','r3'))}"
                if ok else
                f"team tower postcondition failed xy_yb={planar_yb:.3f} xy_br={planar_br:.3f} "
                f"dz_yb={dz_yb:.3f} dz_br={dz_br:.3f}"
            ),
            {**self.state(), "team_tower_timing": timings},
        )

    def set_speed_multiplier(self, value: float) -> None:
        value = float(value)
        if value not in (1.0, 2.0, 3.0):
            raise ValueError("speed multiplier must be 1, 2, or 3")
        self.speed_multiplier = value
        for c in self.controllers.values():
            c.speed_multiplier = value

    def _sample_block_layout(self) -> dict[str, tuple[float, float, float]]:
        """Sample a seed-dependent, collision-free three-block task layout."""
        stack_xy = np.asarray(TEAM_STACK_SITE, dtype=float)
        colors = ["red", "blue", "yellow"]
        y_lo, y_hi = TEAM_BLOCK_Y_RANGE
        slack = (y_hi - y_lo) - 2.0 * TEAM_BLOCK_MIN_ROW_SEPARATION_M
        for _ in range(200):
            # Rows first: three ordered Y values at least one row apart, the
            # leftover span distributed at random, then rows assigned to colors.
            # (Rejection sampling of free XY points almost never satisfies the
            # row constraint inside this range.)
            offsets = np.sort(self.rng.uniform(0.0, max(slack, 0.0), size=3))
            rows = [y_lo + i * TEAM_BLOCK_MIN_ROW_SEPARATION_M + float(offsets[i]) for i in range(3)]
            self.rng.shuffle(colors)
            points: dict[str, tuple[float, float, float]] = {}
            for color, y in zip(colors, rows):
                for _ in range(50):
                    xy = np.array([self.rng.uniform(*TEAM_BLOCK_X_RANGE), y], dtype=float)
                    if float(np.linalg.norm(xy - stack_xy)) < TEAM_BLOCK_STACK_SITE_CLEARANCE_M:
                        continue
                    if any(
                        float(np.linalg.norm(xy - np.asarray(existing[:2], dtype=float)))
                        < TEAM_BLOCK_MIN_SEPARATION_M
                        for existing in points.values()
                    ):
                        continue
                    # The block task and warehouse task share one MuJoCo
                    # world. Reject block positions whose footprint overlaps a
                    # seeded warehouse load; seed 25 previously started a red
                    # cube inside the pipe and tipped it before the mission.
                    overlaps_cargo = False
                    for cargo_spec in self.warehouse_specs:
                        cargo_half_x = max(0.045, float(cargo_spec.dimensions_m[0]) / 2.0)
                        cargo_half_y = max(
                            float(cargo_spec.half_length_m),
                            float(cargo_spec.dimensions_m[1]) / 2.0,
                        )
                        if (
                            abs(float(xy[0]) - float(cargo_spec.start_xyz[0]))
                            < cargo_half_x + TARGET_BLOCK_HALF_M + 0.045
                            and abs(float(xy[1]) - float(cargo_spec.start_xyz[1]))
                            < cargo_half_y + TARGET_BLOCK_HALF_M + 0.045
                        ):
                            overlaps_cargo = True
                            break
                        # Keep the two measured inward-grasp base footprints
                        # observable and reachable. Obstacles may constrain the
                        # route, but a reset must not spawn a separate task cube
                        # directly on the only feasible endpoint stance.
                        for sign in (-1.0, 1.0):
                            long_axis = np.asarray((
                                -math.sin(float(cargo_spec.start_yaw_rad)),
                                math.cos(float(cargo_spec.start_yaw_rad)),
                            ))
                            endpoint = np.asarray(cargo_spec.start_xyz[:2], dtype=float) + (
                                sign * long_axis * float(cargo_spec.half_length_m)
                            )
                            inward = -sign * long_axis
                            arm_yaw = math.radians(max(
                                -90.0, min(90.0, math.degrees(math.atan2(
                                    float(inward[1]), float(inward[0]),
                                ))),
                            ))
                            reachable = np.asarray((math.cos(arm_yaw), math.sin(arm_yaw)))
                            grasp_base = endpoint - reachable * 0.16
                            if float(np.linalg.norm(xy - grasp_base)) < 0.11:
                                overlaps_cargo = True
                                break
                        if overlaps_cargo:
                            break
                    if overlaps_cargo:
                        continue
                    points[color] = (
                        float(xy[0]), float(xy[1]),
                        float(self.rng.uniform(-math.pi, math.pi)),
                    )
                    break
            if len(points) == 3:
                return points
        # A reset must never fail: fall back to a fixed layout that satisfies
        # every constraint by construction.
        return dict(TEAM_DETERMINISTIC_LAYOUT)

    def reset(self, seed: int | None = None) -> dict:
        if self.warehouse_layout in {"arena", "camera_team"} and seed is not None and int(seed) != self._warehouse_geometry_seed:
            raise ValueError("ARENA_NEW_SEED_REQUIRES_NEW_WORLD")
        engine = getattr(self, "_mixed_engine", None)
        if engine is not None:
            engine.close()
            self._mixed_engine = None
        self._research_episode = None
        self._solo_traffic_tasks = {}
        self._warehouse_crew = None
        self._crew_motion_recorder = None
        self._warehouse_crew_metrics = {}
        with self.command_lock:
            if seed is not None:
                self.seed = int(seed); self.rng = np.random.default_rng(self.seed)
            from sim.warehouse_mission import mixed_cargo_zone_positions_for_seed
            if self.warehouse_layout in {"arena", "camera_team"}:
                if self.seed != self._warehouse_geometry_seed:
                    raise ValueError("ARENA_NEW_SEED_REQUIRES_NEW_WORLD: static geometry is compiled at construction")
                self.warehouse_zone_positions = self.warehouse_arena.zone_positions
                self.warehouse_specs = self.warehouse_arena.cargo_specs
            else:
                positions_factory = mixed_cargo_zone_positions_for_seed if self.warehouse_layout == "mixed" else cargo_zone_positions_for_seed
                self.warehouse_zone_positions = positions_factory(self.seed, self._warehouse_geometry_specs)
                self.warehouse_specs = tuple(replace(spec,
                    start_xyz=self.warehouse_zone_positions[spec.cargo_id]["A"],
                    goal_xyz=self.warehouse_zone_positions[spec.cargo_id]["B"],
                    carriers=(), scout="") for spec in self._warehouse_geometry_specs)
            self.warehouse_spec_by_id = {
                spec.cargo_id: spec for spec in self.warehouse_specs
            }
            mujoco.mj_resetData(self.model, self.data)
            self.beam_transport_trace = []
            self.beam_mission_status = "READY"
            self.warehouse_trace = []
            self.warehouse_status = "READY"
            self.warehouse_mission = None
            with self._warehouse_goal_lock:
                self._warehouse_requested_destination = None
                self._warehouse_goal_revision = 0
                self._warehouse_goal_reason = ""
            self._warehouse_carrier_jobs = {rid: 0 for rid in self.robot_ids}
            for idx, (rid, c) in enumerate(self.controllers.items()):
                c.seed = self.seed + idx; c.rng = np.random.default_rng(c.seed)
                c.motor_state[:] = 0.0; c.motor_command[:] = 0.0; c.servo_command_pulses.clear()
                c.grasp_color = None; c.pregrasp_color = None; c.spatial_memory.clear(); c._observation_id = 0; c._frame_step = 0
                c._latest_robot_bgr = None; c._latest_robot_frame_seq = 0
                c._latest_robot_jpeg = None; c._latest_robot_jpeg_seq = None; c._latest_robot_jpeg_quality = None
                c._presentation_dirty = True
                x, y, z, yaw = self.warehouse_spawns[rid]
                c.set_base_pose_for_test((x, y, z), yaw)
                c.set_servo_pulses(SEARCH_POSE, forward_only=True)
                if c._real_stack is not None:
                    c._real_stack.reset()
                for path in (Path(f"/tmp/ugrp-sim-{rid}-pick-plan.json"), Path(f"/tmp/ugrp-sim-{rid}-carry-handoff.json")):
                    try: path.unlink()
                    except FileNotFoundError: pass
            layout = self._sample_block_layout()
            r1 = self.controllers["r1"]
            if self.warehouse_layout == "arena":
                layout = {color: (-8.-i, -8., 0.) for i, color in enumerate(layout)}
                r1.set_free_body_pose_for_reset(BEAM_BODY_NAME, (-8., -9., .06), 0.)
            for color, (x, y, yaw) in layout.items():
                r1.set_free_body_pose_for_reset(
                    f"{color}_block", (x, y, TARGET_BLOCK_HALF_M), yaw,
                )
            for spec in self.warehouse_specs:
                r1.set_free_body_pose_for_reset(
                    spec.body_name, spec.start_xyz, spec.start_yaw_rad,
                )
            settle = int(round(0.30 / float(self.model.opt.timestep)))
            for _ in range(settle):
                self._physics_step_for(self.controllers["r1"], STOP)
            return self.state()

    def act(self, robot_id: object, action: str, **params) -> ActionResult:
        rid = self.normalize_robot_id(robot_id)
        action = str(action).strip().lower()
        mixed = getattr(self, "_mixed_engine", None)
        if mixed is not None and not mixed.stopping and action not in {"warehouse_research", "reset", "stop_motion"}:
            return ActionResult(False, action, "MIXED_TASK_ACTIVE: stop the episode before manual robot actions", {})
        if mixed is not None and action == "stop_motion":
            mixed.close()
        if action == "warehouse_research":
            request = params.get("request") or {}
            if str(request.get("operation", "")).startswith("mixed_"):
                from sim.mixed_warehouse import dispatch_mixed
                try:
                    public = dispatch_mixed(self, rid, request)
                    return ActionResult(public.get("ok", True), action, public.get("reason", "MIXED_OK"), {"research": public})
                except Exception as exc:
                    return ActionResult(False, action, str(exc), {"research": {"ok": False, "reason": str(exc)}})
            from sim.warehouse_research import dispatch
            try:
                with self.command_lock:
                    public = dispatch(self, rid, params.get("request"))
                return ActionResult(public.get("ok", True), action,
                                    str(public.get("reason", "RESEARCH_OPERATION_COMPLETE")),
                                    {"research": public})
            except Exception as exc:
                return ActionResult(False, action, str(exc), {"research": {"ok": False, "reason": str(exc)}})
        if action == "reset":
            seed = params.pop("seed", None)
            with self.command_lock:
                self._research_episode = None
                return ActionResult(True, action, "multi-robot v2 simulation reset", self.reset(seed))
        episode = getattr(self, "_research_episode", None)
        if episode and action != "observe_scene":
            episode["closed"] = True
            episode["observations"].clear()
        if action in {"team_beam_transport", "team_zone_transfer"}:
            return ActionResult(
                False,
                action,
                "COOPERATIVE_QUORUM_REQUIRED: submit the same warehouse mission from all three TEAM agents",
                self.state(),
            )
        robot = self.controllers[rid]
        target_color = str(params.get("target_color") or "red")
        destination_color = str(params.get("destination_color") or "")
        try:
            if action in TEAM_CHASSIS_ACTIONS:
                # pick now owns bounded near-field base creep / face-placement
                # motion, so it must use the same soft start clearance as every
                # other chassis-capable public action.
                self._refuse_if_peer_close(
                    robot, action, TEAM_PEER_CLEARANCE_M,
                )
            # Public pickup actions must exercise the exact REAL controller
            # source through NamespacedMasterPi.act().  The geometric helpers
            # below are reserved for SIM-only atomic team primitives; routing a
            # normal approach/pick through them would leak MuJoCo block truth
            # into the actor path and grant perfect simulator-only positioning.
            if action == "search_destination":
                if robot.grasp_color != target_color:
                    raise RuntimeError(f"{rid} is not carrying {target_color}")
                if destination_color not in {"red", "blue", "yellow"}:
                    raise RuntimeError(f"invalid destination_color={destination_color}")
                reason = f"SIM team geometric route locked {destination_color} as destination"
                return ActionResult(True, action, reason, robot.state())
            if action == "stage_base":
                reason = self._geometric_stage_base(robot, target_color)
                return ActionResult(True, action, reason, robot.state())
            if action == "team_tower":
                return self._team_tower()
            if action in {"place", "stack_on"}:
                reason = self._geometric_place_on(robot, target_color, destination_color)
                return ActionResult(True, action, reason, robot.state())
            result = robot.act(action, **params)
            result.state["robot_id"] = rid
            return result
        except PeerTooClose as exc:
            # Do not stay stalled inside the transit lane or the tower row:
            # back out to the parking column so the peer we yielded to can
            # pass, then report so the planner waits and retries.
            self._retreat_best_effort(robot)
            return ActionResult(False, action, str(exc), robot.state())
        except Exception as exc:
            robot.set_motor_commands(STOP)
            return ActionResult(False, action, str(exc), robot.state())

    def act_parallel(
        self,
        commands: Sequence[Mapping[str, object]],
        *,
        on_result: Callable[[str, ActionResult, dict[str, float]], None] | None = None,
    ) -> dict[str, ActionResult]:
        """Execute independent robot commands concurrently.

        The caller supplies one command per robot. Controller state is isolated
        by robot namespace, while the shared MuJoCo world remains protected by
        the existing physics synchronization. This is the multi-robot entry
        point used when the planner wants R1/R2/R3 to receive work together.
        """
        jobs = []
        for command in commands:
            rid = self.normalize_robot_id(command.get("robot_id", "r1"))
            action = str(command.get("action", "")).strip().lower()
            params = {
                str(k): v for k, v in command.items()
                if k not in {
                    "id", "robot_id", "action", "created",
                    "team_batch_id", "team_batch_expected",
                }
            }
            jobs.append((rid, action, params))

        if len({rid for rid, _action, _params in jobs}) != len(jobs):
            raise ValueError("act_parallel accepts at most one command per robot")
        if any(action == "warehouse_research" for _rid, action, _params in jobs):
            raise ValueError("research requests must not be coalesced with other commands")
        episode = getattr(self, "_research_episode", None)
        if episode and any(action != "observe_scene" for _rid, action, _params in jobs):
            episode["closed"] = True
            episode["observations"].clear()

        if any(action == "team_zone_transfer" for _rid, action, _params in jobs):
            started = wall_time.perf_counter()
            failure = ""
            if len(jobs) != 3 or any(action != "team_zone_transfer" for _rid, action, _params in jobs):
                failure = "BATCH_INCOMPLETE: zone transfer requires three matching TEAM participants"
            signatures = {
                (
                    str(params.get("mission_id") or ""),
                    str(params.get("source_zone") or "").upper(),
                    str(params.get("destination_zone") or "").upper(),
                    str(params.get("selector") or "").lower(),
                )
                for _rid, _action, params in jobs
            }
            if not failure and (len(signatures) != 1 or not next(iter(signatures))[0]):
                failure = f"MISSION_ID_MISMATCH: {sorted(signatures)}"
            mission_id, source_zone, destination_zone, selector = (
                next(iter(signatures)) if signatures else ("", "", "", "")
            )
            mission = {
                "mission_id": mission_id,
                "source_zone": source_zone,
                "destination_zone": destination_zone,
                "selector": selector,
            }
            if failure:
                team_result = ActionResult(False, "team_zone_transfer", failure, self.state())
            else:
                team_result = self._cooperative_warehouse_transfer(mission)
            duration = wall_time.perf_counter() - started
            timing = {
                "start_offset_s": 0.0,
                "end_offset_s": round(duration, 6),
                "duration_s": round(duration, 6),
            }
            results: dict[str, ActionResult] = {}
            for rid, _action, _params in jobs:
                member_state = dict(team_result.state)
                member_state.update({
                    "robot_id": rid,
                    "mission_id": mission_id,
                    "joint_physics": True,
                })
                result = ActionResult(
                    team_result.ok, "team_zone_transfer", team_result.reason,
                    member_state,
                )
                results[rid] = result
                if on_result is not None:
                    on_result(rid, result, dict(timing))
            self.last_parallel_timing = {
                "robots": {rid: dict(timing) for rid in results},
                "all_overlap_s": round(duration, 6) if len(results) == 3 else 0.0,
                "wall_s": round(duration, 6),
                "joint_physics": True,
                "mission_id": mission_id,
            }
            return results

        if any(action == "team_beam_transport" for _rid, action, _params in jobs):
            started = wall_time.perf_counter()
            failure = ""
            if len(jobs) != 3 or any(action != "team_beam_transport" for _rid, action, _params in jobs):
                failure = "BATCH_INCOMPLETE: cooperative beam transport requires exactly three matching commands"
            mission_ids = {
                str(params.get("mission_id") or "").strip()
                for _rid, _action, params in jobs
            }
            roles = {
                rid: str(params.get("role") or "").strip().lower()
                for rid, _action, params in jobs
            }
            if not failure and mission_ids != {"beam_transport_v1"}:
                failure = f"MISSION_ID_MISMATCH: {sorted(mission_ids)}"
            if not failure and roles != BEAM_ROLE_BY_ROBOT:
                failure = f"ROLE_MISMATCH: expected={BEAM_ROLE_BY_ROBOT} received={roles}"

            if failure:
                team_result = ActionResult(
                    False, "team_beam_transport", failure, self.state(),
                )
            else:
                team_result = self._cooperative_beam_transport("beam_transport_v1")
            duration = wall_time.perf_counter() - started
            timing = {
                "start_offset_s": 0.0,
                "end_offset_s": round(duration, 6),
                "duration_s": round(duration, 6),
            }
            results: dict[str, ActionResult] = {}
            for rid, _action, _params in jobs:
                member_state = dict(team_result.state)
                member_state.update({
                    "robot_id": rid,
                    "mission_id": "beam_transport_v1",
                    "role": roles.get(rid),
                    "joint_physics": True,
                })
                result = ActionResult(
                    team_result.ok,
                    "team_beam_transport",
                    team_result.reason,
                    member_state,
                )
                results[rid] = result
                if on_result is not None:
                    on_result(rid, result, dict(timing))
            self.last_parallel_timing = {
                "robots": {rid: dict(timing) for rid in results},
                "all_overlap_s": round(duration, 6) if len(results) == 3 else 0.0,
                "wall_s": round(duration, 6),
                "joint_physics": True,
                "mission_id": "beam_transport_v1",
            }
            return results

        start_barrier = threading.Barrier(len(jobs)) if len(jobs) > 1 else None
        cohort_started = wall_time.perf_counter()
        timing_lock = threading.Lock()
        timings: dict[str, dict[str, float]] = {}

        def run_job(rid: str, action: str, params: dict[str, object]) -> ActionResult:
            # Lazy REAL-stack construction/imports may take different amounts
            # of time per robot. Synchronize immediately before public dispatch
            # so a fast slot cannot finish a short primitive before its peers
            # have even entered the parallel cohort.
            if start_barrier is not None:
                start_barrier.wait(timeout=5.0)
            started = wall_time.perf_counter()
            try:
                return self.act(rid, action, **params)
            finally:
                ended = wall_time.perf_counter()
                with timing_lock:
                    timings[rid] = {
                        "start_offset_s": round(started - cohort_started, 6),
                        "end_offset_s": round(ended - cohort_started, 6),
                        "duration_s": round(ended - started, 6),
                    }

        results: dict[str, ActionResult] = {}
        with ThreadPoolExecutor(max_workers=len(jobs) or 1) as pool:
            futures = {
                pool.submit(run_job, rid, action, params): rid
                for rid, action, params in jobs
            }
            for future in as_completed(futures):
                rid = futures[future]
                result = future.result()
                results[rid] = result
                if on_result is not None:
                    on_result(rid, result, dict(timings.get(rid) or {}))
        if timings:
            latest_start = max(item["start_offset_s"] for item in timings.values())
            earliest_end = min(item["end_offset_s"] for item in timings.values())
            self.last_parallel_timing = {
                "robots": dict(timings),
                "all_overlap_s": round(max(0.0, earliest_end - latest_start), 6),
                "wall_s": round(wall_time.perf_counter() - cohort_started, 6),
            }
        return results

    def robot_state(self, robot_id: object) -> dict:
        return self.robot(robot_id).state()

    def scene_detections(self, robot_id: object = "r1") -> dict:
        return self.robot(robot_id).scene_detections()

    def spatial_memory_public(self, robot_id: object = "r1") -> dict:
        return self.robot(robot_id).spatial_memory_public()

    def state(self) -> dict:
        return {
            "seed": int(self.seed),
            "model": "masterpi_multi_v2_shared_world",
            "robot_ids": list(self.robot_ids),
            "robots": {rid: c.state() for rid, c in self.controllers.items()},
            "sim_speed": self.speed_multiplier,
            "shared_objects": {
                color: [round(float(v), 4) for v in self.controllers["r1"].body_xyz(f"{color}_block")]
                for color in ("red", "blue", "yellow")
            },
            "cooperative_payload": self.beam_state(),
            "warehouse": self.warehouse_state(),
        }

    def render_rgb(self, *, robot_id: object = "r1", camera: str = "robot_cam") -> np.ndarray:
        return self.robot(robot_id).render_rgb(camera)

    def render_snapshot_async(self, cameras: Sequence[CameraKey]) -> Future[RenderBatch]:
        """Freeze one SIM instant, then render requested RGB views off physics lock.

        At most three batches may be outstanding. Full capacity raises
        ``SnapshotBackpressure`` before taking physics_lock. The returned
        Future yields ``RenderBatch(frame_id, sim_time, rgb)``; only pixels and
        capture provenance are returned to the caller.
        """
        if threading.get_ident() == self._render_thread_id:
            raise RuntimeError("snapshot capture cannot queue from render owner thread")
        if self._render_executor is None:
            raise RuntimeError("multi MasterPi world was created with render=False")
        with self._snapshot_submit_lock:
            if self._render_closed or self._render_executor is None:
                raise RuntimeError("multi MasterPi world is closing")
            if self._snapshot_broker is None:
                # Allocate the extra copied context only when this API is used.
                self._snapshot_broker = self._render_executor.submit(
                    SnapshotRenderBroker, self).result(timeout=30.0)
            broker = self._snapshot_broker
        slot = broker.acquire()
        try:
            with self._snapshot_submit_lock:
                if self._render_closed or self._render_executor is None:
                    raise RuntimeError("multi MasterPi world is closing")
                snapshot = broker.capture(self, cameras, slot)
                future = self._render_executor.submit(broker.render, snapshot)
                future.add_done_callback(lambda _done: broker.release(snapshot))
                return future
        except Exception:
            # A failed capture/submit has no queued owner to return this slot.
            broker.slots.put_nowait(slot)
            raise

    def render_jpeg(self, *, robot_id: object = "r1", camera: str = "robot_cam", quality: int = 82) -> bytes:
        rgb = self.render_rgb(robot_id=robot_id, camera=camera)
        buf = io.BytesIO(); Image.fromarray(rgb).save(buf, format="JPEG", quality=int(quality)); return buf.getvalue()

    def render_team_jpeg(self, camera: str = "cctv_top", quality: int = 86) -> bytes:
        executor = self._render_executor
        if executor is None or self.observer_renderer is None:
            raise RuntimeError("multi MasterPi world was created with render=False")
        if threading.get_ident() == self._render_thread_id:
            rgb = self._render_team_rgb_direct(camera)
        else:
            rgb = executor.submit(self._render_team_rgb_direct, camera).result(timeout=30.0)
        buf = io.BytesIO(); Image.fromarray(rgb).save(buf, format="JPEG", quality=int(quality)); return buf.getvalue()

    def close(self) -> None:
        engine = getattr(self, "_mixed_engine", None)
        if engine is not None:
            engine.close()
        with self._snapshot_submit_lock:
            self._render_closed = True
            executor = self._render_executor
        if executor is not None:
            try:
                executor.submit(self._close_renderers).result(timeout=15.0)
            finally:
                executor.shutdown(wait=True, cancel_futures=True)
                self._render_executor = None
        self.renderer = None
        self.observer_renderer = None
        self._snapshot_broker = None
        for c in self.controllers.values():
            c.renderer = None; c.observer_renderer = None
