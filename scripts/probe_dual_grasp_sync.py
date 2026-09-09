#!/usr/bin/env python3
"""Bounded physical probe for simultaneous versus delayed dual beam grasp.

The only direct pose writes are fixture setup, before each trial begins.  During
the trial, arms move through the shared servo interpolator and contacts come from
MuJoCo. Artificial grasp welds are OFF by default; --with-weld explicitly enables
them after bilateral finger contact for diagnostic comparisons only.
Every attempted condition retains a JSON trace and close external-camera MP4,
including failures.  This is intentionally independent of the production beam
mission controller and its structural demo path.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import traceback
from typing import Any
from unittest.mock import patch
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.cooperative_payload import (
    BEAM_BODY_NAME,
    BEAM_CARRIER_IDS,
    BEAM_ENDPOINT_OFFSETS_M,
    BEAM_GEOM_NAME,
    BEAM_HEIGHT_M,
    BEAM_HALF_HEIGHT_M,
    BEAM_LENGTH_M,
    BEAM_MASS_KG,
    BEAM_START,
    BEAM_WIDTH_M,
    beam_pose,
)
from sim.multi_masterpi_production import (
    TEAM_APPROACH_STANDOFF_M,
    MultiMasterPiProductionV2,
)
import sim.multi_masterpi_production as multi_production

PLAIN_BEAM_MASS_KG = BEAM_MASS_KG + 0.008 * len(BEAM_CARRIER_IDS)


def _git(args: list[str]) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, capture_output=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else f"unavailable: {result.stderr.strip()}"


def _camera_look_at(world: MultiMasterPiProductionV2, approach_distance: float = 0.0) -> None:
    """Aim an existing presentation camera closely at the beam fixture."""
    camera_id = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_warehouse",
    )
    position = np.asarray((0.18, -2.68, 0.78), dtype=float)
    target = np.asarray((BEAM_START[0], BEAM_START[1], 0.09), dtype=float)
    if approach_distance > 0:
        position = np.asarray((-0.50, -3.12, 1.14))
        target = np.asarray((BEAM_START[0] - approach_distance / 2, BEAM_START[1], 0.08))
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    quat = np.empty(4, dtype=float)
    mujoco.mju_mat2Quat(quat, np.column_stack((right, up, -forward)).ravel())
    world.model.cam_pos[camera_id] = position
    world.model.cam_quat[camera_id] = quat
    world.model.cam_fovy[camera_id] = 47.0
    mujoco.mj_forward(world.model, world.data)


def _plain_beam_contact(world: MultiMasterPiProductionV2, rid: str) -> dict[str, Any]:
    """Read this robot's two finger contacts against the uniform beam geom."""
    robot = world.controllers[rid]
    beam = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, BEAM_GEOM_NAME)
    left = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_GEOM, robot._n("left_finger"),
    )
    right = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_GEOM, robot._n("right_finger"),
    )
    hits = {left: False, right: False}
    forces = {left: 0.0, right: 0.0}
    with world.physics_lock:
        for index in range(world.data.ncon):
            contact = world.data.contact[index]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if beam not in (g1, g2):
                continue
            finger = g2 if g1 == beam else g1
            if finger not in hits:
                continue
            wrench = np.zeros(6, dtype=float)
            if int(contact.efc_address) >= 0:
                mujoco.mj_contactForce(world.model, world.data, index, wrench)
            hits[finger] = True
            forces[finger] += abs(float(wrench[0]))
    return {
        "contact_geom": BEAM_GEOM_NAME,
        "left": hits[left],
        "right": hits[right],
        "bilateral": bool(hits[left] and hits[right]),
        "left_force_n": forces[left],
        "right_force_n": forces[right],
    }


def _plain_beam_xml(original_builder: Any) -> Any:
    """Wrap the production XML builder with the diagnostic's plain beam fixture."""
    def build(*args: Any, **kwargs: Any) -> str:
        root = ET.fromstring(original_builder(*args, **kwargs))
        beam = root.find(f".//body[@name='{BEAM_BODY_NAME}']")
        if beam is None:
            raise RuntimeError("beam body missing from generated model")
        geom = beam.find(f"geom[@name='{BEAM_GEOM_NAME}']")
        if geom is None:
            raise RuntimeError("main beam geom missing from generated model")
        geom.set(
            "size",
            f"{BEAM_WIDTH_M / 2:.6f} {BEAM_LENGTH_M / 2:.6f} {BEAM_HEIGHT_M / 2:.6f}",
        )
        # Preserve the prior fixture's complete 0.196 kg physical mass while
        # consolidating it into the one uniform box (0.180 kg bar + 2x0.008 kg).
        geom.set("mass", f"{PLAIN_BEAM_MASS_KG:.6f}")
        for rid in BEAM_CARRIER_IDS:
            anchor = beam.find(f"body[@name='{BEAM_BODY_NAME}_{rid}_endpoint']")
            if anchor is None:
                raise RuntimeError(f"{rid} weld coordinate anchor missing")
            for child in list(anchor):
                if child.tag in {"geom", "site"}:
                    anchor.remove(child)
        return ET.tostring(root, encoding="unicode")
    return build


def _plain_beam_model_record(world: MultiMasterPiProductionV2) -> dict[str, Any]:
    """Audit the already compiled uniform beam fixture."""
    beam_gid = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_GEOM, BEAM_GEOM_NAME,
    )
    removed: list[dict[str, Any]] = []
    for rid in BEAM_CARRIER_IDS:
        endpoint_name = f"{BEAM_BODY_NAME}_{rid}_endpoint"
        gid = mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_GEOM, f"{endpoint_name}_geom",
        )
        sid = mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_SITE, f"{endpoint_name}_site",
        )
        removed.append({"robot_id": rid, "geom_id": int(gid), "site_id": int(sid)})
        # Keep the child body only as an invisible coordinate anchor for its weld.
        robot = world.controllers[rid]
        robot.finger_payload_contact = (
            lambda endpoint_for, *, _rid=rid: _plain_beam_contact(world, _rid)
        )
    beam_bid = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_BODY, BEAM_BODY_NAME,
    )
    body_ids = [beam_bid]
    for rid in BEAM_CARRIER_IDS:
        body_ids.append(mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_BODY, f"{BEAM_BODY_NAME}_{rid}_endpoint",
        ))
    return {
        "compiled_contact_geometries": {
            name: {"size": world.model.geom_size[gid].tolist(),
                   "friction": world.model.geom_friction[gid].tolist(),
                   "condim": int(world.model.geom_condim[gid]),
                   "rgba": world.model.geom_rgba[gid].tolist()}
            for name in [BEAM_GEOM_NAME, "r1__left_finger", "r1__left_finger_pad_visual", "r1__right_finger"]
            for gid in [mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, name)] if gid >= 0
        },
        "shape": "single_uniform_box",
        "dimensions_m": [BEAM_WIDTH_M, BEAM_LENGTH_M, BEAM_HEIGHT_M],
        "compiled_geom_mass_requested_kg": PLAIN_BEAM_MASS_KG,
        "compiled_total_body_mass_kg": float(sum(world.model.body_mass[bid] for bid in body_ids)),
        "main_geom_half_size_m": [float(v) for v in world.model.geom_size[beam_gid]],
        "removed_handle_geometries_and_sites": removed,
        "endpoint_bodies": "invisible coordinate anchors only; no collision geometry",
        "contact_source": BEAM_GEOM_NAME,
    }


class Video:
    def __init__(self, world: MultiMasterPiProductionV2, path: Path, fps: int):
        self.world = world
        self.fps = fps
        self.frames = 0
        self.next_sim_time = float(world.data.time)
        self.stage = "fixture_setup"
        self.process = subprocess.Popen(
            [
                "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                "-pix_fmt", "bgr24", "-s", "960x720", "-r", str(fps),
                "-i", "-", "-an", "-c:v", "libx264", "-preset", "veryfast",
                "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
        )
        if self.process.stdin is None:
            raise RuntimeError("ffmpeg stdin unavailable")

    def capture(self, force: bool = False) -> None:
        now = float(self.world.data.time)
        if not force and now + 1e-9 < self.next_sim_time:
            return
        jpeg = self.world.render_team_jpeg(camera="cctv_warehouse", quality=92)
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        frame = cv2.resize(frame, (960, 720))
        pose = beam_pose(self.world.data, self.world.model)
        label = (
            f"{self.stage} | t={now:.2f}s | "
            f"beam z={float(pose['position'][2]):.3f}m"
        )
        cv2.rectangle(frame, (0, 0), (960, 54), (15, 18, 22), -1)
        cv2.putText(
            frame, label, (16, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.67,
            (238, 238, 238), 2, cv2.LINE_AA,
        )
        self.process.stdin.write(frame.tobytes())
        self.frames += 1
        self.next_sim_time = now + 1.0 / self.fps

    def hold(self, seconds: float = 0.5) -> None:
        for _ in range(max(1, int(round(seconds * self.fps)))):
            self.capture(force=True)

    def close(self) -> None:
        if self.process.stdin is not None:
            self.process.stdin.close()
        rc = self.process.wait(timeout=30.0)
        if rc:
            raise RuntimeError(f"ffmpeg exited {rc}")


def _pose_metrics(world: MultiMasterPiProductionV2) -> dict[str, Any]:
    pose = beam_pose(world.data, world.model)
    quat = np.asarray(pose["quaternion"], dtype=float)
    rotation = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(rotation, quat)
    matrix = rotation.reshape(3, 3)
    up = matrix[:, 2]
    tilt_deg = math.degrees(math.acos(float(np.clip(up[2], -1.0, 1.0))))
    return {
        "bases": {rid: {"xyz": list(map(float,world.controllers[rid].base_xyz())), "rpy": list(map(float,world.controllers[rid].base_rpy()))} for rid in BEAM_CARRIER_IDS},
        "sim_time_s": round(float(world.data.time), 6),
        "position_m": [round(float(v), 6) for v in pose["position"]],
        "height_above_start_m": round(float(pose["position"][2]) - BEAM_START[2], 6),
        "tilt_deg": round(tilt_deg, 4),
        "constraints_active": {
            rid: bool(world._beam_constraint_active(rid)) for rid in BEAM_CARRIER_IDS
        },
    }


def _step(world: MultiMasterPiProductionV2, seconds: float) -> None:
    steps = max(1, int(round(seconds / float(world.model.opt.timestep))))
    for _ in range(steps):
        world._physics_step_for(world.controllers["r1"])
        if world.frame_callback is not None:
            world.frame_callback()


def run_trial(out_dir: Path, *, name: str, delay_s: float, seed: int, fps: int, weld_assistance: bool = False, side_grasp: bool = False, approach_distance: float = 0.0) -> dict[str, Any]:
    video_path = out_dir / f"{name}.mp4"
    json_path = out_dir / f"{name}.json"
    trace: list[dict[str, Any]] = []
    report: dict[str, Any] = {
        "condition": name,
        "weld_assistance": weld_assistance,
        "side_grasp": side_grasp,
        "approach_distance_m": approach_distance,
        "second_grasp_delay_s": delay_s,
        "seed": seed,
        "video": str(video_path),
        "trace": trace,
        "ok": False,
    }
    world: MultiMasterPiProductionV2 | None = None
    video: Video | None = None

    def record(event: str, **extra: Any) -> None:
        assert world is not None
        item = {"event": event, **_pose_metrics(world), **extra}
        trace.append(item)
        if video is not None:
            video.stage = event
            video.hold(0.35)

    try:
        original_builder = multi_production.build_multi_robot_xml
        with patch.object(
            multi_production,
            "build_multi_robot_xml",
            _plain_beam_xml(original_builder),
        ):
            world = MultiMasterPiProductionV2(
                seed=seed, width=960, height=720, render=True,
            )
        fixture = _plain_beam_model_record(world)
        _camera_look_at(world, approach_distance)
        precision = world._precision_module()
        hover = world._hover_pose()
        grasp = precision.solve_ik(
            precision.CAPTURE_FINGERTIP_RADIUS_CM,
            precision.CAPTURE_GRASP_HEIGHT_CM,
            precision.CAPTURE_GRASP_PITCH_DEG,
        )
        if side_grasp and weld_assistance:
            raise ValueError("side grasp requires unassisted contact physics")
        yaw_pulses = {rid: (2500 if rid == "r1" else 500) if side_grasp else 1500 for rid in BEAM_CARRIER_IDS}
        base_x = BEAM_START[0] - TEAM_APPROACH_STANDOFF_M
        handle_margin = 0.008
        setup_poses: dict[str, list[float]] = {}
        for rid in BEAM_CARRIER_IDS:
            robot = world.controllers[rid]
            pose = (
                base_x,
                BEAM_START[1] + BEAM_ENDPOINT_OFFSETS_M[rid][1]
                + (-handle_margin if rid == "r1" else handle_margin),
                float(robot.base_xyz()[2]),
            )
            if side_grasp:
                sign = 1.0 if rid == "r1" else -1.0
                pose = (BEAM_START[0], BEAM_START[1] - sign * (0.16 + TEAM_APPROACH_STANDOFF_M), float(robot.base_xyz()[2]))
            if approach_distance > 0:
                pose = (pose[0] - approach_distance, pose[1], pose[2])
            robot.set_base_pose_for_test(pose, 0.0)
            setup_poses[rid] = [float(v) for v in pose]
        # R2 is irrelevant to this isolated fixture and remains at its default pose.
        video = Video(world, video_path, fps)
        world.frame_callback = video.capture
        record(
            "fixture_setup_complete",
            setup_pose_write=True,
            setup_only=True,
            carrier_base_poses=setup_poses,
            scout_motion=False,
            beam_fixture=fixture,
        )

        if side_grasp:
            from sim.masterpi_production_v2 import SEARCH_POSE
            world._team_joint_move_servos({rid: {**SEARCH_POSE, 1: precision.GRIPPER_OPEN, 6: 1500} for rid in BEAM_CARRIER_IDS}, 0.6)
            if approach_distance > 0:
                approach_trace = []
                collision_steps = 0
                original_step = world._physics_step_for
                next_sample = float(world.data.time)
                def approach_step(robot, commands=None):
                    nonlocal collision_steps, next_sample
                    original_step(robot, commands)
                    touched = False
                    for k in range(world.data.ncon):
                        c = world.data.contact[k]
                        names = [mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or "" for g in (c.geom1, c.geom2)]
                        if BEAM_GEOM_NAME in names and any(n.startswith(("r1__", "r3__")) for n in names):
                            touched = True
                    collision_steps += int(touched)
                    if world.data.time >= next_sample:
                        approach_trace.append({"sim_time_s": float(world.data.time), "bases": {rid: list(map(float,world.controllers[rid].base_xyz())) for rid in BEAM_CARRIER_IDS}, "payload_contact": touched})
                        next_sample = float(world.data.time) + 0.1
                record("wheel_approach_started", target_x=BEAM_START[0], requested_distance_m=approach_distance)
                world._physics_step_for = approach_step
                try:
                    world._team_joint_move_base_axis("x", {rid: BEAM_START[0] for rid in BEAM_CARRIER_IDS}, tolerance_m=0.002, max_sim_s=15.0)
                    for rid in BEAM_CARRIER_IDS:
                        world._settle(world.controllers[rid])
                    record("approach_stopped_before_alignment")
                    for rid in BEAM_CARRIER_IDS:
                        world._move_axis(world.controllers[rid], "x", BEAM_START[0], tolerance=0.0025, max_pulses=30)
                    for rid in BEAM_CARRIER_IDS:
                        world._settle(world.controllers[rid])
                    record("approach_fine_alignment_complete")
                finally:
                    world._physics_step_for = original_step
                    report["approach"] = {"trajectory": approach_trace, "payload_contact_steps": collision_steps, "physics_timestep_s":float(world.model.opt.timestep)}
                record("wheel_approach_complete", bases={rid: list(map(float,world.controllers[rid].base_xyz())) for rid in BEAM_CARRIER_IDS}, payload_contact_steps=collision_steps)
                if collision_steps:
                    raise RuntimeError("PAYLOAD_CONTACT_DURING_APPROACH")
            record("folded_before_side_rotation")
            world._team_joint_move_servos({rid: {6: yaw_pulses[rid]} for rid in BEAM_CARRIER_IDS}, 1.0, settle_s=0.15)
            record("side_rotation_complete", yaw_pulses=yaw_pulses)
        world._team_joint_move_servos(
            {rid: {1: precision.GRIPPER_OPEN, 6: yaw_pulses[rid], **hover} for rid in BEAM_CARRIER_IDS},
            0.70, settle_s=0.08,
        )
        for height in precision.GRASP_DESCENT_HEIGHTS_CM:
            if height <= precision.CAPTURE_GRASP_HEIGHT_CM + 0.15:
                continue
            descent = precision.solve_ik(
                precision.CAPTURE_FINGERTIP_RADIUS_CM,
                height,
                precision.CAPTURE_GRASP_PITCH_DEG,
            )
            world._team_joint_move_servos(
                {rid: descent for rid in BEAM_CARRIER_IDS}, 0.32,
            )
        world._team_joint_move_servos(
            {rid: grasp for rid in BEAM_CARRIER_IDS}, 0.42, settle_s=0.08,
        )
        record("preclose_aligned")

        if delay_s <= 0.0:
            world._team_joint_move_servos(
                {rid: {1: precision.GRIPPER_CLOSE} for rid in BEAM_CARRIER_IDS},
                0.65, settle_s=0.18,
            )
            contacts = {
                rid: world.controllers[rid].finger_payload_contact(rid)
                for rid in BEAM_CARRIER_IDS
            }
            record("simultaneous_close_complete", contacts=contacts)
            if not all(bool(v.get("bilateral")) for v in contacts.values()):
                raise RuntimeError(f"CARRIER_CONTACT_MISSING: {contacts}")
            for rid in BEAM_CARRIER_IDS:
                if weld_assistance:
                    world._activate_beam_constraint(world.controllers[rid])
        else:
            world._team_joint_move_servos(
                {"r1": {1: precision.GRIPPER_CLOSE}}, 0.65, settle_s=0.18,
            )
            first = world.controllers["r1"].finger_payload_contact("r1")
            record("r1_close_complete", contact=first)
            if not bool(first.get("bilateral")):
                raise RuntimeError(f"CARRIER_CONTACT_MISSING: r1={first}")
            if weld_assistance:
                world._activate_beam_constraint(world.controllers["r1"])
            record("r1_grasp_checked", weld_assistance=weld_assistance)
            _step(world, delay_s)
            record("delay_elapsed", requested_delay_s=delay_s)
            world._team_joint_move_servos(
                {"r3": {1: precision.GRIPPER_CLOSE}}, 0.65, settle_s=0.18,
            )
            second = world.controllers["r3"].finger_payload_contact("r3")
            record("r3_close_complete", contact=second)
            if not bool(second.get("bilateral")):
                raise RuntimeError(f"CARRIER_CONTACT_MISSING: r3={second}")
            if weld_assistance:
                world._activate_beam_constraint(world.controllers["r3"])

        contacts = {
            rid: world.controllers[rid].finger_payload_contact(rid)
            for rid in BEAM_CARRIER_IDS
        }
        record("dual_grasp_confirmed", contacts=contacts)
        if not all(bool(v.get("bilateral")) for v in contacts.values()):
            raise RuntimeError(f"BILATERAL_CONTACT_LOST_BEFORE_LIFT: {contacts}")
        if weld_assistance and not all(world._beam_constraint_active(rid) for rid in BEAM_CARRIER_IDS):
            raise RuntimeError("COOPERATIVE_GRASP_CONSTRAINT_MISSING")

        world._team_joint_move_servos(
            {rid: hover for rid in BEAM_CARRIER_IDS}, 0.70, settle_s=0.25,
        )
        record("lift_complete", contacts={rid: world.controllers[rid].finger_payload_contact(rid) for rid in BEAM_CARRIER_IDS})
        final = _pose_metrics(world)
        if final["height_above_start_m"] < 0.035:
            raise RuntimeError(f"COOPERATIVE_LIFT_FAILED: {final}")
        if side_grasp:
            for index in range(20):
                _step(world, 0.1)
                contacts = {rid: world.controllers[rid].finger_payload_contact(rid) for rid in BEAM_CARRIER_IDS}
                # Record continuous physical hold without inserting frozen video frames.
                item = {"event": "physical_hold_sample", **_pose_metrics(world), "contacts": contacts}
                trace.append(item)
                if item["height_above_start_m"] < 0.03 or not all(c["bilateral"] for c in contacts.values()):
                    raise RuntimeError("UNASSISTED_HOLD_FAILED")
            record("physical_hold_complete", duration_s=2.0)
            final = _pose_metrics(world)
        report["ok"] = True
        report["final"] = final
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
        if world is not None:
            try:
                record("trial_failed", error=report["error"])
                report["final"] = _pose_metrics(world)
            except Exception as snapshot_exc:
                report["snapshot_error"] = f"{type(snapshot_exc).__name__}: {snapshot_exc}"
    finally:
        if world is not None:
            world.frame_callback = None
        if video is not None:
            try:
                video.hold(0.8)
                video.close()
                report["video_frames"] = video.frames
                report["video_duration_s"] = round(video.frames / fps, 3)
            except Exception as exc:
                report["video_error"] = f"{type(exc).__name__}: {exc}"
        if world is not None:
            world.close()
        json_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--fps", type=int, default=12)
    assistance = parser.add_mutually_exclusive_group()
    assistance.add_argument("--with-weld", dest="weld_assistance", action="store_true",
                            help="Explicit diagnostic opt-in to artificial grasp weld assistance.")
    assistance.add_argument("--no-weld", dest="weld_assistance", action="store_false",
                            help="Use unassisted contact physics (the default).")
    parser.set_defaults(weld_assistance=False)
    parser.add_argument("--side-grasp", action="store_true", help="Forward bases, arms rotated +/-90 degrees; no weld permitted.")
    parser.add_argument("--approach-distance", type=float, default=0.0, help="Start this many meters behind the side grasp waypoint (0 to 1m).")
    args = parser.parse_args()
    if not 0 <= args.approach_distance <= 1.0 or (args.approach_distance > 0 and not args.side_grasp):
        parser.error("--approach-distance requires --side-grasp and a distance between 0 and 1m")
    if args.side_grasp and args.weld_assistance:
        parser.error("--side-grasp cannot be combined with --with-weld")
    out_dir = args.out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest: dict[str, Any] = {
        "probe": "plain_beam_weld_ablation_v3",
        "repository": str(ROOT),
        "git_sha": _git(["rev-parse", "HEAD"]),
        "git_status_porcelain": _git(["status", "--porcelain"]),
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "mujoco": getattr(mujoco, "__version__", "unknown"),
            "executable": sys.executable,
            "UGRP_BEAM_DYNAMIC": os.environ.get("UGRP_BEAM_DYNAMIC"),
        },
        "config": {
            "weld_assistance": args.weld_assistance,
            "side_grasp": args.side_grasp,
            "approach_distance_m": args.approach_distance,
            "seed": args.seed,
            "fps": args.fps,
            "conditions": [
                {"name": "baseline", "second_grasp_delay_s": 0.0},
                {"name": "delayed_2s", "second_grasp_delay_s": 2.0},
            ],
            "max_expected_trial_sim_s": 15.0 + (15.0 if args.approach_distance else 0.0),
            "fixture_pose_writes_allowed_during_trial": False,
            "payload_pose_writes": False,
            "scout_or_navigation": False,
            "known_waypoint_wheel_approach": args.approach_distance > 0,
            "payload": {
                "shape": "single_uniform_box",
                "dimensions_m": [BEAM_WIDTH_M, BEAM_LENGTH_M, BEAM_HEIGHT_M],
                "mass_kg": PLAIN_BEAM_MASS_KG,
                "mass_basis": "preserves original 0.180 kg bar plus two 0.008 kg handle geoms",
                "separate_handles": False,
                "endpoint_markers": False,
            },
        },
        "trials": [],
    }
    for name, delay in (("baseline", 0.0), ("delayed_2s", 2.0)):
        manifest["trials"].append(
            run_trial(out_dir, name=name, delay_s=delay, seed=args.seed, fps=args.fps, weld_assistance=args.weld_assistance, side_grasp=args.side_grasp, approach_distance=args.approach_distance)
        )
    manifest["all_ok"] = all(item.get("ok") for item in manifest["trials"])
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, ensure_ascii=False, sort_keys=True))
    return 0 if manifest["all_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
