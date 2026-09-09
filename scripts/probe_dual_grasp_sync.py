#!/usr/bin/env python3
"""Bounded physical probe for simultaneous versus delayed dual beam grasp.

The only direct pose writes are fixture setup, before each trial begins.  During
the trial, arms move through the shared servo interpolator, contacts come from
MuJoCo, and endpoint welds are enabled only after bilateral finger contact.
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

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.cooperative_payload import (
    BEAM_CARRIER_IDS,
    BEAM_ENDPOINT_OFFSETS_M,
    BEAM_HALF_HEIGHT_M,
    BEAM_START,
    beam_pose,
)
from sim.multi_masterpi_production import (
    TEAM_APPROACH_STANDOFF_M,
    MultiMasterPiProductionV2,
)


def _git(args: list[str]) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, capture_output=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else f"unavailable: {result.stderr.strip()}"


def _camera_look_at(world: MultiMasterPiProductionV2) -> None:
    """Aim an existing presentation camera closely at the beam fixture."""
    camera_id = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_warehouse",
    )
    position = np.asarray((0.18, -2.68, 0.78), dtype=float)
    target = np.asarray((BEAM_START[0], BEAM_START[1], 0.09), dtype=float)
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


def run_trial(out_dir: Path, *, name: str, delay_s: float, seed: int, fps: int) -> dict[str, Any]:
    video_path = out_dir / f"{name}.mp4"
    json_path = out_dir / f"{name}.json"
    trace: list[dict[str, Any]] = []
    report: dict[str, Any] = {
        "condition": name,
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
        world = MultiMasterPiProductionV2(seed=seed, width=960, height=720, render=True)
        _camera_look_at(world)
        precision = world._precision_module()
        hover = world._hover_pose()
        grasp = precision.solve_ik(
            precision.CAPTURE_FINGERTIP_RADIUS_CM,
            precision.CAPTURE_GRASP_HEIGHT_CM,
            precision.CAPTURE_GRASP_PITCH_DEG,
        )
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
        )

        world._team_joint_move_servos(
            {rid: {1: precision.GRIPPER_OPEN, 6: 1500, **hover} for rid in BEAM_CARRIER_IDS},
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
                world._activate_beam_constraint(world.controllers[rid])
        else:
            world._team_joint_move_servos(
                {"r1": {1: precision.GRIPPER_CLOSE}}, 0.65, settle_s=0.18,
            )
            first = world.controllers["r1"].finger_payload_contact("r1")
            record("r1_close_complete", contact=first)
            if not bool(first.get("bilateral")):
                raise RuntimeError(f"CARRIER_CONTACT_MISSING: r1={first}")
            world._activate_beam_constraint(world.controllers["r1"])
            record("r1_constraint_activated_after_contact")
            _step(world, delay_s)
            record("delay_elapsed", requested_delay_s=delay_s)
            world._team_joint_move_servos(
                {"r3": {1: precision.GRIPPER_CLOSE}}, 0.65, settle_s=0.18,
            )
            second = world.controllers["r3"].finger_payload_contact("r3")
            record("r3_close_complete", contact=second)
            if not bool(second.get("bilateral")):
                raise RuntimeError(f"CARRIER_CONTACT_MISSING: r3={second}")
            world._activate_beam_constraint(world.controllers["r3"])

        contacts = {
            rid: world.controllers[rid].finger_payload_contact(rid)
            for rid in BEAM_CARRIER_IDS
        }
        record("dual_grasp_confirmed", contacts=contacts)
        if not all(bool(v.get("bilateral")) for v in contacts.values()):
            raise RuntimeError(f"BILATERAL_CONTACT_LOST_BEFORE_LIFT: {contacts}")
        if not all(world._beam_constraint_active(rid) for rid in BEAM_CARRIER_IDS):
            raise RuntimeError("COOPERATIVE_GRASP_CONSTRAINT_MISSING")

        world._team_joint_move_servos(
            {rid: hover for rid in BEAM_CARRIER_IDS}, 0.70, settle_s=0.25,
        )
        record("lift_complete")
        final = _pose_metrics(world)
        if final["height_above_start_m"] < 0.035:
            raise RuntimeError(f"COOPERATIVE_LIFT_FAILED: {final}")
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
    args = parser.parse_args()
    out_dir = args.out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest: dict[str, Any] = {
        "probe": "dual_grasp_sync_v1",
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
            "seed": args.seed,
            "fps": args.fps,
            "conditions": [
                {"name": "baseline", "second_grasp_delay_s": 0.0},
                {"name": "delayed_2s", "second_grasp_delay_s": 2.0},
            ],
            "max_expected_trial_sim_s": 15.0,
            "fixture_pose_writes_allowed_during_trial": False,
            "payload_pose_writes": False,
            "scout_or_navigation": False,
        },
        "trials": [],
    }
    for name, delay in (("baseline", 0.0), ("delayed_2s", 2.0)):
        manifest["trials"].append(
            run_trial(out_dir, name=name, delay_s=delay, seed=args.seed, fps=args.fps)
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
