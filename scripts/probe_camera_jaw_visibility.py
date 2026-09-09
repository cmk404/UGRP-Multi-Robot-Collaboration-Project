#!/usr/bin/env python3
"""Capture camera-only before/after pairs for isolated jaw and look commands."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
from pathlib import Path
import sys
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


COMMANDS = (
    {"kind": "arm", "servo_id": 1, "pulse": 1500},
    {"kind": "arm", "servo_id": 1, "pulse": 2000},
    {"kind": "arm", "servo_id": 1, "pulse": 1500},
    {"kind": "arm", "servo_id": 1, "pulse": 2000},
    {"kind": "look", "pan_pulse": 1550},
    {"kind": "look", "pan_pulse": 1500},
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--robot", choices=("r1", "r3"), default="r1")
    args = parser.parse_args()
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)

    import mujoco
    from harness.camera_grasp_controller import STARTUP_COMMANDS
    from sim.camera_robot_port import CameraRobotPort
    import sim.multi_masterpi_production as production
    from scripts.probe_dual_grasp_sync import _git, _plain_beam_xml, Video

    started = time.monotonic()
    source_sha = _git(["rev-parse", "HEAD"])
    report = {
        "source_sha": source_sha,
        "config": {
            "out_dir": str(out),
            "robot": args.robot,
            "render_width": 640,
            "render_height": 480,
            "seed": 11,
            "settle_seconds": 1.0,
            "weld": False,
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "mujoco": getattr(mujoco, "__version__", None),
        },
        "startup_commands": [dict(command) for command in STARTUP_COMMANDS],
        "command_sequence": [dict(command) for command in COMMANDS],
        "captures": [],
        "error": None,
    }
    world = None
    ports = {}
    video = None

    def capture(index: int, phase: str) -> dict[str, object]:
        own = world.render_jpeg(robot_id=args.robot, camera="robot_cam")
        top = world.render_team_jpeg(camera="cctv_top")
        files = {}
        for view, data in (("own", own), ("top", top)):
            name = f"{index:02d}-{phase}-{view}.jpg"
            (out / name).write_bytes(data)
            files[view] = {"path": name, "sha256": hashlib.sha256(data).hexdigest()}
        return {
            "index": index,
            "phase": phase,
            "sim_time": float(world.data.time),
            "files": files,
        }

    def settle(seconds: float) -> None:
        for _ in range(round(seconds / float(world.model.opt.timestep))):
            for port in ports.values():
                port.tick(float(world.data.time))
            world._physics_step_for(world.controllers["r1"])
            if video is not None:
                video.capture()

    try:
        with patch.object(
            production,
            "build_multi_robot_xml",
            _plain_beam_xml(production.build_multi_robot_xml),
        ):
            world = production.MultiMasterPiProductionV2(
                seed=11, render=True, width=640, height=480
            )
        for rid, y in (("r1", -2.325), ("r3", -1.675)):
            world.controllers[rid].set_base_pose_for_test((-.02, y, .0324), 0.0)
        for name in ("cctv_top", "cctv_warehouse"):
            cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            world.model.cam_pos[cid] = (.55, -2.0, 2.5)
            world.model.cam_quat[cid] = (1, 0, 0, 0)
            world.model.cam_fovy[cid] = 55
        mujoco.mj_forward(world.model, world.data)

        ports = {
            rid: CameraRobotPort(world, rid, allow_reverse=True)
            for rid in ("r1", "r3")
        }
        for port in ports.values():
            for command in STARTUP_COMMANDS:
                port.apply(command, float(world.data.time))
        settle(1.0)

        video = Video(world, out / "motion.mp4", 12)
        world.frame_callback = video.capture
        video.capture(force=True)
        report["captures"].append(capture(0, "initial"))

        for index, command in enumerate(COMMANDS, start=1):
            before = capture(index, "before")
            ports[args.robot].apply(command, float(world.data.time))
            video.stage = f"jaw_visibility_{index}"
            settle(1.0)
            after = capture(index, "after")
            report["captures"].extend((before, after))
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        cleanup_errors = []
        for port in ports.values():
            try:
                port.stop()
            except Exception as exc:
                cleanup_errors.append(f"port.stop: {type(exc).__name__}: {exc}")
        if world is not None:
            world.frame_callback = None
        if video is not None:
            try:
                video.close()
            except Exception as exc:
                cleanup_errors.append(f"video.close: {type(exc).__name__}: {exc}")
        if world is not None:
            try:
                world.close()
            except Exception as exc:
                cleanup_errors.append(f"world.close: {type(exc).__name__}: {exc}")
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
            if report["error"] is None:
                report["error"] = "; ".join(cleanup_errors)
        report["wall_elapsed_s"] = round(time.monotonic() - started, 3)
        report["files"] = {
            str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in out.rglob("*")
            if path.is_file()
        }
        (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")

    print(json.dumps({key: value for key, value in report.items() if key != "files"}))
    return int(report["error"] is not None)


if __name__ == "__main__":
    raise SystemExit(main())
