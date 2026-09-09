#!/usr/bin/env python3
"""Run deterministic low-level image servoing without an LLM or truth feedback."""
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

from scripts.run_camera_pair_transport import evaluate_grasp_samples


INPUT_BOUNDARY = (
    "current/prior own RGB, current/prior shared top RGB, and this robot's issued command "
    "history only; deterministic pixel-servo; no simulator state, geometry, "
    "evaluator feedback, peer commands, or model calls"
)


def _canonical(value: object) -> object:
    """Normalize JSON values, including dictionaries with numeric keys."""
    return json.loads(json.dumps(value, sort_keys=True))


def _digest(value: object) -> str:
    encoded = json.dumps(_canonical(value), sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--rounds", type=int, default=80)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--active-robot", choices=("r1", "r3", "both"), default="r1")
    args = parser.parse_args()
    if not 1 <= args.rounds <= 200:
        parser.error("rounds must be 1..200")
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)

    import mujoco
    from harness.camera_grasp_controller import STARTUP_COMMANDS
    from harness.camera_pixel_grasp import PixelGraspController
    from sim.camera_robot_port import CameraRobotPort
    import sim.multi_masterpi_production as production
    from scripts.probe_dual_grasp_sync import (
        Video,
        _git,
        _plain_beam_contact,
        _plain_beam_xml,
        _pose_metrics,
    )

    config = {
        "out_dir": str(out),
        "rounds": args.rounds,
        "seed": args.seed,
        "active_robot": args.active_robot,
        "render_width": 640,
        "render_height": 480,
        "settle_seconds": 1.0,
        "weld": False,
        "policy": "pixel-servo",
        "policy_description": "diagnostic low-level image servo; not LLM cooperation",
    }
    report = {
        "source_sha": _git(["rev-parse", "HEAD"]),
        "config": config,
        "config_sha256": _digest(config),
        "input_boundary": INPUT_BOUNDARY,
        "input_boundary_sha256": hashlib.sha256(INPUT_BOUNDARY.encode()).hexdigest(),
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "mujoco": getattr(mujoco, "__version__", None),
        },
        "startup_commands": _canonical(STARTUP_COMMANDS),
        "startup_history": {},
        "calls": [],
        "rounds_completed": 0,
        "error": None,
        "grasp_success": None,
        "success_claim": "dual-grasp evaluation pending completion of the fixed round budget",
    }
    started = time.monotonic()
    world = None
    video = None
    ports = {}
    controllers = {}
    active_ids = ("r1", "r3") if args.active_robot == "both" else (args.active_robot,)
    samples = []

    try:
        with patch.object(
            production,
            "build_multi_robot_xml",
            _plain_beam_xml(production.build_multi_robot_xml),
        ):
            world = production.MultiMasterPiProductionV2(
                seed=args.seed, render=True, width=640, height=480
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
        for _ in range(round(1.0 / float(world.model.opt.timestep))):
            for port in ports.values():
                port.tick(float(world.data.time))
            world._physics_step_for(world.controllers["r1"])

        for rid in active_ids:
            (out / rid).mkdir()
            controllers[rid] = PixelGraspController(rid, startup_commands=STARTUP_COMMANDS)
            report["startup_history"][rid] = _canonical(controllers[rid].history)

        video = Video(world, out / "motion.mp4", 12)
        world.frame_callback = video.capture
        video.capture(force=True)
        next_referee_sample = float(world.data.time) + 0.1
        with (out / "evaluation-only.jsonl").open("w") as referee:
            for index in range(args.rounds):
                # All inputs are captured at one simulation instant, before any
                # controller action for this round is applied.
                top = world.render_team_jpeg(camera="cctv_top")
                snapshots = {
                    rid: world.render_jpeg(robot_id=rid, camera="robot_cam")
                    for rid in active_ids
                }
                active = (
                    args.active_robot
                    if args.active_robot != "both"
                    else ("r1", "r3")[index % 2]
                )
                actions = {}
                for rid in active_ids:
                    own = snapshots[rid]
                    own_path = f"{rid}/{index:03d}-own.jpg"
                    top_path = f"{rid}/{index:03d}-top.jpg"
                    (out / own_path).write_bytes(own)
                    (out / top_path).write_bytes(top)
                    before_history = _canonical(controllers[rid].history)
                    action = controllers[rid].step(own, top, active=(rid == active))
                    action = _canonical(action)
                    actions[rid] = action
                    image_record = {
                        "own": {"path": own_path, "sha256": hashlib.sha256(own).hexdigest()},
                        "top": {"path": top_path, "sha256": hashlib.sha256(top).hexdigest()},
                    }
                    report["calls"].append(
                        {
                            "round": index,
                            "robot_id": rid,
                            "active": rid == active,
                            "images": image_record,
                            "input_sha256": _digest(
                                {"images": image_record, "history": before_history, "active": rid == active}
                            ),
                            "action": action,
                            "observation": _canonical(controllers[rid].last_observation),
                            "decision": _canonical(controllers[rid].last_decision),
                            "history": _canonical(controllers[rid].history),
                        }
                    )
                now = float(world.data.time)
                for rid, action in actions.items():
                    ports[rid].apply(action, now)

                video.stage = f"pixel_servo_{index + 1}_{active}"
                for _ in range(round(1.0 / float(world.model.opt.timestep))):
                    for port in ports.values():
                        port.tick(float(world.data.time))
                    world._physics_step_for(world.controllers["r1"])
                    video.capture()
                    if float(world.data.time) + 1e-9 >= next_referee_sample:
                        sample = {
                            "event": "grasp_referee_sample",
                            **_pose_metrics(world),
                            "contacts": {
                                rid: _plain_beam_contact(world, rid)
                                for rid in ("r1", "r3")
                            },
                        }
                        samples.append(sample)
                        referee.write(json.dumps(sample) + "\n")
                        referee.flush()
                        next_referee_sample += 0.1
                for port in ports.values():
                    port.tick(float(world.data.time))
                    port.stop()
                report["rounds_completed"] += 1
                (out / "progress.json").write_text(json.dumps(report, indent=2) + "\n")

        evaluation = evaluate_grasp_samples(samples)
        report.update(evaluation)
        report["success_claim"] = (
            f"output-only dual-grasp criterion evaluated after {args.rounds} fixed rounds: "
            f"grasp_success={evaluation['grasp_success']}; longest qualifying hold "
            f"{evaluation['longest_qualifying_duration_s']:.1f}s; max lift "
            f"{evaluation['max_lift_m']:.3f}m. Single-active-robot runs do not claim solo success."
        )
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
            for rid in ("r1", "r3"):
                world.controllers[rid].set_motor_commands(production.STOP)
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
            if path.is_file() and path.name != "result.json"
        }
        (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")

    print(json.dumps({key: value for key, value in report.items() if key not in {"calls", "files"}}))
    return int(report["error"] is not None)


if __name__ == "__main__":
    raise SystemExit(main())
