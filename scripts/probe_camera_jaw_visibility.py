#!/usr/bin/env python3
"""Replay camera-observed actions, then capture bounded jaw-motion RGB pairs."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
from pathlib import Path
import sys
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _canonical_hash(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def _validate_raw_actions(actions: list[dict], option: str) -> None:
    fields = {
        "wait": {"kind"},
        "look": {"kind", "pan_pulse"},
        "arm": {"kind", "servo_id", "pulse"},
        "drive": {"kind", "forward", "turn", "duration_s"},
    }
    for index, action in enumerate(actions):
        kind = action.get("kind")
        if kind not in fields or set(action) != fields[kind]:
            raise ValueError(f"{option}[{index}] has invalid fields")
        if kind in ("look", "arm"):
            values = [action["pan_pulse"]] if kind == "look" else [action["servo_id"], action["pulse"]]
            if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
                raise ValueError(f"{option}[{index}] servo fields must be integers")
        if kind == "look" and not 500 <= action["pan_pulse"] <= 2500:
            raise ValueError(f"{option}[{index}] pan_pulse is out of bounds")
        if kind == "arm" and (
            action["servo_id"] not in (1, 3, 4, 5) or not 500 <= action["pulse"] <= 2500
        ):
            raise ValueError(f"{option}[{index}] arm command is out of bounds")
        if kind == "drive":
            values = (action["forward"], action["turn"], action["duration_s"])
            if any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(float(value)) for value in values):
                raise ValueError(f"{option}[{index}] drive fields must be finite numbers")
            if not (-.05 <= action["forward"] <= .15 and -.2 <= action["turn"] <= .2
                    and 0 <= action["duration_s"] <= 1):
                raise ValueError(f"{option}[{index}] drive command is out of bounds")


def _load_actions(value: str | None, option: str) -> tuple[list[dict], dict | None]:
    if value is None:
        return [], None
    if value.lstrip().startswith("["):
        raw = value.encode()
        source = "inline"
    else:
        path = Path(value).expanduser().resolve()
        raw = path.read_bytes()
        source = str(path)
    actions = json.loads(raw)
    if not isinstance(actions, list) or not all(isinstance(action, dict) for action in actions):
        raise ValueError(f"{option} must be a JSON list of raw action objects")
    _validate_raw_actions(actions, option)
    return actions, {"source": source, "sha256": hashlib.sha256(raw).hexdigest()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--robot", choices=("r1", "r3"), default="r1")
    parser.add_argument("--close-pulse", type=int, default=1500)
    parser.add_argument("--render-width", type=int, choices=(640, 1280), default=640)
    parser.add_argument("--render-height", type=int, choices=(480, 960), default=480)
    parser.add_argument(
        "--replay-actions",
        help="JSON list, or path to a JSON list, of previously observed own raw actions",
    )
    parser.add_argument(
        "--probe-actions",
        help="Optional JSON list, or path to one, overriding the six default probe actions",
    )
    args = parser.parse_args()
    if not 500 <= args.close_pulse <= 1999:
        parser.error("--close-pulse must be 500..1999")
    if (args.render_width, args.render_height) not in ((640, 480), (1280, 960)):
        parser.error("render size must be 640x480 or 1280x960")
    try:
        replay_actions, replay_input = _load_actions(args.replay_actions, "--replay-actions")
        probe_actions, probe_input = _load_actions(args.probe_actions, "--probe-actions")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        parser.error(str(exc))
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)

    import mujoco
    import numpy as np
    from harness.camera_grasp_controller import STARTUP_COMMANDS
    from sim.camera_robot_port import CameraRobotPort
    import sim.multi_masterpi_production as production
    from scripts.probe_dual_grasp_sync import _git, _plain_beam_xml, Video
    from sim.warehouse_observation import camera_intrinsics_from_vertical_fovy

    last_look = next(
        (int(command["pan_pulse"]) for command in reversed(
            [*STARTUP_COMMANDS, *replay_actions]) if command.get("kind") == "look"),
        1500,
    )
    probe_look = min(2500, last_look + 50)
    default_commands = (
        {"kind": "arm", "servo_id": 1, "pulse": args.close_pulse},
        {"kind": "arm", "servo_id": 1, "pulse": 2000},
        {"kind": "arm", "servo_id": 1, "pulse": args.close_pulse},
        {"kind": "arm", "servo_id": 1, "pulse": 2000},
        {"kind": "look", "pan_pulse": probe_look},
        {"kind": "look", "pan_pulse": last_look},
    )
    commands = probe_actions if args.probe_actions is not None else list(default_commands)

    started = time.monotonic()
    source_sha = _git(["rev-parse", "HEAD"])
    report = {
        "source_sha": source_sha,
        "config": {
            "out_dir": str(out),
            "robot": args.robot,
            "close_pulse": args.close_pulse,
            "render_width": args.render_width,
            "render_height": args.render_height,
            "seed": 11,
            "settle_seconds": 1.0,
            "weld": False,
            "purpose": "bounded image-only active-perception amplitude diagnostic",
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "mujoco": getattr(mujoco, "__version__", None),
        },
        "startup_commands": [dict(command) for command in STARTUP_COMMANDS],
        "input_boundary": (
            "fixed fixture/startup plus caller-supplied previously observed own raw actions; "
            "no controller, tracker, evaluator, simulator state, or geometry feedback"
        ),
        "replay_input": replay_input,
        "replay_actions": replay_actions,
        "replay_actions_sha256": _canonical_hash(replay_actions),
        "replay": [],
        "probe_input": probe_input,
        "probe_actions_sha256": _canonical_hash(commands),
        "command_sequence": [dict(command) for command in commands],
        "captures": [],
        "error": None,
    }
    world = None
    ports = {}
    video = None
    evaluation_file = None
    top_camera_id = None

    def output_reference(index: int, phase: str) -> None:
        robot = world.controllers[args.robot]
        intrinsics = camera_intrinsics_from_vertical_fovy(
            args.render_width, args.render_height, float(world.model.cam_fovy[top_camera_id])
        )
        rotation = world.data.cam_xmat[top_camera_id].reshape(3, 3)
        camera_position = world.data.cam_xpos[top_camera_id]
        points = {}
        for label in ("left_finger", "right_finger"):
            geom_id = mujoco.mj_name2id(
                world.model, mujoco.mjtObj.mjOBJ_GEOM, robot._n(label)
            )
            if geom_id < 0:
                points[label] = {"pixel": None, "camera_depth_m": None, "error": "geom missing"}
                continue
            xyz = np.asarray(world.data.geom_xpos[geom_id], dtype=float) - camera_position
            x, y, z = xyz @ rotation
            if not np.isfinite((x, y, z)).all() or z >= -1e-6:
                projected = None
            else:
                projected = [
                    float(intrinsics["cx_px"] + x / -z * intrinsics["fx_px"]),
                    float(intrinsics["cy_px"] - y / -z * intrinsics["fy_px"]),
                ]
                if not np.isfinite(projected).all():
                    projected = None
            points[label] = {
                "pixel": projected,
                "camera_depth_m": float(-z) if math.isfinite(float(z)) else None,
            }
        left, right = points["left_finger"]["pixel"], points["right_finger"]["pixel"]
        axis = None if left is None or right is None else math.atan2(
            right[1] - left[1], right[0] - left[0]
        )
        base_rpy = [float(value) for value in robot.base_rpy()]
        record = {
            "event": "output_only_geom_center_reference",
            "reference_scope": "projected finger geom centers, not exact fingertips",
            "index": index,
            "phase": phase,
            "sim_time": float(world.data.time),
            "camera": "cctv_top",
            "image_size": [args.render_width, args.render_height],
            "points": points,
            "connecting_axis_angle_rad": axis,
            "base_rpy_rad": base_rpy if all(math.isfinite(value) for value in base_rpy) else None,
        }
        evaluation_file.write(json.dumps(record) + "\n")
        evaluation_file.flush()

    def capture(index: int, phase: str) -> dict[str, object]:
        own = world.render_jpeg(robot_id=args.robot, camera="robot_cam")
        top = world.render_team_jpeg(camera="cctv_top")
        files = {}
        for view, data in (("own", own), ("top", top)):
            name = f"{index:02d}-{phase}-{view}.jpg"
            (out / name).write_bytes(data)
            files[view] = {"path": name, "sha256": hashlib.sha256(data).hexdigest()}
        # Truth-derived projection is recorded only after both RGB inputs are
        # fixed on disk and cannot affect this or any later raw action.
        output_reference(index, phase)
        return {
            "index": index,
            "phase": phase,
            "sim_time": float(world.data.time),
            "files": files,
        }

    def settle(seconds: float, *, capture_video: bool = True) -> None:
        for _ in range(round(seconds / float(world.model.opt.timestep))):
            for port in ports.values():
                port.tick(float(world.data.time))
            world._physics_step_for(world.controllers["r1"])
            if video is not None and capture_video:
                video.capture()

    try:
        with patch.object(
            production,
            "build_multi_robot_xml",
            _plain_beam_xml(production.build_multi_robot_xml),
        ):
            world = production.MultiMasterPiProductionV2(
                seed=11, render=True, width=args.render_width, height=args.render_height
            )
        for rid, y in (("r1", -2.325), ("r3", -1.675)):
            world.controllers[rid].set_base_pose_for_test((-.02, y, .0324), 0.0)
        for name in ("cctv_top", "cctv_warehouse"):
            cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            world.model.cam_pos[cid] = (.55, -2.0, 2.5)
            world.model.cam_quat[cid] = (1, 0, 0, 0)
            world.model.cam_fovy[cid] = 55
            if name == "cctv_top":
                top_camera_id = cid
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
        evaluation_file = (out / "evaluation-only.jsonl").open("w")
        world.frame_callback = video.capture
        video.capture(force=True)
        report["captures"].append(capture(0, "startup"))

        # Replay only caller-supplied own raw actions. CameraRobotPort validates
        # the same bounded action schema as the original run. No image, tracker,
        # controller, geometry, or evaluator value changes this sequence.
        for index, command in enumerate(replay_actions, start=1):
            ports[args.robot].apply(command, float(world.data.time))
            video.stage = f"replay_{index}"
            settle(1.0, capture_video=False)
            for port in ports.values():
                port.tick(float(world.data.time))
                port.stop()
            report["replay"].append({"index": index, "action": command})
        report["captures"].append(capture(len(replay_actions), "post-replay"))

        for index, command in enumerate(commands, start=1):
            before = capture(index, "before")
            ports[args.robot].apply(command, float(world.data.time))
            video.stage = f"jaw_visibility_{index}"
            settle(1.0)
            for port in ports.values():
                port.tick(float(world.data.time))
                port.stop()
            after = capture(index, "after")
            report["captures"].extend((before, after))
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        cleanup_errors = []
        if evaluation_file is not None:
            try:
                evaluation_file.close()
            except Exception as exc:
                cleanup_errors.append(f"evaluation.close: {type(exc).__name__}: {exc}")
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
