"""Configure and run a local MuJoCo world: python -m scripts.sim_cli --help."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import platform
import queue
import signal
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from sim.session_config import ROBOTS, load_config, validate_config
from sim.session_scenes import DEFAULT_SCENE, Scene, catalog

ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def source_info():
    def git(*args):
        return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()
    try:
        return {"source_sha": git("rev-parse", "HEAD"), "source_dirty": bool(git("status", "--porcelain"))}
    except (OSError, subprocess.CalledProcessError):
        return {"source_sha": None, "source_dirty": None}


def capture(sim, output, label):
    (output / f"{label}-overview.jpg").write_bytes(sim._world.render_team_jpeg(camera="cctv_warehouse"))
    for rid in ROBOTS:
        observation = sim.observe(rid, include_top=rid == "r1")
        (output / f"{label}-{rid}.jpg").write_bytes(base64.b64decode(observation["image"]))
        if rid == "r1":
            (output / f"{label}-top.jpg").write_bytes(base64.b64decode(observation["top_rgb"]["image"]))
        write_json(output / f"{label}-{rid}-observation.json", observation)


def run(config, args):
    if not args.headless and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise ValueError("native viewer requires a desktop display; use --headless on a server")
    if args.headless and args.paused:
        raise ValueError("--paused requires the native viewer")
    from sim.session import Simulation, SimulationStateError
    import mujoco

    output = Path(args.output) if args.output else ROOT / "outputs" / f"sim-{datetime.now():%Y%m%d-%H%M%S}-{uuid4().hex[:6]}"
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "config.json", config)
    metadata = {**source_info(), "python": platform.python_version(), "platform": platform.platform(),
                "mujoco": mujoco.__version__, "headless": args.headless, "capture": args.capture,
                "config_sha256": hashlib.sha256((output / "config.json").read_bytes()).hexdigest()}
    write_json(output / "session.json", metadata)
    print(f"Output: {output.resolve()}", flush=True)
    keys = queue.SimpleQueue()
    sim = None
    video = None
    decisions = (output / "controller-decisions.jsonl").open("x", encoding="utf-8")
    def record_decision(record):
        decisions.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        decisions.flush()
    started = time.monotonic()
    paused = args.paused
    state_error = None
    result = {**metadata, "seed": config["scene"]["seed"], "policy": "local_controller" if config["controllers"] else "raw_commands",
              "case": config["scene"]["layout"], "scope": "simulation_runtime_check",
              "protocol_complete": False, "stop_reason": "error"}
    result["runtime_events"] = []
    if not config["controllers"]:
        result["model_calls"] = 0
    def interrupted(*_):
        raise KeyboardInterrupt
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        # Record entry bytes before trusted Python can fail in a builder/factory.
        # Successful construction also records the exact executed bytes below.
        from sim.session_extensions import validate_reference
        refs = list(config["action_plugins"].values()) + [v["factory"] for v in config["controllers"].values()]
        if config["scene"]["builder"]:
            refs.append(config["scene"]["builder"])
        inputs = output / "input-files"
        inputs.mkdir()
        receipts = []
        for index, path in enumerate(dict.fromkeys((args.config.resolve().parent / validate_reference(ref)[0]).resolve() for ref in refs)):
            data = path.read_bytes()
            saved = inputs / f"{index:02d}-{path.name}"
            saved.write_bytes(data)
            receipts.append({"path": str(path), "saved": str(saved.relative_to(output)), "sha256": hashlib.sha256(data).hexdigest()})
        write_json(output / "input-files.json", receipts)
        sim = Simulation(config, render=args.capture or args.video, base_dir=args.config.resolve().parent,
                         decision_sink=record_decision)
        source_dir = output / "extensions"
        source_dir.mkdir()
        source_manifest = []
        for index, (path, entry) in enumerate(sim.extensions.sources.items()):
            saved = source_dir / f"{index:02d}-{Path(path).name}"
            saved.write_bytes(entry["bytes"])
            source_manifest.append({"path": path, "saved": str(saved.relative_to(output)),
                                    "sha256": entry["sha256"]})
        write_json(output / "extensions.json", {"base_dir": str(sim.extensions.base_dir),
                   "entry_files": source_manifest, "resolved_objects": sim.extensions.objects})
        map_dir = output / "scene-sources"
        map_dir.mkdir()
        map_manifest = []
        for index, (path, entry) in enumerate(sim.scene.sources.items()):
            saved = map_dir / f"{index:02d}-{Path(path).name}"
            saved.write_bytes(entry["bytes"])
            map_manifest.append({"path": path, "saved": str(saved.relative_to(output)), "sha256": entry["sha256"]})
        (output / "scene.xml").write_text(sim._world.scene_xml, encoding="utf-8")
        write_json(output / "scene.json", {**sim.scene.record(), "source_files": map_manifest,
                   "geometry_modified": bool(sim.extensions.objects),
                   "scene_xml_sha256": hashlib.sha256(sim._world.scene_xml.encode()).hexdigest()})
        mujoco.mj_saveModel(sim._world.model, str(output / "model.mjb"))
        write_json(output / "physics.json", {"timestep_s": sim.timestep,
                   "dynamics": sim._world.dynamics, "calibration": sim._world.calibration_status,
                   "cargo_ids": sim.scene.inventory})
        write_json(output / "initial-evaluation.json", sim.evaluation_state())
        if args.capture:
            capture(sim, output, "initial")
        if args.video:
            from sim.session_recording import Video
            video = Video(output, camera=args.video_camera, fps=args.video_fps)
            video.frame(sim)
        viewer = None if args.headless else sim.launch_viewer(camera=args.camera, key_callback=keys.put)
        if viewer:
            print("MuJoCo: Space pause/resume | N one physics tick | R reset | close window to exit", flush=True)
        batch = max(1, round(.02 / sim.timestep))
        while True:
            tick_started = time.monotonic()
            single_step = False
            while not keys.empty():
                key = keys.get()
                if key == 32:
                    if state_error is None:
                        paused = not paused
                    print("Paused" if paused else "Running", flush=True)
                elif key in (ord("R"), ord("r")):
                    sim.reset()
                    state_error = None
                    paused = True
                    print(f"Reset episode {sim.episode}; paused", flush=True)
                elif key in (ord("N"), ord("n")) and paused and state_error is None:
                    single_step = True
            if viewer is not None and not viewer.is_running():
                result["stop_reason"] = "window_closed"
                break
            if time.monotonic() - started >= config["run"]["wall_seconds"]:
                result["stop_reason"] = "wall_limit"
                break
            remaining = config["run"]["sim_seconds"] - sim.time
            if remaining <= sim.timestep * 1e-6:
                result["stop_reason"] = "sim_limit"
                break
            advanced = 0.0
            try:
                if not paused or single_step:
                    steps = 1 if single_step else min(batch, max(1, math.ceil(remaining / sim.timestep - 1e-9)))
                    sim.step(steps)
                    advanced = steps * sim.timestep
                if video is not None and advanced:
                    video.frame(sim)
                if viewer is not None:
                    sim.sync_viewer()
            except SimulationStateError as error:
                if state_error is None:
                    state_error = error
                    result["runtime_events"].append(error.record)
                    write_json(output / "runtime-events.json", result["runtime_events"])
                    print(f"Physics paused: {error}. Press R to reset; Space cannot resume an invalid episode.", flush=True)
                if viewer is None:
                    raise
                paused = True
                advanced = 0.0
            if viewer is not None:
                status = "Physics state changed / unstable. Press R to reset." if state_error else (
                    "Paused: Space to run | N step | R reset" if paused else "Running: Space to pause | R reset")
                viewer.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, status, ""))
            if viewer is not None or paused:
                delay = (advanced / config["run"]["realtime_factor"] if advanced else .02) - (time.monotonic() - tick_started)
                if delay > 0:
                    time.sleep(delay)
        result["protocol_complete"] = result["stop_reason"] == "sim_limit" and not result["runtime_events"]
        if args.capture and state_error is None:
            capture(sim, output, "final")
        return 2 if result["runtime_events"] or (args.headless and not result["protocol_complete"]) else 0
    except KeyboardInterrupt:
        result["stop_reason"] = "interrupted"
        return 130
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        try:
            if sim is not None:
                try:
                    result["sim_s"] = sim.time
                    result["episodes"] = sim.episode + 1
                    result["controller_calls"] = sim.controller_calls
                    result["commands"] = sum(row["event"] == "command" for row in sim.command_history)
                    (output / "commands.jsonl").write_text("".join(json.dumps(row) + "\n" for row in sim.command_history), encoding="utf-8")
                    write_json(output / "final-evaluation.json", sim.evaluation_state())
                finally:
                    try:
                        if video is not None:
                            result["video"] = video.close()
                    finally:
                        sim.close()
        except Exception as error:
            result.update(protocol_complete=False, stop_reason="error", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            decisions.close()
            result["wall_s"] = time.monotonic() - started
            result["artifacts_sha256"] = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in sorted(output.rglob("*")) if p.is_file() and p.name != "result.json"}
            write_json(output / "result.json", result)
            print(f"Stopped: {result['stop_reason']} | {output.resolve() / 'result.json'}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description="UGRP native MuJoCo + configurable local simulation")
    sub = parser.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init", help="write a new editable JSON configuration")
    init.add_argument("path", type=Path)
    init.add_argument("--scene", "--layout", dest="layout", default=DEFAULT_SCENE,
                      help="scene ID from scenes, or navigation/file / pair_navigation/file")
    init.add_argument("--map-file", help="authored map JSON, relative to the new config directory")
    init.add_argument("--seed", type=int, default=11)
    new = sub.add_parser("new", help="create a standalone experiment with scene/controller/action files")
    new.add_argument("directory", type=Path)
    new.add_argument("--scene", default=DEFAULT_SCENE)
    new.add_argument("--template", choices=("research", "extensions-demo"), default="research")
    inspect = sub.add_parser("inspect", help="validate and print the resolved configuration (no MuJoCo needed)")
    inspect.add_argument("config", type=Path)
    sub.add_parser("layouts", help="alias of scenes")
    scenes = sub.add_parser("scenes", help="list all existing research scenes and scope")
    scenes.add_argument("--json", action="store_true")
    sub.add_parser("workflows", help="list established planners, policies, training, evaluation and hardware entry points")
    sub.add_parser("doctor", help="report local Python, MuJoCo, display and recorder availability")
    execute = sub.add_parser("run", help="run config in MuJoCo's native window")
    execute.add_argument("config", type=Path)
    execute.add_argument("--headless", action="store_true", help="same physics without a window, as fast as possible")
    execute.add_argument("--capture", action="store_true", help="save calibrated robot and top RGB at start/end")
    execute.add_argument("--paused", action="store_true")
    execute.add_argument("--video", action="store_true", help="record an observer MP4, requires ffmpeg")
    execute.add_argument("--video-camera", default="cctv_warehouse")
    execute.add_argument("--video-fps", type=int, choices=range(1, 31), default=10)
    execute.add_argument("--camera", default="free", help="native view: free, cctv_top, cctv_warehouse, r1__robot_cam, ...")
    execute.add_argument("--controller", action="append", default=[], metavar="ROBOT=FILE.py:FACTORY",
                         help="replace/add a robot controller; paths relative to the config directory")
    execute.add_argument("--scene-builder", help="replace scene builder, relative to the config directory")
    execute.add_argument("--seed", type=int)
    execute.add_argument("--sim-seconds", type=float)
    execute.add_argument("--wall-seconds", type=float)
    execute.add_argument("--realtime-factor", type=float)
    execute.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command in ("layouts", "scenes"):
            rows = catalog()
            print(json.dumps(rows, indent=2, ensure_ascii=False) if getattr(args, "json", False)
                  else "\n".join(f"{row['id']:<44} {row['scope']}" for row in rows))
            return 0
        if args.command == "workflows":
            print((ROOT / "configs/simulation_workflows.json").read_text())
            return 0
        if args.command == "doctor":
            from importlib.metadata import PackageNotFoundError, version
            packages = {}
            for name in ("mujoco", "numpy", "opencv-python-headless", "pillow", "glfw"):
                try:
                    packages[name] = version(name)
                except PackageNotFoundError:
                    packages[name] = None
            print(json.dumps({"python": sys.executable, "version": platform.python_version(),
                              "platform": platform.platform(), "packages": packages,
                              "mjpython": str(Path(sys.executable).with_name("mjpython")),
                              "display_available": sys.platform == "darwin" or bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")),
                              "ffmpeg": shutil.which("ffmpeg"), "scope": "dependency discovery, no rendering probe"}, indent=2))
            return int(any(value is None for value in packages.values()))
        if args.command == "new":
            if args.template == "research":
                config = validate_config({"version": 1, "scene": {"layout": args.scene, "seed": 11,
                    "builder": "scene.py:build_scene"},
                    "action_plugins": {"nudge": "actions.py:nudge"},
                    "controllers": {"r1": {"factory": "controller.py:create_idle_controller"}}})
                Scene(config["scene"], args.directory)
            shutil.copytree(ROOT / "examples" / "simulation_extensions", args.directory,
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
            if args.template == "research":
                write_json(args.directory / "config.json", config)
                (args.directory / "scene.py").write_text('"""Add geometry here; the selected research scene is preserved."""\ndef build_scene(*, seed, params):\n    return []\n')
            print(f"Created {args.directory.resolve()} (edit config.json, scene.py, controller.py, actions.py)")
            return 0
        if args.command == "init":
            config = validate_config({"version": 1, "scene": {"layout": args.layout, "seed": args.seed, "map_file": args.map_file}})
            Scene(config["scene"], args.path.resolve().parent)
            with args.path.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(config, indent=2) + "\n")
            print(args.path.resolve())
            return 0
        config = load_config(args.config)
        if args.command == "inspect":
            scene = Scene(config["scene"], args.config.resolve().parent)
            print(json.dumps({"config": config, "scene": scene.record(), "source_files": {
                path: entry["sha256"] for path, entry in scene.sources.items()}}, indent=2, ensure_ascii=False))
            return 0
        for override in args.controller:
            if "=" not in override:
                raise ValueError("--controller requires ROBOT=FILE.py:FACTORY")
            robot, ref = override.split("=", 1)
            config["controllers"][robot] = {**config["controllers"].get(robot, {}), "factory": ref}
        if args.scene_builder is not None:
            config["scene"]["builder"] = args.scene_builder
        if args.seed is not None:
            config["scene"]["seed"] = args.seed
        for flag in ("sim_seconds", "wall_seconds", "realtime_factor"):
            if getattr(args, flag) is not None:
                config["run"][flag] = getattr(args, flag)
        return run(validate_config(config), args)
    except (ValueError, OSError, RuntimeError) as error:
        parser.exit(2, f"sim: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
