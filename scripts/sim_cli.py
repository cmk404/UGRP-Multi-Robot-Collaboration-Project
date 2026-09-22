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
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from sim.session_config import LAYOUTS, ROBOTS, load_config, validate_config

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
    from sim.session import Simulation
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
    started = time.monotonic()
    paused = args.paused
    result = {**metadata, "seed": config["scene"]["seed"], "policy": "raw_commands",
              "case": config["scene"]["layout"], "scope": "simulation_runtime_check",
              "model_calls": 0, "protocol_complete": False, "stop_reason": "error"}
    def interrupted(*_):
        raise KeyboardInterrupt
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        sim = Simulation(config, render=args.capture)
        mujoco.mj_saveModel(sim._world.model, str(output / "model.mjb"))
        write_json(output / "physics.json", {"timestep_s": sim.timestep,
                   "dynamics": sim._world.dynamics, "calibration": sim._world.calibration_status,
                   "cargo_ids": [spec.cargo_id for spec in sim._world.warehouse_specs]})
        write_json(output / "initial-evaluation.json", sim.evaluation_state())
        if args.capture:
            capture(sim, output, "initial")
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
                    paused = not paused
                    print("Paused" if paused else "Running", flush=True)
                elif key in (ord("R"), ord("r")):
                    sim.reset()
                    paused = True
                    print(f"Reset episode {sim.episode}; paused", flush=True)
                elif key in (ord("N"), ord("n")) and paused:
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
            if not paused or single_step:
                steps = 1 if single_step else min(batch, max(1, math.ceil(remaining / sim.timestep - 1e-9)))
                sim.step(steps)
                advanced = steps * sim.timestep
            if viewer is not None:
                sim.sync_viewer()
            if viewer is not None or paused:
                delay = (advanced / config["run"]["realtime_factor"] if advanced else .02) - (time.monotonic() - tick_started)
                if delay > 0:
                    time.sleep(delay)
        result["protocol_complete"] = result["stop_reason"] == "sim_limit"
        if args.capture:
            capture(sim, output, "final")
        return 0
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
                    result["commands"] = sum(row["event"] == "command" for row in sim.command_history)
                    (output / "commands.jsonl").write_text("".join(json.dumps(row) + "\n" for row in sim.command_history), encoding="utf-8")
                    write_json(output / "final-evaluation.json", sim.evaluation_state())
                finally:
                    sim.close()
        except Exception as error:
            result.update(protocol_complete=False, stop_reason="error", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            result["wall_s"] = time.monotonic() - started
            result["artifacts_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in sorted(output.iterdir()) if p.is_file() and p.name != "result.json"}
            write_json(output / "result.json", result)
            print(f"Stopped: {result['stop_reason']} | {output.resolve() / 'result.json'}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description="UGRP native MuJoCo + configurable local simulation")
    sub = parser.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init", help="write a new editable JSON configuration")
    init.add_argument("path", type=Path)
    init.add_argument("--layout", choices=LAYOUTS, default="camera_team")
    init.add_argument("--seed", type=int, default=41)
    inspect = sub.add_parser("inspect", help="validate and print the resolved configuration (no MuJoCo needed)")
    inspect.add_argument("config", type=Path)
    sub.add_parser("layouts", help="list built-in scene layouts")
    execute = sub.add_parser("run", help="run config in MuJoCo's native window")
    execute.add_argument("config", type=Path)
    execute.add_argument("--headless", action="store_true", help="same physics without a window, as fast as possible")
    execute.add_argument("--capture", action="store_true", help="save calibrated robot and top RGB at start/end")
    execute.add_argument("--paused", action="store_true")
    execute.add_argument("--camera", default="free", help="native view: free, cctv_top, cctv_warehouse, r1__robot_cam, ...")
    execute.add_argument("--seed", type=int)
    execute.add_argument("--sim-seconds", type=float)
    execute.add_argument("--wall-seconds", type=float)
    execute.add_argument("--realtime-factor", type=float)
    execute.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "layouts":
            print("\n".join(LAYOUTS))
            return 0
        if args.command == "init":
            config = validate_config({"version": 1, "scene": {"layout": args.layout, "seed": args.seed}})
            with args.path.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(config, indent=2) + "\n")
            print(args.path.resolve())
            return 0
        config = load_config(args.config)
        if args.command == "inspect":
            print(json.dumps(config, indent=2, ensure_ascii=False))
            return 0
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
