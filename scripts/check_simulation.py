"""Finite real-physics integration check; no model, network or external services."""
import argparse
import base64
import hashlib
import json
import time
from pathlib import Path

import numpy as np

from scripts.sim_cli import source_info, write_json
from sim.session import Simulation
from sim.session_config import LAYOUTS, ROBOTS, load_config, validate_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = {**source_info(), "seed": 41, "policy": "raw_commands", "case": "api",
              "scope": "simulation_runtime_check", "protocol_complete": False, "model_calls": 0}
    started = time.monotonic()
    try:
        config = load_config("configs/simulation/drive.json")
        write_json(args.output / "config.json", config)
        with Simulation(config, render=True) as sim:
            before = sim.evaluation_state()
            initial_qpos = sim._world.data.qpos.copy()
            cameras = sim._world.model.cam_fovy.copy()
            cargo_id = sim._world.warehouse_specs[0].cargo_id
            for rid in ROBOTS:
                observation = sim.observe(rid)
                assert set(observation) == {"robot_id", "frame_id", "sim_time", "image", "sha256", "camera",
                                             "actuator_state", "top_rgb", "episode", "episode_time_s"}
                for name, frame in ((rid, observation), ("top", observation["top_rgb"])):
                    jpeg = base64.b64decode(frame["image"])
                    assert hashlib.sha256(jpeg).hexdigest() == frame["sha256"]
                    assert jpeg.startswith(b"\xff\xd8")
                    (args.output / f"{name}.jpg").write_bytes(jpeg)
            sim.step(round(3 / sim.timestep))
            after = sim.evaluation_state()
            assert abs(sim.time - 3) < 1e-8
            assert after["active_equalities"] == 0
            distance = float(np.linalg.norm(np.array(after["robots_xyz_m"]["r1"]) - before["robots_xyz_m"]["r1"]))
            assert distance > .001, distance
            assert sim.observe("r1")["actuator_state"]["motor_commands"] == [0.0] * 4
            assert sim.observe("r3")["actuator_state"]["motor_commands"] == [0.0] * 4
            assert np.array_equal(cameras, sim._world.model.cam_fovy)
            commands = sum(row["event"] == "command" for row in sim.command_history)
            assert commands == 3
            model, data = sim._world.model, sim._world.data
            sim.reset()
            assert sim._world.model is model and sim._world.data is data
            assert np.allclose(initial_qpos, sim._world.data.qpos, atol=1e-10)
            sim.step()
            assert sum(row["event"] == "command" for row in sim.command_history) == 4
            result.update(commands=commands, sim_s=3.0, r1_displacement_m=distance,
                          reset_reproduced=True, rgb_boundary_checked=True, camera_fov_preserved=True)
            write_json(args.output / "evaluation.json", {"before": before, "after": after})
        for layout in LAYOUTS:
            with Simulation(validate_config({"version": 1, "scene": {"layout": layout}})) as sim:
                sim.step()
                assert sim.evaluation_state()["active_equalities"] == 0
        pose = {"xyz_m": [-.5, .3, .08], "yaw_deg": 45}
        with Simulation(validate_config({"version": 1, "scene": {"cargo_ids": [cargo_id], "robots": {"r1": pose}}})) as sim:
            assert np.allclose(sim.evaluation_state()["robots_xyz_m"]["r1"], pose["xyz_m"])
            assert [item.cargo_id for item in sim._world.warehouse_specs] == [cargo_id]
            sim.step(2)
            sim.reset()
            assert np.allclose(sim.evaluation_state()["robots_xyz_m"]["r1"], pose["xyz_m"])
        result.update(protocol_complete=True, layouts=list(LAYOUTS), scene_overrides_checked=True)
        return 0
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        result["wall_s"] = time.monotonic() - started
        result["artifacts_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
        write_json(args.output / "result.json", result)
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
