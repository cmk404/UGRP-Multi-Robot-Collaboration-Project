"""Exercise copied user files against real physics; no model or hardware calls."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from scripts.sim_cli import main as cli, source_info, write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    result = {**source_info(), "scope": "simulation_extension_runtime_check", "case": "extensions-api",
              "model_calls": 0, "protocol_complete": False}
    try:
        import mujoco
        import numpy as np
        from sim.session import Simulation
        from sim.session_config import load_config

        experiment = output / "experiment"
        cli(["new", str(experiment)])
        config_path = experiment / "config.json"
        for name, width, controller in (("active", .35, "create_controller"),
                                         ("idle", .35, "create_idle_controller")):
            config = load_config(config_path)
            config["scene"]["params"]["barrier_half_width_m"] = width
            write_json(config_path, config)
            assert cli(["run", str(config_path), "--headless", "--output", str(output / name),
                        "--controller", f"r1=controller.py:{controller}"]) == 0
            model = mujoco.MjModel.from_binary_path(str(output / name / "model.mjb"))
            assert model.geom("user__barrier__geom").size[0] == width
            assert model.body("user__ball").mass[0] == .08
            run = json.loads((output / name / "result.json").read_text())
            assert run["protocol_complete"] and run["controller_calls"] == 10
            rows = [json.loads(line) for line in (output / name / "controller-decisions.jsonl").read_text().splitlines()]
            assert len(rows) == 10 and rows[0]["raw_command"]["forward"] == (.08 if name == "active" else 0)
            assert all(row["observation"]["image"] and row["observation"]["top_rgb"]["image"] for row in rows)
            evaluation = json.loads((output / name / "final-evaluation.json").read_text())
            assert evaluation["active_equalities"] == 0

        positions = [json.loads((output / name / "final-evaluation.json").read_text())["robots_xyz_m"]["r1"]
                     for name in ("active", "idle")]
        displacement = float(np.linalg.norm(np.array(positions[0]) - positions[1]))
        assert displacement > 1e-5, "changing the controller must affect actual robot physics"

        config = load_config(config_path)
        config["controllers"] = {}
        config["scene"]["params"]["barrier_half_width_m"] = .6
        with Simulation(config, base_dir=experiment) as sim:
            assert sim._world.model.geom("user__barrier__geom").size[0] == .6
            original_model = sim._world.model
            joint = sim._world.model.joint("user__ball__joint")
            index = int(joint.qposadr[0])
            initial = sim._world.data.qpos[index:index+7].copy()
            sim._world.data.qpos[index + 2] += .3  # privileged reset diagnostic only
            sim.step(50)
            assert not np.allclose(initial, sim._world.data.qpos[index:index+7])
            sim.reset()
            assert sim._world.model is original_model
            assert np.allclose(initial, sim._world.data.qpos[index:index+7], atol=1e-10)
        result.update(protocol_complete=True, controller_position_difference_m=displacement,
                      checks=["copied_files", "geometry_parameter_change", "dynamic_object_reset",
                              "controller_swap", "own_rgb_and_top", "recorded_requests", "weld_off"],
                      commands=sum(json.loads((output / name / "result.json").read_text())["commands"]
                                   for name in ("active", "idle")))
        return 0
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        result["wall_s"] = time.monotonic() - started
        write_json(output / "result.json", result)


if __name__ == "__main__":
    raise SystemExit(main())
