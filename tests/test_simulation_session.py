import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.sim_cli import main
from sim.session import Simulation
from sim.session_config import DEFAULT_CONFIG, validate_config


class Robot:
    def __init__(self):
        self.servo_command_pulses = {1: 1500, 3: 1500, 4: 1500, 5: 1500, 6: 1500}
        self.motors = [0.] * 4

    def set_motor_commands(self, values):
        self.motors = list(values)

    def set_servo_pulses(self, values):
        self.servo_command_pulses.update(values)


class World:
    def __init__(self, **kwargs):
        self.model = SimpleNamespace(opt=SimpleNamespace(timestep=.01))
        self.data = SimpleNamespace(time=0.)
        self.closed = False
        self.kwargs = kwargs

    def reset(self, seed=None):
        self.data.time = .3
        self.robots = {rid: Robot() for rid in ("r1", "r2", "r3")}

    def robot(self, rid):
        return self.robots[rid]

    def _physics_step_for(self, _):
        self.data.time += self.model.opt.timestep

    def render_jpeg(self, **_):
        return b"owned-rgb"

    def render_team_jpeg(self, **_):
        return b"top-rgb"

    def close(self):
        self.closed = True


@pytest.mark.parametrize("bad", [
    {}, {"version": True}, {"version": 2}, {"version": 1, "unknown": 1},
    {"version": 1, "scene": {"layout": "typo"}},
    {"version": 1, "scene": {"seed": True}},
    {"version": 1, "scene": {"cargo_ids": []}},
    {"version": 1, "scene": {"cargo_ids": ["x", "x"]}},
    {"version": 1, "scene": {"robots": {"r4": {}}}},
    {"version": 1, "scene": {"robots": {"r1": {"xyz_m": [0, 0, float("nan")], "yaw_deg": 0}}}},
    {"version": 1, "camera": {"width": False}},
    {"version": 1, "control": {"allow_reverse": "false"}},
    {"version": 1, "run": {"wall_seconds": float("inf")}},
    {"version": 1, "actions": [{"at_s": 0, "robot": "r1", "command": {"kind": "pickup"}}]},
    {"version": 1, "actions": [{"at_s": 30, "robot": "r1", "command": {"kind": "wait"}}]},
])
def test_bad_config_fails_without_importing_physics(bad):
    with pytest.raises(ValueError):
        validate_config(bad)


def test_config_is_an_independent_resolved_copy():
    config = validate_config({"version": 1})
    assert config == DEFAULT_CONFIG
    config["scene"]["robots"]["r1"] = {}
    assert validate_config({"version": 1})["scene"]["robots"] == {}


def test_step_leases_reset_and_no_privileged_observation():
    config = {"version": 1, "actions": [{"at_s": .01, "robot": "r1", "command":
              {"kind": "drive", "forward": .1, "turn": 0, "duration_s": .02}}]}
    with Simulation(config, render=True, world_factory=World) as sim:
        sim.step()
        assert sim._world.robot("r1").motors == [0.] * 4
        sim.step()
        assert sim._world.robot("r1").motors == [.1] * 4
        sim.step(2)
        assert sim.time == pytest.approx(.04)
        assert sim._world.robot("r1").motors == [0.] * 4
        observation = sim.observe("r1")
        assert set(observation) == {"robot_id", "frame_id", "sim_time", "image", "sha256", "camera",
                                     "actuator_state", "top_rgb", "episode", "episode_time_s"}
        original_model = sim._world.model
        sim.reset()
        assert sim._world.model is original_model
        assert sim.time == 0
        sim.step(2)
        assert sim._world.robot("r1").motors == [.1] * 4
        assert sim.episode == 1
    assert sim._world.closed
    sim.close()
    with pytest.raises(RuntimeError, match="closed"):
        sim.step()


def test_invalid_action_never_touches_motor_and_no_render_is_explicit():
    with Simulation({"version": 1}, world_factory=World) as sim:
        with pytest.raises(ValueError):
            sim.apply("r1", {"kind": "drive", "forward": 1, "turn": 0, "duration_s": 1})
        assert sim._world.robot("r1").motors == [0.] * 4
        with pytest.raises(RuntimeError, match="render=True"):
            sim.observe("r1")


def test_cli_init_inspect_and_refuse_overwrite(tmp_path, capsys):
    target = tmp_path / "scene.json"
    assert main(["init", str(target), "--seed", "42", "--layout", "arena"]) == 0
    assert json.loads(target.read_text())["scene"]["seed"] == 42
    assert main(["inspect", str(target)]) == 0
    with pytest.raises(SystemExit) as stopped:
        main(["init", str(target)])
    assert stopped.value.code == 2
    assert json.loads(target.read_text())["scene"]["seed"] == 42


def test_config_examples_are_valid():
    for path in (Path(__file__).resolve().parents[1] / "configs/simulation").glob("*.json"):
        validate_config(json.loads(path.read_text()))


def test_close_waits_for_native_render_owner_before_world_cleanup(monkeypatch):
    order = []

    class Viewer:
        def close(self):
            order.append("exit_requested")

        def _sim(self):
            return object() if "render_destroyed" not in order else None

    sim = Simulation({"version": 1}, world_factory=World)
    sim._viewer = Viewer()
    monkeypatch.setattr("sim.session.time.sleep", lambda _: order.append("render_destroyed"))
    sim._world.close = lambda: order.append("world_closed")
    sim.close()
    assert order == ["exit_requested", "render_destroyed", "world_closed"]
