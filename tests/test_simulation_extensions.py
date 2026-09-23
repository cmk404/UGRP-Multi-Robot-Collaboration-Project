import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from scripts.sim_cli import main
from sim.scene_objects import append_objects, validate_objects
from sim.session import Simulation
from sim.session_config import load_config, validate_config
from sim.session_extensions import Extensions
from tests.test_simulation_session import World


def scaffold(tmp_path):
    directory = tmp_path / "experiment"
    assert main(["new", str(directory), "--template", "extensions-demo"]) == 0
    return directory, load_config(directory / "config.json")


def test_copied_experiment_resolves_from_config_not_cwd_and_resets(tmp_path, monkeypatch):
    directory, config = scaffold(tmp_path)
    monkeypatch.chdir(tmp_path)
    calls = []
    with Simulation(config, base_dir=directory, world_factory=World, decision_sink=calls.append) as sim:
        assert len(sim._world.kwargs["scene_objects"]) == 2
        assert sim.render
        sim.step(61)
        assert [round(row["at_s"], 2) for row in calls] == [0, .2, .4, .6]
        assert sim._world.robot("r1").motors == [0.] * 4
        observation = calls[1]["observation"]
        assert observation["image"] and observation["top_rgb"]["image"]
        assert {row["robot"] for row in observation["command_history"]} == {"r1"}
        assert not ({"robots_xyz_m", "contacts", "joints", "scene", "success"} & set(observation))
        first = copy.deepcopy(calls[0])
        sim.reset()
        sim.step()
        assert calls[-1]["response"] == first["response"]
        assert calls[-1]["observation"]["command_history"] == []
        with pytest.raises(ValueError, match="owned"):
            sim.apply("r1", {"kind": "wait"})
        # Swap only an extension reference; no core editing or registration.
    config["controllers"]["r1"]["factory"] = "controller.py:create_idle_controller"
    with Simulation(config, base_dir=directory, world_factory=World) as sim:
        sim.step()
        assert sim._world.robot("r1").motors == [0.] * 4


def test_scaffold_and_inspect_do_not_execute_code_or_overwrite(tmp_path):
    directory, config = scaffold(tmp_path)
    (directory / "scene.py").write_text("raise AssertionError('must not execute on inspect')")
    assert main(["inspect", str(directory / "config.json")]) == 0
    with pytest.raises(SystemExit) as error:
        main(["new", str(directory), "--template", "extensions-demo"])
    assert error.value.code == 2
    assert "must not execute" in (directory / "scene.py").read_text()


@pytest.mark.parametrize("output", ["{'kind': 'drive', 'forward': 9, 'turn': 0, 'duration_s': 1}",
                                  "{'kind': 'nudge'}", "[{'kind': 'wait'}]"])
def test_custom_action_is_checked_before_motor_or_world_mutation(tmp_path, output):
    (tmp_path / "action.py").write_text(f"def lower(params):\n    return {output}\n")
    config = {"version": 1, "action_plugins": {"custom": "action.py:lower"}}
    with Simulation(config, base_dir=tmp_path, world_factory=World) as sim:
        with pytest.raises((ValueError, TypeError)):
            sim.apply("r2", {"kind": "custom"})
        assert sim._world.robot("r2").motors == [0.] * 4
        assert len(sim.command_history) == 1
    config["actions"] = [{"at_s": 0, "robot": "r1", "command": {"kind": "custom"}}]
    def forbidden_world(**_):
        pytest.fail("invalid scheduled command must fail before constructing physics")
    with pytest.raises((ValueError, TypeError)):
        Simulation(config, base_dir=tmp_path, world_factory=forbidden_world)


def test_controller_failure_records_exact_unmutated_input_and_closes_world(tmp_path):
    (tmp_path / "controller.py").write_text('''
class Controller:
    def act(self, observation):
        observation.clear()
        raise RuntimeError("policy failed")
def create(**kwargs):
    return Controller()
''')
    records = []
    config = {"version": 1, "controllers": {"r1": {"factory": "controller.py:create"}}}
    with pytest.raises(RuntimeError, match="policy failed"):
        with Simulation(config, base_dir=tmp_path, world_factory=World, decision_sink=records.append) as sim:
            sim.step()
    assert sim._world.closed
    assert records[0]["observation"]["robot_id"] == "r1"
    assert records[0]["error"] == "RuntimeError: policy failed"


def test_plugin_source_refresh_and_duplicate_geometry(tmp_path):
    path = tmp_path / "scene.py"
    path.write_text("def build(**kwargs): return [{'name': 'one'}]\n")
    config = validate_config({"version": 1, "scene": {"builder": "scene.py:build"}})
    first = Extensions(config, tmp_path)
    path.write_text("def build(**kwargs): return [{'name': 'two'}]\n")
    second = Extensions(config, tmp_path)
    assert first.objects[0]["name"] == "one" and second.objects[0]["name"] == "two"
    assert first.sources[str(path)]["sha256"] != second.sources[str(path)]["sha256"]
    config["scene"]["objects"] = [{"name": "two"}]
    with pytest.raises(ValueError, match="unique"):
        Extensions(config, tmp_path)


@pytest.mark.parametrize("patch", [
    {"controllers": {"r4": {"factory": "controller.py:create"}}},
    {"controllers": {"r1": {"factory": "controller.py:create", "period_s": 0}}},
    {"controllers": {"r1": {"factory": "controller.py:create"}},
     "actions": [{"at_s": 0, "robot": "r1", "command": {"kind": "wait"}}]},
    {"action_plugins": {"drive": "actions.py:create"}},
    {"scene": {"builder": "bad-path"}},
    {"scene": {"objects": [{"name": "sphere", "shape": "sphere", "size_m": [.1, .1]}]}},
    {"scene": {"objects": [{"name": "bad", "mass_kg": float("nan")}]}}
])
def test_extension_contract_rejects_ambiguous_or_invalid_config(patch):
    with pytest.raises(ValueError):
        validate_config({"version": 1, **patch})


def test_geometry_is_namespaced_collision_enabled_and_has_free_joint_only_when_dynamic():
    root = ET.Element("worldbody")
    append_objects(root, [{"name": "ramp", "euler_deg": [0, 15, 90]},
                          {"name": "ball", "shape": "sphere", "size_m": [.05], "dynamic": True}])
    assert [body.get("name") for body in root] == ["user__ramp", "user__ball"]
    assert root[0].find("freejoint") is None
    assert root[1].find("freejoint") is not None
    assert root[0].find("geom").get("contype") == "1"
    assert root[0].get("quat") != "1.0 0.0 0.0 0.0"
