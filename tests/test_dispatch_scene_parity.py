"""The standard session and research runner compile/reset one dispatch definition.

No renderer, controller, model inference, or cargo transport is started here.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from scripts.research_dispatch_scene import DispatchScene
from sim.research_dispatch_arena import episode
from sim.session import Simulation
from sim.session_scenes import Scene


# Original raw scene hashes from e099a4a F1 and the 6089f80 legacy replay.
HISTORICAL_XML = {
    None: "c1b9574303ac4f832e2b52bec166dcdf4c2c96fb41b0321579f692c17bcdc965",
    "local_contact_fine": "90c279d1c8fbd4b30c41fcbfd28254610f0aa8ac9357fe65898cba127a05a674",
}


def world_state(world):
    model, data = world.model, world.data
    return {
        "xml": world.scene_xml,
        "qpos": data.qpos.copy(), "qvel": data.qvel.copy(),
        "eq_active": data.eq_active.copy(),
        "camera_pos": model.cam_pos.copy(), "camera_quat": model.cam_quat.copy(),
        "camera_fovy": model.cam_fovy.copy(),
        "geom_size": model.geom_size.copy(), "geom_friction": model.geom_friction.copy(),
        "body_mass": model.body_mass.copy(),
        "pair_friction": model.pair_friction.copy(),
        "timestep": float(model.opt.timestep), "pairs": int(model.npair),
        "time": float(data.time),
    }


def assert_same_state(a, b):
    assert a.keys() == b.keys()
    for key in a:
        if isinstance(a[key], np.ndarray):
            np.testing.assert_array_equal(a[key], b[key], err_msg=key)
        else:
            assert a[key] == b[key], key


@pytest.mark.parametrize("profile", [None, "local_contact", "local_contact_fine"])
def test_dispatch_runner_and_standard_session_share_xml_and_initial_state(tmp_path, profile):
    config = episode("open", 11)
    if profile is not None:
        config["contact_solver_profile"] = profile
    runner = DispatchScene(config, tmp_path / "runner", render=False)
    standard = None
    try:
        runner.open()
        standard = Simulation({"version": 1,
            "scene": {"layout": "dispatch/open", "seed": 11, "contact_profile": profile},
            "camera": {"width": 960, "height": 720}}, render=False)
        actual = world_state(runner.world)
        assert_same_state(actual, world_state(standard._world))
        assert runner.definition.config == config
        assert standard.scene.config == episode("open", 11)
        assert runner.definition.scene["contact_profile"] == standard.scene.scene["contact_profile"] == profile
        assert runner.manifest == standard.scene.manifest
        assert runner.initial_invariants["weld_active"] is False
        assert actual["pairs"] == (12 if profile else 0)
        assert actual["timestep"] == ({"local_contact": .0005,
                                        "local_contact_fine": .00025}.get(profile, .002))
        if profile in HISTORICAL_XML:
            assert hashlib.sha256(actual["xml"].encode()).hexdigest() == HISTORICAL_XML[profile]
        # Servo folding and settling can move free bodies away from authored
        # spawn positions. Full settled state parity above is the relevant check.
    finally:
        if standard is not None:
            standard.close()
        runner.close()


def test_dispatch_runner_preserves_setup_only_spawn_override(tmp_path):
    config = episode("open", 11)
    original = copy.deepcopy(config)
    config["setup_only"]["spawns"]["r2"][0] += .01
    scene = DispatchScene(config, tmp_path / "runner", render=False)
    baseline = DispatchScene(original, tmp_path / "baseline", render=False)
    try:
        scene.open()
        baseline.open()
        assert scene.definition.config == config
        assert hashlib.sha256(scene.xml.encode()).hexdigest() == HISTORICAL_XML[None]
        assert scene.world.robot("r2").base_xyz()[0] - baseline.world.robot("r2").base_xyz()[0] == pytest.approx(.01, abs=.003)
    finally:
        scene.close()
        baseline.close()


@pytest.mark.parametrize("profile", [None, "local_contact"])
def test_act_preview_uses_exact_standard_map_scene(tmp_path, profile):
    from sim.act_map_suite import load_suite, scene_config
    _, cases = load_suite()
    case = cases[0]
    config = scene_config(case, physics_seed=11)
    if profile is not None:
        config["contact_solver_profile"] = profile
    scene = DispatchScene(config, tmp_path / "preview", render=False)
    standard = None
    try:
        scene.open()
        standard = Simulation({"version": 1,
            "scene": {"layout": "act/" + case["id"], "seed": 11, "contact_profile": profile},
            "camera": {"width": 960, "height": 720}}, render=False)
        assert scene.definition.selection == "act/" + case["id"]
        assert scene.definition.config == config
        assert_same_state(world_state(scene.world), world_state(standard._world))
    finally:
        if standard is not None:
            standard.close()
        scene.close()


def test_dispatch_scene_closes_world_after_setup_error(tmp_path, monkeypatch):
    from sim import multi_masterpi_production
    closed = []
    class FakeWorld:
        scene_xml = "<mujoco/>"
        def __init__(self, **kwargs):
            assert kwargs["xml_transform"] is not None
        def close(self):
            closed.append(True)
    def fail_setup(self, world):
        raise RuntimeError("setup failed")
    monkeypatch.setattr(multi_masterpi_production, "MultiMasterPiProductionV2", FakeWorld)
    monkeypatch.setattr(Scene, "setup", fail_setup)
    scene = DispatchScene(episode("open", 11), tmp_path / "failed", render=False)
    with pytest.raises(RuntimeError, match="setup failed"):
        scene.open()
    assert closed == [True] and scene.world is None
