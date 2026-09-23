"""Version boundary checks only; no simulator, renderer, or model inference."""
import copy
import json
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from harness import rgb_execution_bundle as contract
from harness.rgb_skill_execution import _applied_contract


def test_historical_success_is_separate_from_experimental_adapter():
    current, digest = contract.load_bundle(contract.RUNNABLE_ID)
    baseline, _ = contract.load_bundle(contract.BASELINE_ID, require_runnable=False)
    legacy, _ = contract.load_bundle(contract.LEGACY_ID, require_runnable=False)
    assert len(digest) == 64
    assert current["status"] == "experimental_unqualified"
    assert baseline["observed_result"]["physical_success"] is True
    assert current["effective"]["physics"]["timestep_s"] == .00025
    assert baseline["effective"]["physics"]["timestep_s"] == .00025
    assert legacy["effective"]["physics"]["timestep_s"] == .002
    assert current["effective"]["execution"]["pose_schedule_probe"]["pan_samples"] == [1506, 1521, 1539, 1554, 1560]
    assert baseline["effective"]["execution"]["pose_schedule_probe"]["pan_samples"] == [1506, 1521, 1539, 1554, 1560]
    assert current["effective"]["execution"]["pair_pose_schedule_probe"] == {"pan_samples": [1530, 1560], "completion_s": .4}
    assert current["effective"]["execution"]["solo_goal_control_inset_px"] == 1.
    assert current["effective"]["execution"]["pair_heading_prior_bounds"] == {
        "pad_px": 8, "max_center_step_px": 8, "max_bounds_step_px": 10,
        "reject_clipped_prior_mask": True}
    assert current["effective"]["execution"]["pair_heading_component_fallback"] == {
        "membership": "centroid_in_prior_own_bounds",
        "reject_clipped_support": True,
        "requires_four_corners": True}
    assert "execution.owner" in contract.baseline_diff(current["effective"])
    with pytest.raises(ValueError, match="historical success bundle cannot run"):
        contract.load_bundle(contract.BASELINE_ID)
    with pytest.raises(ValueError, match="original source checkout"):
        contract.load_bundle(contract.LEGACY_ID)


@pytest.mark.parametrize("bundle_id", sorted(contract.RETIRED_IDS))
def test_retired_adapter_is_readable_but_cannot_run_as_current_source(bundle_id):
    old, _ = contract.load_bundle(bundle_id, require_runnable=False)
    assert old["id"] == bundle_id
    assert old["status"] == "experimental_unqualified"
    with pytest.raises(ValueError, match="original source checkout"):
        contract.load_bundle(bundle_id)


def isolated_registry(tmp_path):
    root = tmp_path / "repo"
    registry = root / contract.REGISTRY
    registry.mkdir(parents=True)
    for name in contract.source_closure():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(contract.ROOT / name, path)
    for bundle_id in (contract.RUNNABLE_ID, contract.BASELINE_ID, *contract.RETIRED_IDS):
        shutil.copyfile(contract.ROOT / contract.REGISTRY / (bundle_id + ".json"),
                        registry / (bundle_id + ".json"))
    return root


def test_required_source_set_and_bytes_fail_closed(tmp_path):
    root = isolated_registry(tmp_path)
    value, _ = contract.load_bundle(contract.RUNNABLE_ID, root=root)
    path = root / contract.REGISTRY / (contract.RUNNABLE_ID + ".json")
    removed = "harness/camera_varied_start_student.py"
    assert removed in contract.source_closure()
    assert len(contract.source_closure()) >= 129
    value["source_files_sha256"].pop(removed)
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="required source set mismatch"):
        contract.load_bundle(contract.RUNNABLE_ID, root=root)
    shutil.copyfile(contract.ROOT / contract.REGISTRY / path.name, path)
    (root / removed).write_text((root / removed).read_text() + "\n# changed execution source\n")
    with pytest.raises(ValueError, match="source drift"):
        contract.load_bundle(contract.RUNNABLE_ID, root=root)


def test_local_import_closure_catches_from_package_relative_and_initializer(tmp_path):
    root = isolated_registry(tmp_path)
    seed = root / "harness/rgb_execution_contract.py"
    seed.write_text(seed.read_text() + "\nfrom harness import bundle_probe\n")
    (root / "harness/bundle_probe.py").write_text("from .bundle_pkg import helper\n")
    package = root / "harness/bundle_pkg"
    package.mkdir()
    (package / "__init__.py").write_text("from .helper import answer\n")
    (package / "helper.py").write_text("answer = 1\n")
    closure = contract.source_closure(root=root)
    assert {"harness/bundle_probe.py", "harness/bundle_pkg/__init__.py",
            "harness/bundle_pkg/helper.py"} <= closure
    with pytest.raises(ValueError, match="required source set mismatch"):
        contract.load_bundle(contract.RUNNABLE_ID, root=root)


def test_live_scene_values_and_measured_macro_schedule_are_checked():
    bundle, _ = contract.load_bundle(contract.RUNNABLE_ID)
    model = SimpleNamespace(npair=12, opt=SimpleNamespace(timestep=.00025))
    world = SimpleNamespace(model=model, data=SimpleNamespace(eq_active=SimpleNamespace(any=lambda: False)),
                            width=960, height=720, observer_width=960, observer_height=720)
    pairs = '<pair/>'*12
    scene = SimpleNamespace(world=world, manifest={"contact_solver_profile": "local_contact_fine"},
                            xml=f'<mujoco><option timestep="0.00025"/><contact>{pairs}</contact></mujoco>')
    actual = _applied_contract(scene)
    assert actual["execution"]["pose_schedule_probe"] == {"pan_samples": [1506, 1521, 1539, 1554, 1560], "completion_s": .75}
    contract.require_effective(bundle, actual)
    model.opt.timestep = .002
    with pytest.raises(ValueError, match="XML/model timestep differs"):
        _applied_contract(scene)
    model.opt.timestep = .00025
    world.observer_width = 1280
    with pytest.raises(ValueError, match="camera.top_raw_size"):
        contract.require_effective(bundle, _applied_contract(scene))
    world.observer_width = 960
    model.npair = 1
    with pytest.raises(ValueError, match="XML/model explicit contact pair"):
        _applied_contract(scene)
    altered = copy.deepcopy(actual)
    altered["execution"]["pose_schedule_probe"]["pan_samples"] = [1560]
    with pytest.raises(ValueError, match="execution.pose_schedule_probe.pan_samples"):
        contract.require_effective(bundle, altered)


def test_existing_registry_bytes_cannot_change_or_disappear(tmp_path):
    root = isolated_registry(tmp_path)
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(["git", "add", "config"], cwd=root, check=True)
    subprocess.run(["git", "-c", "user.name=Test", "-c", "user.email=test@example.invalid",
                    "commit", "-qm", "baseline"], cwd=root, check=True)
    contract.verify_registry_immutable("HEAD", root=root)
    path = root / contract.REGISTRY / (contract.RUNNABLE_ID + ".json")
    original = path.read_bytes()
    path.write_bytes(original + b"\n")
    with pytest.raises(ValueError, match="create a new ID"):
        contract.verify_registry_immutable("HEAD", root=root)
    path.unlink()
    with pytest.raises(ValueError, match="removed"):
        contract.verify_registry_immutable("HEAD", root=root)
    path.write_bytes(original)
    (root / contract.REGISTRY / "future-v2.json").write_text("{}\n")
    contract.verify_registry_immutable("HEAD", root=root)
    with pytest.raises(subprocess.CalledProcessError):
        contract.verify_registry_immutable("missing-base", root=root)
