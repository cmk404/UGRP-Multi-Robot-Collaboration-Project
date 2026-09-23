"""Local input preparation uses temporary JSON only, never physics or inference."""
import json
from pathlib import Path

import pytest

from harness.rgb_communication_study import ContractError, digest_file, read_json, verify_local_assets
from scripts import prepare_rgb_communication_replay as preparation
from scripts.prepare_rgb_communication_replay import prepare_assets


def test_pair_role_assignment_is_explicit_and_rejects_unknown_layout():
    assert preparation.pair_roles_for_assignment("r1-lower-r3-upper") == {
        "r1": "lower", "r3": "upper"}
    assert preparation.pair_roles_for_assignment("r1-upper-r3-lower") == {
        "r1": "upper", "r3": "lower"}
    with pytest.raises(ContractError, match="unknown pair role assignment"):
        preparation.pair_roles_for_assignment("r1-upper-r3-upper")


def inputs(tmp_path):
    root = tmp_path / "repo"
    fixture = root / "tests/fixtures/camera_goal_transport/reference-top.jpg"
    fixture.parent.mkdir(parents=True)
    fixture.write_bytes(b"\xff\xd8test-only\xff\xd9")
    assets = tmp_path / "existing"
    for folder, name, staged in (("grasp", "student-skill.json", False),
                                 ("varied", "varied-start-skill.json", True)):
        directory = assets / folder
        directory.mkdir(parents=True)
        (directory / "model.json").write_text('{"test_only":true}')
        model = {"path": "model.json", "sha256": digest_file(directory / "model.json")}
        (directory / name).write_text(json.dumps({"models": {"r1": {"stage": model} if staged else model}}))
        (directory / "training-report.json").write_text('{"test_only":true}')
    output = root / "outputs/new-evidence"
    output.mkdir(parents=True)
    return root, assets, output


def test_reference_mode_preserves_assets_and_does_not_copy_models(tmp_path):
    root, assets, output = inputs(tmp_path)
    before = {str(p): digest_file(p) for p in assets.rglob("*") if p.is_file()}
    result = prepare_assets(root, assets, output, mode="reference")
    assert not (output / "assets").exists()
    assert result["grasp_model_dir"] == str(assets / "grasp")
    assert result["stage_model_dir"] == str(assets / "varied")
    assert {str(p): digest_file(p) for p in assets.rglob("*") if p.is_file()} == before
    catalog = read_json(output / result["local_assets"]["path"])
    verify_local_assets(catalog, {"input_hashes": catalog["files"]})
    for refs in result["provenance"].values():
        for ref in refs:
            assert digest_file(output / ref["path"]) == ref["sha256"]
            pointer = read_json(output / ref["path"])
            assert digest_file(Path(pointer["path"])) == pointer["sha256"]


def test_copy_mode_retains_portable_relative_capsule(tmp_path):
    root, assets, output = inputs(tmp_path)
    result = prepare_assets(root, assets, output, mode="copy")
    assert result["local_assets"] is None
    assert result["grasp_model_dir"] == "outputs/new-evidence/assets/grasp"
    assert (output / "assets/grasp/model.json").read_bytes() == (assets / "grasp/model.json").read_bytes()
    assert (root / result["reference_top"]).is_file()


@pytest.mark.parametrize("bad_path", ["../escape.json", "/tmp/escape.json"])
def test_preparation_rejects_model_escape(tmp_path, bad_path):
    root, assets, output = inputs(tmp_path)
    manifest = assets / "grasp/student-skill.json"
    value = read_json(manifest)
    value["models"]["r1"]["path"] = bad_path
    manifest.write_text(json.dumps(value))
    with pytest.raises(ContractError, match="escapes"):
        prepare_assets(root, assets, output, mode="reference")


def test_preparation_rejects_changed_saved_model(tmp_path):
    root, assets, output = inputs(tmp_path)
    (assets / "grasp/model.json").write_text('{"test_only":"tampered"}')
    with pytest.raises(ContractError, match="hash mismatch"):
        prepare_assets(root, assets, output, mode="reference")


def test_d3_rejects_copy_before_any_output_or_asset_write(tmp_path, monkeypatch):
    root, assets, _ = inputs(tmp_path)
    output = root / "outputs/not-created"
    monkeypatch.setattr(preparation, "source_state", lambda _: {"clean": True})
    with pytest.raises(ContractError, match="read-only local assets"):
        preparation.prepare(root, assets, output=output, asset_mode="copy", submitter="D3",
                            pair_role_assignment="r1-upper-r3-lower")
    assert not output.exists()
