"""Preflight and provenance checks for managed local research inputs."""
from __future__ import annotations

import json
from pathlib import Path
from unittest import mock

import pytest

from sim import workflow_manager as wm


@pytest.fixture
def project(tmp_path: Path) -> Path:
    (tmp_path / "configs").mkdir()
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "fixture.py").write_text("", encoding="utf-8")
    workflows = []
    for workflow_id, required in (("local", {"run": ["CONFIG"], "console": ["CONFIG"]}),
                                  ("act-map-suite", ["--spec"]), ("act-input-training", [])):
        workflows.append({"id": workflow_id, "version": "1.0.0", "entry": "scripts/fixture.py",
                          "runner": "scripts.fixture", "output_flag": "--output", "output_kind": "directory",
                          "required_inputs": required, "side_effect": "fixture"})
    (tmp_path / wm.CATALOG).write_text(json.dumps({"schema": "ugrp.local_workflow_catalog.v1",
                                                    "workflows": workflows}), encoding="utf-8")
    return tmp_path


def local_config(project: Path) -> tuple[Path, dict[str, Path]]:
    folder = project / "case"
    folder.mkdir()
    paths = {name: folder / name for name in ("map.json", "builder.py", "action.py", "controller.py")}
    for name, path in paths.items():
        # The planner must never execute these extension modules.
        path.write_text("raise RuntimeError('extension executed')\n" if name.endswith(".py") else "{}\n",
                        encoding="utf-8")
    config = folder / "config.json"
    config.write_text(json.dumps({"version": 1,
                                  "scene": {"layout": "navigation/file", "map_file": "map.json",
                                            "builder": "builder.py:build"},
                                  "action_plugins": {"custom": "action.py:lower"},
                                  "controllers": {"r1": {"factory": "controller.py:create"}}}), encoding="utf-8")
    return config, paths


def test_local_plan_records_map_and_extensions_without_executing_them(project: Path) -> None:
    config, paths = local_config(project)
    planned = wm.plan(project, "local", ["run", str(config), "--headless"])
    assert {row["path"] for row in planned["inputs"]} == {str(config), *(str(path) for path in paths.values())}
    assert planned["execution_started"] is False
    assert not (project / "outputs").exists()


def test_local_plan_uses_effective_extension_overrides(project: Path) -> None:
    config, paths = local_config(project)
    alternate_builder = config.parent / "alternate_builder.py"
    alternate_controller = config.parent / "alternate_controller.py"
    for path in (alternate_builder, alternate_controller):
        path.write_text("raise RuntimeError('extension executed')\n", encoding="utf-8")
    planned = wm.plan(project, "local", ["run", str(config), "--headless",
                                         "--scene-builder", "alternate_builder.py:build",
                                         "--controller", "r1=alternate_controller.py:create"])
    recorded = {row["path"] for row in planned["inputs"]}
    assert str(alternate_builder) in recorded and str(alternate_controller) in recorded
    assert str(paths["builder.py"]) not in recorded and str(paths["controller.py"]) not in recorded


def test_local_rejects_missing_nested_input_before_record(project: Path) -> None:
    config, paths = local_config(project)
    paths["map.json"].unlink()
    with pytest.raises(ValueError, match="local config input does not exist"):
        wm.plan(project, "local", ["run", str(config), "--headless"])
    with pytest.raises(ValueError, match="local config input does not exist"):
        wm.run_inprocess(project, "local", ["run", str(config), "--headless"], lambda _: 0, output=None)
    assert not (project / "outputs").exists()


def test_local_record_detects_nested_input_change(project: Path) -> None:
    config, paths = local_config(project)

    def invoke(output: Path) -> int:
        output.mkdir()
        paths["map.json"].write_text('{"changed":true}\n', encoding="utf-8")
        return 0

    assert wm.run_inprocess(project, "local", ["run", str(config), "--headless"], invoke, output=None) == 0
    record = wm._records(project)[0]
    manifest = json.loads(record.read_text(encoding="utf-8"))
    before = {row["path"]: row["sha256"] for row in manifest["inputs_before"]}
    after = {row["path"]: row["sha256"] for row in manifest["inputs_after"]}
    assert before[str(paths["map.json"])] != after[str(paths["map.json"])]
    assert manifest["inputs_changed_during_run"] is True


def test_act_map_spec_is_required_validated_and_recorded(project: Path) -> None:
    spec = project / "suite.json"
    with pytest.raises(ValueError, match="requires --spec"):
        wm.plan(project, "act-map-suite", [])
    with pytest.raises(ValueError, match="--spec input is not a file"):
        wm.plan(project, "act-map-suite", ["--spec", str(spec)])
    spec.mkdir()
    with pytest.raises(ValueError, match="--spec input is not a file"):
        wm.plan(project, "act-map-suite", ["--spec", str(spec)])
    spec.rmdir()
    spec.write_text("{}\n", encoding="utf-8")
    planned = wm.plan(project, "act-map-suite", ["--spec", str(spec)])
    assert [row["path"] for row in planned["inputs"]] == [str(spec)]


def test_act_input_training_rejects_resume(project: Path) -> None:
    for argv in (["--resume", "prior"], ["--resume=prior"]):
        with pytest.raises(ValueError, match="fresh output"):
            wm.plan(project, "act-input-training", argv)


def test_act_map_render_selects_mjpython_on_mac(project: Path) -> None:
    python = project / "python"
    mjpython = project / "mjpython"
    python.write_text("", encoding="utf-8")
    mjpython.write_text("", encoding="utf-8")
    mjpython.chmod(0o755)
    row = {"id": "act-map-suite"}
    with mock.patch.object(wm.sys, "platform", "darwin"), mock.patch.object(wm.sys, "executable", str(python)):
        assert wm._select_python(row, []) == str(python)
        assert wm._select_python(row, ["--render"]) == str(mjpython)
