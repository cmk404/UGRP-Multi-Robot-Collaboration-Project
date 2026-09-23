"""The start menu only builds supported native CLI argument vectors."""
from pathlib import Path
import json
import os
import subprocess
import sys

import pytest

from sim.research_dispatch_arena import VARIANTS
from sim.session_scenes import catalog
from sim.simulation_launch_options import build_command, dispatch_maps, preview_maps


ROOT = Path(__file__).resolve().parents[1]


def test_mode_maps_follow_existing_catalog_without_file_backed_scenes():
    assert dispatch_maps() == list(VARIANTS)
    assert preview_maps() == [row["id"] for row in catalog() if not row["id"].endswith("/file")]
    assert "navigation/file" not in preview_maps()
    assert "pair_navigation/file" not in preview_maps()


def test_dispatch_argv_preserves_natural_language_as_one_argument():
    task = "dock_b로 운반해; $(touch /tmp/not-run)\n다시 확인해"
    assert build_command(ROOT, {"mode": "llm_dispatch", "map": "open", "speed": 2,
                                "model": "gemini-3.8-flash", "task": task}) == [
        "bash", "scripts/open_simulation.command", "dispatch", "--task", task,
        "--variant", "open", "--model", "gemini-3.8-flash", "--realtime-factor", "2",
    ]


@pytest.mark.parametrize("realtime", [False, True])
def test_mac_launcher_uses_only_one_native_viewer_process(tmp_path, realtime):
    python = tmp_path / "python"
    python.write_text(f"#!{sys.executable}\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n")
    python.chmod(0o755)
    (tmp_path / "mjpython").symlink_to(python)
    uname = tmp_path / "uname"
    uname.write_text("#!/bin/sh\necho Darwin\n")
    uname.chmod(0o755)
    options = ["--realtime-control"] if realtime else []
    result = subprocess.run(["bash", "scripts/open_simulation.command", "dispatch", *options],
                            cwd=ROOT, env={**os.environ, "UGRP_SIM_PYTHON": str(python),
                                           "PATH": str(tmp_path) + os.pathsep + os.environ["PATH"]},
                            check=True, capture_output=True, text=True)
    argv = json.loads(result.stdout)
    assert argv[argv.index("--") + 1] == str(python if realtime else tmp_path / "mjpython")


def test_preview_argv_has_no_model_or_task_and_uses_registered_scene():
    command = build_command(ROOT, {"mode": "preview", "map": "dispatch/shared_crossing",
                                   "speed": "0.5"})
    assert command == [
        "bash", "scripts/open_simulation.command", "run", "configs/simulation/local.json",
        "--scene", "dispatch/shared_crossing", "--realtime-factor", "0.5",
        "--paused", "--capture", "--sim-seconds", "600",
    ]
    assert "--model" not in command and "--task" not in command


@pytest.mark.parametrize("selection", [
    {"mode": "llm_dispatch", "map": "dispatch/open", "speed": 1, "model": "m", "task": "go"},
    {"mode": "llm_dispatch", "map": "unknown", "speed": 1, "model": "m", "task": "go"},
    {"mode": "preview", "map": "open", "speed": 1},
    {"mode": "preview", "map": "navigation/file", "speed": 1},
    {"mode": "preview", "map": "dispatch/open", "speed": 1, "task": "go"},
    {"mode": "preview", "map": "dispatch/open", "speed": 1, "model": "m"},
    {"mode": "preview", "map": "dispatch/open", "speed": 0},
    {"mode": "preview", "map": "dispatch/open", "speed": True},
    {"mode": "preview", "map": "dispatch/open", "speed": "1e0"},
    {"mode": "preview", "map": "dispatch/open", "speed": 1, "extra": "--headless"},
    {"mode": "bad", "map": "dispatch/open", "speed": 1},
])
def test_unsupported_combinations_are_rejected(selection):
    with pytest.raises(ValueError):
        build_command(ROOT, selection)


@pytest.mark.parametrize("model,task", [
    ("", "go"), ("-bad", "go"), ("two words", "go"), ("bad\nmodel", "go"),
    ("x" * 161, "go"), ("m", ""), ("m", "   "), ("m", "x" * 4001),
    ("m", "go\0now"),
])
def test_dispatch_requires_bounded_model_id_and_task(model, task):
    with pytest.raises(ValueError):
        build_command(ROOT, {"mode": "llm_dispatch", "map": "open", "speed": 1,
                             "model": model, "task": task})
