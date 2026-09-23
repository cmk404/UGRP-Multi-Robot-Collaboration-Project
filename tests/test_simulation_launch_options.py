"""The start menu only builds supported native CLI argument vectors."""
from pathlib import Path

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
