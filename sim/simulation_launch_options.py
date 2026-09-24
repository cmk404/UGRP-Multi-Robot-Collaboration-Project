"""Validated choices for the local simulation start menu.

The menu only selects existing native CLI paths. It never interprets a task
as a command or makes a research scene imply a controller.
"""
from __future__ import annotations

from pathlib import Path

from sim.research_dispatch_arena import VARIANTS
from sim.session_scenes import catalog


_SPEEDS = {0.5: "0.5", 1: "1", 2: "2", 4: "4"}
_FIELDS = frozenset({"mode", "map", "speed", "model", "task", "coordination"})


def dispatch_maps() -> list[str]:
    """Variants supported by the existing peer plan and RGB skill runner."""
    return list(VARIANTS)


def preview_maps() -> list[str]:
    """Registered local scenes that need no caller supplied map file."""
    return [row["id"] for row in catalog() if not row["id"].endswith("/file")]


def _speed(value: object) -> str:
    if isinstance(value, bool):
        raise ValueError("speed: choose 0.5, 1, 2, or 4")
    if isinstance(value, str):
        if value in {"0.5", "1", "2", "4"}:
            return value
    elif type(value) in (int, float):
        if value in _SPEEDS:
            return _SPEEDS[value]
    raise ValueError("speed: choose 0.5, 1, 2, or 4")


def build_command(root: Path, selection: dict) -> list[str]:
    """Return a shell-free argv for the native CLI; execute with ``cwd=root``.

    ``speed`` changes observer pacing only. It does not choose a different
    policy, alter the physics configuration, or establish a new experiment.
    """
    if not isinstance(selection, dict):
        raise ValueError("selection: object required")
    if selection.keys() - _FIELDS:
        raise ValueError("selection: unsupported field")
    if not (Path(root) / "scripts/open_simulation.command").is_file():
        raise ValueError("root: simulation launcher is unavailable")

    mode = selection.get("mode")
    map_id = selection.get("map")
    speed = _speed(selection.get("speed"))
    if not isinstance(map_id, str):
        raise ValueError("map: choose an available map")

    base = ["bash", "scripts/open_simulation.command"]
    if mode == "llm_dispatch":
        if map_id not in dispatch_maps():
            raise ValueError("map: unsupported dispatch variant")
        model = selection.get("model")
        if (not isinstance(model, str) or not model or len(model) > 160
                or model.startswith("-") or any(char.isspace() or char == "\0" for char in model)):
            raise ValueError("model: nonempty ID up to 160 characters, without whitespace or leading dash")
        task = selection.get("task")
        if not isinstance(task, str) or not task.strip() or len(task) > 4000 or "\0" in task:
            raise ValueError("task: nonempty instruction up to 4000 characters required")
        coordination = selection.get("coordination", "plan_first")
        if coordination not in ("plan_first", "dynamic"):
            raise ValueError("coordination: plan_first or dynamic")
        extra = ["--coordination", "dynamic"] if coordination == "dynamic" else []
        return [*base, "dispatch", "--task", task, "--variant", map_id,
                "--model", model, "--realtime-factor", speed, *extra]

    if mode == "preview":
        if map_id not in preview_maps():
            raise ValueError("map: unregistered or file-backed scene")
        if (selection.get("model") not in (None, "") or selection.get("task") not in (None, "")
                or selection.get("coordination") not in (None, "plan_first")):
            raise ValueError("preview: model and task are unavailable")
        if not (Path(root) / "configs/simulation/local.json").is_file():
            raise ValueError("root: local simulation configuration is unavailable")
        return [*base, "run", "configs/simulation/local.json", "--scene", map_id,
                "--realtime-factor", speed, "--paused", "--capture", "--sim-seconds", "600"]

    raise ValueError("mode: choose llm_dispatch or preview")
