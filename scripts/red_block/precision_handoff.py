"""Short-lived handoffs between coarse approach and near-field pick.

Version 2 stores only the stopped coarse staging state (pose + gaze + range).
``pick`` must re-observe the target and compute its own final IK immediately
before grasping.  Version 1 exact pick plans remain readable for compatibility.
Any chassis/arm/camera motion invalidates the file through robot.py.
"""
from __future__ import annotations

import json
import os
import time
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import asdict
from pathlib import Path
from typing import Any


PLAN_PATH = Path("/tmp/ugrp-red-pick-plan.json")
_PLAN_PATH_OVERRIDE = ContextVar("ugrp_precision_plan_path", default=None)
MAX_PLAN_AGE_SECONDS = 60.0
COARSE_HANDOFF_VERSION = 2
COARSE_HANDOFF_KIND = "coarse_approach"
POSE_TOLERANCE_PULSE = 20


def _plan_path() -> Path:
    return _PLAN_PATH_OVERRIDE.get() or PLAN_PATH


@contextmanager
def use_plan_path(path: Path):
    """Use a robot-local SIM handoff path without changing the REAL default."""
    token = _PLAN_PATH_OVERRIDE.set(Path(path))
    try:
        yield
    finally:
        _PLAN_PATH_OVERRIDE.reset(token)


def invalidate_pick_plan() -> None:
    path = _plan_path()
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def save_pick_plan(
    plan: Any,
    *,
    robot_pose: dict[int, int],
    target_visible: bool,
    target_color: str = "red",
) -> None:
    if target_color not in {"red", "blue", "yellow"}:
        raise ValueError("target_color must be red, blue, or yellow")
    payload = {
        "version": 1,
        "created_at": time.time(),
        "target_visible": bool(target_visible),
        "target_color": target_color,
        "measurement_pose": {str(k): int(v) for k, v in robot_pose.items() if int(k) in {3, 4, 5, 6}},
        "plan": asdict(plan),
    }
    path = _plan_path()
    temporary = path.with_suffix(f".tmp.{os.getpid()}")
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def save_coarse_handoff(
    *,
    robot_pose: dict[int, int],
    target_visible: bool,
    target_color: str,
    gaze_pan: int,
    gaze_tilt: int,
    coarse_radius_cm: float | None = None,
) -> None:
    """Persist only the stopped coarse staging state for the next ``pick``."""
    if target_color not in {"red", "blue", "yellow"}:
        raise ValueError("target_color must be red, blue, or yellow")
    payload = {
        "version": COARSE_HANDOFF_VERSION,
        "kind": COARSE_HANDOFF_KIND,
        "created_at": time.time(),
        "target_visible": bool(target_visible),
        "target_color": target_color,
        "measurement_pose": {
            str(k): int(v) for k, v in robot_pose.items() if int(k) in {3, 4, 5, 6}
        },
        "gaze": {"pan": int(gaze_pan), "tilt": int(gaze_tilt)},
        "coarse_radius_cm": (
            None if coarse_radius_cm is None else float(coarse_radius_cm)
        ),
    }
    path = _plan_path()
    temporary = path.with_suffix(f".tmp.{os.getpid()}")
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def load_coarse_handoff_payload(*, now: float | None = None) -> dict[str, Any]:
    """Load and validate a fresh version-2 coarse approach handoff."""
    path = _plan_path()
    if not path.is_file():
        raise RuntimeError(
            "precision pick plan is missing; fresh coarse approach handoff is required before pick"
        )
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff is unreadable; run approach again") from exc
    if (
        payload.get("version") != COARSE_HANDOFF_VERSION
        or payload.get("kind") != COARSE_HANDOFF_KIND
    ):
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff has an unsupported format; run approach again")
    color = payload.get("target_color")
    if color not in {"red", "blue", "yellow"}:
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff has an unsupported target color; run approach again")
    gaze = payload.get("gaze")
    if not isinstance(gaze, dict) or not all(k in gaze for k in ("pan", "tilt")):
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff has no valid gaze; run approach again")
    created = payload.get("created_at")
    if isinstance(created, bool) or not isinstance(created, (int, float)):
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff has no valid timestamp; run approach again")
    age = (time.time() if now is None else float(now)) - float(created)
    if age < -2.0 or age > MAX_PLAN_AGE_SECONDS:
        invalidate_pick_plan()
        raise RuntimeError(
            f"precision pick plan is stale ({age:.1f}s); run approach immediately before pick"
        )
    if payload.get("target_visible") is not True:
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff did not verify target visibility; run approach again")
    return payload


def require_coarse_handoff_color(payload: dict[str, Any], target_color: str) -> None:
    planned = payload.get("target_color")
    if planned != target_color:
        invalidate_pick_plan()
        raise RuntimeError(
            f"coarse approach handoff target is {planned}, not requested {target_color}; run approach again"
        )


def validate_coarse_handoff_pose(
    current_pose: dict[int, int], payload: dict[str, Any]
) -> None:
    """Require that nothing moved between coarse approach exit and pick start."""
    saved = payload.get("measurement_pose")
    if not isinstance(saved, dict):
        invalidate_pick_plan()
        raise RuntimeError("coarse approach handoff has no measurement pose; run approach again")
    for servo in (3, 4, 5, 6):
        expected = saved.get(str(servo))
        current = current_pose.get(servo)
        if expected is None or current is None:
            invalidate_pick_plan()
            raise RuntimeError(
                "arm/camera pose changed or is unknown since coarse approach; run approach again"
            )
        if abs(int(current) - int(expected)) > POSE_TOLERANCE_PULSE:
            invalidate_pick_plan()
            raise RuntimeError(
                f"arm/camera moved after coarse approach (servo {servo}: {current} vs {expected}); "
                "run approach again"
            )


def load_pick_plan_payload(*, now: float | None = None) -> dict[str, Any]:
    path = _plan_path()
    if not path.is_file():
        raise RuntimeError("precision pick plan is missing; run approach immediately before pick")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        invalidate_pick_plan()
        raise RuntimeError("precision pick plan is unreadable; run approach again") from exc
    if payload.get("version") != 1 or not isinstance(payload.get("plan"), dict):
        invalidate_pick_plan()
        raise RuntimeError("precision pick plan has an unsupported format; run approach again")
    # Version 1 plans created before generic colours are red by definition.
    color = payload.get("target_color", "red")
    if color not in {"red", "blue", "yellow"}:
        invalidate_pick_plan()
        raise RuntimeError("precision pick plan has an unsupported target color; run approach again")
    created = payload.get("created_at")
    if isinstance(created, bool) or not isinstance(created, (int, float)):
        invalidate_pick_plan()
        raise RuntimeError("precision pick plan has no valid timestamp; run approach again")
    age = (time.time() if now is None else float(now)) - float(created)
    if age < -2.0 or age > MAX_PLAN_AGE_SECONDS:
        invalidate_pick_plan()
        raise RuntimeError(
            f"precision pick plan is stale ({age:.1f}s); run approach immediately before pick"
        )
    return payload


def require_pick_plan_color(payload: dict[str, Any], target_color: str) -> None:
    planned = payload.get("target_color", "red")
    if planned != target_color:
        invalidate_pick_plan()
        raise RuntimeError(
            f"precision pick plan target is {planned}, not requested {target_color}; run approach again"
        )


def validate_measurement_pose(current_pose: dict[int, int], payload: dict[str, Any]) -> None:
    saved = payload.get("measurement_pose")
    if not isinstance(saved, dict):
        invalidate_pick_plan()
        raise RuntimeError("precision pick plan has no measurement pose; run approach again")
    for servo in (3, 4, 5, 6):
        expected = saved.get(str(servo))
        current = current_pose.get(servo)
        if expected is None or current is None:
            invalidate_pick_plan()
            raise RuntimeError("arm/camera pose changed or is unknown since approach; run approach again")
        if abs(int(current) - int(expected)) > POSE_TOLERANCE_PULSE:
            invalidate_pick_plan()
            raise RuntimeError(
                f"arm/camera moved after approach (servo {servo}: {current} vs {expected}); run approach again"
            )


def rebuild_pick_plan(precision: Any, payload: dict[str, Any]) -> Any:
    raw = payload["plan"]
    block_raw = raw.get("block")
    if not isinstance(block_raw, dict):
        raise RuntimeError("precision pick plan is missing its block coordinate")
    block = precision.BlockEstimate(**block_raw)
    hover_pose = {int(k): int(v) for k, v in raw["hover_pose"].items()}
    grasp_pose = {int(k): int(v) for k, v in raw["grasp_pose"].items()}
    return precision.PickPlan(
        block=block,
        base_pulse=int(raw["base_pulse"]),
        hover_pose=hover_pose,
        grasp_pose=grasp_pose,
        fingertip_radius_cm=float(raw["fingertip_radius_cm"]),
    )
