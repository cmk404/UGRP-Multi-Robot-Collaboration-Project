"""Short-lived evidence handoff from a successful precision pick to place.

The MasterPi gripper has no force/current sensor, so this file is deliberately
*not* a claim that an object is certainly held. It only proves that the same
robot recently completed the guarded precision-pick sequence, the pickup floor
was visually clear, and the gripper was commanded closed. Destructive place
scripts require this causal evidence in addition to the persisted gripper pose.
"""
from __future__ import annotations

import json
import os
import time
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any


CARRY_PATH = Path("/tmp/ugrp-red-carry-handoff.json")
_CARRY_PATH_OVERRIDE = ContextVar("ugrp_carry_handoff_path", default=None)
CARRY_MAX_AGE_SECONDS = 600.0
GRIPPER_CLOSED_MAX_PULSE = 1700
CARRY_POSE_SERVOS = (3, 4, 5, 6)


def _carry_path() -> Path:
    return _CARRY_PATH_OVERRIDE.get() or CARRY_PATH


@contextmanager
def use_carry_path(path: Path):
    """Use a robot-local SIM handoff path without changing the REAL default."""
    token = _CARRY_PATH_OVERRIDE.set(Path(path))
    try:
        yield
    finally:
        _CARRY_PATH_OVERRIDE.reset(token)


def invalidate_carry_handoff() -> None:
    path = _carry_path()
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def invalidate_return_path() -> None:
    """Keep carry evidence but revoke the no-odometry pickup-site return path."""
    path = _carry_path()
    if not path.is_file():
        return
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        invalidate_carry_handoff()
        return
    if not isinstance(payload, dict) or "return_path" not in payload:
        return
    payload.pop("return_path", None)
    temporary = path.with_suffix(f".tmp.{os.getpid()}")
    try:
        temporary.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
        temporary.replace(path)
    except OSError:
        try:
            temporary.unlink()
        except OSError:
            pass
        invalidate_carry_handoff()


def save_carry_handoff(
    *,
    gripper_pulse: int,
    robot_pose: dict[int, int],
    target_color: str = "red",
    return_base_pulse: int | None = None,
    return_hover_pose: dict[int, int] | None = None,
    return_descent_poses: list[dict[int, int]] | None = None,
) -> None:
    pulse = int(gripper_pulse)
    if pulse > GRIPPER_CLOSED_MAX_PULSE:
        raise RuntimeError(
            f"refusing carry handoff with open gripper command (servo1={pulse})"
        )
    if target_color not in {"red", "blue", "yellow"}:
        raise RuntimeError("refusing carry handoff with unsupported target color")
    payload = {
        "version": 1,
        "created_at": time.time(),
        "source": "precision_pick_floor_clear",
        "grasp_state": "PROBABLE_HELD",
        "target_color": target_color,
        "gripper_pulse": pulse,
        "robot_pose": {str(k): int(v) for k, v in robot_pose.items()},
    }
    if (
        return_base_pulse is not None
        or return_hover_pose is not None
        or return_descent_poses is not None
    ):
        if return_base_pulse is None or return_hover_pose is None or return_descent_poses is None:
            raise RuntimeError("carry handoff return path must be complete")
        payload["return_path"] = {
            "base_pulse": int(return_base_pulse),
            "hover_pose": {str(k): int(v) for k, v in return_hover_pose.items()},
            "descent_poses": [
                {str(k): int(v) for k, v in pose.items()}
                for pose in return_descent_poses
            ],
        }
    path = _carry_path()
    temporary = path.with_suffix(f".tmp.{os.getpid()}")
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def load_carry_handoff(*, now: float | None = None) -> dict[str, Any]:
    path = _carry_path()
    if not path.is_file():
        raise RuntimeError(
            "recent precision-pick carry handoff is missing; refusing placement"
        )
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        invalidate_carry_handoff()
        raise RuntimeError("carry handoff is unreadable; refusing placement") from exc
    if (
        payload.get("version") != 1
        or payload.get("source") != "precision_pick_floor_clear"
        or payload.get("grasp_state") != "PROBABLE_HELD"
    ):
        invalidate_carry_handoff()
        raise RuntimeError(
            "carry handoff has an unsupported provenance; refusing placement"
        )
    color = payload.get("target_color", "red")
    if color not in {"red", "blue", "yellow"}:
        invalidate_carry_handoff()
        raise RuntimeError("carry handoff has an unsupported target color; refusing placement")
    created = payload.get("created_at")
    if isinstance(created, bool) or not isinstance(created, (int, float)):
        invalidate_carry_handoff()
        raise RuntimeError("carry handoff has no valid timestamp; refusing placement")
    age = (time.time() if now is None else float(now)) - float(created)
    if age < -2.0 or age > CARRY_MAX_AGE_SECONDS:
        invalidate_carry_handoff()
        raise RuntimeError(f"carry handoff is stale ({age:.1f}s); refusing placement")
    pulse = payload.get("gripper_pulse")
    if isinstance(pulse, bool) or not isinstance(pulse, int) or pulse > GRIPPER_CLOSED_MAX_PULSE:
        invalidate_carry_handoff()
        raise RuntimeError("carry handoff does not contain a closed-gripper command")
    saved_pose = payload.get("robot_pose")
    if not isinstance(saved_pose, dict):
        invalidate_carry_handoff()
        raise RuntimeError("carry handoff has no saved robot pose; refusing placement")
    for servo in CARRY_POSE_SERVOS:
        value = saved_pose.get(str(servo))
        if isinstance(value, bool) or not isinstance(value, int):
            invalidate_carry_handoff()
            raise RuntimeError(
                f"carry handoff is missing servo {servo} pose; refusing placement"
            )
    return payload



def require_return_path(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate the exact inverse path back to the pickup site.

    Older handoffs deliberately lack this field and therefore cannot authorize
    an autonomous put-down. That is safer than guessing a floor pose.
    """
    raw = payload.get("return_path")
    if not isinstance(raw, dict):
        raise RuntimeError(
            "carry handoff has no pickup return path; refusing an uncalibrated put-down"
        )
    base = raw.get("base_pulse")
    hover = raw.get("hover_pose")
    descent = raw.get("descent_poses")
    if isinstance(base, bool) or not isinstance(base, int) or not 500 <= base <= 2500:
        raise RuntimeError("carry handoff return path has invalid base pulse")
    if not isinstance(hover, dict) or not isinstance(descent, list) or not descent:
        raise RuntimeError("carry handoff return path is incomplete")

    def pose_dict(value: Any, label: str) -> dict[int, int]:
        if not isinstance(value, dict):
            raise RuntimeError(f"carry handoff {label} is invalid")
        out: dict[int, int] = {}
        for servo in (3, 4, 5):
            pulse = value.get(str(servo), value.get(servo))
            if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
                raise RuntimeError(f"carry handoff {label} is missing valid servo {servo}")
            out[servo] = int(pulse)
        return out

    return {
        "base_pulse": int(base),
        "hover_pose": pose_dict(hover, "return hover pose"),
        "descent_poses": [pose_dict(pose, f"return descent pose {i}") for i, pose in enumerate(descent)],
    }

def require_carry_handoff(
    *,
    current_gripper_pulse: int,
    current_robot_pose: dict[int, int],
    target_color: str = "red",
) -> dict[str, Any]:
    payload = load_carry_handoff()
    if payload.get("target_color", "red") != target_color:
        invalidate_carry_handoff()
        raise RuntimeError(
            "carry handoff object identity does not match requested placement; refusing placement"
        )
    saved_pulse = int(payload["gripper_pulse"])
    current = int(current_gripper_pulse)
    if current > GRIPPER_CLOSED_MAX_PULSE:
        invalidate_carry_handoff()
        raise RuntimeError(f"gripper is open (servo1={current}); refusing placement")
    # A later closing command can change pressure slightly, so exact equality is
    # not required. The invariant is that neither saved nor current command is
    # an open/release command.
    if saved_pulse > GRIPPER_CLOSED_MAX_PULSE:
        invalidate_carry_handoff()
        raise RuntimeError("carry handoff was created with an open gripper")
    saved_pose = payload["robot_pose"]
    for servo in CARRY_POSE_SERVOS:
        saved = int(saved_pose[str(servo)])
        current_pose = current_robot_pose.get(servo)
        if isinstance(current_pose, bool) or not isinstance(current_pose, int):
            invalidate_carry_handoff()
            raise RuntimeError(
                f"current servo {servo} pose is unavailable; refusing placement"
            )
        if current_pose != saved:
            invalidate_carry_handoff()
            raise RuntimeError(
                f"carry handoff pose changed at servo {servo} "
                f"({saved}->{current_pose}); refusing placement"
            )
    return payload
