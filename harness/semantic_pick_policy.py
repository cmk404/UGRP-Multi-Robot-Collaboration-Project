"""Two camera-only planners and a command-grounded semantic arm interpreter."""

from __future__ import annotations

import base64
import hashlib
import itertools
import json
import math
import re
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np
from harness.visual_arm import tool_pose


COARSE_KINDS = frozenset({"approach", "pick", "check_grip", "wait"})
SEMANTIC_UNITS = frozenset({
    "MV_FWD", "MV_BACK", "MV_LEFT", "MV_RIGHT", "MV_UP", "MV_DOWN",
    "PITCH_UP", "PITCH_DOWN", "GRASP", "RELEASE", "STILL",
    "BASE_FWD", "BASE_LEFT", "BASE_RIGHT",
})

PROFILE = {
    "schema": "ugrp.semantic_pick.v1",
    "translation_increment_m": 0.01,
    "pitch_increment_deg": 5.0,
    "translation_residual_limit_m": 0.002,
    "pitch_residual_limit_deg": 1.0,
    "pulse_bounds": [500, 2500],
    "grasp_pulse": 1500,
    "release_pulse": 2000,
    "base_forward": {"fwd": 0.1, "turn": 0.0, "duration": 0.6},
    "base_left": {"fwd": 0.0, "turn": 0.12, "duration": 0.4},
    "base_right": {"fwd": 0.0, "turn": -0.12, "duration": 0.4},
    "still_duration": 0.3,
    "frame": "robot_base_x_forward_y_left_z_up",
}
PROFILE_HASH = hashlib.sha256(
    json.dumps(PROFILE, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()

_SYSTEM_COMMON = """You control one MasterPi in a raise-and-hold box task.
The target is the small cyan box. Other colored objects are not the target.
Use only the two current RGB images and the listed actions previously selected by you.
OWN_CAMERA is the robot's unchanged physical wrist camera. SHARED_TOP_CAMERA is the
unchanged fixed overhead camera. The action history is issued-command history, never
measured joint state or proof that motion succeeded. Do not infer hidden simulator
state. Return exactly one JSON object with keys reason and action.
reason must be a nonempty string of at most 300 characters."""

_MODE_PROMPTS = {
    "skill": """Choose one coarse action. Allowed actions are {\"kind\":\"approach\"},
{\"kind\":\"pick\"}, {\"kind\":\"check_grip\"}, or
{\"kind\":\"wait\",\"duration\":0.3}. approach delegates the complete visual approach;
pick delegates close, lift, and visual hold checking. Never release or finish.""",
    "semantic": """Choose one fine semantic unit as
{\"kind\":\"semantic\",\"unit\":UNIT}. UNIT is one of MV_FWD, MV_BACK, MV_LEFT,
MV_RIGHT, MV_UP, MV_DOWN, PITCH_UP, PITCH_DOWN, GRASP, RELEASE, STILL, BASE_FWD,
BASE_LEFT, BASE_RIGHT. MV units are fixed 1 cm Cartesian end-effector increments in
robot-base axes (+x forward,+y left,+z up), not screen directions or servo increments.
Pitch units are fixed 5 degree changes. GRASP closes the fingers, RELEASE opens them,
STILL waits 0.3 seconds. BASE_FWD moves the mobile base forward at 0.1 for 0.6 seconds;
BASE_LEFT and BASE_RIGHT rotate the mobile base at +0.12/-0.12 for 0.4 seconds.
Raise the box and hold it; do not intentionally
release a held box.""",
}


class PickMatchPlanner:
    """Shared request boundary for coarse-skill and fine-semantic cohorts."""

    def __init__(self, completer: Any, mode: str = "skill"):
        if mode not in _MODE_PROMPTS:
            raise ValueError("mode must be skill or semantic")
        self.completer = completer
        self.mode = mode
        self._issued: list[dict[str, Any]] = []
        self.last_request: dict[str, Any] | None = None
        self.last_response: Any = None

    @property
    def issued_actions(self) -> tuple[dict[str, Any], ...]:
        return tuple(dict(action) for action in self._issued)

    def record_issued(self, action: Mapping[str, Any]) -> None:
        parsed = _validate_model_action(action, self.mode)
        self._issued.append(parsed)
        self._issued = self._issued[-4:]

    def prepare_request(self, own_jpeg: bytes, top_jpeg: bytes) -> dict[str, Any]:
        own = _validate_jpeg(own_jpeg, "own_jpeg")
        top = _validate_jpeg(top_jpeg, "top_jpeg")
        context = {"task": "raise_and_hold_box", "mode": self.mode,
                   "own_model_selected_actions": self.issued_actions}
        request = {
            "messages": [
                {"role": "system", "content": _SYSTEM_COMMON + "\n" + _MODE_PROMPTS[self.mode]},
                {"role": "user", "content": json.dumps(context, separators=(",", ":"))},
            ],
            "images": [
                {"label": "OWN_CAMERA", "image": _data_uri(own)},
                {"label": "SHARED_TOP_CAMERA", "image": _data_uri(top)},
            ],
            "input_frames": [
                {"camera": "own", "sha256": hashlib.sha256(own).hexdigest()},
                {"camera": "shared_top", "sha256": hashlib.sha256(top).hexdigest()},
            ],
            "profile_hash": PROFILE_HASH,
        }
        self.last_request = request
        return request

    def decide(self, own_jpeg: bytes, top_jpeg: bytes) -> dict[str, Any]:
        self.last_response = None
        request = self.prepare_request(own_jpeg, top_jpeg)
        raw = self.completer.complete(request["messages"], images=request["images"])
        self.last_response = raw
        try:
            if isinstance(raw, str):
                # Transport formatting only: accept one complete fenced JSON
                # object, never extract an action out of surrounding prose.
                fenced = re.fullmatch(r"\s*```(?:json)?\s*\n(.*?)\n```\s*", raw, re.DOTALL)
                raw = fenced.group(1) if fenced else raw
            value = json.loads(raw) if isinstance(raw, str) else raw
            if not isinstance(value, Mapping) or set(value) != {"reason", "action"}:
                raise ValueError("response must contain exactly reason and action")
            reason = value["reason"]
            if not isinstance(reason, str) or not reason.strip() or len(reason) > 300:
                raise ValueError("reason must be a non-empty string of at most 300 characters")
            return {"reason": reason, "action": _validate_model_action(value["action"], self.mode)}
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"INVALID_PICK_RESPONSE:{exc}") from exc


class SemanticPickInterpreter:
    """Compile semantic units using only the history of commands actually issued."""

    def __init__(self, initial_issued: Mapping[int | str, int]):
        self._pulses = _initial_pulses(initial_issued)

    @property
    def issued_targets(self) -> dict[str, int]:
        return {str(key): value for key, value in sorted(self._pulses.items())}

    def compile(self, action: Mapping[str, Any]) -> dict[str, Any]:
        parsed = _validate_model_action(action, "semantic")
        unit = parsed["unit"]
        if unit == "GRASP":
            return {"kind": "pose", "pulses": {1: PROFILE["grasp_pulse"]}}
        if unit == "RELEASE":
            return {"kind": "pose", "pulses": {1: PROFILE["release_pulse"]}}
        if unit == "STILL":
            return {"kind": "wait", "duration": PROFILE["still_duration"]}
        if unit.startswith("BASE_"):
            key = {"BASE_FWD": "base_forward", "BASE_LEFT": "base_left",
                   "BASE_RIGHT": "base_right"}[unit]
            drive = PROFILE[key]
            return {"kind": "drive", **drive}

        current = tool_pose(self._pulses)
        xyz = np.array([current.x_m, current.y_m, current.z_m], dtype=float)
        target_pitch = current.pitch_deg
        offsets = {
            "MV_FWD": (0.01, 0, 0), "MV_BACK": (-0.01, 0, 0),
            "MV_LEFT": (0, 0.01, 0), "MV_RIGHT": (0, -0.01, 0),
            "MV_UP": (0, 0, 0.01), "MV_DOWN": (0, 0, -0.01),
        }
        if unit in offsets:
            xyz += np.asarray(offsets[unit])
        else:
            target_pitch += 5.0 if unit == "PITCH_UP" else -5.0
        solved = self._solve(xyz, target_pitch)
        return {"kind": "pose", "pulses": solved}

    def record_issued(self, raw_action: Mapping[str, Any]) -> None:
        if not isinstance(raw_action, Mapping):
            raise ValueError("raw action must be a mapping")
        kind = raw_action.get("kind")
        if kind == "arm" and set(raw_action) == {"kind", "servo_id", "pulse"}:
            servo, pulse = raw_action["servo_id"], raw_action["pulse"]
            if servo not in {1, 3, 4, 5}:
                raise ValueError("invalid arm servo")
            self._pulses[servo] = _pulse(pulse)
        elif kind == "look" and set(raw_action) == {"kind", "pan_pulse"}:
            self._pulses[6] = _pulse(raw_action["pan_pulse"])
        elif kind not in {"drive", "wait"}:
            raise ValueError("unsupported raw issued action")

    def _solve(self, xyz: np.ndarray, pitch: float) -> dict[int, int]:
        seed = np.asarray([self._pulses[s] for s in (3, 4, 5, 6)], dtype=float)

        def residual(values: np.ndarray) -> np.ndarray:
            pose = tool_pose(dict(zip((3, 4, 5, 6), values)))
            return np.asarray([pose.x_m-xyz[0], pose.y_m-xyz[1], pose.z_m-xyz[2],
                               (pose.pitch_deg-pitch) / 100.0])

        values = seed.copy()
        # PWM-to-metre derivatives are around 1e-4, so damping must be on the
        # scale of J.T@J rather than a generic unit-scale optimizer default.
        damping = 1e-10
        for _ in range(120):
            error = residual(values)
            jacobian = np.empty((4, 4), dtype=float)
            for column in range(4):
                step = 1.0 if values[column] <= 2499 else -1.0
                shifted = values.copy()
                shifted[column] += step
                jacobian[:, column] = (residual(shifted) - error) / step
            lhs = jacobian.T @ jacobian + damping * np.eye(4)
            try:
                delta = np.linalg.solve(lhs, -(jacobian.T @ error))
            except np.linalg.LinAlgError as exc:
                raise ValueError("SEMANTIC_TARGET_UNREACHABLE") from exc
            delta = np.clip(delta, -100.0, 100.0)
            proposed = np.clip(values + delta, 500.0, 2500.0)
            if np.linalg.norm(residual(proposed)) < np.linalg.norm(error):
                values = proposed
                damping = max(1e-14, damping * 0.35)
            else:
                damping = min(1e8, damping * 10.0)
            if np.linalg.norm(delta) < 1e-4:
                break
        rounded = np.rint(values).astype(int)
        best = None
        for delta in itertools.product(range(-2, 3), repeat=4):
            candidate = np.clip(rounded + np.asarray(delta), 500, 2500).astype(int)
            pose = tool_pose(dict(zip((3, 4, 5, 6), (int(v) for v in candidate))))
            position_error = float(np.linalg.norm(
                np.asarray([pose.x_m, pose.y_m, pose.z_m]) - xyz))
            pitch_error = abs(pose.pitch_deg - pitch)
            score = position_error + pitch_error / 100.0
            if best is None or score < best[0]:
                best = (score, position_error, pitch_error, candidate)
        assert best is not None
        if best[1] > PROFILE["translation_residual_limit_m"] or best[2] > PROFILE["pitch_residual_limit_deg"]:
            raise ValueError("SEMANTIC_TARGET_UNREACHABLE")
        return dict(zip((3, 4, 5, 6), (int(v) for v in best[3])))


def _validate_model_action(value: Any, mode: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("action must be an object")
    if mode == "skill":
        kind = value.get("kind")
        expected = {"kind", "duration"} if kind == "wait" else {"kind"}
        if kind not in COARSE_KINDS or set(value) != expected:
            raise ValueError("invalid coarse action")
        if kind == "wait" and (isinstance(value["duration"], bool) or value["duration"] != 0.3):
            raise ValueError("coarse wait duration must be 0.3")
        return dict(value)
    if set(value) != {"kind", "unit"} or value.get("kind") != "semantic" or value.get("unit") not in SEMANTIC_UNITS:
        raise ValueError("invalid semantic action")
    return dict(value)


def _validate_jpeg(value: Any, name: str) -> bytes:
    if not isinstance(value, (bytes, bytearray)):
        raise ValueError(f"{name} must be JPEG bytes")
    data = bytes(value)
    if cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR) is None:
        raise ValueError(f"{name} is not a valid JPEG")
    return data


def _data_uri(value: bytes) -> str:
    return "data:image/jpeg;base64," + base64.b64encode(value).decode("ascii")


def _pulse(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 500 <= value <= 2500:
        raise ValueError("pulse must be an integer in 500..2500")
    return value


def _initial_pulses(value: Mapping[int | str, int]) -> dict[int, int]:
    if not isinstance(value, Mapping):
        raise ValueError("initial_issued must be a mapping")
    result = {}
    for servo in (1, 3, 4, 5, 6):
        raw = value.get(servo, value.get(str(servo)))
        result[servo] = _pulse(raw)
    return result


__all__ = ["PickMatchPlanner", "SemanticPickInterpreter", "PROFILE", "PROFILE_HASH",
           "COARSE_KINDS", "SEMANTIC_UNITS"]
