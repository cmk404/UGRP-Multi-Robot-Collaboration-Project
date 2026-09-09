"""Stateless two-camera policy for the cooperative orange-beam task."""

from __future__ import annotations

import base64
import copy
import json
import math
import re
from typing import Any


_ROBOTS = {"r1", "r3"}
_ACTION_FIELDS = {
    "drive": {"kind", "forward", "turn", "duration_s"},
    "look": {"kind", "pan_pulse"},
    "arm": {"kind", "servo_id", "pulse"},
    "wait": {"kind"},
}

_SYSTEM_TASK = """You control {robot_id}, one of two robots (r1 and r3). Cooperate to carry
the plain orange beam from the nearby blue square into the neighboring green square.

Your only observations are exactly two current RGB images: OWN_VIEW is {robot_id}'s
wrist robot_cam, and OVERHEAD is cctv_top. Use only their visible pixels. You have no
world coordinates, poses, simulator state, contacts, PWM or actuator state, memory,
history, peer status, messages, action budget, or runtime feedback. Do not invent or
request any of them. Do not assume a precomputed target pose, route, grasp, or motion
macro. Choose just the next short raw action from the current images. Each robot can
turn and drive its base and can move its side arm independently.

Return one JSON object with exactly "reason" and "action". reason is a nonempty string
of at most 500 characters. action must have exactly one of these forms:
{"kind":"drive","forward":0..0.15,"turn":-0.2..0.2,"duration_s":0..1}
{"kind":"look","pan_pulse":500..2500}
{"kind":"arm","servo_id":1|3|4|5,"pulse":500..2500}
{"kind":"wait"}
Numbers must be finite. Return JSON only."""

_USER_TEXT = "Inspect OWN_VIEW and OVERHEAD pixels and select the next action."


def static_task(robot_id: str, task: str = 'carry') -> str:
    if task not in {'carry', 'grasp'}:
        raise ValueError('task must be carry or grasp')
    text = _SYSTEM_TASK.replace('{robot_id}', robot_id)
    if task == 'grasp':
        text = text.replace(
            'Cooperate to carry\nthe plain orange beam from the nearby blue square into the neighboring green square.',
            'Cooperate to grasp and lift the plain orange beam at the nearby blue square.\nHold it raised together; do not transport or release it in this task.')
        text += '''
Fixed hardware command documentation (not measured state): servo 1 is the gripper;
2000 opens and 1500 closes it. Servo 3 is wrist pitch, 4 elbow, 5 shoulder.
Increasing servo 5 lowers the shoulder angle; increasing 4 bends the elbow more;
increasing 3 raises wrist pitch. The look action turns the whole arm and wrist camera,
not just the camera: 1500 is forward, 2500 is left, 500 is right relative to the base.
No grasp/lift macro is available: visually choose each joint command. Use small
adjustments near the object; do not repeatedly drive through it. Position open jaws
around the visible beam before closing, then visually verify capture and lift.
Identify yourself by relating OWN_VIEW to OVERHEAD; do not assign yourself to a
particular overhead robot solely from your name. Choose wait if already holding.
'''
    return text


def _jpeg_data_uri(value: bytes, name: str) -> str:
    if not isinstance(value, bytes):
        raise ValueError(f"{name} must be bytes")
    if len(value) < 4 or not value.startswith(b"\xff\xd8") or not value.endswith(b"\xff\xd9"):
        raise ValueError(f"{name} must be a JPEG")
    return "data:image/jpeg;base64," + base64.b64encode(value).decode("ascii")


def _number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("action number must be finite")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("action number must be finite")
    return result


def _validate_action(action: Any) -> dict[str, Any]:
    if not isinstance(action, dict) or action.get("kind") not in _ACTION_FIELDS:
        raise ValueError("invalid action")
    kind = action["kind"]
    if set(action) != _ACTION_FIELDS[kind]:
        raise ValueError("invalid action fields")
    if kind == "drive":
        forward = _number(action["forward"])
        turn = _number(action["turn"])
        duration = _number(action["duration_s"])
        if not 0 <= forward <= 0.15 or not -0.2 <= turn <= 0.2 or not 0 <= duration <= 1:
            raise ValueError("action out of bounds")
    elif kind == "look":
        pulse = action["pan_pulse"]
        if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
            raise ValueError("action out of bounds")
    elif kind == "arm":
        servo, pulse = action["servo_id"], action["pulse"]
        if isinstance(servo, bool) or not isinstance(servo, int) or servo not in {1, 3, 4, 5}:
            raise ValueError("action out of bounds")
        if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
            raise ValueError("action out of bounds")
    return copy.deepcopy(action)


class CameraPairPlanner:
    """Ask a completer using only this robot's current view and the overhead view."""

    def __init__(self, robot_id: str, completer: Any, task: str = 'carry') -> None:
        if robot_id not in _ROBOTS:
            raise ValueError("robot_id must be r1 or r3")
        self.robot_id = robot_id
        self.completer = completer
        self.task = task
        static_task(robot_id, task)
        self.last_request: dict[str, Any] | None = None
        self.last_response: Any = None

    def decide(self, own_jpeg: bytes, overhead_jpeg: bytes) -> dict[str, Any]:
        messages = [
            {"role": "system", "content": static_task(self.robot_id, self.task)},
            {"role": "user", "content": _USER_TEXT},
        ]
        images = [
            {"label": "OWN_VIEW", "image": _jpeg_data_uri(own_jpeg, "own_jpeg")},
            {"label": "OVERHEAD", "image": _jpeg_data_uri(overhead_jpeg, "overhead_jpeg")},
        ]
        self.last_request = copy.deepcopy({"messages": messages, "images": images})
        raw = self.completer.complete(messages, images=images)
        self.last_response = raw
        if not isinstance(raw, str):
            raise ValueError("response must be JSON text")
        text = raw.strip()
        fenced = re.fullmatch(r"```(?:json)?\s*\n([\s\S]*?)\n```", text, re.IGNORECASE)
        if fenced:
            text = fenced.group(1).strip()
        try:
            response = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError("response must be JSON text") from exc
        if not isinstance(response, dict) or set(response) != {"reason", "action"}:
            raise ValueError("invalid response fields")
        reason = response["reason"]
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 500:
            raise ValueError("invalid reason")
        return _validate_action(response["action"])
