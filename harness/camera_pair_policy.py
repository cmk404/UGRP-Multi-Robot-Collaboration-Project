"""Stateless two-camera policy for the cooperative orange-beam task."""

from __future__ import annotations

import base64
import copy
import json
import math
import re
from typing import Any

from harness.camera_action_learning import CameraActionLearner, visual_features


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

_MODE_ORDER = {"baseline": 0, "memory": 1, "temporal": 2, "learned": 3}
_LEARNING_CANDIDATES = (
    {"kind": "wait"},
    {"kind": "drive", "forward": 0.05, "turn": 0.0, "duration_s": 0.3},
    {"kind": "drive", "forward": 0.0, "turn": -0.1, "duration_s": 0.3},
    {"kind": "drive", "forward": 0.0, "turn": 0.1, "duration_s": 0.3},
)


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

    def __init__(self, robot_id: str, completer: Any, task: str = 'carry', mode: str = "baseline") -> None:
        if robot_id not in _ROBOTS:
            raise ValueError("robot_id must be r1 or r3")
        self.robot_id = robot_id
        self.completer = completer
        self.task = task
        static_task(robot_id, task)
        if mode not in _MODE_ORDER:
            raise ValueError("mode must be baseline, memory, temporal, or learned")
        self.mode = mode
        self.last_request: dict[str, Any] | None = None
        self.last_response: Any = None
        self.last_learning_summary: dict[str, Any] | None = None
        self._actions: list[dict[str, Any]] = []
        self._previous_images: tuple[bytes, bytes] | None = None
        self._previous_features = None
        self._pending_action: dict[str, Any] | None = None
        self._record_open = False
        self._learner = CameraActionLearner()

    def prepare_request(self, own_jpeg: bytes, overhead_jpeg: bytes) -> dict[str, Any]:
        """Build the next request and advance replay state from the prior issued action.

        Auditors can reconstruct an offline sequence by alternating this method with
        :meth:`record_action`.  Calling ``prepare_request`` observes the pixel change
        since the preceding call; ``record_action`` associates only a validated issued
        command with that interval.  Neither method observes physical robot state.
        """
        own_uri = _jpeg_data_uri(own_jpeg, "own_jpeg")
        overhead_uri = _jpeg_data_uri(overhead_jpeg, "overhead_jpeg")
        level = _MODE_ORDER[self.mode]
        current_features = visual_features(own_jpeg, overhead_jpeg) if level >= 3 else None
        if level >= 3 and self._pending_action is not None and self._previous_features is not None:
            self._learner.observe(self._pending_action, self._previous_features, current_features)
        self._pending_action = None

        system = static_task(self.robot_id, self.task)
        user = _USER_TEXT
        if level >= 1:
            system = system.replace(
                "memory,\nhistory, peer status, messages, action budget, or runtime feedback.",
                "peer status, messages, action budget, measured actuator state, or non-camera runtime feedback.",
            )
            system += "\nYou may use the bounded history of your own issued commands below. It is command history, not measured actuator state or proof of execution."
            user += "\nOWN_ISSUED_ACTIONS (oldest to newest, maximum 4): " + json.dumps(self._actions, separators=(",", ":"))
        images = [
            {"label": "OWN_VIEW", "image": own_uri},
            {"label": "OVERHEAD", "image": overhead_uri},
        ]
        if level >= 2 and self._previous_images is not None:
            system = system.replace(
                f"exactly two current RGB images: OWN_VIEW is {self.robot_id}'s\nwrist robot_cam, and OVERHEAD is cctv_top.",
                f"the current and immediately previous RGB image pairs: OWN_VIEW is {self.robot_id}'s\nwrist robot_cam, and OVERHEAD is cctv_top.",
            )
            system = system.replace("from the current images.", "from the provided images.")
            system += "\nPrevious and current image pairs are time ordered camera samples only; scene changes may include peer motion and do not prove your command executed."
            images.extend([
                {"label": "PREVIOUS_OWN_VIEW", "image": _jpeg_data_uri(self._previous_images[0], "previous_own_jpeg")},
                {"label": "PREVIOUS_OVERHEAD", "image": _jpeg_data_uri(self._previous_images[1], "previous_overhead_jpeg")},
            ])
        if level >= 3:
            candidates = list(_LEARNING_CANDIDATES)
            for issued in self._actions:
                if issued not in candidates:
                    candidates.append(copy.deepcopy(issued))
            self.last_learning_summary = self._learner.summarize(candidates[-8:])
            system += "\nLEARNED_VISUAL_EFFECT is a bounded ridge estimate from RGB feature changes after your issued commands. Treat sample support, training RMSE, and peer-motion confounding explicitly. Training RMSE is not calibrated uncertainty. It does not establish grasp or success and must not be auto-executed as a macro."
            user += "\nLEARNED_VISUAL_EFFECT: " + json.dumps(self.last_learning_summary, separators=(",", ":"))

        messages = [{"role": "system", "content": system}, {"role": "user", "content": user}]
        self.last_request = copy.deepcopy({"messages": messages, "images": images})
        self._previous_images = (bytes(own_jpeg), bytes(overhead_jpeg)) if level >= 2 else None
        self._previous_features = current_features
        self._record_open = True
        return copy.deepcopy(self.last_request)

    def record_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Record one validated command issued after the latest prepared request."""
        if not self._record_open:
            raise ValueError("prepare_request must precede record_action")
        validated = _validate_action(action)
        self._actions.append(validated)
        del self._actions[:-4]
        if _MODE_ORDER[self.mode] >= 3:
            self._pending_action = copy.deepcopy(validated)
        self._record_open = False
        return copy.deepcopy(validated)

    def decide(self, own_jpeg: bytes, overhead_jpeg: bytes) -> dict[str, Any]:
        request = self.prepare_request(own_jpeg, overhead_jpeg)
        messages, images = request["messages"], request["images"]
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
        action = _validate_action(response["action"])
        self.record_action(action)
        return action
