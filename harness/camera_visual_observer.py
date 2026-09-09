"""Pixel-only temporal observer for the cooperative camera grasp task."""

from __future__ import annotations

import base64
import copy
import json
import math
import re
from typing import Any

import cv2
import numpy as np
from harness.camera_motion_identity import ImageMotionIdentity


_ROBOTS = {"r1", "r3"}
_FIELDS = {
    "view", "jaws", "target", "confidence", "identity_confidence",
    "self_center", "capture_visible", "lift_visible", "suggested_action", "reason",
}
_ACTION_FIELDS = {
    "drive": {"kind", "forward", "turn", "duration_s"},
    "look": {"kind", "pan_pulse"},
    "arm": {"kind", "servo_id", "pulse"},
    "wait": {"kind"},
}

_SYSTEM = """You are a visual observer for robot {robot_id} during cooperative grasping.
Use only visible pixels in the supplied JPEG camera frames and the bounded list of
raw commands previously issued to this robot. You receive CURRENT_OWN_VIEW (this
robot's wrist camera) and CURRENT_OVERHEAD_VIEW, and sometimes the immediately
previous matching pair. There is no world state, simulator state, pose, contact,
depth, actuator feedback, or command-success signal. Never derive or invent any.

Identify this robot in the overhead frames by matching its own-camera visual change
to overhead motion and by using PRIOR_OWN_ISSUED_ACTIONS as a hypothesis about this
robot's commanded self-motion. Explicitly distinguish camera/arm motion, base motion,
and peer motion. Never assign overhead identity from the r1/r3 name or a customary
location. In overhead view, self_center is the normalized visible-pixel center of the
identified robot and is intended for root-motion tracking; return null if identity
or visibility is insufficient.
Track the SAME beam grasp point across consecutive frames when possible; do not
switch to a different convenient point to make the alignment error look smaller.

Return exactly one JSON object with these keys:
{"view":"own|overhead","jaws":[[x,y],[x,y]]|null,"target":[x,y]|null,
"confidence":0..1,"identity_confidence":0..1,"self_center":[x,y]|null,
"capture_visible":false,"lift_visible":false,"suggested_action":{"kind":"wait"},
"reason":"..."}
All points are normalized image coordinates from visible pixels only. jaws are the
two visible jaw-tip points. target is a chosen grasp point on the plain orange beam
to approach, even when it is currently far outside the jaws. jaws and target must use the single image named by view; return
target null when that beam point is not visible in that same image. Return jaws null
when both jaw tips are not visibly localizable; do not estimate hidden landmarks.
Set capture_visible true only with visible evidence that the jaws enclose the beam.
Set lift_visible true only when actual visible separation from the supporting surface
is present. Do not infer capture or lift from an issued command. Explain frame-to-frame
reasoning, including camera motion versus peer motion, in a nonempty reason of at most
1000 characters.

suggested_action is only a visual proposal for target approach/search, or for lifting
when capture is visibly established. The controller applies its own phase gates and
limits before any execution; never assume the proposal executes. Choose one small raw
action: wait; drive with forward -0.05..0.05, turn -0.1..0.1, duration_s 0..0.4;
look with pan_pulse 500..2500; or arm with servo_id 1|3|4|5 and pulse 500..2500.
An arm servo's absolute first target is allowed. After a prior command to the same
servo_id, keep its target change within 100 pulse units. Prefer reversible adjustments
and retain null landmarks whenever pixels do not support them. Return JSON only."""

_SYSTEM += """
Fixed hardware command documentation, not measured state: servo 1 opens at 2000
and closes at 1500; servo 3 is wrist pitch, 4 elbow, 5 shoulder. Increasing 5 lowers
the shoulder angle, increasing 4 bends the elbow, increasing 3 raises wrist pitch.
look turns the whole arm AND wrist camera: 1500 forward, 2500 left, 500 right.
These are command conventions, not a known current pose or a grasp macro.
The caller isolates one robot's command at a time. PIXEL_MOTION_CUE is computed
only from consecutive overhead images after this robot's issued command. A valid
center denotes a compact moving image region, NOT a world coordinate or exact
robot center. Use it to reject a contradictory overhead identity. A supplementary
CURRENT_OVERHEAD_CROP, when present, is a crop of the SAME current overhead JPEG.
Its original normalized bounds are supplied. Return ALL points in the original
uncropped view coordinates, never crop coordinates. Prefer overhead for jaws if
they are visible there but absent from the wrist view. Never invent occluded tips.
"""


def _jpeg_uri(value: bytes, name: str) -> str:
    if not isinstance(value, bytes):
        raise ValueError(f"{name} must be bytes")
    if len(value) < 4 or not value.startswith(b"\xff\xd8") or not value.endswith(b"\xff\xd9"):
        raise ValueError(f"{name} must be a JPEG")
    return "data:image/jpeg;base64," + base64.b64encode(value).decode("ascii")


def _unit_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} must be between 0 and 1")
    return result


def _point(value: Any, name: str) -> list[float] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must be a normalized point or null")
    return [_unit_number(value[0], name), _unit_number(value[1], name)]


def _suggested_action(value: Any, issued_actions: list[Any]) -> dict[str, Any]:
    if not isinstance(value, dict) or value.get("kind") not in _ACTION_FIELDS:
        raise ValueError("invalid suggested_action")
    kind = value["kind"]
    if set(value) != _ACTION_FIELDS[kind]:
        raise ValueError("invalid suggested_action fields")
    result = copy.deepcopy(value)
    if kind == "drive":
        forward = _finite_number(value["forward"], "forward")
        turn = _finite_number(value["turn"], "turn")
        duration = _finite_number(value["duration_s"], "duration_s")
        if not -0.05 <= forward <= 0.05 or not -0.1 <= turn <= 0.1 or not 0 <= duration <= 0.4:
            raise ValueError("suggested drive out of bounds")
    elif kind == "look":
        pulse = value["pan_pulse"]
        if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
            raise ValueError("suggested look out of bounds")
    elif kind == "arm":
        servo, pulse = value["servo_id"], value["pulse"]
        if isinstance(servo, bool) or not isinstance(servo, int) or servo not in {1, 3, 4, 5}:
            raise ValueError("suggested arm out of bounds")
        if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
            raise ValueError("suggested arm out of bounds")
        prior = next((a for a in reversed(issued_actions)
                      if isinstance(a, dict) and a.get("kind") == "arm"
                      and a.get("servo_id") == servo and isinstance(a.get("pulse"), int)
                      and not isinstance(a.get("pulse"), bool)), None)
        if servo != 1 and prior is not None and abs(pulse - prior["pulse"]) > 100:
            raise ValueError("suggested arm target changes by more than 100")
    return result


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _validate_observation(value: Any, issued_actions: list[Any]) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != _FIELDS:
        raise ValueError("invalid observation fields")
    if value["view"] not in {"own", "overhead"}:
        raise ValueError("invalid observation view")
    jaws = value["jaws"]
    if jaws is not None:
        if not isinstance(jaws, list) or len(jaws) != 2:
            raise ValueError("jaws must contain exactly two points or be null")
        jaws = [_point(jaws[0], "jaw point"), _point(jaws[1], "jaw point")]
        if any(point is None for point in jaws):
            raise ValueError("jaw points cannot be null")
    reason = value["reason"]
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 1000:
        raise ValueError("invalid reason")
    for name in ("capture_visible", "lift_visible"):
        if not isinstance(value[name], bool):
            raise ValueError(f"{name} must be boolean")
    target = _point(value["target"], "target")
    if value["capture_visible"] and (jaws is None or target is None):
        raise ValueError("visible capture requires visible jaws and target")
    return {
        "view": value["view"],
        "jaws": jaws,
        "target": target,
        "confidence": _unit_number(value["confidence"], "confidence"),
        "identity_confidence": _unit_number(value["identity_confidence"], "identity_confidence"),
        "self_center": _point(value["self_center"], "self_center"),
        "capture_visible": value["capture_visible"],
        "lift_visible": value["lift_visible"],
        "suggested_action": _suggested_action(value["suggested_action"], issued_actions),
        "reason": reason,
    }


def parse_observation(raw: str) -> dict[str, Any]:
    """Parse an observer reply without executing or otherwise interpreting actions."""
    if not isinstance(raw, str):
        raise ValueError("response must be JSON text")
    text = raw.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*\n([\s\S]*?)\n```", text, re.IGNORECASE)
    if fenced:
        text = fenced.group(1).strip()
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError("response must be JSON text") from exc
    return _validate_observation(parsed, [])


class CameraVisualObserver:
    """Obtain a strictly validated observation from two temporal camera views."""

    def __init__(self, robot_id: str, completer: Any) -> None:
        if robot_id not in _ROBOTS:
            raise ValueError("robot_id must be r1 or r3")
        self.robot_id = robot_id
        self.completer = completer
        self.last_request: dict[str, Any] | None = None
        self.last_response: Any = None
        self._previous_images: tuple[bytes, bytes] | None = None
        self._motion = ImageMotionIdentity()
        self.last_motion_cue = None

    def prepare_request(
        self, own_jpeg: bytes, overhead_jpeg: bytes, issued_actions: list[Any]
    ) -> dict[str, Any]:
        own_uri = _jpeg_uri(own_jpeg, "own_jpeg")
        overhead_uri = _jpeg_uri(overhead_jpeg, "overhead_jpeg")
        if not isinstance(issued_actions, list):
            raise ValueError("issued_actions must be a list")
        try:
            actions = copy.deepcopy(issued_actions[-8:])
            for action in actions:
                _suggested_action(action, [])
            action_text = json.dumps(actions, separators=(",", ":"), allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError("issued_actions must be JSON-compatible raw actions") from exc

        images = [
            {"label": "CURRENT_OWN_VIEW", "image": own_uri},
            {"label": "CURRENT_OVERHEAD_VIEW", "image": overhead_uri},
        ]
        if self._previous_images is not None:
            images.extend([
                {"label": "PREVIOUS_OWN_VIEW", "image": _jpeg_uri(self._previous_images[0], "previous_own_jpeg")},
                {"label": "PREVIOUS_OVERHEAD_VIEW", "image": _jpeg_uri(self._previous_images[1], "previous_overhead_jpeg")},
            ])
        cue = self._motion.update(overhead_jpeg, actions[-1] if actions else None)
        self.last_motion_cue = cue
        crop_bounds = None
        if cue['valid'] and cue['center'] is not None:
            decoded = cv2.imdecode(np.frombuffer(overhead_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            height, width = decoded.shape[:2]
            x, y = cue['center']
            x0, x1 = max(0, int((x-.22)*width)), min(width, int((x+.22)*width))
            y0, y1 = max(0, int((y-.18)*height)), min(height, int((y+.18)*height))
            okay, encoded = cv2.imencode('.jpg', decoded[y0:y1, x0:x1], [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not okay:
                raise ValueError('overhead crop encoding failed')
            crop_bounds = [x0/width, y0/height, x1/width, y1/height]
            images.append({'label': 'CURRENT_OVERHEAD_CROP', 'image': _jpeg_uri(encoded.tobytes(), 'crop')})
        messages = [
            {"role": "system", "content": _SYSTEM.replace("{robot_id}", self.robot_id)},
            {"role": "user", "content": "Inspect the frames. PRIOR_OWN_ISSUED_ACTIONS (oldest to newest, maximum 8): " + action_text
             + '\nPIXEL_MOTION_CUE: ' + json.dumps(cue, sort_keys=True)
             + '\nCURRENT_OVERHEAD_CROP original normalized bounds: ' + json.dumps(crop_bounds)},
        ]
        request = {"messages": messages, "images": images}
        self.last_request = copy.deepcopy(request)
        self._previous_images = (bytes(own_jpeg), bytes(overhead_jpeg))
        return copy.deepcopy(request)

    def observe(
        self, own_jpeg: bytes, overhead_jpeg: bytes, issued_actions: list[Any]
    ) -> dict[str, Any]:
        request = self.prepare_request(own_jpeg, overhead_jpeg, issued_actions)
        raw = self.completer.complete(request["messages"], images=request["images"])
        self.last_response = raw
        return self.validate_response(raw, issued_actions)

    def validate_response(self, raw, issued_actions):
        parsed = _validate_observation(parse_observation(raw), issued_actions[-8:])
        cue = self.last_motion_cue
        center = parsed['self_center']
        consistent = bool(cue and cue['valid'] and center is not None
                          and math.dist(center, cue['center']) <= .12)
        # Semantic confidence cannot override contradictory/no motion evidence.
        if not consistent:
            parsed['identity_confidence'] = 0.0
        parsed['motion_identity'] = {'consistent': consistent, 'cue': copy.deepcopy(cue)}
        return parsed
