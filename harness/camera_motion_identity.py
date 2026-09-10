"""Infer a bounded overhead motion anchor from consecutive camera pixels."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def _decode_jpeg(value: bytes) -> np.ndarray:
    if not isinstance(value, bytes):
        raise ValueError("top_jpeg must be bytes")
    if len(value) < 4 or not value.startswith(b"\xff\xd8") or not value.endswith(b"\xff\xd9"):
        raise ValueError("top_jpeg must be a JPEG")
    frame = cv2.imdecode(np.frombuffer(value, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.ndim != 3 or frame.shape[0] < 2 or frame.shape[1] < 2:
        raise ValueError("top_jpeg must decode to a color image")
    return frame


def _meaningful(action: Any) -> bool:
    if not isinstance(action, dict):
        return False
    kind = action.get("kind")
    if kind in {"arm", "look"}:
        return True
    if kind != "drive":
        return False
    forward, turn, duration = action.get("forward"), action.get("turn"), action.get("duration_s")
    numbers = (forward, turn, duration)
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in numbers):
        return False
    return duration > 0 and (forward != 0 or turn != 0)


class ImageMotionIdentity:
    """Track the compact image-difference region after this robot's own command.

    The returned center is a motion anchor in image coordinates, not a robot body
    center.  No simulator, peer, pose, or actuator state is consumed.
    """

    def __init__(self) -> None:
        self._previous: np.ndarray | None = None
        self._anchor: list[float] | None = None
        self._age = 0

    def _result(self, *, fresh: bool, changed: int, valid: bool, reason: str) -> dict[str, Any]:
        return {
            "center": None if self._anchor is None else list(self._anchor),
            "fresh": fresh,
            "age": self._age,
            "changed_pixels": changed,
            "valid": valid,
            "reason": reason,
        }

    def _age_anchor(self) -> None:
        if self._anchor is None:
            self._age = 0
            return
        self._age += 1
        if self._age > 6:
            self._anchor = None

    def update(self, top_jpeg: bytes, previous_action: dict | None) -> dict[str, Any]:
        current = _decode_jpeg(top_jpeg)
        previous = self._previous
        if previous is not None and previous.shape != current.shape:
            self._previous = current
            self._age_anchor()
            return self._result(fresh=False, changed=0, valid=False, reason="frame size changed")
        self._previous = current
        if previous is None:
            return self._result(fresh=False, changed=0, valid=False, reason="no consecutive frame")

        color_delta = cv2.absdiff(current, previous).max(axis=2)
        previous_gray = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
        current_gray = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)
        gray_delta = cv2.absdiff(current_gray, previous_gray)
        changed_mask = np.maximum(color_delta, gray_delta) > 20
        ys, xs = np.nonzero(changed_mask)
        changed = int(xs.size)

        if not _meaningful(previous_action):
            self._age_anchor()
            retained = self._anchor is not None
            reason = "retained motion anchor; no meaningful prior own action" if retained else "no meaningful prior own action"
            return self._result(fresh=False, changed=changed, valid=retained, reason=reason)
        if changed < 20:
            self._age_anchor()
            retained = self._anchor is not None
            reason = "retained motion anchor; too few changed pixels" if retained else "too few changed pixels"
            return self._result(fresh=False, changed=changed, valid=retained, reason=reason)

        height, width = current.shape[:2]
        normalized_x = xs.astype(np.float64) / (width - 1)
        normalized_y = ys.astype(np.float64) / (height - 1)
        center = np.array([np.median(normalized_x), np.median(normalized_y)])
        distances = np.sqrt((normalized_x - center[0]) ** 2 + (normalized_y - center[1]) ** 2)
        compact_fraction = float(np.count_nonzero(distances <= 0.10)) / changed
        if compact_fraction < 0.80:
            self._age_anchor()
            return self._result(fresh=False, changed=changed, valid=False, reason="motion is broad or has multiple regions")

        candidate = [float(center[0]), float(center[1])]
        if self._anchor is not None:
            jump = float(np.hypot(candidate[0] - self._anchor[0], candidate[1] - self._anchor[1]))
            if jump > 0.15:
                self._age_anchor()
                return self._result(fresh=False, changed=changed, valid=False, reason="motion anchor jump is ambiguous")

        self._anchor = candidate
        self._age = 0
        return self._result(fresh=True, changed=changed, valid=True, reason="compact motion follows prior own action")
