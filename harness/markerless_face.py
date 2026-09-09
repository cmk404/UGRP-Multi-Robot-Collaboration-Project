"""Marker-free face-normal selection from bounded own-frame evidence."""
from __future__ import annotations

import math
from collections import deque
from collections.abc import Mapping, Sequence
from typing import Any


_MIN_IOU = 0.70
_REQUIRED_STABLE_FITS = 3
_MAX_MOD90_SPREAD_RAD = math.radians(10.0)
_PERIOD = math.pi / 2.0


def _unit(angle: float) -> tuple[float, float]:
    return (math.cos(angle), math.sin(angle))


def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _mod90_distance(a: float, b: float) -> float:
    delta = (a - b + _PERIOD / 2.0) % _PERIOD - _PERIOD / 2.0
    return abs(delta)


def _finite_xy(value: Any) -> tuple[float, float] | None:
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or len(value) < 2):
        return None
    x, y = value[0], value[1]
    if (isinstance(x, bool) or isinstance(y, bool)
            or not isinstance(x, (int, float)) or not isinstance(y, (int, float))
            or not math.isfinite(float(x)) or not math.isfinite(float(y))):
        return None
    return float(x), float(y)


class MarkerlessFaceAligner:
    """Select a visible cuboid face only after three consistent RGB yaw fits.

    Angles and target positions are robot-own-frame estimates supplied by the
    markerless detector. The helper does not consume images, IDs, routes, or
    simulator state and does not command motion.
    """

    def __init__(self) -> None:
        self._fits: deque[float] = deque(maxlen=_REQUIRED_STABLE_FITS)
        self._previous_normal: tuple[float, float] | None = None

    def observe(self, box: Mapping[str, Any], target_xy: Sequence[float]) -> dict[str, Any]:
        base = {"ready": False, "normal_xy": None, "reason": "", "evidence": {}}
        if not isinstance(box, Mapping):
            self._fits.clear()
            return {**base, "reason": "INVALID_BOX_OBSERVATION"}
        target = _finite_xy(target_xy)
        if target is None or math.hypot(*target) <= 1e-9:
            self._fits.clear()
            return {**base, "reason": "INVALID_TARGET_XY"}
        yaw = box.get("estimated_yaw_mod_pi_rad")
        iou = box.get("floor_hypothesis_projection_iou")
        if (box.get("visible") is not True or isinstance(yaw, bool)
                or not isinstance(yaw, (int, float)) or not math.isfinite(float(yaw))
                or isinstance(iou, bool) or not isinstance(iou, (int, float))
                or not math.isfinite(float(iou)) or float(iou) < _MIN_IOU):
            # A missing or weak intervening frame breaks local temporal
            # continuity; never combine accepted fits across that gap.
            self._fits.clear()
            return {**base, "reason": "BOX_YAW_EVIDENCE_NOT_ACCEPTED",
                    "evidence": {"accepted_fit_count": 0,
                                 "minimum_projection_iou": _MIN_IOU}}

        yaw = float(yaw) % math.pi
        self._fits.append(yaw)
        spread = max((_mod90_distance(a, b) for a in self._fits for b in self._fits),
                     default=0.0)
        evidence = {
            "accepted_fit_count": len(self._fits),
            "required_fit_count": _REQUIRED_STABLE_FITS,
            "projection_iou": float(iou),
            "minimum_projection_iou": _MIN_IOU,
            "yaw_mod_90_rad": yaw % _PERIOD,
            "max_pairwise_yaw_mod_90_deg": math.degrees(spread),
            "maximum_allowed_yaw_mod_90_deg": math.degrees(_MAX_MOD90_SPREAD_RAD),
        }
        if len(self._fits) < _REQUIRED_STABLE_FITS:
            return {**base, "reason": "WAITING_FOR_THREE_STABLE_YAW_FITS", "evidence": evidence}
        if spread > _MAX_MOD90_SPREAD_RAD + 1e-9:
            return {**base, "reason": "UNSTABLE_YAW_FITS", "evidence": evidence}

        candidates = [_unit(yaw + k * _PERIOD) for k in range(4)]
        if self._previous_normal is None:
            toward_chassis = (-target[0] / math.hypot(*target),
                              -target[1] / math.hypot(*target))
            selected = max(candidates, key=lambda normal: _dot(normal, toward_chassis))
            selection = "outward_face_toward_chassis"
        else:
            selected = max(candidates, key=lambda normal: _dot(normal, self._previous_normal))
            selection = "current_own_frame_candidate_nearest_previous_normal"
        self._previous_normal = selected
        evidence["selection"] = selection
        return {"ready": True, "normal_xy": [selected[0], selected[1]],
                "reason": "STABLE_MARKERLESS_FACE_NORMAL", "evidence": evidence}


__all__ = ["MarkerlessFaceAligner"]
