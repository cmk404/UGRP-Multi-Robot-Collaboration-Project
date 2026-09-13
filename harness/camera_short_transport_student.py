"""Small RGB-only predictor for the fixed short-transport demonstration domain."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness.camera_beam_features import extract_beams

FEATURE_NAMES = ("top_payload_x", "top_payload_y", "top_payload_area", "top_own_x", "top_own_y",
                 "own_orange_fraction", "own_orange_y")
REGRESSION_NAMES = ("bias", "payload_dx", "own_robot_dx")


def _decode(jpeg: bytes) -> np.ndarray | None:
    if not isinstance(jpeg, bytes) or not jpeg:
        return None
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


def _mask_stats(mask: np.ndarray) -> tuple[float, float, float] | None:
    ys, xs = np.nonzero(mask)
    if len(ys) == 0:
        return None
    return (float(len(ys) / mask.size), float(np.mean(xs) / mask.shape[1]),
            float(np.mean(ys) / mask.shape[0]))


def extract_transport_features(own_jpeg: bytes, top_jpeg: bytes, robot_id: str) -> dict[str, Any]:
    """Extract normalized image measurements; no scene calibration or state is used."""
    if robot_id not in {"r1", "r3"}:
        return {"ok": False, "reason": "unknown_robot_id", "features": None}
    own, top = _decode(own_jpeg), _decode(top_jpeg)
    if own is None or top is None:
        return {"ok": False, "reason": "invalid_jpeg", "features": None}
    candidates = [item for item in extract_beams(top_jpeg)
                  if .30 <= float(item["center"][1]) <= .70 and not item["touches_border"]]
    if not candidates:
        return {"ok": False, "reason": "top_payload_not_found", "features": None}
    payload = min(candidates, key=lambda item: (abs(float(item["center"][1]) - .5),
                                                -float(item["length_px"])))
    own_hsv, top_hsv = cv2.cvtColor(own, cv2.COLOR_BGR2HSV), cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(own_hsv, np.array((3, 105, 45), np.uint8),
                         np.array((24, 255, 255), np.uint8)) > 0
    own_stats = _mask_stats(orange)
    yellow = cv2.inRange(top_hsv, np.array((20, 70, 50), np.uint8),
                         np.array((40, 255, 255), np.uint8)) > 0
    split = top.shape[0] // 2
    yellow[:split if robot_id == "r1" else 0] = False
    if robot_id == "r3":
        yellow[split:] = False
    yellow_stats = _mask_stats(yellow)
    if own_stats is None or yellow_stats is None:
        return {"ok": False, "reason": "own_visual_evidence_missing", "features": None}
    features = {
        "top_payload_x": float(payload["center"][0]),
        "top_payload_y": float(payload["center"][1]),
        "top_payload_area": float(payload["area_px"] / (top.shape[0] * top.shape[1])),
        "top_own_x": yellow_stats[1],
        "top_own_y": yellow_stats[2],
        "own_orange_fraction": own_stats[0],
        "own_orange_y": own_stats[2],
    }
    return {"ok": True, "reason": "rgb_features", "features": features,
            "diagnostics": {"top_candidate_count": len(candidates)}}


def _issued_drive(history: Sequence[Mapping[str, Any]]) -> float | None:
    total = 0.0
    for action in history:
        if not isinstance(action, Mapping) or action.get("kind") not in {"drive", "mecanum"}:
            return None
        forward, duration = action.get("forward"), action.get("duration_s")
        if (isinstance(forward, bool) or not isinstance(forward, (int, float))
                or isinstance(duration, bool) or not isinstance(duration, (int, float))
                or not math.isfinite(float(forward)) or not math.isfinite(float(duration))
                or not 0 <= float(duration) <= 1.0):
            return None
        total += float(forward) * float(duration)
    return total


def regression_vector(current: Mapping[str, float], anchor: Mapping[str, float],
                      issued_drive: float) -> list[float]:
    # Issued history is validated/audited, but cannot advance visual progress.
    # A stalled actuator with unchanged images must therefore predict unchanged progress.
    del issued_drive
    return [1.0, current["top_payload_x"] - anchor["top_payload_x"],
            current["top_own_x"] - anchor["top_own_x"]]


def _supported(model: Mapping[str, Any], features: Mapping[str, float]) -> bool:
    support = model.get("feature_support")
    if not isinstance(support, Mapping):
        return False
    for name in FEATURE_NAMES:
        bounds = support.get(name)
        if (not isinstance(bounds, list) or len(bounds) != 2
                or not all(not isinstance(v, bool) and isinstance(v, (int, float))
                           and math.isfinite(float(v)) for v in bounds)
                or float(bounds[0]) > float(bounds[1])
                or not float(bounds[0]) <= features[name] <= float(bounds[1])):
            return False
    return True


def _held_estimate(model: Mapping[str, Any], features: Mapping[str, float]) -> bool:
    positive, negative, scale = (model.get("held_positive_center"),
                                 model.get("held_negative_center"),
                                 model.get("held_feature_scale"))
    if all(isinstance(value, list) and len(value) == len(FEATURE_NAMES)
           for value in (positive, negative, scale)):
        if not all(not isinstance(item, bool) and isinstance(item, (int, float))
                   and math.isfinite(float(item))
                   for values in (positive, negative, scale) for item in values):
            return False
        vector = np.asarray([features[name] for name in FEATURE_NAMES], float)
        divisor = np.maximum(np.asarray(scale, float), 1e-6)
        d_positive = float(np.linalg.norm((vector - np.asarray(positive, float)) / divisor))
        d_negative = float(np.linalg.norm((vector - np.asarray(negative, float)) / divisor))
        return d_positive <= d_negative
    return features["own_orange_fraction"] >= float(model.get("held_own_orange_min", 1.0))


def predict_transport(model: Mapping[str, Any], own_jpeg: bytes, top_jpeg: bytes,
                      initial_own_jpeg: bytes, initial_top_jpeg: bytes,
                      own_command_history: Sequence[Mapping[str, Any]] = ()) -> dict[str, Any]:
    """Predict bounded motion from current/first-carry RGB and issued commands only."""
    bad = {"ok": False, "held_estimate": False, "ready": False, "forward": 0.0}
    if not isinstance(model, Mapping):
        return {**bad, "diagnostics": {"reason": "invalid_model"}}
    rid = model.get("robot_id")
    if (model.get("schema") != "ugrp.camera_short_transport_model.v1"
            or rid not in {"r1", "r3"}):
        return {**bad, "diagnostics": {"reason": "invalid_model"}}
    current = extract_transport_features(own_jpeg, top_jpeg, rid)
    anchor = extract_transport_features(initial_own_jpeg, initial_top_jpeg, rid)
    issued = _issued_drive(own_command_history)
    if not current["ok"] or not anchor["ok"] or issued is None:
        return {**bad, "diagnostics": {"reason": "invalid_input_features",
                "current": current.get("reason"), "anchor": anchor.get("reason")}}
    cf, af = current["features"], anchor["features"]
    if not _supported(model, cf) or not _supported(model, af):
        return {**bad, "diagnostics": {"reason": "rgb_outside_learned_support",
                                        "features": cf}}
    coefficients = model.get("progress_coefficients")
    if (not isinstance(coefficients, list) or len(coefficients) != len(REGRESSION_NAMES)
            or not all(not isinstance(value, bool) and isinstance(value, (int, float))
                       and math.isfinite(float(value)) for value in coefficients)):
        return {**bad, "diagnostics": {"reason": "invalid_model_coefficients"}}
    raw_progress = float(np.dot(np.asarray(coefficients, float), regression_vector(cf, af, issued)))
    if not math.isfinite(raw_progress):
        return {**bad, "diagnostics": {"reason": "nonfinite_prediction"}}
    progress = max(0.0, raw_progress)
    held = _held_estimate(model, cf)
    ready_at, slow_at = model.get("ready_progress_m"), model.get("slow_progress_m")
    if (not all(not isinstance(v, bool) and isinstance(v, (int, float))
                and math.isfinite(float(v)) for v in (ready_at, slow_at))
            or not 0.0 <= float(slow_at) < float(ready_at) <= .20):
        return {**bad, "diagnostics": {"reason": "invalid_model_thresholds"}}
    ready_at, slow_at = float(ready_at), float(slow_at)
    ready = bool(held and progress >= ready_at)
    if ready or not held:
        forward = 0.0
    elif progress <= slow_at:
        forward = .10
    else:
        forward = max(.02, .10 * max(0.0, min(1.0,
            (ready_at - progress) / max(.001, ready_at - slow_at))))
    return {"ok": True, "held_estimate": held, "ready": ready,
            "forward": float(max(0.0, min(.10, forward))),
            "diagnostics": {"reason": "learned_fixed_scene_rgb", "progress_m": progress,
                            "issued_drive": issued, "features": cf}}
