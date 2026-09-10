"""Pixel-only temporal landmark tracking; no simulator inputs."""

from __future__ import annotations

import copy
from typing import Any

import cv2
import numpy as np


def _decode(value: bytes, name: str) -> np.ndarray:
    if not isinstance(value, bytes) or len(value) < 4 or not value.startswith(b"\xff\xd8") or not value.endswith(b"\xff\xd9"):
        raise ValueError(f"{name} must be JPEG bytes")
    frame = cv2.imdecode(np.frombuffer(value, np.uint8), cv2.IMREAD_COLOR)
    if frame is None or min(frame.shape[:2]) < 8:
        raise ValueError(f"{name} must decode to an image")
    return frame


def _points(observation: dict[str, Any]) -> tuple[list[list[float]], list[str]] | None:
    jaws, target = observation.get("jaws"), observation.get("target")
    if jaws is None or target is None or not isinstance(jaws, list) or len(jaws) != 2:
        return None
    values = [jaws[0], jaws[1], target]
    for point in values:
        if (not isinstance(point, list) or len(point) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 <= v <= 1 for v in point)):
            return None
    return copy.deepcopy(values), ["jaw0", "jaw1", "target"]


def _trusted(observation: dict[str, Any]) -> bool:
    confidence = observation.get("confidence")
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or confidence < 0.7:
        return False
    view = observation.get("view")
    if view not in {"own", "overhead"}:
        return False
    if view == "overhead":
        identity = observation.get("identity_confidence")
        if isinstance(identity, bool) or not isinstance(identity, (int, float)) or identity < 0.8:
            return False
    return _points(observation) is not None


def _patch(gray: np.ndarray, point: np.ndarray, radius: int = 4) -> np.ndarray | None:
    x, y = np.rint(point).astype(int)
    if x - radius < 0 or y - radius < 0 or x + radius >= gray.shape[1] or y + radius >= gray.shape[0]:
        return None
    return gray[y-radius:y+radius+1, x-radius:x+radius+1]


class CameraLandmarkTracker:
    """Bridge trusted, visible same-view landmarks for at most three frames."""

    def __init__(self) -> None:
        self._frames: dict[str, np.ndarray | None] = {"own": None, "overhead": None}
        self._tracks: dict[str, dict[str, Any] | None] = {"own": None, "overhead": None}
        self._last_view: str | None = None

    @staticmethod
    def _pixels(points: list[list[float]], shape: tuple[int, ...]) -> np.ndarray:
        h, w = shape[:2]
        return np.array([[[p[0] * (w - 1), p[1] * (h - 1)]] for p in points], dtype=np.float32)

    @staticmethod
    def _normalized(points: np.ndarray, shape: tuple[int, ...]) -> list[list[float]]:
        h, w = shape[:2]
        return [[float(p[0] / (w - 1)), float(p[1] / (h - 1))] for p in points.reshape(-1, 2)]

    def _flow(self, old: np.ndarray, new: np.ndarray, normalized: list[list[float]]) -> list[list[float]] | None:
        if old.shape != new.shape:
            return None
        old_gray, new_gray = cv2.cvtColor(old, cv2.COLOR_BGR2GRAY), cv2.cvtColor(new, cv2.COLOR_BGR2GRAY)
        p0 = self._pixels(normalized, old.shape)
        params = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
                      flags=cv2.OPTFLOW_LK_GET_MIN_EIGENVALS, minEigThreshold=1e-4)
        p1, status1, eig = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **params)
        if p1 is None or status1 is None or eig is None:
            return None
        back_params = dict(params)
        back_params["flags"] = 0
        p0_back, status2, _ = cv2.calcOpticalFlowPyrLK(new_gray, old_gray, p1, None, **back_params)
        if p0_back is None or status2 is None:
            return None
        fb = np.linalg.norm(p0_back.reshape(-1, 2) - p0.reshape(-1, 2), axis=1)
        if not np.all(status1.reshape(-1) == 1) or not np.all(status2.reshape(-1) == 1):
            return None
        if np.any(eig.reshape(-1) < 1e-4) or np.any(fb > 1.5):
            return None
        flat0, flat1 = p0.reshape(-1, 2), p1.reshape(-1, 2)
        h, w = new.shape[:2]
        if np.any(flat1[:, 0] < 0) or np.any(flat1[:, 0] > w - 1) or np.any(flat1[:, 1] < 0) or np.any(flat1[:, 1] > h - 1):
            return None
        for before, after in zip(flat0, flat1):
            a, b = _patch(old_gray, before), _patch(new_gray, after)
            if a is None or b is None or float(np.mean(cv2.absdiff(a, b))) > 30.0:
                return None
        return self._normalized(p1, new.shape)

    @staticmethod
    def _agrees(model: list[list[float]], flowed: list[list[float]], shape: tuple[int, ...]) -> bool:
        h, w = shape[:2]
        for actual, predicted in zip(model, flowed):
            distance = np.hypot((actual[0] - predicted[0]) * (w - 1), (actual[1] - predicted[1]) * (h - 1))
            if distance > 8.0:
                return False
        return True

    def update(self, own_jpeg: bytes, top_jpeg: bytes, observation: dict) -> dict:
        if not isinstance(observation, dict):
            raise ValueError("observation must be a dict")
        frames = {"own": _decode(own_jpeg, "own_jpeg"), "overhead": _decode(top_jpeg, "top_jpeg")}
        result = copy.deepcopy(observation)
        view = result.get("view")
        if view not in frames:
            result["landmark_tracking"] = {"source": "unavailable", "age": 0, "confidence": 0.0, "reason": "invalid view"}
            self._frames = frames
            return result

        if self._last_view is not None and view != self._last_view:
            # That track was not propagated through the intervening frame pair.
            self._tracks[self._last_view] = None
        self._last_view = view

        track = self._tracks[view]
        old = self._frames[view]
        flowed = None if track is None or old is None else self._flow(old, frames[view], track["points"])
        model_points = _points(result)
        model_values = None if model_points is None else model_points[0]
        current_trusted = _trusted(result)
        model_featured = (current_trusted
                          and self._flow(frames[view], frames[view], model_values) is not None)
        overhead_identity_ok = (view != "overhead" or (
            isinstance(result.get("identity_confidence"), (int, float))
            and not isinstance(result.get("identity_confidence"), bool)
            and result["identity_confidence"] >= 0.8))

        if model_featured and flowed is not None and self._agrees(model_values, flowed, frames[view].shape):
            fused = [[0.25 * model[0] + 0.75 * flow[0], 0.25 * model[1] + 0.75 * flow[1]]
                     for model, flow in zip(model_values, flowed)]
            self._tracks[view] = {"points": fused, "age": 0, "seed_confidence": float(result["confidence"])}
            result["jaws"] = copy.deepcopy(fused[:2])
            result["target"] = copy.deepcopy(fused[2])
            source, age = "fused", 0
            track_confidence = float(result["confidence"])
            reason = "trusted model landmarks fused with agreeing verified flow"
        elif model_featured and flowed is None:
            self._tracks[view] = {"points": model_values, "age": 0, "seed_confidence": float(result["confidence"])}
            source, age = "model", 0
            track_confidence = float(result["confidence"])
            reason = "trusted textured current model landmarks"
        elif flowed is not None and track is not None and track["age"] < 3 and overhead_identity_ok:
            age = track["age"] + 1
            seed_confidence = track["seed_confidence"]
            track_confidence = min(seed_confidence, seed_confidence - 0.03 * age)
            self._tracks[view] = {"points": flowed, "age": age, "seed_confidence": seed_confidence}
            result["jaws"] = copy.deepcopy(flowed[:2])
            result["target"] = copy.deepcopy(flowed[2])
            result["confidence"] = track_confidence
            # Flow supplies geometry only and is never physical success evidence.
            result["capture_visible"] = False
            result["lift_visible"] = False
            source = "optical_flow"
            reason = "current landmarks missing, low-trust, or jump over 8px; retained verified flow"
        else:
            self._tracks[view] = None
            source, age, track_confidence = "unavailable", 0, 0.0
            result["jaws"] = None
            result["target"] = None
            result["capture_visible"] = False
            result["lift_visible"] = False
            reason = "no trusted visible same-view landmark bridge"

        result["landmark_tracking"] = {
            "source": source, "age": age, "confidence": track_confidence, "reason": reason,
        }
        self._frames = frames
        return result
