"""Small, auditable online model of command-associated RGB changes.

The learner intentionally models correlation, not robot state or action success.  Its
only inputs are validated commands and pixel summaries computed from consecutive
own/top JPEG pairs.  A peer may have moved between frames, so every prediction is
reported with sample count and residual uncertainty.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

import cv2
import numpy as np


_MAX_TRANSITIONS = 32
_RIDGE = 1.0


def visual_features(own_jpeg: bytes, top_jpeg: bytes) -> np.ndarray:
    """Return 24 RGB means: own then top, each 2x2 regions in row-major order."""
    values: list[float] = []
    for encoded in (own_jpeg, top_jpeg):
        bgr = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("JPEG pixels could not be decoded")
        rgb = cv2.cvtColor(cv2.resize(bgr, (16, 16), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        rgb = rgb.astype(np.float64) / 255.0
        for row in range(2):
            for column in range(2):
                region = rgb[row * 8:(row + 1) * 8, column * 8:(column + 1) * 8]
                values.extend(region.mean(axis=(0, 1)).tolist())
    return np.asarray(values, dtype=np.float64)


def action_features(action: dict[str, Any], previous: dict[str, Any] | None = None) -> np.ndarray:
    """Encode an issued command; these values are not measured actuator state."""
    vector = np.zeros(16, dtype=np.float64)
    kind = action["kind"]
    index = {"wait": 0, "drive": 1, "look": 2, "arm": 3}[kind]
    vector[index] = 1.0
    if kind == "drive":
        vector[4:7] = (action["forward"] / 0.15, action["turn"] / 0.2, action["duration_s"])
    elif kind == "look":
        vector[7] = (action["pan_pulse"] - 1500) / 1000
    elif kind == "arm":
        vector[8] = (action["servo_id"] - 3) / 2
        vector[9] = (action["pulse"] - 1500) / 1000
    # Change terms describe the change in *issued command values*, never measured
    # servo/base motion.  The first sample has an explicitly unknown baseline.
    if previous is None:
        vector[14] = 1.0
    elif kind == previous.get("kind") == "drive":
        vector[10:13] = (
            (action["forward"] - previous["forward"]) / 0.15,
            (action["turn"] - previous["turn"]) / 0.2,
            action["duration_s"] - previous["duration_s"],
        )
    elif kind == previous.get("kind") == "look":
        vector[13] = (action["pan_pulse"] - previous["pan_pulse"]) / 1000
    elif kind == previous.get("kind") == "arm" and action["servo_id"] == previous.get("servo_id"):
        vector[13] = (action["pulse"] - previous["pulse"]) / 1000
    vector[15] = 1.0
    return vector


@dataclass(frozen=True)
class Transition:
    command: dict[str, Any]
    command_features: tuple[float, ...]
    before: tuple[float, ...]
    after: tuple[float, ...]


class CameraActionLearner:
    """Bounded ridge predictor for pixel-feature deltas after issued commands."""

    def __init__(self, max_transitions: int = _MAX_TRANSITIONS) -> None:
        if not 1 <= max_transitions <= _MAX_TRANSITIONS:
            raise ValueError(f"max_transitions must be between 1 and {_MAX_TRANSITIONS}")
        self.max_transitions = max_transitions
        self.transitions: list[Transition] = []
        self._previous_by_channel: dict[tuple[Any, ...], dict[str, Any]] = {}

    @staticmethod
    def _channel(command: dict[str, Any]) -> tuple[Any, ...]:
        return (command["kind"], command.get("servo_id"))

    @staticmethod
    def _within_observed_range(candidate: dict[str, Any], observed: list[dict[str, Any]]) -> bool:
        fields = {
            "wait": (),
            "drive": ("forward", "turn", "duration_s"),
            "look": ("pan_pulse",),
            "arm": ("pulse",),
        }[candidate["kind"]]
        return all(
            min(float(item[field]) for item in observed) <= float(candidate[field]) <= max(float(item[field]) for item in observed)
            for field in fields
        )

    def observe(self, command: dict[str, Any], before: np.ndarray, after: np.ndarray) -> None:
        if before.shape != (24,) or after.shape != (24,):
            raise ValueError("visual feature vectors must have length 24")
        channel = self._channel(command)
        encoded = action_features(command, self._previous_by_channel.get(channel))
        self.transitions.append(Transition(dict(command), tuple(encoded), tuple(before), tuple(after)))
        self._previous_by_channel[channel] = dict(command)
        del self.transitions[:-self.max_transitions]

    def summarize(self, candidates: Iterable[dict[str, Any]]) -> dict[str, Any]:
        """Predict candidate deltas and expose uncertainty; output is JSON-safe."""
        n = len(self.transitions)
        candidate_list = [dict(item) for item in candidates]
        if not n:
            return {
                "sample_count": n,
                "minimum_samples_per_command_channel": 3,
                "feature_order": "own,top; each 2x2 row-major regions; RGB means",
                "supported_candidate_count": 0,
                "predictions": [{
                    "command": candidate,
                    "sample_count": 0,
                    "predicted_pixel_feature_delta": None,
                    "unsupported_reason": "fewer than 3 transitions for this command kind/servo channel",
                } for candidate in candidate_list],
                "warning": "No transitions yet; commands are issued, not measured state, and outcomes are confounded by peer/scene motion.",
            }
        x = np.stack([np.asarray(item.command_features) for item in self.transitions])
        y = np.stack([np.asarray(item.after) - np.asarray(item.before) for item in self.transitions])
        weights = np.linalg.solve(x.T @ x + _RIDGE * np.eye(x.shape[1]), x.T @ y)
        residual = y - x @ weights
        training_rmse = float(np.sqrt(np.mean(residual * residual)))
        predictions = []
        supported_candidate_count = 0
        for candidate in candidate_list:
            channel = self._channel(candidate)
            observed = [item.command for item in self.transitions if self._channel(item.command) == channel]
            support = len(observed)
            item: dict[str, Any] = {
                "command": candidate,
                "sample_count": support,
                "predicted_pixel_feature_delta": None,
            }
            in_range = bool(observed) and self._within_observed_range(candidate, observed)
            if support >= 3 and in_range:
                supported_candidate_count += 1
                delta = action_features(candidate, self._previous_by_channel.get(channel)) @ weights
                item["predicted_pixel_feature_delta"] = [round(float(v), 5) for v in delta]
            elif support < 3:
                item["unsupported_reason"] = "fewer than 3 transitions for this command kind/servo channel"
            else:
                item["unsupported_reason"] = "candidate command values fall outside the observed range"
            predictions.append(item)
        return {
            "sample_count": n,
            "minimum_samples_per_command_channel": 3,
            "feature_order": "own,top; each 2x2 row-major regions; RGB means",
            "training_rmse": round(training_rmse, 5),
            "supported_candidate_count": supported_candidate_count,
            "predictions": predictions,
            "warning": "Training RMSE is not calibrated uncertainty. Associations are confounded by peer/scene motion and do not prove execution, contact, grasp, or success; unsupported channels are not extrapolated.",
        }
