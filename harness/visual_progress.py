"""Bounded, camera-only evidence about visual motion between navigation frames."""
from __future__ import annotations

import copy
from collections import deque
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np


class VisualProgressHistory:
    """Record observations without selecting, changing, or recommending actions."""

    def __init__(self, max_entries: int = 6, unchanged_threshold: float = 0.018):
        if isinstance(max_entries, bool) or not isinstance(max_entries, int) or not 2 <= max_entries <= 12:
            raise ValueError("INVALID_PROGRESS_HISTORY_SIZE")
        self._entries: deque[dict[str, Any]] = deque(maxlen=max_entries)
        self.unchanged_threshold = float(unchanged_threshold)

    def compare(self, previous_jpeg: bytes | None, current_jpeg: bytes,
                previous_action: Mapping[str, Any] | None) -> dict[str, Any]:
        evidence: dict[str, Any] = {
            "comparison_available": previous_jpeg is not None,
            "nav_mean_absolute_change": None,
            "nav_nearly_unchanged": None,
            "same_action_streak": self._same_action_streak(previous_action),
            "stagnation_streak": 0,
        }
        if previous_jpeg is not None:
            change = float(np.mean(cv2.absdiff(_small_gray(previous_jpeg), _small_gray(current_jpeg)))) / 255.0
            evidence["nav_mean_absolute_change"] = round(change, 5)
            evidence["nav_nearly_unchanged"] = change <= self.unchanged_threshold
            if evidence["nav_nearly_unchanged"]:
                evidence["stagnation_streak"] = self._stagnation_streak() + 1
        return evidence

    def record(self, *, action: Mapping[str, Any], evidence: Mapping[str, Any],
               nav_frame_id: str | int, nav_sha256: str) -> None:
        self._entries.append({"action": copy.deepcopy(dict(action)),
                              "nav_frame_id": copy.deepcopy(nav_frame_id), "nav_sha256": nav_sha256,
                              "nav_mean_absolute_change": evidence.get("nav_mean_absolute_change"),
                              "nav_nearly_unchanged": evidence.get("nav_nearly_unchanged")})

    def recent(self) -> list[dict[str, Any]]:
        return copy.deepcopy(list(self._entries))

    def _same_action_streak(self, action: Mapping[str, Any] | None) -> int:
        if action is None:
            return 0
        streak, target = 0, dict(action)
        for item in reversed(self._entries):
            if item["action"] != target:
                break
            streak += 1
        return streak

    def _stagnation_streak(self) -> int:
        streak = 0
        for item in reversed(self._entries):
            if item["nav_nearly_unchanged"] is not True:
                break
            streak += 1
        return streak


def _small_gray(jpeg: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("INVALID_PROGRESS_IMAGE")
    return cv2.resize(image, (64, 48), interpolation=cv2.INTER_AREA)
