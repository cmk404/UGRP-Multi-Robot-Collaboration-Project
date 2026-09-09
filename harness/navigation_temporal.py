"""Bounded temporal evidence from own navigation RGB and executed macros.

This module reports observations only.  It does not select actions, infer a
route, track object identity, or estimate position or distance.
"""
from __future__ import annotations

import copy
import math
from collections import deque
from collections.abc import Mapping
from typing import Any

import cv2

from harness.visual_drive_guard import _decode


_TERMINAL_MACRO_STATES = {"completed", "interrupted", "stopped"}
_FRAME_HISTORY = 12
_SIMILAR_GRAY_MAD = 0.015


class NavigationTemporalEvidence:
    """Accumulate bounded, own-RGB temporal evidence for one robot."""

    def __init__(self, robot_id: str):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError("INVALID_ROBOT_ID")
        self.robot_id = robot_id
        self._seen_macro_ids: deque[str] = deque(maxlen=64)
        self._seen_macro_id_set: set[str] = set()
        self._forward_control_total = 0.0
        self._turn_only_macros = 0
        self._direction_changes = 0
        self._last_turn_sign: int | None = None
        self._observation_count = 0
        self._frames: deque[dict[str, Any]] = deque(maxlen=_FRAME_HISTORY)
        self._recent_blocker: dict[str, Any] | None = None
        self._recent_blocked_observation: dict[str, Any] | None = None
        self._recent_blocked_at = 0
        self._recent_blocked_forward_total = 0.0
        self._current_vetoed = False

    def observe(self, nav_obs: Mapping[str, Any], current_nav_evidence: Mapping[str, Any],
                execution_feedback: Mapping[str, Any] | None) -> dict[str, Any]:
        """Return compact evidence after accounting for one terminal macro once."""
        if (not isinstance(nav_obs, Mapping) or nav_obs.get("robot_id") != self.robot_id
                or nav_obs.get("camera") != "nav_cam"):
            raise ValueError("NAVIGATION_TEMPORAL_OWNERSHIP_MISMATCH")
        if not isinstance(current_nav_evidence, Mapping):
            raise ValueError("INVALID_CURRENT_NAV_EVIDENCE")

        frame, _digest = _decode(nav_obs)
        tiny = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (24, 18),
                          interpolation=cv2.INTER_AREA)
        self._account_macro(execution_feedback)
        self._observation_count += 1

        stop = current_nav_evidence.get("forward_stop_check")
        vetoed = isinstance(stop, Mapping) and stop.get("vetoed") is True
        self._current_vetoed = vetoed
        if vetoed:
            # Retain one original, validated own observation; never a derived or foreign image.
            self._recent_blocked_observation = copy.deepcopy(dict(nav_obs))
            self._recent_blocked_at = self._observation_count
            self._recent_blocked_forward_total = self._forward_control_total
        blocker = _visible_near_blocker(current_nav_evidence)
        if blocker is not None:
            self._recent_blocker = {
                **blocker,
                "observation": self._observation_count,
                "forward_control_total": self._forward_control_total,
            }

        revisited = self._potential_revisited_blocked_view(tiny)
        self._frames.append({
            "gray": tiny,
            "observation": self._observation_count,
            "forward_control_total": self._forward_control_total,
            "vetoed": vetoed,
        })

        result: dict[str, Any] = {
            "executed_since_forward": {
                "turn_only_macros": self._turn_only_macros,
                "turn_direction_changes": self._direction_changes,
            },
            "forward_control_s_total": round(self._forward_control_total, 3),
            "meaning": "executed control time is not distance or passage evidence",
        }
        if self._recent_blocker is not None:
            saved = self._recent_blocker
            result["most_recent_near_blocker"] = {
                "label": saved["label"], "side": saved["side"],
                "bbox_norm": copy.deepcopy(saved["bbox_norm"]),
                "observations_ago": self._observation_count - saved["observation"],
                "forward_control_s_since_seen": round(
                    self._forward_control_total - saved["forward_control_total"], 3),
            }
        if revisited is not None:
            result["potential_revisited_blocked_view"] = revisited
        return result

    def eligible_recent_blocked_observation(self) -> dict[str, Any] | None:
        """Return one prior blocked own-RGB observation when comparison is warranted."""
        if self._recent_blocked_observation is None or self._current_vetoed:
            return None
        age = self._observation_count - self._recent_blocked_at
        if (age < 1 or age > _FRAME_HISTORY or self._turn_only_macros < 2
                or self._forward_control_total != self._recent_blocked_forward_total):
            return None
        return copy.deepcopy(self._recent_blocked_observation)

    @property
    def recent_blocked_observations_ago(self) -> int | None:
        if self._recent_blocked_observation is None:
            return None
        return self._observation_count - self._recent_blocked_at

    def _account_macro(self, feedback: Mapping[str, Any] | None) -> None:
        if not isinstance(feedback, Mapping):
            return
        macro_result = feedback.get("last_macro")
        if not isinstance(macro_result, Mapping):
            return
        macro_id = macro_result.get("decision_id")
        if not isinstance(macro_id, str) or not macro_id or macro_id in self._seen_macro_id_set:
            return
        if macro_result.get("status") not in _TERMINAL_MACRO_STATES:
            return
        macro = macro_result.get("macro")
        elapsed = macro_result.get("elapsed_drive_control_s")
        if (not isinstance(macro, Mapping) or isinstance(elapsed, bool)
                or not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0):
            return
        self._remember_macro_id(macro_id)
        if macro.get("kind") != "drive" or elapsed <= 0:
            return
        fwd, turn = macro.get("fwd"), macro.get("turn")
        if not _finite_number(fwd) or not _finite_number(turn):
            return
        if fwd > 0:
            self._forward_control_total += float(elapsed)
            self._turn_only_macros = 0
            self._direction_changes = 0
            self._last_turn_sign = None
        elif fwd == 0 and turn != 0:
            sign = 1 if turn > 0 else -1
            if self._last_turn_sign is not None and sign != self._last_turn_sign:
                self._direction_changes += 1
            self._last_turn_sign = sign
            self._turn_only_macros += 1

    def _remember_macro_id(self, macro_id: str) -> None:
        if len(self._seen_macro_ids) == self._seen_macro_ids.maxlen:
            self._seen_macro_id_set.discard(self._seen_macro_ids[0])
        self._seen_macro_ids.append(macro_id)
        self._seen_macro_id_set.add(macro_id)

    def _potential_revisited_blocked_view(self, tiny: Any) -> dict[str, Any] | None:
        candidates = []
        for saved in self._frames:
            observations_ago = self._observation_count - saved["observation"]
            if (not saved["vetoed"] or observations_ago < 2
                    or self._forward_control_total != saved["forward_control_total"]):
                continue
            delta = float(cv2.absdiff(tiny, saved["gray"]).mean() / 255.0)
            if delta <= _SIMILAR_GRAY_MAD:
                candidates.append((delta, observations_ago))
        if not candidates:
            return None
        delta, observations_ago = min(candidates)
        return {
            "similarity_gray_mad": round(delta, 4),
            "matched_observations_ago": observations_ago,
            "meaning": "similar appearance only; not proof of the same place or object",
        }


def _finite_number(value: Any) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def _visible_near_blocker(evidence: Mapping[str, Any]) -> dict[str, Any] | None:
    candidates = []
    for label, key in (("orange", "orange_obstacles"), ("peer", "magenta_peers")):
        regions = evidence.get(key)
        if not isinstance(regions, list):
            continue
        for region in regions[:2]:
            if not isinstance(region, Mapping):
                continue
            bbox = region.get("bbox_norm")
            if (not isinstance(bbox, list) or len(bbox) != 4
                    or not all(_finite_number(item) for item in bbox)):
                continue
            x, y, width, height = (float(item) for item in bbox)
            area = region.get("area_fraction", 0.0)
            if not _finite_number(area):
                area = 0.0
            # "Near" is only a conservative screen-space description.
            if y + height < 0.55 and float(area) < 0.02:
                continue
            center = x + width / 2.0
            side = "left" if center < 0.4 else "right" if center > 0.6 else "center"
            candidates.append((float(area), {"label": label, "side": side,
                                             "bbox_norm": [round(float(v), 4) for v in bbox]}))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


__all__ = ["NavigationTemporalEvidence"]
