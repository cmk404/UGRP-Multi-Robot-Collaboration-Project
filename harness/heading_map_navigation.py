"""Heading-first authored-map navigation using only the supplied RGB frames.

The initial forward pulse establishes the otherwise ambiguous chassis front from
observed top-image displacement.  Later yaw updates come from rotation of the
isolated yellow chassis appearance, never from issued turn commands.
"""

from __future__ import annotations

import hashlib
import math
from typing import Any, Mapping

import cv2
import numpy as np

from harness.known_map_navigation import (
    KnownMapNavigator, _CALIBRATION_STOP, _PROBE_DURATION_S,
    _PROBE_TARGET_DISPLACEMENT_M, _STOP, _decode_jpeg, _probe_disk_clear,
    plan_grid_path,
)


Point = tuple[float, float]
_TURN_DURATION_S = 0.6
_TURN_LIMIT = 0.15
_ALIGN_RAD = math.radians(10.0)


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _estimate_patch_rotation_deg(before: np.ndarray, after: np.ndarray,
                                 expected_sign: int = 0) -> tuple[float | None, dict[str, Any]]:
    """Match a centred chassis mask over small rotations.

    Positive image rotation has the same measured sign as world yaw for the
    declared fixed nadir view. ``expected_sign`` checks the independently
    measured response direction; the search itself always covers both signs.
    """
    if before.shape != after.shape or before.ndim != 2 or before.size == 0:
        return None, {"ok": False, "reason": "heading_patch_shape"}
    if np.count_nonzero(before) < 80 or np.count_nonzero(after) < 80:
        return None, {"ok": False, "reason": "heading_patch_sparse"}
    angles = np.arange(-12.0, 12.01, 0.5)
    h, w = before.shape
    # Smooth over individual mecanum roller facets while retaining the spatial
    # asymmetry of the complete chassis. This materially stabilizes consecutive
    # turns as different rollers become yellow-visible from the nadir camera.
    before_match = cv2.GaussianBlur(before, (0, 0), 2.0)
    after_match = cv2.GaussianBlur(after, (0, 0), 2.0)
    scores = []
    for angle in angles:
        rotation = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), float(angle), 1.0)
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                matrix = rotation.copy()
                matrix[:, 2] += (dx, dy)
                transformed = cv2.warpAffine(before_match, matrix, (w, h))
                score = float(cv2.matchTemplate(transformed, after_match,
                                                cv2.TM_CCOEFF_NORMED)[0, 0])
                scores.append((score, float(angle), dx, dy))
    best_score, best_angle, best_dx, best_dy = max(scores)
    zero_score = max(score for score, angle, _, _ in scores if angle == 0.0)
    diagnostics = {"ok": False, "image_rotation_deg": best_angle,
                   "score": round(best_score, 4), "zero_rotation_score": round(zero_score, 4),
                   "score_gain": round(best_score - zero_score, 4),
                   "registration_translation_px": [best_dx, best_dy]}
    # Unchanged-camera diagnostics with smoothing produced >=0.82 correlation;
    # small 3 degree pulses retained >=0.042 gain over the no-rotation match.
    if best_score < 0.80 or best_score - zero_score < 0.035 or abs(best_angle) < 1.0:
        diagnostics["reason"] = "heading_rotation_unresolved"
        return None, diagnostics
    if expected_sign and int(math.copysign(1, best_angle)) != expected_sign:
        diagnostics["reason"] = "heading_rotation_opposite_command"
        return None, diagnostics
    diagnostics["ok"] = True
    return best_angle, diagnostics


class HeadingMapNavigator(KnownMapNavigator):
    """Rotate toward the next map waypoint, then drive straight toward it."""

    def __init__(self, map_data: Mapping[str, Any], robot_id: str,
                 condition: str = "map") -> None:
        super().__init__(map_data, robot_id, condition)
        self._phase = "initial"
        self._heading_rad: float | None = None
        self._forward_gain_m_per_impulse: float | None = None
        self._straight_origin: Point | None = None
        self._heading_patch: np.ndarray | None = None
        self._pending_turn_sign = 0
        self._heading_losses = 0
        self._last_heading_observation: dict[str, Any] = {"ok": False, "reason": "not_calibrated"}

    def _patch(self, top: np.ndarray, position: Point) -> np.ndarray | None:
        hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([12, 60, 30], np.uint8),
                           np.array([42, 255, 255], np.uint8))
        # Choose pixels close to the already RGB-localized world position.  The
        # crop is recentered, making translation irrelevant to rotation matching.
        camera = self.map_data["top_camera"]
        h, w = top.shape[:2]
        plane_distance = float(camera["position_m"][2]) - 0.09
        visible_h = 2 * plane_distance * math.tan(math.radians(float(camera["fov_y_deg"])) / 2)
        visible_w = visible_h * w / h
        u = (position[0] - float(camera["position_m"][0])) * w / visible_w + (w - 1) / 2
        v = (float(camera["position_m"][1]) - position[1]) * h / visible_h + (h - 1) / 2
        yy, xx = np.indices(mask.shape)
        radius_px = max(32.0, 52.0 * w / 960.0)
        selected = (mask > 0) & ((xx - u) ** 2 + (yy - v) ** 2 <= radius_px ** 2)
        ys, xs = np.where(selected)
        if len(xs) < 80:
            return None
        size = 96
        # Use the localized chassis envelope centre rather than the yellow-pixel
        # centroid, which shifts as individual mecanum rollers rotate into view.
        transform = np.float32([[1, 0, size / 2 - u],
                                [0, 1, size / 2 - v]])
        isolated = np.where(selected, 255, 0).astype(np.uint8)
        return cv2.warpAffine(isolated, transform, (size, size), flags=cv2.INTER_NEAREST)

    def _result_heading(self, action: Mapping[str, Any], status: str,
                        localization: Mapping[str, Any], *, path=None, done=False):
        result = self._result(action, status, localization, path=path, done=done)
        result["diagnostics"]["heading"] = {
            "estimate_rad": self._heading_rad,
            "estimate_deg": None if self._heading_rad is None else math.degrees(self._heading_rad),
            "observation": dict(self._last_heading_observation),
            "source": "forward RGB displacement plus overhead RGB chassis rotation",
        }
        return result

    def decide(self, own_jpeg: bytes, top_jpeg: bytes, frame_id: int) -> dict[str, Any]:
        own = _decode_jpeg(own_jpeg, "own_jpeg")
        top = _decode_jpeg(top_jpeg, "top_jpeg")
        if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id <= self._last_frame:
            raise ValueError("frame_id must increase")
        self._last_frame = frame_id
        self._image_history.append({"frame_id": frame_id,
            "own": {"decoded": True, "shape": list(own.shape),
                    "sha256": hashlib.sha256(own_jpeg).hexdigest()},
            "top_sha256": hashlib.sha256(top_jpeg).hexdigest()})
        self._image_history = self._image_history[-4:]
        position, localization = self._localize(top)
        if position is None:
            self._straight_origin = None
            self._goal_confirmations = 0
            self._localization_losses += 1
            terminal = self._localization_losses >= 4
            return self._result_heading(_STOP, "localization_lost" if terminal else "localization_uncertain",
                                        localization, done=terminal)
        self._localization_losses = 0
        previous = self._position
        self._position = position
        if (previous is not None and self._phase == "navigating" and self._issued and
                float(self._issued[-1].get("forward", 0.0)) > 0 and
                float(self._issued[-1].get("turn", 0.0)) == 0.0):
            prior = self._issued[-1]
            observed = math.dist(previous, position)
            impulse = float(prior["forward"]) * float(prior["duration_s"])
            if 0.01 <= observed <= 0.12 and impulse > 0:
                observed_gain = observed / impulse
                # The largest valid RGB-observed response produces the shortest,
                # hence most conservative, future lease for the 4 cm target.
                self._forward_gain_m_per_impulse = max(
                    self._forward_gain_m_per_impulse or 0.0,
                    min(3.0, max(0.1, observed_gain)))
                if self._straight_origin is None:
                    self._straight_origin = previous
                straight_displacement = math.dist(self._straight_origin, position)
                if straight_displacement >= 0.08:
                    self._heading_rad = math.atan2(position[1] - self._straight_origin[1],
                                                   position[0] - self._straight_origin[0])
                    self._last_heading_observation = {
                        "ok": True, "kind": "straight_displacement_refresh",
                        "displacement_m": straight_displacement}
                    self._straight_origin = position
        goal = tuple(map(float, self.map_data["zones"]["goal"]["center_m"]))
        planned = [position, goal] if self.condition == "direct" else plan_grid_path(self.map_data, position, goal)
        if plan_grid_path(self.map_data, position, goal) is None:
            return self._result_heading(_STOP, "no_map_route", localization, done=True)
        patch = self._patch(top, position)
        if patch is None:
            self._straight_origin = None
            self._goal_confirmations = 0
            self._heading_losses += 1
            terminal = self._heading_losses >= 4
            return self._result_heading(_STOP, "heading_visual_lost" if terminal else "heading_visual_uncertain",
                                        {**localization, "ok": False}, path=planned, done=terminal)
        self._heading_losses = 0

        if self._phase == "initial":
            if not _probe_disk_clear(self.map_data, position):
                self._phase = "failed"
                return self._result_heading(_STOP, "calibration_unsafe_clearance", localization,
                                            path=planned, done=True)
            self._probe_origin = position
            self._probe_pulses["forward"] = 1
            self._probe_impulse["forward"] = 0.08 * _PROBE_DURATION_S
            self._phase = "forward_probe_issued"
            return self._result_heading({"kind": "mecanum", "forward": 0.08, "left": 0.0,
                                         "turn": 0.0, "duration_s": _PROBE_DURATION_S},
                                        "calibrating_forward", localization, path=planned)
        if self._phase == "forward_probe_issued":
            self._settle_previous = position
            self._phase = "forward_settling"
            self._settle_checks = 0
            return self._result_heading(_CALIBRATION_STOP, "settling_forward_probe", localization, path=planned)
        if self._phase == "forward_settling":
            residual = math.dist(position, self._settle_previous)
            if residual > 0.004:
                self._settle_previous = position
                self._settle_checks += 1
                if self._settle_checks > 3:
                    self._phase = "failed"
                    return self._result_heading(_STOP, "calibration_failed_to_settle", localization,
                                                path=planned, done=True)
                return self._result_heading(_CALIBRATION_STOP, "settling_forward_probe", localization, path=planned)
            delta = np.subtract(position, self._probe_origin)
            displacement = float(np.linalg.norm(delta))
            if displacement < _PROBE_TARGET_DISPLACEMENT_M:
                if self._probe_pulses["forward"] >= 6:
                    self._phase = "failed"
                    return self._result_heading(_STOP, "calibration_insufficient_visual_signal", localization,
                                                path=planned, done=True)
                if not _probe_disk_clear(self.map_data, position):
                    self._phase = "failed"
                    return self._result_heading(_STOP, "calibration_unsafe_clearance", localization,
                                                path=planned, done=True)
                self._probe_pulses["forward"] += 1
                self._probe_impulse["forward"] += 0.08 * _PROBE_DURATION_S
                self._phase = "forward_probe_issued"
                return self._result_heading({"kind": "mecanum", "forward": 0.08, "left": 0.0,
                                             "turn": 0.0, "duration_s": _PROBE_DURATION_S},
                                            "calibrating_forward_repeat", localization, path=planned)
            self._heading_rad = math.atan2(float(delta[1]), float(delta[0]))
            self._straight_origin = position
            self._forward_gain_m_per_impulse = displacement / self._probe_impulse["forward"]
            self._heading_patch = patch
            self._last_heading_observation = {"ok": True, "kind": "forward_displacement",
                                              "displacement_m": displacement}
            self._phase = "navigating"

        if self._phase == "turn_issued":
            assert self._heading_patch is not None and self._heading_rad is not None
            image_delta, observed = _estimate_patch_rotation_deg(
                self._heading_patch, patch, self._pending_turn_sign)
            self._last_heading_observation = observed
            if image_delta is None:
                self._phase = "failed"
                return self._result_heading(_STOP, "heading_rotation_unresolved", localization,
                                            path=planned, done=True)
            self._heading_rad = _wrap_angle(self._heading_rad + math.radians(image_delta))
            self._phase = "navigating"

        if self._phase == "failed":
            return self._result_heading(_STOP, "heading_failed", localization, path=planned, done=True)
        if planned is None:
            return self._result_heading(_STOP, "no_map_route", localization, done=True)
        if previous is not None and math.dist(previous, position) > 0.22:
            return self._result_heading(_STOP, "localization_large_jump",
                                        {**localization, "ok": False}, path=planned)
        conservative_radius = max(0.0, float(self.map_data["zones"]["goal"]["radius_m"]) - 0.025)
        if math.dist(position, goal) <= conservative_radius:
            self._straight_origin = None
            self._goal_confirmations += 1
            done = self._goal_confirmations >= 2
            return self._result_heading(_STOP, "arrived" if done else "goal_confirmation_1_of_2",
                                        localization, path=planned, done=done)
        self._goal_confirmations = 0
        assert self._heading_rad is not None
        waypoint = planned[min(1, len(planned) - 1)]
        desired = math.atan2(waypoint[1] - position[1], waypoint[0] - position[0])
        error = _wrap_angle(desired - self._heading_rad)
        if abs(error) > _ALIGN_RAD:
            self._straight_origin = None
            turn = _TURN_LIMIT if error > 0 else -_TURN_LIMIT
            self._pending_turn_sign = 1 if turn > 0 else -1
            self._heading_patch = patch
            self._phase = "turn_issued"
            return self._result_heading({"kind": "mecanum", "forward": 0.0, "left": 0.0,
                                         "turn": turn, "duration_s": _TURN_DURATION_S},
                                        "aligning_heading", localization, path=planned)
        distance = math.dist(position, waypoint)
        # Preserve the map baseline's roughly 4 cm visual step, but spend fewer
        # runner cycles by choosing a lease from the observed forward calibration.
        target_step = min(0.04, distance)
        forward = 0.10
        gain = max(self._forward_gain_m_per_impulse or 0.0, 0.05)
        duration = min(0.6, max(0.25, target_step / (forward * gain)))
        self._heading_patch = patch
        return self._result_heading({"kind": "mecanum", "forward": forward, "left": 0.0,
                                     "turn": 0.0, "duration_s": duration},
                                    "navigating_forward", localization, path=planned)


__all__ = ["HeadingMapNavigator"]
