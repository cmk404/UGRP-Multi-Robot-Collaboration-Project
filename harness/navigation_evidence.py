"""Compact RGB-only evidence for an external navigation decision maker.

The output describes visible colour regions and the existing forward stop
check.  It deliberately contains no route, steering recommendation, pose, or
hidden simulator state.
"""
from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from harness.visual_drive_guard import (
    MAGENTA_HSV,
    ORANGE_HSV,
    _decode,
    validate_visual_drive,
)


_TARGET_HSV = {
    "A": ((100, 105, 35), (132, 255, 255)),  # blue floor
    "B": ((40, 85, 30), (84, 255, 255)),    # green floor
    "C": ((22, 105, 55), (38, 255, 255)),   # yellow floor
}
_TARGET_COLOR = {"A": "blue", "B": "green", "C": "yellow"}


def _regions(hsv: np.ndarray, limits: tuple[tuple[int, int, int], tuple[int, int, int]],
             minimum_fraction: float) -> list[dict[str, Any]]:
    height, width = hsv.shape[:2]
    mask = cv2.inRange(hsv, np.asarray(limits[0], np.uint8),
                       np.asarray(limits[1], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    minimum_area = max(70.0, height * width * minimum_fraction)
    result = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < minimum_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        result.append({
            "bbox_norm": [round(x / width, 4), round(y / height, 4),
                          round(w / width, 4), round(h / height, 4)],
            "area_fraction": round(area / (width * height), 6),
        })
    return sorted(result, key=lambda region: region["area_fraction"], reverse=True)


def _measure(frame: np.ndarray, zone: str) -> dict[str, Any]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange = _regions(hsv, ORANGE_HSV, .00025)[:2]
    peers = _regions(hsv, MAGENTA_HSV, .00025)[:1]
    targets = _regions(hsv, _TARGET_HSV[zone], .001)[:1]
    return {"orange": orange, "peers": peers, "target": targets[0] if targets else None}


def _largest_obstacle_area(measured: Mapping[str, Any]) -> float:
    areas = [r["area_fraction"] for key in ("orange", "peers") for r in measured[key]]
    return max(areas, default=0.0)


def _center(region: Mapping[str, Any] | None) -> list[float] | None:
    if region is None:
        return None
    x, y, width, height = region["bbox_norm"]
    return [x + width / 2.0, y + height / 2.0]


def navigation_evidence(nav_observation: Mapping[str, Any], destination_zone: str,
                        previous_observation: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Return bounded evidence derived only from validated own ``nav_cam`` RGB."""
    zone = str(destination_zone).upper()
    if zone not in _TARGET_HSV:
        raise ValueError("destination_zone must be A, B, or C")

    frame, digest = _decode(nav_observation)
    measured = _measure(frame, zone)
    guard = validate_visual_drive(nav_observation,
                                  {"kind": "drive", "fwd": .01, "turn": 0.0})
    result: dict[str, Any] = {
        "source": "validated own nav_cam RGB only",
        "image_sha256": digest,
        "orange_obstacles": measured["orange"],
        "magenta_peers": measured["peers"],
        "target_floor": {"zone": zone, "color": _TARGET_COLOR[zone],
                         "visible": measured["target"] is not None,
                         "region": measured["target"]},
        "forward_stop_check": {
            "vetoed": not guard["allowed"],
            "reason": guard["reason"],
            "meaning": "same-RGB positive-forward stop check; clear does not establish a safe route",
        },
    }

    if previous_observation is not None:
        current_robot = nav_observation.get("robot_id")
        previous_robot = previous_observation.get("robot_id")
        if (isinstance(current_robot, str) and isinstance(previous_robot, str)
                and current_robot != previous_robot):
            raise ValueError("PREVIOUS_NAVIGATION_ROBOT_MISMATCH")
        previous_frame, _ = _decode(previous_observation)
        previous = _measure(previous_frame, zone)
        current_center = _center(measured["target"])
        previous_center = _center(previous["target"])
        center_delta = None
        if current_center is not None and previous_center is not None:
            center_delta = [round(current_center[i] - previous_center[i], 4) for i in (0, 1)]
        result["screen_change"] = {
            "largest_obstacle_area_fraction_delta": round(
                _largest_obstacle_area(measured) - _largest_obstacle_area(previous), 6),
            "target_center_norm_delta": center_delta,
            "meaning": "screen change only; no object identity tracking, so blobs may differ; not physical distance",
        }
    return result


__all__ = ["navigation_evidence"]
