"""RGB-only stop veto for LLM-authored navigation actions.

This module does not choose steering, propose a route, or mutate an action.  It
only rejects a drive macro when the robot's own current ``nav_cam`` image shows
an obstacle inside a fixed conservative footprint.
"""
from __future__ import annotations

import base64
import hashlib
import math
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from sim.navigation_camera_profile import NAV_CAMERA_NAME, ground_point_robot


ORANGE_HSV = ((4, 120, 55), (19, 255, 255))
MAGENTA_HSV = ((138, 85, 45), (179, 255, 255))
ORANGE_STOP_FORWARD_M = 0.45
ORANGE_HALF_CORRIDOR_M = 0.23
PEER_STOP_FORWARD_M = 0.55
PEER_HALF_CORRIDOR_M = 0.30
PEER_HEIGHT_RANGE_SCALE = 0.68
TURN_IMMEDIATE_RADIUS_M = 0.23
PEER_TURN_IMMEDIATE_RADIUS_M = 0.30


def _result(allowed: bool, reason: str, evidence: Mapping[str, Any]) -> dict[str, Any]:
    return {"allowed": bool(allowed), "reason": str(reason), "evidence": dict(evidence)}


def _drive_values(action: Mapping[str, Any]) -> tuple[bool, float, float]:
    if not isinstance(action, Mapping) or action.get("kind") != "drive":
        return False, 0.0, 0.0
    forward = action.get("fwd", action.get("forward", 0.0))
    turn = action.get("turn", 0.0)
    try:
        forward = float(forward); turn = float(turn)
    except (TypeError, ValueError):
        return True, math.nan, math.nan
    return True, forward, turn


def _decode(observation: Mapping[str, Any]) -> tuple[np.ndarray, str]:
    if not isinstance(observation, Mapping):
        raise ValueError("MISSING_NAVIGATION_OBSERVATION")
    if observation.get("camera") != NAV_CAMERA_NAME:
        raise ValueError("INVALID_NAVIGATION_CAMERA")
    encoded = observation.get("image")
    claimed_hash = observation.get("sha256")
    if not isinstance(encoded, str) or not encoded or not isinstance(claimed_hash, str):
        raise ValueError("MISSING_NAVIGATION_IMAGE")
    try:
        payload = base64.b64decode(encoded, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError("INVALID_NAVIGATION_IMAGE") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if digest != claimed_hash:
        raise ValueError("IMAGE_HASH_MISMATCH")
    frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.ndim != 3 or frame.shape[0] < 24 or frame.shape[1] < 32:
        raise ValueError("INVALID_NAVIGATION_IMAGE")
    return frame, digest


def _regions(frame: np.ndarray, limits, kind: str) -> list[dict[str, Any]]:
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(limits[0], np.uint8), np.asarray(limits[1], np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    minimum_area = max(70.0, width * height * 0.00025)
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < minimum_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        bottom = min(height - 1, y + h - 1)
        samples = [ground_point_robot(px, bottom, width, height)
                   for px in (x, x + (w - 1) / 2.0, x + w - 1)]
        valid = [point for point in samples if point is not None and point[0] >= 0.0]
        if not valid:
            continue
        scale = PEER_HEIGHT_RANGE_SCALE if kind == "magenta_peer" else 1.0
        forward = min(point[0] for point in valid) * scale
        lateral = sorted(point[1] * scale for point in valid)
        found.append({
            "kind": kind,
            "pixel_bbox": [int(x), int(y), int(w), int(h)],
            "area_px": area,
            "estimated_forward_m": float(forward),
            "estimated_lateral_interval_m": [float(lateral[0]), float(lateral[-1])],
            "range_model": ("known magenta peer-height conservative scale"
                            if kind == "magenta_peer" else "ground-contact bottom edge"),
        })
    return sorted(found, key=lambda item: item["estimated_forward_m"])


def _intersects(interval: list[float], half_width: float) -> bool:
    return interval[0] <= half_width and interval[1] >= -half_width


def _distance_to_centerline(interval: list[float]) -> float:
    return 0.0 if interval[0] <= 0.0 <= interval[1] else min(abs(v) for v in interval)


def validate_visual_drive(nav_observation: Mapping[str, Any],
                          action: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one LLM action; return a stop veto and RGB evidence only."""
    is_drive, forward, turn = _drive_values(action)
    if not is_drive:
        return _result(True, "NON_DRIVE_ACTION", {"inspection": "not required"})
    if not math.isfinite(forward) or not math.isfinite(turn):
        return _result(False, "INVALID_DRIVE_ACTION", {"inspection": "action rejected before motion"})
    try:
        frame, digest = _decode(nav_observation)
    except ValueError as exc:
        return _result(False, str(exc), {"image_sha256": None, "inspection": "no fallback source"})

    orange = _regions(frame, ORANGE_HSV, "orange_obstacle")
    peers = _regions(frame, MAGENTA_HSV, "magenta_peer")
    evidence = {
        "image_sha256": digest,
        "camera": NAV_CAMERA_NAME,
        "image_size": [int(frame.shape[1]), int(frame.shape[0])],
        "orange_regions": orange,
        "magenta_peer_regions": peers,
        "thresholds_m": {
            "orange_forward": ORANGE_STOP_FORWARD_M,
            "orange_lateral_half": ORANGE_HALF_CORRIDOR_M,
            "peer_forward": PEER_STOP_FORWARD_M,
            "peer_lateral_half": PEER_HALF_CORRIDOR_M,
        },
        "minimum_estimated_clearance_m": min(
            [r["estimated_forward_m"] for r in orange + peers], default=None),
    }
    if forward > 0.0:
        blocked_orange = [r for r in orange if r["estimated_forward_m"] < ORANGE_STOP_FORWARD_M
                          and _intersects(r["estimated_lateral_interval_m"], ORANGE_HALF_CORRIDOR_M)]
        if blocked_orange:
            evidence["blocking_region"] = blocked_orange[0]
            return _result(False, "ORANGE_OBSTACLE_IN_FORWARD_FOOTPRINT", evidence)
        blocked_peer = [r for r in peers if r["estimated_forward_m"] < PEER_STOP_FORWARD_M
                        and _intersects(r["estimated_lateral_interval_m"], PEER_HALF_CORRIDOR_M)]
        if blocked_peer:
            evidence["blocking_region"] = blocked_peer[0]
            return _result(False, "MAGENTA_PEER_IN_FORWARD_FOOTPRINT", evidence)
        return _result(True, "VISUAL_FORWARD_CLEAR", evidence)

    # A turn remains the LLM's own action. Veto only a colored body already
    # inside the immediate swept radius; visible objects farther away do not
    # trigger a global/blind prohibition.
    immediate = [r for r in orange if math.hypot(r["estimated_forward_m"],
                 _distance_to_centerline(r["estimated_lateral_interval_m"])) < TURN_IMMEDIATE_RADIUS_M]
    immediate += [r for r in peers if math.hypot(r["estimated_forward_m"],
                  _distance_to_centerline(r["estimated_lateral_interval_m"])) < PEER_TURN_IMMEDIATE_RADIUS_M]
    if immediate:
        evidence["blocking_region"] = min(immediate, key=lambda r: r["estimated_forward_m"])
        return _result(False, "VISIBLE_IMMEDIATE_OVERLAP_BLOCKS_TURN", evidence)
    return _result(True, "IN_PLACE_TURN_VISUALLY_CLEAR", evidence)


__all__ = ["validate_visual_drive"]
