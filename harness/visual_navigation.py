"""Deterministic navigation using one robot's fixed forward RGB camera only."""
from __future__ import annotations

import base64
import hashlib
import math
from collections import deque
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from sim.navigation_camera_profile import NAV_CAMERA_NAME, ground_point_robot


_GOAL_HSV = {
    "A": ((100, 105, 35), (132, 255, 255)),
    "B": ((40, 85, 30), (84, 255, 255)),
    # Keep a gap from the orange obstacle band (H <= 19).
    "C": ((22, 105, 55), (38, 255, 255)),
}
_ORANGE_HSV = ((4, 120, 55), (19, 255, 255))
_MAGENTA_HSV = ((138, 85, 45), (179, 255, 255))
_ALLOWED_FIELDS = {
    "robot_id", "frame_id", "sim_time", "image", "sha256", "camera",
    "actuator_state",
}
_MAX_SEARCH_TURNS = 16
_MAX_DETOUR_STEPS = 24
_ZONE_SIDE_M = .82


def _drive(forward: float, turn: float, duration: float) -> dict[str, Any]:
    return {"kind": "drive", "fwd": max(0.0, min(.15, float(forward))),
            "turn": max(-.2, min(.2, float(turn))),
            "duration": max(.05, min(1.0, float(duration)))}


def _mask(hsv: np.ndarray, limits: tuple[tuple[int, int, int], tuple[int, int, int]]) -> np.ndarray:
    result = cv2.inRange(hsv, np.asarray(limits[0], np.uint8), np.asarray(limits[1], np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)


def _components(mask: np.ndarray, minimum_area: float) -> list[dict[str, Any]]:
    height, width = mask.shape
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < minimum_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        bottom = min(height - 1, y + h - 1)
        u = x + (w - 1) / 2.0
        ground = ground_point_robot(u, bottom, width=width, height=height)
        far_ground = ground_point_robot(u, y, width=width, height=height)
        bottom_left = ground_point_robot(x, bottom, width=width, height=height)
        bottom_right = ground_point_robot(x + w - 1, bottom, width=width, height=height)
        result.append({
            "pixel_bbox": [int(x), int(y), int(w), int(h)],
            "area_px": area,
            "area_ratio": area / float(width * height),
            "bottom_pixel": [float(u), int(bottom)],
            "ground_bottom_m": None if ground is None else [float(ground[0]), float(ground[1])],
            "ground_far_edge_m": None if far_ground is None else [float(far_ground[0]), float(far_ground[1])],
            "ground_bottom_edges_m": [None if point is None else [float(point[0]), float(point[1])]
                                      for point in (bottom_left, bottom_right)],
        })
    return sorted(result, key=lambda item: item["area_px"], reverse=True)


class VisualNavigator:
    """A bounded-state, camera-only short-motion policy.

    The static camera calibration is an owned sensor property.  No layout,
    robot pose, destination slot, or peer identity/state enters this class.
    """

    def __init__(self, destination_zone: str, robot_id: str):
        zone = str(destination_zone).upper()
        if zone not in _GOAL_HSV:
            raise ValueError("destination_zone must be A, B, or C")
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError("robot_id must be a non-empty string")
        self.destination_zone = zone
        self.robot_id = robot_id
        self.last_observation: dict[str, Any] | None = None
        self.history: deque[dict[str, Any]] = deque(maxlen=24)
        self._last_frame: int | str | None = None
        self._last_time = -math.inf
        self._detour_sign = 0
        self._detour_active = False
        self._detour_steps = 0
        self._detour_clear_frames = 0
        self._search_step = 0
        self._peer_waits = 0

    def decide(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        frame, frame_id, sim_time = self._validate_and_decode(observation)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pixels = frame.shape[0] * frame.shape[1]
        goal = _components(_mask(hsv, _GOAL_HSV[self.destination_zone]), max(180.0, pixels * .0010))
        obstacles = _components(_mask(hsv, _ORANGE_HSV), max(100.0, pixels * .0005))
        peers = _components(_mask(hsv, _MAGENTA_HSV), max(70.0, pixels * .0003))

        # A magenta chassis band sits above the floor.  Its ground-plane ray
        # overstates range; 0.68 is the conservative lower-height bound for the
        # known fixed 0.32 m camera mount, used only for collision avoidance.
        for peer in peers:
            projected = peer["ground_bottom_m"]
            peer["conservative_range_m"] = None if projected is None else max(0.0, .68 * projected[0])
        for region in goal:
            far_edge = region["ground_far_edge_m"]
            region["estimated_zone_center_m"] = (None if far_edge is None else
                [far_edge[0] - _ZONE_SIDE_M / 2.0, far_edge[1]])

        action, rationale = self._choose(frame.shape, goal, obstacles, peers)
        record = {
            "frame_id": frame_id,
            "sim_time": sim_time,
            "goal": {"zone": self.destination_zone, "regions": goal},
            "obstacles": obstacles,
            "peers": peers,
            "decision": action.copy(),
            "rationale": rationale,
        }
        self.history.append(record)
        self.last_observation = {**record, "history": list(self.history)}
        self._last_frame = frame_id
        self._last_time = sim_time
        return action

    def _choose(self, shape: tuple[int, ...], goal: list[dict[str, Any]],
                obstacles: list[dict[str, Any]], peers: list[dict[str, Any]]) -> tuple[dict[str, Any], str]:
        height, width = shape[:2]

        peer_danger = [p for p in peers if p["conservative_range_m"] is not None
                       and p["conservative_range_m"] < 1.0
                       and p["ground_bottom_m"] is not None
                       and abs(p["ground_bottom_m"][1]) < .38]
        if peer_danger and self._peer_waits < 3:
            self._peer_waits += 1
            return {"kind": "wait", "duration": .5}, "camera-visible peer in inflated forward corridor"
        if peer_danger:
            self._peer_waits = 0
            side = self._open_side(obstacles + peers)
            return _drive(0.0, .16 * side, .55), "bounded peer wait exhausted; turn toward visible open side"
        self._peer_waits = 0

        danger = [o for o in obstacles if o["ground_bottom_m"] is not None
                  and 0.0 <= o["ground_bottom_m"][0] < .9
                  and self._lateral_clearance(o) < .34]
        if danger and not self._detour_active:
            self._detour_active = True
            self._detour_steps = 0
            self._detour_clear_frames = 0
            if not self._detour_sign:
                self._detour_sign = self._open_side(obstacles + peers)

        if self._detour_active:
            self._detour_steps += 1
            if self._detour_steps > _MAX_DETOUR_STEPS:
                self._detour_active = False
                return {"kind": "finish", "reason": "NAVIGATION_DETOUR_EXHAUSTED"}, "bounded visual detour exhausted"
            nearby = [o for o in obstacles if o["ground_bottom_m"] is not None
                      and 0.0 <= o["ground_bottom_m"][0] < 1.3]
            if danger:
                self._detour_clear_frames = 0
                return _drive(.025, .18 * self._detour_sign, .65), "orange obstacle in inflated chassis/load corridor"
            if nearby:
                self._detour_clear_frames = 0
                return _drive(.08, .045 * self._detour_sign, .7), "passing visible orange obstacle on persistent detour side"
            self._detour_clear_frames += 1
            if self._detour_clear_frames < 3:
                return _drive(.06, 0.0, .5), "confirming orange obstacle has cleared camera view"
            self._detour_active = False
            self._detour_sign = 0

        if danger:
            # Defensive fallback; starting a detour above normally consumes it.
            return _drive(.025, .18 * self._detour_sign, .65), "orange obstacle in inflated chassis/load corridor"

        if goal:
            region = goal[0]
            x, y, w, h = region["pixel_bbox"]
            center_error = ((x + w / 2.0) - width / 2.0) / (width / 2.0)
            ground = region["ground_bottom_m"]
            zone_center = region["estimated_zone_center_m"]
            central_lower_coverage = (region["area_ratio"] >= .10 and w >= .42 * width
                                      and y + h >= .94 * height and abs(center_error) <= .32)
            center_ahead = (zone_center is not None and .18 <= zone_center[0] <= .40
                            and abs(zone_center[1]) <= .25)
            if central_lower_coverage and center_ahead:
                return {"kind": "finish", "reason": "NAVIGATION_ARRIVED"}, "carried-box center is inside visible destination floor region"
            turn = max(-.16, min(.16, -.16 * center_error))
            forward = .045 if abs(center_error) > .45 else (.08 if ground is None or ground[0] < .8 else .13)
            return _drive(forward, turn, .65), "steer toward image-derived destination floor region"

        # A single-direction in-place scan covers one bounded full-circle
        # attempt without ever advancing into unseen floor.  The macro runtime
        # limits every segment; exhaustion is explicit instead of looping.
        if self._search_step >= _MAX_SEARCH_TURNS:
            return {"kind": "finish", "reason": "NAVIGATION_SEARCH_EXHAUSTED"}, "bounded full-circle RGB search exhausted"
        self._search_step += 1
        return _drive(0.0, .2, 1.0), "bounded full-circle destination scan"

    @staticmethod
    def _open_side(items: list[dict[str, Any]]) -> int:
        # Positive turn is selected for the image side with less close colored
        # occupancy.  Ground y is positive to robot-left, hence its inverse
        # relation to image u is already captured by the sign below.
        left = right = 0.0
        for item in items:
            x, _y, w, _h = item["pixel_bbox"]
            weight = float(item["area_px"])
            if x + w / 2.0 < 320:
                left += weight
            else:
                right += weight
        return 1 if left <= right else -1

    @staticmethod
    def _lateral_clearance(item: dict[str, Any]) -> float:
        """Distance from the robot centerline to the object's nearest seen edge."""
        ys = [point[1] for point in item.get("ground_bottom_edges_m", ()) if point is not None]
        if not ys:
            center = item.get("ground_bottom_m")
            return math.inf if center is None else abs(float(center[1]))
        low, high = min(ys), max(ys)
        return 0.0 if low <= 0.0 <= high else min(abs(low), abs(high))

    def _validate_and_decode(self, observation: Mapping[str, Any]) -> tuple[np.ndarray, int | str, float]:
        if not isinstance(observation, Mapping) or set(observation) != _ALLOWED_FIELDS:
            raise ValueError("INVALID_NAVIGATION_OBSERVATION_FIELDS")
        if observation["robot_id"] != self.robot_id:
            raise ValueError("FOREIGN_FRAME")
        if observation["camera"] != NAV_CAMERA_NAME:
            raise ValueError("INVALID_NAVIGATION_CAMERA")
        frame_id = observation["frame_id"]
        stale_numeric = (isinstance(frame_id, int) and isinstance(self._last_frame, int)
                         and frame_id <= self._last_frame)
        if (isinstance(frame_id, bool) or not isinstance(frame_id, (int, str))
                or frame_id == self._last_frame or stale_numeric):
            raise ValueError("STALE_OR_INVALID_FRAME")
        sim_time = observation["sim_time"]
        if isinstance(sim_time, bool) or not isinstance(sim_time, (int, float)) or not math.isfinite(sim_time):
            raise ValueError("INVALID_SIM_TIME")
        if sim_time < 0 or sim_time < self._last_time:
            raise ValueError("STALE_OR_INVALID_SIM_TIME")
        encoded, digest = observation["image"], observation["sha256"]
        if not isinstance(encoded, str) or not isinstance(digest, str):
            raise ValueError("INVALID_CAMERA_IMAGE")
        try:
            data = base64.b64decode(encoded, validate=True)
        except (ValueError, TypeError) as exc:
            raise ValueError("INVALID_CAMERA_IMAGE") from exc
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError("IMAGE_HASH_MISMATCH")
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("INVALID_CAMERA_IMAGE")
        if not isinstance(observation["actuator_state"], Mapping):
            raise ValueError("INVALID_ACTUATOR_STATE")
        return frame, frame_id, float(sim_time)
