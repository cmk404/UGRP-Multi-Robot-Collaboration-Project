"""Interpretable navigation from an authored map and RGB images only.

The map describes immutable geometry.  Runtime position is always estimated from
the fixed overhead RGB image; issued commands are retained only to associate an
observed image displacement with a control probe.  ``own_jpeg`` is decoded,
validated, and archived for audit, but the current classical navigation policy
does not derive position or commands from its pixels; top RGB determines both.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import heapq
import math
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from sim.authored_navigation_map import validate_map


Point = tuple[float, float]
Cell = tuple[int, int]
_STOP = {"kind": "mecanum", "forward": 0.0, "left": 0.0, "turn": 0.0, "duration_s": 0.25}


def _decode_jpeg(value: bytes, name: str) -> np.ndarray:
    if not isinstance(value, bytes) or not value.startswith(b"\xff\xd8"):
        raise ValueError(f"{name} must be JPEG bytes")
    image = cv2.imdecode(np.frombuffer(value, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"{name} must be a decodable JPEG")
    return image


def pixel_to_world(
    pixel_uv: Sequence[float], image_shape: Sequence[int], top_camera: Mapping[str, Any],
    *, feature_plane_height_m: float = 0.09,
) -> Point:
    """Project a pixel onto a horizontal plane for the declared fixed nadir camera.

    This intentionally supports only the v1 camera orientation.  The yellow robot
    feature is approximated as lying 9 cm above the floor (configurable from 8--10
    cm for sensitivity tests).
    """
    if not 0.08 <= feature_plane_height_m <= 0.10:
        raise ValueError("feature plane height must be in [0.08, 0.10] m")
    if list(top_camera.get("quaternion_wxyz", ())) != [1, 0, 0, 0]:
        raise ValueError("only the declared fixed nadir camera is supported")
    h, w = int(image_shape[0]), int(image_shape[1])
    if h <= 0 or w <= 0:
        raise ValueError("invalid image shape")
    cx, cy, cz = (float(v) for v in top_camera["position_m"])
    plane_distance = cz - feature_plane_height_m
    visible_h = 2.0 * plane_distance * math.tan(math.radians(float(top_camera["fov_y_deg"])) / 2.0)
    visible_w = visible_h * w / h
    u, v = float(pixel_uv[0]), float(pixel_uv[1])
    return cx + (u - (w - 1) / 2.0) * visible_w / w, cy - (v - (h - 1) / 2.0) * visible_h / h


@dataclass(frozen=True)
class _Grid:
    xmin: float
    ymin: float
    resolution: float
    width: int
    height: int
    blocked: np.ndarray

    def cell(self, point: Point) -> Cell:
        return (int(round((point[0] - self.xmin) / self.resolution)),
                int(round((point[1] - self.ymin) / self.resolution)))

    def point(self, cell: Cell) -> Point:
        return self.xmin + cell[0] * self.resolution, self.ymin + cell[1] * self.resolution

    def clear(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and not bool(self.blocked[y, x])


def _build_grid(data: Mapping[str, Any]) -> _Grid:
    data = validate_map(data)
    xmin, xmax, ymin, ymax = map(float, data["bounds_m"])
    res = float(data["grid_resolution_m"])
    width = int(math.floor((xmax - xmin) / res)) + 1
    height = int(math.floor((ymax - ymin) / res)) + 1
    blocked = np.zeros((height, width), dtype=bool)
    inflate = float(data["footprint"]["unloaded_radius_m"]) + float(data["footprint"]["safety_margin_m"])
    xs = xmin + np.arange(width) * res
    ys = ymin + np.arange(height) * res
    # Bounds describe the usable floor, so the chassis centre must remain one
    # inflated footprint radius inside them.
    blocked |= ((xs[None, :] < xmin + inflate) | (xs[None, :] > xmax - inflate) |
                (ys[:, None] < ymin + inflate) | (ys[:, None] > ymax - inflate))
    for obstacle in data["obstacles"]:
        if bool(obstacle.get("traversable", False)):
            continue
        ox, oy = map(float, obstacle["center_m"])
        hx, hy = map(float, obstacle["half_extents_m"])
        blocked |= ((np.abs(xs[None, :] - ox) <= hx + inflate) &
                    (np.abs(ys[:, None] - oy) <= hy + inflate))
    return _Grid(xmin, ymin, res, width, height, blocked)


def _probe_disk_clear(data: Mapping[str, Any], position: Point, radius_m: float = 0.05) -> bool:
    """Check a free centre-position disk enclosing either unknown-yaw probe."""
    grid = _build_grid(data)
    centre = grid.cell(position)
    reach = int(math.ceil(radius_m / grid.resolution))
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            if math.hypot(dx, dy) * grid.resolution <= radius_m + grid.resolution * 0.5:
                if not grid.clear((centre[0] + dx, centre[1] + dy)):
                    return False
    return True


def _line_clear(grid: _Grid, a: Cell, b: Cell) -> bool:
    # Dense sampling plus a supercover-sized radius prevents diagonal corner cuts.
    steps = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 1
    for t in np.linspace(0.0, 1.0, steps):
        x = int(round(a[0] + (b[0] - a[0]) * t))
        y = int(round(a[1] + (b[1] - a[1]) * t))
        if not grid.clear((x, y)):
            return False
        if 0 < t < 1 and not (grid.clear((x - 1, y)) and grid.clear((x + 1, y)) and
                              grid.clear((x, y - 1)) and grid.clear((x, y + 1))):
            return False
    return True


def plan_grid_path(data: Mapping[str, Any], start: Point, goal: Point) -> list[Point] | None:
    """Return an inflated-grid A* path, simplified only across clear line segments."""
    grid = _build_grid(data)
    source, target = grid.cell(start), grid.cell(goal)
    if not grid.clear(source) or not grid.clear(target):
        return None
    frontier: list[tuple[float, Cell]] = [(0.0, source)]
    cost = {source: 0.0}
    parent: dict[Cell, Cell] = {}
    moves = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == target:
            break
        for dx, dy in moves:
            nxt = current[0] + dx, current[1] + dy
            if not grid.clear(nxt):
                continue
            if dx and dy and (not grid.clear((current[0] + dx, current[1])) or
                              not grid.clear((current[0], current[1] + dy))):
                continue
            new_cost = cost[current] + math.hypot(dx, dy)
            if new_cost < cost.get(nxt, math.inf):
                cost[nxt], parent[nxt] = new_cost, current
                heuristic = math.hypot(target[0] - nxt[0], target[1] - nxt[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
    if target not in cost:
        return None
    cells = [target]
    while cells[-1] != source:
        cells.append(parent[cells[-1]])
    cells.reverse()
    simple = [cells[0]]
    anchor = 0
    while anchor < len(cells) - 1:
        candidate = len(cells) - 1
        while candidate > anchor + 1 and not _line_clear(grid, cells[anchor], cells[candidate]):
            candidate -= 1
        simple.append(cells[candidate])
        anchor = candidate
    points = [start] + [grid.point(c) for c in simple[1:-1]] + [goal]
    return points


class KnownMapNavigator:
    """Classical authored-map baseline whose navigation signal is fixed top RGB.

    The own-camera JPEG remains a required decoded audit input.  It currently has
    no bearing on localization or control.
    """

    def __init__(self, map_data: Mapping[str, Any], robot_id: str, condition: str = "map") -> None:
        validated_map = validate_map(map_data)
        if robot_id not in {"r1", "r3"}:
            raise ValueError("robot_id must be r1 or r3")
        if condition not in {"map", "direct"}:
            raise ValueError("condition must be map or direct")
        self.map_data = validated_map
        self.robot_id = robot_id
        self.condition = condition
        self._last_frame = -1
        self._position: Point | None = None
        self._phase = "initial"
        self._probe_origin: Point | None = None
        self._forward_delta: np.ndarray | None = None
        self._jacobian: np.ndarray | None = None
        self._goal_confirmations = 0
        self._localization_losses = 0
        self._issued: list[dict[str, Any]] = []
        self._image_history: list[dict[str, Any]] = []

    def _localize(self, image: np.ndarray) -> tuple[Point | None, dict[str, Any]]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([20, 70, 50], np.uint8), np.array([40, 255, 255], np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        components: list[tuple[Point, int]] = []
        xmin, xmax, ymin, ymax = map(float, self.map_data["bounds_m"])
        for i in range(1, count):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < 20:
                continue
            point = pixel_to_world(centroids[i], image.shape, self.map_data["top_camera"])
            if xmin <= point[0] <= xmax and ymin <= point[1] <= ymax:
                components.append((point, area))
        # The unchanged MasterPi appearance exposes several disconnected yellow
        # structural/wheel regions from above.  Group only components fitting in
        # one declared chassis diameter before associating a robot candidate.
        chassis_diameter = 2.0 * float(self.map_data["footprint"]["unloaded_radius_m"])
        parent = list(range(len(components)))

        def find(index: int) -> int:
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        def union(first: int, second: int) -> None:
            a, b = find(first), find(second)
            if a != b:
                parent[b] = a

        for first in range(len(components)):
            for second in range(first + 1, len(components)):
                if math.dist(components[first][0], components[second][0]) <= chassis_diameter:
                    union(first, second)
        groups: dict[int, list[tuple[Point, int]]] = {}
        for index, component in enumerate(components):
            groups.setdefault(find(index), []).append(component)
        candidates: list[tuple[Point, int, int, float]] = []
        rejected_oversized = 0
        for group in groups.values():
            span = max((math.dist(a[0], b[0]) for a in group for b in group), default=0.0)
            if span > chassis_diameter:
                rejected_oversized += 1
                continue
            total_area = sum(item[1] for item in group)
            center = (sum(item[0][0] * item[1] for item in group) / total_area,
                      sum(item[0][1] * item[1] for item in group) / total_area)
            candidates.append((center, total_area, len(group), span))
        reference = self._position
        if reference is None:
            reference = tuple(map(float, self.map_data["zones"]["start"]["center_m"]))
            max_distance = float(self.map_data["zones"]["start"]["radius_m"]) + 0.18
        else:
            max_distance = 0.22
        ranked = sorted(((math.dist(point, reference), point, area, count, span)
                         for point, area, count, span in candidates), key=lambda x: x[0])
        if not ranked or ranked[0][0] > max_distance:
            return None, {"ok": False, "reason": "yellow_component_lost",
                          "candidate_count": len(candidates), "component_count": len(components),
                          "rejected_oversized_groups": rejected_oversized}
        if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < 0.08:
            return None, {"ok": False, "reason": "ambiguous_yellow_groups",
                          "candidate_count": len(candidates), "component_count": len(components),
                          "rejected_oversized_groups": rejected_oversized}
        distance, point, area, group_components, group_span = ranked[0]
        confidence = max(0.0, min(1.0, 0.9 - distance / max(max_distance, 1e-6) * 0.25))
        return point, {"ok": True, "confidence": round(confidence, 3), "candidate_count": len(candidates),
                       "component_count": len(components), "group_component_count": group_components,
                       "group_span_m": group_span, "feature_area_px": area,
                       "feature_plane_height_m": 0.09,
                       "assumption": "yellow feature centroid lies on a 0.09 m horizontal plane"}

    def _result(self, action: Mapping[str, Any], status: str, localization: Mapping[str, Any],
                *, path: list[Point] | None = None, done: bool = False) -> dict[str, Any]:
        emitted = dict(action)
        self._issued.append(emitted)
        calibration: dict[str, Any] = {"phase": self._phase, "uses_issued_commands_as_measurement": False}
        if self._jacobian is not None:
            calibration["visual_displacement_jacobian"] = self._jacobian.tolist()
            calibration["condition_number"] = float(np.linalg.cond(self._jacobian))
        return {"action": emitted, "done": done, "status": status,
                "diagnostics": {"position_estimate_m": list(self._position) if self._position else None,
                                "path": [list(p) for p in path] if path else None,
                                "localization": dict(localization), "calibration": calibration,
                                "condition": self.condition, "issued_action_count": len(self._issued),
                                "own_rgb": self._image_history[-1]["own"]}}

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
            self._goal_confirmations = 0
            self._localization_losses += 1
            terminal = self._localization_losses >= 4
            return self._result(_STOP, "localization_lost" if terminal else "localization_uncertain",
                                localization, done=terminal)
        self._localization_losses = 0
        previous = self._position
        self._position = position
        goal = tuple(map(float, self.map_data["zones"]["goal"]["center_m"]))
        planned = ([position, goal] if self.condition == "direct" else plan_grid_path(self.map_data, position, goal))

        if self._phase == "initial":
            # A route is insufficient: unknown yaw means either probe may displace
            # the chassis in any map direction around the current image estimate.
            if plan_grid_path(self.map_data, position, goal) is None:
                return self._result(_STOP, "no_map_route", localization, done=True)
            if not _probe_disk_clear(self.map_data, position):
                self._phase = "failed"
                return self._result(_STOP, "calibration_unsafe_clearance", localization,
                                    path=planned, done=True)
            self._probe_origin = position
            self._phase = "forward_probe_issued"
            return self._result({"kind": "mecanum", "forward": 0.08, "left": 0.0, "turn": 0.0,
                                 "duration_s": 0.4}, "calibrating_forward", localization, path=planned)
        if self._phase == "forward_probe_issued":
            delta = np.subtract(position, self._probe_origin)
            if float(np.linalg.norm(delta)) < 0.008:
                self._phase = "failed"
                return self._result(_STOP, "calibration_no_visual_progress", localization,
                                    path=planned, done=True)
            self._forward_delta = delta / (0.08 * 0.4)
            self._probe_origin = position
            self._phase = "lateral_probe_issued"
            return self._result({"kind": "mecanum", "forward": 0.0, "left": 0.06, "turn": 0.0,
                                 "duration_s": 0.4}, "calibrating_lateral", localization, path=planned)
        if self._phase == "lateral_probe_issued":
            delta = np.subtract(position, self._probe_origin)
            if float(np.linalg.norm(delta)) < 0.006:
                self._phase = "failed"
                return self._result(_STOP, "calibration_no_visual_progress", localization,
                                    path=planned, done=True)
            self._jacobian = np.column_stack((self._forward_delta, delta / (0.06 * 0.4)))
            if not np.all(np.isfinite(self._jacobian)) or abs(float(np.linalg.det(self._jacobian))) < 0.08 or np.linalg.cond(self._jacobian) > 25:
                self._phase = "failed"
                return self._result(_STOP, "calibration_singular", localization,
                                    path=planned, done=True)
            self._phase = "navigating"
        if self._phase == "failed":
            return self._result(_STOP, "calibration_failed", localization, path=planned, done=True)
        if planned is None:
            return self._result(_STOP, "no_map_route", localization, done=True)

        conservative_radius = max(0.0, float(self.map_data["zones"]["goal"]["radius_m"]) - 0.025)
        if math.dist(position, goal) <= conservative_radius:
            self._goal_confirmations += 1
            if self._goal_confirmations >= 2:
                return self._result(_STOP, "arrived", localization, path=planned, done=True)
            return self._result(_STOP, "goal_confirmation_1_of_2", localization, path=planned)
        self._goal_confirmations = 0
        waypoint = planned[min(1, len(planned) - 1)]
        vector = np.subtract(waypoint, position)
        length = float(np.linalg.norm(vector))
        desired = vector * min(0.035, length) / max(length, 1e-9)
        impulse = np.linalg.solve(self._jacobian, desired)
        forward = float(np.clip(impulse[0] / 0.25, -0.05, 0.10))
        left = float(np.clip(impulse[1] / 0.25, -0.08, 0.08))
        if previous is not None and math.dist(previous, position) > 0.22:
            return self._result(_STOP, "localization_large_jump", {**localization, "ok": False}, path=planned)
        return self._result({"kind": "mecanum", "forward": forward, "left": left, "turn": 0.0,
                             "duration_s": 0.25}, "navigating", localization, path=planned)


__all__ = ["KnownMapNavigator", "pixel_to_world", "plan_grid_path"]
