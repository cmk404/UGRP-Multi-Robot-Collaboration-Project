"""Conservative two-edge camera-only reconstruction of a known square zone.

The normal placement path uses this only when its three-edge reconstruction is
underconstrained.  This module uses only RGB color boundaries, camera
calibration, and the known 0.82 m zone side.  Simulator pose and warehouse
truth are not inputs.
"""
from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from harness.visual_placement import _ZONE_SIDE_M, _zone_mask
from sim.navigation_camera_profile import ground_point_robot


def reconstruct_zone_from_corner(frame: np.ndarray, zone: str) -> dict[str, Any] | None:
    """Fit a square from two supported perpendicular visible floor edges."""
    if not isinstance(frame, np.ndarray) or frame.ndim != 3:
        raise ValueError("frame must be a BGR image")
    mask = _zone_mask(frame, zone)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < frame.shape[0] * frame.shape[1] * .01:
        return None

    metric, pixels = [], []
    height, width = frame.shape[:2]
    for u, v in contour[:, 0, :]:
        # A contour coincident with an image edge is clipping, not a measured
        # floor-zone boundary.
        if min(int(u), int(v), width - 1 - int(u), height - 1 - int(v)) < 8:
            continue
        point = ground_point_robot(float(u), float(v), width, height)
        if point is not None and 0.0 < point[0] < 5.0 and abs(point[1]) < 5.0:
            metric.append(point)
            pixels.append((int(u), int(v)))
    if len(metric) < 80:
        return None
    points = np.asarray(metric, np.float32)
    polygon = cv2.approxPolyDP(points.reshape(-1, 1, 2), .025, True).reshape(-1, 2)
    if len(polygon) < 3:
        return None

    candidates: list[dict[str, Any]] = []
    count = len(polygon)
    for index in range(count):
        vertex = polygon[index]
        before = polygon[(index - 1) % count]
        after = polygon[(index + 1) % count]
        first, second = before - vertex, after - vertex
        first_len, second_len = float(np.linalg.norm(first)), float(np.linalg.norm(second))
        if not (.25 <= first_len <= .96 and .25 <= second_len <= .96):
            continue
        first_axis, second_axis = first / first_len, second / second_len
        angle = math.degrees(math.acos(float(np.clip(abs(np.dot(first_axis, second_axis)), 0, 1))))
        axes = _orthogonal_axes(first_axis, second_axis)
        if axes is None:
            continue
        square_first, square_second, angular_error = axes
        supports = [_line_support(points, vertex, endpoint)
                    for endpoint in (before, after)]
        if min(supports) < 20:
            continue
        corners = np.asarray([
            vertex,
            vertex + square_first * _ZONE_SIDE_M,
            vertex + square_first * _ZONE_SIDE_M + square_second * _ZONE_SIDE_M,
            vertex + square_second * _ZONE_SIDE_M,
        ], np.float32)
        visible, green = _interior_support(mask, corners)
        if visible < 8 or green / visible < .78:
            continue
        candidates.append({
            "corners": corners,
            "center": np.mean(corners, axis=0),
            "score": min(supports) + green,
            "edge_lengths_m": [first_len, second_len],
            "edge_support_counts": supports,
            "perpendicular_angle_deg": angle,
            "orthogonality_error_deg": angular_error,
            "interior_visible_samples": visible,
            "interior_color_samples": green,
        })
    if not candidates:
        return None

    # Several observed corners of the same square should imply one center.
    groups: list[list[dict[str, Any]]] = []
    for candidate in sorted(candidates, key=lambda item: item["score"], reverse=True):
        group = next((items for items in groups
                      if np.linalg.norm(candidate["center"] - items[0]["center"]) <= .08), None)
        if group is not None:
            group.append(candidate)
        else:
            groups.append([candidate])
    groups.sort(key=lambda items: max(item["score"] for item in items), reverse=True)
    if len(groups) > 1:
        best_score = max(item["score"] for item in groups[0])
        runner_up = max(item["score"] for item in groups[1])
        if runner_up >= best_score * .85:
            return None
    best_group = groups[0]
    best = max(best_group, key=lambda item: item["score"])
    return {
        "calibrated_zone_square_corners_m": [[float(x), float(y)] for x, y in best["corners"]],
        "known_zone_side_m": _ZONE_SIDE_M,
        "visible_corner_consensus_count": len(best_group),
        "visible_edge_lengths_m": best["edge_lengths_m"],
        "visible_edge_support_counts": best["edge_support_counts"],
        "perpendicular_angle_deg": best["perpendicular_angle_deg"],
        "orthogonality_error_deg": best["orthogonality_error_deg"],
        "interior_color_fraction": best["interior_color_samples"] / best["interior_visible_samples"],
        "interior_visible_samples": best["interior_visible_samples"],
        "method": "two_visible_perpendicular_rgb_edges+known_square_side",
    }


def _orthogonal_axes(first: np.ndarray, second: np.ndarray,
                     max_error_deg: float = 3.0) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Return a balanced exact-right-angle fit, or reject angular ambiguity."""
    theta_first = math.atan2(float(first[1]), float(first[0]))
    cross = float(first[0] * second[1] - first[1] * second[0])
    turn = math.pi / 2 if cross >= 0 else -math.pi / 2
    theta_second = math.atan2(float(second[1]), float(second[0]))
    residual = (theta_second - (theta_first + turn) + math.pi) % (2 * math.pi) - math.pi
    error_deg = abs(math.degrees(residual))
    if error_deg > max_error_deg:
        return None
    # Split calibration/segmentation error evenly across both measured edges.
    theta = theta_first + residual / 2
    first_fit = np.asarray((math.cos(theta), math.sin(theta)), np.float32)
    second_theta = theta + turn
    second_fit = np.asarray((math.cos(second_theta), math.sin(second_theta)), np.float32)
    return first_fit, second_fit, error_deg


def signed_footprint_margins(square: dict[str, Any], center: tuple[float, float],
                             dimensions: tuple[float, float, float],
                             margin_m: float = .05) -> list[float]:
    """Apply the existing conservative footprint test to an experimental fit."""
    corners = np.asarray(square["calibrated_zone_square_corners_m"], np.float32)
    x, y = center
    hx, hy = dimensions[0] / 2 + margin_m, dimensions[1] / 2 + margin_m
    footprint = ((x, y), (x-hx, y-hy), (x-hx, y+hy), (x+hx, y-hy), (x+hx, y+hy))
    return [float(cv2.pointPolygonTest(corners, point, True)) for point in footprint]


def _line_support(points: np.ndarray, start: np.ndarray, end: np.ndarray) -> int:
    edge = end - start
    length = max(float(np.linalg.norm(edge)), 1e-9)
    distance = np.abs(edge[0] * (points[:, 1] - start[1])
                      - edge[1] * (points[:, 0] - start[0])) / length
    projection = np.dot(points - start, edge) / max(float(np.dot(edge, edge)), 1e-9)
    return int(np.count_nonzero((distance < .025) & (projection >= -.03) & (projection <= 1.03)))


def _project(point: np.ndarray, width: int, height: int) -> tuple[int, int] | None:
    # Keep projection logic in the existing calibrated implementation.
    from harness.visual_placement import _project_ground
    projected = _project_ground((float(point[0]), float(point[1])), width, height)
    if projected is None:
        return None
    return int(round(projected[0])), int(round(projected[1]))


def _interior_support(mask: np.ndarray, corners: np.ndarray) -> tuple[int, int]:
    height, width = mask.shape
    visible = green = 0
    # Avoid boundary samples; occluded/out-of-frame samples do not count as
    # either positive or negative evidence.
    for a in np.linspace(.15, .85, 5):
        for b in np.linspace(.15, .85, 5):
            point = (corners[0] * (1-a) * (1-b) + corners[1] * a * (1-b)
                     + corners[3] * (1-a) * b + corners[2] * a * b)
            pixel = _project(point, width, height)
            if pixel is None:
                continue
            u, v = pixel
            if not (8 <= u < width - 8 and 8 <= v < height - 8):
                continue
            visible += 1
            patch = mask[v-3:v+4, u-3:u+4]
            green += int(patch.size > 0 and np.mean(patch > 0) >= .60)
    return visible, green


__all__ = ["reconstruct_zone_from_corner", "signed_footprint_margins"]
