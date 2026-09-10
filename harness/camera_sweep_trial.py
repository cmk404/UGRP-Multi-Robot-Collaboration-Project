"""Image-only jaw-sweep topology gate for bounded exploratory closure."""
from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np


def _beam_polygon(beam: dict | None, image_size: tuple[int, int]) -> np.ndarray | None:
    if not isinstance(beam, dict) or beam.get("image_size") != list(image_size):
        return None
    corners = beam.get("corners4")
    if not isinstance(corners, (list, tuple)) or len(corners) != 4:
        return None
    try:
        polygon = np.asarray(corners, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if polygon.shape != (4, 2) or not np.all(np.isfinite(polygon)):
        return None
    width, height = image_size
    polygon = polygon * np.asarray((width, height), dtype=np.float64)
    if (np.any(polygon[:, 0] <= 0) or np.any(polygon[:, 0] >= width - 1)
            or np.any(polygon[:, 1] <= 0) or np.any(polygon[:, 1] >= height - 1)
            or abs(float(cv2.contourArea(polygon.astype(np.float32)))) <= 0):
        return None
    return polygon.astype(np.float32)


def evaluate_sweep_topology(candidates: tuple[dict[str, Any], ...], transition: dict | None,
                            old_beam: dict | None, new_beam: dict | None,
                            endpoint: list[float] | None) -> dict[str, Any]:
    """Return a compact fail-closed result; candidate masks remain ephemeral."""
    result: dict[str, Any] = {
        "passed": False, "candidate_count": 0, "candidates": [], "reason": None,
    }
    if not isinstance(transition, dict) or transition.get("from_pulse") != 1500 \
            or transition.get("to_pulse") != 2000:
        result["reason"] = "not a fresh 1500-to-2000 jaw transition"
        return result
    image_size = transition.get("image_size")
    if (not isinstance(image_size, list) or len(image_size) != 2
            or any(isinstance(v, bool) or not isinstance(v, int) or v < 16 for v in image_size)):
        result["reason"] = "invalid transition image size"
        return result
    width, height = image_size
    old_polygon = _beam_polygon(old_beam, (width, height))
    new_polygon = _beam_polygon(new_beam, (width, height))
    if old_polygon is None or new_polygon is None:
        result["reason"] = "old/current robust beam polygons are invalid"
        return result
    try:
        end = np.asarray(endpoint, dtype=np.float64)
    except (TypeError, ValueError):
        end = np.empty(0)
    if (end.shape != (2,) or not np.all(np.isfinite(end))
            or not 0 < end[0] < 1 or not 0 < end[1] < 1):
        result["reason"] = "selected beam endpoint is invalid or on image border"
        return result
    end = end * np.asarray((width, height), dtype=np.float64)
    if not candidates:
        result["reason"] = "no supported undilated motion candidates"
        return result

    beam_block = np.zeros((height, width), np.uint8)
    # Draw separately so coincident old/new polygons form a union rather than
    # cancelling their interiors under OpenCV's even-odd multi-contour fill.
    cv2.fillPoly(beam_block, [np.round(old_polygon).astype(np.int32)], 1)
    cv2.fillPoly(beam_block, [np.round(new_polygon).astype(np.int32)], 1)
    summaries = []
    for candidate in candidates:
        reasons = []
        mask = candidate.get("mask")
        axis = np.asarray(candidate.get("axis"), dtype=np.float64)
        if (not isinstance(mask, np.ndarray) or mask.shape != (height, width)
                or mask.dtype != np.bool_ or axis.shape != (2,)
                or not np.all(np.isfinite(axis))):
            reasons.append("invalid supported candidate geometry")
            selected = np.zeros((height, width), dtype=bool)
        else:
            norm = float(np.linalg.norm(axis))
            if not math.isfinite(norm) or norm <= 1e-9:
                reasons.append("invalid supported candidate axis")
                selected = np.zeros((height, width), dtype=bool)
            else:
                axis = axis / norm
                if axis[0] < 0:
                    axis = -axis
                selected = mask
        external = selected & (beam_block == 0)
        ys, xs = np.nonzero(external)
        points = np.column_stack((xs, ys)).astype(np.float64)
        raw_y, raw_x = np.nonzero(selected)
        raw_points = np.column_stack((raw_x, raw_y)).astype(np.float64)
        left = right = np.empty((0, 2))
        margins = None
        inside_distance = None
        intersection_area = 0.0
        if not reasons and len(raw_points):
            center = raw_points.mean(axis=0)
            projections = (points - center) @ axis if len(points) else np.empty(0)
            left, right = points[projections < 0], points[projections > 0]
        if len(left) == 0 or len(right) == 0:
            reasons.append("beam subtraction removed one side")
        else:
            left_center, right_center = left.mean(axis=0), right.mean(axis=0)
            left_projection = float(left_center @ axis)
            end_projection = float(end @ axis)
            right_projection = float(right_center @ axis)
            margins = [end_projection - left_projection,
                       right_projection - end_projection]
            if not left_projection < end_projection < right_projection:
                reasons.append("shaft endpoint is not strictly centroid-bracketed")
            hull = cv2.convexHull(points.astype(np.float32))
            if len(hull) < 3 or cv2.contourArea(hull) <= 0:
                reasons.append("external support hull has no positive area")
            else:
                inside_distance = float(cv2.pointPolygonTest(
                    hull, tuple(end.astype(float)), True
                ))
                if not math.isfinite(inside_distance) or inside_distance <= 0:
                    reasons.append("shaft endpoint is not strictly inside support hull")
                try:
                    intersection_area = float(cv2.intersectConvexConvex(
                        hull.astype(np.float32),
                        new_polygon.reshape(-1, 1, 2).astype(np.float32),
                    )[0])
                except cv2.error:
                    intersection_area = 0.0
                if not math.isfinite(intersection_area) or intersection_area <= 0:
                    reasons.append("no positive-area beam/support intersection")
        summaries.append({
            "threshold": int(candidate.get("threshold", -1)),
            "supporters": int(candidate.get("supporters", 0)),
            "raw_pixels": int(selected.sum()),
            "external_pixels": int(external.sum()),
            "removed_beam_pixels": int((selected & (beam_block > 0)).sum()),
            "left_pixels": int(len(left)), "right_pixels": int(len(right)),
            "bracket_margins_px": margins,
            "inside_distance_px": inside_distance,
            "intersection_area_px2": intersection_area,
            "passed": not reasons, "reasons": reasons,
        })
    result["candidate_count"] = len(summaries)
    result["candidates"] = summaries
    result["passed"] = bool(summaries) and all(item["passed"] for item in summaries)
    result["reason"] = ("all supported candidates pass sweep topology"
                        if result["passed"] else "at least one supported candidate failed")
    return result
