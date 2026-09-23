"""Image-only extraction of orange beam features from camera JPEGs."""

from __future__ import annotations

import math
from functools import lru_cache
import copy
from typing import Any

import cv2
import numpy as np

from harness.camera_beam_shaft import robust_shaft_geometry


# Broad enough for the shaded orange faces seen by both cameras while excluding
# gray flooring and the yellow/gold parts of the robots.
_ORANGE_LOW = np.array((3, 105, 45), dtype=np.uint8)
_ORANGE_HIGH = np.array((24, 255, 255), dtype=np.uint8)


def _ordered_corners(points: np.ndarray, center: tuple[float, float]) -> list[list[float]]:
    cx, cy = center
    ordered = sorted(points.tolist(), key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    start = min(range(4), key=lambda i: (ordered[i][1], ordered[i][0]))
    return ordered[start:] + ordered[:start]


def extract_beams(jpeg: bytes, robust_shaft: bool = False, *, hue_upper: int = 24,
                  min_saturation: int = 105, include_contour: bool = False) -> list[dict[str, Any]]:
    """Return orange connected components described only by their image pixels.

    Coordinates are normalized by image width and height. By default,
    ``length_px`` and ``width_px`` describe the component's 2-D minimum-area
    rectangle. With ``robust_shaft=True`` they describe its longest stable
    central shaft. Neither mode estimates physical dimensions or 3-D pose.
    """

    if not jpeg:
        return []
    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.size == 0:
        return []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return _extract_beams_from_hsv(hsv, robust_shaft=robust_shaft,
        hue_upper=hue_upper, min_saturation=min_saturation,
        include_contour=include_contour)


def _extract_beams_from_hsv(hsv: np.ndarray, *, robust_shaft: bool,
                            hue_upper: int, min_saturation: int,
                            include_contour: bool = False) -> list[dict[str, Any]]:
    height, width = hsv.shape[:2]
    if hue_upper not in (24, 35):
        raise ValueError("unsupported beam hue calibration")
    if isinstance(min_saturation,bool) or not isinstance(min_saturation,int) or not 0<=min_saturation<=255:
        raise ValueError("unsupported beam saturation calibration")
    low = _ORANGE_LOW.copy(); low[1] = min_saturation
    mask = cv2.inRange(hsv, low, np.array((hue_upper, 255, 255), np.uint8))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    minimum_area = max(100.0, width * height * 0.00045)
    candidates: list[dict[str, Any]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < minimum_area:
            continue
        (cx, cy), (rect_width, rect_height), angle = cv2.minAreaRect(contour)
        if rect_width <= 0.0 or rect_height <= 0.0:
            continue
        if rect_width >= rect_height:
            length_px, width_px = float(rect_width), float(rect_height)
            major_angle = math.radians(angle)
        else:
            length_px, width_px = float(rect_height), float(rect_width)
            major_angle = math.radians(angle + 90.0)
        dx = math.cos(major_angle) * length_px / 2.0
        dy = math.sin(major_angle) * length_px / 2.0
        endpoints_px = sorted(((cx - dx, cy - dy), (cx + dx, cy + dy)))
        box = _ordered_corners(cv2.boxPoints(((cx, cy), (rect_width, rect_height), angle)), (cx, cy))
        touches_border = bool(
            np.any(contour[:, 0, 0] <= 1)
            or np.any(contour[:, 0, 1] <= 1)
            or np.any(contour[:, 0, 0] >= width - 2)
            or np.any(contour[:, 0, 1] >= height - 2)
        )
        if robust_shaft:
            component = np.zeros_like(mask)
            cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
            component = cv2.bitwise_and(component, mask)
            shaft = robust_shaft_geometry(component)
            if shaft is None:
                continue
            cx, cy = shaft["center_px"]
            endpoints_px = sorted(map(tuple, shaft["endpoints_px"].tolist()))
            box = _ordered_corners(shaft["corners_px"], (cx, cy))
            width_px = float(shaft["width_px"])
            length_px = float(shaft["length_px"])
        candidates.append(
            {
                "center": [float(cx / width), float(cy / height)],
                "endpoints": [[float(x / width), float(y / height)] for x, y in endpoints_px],
                "corners4": [[float(x / width), float(y / height)] for x, y in box],
                "width_px": width_px,
                "length_px": length_px,
                "area_px": area,
                "image_size": [int(width), int(height)],
                "touches_border": touches_border,
                **({"contour_px": contour[:, 0, :].tolist()} if include_contour else {}),
            }
        )
    return sorted(candidates, key=lambda item: (-item["area_px"], item["center"][0], item["center"][1]))


@lru_cache(maxsize=4)
def _cached_carried_beam_scans(jpeg: bytes) -> tuple[tuple[dict[str, Any], ...], ...]:
    """Only image segmentation is shared; tracker identity remains per instance.

    The cache owns these mutable dictionaries and never exposes them directly.
    Four JPEGs bound retained memory even when many distinct frames arrive.
    """
    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.size == 0:
        return tuple(() for _ in range(105, 191, 5))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return tuple(tuple(_extract_beams_from_hsv(hsv, robust_shaft=True,
        hue_upper=35, min_saturation=saturation))
        for saturation in range(105, 191, 5))


def carried_beam_scans(jpeg: bytes) -> tuple[tuple[dict[str, Any], ...], ...]:
    """Independent copies of exact 18-threshold RGB candidates for one JPEG."""
    return copy.deepcopy(_cached_carried_beam_scans(bytes(jpeg)))


def select_beam(
    candidates: list[dict[str, Any]],
    anchor: list[float] | None,
    previous_center: list[float] | None,
) -> dict[str, Any] | None:
    """Select by temporal continuity, falling back to an image-derived anchor."""

    if not candidates:
        return None
    reference = previous_center if previous_center is not None else anchor
    if reference is None or len(reference) != 2:
        return None
    rx, ry = float(reference[0]), float(reference[1])
    chosen = min(
        candidates,
        key=lambda item: (
            (float(item["center"][0]) - rx) ** 2 + (float(item["center"][1]) - ry) ** 2,
            -float(item["area_px"]),
            float(item["center"][0]),
            float(item["center"][1]),
        ),
    )

    # A disappeared target must not jump to a distant orange distractor.
    if previous_center is not None and math.hypot(chosen["center"][0]-rx, chosen["center"][1]-ry) > 0.12:
        return None
    return chosen
