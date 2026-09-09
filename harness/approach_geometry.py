"""Pure geometry checks shared by visual face-approach controllers."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FaceStandoff:
    reached: bool
    radial_distance_m: float
    face_alignment: float


def assess_face_standoff(
    target_xy,
    face_normal_xy,
    *,
    standoff_m: float = 0.35,
    waypoint_tolerance_m: float = 0.055,
    alignment_tolerance_deg: float = 15.0,
) -> FaceStandoff:
    """Recognize a reached face standoff from the current marker observation.

    The distance band reuses the configured face waypoint distance and its
    existing waypoint acceptance tolerance. Alignment checks that the visible
    face normal points from the target toward the chassis. This lets a caller
    finish the face-positioning subgoal without requiring the derived waypoint
    itself to cross the chassis origin.
    """

    tx, ty = (float(value) for value in target_xy)
    nx, ny = (float(value) for value in face_normal_xy)
    if not all(math.isfinite(value) for value in (tx, ty, nx, ny)):
        raise ValueError("target and face normal coordinates must be finite")
    radial = math.hypot(tx, ty)
    normal_length = math.hypot(nx, ny)
    if radial <= 1e-9 or normal_length <= 1e-9:
        raise ValueError("target and face normal must have a defined direction")
    alignment = (nx * -tx + ny * -ty) / (normal_length * radial)
    within_distance = abs(radial - standoff_m) <= waypoint_tolerance_m
    aligned = alignment >= math.cos(math.radians(alignment_tolerance_deg))
    return FaceStandoff(within_distance and aligned, radial, alignment)
