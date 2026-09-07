"""Calibrated camera/FK geometry for the physical MasterPi eye-in-hand camera.

This module is read-only: it never sends actuator commands.  It converts a
camera detection plus the *last commanded* PWM pose into a robot-base-relative
floor position.  Values are inherited from the physical pickup controller that
was previously tuned on ugrp1; calibration metadata is explicit so REAL state
never masquerades as simulator ground truth.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

# Physical arm geometry, centimetres.
LINK_1_CM = 9.30
LINK_2_CM = 6.50
LINK_3_CM = 6.20
CAMERA_LINK_CM = 7.0
CAMERA_Z_OFFSET_CM = 2.5

# Legacy MasterPi PWM calibration.
PULSE_PER_DEGREE = 2000.0 / 180.0
SERVO_DEVIATION = {3: 54, 4: 53, 5: 89, 6: 64}
BASE_CENTER = 1500

# Physical camera calibration currently used by the proven pickup controller.
CAMERA_VFOV_DEG = 48.0
CAMERA_HFOV_DEG = math.degrees(
    2.0 * math.atan(math.tan(math.radians(CAMERA_VFOV_DEG / 2.0)) * 4.0 / 3.0)
)
CAMERA_PITCH_OFFSET_DEG = 0.0

# The detection is a block body; the bottom of its bounding box is the floor
# contact ray.  For a 3-D centre estimate, add half-height separately.
DEFAULT_BLOCK_HEIGHT_M = 0.03
REQUIRED_SERVOS = (3, 4, 5, 6)
CALIBRATION_ID = "ugrp1-eye-in-hand-v1"
CALIBRATION_SOURCE = "physical_pick_controller_2026-08"


@dataclass(frozen=True)
class ArmPoint:
    radius_cm: float
    height_cm: float
    pitch_deg: float


@dataclass(frozen=True)
class GroundProjection:
    """Robot-base-relative metric estimate from one RGB detection."""

    # Base frame convention: +x forward, +y left, +z up.
    x_forward_m: float
    y_left_m: float
    z_ground_m: float
    range_m: float
    bearing_deg: float
    camera_height_m: float
    camera_radius_m: float
    ray_pitch_deg: float
    image_contact_xy: tuple[float, float]
    calibration_id: str = CALIBRATION_ID

    @property
    def position_xy(self) -> list[float]:
        return [self.x_forward_m, self.y_left_m]


def _pose_ints(pose: Mapping[int | str, Any] | None) -> dict[int, int]:
    if not isinstance(pose, Mapping):
        return {}
    out: dict[int, int] = {}
    for key, value in pose.items():
        try:
            servo = int(key)
            pulse = int(value)
        except (TypeError, ValueError):
            continue
        if servo in {1, 3, 4, 5, 6} and 400 <= pulse <= 2600:
            out[servo] = pulse
    return out


def nominal_pulse(pose: Mapping[int | str, Any], servo: int) -> float:
    p = _pose_ints(pose)
    if servo not in p:
        raise ValueError(f"missing servo {servo}")
    return float(p[servo] - SERVO_DEVIATION.get(servo, 0))


def forward_kinematics(
    pose: Mapping[int | str, Any],
    *,
    tool_length_cm: float = CAMERA_LINK_CM,
) -> ArmPoint | None:
    """Return camera/tool radial position, height and pitch in the base plane."""
    p = _pose_ints(pose)
    if any(servo not in p for servo in (3, 4, 5)):
        return None
    theta3 = (nominal_pulse(p, 3) - 1500.0) / PULSE_PER_DEGREE
    theta4 = (nominal_pulse(p, 4) - 1500.0) / PULSE_PER_DEGREE
    theta5 = 90.0 - (nominal_pulse(p, 5) - 1500.0) / PULSE_PER_DEGREE
    pitch = theta3 + theta5 - theta4
    shoulder = math.radians(theta5)
    forearm = math.radians(theta5 - theta4)
    tool = math.radians(pitch)
    radius = (
        LINK_2_CM * math.cos(shoulder)
        + LINK_3_CM * math.cos(forearm)
        + tool_length_cm * math.cos(tool)
    )
    height = (
        LINK_1_CM
        + LINK_2_CM * math.sin(shoulder)
        + LINK_3_CM * math.sin(forearm)
        + tool_length_cm * math.sin(tool)
    )
    if not all(math.isfinite(v) for v in (radius, height, pitch)):
        return None
    return ArmPoint(radius, height, pitch)


def project_floor_pixel(
    pose: Mapping[int | str, Any],
    *,
    nx: float,
    ny: float,
    max_range_m: float = 0.80,
) -> GroundProjection | None:
    """Project one normalized image point to z=0 in the robot base frame.

    nx, ny use image coordinates (0..1, top-left origin).  The returned frame is
    instantaneous robot-base-relative; it must not be treated as persistent
    world coordinates across chassis motion until odometry is integrated.
    """
    p = _pose_ints(pose)
    if any(servo not in p for servo in REQUIRED_SERVOS):
        return None
    if not (0.0 <= nx <= 1.0 and 0.0 <= ny <= 1.0):
        return None
    camera = forward_kinematics(p)
    if camera is None:
        return None
    camera_height_cm = camera.height_cm + CAMERA_Z_OFFSET_CM
    pixel_down = (ny - 0.5) * CAMERA_VFOV_DEG
    ray_pitch = camera.pitch_deg + CAMERA_PITCH_OFFSET_DEG - pixel_down
    if camera_height_cm <= 0.5 or not -88.0 <= ray_pitch <= -5.0:
        return None
    forward_from_camera_cm = camera_height_cm / math.tan(math.radians(-ray_pitch))
    ground_radius_cm = camera.radius_cm + forward_from_camera_cm
    if not 2.0 <= ground_radius_cm <= max_range_m * 100.0:
        return None
    pixel_left_deg = (0.5 - nx) * CAMERA_HFOV_DEG
    servo_left_deg = (p[6] - BASE_CENTER) / PULSE_PER_DEGREE
    bearing_deg = servo_left_deg + pixel_left_deg
    yaw = math.radians(bearing_deg)
    # Old controller used (left, forward); expose standard base (+x fwd,+y left).
    x_forward_m = ground_radius_cm * math.cos(yaw) / 100.0
    y_left_m = ground_radius_cm * math.sin(yaw) / 100.0
    return GroundProjection(
        x_forward_m=x_forward_m,
        y_left_m=y_left_m,
        z_ground_m=0.0,
        range_m=ground_radius_cm / 100.0,
        bearing_deg=bearing_deg,
        camera_height_m=camera_height_cm / 100.0,
        camera_radius_m=camera.radius_cm / 100.0,
        ray_pitch_deg=ray_pitch,
        image_contact_xy=(nx, ny),
    )


def project_detection(
    pose: Mapping[int | str, Any],
    detection: Mapping[str, Any] | None,
) -> GroundProjection | None:
    """Project a detector bbox using its lower-centre floor contact point."""
    if not isinstance(detection, Mapping) or detection.get("visible") is not True:
        return None
    try:
        cx = float(detection["cx"])
    except (KeyError, TypeError, ValueError):
        return None
    bbox = detection.get("bbox")
    try:
        if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
            bottom_y = float(bbox[3])
        else:
            bottom_y = float(detection["cy"])
    except (KeyError, TypeError, ValueError):
        return None
    bottom_y = min(0.995, max(0.005, bottom_y))
    return project_floor_pixel(pose, nx=cx, ny=bottom_y)


def metric_memory_entry(
    pose: Mapping[int | str, Any],
    detection: Mapping[str, Any] | None,
    *,
    pose_age_s: float | None = None,
    pose_stable: bool = False,
) -> dict[str, Any] | None:
    """Build a REAL spatial-memory patch if the commanded pose is trustworthy."""
    projection = project_detection(pose, detection)
    if projection is None:
        return None
    if pose_age_s is not None and (pose_age_s < 0 or pose_age_s > 10.0):
        return None
    # Do not claim calibrated persistent world coordinates while arm command is
    # still changing.  An unstable pose remains available to the image memory.
    if not pose_stable:
        return None
    return {
        "position_xy": [projection.x_forward_m, projection.y_left_m],
        "height_m": DEFAULT_BLOCK_HEIGHT_M / 2.0,
        "distance_m": projection.range_m,
        "bearing_deg": projection.bearing_deg,
        "metric_position_available": True,
        "calibrated": True,
        "reference_frame": "robot_base_at_observation",
        "coordinate_convention": "+x_forward,+y_left,+z_up",
        "calibration_id": projection.calibration_id,
        "calibration_source": CALIBRATION_SOURCE,
        "camera_height_m": projection.camera_height_m,
        "camera_radius_m": projection.camera_radius_m,
        "ray_pitch_deg": projection.ray_pitch_deg,
        "image_contact_xy": list(projection.image_contact_xy),
        "pose_age_s": pose_age_s,
        "pose_stable": True,
        "metric_source": "robot_camera_rgb+commanded_arm_fk",
    }
