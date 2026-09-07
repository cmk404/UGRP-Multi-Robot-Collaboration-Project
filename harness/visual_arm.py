"""Pure calibrated camera/arm transforms for a camera-only MasterPi actor.

All inputs are robot-owned servo commands/encoders and camera-derived metric
coordinates.  No simulator world, body pose, object pose, depth buffer,
contact, inverse-dynamics, or weld state is accepted or read here.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass


# Physical geometry and the centered provisional mount used by production SIM.
LINK_1_CM = 9.30
LINK_2_CM = 6.50
LINK_3_CM = 6.20
GRIPPER_LINK_CM = 10.00
ROBOT_BASE_FLOOR_HEIGHT_CM = 3.25
CAMERA_LOCAL_X_CM = 6.70
CAMERA_LOCAL_Z_CM = 1.36
CAMERA_OPTICAL_PITCH_FROM_TOOL_DEG = 7.459176530462297
PULSE_PER_DEGREE = 2000.0 / 180.0
SERVO_DEVIATION = {3: 54, 4: 53, 5: 89, 6: 64}
BASE_CENTER = 1500
SAFE_PULSE_MIN = 500
SAFE_PULSE_MAX = 2500
CALIBRATED_PAN_MIN = 1300
CALIBRATED_PAN_MAX = 1700
CALIBRATED_GRASP_RADIUS_CM = (14.5, 18.0)


@dataclass(frozen=True)
class ToolPose:
    """Tool point in the robot floor frame (+x forward, +y left, +z up)."""

    x_m: float
    y_m: float
    z_m: float
    yaw_left_deg: float
    pitch_deg: float


def tool_pose(
    servo_pose: Mapping[int | str, int | float],
    *,
    tool_length_cm: float = GRIPPER_LINK_CM,
) -> ToolPose:
    """Forward kinematics for the gripper/tool axis from owned servo PWM."""
    pose = _required_pose(servo_pose)
    length = _finite("tool_length_cm", tool_length_cm)
    if length < 0.0:
        raise ValueError("tool_length_cm must be non-negative")
    theta3 = (_nominal(pose, 3) - 1500.0) / PULSE_PER_DEGREE
    theta4 = (_nominal(pose, 4) - 1500.0) / PULSE_PER_DEGREE
    theta5 = 90.0 - (_nominal(pose, 5) - 1500.0) / PULSE_PER_DEGREE
    pitch = theta3 + theta5 - theta4
    shoulder = math.radians(theta5)
    forearm = math.radians(theta5 - theta4)
    tool = math.radians(pitch)
    radius = (
        LINK_2_CM * math.cos(shoulder)
        + LINK_3_CM * math.cos(forearm)
        + length * math.cos(tool)
    )
    height = (
        ROBOT_BASE_FLOOR_HEIGHT_CM
        + LINK_1_CM
        + LINK_2_CM * math.sin(shoulder)
        + LINK_3_CM * math.sin(forearm)
        + length * math.sin(tool)
    )
    yaw_deg = (pose[6] - BASE_CENTER) / PULSE_PER_DEGREE
    yaw = math.radians(yaw_deg)
    return ToolPose(
        radius * math.cos(yaw) / 100.0,
        radius * math.sin(yaw) / 100.0,
        height / 100.0,
        yaw_deg,
        pitch,
    )


def camera_optical_to_robot_base(
    servo_pose: Mapping[int | str, int | float],
    optical_xyz_m: Sequence[int | float],
) -> tuple[float, float, float]:
    """Transform camera optical XYZ to the robot floor frame.

    Optical coordinates follow OpenCV: +x image-right, +y image-down, +z
    forward.  The XYZ value must come from RGB geometry (for example known-size
    ranging); this function does not manufacture depth from a pixel.
    """
    if len(optical_xyz_m) != 3:
        raise ValueError("optical_xyz_m must contain x, y, z")
    ox, oy, oz = (_finite("optical coordinate", value) for value in optical_xyz_m)
    pose = _required_pose(servo_pose)
    wrist = tool_pose(pose, tool_length_cm=0.0)
    yaw = math.radians(wrist.yaw_left_deg)
    pitch = math.radians(wrist.pitch_deg)
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    ex = (cy * cp, sy * cp, sp)       # tool forward
    ey = (-sy, cy, 0.0)               # tool left
    ez = (-cy * sp, -sy * sp, cp)     # tool up
    camera_origin_cm = tuple(
        value * 100.0 + CAMERA_LOCAL_X_CM * ex[i] + CAMERA_LOCAL_Z_CM * ez[i]
        for i, value in enumerate((wrist.x_m, wrist.y_m, wrist.z_m))
    )
    angle = math.radians(CAMERA_OPTICAL_PITCH_FROM_TOOL_DEG)
    sa, ca = math.sin(angle), math.cos(angle)
    # Camera columns in the gripper frame: image-right, image-down, forward.
    camera_axes = ((0.0, -1.0, 0.0), (sa, 0.0, -ca), (ca, 0.0, sa))

    def to_base(axis: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(axis[0] * ex[i] + axis[1] * ey[i] + axis[2] * ez[i] for i in range(3))

    axes = tuple(to_base(axis) for axis in camera_axes)
    optical_cm = (ox * 100.0, oy * 100.0, oz * 100.0)
    return tuple(
        (camera_origin_cm[i] + sum(optical_cm[j] * axes[j][i] for j in range(3))) / 100.0
        for i in range(3)
    )


def camera_to_base(
    optical_xyz: Sequence[int | float],
    pose: Mapping[int | str, int | float],
) -> tuple[float, float, float]:
    """Runner-facing alias for :func:`camera_optical_to_robot_base`."""
    return camera_optical_to_robot_base(pose, optical_xyz)


def camera_extrinsics(
    pose: Mapping[int | str, int | float],
) -> tuple[tuple[float, float, float], tuple[tuple[float, float, float], ...]]:
    """Return camera origin and OpenCV optical axes in the robot floor frame."""
    origin = camera_to_base((0.0, 0.0, 0.0), pose)
    axes = []
    for optical in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        endpoint = camera_to_base(optical, pose)
        axes.append(tuple(endpoint[i] - origin[i] for i in range(3)))
    return origin, tuple(axes)


def solve_grip_site_ik(
    target_xyz_m: Sequence[int | float],
    *,
    preferred_pitch_deg: float = -66.0,
    calibrated_grasp_only: bool = True,
) -> dict[int, int]:
    """Solve servo 3/4/5 and yaw servo 6 for a floor-frame grip-site target.

    The supported grasp orientation is radial: the tool yaw points from the
    arm axis toward the target and pitch is searched from -90 through -40
    degrees.  Cube face orientation cannot be inferred from target XYZ alone.
    """
    if len(target_xyz_m) != 3:
        raise ValueError("target_xyz_m must contain x, y, z")
    x, y, z = (_finite("target coordinate", value) for value in target_xyz_m)
    preferred = _finite("preferred_pitch_deg", preferred_pitch_deg)
    radius_cm = math.hypot(x, y) * 100.0
    if calibrated_grasp_only and not CALIBRATED_GRASP_RADIUS_CM[0] <= radius_cm <= CALIBRATED_GRASP_RADIUS_CM[1]:
        raise ValueError("target radius is outside the calibrated 14.5..18.0 cm grasp envelope")
    yaw_deg = math.degrees(math.atan2(y, x))
    pan = int(round(BASE_CENTER + yaw_deg * PULSE_PER_DEGREE))
    if not CALIBRATED_PAN_MIN <= pan <= CALIBRATED_PAN_MAX:
        raise ValueError("target yaw is outside the calibrated servo-6 grasp sector")
    height_axle_cm = z * 100.0 - ROBOT_BASE_FLOOR_HEIGHT_CM
    candidates: list[tuple[float, dict[int, int]]] = []
    for pitch in range(-90, -39):
        arm = _ik_at_pitch(radius_cm, height_axle_cm, float(pitch))
        if arm is not None:
            arm[6] = pan
            candidates.append((abs(float(pitch) - preferred), arm))
    if not candidates:
        raise ValueError("target has no safe physical arm IK solution")
    return min(candidates, key=lambda item: item[0])[1]


def solve_grip_ik(
    x_forward_m: int | float,
    y_left_m: int | float,
    z_above_floor_m: int | float,
    pitch_deg: int | float = -90.0,
) -> dict[int, int]:
    """Runner-facing calibrated grasp IK for a metric robot-frame target."""
    return solve_grip_site_ik(
        (x_forward_m, y_left_m, z_above_floor_m),
        preferred_pitch_deg=float(pitch_deg),
    )


def forward_grip(pose: Mapping[int | str, int | float]) -> tuple[float, float, float]:
    """Return the grip-site XYZ in the robot floor frame."""
    result = tool_pose(pose, tool_length_cm=GRIPPER_LINK_CM)
    return result.x_m, result.y_m, result.z_m


def _ik_at_pitch(radius_cm: float, height_cm: float, pitch_deg: float) -> dict[int, int] | None:
    alpha = math.radians(pitch_deg)
    horizontal = radius_cm - GRIPPER_LINK_CM * math.cos(alpha)
    vertical = height_cm - LINK_1_CM - GRIPPER_LINK_CM * math.sin(alpha)
    diagonal = math.hypot(horizontal, vertical)
    if diagonal <= 1e-9 or not abs(LINK_2_CM - LINK_3_CM) <= diagonal <= LINK_2_CM + LINK_3_CM:
        return None
    elbow_cos = _clip((LINK_2_CM**2 + LINK_3_CM**2 - diagonal**2) / (2.0 * LINK_2_CM * LINK_3_CM))
    shoulder_cos = _clip((diagonal**2 + LINK_2_CM**2 - LINK_3_CM**2) / (2.0 * LINK_2_CM * diagonal))
    theta4 = 180.0 - math.degrees(math.acos(elbow_cos))
    line_angle = math.acos(_clip(horizontal / diagonal))
    theta5 = math.degrees((-1.0 if vertical < 0.0 else 1.0) * line_angle + math.acos(shoulder_cos))
    theta3 = pitch_deg - theta5 + theta4
    result = {
        3: int(round(theta3 * PULSE_PER_DEGREE + 1500)) + SERVO_DEVIATION[3],
        4: int(round(theta4 * PULSE_PER_DEGREE + 1500)) + SERVO_DEVIATION[4],
        5: int(round(1500 + (90.0 - theta5) * PULSE_PER_DEGREE)) + SERVO_DEVIATION[5],
    }
    return result if all(SAFE_PULSE_MIN <= pulse <= SAFE_PULSE_MAX for pulse in result.values()) else None


def _required_pose(values: Mapping[int | str, int | float]) -> dict[int, float]:
    if not isinstance(values, Mapping):
        raise ValueError("servo_pose must be a mapping")
    result: dict[int, float] = {}
    for servo in (3, 4, 5, 6):
        value = values.get(servo, values.get(str(servo)))
        result[servo] = _finite(f"servo {servo}", value)
        if not SAFE_PULSE_MIN <= result[servo] <= SAFE_PULSE_MAX:
            raise ValueError(f"servo {servo} is outside the physical PWM range")
    return result


def _nominal(pose: Mapping[int, float], servo: int) -> float:
    return pose[servo] - SERVO_DEVIATION[servo]


def _finite(name: str, value: int | float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _clip(value: float) -> float:
    return max(-1.0, min(1.0, value))
