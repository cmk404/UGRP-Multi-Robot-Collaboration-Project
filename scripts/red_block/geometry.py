"""Pure physical arm/camera geometry shared by REAL delivery skills.

No OpenCV, SSH, I2C, motors, or servos are touched here.  Constants and
formulas are inherited from physical_state_machine_reference.py, the pickup
controller tuned on ugrp1 in August 2026.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Mapping

from poses import BASE_CENTER, clamp_pulse


FRAME_WIDTH = 640.0
FRAME_HEIGHT = 480.0
LINK_1 = 9.30
LINK_2 = 6.50
LINK_3 = 6.20
GRIPPER_LINK = 10.00

# ugrp1 measured eye-in-hand camera calibration (2026-08-30/31).  The previous
# delivery estimator treated the raw fisheye image as a centred 48-degree
# pinhole and also forgot that controller FK is axle-relative, not floor-relative.
# Those two errors shortened a real ~20.8 cm destination to ~17.2 cm.
ROBOT_BASE_FLOOR_HEIGHT_CM = 3.25  # physical 65 mm wheel radius
CAMERA_LOCAL_X_CM = 6.70
CAMERA_LOCAL_Z_CM = 1.36
CAMERA_OPTICAL_PITCH_FROM_TOOL_DEG = 7.459176530462297
CAMERA_FX_PX = 619.5194540517232
CAMERA_FY_PX = 622.1654884818748
CAMERA_CX_PX = 287.68910608896766
CAMERA_CY_PX = 218.69686648455726
CAMERA_FISHEYE_D = (
    -0.0199819517209417,
    -0.1707593617153504,
    -0.2568318201202376,
    0.9650933421665181,
)
CAMERA_OPTICAL_CENTER_NX = CAMERA_CX_PX / FRAME_WIDTH
CAMERA_OPTICAL_CENTER_NY = CAMERA_CY_PX / FRAME_HEIGHT
# Compatibility names retained for callers/tests that report these values.
CAMERA_LINK_CM = CAMERA_LOCAL_X_CM
CAMERA_Z_OFFSET_CM = CAMERA_LOCAL_Z_CM
CAMERA_PITCH_OFFSET_DEG = CAMERA_OPTICAL_PITCH_FROM_TOOL_DEG
CAMERA_VFOV_DEG = 48.0
CAMERA_HFOV_DEG = math.degrees(
    2.0 * math.atan(math.tan(math.radians(CAMERA_VFOV_DEG / 2.0)) * 4.0 / 3.0)
)
PULSE_PER_DEGREE = 2000.0 / 180.0
SERVO_DEVIATION = {3: 54, 4: 53, 5: 89, 6: 64}

BLOCK_HALF_DEPTH_CM = 1.5
GRIPPER_TIP_PAST_CENTER_CM = 0.5
# Placement targets the centre of the carried cube over the centre of the
# destination cube.  The +0.5 cm TCP correction above belongs to pickup/grasp
# geometry and must not be reused for stacking; doing so leaves a 30-mm cube
# balanced on the destination edge.
PLACE_GRIP_CENTER_RADIAL_OFFSET_CM = 0.0
HOVER_HEIGHT_CM = 8.0
DEFAULT_BLOCK_HEIGHT_CM = 3.0
GRASP_HEIGHT_RATIO = 0.35
GRASP_HEIGHT_OFFSET_CM = -0.25
GRASP_HEIGHT_MIN_CM = 0.60
GRASP_HEIGHT_MAX_CM = 4.00
HOVER_CLEARANCE_ABOVE_BLOCK_CM = 4.0
GRASP_PITCH_PREFERRED_DEG = -66
HAND_EYE_YAW_OFFSET_PULSE = 0
HAND_EYE_LATERAL_OFFSET_CM = 0.0
PAN_MIN = 1300
PAN_MAX = 1700
CALIBRATED_FINGERTIP_RADIUS_MIN_CM = 14.5
CALIBRATED_FINGERTIP_RADIUS_MAX_CM = 18.0
PLACE_DROP_CLEARANCE_CM = 0.20


@dataclass(frozen=True)
class ArmPoint:
    radius_cm: float
    height_cm: float
    pitch_deg: float


@dataclass(frozen=True)
class BlockEstimate:
    radius_cm: float
    lateral_left_cm: float
    forward_cm: float
    yaw_left_deg: float
    camera_radius_cm: float
    camera_height_cm: float
    ray_pitch_deg: float
    block_height_cm: float
    nx: float
    ny: float


@dataclass(frozen=True)
class PlacePlan:
    target: BlockEstimate
    base_pulse: int
    hover_pose: dict[int, int]
    release_pose: dict[int, int]
    fingertip_radius_cm: float
    target_height_cm: float
    carried_height_cm: float
    release_height_cm: float
    hover_height_cm: float


def _nominal_pulse(pose: Mapping[int, int], servo: int) -> float:
    if servo not in pose:
        raise ValueError(f"missing servo {servo}")
    return float(pose[servo] - SERVO_DEVIATION.get(servo, 0))


def forward_kinematics(
    pose: Mapping[int, int],
    tool_length_cm: float = CAMERA_LINK_CM,
) -> ArmPoint:
    theta3 = (_nominal_pulse(pose, 3) - 1500.0) / PULSE_PER_DEGREE
    theta4 = (_nominal_pulse(pose, 4) - 1500.0) / PULSE_PER_DEGREE
    theta5 = 90.0 - (_nominal_pulse(pose, 5) - 1500.0) / PULSE_PER_DEGREE
    pitch = theta3 + theta5 - theta4
    shoulder = math.radians(theta5)
    forearm = math.radians(theta5 - theta4)
    tool = math.radians(pitch)
    radius = (
        LINK_2 * math.cos(shoulder)
        + LINK_3 * math.cos(forearm)
        + tool_length_cm * math.cos(tool)
    )
    height = (
        LINK_1
        + LINK_2 * math.sin(shoulder)
        + LINK_3 * math.sin(forearm)
        + tool_length_cm * math.sin(tool)
    )
    if not all(math.isfinite(v) for v in (radius, height, pitch)):
        raise ValueError("non-finite FK result")
    return ArmPoint(radius, height, pitch)


def _fisheye_undistort_normalized(px: float, py: float) -> tuple[float, float]:
    """Invert OpenCV's calibrated fisheye model without importing OpenCV.

    Returns ideal pinhole coordinates (x-right, y-down, z-forward=1).  The
    Newton solve mirrors cv2.fisheye.undistortPoints for this measured K/D.
    """
    xd = (float(px) - CAMERA_CX_PX) / CAMERA_FX_PX
    yd = (float(py) - CAMERA_CY_PX) / CAMERA_FY_PX
    theta_d = math.hypot(xd, yd)
    if theta_d <= 1e-12:
        return 0.0, 0.0
    theta = theta_d
    k1, k2, k3, k4 = CAMERA_FISHEYE_D
    for _ in range(12):
        t2 = theta * theta
        t4 = t2 * t2
        t6 = t4 * t2
        t8 = t4 * t4
        poly = 1.0 + k1*t2 + k2*t4 + k3*t6 + k4*t8
        f = theta * poly - theta_d
        derivative = 1.0 + 3.0*k1*t2 + 5.0*k2*t4 + 7.0*k3*t6 + 9.0*k4*t8
        if abs(derivative) <= 1e-12:
            break
        step = f / derivative
        theta -= step
        if abs(step) <= 1e-12:
            break
    radius = math.tan(theta)
    scale = radius / theta_d
    return xd * scale, yd * scale


def _camera_geometry(pose: Mapping[int, int]) -> tuple[tuple[float, float, float], tuple[tuple[float, float, float], ...], float]:
    """Return camera origin, gripper-frame basis and tool pitch in robot frame."""
    wrist = forward_kinematics(pose, 0.0)
    yaw = math.radians((float(pose[6]) - BASE_CENTER) / PULSE_PER_DEGREE)
    pitch = math.radians(wrist.pitch_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # Columns are gripper-frame +x (tool), +y (left), +z (tool-normal/up).
    ex = (cy * cp, sy * cp, sp)
    ey = (-sy, cy, 0.0)
    ez = (-cy * sp, -sy * sp, cp)
    wrist_origin = (
        wrist.radius_cm * cy,
        wrist.radius_cm * sy,
        ROBOT_BASE_FLOOR_HEIGHT_CM + wrist.height_cm,
    )
    origin = tuple(
        wrist_origin[i] + CAMERA_LOCAL_X_CM * ex[i] + CAMERA_LOCAL_Z_CM * ez[i]
        for i in range(3)
    )
    return origin, (ex, ey, ez), wrist.pitch_deg


def _camera_ray_robot(pose: Mapping[int, int], px: float, py: float) -> tuple[float, float, float]:
    """Convert one raw fisheye pixel into a robot-frame ray."""
    xu, yu = _fisheye_undistort_normalized(px, py)
    _origin, basis, _pitch = _camera_geometry(pose)
    ex, ey, ez = basis
    a = math.radians(CAMERA_OPTICAL_PITCH_FROM_TOOL_DEG)
    sa, ca = math.sin(a), math.cos(a)
    # Measured rigid camera orientation in the gripper frame.  MuJoCo/OpenCV
    # convention: image +x is camera +x, image +y is -camera +y, view is -z.
    cam_x = (0.0, -1.0, 0.0)
    cam_y = (-sa, 0.0, ca)
    cam_z = (-ca, 0.0, -sa)
    parent = (
        xu * cam_x[0] - yu * cam_y[0] - cam_z[0],
        xu * cam_x[1] - yu * cam_y[1] - cam_z[1],
        xu * cam_x[2] - yu * cam_y[2] - cam_z[2],
    )
    ray = tuple(parent[0] * ex[i] + parent[1] * ey[i] + parent[2] * ez[i] for i in range(3))
    norm = math.sqrt(sum(v*v for v in ray))
    if norm <= 1e-12:
        raise ValueError("degenerate camera ray")
    return tuple(v / norm for v in ray)


def undistorted_ray(px: float, py: float) -> tuple[float, float]:
    """Ideal pinhole coordinates (x-right, y-down, z-forward=1) of a raw pixel."""
    return _fisheye_undistort_normalized(float(px), float(py))


def _camera_rotation(pose: Mapping[int, int]) -> tuple[tuple[float, float, float], ...]:
    """Camera-to-robot rotation columns (X-right, Y-down, Z-forward)."""
    _origin, basis, _pitch = _camera_geometry(pose)
    ex, ey, ez = basis

    def to_robot(v: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(v[0] * ex[i] + v[1] * ey[i] + v[2] * ez[i] for i in range(3))

    a = math.radians(CAMERA_OPTICAL_PITCH_FROM_TOOL_DEG)
    sa, ca = math.sin(a), math.cos(a)
    cam_x = (0.0, -1.0, 0.0)
    cam_y = (-sa, 0.0, ca)
    cam_z = (-ca, 0.0, -sa)
    return (
        to_robot(cam_x),
        to_robot(tuple(-v for v in cam_y)),
        to_robot(tuple(-v for v in cam_z)),
    )


def _project_pixel(
    origin_cm: tuple[float, float, float],
    rot: tuple[tuple[float, float, float], ...],
    point_cm: tuple[float, float, float],
) -> tuple[float, float] | None:
    """Forward fisheye projection of a robot-frame point. None if behind camera."""
    rel = (
        point_cm[0] - origin_cm[0],
        point_cm[1] - origin_cm[1],
        point_cm[2] - origin_cm[2],
    )
    pc = tuple(sum(rot[c][i] * rel[i] for i in range(3)) for c in range(3))
    if pc[2] <= 1e-9:
        return None
    x, y = pc[0] / pc[2], pc[1] / pc[2]
    r = math.hypot(x, y)
    if r <= 1e-12:
        return CAMERA_CX_PX, CAMERA_CY_PX
    theta = math.atan(r)
    t2 = theta * theta
    k1, k2, k3, k4 = CAMERA_FISHEYE_D
    theta_d = theta * (1.0 + k1*t2 + k2*t2*t2 + k3*t2*t2*t2 + k4*t2*t2*t2*t2)
    scale = theta_d / r
    return CAMERA_FX_PX * x * scale + CAMERA_CX_PX, CAMERA_FY_PX * y * scale + CAMERA_CY_PX


def cube_range_estimate_cm(
    pose: Mapping[int, int],
    cx_px: float,
    cy_px: float,
    w_px: float,
    h_px: float,
) -> float | None:
    """Range (cm, robot frame) by fitting a 30-mm floor cube to a detection.

    The blob centre gives the bearing through the calibrated fisheye model;
    distance is the golden-section fit whose forward-projected cube corners
    best reproduce the detected pixel extent.  This removes the fronto-parallel
    bias of apparent-size ranging (corner-on/tilted views inflate the bbox, so
    plain pinhole sizing reads ~25% near at far range).  Returns None only for
    degenerate input or geometry.
    """
    if w_px <= 0 or h_px <= 0:
        return None
    try:
        origin, _basis, _pitch = _camera_geometry(pose)
        rot = _camera_rotation(pose)
        xu, yu = _fisheye_undistort_normalized(float(cx_px), float(cy_px))
    except (ValueError, KeyError, TypeError):
        return None
    norm = math.sqrt(xu * xu + yu * yu + 1.0)
    if norm <= 1e-12:
        return None
    direction = tuple(
        (rot[0][i] * xu + rot[1][i] * yu + rot[2][i]) / norm for i in range(3)
    )
    half = BLOCK_HALF_DEPTH_CM

    def extent(dist_cm: float) -> tuple[float, float] | None:
        cx = origin[0] + direction[0] * dist_cm
        cyy = origin[1] + direction[1] * dist_cm
        cz = origin[2] + direction[2] * dist_cm
        xs: list[float] = []
        ys: list[float] = []
        for dx in (-half, half):
            for dy in (-half, half):
                for z in (cz - half, cz + half):
                    q = _project_pixel(origin, rot, (cx + dx, cyy + dy, z))
                    if q is None:
                        return None
                    xs.append(q[0])
                    ys.append(q[1])
        return max(xs) - min(xs), max(ys) - min(ys)

    def loss(dist_cm: float) -> float:
        e = extent(dist_cm)
        if e is None:
            return float("inf")
        return (e[0] - w_px) ** 2 + (e[1] - h_px) ** 2

    lo, hi = 5.0, 300.0
    inv_phi = (math.sqrt(5.0) - 1.0) / 2.0
    c = hi - inv_phi * (hi - lo)
    d = lo + inv_phi * (hi - lo)
    for _ in range(25):
        if loss(c) < loss(d):
            hi = d
        else:
            lo = c
        c = hi - inv_phi * (hi - lo)
        d = lo + inv_phi * (hi - lo)
    best = (lo + hi) / 2.0
    if not math.isfinite(loss(best)):
        return None
    return best


def estimate_block(pose: Mapping[int, int], blob) -> BlockEstimate | None:
    """Estimate block centre from calibrated raw-fisheye floor contact geometry."""
    if any(servo not in pose for servo in (3, 4, 5, 6)):
        return None
    origin, _basis, tool_pitch_deg = _camera_geometry(pose)
    bottom_px = float(blob.cx)
    bottom_py = min(FRAME_HEIGHT - 1.0, float(blob.cy) + float(blob.height) / 2.0)
    ray = _camera_ray_robot(pose, bottom_px, bottom_py)
    if origin[2] <= 0.5 or ray[2] >= -1e-6:
        return None
    t = -origin[2] / ray[2]
    if t <= 0 or not math.isfinite(t):
        return None
    ground_x = origin[0] + t * ray[0]
    ground_y = origin[1] + t * ray[1]
    ground_radius = math.hypot(ground_x, ground_y)
    if not 1.0 <= ground_radius <= 70.0:
        return None

    # The detector's lower silhouette is the near floor edge of the 30-mm cube.
    # Move half a cube depth away from the robot to obtain the cube centre.
    ux, uy = ground_x / ground_radius, ground_y / ground_radius
    forward = ground_x + BLOCK_HALF_DEPTH_CM * ux
    lateral = ground_y + BLOCK_HALF_DEPTH_CM * uy
    radius = math.hypot(forward, lateral)
    if not 3.0 <= radius <= 70.0:
        return None
    yaw_left_deg = math.degrees(math.atan2(lateral, forward))

    # Estimate height from the top silhouette on the same front-face plane.
    top_py = max(0.0, float(blob.cy) - float(blob.height) / 2.0)
    top_ray = _camera_ray_robot(pose, float(blob.cx), top_py)
    plane_range = ground_radius
    horizontal_along = top_ray[0] * ux + top_ray[1] * uy
    camera_along = origin[0] * ux + origin[1] * uy
    measured_height = DEFAULT_BLOCK_HEIGHT_CM
    if horizontal_along > 1e-6:
        top_t = (plane_range - camera_along) / horizontal_along
        candidate_height = origin[2] + top_t * top_ray[2]
        if 0.4 <= candidate_height <= 12.0:
            measured_height = candidate_height
    block_height = max(0.8, min(10.0, measured_height))

    horizontal_ray = math.hypot(ray[0], ray[1])
    ray_pitch = math.degrees(math.atan2(ray[2], horizontal_ray))
    camera_radius = math.hypot(origin[0], origin[1])
    return BlockEstimate(
        radius_cm=radius,
        lateral_left_cm=lateral,
        forward_cm=forward,
        yaw_left_deg=yaw_left_deg,
        camera_radius_cm=camera_radius,
        camera_height_cm=origin[2],
        ray_pitch_deg=ray_pitch,
        block_height_cm=block_height,
        nx=float(blob.nx),
        ny=float(blob.ny),
    )


def median_block_estimate(samples: list[BlockEstimate]) -> BlockEstimate:
    if not samples:
        raise ValueError("at least one block estimate is required")
    return BlockEstimate(
        radius_cm=statistics.median(item.radius_cm for item in samples),
        lateral_left_cm=statistics.median(item.lateral_left_cm for item in samples),
        forward_cm=statistics.median(item.forward_cm for item in samples),
        yaw_left_deg=statistics.median(item.yaw_left_deg for item in samples),
        camera_radius_cm=statistics.median(item.camera_radius_cm for item in samples),
        camera_height_cm=statistics.median(item.camera_height_cm for item in samples),
        ray_pitch_deg=statistics.median(item.ray_pitch_deg for item in samples),
        block_height_cm=statistics.median(item.block_height_cm for item in samples),
        nx=statistics.median(item.nx for item in samples),
        ny=statistics.median(item.ny for item in samples),
    )


def median_absolute_deviation(values: list[float]) -> float:
    if not values:
        raise ValueError("at least one value is required")
    med = statistics.median(values)
    return float(statistics.median(abs(value - med) for value in values))


def inverse_kinematics_at_pitch(
    radius_cm: float,
    height_cm: float,
    pitch_deg: float,
) -> dict[int, int] | None:
    alpha = math.radians(pitch_deg)
    horizontal = radius_cm - GRIPPER_LINK * math.cos(alpha)
    vertical = height_cm - LINK_1 - GRIPPER_LINK * math.sin(alpha)
    diagonal = math.hypot(horizontal, vertical)
    if diagonal <= 1e-9 or not abs(LINK_2 - LINK_3) <= diagonal <= LINK_2 + LINK_3:
        return None
    elbow_cos = (LINK_2**2 + LINK_3**2 - diagonal**2) / (2.0 * LINK_2 * LINK_3)
    shoulder_cos = (diagonal**2 + LINK_2**2 - LINK_3**2) / (2.0 * LINK_2 * diagonal)
    elbow_cos = max(-1.0, min(1.0, elbow_cos))
    shoulder_cos = max(-1.0, min(1.0, shoulder_cos))
    theta4 = 180.0 - math.degrees(math.acos(elbow_cos))
    line_cos = max(-1.0, min(1.0, horizontal / diagonal))
    line_angle = math.acos(line_cos)
    theta5 = math.degrees(
        (-1.0 if vertical < 0.0 else 1.0) * line_angle + math.acos(shoulder_cos)
    )
    theta3 = pitch_deg - theta5 + theta4
    pose = {
        3: int(round(theta3 * PULSE_PER_DEGREE + 1500)) + SERVO_DEVIATION[3],
        4: int(round(theta4 * PULSE_PER_DEGREE + 1500)) + SERVO_DEVIATION[4],
        5: int(round(1500 + (90.0 - theta5) * PULSE_PER_DEGREE)) + SERVO_DEVIATION[5],
    }
    if any(not 550 <= pulse <= 2450 for pulse in pose.values()):
        return None
    return pose


def solve_ik(radius_cm: float, height_cm: float, preferred_pitch_deg: float) -> dict[int, int]:
    candidates: list[tuple[float, dict[int, int]]] = []
    for pitch in range(-85, -39):
        pose = inverse_kinematics_at_pitch(radius_cm, height_cm, float(pitch))
        if pose is None:
            continue
        candidates.append((abs(pitch - preferred_pitch_deg), pose))
    if not candidates:
        raise RuntimeError(
            f"IK has no safe solution for radius={radius_cm:.2f}, height={height_cm:.2f}"
        )
    return min(candidates, key=lambda item: item[0])[1]


def solve_hover_ik(radius_cm: float, desired_height_cm: float) -> tuple[dict[int, int], float]:
    height = desired_height_cm
    while height >= HOVER_HEIGHT_CM - 1e-6:
        try:
            return solve_ik(radius_cm, height, preferred_pitch_deg=-55), height
        except RuntimeError:
            height -= 0.25
    raise RuntimeError(
        f"no safe hover IK from {desired_height_cm:.2f}cm down to {HOVER_HEIGHT_CM:.2f}cm"
    )


def carried_grasp_offset_cm(carried_height_cm: float = DEFAULT_BLOCK_HEIGHT_CM) -> float:
    height = max(0.8, min(10.0, float(carried_height_cm)))
    return max(
        GRASP_HEIGHT_MIN_CM,
        min(GRASP_HEIGHT_MAX_CM, height * GRASP_HEIGHT_RATIO + GRASP_HEIGHT_OFFSET_CM),
    )


def calculate_place_plan(
    target: BlockEstimate,
    *,
    carried_height_cm: float = DEFAULT_BLOCK_HEIGHT_CM,
    drop_clearance_cm: float = PLACE_DROP_CLEARANCE_CM,
) -> PlacePlan:
    """Build a conservative stack release plan inside the calibrated reach envelope."""
    fingertip_radius = target.radius_cm + PLACE_GRIP_CENTER_RADIAL_OFFSET_CM
    if not CALIBRATED_FINGERTIP_RADIUS_MIN_CM <= fingertip_radius <= CALIBRATED_FINGERTIP_RADIUS_MAX_CM:
        raise RuntimeError(
            f"measured fingertip radius {fingertip_radius:.2f}cm is outside calibrated place envelope"
        )
    corrected_lateral = target.lateral_left_cm - HAND_EYE_LATERAL_OFFSET_CM
    coordinate_yaw_deg = math.degrees(math.atan2(corrected_lateral, max(0.1, target.forward_cm)))
    base_pulse = clamp_pulse(
        int(round(BASE_CENTER + coordinate_yaw_deg * PULSE_PER_DEGREE)) + HAND_EYE_YAW_OFFSET_PULSE,
        PAN_MIN,
        PAN_MAX,
    )
    target_height = max(0.8, min(10.0, target.block_height_cm))
    carried_height = max(0.8, min(10.0, float(carried_height_cm)))
    release_height = target_height + carried_grasp_offset_cm(carried_height) + max(0.0, drop_clearance_cm)
    hover_requested = max(
        HOVER_HEIGHT_CM,
        target_height + carried_height + HOVER_CLEARANCE_ABOVE_BLOCK_CM,
    )
    hover_pose, hover_height = solve_hover_ik(fingertip_radius, hover_requested)
    release_pose = solve_ik(
        fingertip_radius,
        release_height,
        preferred_pitch_deg=GRASP_PITCH_PREFERRED_DEG,
    )
    return PlacePlan(
        target=target,
        base_pulse=base_pulse,
        hover_pose=hover_pose,
        release_pose=release_pose,
        fingertip_radius_cm=fingertip_radius,
        target_height_cm=target_height,
        carried_height_cm=carried_height,
        release_height_cm=release_height,
        hover_height_cm=hover_height,
    )
