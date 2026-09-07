"""Physics-fidelity v2 core for MasterPi.

This module is intentionally separate from the production demo simulator until
real-robot calibration gates pass.  It fixes the largest structural errors of
`MasterPiPhysicsWorld`:

* free 6-DoF chassis instead of world-frame x/y/yaw slide joints;
* four explicit wheel hinge joints and physical wheel/floor contacts;
* motor-command -> reduced mecanum wrench dynamics (ABAB wheel layout);
* explicit 1.10 kg total robot mass instead of mass inferred from visual geoms;
* 4-DOF+gripper arm geometry using the physical pickup-controller link lengths;
* measured ugrp1 fisheye intrinsics + rigid hand-eye transform for robot_cam;
* simplified chassis/arm collision geometry enabled against the world/blocks.

The mecanum traction model is a *reduced dynamics model*, not a claim that each
45-degree passive roller is geometrically simulated.  Its force, damping,
latency, slip and inertial distribution remain calibration parameters.  Do not
use this class for sim-to-real training until those parameters have been fitted
to measured MasterPi trajectories and the fidelity gate says so.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Mapping, Sequence

import mujoco
import numpy as np
from sim.calibration_schema import CALIBRATABLE_DYNAMICS, CALIBRATABLE_HARDWARE, REQUIRED_VALIDATED_PARAMETERS
from sim.masterpi_camera_profile import (
    CAMERA_CALIBRATION_ID,
    CAMERA_MOUNT_STATUS,
    CAMERA_FX_PX,
    CAMERA_FY_PX,
    CAMERA_CX_PX,
    CAMERA_CY_PX,
    CAMERA_PINHOLE_HFOV_DEG,
    CAMERA_PINHOLE_VFOV_DEG,
    CAMERA_LOCAL_POS_M,
    CAMERA_LOCAL_QUAT_WXYZ,
    mujoco_pixel_intrinsic,
    raw_fisheye_remap,
)
from sim.masterpi_geometry import (
    OFFICIAL_TOTAL_MASS_KG,
    NOMINAL_WHEEL_RADIUS_M,
    NOMINAL_WHEELBASE_M,
    NOMINAL_TRACK_M,
    NOMINAL_WHEEL_WIDTH_M,
    NOMINAL_DECK_LENGTH_M,
    NOMINAL_DECK_WIDTH_M,
    NOMINAL_DECK_TOP_FROM_FLOOR_M,
    NOMINAL_BODY_TOP_FROM_FLOOR_M,
    NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M,
    NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M,
    NOMINAL_LOWER_BODY_HEIGHT_M,
    NOMINAL_ULTRASONIC_X_M,
    NOMINAL_PI_CAGE_CENTER_X_M,
    NOMINAL_PI_CAGE_HALF_LENGTH_M,
    NOMINAL_PI_CAGE_HALF_WIDTH_M,
    NOMINAL_PI_BOARD_HALF_LENGTH_M,
    NOMINAL_PI_BOARD_HALF_WIDTH_M,
    NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M,
    NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M,
    NOMINAL_PI_BOARD_Z_FROM_FLOOR_M,
    NOMINAL_PI_HEATSINK_Z_FROM_FLOOR_M,
    NOMINAL_EXPANSION_BOARD_Z_FROM_FLOOR_M,
    NOMINAL_YAW_AXIS_FROM_BASE_M,
    NOMINAL_YAW_TO_SHOULDER_M,
    REAL_CAPTURE_CAMERA_LINK_M,
    REAL_CAPTURE_CAMERA_LOCAL_Z_M,
    REAL_CAPTURE_CAMERA_PITCH_OFFSET_DEG,
    SERVO_BODY_HALF,
    MICRO_SERVO_BODY_HALF,
    ARM_PLATE_HALF_WIDTH_M,
    ARM_PLATE_SIDE_OFFSET_M,
    ARM_HORN_SIDE_OFFSET_M,
    ARM_HORN_HALF_THICKNESS_M,
    ARM_PLATE_THICKNESS_M,
    ARM_LINK_HALF_HEIGHT_M,
    WRIST_LINK_HALF_HEIGHT_M,
    ARM_LINK_RAIL_Z_OFFSET_M,
    WRIST_LINK_RAIL_Z_OFFSET_M,
    FINGER_HALF_LENGTH_M,
    FINGER_HALF_WIDTH_M,
    FINGER_HALF_HEIGHT_M,
    GRIPPER_PAD_HALF_LENGTH_M,
    GRIPPER_PAD_HALF_WIDTH_M,
    GRIPPER_PAD_HALF_HEIGHT_M,
)

from harness.real_geometry import (
    CAMERA_VFOV_DEG,
    DEFAULT_BLOCK_HEIGHT_M,
    LINK_1_CM,
    LINK_2_CM,
    LINK_3_CM,
    CAMERA_LINK_CM,
    CAMERA_Z_OFFSET_CM,
    PULSE_PER_DEGREE,
    SERVO_DEVIATION,
)

SCENE_PATH = Path(__file__).with_name("masterpi_scene.xml")
WHEEL_RADIUS_M = NOMINAL_WHEEL_RADIUS_M
WHEELBASE_M = NOMINAL_WHEELBASE_M
TRACK_M = NOMINAL_TRACK_M
TARGET_BLOCK_SIDE_M = float(DEFAULT_BLOCK_HEIGHT_M)
TARGET_BLOCK_HALF_M = TARGET_BLOCK_SIDE_M / 2.0
GRIPPER_MAX_CLOSE_M = 0.016
# A grasp counts as physically lifted only when the cube centre is at least one
# full cube side above the floor (bottom clearance >= half the cube side) while
# bilateral finger contact is still present.
TARGET_BLOCK_LIFT_CENTER_M = TARGET_BLOCK_SIDE_M
# Hiwonder's 30 mm colour targets are lightweight foam/sponge blocks.  The old
# 50 g placeholder implied ~1850 kg/m^3 (stone/glass-like) and made otherwise
# stable REAL-controller grasps fall out during the lift.  Keep this explicitly
# provisional until the user's physical cube is weighed; calibration remains
# invalid/structural-only meanwhile.
PROVISIONAL_TARGET_BLOCK_MASS_KG = 0.005

# Provisional dynamics parameters.  These are intentionally collected in one
# place because they must be replaced by measured values before training.
MOTOR_TIME_CONSTANT_S = 0.085
MAX_WHEEL_RAD_S = 12.0
MAX_FORWARD_FORCE_N = 2.2
MAX_LATERAL_FORCE_N = 1.65
MAX_YAW_TORQUE_NM = 0.12
LINEAR_DAMPING_N_PER_MPS = 1.4
YAW_DAMPING_NM_PER_RADPS = 0.08
# The reduced mecanum model applies calibrated forward/lateral/yaw traction as
# a body wrench below.  The invisible cylinders are support/collision proxies,
# not a second tyre model.  Giving them ordinary cylinder sliding friction
# double-counts traction and makes lateral motion unrealistically weak because
# a cylinder must scrub sideways.  Keep only a tiny numerical support friction;
# actual drivetrain response remains owned by the explicit wrench parameters.
REDUCED_MECANUM_SUPPORT_FRICTION = 0.001
CALIBRATION_FILE = Path(__file__).with_name("masterpi_dynamics_calibration.json")
def load_fitted_parameters(path: Path = CALIBRATION_FILE) -> tuple[dict[str, float], str]:
    """Load finite REAL-measured/fitted hardware parameters from the manifest.

    Fitted values may be used while iterating on the twin, but only the final
    validator is allowed to set ``validated=true``. A manifest claiming
    validation without every hardware parameter remains unvalidated here.
    """
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}, "STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED"
    params = manifest.get("parameters") if isinstance(manifest, dict) else None
    if not isinstance(params, dict):
        return {}, "STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED"
    fitted: dict[str, float] = {}
    for key in REQUIRED_VALIDATED_PARAMETERS:
        value = params.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            fitted[key] = float(value)
    if not fitted:
        return {}, "STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED"
    complete = all(key in fitted for key in REQUIRED_VALIDATED_PARAMETERS)
    if manifest.get("validated") is True and complete:
        return fitted, "REAL_CALIBRATED_VALIDATED"
    return fitted, "REAL_FITTED_UNVALIDATED"


# Backward-compatible boundary used by earlier calibration tests/tools.
def load_fitted_dynamics(path: Path = CALIBRATION_FILE) -> tuple[dict[str, float], str]:
    fitted, status = load_fitted_parameters(path)
    return {k: v for k, v in fitted.items() if k in CALIBRATABLE_DYNAMICS}, status

def load_fitted_hardware(path: Path = CALIBRATION_FILE) -> tuple[dict[str, float], str]:
    fitted, status = load_fitted_parameters(path)
    return {k: v for k, v in fitted.items() if k in CALIBRATABLE_HARDWARE}, status
# The physical MasterPi's geared driveline resists back-driving when commands
# return to zero. Without this brake the free-body base rolls several cm from
# arm reaction alone, which is impossible for the physical pickup controller.
STOP_LINEAR_DAMPING_N_PER_MPS = 18.0
STOP_YAW_DAMPING_NM_PER_RADPS = 0.80
# Parallel-jaw grip stiffness is provisional until measured on hardware.  The
# previous kp=220 produced only ~0.5-0.8 N at the task cube and allowed a
# physically lifted block to fall onto one jaw.  ~450 keeps contact forces in a
# low-single-newton regime while preserving real MuJoCo contact/slip dynamics.
# Provisional actuator stiffness for the commanded closed-gripper position.
# 450 N/m produced only ~0.24 N/finger in the seed-11 grasp and released a
# 5 g foam cube under modest chassis yaw, unlike the real position-holding
# micro-servo. This remains explicitly uncalibrated until measured on hardware.
GRIPPER_POSITION_KP = 1200.0
GRIPPER_JAW_DAMPING = 1.2

# Simulator-normalized wheel space (front-left, front-right, rear-left, rear-right).
# These are kinematic signs inside MuJoCo, NOT raw MasterPi MotorTransport signs.
# The physical Pi has board/motor wiring inversions encoded in masterpi_control.DRIVE_MAP.
FORWARD_PATTERN = np.array([1.0, 1.0, 1.0, 1.0])
LEFT_PATTERN = np.array([-1.0, 1.0, 1.0, -1.0])
YAW_LEFT_PATTERN = np.array([-1.0, 1.0, -1.0, 1.0])

# The canonical scene historically contained a decorative warehouse.  It is not
# part of the user's physical MasterPi setup and must not become a SIM-only cue.
NONREAL_WORKCELL_GEOMS = {
    "wall_back", "wall_left", "wall_right_short",
    "rack_post_a", "rack_post_b", "rack_post_c", "rack_post_d",
    "rack_shelf_low", "rack_shelf_high", "workbench",
    "delivery_blue", "delivery_yellow", "crate_blue", "crate_yellow",
}


def _mecanum_hub_xml(prefix: str, count: int = 8) -> str:
    """Visual-only grey centre + radial spokes of the real MasterPi wheel hub."""
    lines = [
        f'<geom name="{prefix}_hub_visual" type="cylinder" size=".009 {NOMINAL_WHEEL_WIDTH_M*.50:.6f}" '
        'euler="1.5708 0 0" material="hub" mass="0" contype="0" conaffinity="0"/>'
    ]
    r0 = 0.007
    r1 = WHEEL_RADIUS_M * 0.67
    for i in range(count):
        phi = 2.0 * math.pi * i / count
        p0 = (r0 * math.cos(phi), 0.0, r0 * math.sin(phi))
        p1 = (r1 * math.cos(phi), 0.0, r1 * math.sin(phi))
        lines.append(
            f'<geom name="{prefix}_hub_spoke_{i}" type="capsule" '
            f'fromto="{p0[0]:.6f} {p0[1]:.6f} {p0[2]:.6f} '
            f'{p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}" size=".0024" '
            'material="hub" mass="0" contype="0" conaffinity="0"/>'
        )
    return "\n    ".join(lines)


def _mecanum_rollers_xml(prefix: str, handedness: int, count: int = 8) -> str:
    """Visual-only passive rollers around one wheel collision cylinder.

    The reduced mecanum wrench model remains the traction model.  These capsules
    make the hardware silhouette and wheel rotation legible without introducing
    dozens of small contact bodies that would destabilize the task simulation.
    """
    lines: list[str] = []
    r = WHEEL_RADIUS_M * 0.84
    half = 0.0125
    for i in range(count):
        phi = 2.0 * math.pi * i / count
        cx = r * math.cos(phi)
        cz = r * math.sin(phi)
        # Roller axis mixes wheel-axle and circumferential tangent directions,
        # giving the characteristic +/-45 degree ABAB mecanum appearance.
        tx, tz = -math.sin(phi), math.cos(phi)
        ay = 0.38
        at = 0.925 * float(handedness)
        norm = math.sqrt(ay * ay + at * at)
        ay /= norm
        at /= norm
        ax = at * tx
        az = at * tz
        p0 = (cx - half * ax, -half * ay, cz - half * az)
        p1 = (cx + half * ax, +half * ay, cz + half * az)
        lines.append(
            f'<geom name="{prefix}_roller_{i}" type="capsule" '
            f'fromto="{p0[0]:.6f} {p0[1]:.6f} {p0[2]:.6f} '
            f'{p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}" size=".0042" '
            'material="orange" mass="0" contype="0" conaffinity="0"/>'
        )
    return "\n    ".join(lines)


HUB_FL = _mecanum_hub_xml("wheel_fl")
HUB_FR = _mecanum_hub_xml("wheel_fr")
HUB_RL = _mecanum_hub_xml("wheel_rl")
HUB_RR = _mecanum_hub_xml("wheel_rr")
ROLLER_FL = _mecanum_rollers_xml("wheel_fl", +1)
ROLLER_FR = _mecanum_rollers_xml("wheel_fr", -1)
ROLLER_RL = _mecanum_rollers_xml("wheel_rl", -1)
ROLLER_RR = _mecanum_rollers_xml("wheel_rr", +1)


ROBOT_V2_XML = f"""
<body name="robot" pos="0 0 {WHEEL_RADIUS_M:.6f}">
  <freejoint name="base_free"/>
  <inertial pos="0 0 .042" mass=".735" diaginertia=".0020 .0027 .0032"/>

  <!-- Hiwonder assembly-faithful lower chassis. The physical part is an
       inverted-U metal tray: one top sheet and down-turned skirts with the TT
       motors visible from below. The former full bottom plate made the base look
       like an oversized sealed box. Keep one invisible hull for stable contact. -->
  <geom name="base_lower_collision" type="box"
        pos="0 0 {(NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M + NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M)/2 - WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_DECK_LENGTH_M/2 - .002:.6f} {NOMINAL_DECK_WIDTH_M/2 - .003:.6f} {NOMINAL_LOWER_BODY_HEIGHT_M/2:.6f}"
        rgba="0 0 0 0" contype="2" conaffinity="1" friction=".55 .01 .001"/>
  <geom name="base_top" type="box"
        pos="0 0 {NOMINAL_DECK_TOP_FROM_FLOOR_M - WHEEL_RADIUS_M - .002:.6f}"
        size="{NOMINAL_DECK_LENGTH_M/2:.6f} {NOMINAL_DECK_WIDTH_M/2:.6f} .002"
        material="dark" contype="2" conaffinity="1"/>
  <geom name="left_side_panel_visual" type="box"
        pos="0 {NOMINAL_DECK_WIDTH_M/2 - .0015:.6f} {(NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M + NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M)/2 - WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_DECK_LENGTH_M/2 - .004:.6f} .0015 {NOMINAL_LOWER_BODY_HEIGHT_M/2:.6f}"
        material="aluminum" mass="0" contype="0" conaffinity="0"/>
  <geom name="right_side_panel_visual" type="box"
        pos="0 {-NOMINAL_DECK_WIDTH_M/2 + .0015:.6f} {(NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M + NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M)/2 - WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_DECK_LENGTH_M/2 - .004:.6f} .0015 {NOMINAL_LOWER_BODY_HEIGHT_M/2:.6f}"
        material="aluminum" mass="0" contype="0" conaffinity="0"/>
  <geom name="front_plate" type="box"
        pos="{NOMINAL_DECK_LENGTH_M/2:.6f} 0 {(NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M + NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M)/2 - WHEEL_RADIUS_M:.6f}"
        size=".0015 {NOMINAL_DECK_WIDTH_M/2 - .003:.6f} {NOMINAL_LOWER_BODY_HEIGHT_M/2:.6f}"
        material="aluminum" contype="2" conaffinity="1"/>
  <geom name="rear_plate_visual" type="box"
        pos="{-NOMINAL_DECK_LENGTH_M/2:.6f} 0 {(NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M + NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M)/2 - WHEEL_RADIUS_M:.6f}"
        size=".0015 {NOMINAL_DECK_WIDTH_M/2 - .003:.6f} {NOMINAL_LOWER_BODY_HEIGHT_M/2:.6f}"
        material="aluminum" mass="0" contype="0" conaffinity="0"/>

  <!-- Dark recesses reproduce the prominent holes/slots in the folded chassis
       side sheets without turning those decorative holes into collision gaps. -->
  <geom name="left_chassis_hole_front" type="cylinder" pos=".035 {NOMINAL_DECK_WIDTH_M/2+.0002:.6f} {-WHEEL_RADIUS_M+.032:.6f}" size=".0045 .0005" euler="1.5708 0 0" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="left_chassis_hole_mid" type="cylinder" pos="0 {NOMINAL_DECK_WIDTH_M/2+.0002:.6f} {-WHEEL_RADIUS_M+.032:.6f}" size=".0040 .0005" euler="1.5708 0 0" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="left_chassis_hole_rear" type="cylinder" pos="-.035 {NOMINAL_DECK_WIDTH_M/2+.0002:.6f} {-WHEEL_RADIUS_M+.032:.6f}" size=".0045 .0005" euler="1.5708 0 0" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="right_chassis_hole_front" type="cylinder" pos=".035 {-NOMINAL_DECK_WIDTH_M/2-.0002:.6f} {-WHEEL_RADIUS_M+.032:.6f}" size=".0045 .0005" euler="1.5708 0 0" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="right_chassis_hole_mid" type="cylinder" pos="0 {-NOMINAL_DECK_WIDTH_M/2-.0002:.6f} {-WHEEL_RADIUS_M+.032:.6f}" size=".0040 .0005" euler="1.5708 0 0" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="right_chassis_hole_rear" type="cylinder" pos="-.035 {-NOMINAL_DECK_WIDTH_M/2-.0002:.6f} {-WHEEL_RADIUS_M+.032:.6f}" size=".0045 .0005" euler="1.5708 0 0" material="dark" mass="0" contype="0" conaffinity="0"/>

  <!-- TT motor/gearbox bodies are exposed under the real inverted-U tray. -->
  <geom name="motor_fl_gearbox_visual" type="box" pos="{WHEELBASE_M/2:.6f} .037 0" size=".018 .018 .010" material="black" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_fr_gearbox_visual" type="box" pos="{WHEELBASE_M/2:.6f} -.037 0" size=".018 .018 .010" material="black" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_rl_gearbox_visual" type="box" pos="{-WHEELBASE_M/2:.6f} .037 0" size=".018 .018 .010" material="black" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_rr_gearbox_visual" type="box" pos="{-WHEELBASE_M/2:.6f} -.037 0" size=".018 .018 .010" material="black" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_fl_can_visual" type="cylinder" pos="{WHEELBASE_M/2:.6f} .014 0" size=".011 .012" euler="1.5708 0 0" rgba=".18 .19 .20 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_fr_can_visual" type="cylinder" pos="{WHEELBASE_M/2:.6f} -.014 0" size=".011 .012" euler="1.5708 0 0" rgba=".18 .19 .20 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_rl_can_visual" type="cylinder" pos="{-WHEELBASE_M/2:.6f} .014 0" size=".011 .012" euler="1.5708 0 0" rgba=".18 .19 .20 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="motor_rr_can_visual" type="cylinder" pos="{-WHEELBASE_M/2:.6f} -.014 0" size=".011 .012" euler="1.5708 0 0" rgba=".18 .19 .20 1" mass="0" contype="0" conaffinity="0"/>

  <!-- Raspberry Pi / expansion-board area.  The real MasterPi is an open
       board stack held by slim copper standoffs.  Do not render solid side walls
       or a solid 64x92 mm lid: Hiwonder's assembly images show four posts and a
       heavily cut-out top bracket.  Keep one invisible proxy for stable contact. -->
  <geom name="rear_cage_collision" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} 0 {(NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M + NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M)/2 - WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_PI_CAGE_HALF_LENGTH_M:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M:.6f} {(NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M - NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M)/2:.6f}"
        rgba="0 0 0 0" contype="2" conaffinity="1"/>
  <geom name="pi_board_visual" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} 0 {NOMINAL_PI_BOARD_Z_FROM_FLOOR_M - WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_PI_BOARD_HALF_LENGTH_M:.6f} {NOMINAL_PI_BOARD_HALF_WIDTH_M:.6f} .002"
        rgba=".035 .23 .09 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="pi_heatsink_visual" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M - .006:.6f} 0 {NOMINAL_PI_HEATSINK_Z_FROM_FLOOR_M - WHEEL_RADIUS_M:.6f}"
        size=".012 .014 .003" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="expansion_board_visual" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} 0 {NOMINAL_EXPANSION_BOARD_Z_FROM_FLOOR_M - WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_PI_BOARD_HALF_LENGTH_M:.6f} {NOMINAL_PI_BOARD_HALF_WIDTH_M:.6f} .0015"
        rgba=".10 .11 .12 1" mass="0" contype="0" conaffinity="0"/>

  <!-- Current Hiwonder assembly imagery shows the electronics behind thin
       perforated sheet-metal side panels and under a broad slotted top cover.
       Earlier V2 incorrectly reduced this to an open rail cage.  MuJoCo cannot
       subtract holes from boxes, so use the real sheet silhouette plus dark,
       non-colliding inset decals for the visible cut-outs. -->
  <geom name="electronics_left_side_visual" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M:.6f} {(NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M+NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M)/2-WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_PI_CAGE_HALF_LENGTH_M:.6f} .0014 {(NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M)/2:.6f}"
        material="aluminum" mass="0" contype="0" conaffinity="0"/>
  <geom name="electronics_right_side_visual" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M:.6f} {(NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M+NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M)/2-WHEEL_RADIUS_M:.6f}"
        size="{NOMINAL_PI_CAGE_HALF_LENGTH_M:.6f} .0014 {(NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M)/2:.6f}"
        material="aluminum" mass="0" contype="0" conaffinity="0"/>
  <geom name="electronics_top_cover_visual" type="box"
        pos="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} 0 {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.0012:.6f}"
        size="{NOMINAL_PI_CAGE_HALF_LENGTH_M:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M:.6f} .0012"
        material="aluminum" mass="0" contype="0" conaffinity="0"/>
  <!-- Top-cover slot decals: presentation only, no collision or dynamics. -->
  <geom name="top_cover_slot_1" type="box" pos="{NOMINAL_PI_CAGE_CENTER_X_M-.018:.6f} .025 {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M+.00005:.6f}" size=".009 .003 .00015" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="top_cover_slot_2" type="box" pos="{NOMINAL_PI_CAGE_CENTER_X_M-.018:.6f} .010 {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M+.00005:.6f}" size=".009 .003 .00015" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="top_cover_slot_3" type="box" pos="{NOMINAL_PI_CAGE_CENTER_X_M-.018:.6f} -.005 {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M+.00005:.6f}" size=".009 .003 .00015" material="dark" mass="0" contype="0" conaffinity="0"/>
  <geom name="top_cover_slot_4" type="box" pos="{NOMINAL_PI_CAGE_CENTER_X_M-.018:.6f} -.020 {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M+.00005:.6f}" size=".009 .003 .00015" material="dark" mass="0" contype="0" conaffinity="0"/>

  <!-- Six M4x50 copper columns from Hiwonder's packing/assembly sequence. -->
  <geom name="cage_front_left_standoff" type="capsule"
        fromto="{NOMINAL_PI_CAGE_CENTER_X_M+NOMINAL_PI_CAGE_HALF_LENGTH_M-.003:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M-.003:.6f} {NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M-WHEEL_RADIUS_M:.6f} {NOMINAL_PI_CAGE_CENTER_X_M+NOMINAL_PI_CAGE_HALF_LENGTH_M-.003:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M-.003:.6f} {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.003:.6f}"
        size=".002" rgba=".72 .42 .16 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="cage_front_right_standoff" type="capsule"
        fromto="{NOMINAL_PI_CAGE_CENTER_X_M+NOMINAL_PI_CAGE_HALF_LENGTH_M-.003:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M+.003:.6f} {NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M-WHEEL_RADIUS_M:.6f} {NOMINAL_PI_CAGE_CENTER_X_M+NOMINAL_PI_CAGE_HALF_LENGTH_M-.003:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M+.003:.6f} {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.003:.6f}"
        size=".002" rgba=".72 .42 .16 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="cage_rear_left_standoff" type="capsule"
        fromto="{NOMINAL_PI_CAGE_CENTER_X_M-NOMINAL_PI_CAGE_HALF_LENGTH_M+.003:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M-.003:.6f} {NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M-WHEEL_RADIUS_M:.6f} {NOMINAL_PI_CAGE_CENTER_X_M-NOMINAL_PI_CAGE_HALF_LENGTH_M+.003:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M-.003:.6f} {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.003:.6f}"
        size=".002" rgba=".72 .42 .16 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="cage_rear_right_standoff" type="capsule"
        fromto="{NOMINAL_PI_CAGE_CENTER_X_M-NOMINAL_PI_CAGE_HALF_LENGTH_M+.003:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M+.003:.6f} {NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M-WHEEL_RADIUS_M:.6f} {NOMINAL_PI_CAGE_CENTER_X_M-NOMINAL_PI_CAGE_HALF_LENGTH_M+.003:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M+.003:.6f} {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.003:.6f}"
        size=".002" rgba=".72 .42 .16 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="cage_mid_left_standoff" type="capsule"
        fromto="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M-.003:.6f} {NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M-WHEEL_RADIUS_M:.6f} {NOMINAL_PI_CAGE_CENTER_X_M:.6f} {NOMINAL_PI_CAGE_HALF_WIDTH_M-.003:.6f} {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.003:.6f}"
        size=".002" rgba=".72 .42 .16 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="cage_mid_right_standoff" type="capsule"
        fromto="{NOMINAL_PI_CAGE_CENTER_X_M:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M+.003:.6f} {NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M-WHEEL_RADIUS_M:.6f} {NOMINAL_PI_CAGE_CENTER_X_M:.6f} {-NOMINAL_PI_CAGE_HALF_WIDTH_M+.003:.6f} {NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M-WHEEL_RADIUS_M-.003:.6f}"
        size=".002" rgba=".72 .42 .16 1" mass="0" contype="0" conaffinity="0"/>

  <!-- The old rail-only cage representation was removed: official assembly
       imagery shows the broad perforated sheet-metal cover above. -->

  <!-- Front ultrasonic pair visible on the physical chassis, mounted low on
       the front plate rather than on a tall body block. -->
  <geom name="ultrasonic_bracket" type="box"
        pos="{NOMINAL_ULTRASONIC_X_M - .004:.6f} 0 {0.054 - WHEEL_RADIUS_M:.6f}"
        size=".004 .030 .013" material="black" mass="0" contype="0" conaffinity="0"/>
  <geom name="ultrasonic_left" type="cylinder"
        pos="{NOMINAL_ULTRASONIC_X_M:.6f} .017 {0.054 - WHEEL_RADIUS_M:.6f}" size=".010 .006"
        euler="0 1.5708 0" rgba=".72 .76 .80 1" mass="0" contype="0" conaffinity="0"/>
  <geom name="ultrasonic_right" type="cylinder"
        pos="{NOMINAL_ULTRASONIC_X_M:.6f} -.017 {0.054 - WHEEL_RADIUS_M:.6f}" size=".010 .006"
        euler="0 1.5708 0" rgba=".72 .76 .80 1" mass="0" contype="0" conaffinity="0"/>

  <!-- Official wheel envelope: 65 mm diameter, 185 x 162 mm total robot base. -->
  <body name="wheel_fl_body" pos="{WHEELBASE_M/2:.6f} {TRACK_M/2:.6f} 0">
    <joint name="wheel_fl_joint" type="hinge" axis="0 1 0" damping=".0015" armature=".00002"/>
    <inertial pos="0 0 0" mass=".05" diaginertia=".000018 .000029 .000018"/>
    <geom name="wheel_fl" type="cylinder" size="{WHEEL_RADIUS_M:.6f} {NOMINAL_WHEEL_WIDTH_M/2:.6f}" euler="1.5708 0 0"
          rgba="0 0 0 0" contype="2" conaffinity="1" friction="{REDUCED_MECANUM_SUPPORT_FRICTION:.6f} .0001 .00001" condim="4" priority="1"/>
    {HUB_FL}
    {ROLLER_FL}
  </body>
  <body name="wheel_fr_body" pos="{WHEELBASE_M/2:.6f} {-TRACK_M/2:.6f} 0">
    <joint name="wheel_fr_joint" type="hinge" axis="0 1 0" damping=".0015" armature=".00002"/>
    <inertial pos="0 0 0" mass=".05" diaginertia=".000018 .000029 .000018"/>
    <geom name="wheel_fr" type="cylinder" size="{WHEEL_RADIUS_M:.6f} {NOMINAL_WHEEL_WIDTH_M/2:.6f}" euler="1.5708 0 0"
          rgba="0 0 0 0" contype="2" conaffinity="1" friction="{REDUCED_MECANUM_SUPPORT_FRICTION:.6f} .0001 .00001" condim="4" priority="1"/>
    {HUB_FR}
    {ROLLER_FR}
  </body>
  <body name="wheel_rl_body" pos="{-WHEELBASE_M/2:.6f} {TRACK_M/2:.6f} 0">
    <joint name="wheel_rl_joint" type="hinge" axis="0 1 0" damping=".0015" armature=".00002"/>
    <inertial pos="0 0 0" mass=".05" diaginertia=".000018 .000029 .000018"/>
    <geom name="wheel_rl" type="cylinder" size="{WHEEL_RADIUS_M:.6f} {NOMINAL_WHEEL_WIDTH_M/2:.6f}" euler="1.5708 0 0"
          rgba="0 0 0 0" contype="2" conaffinity="1" friction="{REDUCED_MECANUM_SUPPORT_FRICTION:.6f} .0001 .00001" condim="4" priority="1"/>
    {HUB_RL}
    {ROLLER_RL}
  </body>
  <body name="wheel_rr_body" pos="{-WHEELBASE_M/2:.6f} {-TRACK_M/2:.6f} 0">
    <joint name="wheel_rr_joint" type="hinge" axis="0 1 0" damping=".0015" armature=".00002"/>
    <inertial pos="0 0 0" mass=".05" diaginertia=".000018 .000029 .000018"/>
    <geom name="wheel_rr" type="cylinder" size="{WHEEL_RADIUS_M:.6f} {NOMINAL_WHEEL_WIDTH_M/2:.6f}" euler="1.5708 0 0"
          rgba="0 0 0 0" contype="2" conaffinity="1" friction="{REDUCED_MECANUM_SUPPORT_FRICTION:.6f} .0001 .00001" condim="4" priority="1"/>
    {HUB_RR}
    {ROLLER_RR}
  </body>

  <!-- Physical topology from Hiwonder's straight-arm reference: ID6 yaw is
       below the ID5 shoulder.  LINK_1 is base/axle -> shoulder, not floor ->
       shoulder.  The prior V2 accidentally subtracted wheel radius and made the
       entire arm ~32.5 mm too low while also co-locating yaw and shoulder. -->
  <body name="arm_base" pos="0 0 {NOMINAL_YAW_AXIS_FROM_BASE_M:.6f}">
    <joint name="arm_yaw" type="hinge" axis="0 0 1" range="-1.75 1.75" damping=".025"/>
    <inertial pos="0 0 .012" mass=".030" diaginertia=".000025 .000025 .000020"/>
    <geom name="base_yaw_servo_visual" type="box" pos="0 0 -.002" size="{SERVO_BODY_HALF[0]:.6f} {SERVO_BODY_HALF[1]:.6f} {SERVO_BODY_HALF[2]:.6f}"
          material="black" mass="0" contype="0" conaffinity="0"/>
    <geom name="base_yaw_bearing_visual" type="cylinder" pos="0 0 -.021" size=".024 .004"
          material="aluminum" mass="0" contype="0" conaffinity="0"/>
    <geom name="arm_pedestal_collision" type="cylinder" size=".025 .020" pos="0 0 -.002" rgba="0 0 0 0"
          contype="2" conaffinity="1"/>
    <!-- U-bracket carrying the shoulder above the yaw bearing. -->
    <geom name="shoulder_mount_plate_left" type="box" pos="0 {ARM_PLATE_SIDE_OFFSET_M:.6f} {NOMINAL_YAW_TO_SHOULDER_M/2:.6f}"
          size=".021 {ARM_PLATE_THICKNESS_M:.6f} {NOMINAL_YAW_TO_SHOULDER_M/2 + .004:.6f}"
          material="orange" mass="0" contype="0" conaffinity="0"/>
    <geom name="shoulder_mount_plate_right" type="box" pos="0 {-ARM_PLATE_SIDE_OFFSET_M:.6f} {NOMINAL_YAW_TO_SHOULDER_M/2:.6f}"
          size=".021 {ARM_PLATE_THICKNESS_M:.6f} {NOMINAL_YAW_TO_SHOULDER_M/2 + .004:.6f}"
          material="orange" mass="0" contype="0" conaffinity="0"/>

    <body name="shoulder_link" pos="0 0 {NOMINAL_YAW_TO_SHOULDER_M:.6f}">
      <joint name="shoulder" type="hinge" axis="0 -1 0" range="-.15 2.80" damping=".035"/>
      <inertial pos="{LINK_2_CM / 200.0:.6f} 0 0" mass=".050" diaginertia=".000020 .000030 .000030"/>
      <geom name="shoulder_servo_visual" type="box" pos=".004 0 0" size="{SERVO_BODY_HALF[0]:.6f} {SERVO_BODY_HALF[1]:.6f} {SERVO_BODY_HALF[2]:.6f}"
            material="black" mass="0" contype="0" conaffinity="0"/>
      <geom name="shoulder_horn_outer" type="cylinder" pos="0 {ARM_HORN_SIDE_OFFSET_M:.6f} 0" size="{ARM_PLATE_HALF_WIDTH_M:.6f} {ARM_HORN_HALF_THICKNESS_M:.6f}"
            euler="1.5708 0 0" material="orange" mass="0" contype="0" conaffinity="0"/>
      <geom name="shoulder_horn_center" type="cylinder" pos="0 {ARM_HORN_SIDE_OFFSET_M + .002:.6f} 0" size=".006 .0012"
            euler="1.5708 0 0" rgba=".82 .84 .86 1" mass="0" contype="0" conaffinity="0"/>
      <!-- Real Hiwonder bracket is an open orange frame. Two slim rails retain
           the visible outer envelope while leaving the centre genuinely open. -->
      <geom name="upper_arm_plate_left" type="box" pos="{LINK_2_CM/200.0:.6f} {ARM_PLATE_SIDE_OFFSET_M:.6f} {ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
            size="{LINK_2_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
            material="orange" mass="0" contype="0" conaffinity="0"/>
      <geom name="upper_arm_plate_left_lower" type="box" pos="{LINK_2_CM/200.0:.6f} {ARM_PLATE_SIDE_OFFSET_M:.6f} {-ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
            size="{LINK_2_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
            material="orange" mass="0" contype="0" conaffinity="0"/>
      <geom name="upper_arm_plate_right" type="box" pos="{LINK_2_CM/200.0:.6f} -{ARM_PLATE_SIDE_OFFSET_M:.6f} {ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
            size="{LINK_2_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
            material="orange" mass="0" contype="0" conaffinity="0"/>
      <geom name="upper_arm_plate_right_lower" type="box" pos="{LINK_2_CM/200.0:.6f} -{ARM_PLATE_SIDE_OFFSET_M:.6f} {-ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
            size="{LINK_2_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
            material="orange" mass="0" contype="0" conaffinity="0"/>
      <geom name="upper_arm_collision" type="capsule" fromto="0 0 0 {LINK_2_CM / 100.0:.6f} 0 0"
            size=".011" rgba="0 0 0 0" contype="2" conaffinity="1"/>

      <body name="elbow_link" pos="{LINK_2_CM / 100.0:.6f} 0 0">
        <joint name="elbow" type="hinge" axis="0 -1 0" range="-1.80 1.80" damping=".030"/>
        <inertial pos="{LINK_3_CM / 200.0:.6f} 0 0" mass=".040" diaginertia=".000016 .000022 .000022"/>
        <geom name="elbow_servo_visual" type="box" pos="0 0 0" size="{SERVO_BODY_HALF[0]:.6f} {SERVO_BODY_HALF[1]:.6f} {SERVO_BODY_HALF[2]:.6f}"
              material="black" mass="0" contype="0" conaffinity="0"/>
        <geom name="elbow_horn_outer" type="cylinder" pos="0 {ARM_HORN_SIDE_OFFSET_M:.6f} 0" size="{ARM_PLATE_HALF_WIDTH_M:.6f} {ARM_HORN_HALF_THICKNESS_M:.6f}"
              euler="1.5708 0 0" material="orange" mass="0" contype="0" conaffinity="0"/>
        <geom name="elbow_horn_center" type="cylinder" pos="0 {ARM_HORN_SIDE_OFFSET_M + .002:.6f} 0" size=".006 .0012"
              euler="1.5708 0 0" rgba=".82 .84 .86 1" mass="0" contype="0" conaffinity="0"/>
        <geom name="forearm_plate_left" type="box" pos="{LINK_3_CM/200.0:.6f} {ARM_PLATE_SIDE_OFFSET_M:.6f} {ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
              size="{LINK_3_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
              material="orange" mass="0" contype="0" conaffinity="0"/>
        <geom name="forearm_plate_left_lower" type="box" pos="{LINK_3_CM/200.0:.6f} {ARM_PLATE_SIDE_OFFSET_M:.6f} {-ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
              size="{LINK_3_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
              material="orange" mass="0" contype="0" conaffinity="0"/>
        <geom name="forearm_plate_right" type="box" pos="{LINK_3_CM/200.0:.6f} -{ARM_PLATE_SIDE_OFFSET_M:.6f} {ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
              size="{LINK_3_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
              material="orange" mass="0" contype="0" conaffinity="0"/>
        <geom name="forearm_plate_right_lower" type="box" pos="{LINK_3_CM/200.0:.6f} -{ARM_PLATE_SIDE_OFFSET_M:.6f} {-ARM_LINK_RAIL_Z_OFFSET_M:.6f}"
              size="{LINK_3_CM/200.0:.6f} {ARM_PLATE_THICKNESS_M:.6f} {ARM_LINK_HALF_HEIGHT_M:.6f}"
              material="orange" mass="0" contype="0" conaffinity="0"/>
        <geom name="forearm_collision" type="capsule" fromto="0 0 0 {LINK_3_CM / 100.0:.6f} 0 0"
              size=".010" rgba="0 0 0 0" contype="2" conaffinity="1"/>

        <body name="wrist_link" pos="{LINK_3_CM / 100.0:.6f} 0 0">
          <joint name="wrist_pitch" type="hinge" axis="0 -1 0" range="-1.80 1.80" damping=".020"/>
          <inertial pos=".035 0 0" mass=".025" diaginertia=".000010 .000018 .000018"/>
          <geom name="wrist_servo_visual" type="box" pos="0 0 0" size="{SERVO_BODY_HALF[0]:.6f} {SERVO_BODY_HALF[1]:.6f} {SERVO_BODY_HALF[2]:.6f}"
                material="black" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_horn_outer" type="cylinder" pos="0 .022 0" size=".017 .003"
                euler="1.5708 0 0" material="orange" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_horn_center" type="cylinder" pos="0 {ARM_HORN_SIDE_OFFSET_M + .002:.6f} 0" size=".006 .0012"
                euler="1.5708 0 0" rgba=".82 .84 .86 1" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_plate_left" type="box" pos=".027 {ARM_PLATE_SIDE_OFFSET_M:.6f} {WRIST_LINK_RAIL_Z_OFFSET_M:.6f}" size=".027 {ARM_PLATE_THICKNESS_M:.6f} {WRIST_LINK_HALF_HEIGHT_M:.6f}"
                material="orange" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_plate_left_lower" type="box" pos=".027 {ARM_PLATE_SIDE_OFFSET_M:.6f} {-WRIST_LINK_RAIL_Z_OFFSET_M:.6f}" size=".027 {ARM_PLATE_THICKNESS_M:.6f} {WRIST_LINK_HALF_HEIGHT_M:.6f}"
                material="orange" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_plate_right" type="box" pos=".027 {-ARM_PLATE_SIDE_OFFSET_M:.6f} {WRIST_LINK_RAIL_Z_OFFSET_M:.6f}" size=".027 {ARM_PLATE_THICKNESS_M:.6f} {WRIST_LINK_HALF_HEIGHT_M:.6f}"
                material="orange" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_plate_right_lower" type="box" pos=".027 {-ARM_PLATE_SIDE_OFFSET_M:.6f} {-WRIST_LINK_RAIL_Z_OFFSET_M:.6f}" size=".027 {ARM_PLATE_THICKNESS_M:.6f} {WRIST_LINK_HALF_HEIGHT_M:.6f}"
                material="orange" mass="0" contype="0" conaffinity="0"/>
          <geom name="wrist_collision" type="capsule" fromto="0 0 0 .055 0 0"
                size=".009" rgba="0 0 0 0" contype="2" conaffinity="1"/>

          <body name="gripper" pos="0 0 0">
            <inertial pos=".065 0 .006" mass=".010" diaginertia=".000010 .000017 .000017"/>
            <geom name="gripper_servo_visual" type="box" pos=".030 0 -.003" size="{MICRO_SERVO_BODY_HALF[0]:.6f} {MICRO_SERVO_BODY_HALF[1]:.6f} {MICRO_SERVO_BODY_HALF[2]:.6f}"
                  material="black" mass="0" contype="0" conaffinity="0"/>
            <geom name="gripper_crossbar_visual" type="box" pos=".052 0 -.012" size=".024 .034 .0035"
                  material="aluminum" mass="0" contype="0" conaffinity="0"/>

            <!-- Physical rigid eye-in-hand bracket; controller math remains a
                 separate approximation in harness/real_geometry.py. -->
            <geom name="camera_bracket" type="box" pos=".036 0 .018" size=".019 .026 .005"
                  material="aluminum" contype="2" conaffinity="1"/>
            <geom name="camera_body" type="box" pos=".050 0 .025" size=".016 .022 .014"
                  material="black" mass="0" contype="0" conaffinity="0"/>
            <geom name="camera_lens_visual" type="cylinder" pos=".067 0 .025" size=".009 .003" euler="0 1.5708 0"
                  rgba=".02 .02 .025 1" mass="0" contype="0" conaffinity="0"/>
            <camera name="robot_cam" pos="{CAMERA_LINK_CM / 100.0:.6f} 0 {CAMERA_Z_OFFSET_CM / 100.0:.6f}"
                    xyaxes="0 -1 0 0 0 1" fovy="{CAMERA_VFOV_DEG:.6f}"/>
            <site name="camera_axis_site" pos="{CAMERA_LINK_CM / 100.0:.6f} 0 0" size=".001" rgba="0 0 0 0"/>

            <!-- The physical Hiwonder gripper is servo/linkage driven.  The
                 dynamic closure remains the already-validated reduced-order
                 symmetric slide coordinate until linkage pivot dimensions are
                 measured, but visible metal arms/pivots and orange pads are
                 separated from the invisible contact proxies below. -->
            <body name="left_jaw" pos="0 .035 0">
              <joint name="left_gripper_close" type="slide" axis="0 -1 0" range="0 {GRIPPER_MAX_CLOSE_M:.6f}" damping="{GRIPPER_JAW_DAMPING:.3f}"/>
              <inertial pos=".095 0 -.018" mass=".005" diaginertia=".000002 .000006 .000006"/>
              <geom name="left_jaw_link_visual" type="capsule" fromto=".039 0 -.006 .090 0 0" size=".0022"
                    material="aluminum" mass="0" contype="0" conaffinity="0"/>
              <geom name="left_jaw_link_upper_visual" type="capsule" fromto=".047 0 .004 .089 0 0" size=".0016"
                    material="aluminum" mass="0" contype="0" conaffinity="0"/>
              <geom name="left_jaw_pivot_visual" type="cylinder" pos=".043 0 -.002" size=".005 .0015" euler="1.5708 0 0"
                    material="dark" mass="0" contype="0" conaffinity="0"/>
              <geom name="left_finger_pad_visual" type="box" pos=".100 0 0" size="{GRIPPER_PAD_HALF_LENGTH_M:.6f} {GRIPPER_PAD_HALF_WIDTH_M:.6f} {GRIPPER_PAD_HALF_HEIGHT_M:.6f}"
                    material="orange" mass="0" contype="0" conaffinity="0"/>
              <geom name="left_finger" type="box" pos=".100 0 0" size="{FINGER_HALF_LENGTH_M:.6f} {FINGER_HALF_WIDTH_M:.6f} {FINGER_HALF_HEIGHT_M:.6f}"
                    rgba="0 0 0 0" contype="2" conaffinity="1" friction="2.0 .03 .002" solref=".006 1" solimp=".90 .97 .002"/>
            </body>
            <body name="right_jaw" pos="0 -.035 0">
              <joint name="right_gripper_close" type="slide" axis="0 1 0" range="0 {GRIPPER_MAX_CLOSE_M:.6f}" damping="{GRIPPER_JAW_DAMPING:.3f}"/>
              <inertial pos=".095 0 -.018" mass=".005" diaginertia=".000002 .000006 .000006"/>
              <geom name="right_jaw_link_visual" type="capsule" fromto=".039 0 -.006 .090 0 0" size=".0022"
                    material="aluminum" mass="0" contype="0" conaffinity="0"/>
              <geom name="right_jaw_link_upper_visual" type="capsule" fromto=".047 0 .004 .089 0 0" size=".0016"
                    material="aluminum" mass="0" contype="0" conaffinity="0"/>
              <geom name="right_jaw_pivot_visual" type="cylinder" pos=".043 0 -.002" size=".005 .0015" euler="1.5708 0 0"
                    material="dark" mass="0" contype="0" conaffinity="0"/>
              <geom name="right_finger_pad_visual" type="box" pos=".100 0 0" size="{GRIPPER_PAD_HALF_LENGTH_M:.6f} {GRIPPER_PAD_HALF_WIDTH_M:.6f} {GRIPPER_PAD_HALF_HEIGHT_M:.6f}"
                    material="orange" mass="0" contype="0" conaffinity="0"/>
              <geom name="right_finger" type="box" pos=".100 0 0" size="{FINGER_HALF_LENGTH_M:.6f} {FINGER_HALF_WIDTH_M:.6f} {FINGER_HALF_HEIGHT_M:.6f}"
                    rgba="0 0 0 0" contype="2" conaffinity="1" friction="2.0 .03 .002" solref=".006 1" solimp=".90 .97 .002"/>
            </body>
            <site name="grip_site" pos=".100 0 0" size=".001" rgba="0 0 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </body>
</body>
"""


def build_v2_xml(hardware: Mapping[str, float] | None = None) -> str:
    hardware = dict(hardware or {})
    root = ET.fromstring(SCENE_PATH.read_text())
    root.set("model", "ugrp_masterpi_dynamics_v2")
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")
    option.set("cone", "elliptic")
    option.set("iterations", "80")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    # Reserve a larger offscreen framebuffer for the human-facing observer
    # renderer. This does not change robot_cam's explicit 640x480 sensor model.
    global_visual.set("offwidth", "1280")
    global_visual.set("offheight", "720")

    world = root.find("worldbody")
    if world is None:
        raise RuntimeError("canonical scene has no worldbody")
    for child in list(world):
        if child.tag == "geom" and child.get("name") in NONREAL_WORKCELL_GEOMS:
            world.remove(child)
    old_robot = next((child for child in list(world) if child.tag == "body" and child.get("name") == "robot"), None)
    if old_robot is None:
        raise RuntimeError("canonical scene has no robot body")
    idx = list(world).index(old_robot)
    world.remove(old_robot)
    world.insert(idx, ET.fromstring(ROBOT_V2_XML))

    # Static dimensions are direct physical measurements. Apply them to the
    # generated model rather than storing decorative numbers in the manifest.
    robot = next((child for child in list(world) if child.tag == "body" and child.get("name") == "robot"), None)
    if robot is None:
        raise RuntimeError("v2 robot replacement failed")
    wheel_radius = float(hardware.get("wheel_radius_m", WHEEL_RADIUS_M))
    wheelbase = float(hardware.get("wheelbase_m", WHEELBASE_M))
    track = float(hardware.get("track_m", TRACK_M))
    if not (0.015 <= wheel_radius <= 0.08 and 0.05 <= wheelbase <= 0.30 and 0.05 <= track <= 0.30):
        raise ValueError("calibrated wheel geometry is outside physical bounds")
    robot_pos = [float(v) for v in robot.get("pos", "0 0 0").split()]
    robot_pos[2] = wheel_radius
    robot.set("pos", " ".join(f"{v:.6f}" for v in robot_pos))
    wheel_positions = {
        "wheel_fl_body": (+wheelbase / 2.0, +track / 2.0),
        "wheel_fr_body": (+wheelbase / 2.0, -track / 2.0),
        "wheel_rl_body": (-wheelbase / 2.0, +track / 2.0),
        "wheel_rr_body": (-wheelbase / 2.0, -track / 2.0),
    }
    for body in robot.iter("body"):
        name = body.get("name", "")
        if name in wheel_positions:
            x, y = wheel_positions[name]
            body.set("pos", f"{x:.6f} {y:.6f} 0")
            geom = next((g for g in body.findall("geom") if g.get("name", "").startswith("wheel_")), None)
            if geom is not None:
                size = [float(v) for v in geom.get("size", ".034 .014").split()]
                size[0] = wheel_radius
                geom.set("size", " ".join(f"{v:.6f}" for v in size))
        elif name == "arm_base":
            body.set("pos", f"0 0 {NOMINAL_YAW_AXIS_FROM_BASE_M:.6f}")

    gripper_kp = float(hardware.get("gripper_position_kp", GRIPPER_POSITION_KP))
    gripper_mu = float(hardware.get("gripper_finger_friction", 3.4))
    if not (20.0 <= gripper_kp <= 3000.0 and 0.1 <= gripper_mu <= 8.0):
        raise ValueError("calibrated gripper parameters are outside physical bounds")
    for geom in robot.iter("geom"):
        if geom.get("name") in {"left_finger", "right_finger"}:
            old = [float(v) for v in geom.get("friction", "3.4 .05 .003").split()]
            old[0] = gripper_mu
            geom.set("friction", " ".join(f"{v:.6f}" for v in old))

    # robot_cam uses measured physical intrinsics/distortion, but the rigid mount
    # is a centered provisional structural mount until held-out hand-eye evidence
    # exists. The legacy 3-parameter manifest fields remain compatibility-only.
    for camera in robot.iter("camera"):
        if camera.get("name") == "robot_cam":
            camera.set("pos", " ".join(f"{v:.12f}" for v in CAMERA_LOCAL_POS_M))
            camera.attrib.pop("xyaxes", None)
            camera.attrib.pop("euler", None)
            camera.attrib.pop("axisangle", None)
            camera.set("quat", " ".join(f"{v:.12f}" for v in CAMERA_LOCAL_QUAT_WXYZ))
            camera.attrib.pop("fovy", None)
            camera.set("focalpixel", f"{CAMERA_FX_PX:.12f} {CAMERA_FY_PX:.12f}")
            # MuJoCo principalpixel is a centre-relative offset with opposite
            # sign to OpenCV's top-left absolute (cx, cy).
            camera.set(
                "principalpixel",
                f"{320.0 - CAMERA_CX_PX:.12f} {240.0 - CAMERA_CY_PX:.12f}",
            )
            camera.set("resolution", "640 480")
            camera.set("sensorsize", "640 480")

    # REAL pickup geometry uses 1.5 cm half-depth / 3.0 cm nominal cube height.
    # The legacy scene uses 50 mm cubes; V2 corrects only its generated scene.
    for color in ("red", "blue", "yellow"):
        body = next((b for b in world.findall("body") if b.get("name") == f"{color}_block"), None)
        if body is None:
            raise RuntimeError(f"canonical scene has no {color}_block body")
        pos = [float(v) for v in body.get("pos", "0 0 0").split()]
        pos[2] = TARGET_BLOCK_HALF_M
        body.set("pos", " ".join(f"{v:.6f}" for v in pos))
        geom = next((g for g in body.findall("geom") if g.get("name") == f"{color}_block_geom"), None)
        if geom is None:
            raise RuntimeError(f"canonical scene has no {color}_block_geom")
        geom.set("size", f"{TARGET_BLOCK_HALF_M:.6f} {TARGET_BLOCK_HALF_M:.6f} {TARGET_BLOCK_HALF_M:.6f}")
        block_mass = float(hardware.get("block_mass_kg", PROVISIONAL_TARGET_BLOCK_MASS_KG))
        block_mu = float(hardware.get("block_floor_friction", 1.0))
        if not (0.001 <= block_mass <= 0.5 and 0.05 <= block_mu <= 4.0):
            raise ValueError("calibrated block parameters are outside physical bounds")
        geom.set("mass", f"{block_mass:.8f}")
        friction = [float(v) for v in geom.get("friction", "1 .01 .001").split()]
        friction[0] = block_mu
        geom.set("friction", " ".join(f"{v:.6f}" for v in friction))
        # With the default soft impedance a 5 g cube resting on another cube
        # creeps sideways (~1 mm/min at rest, faster once offset) and a
        # three-cube tower topples by itself within ~90 s of sim time. A stiff
        # impedance on the cube geoms removes the creep; no priority override,
        # so finger-cube contacts still take the finger friction.
        geom.set("solimp", "0.99 0.999 0.001 0.5 2")

    # Solid workcell geometry must constrain a physical-learning model.  Colored
    # delivery pads remain non-colliding markings.
    solid_prefixes = ("wall_", "rack_", "workbench", "crate_")
    for geom in world.iter("geom"):
        name = geom.get("name", "")
        if name.startswith(solid_prefixes):
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
            if geom.get("friction") is None:
                geom.set("friction", ".65 .01 .001")

    actuator = ET.SubElement(root, "actuator")
    for wheel in ("fl", "fr", "rl", "rr"):
        ET.SubElement(
            actuator,
            "velocity",
            name=f"wheel_{wheel}_drive",
            joint=f"wheel_{wheel}_joint",
            kv=".001",
            ctrllimited="true",
            ctrlrange=f"{-MAX_WHEEL_RAD_S} {MAX_WHEEL_RAD_S}",
            forcelimited="true",
            forcerange="-.002 .002",
        )
    ET.SubElement(actuator, "position", name="servo_arm_yaw", joint="arm_yaw", kp="4.0", forcelimited="true", forcerange="-1.2 1.2")
    ET.SubElement(actuator, "position", name="servo_shoulder", joint="shoulder", kp="7.0", forcelimited="true", forcerange="-2.2 2.2")
    ET.SubElement(actuator, "position", name="servo_elbow", joint="elbow", kp="6.0", forcelimited="true", forcerange="-1.8 1.8")
    ET.SubElement(actuator, "position", name="servo_wrist", joint="wrist_pitch", kp="3.5", forcelimited="true", forcerange="-1.0 1.0")
    ET.SubElement(actuator, "position", name="servo_gripper_left", joint="left_gripper_close", kp=f"{gripper_kp:.3f}", forcelimited="true", forcerange="-18 18")
    ET.SubElement(actuator, "position", name="servo_gripper_right", joint="right_gripper_close", kp=f"{gripper_kp:.3f}", forcelimited="true", forcerange="-18 18")

    return ET.tostring(root, encoding="unicode")


XML = build_v2_xml()


def _quat_to_rpy(q: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = (float(v) for v in q)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class MasterPiDynamicsV2:
    """Low-level physical core with the same command domain as real MasterPi."""

    wheel_names = ("fl", "fr", "rl", "rr")

    def __init__(
        self,
        *,
        seed: int = 1,
        dynamics: Mapping[str, float] | None = None,
        hardware: Mapping[str, float] | None = None,
        use_calibration_manifest: bool = True,
        render: bool = False,
        width: int = 160,
        height: int = 120,
    ):
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)
        defaults = {
            "motor_time_constant_s": MOTOR_TIME_CONSTANT_S,
            "max_forward_force_n": MAX_FORWARD_FORCE_N,
            "max_lateral_force_n": MAX_LATERAL_FORCE_N,
            "max_yaw_torque_nm": MAX_YAW_TORQUE_NM,
            "linear_damping_n_per_mps": LINEAR_DAMPING_N_PER_MPS,
            "yaw_damping_nm_per_radps": YAW_DAMPING_NM_PER_RADPS,
            "stop_linear_damping_n_per_mps": STOP_LINEAR_DAMPING_N_PER_MPS,
            "stop_yaw_damping_nm_per_radps": STOP_YAW_DAMPING_NM_PER_RADPS,
        }
        hardware_defaults = {
            "wheel_radius_m": WHEEL_RADIUS_M,
            "wheelbase_m": WHEELBASE_M,
            "track_m": TRACK_M,
            "servo_deadband_pwm": 0.0,
            "servo_rate_pwm_per_s": 2000.0,
            "camera_link_cm": REAL_CAPTURE_CAMERA_LINK_M * 100.0,
            "camera_z_offset_cm": REAL_CAPTURE_CAMERA_LOCAL_Z_M * 100.0,
            "camera_pitch_offset_deg": REAL_CAPTURE_CAMERA_PITCH_OFFSET_DEG,
            "servo6_center_pwm": 1500.0,
            "block_mass_kg": PROVISIONAL_TARGET_BLOCK_MASS_KG,
            "block_floor_friction": 1.0,
            "gripper_position_kp": GRIPPER_POSITION_KP,
            "gripper_finger_friction": 3.4,
        }
        self.calibration_status = "STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED"
        self.calibration_parameters: dict[str, float] = {}
        if use_calibration_manifest and dynamics is None:
            fitted_dyn, dyn_status = load_fitted_dynamics()
            fitted_hw, hw_status = load_fitted_hardware()
            defaults.update(fitted_dyn)
            hardware_defaults.update(fitted_hw)
            self.calibration_parameters = {**fitted_dyn, **fitted_hw}
            statuses = (dyn_status, hw_status)
            if "REAL_CALIBRATED_VALIDATED" in statuses:
                # The loader can only emit validated when the complete manifest
                # is present, so either branch implies both subsets came from it.
                self.calibration_status = "REAL_CALIBRATED_VALIDATED"
            elif "REAL_FITTED_UNVALIDATED" in statuses:
                self.calibration_status = "REAL_FITTED_UNVALIDATED"
        if dynamics:
            unknown = set(dynamics) - set(defaults)
            if unknown:
                raise ValueError(f"unknown dynamics parameters: {sorted(unknown)}")
            defaults.update({k: float(v) for k, v in dynamics.items()})
            self.calibration_parameters.update({k: float(v) for k, v in dynamics.items()})
            self.calibration_status = "EXPLICIT_DYNAMICS_OVERRIDE"
        if hardware:
            unknown = set(hardware) - set(hardware_defaults)
            if unknown:
                raise ValueError(f"unknown hardware parameters: {sorted(unknown)}")
            hardware_defaults.update({k: float(v) for k, v in hardware.items()})
            self.calibration_parameters.update({k: float(v) for k, v in hardware.items()})
            self.calibration_status = "EXPLICIT_DYNAMICS_OVERRIDE"
        if defaults["motor_time_constant_s"] <= 0:
            raise ValueError("motor_time_constant_s must be positive")
        if any(not math.isfinite(v) or v < 0 for k, v in defaults.items() if k != "motor_time_constant_s"):
            raise ValueError("dynamics force/damping parameters must be finite and non-negative")
        self.dynamics = defaults
        self.physical_params = hardware_defaults
        self.model = mujoco.MjModel.from_xml_string(build_v2_xml(self.physical_params))
        self.data = mujoco.MjData(self.model)
        self.width = int(width)
        self.height = int(height)
        if self.width < 32 or self.height < 24:
            raise ValueError("render size is too small for camera-derived observations")
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width) if render else None
        # Human-facing observer/CCTV views use an independent higher-resolution
        # renderer. The robot camera intentionally stays at the physical 640x480
        # sensor contract (including its fisheye profile), while third-person UI
        # video should remain crisp and must not change perception geometry.
        self.observer_width = int(os.environ.get("UGRP_SIM_OBSERVER_WIDTH", "1280"))
        self.observer_height = int(os.environ.get("UGRP_SIM_OBSERVER_HEIGHT", "720"))
        self.observer_width = max(self.width, self.observer_width)
        self.observer_height = max(self.height, self.observer_height)
        self.observer_renderer = (
            mujoco.Renderer(self.model, height=self.observer_height, width=self.observer_width)
            if render else None
        )
        self.robot_bid = self._body("robot")
        self.gripper_bid = self._body("gripper")
        self.robot_cam_cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "robot_cam")
        if self.robot_cam_cid < 0:
            raise KeyError("robot_cam")
        self.front_cam_cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_front_left")
        if self.front_cam_cid < 0:
            raise KeyError("cctv_front_left")
        # The optical center used by the physical controller can sit inside the
        # decorative camera housing.  A real lens never photographs its own
        # housing, so put only those three visual geoms in a sensor-hidden group.
        for geom_name in ("camera_bracket", "camera_body", "camera_lens_visual"):
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if gid >= 0:
                self.model.geom_group[gid] = 5
        self._robot_sensor_scene_option = None
        if self.renderer is not None:
            self._robot_sensor_scene_option = mujoco.MjvOption()
            self._robot_sensor_scene_option.geomgroup[:] = 1
            self._robot_sensor_scene_option.geomgroup[5] = 0
        self._robot_fisheye_map: tuple[np.ndarray, np.ndarray] | None = None
        self._configure_measured_robot_camera()
        self.base_jid = self._joint("base_free")
        self.base_qadr = int(self.model.jnt_qposadr[self.base_jid])
        self.base_dadr = int(self.model.jnt_dofadr[self.base_jid])
        self.wheel_act = np.array([self._actuator(f"wheel_{n}_drive") for n in self.wheel_names], dtype=int)
        self.servo_act = {
            "yaw": self._actuator("servo_arm_yaw"),
            "shoulder": self._actuator("servo_shoulder"),
            "elbow": self._actuator("servo_elbow"),
            "wrist": self._actuator("servo_wrist"),
        }
        self.gripper_act = (
            self._actuator("servo_gripper_left"),
            self._actuator("servo_gripper_right"),
        )
        self.arm_joint = {
            "yaw": self._joint("arm_yaw"),
            "shoulder": self._joint("shoulder"),
            "elbow": self._joint("elbow"),
            "wrist": self._joint("wrist_pitch"),
        }
        self.gripper_joint = (
            self._joint("left_gripper_close"),
            self._joint("right_gripper_close"),
        )
        self.motor_state = np.zeros(4, dtype=float)
        self.motor_command = np.zeros(4, dtype=float)
        self.servo_command_pulses: dict[int, int] = {}
        self.reset()

    def _body(self, name: str) -> int:
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if idx < 0:
            raise KeyError(name)
        return idx

    def _joint(self, name: str) -> int:
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if idx < 0:
            raise KeyError(name)
        return idx

    def _actuator(self, name: str) -> int:
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if idx < 0:
            raise KeyError(name)
        return idx

    def _site(self, name: str) -> int:
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
        if idx < 0:
            raise KeyError(name)
        return idx

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def _configure_measured_robot_camera(self) -> None:
        """Configure measured intrinsics plus the centered provisional mount.

        The shared controller consumes raw 640x480 uStreamer pixels on the
        physical robot. MuJoCo renders an ideal pinhole image first, then the
        precomputed OpenCV-fisheye map converts it to the raw pixel geometry.
        """
        self.model.cam_pos[self.robot_cam_cid] = np.asarray(CAMERA_LOCAL_POS_M, dtype=float)
        self.model.cam_quat[self.robot_cam_cid] = np.asarray(CAMERA_LOCAL_QUAT_WXYZ, dtype=float)
        self.model.cam_resolution[self.robot_cam_cid] = np.asarray([self.width, self.height], dtype=int)
        self.model.cam_sensorsize[self.robot_cam_cid] = np.asarray([float(self.width), float(self.height)], dtype=float)
        self.model.cam_intrinsic[self.robot_cam_cid] = mujoco_pixel_intrinsic(self.width, self.height)
        if self.renderer is not None:
            self._robot_fisheye_map = raw_fisheye_remap(self.width, self.height)
        mujoco.mj_forward(self.model, self.data)

    def _sync_real_camera_mount(self) -> None:
        """Keep robot_cam on the centered provisional rigid mount."""
        self.model.cam_pos[self.robot_cam_cid] = np.asarray(CAMERA_LOCAL_POS_M, dtype=float)
        self.model.cam_quat[self.robot_cam_cid] = np.asarray(CAMERA_LOCAL_QUAT_WXYZ, dtype=float)
        mujoco.mj_forward(self.model, self.data)

    def _sync_front_follow_camera(self) -> None:
        """Put the presentation camera in front of the robot, looking back at it.

        This is intentionally a viewer-only camera: it never feeds controller
        perception.  The pose follows the chassis so the browser gets the
        Minecraft-style front-facing third-person view requested for SIM.
        """
        robot_pos = np.asarray(self.data.xpos[self.robot_bid], dtype=float)
        robot_rot = np.asarray(self.data.xmat[self.robot_bid], dtype=float).reshape(3, 3)
        forward = np.asarray(robot_rot[:, 0], dtype=float)
        forward[2] = 0.0
        norm = float(np.linalg.norm(forward))
        if norm < 1e-9:
            forward = np.asarray([1.0, 0.0, 0.0], dtype=float)
        else:
            forward /= norm

        camera_pos = robot_pos + 0.90 * forward + np.asarray([0.0, 0.0, 0.38])
        target = robot_pos + np.asarray([0.0, 0.0, 0.13])
        view = target - camera_pos
        view /= max(1e-9, float(np.linalg.norm(view)))
        world_up = np.asarray([0.0, 0.0, 1.0], dtype=float)
        right = np.cross(view, world_up)
        right /= max(1e-9, float(np.linalg.norm(right)))
        image_up = np.cross(right, view)
        image_up /= max(1e-9, float(np.linalg.norm(image_up)))
        camera_matrix = np.column_stack((right, image_up, -view))
        quat = np.zeros(4, dtype=float)
        mujoco.mju_mat2Quat(quat, camera_matrix.reshape(-1))
        self.model.cam_pos[self.front_cam_cid] = camera_pos
        self.model.cam_quat[self.front_cam_cid] = quat
        mujoco.mj_forward(self.model, self.data)

    def render_rgb(self, camera: str = "robot_cam") -> np.ndarray:
        if self.renderer is None:
            raise RuntimeError("MasterPiDynamicsV2 was created with render=False")
        if camera == "robot_cam":
            self._sync_real_camera_mount()
            self.renderer.update_scene(
                self.data,
                camera=camera,
                scene_option=self._robot_sensor_scene_option,
            )
            ideal = self.renderer.render().copy()
            if self._robot_fisheye_map is None:
                return ideal
            import cv2
            map_x, map_y = self._robot_fisheye_map
            return cv2.remap(
                ideal,
                map_x,
                map_y,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
        if camera == "cctv_front_left":
            self._sync_front_follow_camera()
        observer = self.observer_renderer or self.renderer
        observer.update_scene(self.data, camera=camera)
        return observer.render().copy()

    def body_xyz(self, name: str) -> np.ndarray:
        return np.array(self.data.xpos[self._body(name)], dtype=float)

    def set_free_body_pose_for_reset(self, name: str, xyz: Sequence[float], yaw: float = 0.0) -> None:
        """Set a free body's initial pose. Reset/curriculum helper, never an action."""
        bid = self._body(name)
        jid = int(self.model.body_jntadr[bid])
        if jid < 0 or int(self.model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
            raise ValueError(f"{name} is not a free body")
        qa = int(self.model.jnt_qposadr[jid])
        da = int(self.model.jnt_dofadr[jid])
        pos = np.asarray(xyz, dtype=float)
        if pos.shape != (3,) or not np.all(np.isfinite(pos)):
            raise ValueError("xyz must contain three finite values")
        self.data.qpos[qa:qa + 3] = pos
        self.data.qpos[qa + 3:qa + 7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        self.data.qvel[da:da + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def finger_block_contact(self, color: str = "red") -> dict:
        if color not in {"red", "blue", "yellow"}:
            raise ValueError("unsupported block color")
        block = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{color}_block_geom")
        left = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger")
        right = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger")
        hit_l = hit_r = False
        force_l = force_r = 0.0
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            if block not in (g1, g2):
                continue
            other = g2 if g1 == block else g1
            if other not in (left, right):
                continue
            wrench = np.zeros(6, dtype=float)
            if int(con.efc_address) >= 0:
                mujoco.mj_contactForce(self.model, self.data, i, wrench)
            normal = abs(float(wrench[0]))
            if other == left:
                hit_l = True
                force_l += normal
            elif other == right:
                hit_r = True
                force_r += normal
        return {
            "left": hit_l,
            "right": hit_r,
            "bilateral": bool(hit_l and hit_r),
            "left_force_n": force_l,
            "right_force_n": force_r,
        }

    def nonfinger_block_contact_force(self, color: str = "red") -> float:
        block = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{color}_block_geom")
        floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        fingers = {
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger"),
        }
        peak = 0.0
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            if block not in (g1, g2):
                continue
            other = g2 if g1 == block else g1
            if other == floor or other in fingers:
                continue
            wrench = np.zeros(6, dtype=float)
            if int(con.efc_address) >= 0:
                mujoco.mj_contactForce(self.model, self.data, i, wrench)
            peak = max(peak, abs(float(wrench[0])))
        return peak

    def robot_body_ids(self) -> list[int]:
        out = []
        for bid in range(self.model.nbody):
            cur = bid
            while cur > 0:
                if cur == self.robot_bid:
                    out.append(bid)
                    break
                cur = int(self.model.body_parentid[cur])
        return out

    @property
    def robot_mass_kg(self) -> float:
        return float(sum(float(self.model.body_mass[bid]) for bid in self.robot_body_ids()))

    def reset(self) -> dict:
        mujoco.mj_resetData(self.model, self.data)
        self.motor_state[:] = 0.0
        self.motor_command[:] = 0.0
        self.servo_command_pulses.clear()
        # A real, commonly used floor-search pose, not a fabricated simulator pose.
        self.set_servo_pulses({1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}, forward_only=True)
        # Settle gravity/contact with motors off.  This is intentionally physical:
        # the chassis is allowed to develop small z/roll/pitch motion on its wheels.
        for _ in range(int(0.35 / self.model.opt.timestep)):
            self._physics_step(np.zeros(4))
        return self.state()

    def pulse_to_joint_targets(self, pose: Mapping[int, int]) -> dict[str, float]:
        def nominal(servo: int) -> float:
            return float(pose[servo] - SERVO_DEVIATION.get(servo, 0))

        out: dict[str, float] = {}
        if 6 in pose:
            # REAL yaw/bearing control uses raw servo-6 PWM around BASE_CENTER;
            # preserve that convention at the hardware boundary. Hand-eye yaw
            # offsets belong in the controller calibration, not this actuator map.
            out["yaw"] = math.radians((float(pose[6]) - float(self.physical_params["servo6_center_pwm"])) / PULSE_PER_DEGREE)
        if 5 in pose:
            theta5 = 90.0 - (nominal(5) - 1500.0) / PULSE_PER_DEGREE
            out["shoulder"] = math.radians(theta5)
        if 4 in pose:
            theta4 = (nominal(4) - 1500.0) / PULSE_PER_DEGREE
            out["elbow"] = math.radians(-theta4)
        if 3 in pose:
            theta3 = (nominal(3) - 1500.0) / PULSE_PER_DEGREE
            out["wrist"] = math.radians(theta3)
        if 1 in pose:
            # Physical controller: ~2000=open, ~1500=closed. V2 gives the
            # symmetric jaws enough travel to close around the REAL 30 mm cube.
            closure = max(0.0, min(1.0, (2000.0 - float(pose[1])) / 500.0))
            out["gripper"] = closure * GRIPPER_MAX_CLOSE_M
        return out

    def set_servo_pulses(self, pose: Mapping[int, int], *, forward_only: bool = False) -> dict[str, float]:
        cleaned = {int(k): int(max(500, min(2500, int(v)))) for k, v in pose.items()}
        self.servo_command_pulses.update(cleaned)
        targets = self.pulse_to_joint_targets(cleaned)
        for name, value in targets.items():
            if name == "gripper":
                for aid, jid in zip(self.gripper_act, self.gripper_joint):
                    lo, hi = (float(v) for v in self.model.jnt_range[jid])
                    clipped = max(lo, min(hi, value)) if int(self.model.jnt_limited[jid]) else value
                    self.data.ctrl[aid] = clipped
                    if forward_only:
                        qadr = int(self.model.jnt_qposadr[jid])
                        self.data.qpos[qadr] = clipped
                continue
            aid = self.servo_act[name]
            jid = self.arm_joint[name]
            lo, hi = (float(v) for v in self.model.jnt_range[jid])
            if int(self.model.jnt_limited[jid]):
                value = max(lo, min(hi, value))
            self.data.ctrl[aid] = value
            if forward_only:
                qadr = int(self.model.jnt_qposadr[jid])
                self.data.qpos[qadr] = value
        if forward_only:
            mujoco.mj_forward(self.model, self.data)
        return targets

    def _servo_effective_target_duration(self, start: int, target: int, requested_s: float) -> tuple[int, float]:
        deadband = max(0.0, float(self.physical_params.get("servo_deadband_pwm", 0.0)))
        rate = max(1.0, float(self.physical_params.get("servo_rate_pwm_per_s", 2000.0)))
        delta = abs(float(target) - float(start))
        if delta <= deadband:
            return int(start), float(requested_s)
        return int(target), max(float(requested_s), delta / rate)

    def move_servos_timed(self, pose: Mapping[int, int], duration_s: float) -> dict:
        """Move several PWM axes concurrently through the calibrated servo plant."""
        duration_s = float(duration_s)
        if duration_s <= 0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        requested = {int(k): int(max(500, min(2500, int(v)))) for k, v in pose.items()}
        start = {servo: int(self.servo_command_pulses.get(servo, pulse)) for servo, pulse in requested.items()}
        targets: dict[int, int] = {}
        durations: dict[int, float] = {}
        for servo, pulse in requested.items():
            targets[servo], durations[servo] = self._servo_effective_target_duration(start[servo], pulse, duration_s)
        total = max(durations.values(), default=duration_s)
        steps = max(1, int(round(total / float(self.model.opt.timestep))))
        for i in range(steps):
            elapsed = min(total, float(i + 1) * total / float(steps))
            command = {}
            for servo, pulse in targets.items():
                u = min(1.0, elapsed / max(1e-9, durations[servo]))
                command[servo] = int(round((1.0 - u) * start[servo] + u * pulse))
            self.set_servo_pulses(command)
            self._physics_step()
        self.set_servo_pulses(targets)
        return self.state()

    def move_servo_timed(self, servo: int, pulse: int, duration_s: float, *, settle_s: float = 0.0) -> dict:
        """Apply one physical servo command over its requested travel duration.

        The real Hiwonder servo controller interpolates from the current PWM to
        the requested PWM during `duration_s`; setting a MuJoCo position target
        instantaneously creates accelerations the hardware never produces.
        """
        servo = int(servo)
        pulse = int(max(500, min(2500, int(pulse))))
        duration_s = float(duration_s)
        settle_s = float(settle_s)
        if duration_s <= 0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        if settle_s < 0 or not math.isfinite(settle_s):
            raise ValueError("settle_s must be finite and non-negative")
        start = int(self.servo_command_pulses.get(servo, pulse))
        effective_target, effective_duration = self._servo_effective_target_duration(start, pulse, duration_s)
        steps = max(1, int(round(effective_duration / float(self.model.opt.timestep))))
        for i in range(steps):
            u = float(i + 1) / float(steps)
            current = int(round((1.0 - u) * start + u * effective_target))
            self.set_servo_pulses({servo: current})
            self._physics_step()
        self.set_servo_pulses({servo: effective_target})
        if settle_s > 0:
            for _ in range(max(1, int(round(settle_s / float(self.model.opt.timestep))))):
                self._physics_step()
        return self.state()

    def move_pose_timed(self, target: Mapping[int, int], *, lowering: bool, settle_s: float = 0.15) -> dict:
        """Mirror scripts/red_block/Robot.move_pose ordering and duration law."""
        lower_rank = {1: 0, 6: 1, 3: 2, 4: 3, 5: 4}
        raise_rank = {5: 0, 4: 1, 3: 2, 6: 3, 1: 4}
        rank = lower_rank if lowering else raise_rank
        for servo in sorted((int(k) for k in target), key=rank.__getitem__):
            pulse = int(max(500, min(2500, int(target[servo]))))
            previous = self.servo_command_pulses.get(servo)
            if previous == pulse:
                continue
            duration = 1.5 if previous is None else min(2.5, max(0.4, abs(pulse - previous) / 800.0))
            self.move_servo_timed(servo, pulse, duration, settle_s=settle_s)
        return self.state()

    def set_motor_commands(self, commands: Sequence[float]) -> None:
        arr = np.asarray(commands, dtype=float)
        if arr.shape != (4,) or not np.all(np.isfinite(arr)):
            raise ValueError("motor commands must be four finite values")
        self.motor_command[:] = np.clip(arr, -1.0, 1.0)

    def _physics_step(self, commands: np.ndarray | None = None) -> None:
        dt = float(self.model.opt.timestep)
        if commands is not None:
            self.motor_command[:] = np.clip(np.asarray(commands, dtype=float), -1.0, 1.0)
        alpha = 1.0 - math.exp(-dt / self.dynamics["motor_time_constant_s"])
        self.motor_state += alpha * (self.motor_command - self.motor_state)
        self.data.ctrl[self.wheel_act] = self.motor_state * MAX_WHEEL_RAD_S

        # ABAB reduced mecanum traction.  The action remains four physical motor
        # commands; only the unresolved roller/contact microgeometry is reduced
        # to a calibrated body wrench.
        fwd = float(np.dot(self.motor_state, FORWARD_PATTERN) / 4.0)
        left = float(np.dot(self.motor_state, LEFT_PATTERN) / 4.0)
        yaw_cmd = float(np.dot(self.motor_state, YAW_LEFT_PATTERN) / 4.0)

        qvel = self.data.qvel[self.base_dadr:self.base_dadr + 6]
        vx_w, vy_w = float(qvel[0]), float(qvel[1])
        wz = float(qvel[5])
        _, _, yaw = self.base_rpy()
        cy, sy = math.cos(yaw), math.sin(yaw)
        vx_local = cy * vx_w + sy * vy_w
        vy_local = -sy * vx_w + cy * vy_w

        stopped_command = float(np.max(np.abs(self.motor_command))) < 1e-6
        linear_damping = self.dynamics[
            "stop_linear_damping_n_per_mps" if stopped_command else "linear_damping_n_per_mps"
        ]
        yaw_damping = self.dynamics[
            "stop_yaw_damping_nm_per_radps" if stopped_command else "yaw_damping_nm_per_radps"
        ]
        fx_local = self.dynamics["max_forward_force_n"] * fwd - linear_damping * vx_local
        fy_local = self.dynamics["max_lateral_force_n"] * left - linear_damping * vy_local
        tz_local = self.dynamics["max_yaw_torque_nm"] * yaw_cmd - yaw_damping * wz
        fx_world = cy * fx_local - sy * fy_local
        fy_world = sy * fx_local + cy * fy_local

        self.data.xfrc_applied[self.robot_bid, :] = 0.0
        self.data.xfrc_applied[self.robot_bid, 0] = fx_world
        self.data.xfrc_applied[self.robot_bid, 1] = fy_world
        self.data.xfrc_applied[self.robot_bid, 5] = tz_local
        mujoco.mj_step(self.model, self.data)

    def step(self, *, motor_commands: Sequence[float] | None = None, duration_s: float = .02) -> dict:
        if duration_s <= 0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        if motor_commands is not None:
            self.set_motor_commands(motor_commands)
        steps = max(1, int(round(duration_s / float(self.model.opt.timestep))))
        for _ in range(steps):
            self._physics_step()
        return self.state()

    def stop(self, *, settle_s: float = .25) -> dict:
        self.motor_command[:] = 0.0
        return self.step(duration_s=settle_s)

    def base_rpy(self) -> tuple[float, float, float]:
        q = self.data.qpos[self.base_qadr + 3:self.base_qadr + 7]
        return _quat_to_rpy(q)

    def base_xyz(self) -> np.ndarray:
        return np.array(self.data.qpos[self.base_qadr:self.base_qadr + 3], dtype=float)

    def site_xyz(self, name: str) -> np.ndarray:
        return np.array(self.data.site_xpos[self._site(name)], dtype=float)

    def set_base_pose_for_test(self, xyz=(0.0, 0.0, WHEEL_RADIUS_M), yaw: float = 0.0) -> None:
        """Deterministic test/reset helper; not a training action."""
        self.data.qpos[self.base_qadr:self.base_qadr + 3] = np.asarray(xyz, dtype=float)
        self.data.qpos[self.base_qadr + 3:self.base_qadr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        self.data.qvel[self.base_dadr:self.base_dadr + 6] = 0
        mujoco.mj_forward(self.model, self.data)

    def contact_summary(self) -> dict:
        floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        wheel_ids = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"wheel_{n}") for n in self.wheel_names}
        wheel_floor = 0
        robot_world = 0
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            pair = {int(con.geom1), int(con.geom2)}
            if floor in pair and pair & wheel_ids:
                wheel_floor += 1
            if floor in pair:
                robot_world += 1
        return {"contacts": int(self.data.ncon), "wheel_floor_contacts": wheel_floor, "floor_contacts": robot_world}

    def state(self) -> dict:
        roll, pitch, yaw = self.base_rpy()
        xyz = self.base_xyz()
        return {
            "base_xyz": [round(float(v), 6) for v in xyz],
            "base_rpy": [round(float(v), 6) for v in (roll, pitch, yaw)],
            "motor_command": [round(float(v), 4) for v in self.motor_command],
            "motor_state": [round(float(v), 4) for v in self.motor_state],
            "robot_mass_kg": round(self.robot_mass_kg, 6),
            "camera_vfov_deg": float(CAMERA_PINHOLE_VFOV_DEG),
            "camera_pinhole_hfov_deg": float(CAMERA_PINHOLE_HFOV_DEG),
            "camera_pinhole_vfov_deg": float(CAMERA_PINHOLE_VFOV_DEG),
            "camera_controller_projection_vfov_deg": float(CAMERA_VFOV_DEG),
            "camera_calibration_id": CAMERA_CALIBRATION_ID,
            "camera_mount_status": CAMERA_MOUNT_STATUS,
            "camera_raw_fisheye": True,
            "contacts": self.contact_summary(),
            "model": "masterpi_dynamics_v2_reduced_mecanum",
            "calibration_status": self.calibration_status,
            "calibration_parameters": dict(self.calibration_parameters),
            "physical_parameters": dict(self.physical_params),
        }
