#!/usr/bin/env python3
"""Camera-guided placement of a runtime-selected block on another block.

Safety contract:
- starts only when the persisted gripper command says the jaws are closed;
- on live hardware also requires a fresh causal handoff from precision pick;
- keeps the carried block high while searching/turning/approaching;
- uses stopped, fresh camera measurements before every chassis pulse;
- never uses stale pre-pick map coordinates as metric truth;
- opens the gripper exactly once, only after a calibrated IK plan exists;
- never auto-regrasps after release; failed visual verification returns non-zero.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from camera import DEBUG_PATH, capture_bgr, detect_target_blob, save_debug_frame
from carry_handoff import require_carry_handoff
from deploy import DEFAULT_HOST, deploy_and_run
from geometry import (
    BlockEstimate,
    CALIBRATED_FINGERTIP_RADIUS_MAX_CM,
    CALIBRATED_FINGERTIP_RADIUS_MIN_CM,
    DEFAULT_BLOCK_HEIGHT_CM,
    CAMERA_OPTICAL_CENTER_NX,
    CAMERA_OPTICAL_CENTER_NY,
    PULSE_PER_DEGREE,
    calculate_place_plan,
    estimate_block,
    median_absolute_deviation,
    median_block_estimate,
)
from poses import BASE_CENTER, GRIPPER_CLOSED, GRIPPER_ID, GRIPPER_OPEN, POSE_SEARCH, clamp_pulse, pose_with_base
from robot import Robot, is_robot


DELIVERY_PAN_MIN = 1300
DELIVERY_PAN_MAX = 1700
DELIVERY_TILT_MIN = 650
DELIVERY_TILT_MAX = 1160
DELIVERY_SCAN_PANS = (1500, 1320, 1680, 1500)
DELIVERY_SCAN_TILTS = (740, 650, 880, 1040)
DELIVERY_SCAN_POSE = {3: POSE_SEARCH[3], 4: POSE_SEARCH[4], 5: POSE_SEARCH[5]}
TARGET_MIN_AREA = 800
CENTER_DEADBAND = 0.045
CENTER_CONFIRMATIONS = 3
MAX_CENTER_FRAMES = 45
BODY_PAN_DEADBAND = 8
BODY_TURN_SPEED = 35
BODY_TURN_SECONDS = 0.12
MAX_BODY_SEARCH_PULSES = 28
MAX_BODY_ALIGN_PULSES = 24
# Carrying is different from empty-gripper approach: repeated 35-speed
# start/stop pulses can ratchet a friction grasp downward.  Use the lowest
# physically measured-effective chassis command and cover range with fewer,
# longer bounded segments, always remeasuring from rest between segments.
CARRY_APPROACH_SPEED = 35
CARRY_BODY_CENTER_DEADBAND = 0.060
CARRY_BODY_BEARING_DEADBAND_DEG = 0.75
CARRY_BODY_COARSE_BEARING_DEG = 10.0
CARRY_BODY_COARSE_TURN_SECONDS = 0.34
CARRY_BODY_FINE_TURN_SECONDS = 0.16
# Near the destination, the 30-mm cube reaches the lower image boundary before
# the arm enters the calibrated 14.0..17.5 cm placement band.  Do not keep
# driving from a clipped bbox: tilt only the wrist farther down while preserving
# shoulder/elbow/base and the closed gripper.  This keeps the same grasp but
# moves the eye-in-hand near field closer to the robot.
CARRY_NEAR_VIEW_TRIGGER_NY = 0.66
CARRY_TRANSPORT_WRIST_PULSE = 1400
# 1550 is the minimum measured-geometry view that keeps a ~21 cm destination
# floor edge comfortably inside the raw 480 px image.  The old 1650 near view
# lowered the held cube ~1.8 cm farther than necessary and consumed grip margin.
CARRY_NEAR_VIEW_WRIST_PULSE = 1550
CARRY_NEAR_VIEW_DURATION = 0.30
# After the final chassis trim, use a slightly steeper view so a 17-19 cm cube
# is fully visible, but only after the chassis has stopped.
CARRY_CLOSE_VIEW_WRIST_PULSE = 1625
CARRY_CLOSE_VIEW_DURATION = 0.30
CARRY_VIEW_BOTTOM_TRIGGER = 0.94
MAX_APPROACH_PULSES = 16
CAMERA_SETTLE_SECONDS = 0.16
# Three independent stopped frames are enough for the existing median+MAD gate
# and reduce time spent hanging from a friction-only grasp.
MEASURE_SAMPLES = 3
MEASURE_MIN_SAMPLES = 3
CARRY_LOW_VIEW_MEASURE_SAMPLES = 1
RANGE_MAD_MAX_CM = 1.0
LATERAL_MAD_MAX_CM = 0.55
# A near/close wrist transition moves the camera monotonically toward the
# destination.  The true destination must not collapse into a tiny component
# after that transition; accepting such a switch would steer the held block
# using robot/self geometry.  These are ratios, not a scene/seed location.
DESTINATION_VIEW_MIN_AREA_RATIO = 0.50
DESTINATION_VIEW_MIN_SHORT_SIDE_RATIO = 0.65
TARGET_RADIUS_MIN_CM = CALIBRATED_FINGERTIP_RADIUS_MIN_CM - 0.5
TARGET_RADIUS_MAX_CM = CALIBRATED_FINGERTIP_RADIUS_MAX_CM - 0.5
# Delivery mirrors the successful REAL pickup strategy: vision stages the target
# just outside the final manipulation envelope, then one bounded straight trim is
# executed with the wrist in the high transport pose.  Do not lower the wrist a
# second time after this trim; that transition was the reproduced carry-drop.
TERMINAL_TRIM_ENTRY_MAX_CM = TARGET_RADIUS_MAX_CM + 8.0
TERMINAL_STAGED_TARGET_RADIUS_CM = 17.0
GRIPPER_CLOSED_MAX_PULSE = 1700
VERIFY_FRAMES = 7
VERIFY_MIN_HITS = 4
VERIFY_MAX_X_ERROR = 0.10
VERIFY_MAX_JOIN_GAP = 0.12
# A correctly stacked upper cube can completely occlude the destination in the
# close release view.  Before moving the arm away, verify the causal support
# silhouette instead of requiring both colors simultaneously.  The supported
# upper cube is centered on the calibrated optical axis, remains above the image
# bottom, and appears as a squat visible upper face.  A dropped floor cube in
# the reproduced miss is tall and clipped at the bottom, so it fails these gates.
RELEASE_VERIFY_FRAMES = 5
RELEASE_VERIFY_MIN_HITS = 3
RELEASE_VERIFY_MAX_DESTINATION_HITS = 1
RELEASE_VERIFY_MAX_X_ERROR = 0.075
RELEASE_VERIFY_MIN_TOP_NY = 0.64
RELEASE_VERIFY_MAX_BOTTOM_NY = 0.985
RELEASE_VERIFY_MAX_HEIGHT_WIDTH_RATIO = 0.50
RELEASE_VERIFY_MAX_CENTER_SPREAD = 0.025
# Reuse the deploy helper's standard debug path so a failed REAL place copies
# its last annotated camera frame back to the Oracle host automatically.
DEBUG_PLACE = DEBUG_PATH


class PlaceError(RuntimeError):
    pass


def carry_body_turn_for_bearing_deg(bearing_deg: float) -> tuple[str, int, float] | None:
    """Choose one feedback-bounded body turn while carrying.

    Repeated 60-ms turn pulses were a major source of grip ratcheting.  Use the
    calibrated camera bearing only for direction/coarse-vs-fine tier, then
    remeasure after each bounded turn.
    """
    bearing = float(bearing_deg)
    if not math.isfinite(bearing):
        raise PlaceError("non-finite destination bearing")
    magnitude = abs(bearing)
    if magnitude <= CARRY_BODY_BEARING_DEADBAND_DEG:
        return None
    direction = "rotate-left" if bearing > 0.0 else "rotate-right"
    duration = CARRY_BODY_COARSE_TURN_SECONDS if magnitude > CARRY_BODY_COARSE_BEARING_DEG else CARRY_BODY_FINE_TURN_SECONDS
    return direction, CARRY_APPROACH_SPEED, duration


def carry_approach_motion(radius_cm: float) -> tuple[str, int, float] | None:
    """Choose a low-start-count carried-object chassis segment from fresh range.

    A friction grasp is damaged mainly by repeated acceleration/deceleration
    cycles.  At the minimum effective wheel command, use a long far segment and
    only one or two shorter trim segments, remeasuring from rest between them.
    """
    radius = float(radius_cm)
    if not math.isfinite(radius):
        raise PlaceError("non-finite destination range")
    if TARGET_RADIUS_MIN_CM <= radius <= TARGET_RADIUS_MAX_CM:
        return None
    if radius > TARGET_RADIUS_MAX_CM:
        excess = radius - TARGET_RADIUS_MAX_CM
        if excess > 18.0:
            duration = 0.60
        elif excess > 8.0:
            duration = 0.42
        elif excess > 3.0:
            duration = 0.18
        else:
            duration = 0.10
        return "forward", CARRY_APPROACH_SPEED, duration
    return "backward", CARRY_APPROACH_SPEED, 0.12


def _blocking_wrist(robot: Robot, target: int, duration: float, reason: str) -> bool:
    current = int(robot.pose.get(5, target))
    target = int(target)
    if current == target:
        return False
    pulse = robot.pose.get(GRIPPER_ID)
    if pulse is None or int(pulse) > GRIPPER_CLOSED_MAX_PULSE:
        raise PlaceError(f"refusing {reason} wrist move without a closed gripper")
    robot.move_servo(5, target, duration)
    if not robot.dry_run:
        time.sleep(CAMERA_SETTLE_SECONDS)
    print(f"delivery {reason} wrist {current}->{target}; reacquire target", flush=True)
    return True


def prepare_carry_chassis_motion(robot: Robot) -> bool:
    """Return to the high transport wrist before any chassis acceleration."""
    current = int(robot.pose.get(5, CARRY_TRANSPORT_WRIST_PULSE))
    if current == CARRY_TRANSPORT_WRIST_PULSE:
        return False
    duration = max(0.20, min(0.45, abs(current - CARRY_TRANSPORT_WRIST_PULSE) / 900.0))
    return _blocking_wrist(robot, CARRY_TRANSPORT_WRIST_PULSE, duration, "transport")


def set_post_motion_delivery_view(robot: Robot, *, close: bool) -> bool:
    """Select a stopped observation pose after a carried chassis segment."""
    if close:
        return _blocking_wrist(robot, CARRY_CLOSE_VIEW_WRIST_PULSE, CARRY_CLOSE_VIEW_DURATION, "close-view")
    return _blocking_wrist(robot, CARRY_NEAR_VIEW_WRIST_PULSE, CARRY_NEAR_VIEW_DURATION, "near-view")


def restore_post_motion_delivery_view(robot: Robot, previous_wrist: int) -> bool:
    """Restore a low observation view after a chassis segment, if one was active.

    Chassis acceleration must always happen at the high transport wrist, but a
    close-range destination can disappear from that camera pose.  Remembering
    only the commanded pre-motion wrist is actor-visible state and lets the
    controller return to the same near/close observation class before asking
    vision to reacquire.  Intermediate/high wrist positions are deliberately
    not lowered further.
    """
    previous_wrist = int(previous_wrist)
    if previous_wrist >= CARRY_CLOSE_VIEW_WRIST_PULSE:
        return set_post_motion_delivery_view(robot, close=True)
    if previous_wrist >= CARRY_NEAR_VIEW_WRIST_PULSE:
        return set_post_motion_delivery_view(robot, close=False)
    return False


def maybe_enter_near_delivery_view(robot: Robot, blob) -> bool:
    """Move only as far down as needed to keep the destination floor edge visible."""
    if blob is None:
        return False
    current = int(robot.pose.get(5, CARRY_TRANSPORT_WRIST_PULSE))
    bottom_ny = float(getattr(blob, "ny", 0.0)) + float(getattr(blob, "height", 0.0)) / (2.0 * 480.0)
    if current < CARRY_NEAR_VIEW_WRIST_PULSE and (
        float(getattr(blob, "ny", 0.0)) >= CARRY_NEAR_VIEW_TRIGGER_NY
        or bottom_ny >= CARRY_VIEW_BOTTOM_TRIGGER
    ):
        return _blocking_wrist(robot, CARRY_NEAR_VIEW_WRIST_PULSE, CARRY_NEAR_VIEW_DURATION, "near-view")
    if current < CARRY_CLOSE_VIEW_WRIST_PULSE and bottom_ny >= CARRY_VIEW_BOTTOM_TRIGGER:
        return _blocking_wrist(robot, CARRY_CLOSE_VIEW_WRIST_PULSE, CARRY_CLOSE_VIEW_DURATION, "close-view")
    return False


def destination_observation_consistent(reference, candidate) -> bool:
    """Check a post-near-view destination observation against its prior view.

    This is an actor-visible continuity check.  When the wrist is moved closer
    to a floor target, a genuine block remains similarly sized or grows; a
    narrow robot/self fragment must not replace it as the delivery reference.
    The check intentionally does not constrain image position because the
    camera command itself can change that position substantially.
    """
    if reference is None or candidate is None:
        return False
    reference_area = float(getattr(reference, "area", 0.0) or 0.0)
    candidate_area = float(getattr(candidate, "area", 0.0) or 0.0)
    reference_short = min(
        float(getattr(reference, "width", 0.0) or 0.0),
        float(getattr(reference, "height", 0.0) or 0.0),
    )
    candidate_short = min(
        float(getattr(candidate, "width", 0.0) or 0.0),
        float(getattr(candidate, "height", 0.0) or 0.0),
    )
    if reference_area <= 0.0 or reference_short <= 0.0:
        return False
    return bool(
        candidate_area >= reference_area * DESTINATION_VIEW_MIN_AREA_RATIO
        and candidate_short >= reference_short * DESTINATION_VIEW_MIN_SHORT_SIDE_RATIO
    )


def _delivery_pose(pan: int = BASE_CENTER, tilt: int | None = None, *, gripper: int = GRIPPER_CLOSED) -> dict[int, int]:
    pose = pose_with_base(DELIVERY_SCAN_POSE, clamp_pulse(pan, DELIVERY_PAN_MIN, DELIVERY_PAN_MAX), gripper)
    if tilt is not None:
        pose[3] = clamp_pulse(tilt, DELIVERY_TILT_MIN, DELIVERY_TILT_MAX)
    return pose


def _require_closed_gripper(robot: Robot, target_color: str = "red") -> None:
    pulse = robot.pose.get(GRIPPER_ID)
    if pulse is None:
        raise PlaceError("persisted gripper pose is unavailable; refusing a release sequence")
    if pulse > GRIPPER_CLOSED_MAX_PULSE:
        raise PlaceError(f"gripper is not closed (servo1={pulse}); refusing placement without a held-block precondition")
    if robot.dry_run:
        return
    try:
        handoff_args = {
            "current_gripper_pulse": int(pulse),
            "current_robot_pose": robot.pose,
        }
        if target_color != "red":
            handoff_args["target_color"] = target_color
        require_carry_handoff(**handoff_args)
    except RuntimeError as exc:
        raise PlaceError(str(exc)) from exc


def _detect(color: str, *, quiet: bool = True):
    frame = capture_bgr(quiet=quiet)
    blob = detect_target_blob(frame, color, min_area=TARGET_MIN_AREA, crop_left=0)
    return frame, blob


def _move_delivery_gaze(robot: Robot, pan: int, tilt: int) -> None:
    pan = clamp_pulse(pan, DELIVERY_PAN_MIN, DELIVERY_PAN_MAX)
    tilt = clamp_pulse(tilt, DELIVERY_TILT_MIN, DELIVERY_TILT_MAX)
    robot.nudge_servos({6: pan, 3: tilt}, duration=0.12)
    if not robot.dry_run:
        time.sleep(CAMERA_SETTLE_SECONDS)


def scan_arm_for_target(robot: Robot, color: str):
    for tilt in DELIVERY_SCAN_TILTS:
        for pan in DELIVERY_SCAN_PANS:
            _move_delivery_gaze(robot, pan, tilt)
            frame, blob = _detect(color)
            if blob is not None:
                save_debug_frame(frame, blob, DEBUG_PLACE, label=color)
                return blob
    return None


def acquire_target(robot: Robot, color: str):
    """Search with the high closed-gripper pose; only guarded turn pulses widen FOV."""
    robot.move_pose(_delivery_pose(), lowering=False)
    for body_pulse in range(MAX_BODY_SEARCH_PULSES + 1):
        blob = scan_arm_for_target(robot, color)
        if blob is not None:
            print(f"{color} target acquired after body-search pulse {body_pulse}", flush=True)
            return blob
        if body_pulse >= MAX_BODY_SEARCH_PULSES:
            break
        robot.stop()
        robot.drive("rotate-left", BODY_TURN_SPEED, BODY_TURN_SECONDS)
        robot.stop()
        _move_delivery_gaze(robot, BASE_CENTER, POSE_SEARCH[3])
    raise PlaceError(f"{color} target not found in bounded delivery search")


def center_target(robot: Robot, color: str, *, confirmations: int = CENTER_CONFIRMATIONS):
    stable = 0
    misses = 0
    last_blob = None
    post_view_reference = None
    for _ in range(MAX_CENTER_FRAMES):
        frame, blob = _detect(color)
        if blob is None:
            misses += 1
            if misses >= 5:
                raise PlaceError(f"{color} target lost while centering")
            continue
        misses = 0
        if post_view_reference is not None:
            if not destination_observation_consistent(post_view_reference, blob):
                raise PlaceError(
                    f"{color} destination observation became inconsistent after near-view; "
                    "refusing to continue toward release"
                )
            post_view_reference = None
        if maybe_enter_near_delivery_view(robot, blob):
            post_view_reference = blob
            stable = 0
            misses = 0
            continue
        last_blob = blob
        error_x = CAMERA_OPTICAL_CENTER_NX - blob.nx
        error_y = CAMERA_OPTICAL_CENTER_NY - blob.ny
        if abs(error_x) <= CENTER_DEADBAND and abs(error_y) <= CENTER_DEADBAND:
            stable += 1
            if stable >= confirmations:
                save_debug_frame(frame, blob, DEBUG_PLACE, label=color)
                return blob
            continue
        stable = 0
        pan = robot.pose.get(6, BASE_CENTER)
        tilt = robot.pose.get(3, POSE_SEARCH[3])
        pan_step = int(round(error_x * 640.0 * 0.28))
        tilt_step = int(round(error_y * 480.0 * 0.28))
        pan_step = max(-24, min(24, pan_step))
        tilt_step = max(-20, min(20, tilt_step))
        if pan_step == 0 and abs(error_x) > CENTER_DEADBAND:
            pan_step = 1 if error_x > 0 else -1
        if tilt_step == 0 and abs(error_y) > CENTER_DEADBAND:
            tilt_step = 1 if error_y > 0 else -1
        next_pan = clamp_pulse(pan + pan_step, DELIVERY_PAN_MIN, DELIVERY_PAN_MAX)
        next_tilt = clamp_pulse(tilt + tilt_step, DELIVERY_TILT_MIN, DELIVERY_TILT_MAX)
        if next_pan == pan and abs(error_x) > CENTER_DEADBAND:
            raise PlaceError("delivery camera reached pan limit before centering target")
        _move_delivery_gaze(robot, next_pan, next_tilt)
    raise PlaceError(f"{color} target did not stabilize in camera center; last={last_blob}")


def align_body_to_target(robot: Robot, color: str):
    blob = center_target(robot, color)
    for pulse_number in range(MAX_BODY_ALIGN_PULSES + 1):
        pan = robot.pose.get(6, BASE_CENTER)
        error = pan - BASE_CENTER
        if abs(error) <= BODY_PAN_DEADBAND:
            return blob
        if pulse_number >= MAX_BODY_ALIGN_PULSES:
            break
        bearing_deg = float(error) / PULSE_PER_DEGREE
        turn = carry_body_turn_for_bearing_deg(bearing_deg)
        if turn is None:
            return blob
        direction, speed, duration = turn
        previous_wrist = int(robot.pose.get(5, CARRY_TRANSPORT_WRIST_PULSE))
        robot.stop()
        prepare_carry_chassis_motion(robot)
        robot.drive(direction, speed, duration)
        robot.stop()
        if not robot.dry_run:
            time.sleep(CAMERA_SETTLE_SECONDS)
        restore_post_motion_delivery_view(robot, previous_wrist)
        blob = center_target(robot, color)
    raise PlaceError("body alignment exceeded guarded turn-pulse limit")


def stationary_measurement(robot: Robot, color: str):
    """Measure from a stopped chassis, minimizing dwell in low carry views."""
    robot.stop()
    low_view = int(robot.pose.get(5, CARRY_TRANSPORT_WRIST_PULSE)) > CARRY_TRANSPORT_WRIST_PULSE
    # A blocking wrist view transition already includes CAMERA_SETTLE_SECONDS.
    # Do not add another gravity dwell while the cube is hanging in a low view.
    if not robot.dry_run and not low_view:
        time.sleep(CAMERA_SETTLE_SECONDS)
    sample_goal = CARRY_LOW_VIEW_MEASURE_SAMPLES if low_view else MEASURE_SAMPLES
    minimum = 1 if low_view else MEASURE_MIN_SAMPLES
    samples = []
    last_frame = None
    last_blob = None
    for _ in range(max(sample_goal * 3, sample_goal)):
        frame, blob = _detect(color)
        if blob is None:
            continue
        estimate = estimate_block(robot.pose, blob)
        if estimate is None:
            continue
        samples.append(estimate)
        last_frame = frame
        last_blob = blob
        if len(samples) >= sample_goal:
            break
    if len(samples) < minimum:
        raise PlaceError(f"insufficient stopped {color} coordinate samples ({len(samples)}/{minimum})")
    result = median_block_estimate(samples)
    range_mad = median_absolute_deviation([item.radius_cm for item in samples])
    lateral_mad = median_absolute_deviation([item.lateral_left_cm for item in samples])
    if range_mad > RANGE_MAD_MAX_CM or lateral_mad > LATERAL_MAD_MAX_CM:
        raise PlaceError(
            f"unstable {color} coordinate (range MAD={range_mad:.2f}cm, lateral MAD={lateral_mad:.2f}cm)"
        )
    if last_frame is not None:
        save_debug_frame(last_frame, last_blob, DEBUG_PLACE, label=color)
    print(json.dumps({
        "target": color,
        "estimate": asdict(result),
        "range_mad_cm": round(range_mad, 3),
        "lateral_mad_cm": round(lateral_mad, 3),
        "samples": len(samples),
        "low_view": low_view,
    }, sort_keys=True), flush=True)
    return result


def terminal_staged_estimate(measured: BlockEstimate) -> BlockEstimate:
    """Convert the last fresh close observation into the bounded final place setpoint.

    This is intentionally the same pattern used by REAL pickup's blind final
    grasp: RGB/FK supplies bearing, one bounded chassis trim supplies the final
    longitudinal staging, and the arm uses a conservative known setpoint inside
    the existing 18 cm fingertip envelope.  No simulator object position is read.
    """
    yaw = math.radians(float(measured.yaw_left_deg))
    radius = TERMINAL_STAGED_TARGET_RADIUS_CM
    return BlockEstimate(
        radius_cm=radius,
        lateral_left_cm=radius * math.sin(yaw),
        forward_cm=radius * math.cos(yaw),
        yaw_left_deg=float(measured.yaw_left_deg),
        camera_radius_cm=float(measured.camera_radius_cm),
        camera_height_cm=float(measured.camera_height_cm),
        ray_pitch_deg=float(measured.ray_pitch_deg),
        block_height_cm=DEFAULT_BLOCK_HEIGHT_CM,
        nx=float(measured.nx),
        ny=float(measured.ny),
    )


def approach_target(robot: Robot, color: str):
    """Approach only from fresh stopped measurements; never blind-latch a place target."""
    for pulse_number in range(MAX_APPROACH_PULSES + 1):
        align_body_to_target(robot, color)
        measured = stationary_measurement(robot, color)
        radius = measured.radius_cm
        motion = carry_approach_motion(radius)
        if motion is None:
            prepare_carry_chassis_motion(robot)
            return measured
        if pulse_number >= MAX_APPROACH_PULSES:
            break
        direction, speed, duration = motion
        robot.stop()
        prepare_carry_chassis_motion(robot)
        robot.drive(direction, speed, duration)
        robot.stop()
        # Far segments land in the ~20 cm observation region; final trim
        # segments need the steeper stopped close view.  This decision uses only
        # the fresh pre-motion RGB/FK range, never simulator object truth.
        terminal_trim = direction == "forward" and radius <= TERMINAL_TRIM_ENTRY_MAX_CM
        if terminal_trim:
            # The fresh close-view bearing remains valid for a straight trim.
            # Stay in the transport wrist; lowering for another image is more
            # destructive than the bounded open-loop longitudinal finish.
            if not robot.dry_run:
                time.sleep(CAMERA_SETTLE_SECONDS)
            return terminal_staged_estimate(measured)
        set_post_motion_delivery_view(robot, close=False)
        if not robot.dry_run:
            time.sleep(CAMERA_SETTLE_SECONDS)
    raise PlaceError(f"could not enter calibrated place radius [{TARGET_RADIUS_MIN_CM:.1f},{TARGET_RADIUS_MAX_CM:.1f}]cm")


def move_closed_grasp_arm_together(
    robot: Robot,
    pose: dict[int, int],
    *,
    duration: float,
) -> None:
    """Move shoulder/elbow/wrist as one interpolated carried-object motion.

    Sequentially changing 3->4->5 creates transient arm geometries that are not
    on the planned IK path and can unload a friction grasp.  The MasterPi PWM
    board accepts all three targets immediately and interpolates them in parallel,
    so issue one concurrent batch and wait for that interpolation to complete.
    """
    pulse = robot.pose.get(GRIPPER_ID)
    if pulse is None or int(pulse) > GRIPPER_CLOSED_MAX_PULSE:
        raise PlaceError("refusing carried arm transition without a closed gripper")
    updates = {servo: int(pose[servo]) for servo in (3, 4, 5)}
    robot.nudge_servos(updates, duration=duration)
    if not robot.dry_run:
        time.sleep(duration + 0.12)


def _bbox_edges(blob) -> tuple[float, float, float, float]:
    half_w = blob.width / (2.0 * 640.0)
    half_h = blob.height / (2.0 * 480.0)
    return blob.nx - half_w, blob.nx + half_w, blob.ny - half_h, blob.ny + half_h


def _release_support_candidate(blob) -> bool:
    """Return whether one release-view blob is consistent with a supported upper cube."""
    if blob is None:
        return False
    width = float(getattr(blob, "width", 0.0) or 0.0)
    height = float(getattr(blob, "height", 0.0) or 0.0)
    if width <= 1e-6 or height <= 1e-6:
        return False
    top_ny = float(blob.ny) - height / (2.0 * 480.0)
    bottom_ny = float(blob.ny) + height / (2.0 * 480.0)
    return bool(
        abs(float(blob.nx) - CAMERA_OPTICAL_CENTER_NX) <= RELEASE_VERIFY_MAX_X_ERROR
        and top_ny >= RELEASE_VERIFY_MIN_TOP_NY
        and bottom_ny <= RELEASE_VERIFY_MAX_BOTTOM_NY
        and height / width <= RELEASE_VERIFY_MAX_HEIGHT_WIDTH_RATIO
    )


def verify_release_support(robot: Robot, target_color: str, destination_color: str) -> bool:
    """Verify a newly released stack before the eye-in-hand camera moves away.

    This is deliberately actor-visible only.  A supported upper cube hides most
    or all of the lower destination from the close release camera.  Requiring the
    lower color to remain visible made the old verifier reject the physically
    stable stack.  We instead require a repeated, stable supported-face silhouette
    on the planned optical axis and reject any case where the destination remains
    plainly visible or the released cube is floor-clipped/tall.
    """
    if robot.dry_run:
        return False
    hits = 0
    destination_hits = 0
    centers: list[tuple[float, float]] = []
    last_frame = None
    last_blob = None
    for _ in range(RELEASE_VERIFY_FRAMES):
        frame = capture_bgr(quiet=True)
        carried = detect_target_blob(frame, target_color, min_area=800, crop_left=0)
        destination = detect_target_blob(frame, destination_color, min_area=800, crop_left=0)
        if destination is not None:
            destination_hits += 1
        if _release_support_candidate(carried):
            hits += 1
            centers.append((float(carried.nx), float(carried.ny)))
            last_frame, last_blob = frame, carried
        time.sleep(0.04)
    spread = 999.0
    if centers:
        xs = [p[0] for p in centers]
        ys = [p[1] for p in centers]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
    ok = bool(
        hits >= RELEASE_VERIFY_MIN_HITS
        and destination_hits <= RELEASE_VERIFY_MAX_DESTINATION_HITS
        and spread <= RELEASE_VERIFY_MAX_CENTER_SPREAD
    )
    print(
        f"release support verification hits={hits}/{RELEASE_VERIFY_FRAMES} "
        f"destination-visible={destination_hits} spread={spread:.3f} ok={ok}",
        flush=True,
    )
    if ok and last_frame is not None:
        save_debug_frame(last_frame, last_blob, DEBUG_PLACE, label=f"{target_color}-supported-on-{destination_color}")
    return ok


def verify_stack(robot: Robot, target_color: str, destination_color: str) -> bool:
    """Verify selected-object-over-destination geometry after release."""
    robot.move_pose(_delivery_pose(gripper=GRIPPER_OPEN), lowering=False)
    try:
        acquire_target_after_release = scan_arm_for_target(robot, destination_color)
    except Exception:
        acquire_target_after_release = None
    if acquire_target_after_release is None:
        return False
    try:
        center_target(robot, destination_color, confirmations=2)
    except PlaceError:
        return False

    hits = 0
    for _ in range(VERIFY_FRAMES):
        frame = capture_bgr(quiet=True)
        target = detect_target_blob(frame, destination_color, min_area=TARGET_MIN_AREA, crop_left=0)
        carried = detect_target_blob(frame, target_color, min_area=800, crop_left=0)
        if target is None or carried is None:
            continue
        carried_left, carried_right, _carried_top, carried_bottom = _bbox_edges(carried)
        tgt_left, tgt_right, tgt_top, _tgt_bottom = _bbox_edges(target)
        x_overlap = min(carried_right, tgt_right) - max(carried_left, tgt_left)
        x_error = abs(carried.nx - target.nx)
        join_gap = abs(carried_bottom - tgt_top)
        carried_is_above = carried.ny < target.ny
        if x_overlap > 0 and x_error <= VERIFY_MAX_X_ERROR and join_gap <= VERIFY_MAX_JOIN_GAP and carried_is_above:
            hits += 1
            save_debug_frame(frame, carried, DEBUG_PLACE, label=f"{target_color}-on-{destination_color}")
        if not robot.dry_run:
            time.sleep(0.04)
    print(f"stack verification {hits}/{VERIFY_FRAMES} frames", flush=True)
    return hits >= VERIFY_MIN_HITS


def execute_place(
    robot: Robot,
    target_color: str,
    destination_color: str,
    *,
    carried_height_cm: float = DEFAULT_BLOCK_HEIGHT_CM,
) -> None:
    if target_color == destination_color:
        raise PlaceError("target and destination colors must differ")
    _require_closed_gripper(robot, target_color)
    acquire_target(robot, destination_color)
    measured = approach_target(robot, destination_color)
    # These colour-sort task objects are the calibrated 30-mm cubes represented
    # by DEFAULT_BLOCK_HEIGHT_CM.  Perspective-derived blob height becomes badly
    # biased in the near-view fisheye image (the reproduced blue cube inflated
    # from 3 cm to 7.85 cm), while the lower edge still gives usable range.  Use
    # vision for x/y/range and the known physical object dimension for stack z.
    measured = replace(measured, block_height_cm=DEFAULT_BLOCK_HEIGHT_CM)
    plan = calculate_place_plan(measured, carried_height_cm=carried_height_cm)
    print(json.dumps({
        "place_plan": {
            "color": destination_color,
            "base_pulse": plan.base_pulse,
            "fingertip_radius_cm": round(plan.fingertip_radius_cm, 3),
            "target_height_cm": round(plan.target_height_cm, 3),
            "carried_height_cm": round(plan.carried_height_cm, 3),
            "release_height_cm": round(plan.release_height_cm, 3),
            "hover_height_cm": round(plan.hover_height_cm, 3),
            "hover_pose": plan.hover_pose,
            "release_pose": plan.release_pose,
        }
    }, sort_keys=True), flush=True)

    # Rotate over the measured target while the object is still high.  Servo 6
    # and joints 3/4/5 are independent PWM channels; issue one interpolated
    # batch instead of waiting for base yaw and then starting the arm.
    hover = dict(plan.hover_pose)
    hover[6] = plan.base_pulse
    hover[GRIPPER_ID] = GRIPPER_CLOSED
    release = dict(plan.release_pose)
    release[6] = plan.base_pulse
    release[GRIPPER_ID] = GRIPPER_CLOSED
    hover_updates = {servo: int(hover[servo]) for servo in (3, 4, 5, 6)}
    robot.nudge_servos(hover_updates, duration=1.10)
    if not robot.dry_run:
        time.sleep(1.22)
    if not robot.dry_run:
        time.sleep(0.18)
    move_closed_grasp_arm_together(robot, release, duration=0.85)
    if not robot.dry_run:
        time.sleep(0.22)
    print("release gripper", flush=True)
    robot.move_servo(GRIPPER_ID, GRIPPER_OPEN, 0.7)
    if not robot.dry_run:
        time.sleep(0.45)
    # Verify while the camera still has the causal release view.  Once the
    # eye-in-hand arm retracts, a close correctly stacked pair may disappear
    # entirely from the generic delivery scan.
    release_supported = False if robot.dry_run else verify_release_support(robot, target_color, destination_color)
    hover_open = dict(hover)
    hover_open[GRIPPER_ID] = GRIPPER_OPEN
    robot.move_pose(hover_open, lowering=False)
    robot.stop()

    if robot.dry_run:
        print("dry-run: release verification skipped", flush=True)
        return
    if not release_supported and not verify_stack(robot, target_color, destination_color):
        raise PlaceError(
            f"release completed but {target_color}-on-{destination_color} stack postcondition was not visually verified"
        )


def run_on_robot(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)

    def handle_signal(signum: int, _frame: object) -> None:
        try:
            robot.stop()
        finally:
            raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    robot.stop()
    print(json.dumps({"probe": robot.probe()}, sort_keys=True), flush=True)
    if args.locate_only:
        _require_closed_gripper(robot, args.target_color)
        acquire_target(robot, args.destination_color)
        robot.stop()
        print(f"located destination {args.destination_color} while carrying {args.target_color}", flush=True)
        return 0
    execute_place(
        robot, args.target_color, args.destination_color,
        carried_height_cm=args.carried_height_cm,
    )
    robot.stop()
    print(f"verified {args.target_color}_on_{args.destination_color}", flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="들고 있는 블록을 다른 색 블록 위에 배치합니다.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true", help="I2C/motor 명령 없이 계획 경로만 검사합니다")
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    parser.add_argument("--destination-color", choices=("red", "blue", "yellow"), required=True)
    parser.add_argument("--carried-height-cm", type=float, default=DEFAULT_BLOCK_HEIGHT_CM)
    parser.add_argument("--locate-only", action="store_true", help="집게를 열지 않고 목적 블록 탐색만 수행")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 0.8 <= args.carried_height_cm <= 10.0:
        raise ValueError("carried-height-cm must be in 0.8..10.0")
    if args.destination_color == args.target_color:
        raise ValueError("destination-color must differ from target-color")
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra = [
        "--target-color", args.target_color,
        "--destination-color", args.destination_color,
        "--carried-height-cm", str(args.carried_height_cm),
    ]
    if args.dry_run:
        extra.append("--dry-run")
    if args.locate_only:
        extra.append("--locate-only")
    return deploy_and_run("place.py", extra, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PlaceError, RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
