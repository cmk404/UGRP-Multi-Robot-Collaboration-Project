#!/usr/bin/env python3
"""Pick up a red block in front of MasterPi.

Mac에서 실행하면 이 폴더를 ugrp1에 복사한 뒤 Pi에서 돌립니다.

    python3 scripts/pick_red_block.py
    python3 scripts/red_block/pick.py --detect-only
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from camera import DEBUG_PATH, FRAME_PATH, capture_bgr, detect_target_blob, save_debug_frame
from carry_handoff import invalidate_carry_handoff, save_carry_handoff
from deploy import DEFAULT_HOST, deploy_and_run
from poses import GRIPPER_CLOSED
from robot import Robot, is_robot
from precision_handoff import (
    invalidate_pick_plan,
    load_coarse_handoff_payload,
    require_coarse_handoff_color,
    validate_coarse_handoff_pose,
)
from approach import (
    _bounded_face_direction_restage,
    _orient_chassis_for_straight_close,
    acquire_inherited_target,
    configure_target_detector,
)
from gaze_hold import GAZE_MODE_FIXATE
from track import Gaze, PAN_SERVO, TILT_SERVO


def look_for_block(frame_path: Path = FRAME_PATH, debug_path: Path = DEBUG_PATH, *, target_color: str = "red"):
    import cv2

    frame = capture_bgr()
    blob = detect_target_blob(frame, target_color)
    save_debug_frame(frame, blob, debug_path, label=target_color)
    cv2.imwrite(str(frame_path), frame)
    if blob is None:
        raise RuntimeError(f"{target_color} block not found; debug frame saved at {debug_path}")
    print(json.dumps({"blob": asdict(blob), "frame": str(frame_path)}, sort_keys=True), flush=True)
    return blob


def load_precision_controller():
    import physical_state_machine_reference as precision
    return precision


PICK_ARM_MAX_BEARING_DEG = 36.0
PICK_FACE_MAX_ERROR_DEG = 24.0
PICK_VISUAL_MAD_LIMIT = 0.012
PRECAPTURE_MOVE_SECONDS = 0.65
PRECAPTURE_SETTLE_SECONDS = 0.75



def _measure_arm_only_pick_plan(
    robot: Robot,
    video,
    precision,
    *,
    flip_x: bool,
    flip_y: bool,
):
    """Freeze the stopped pre-capture view and solve the grasp fresh."""
    capture_pose = getattr(precision, "CAPTURE_ARM_POSE", None)
    if not isinstance(capture_pose, dict):
        raise RuntimeError("precision controller has no capture arm pose")
    for servo, expected in capture_pose.items():
        actual = int(robot.pose.get(servo, -9999))
        if abs(actual - int(expected)) > 30:
            raise RuntimeError(
                f"near-field pick lost calibrated reach camera pose at servo {servo}: "
                f"expected {expected}, got {actual}; run approach to restage"
            )

    gaze = Gaze(
        pan=int(robot.pose.get(6, precision.BASE_CENTER)),
        tilt=int(robot.pose.get(3, capture_pose.get(3, 600))),
    )
    lock = acquire_inherited_target(precision, video, robot=robot, gaze=gaze)
    target_nx = float(getattr(precision, "CAPTURE_TARGET_NX", 0.5))
    gaze = precision.fine_align_horizontal(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        target_nx=target_nx,
    )
    robot.stop()
    video.settle(precision.CAMERA_DELAY_SECONDS + precision.SERVO_SETTLE_SECONDS)

    blobs = []
    for _ in range(precision.FINAL_SAMPLE_COUNT * 3):
        _frame, blob = precision.read_locked(video, lock)
        if blob is None:
            continue
        blobs.append(blob)
        if len(blobs) >= precision.FINAL_SAMPLE_COUNT:
            break
        time.sleep(0.025)
    if len(blobs) < precision.FINAL_MIN_VALID_SAMPLES:
        raise RuntimeError(
            f"only {len(blobs)} valid near-field pick visual samples; run approach to restage"
        )

    nys = [float(item.ny) for item in blobs]
    nxs = [float(item.nx) for item in blobs]
    ny = float(statistics.median(nys))
    nx = float(statistics.median(nxs))
    ny_mad = float(precision.median_absolute_deviation(nys))
    nx_mad = float(precision.median_absolute_deviation(nxs))
    if ny_mad > PICK_VISUAL_MAD_LIMIT or nx_mad > PICK_VISUAL_MAD_LIMIT:
        raise RuntimeError(
            f"near-field pick visual coordinate is unstable "
            f"(nx MAD={nx_mad:.3f}, ny MAD={ny_mad:.3f}); run approach to restage"
        )
    target_ny = float(precision.CAPTURE_TARGET_NY)
    capture_tol = float(precision.CAPTURE_TARGET_TOLERANCE_NY)
    capture_high = min(
        float(precision.CAPTURE_TOO_CLOSE_NY) - 0.005,
        target_ny + capture_tol + 0.015,
    )
    if ny < target_ny - capture_tol - 0.015 or ny > capture_high:
        raise RuntimeError(
            "near-field pick target is outside calibrated stopped capture window "
            f"(ny={ny:.3f}, target={target_ny:.3f}); run approach to restage"
        )

    block_radius = float(precision.CAPTURE_BLOCK_RADIUS_CM)
    pixel_left_deg = (target_nx - nx) * precision.CAMERA_HFOV_DEG
    servo_left_deg = (
        int(robot.pose[6]) - precision.BASE_CENTER
    ) / precision.PULSE_PER_DEGREE
    yaw_left_deg = servo_left_deg + pixel_left_deg
    if abs(yaw_left_deg) > PICK_ARM_MAX_BEARING_DEG:
        raise RuntimeError(
            f"near-field pick target bearing {yaw_left_deg:+.1f}deg exceeds "
            f"servo-6 corridor +/-{PICK_ARM_MAX_BEARING_DEG:.1f}deg; "
            "run approach to re-aim chassis"
        )

    # Face direction is verified from the same stopped camera geometry.  Any
    # chassis face-normal correction already happened at ~26 cm before the
    # close view; this final gate is verification-only.
    measure_face = getattr(precision, "measure_arm_face_alignment", None)
    if callable(measure_face):
        try:
            face, _bearing, _signed, _face_blob = measure_face(
                robot,
                video,
                lock,
                gaze,
                flip_x=flip_x,
                target_nx=target_nx,
            )
        except RuntimeError as exc:
            if exc.__class__.__name__ == "LockedTargetLost":
                raise
            print(
                f"near-field pick: stable face cue unavailable ({exc}); "
                "continue with center/bearing grasp gate",
                flush=True,
            )
        else:
            if float(face.error_deg) > PICK_FACE_MAX_ERROR_DEG:
                raise RuntimeError(
                    f"near-field pick face error {face.error_deg:.1f}deg exceeds "
                    f"{PICK_FACE_MAX_ERROR_DEG:.1f}deg; run approach to restage face direction"
                )
            print(
                f"near-field pick face verified error={face.error_deg:.1f}deg",
                flush=True,
            )

    yaw = math.radians(yaw_left_deg)
    camera = precision.forward_kinematics(robot.pose, precision.CAMERA_LINK_CM)
    block = precision.BlockEstimate(
        radius_cm=block_radius,
        lateral_left_cm=block_radius * math.sin(yaw),
        forward_cm=block_radius * math.cos(yaw),
        yaw_left_deg=yaw_left_deg,
        camera_radius_cm=camera.radius_cm,
        camera_height_cm=camera.height_cm + precision.CAMERA_Z_OFFSET_CM,
        ray_pitch_deg=0.0,
        block_height_cm=precision.DEFAULT_BLOCK_HEIGHT_CM,
        nx=nx,
        ny=ny,
    )
    print(
        f"near-field pick measurement: capture-pixel=({nx:.3f},{ny:.3f}) "
        f"calibrated-radius={block_radius:.2f}cm bearing={yaw_left_deg:+.1f}deg "
        f"samples={len(blobs)} nx-MAD={nx_mad:.3f} ny-MAD={ny_mad:.3f}",
        flush=True,
    )
    return precision.calculate_pick_plan(robot, gaze, block, capture_profile=True)


def _prepare_nearfield_pick_plan(
    robot: Robot,
    video,
    precision,
    coarse_payload,
    *,
    flip_x: bool,
    flip_y: bool,
):
    """Own every precision step after coarse approach, then solve IK fresh."""
    gaze_raw = coarse_payload.get("gaze") or {}
    gaze = Gaze(
        pan=int(gaze_raw.get("pan", robot.pose.get(PAN_SERVO, precision.BASE_CENTER))),
        tilt=int(gaze_raw.get("tilt", robot.pose.get(TILT_SERVO, precision.POSE_SEARCH[3]))),
    )
    lock = acquire_inherited_target(
        precision, video, robot=robot, gaze=gaze
    )
    gaze = precision.centre_gaze(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        flip_y=flip_y,
        confirmations=precision.CENTER_CONFIRMATIONS,
        allow_limit_lock=True,
    )

    # Resolve face-normal position while the target is still ~26 cm away and
    # occupies a forgiving part of the wide view.  Close-view dog-legs are
    # forbidden: once PRECAPTURE starts, pick may only creep along the selected
    # face-normal line and use the arm/servo-6 for residual alignment.
    gaze = _bounded_face_direction_restage(
        robot,
        video,
        lock,
        gaze,
        precision,
        flip_x=flip_x,
        flip_y=flip_y,
    )
    gaze = _orient_chassis_for_straight_close(
        robot,
        video,
        lock,
        gaze,
        precision,
        flip_x=flip_x,
        flip_y=flip_y,
        gaze_mode=GAZE_MODE_FIXATE,
        pursue_state=None,
    )

    # Enter the far/close overlap pose only after the chassis is on the selected
    # face-normal ray.  No side/dog-leg base correction is permitted below this
    # point because close-range image displacement is too large.
    robot.nudge_servos(
        dict(precision.PRECAPTURE_ARM_POSE), duration=PRECAPTURE_MOVE_SECONDS
    )
    note_ego_motion = getattr(lock, "note_ego_motion", None)
    if callable(note_ego_motion):
        note_ego_motion(frames=5, allowance=0.75)
    if not bool(getattr(robot, "dry_run", True)):
        time.sleep(PRECAPTURE_SETTLE_SECONDS)
    gaze = Gaze(
        pan=int(robot.pose.get(PAN_SERVO, gaze.pan)),
        tilt=int(robot.pose.get(TILT_SERVO, precision.PRECAPTURE_ARM_POSE[TILT_SERVO])),
    )
    gaze, lock, _ = precision.visual_precapture_approach(
        robot, video, lock, gaze, flip_x=flip_x
    )

    gaze, lock = precision.establish_capture_pose(
        robot, video, lock, gaze, flip_x=flip_x
    )
    target_nx = float(getattr(precision, "CAPTURE_TARGET_NX", 0.5))
    gaze, _ = precision.visual_capture_approach(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        target_nx=target_nx,
        target_ny=float(precision.ARM_FACE_CAPTURE_TARGET_NY),
    )

    # Close view is verification-only for face direction.  Never invoke
    # align_block_face_with_arm_yaw here: that routine can dog-leg the chassis,
    # and a few centimetres of lateral motion at this range can throw the block
    # completely out of frame.
    saved_face_edge_deg = None
    measure_face = getattr(precision, "measure_arm_face_alignment", None)
    if callable(measure_face):
        try:
            face, _bearing, signed_error, _ = measure_face(
                robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
            )
        except RuntimeError as exc:
            if exc.__class__.__name__ == "LockedTargetLost":
                raise
            print(
                f"near-field pick: face cue unavailable during close verification ({exc}); "
                "continue with final capture/bearing gates",
                flush=True,
            )
        else:
            if abs(float(signed_error)) > float(precision.FACE_ALIGNMENT_ABORT_DEG):
                raise RuntimeError(
                    f"near-field pick close face verification is {abs(float(signed_error)):.1f}deg "
                    "from face normal"
                )
            saved_face_edge_deg = float(face.edge_angle_deg)

    gaze, capture_blob = precision.visual_capture_approach(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        target_nx=target_nx,
        target_ny=float(precision.CAPTURE_TARGET_NY),
    )
    if saved_face_edge_deg is not None:
        final_bearing = precision.target_bearing_left_deg(
            gaze, capture_blob, flip_x=flip_x, image_center_nx=target_nx
        )
        final_face_error = abs(
            precision.arm_face_signed_error_deg(saved_face_edge_deg, final_bearing)
        )
        if final_face_error > float(precision.FACE_ALIGNMENT_ABORT_DEG):
            robot.stop()
            raise RuntimeError(
                f"near-field pick final straight reach changed arm bearing to "
                f"{final_bearing:+.1f}deg; face-normal mismatch={final_face_error:.1f}deg"
            )

    return _measure_arm_only_pick_plan(
        robot, video, precision, flip_x=flip_x, flip_y=flip_y
    )


def run_arm_only_pick(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision=None,
    target_color: str = "red",
    video=None,
) -> int:
    """Consume a coarse handoff and own all near-field alignment + grasp."""
    coarse_payload = load_coarse_handoff_payload()
    require_coarse_handoff_color(coarse_payload, target_color)
    validate_coarse_handoff_pose(robot.pose, coarse_payload)
    precision = configure_target_detector(precision or load_precision_controller(), target_color)
    dry_run = bool(getattr(robot, "dry_run", True))
    if not dry_run:
        invalidate_carry_handoff()
    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    try:
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        plan = _prepare_nearfield_pick_plan(
            robot, video, precision, coarse_payload, flip_x=flip_x, flip_y=flip_y
        )
        print("near-field grasp attempt 1/1", flush=True)
        reference = precision.execute_pick_to_hover(robot, video, plan)
        floor_clear, _remaining = precision.verify_grasp(robot, video, reference)
        if not floor_clear:
            robot.move_servo(precision.GRIPPER_ID, precision.GRIPPER_OPEN, 0.55)
            raise RuntimeError(
                f"{target_color} block is still visible on the floor after the near-field grasp"
            )
        print(
            "near-field grasp complete; pickup floor is visually clear -> PROBABLE_HELD",
            flush=True,
        )
        delivery_carry_pose = getattr(precision, "DELIVERY_CARRY_POSE", None)
        if not isinstance(delivery_carry_pose, dict):
            raise RuntimeError("precision controller has no delivery carry pose")
        robot.move_pose(delivery_carry_pose, lowering=False)
        if not dry_run:
            save_carry_handoff(
                gripper_pulse=robot.pose.get(precision.GRIPPER_ID, GRIPPER_CLOSED),
                robot_pose=robot.pose,
                target_color=target_color,
                return_base_pulse=plan.base_pulse,
                return_hover_pose=plan.hover_pose,
                return_descent_poses=precision.grasp_descent_waypoints(plan),
            )
        return 0
    except BaseException:
        if not dry_run:
            invalidate_carry_handoff()
        raise
    finally:
        invalidate_pick_plan()
        if owns_video:
            video.close()
        robot.stop()


def run_precision_pick(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision=None,
    target_color: str = "red",
    video=None,
) -> int:
    """Compatibility entrypoint for coarse-handoff near-field precision pick."""
    return run_arm_only_pick(
        robot,
        flip_x=flip_x,
        flip_y=flip_y,
        precision=precision,
        target_color=target_color,
        video=video,
    )


def run_on_robot(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)

    def handle_signal(signum: int, _frame: object) -> None:
        try:
            invalidate_pick_plan()
            robot.stop()
        finally:
            raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    robot.stop()
    probe = robot.probe()
    print(json.dumps({"probe": probe}, sort_keys=True), flush=True)

    if args.detect_only:
        look_for_block(target_color=getattr(args, "target_color", "red"))
        print("detect-only: not moving to grasp", flush=True)
        return 0

    # Agent path: approach established only coarse arm reach. Pick now owns the
    # near-field camera transition, conditional face placement, final depth, IK
    # and destructive grasp in one sensor-validated action.
    return run_precision_pick(
        robot,
        flip_x=args.flip_x,
        flip_y=args.flip_y,
        target_color=getattr(args, "target_color", "red"),
    )


VERIFY_FRAME_COUNT = 7
VERIFY_MAX_MATCH_HITS = 1
VERIFY_REFERENCE_MIN_AREA = 800
VERIFY_POSITION_TOLERANCE = 0.18
VERIFY_SIZE_RATIO_MIN = 0.45
VERIFY_SIZE_RATIO_MAX = 2.20


def _matches_pickup_site(reference, observed) -> bool:
    """Return True when a red blob still occupies the pre-grasp pickup site."""
    if reference is None or observed is None:
        return False
    distance = ((reference.nx - observed.nx) ** 2 + (reference.ny - observed.ny) ** 2) ** 0.5
    if distance > VERIFY_POSITION_TOLERANCE:
        return False
    reference_area = max(1.0, float(reference.area))
    ratio = float(observed.area) / reference_area
    return VERIFY_SIZE_RATIO_MIN <= ratio <= VERIFY_SIZE_RATIO_MAX


def _verify_pickup_disappearance(reference, *, settle: bool) -> bool:
    """Verify grasp using the transferable cue proven by the tuned controller.

    The eye-in-hand camera cannot see the gripper and servo 1 exposes no force
    sensor.  A red blob remaining at the exact pickup site after lifting back
    to the same hover view is therefore a direct failure cue.  Success requires
    that cue to disappear across a multi-frame sample.
    """
    if settle:
        time.sleep(0.20)
    matches = 0
    valid_frames = 0
    last_frame = None
    last_blob = None
    for _ in range(VERIFY_FRAME_COUNT):
        frame = capture_bgr(quiet=True)
        observed = detect_red_blob(frame, min_area=VERIFY_REFERENCE_MIN_AREA, crop_left=0)
        last_frame, last_blob = frame, observed
        valid_frames += 1
        if _matches_pickup_site(reference, observed):
            matches += 1
        if settle:
            time.sleep(0.04)
    if last_frame is not None:
        save_debug_frame(last_frame, last_blob, DEBUG_PATH)
    print(
        f"pickup-site verification matches={matches}/{valid_frames} "
        f"allowed<={VERIFY_MAX_MATCH_HITS}",
        flush=True,
    )
    return valid_frames == VERIFY_FRAME_COUNT and matches <= VERIFY_MAX_MATCH_HITS



def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="카메라에서 빨간 블록을 찾아 MasterPi 팔로 집습니다."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="SSH alias or host (default: ugrp1)")
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--detect-only", action="store_true", help="감지와 사진 저장만 하고 팔은 움직이지 않습니다")
    parser.add_argument("--preserve-gaze", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true", help="I2C 명령을 보내지 않고 동작 계획만 출력합니다")
    parser.add_argument("--flip-x", action="store_true", help="카메라 좌우가 뒤집힌 경우 사용합니다")
    parser.add_argument("--flip-y", action="store_true", help="카메라 상하가 뒤집힌 경우 사용합니다")
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    # Parse legacy flags only to reject them explicitly. Silently accepting a
    # fixed-pose tuning knob would suggest it still changes the precision grasp.
    parser.add_argument("--chassis-align", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--grasp-pulse", type=int, default=None, help=argparse.SUPPRESS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.chassis_align:
        raise ValueError("legacy --chassis-align is disabled; approach owns bounded base restaging")
    if args.grasp_pulse is not None:
        raise ValueError("legacy --grasp-pulse is disabled; pick solves the grasp from fresh camera evidence")
    if args.on_robot or is_robot():
        return run_on_robot(args)
    forwarded: list[str] = []
    if args.detect_only:
        forwarded.append("--detect-only")
    if args.preserve_gaze:
        forwarded.append("--preserve-gaze")
    if args.dry_run:
        forwarded.append("--dry-run")
    if args.flip_x:
        forwarded.append("--flip-x")
    if args.flip_y:
        forwarded.append("--flip-y")
    forwarded.extend(["--target-color", args.target_color])
    return deploy_and_run("pick.py", forwarded, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
