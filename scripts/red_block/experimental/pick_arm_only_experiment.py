#!/usr/bin/env python3
"""Pick up a red block in front of MasterPi.

Mac에서 실행하면 이 폴더를 ugrp1에 복사한 뒤 Pi에서 돌립니다.

    python3 scripts/pick_red_block.py
    python3 scripts/red_block/pick.py --detect-only
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
import math
import statistics
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
    load_pick_plan_payload,
    require_pick_plan_color,
    validate_measurement_pose,
)
from approach import acquire_inherited_target, configure_target_detector
from track import Gaze


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


def _measure_arm_only_pick_plan(robot: Robot, video, precision, *, flip_x: bool, flip_y: bool):
    """Verify the coarse visual reach handoff and solve arm-only IK fresh."""
    # The 15 cm calibration is valid only in the explicit pre-capture pose.
    for servo, expected in precision.PRECAPTURE_ARM_POSE.items():
        actual = int(robot.pose.get(servo, -9999))
        if abs(actual - int(expected)) > 30:
            raise RuntimeError(
                f"arm-only pick lost calibrated reach camera pose at servo {servo}: "
                f"expected {expected}, got {actual}; run approach"
            )

    gaze = Gaze(pan=int(robot.pose.get(6, precision.BASE_CENTER)), tilt=int(robot.pose.get(3, 500)))
    lock = acquire_inherited_target(precision, video)
    gaze = precision.centre_gaze(
        robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
        confirmations=precision.CENTER_CONFIRMATIONS, allow_limit_lock=True,
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
            f"only {len(blobs)} valid arm-only pick visual samples; run approach to restage"
        )

    nys = [float(b.ny) for b in blobs]
    nxs = [float(b.nx) for b in blobs]
    ny = float(statistics.median(nys))
    nx = float(statistics.median(nxs))
    ny_mad = float(precision.median_absolute_deviation(nys))
    nx_mad = float(precision.median_absolute_deviation(nxs))
    if ny_mad > 0.012 or nx_mad > 0.012:
        raise RuntimeError(
            f"arm-only pick visual coordinate is unstable (nx MAD={nx_mad:.3f}, ny MAD={ny_mad:.3f}); "
            "run approach to restage"
        )
    if ny < precision.PRECAPTURE_TARGET_NY - 0.04 or ny >= precision.PRECAPTURE_TOO_CLOSE_NY:
        raise RuntimeError(
            f"arm-only pick target is outside calibrated visual reach window (ny={ny:.3f}); run approach"
        )

    # PRECAPTURE_TARGET_NY is explicitly calibrated to a 15 cm block-centre
    # radius. Use image x + servo-6 only for yaw; do not feed the biased near
    # floor-ray depth back into the arm plan.
    block_radius = float(precision.CAPTURE_BLOCK_RADIUS_CM)
    pixel_left_deg = (0.5 - nx) * precision.CAMERA_HFOV_DEG
    servo_left_deg = (int(robot.pose[6]) - precision.BASE_CENTER) / precision.PULSE_PER_DEGREE
    yaw_left_deg = servo_left_deg + pixel_left_deg
    if abs(yaw_left_deg) > PICK_ARM_MAX_BEARING_DEG:
        raise RuntimeError(
            f"arm-only pick target bearing {yaw_left_deg:+.1f}deg exceeds "
            f"servo-6 corridor +/-{PICK_ARM_MAX_BEARING_DEG:.1f}deg; run approach to re-aim chassis"
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
        f"arm-only pick measurement: visual-ny={ny:.3f} calibrated-radius={block_radius:.2f}cm "
        f"bearing={yaw_left_deg:+.1f}deg samples={len(blobs)} nx-MAD={nx_mad:.3f} ny-MAD={ny_mad:.3f}",
        flush=True,
    )
    return precision.calculate_pick_plan(robot, gaze, block, capture_profile=True)


def run_arm_only_pick(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision=None,
    target_color: str = "red",
    video=None,
) -> int:
    """Measure a nearby block at execution time and grasp it with arm IK.

    ``approach`` only has to put the cube inside the verified arm reach envelope.
    Pick no longer consumes a fragile approach-created FK/IK file.  It freezes
    the current camera pose, confirms a stable multi-frame floor coordinate,
    aims servo 6 and solves the arm joints itself.  No chassis command is issued
    by this function.
    """
    precision = configure_target_detector(precision or load_precision_controller(), target_color)
    dry_run = bool(getattr(robot, "dry_run", True))
    invalidate_pick_plan()
    if not dry_run:
        invalidate_carry_handoff()
    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    try:
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        plan = _measure_arm_only_pick_plan(
            robot, video, precision, flip_x=flip_x, flip_y=flip_y
        )
        print("arm-only grasp attempt 1/1", flush=True)
        reference = precision.execute_pick_to_hover(robot, video, plan)
        floor_clear, _remaining = precision.verify_grasp(robot, video, reference)
        if not floor_clear:
            robot.move_servo(precision.GRIPPER_ID, precision.GRIPPER_OPEN, 0.55)
            raise RuntimeError(
                f"{target_color} block is still visible on the floor after the arm-only grasp"
            )
        print(
            "arm-only grasp complete; pickup floor is visually clear -> PROBABLE_HELD",
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


def _valid_precision_plan_available(robot: Robot, target_color: str) -> bool:
    try:
        payload = load_pick_plan_payload()
        require_pick_plan_color(payload, target_color)
        validate_measurement_pose(robot.pose, payload)
        return True
    except RuntimeError:
        return False


def run_precision_pick(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision=None,
    target_color: str = "red",
    video=None,
) -> int:
    """Adaptive pick: consume proven precision plan when present, else arm-only."""
    if _valid_precision_plan_available(robot, target_color):
        print("adaptive pick: using proven approach-created precision plan", flush=True)
        import pick_precision_fallback as proven
        return proven.run_precision_pick(
            robot,
            flip_x=flip_x,
            flip_y=flip_y,
            precision=precision,
            target_color=target_color,
            video=video,
        )

    from pickup_strategy import precision_fallback_reason
    pending_fallback = precision_fallback_reason(target_color)
    if pending_fallback is not None:
        raise RuntimeError(
            "precision fallback is pending but no validated precision pick plan exists; "
            f"run approach to complete the preserved precision path ({pending_fallback})"
        )

    try:
        return run_arm_only_pick(
            robot,
            flip_x=flip_x,
            flip_y=flip_y,
            precision=precision,
            target_color=target_color,
            video=video,
        )
    except RuntimeError as exc:
        text = str(exc).lower()
        if "still visible on the floor after the arm-only grasp" in text:
            # Return to the search/pre-capture arm family so the next approach
            # can safely enter the exact previously validated precision pipeline.
            controller = precision or load_precision_controller()
            try:
                robot.nudge_servos(dict(controller.PRECAPTURE_ARM_POSE), duration=0.65)
                if not bool(getattr(robot, "dry_run", True)):
                    time.sleep(0.20)
            finally:
                from pickup_strategy import request_precision_fallback
                request_precision_fallback(target_color, str(exc))
            raise RuntimeError(
                f"{exc}; proven precision fallback requested for the next approach"
            ) from exc
        raise

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

    # Agent path: approach only established arm reach. Pick performs its own
    # stopped multi-frame measurement and arm-only IK immediately before grasp.
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
        raise ValueError("legacy --chassis-align is disabled; approach owns coarse chassis positioning")
    if args.grasp_pulse is not None:
        raise ValueError("legacy --grasp-pulse is disabled; pick computes arm IK from fresh vision")
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
