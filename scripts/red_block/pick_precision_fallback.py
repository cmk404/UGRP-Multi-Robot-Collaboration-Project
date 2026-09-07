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
    rebuild_pick_plan,
    validate_measurement_pose,
)


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


def run_precision_pick(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision=None,
    target_color: str = "red",
    video=None,
) -> int:
    """Execute the metric FK/IK grasp computed by precision approach.

    This restores the uploaded controller's blind-grasp sequence instead of
    deriving a fixed arm pose from one image x coordinate.  The pre-grasp plan
    is short-lived and rejected if the arm/camera moved after approach.
    """
    precision = precision or load_precision_controller()
    if target_color != "red":
        def selected(frame, **kwargs):
            return detect_target_blob(frame, target_color, **kwargs)
        precision.detect_red_blob = selected
    dry_run = bool(getattr(robot, "dry_run", True))
    # A new destructive pick attempt supersedes any old carry evidence. Only
    # this attempt may create a fresh place handoff after visual floor-clear.
    if not dry_run:
        invalidate_carry_handoff()
    payload = load_pick_plan_payload()
    require_pick_plan_color(payload, target_color)
    validate_measurement_pose(robot.pose, payload)
    plan = rebuild_pick_plan(precision, payload)

    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    try:
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        print("precision blind grasp attempt 1/1", flush=True)
        reference = precision.execute_pick_to_hover(robot, video, plan)
        floor_clear, remaining = precision.verify_grasp(robot, video, reference)
        if not floor_clear:
            robot.move_servo(precision.GRIPPER_ID, precision.GRIPPER_OPEN, 0.55)
            invalidate_pick_plan()
            raise RuntimeError(
                "red block is still visible after the grasp; refusing an automatic closer retry"
            )
        print(
            "grasp sequence complete; pickup site is visually clear, but no positive "
            "gripper sensor exists -> PROBABLE_HELD only",
            flush=True,
        )
        # Do not finish in the legacy high CARRY_POSE: its eye-in-hand camera
        # points above the floor, which makes the next destination search blind.
        # DELIVERY_CARRY_POSE keeps the grasp elevated while preserving a useful
        # floor view.  This is part of the shared REAL controller, not a SIM-only
        # pose substitution.
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
        invalidate_pick_plan()
        return 0
    except BaseException:
        invalidate_pick_plan()
        if not dry_run:
            invalidate_carry_handoff()
        raise
    finally:
        if owns_video:
            video.close()
        robot.stop()


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

    # Agent path: approach has already performed body alignment, stopped
    # multi-frame ranging and final FK/IK planning.  Do not reset to search
    # pose or recompute yaw from a single nx sample here.
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
        raise ValueError("legacy --chassis-align is disabled; run approach to align the chassis")
    if args.grasp_pulse is not None:
        raise ValueError("legacy --grasp-pulse is disabled; precision approach owns the grasp plan")
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
