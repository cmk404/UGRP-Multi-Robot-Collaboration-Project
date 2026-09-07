#!/usr/bin/env python3
"""Coarse visual approach for the red-block stack.

This public action only brings the block inside the verified arm reach corridor.
Final stopped measurement, servo-6 aiming, IK and grasp now belong to ``pick``.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any, Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from deploy import DEFAULT_HOST, deploy_and_run
from camera import detect_target_blob
from precision_handoff import invalidate_pick_plan, save_pick_plan
from near_look_handoff import (
    invalidate_near_look_handoff,
    require_near_look_handoff,
)
from poses import BASE_CENTER, POSE_SEARCH
from robot import Robot, is_robot
from track import Gaze, PAN_SERVO, TILT_SERVO


ACQUIRE_ATTEMPTS = 10
# Near a large floor target, chassis yaw can throw the cube out of the eye-in-hand
# field before the final arm-face controller gets a chance to work.  Once a
# stopped metric estimate says the cube is already in this corridor and only a
# modest residual yaw remains, preserve chassis heading and let servo 6 own yaw.
NEAR_ARM_ONLY_MAX_RADIUS_CM = 30.0
NEAR_ARM_ONLY_MAX_BEARING_DEG = 12.0
# Stop the chassis with margin inside the verified arm-only reach envelope.
APPROACH_PICK_RADIUS_CM = 22.0
METRIC_POSE_SERVOS = (3, 4, 5, 6)
MAX_APPROACH_ATTEMPTS = 2
PRECAPTURE_MOVE_SECONDS = 0.65
PRECAPTURE_SETTLE_SECONDS = 0.75
NEAR_LOOK_NORMALIZE_MOVE_SECONDS = 0.75
NEAR_LOOK_NORMALIZE_SETTLE_SECONDS = 0.35
POSE_REASSERT_SECONDS = 1.25
POSE_REASSERT_SETTLE_SECONDS = 0.20
RECOVERABLE_APPROACH_ERRORS = (
    "range changed implausibly",
    "locked red block not found",
    "insufficient valid range samples",
)


def recoverable_approach_error(error: BaseException) -> bool:
    text = str(error).lower()
    return any(marker in text for marker in RECOVERABLE_APPROACH_ERRORS)



def load_precision_controller() -> Any:
    import physical_state_machine_reference as precision
    return precision


def configure_target_detector(precision: Any, target_color: str) -> Any:
    """Route the known guarded controller through the selected colour detector.

    Its calibrated red function remains untouched for red.  For blue/yellow
    only the detector binding changes; all bounded motion, metric, and
    verification gates stay in the proven precision controller.
    """
    if target_color == "red":
        return precision
    original = precision.detect_red_blob
    def selected(frame, **kwargs):
        return detect_target_blob(frame, target_color, **kwargs)
    precision.detect_red_blob = selected
    precision._ugrp_original_detect_red_blob = original
    return precision


def acquire_inherited_target(precision: Any, video: Any) -> Any:
    """Confirm the target in the gaze handed over by track without a new scan."""
    for attempt in range(1, ACQUIRE_ATTEMPTS + 1):
        frame = video.read()
        candidate = precision.detect_red_blob(
            frame,
            min_area=precision.TRACK_MIN_AREA,
            crop_left=0,
        )
        if candidate is not None:
            confirmed = precision.confirmed_candidate(video, candidate)
            if confirmed is not None:
                print(
                    f"precision approach reacquired target attempt={attempt} "
                    f"nx={confirmed.nx:.3f} ny={confirmed.ny:.3f} area={confirmed.area}",
                    flush=True,
                )
                return precision.TargetLock(confirmed)
        time.sleep(0.04)
    raise RuntimeError(
        "track handoff lost the red block before precision approach; run search/track again"
    )


def run_coarse_approach(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision: Any | None = None,
    target_color: str = "red",
    video=None,
) -> int:
    """Bring the selected block into the arm-only grasp envelope.

    Public ``approach`` deliberately stops at coarse reach.  It does not solve
    cube-face routing, enter the close capture pose, or build an FK/IK grasp
    plan.  ``pick`` owns the final stopped multi-frame measurement and arm IK.
    This keeps repeated face/capture refinement out of the LLM action loop.
    """
    precision = configure_target_detector(precision or load_precision_controller(), target_color)
    # Legacy plan files must never authorize the new self-measuring pick path.
    invalidate_pick_plan()
    missing_pose = [servo for servo in METRIC_POSE_SERVOS if servo not in robot.pose]
    if missing_pose:
        raise RuntimeError(
            "coarse approach requires a trusted full arm pose for metric camera geometry; "
            f"missing servos {missing_pose}. Run search first."
        )
    search_family = all(
        int(robot.pose.get(servo)) == int(POSE_SEARCH[servo]) for servo in (4, 5)
    )
    if not search_family:
        detail = ", ".join(f"servo{servo}={robot.pose.get(servo)}" for servo in (4, 5))
        raise RuntimeError(
            "coarse approach requires search/track camera geometry; "
            f"{detail}. Run search first."
        )

    pose_fresh = getattr(robot, "pose_state_fresh", lambda: True)()
    if not pose_fresh:
        reassert = getattr(robot, "reassert_pose_together", None)
        if not callable(reassert):
            raise RuntimeError(
                "coarse approach requires a trusted camera pose; pose telemetry is stale"
            )
        trusted_pose = {servo: int(robot.pose[servo]) for servo in METRIC_POSE_SERVOS}
        reassert(trusted_pose, duration=POSE_REASSERT_SECONDS)
        if not bool(getattr(robot, "dry_run", True)):
            time.sleep(POSE_REASSERT_SETTLE_SECONDS)
        if not getattr(robot, "pose_state_fresh", lambda: True)():
            raise RuntimeError(
                "coarse approach requires a trusted camera pose; pose telemetry remains stale"
            )

    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    current_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
    current_tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
    gaze = Gaze(pan=current_pan, tilt=current_tilt)
    print(f"coarse approach: inherited gaze pan={current_pan} tilt={current_tilt}", flush=True)
    try:
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        lock = acquire_inherited_target(precision, video)

        # Centre first, then put the chassis roughly on the same ray.  This is
        # the only yaw ownership in approach; cube-face orientation is no longer
        # a chassis-routing requirement.
        gaze = precision.centre_gaze(
            robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
            confirmations=precision.CENTER_CONFIRMATIONS, allow_limit_lock=True,
        )
        estimate = precision.estimate_block(robot.pose, lock.last)
        if estimate is None:
            raise RuntimeError("coarse approach could not obtain a valid stopped range sample")
        if abs(float(estimate.yaw_left_deg)) > NEAR_ARM_ONLY_MAX_BEARING_DEG:
            gaze = precision.align_body_to_gaze(
                robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
            )

        # One stationary re-measure/retry is owned internally by approach. A
        # single implausible short-pulse range delta is a sensor-settle event,
        # not a reason to bounce the whole task back through the LLM.
        result = None
        for local_attempt in range(2):
            try:
                result = precision.approach_with_locked_gaze(
                    robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
                    target_radius_cm=precision.CAPTURE_STAGING_RADIUS_CM,
                    allow_blind_arrival=False,
                    allow_body_realign=True,
                )
                break
            except RuntimeError as exc:
                if local_attempt or "range changed implausibly" not in str(exc).lower():
                    raise
                robot.stop()
                video.settle(precision.CAMERA_DELAY_SECONDS + 0.08)
                lock = acquire_inherited_target(precision, video)
                gaze = precision.centre_gaze(
                    robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
                    confirmations=precision.CENTER_CONFIRMATIONS, allow_limit_lock=True,
                )
                print("coarse approach: refreshed stopped range after one implausible delta", flush=True)
        if result is None:
            raise RuntimeError("coarse approach could not establish arm-reach staging")
        gaze = result.gaze

        # Metric floor-ray is intentionally abandoned before near-field grasp
        # depth. Move only the camera-arm joints into the proven far-view
        # pre-capture pose, then use image y for the last few straight chassis
        # creeps. There is no cube-face routing and no close-capture/IK here.
        robot.nudge_servos(dict(precision.PRECAPTURE_ARM_POSE), duration=PRECAPTURE_MOVE_SECONDS)
        if not robot.dry_run:
            time.sleep(PRECAPTURE_SETTLE_SECONDS)
        gaze = Gaze(
            pan=int(robot.pose.get(PAN_SERVO, gaze.pan)),
            tilt=int(robot.pose.get(TILT_SERVO, precision.PRECAPTURE_ARM_POSE[TILT_SERVO])),
        )
        gaze, lock, precapture_blob = precision.visual_precapture_approach(
            robot, video, lock, gaze, flip_x=flip_x
        )
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        _frame, fresh_blob = precision.read_locked(video, lock)
        if fresh_blob is None:
            fresh_blob = precapture_blob
        ny = float(fresh_blob.ny)
        if ny < precision.PRECAPTURE_TARGET_NY - 0.035 or ny >= precision.PRECAPTURE_TOO_CLOSE_NY:
            raise RuntimeError(
                f"coarse approach did not leave target in arm-reach visual window "
                f"(ny={ny:.3f})"
            )
        print(
            f"coarse approach ready: visual-reach ny={ny:.3f} pan={gaze.pan}; "
            "final arm measurement belongs to pick",
            flush=True,
        )
        return 0
    except BaseException:
        invalidate_pick_plan()
        raise
    finally:
        if owns_video:
            video.close()
        robot.stop()


def run_approach(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision: Any | None = None,
    target_color: str = "red",
    video=None,
) -> int:
    """Adaptive approach: arm-only staging first, proven precision path on recovery."""
    from pickup_strategy import (
        clear_precision_fallback,
        precision_fallback_reason,
        should_use_precision_fallback,
    )

    if should_use_precision_fallback(target_color):
        reason = precision_fallback_reason(target_color)
        print(
            f"adaptive approach: precision fallback selected after arm-only failure: {reason}",
            flush=True,
        )
        import approach_precision_fallback as proven
        result = proven.run_approach(
            robot,
            flip_x=flip_x,
            flip_y=flip_y,
            precision=precision,
            target_color=target_color,
            video=video,
        )
        if result == 0:
            clear_precision_fallback(target_color)
        return result

    print("adaptive approach: coarse arm-only staging path", flush=True)
    return run_coarse_approach(
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
    print(json.dumps({"probe": robot.probe()}, sort_keys=True), flush=True)
    for attempt in range(1, MAX_APPROACH_ATTEMPTS + 1):
        try:
            return run_approach(
                robot, flip_x=args.flip_x, flip_y=args.flip_y,
                target_color=getattr(args, "target_color", "red"),
            )
        except RuntimeError as exc:
            robot.stop()
            if attempt >= MAX_APPROACH_ATTEMPTS or not recoverable_approach_error(exc):
                raise
            print(
                f"precision approach recovery {attempt}/{MAX_APPROACH_ATTEMPTS - 1}: "
                f"{exc}; stopped and reacquiring the current target once",
                flush=True,
            )
            invalidate_pick_plan()
            time.sleep(0.35)
    raise RuntimeError("precision approach retry loop ended unexpectedly")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="시선을 고정한 채 차체를 정렬하고 정지-재측정 방식으로 빨간 블록에 접근합니다."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra: list[str] = []
    if args.dry_run:
        extra.append("--dry-run")
    if args.flip_x:
        extra.append("--flip-x")
    if args.flip_y:
        extra.append("--flip-y")
    extra.extend(["--target-color", args.target_color])
    return deploy_and_run("approach.py", extra, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
