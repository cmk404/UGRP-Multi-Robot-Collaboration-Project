#!/usr/bin/env python3
"""Precision gaze-locked approach imported from the known-good pickup controller.

The active action used to reduce approach to image nx/ny/area heuristics.  This
version restores the uploaded controller's actual sequence: reacquire the
tracked target, centre the eye-in-hand camera, rotate the chassis until camera
pan is centred, then approach using stop-measure-pulse-stop closed-loop range
control.  At the final stopped pose it computes and stores the FK/IK pick plan
for the immediately following pick action.
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


def run_approach(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision: Any | None = None,
    target_color: str = "red",
    video=None,
) -> int:
    precision = configure_target_detector(precision or load_precision_controller(), target_color)
    invalidate_pick_plan()
    missing_pose = [servo for servo in METRIC_POSE_SERVOS if servo not in robot.pose]
    if missing_pose:
        raise RuntimeError(
            "precision approach requires a trusted full arm pose for metric camera geometry; "
            f"missing servos {missing_pose}. Run search first to establish the calibrated pose."
        )
    search_family = all(
        int(robot.pose.get(servo)) == int(POSE_SEARCH[servo]) for servo in (4, 5)
    )
    capture_pose = getattr(precision, "CAPTURE_ARM_POSE", None)
    capture_family = bool(capture_pose) and all(
        int(robot.pose.get(servo)) == int(capture_pose[servo]) for servo in (4, 5)
    )
    if not search_family and not capture_family:
        detail = ", ".join(
            f"servo{servo}={robot.pose.get(servo)}" for servo in (4, 5)
        )
        raise RuntimeError(
            "precision approach requires search/track camera geometry or a fresh "
            f"close near-look search handoff; {detail}. Run search first."
        )
    capture_handoff_validated = False
    if capture_family and not bool(getattr(robot, "dry_run", True)):
        try:
            require_near_look_handoff(current_pose=robot.pose)
            capture_handoff_validated = True
        except RuntimeError as exc:
            raise RuntimeError(
                "precision approach requires search/track camera geometry or a fresh "
                f"close near-look search handoff; {exc}"
            ) from exc

    # A timestamp expiring does not mean the known arm geometry suddenly became
    # unusable.  Search already restores trust by re-commanding the exact saved
    # pose concurrently.  Do the same here so a stationary robot does not get
    # trapped in approach -> stale-pose failure -> approach.  For close capture
    # geometry we validate the causal near-look token *before* reasserting it;
    # reassert_pose_together intentionally invalidates that token because it is
    # an actuator command, but the exact fixed 4/5 geometry has just been proven.
    pose_fresh = getattr(robot, "pose_state_fresh", lambda: True)()
    if not pose_fresh:
        reassert = getattr(robot, "reassert_pose_together", None)
        if not callable(reassert):
            raise RuntimeError(
                "precision approach requires a trusted full arm pose for metric camera geometry; "
                "pose telemetry is stale and this controller cannot reassert it"
            )
        trusted_pose = {servo: int(robot.pose[servo]) for servo in METRIC_POSE_SERVOS}
        print(
            "precision approach: commanded pose timestamp is stale; "
            "reassert exact known camera/arm pose before metric projection",
            flush=True,
        )
        reassert(trusted_pose, duration=POSE_REASSERT_SECONDS)
        if not bool(getattr(robot, "dry_run", True)):
            time.sleep(POSE_REASSERT_SETTLE_SECONDS)
        if not getattr(robot, "pose_state_fresh", lambda: True)():
            raise RuntimeError(
                "precision approach requires a trusted full arm pose for metric camera geometry; "
                "pose telemetry remains stale after exact-pose reassertion"
            )

    if capture_family:
        # Single-use provenance.  It was validated immediately before any stale
        # pose reassertion; subsequent body/arm motion may legitimately invalidate
        # the on-disk token, so consume it now.
        if bool(getattr(robot, "dry_run", True)) or capture_handoff_validated:
            invalidate_near_look_handoff()
    elif search_family:
        invalidate_near_look_handoff()
    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    current_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
    current_tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
    gaze = Gaze(pan=current_pan, tilt=current_tilt)
    print(
        f"precision approach: inherited gaze pan={current_pan} tilt={current_tilt}",
        flush=True,
    )
    try:
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        lock = acquire_inherited_target(precision, video)

        # A target found through close near-look geometry must not be yawed or
        # grasped in-place. That old branch bypassed face alignment entirely,
        # and a physical run with ny~0.87 showed one yaw pulse can throw the cube
        # out of view. Retreat straight first, restore search geometry, then join
        # the same face-normal pipeline as every other target.
        if capture_family:
            print(
                "precision approach: inherited close near-look target; "
                "create clearance before restoring search geometry",
                flush=True,
            )
            gaze = precision.retreat_for_body_alignment(
                robot, video, lock, gaze, flip_y=flip_y,
                safe_ny=precision.CLOSE_NORMALIZE_SAFE_NY,
            )
            normalize_pose = {
                3: int(POSE_SEARCH[3]),
                4: int(POSE_SEARCH[4]),
                5: int(POSE_SEARCH[5]),
            }
            robot.nudge_servos(normalize_pose, duration=NEAR_LOOK_NORMALIZE_MOVE_SECONDS)
            if not robot.dry_run:
                time.sleep(NEAR_LOOK_NORMALIZE_MOVE_SECONDS + NEAR_LOOK_NORMALIZE_SETTLE_SECONDS)
            gaze = Gaze(
                pan=int(robot.pose.get(PAN_SERVO, gaze.pan)),
                tilt=int(POSE_SEARCH[3]),
            )
            video.settle(precision.CAMERA_DELAY_SECONDS)
            lock = acquire_inherited_target(precision, video)
            print(
                "precision approach: close near-look normalized to search geometry; "
                "continue through common face-alignment pipeline",
                flush=True,
            )

        # Distance/visibility clearance comes before any chassis yaw. This is the
        # key ordering invariant exposed by the ny~0.87 physical failures.
        gaze = precision.retreat_for_body_alignment(
            robot, video, lock, gaze, flip_y=flip_y,
        )

        # 1) Camera/arm centres the target first.
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

        # 2) Far targets still benefit from chassis alignment. A near target
        # does not: the 2026-08-31 REAL run entered with only ~6deg residual
        # bearing, then repeated chassis-yaw pulses destroyed the visual lock.
        # Select the arm-only corridor from a stopped metric observation.
        arm_only_near = False
        estimate_block = getattr(precision, "estimate_block", None)
        if callable(estimate_block) and getattr(lock, "last", None) is not None:
            initial_block = estimate_block(robot.pose, lock.last)
            if initial_block is not None:
                arm_only_near = (
                    initial_block.radius_cm <= NEAR_ARM_ONLY_MAX_RADIUS_CM
                    and abs(initial_block.yaw_left_deg) <= NEAR_ARM_ONLY_MAX_BEARING_DEG
                )
                print(
                    f"precision approach: stopped initial geometry "
                    f"radius={initial_block.radius_cm:.2f}cm "
                    f"bearing={initial_block.yaw_left_deg:+.1f}deg -> "
                    f"{'arm-only near corridor' if arm_only_near else 'body-align corridor'}",
                    flush=True,
                )
        if arm_only_near:
            print(
                "precision approach: skip chassis body-align; servo 6 will own residual yaw",
                flush=True,
            )
        else:
            gaze = precision.align_body_to_gaze(
                robot,
                video,
                lock,
                gaze,
                flip_x=flip_x,
                flip_y=flip_y,
            )

        # 3) Reach the normal capture-staging corridor while the target is still
        # centred on the chassis.  Face-normal placement is deliberately deferred
        # until AFTER the final visual capture: once we translate beside the cube,
        # the chassis must not re-face or drive forward again.
        staging = precision.approach_with_locked_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            flip_y=flip_y,
            target_radius_cm=precision.CAPTURE_STAGING_RADIUS_CM,
            allow_blind_arrival=False,
            allow_body_realign=not arm_only_near,
        )
        gaze = staging.gaze

        # 4) Enter the explicit far-view pre-capture pose before the overlap
        # creep. visual_precapture_approach() validates this pose, but the old
        # caller relied on centre_gaze() happening to hit servo-3's lower limit.
        # That made the transition scene-dependent even though PRECAPTURE_ARM_POSE
        # is already the controller's declared geometry. Preserve the aligned pan
        # while moving only the fixed 3/4/5 camera-arm joints.
        robot.nudge_servos(dict(precision.PRECAPTURE_ARM_POSE), duration=PRECAPTURE_MOVE_SECONDS)
        if not robot.dry_run:
            time.sleep(PRECAPTURE_SETTLE_SECONDS)
        gaze = Gaze(
            pan=int(robot.pose.get(PAN_SERVO, gaze.pan)),
            tilt=int(robot.pose.get(TILT_SERVO, precision.PRECAPTURE_ARM_POSE[TILT_SERVO])),
        )

        # 5) Bridge the far floor-search view into the close capture camera's
        # field of view before changing arm geometry.  Without this overlap
        # stage, a 17-20 cm target disappears when the camera starts looking at
        # the 11-12 cm grasp region.
        gaze, lock, _ = precision.visual_precapture_approach(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
        )

        # 6) Enter the close camera geometry, but stop EARLY while the full
        # oriented cube contour is still valid for floor-face projection.  The
        # old final capture window was excellent for straight-on hand-eye grasp,
        # but too close for reliable face-normal estimation.
        gaze, lock = precision.establish_capture_pose(
            robot, video, lock, gaze, flip_x=flip_x
        )
        gaze, capture_blob = precision.visual_capture_approach(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            target_ny=precision.ARM_FACE_CAPTURE_TARGET_NY,
        )

        # 7) Final face placement: translate sideways only when required, then
        # servo 6 points the arm/camera back at the cube. Chassis re-facing is
        # forbidden after this step; only a later bounded STRAIGHT reach refinement
        # is allowed before IK.
        print(
            "precision approach: final side placement + arm-base yaw; chassis heading fixed",
            flush=True,
        )
        gaze, capture_blob, final_bearing = precision.align_block_face_with_arm_yaw(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            flip_y=flip_y,
            target_nx=precision.CAPTURE_TARGET_NX,
        )

        # Preserve the reliable early face observation before entering the final
        # hand-eye depth. Archived REAL frames show that floor projection can be
        # unavailable around ny~=0.41 even though range is already ideal. Straight
        # forward refinement does not intentionally change chassis yaw or cube
        # orientation, so the saved floor-edge direction remains the conservative
        # reference while servo 6 follows the target bearing.
        aligned_face, _, aligned_signed_error, _ = precision.measure_arm_face_alignment(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            target_nx=precision.CAPTURE_TARGET_NX,
        )
        if abs(aligned_signed_error) > precision.FACE_ALIGNMENT_ABORT_DEG:
            raise RuntimeError(
                f"arm-face verification drifted to {abs(aligned_signed_error):.1f}deg "
                "before final reach refinement"
            )
        saved_face_edge_deg = float(aligned_face.edge_angle_deg)

        # The ny~=0.30 face staging point is intentionally too far for the final
        # 17.5-cm fingertip calibration. Continue only straight forward to the
        # existing measured hand-eye target. This primitive keeps the chassis
        # heading fixed and uses servo 6 for horizontal image correction.
        gaze, capture_blob = precision.visual_capture_approach(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            target_nx=precision.CAPTURE_TARGET_NX,
            target_ny=precision.CAPTURE_TARGET_NY,
        )
        final_bearing = precision.target_bearing_left_deg(
            gaze,
            capture_blob,
            flip_x=flip_x,
            image_center_nx=precision.CAPTURE_TARGET_NX,
        )
        final_face_error = abs(
            precision.arm_face_signed_error_deg(saved_face_edge_deg, final_bearing)
        )
        if final_face_error > precision.FACE_ALIGNMENT_ABORT_DEG:
            robot.stop()
            raise RuntimeError(
                f"arm ray changed to {final_bearing:+.1f}deg during final straight "
                f"reach refinement; saved face-normal mismatch is {final_face_error:.1f}deg"
            )
        print(
            f"precision approach: final hand-eye depth reached ny={capture_blob.ny:.3f} "
            f"arm-bearing={final_bearing:+.1f}deg saved-face-error={final_face_error:.1f}deg; "
            "no chassis re-face/lateral motion allowed",
            flush=True,
        )

        # 8) Re-measure the stopped cube from the SAME arm yaw that will grasp it.
        # The floor-ray estimate gives visual radius; retain the existing
        # longitudinal camera->gripper reach correction, now applied along the
        # arm ray instead of the chassis centreline.
        gaze, arm_samples, final_blob = precision.stationary_range_samples(
            robot,
            video,
            lock,
            gaze,
            count=precision.ARM_FACE_RANGE_SAMPLE_COUNT,
            flip_x=flip_x,
            flip_y=flip_y,
            image_center_nx=precision.CAPTURE_TARGET_NX,
        )
        if len(arm_samples) < precision.ARM_FACE_MIN_RANGE_SAMPLES or final_blob is None:
            raise RuntimeError(
                f"only {len(arm_samples)} valid arm-face range samples; "
                "refusing blind arm-only grasp"
            )
        visual_block = precision.median_block_estimate(arm_samples)
        corrected_block = precision.apply_radial_reach_correction(
            visual_block, precision.CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM
        )
        print(
            f"precision approach: arm-only geometry visual-radius={visual_block.radius_cm:.2f}cm "
            f"corrected-radius={corrected_block.radius_cm:.2f}cm "
            f"bearing={corrected_block.yaw_left_deg:+.1f}deg "
            f"face-bearing={final_bearing:+.1f}deg; chassis remains stopped",
            flush=True,
        )

        # Preserve the proven capture descent profile (-90deg close approach),
        # but let the stopped measured radius/yaw choose servo 6 and arm reach.
        plan = precision.calculate_pick_plan(
            robot, gaze, corrected_block, capture_profile=True
        )

        save_kwargs = {"robot_pose": robot.pose, "target_visible": True}
        if target_color != "red":
            save_kwargs["target_color"] = target_color
        save_pick_plan(plan, **save_kwargs)
        print(
            "precision pregrasp ready: calibrated visual capture saved for blind pick",
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
