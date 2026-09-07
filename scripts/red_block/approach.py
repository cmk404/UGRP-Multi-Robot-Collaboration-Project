#!/usr/bin/env python3
"""Coarse, REAL-first visual approach for the block pickup stack.

``approach`` owns only coarse chassis staging.  It leaves the selected block in
a stopped arm-reach corridor and persists a short-lived pose/gaze handoff.
Near-field face placement, capture-depth creep, servo-6 aiming, IK and grasp all
belong to ``pick``.  Pure mecanum strafe is never required by this controller.
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
from gaze_hold import GAZE_MODE_FIXATE, GAZE_MODES, PursueState, normalize_gaze_mode
import os as _os


def _default_gaze_mode() -> str:
    """Env override lets the SIM worker A/B modes without tool-schema edits."""
    try:
        return normalize_gaze_mode(_os.environ.get("UGRP_GAZE_MODE", GAZE_MODE_FIXATE))
    except ValueError:
        return GAZE_MODE_FIXATE
from precision_handoff import invalidate_pick_plan, save_coarse_handoff
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
# REAL traces show 0.08 s far-motion fragments can fail to overcome drivetrain
# deadband while repeated body-yaw corrections can overshoot.  Coarse approach
# therefore uses bounded stop-measure-pulse-stop motion with continuous far
# streaming disabled, and stops metric depth work before the final grasp zone.
COARSE_STAGING_RADIUS_CM = 26.0
COARSE_MAX_MOTION_STEPS = 24
COARSE_BODY_ALIGN_MIN_RADIUS_CM = 32.0
COARSE_BODY_ALIGN_TRIGGER_DEG = 18.0
COARSE_BODY_ALIGN_ACCEPT_AFTER_OVERSHOOT_DEG = 30.0
COARSE_FACE_MAX_REPOSITIONS = 8
# The final visual close runs straight from ~26 cm staging through pre-capture
# overlap and then to the fixed hand-eye capture depth.  A 9-10 deg residual at
# 26 cm can therefore grow past 20 deg at the stopped grasp view simply from
# perspective.  Stage to ~7 deg here so the same straight-only close remains
# inside the ~12 deg face-normal envelope.
COARSE_FACE_ACCEPT_DEG = 7.0
COARSE_FACE_MIN_IMPROVEMENT_DEG = 1.0
COARSE_FACE_MIN_BEARING_PROGRESS_DEG = 0.8
COARSE_FACE_MAX_STALLED_REPOSITIONS = 2
COARSE_FACE_MAX_ARM_BEARING_DEG = 34.0
METRIC_POSE_SERVOS = (3, 4, 5, 6)
MAX_APPROACH_ATTEMPTS = 2
PRECAPTURE_MOVE_SECONDS = 0.65
PRECAPTURE_SETTLE_SECONDS = 0.75
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
    contextual = getattr(precision, "set_target_detector", None)
    if callable(contextual):
        if target_color == "red":
            contextual(None)
        else:
            contextual(
                lambda frame, **kwargs: detect_target_blob(
                    frame, target_color, **kwargs
                )
            )
        return precision
    if target_color == "red":
        return precision
    # Compatibility for older deployed controller modules. New controller
    # source uses a ContextVar-backed detector and never reaches this branch.
    original = precision.detect_red_blob
    def selected(frame, **kwargs):
        return detect_target_blob(frame, target_color, **kwargs)
    precision.detect_red_blob = selected
    precision._ugrp_original_detect_red_blob = original
    return precision


def acquire_inherited_target(
    precision: Any,
    video: Any,
    *,
    robot: Robot | None = None,
    gaze: Gaze | None = None,
) -> Any:
    """Confirm the inherited target, then try a tiny local pan recovery.

    This deliberately does not invoke the public ``search`` skill.  A short
    handoff loss after track/approach should cost a few camera frames, not a
    full scan + LLM replan.
    """
    def sample(attempt_label: str) -> Any | None:
        for attempt in range(1, ACQUIRE_ATTEMPTS + 1):
            frame = video.read()
            candidate = precision.detect_red_blob(
                frame, min_area=precision.TRACK_MIN_AREA, crop_left=0
            )
            if candidate is not None:
                confirmed = precision.confirmed_candidate(video, candidate)
                if confirmed is not None:
                    print(
                        f"local target reacquired {attempt_label} attempt={attempt} "
                        f"nx={confirmed.nx:.3f} ny={confirmed.ny:.3f} area={confirmed.area}",
                        flush=True,
                    )
                    return precision.TargetLock(confirmed)
            time.sleep(0.04)
        return None

    found = sample("inherited")
    if found is not None:
        return found
    if robot is None or gaze is None:
        raise RuntimeError(
            "track handoff lost the target and local recovery has no robot gaze"
        )

    origin_pan = int(gaze.pan)
    pan_min = int(getattr(precision, "PAN_MIN", 1050))
    pan_max = int(getattr(precision, "PAN_MAX", 1950))
    settle = float(getattr(precision, "CAMERA_DELAY_SECONDS", 0.12))
    tried: set[int] = set()
    for offset in (-80, 80, -160, 160):
        pan = max(pan_min, min(pan_max, origin_pan + offset))
        if pan == origin_pan or pan in tried:
            continue
        tried.add(pan)
        robot.nudge_servos(
            {PAN_SERVO: pan, TILT_SERVO: int(gaze.tilt)}, duration=0.10
        )
        gaze.pan = pan
        video.settle(settle)
        found = sample(f"micro-pan={pan}")
        if found is not None:
            return found

    if int(gaze.pan) != origin_pan:
        robot.nudge_servos(
            {PAN_SERVO: origin_pan, TILT_SERVO: int(gaze.tilt)}, duration=0.10
        )
        gaze.pan = origin_pan
        video.settle(settle)
    raise RuntimeError(
        "track handoff lost the target after bounded local pan recovery"
    )

def _bounded_face_direction_restage(
    robot: Robot,
    video: Any,
    lock: Any,
    gaze: Gaze,
    precision: Any,
    *,
    flip_x: bool,
    flip_y: bool,
) -> Gaze:
    """Use bounded REAL-safe dog-legs while stopped vision confirms progress.

    The controller never asks the mecanum base for pure lateral motion.  A
    dog-leg is rotate -> backward -> restore-heading, followed by straight-only
    depth recovery and a fresh stopped face measurement.  REAL lateral response
    is weak and noisy, so one low-progress sample is tolerated, but the motion
    budget remains hard-bounded and two consecutive no-progress repositions
    fail closed instead of oscillating.
    """
    measure = getattr(precision, "measure_arm_face_alignment", None)
    dogleg = getattr(precision, "face_reposition_dogleg", None)
    fine_x = getattr(precision, "fine_align_horizontal", None)
    if not all(callable(fn) for fn in (measure, dogleg, fine_x)):
        return gaze
    target_nx = float(getattr(precision, "CAPTURE_TARGET_NX", 0.5))
    try:
        current, bearing, signed_error, _blob = measure(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
        )
    except RuntimeError as exc:
        if exc.__class__.__name__ == "LockedTargetLost":
            raise
        print(
            f"pick staging: no stable block-face cue ({exc}); "
            "skip base side restage and let pick verify the stopped geometry",
            flush=True,
        )
        return gaze

    print(
        f"pick staging face sample error={current.error_deg:.1f}deg "
        f"arm-bearing={bearing:+.1f}deg",
        flush=True,
    )
    if current.error_deg <= COARSE_FACE_ACCEPT_DEG:
        return gaze
    if abs(bearing) >= COARSE_FACE_MAX_ARM_BEARING_DEG:
        raise RuntimeError(
            f"pick staging face restage starts at arm bearing {bearing:+.1f}deg; "
            "run search/track to re-aim before another base reposition"
        )

    initial_sign = 1.0 if signed_error >= 0.0 else -1.0
    side = "right" if signed_error > 0.0 else "left"
    # Near-field pick staging owns its own bounded side-reposition budget.  The shared
    # precision controller's FACE_REPOSITION_MAX_PULSES is used by a different
    # near-field face-alignment routine and used to cap this path at six even
    # while every stopped measurement was still making physical progress.  Keep
    # the same cross-normal, arm-bearing and stalled-progress fail-closed gates,
    # but allow two more measured dog-legs before declaring exhaustion.
    max_repositions = COARSE_FACE_MAX_REPOSITIONS
    max_stalled = min(
        COARSE_FACE_MAX_STALLED_REPOSITIONS,
        max(1, int(getattr(
            precision,
            "FACE_REPOSITION_MAX_STALLED_PULSES",
            COARSE_FACE_MAX_STALLED_REPOSITIONS,
        ))),
    )
    previous_error = float(current.error_deg)
    previous_bearing = float(bearing)
    best_error = previous_error
    stalled = 0
    for attempt in range(1, max_repositions + 1):
        print(
            f"pick staging face dog-leg={attempt}/{max_repositions} "
            f"side={side} error={previous_error:.1f}deg",
            flush=True,
        )
        dogleg(robot, lock, side=side)
        gaze = fine_x(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
        )
        restored = precision.approach_with_locked_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            flip_y=flip_y,
            target_radius_cm=COARSE_STAGING_RADIUS_CM,
            allow_blind_arrival=False,
            allow_body_realign=False,
            allow_continuous_far=False,
            max_motion_steps=6,
        )
        gaze = restored.gaze
        current, bearing, signed_error, _blob = measure(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
        )
        improvement = previous_error - float(current.error_deg)
        bearing_progress = (float(bearing) - previous_bearing) * initial_sign
        best_error = min(best_error, float(current.error_deg))
        print(
            f"pick staging face verify error={current.error_deg:.1f}deg "
            f"improvement={improvement:+.1f}deg arm-bearing={bearing:+.1f}deg "
            f"bearing-progress={bearing_progress:+.1f}deg",
            flush=True,
        )
        if current.error_deg <= COARSE_FACE_ACCEPT_DEG:
            return gaze
        if abs(bearing) > COARSE_FACE_MAX_ARM_BEARING_DEG:
            raise RuntimeError(
                f"pick staging face restage reached arm bearing {bearing:+.1f}deg; "
                "refusing another side reposition"
            )
        sign = 1.0 if signed_error >= 0.0 else -1.0
        if sign != initial_sign:
            raise RuntimeError(
                "pick staging face restage crossed the selected face normal; "
                "refusing an opposite-side correction"
            )
        if (
            improvement < COARSE_FACE_MIN_IMPROVEMENT_DEG
            and bearing_progress < COARSE_FACE_MIN_BEARING_PROGRESS_DEG
        ):
            stalled += 1
        else:
            stalled = 0
        if stalled >= max_stalled:
            raise RuntimeError(
                f"pick staging face restage stalled for {stalled} consecutive dog-legs; "
                f"current={current.error_deg:.1f}deg best={best_error:.1f}deg "
                f"bearing-progress={bearing_progress:+.1f}deg"
            )
        previous_error = float(current.error_deg)
        previous_bearing = float(bearing)
    raise RuntimeError(
        f"pick staging face restage remained {previous_error:.1f}deg diagonal after "
        f"{max_repositions} bounded dog-legs (best={best_error:.1f}deg)"
    )


def _orient_chassis_for_straight_close(
    robot: Robot,
    video: Any,
    lock: Any,
    gaze: Gaze,
    precision: Any,
    *,
    flip_x: bool,
    flip_y: bool,
    gaze_mode: str,
    pursue_state: PursueState | None,
) -> Gaze:
    """Point the chassis down the selected face-normal line before close creep.

    The dog-leg above changes the *position* of the chassis until the arm ray is
    face-normal, but deliberately restores the old chassis heading after every
    side step.  If close approach starts from that restored heading, straight
    motion walks off the selected normal line: the target bearing grows as range
    shrinks and servo 6 eventually runs into its physical pan limit.  Once the
    position is correct, rotate in place to absorb the remaining arm bearing,
    re-centre the camera, then re-establish the same 26 cm stopped radius.  The
    ensuing forward/backward capture motion is therefore along the face normal
    and does not need pure mecanum strafe or hidden SIM geometry.
    """
    align_body = getattr(precision, "align_body_to_gaze", None)
    fine_x = getattr(precision, "fine_align_horizontal", None)
    stage = getattr(precision, "approach_with_locked_gaze", None)
    if not all(callable(fn) for fn in (align_body, fine_x, stage)):
        return gaze

    target_nx = float(getattr(precision, "CAPTURE_TARGET_NX", 0.5))
    print(
        f"pick staging face position fixed; orient chassis along final approach ray "
        f"from pan={gaze.pan}",
        flush=True,
    )
    gaze = align_body(
        robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y
    )
    gaze = fine_x(
        robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
    )
    restored = stage(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        flip_y=flip_y,
        target_radius_cm=COARSE_STAGING_RADIUS_CM,
        allow_blind_arrival=False,
        allow_body_realign=False,
        allow_continuous_far=False,
        max_motion_steps=6,
        gaze_mode=gaze_mode,
        pursue_state=pursue_state,
    )
    return restored.gaze


def run_coarse_approach(
    robot: Robot,
    *,
    flip_x: bool = False,
    flip_y: bool = False,
    precision: Any | None = None,
    target_color: str = "red",
    video=None,
    gaze_mode: str | None = None,
    fast_camera_control: bool = False,
) -> int:
    """Bring the target into a robust pre-capture reach corridor."""
    gaze_mode = normalize_gaze_mode(gaze_mode or _default_gaze_mode())
    precision = configure_target_detector(precision or load_precision_controller(), target_color)
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

    if not getattr(robot, "pose_state_fresh", lambda: True)():
        reassert = getattr(robot, "reassert_pose_together", None)
        if not callable(reassert):
            raise RuntimeError(
                "coarse approach requires a trusted camera pose; "
                "pose telemetry is stale and this controller cannot reassert it"
            )
        trusted_pose = {servo: int(robot.pose[servo]) for servo in METRIC_POSE_SERVOS}
        reassert(trusted_pose, duration=POSE_REASSERT_SECONDS)
        if not bool(getattr(robot, "dry_run", True)):
            time.sleep(POSE_REASSERT_SETTLE_SECONDS)
        if not getattr(robot, "pose_state_fresh", lambda: True)():
            raise RuntimeError(
                "coarse approach requires a trusted camera pose; "
                "pose telemetry remains stale after exact-pose reassertion"
            )

    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    current_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
    current_tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
    gaze = Gaze(pan=current_pan, tilt=current_tilt)
    print(f"coarse approach: inherited gaze pan={current_pan} tilt={current_tilt}", flush=True)
    # One shared feedforward memory for the whole run: staging, pre-capture
    # and capture trims all replay the previous pulse's correction, so a
    # steady creep converges to zero recenter loops.  None = legacy behaviour.
    pursue_state = PursueState(gaze=gaze) if gaze_mode != GAZE_MODE_FIXATE else None
    try:
        robot.stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        lock = acquire_inherited_target(precision, video, robot=robot, gaze=gaze)
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
        initial = precision.estimate_block(robot.pose, lock.last)
        if initial is None:
            # The metric floor ray is intentionally bounded to the calibrated
            # coarse corridor.  A real, actor-visible target can therefore be
            # perfectly valid yet still be too far away for metric range (for
            # example seed 11 is ~1.1 m from the base while the floor-ray model
            # rejects >70 cm).  Bearing is still observable from pan+pixel, so
            # absorb a large inherited bearing once, then let the existing
            # guarded far-view fallback drive straight until metric range enters
            # its calibrated domain.  Near/ambiguous invalid geometry still
            # fails inside approach_with_locked_gaze because it must satisfy the
            # visual-far gates before any wheel pulse is allowed.
            initial_bearing = precision.target_bearing_left_deg(
                gaze, lock.last, flip_x=flip_x
            )
            if abs(initial_bearing) > COARSE_BODY_ALIGN_TRIGGER_DEG:
                print(
                    "coarse approach: metric range unavailable at inherited bearing "
                    f"{initial_bearing:+.1f}deg; perform one bounded body re-face",
                    flush=True,
                )
                gaze = precision.align_body_to_gaze(
                    robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y
                )
                robot.stop()
                video.settle(precision.CAMERA_DELAY_SECONDS)
                lock = acquire_inherited_target(precision, video, robot=robot, gaze=gaze)
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
                initial = precision.estimate_block(robot.pose, lock.last)
            if initial is None:
                print(
                    "coarse approach: metric range still unavailable; defer to "
                    "guarded actor-visible far-view staging",
                    flush=True,
                )
        if initial is not None:
            print(
                f"coarse approach initial radius={initial.radius_cm:.2f}cm "
                f"bearing={initial.yaw_left_deg:+.1f}deg",
                flush=True,
            )

        # REAL 2026-09-02 showed repeated near-target yaw corrections can cross
        # the setpoint by >10 deg.  Only a clearly far, clearly off-axis target
        # gets one body-alignment episode.  A modest residual is left to servo 6.
        if (
            initial is not None
            and
            initial.radius_cm >= COARSE_BODY_ALIGN_MIN_RADIUS_CM
            and abs(initial.yaw_left_deg) > COARSE_BODY_ALIGN_TRIGGER_DEG
        ):
            try:
                gaze = precision.align_body_to_gaze(
                    robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y
                )
            except RuntimeError as exc:
                if "overshot to" not in str(exc).lower():
                    raise
                robot.stop()
                video.settle(precision.CAMERA_DELAY_SECONDS)
                lock = acquire_inherited_target(precision, video, robot=robot, gaze=gaze)
                gaze = precision.centre_gaze(
                    robot,
                    video,
                    lock,
                    gaze,
                    flip_x=flip_x,
                    flip_y=flip_y,
                    confirmations=2,
                    allow_limit_lock=True,
                )
                after_overshoot = precision.estimate_block(robot.pose, lock.last)
                if (
                    after_overshoot is None
                    or abs(after_overshoot.yaw_left_deg)
                    > COARSE_BODY_ALIGN_ACCEPT_AFTER_OVERSHOOT_DEG
                ):
                    raise
                print(
                    f"coarse approach: body yaw overshot but fresh bearing is "
                    f"{after_overshoot.yaw_left_deg:+.1f}deg; accept residual for arm yaw",
                    flush=True,
                )

        # Far from contact, keep one bounded camera-guarded forward segment
        # active.  ``continuous_far_approach`` still stops on every fresh
        # observation when the target is lost, lateral drift/range prediction
        # becomes implausible, the pan limit is reached, or the hard radius is
        # approached.  The stopped pulse controller remains authoritative in
        # the near field.  This removes the repeated 0.28 s stop/measure/pulse
        # tax without weakening the REAL safety gates.
        staging = precision.approach_with_locked_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            flip_y=flip_y,
            target_radius_cm=COARSE_STAGING_RADIUS_CM,
            allow_blind_arrival=False,
            allow_body_realign=False,
            allow_continuous_far=bool(fast_camera_control),
            max_motion_steps=COARSE_MAX_MOTION_STEPS,
            gaze_mode=gaze_mode,
            pursue_state=pursue_state,
        )
        gaze = staging.gaze

        # Coarse approach ends here.  Do not pay for face dog-legs, capture-pose
        # transitions or final hand-eye creep in this public skill.  Save only
        # the stopped causal handoff; pick must re-observe before any grasp.
        stop = getattr(robot, "stop", None)
        if callable(stop):
            stop()
        video.settle(precision.CAMERA_DELAY_SECONDS)
        fresh = None
        read_locked = getattr(precision, "read_locked", None)
        if callable(read_locked):
            _frame, fresh = read_locked(video, lock)
        target_visible = fresh is not None or bool(getattr(staging, "target_still_visible", False))
        if not target_visible:
            raise RuntimeError(
                "coarse approach reached staging but lost the target before handoff"
            )
        coarse_radius = (
            None if getattr(staging, "block", None) is None
            else float(staging.block.radius_cm)
        )
        save_coarse_handoff(
            robot_pose=robot.pose,
            target_visible=True,
            target_color=target_color,
            gaze_pan=int(gaze.pan),
            gaze_tilt=int(gaze.tilt),
            coarse_radius_cm=coarse_radius,
        )
        print(
            f"coarse staging ready radius={coarse_radius!r}cm pan={gaze.pan}; "
            "pick owns all near-field alignment, capture-depth motion and IK",
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
    gaze_mode: str | None = None,
    fast_camera_control: bool = False,
) -> int:
    """Public approach: always use the bounded coarse REAL-first path."""
    try:
        from pickup_strategy import clear_precision_fallback
        clear_precision_fallback(target_color)
    except Exception:
        pass
    last_error: RuntimeError | None = None
    for attempt in range(1, MAX_APPROACH_ATTEMPTS + 1):
        try:
            return run_coarse_approach(
                robot,
                flip_x=flip_x,
                flip_y=flip_y,
                precision=precision,
                target_color=target_color,
                video=video,
                gaze_mode=gaze_mode,
                fast_camera_control=fast_camera_control,
            )
        except RuntimeError as exc:
            last_error = exc
            stop = getattr(robot, "stop", None)
            if callable(stop):
                stop()
            if attempt >= MAX_APPROACH_ATTEMPTS or not recoverable_approach_error(exc):
                raise
            print(
                f"coarse approach local recovery {attempt}/{MAX_APPROACH_ATTEMPTS - 1}: "
                f"{exc}; retry current target without public search/track",
                flush=True,
            )
            invalidate_pick_plan()
            time.sleep(0.20)
    assert last_error is not None
    raise last_error


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
    return run_approach(
        robot, flip_x=args.flip_x, flip_y=args.flip_y,
        target_color=getattr(args, "target_color", "red"),
        gaze_mode=getattr(args, "gaze_mode", GAZE_MODE_FIXATE),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="블록을 coarse arm-reach corridor까지만 정지-재측정 방식으로 접근시킵니다."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    parser.add_argument("--gaze-mode", choices=GAZE_MODES, default=GAZE_MODE_FIXATE,
                        help="fixate=legacy locked gaze (default), pursue=per-pulse gaze trim")
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
    extra.extend(["--gaze-mode", getattr(args, "gaze_mode", GAZE_MODE_FIXATE)])
    return deploy_and_run("approach.py", extra, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
