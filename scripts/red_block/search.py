#!/usr/bin/env python3
"""Bounded physical search for the red floor block.

Search is deliberately separate from tracking: sweep the safe eye-in-hand
camera envelope, and if no bounded red target is confirmed, rotate the chassis
in short guarded pulses and sweep again. It never translates the chassis.
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path
from typing import Callable, Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from camera import (
    DEBUG_PATH,
    LiveCamera,
    detect_red_blob,
    detect_red_suspicion,
    detect_target_blob,
    save_debug_frame,
)
from deploy import DEFAULT_HOST, deploy_and_run
from poses import BASE_CENTER, GRIPPER_OPEN, POSE_SEARCH, pose_with_base
from physical_state_machine_reference import CAPTURE_ARM_POSE
from near_look_handoff import invalidate_near_look_handoff, save_near_look_handoff
from robot import Robot, is_robot

PAN_SERVO = 6
TILT_SERVO = 3
PAN_TRACK_MIN = 1050
PAN_TRACK_MAX = 1950
# Search, tracking and precision gaze-lock share the same physically-used
# safe pan envelope.  A confirmed block must not become "not found" merely
# because it was first seen with the head turned sideways; approach will rotate
# the chassis under the locked gaze afterwards.
PAN_SEARCH_MIN = 1050
PAN_SEARCH_MAX = 1950
PAN_EDGE_MARGIN = 80
METRIC_POSE_SERVOS = (3, 4, 5, 6)


def metric_pose_known(pose) -> bool:
    """Metric camera geometry is valid only when every kinematic servo is known."""
    return all(servo in pose for servo in METRIC_POSE_SERVOS)


SEARCH_TRACK_FIXED_SERVOS = (4, 5)


def search_track_pose_compatible(pose) -> bool:
    """Whether a current image was produced by the search/track arm geometry.

    Track is allowed to move only camera tilt (3) and pan/base-yaw (6). Servos
    4/5 therefore identify the fixed search/track kinematic family. A post-pick
    close/verification pose can still show a large red block, but accepting that
    image as SEARCH success hands an invalid camera geometry to approach.
    """
    return metric_pose_known(pose) and all(
        int(pose.get(servo)) == int(POSE_SEARCH[servo])
        for servo in SEARCH_TRACK_FIXED_SERVOS
    )


def current_candidate_is_trackable(pan: int, blob) -> bool:
    """Whether arm-only track still has room to center this current view."""
    if blob is None:
        return False
    # nx<0.5 asks the current controller for larger pan; nx>0.5 asks for
    # smaller pan. Do not report SEARCH achieved if that correction points
    # farther into an already saturated servo limit.
    if pan >= PAN_TRACK_MAX - PAN_EDGE_MARGIN and blob.nx < 0.5:
        return False
    if pan <= PAN_TRACK_MIN + PAN_EDGE_MARGIN and blob.nx > 0.5:
        return False
    return True

# Human-like floor search: keep the chassis still and sweep the eye-in-hand
# camera through a broad sector first.  The sequence is deliberately serpentine
# instead of bouncing 1300 -> 1700 -> 1300 at every tilt.  Only after this full
# head sweep fails do we rotate the chassis into a new sector.
FLOOR_SCAN_POINTS = (
    # Feet / immediate floor first.
    (1500, 500),
    (1300, 540),
    (1125, 560),
    (1050, 600),
    # Sweep back through centre to the other side while looking slightly farther.
    (1225, 620),
    (1450, 640),
    (1700, 620),
    (1875, 600),
    (1950, 620),
    # Return in the opposite direction over the mid/far floor band.
    (1875, 680),
    (1650, 700),
    (1400, 720),
    (1175, 740),
    (1050, 760),
    # One final broad far-floor sweep back toward centre/right.
    (1250, 800),
    (1500, 820),
    (1750, 800),
    (1950, 780),
)
HIGH_FALLBACK_POINTS = (
    (1950, 940),
    (1700, 960),
    (1500, 980),
    (1300, 960),
    (1050, 940),
)
HIGH_FALLBACK_PULSES = {2}
CAMERA_SETTLE_SECONDS = 0.12
TRACK_MIN_AREA = 500
BODY_TURN_SPEED = 35
BODY_TURN_SECONDS = 0.50
DEFAULT_SECONDS = 30.0
MAX_BODY_PULSES = 4
SEARCH_CONFIRM_HITS = 3
SEARCH_CONFIRM_MAX_READS = 9
# Once the component is already cube-sized and inside the tracker envelope,
# one additional consistent frame is enough.  Requiring three total hits made
# the real robot repeatedly sweep away from 1.6k..6k px blocks it had already
# seen clearly.  Weak/distant candidates keep the stricter three-hit gate.
OBVIOUS_CONFIRM_HITS = 2
OBVIOUS_CONFIRM_MAX_READS = 4
OBVIOUS_FRONT_MIN_AREA = 1500
OBVIOUS_FRONT_X_DEADBAND = 0.40
# Peripheral red is observation, not confirmation. Never discard it: use it
# to steer the camera toward the visible fragment and re-run the normal detector.
SUSPICION_MIN_AREA = 120
SUSPICION_PAN_GAIN = 260.0
SUSPICION_TILT_GAIN = 180.0
SUSPICION_MIN_STEP = 55
SUSPICION_MAX_STEP = 170
SUSPICION_REFINEMENT_STEPS = 2
BOTTOM_HINT_NY = 0.80
NEAR_LOOK_MOVE_SECONDS = 0.85
NEAR_LOOK_SETTLE_SECONDS = 0.18


def obvious_front_candidate(pan: int, blob) -> bool:
    """A clearly visible block anywhere in the safe head sector ends search now."""
    return (
        blob is not None
        and PAN_TRACK_MIN <= int(pan) <= PAN_TRACK_MAX
        and current_candidate_is_trackable(int(pan), blob)
        and float(blob.area) >= OBVIOUS_FRONT_MIN_AREA
        and abs(float(blob.nx) - 0.5) <= OBVIOUS_FRONT_X_DEADBAND
    )


def head_move_duration(previous_pan: int, previous_tilt: int, pan: int, tilt: int) -> float:
    """Move the camera smoothly instead of snapping between scan points."""
    travel = max(abs(int(pan) - int(previous_pan)), abs(int(tilt) - int(previous_tilt)))
    return min(0.40, max(0.18, 0.16 + travel / 3000.0))


def settle_after_head_move(duration: float) -> None:
    """Wait until a commanded head interpolation is actually complete.

    ``Robot.nudge_servos`` intentionally returns after a short 50 ms command
    handoff because tracking uses 100 ms micro-corrections.  Search uses much
    wider 180..400 ms sweeps, so reading after only the generic camera delay can
    sample while the head is still moving.  Account for that 50 ms handoff and
    then add the camera's post-motion settling delay.
    """
    remaining_motion = max(0.0, float(duration) - 0.05)
    time.sleep(remaining_motion + CAMERA_SETTLE_SECONDS)


def same_candidate(a, b) -> bool:
    if a is None or b is None:
        return False
    ratio = b.area / max(1.0, float(a.area))
    # Distant cubes can be only ~400-700 px and jitter more under compression.
    # Require spatial consistency but allow wider area variation than a close
    # tracking target.
    return (
        abs(a.nx - b.nx) <= 0.10
        and abs(a.ny - b.ny) <= 0.10
        and 0.30 <= ratio <= 2.5
    )


def valid_search_candidate(blob, frame) -> bool:
    """Search never locks onto robot/self fragments clipped by image borders."""
    if blob is None:
        return False
    height, width = frame.shape[:2]
    left = blob.cx - blob.width // 2
    right = blob.cx + blob.width // 2
    top = blob.cy - blob.height // 2
    bottom = blob.cy + blob.height // 2
    # The gripper/self artifact is primarily bottom-clipped and the left crop
    # contains arm geometry. A real floor cube can legitimately enter from
    # the right edge; rejecting every right-clipped red target made search
    # ignore user-visible blocks near the right side of the frame.
    return left > 101 and top > 2 and bottom < height - 3


def read_search_candidate(camera: LiveCamera, detector: Callable = detect_red_blob):
    frame = camera.read(quiet=True)
    blob = detector(frame, min_area=TRACK_MIN_AREA, crop_left=100)
    return frame, blob if valid_search_candidate(blob, frame) else None


def read_search_observation(
    camera: LiveCamera,
    detector: Callable = detect_red_blob,
    *,
    allow_red_suspicion: bool = True,
):
    """Read selected-colour evidence and optional red-only peripheral hints."""
    frame, candidate = read_search_candidate(camera, detector)
    suspicion = None
    if candidate is None and allow_red_suspicion:
        try:
            suspicion = detect_red_suspicion(frame, min_area=SUSPICION_MIN_AREA)
        except Exception:
            # The peripheral channel is best-effort. It must never break the
            # ordinary bounded search path if a frame/backend is malformed.
            suspicion = None
    return frame, candidate, suspicion


def _bounded_hint_step(error: float, gain: float) -> int:
    if abs(error) < 0.04:
        return 0
    magnitude = int(round(abs(error) * gain))
    magnitude = max(SUSPICION_MIN_STEP, min(SUSPICION_MAX_STEP, magnitude))
    return magnitude if error > 0 else -magnitude


def suspicion_gaze_target(pan: int, tilt: int, blob) -> tuple[int, int]:
    """Move gaze toward visible red without claiming that red is the block."""
    pan_delta = _bounded_hint_step(0.5 - float(blob.nx), SUSPICION_PAN_GAIN)
    tilt_delta = _bounded_hint_step(0.5 - float(blob.ny), SUSPICION_TILT_GAIN)
    return (
        max(PAN_SEARCH_MIN, min(PAN_SEARCH_MAX, int(pan) + pan_delta)),
        max(500, min(980, int(tilt) + tilt_delta)),
    )


def _read_near_look_candidate(
    camera: LiveCamera, detector: Callable = detect_red_blob
):
    frame = camera.read(quiet=True)
    blob = detector(
        frame, min_area=TRACK_MIN_AREA, crop_left=0,
        edge_frac=0.0, min_edge_area=0,
    )
    return frame, blob


def _confirm_near_look(
    camera: LiveCamera, first, detector: Callable = detect_red_blob
):
    if first is None:
        return None
    matches = [first]
    for _ in range(4):
        time.sleep(0.045)
        _frame, current = _read_near_look_candidate(camera, detector)
        if current is not None and same_candidate(first, current):
            matches.append(current)
            if len(matches) >= 2:
                return current
    return None


def refine_bottom_near_look(
    robot: Robot, camera: LiveCamera, detector: Callable = detect_red_blob, *, phase: str
):
    """Inspect the immediate floor when search tilt is already saturated down."""
    pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
    updates = dict(CAPTURE_ARM_POSE)
    updates[PAN_SERVO] = pan
    print(
        f"search {phase}: red is below the normal floor view at tilt={robot.pose.get(TILT_SERVO)}; "
        "switch to close near-look camera geometry instead of ignoring it",
        flush=True,
    )
    robot.nudge_servos(updates, duration=NEAR_LOOK_MOVE_SECONDS)
    settle_after_head_move(NEAR_LOOK_MOVE_SECONDS)
    time.sleep(NEAR_LOOK_SETTLE_SECONDS)
    camera.read(quiet=True)
    time.sleep(0.04)
    frame, candidate = _read_near_look_candidate(camera, detector)
    found = _confirm_near_look(camera, candidate, detector)
    if found is None:
        return None
    save_debug_frame(frame, found, DEBUG_PATH, label=f"search-{phase}-near-look-red")
    if not bool(getattr(robot, "dry_run", True)):
        save_near_look_handoff(robot_pose=robot.pose)
    print(
        f"search confirmed red phase={phase}-near-look pan={pan} "
        f"nx={found.nx:.3f} ny={found.ny:.3f} area={found.area}",
        flush=True,
    )
    return found


def _bottom_hint_needs_near_look(tilt: int, hint) -> bool:
    return int(tilt) <= 500 and float(hint.ny) >= BOTTOM_HINT_NY


def refine_suspicion(
    robot: Robot,
    camera: LiveCamera,
    suspicion,
    detector: Callable = detect_red_blob,
    *,
    phase: str,
):
    """Follow visible peripheral red immediately and re-run normal detection."""
    hint = suspicion
    for step in range(1, SUSPICION_REFINEMENT_STEPS + 1):
        pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
        tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
        if _bottom_hint_needs_near_look(tilt, hint):
            return refine_bottom_near_look(
                robot, camera, detector, phase=phase
            )
        target_pan, target_tilt = suspicion_gaze_target(pan, tilt, hint)
        if (target_pan, target_tilt) == (pan, tilt):
            return None
        print(
            f"search {phase}: peripheral red hint nx={hint.nx:.3f} ny={hint.ny:.3f} "
            f"area={hint.area} -> look pan={target_pan} tilt={target_tilt} "
            f"step={step}/{SUSPICION_REFINEMENT_STEPS}",
            flush=True,
        )
        duration = head_move_duration(pan, tilt, target_pan, target_tilt)
        robot.nudge_servos(
            {PAN_SERVO: target_pan, TILT_SERVO: target_tilt}, duration=duration
        )
        settle_after_head_move(duration)
        camera.read(quiet=True)
        time.sleep(0.04)
        frame, candidate, next_hint = read_search_observation(camera, detector)
        if candidate is not None:
            found = confirm_search_candidate(camera, target_pan, candidate, detector)
            if found is not None:
                save_debug_frame(
                    frame, found, DEBUG_PATH, label=f"search-{phase}-hint-refined-red"
                )
                print(
                    f"search confirmed red phase={phase}-hint-refined "
                    f"pan={target_pan} tilt={target_tilt} nx={found.nx:.3f} "
                    f"ny={found.ny:.3f} area={found.area}",
                    flush=True,
                )
                return found
        if next_hint is None:
            return None
        hint = next_hint
    return None


def confirmed_red(
    camera: LiveCamera,
    first,
    detector: Callable = detect_red_blob,
    *,
    required_hits: int = SEARCH_CONFIRM_HITS,
    max_reads: int = SEARCH_CONFIRM_MAX_READS,
):
    """Confirm a candidate by repeated hits at the same stopped gaze.

    A single frame is never enough.  Strong cube-sized evidence may use the
    two-hit fast path; weak/distant evidence retains the stricter three-hit
    policy so lowering search latency does not re-introduce red-speck locks.
    """
    if first is None:
        return None
    required_hits = max(2, int(required_hits))
    max_reads = max(1, int(max_reads))
    matches = [first]
    for _ in range(max_reads):
        time.sleep(0.045)
        _frame, current = read_search_candidate(camera, detector)
        if current is not None and same_candidate(first, current):
            matches.append(current)
            if len(matches) >= required_hits:
                return current
    return None


def confirm_search_candidate(
    camera: LiveCamera,
    pan: int,
    candidate,
    detector: Callable = detect_red_blob,
):
    """Use a fast but still multi-frame gate for an obvious floor block."""
    if obvious_front_candidate(pan, candidate):
        return confirmed_red(
            camera, candidate, detector,
            required_hits=OBVIOUS_CONFIRM_HITS,
            max_reads=OBVIOUS_CONFIRM_MAX_READS,
        )
    return confirmed_red(camera, candidate, detector)


def _sighting_rank(pan: int, blob) -> tuple[int, int, float, float]:
    """Prefer block-sized evidence before tiny red specks.

    The previous ordering preferred a ~350 px centered red patch over a
    5,000+ px cube seen sideways.  That made search repeatedly turn away from
    the user's clearly visible block.  Strong, repeatedly visible cube-sized
    evidence wins first; inheritance/centering only break ties afterwards.
    """
    strong = int(float(blob.area) >= OBVIOUS_FRONT_MIN_AREA)
    trackable = int(
        PAN_TRACK_MIN <= int(pan) <= PAN_TRACK_MAX
        and current_candidate_is_trackable(int(pan), blob)
    )
    centre_quality = -abs(float(blob.nx) - 0.5)
    return strong, trackable, centre_quality, float(blob.area)


def _confirm_at_gaze(
    robot: Robot,
    camera: LiveCamera,
    pan: int,
    tilt: int,
    detector: Callable,
    *,
    phase: str,
):
    previous_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
    previous_tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
    duration = head_move_duration(previous_pan, previous_tilt, pan, tilt)
    robot.nudge_servos(
        {PAN_SERVO: pan, TILT_SERVO: tilt},
        duration=duration,
    )
    settle_after_head_move(duration)
    camera.read(quiet=True)
    time.sleep(0.05)
    frame, candidate = read_search_candidate(camera, detector)
    found = confirmed_red(camera, candidate, detector)
    if found is not None:
        save_debug_frame(frame, found, DEBUG_PATH, label=f"search-{phase}-red")
        print(
            f"search confirmed red phase={phase} pan={pan} tilt={tilt} "
            f"nx={found.nx:.3f} ny={found.ny:.3f} area={found.area}",
            flush=True,
        )
    return found


def scan_points(
    robot: Robot,
    camera: LiveCamera,
    points,
    detector: Callable = detect_red_blob,
    *,
    phase: str = "floor",
    allow_red_suspicion: bool = True,
):
    """Sweep the whole head sector, remember sightings, then verify the best one.

    This deliberately does *not* stop at the first red frame. A human search
    first scans the scene, remembers where the object looked clearest, then
    returns gaze to that direction. This also avoids body turns caused by one
    transient peripheral detection.
    """
    previous_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
    previous_tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
    sightings = []
    for pan, tilt in points:
        duration = head_move_duration(previous_pan, previous_tilt, pan, tilt)
        robot.nudge_servos({PAN_SERVO: pan, TILT_SERVO: tilt}, duration=duration)
        previous_pan, previous_tilt = pan, tilt
        settle_after_head_move(duration)
        camera.read(quiet=True)
        time.sleep(0.05)
        frame, candidate, suspicion = read_search_observation(
            camera, detector, allow_red_suspicion=allow_red_suspicion,
        )
        if candidate is not None:
            detail = f"yes nx={candidate.nx:.3f} ny={candidate.ny:.3f} area={candidate.area}"
        elif suspicion is not None:
            detail = (
                f"hint nx={suspicion.nx:.3f} ny={suspicion.ny:.3f} "
                f"area={suspicion.area}"
            )
        else:
            detail = "no"
        print(f"search {phase} pan={pan} tilt={tilt} red={detail}", flush=True)
        if candidate is None and suspicion is not None:
            found = refine_suspicion(
                robot, camera, suspicion, detector, phase=phase
            )
            if found is not None:
                return found
            previous_pan = int(robot.pose.get(PAN_SERVO, pan))
            previous_tilt = int(robot.pose.get(TILT_SERVO, tilt))
        if obvious_front_candidate(pan, candidate):
            print(
                f"search: obvious block at pan={pan}; confirm now and preserve this gaze",
                flush=True,
            )
            found = confirm_search_candidate(camera, pan, candidate, detector)
            if found is not None:
                save_debug_frame(frame, found, DEBUG_PATH, label=f"search-{phase}-obvious-red")
                print(
                    f"search confirmed red phase={phase}-obvious pan={pan} tilt={tilt} "
                    f"nx={found.nx:.3f} ny={found.ny:.3f} area={found.area}",
                    flush=True,
                )
                return found
        if candidate is not None:
            sightings.append((pan, tilt, candidate))

    if not sightings:
        return None

    # Try the most tracker-friendly sightings first. If a transient candidate
    # vanishes when we look back, fall through to the next remembered view.
    sightings.sort(key=lambda item: _sighting_rank(item[0], item[2]), reverse=True)
    for pan, tilt, remembered in sightings:
        print(
            f"search: look back at best sighting pan={pan} tilt={tilt} "
            f"remembered-nx={remembered.nx:.3f}",
            flush=True,
        )
        found = _confirm_at_gaze(
            robot, camera, pan, tilt, detector, phase=phase
        )
        if found is not None:
            return found
    return None


def scan_once(robot: Robot, camera: LiveCamera, detector: Callable = detect_red_blob):
    """Compatibility helper: one floor-priority scan without body motion."""
    return scan_points(robot, camera, FLOOR_SCAN_POINTS, detector, phase="floor")


def run_search(
    robot: Robot,
    *,
    seconds: float = DEFAULT_SECONDS,
    detector: Callable = detect_red_blob,
    target_color: str = "red",
    camera: LiveCamera | None = None,
) -> int:
    invalidate_near_look_handoff()
    owns_camera = camera is None
    camera = camera or LiveCamera()
    try:
        # First respect what the robot already sees. Search should not destroy
        # a useful current gaze before checking it, especially when a target is
        # already visible near the side of the frame.
        camera.read(quiet=True)
        time.sleep(0.04)
        frame, current, current_hint = read_search_observation(
            camera, detector, allow_red_suspicion=(target_color == "red"),
        )
        current_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
        found = confirm_search_candidate(camera, current_pan, current, detector)
        current_trackable = (
            found is not None and current_candidate_is_trackable(current_pan, found)
        )
        pose_fresh = robot.pose_state_fresh()
        current_pose_compatible = search_track_pose_compatible(robot.pose)
        if found is None and current_hint is not None and current_pose_compatible and pose_fresh:
            refined = refine_suspicion(
                robot, camera, current_hint, detector, phase="current-view"
            )
            if refined is not None:
                robot.stop()
                return 0
        incompatible_current_pose = (
            current_trackable
            and metric_pose_known(robot.pose)
            and not current_pose_compatible
        )
        if incompatible_current_pose:
            print(
                "search current target is visible from non-search arm geometry; "
                "reset to calibrated floor-search pose before handoff",
                flush=True,
            )
            current_trackable = False
        if current_trackable and current_pose_compatible and pose_fresh:
            save_debug_frame(frame, found, DEBUG_PATH, label="search-current-red")
            print(
                f"search confirmed red phase=current-view nx={found.nx:.3f} "
                f"ny={found.ny:.3f} area={found.area} pan={current_pan}",
                flush=True,
            )
            robot.stop()
            return 0
        if current_trackable and current_pose_compatible and not pose_fresh:
            # A stale timestamp alone must not make us look away from a block
            # that is already in view.  Re-command the saved metric pose in one
            # concurrent interpolation, wait for it to settle, and prove the
            # target is still there before falling back to a fresh scan.
            trusted_pose = {servo: int(robot.pose[servo]) for servo in METRIC_POSE_SERVOS}
            print(
                "search current target visible with stale pose; reassert current "
                "camera/arm pose once and re-confirm before scanning",
                flush=True,
            )
            robot.reassert_pose_together(trusted_pose, duration=1.25)
            time.sleep(CAMERA_SETTLE_SECONDS)
            camera.read(quiet=True)
            time.sleep(0.04)
            frame, reacquired = read_search_candidate(camera, detector)
            current_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
            found = confirm_search_candidate(camera, current_pan, reacquired, detector)
            if found is not None and current_candidate_is_trackable(current_pan, found):
                save_debug_frame(
                    frame, found, DEBUG_PATH, label="search-current-reasserted-red"
                )
                print(
                    f"search confirmed red phase=current-reasserted nx={found.nx:.3f} "
                    f"ny={found.ny:.3f} area={found.area} pan={current_pan}",
                    flush=True,
                )
                robot.stop()
                return 0
            current_trackable = False
        if current_trackable:
            missing = [servo for servo in METRIC_POSE_SERVOS if servo not in robot.pose]
            reason = (
                f"missing servos {missing}" if missing
                else f"pose telemetry stale age={robot.pose_state_age_s!r}s"
            )
            print(
                "search current target visible but metric arm pose is not trusted "
                f"({reason}); establish calibrated search pose and reacquire",
                flush=True,
            )
        elif found is not None and not incompatible_current_pose:
            print(
                f"search current target is beyond trackable pan range "
                f"pan={current_pan} nx={found.nx:.3f}; reacquire from centered floor scan",
                flush=True,
            )

        search_pose = pose_with_base(POSE_SEARCH, BASE_CENTER, GRIPPER_OPEN)
        search_pose[TILT_SERVO] = FLOOR_SCAN_POINTS[0][1]
        print("search: establish calibrated pose -> feet-first floor scan", flush=True)
        if not pose_fresh and metric_pose_known(robot.pose):
            # The pose timestamp can expire between separate harness skills even
            # when all commanded joints are known. Reassert the safe search pose
            # concurrently and wait once, instead of clearing state and paying
            # five independent 1.5 s waits. The commands are sent even when a
            # saved pulse already matches, because stale state is not trusted.
            robot.reassert_pose_together(search_pose, duration=1.25)
        else:
            if not pose_fresh:
                # Incomplete state is genuinely unknown: retain the conservative
                # ordered path rather than pretending a concurrent reset is safe.
                robot.pose.clear()
            robot.move_pose(search_pose, lowering=True)
        # `seconds` is actual SEARCH time after setup.
        deadline = time.monotonic() + seconds
        for body_pulse in range(MAX_BODY_PULSES + 1):
            # Floor gets first chance at every chassis heading. A higher scan
            # is deliberately rare so a floor target is not starved by arm
            # motion that usually cannot help the task.
            found = scan_points(
                robot, camera, FLOOR_SCAN_POINTS, detector, phase="floor",
                allow_red_suspicion=(target_color == "red"),
            )
            if found is not None:
                found_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
                if (
                    PAN_TRACK_MIN <= found_pan <= PAN_TRACK_MAX
                    and current_candidate_is_trackable(found_pan, found)
                ):
                    robot.stop()
                    return 0
                # Keep looking at the target while moving gaze only as far as
                # the precision tracker can inherit. If it remains visible at
                # that boundary, let track/body-align take over. Only if the
                # object is genuinely outside the tracker sector do a single
                # torso turn while keeping the head pointed to that side.
                found_tilt = int(robot.pose.get(TILT_SERVO, POSE_SEARCH[3]))
                handoff_pan = (
                    PAN_TRACK_MIN if found_pan < PAN_TRACK_MIN else PAN_TRACK_MAX
                )
                print(
                    f"search: peripheral target seen pan={found_pan}; "
                    f"keep sight and hand off toward pan={handoff_pan}",
                    flush=True,
                )
                handoff = _confirm_at_gaze(
                    robot, camera, handoff_pan, found_tilt, detector,
                    phase="peripheral-handoff",
                )
                if handoff is not None and current_candidate_is_trackable(handoff_pan, handoff):
                    robot.stop()
                    return 0
                if handoff is not None:
                    direction = "rotate-right" if handoff.nx > 0.5 else "rotate-left"
                else:
                    direction = "rotate-left" if found_pan > BASE_CENTER else "rotate-right"
                duration = 0.18
                print(
                    f"search: target still outside tracker sector -> "
                    f"hold gaze pan={handoff_pan} and turn body {direction} {duration:.2f}s",
                    flush=True,
                )
                robot.drive(direction, BODY_TURN_SPEED, duration)
                time.sleep(CAMERA_SETTLE_SECONDS)
                continue
            if body_pulse in HIGH_FALLBACK_PULSES:
                found = scan_points(
                    robot, camera, HIGH_FALLBACK_POINTS, detector, phase="high-fallback",
                    allow_red_suspicion=(target_color == "red"),
                )
                if found is not None:
                    found_pan = int(robot.pose.get(PAN_SERVO, BASE_CENTER))
                    if PAN_TRACK_MIN <= found_pan <= PAN_TRACK_MAX:
                        robot.stop()
                        return 0
            if body_pulse >= MAX_BODY_PULSES or time.monotonic() >= deadline:
                break
            pulse_no = body_pulse + 1
            print(
                f"search: full head sweep empty -> rotate body sector {pulse_no}/{MAX_BODY_PULSES}",
                flush=True,
            )
            # No target is visible in this branch, so gaze recentering does not
            # carry measurement authority.  Let the PWM board recenter the head
            # while the chassis enters the next search sector instead of waiting
            # for one subsystem and then starting the other.
            robot.move_servos_and_drive(
                {PAN_SERVO: BASE_CENTER, TILT_SERVO: POSE_SEARCH[3]},
                servo_duration=0.30,
                direction="rotate-left",
                speed=BODY_TURN_SPEED,
                drive_duration=BODY_TURN_SECONDS,
                settle=CAMERA_SETTLE_SECONDS,
            )
        frame = camera.read(quiet=True)
        save_debug_frame(frame, None, DEBUG_PATH, label="search-miss")
        raise RuntimeError(
            f"{target_color} block not confirmed after bounded body search; debug frame at {DEBUG_PATH}"
        )
    finally:
        if owns_camera:
            camera.close()
        robot.stop()


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
    target_color = getattr(args, "target_color", "red")
    detector = lambda frame, **kwargs: detect_target_blob(frame, target_color, **kwargs)
    return run_search(robot, seconds=args.seconds, detector=detector, target_color=target_color)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="바닥을 우선 훑고 제자리 회전하며 빨간 블록을 찾습니다.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 2.0 <= args.seconds <= 60.0:
        raise ValueError("seconds must be between 2 and 60")
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra = ["--seconds", str(args.seconds), "--target-color", args.target_color]
    if args.dry_run:
        extra.append("--dry-run")
    return deploy_and_run("search.py", extra, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
