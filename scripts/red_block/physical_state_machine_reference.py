#!/usr/bin/env python3
"""Face-aligned, camera-verified red-block pickup for MasterPi.

The camera is mounted parallel to the arm and cannot see the gripper.  The
controller therefore gathers all visual information before the blind grasp:
slow scan, target lock, gaze-stabilised chassis alignment/approach, a complete
stop, delay flushing, multi-frame coordinate estimation, FK/IK, then grasp.
Before the final approach it projects the visible lower block face onto the
floor and uses guarded mecanum orbit pulses to avoid a diagonal grasp.  After
lifting, it returns the camera to the exact pre-grasp view and verifies that
the block disappeared; a visible block triggers one fully remeasured retry.

This file intentionally contains no buzzer command and imports none of the
MasterPi colour-demo functions that use the buzzer.
"""

from __future__ import annotations

import argparse
from contextvars import ContextVar
import math
import signal
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np


RED_BLOCK_DIR = Path(__file__).resolve().parent
if str(RED_BLOCK_DIR) not in sys.path:
    sys.path.insert(0, str(RED_BLOCK_DIR))

from camera import (
    DEBUG_PATH,
    capture_bgr,
    detect_red_blob as _default_detect_red_blob,
    save_debug_frame,
)
from gaze_hold import (
    GAZE_MODE_FIXATE,
    PURSUE_TRIM_GAIN,
    PursueState,
    clamp_hold,
    normalize_gaze_mode,
    recall_corr,
    store_corr,
)
from recorder import record_camera_frame
from poses import BASE_CENTER, GRIPPER_ID, POSE_SEARCH, clamp_pulse
from geometry import cube_range_estimate_cm, undistorted_ray
from robot import Robot
from track import DEADBAND as TRACK_DEADBAND, Gaze, PAN_SERVO, TILT_SERVO, TRACK_MIN_AREA, tracking_step


# The REAL CLI normally runs one skill per process, while the multi-robot SIM
# runs several copies of this controller in worker threads.  Mutating the
# module-global ``detect_red_blob`` binding made blue/yellow actions race with
# each other and forced the whole public skill behind one global lock.  Keep
# the physical red detector as the default and select an alternate detector in
# the current execution context only.
_TARGET_DETECTOR = ContextVar("ugrp_precision_target_detector", default=None)


def set_target_detector(detector) -> None:
    """Select a detector for the current controller execution context."""
    _TARGET_DETECTOR.set(detector)


def clear_target_detector() -> None:
    """Restore the physical red detector for the current execution context."""
    _TARGET_DETECTOR.set(None)


def detect_red_blob(frame, **kwargs):
    detector = _TARGET_DETECTOR.get() or _default_detect_red_blob
    return detector(frame, **kwargs)


STREAM_URL = "http://127.0.0.1:8080/stream"
FRAME_WIDTH = 640.0
FRAME_HEIGHT = 480.0

# Verified physical setup and SDK link lengths, centimetres.
LINK_1 = 9.30
LINK_2 = 6.50
LINK_3 = 6.20
GRIPPER_LINK = 10.00
PULSE_PER_DEGREE = 2000.0 / 180.0
SERVO_DEVIATION = {3: 54, 4: 53, 5: 89, 6: 64}

# User-verified arm reference pulses.  Servo 1 uses the official maximum-open
# pickup value so larger objects can enter the jaws before descent.
BASIC_POSE = {1: 2000, 3: 700, 4: 2200, 5: 780, 6: 1500}
SCAN_ARM_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
# Legacy high carry pose used for transport when no visual destination search is
# required. Its eye-in-hand camera points above the floor in the current MasterPi
# geometry, so it must not be the post-pick pose for autonomous delivery.
CARRY_POSE = {1: 1500, 3: 700, 4: 2200, 5: 780}
# Delivery carry pose: keep the cube well above the floor while retaining a
# downward camera ray.  This is composed only from the same REAL servo/FK model;
# no simulator object truth is used.  The 2026-08-31 seed-11 trace passed through
# essentially this configuration while holding with bilateral contact before the
# old code continued upward into CARRY_POSE.
DELIVERY_CARRY_POSE = {1: 1500, 3: 600, 4: 2200, 5: 1400}
GRIPPER_OPEN = 2000
GRIPPER_CLOSE = 1500

# Safe camera/arm tracking envelope.
PAN_MIN = 1050
PAN_MAX = 1950
TILT_MIN = 500
TILT_MAX = 1200
SCAN_PANS = (1300, 1500, 1700, 1500)
SCAN_TILTS = (650, 820, 990, 1160)
SCAN_STEP = 20
TRACK_PAN_STEP = 14
TRACK_PAN_MEDIUM_STEP = 28
TRACK_PAN_COARSE_STEP = 40
TRACK_TILT_STEP = 12
TRACK_TILT_MEDIUM_STEP = 24
TRACK_TILT_COARSE_STEP = 36
TRACK_MEDIUM_ERROR = 0.07
TRACK_COARSE_ERROR = 0.12
# Must not be narrower than track.DEADBAND. Otherwise the PID can intentionally
# stop correcting at ~5% image error while this state machine waits forever for
# a tighter lock (observed physically near the block at the lower tilt limit).
IMAGE_X_DEADBAND = max(0.055, TRACK_DEADBAND)
IMAGE_Y_DEADBAND = max(0.055, TRACK_DEADBAND)
FINAL_X_DEADBAND = 0.012
FINAL_X_DRIFT_LIMIT = 0.025
FINAL_X_CONFIRMATIONS = 3
# Error-banded pan correction. Keep the historical 8-PWM precision cap as
# FINAL_X_MAX_STEP_PULSE for compatibility, but allow larger bounded corrections
# when the error is clearly outside the precision band.
FINAL_X_MAX_STEP_PULSE = 8
FINAL_X_COARSE_MAX_STEP_PULSE = 28
FINAL_X_VERY_COARSE_MAX_STEP_PULSE = 48
FINAL_X_FINE_STEP_PULSE = 14
FINAL_X_PRECISION_STEP_PULSE = 6
FINAL_X_OVERSHOOT_STEP_PULSE = 4
FINAL_X_RESPONSE_EPSILON = 0.006
FINAL_X_VERY_COARSE_ERROR = 0.15
FINAL_X_COARSE_ERROR = 0.05
FINAL_X_FINE_ERROR = 0.025
FINAL_X_MAX_ITERATIONS = 24
FINAL_LATERAL_MAD_MAX_CM = 0.35
# Chassis re-facing must use the target bearing, not pan PWM alone.  The
# tracker intentionally accepts a few percent of residual image error; after a
# mecanum strafe that residual can represent several degrees of target bearing
# even when the pan servo itself is near centre.
BODY_BEARING_DEADBAND_DEG = 0.5
# Do not chase camera/FK noise once the chassis is already visually straight.
# A residual in this corridor gets at most one physical fine pulse, then the
# controller proceeds with approach unless the next fresh measurement is worse.
BODY_FINE_ACCEPT_DEG = 3.0
BODY_REALIGN_LIMIT = 70
PAN_LIMIT_MARGIN = 25

# Video timing. LiveVideo drops queued frames; settling still ensures the
# newest frame was captured after a physical motion, not during it.
CAMERA_DELAY_SECONDS = 0.12
SERVO_SETTLE_SECONDS = 0.08
TARGET_MISSING_LIMIT = 5
LOCK_HOLD_FRAMES = 18
ACQUIRE_CONFIRMATIONS = 3
CENTER_CONFIRMATIONS = 3
FINAL_SAMPLE_COUNT = 7
FINAL_MIN_VALID_SAMPLES = 5
# The physical defaults use three range/capture samples. SIM may lower these
# through its environment adapter because its rendered sensor is deterministic;
# REAL keeps the multi-frame noise rejection unchanged.
APPROACH_RANGE_SAMPLE_COUNT = 3
APPROACH_RANGE_MIN_VALID = 2
CAPTURE_SAMPLE_COUNT = 3
CAPTURE_SAMPLE_MIN_HITS = 2

# Block-face alignment.  REAL evidence showed mecanum pure strafe is not a
# trustworthy positioning primitive here: nominal left/right motion can leak
# yaw and fore/aft travel.  Face alignment therefore repositions with a bounded
# dog-leg made only from the physically reliable rotate + forward/backward
# primitives, then judges progress from a fresh stopped face measurement.
FACE_ALIGN_START_RADIUS_CM = 25.5
FACE_ALIGNMENT_TOLERANCE_DEG = 12.0
FACE_ALIGNMENT_ABORT_DEG = 20.0
FACE_MIN_RECTANGULARITY = 0.48
FACE_SAMPLE_COUNT = 5
FACE_MIN_VALID_SAMPLES = 3

# Historical pure-strafe diagnostic constants are retained for the standalone
# diagnostic helper/public primitive, but face-alignment controllers do not use
# that helper anymore.
ORBIT_SPEED = 40
ORBIT_PULSE_SECONDS = 0.20
ORBIT_MAX_PULSES = 10
ORBIT_MIN_BEARING_PROGRESS_DEG = 1.0
ORBIT_MAX_BEARING_STEP_DEG = 18.0
ORBIT_MAX_LOW_PROGRESS_PULSES = 2
ORBIT_TARGET_MARGIN_DEG = 3.0

# Non-strafe side reposition.  To move right while increasing clearance, yaw
# left, back up, then undo the yaw; mirror for a left reposition.  The following
# straight depth-restoration stage removes the intentional backward component.
FACE_REPOSITION_TURN_SECONDS = 0.18
# Restore is a separate calibration axis. REAL currently uses the same measured
# duration in both directions; SIM may override only the restore leg to account
# for its first-order drivetrain response without changing the shared policy.
FACE_REPOSITION_RESTORE_TURN_SECONDS = FACE_REPOSITION_TURN_SECONDS
# REAL traces showed that ~0.08 s low-speed linear pulses can sit inside the
# drivetrain deadband and barely change range.  The dog-leg is our replacement
# for unreliable pure mecanum strafe, so its straight segment must be long
# enough to create a measurable side viewpoint before we spend another vision
# cycle evaluating it.
FACE_REPOSITION_BACKWARD_SECONDS = 0.18
FACE_REPOSITION_MAX_PULSES = 6
FACE_REPOSITION_MIN_IMPROVEMENT_DEG = 1.0
FACE_REPOSITION_MAX_STALLED_PULSES = 2

# Final face placement uses the same non-strafe reposition and servo 6 to keep
# the arm ray on the cube.  Success is the stopped face-normal error relative to
# the arm approach ray, never raw image bearing change during chassis motion.
ARM_FACE_MAX_PULSES = FACE_REPOSITION_MAX_PULSES
ARM_FACE_MIN_IMPROVEMENT_DEG = FACE_REPOSITION_MIN_IMPROVEMENT_DEG
ARM_FACE_MAX_STALLED_PULSES = FACE_REPOSITION_MAX_STALLED_PULSES
ARM_FACE_MAX_BEARING_DEG = 38.0
ARM_FACE_MAX_STEP_NY = 0.08
# Stop the first close-camera stage while the full oriented cube contour is
# still projectable onto the floor. REAL 2026-08-31 at ny~=0.312 produced a
# stable face estimate, while archived REAL frames at ny=0.408/0.412 already
# made floor-face projection unavailable. Therefore face orientation is measured
# here, before the separate calibrated hand-eye reach refinement to ny~=0.442.
ARM_FACE_CAPTURE_TARGET_NY = 0.30
ARM_FACE_RANGE_SAMPLE_COUNT = 5
ARM_FACE_MIN_RANGE_SAMPLES = 3

# uStreamer can close one MJPEG HTTP connection while remaining healthy.  A
# transient EOF must not abort a long approach, but reconnects remain bounded.
VIDEO_MAX_RECONNECTS = 10
VIDEO_RECONNECT_DELAY_SECONDS = 0.12

# Grasp verification uses the same camera pose before and after the blind
# descent.  PWM servo 1 exposes no force/current feedback on this MasterPi, so
# a matched red detection remaining on the floor is the reliable failure cue.
VERIFY_FRAME_COUNT = 9
VERIFY_MIN_REFERENCE_HITS = 4
VERIFY_MAX_SUCCESS_HITS = 1
VERIFY_MATCH_DISTANCE = 0.45
VERIFY_SETTLE_SECONDS = 0.45
MAX_PICK_ATTEMPTS = 2

# Verified chassis patterns at the MotorTransport API.
MOTOR_SIGNS = {
    "forward": ((1, -1), (2, 1), (3, 1), (4, -1)),
    "backward": ((1, 1), (2, -1), (3, -1), (4, 1)),
    "left": ((1, -1), (2, -1), (3, -1), (4, -1)),
    "right": ((1, 1), (2, 1), (3, 1), (4, 1)),
    "rotate-left": ((1, -1), (2, 1), (3, -1), (4, 1)),
    "rotate-right": ((1, 1), (2, -1), (3, 1), (4, -1)),
}
# Current physical observation: wheel commands <=30 do not move this chassis.
# Use 35 as the normal command and bound travel with short durations instead.
MOTOR_SPEED = 35
TURN_SPEED = MOTOR_SPEED
TURN_PULSE_SECONDS = 0.12
# The physical chassis changes bearing by roughly a full several-degree chunk
# at the minimum usable wheel command.  Use a shorter pulse near centre and,
# critically, never chase a small overshoot by immediately reversing direction.
TURN_FINE_PULSE_SECONDS = 0.10
TURN_FINE_WINDOW_DEG = 10.0
# Predictive body-yaw pulse sizing. The first pulse uses a conservative physical
# rate prior; subsequent pulses update that rate from the observed bearing
# change. A hard 0.18 s cap keeps every prediction interruptible.
BODY_ALIGN_RATE_PRIOR_DEG_S = 65.0
BODY_ALIGN_RATE_MIN_DEG_S = 30.0
BODY_ALIGN_RATE_MAX_DEG_S = 140.0
BODY_ALIGN_RATE_ALPHA = 0.55
BODY_ALIGN_MAX_PULSE_SECONDS = 0.18
BODY_BEARING_CROSS_ACCEPT_DEG = 8.0
# Seed-12/13 replay after the one-direction anti-ping-pong fix showed clean
# monotonic convergence from ~25 deg, but the slow fine-turn regime needs up to
# 14 physical pulses before the next stopped camera sample enters the 1.5-deg
# deadband. Keep the loop bounded while allowing that measured convergence.
BODY_ALIGN_MAX_PULSES = 15
# Chassis yaw near the camera can throw a bottom-edge floor block completely out
# of view. Back straight away first, with camera pan frozen, until there is
# enough visual margin to rotate.
BODY_ALIGN_TOO_CLOSE_NY = 0.78
BODY_ALIGN_SAFE_NY = 0.68
CLOSE_NORMALIZE_SAFE_NY = 0.30
BODY_ALIGN_RETREAT_SECONDS = 0.14
BODY_ALIGN_RETREAT_MAX_PULSES = 8
BODY_ALIGN_RETREAT_MIN_PROGRESS_NY = 0.008
FAR_FORWARD_SPEED = MOTOR_SPEED
NEAR_FORWARD_SPEED = MOTOR_SPEED
FAR_FORWARD_SECONDS = 0.28
MID_FORWARD_SECONDS = 0.20
NEAR_FORWARD_SECONDS = 0.14
# Predictive pulse sizing from the mined motion model
# (outputs/motion_gain_table.json): forward gain ~15 cm/s at MOTOR_SPEED,
# linear across 0.2-0.28 s.  Far-band pulses are sized from the live
# remaining distance instead of a fixed 0.28 s, floored at the proven value
# and capped so one blind pulse never crosses the whole workspace.  Every
# pulse is still followed by the same stopped verification, so a wrong model
# only costs one pulse, and overshoot recovers via the backward branch.
PREDICTIVE_FORWARD_CM_S = 15.0
# DISABLED AGAIN (2026-09-04): re-enabled with the truth-anchored estimator,
# failed live the same night.  A 0.60 s pulse read 87.28 -> 66.38cm and the
# gate refused; a privileged-motion sweep then showed why: SIM wheel gain is
# wild and state-dependent (0.28 s moves 3.6-21cm, 0.60 s moves 72-79cm,
# standstill vs rolling, V2_STRUCTURAL_UNCALIBRATED).  Duration-based sizing
# is invalid no matter how good the estimator is.  Floor == cap restores the
# proven fixed 0.28 s.
PREDICTIVE_FAR_PULSE_CAP_S = FAR_FORWARD_SECONDS
# Far from the block, stopping after every 0.28 s pulse dominates wall time.
# Allow one bounded forward streaming burst while continuously observing the
# target. The stream is disabled well before face alignment / final capture, so
# the existing stopped precision controller remains authoritative near contact.
COARSE_STREAM_ENTER_RADIUS_CM = 38.0
COARSE_STREAM_EXIT_RADIUS_CM = 36.0
COARSE_STREAM_TARGET_MARGIN_CM = 8.0
COARSE_STREAM_MAX_SECONDS = 0.45
COARSE_STREAM_MAX_FRAMES = 10
# Physical REAL reads at its native camera cadence. The deterministic SIM fast
# adapter may intentionally evaluate the same visual stop gates less often
# while the far-field motor command remains active.
COARSE_STREAM_FRAME_DELAY_SECONDS = 0.0
COARSE_STREAM_MAX_X_ERROR = 0.16
COARSE_STREAM_MISSING_LIMIT = 2
COARSE_STREAM_INVALID_RANGE_LIMIT = 2
COARSE_STREAM_RECENTER_CONFIRMATIONS = 1
# Receding-horizon guard. While the far-approach motors remain on, estimate a
# very short future range from the latest visual motion. The next forward
# continuation is effectively pre-authorized only while observations stay
# inside this envelope; any disagreement or predicted boundary crossing stops
# the motors before another control cycle. Near the block this path is disabled.
PREDICTIVE_HORIZON_SECONDS = 0.18
PREDICTIVE_SEED_CLOSING_RATE_CM_S = 18.0
PREDICTIVE_RATE_ALPHA = 0.55
PREDICTIVE_MAX_CLOSING_RATE_CM_S = 30.0
PREDICTIVE_MIN_CLOSING_RATE_CM_S = -8.0
PREDICTIVE_RANGE_ERROR_CM = 1.8
PREDICTIVE_STOP_GUARD_CM = 1.0
VISUAL_FALLBACK_SECONDS = 0.16
VISUAL_FALLBACK_MAX_STEPS = 12
VISUAL_FAR_MAX_NY = 0.62
VISUAL_FAR_MAX_AREA = 7000
VISUAL_FAR_MAX_HEIGHT = 100
VISUAL_BLIND_MIN_HEIGHT = 80
VISUAL_BLIND_MIN_NY = 0.57
BLIND_LATCH_MAX_RADIUS_CM = 18.7
FORWARD_PULSE_TRAVEL_EST_CM = 1.2
# Range change must be judged against the pulse that just ran.  A fixed 5 cm
# limit rejected a real 0.28 s far pulse that measured 5.30 cm.  The dynamic
# limit remains tighter for 0.20/0.14 s pulses near the block.
RANGE_DROP_LIMIT_CM_PER_SECOND = 24.0
RANGE_DROP_MARGIN_CM = 0.8
RANGE_INCREASE_LIMIT_CM = 1.5
# Backward coarse staging is the mirror safety case: range should increase by a
# bounded amount. This also repairs the old `radius <= target => success` bug.
RANGE_RISE_LIMIT_CM_PER_SECOND = 24.0
RANGE_RISE_MARGIN_CM = 0.8
# Gaze recenters pan after every staging pulse by design (motion_pulse_with_gaze
# ends in centre_gaze), so consecutive range samples look from different
# viewpoints and the apparent cube breathes even with zero radial motion.
# The plausibility gate models translation only, so it gets a viewpoint term:
# ~0.5% of radius per pan-degree, capped.  Precedent: the 24 cm/s drop limit
# itself was loosened after it rejected a real 0.28 s far pulse.
VIEWPOINT_PAN_SLOPE_PER_DEG = 0.005
# Tilt moves the ray pitch itself, so far floor triangulation breathes much
# harder per tilt-degree (~2%/deg at 60-90cm) than per pan-degree.  Same
# treatment, steeper slope, shared cap.
VIEWPOINT_TILT_SLOPE_PER_DEG = 0.02
VIEWPOINT_EXTRA_CAP_CM = 6.0
COARSE_RADIUS_TOLERANCE_CM = 1.0
BACKWARD_FAR_SECONDS = 0.20
BACKWARD_NEAR_SECONDS = 0.10
BACKWARD_FINE_SECONDS = 0.06

# Geometry and grasp targets.
CAMERA_LINK_CM = 7.0
CAMERA_Z_OFFSET_CM = 2.5
CAMERA_PITCH_OFFSET_DEG = 0.0
CAMERA_VFOV_DEG = 48.0
CAMERA_HFOV_DEG = math.degrees(
    2.0 * math.atan(math.tan(math.radians(CAMERA_VFOV_DEG / 2.0)) * 4.0 / 3.0)
)
BLOCK_HALF_DEPTH_CM = 1.5
# Apparent-size ranging (far complement to the floor ray).  The cubes are a
# known 3 cm, so pinhole angular size gives range independent of the grazing
# floor geometry that fails past ~70 cm.  Conservative by construction: the
# LARGER pixel extent (which may include a visible top face) maps to the
# SMALLER distance, so errors land on the cautious side.  Edge-truncated and
# tiny blobs are rejected (quantization + cut-off corrupt the size).
SIZE_BLOCK_CM = BLOCK_HALF_DEPTH_CM * 2.0
SIZE_MIN_PX = 4
SIZE_EDGE_MARGIN_NX = 0.08
SIZE_BOTTOM_TRUNC_NY = 0.98
SIZE_RANGE_MIN_CM = 25.0
SIZE_RANGE_MAX_CM = 300.0
# Floor/size fusion: floor triangulation answers at or below this radius;
# above it only apparent size answers (floor stays as degraded fallback when
# size is unavailable).  Far floor noise (~18cm/deg at 1.1m) reads 54cm at
# 67cm truth and flickers around any higher cap, flipping instruments
# mid-staging; 50cm keeps floor's validated near zone and gives far to size.
FLOOR_FAR_CUT_CM = 50.0
# Metric range is useful only for coarse staging.  Near the block, physical
# tests showed the eye-in-hand ray model can underestimate forward distance by
# multiple centimetres.  The final grasp therefore switches to a fixed-pose
# visual capture window and a calibrated MasterPi end-effector radius.
STOP_RADIUS_CM = 16.2
CAPTURE_STAGING_RADIUS_CM = 22.0
# The close 11-12 cm camera pose cannot see a target still sitting around
# 17-20 cm.  First image-servo the existing floor-search view until the cube
# enters the overlap between the far and close camera views, then switch.
PRECAPTURE_ARM_POSE = {3: 500, 4: 2320, 5: 1320}
# Hand-eye FK: a 15 cm block radius projects to far-view ny~=0.719 and,
# after switching to CAPTURE_ARM_POSE, close-view ny~=0.107.  That is a safe
# overlap: the target is already inside the close image but still well before
# the final TCP capture point (ny~=0.442 at r=12 cm).  The former 0.80 gate
# corresponded to ~13.7 cm and was unnecessarily late; a real run reached
# ny=0.760 with continuous visual progress and then failed only because the
# arbitrary 8-pulse budget expired.
PRECAPTURE_TARGET_NY = 0.72
# The far->close handoff is a visibility-overlap gate, not a grasp-depth gate.
# Camera contour jitter at this stage is a few pixels; accept a target already
# inside the proven overlap instead of issuing another forward pulse merely to
# cross an exact scalar boundary (a live SIM replay reached 0.717 vs 0.720).
PRECAPTURE_TARGET_TOLERANCE_NY = 0.01
PRECAPTURE_TOO_CLOSE_NY = 0.91
# Measured-camera randomized starts can enter this stage near ny~=0.39--0.40.
# Seeds 13/14 advanced monotonically and centered through 12 creeps but ended at
# 0.685/0.677, below the safe overlap lower edge 0.710. Two more bounded creeps
# are enough; every sample still enforces x drift, minimum progress, reverse
# progress and PRECAPTURE_TOO_CLOSE_NY before switching camera pose.
PRECAPTURE_MAX_CREEP_PULSES = 14
# Coarse-to-fine image servoing.  The live 2026-08-31 trajectory needed eight
# identical 60 ms pulses to move ny=0.423 -> 0.742.  Far from the handoff
# boundary there is >0.15 normalized-image margin, so use a bounded longer
# pulse and taper back to the already-proven 60 ms pulse near the boundary.
PRECAPTURE_FAST_MAX_NY = 0.55
PRECAPTURE_MID_MAX_NY = 0.65
PRECAPTURE_FAST_CREEP_SECONDS = 0.10
PRECAPTURE_MID_CREEP_SECONDS = 0.08
PRECAPTURE_NEAR_CREEP_SECONDS = 0.06
CAPTURE_ARM_POSE = {3: 600, 4: 2200, 5: 1900}
# Final capture is a hand-eye target, NOT image-centre tracking.  The camera and
# gripper are different physical points on the arm.  The fixed close-view image
# stop remains tied to the *observed* legacy visual window around block-radius
# 12 cm: both recent physical runs traversed this window cleanly and reached
# ny~=0.43..0.50.  Do not silently move the chassis stop just because the jaw
# reach is being corrected.
#
# IMPORTANT physical evidence (2026-08-30): two separate real attempts reached
# the close visual window, then both descended with the same 12.5 cm fingertip
# radius and left the red cube untouched in 9/9 post-grasp frames.  The user also
# observed that the jaws closed behind the cube.  A subsequent real attempt then
# executed the corrected 15.0 cm fingertip reach and still produced a clean 9/9
# visual miss, with the user again observing insufficient forward reach.
#
# The later +5 cm / 17.5 cm choice was only a bounded *provisional physical
# probe*.  Its supporting V2 sweep used the older nominal camera model and is
# no longer valid evidence after the measured ugrp1 fisheye + rigid hand-eye
# profile was installed.  A 2026-08-30 live-state diagnostic with that measured
# camera found bilateral/lift only around 15.5..16.5 cm and failure again at
# 17.0/17.5 cm.  Do not tune REAL reach from SIM alone: keep this value until
# another physical trial or metrology supersedes it, and keep the twin marked
# unvalidated for grasp/contact dynamics.
CAPTURE_TARGET_NX = 0.50
CAPTURE_TARGET_NY = 0.442
CAPTURE_TARGET_TOLERANCE_NY = 0.025
CAPTURE_RETRY_TARGET_NY = 0.49  # legacy API only; active pick does not auto-creep on a miss
CAPTURE_TOO_CLOSE_NY = 0.505
CAPTURE_X_DEADBAND = 0.07
# Do not spend a three-frame pan confirmation at every loop head when the
# existing three-frame sample already proves x is close.  Corrections start well
# inside the hard 0.07 drift gate and are followed by a fresh sample.
CAPTURE_X_ALIGN_TRIGGER = 0.025
CAPTURE_CREEP_SECONDS = 0.06
CAPTURE_RETREAT_SECONDS = 0.06
CAPTURE_MAX_RETREAT_PULSES = 5
# Minimum effective drive pulses move ~0.02+ ny while the capture window is
# only 0.05 wide, so bang-bang control cannot reliably land inside by motion
# alone: a 0.06 s retreat moved 0.408 -> 0.342 straight past the band (SIM).
# An overshoot this small is physically irrelevant for the grasp (pick
# re-measures everything; collision limit is 0.91, face-geometry limit 0.47
# for the 0.30 face target) — accept it as ready instead of retreating into
# a ping-pong the actuator cannot win.
CAPTURE_WINDOW_OVERSHOOT_EPS_NY = 0.0
# The measured ugrp1 camera can enter the close view around ny~=0.13 after the
# far->close handoff.  A replayed monotonic, centered trajectory reached only
# ny=0.406 after six 60 ms creeps, just below the safe window lower edge 0.417;
# one additional bounded creep reaches the window. Every sample still checks
# X drift, reverse progress, and CAPTURE_TOO_CLOSE_NY before accepting descent.
CAPTURE_MAX_CREEP_PULSES = 7
CAPTURE_MIN_PROGRESS_NY = 0.008
# Final close-camera creep may use a shorter actuator pulse than pre-capture.
# Keep a separate progress threshold so shortening that pulse does not make a
# healthy, monotonic final approach fail the longer-pulse REAL watchdog. REAL
# defaults to the same threshold; digital twins may calibrate it independently.
CAPTURE_CREEP_MIN_PROGRESS_NY = CAPTURE_MIN_PROGRESS_NY
# REAL keeps the historical per-pulse watchdog: zero low-progress pulses are
# tolerated. A digital twin with a shorter, quantized pulse may override this
# to one and must then demonstrate the same minimum progress cumulatively on
# the immediately following pulse.
CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES = 0
CAPTURE_VISUAL_BLOCK_RADIUS_CM = 12.0
CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM = 5.0
CAPTURE_BLOCK_RADIUS_CM = (
    CAPTURE_VISUAL_BLOCK_RADIUS_CM + CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM
)
CAPTURE_FINGERTIP_RADIUS_CM = CAPTURE_BLOCK_RADIUS_CM + 0.5
# Face-normal relocation happens while the far/search camera still sees the
# complete cube.  Stop the chassis at a corrected 20 cm radius: that keeps the
# raw floor-ray estimate around 15 cm (outside CHASSIS_HARD_LIMIT_CM), leaves
# several centimetres for the proven far->close visual handoff, and still makes
# a useful lateral viewpoint change.  The close straight approach changes the
# target bearing even though chassis yaw stays fixed, so entering it at the old
# 15-20 deg edge was unsafe. Keep 14 deg as the acceptance gate, but target a
# stronger 6 deg residual when a side correction is actually required.  The
# camera face-normal estimate on the failed seed was ~6 deg conservative versus
# SIM truth, so a 14-deg movement target produced almost pure forward motion and
# never created the lateral margin it was meant to create.  The tighter movement
# target leaves room for that observation bias plus the measured 5.1-deg close
# straight bearing drift, while the separate 14-deg gate remains the authority.
FACE_ROUTE_TARGET_RADIUS_CM = 20.0
FACE_ROUTE_TARGET_ERROR_DEG = 6.0
FACE_ROUTE_BEARING_TOLERANCE_DEG = 2.0
FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG = 14.0
# Do not plan exactly on servo 6's hard corridor edge. REAL trace
# 20260901T065618Z produced an ideal -41.8deg target even though -36deg would
# still leave only ~11.8deg face error. Keep 2deg of pan/FOV margin and project
# a geometrically valid route into this usable corridor instead of aborting.
FACE_ROUTE_ARM_BEARING_MARGIN_DEG = 2.0
FACE_ROUTE_TRAVEL_BEARING_TOLERANCE_DEG = 4.0
# Side relocation is a 2-D endpoint, not a radial-only stop.  The temporary
# travel-frame target bearing must also reach the bearing implied by the planned
# endpoint before the chassis may restore yaw.  Seed 2101568344 reached the
# radial window at +55deg while the planned endpoint was still near +63deg,
# leaving roughly 2.3 cm of side translation undone.
FACE_ROUTE_ENDPOINT_BEARING_TOLERANCE_DEG = 4.0
FACE_ROUTE_RANGE_TOLERANCE_CM = 0.9
# Keep the corrected range >=19 cm (raw floor-ray ~=14 cm), i.e. at/above the
# existing chassis hard-limit boundary even while finishing the lateral chord.
FACE_ROUTE_MAX_RANGE_OVERSHOOT_CM = 1.0
# A long diagonal chord can cut inside the final target radius before its
# lateral coordinate is complete. Keep rotate+straight motion, but split such
# routes into short same-radius chords and a final radial leg. This avoids pure
# mecanum strafe while preserving the independent 19 cm fail-closed envelope.
FACE_ROUTE_MAX_SIDE_LEG_DEG = 24.0
FACE_ROUTE_DIRECT_CHORD_EPSILON_CM = 0.05
FACE_ROUTE_MAX_TURN_PULSES = 12
FACE_ROUTE_MAX_FORWARD_PULSES = 16
FACE_ROUTE_FORWARD_SECONDS = 0.06
FACE_ROUTE_MIN_IMPROVEMENT_DEG = 3.0
# FK z is axle-relative, while the block is on the floor roughly one 65-mm
# wheel radius below the axle.  The former 0.0-cm target therefore put the
# finger centre near 3 cm above the floor and pinched the 30-mm cube close to
# its top edge.  Lower the capture TCP by 6 mm.  At the current provisional
# 17.5-cm REAL reach this still keeps servo 5 inside the calibrated pulse range
# (about 2485 rather than riding the 2500 hard limit).
CAPTURE_GRASP_HEIGHT_CM = -0.60
GRASP_DESCENT_HEIGHTS_CM = (4.0, 3.0, 2.0, 1.0, 0.4)
# The metric controller no longer owns the final capture distance.  Keep a
# modest tolerance for legacy callers; active pick approach stops at the coarse
# staging radius above and hands final distance to visual_capture_approach().
FINAL_APPROACH_TOLERANCE_CM = 0.8
FINAL_CREEP_WINDOW_CM = 2.0
FINAL_CREEP_SECONDS = 0.07
CHASSIS_HARD_LIMIT_CM = 14.0
GRIPPER_TIP_PAST_CENTER_CM = 0.5
CALIBRATED_FINGERTIP_RADIUS_MIN_CM = 10.5
CALIBRATED_FINGERTIP_RADIUS_MAX_CM = 18.0
HOVER_HEIGHT_CM = 8.0
DEFAULT_BLOCK_HEIGHT_CM = 3.0
# This manipulation task uses the known ~3 cm MasterPi cube. Vision-derived
# apparent height can roughly double near the camera (latest physical run:
# 6.04 cm), which previously lifted the grasp centre to 1.86 cm. Use the known
# physical cube height for vertical grasp placement; keep measured height only
# for conservative hover clearance.
GRASP_REFERENCE_BLOCK_HEIGHT_CM = DEFAULT_BLOCK_HEIGHT_CM
GRASP_HEIGHT_RATIO = 0.35
GRASP_HEIGHT_OFFSET_CM = -0.25
GRASP_HEIGHT_MIN_CM = 0.60
GRASP_HEIGHT_MAX_CM = 4.00
HOVER_CLEARANCE_ABOVE_BLOCK_CM = 4.0
VISUAL_REFERENCE_HEIGHT_PX = 90.0
VISUAL_REFERENCE_BLOCK_HEIGHT_CM = 3.0
GRASP_PITCH_PREFERRED_DEG = -66
CAPTURE_GRASP_PITCH_DEG = -90
HAND_EYE_YAW_OFFSET_PULSE = 0
HAND_EYE_LATERAL_OFFSET_CM = 0.0


class State(Enum):
    BASIC_POSE = auto()
    SCAN = auto()
    LOCK_GAZE = auto()
    ALIGN_BODY = auto()
    APPROACH_TO_FACE = auto()
    ALIGN_FACE = auto()
    FINAL_APPROACH = auto()
    STOP_AND_MEASURE = auto()
    CALCULATE_ARM = auto()
    GRASP = auto()
    VERIFY_GRASP = auto()
    COMPLETE = auto()


class LockedTargetLost(RuntimeError):
    """The locked block is no longer visible after a stopped recovery scan."""


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
    source: str = "floor_ray"


@dataclass(frozen=True)
class PickPlan:
    block: BlockEstimate
    base_pulse: int
    hover_pose: dict[int, int]
    grasp_pose: dict[int, int]
    fingertip_radius_cm: float


@dataclass(frozen=True)
class ApproachResult:
    gaze: Gaze
    block: BlockEstimate
    target_still_visible: bool


@dataclass(frozen=True)
class StreamingApproachResult:
    gaze: Gaze
    duration_s: float
    frames: int
    last_estimate: BlockEstimate | None
    stop_reason: str


@dataclass(frozen=True)
class FaceAlignmentEstimate:
    edge_angle_deg: float
    error_deg: float
    edge_length_cm: float


@dataclass(frozen=True)
class FaceNormalRoutePlan:
    """Actor-visible side-placement plan in the chassis frame.

    ``travel_bearing_deg`` is the temporary chassis heading, relative to the
    heading at which the face was measured. ``target_bearing_deg`` is where the
    cube should appear after the chassis returns from that temporary travel ray.
    """
    current_radius_cm: float
    current_bearing_deg: float
    face_normal_bearing_deg: float
    target_radius_cm: float
    target_bearing_deg: float
    travel_bearing_deg: float
    travel_distance_cm: float
    travel_target_bearing_deg: float


@dataclass(frozen=True)
class RedSummary:
    hits: int
    total: int
    nx: float | None
    ny: float | None
    area: float | None
    last_blob: object | None
    last_frame: object | None


@dataclass(frozen=True)
class VerificationReference:
    pose: dict[int, int]
    summary: RedSummary


class LiveVideo:
    """Parse uStreamer MJPEG directly with bounded transparent reconnects.

    The Pi's uStreamer can occasionally end one HTTP stream while `/snapshot`
    and the service itself remain healthy.  The old implementation converted a
    single EOF into a permanent camera failure for the whole manipulation.
    Re-open only the MJPEG transport, keep frame sequence monotonic, clear the
    partial JPEG buffer, and give up after a small bounded number of consecutive
    reconnect failures.
    """

    def __init__(
        self,
        url: str = STREAM_URL,
        *,
        opener=None,
        max_reconnects: int = VIDEO_MAX_RECONNECTS,
        reconnect_delay: float = VIDEO_RECONNECT_DELAY_SECONDS,
    ):
        self._url = url
        self._opener = opener or urlopen
        self._max_reconnects = max(0, int(max_reconnects))
        self._reconnect_delay = max(0.0, float(reconnect_delay))
        self._condition = threading.Condition()
        self._frame = None
        self._sequence = 0
        self._delivered = 0
        self._closed = False
        self._error: str | None = None
        self._buffer = bytearray()
        self._response = self._open_initial_response()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _open_response(self):
        try:
            return self._opener(self._url, timeout=3.0)
        except Exception as error:
            raise RuntimeError(
                f"could not open MJPEG stream: {self._url}: {error}"
            ) from error

    def _open_initial_response(self):
        """Tolerate a short camera-service restart before the reader exists.

        A REAL skill may start in the narrow interval after uStreamer received
        SIGTERM but before systemd has rebound port 8080. Treat that interval as
        transport recovery, not as a permanent robot-unavailable failure.
        """
        last_error = None
        for attempt in range(self._max_reconnects + 1):
            try:
                return self._open_response()
            except Exception as error:
                last_error = error
                if attempt >= self._max_reconnects:
                    break
                if self._reconnect_delay:
                    time.sleep(self._reconnect_delay)
        raise RuntimeError(
            f"MJPEG initial open failed after {self._max_reconnects} retries: {last_error}"
        ) from last_error

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    def _decode_buffered_frames(self) -> int:
        decoded = 0
        while not self._closed:
            start = self._buffer.find(b"\xff\xd8")
            if start < 0:
                if len(self._buffer) > 2:
                    del self._buffer[:-2]
                break
            end = self._buffer.find(b"\xff\xd9", start + 2)
            if end < 0:
                if start > 0:
                    del self._buffer[:start]
                if len(self._buffer) > 2_000_000:
                    raise RuntimeError("oversized incomplete MJPEG frame")
                break
            jpeg = bytes(self._buffer[start : end + 2])
            del self._buffer[: end + 2]
            frame = cv2.imdecode(
                np.frombuffer(jpeg, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None:
                continue
            with self._condition:
                self._frame = frame
                self._sequence += 1
                self._condition.notify_all()
            decoded += 1
        return decoded

    def _reader(self) -> None:
        reconnects = 0
        while not self._closed:
            try:
                chunk = self._response.read(4096)
                if not chunk:
                    raise RuntimeError("MJPEG HTTP stream ended")
                self._buffer.extend(chunk)
                if self._decode_buffered_frames() > 0:
                    reconnects = 0
                continue
            except Exception as error:
                if self._closed:
                    return
                try:
                    self._response.close()
                except Exception:
                    pass
                self._buffer.clear()
                reconnects += 1
                if reconnects > self._max_reconnects:
                    with self._condition:
                        self._error = (
                            f"MJPEG reader failed after {self._max_reconnects} "
                            f"reconnects: {error}"
                        )
                        self._condition.notify_all()
                    return
                print(
                    f"MJPEG stream interrupted ({error}); reconnect "
                    f"{reconnects}/{self._max_reconnects}",
                    flush=True,
                )
                if self._reconnect_delay:
                    time.sleep(self._reconnect_delay)
                try:
                    self._response = self._open_response()
                except Exception as reopen_error:
                    # Keep the retry budget consecutive. The next loop iteration
                    # attempts another open without pretending a frame arrived.
                    if reconnects >= self._max_reconnects:
                        with self._condition:
                            self._error = (
                                f"MJPEG reader failed after {self._max_reconnects} "
                                f"reconnects: {reopen_error}"
                            )
                            self._condition.notify_all()
                        return
                    # A tiny empty response object is not fabricated; retry the
                    # opener directly here so `_response.read` is never called on
                    # an invalid transport.
                    while not self._closed and reconnects < self._max_reconnects:
                        reconnects += 1
                        if self._reconnect_delay:
                            time.sleep(self._reconnect_delay)
                        try:
                            self._response = self._open_response()
                            break
                        except Exception as again:
                            reopen_error = again
                    else:
                        if not self._closed:
                            with self._condition:
                                self._error = (
                                    f"MJPEG reader failed after {self._max_reconnects} "
                                    f"reconnects: {reopen_error}"
                                )
                                self._condition.notify_all()
                        return

    def read(self, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._sequence <= self._delivered and self._error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out waiting for a fresh frame")
                self._condition.wait(remaining)
            if self._error is not None:
                raise RuntimeError(self._error)
            frame = self._frame
            self._delivered = self._sequence
        if frame is None:
            raise RuntimeError("camera has no decoded frame")
        record_camera_frame(frame, source=self._url)
        return frame

    def settle(self, seconds: float = CAMERA_DELAY_SECONDS):
        time.sleep(max(0.0, seconds))
        return self.read()

    def close(self) -> None:
        self._closed = True
        try:
            self._response.close()
        except Exception:
            pass
        with self._condition:
            self._condition.notify_all()
        self._thread.join(timeout=1.0)


class TargetLock:
    """Accept only detections consistent with the first confirmed red block."""

    def __init__(self, seed) -> None:
        self.last = seed
        self.misses = 0
        self._ego_motion_frames = 0
        self._ego_allowance = 0.0

    def note_ego_motion(self, frames: int = 3, allowance: float = 0.0) -> None:
        # A commanded chassis pulse can legitimately move the same block far in
        # image space. Relax continuity only for a few immediately-following
        # frames and simultaneously tighten scale/shape similarity.
        # A commanded ARM pose change reprojects even more (SIM: ny jumped
        # 0.623 on precapture entry with identical size) while size/shape
        # persist, so arm moves may pass a wider position allowance; the
        # ratio/shape gates below stay tight either way.
        self._ego_motion_frames = max(self._ego_motion_frames, int(frames))
        self._ego_allowance = max(float(self._ego_allowance), float(allowance))

    def select(self, candidate):
        motion_window = self._ego_motion_frames > 0
        if self._ego_motion_frames > 0:
            self._ego_motion_frames -= 1
        else:
            self._ego_allowance = 0.0
        if candidate is None:
            self.misses += 1
            return None
        jump = math.hypot(candidate.nx - self.last.nx, candidate.ny - self.last.ny)
        ratio = candidate.area / max(1.0, float(self.last.area))
        allowance = 0.26 + min(0.14, self.misses * 0.025)
        ratio_low, ratio_high = 0.22, 4.5
        shape_ok = True
        if motion_window:
            allowance = max(allowance, 0.38, self._ego_allowance)
            ratio_low, ratio_high = 0.45, 2.2
            for attr in ("width", "height"):
                old = float(getattr(self.last, attr, 0.0) or 0.0)
                new = float(getattr(candidate, attr, 0.0) or 0.0)
                if old > 0.0 and new > 0.0 and not 0.55 <= new / old <= 1.8:
                    shape_ok = False
        if jump <= allowance and ratio_low <= ratio <= ratio_high and shape_ok:
            self.last = candidate
            self.misses = 0
            return candidate
        self.misses += 1
        print(
            f"target-switch rejected jump={jump:.3f} area-ratio={ratio:.2f} "
            f"last=({float(self.last.nx):.3f},{float(self.last.ny):.3f},a={float(self.last.area):.0f}) "
            f"candidate=({float(candidate.nx):.3f},{float(candidate.ny):.3f},a={float(candidate.area):.0f})",
            flush=True,
        )
        return None


def state(name: State, detail: str) -> None:
    print(f"\n[STATE {name.value}/{len(State)}] {name.name}: {detail}", flush=True)


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def step_toward(current: int, target: int, step: int) -> int:
    return current + max(-step, min(step, target - current))


def median_absolute_deviation(values: list[float]) -> float:
    centre = statistics.median(values)
    return statistics.median(abs(value - centre) for value in values)


def allowed_range_drop_cm(motion_duration: float) -> float:
    """Maximum credible distance decrease after one stopped forward pulse."""
    return (
        max(0.0, motion_duration) * RANGE_DROP_LIMIT_CM_PER_SECOND
        + RANGE_DROP_MARGIN_CM
    )


def allowed_range_rise_cm(motion_duration: float) -> float:
    """Maximum credible distance increase after one stopped backward pulse."""
    return (
        max(0.0, motion_duration) * RANGE_RISE_LIMIT_CM_PER_SECOND
        + RANGE_RISE_MARGIN_CM
    )


def nominal_pulse(pose: dict[int, int], servo: int) -> float:
    return float(pose[servo] - SERVO_DEVIATION.get(servo, 0))


def forward_kinematics(pose: dict[int, int], tool_length_cm: float) -> ArmPoint:
    theta3 = (nominal_pulse(pose, 3) - 1500.0) / PULSE_PER_DEGREE
    theta4 = (nominal_pulse(pose, 4) - 1500.0) / PULSE_PER_DEGREE
    theta5 = 90.0 - (nominal_pulse(pose, 5) - 1500.0) / PULSE_PER_DEGREE
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
    return ArmPoint(radius, height, pitch)


def inverse_kinematics_at_pitch(
    radius_cm: float,
    height_cm: float,
    pitch_deg: float,
) -> dict[int, int] | None:
    alpha = math.radians(pitch_deg)
    horizontal = radius_cm - GRIPPER_LINK * math.cos(alpha)
    vertical = height_cm - LINK_1 - GRIPPER_LINK * math.sin(alpha)
    diagonal = math.hypot(horizontal, vertical)
    if not abs(LINK_2 - LINK_3) <= diagonal <= LINK_2 + LINK_3:
        return None
    elbow_cos = (LINK_2**2 + LINK_3**2 - diagonal**2) / (
        2.0 * LINK_2 * LINK_3
    )
    shoulder_cos = (diagonal**2 + LINK_2**2 - LINK_3**2) / (
        2.0 * LINK_2 * diagonal
    )
    elbow_cos = max(-1.0, min(1.0, elbow_cos))
    shoulder_cos = max(-1.0, min(1.0, shoulder_cos))
    theta4 = 180.0 - math.degrees(math.acos(elbow_cos))
    line_cos = max(-1.0, min(1.0, horizontal / diagonal))
    line_angle = math.acos(line_cos)
    theta5 = math.degrees(
        (-1.0 if vertical < 0.0 else 1.0) * line_angle
        + math.acos(shoulder_cos)
    )
    theta3 = pitch_deg - theta5 + theta4
    pose = {
        3: int(round(theta3 * PULSE_PER_DEGREE + 1500)) + SERVO_DEVIATION[3],
        4: int(round(theta4 * PULSE_PER_DEGREE + 1500)) + SERVO_DEVIATION[4],
        5: int(round(1500 + (90.0 - theta5) * PULSE_PER_DEGREE))
        + SERVO_DEVIATION[5],
    }
    # Match Hiwonder ArmIK's physical PWM envelope exactly.  The previous
    # 550..2450 clamp incorrectly removed valid near-vertical grasp solutions.
    if any(not 500 <= pulse <= 2500 for pulse in pose.values()):
        return None
    return pose


def solve_ik(
    radius_cm: float,
    height_cm: float,
    preferred_pitch_deg: float,
) -> dict[int, int]:
    candidates: list[tuple[float, dict[int, int]]] = []
    # Hiwonder ArmIK searches all the way to -90 degrees.  Excluding
    # -90 forced the gripper to approach the cube diagonally and physically
    # pushed it away during grasp.
    for pitch in range(-90, -39):
        pose = inverse_kinematics_at_pitch(radius_cm, height_cm, float(pitch))
        if pose is None:
            continue
        # The calibrated r=16.5,z=2 pose uses servo 5 at 2406.  That is still
        # inside the verified 500..2500 pulse range, so do not bias the solver
        # away from the requested wrist pitch merely for crossing 2350.
        score = abs(pitch - preferred_pitch_deg)
        candidates.append((score, pose))
    if not candidates:
        raise RuntimeError(
            f"IK has no safe solution for radius={radius_cm:.2f}, "
            f"height={height_cm:.2f}"
        )
    return min(candidates, key=lambda item: item[0])[1]


def solve_hover_ik(
    radius_cm: float,
    desired_height_cm: float,
) -> tuple[dict[int, int], float]:
    """Use the highest reachable hover, never lower than the proven 8 cm."""
    height = desired_height_cm
    while height >= HOVER_HEIGHT_CM - 1e-6:
        try:
            return solve_ik(
                radius_cm,
                height,
                preferred_pitch_deg=-55,
            ), height
        except RuntimeError:
            height -= 0.25
    raise RuntimeError(
        f"no safe hover IK from {desired_height_cm:.2f}cm down to "
        f"{HOVER_HEIGHT_CM:.2f}cm"
    )


def capture_target_pixel_from_hand_eye(
    pose: dict[int, int] | None = None,
    *,
    block_radius_cm: float = CAPTURE_VISUAL_BLOCK_RADIUS_CM,
    block_center_height_cm: float = DEFAULT_BLOCK_HEIGHT_CM / 2.0,
    target_nx: float = CAPTURE_TARGET_NX,
) -> tuple[float, float]:
    """Project the desired gripper capture point into the fixed camera image.

    This function makes the camera->TCP distinction explicit.  The returned
    pixel is where the block must appear for the *gripper* plan, not where the
    camera should merely look.  ``target_nx`` is kept as an explicit hand-eye
    calibration degree of freedom even though the current nominal mount model
    has no measured lateral offset yet.
    """
    camera_pose = dict(CAPTURE_ARM_POSE if pose is None else pose)
    camera_pose.setdefault(6, BASE_CENTER)
    camera = forward_kinematics(camera_pose, CAMERA_LINK_CM)
    camera_height = camera.height_cm + CAMERA_Z_OFFSET_CM
    forward_delta = float(block_radius_cm) - float(camera.radius_cm)
    vertical_delta = float(block_center_height_cm) - float(camera_height)
    if forward_delta <= 0.05 or vertical_delta >= -0.05:
        raise RuntimeError(
            "capture hand-eye target is not in front of/below the fixed camera"
        )
    ray_pitch = math.degrees(math.atan2(vertical_delta, forward_delta))
    pixel_down_deg = camera.pitch_deg + CAMERA_PITCH_OFFSET_DEG - ray_pitch
    ny = 0.5 + pixel_down_deg / CAMERA_VFOV_DEG
    nx = float(target_nx)
    if not 0.05 <= nx <= 0.95:
        raise RuntimeError(f"capture hand-eye projection left image: nx={nx:.3f}")
    if not 0.05 <= ny <= 0.95:
        raise RuntimeError(f"capture hand-eye projection left image: ny={ny:.3f}")
    return nx, float(ny)


def capture_target_ny_from_hand_eye(
    pose: dict[int, int] | None = None,
    *,
    block_radius_cm: float = CAPTURE_VISUAL_BLOCK_RADIUS_CM,
    block_center_height_cm: float = DEFAULT_BLOCK_HEIGHT_CM / 2.0,
) -> float:
    """Compatibility helper returning the vertical part of the TCP target."""
    _nx, ny = capture_target_pixel_from_hand_eye(
        pose,
        block_radius_cm=block_radius_cm,
        block_center_height_cm=block_center_height_cm,
    )
    return ny


def _blob_pixels(blob) -> tuple[float, float, float, float] | None:
    """Absolute pixel geometry (cx, cy, w, h) with normalized fallback."""
    try:
        w_px = float(getattr(blob, "width", 0.0) or 0.0)
        h_px = float(getattr(blob, "height", 0.0) or 0.0)
        nx = float(blob.nx)
        ny = float(blob.ny)
    except (TypeError, ValueError):
        return None
    cx_px = getattr(blob, "cx", None)
    cy_px = getattr(blob, "cy", None)
    try:
        cx = float(cx_px) if cx_px is not None else nx * FRAME_WIDTH
        cy = float(cy_px) if cy_px is not None else ny * FRAME_HEIGHT
    except (TypeError, ValueError):
        return None
    return cx, cy, w_px, h_px


def _blob_bottom_truncated(blob) -> bool:
    """True when the blob touches the frame bottom (size reads far)."""
    try:
        bottom_ny = float(blob.ny) + float(blob.height) / (2.0 * FRAME_HEIGHT)
    except (TypeError, ValueError):
        return True
    return bottom_ny > SIZE_BOTTOM_TRUNC_NY


def _undistorted_angle(a: tuple[float, float], b: tuple[float, float]) -> float:
    dot = a[0] * b[0] + a[1] * b[1] + 1.0
    na = math.sqrt(a[0] * a[0] + a[1] * a[1] + 1.0)
    nb = math.sqrt(b[0] * b[0] + b[1] * b[1] + 1.0)
    if na <= 1e-12 or nb <= 1e-12:
        return 0.0
    return math.acos(max(-1.0, min(1.0, dot / (na * nb))))


def size_range_estimate(blob) -> float | None:
    """Fisheye-correct range from the known cube size. Frame-size independent.

    Blob edge bearings pass through the calibrated fisheye model instead of
    the old linear pixels*FOV pinhole map (which read ~25% near at far range
    on raw-fisheye frames).  Returns None for tiny/edge-truncated blobs or
    insane values; otherwise the conservative (near-side) distance in cm.
    """
    pix = _blob_pixels(blob)
    if pix is None:
        return None
    cx_px, cy_px, w_px, h_px = pix
    try:
        nx = float(blob.nx)
    except (TypeError, ValueError):
        return None
    if w_px < SIZE_MIN_PX or h_px < SIZE_MIN_PX:
        return None
    if nx < SIZE_EDGE_MARGIN_NX or nx > 1.0 - SIZE_EDGE_MARGIN_NX:
        return None
    if _blob_bottom_truncated(blob):
        return None
    half = SIZE_BLOCK_CM / 2.0
    ang_w = _undistorted_angle(
        undistorted_ray(cx_px - w_px / 2.0, cy_px),
        undistorted_ray(cx_px + w_px / 2.0, cy_px),
    )
    ang_h = _undistorted_angle(
        undistorted_ray(cx_px, cy_px - h_px / 2.0),
        undistorted_ray(cx_px, cy_px + h_px / 2.0),
    )
    if ang_w <= 0 or ang_h <= 0:
        return None
    try:
        dist = min(half / math.tan(ang_w / 2.0), half / math.tan(ang_h / 2.0))
    except (ValueError, ZeroDivisionError):
        return None
    if not SIZE_RANGE_MIN_CM <= dist <= SIZE_RANGE_MAX_CM:
        return None
    return dist


def estimate_block(
    pose: dict[int, int],
    blob,
    *,
    image_center_nx: float = 0.5,
) -> BlockEstimate | None:
    if any(servo not in pose for servo in (3, 4, 5, 6)):
        return None
    camera = forward_kinematics(pose, CAMERA_LINK_CM)
    camera_height = camera.height_cm + CAMERA_Z_OFFSET_CM
    bottom_ny = min(0.995, blob.ny + blob.height / (2.0 * FRAME_HEIGHT))
    pixel_down = (bottom_ny - 0.5) * CAMERA_VFOV_DEG
    ray_pitch = camera.pitch_deg + CAMERA_PITCH_OFFSET_DEG - pixel_down
    floor_radius = None
    forward_from_camera = 0.0
    if camera_height > 0.5 and -88.0 <= ray_pitch <= -5.0:
        forward_from_camera = camera_height / math.tan(math.radians(-ray_pitch))
        radius = camera.radius_cm + forward_from_camera + BLOCK_HALF_DEPTH_CM
        if 3.0 <= radius <= 70.0:
            floor_radius = radius
    if floor_radius is None:
        # Far complement: same yaw geometry, range from apparent size.
        # Cube-model fit first (perspective-correct); fronto-parallel
        # undistorted sizing stays as the degraded fallback.
        return _size_block_estimate(
            pose, blob, camera, camera_height, ray_pitch,
            image_center_nx=image_center_nx,
        )
    if floor_radius > FLOOR_FAR_CUT_CM:
        # Hard instrument cut, not a disagreement veto: far floor
        # triangulation amplifies pixel noise (~18cm per degree at 1.1m) and
        # flickers around the 70cm cap (observed 69.0cm floor vs 90.9cm truth
        # while size read 87.7cm; then 53.95cm floor vs 66.9cm truth).  Above
        # the cut only size answers; floor stays as the degraded fallback so
        # availability never regresses.
        sized = _size_block_estimate(
            pose, blob, camera, camera_height, ray_pitch,
            image_center_nx=image_center_nx,
        )
        if sized is not None:
            print(
                f"far floor {floor_radius:.1f}cm superseded by "
                f"size {sized.radius_cm:.1f}cm",
                flush=True,
            )
            return sized
    radius = floor_radius
    top_ny = max(0.005, blob.ny - blob.height / (2.0 * FRAME_HEIGHT))
    top_pixel_down = (top_ny - 0.5) * CAMERA_VFOV_DEG
    top_ray_pitch = (
        camera.pitch_deg + CAMERA_PITCH_OFFSET_DEG - top_pixel_down
    )
    top_ray_pitch = max(-88.0, min(88.0, top_ray_pitch))
    measured_height = camera_height + forward_from_camera * math.tan(
        math.radians(top_ray_pitch)
    )
    if not 0.4 <= measured_height <= 12.0:
        measured_height = DEFAULT_BLOCK_HEIGHT_CM
    block_height = max(0.8, min(10.0, measured_height))
    pixel_left_deg = (float(image_center_nx) - blob.nx) * CAMERA_HFOV_DEG
    servo_left_deg = (pose[6] - BASE_CENTER) / PULSE_PER_DEGREE
    yaw_left_deg = servo_left_deg + pixel_left_deg
    yaw = math.radians(yaw_left_deg)
    return BlockEstimate(
        radius_cm=radius,
        lateral_left_cm=radius * math.sin(yaw),
        forward_cm=radius * math.cos(yaw),
        yaw_left_deg=yaw_left_deg,
        camera_radius_cm=camera.radius_cm,
        camera_height_cm=camera_height,
        ray_pitch_deg=ray_pitch,
        block_height_cm=block_height,
        nx=blob.nx,
        ny=blob.ny,
    )


def _size_block_estimate(pose, blob, camera, camera_height, ray_pitch, *,
                         image_center_nx: float = 0.5):
    """Far-range BlockEstimate from apparent cube size.

    Same yaw/lateral decomposition as the floor ray; only the radius comes
    from size.  Returns None when the blob is unsuitable for sizing, so
    callers keep their existing fail-closed behaviour.
    """
    pix = _blob_pixels(blob)
    size_d = None
    if pix is not None and not _blob_bottom_truncated(blob):
        _cx, _cy, w_px, h_px = pix
        if w_px >= SIZE_MIN_PX and h_px >= SIZE_MIN_PX:
            size_d = cube_range_estimate_cm(pose, _cx, _cy, w_px, h_px)
    if size_d is None and pix is not None and not _blob_bottom_truncated(blob):
        size_d = size_range_estimate(blob)
    if size_d is None:
        return None
    pixel_left_deg = (float(image_center_nx) - blob.nx) * CAMERA_HFOV_DEG
    servo_left_deg = (pose[6] - BASE_CENTER) / PULSE_PER_DEGREE
    yaw_left_deg = servo_left_deg + pixel_left_deg
    yaw = math.radians(yaw_left_deg)
    print(
        f"size-range fallback radius={size_d:.1f}cm "
        f"(floor ray unavailable, ray_pitch={ray_pitch:.1f}deg)",
        flush=True,
    )
    return BlockEstimate(
        radius_cm=size_d,
        lateral_left_cm=size_d * math.sin(yaw),
        forward_cm=size_d * math.cos(yaw),
        yaw_left_deg=yaw_left_deg,
        camera_radius_cm=camera.radius_cm,
        camera_height_cm=camera_height,
        ray_pitch_deg=ray_pitch,
        block_height_cm=SIZE_BLOCK_CM,
        nx=blob.nx,
        ny=blob.ny,
        source="size_apparent",
    )


def project_floor_pixel(
    pose: dict[int, int],
    nx: float,
    ny: float,
) -> tuple[float, float] | None:
    """Project one image point to floor coordinates (left, forward), in cm."""
    if any(servo not in pose for servo in (3, 4, 5, 6)):
        return None
    camera = forward_kinematics(pose, CAMERA_LINK_CM)
    camera_height = camera.height_cm + CAMERA_Z_OFFSET_CM
    pixel_down = (ny - 0.5) * CAMERA_VFOV_DEG
    ray_pitch = camera.pitch_deg + CAMERA_PITCH_OFFSET_DEG - pixel_down
    if camera_height <= 0.5 or not -88.0 <= ray_pitch <= -5.0:
        return None
    forward_from_camera = camera_height / math.tan(math.radians(-ray_pitch))
    ground_radius = camera.radius_cm + forward_from_camera
    if not 2.0 <= ground_radius <= 80.0:
        return None
    pixel_left_deg = (0.5 - nx) * CAMERA_HFOV_DEG
    servo_left_deg = (pose[6] - BASE_CENTER) / PULSE_PER_DEGREE
    yaw = math.radians(servo_left_deg + pixel_left_deg)
    return ground_radius * math.sin(yaw), ground_radius * math.cos(yaw)


def estimate_face_alignment(
    pose: dict[int, int],
    blob,
    *,
    flip_x: bool,
) -> FaceAlignmentEstimate | None:
    """Estimate the lower visible block-face edge on the floor.

    The contour rectangle's two lowest image corners describe one visible
    lower edge.  Projecting them through the current FK/camera pose removes
    most of the apparent image skew.  A lateral floor edge is +/-90 degrees
    from robot-forward and therefore gives a face-normal approach.
    """
    points = list(getattr(blob, "box_points", ()))
    rectangularity = float(getattr(blob, "rectangularity", 0.0))
    if len(points) != 4 or rectangularity < FACE_MIN_RECTANGULARITY:
        return None
    bottom = sorted(points, key=lambda point: point[1], reverse=True)[:2]
    if flip_x:
        bottom = [(1.0 - nx, ny) for nx, ny in bottom]
    bottom.sort(key=lambda point: point[0])
    first = project_floor_pixel(pose, bottom[0][0], bottom[0][1])
    second = project_floor_pixel(pose, bottom[1][0], bottom[1][1])
    if first is None or second is None:
        return None
    delta_left = second[0] - first[0]
    delta_forward = second[1] - first[1]
    edge_length = math.hypot(delta_left, delta_forward)
    if not 0.4 <= edge_length <= 25.0:
        return None
    edge_angle = math.degrees(math.atan2(delta_left, delta_forward))
    while edge_angle >= 90.0:
        edge_angle -= 180.0
    while edge_angle < -90.0:
        edge_angle += 180.0
    return FaceAlignmentEstimate(
        edge_angle_deg=edge_angle,
        error_deg=90.0 - abs(edge_angle),
        edge_length_cm=edge_length,
    )


def measure_face_alignment(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    *,
    flip_x: bool,
) -> FaceAlignmentEstimate:
    robot.stop()
    time.sleep(CAMERA_DELAY_SECONDS + SERVO_SETTLE_SECONDS)
    estimates: list[FaceAlignmentEstimate] = []
    last_frame = None
    last_blob = None
    for _ in range(FACE_SAMPLE_COUNT * 4):
        frame, blob = read_locked(video, lock)
        if blob is None:
            time.sleep(0.03)
            continue
        estimate = estimate_face_alignment(robot.pose, blob, flip_x=flip_x)
        if estimate is None:
            time.sleep(0.03)
            continue
        estimates.append(estimate)
        last_frame, last_blob = frame, blob
        if len(estimates) >= FACE_SAMPLE_COUNT:
            break
        time.sleep(0.025)
    if len(estimates) < FACE_MIN_VALID_SAMPLES:
        raise RuntimeError(
            f"only {len(estimates)} valid block-face samples; "
            "refusing a diagonal blind approach"
        )
    # Away from the +/-90-degree wrap, the median is robust to a single bad
    # contour.  At the wrap the face is already aligned, so use the median
    # absolute angle and retain the majority sign.
    absolute_angle = statistics.median(
        abs(item.edge_angle_deg) for item in estimates
    )
    signs = [1.0 if item.edge_angle_deg >= 0.0 else -1.0 for item in estimates]
    sign = 1.0 if sum(signs) >= 0.0 else -1.0
    result = FaceAlignmentEstimate(
        edge_angle_deg=sign * absolute_angle,
        error_deg=90.0 - absolute_angle,
        edge_length_cm=statistics.median(
            item.edge_length_cm for item in estimates
        ),
    )
    if last_frame is not None:
        save_debug_frame(last_frame, last_blob, DEBUG_PATH)
    return result


def start_motion(robot: Robot, kind: str, speed: int) -> None:
    if kind not in MOTOR_SIGNS:
        raise ValueError(f"unsupported chassis motion: {kind}")
    speed = clamp(int(speed), 35, 40)
    robot.stop()
    for motor, sign in MOTOR_SIGNS[kind]:
        motor_speed = sign * speed
        print(f"motor {motor} -> {motor_speed}", flush=True)
        robot.motors.write_motor(motor, motor_speed)


def tracking_axis_step_limit(error: float, *, pan: bool) -> int:
    """Allow the PID's prepared correction through in larger coarse steps.

    The old 14/12 PWM clamps forced 5-10 settle frames after every base move.
    Far from image centre we can safely accept the PID's larger already-bounded
    correction, then taper back to the historical fine step near the deadband.
    """
    magnitude = abs(float(error))
    if pan:
        if magnitude >= TRACK_COARSE_ERROR:
            return TRACK_PAN_COARSE_STEP
        if magnitude >= TRACK_MEDIUM_ERROR:
            return TRACK_PAN_MEDIUM_STEP
        return TRACK_PAN_STEP
    if magnitude >= TRACK_COARSE_ERROR:
        return TRACK_TILT_COARSE_STEP
    if magnitude >= TRACK_MEDIUM_ERROR:
        return TRACK_TILT_MEDIUM_STEP
    return TRACK_TILT_STEP


def body_align_pulse_duration(bearing_deg: float, rate_deg_s: float) -> float:
    """Predict one bounded yaw pulse that should approach, not cross, centre."""
    rate = min(BODY_ALIGN_RATE_MAX_DEG_S, max(BODY_ALIGN_RATE_MIN_DEG_S, float(rate_deg_s)))
    residual = max(0.0, abs(float(bearing_deg)) - BODY_BEARING_DEADBAND_DEG)
    predicted = residual / rate
    minimum = TURN_PULSE_SECONDS if abs(float(bearing_deg)) > TURN_FINE_WINDOW_DEG else TURN_FINE_PULSE_SECONDS
    return min(BODY_ALIGN_MAX_PULSE_SECONDS, max(minimum, predicted))


def tracking_update(
    robot: Robot,
    blob,
    gaze: Gaze,
    *,
    last_time: float,
    flip_x: bool,
    flip_y: bool,
) -> tuple[Gaze, float, bool]:
    now = time.monotonic()
    dt = min(0.25, max(0.03, now - last_time))
    previous_pan = gaze.pan
    previous_tilt = gaze.tilt
    nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
    ny = 1.0 - float(blob.ny) if flip_y else float(blob.ny)
    pan_limit = tracking_axis_step_limit(0.5 - nx, pan=True)
    tilt_limit = tracking_axis_step_limit(0.5 - ny, pan=False)
    gaze = tracking_step(blob, gaze, dt=dt, flip_x=flip_x, flip_y=flip_y)
    gaze.pan = clamp(
        step_toward(previous_pan, gaze.pan, pan_limit), PAN_MIN, PAN_MAX
    )
    gaze.tilt = clamp(
        step_toward(previous_tilt, gaze.tilt, tilt_limit), TILT_MIN, TILT_MAX
    )
    robot.nudge_servos(
        {PAN_SERVO: gaze.pan, TILT_SERVO: gaze.tilt},
        duration=0.10,
    )
    moved = gaze.pan != previous_pan or gaze.tilt != previous_tilt
    return gaze, now, moved


def read_locked(video: LiveVideo, lock: TargetLock):
    frame = video.read()
    candidate = detect_red_blob(frame, min_area=TRACK_MIN_AREA, crop_left=0)
    return frame, lock.select(candidate)


def confirmed_candidate(video: LiveVideo, first) -> object | None:
    previous = first
    confirmations = 1
    for _ in range(ACQUIRE_CONFIRMATIONS + 2):
        time.sleep(0.03)
        frame = video.read()
        current = detect_red_blob(frame, min_area=TRACK_MIN_AREA, crop_left=0)
        if current is None:
            confirmations = 0
            continue
        jump = math.hypot(current.nx - previous.nx, current.ny - previous.ny)
        ratio = current.area / max(1.0, float(previous.area))
        if jump <= 0.10 and 0.60 <= ratio <= 1.65:
            confirmations += 1
            previous = current
            if confirmations >= ACQUIRE_CONFIRMATIONS:
                return current
        else:
            confirmations = 1
            previous = current
    return None


def recover_locked_target(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
) -> Gaze | None:
    """Search only near the last gaze while the chassis remains stopped."""
    robot.stop()
    origin_pan, origin_tilt = gaze.pan, gaze.tilt
    print(
        f"target temporarily hidden; local recovery around "
        f"pan={origin_pan} tilt={origin_tilt}",
        flush=True,
    )
    recovery_points = (
        (origin_pan, origin_tilt),
        (origin_pan - 60, origin_tilt),
        (origin_pan + 60, origin_tilt),
        (origin_pan - 120, origin_tilt - 45),
        (origin_pan + 120, origin_tilt - 45),
        (origin_pan, TILT_MIN),
        (origin_pan - 70, TILT_MIN),
        (origin_pan + 70, TILT_MIN),
        (origin_pan, origin_tilt + 80),
    )
    for requested_pan, requested_tilt in recovery_points:
        pan = clamp(requested_pan, PAN_MIN, PAN_MAX)
        tilt = clamp(requested_tilt, TILT_MIN, TILT_MAX)
        robot.nudge_servos(
            {PAN_SERVO: pan, TILT_SERVO: tilt},
            duration=0.10,
        )
        time.sleep(CAMERA_DELAY_SECONDS)
        frame = video.read()
        candidate = detect_red_blob(
            frame,
            min_area=TRACK_MIN_AREA,
            crop_left=0,
        )
        if candidate is None:
            continue
        confirmed = confirmed_candidate(video, candidate)
        if confirmed is None:
            continue
        lock.last = confirmed
        lock.misses = 0
        print(
            f"locked red block recovered nx={confirmed.nx:.3f} "
            f"ny={confirmed.ny:.3f} pan={pan} tilt={tilt}",
            flush=True,
        )
        return Gaze(pan=pan, tilt=tilt)
    robot.nudge_servos(
        {PAN_SERVO: origin_pan, TILT_SERVO: origin_tilt},
        duration=0.10,
    )
    return None


def scan_for_target(
    robot: Robot,
    video: LiveVideo,
    gaze: Gaze,
    *,
    timeout_seconds: float,
) -> tuple[Gaze, TargetLock]:
    deadline = time.monotonic() + timeout_seconds
    rows = []
    for row, tilt in enumerate(SCAN_TILTS):
        pans = SCAN_PANS if row % 2 == 0 else tuple(reversed(SCAN_PANS))
        rows.extend((pan, tilt) for pan in pans)
    waypoint = 0
    while time.monotonic() < deadline:
        target_pan, target_tilt = rows[waypoint % len(rows)]
        while (gaze.pan, gaze.tilt) != (target_pan, target_tilt):
            if time.monotonic() >= deadline:
                break
            gaze.pan = step_toward(gaze.pan, target_pan, SCAN_STEP)
            gaze.tilt = step_toward(gaze.tilt, target_tilt, SCAN_STEP)
            robot.nudge_servos(
                {PAN_SERVO: gaze.pan, TILT_SERVO: gaze.tilt},
                duration=0.10,
            )
            # A decoded MJPEG frame can lag a servo command.  Do not scan the
            # next location until the transport delay has expired.
            time.sleep(CAMERA_DELAY_SECONDS)
            frame = video.read()
            candidate = detect_red_blob(
                frame, min_area=TRACK_MIN_AREA, crop_left=0
            )
            print(
                f"scan pan={gaze.pan} tilt={gaze.tilt} "
                f"red={'yes' if candidate is not None else 'no'}",
                flush=True,
            )
            if candidate is not None:
                confirmed = confirmed_candidate(video, candidate)
                if confirmed is not None:
                    print(
                        f"red block confirmed nx={confirmed.nx:.3f} "
                        f"ny={confirmed.ny:.3f} area={confirmed.area}",
                        flush=True,
                    )
                    save_debug_frame(frame, confirmed, DEBUG_PATH)
                    return gaze, TargetLock(confirmed)
        waypoint += 1
    raise RuntimeError("full camera scan timed out without a confirmed red block")


def centre_gaze(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
    confirmations: int = CENTER_CONFIRMATIONS,
    max_frames: int = 90,
    allow_limit_lock: bool = True,
) -> Gaze:
    stable = 0
    missing = 0
    last_time = time.monotonic()
    for frame_number in range(1, max_frames + 1):
        _, blob = read_locked(video, lock)
        if blob is None:
            missing += 1
            stable = 0
            robot.stop()
            if missing == 1 or missing % 6 == 0:
                print(
                    f"lock hold {missing}/{LOCK_HOLD_FRAMES}; "
                    f"keep pan={gaze.pan} tilt={gaze.tilt}",
                    flush=True,
                )
            if missing >= LOCK_HOLD_FRAMES:
                recovered = recover_locked_target(robot, video, lock, gaze)
                if recovered is None:
                    raise LockedTargetLost(
                        "locked red block not found in local recovery scan"
                    )
                gaze = recovered
                missing = 0
                last_time = time.monotonic()
                continue
            time.sleep(0.04)
            continue
        missing = 0
        gaze, last_time, moved = tracking_update(
            robot,
            blob,
            gaze,
            last_time=last_time,
            flip_x=flip_x,
            flip_y=flip_y,
        )
        nx = 1.0 - blob.nx if flip_x else blob.nx
        ny = 1.0 - blob.ny if flip_y else blob.ny
        error_x = 0.5 - nx
        error_y = 0.5 - ny
        pan_saturated = allow_limit_lock and (
            (gaze.pan >= PAN_MAX and error_x > 0.0)
            or (gaze.pan <= PAN_MIN and error_x < 0.0)
        )
        tilt_saturated = allow_limit_lock and (
            (gaze.tilt >= TILT_MAX and error_y > 0.0)
            or (gaze.tilt <= TILT_MIN and error_y < 0.0)
        )
        centred = (
            abs(error_x) <= IMAGE_X_DEADBAND or pan_saturated
        ) and (
            abs(error_y) <= IMAGE_Y_DEADBAND or tilt_saturated
        )
        stable = stable + 1 if centred else 0
        print(
            f"gaze frame={frame_number} nx={nx:.3f} ny={ny:.3f} "
            f"pan={gaze.pan} tilt={gaze.tilt} stable={stable}/{confirmations} "
            f"limit-lock={'yes' if pan_saturated or tilt_saturated else 'no'}",
            flush=True,
        )
        if stable >= confirmations:
            return gaze
        # The direct MJPEG parser supplies fresh frames quickly.  Wait out the
        # short camera delay only when a physical servo correction occurred.
        time.sleep(CAMERA_DELAY_SECONDS if moved else 0.025)
    raise RuntimeError("could not hold the red block at image centre")


def should_use_continuous_far_approach(
    radius_cm: float,
    target_radius_cm: float,
) -> bool:
    """Use streaming motion only with a large verified stand-off margin."""
    radius = float(radius_cm)
    target = float(target_radius_cm)
    return (
        radius >= COARSE_STREAM_ENTER_RADIUS_CM
        and radius - target >= COARSE_STREAM_TARGET_MARGIN_CM
    )


def continuous_far_approach(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    target_radius_cm: float,
    flip_x: bool,
    flip_y: bool,
) -> StreamingApproachResult:
    """Run a bounded predictive far approach with an always-on stop guard.

    Instead of command -> stop -> observe -> command, the controller keeps one
    coarse forward command active and continuously evaluates fresh camera
    observations.  Each observation prepares the next few hundred milliseconds
    of expected motion.  Continued motion is allowed only while the measured
    trajectory agrees with that prediction.  Target loss, lateral drift,
    implausible closing speed, prediction error, a predicted boundary crossing,
    physical pan limits, or the hard wall-clock bound all pre-empt the continuation and
    fall through the ``finally`` STOP.  This path is never used near contact.
    """
    stop_radius = max(
        COARSE_STREAM_EXIT_RADIUS_CM,
        float(target_radius_cm) + COARSE_STREAM_TARGET_MARGIN_CM,
    )
    started = time.monotonic()
    last_time = started
    missing = 0
    invalid_range = 0
    frames = 0
    last_estimate = None
    last_range: float | None = None
    last_range_elapsed: float | None = None
    closing_rate: float | None = None
    projected_radius: float | None = None
    stop_reason = "time-bound"

    start_motion(robot, "forward", FAR_FORWARD_SPEED)
    try:
        for frame_number in range(1, COARSE_STREAM_MAX_FRAMES + 1):
            elapsed = time.monotonic() - started
            if elapsed >= COARSE_STREAM_MAX_SECONDS:
                stop_reason = "time-bound"
                break

            if COARSE_STREAM_FRAME_DELAY_SECONDS > 0.0:
                time.sleep(COARSE_STREAM_FRAME_DELAY_SECONDS)
            _, blob = read_locked(video, lock)
            frames = frame_number
            if blob is None:
                missing += 1
                if missing >= COARSE_STREAM_MISSING_LIMIT:
                    stop_reason = "target-lost"
                    break
                continue
            missing = 0

            nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
            if abs(0.5 - nx) > COARSE_STREAM_MAX_X_ERROR:
                stop_reason = "lateral-drift"
                break

            estimate = estimate_block(robot.pose, blob)
            if estimate is None:
                invalid_range += 1
                if invalid_range >= COARSE_STREAM_INVALID_RANGE_LIMIT:
                    stop_reason = "range-invalid"
                    break
            else:
                invalid_range = 0
                last_estimate = estimate
                radius = float(estimate.radius_cm)
                if radius <= CHASSIS_HARD_LIMIT_CM:
                    stop_reason = "hard-radius"
                    raise RuntimeError(
                        "chassis safety radius violated during continuous approach"
                    )
                if radius <= stop_radius:
                    stop_reason = "coarse-boundary"
                    break

                if last_range is not None and last_range_elapsed is not None:
                    dt = max(0.03, elapsed - last_range_elapsed)
                    observed_rate = (last_range - radius) / dt
                    # A backwards jump or an implausibly large forward jump means
                    # the current visual range cannot safely authorize the
                    # already-prepared continuation. Stop now and re-measure.
                    if not (
                        PREDICTIVE_MIN_CLOSING_RATE_CM_S
                        <= observed_rate
                        <= PREDICTIVE_MAX_CLOSING_RATE_CM_S
                    ):
                        stop_reason = "prediction-rate"
                        break
                    if closing_rate is not None:
                        expected_radius = last_range - closing_rate * dt
                        if abs(radius - expected_radius) > PREDICTIVE_RANGE_ERROR_CM:
                            stop_reason = "prediction-deviation"
                            break
                        closing_rate = (
                            PREDICTIVE_RATE_ALPHA * observed_rate
                            + (1.0 - PREDICTIVE_RATE_ALPHA) * closing_rate
                        )
                    else:
                        closing_rate = observed_rate

                forecast_rate = max(
                    0.0,
                    closing_rate
                    if closing_rate is not None
                    else PREDICTIVE_SEED_CLOSING_RATE_CM_S,
                )
                projected_radius = radius - forecast_rate * PREDICTIVE_HORIZON_SECONDS
                print(
                    "predictive far approach "
                    f"frame={frame_number} range={radius:.2f}cm "
                    f"rate={forecast_rate:.1f}cm/s "
                    f"horizon={PREDICTIVE_HORIZON_SECONDS:.2f}s "
                    f"projected={projected_radius:.2f}cm stop={stop_radius:.2f}cm",
                    flush=True,
                )
                # This is the receding-horizon equivalent of a human preparing
                # the next step but withholding it when the future looks unsafe.
                if projected_radius <= stop_radius + PREDICTIVE_STOP_GUARD_CM:
                    stop_reason = "predicted-boundary"
                    break

                last_range = radius
                last_range_elapsed = elapsed

            gaze, last_time, _moved = tracking_update(
                robot,
                blob,
                gaze,
                last_time=last_time,
                flip_x=flip_x,
                flip_y=flip_y,
            )
            if (
                gaze.pan <= PAN_MIN + PAN_LIMIT_MARGIN
                or gaze.pan >= PAN_MAX - PAN_LIMIT_MARGIN
            ):
                stop_reason = "pan-physical-limit"
                break
        else:
            stop_reason = "frame-bound"
    finally:
        # Motor STOP is the highest-priority action. No recovery/replanning is
        # attempted until this has completed.
        robot.stop()

    duration = max(0.0, time.monotonic() - started)
    print(
        "continuous far approach stopped "
        f"reason={stop_reason} duration={duration:.2f}s frames={frames} "
        f"range={None if last_estimate is None else round(last_estimate.radius_cm, 2)}cm "
        f"projected={None if projected_radius is None else round(projected_radius, 2)}cm",
        flush=True,
    )

    if stop_reason == "target-lost":
        recovered = recover_locked_target(robot, video, lock, gaze)
        if recovered is None:
            raise LockedTargetLost(
                "locked target disappeared during continuous far approach"
            )
        gaze = recovered
    else:
        time.sleep(CAMERA_DELAY_SECONDS)
        gaze = centre_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=flip_x,
            flip_y=flip_y,
            confirmations=COARSE_STREAM_RECENTER_CONFIRMATIONS,
            max_frames=20,
            allow_limit_lock=True,
        )

    return StreamingApproachResult(
        gaze=gaze,
        duration_s=duration,
        frames=frames,
        last_estimate=last_estimate,
        stop_reason=stop_reason,
    )


def pursue_trim_step(
    robot: Robot,
    gaze: Gaze,
    state: PursueState,
    *,
    blob,
    flip_x: bool,
    target_nx: float,
    deadband: float,
    motion_key: tuple | None = None,
    prenudge: bool = True,
) -> tuple[Gaze, str]:
    """Single post-motion gaze trim with feedforward memory.

    Takes an ALREADY-OBSERVED detection (usually the loop's median sample) and
    never captures a frame itself: a fresh single frame jitters +-0.02 against
    the 3-frame median the gates use, so trimming off it chases noise while
    the median marches (SIM: nx 0.487->0.545 uncorrected).  Applies at most one
    pre-nudge plus one trim.  Returns (gaze, outcome) with outcome in
    {"centered", "trimmed", "lost", "saturated"}; on "lost"/"saturated" the
    caller falls back to the legacy recovery loop.  Set prenudge=False when
    the same pulse was already pre-nudged, so one pulse never double-applies
    its memory.
    """
    entry_pan = gaze.pan
    if prenudge:
        pre = recall_corr(state, motion_key)
        if pre:
            pan, _ = clamp_hold(gaze.pan + pre)
            if pan != gaze.pan:
                gaze.pan = pan
                robot.nudge_servos(
                    {PAN_SERVO: gaze.pan, TILT_SERVO: gaze.tilt}, duration=0.10
                )
                time.sleep(CAMERA_DELAY_SECONDS)
    if blob is None:
        store_corr(state, motion_key, 0)
        state.gaze = gaze
        return gaze, "lost"
    nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
    if abs(nx - float(target_nx)) <= float(deadband):
        state.gaze = gaze
        return gaze, "centered"
    before = gaze.pan
    # Direct proportional centering (deadbeat): pan pulses per image fraction
    # come straight from the calibrated HFOV.  Do NOT route through
    # tracking_step here: its internal 0.05 deadband zeroes exactly the
    # 0.025-0.05 drifts this trim exists to correct (SIM: nx marched
    # 0.487->0.539 with pan frozen because every trim computed ~zero).
    # Sign verified: target right of centre (nx > target) drives pan down.
    # Gain < 1 (PURSUE_TRIM_GAIN): full-step correction oscillates under
    # model error; the residual is re-trimmed on the next pulse anyway.
    step = (float(target_nx) - nx) * CAMERA_HFOV_DEG * PULSE_PER_DEGREE * PURSUE_TRIM_GAIN
    pan, saturated = clamp_hold(int(round(before + step)))
    if saturated:
        store_corr(state, motion_key, 0)
        state.gaze = gaze
        return gaze, "saturated"
    if pan != before:
        gaze.pan = pan
        robot.nudge_servos(
            {PAN_SERVO: gaze.pan, TILT_SERVO: gaze.tilt}, duration=0.10
        )
        time.sleep(CAMERA_DELAY_SECONDS)
    # Total eye movement of this call (pre-nudge + trim): the next identical
    # pulse replays exactly what this one needed, converging the loop to zero.
    store_corr(state, motion_key, gaze.pan - entry_pan)
    state.gaze = gaze
    return gaze, "trimmed"


def motion_pulse_with_gaze(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    kind: str,
    speed: int,
    duration: float,
    flip_x: bool,
    flip_y: bool,
    gaze_mode: str = GAZE_MODE_FIXATE,
    pursue_state: PursueState | None = None,
) -> Gaze:
    # Delayed visual feedback must not steer the arm in the opposite direction
    # during a short chassis pulse.  Hold the last locked gaze, make one small
    # measured motion, stop, flush the delay, then close the visual loop again.
    note_motion = getattr(lock, "note_ego_motion", None)
    if callable(note_motion):
        note_motion()
    start_motion(robot, kind, speed)
    try:
        time.sleep(duration)
    finally:
        robot.stop()
    time.sleep(CAMERA_DELAY_SECONDS)
    if normalize_gaze_mode(gaze_mode) != GAZE_MODE_FIXATE:
        # Pursue fast path: feedforward replay + one verify frame + at most
        # one trim, instead of the full multi-confirmation centre.  Loss and
        # saturation fall through to the legacy recovery below, so worst-case
        # behaviour is unchanged.
        if pursue_state is None:
            pursue_state = PursueState()
        _frame, fresh = read_locked(video, lock)
        gaze, outcome = pursue_trim_step(
            robot, gaze, pursue_state,
            blob=fresh, flip_x=flip_x, target_nx=0.5,
            deadband=IMAGE_X_DEADBAND,
            motion_key=(kind, speed, round(float(duration), 3)),
        )
        print(f"pursue trim outcome={outcome} pan={gaze.pan}", flush=True)
        if outcome in ("centered", "trimmed"):
            return gaze
    _gaze_out = centre_gaze(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        flip_y=flip_y,
        confirmations=3,
        max_frames=35,
        # Near the block the floor target can remain slightly below image
        # centre after the tilt servo reaches its safe mechanical minimum.
        # Treat that saturated axis as locked instead of replaying identical
        # frames until the controller times out.
        allow_limit_lock=True,
    )
    return _gaze_out


def target_bearing_left_deg(
    gaze: Gaze,
    blob,
    *,
    flip_x: bool,
    image_center_nx: float = 0.5,
) -> float:
    """Return the observed target bearing relative to the chassis.

    Pan PWM alone is insufficient after a lateral base move because the tracker
    deliberately tolerates a small residual image error.  Servo yaw plus the
    current horizontal pixel ray is the observable bearing that both REAL and
    SIM can compute without odometry or privileged world coordinates.

    ``image_center_nx`` is explicit because the measured camera principal point
    is not necessarily normalized image centre.  Existing far-view callers keep
    the historical 0.5 default; the fixed capture geometry passes its calibrated
    horizontal target.
    """
    nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
    pixel_left_deg = (float(image_center_nx) - nx) * CAMERA_HFOV_DEG
    servo_left_deg = (int(gaze.pan) - BASE_CENTER) / PULSE_PER_DEGREE
    return servo_left_deg + pixel_left_deg


def retreat_for_body_alignment(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_y: bool,
    safe_ny: float = BODY_ALIGN_SAFE_NY,
) -> Gaze:
    """Create visual clearance before any chassis yaw.

    Retreat is straight backward with pan/tilt frozen, so target motion in
    image-y is directly observable and cannot be hidden by a camera controller.
    """
    _frame, blob = read_locked(video, lock)
    if blob is None:
        raise LockedTargetLost("target missing before body-alignment clearance check")
    ny = 1.0 - float(blob.ny) if flip_y else float(blob.ny)
    safe_ny = float(safe_ny)
    if ny <= safe_ny:
        return gaze
    # Ordinary callers only need this when the target is truly at the bottom.
    # A stricter close-normalization caller deliberately asks for ~0.30.
    if safe_ny >= BODY_ALIGN_SAFE_NY and ny < BODY_ALIGN_TOO_CLOSE_NY:
        return gaze
    print(
        f"target too close for chassis yaw ny={ny:.3f}; "
        f"retreat straight until ny<={safe_ny:.3f}",
        flush=True,
    )
    low_progress = 0
    previous = ny
    previous_area = float(getattr(blob, "area", 0.0) or 0.0)
    previous_height = float(getattr(blob, "height", 0.0) or 0.0)
    for pulse in range(1, BODY_ALIGN_RETREAT_MAX_PULSES + 1):
        note_motion = getattr(lock, "note_ego_motion", None)
        if callable(note_motion):
            note_motion()
        start_motion(robot, "backward", MOTOR_SPEED)
        try:
            time.sleep(BODY_ALIGN_RETREAT_SECONDS)
        finally:
            robot.stop()
        time.sleep(CAMERA_DELAY_SECONDS)
        current = None
        for _ in range(8):
            _frame, candidate = read_locked(video, lock)
            if candidate is not None:
                current = candidate
                break
            time.sleep(0.025)
        if current is None:
            raise LockedTargetLost(
                "target lost during straight close-range retreat; chassis stopped"
            )
        ny = 1.0 - float(current.ny) if flip_y else float(current.ny)
        progress = previous - ny
        area = float(getattr(current, "area", 0.0) or 0.0)
        height = float(getattr(current, "height", 0.0) or 0.0)
        area_progress = (previous_area - area) / max(1.0, previous_area)
        height_progress = (previous_height - height) / max(1.0, previous_height)
        visual_retreat_progress = (
            progress >= BODY_ALIGN_RETREAT_MIN_PROGRESS_NY
            or area_progress >= 0.015
            or height_progress >= 0.010
        )
        print(
            f"body-clearance retreat={pulse}/{BODY_ALIGN_RETREAT_MAX_PULSES} "
            f"ny={previous:.3f}->{ny:.3f} progress={progress:+.3f} "
            f"area-progress={area_progress:+.3f} height-progress={height_progress:+.3f}",
            flush=True,
        )
        if ny <= safe_ny:
            robot.stop()
            return gaze
        if not visual_retreat_progress:
            low_progress += 1
        else:
            low_progress = 0
        if low_progress >= 2:
            raise RuntimeError(
                "straight retreat did not create visual body-alignment clearance; "
                "refusing to rotate near the block"
            )
        previous = ny
        previous_area = area
        previous_height = height
    raise RuntimeError(
        f"target remains too close for chassis yaw after "
        f"{BODY_ALIGN_RETREAT_MAX_PULSES} backward pulses (ny={ny:.3f})"
    )


def align_body_to_gaze(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
) -> Gaze:
    """Absorb camera bearing into chassis yaw without left/right ping-pong.

    The previous loop called ``motion_pulse_with_gaze`` for every yaw pulse.
    That helper immediately ran the pan tracker, so the camera compensated the
    chassis turn before the next bearing sample.  On the real MasterPi this
    produced the observed ``right -> left -> right`` loop for a diagonal block.

    Choose the yaw direction once, hold camera pan fixed while the chassis turns,
    and stop when the bearing is centred or crosses zero by a small amount.  A
    zero crossing is *not* corrected with an opposite chassis pulse; the next
    face measurement is a better authority than an oscillating sub-controller.
    """
    initial_sign: float | None = None
    direction: str | None = None
    predicted_rate = BODY_ALIGN_RATE_PRIOR_DEG_S
    previous_bearing: float | None = None
    previous_duration: float | None = None
    fine_correction_used = False

    for pulse_number in range(1, BODY_ALIGN_MAX_PULSES + 1):
        _frame, blob = read_locked(video, lock)
        if blob is None:
            # Reacquire only while stopped.  Once found, keep the newly acquired
            # pan fixed again for the remaining one-direction chassis correction.
            gaze = centre_gaze(
                robot, video, lock, gaze,
                flip_x=flip_x, flip_y=flip_y,
                # A large one-direction re-face can temporarily push the locked
                # floor cube out of view.  Local recovery is allowed to hold the
                # last gaze for LOCK_HOLD_FRAMES first; do not let those hold
                # frames consume nearly the entire post-recovery centering budget.
                confirmations=2,
                max_frames=LOCK_HOLD_FRAMES + 20,
                allow_limit_lock=True,
            )
            _frame, blob = read_locked(video, lock)
            if blob is None:
                raise LockedTargetLost(
                    "target disappeared during one-direction body alignment"
                )

        bearing = target_bearing_left_deg(gaze, blob, flip_x=flip_x)
        if previous_bearing is not None and previous_duration:
            observed_rate = (abs(previous_bearing) - abs(bearing)) / previous_duration
            if BODY_ALIGN_RATE_MIN_DEG_S <= observed_rate <= BODY_ALIGN_RATE_MAX_DEG_S:
                predicted_rate = (
                    BODY_ALIGN_RATE_ALPHA * observed_rate
                    + (1.0 - BODY_ALIGN_RATE_ALPHA) * predicted_rate
                )
        if abs(bearing) <= BODY_BEARING_DEADBAND_DEG:
            robot.stop()
            print(
                f"body aligned; target-bearing={bearing:+.2f}deg "
                f"camera-pan={gaze.pan} nx={float(blob.nx):.3f}",
                flush=True,
            )
            return gaze
        if fine_correction_used and abs(bearing) <= BODY_FINE_ACCEPT_DEG:
            robot.stop()
            print(
                f"body aligned after one fine correction; residual={bearing:+.2f}deg "
                "accepted so approach can continue",
                flush=True,
            )
            return gaze

        sign = 1.0 if bearing > 0.0 else -1.0
        if initial_sign is None:
            initial_sign = sign
            direction = "rotate-left" if bearing > 0.0 else "rotate-right"
        elif sign != initial_sign:
            robot.stop()
            if abs(bearing) <= BODY_BEARING_CROSS_ACCEPT_DEG:
                print(
                    f"body alignment crossed centre by {bearing:+.2f}deg; "
                    "accepting guarded one-way re-face without reverse pulse",
                    flush=True,
                )
                return gaze
            raise RuntimeError(
                f"body alignment overshot to {bearing:+.2f}deg; "
                "refusing a reverse pulse that would create left/right oscillation"
            )

        assert direction is not None
        fine_now = abs(bearing) <= BODY_FINE_ACCEPT_DEG
        duration = body_align_pulse_duration(bearing, predicted_rate)
        print(
            f"body-align pulse={pulse_number} direction={direction} "
            f"target-bearing={bearing:+.2f}deg pan={gaze.pan} "
            f"nx={float(blob.nx):.3f} duration={duration:.2f}s "
            f"predicted-rate={predicted_rate:.1f}deg/s",
            flush=True,
        )
        # Deliberately do not call motion_pulse_with_gaze here.  Pan tracking
        # during yaw is the feedback coupling that caused the physical ping-pong.
        start_motion(robot, direction, TURN_SPEED)
        try:
            time.sleep(duration)
        finally:
            robot.stop()
        previous_bearing = bearing
        previous_duration = duration
        if fine_now:
            fine_correction_used = True
        time.sleep(CAMERA_DELAY_SECONDS)

    raise RuntimeError(
        f"body alignment exceeded {BODY_ALIGN_MAX_PULSES} one-direction turn pulses"
    )


def face_reposition_dogleg(
    robot: Robot,
    lock: TargetLock,
    *,
    side: str,
) -> None:
    """Create bounded lateral viewpoint change without mecanum strafe.

    ``side`` is the desired translation side in the robot frame.  A right
    reposition yaws left, backs up, then restores heading with a right yaw; the
    left case is mirrored.  Backing while yawed intentionally creates clearance
    rather than driving closer to the cube.  No camera correction is attempted
    mid-dog-leg; every sub-motion stops in ``finally`` and perception resumes
    only after the nominal heading has been restored.
    """
    if side not in {"left", "right"}:
        raise ValueError(f"face reposition side must be left/right, got {side!r}")
    turn_out = "rotate-right" if side == "left" else "rotate-left"
    turn_restore = "rotate-left" if side == "left" else "rotate-right"
    note_motion = getattr(lock, "note_ego_motion", None)
    if callable(note_motion):
        note_motion()
    sequence = (
        (turn_out, TURN_SPEED, FACE_REPOSITION_TURN_SECONDS),
        ("backward", MOTOR_SPEED, FACE_REPOSITION_BACKWARD_SECONDS),
        (turn_restore, TURN_SPEED, FACE_REPOSITION_RESTORE_TURN_SECONDS),
    )
    for kind, speed, duration in sequence:
        start_motion(robot, kind, speed)
        try:
            time.sleep(duration)
        finally:
            robot.stop()
    time.sleep(CAMERA_DELAY_SECONDS)


def orbit_strafe_measure_bearing(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    direction: str,
    flip_x: bool,
) -> tuple[Gaze, float]:
    """Strafe once with frozen camera pan and return measured orbit bearing.

    This is the missing physical feedback in the old face-align loop.  The
    target bearing is sampled before motion and again immediately after the
    stopped strafe, *before* pan tracking or chassis re-facing can hide the
    displacement.  Positive bearing means the target moved to the robot's left.
    """
    if direction not in {"left", "right"}:
        raise ValueError(f"face orbit requires lateral motion, got {direction!r}")
    _frame, before = read_locked(video, lock)
    if before is None:
        raise LockedTargetLost("target missing before verified face-orbit strafe")
    before_bearing = target_bearing_left_deg(gaze, before, flip_x=flip_x)

    note_motion = getattr(lock, "note_ego_motion", None)
    if callable(note_motion):
        note_motion()
    start_motion(robot, direction, ORBIT_SPEED)
    try:
        time.sleep(ORBIT_PULSE_SECONDS)
    finally:
        robot.stop()
    time.sleep(CAMERA_DELAY_SECONDS)

    after = None
    for _ in range(8):
        _frame, candidate = read_locked(video, lock)
        if candidate is not None:
            after = candidate
            break
        time.sleep(0.025)
    if after is None:
        raise LockedTargetLost(
            "target missing immediately after face-orbit strafe; refusing blind re-face"
        )
    after_bearing = target_bearing_left_deg(gaze, after, flip_x=flip_x)
    shift = after_bearing - before_bearing
    print(
        f"orbit physical bearing before={before_bearing:+.2f}deg "
        f"after={after_bearing:+.2f}deg shift={shift:+.2f}deg "
        f"direction={direction}",
        flush=True,
    )
    return gaze, shift


def align_block_face(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
) -> Gaze:
    """Reposition around the block without any pure lateral wheel command.

    The face estimate chooses a side.  Each attempt uses a rotate/back/restore
    dog-leg, re-faces the target, restores the 25.5 cm staging radius with only
    forward/backward motion, and then re-measures the face while stopped.
    Progress authority is the post-reposition face error itself.
    """
    current = measure_face_alignment(robot, video, lock, flip_x=flip_x)
    best_error = current.error_deg
    previous_error = current.error_deg
    stalled = 0
    side = "left" if current.edge_angle_deg >= 0.0 else "right"
    print(
        f"face-align sample=1 floor-edge={current.edge_angle_deg:+.1f}deg "
        f"square-error={current.error_deg:.1f}deg edge-length={current.edge_length_cm:.2f}cm",
        flush=True,
    )
    if current.error_deg <= FACE_ALIGNMENT_TOLERANCE_DEG:
        robot.stop()
        print("block face aligned: chassis centreline is perpendicular to the selected face", flush=True)
        return gaze
    if current.error_deg <= FACE_ALIGNMENT_ABORT_DEG:
        robot.stop()
        print(f"block face inside guarded grasp envelope: square-error={current.error_deg:.1f}deg", flush=True)
        return gaze

    initial_side = side
    for pulse_number in range(1, FACE_REPOSITION_MAX_PULSES + 1):
        print(
            f"face reposition={pulse_number}/{FACE_REPOSITION_MAX_PULSES} side={side} "
            f"error={current.error_deg:.1f}deg best={best_error:.1f}deg",
            flush=True,
        )
        face_reposition_dogleg(robot, lock, side=side)
        gaze = align_body_to_gaze(robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y)
        gaze = centre_gaze(
            robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
            confirmations=2, max_frames=20, allow_limit_lock=True,
        )
        staged = approach_with_locked_gaze(
            robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y,
            target_radius_cm=FACE_ALIGN_START_RADIUS_CM,
            allow_blind_arrival=False,
            allow_body_realign=False,
        )
        gaze = staged.gaze
        current = measure_face_alignment(robot, video, lock, flip_x=flip_x)
        improvement = previous_error - current.error_deg
        best_error = min(best_error, current.error_deg)
        print(
            f"face-align sample={pulse_number + 1} floor-edge={current.edge_angle_deg:+.1f}deg "
            f"square-error={current.error_deg:.1f}deg improvement={improvement:+.1f}deg "
            f"edge-length={current.edge_length_cm:.2f}cm",
            flush=True,
        )
        if current.error_deg <= FACE_ALIGNMENT_ABORT_DEG:
            robot.stop()
            return gaze
        expected_side = "left" if current.edge_angle_deg >= 0.0 else "right"
        if expected_side != initial_side:
            robot.stop()
            raise RuntimeError(
                f"face reposition crossed the selected face normal but remains "
                f"{current.error_deg:.1f}deg diagonal; refusing an opposite dog-leg"
            )
        if improvement < FACE_REPOSITION_MIN_IMPROVEMENT_DEG:
            stalled += 1
        else:
            stalled = 0
        if stalled >= FACE_REPOSITION_MAX_STALLED_PULSES:
            robot.stop()
            raise RuntimeError(
                f"base repositioning did not improve face alignment for {stalled} consecutive attempts; "
                f"current={current.error_deg:.1f}deg best={best_error:.1f}deg"
            )
        previous_error = current.error_deg

    robot.stop()
    raise RuntimeError(
        f"base repositioning exhausted {FACE_REPOSITION_MAX_PULSES} attempts; "
        f"current face error {current.error_deg:.1f}deg, best {best_error:.1f}deg"
    )


def _wrap_degrees_180(value: float) -> float:
    angle = float(value)
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def arm_face_signed_error_deg(edge_angle_deg: float, approach_bearing_deg: float) -> float:
    """Signed yaw still required for the arm ray to become face-normal.

    A square face edge has two equivalent normals 180 degrees apart.  Choose the
    one requiring the smaller arm-base yaw from the currently observed target
    bearing.  Positive means the target bearing must increase (move the chassis
    right so the cube appears further left); negative is the mirrored case.
    """
    normals = (float(edge_angle_deg) + 90.0, float(edge_angle_deg) - 90.0)
    errors = tuple(
        _wrap_degrees_180(normal - float(approach_bearing_deg))
        for normal in normals
    )
    return min(errors, key=abs)


def plan_face_normal_route(
    block: BlockEstimate,
    face: FaceAlignmentEstimate,
    *,
    reach_correction_cm: float = CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM,
    target_radius_cm: float = FACE_ROUTE_TARGET_RADIUS_CM,
    target_error_deg: float = FACE_ROUTE_TARGET_ERROR_DEG,
) -> FaceNormalRoutePlan:
    """Plan one direct side relocation from stopped camera/FK geometry.

    The old repeated dog-leg spent ~4.6 cm retreat to obtain only ~0.7 cm of
    lateral shift in the failing SIM seed.  Here the visible target vector and
    visible face normal define the desired robot position directly.  No SIM
    pose/object truth is used: range, target bearing and face edge all come from
    the same stopped eye-in-hand observation used on REAL.
    """
    corrected_radius = float(block.radius_cm) + float(reach_correction_cm)
    current_bearing = float(block.yaw_left_deg)
    signed_error = arm_face_signed_error_deg(face.edge_angle_deg, current_bearing)
    normal_bearing = _wrap_degrees_180(current_bearing + signed_error)
    if abs(signed_error) <= FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG:
        target_bearing = current_bearing
    else:
        target_bearing = _wrap_degrees_180(
            normal_bearing - math.copysign(float(target_error_deg), signed_error)
        )
    usable_bearing = max(
        0.0,
        float(ARM_FACE_MAX_BEARING_DEG) - float(FACE_ROUTE_ARM_BEARING_MARGIN_DEG),
    )
    if abs(target_bearing) > usable_bearing:
        projected_bearing = math.copysign(usable_bearing, target_bearing)
        projected_error = abs(
            arm_face_signed_error_deg(face.edge_angle_deg, projected_bearing)
        )
        if projected_error > FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG:
            raise RuntimeError(
                f"face normal needs arm bearing {target_bearing:+.1f}deg, but the "
                f"usable servo-6 corridor +/-{usable_bearing:.1f}deg would leave "
                f"{projected_error:.1f}deg face error"
            )
        target_bearing = projected_bearing
    current_yaw = math.radians(current_bearing)
    target_yaw = math.radians(target_bearing)
    current_left = corrected_radius * math.sin(current_yaw)
    current_forward = corrected_radius * math.cos(current_yaw)
    target_left = float(target_radius_cm) * math.sin(target_yaw)
    target_forward = float(target_radius_cm) * math.cos(target_yaw)
    # Robot translation is current target vector minus desired target vector.
    travel_left = current_left - target_left
    travel_forward = current_forward - target_forward
    distance = math.hypot(travel_left, travel_forward)
    travel_bearing = math.degrees(math.atan2(travel_left, travel_forward))
    travel_target_bearing = _wrap_degrees_180(current_bearing - travel_bearing)
    return FaceNormalRoutePlan(
        current_radius_cm=corrected_radius,
        current_bearing_deg=current_bearing,
        face_normal_bearing_deg=normal_bearing,
        target_radius_cm=float(target_radius_cm),
        target_bearing_deg=target_bearing,
        travel_bearing_deg=travel_bearing,
        travel_distance_cm=distance,
        travel_target_bearing_deg=travel_target_bearing,
    )



def _build_face_route_leg(
    *,
    current_radius_cm: float,
    current_bearing_deg: float,
    face_normal_bearing_deg: float,
    target_radius_cm: float,
    target_bearing_deg: float,
) -> FaceNormalRoutePlan:
    """Build one rotate+straight leg between two actor-visible target vectors."""
    current_yaw = math.radians(float(current_bearing_deg))
    target_yaw = math.radians(float(target_bearing_deg))
    current_left = float(current_radius_cm) * math.sin(current_yaw)
    current_forward = float(current_radius_cm) * math.cos(current_yaw)
    target_left = float(target_radius_cm) * math.sin(target_yaw)
    target_forward = float(target_radius_cm) * math.cos(target_yaw)
    travel_left = current_left - target_left
    travel_forward = current_forward - target_forward
    distance = math.hypot(travel_left, travel_forward)
    travel_bearing = math.degrees(math.atan2(travel_left, travel_forward))
    travel_target_bearing = _wrap_degrees_180(
        float(current_bearing_deg) - travel_bearing
    )
    return FaceNormalRoutePlan(
        current_radius_cm=float(current_radius_cm),
        current_bearing_deg=float(current_bearing_deg),
        face_normal_bearing_deg=float(face_normal_bearing_deg),
        target_radius_cm=float(target_radius_cm),
        target_bearing_deg=float(target_bearing_deg),
        travel_bearing_deg=travel_bearing,
        travel_distance_cm=distance,
        travel_target_bearing_deg=travel_target_bearing,
    )


def face_route_min_radius_cm(plan: FaceNormalRoutePlan) -> float:
    """Return the minimum target radius along one planned straight chord."""
    start_yaw = math.radians(float(plan.current_bearing_deg))
    end_yaw = math.radians(float(plan.target_bearing_deg))
    start = (
        float(plan.current_radius_cm) * math.sin(start_yaw),
        float(plan.current_radius_cm) * math.cos(start_yaw),
    )
    end = (
        float(plan.target_radius_cm) * math.sin(end_yaw),
        float(plan.target_radius_cm) * math.cos(end_yaw),
    )
    delta = (end[0] - start[0], end[1] - start[1])
    length_sq = delta[0] * delta[0] + delta[1] * delta[1]
    if length_sq <= 1e-9:
        return math.hypot(*start)
    t = -((start[0] * delta[0]) + (start[1] * delta[1])) / length_sq
    t = max(0.0, min(1.0, t))
    closest = (start[0] + t * delta[0], start[1] + t * delta[1])
    return math.hypot(*closest)


def guarded_face_route_legs(plan: FaceNormalRoutePlan) -> tuple[FaceNormalRoutePlan, ...]:
    """Split an inward-cutting diagonal into bounded rotate+straight legs."""
    minimum = face_route_min_radius_cm(plan)
    if minimum >= float(plan.target_radius_cm) - FACE_ROUTE_DIRECT_CHORD_EPSILON_CM:
        return (plan,)

    shift = _wrap_degrees_180(
        float(plan.target_bearing_deg) - float(plan.current_bearing_deg)
    )
    side_leg_count = max(
        1,
        int(math.ceil(abs(shift) / FACE_ROUTE_MAX_SIDE_LEG_DEG)),
    )
    legs: list[FaceNormalRoutePlan] = []
    current_bearing = float(plan.current_bearing_deg)
    for index in range(1, side_leg_count + 1):
        next_bearing = _wrap_degrees_180(
            float(plan.current_bearing_deg) + shift * index / side_leg_count
        )
        leg = _build_face_route_leg(
            current_radius_cm=plan.current_radius_cm,
            current_bearing_deg=current_bearing,
            face_normal_bearing_deg=plan.face_normal_bearing_deg,
            target_radius_cm=plan.current_radius_cm,
            target_bearing_deg=next_bearing,
        )
        safety_floor = FACE_ROUTE_TARGET_RADIUS_CM - FACE_ROUTE_MAX_RANGE_OVERSHOOT_CM
        leg_minimum = face_route_min_radius_cm(leg)
        if leg_minimum < safety_floor:
            raise RuntimeError(
                "face-route staging radius is too small for a guarded side leg; "
                f"planned minimum={leg_minimum:.1f}cm"
            )
        legs.append(leg)
        current_bearing = next_bearing

    if abs(float(plan.current_radius_cm) - float(plan.target_radius_cm)) > 1e-6:
        legs.append(
            _build_face_route_leg(
                current_radius_cm=plan.current_radius_cm,
                current_bearing_deg=plan.target_bearing_deg,
                face_normal_bearing_deg=plan.face_normal_bearing_deg,
                target_radius_cm=plan.target_radius_cm,
                target_bearing_deg=plan.target_bearing_deg,
            )
        )
    return tuple(legs)

def face_route_travel_endpoint_bearing_deg(
    plan: FaceNormalRoutePlan,
    achieved_travel_target_bearing_deg: float,
) -> float:
    """Target bearing expected at the planned endpoint in the travel frame.

    ``rotate_to_target_bearing`` is deliberately allowed a few degrees of
    temporary-heading tolerance.  Use the *achieved* target bearing after that
    turn to infer the actual chassis travel heading, then transform the planned
    final target bearing into that travel frame.  This keeps the translation
    endpoint actor-visible and avoids pretending that radial range alone fixes
    the robot's lateral coordinate.
    """
    achieved_travel_bearing = _wrap_degrees_180(
        float(plan.current_bearing_deg) - float(achieved_travel_target_bearing_deg)
    )
    return _wrap_degrees_180(
        float(plan.target_bearing_deg) - achieved_travel_bearing
    )


def face_route_translation_errors(
    plan: FaceNormalRoutePlan,
    current: BlockEstimate,
    *,
    achieved_travel_target_bearing_deg: float,
) -> tuple[float, float, float]:
    """Return radial remaining, travel-frame bearing error, endpoint bearing."""
    endpoint_bearing = face_route_travel_endpoint_bearing_deg(
        plan, achieved_travel_target_bearing_deg
    )
    return (
        float(current.radius_cm) - float(plan.target_radius_cm),
        _wrap_degrees_180(endpoint_bearing - float(current.yaw_left_deg)),
        endpoint_bearing,
    )


def rotate_to_target_bearing(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    desired_bearing_deg: float,
    flip_x: bool,
    flip_y: bool,
    tolerance_deg: float = FACE_ROUTE_BEARING_TOLERANCE_DEG,
) -> tuple[Gaze, float]:
    """Rotate chassis until the camera/FK target bearing reaches a setpoint."""
    previous_error: float | None = None
    for pulse in range(1, FACE_ROUTE_MAX_TURN_PULSES + 1):
        # Keep the cube observable while preserving bearing in servo+pixel form.
        gaze = centre_gaze(
            robot, video, lock, gaze,
            flip_x=flip_x, flip_y=flip_y,
            confirmations=1, max_frames=LOCK_HOLD_FRAMES + 20,
            allow_limit_lock=True,
        )
        _frame, blob = read_locked(video, lock)
        if blob is None:
            raise LockedTargetLost("target missing during face-route bearing turn")
        bearing = target_bearing_left_deg(gaze, blob, flip_x=flip_x)
        error = _wrap_degrees_180(float(desired_bearing_deg) - bearing)
        print(
            f"face-route turn={pulse}/{FACE_ROUTE_MAX_TURN_PULSES} "
            f"bearing={bearing:+.1f}deg target={float(desired_bearing_deg):+.1f}deg "
            f"error={error:+.1f}deg pan={gaze.pan}",
            flush=True,
        )
        if abs(error) <= float(tolerance_deg):
            robot.stop()
            return gaze, bearing
        if previous_error is not None and error * previous_error < 0.0:
            if abs(error) <= float(tolerance_deg) * 2.0:
                robot.stop()
                return gaze, bearing
            raise RuntimeError(
                f"face-route chassis turn crossed bearing setpoint by {error:+.1f}deg"
            )
        # Turning right increases target-bearing-left; turning left decreases it.
        direction = "rotate-right" if error > 0.0 else "rotate-left"
        duration = body_align_pulse_duration(error, BODY_ALIGN_RATE_PRIOR_DEG_S)
        note_motion = getattr(lock, "note_ego_motion", None)
        if callable(note_motion):
            note_motion()
        start_motion(robot, direction, TURN_SPEED)
        try:
            time.sleep(duration)
        finally:
            robot.stop()
        time.sleep(CAMERA_DELAY_SECONDS)
        previous_error = error

    # The pulse budget counts actuator commands, not observations. After the
    # final bounded turn, measure once more before declaring exhaustion; the old
    # loop could successfully enter tolerance on its last motion and then fail
    # without ever observing that result. This adds no extra chassis motion.
    gaze = centre_gaze(
        robot, video, lock, gaze,
        flip_x=flip_x, flip_y=flip_y,
        confirmations=1, max_frames=LOCK_HOLD_FRAMES + 20,
        allow_limit_lock=True,
    )
    _frame, blob = read_locked(video, lock)
    if blob is None:
        raise LockedTargetLost("target missing after final face-route bearing pulse")
    bearing = target_bearing_left_deg(gaze, blob, flip_x=flip_x)
    error = _wrap_degrees_180(float(desired_bearing_deg) - bearing)
    print(
        f"face-route turn=verify bearing={bearing:+.1f}deg "
        f"target={float(desired_bearing_deg):+.1f}deg error={error:+.1f}deg pan={gaze.pan}",
        flush=True,
    )
    if abs(error) <= float(tolerance_deg):
        robot.stop()
        return gaze, bearing
    if previous_error is not None and error * previous_error < 0.0 and abs(error) <= float(tolerance_deg) * 2.0:
        robot.stop()
        return gaze, bearing
    raise RuntimeError("face-route chassis turn exhausted bounded bearing pulses")


def face_route_corrected_range(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
) -> tuple[Gaze, BlockEstimate, object]:
    gaze, samples, blob = stationary_range_samples(
        robot, video, lock, gaze,
        count=3, flip_x=flip_x, flip_y=flip_y, image_center_nx=0.5,
    )
    if len(samples) < 2 or blob is None:
        raise RuntimeError(
            f"only {len(samples)} valid face-route range samples; refusing blind diagonal travel"
        )
    visual = median_block_estimate(samples)
    return gaze, apply_radial_reach_correction(
        visual, CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM
    ), blob



def _execute_face_route_leg(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    plan: FaceNormalRoutePlan,
    *,
    flip_x: bool,
    flip_y: bool,
    leg_number: int,
    leg_count: int,
) -> Gaze:
    """Execute one bounded rotate+straight leg and restore the chassis heading."""
    print(
        f"face-route leg={leg_number}/{leg_count} "
        f"target-bearing={plan.target_bearing_deg:+.1f}deg "
        f"target-radius={plan.target_radius_cm:.1f}cm "
        f"travel={plan.travel_distance_cm:.1f}cm at {plan.travel_bearing_deg:+.1f}deg",
        flush=True,
    )
    gaze, achieved_travel_target_bearing = rotate_to_target_bearing(
        robot, video, lock, gaze,
        desired_bearing_deg=plan.travel_target_bearing_deg,
        flip_x=flip_x, flip_y=flip_y,
        tolerance_deg=FACE_ROUTE_TRAVEL_BEARING_TOLERANCE_DEG,
    )
    endpoint_bearing = face_route_travel_endpoint_bearing_deg(
        plan, achieved_travel_target_bearing
    )
    print(
        f"face-route leg={leg_number}/{leg_count} heading accepted "
        f"target-bearing={achieved_travel_target_bearing:+.1f}deg; "
        f"endpoint-bearing={endpoint_bearing:+.1f}deg",
        flush=True,
    )
    previous_radius = float(plan.current_radius_cm)
    previous_bearing_error: float | None = None
    reached_endpoint = False
    safety_floor = FACE_ROUTE_TARGET_RADIUS_CM - FACE_ROUTE_MAX_RANGE_OVERSHOOT_CM
    radial_inward = plan.target_radius_cm < plan.current_radius_cm - 0.1
    for pulse in range(1, FACE_ROUTE_MAX_FORWARD_PULSES + 1):
        gaze, corrected, _ = face_route_corrected_range(
            robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y
        )
        remaining, bearing_error, endpoint_bearing = face_route_translation_errors(
            plan, corrected,
            achieved_travel_target_bearing_deg=achieved_travel_target_bearing,
        )
        print(
            f"face-route leg={leg_number}/{leg_count} "
            f"forward={pulse}/{FACE_ROUTE_MAX_FORWARD_PULSES} "
            f"corrected-radius={corrected.radius_cm:.2f}cm "
            f"remaining={remaining:+.2f}cm bearing={corrected.yaw_left_deg:+.1f}deg "
            f"endpoint-bearing={endpoint_bearing:+.1f}deg bearing-error={bearing_error:+.1f}deg",
            flush=True,
        )
        if corrected.radius_cm < safety_floor:
            raise RuntimeError(
                f"face-route passed safe radial envelope by "
                f"{FACE_ROUTE_TARGET_RADIUS_CM - corrected.radius_cm:.1f}cm "
                f"before reaching 2-D endpoint (bearing-error={bearing_error:+.1f}deg)"
            )
        range_ready = abs(remaining) <= FACE_ROUTE_RANGE_TOLERANCE_CM
        bearing_ready = abs(bearing_error) <= FACE_ROUTE_ENDPOINT_BEARING_TOLERANCE_DEG
        if range_ready and bearing_ready:
            reached_endpoint = True
            break
        if previous_bearing_error is not None and bearing_error * previous_bearing_error < 0.0:
            if range_ready and abs(bearing_error) <= 2.0 * FACE_ROUTE_ENDPOINT_BEARING_TOLERANCE_DEG:
                reached_endpoint = True
                break
            raise RuntimeError(
                f"face-route crossed 2-D endpoint bearing by {bearing_error:+.1f}deg "
                f"with radial remaining {remaining:+.1f}cm"
            )
        if radial_inward and corrected.radius_cm > previous_radius + RANGE_INCREASE_LIMIT_CM:
            raise RuntimeError(
                f"face-route forward motion increased range "
                f"{previous_radius:.2f}->{corrected.radius_cm:.2f}cm"
            )
        note_motion = getattr(lock, "note_ego_motion", None)
        if callable(note_motion):
            note_motion()
        start_motion(robot, "forward", MOTOR_SPEED)
        try:
            time.sleep(FACE_ROUTE_FORWARD_SECONDS)
        finally:
            robot.stop()
        time.sleep(CAMERA_DELAY_SECONDS)
        gaze = centre_gaze(
            robot, video, lock, gaze,
            flip_x=flip_x, flip_y=flip_y,
            confirmations=1, max_frames=LOCK_HOLD_FRAMES + 20,
            allow_limit_lock=True,
        )
        previous_radius = corrected.radius_cm
        previous_bearing_error = bearing_error

    if not reached_endpoint:
        gaze, corrected, _ = face_route_corrected_range(
            robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y
        )
        remaining, bearing_error, endpoint_bearing = face_route_translation_errors(
            plan, corrected,
            achieved_travel_target_bearing_deg=achieved_travel_target_bearing,
        )
        print(
            f"face-route leg={leg_number}/{leg_count} "
            f"forward=verify/{FACE_ROUTE_MAX_FORWARD_PULSES} "
            f"corrected-radius={corrected.radius_cm:.2f}cm remaining={remaining:+.2f}cm "
            f"bearing={corrected.yaw_left_deg:+.1f}deg endpoint-bearing={endpoint_bearing:+.1f}deg "
            f"bearing-error={bearing_error:+.1f}deg",
            flush=True,
        )
        if corrected.radius_cm < safety_floor:
            raise RuntimeError(
                f"face-route passed safe radial envelope by "
                f"{FACE_ROUTE_TARGET_RADIUS_CM - corrected.radius_cm:.1f}cm "
                f"before reaching 2-D endpoint (bearing-error={bearing_error:+.1f}deg)"
            )
        if not (
            abs(remaining) <= FACE_ROUTE_RANGE_TOLERANCE_CM
            and abs(bearing_error) <= FACE_ROUTE_ENDPOINT_BEARING_TOLERANCE_DEG
        ):
            raise RuntimeError(
                f"face-route 2-D translation exhausted bounded pulses: "
                f"remaining={remaining:+.1f}cm bearing-error={bearing_error:+.1f}deg"
            )

    gaze, _ = rotate_to_target_bearing(
        robot, video, lock, gaze,
        desired_bearing_deg=plan.target_bearing_deg,
        flip_x=flip_x, flip_y=flip_y,
    )
    return gaze

def reposition_to_face_normal_corridor(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
) -> tuple[Gaze, FaceAlignmentEstimate, float]:
    """Move to the block's side before close approach, without pure strafe.

    A geometrically safe direct chord stays one rotate+straight leg. If that
    chord would cut inside the final radial envelope, split it into short
    same-radius chords and one final radial leg. Every leg is still
    rotate -> straight forward -> visual bearing restore.
    """
    robot.stop()
    face = measure_face_alignment(robot, video, lock, flip_x=flip_x)
    gaze, corrected, blob = face_route_corrected_range(
        robot, video, lock, gaze, flip_x=flip_x, flip_y=flip_y
    )
    bearing = target_bearing_left_deg(gaze, blob, flip_x=flip_x)
    initial_error = abs(arm_face_signed_error_deg(face.edge_angle_deg, bearing))
    print(
        f"face-route staging corrected-radius={corrected.radius_cm:.2f}cm "
        f"bearing={bearing:+.1f}deg floor-edge={face.edge_angle_deg:+.1f}deg "
        f"arm-face-error={initial_error:.1f}deg",
        flush=True,
    )
    if initial_error <= FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG:
        return gaze, face, bearing

    plan = plan_face_normal_route(corrected, face, reach_correction_cm=0.0)
    if plan.travel_distance_cm > 25.0:
        raise RuntimeError(
            f"face-route requires {plan.travel_distance_cm:.1f}cm translation; "
            "refusing an unexpectedly large side relocation"
        )
    minimum = face_route_min_radius_cm(plan)
    legs = guarded_face_route_legs(plan)
    print(
        f"face-route plan normal={plan.face_normal_bearing_deg:+.1f}deg "
        f"target-bearing={plan.target_bearing_deg:+.1f}deg "
        f"target-radius={plan.target_radius_cm:.1f}cm "
        f"travel={plan.travel_distance_cm:.1f}cm at {plan.travel_bearing_deg:+.1f}deg "
        f"planned-min-radius={minimum:.1f}cm legs={len(legs)}",
        flush=True,
    )
    if len(legs) > 1:
        print(
            "face-route direct chord enters the final radial envelope; "
            "using guarded rotate+straight side legs before radial approach",
            flush=True,
        )

    for index, leg in enumerate(legs, start=1):
        if leg.travel_distance_cm > 25.0:
            raise RuntimeError(
                f"face-route leg {index} requires {leg.travel_distance_cm:.1f}cm translation; "
                "refusing an unexpectedly large relocation"
            )
        gaze = _execute_face_route_leg(
            robot,
            video,
            lock,
            gaze,
            leg,
            flip_x=flip_x,
            flip_y=flip_y,
            leg_number=index,
            leg_count=len(legs),
        )

    gaze = fine_align_horizontal(
        robot, video, lock, gaze, flip_x=flip_x, target_nx=0.5
    )
    final_face, final_bearing, final_signed, _ = measure_arm_face_alignment(
        robot, video, lock, gaze, flip_x=flip_x, target_nx=0.5
    )
    improvement = initial_error - abs(final_signed)
    print(
        f"face-route verified bearing={final_bearing:+.1f}deg "
        f"floor-edge={final_face.edge_angle_deg:+.1f}deg "
        f"error={abs(final_signed):.1f}deg improvement={improvement:+.1f}deg",
        flush=True,
    )
    if abs(final_signed) > FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG:
        raise RuntimeError(
            f"face-route ended {abs(final_signed):.1f}deg from face normal; "
            f"need <= {FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG:.1f}deg before close approach"
        )
    if improvement < FACE_ROUTE_MIN_IMPROVEMENT_DEG:
        raise RuntimeError(
            f"face-route improved face error by only {improvement:.1f}deg"
        )
    return gaze, final_face, final_bearing

def measure_arm_face_alignment(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    target_nx: float,
) -> tuple[FaceAlignmentEstimate, float, float, object]:
    """Measure block-face error relative to the ARM ray, not chassis forward."""
    chassis_face = measure_face_alignment(robot, video, lock, flip_x=flip_x)
    _frame, blob = read_locked(video, lock)
    if blob is None:
        raise LockedTargetLost("target missing while measuring arm-relative block face")
    bearing = target_bearing_left_deg(
        gaze,
        blob,
        flip_x=flip_x,
        image_center_nx=target_nx,
    )
    signed_error = arm_face_signed_error_deg(chassis_face.edge_angle_deg, bearing)
    arm_face = FaceAlignmentEstimate(
        edge_angle_deg=chassis_face.edge_angle_deg,
        error_deg=abs(signed_error),
        edge_length_cm=chassis_face.edge_length_cm,
    )
    return arm_face, bearing, signed_error, blob


def align_block_face_with_arm_yaw(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
    target_nx: float | None = None,
) -> tuple[Gaze, object, float]:
    """Move beside the cube without strafe, then point the arm ray at its face.

    Side placement is a bounded rotate/back/restore dog-leg.  Servo 6 then
    re-centres the cube and a straight-only hand-eye depth correction restores
    the early face-staging depth.  The chassis never receives ``left`` or
    ``right`` from this controller.
    """
    target_nx = CAPTURE_TARGET_NX if target_nx is None else float(target_nx)
    current, bearing, signed_error, blob = measure_arm_face_alignment(
        robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
    )
    best_error = current.error_deg
    previous_error = current.error_deg
    stalled = 0
    initial_sign = 1.0 if signed_error >= 0.0 else -1.0
    side = "right" if signed_error > 0.0 else "left"
    print(
        f"arm-face sample=1 floor-edge={current.edge_angle_deg:+.1f}deg "
        f"arm-bearing={bearing:+.1f}deg error={current.error_deg:.1f}deg pan={gaze.pan}",
        flush=True,
    )
    if current.error_deg <= FACE_ALIGNMENT_ABORT_DEG:
        robot.stop()
        return gaze, blob, bearing
    if abs(bearing) >= ARM_FACE_MAX_BEARING_DEG:
        raise RuntimeError(
            f"arm-base bearing already {bearing:+.1f}deg before face placement; "
            "refusing another base reposition near servo-6 limit"
        )

    for pulse_number in range(1, ARM_FACE_MAX_PULSES + 1):
        print(
            f"arm-face reposition={pulse_number}/{ARM_FACE_MAX_PULSES} side={side} "
            f"error={current.error_deg:.1f}deg arm-bearing={bearing:+.1f}deg",
            flush=True,
        )
        face_reposition_dogleg(robot, lock, side=side)
        # Keep chassis heading nominally restored; only servo 6 follows the cube.
        gaze = fine_align_horizontal(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx,
        )
        # The dog-leg intentionally backed away for clearance.  Recover only the
        # calibrated face-staging depth with straight forward/backward motion.
        gaze, blob = visual_capture_approach(
            robot, video, lock, gaze, flip_x=flip_x,
            target_nx=target_nx, target_ny=ARM_FACE_CAPTURE_TARGET_NY,
        )
        current, bearing, signed_error, blob = measure_arm_face_alignment(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx
        )
        improvement = previous_error - current.error_deg
        best_error = min(best_error, current.error_deg)
        print(
            f"arm-face sample={pulse_number + 1} floor-edge={current.edge_angle_deg:+.1f}deg "
            f"arm-bearing={bearing:+.1f}deg error={current.error_deg:.1f}deg "
            f"improvement={improvement:+.1f}deg pan={gaze.pan}",
            flush=True,
        )
        if abs(bearing) > ARM_FACE_MAX_BEARING_DEG:
            raise RuntimeError(
                f"arm-base bearing reached {bearing:+.1f}deg during face placement; "
                "refusing another base reposition"
            )
        if current.error_deg <= FACE_ALIGNMENT_ABORT_DEG:
            robot.stop()
            return gaze, blob, bearing
        sign = 1.0 if signed_error >= 0.0 else -1.0
        if sign != initial_sign:
            raise RuntimeError(
                f"arm-face placement crossed the selected face normal but remains "
                f"{current.error_deg:.1f}deg away; refusing an opposite base reposition"
            )
        expected_side = "right" if signed_error > 0.0 else "left"
        if expected_side != side:
            raise RuntimeError("arm-face placement would require reversing base reposition side")
        if improvement < ARM_FACE_MIN_IMPROVEMENT_DEG:
            stalled += 1
        else:
            stalled = 0
        if stalled >= ARM_FACE_MAX_STALLED_PULSES:
            raise RuntimeError(
                f"arm-relative face error did not improve for {stalled} consecutive base repositions; "
                f"current={current.error_deg:.1f}deg best={best_error:.1f}deg"
            )
        previous_error = current.error_deg

    raise RuntimeError(
        f"arm-relative face placement exhausted {ARM_FACE_MAX_PULSES} base repositions; "
        f"current={current.error_deg:.1f}deg best={best_error:.1f}deg"
    )


def stationary_range_samples(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    count: int,
    flip_x: bool,
    flip_y: bool,
    image_center_nx: float = 0.5,
) -> tuple[Gaze, list[BlockEstimate], object | None]:
    # Complete all tracking motion first.  Then freeze the camera pose and
    # discard delayed frames, so every blob is paired with the same servo/FK
    # values that physically produced that image.
    robot.stop()
    time.sleep(CAMERA_DELAY_SECONDS + SERVO_SETTLE_SECONDS)
    samples: list[BlockEstimate] = []
    last_blob = None
    attempts = 0
    while len(samples) < count and attempts < count * 4:
        attempts += 1
        _, blob = read_locked(video, lock)
        if blob is None:
            time.sleep(0.05)
            continue
        last_blob = blob
        estimate = estimate_block(
            robot.pose, blob, image_center_nx=image_center_nx
        )
        if estimate is not None:
            samples.append(estimate)
        time.sleep(0.02)
    return gaze, samples, last_blob


def predict_after_forward(
    measured: BlockEstimate,
    travel_cm: float,
) -> BlockEstimate:
    """Dead-reckon one short verified forward pulse after visual loss."""
    forward = max(0.1, measured.forward_cm - travel_cm)
    lateral = measured.lateral_left_cm
    radius = math.hypot(forward, lateral)
    yaw = math.degrees(math.atan2(lateral, forward))
    return BlockEstimate(
        radius_cm=radius,
        lateral_left_cm=lateral,
        forward_cm=forward,
        yaw_left_deg=yaw,
        camera_radius_cm=measured.camera_radius_cm,
        camera_height_cm=measured.camera_height_cm,
        ray_pitch_deg=measured.ray_pitch_deg,
        block_height_cm=measured.block_height_cm,
        nx=0.5,
        ny=measured.ny,
    )


def visual_blind_estimate(
    robot: Robot,
    gaze: Gaze,
    blob,
    *,
    flip_x: bool,
) -> BlockEstimate:
    nx = 1.0 - blob.nx if flip_x else blob.nx
    pixel_left_deg = (0.5 - nx) * CAMERA_HFOV_DEG
    servo_left_deg = (gaze.pan - BASE_CENTER) / PULSE_PER_DEGREE
    yaw_left_deg = servo_left_deg + pixel_left_deg
    yaw = math.radians(yaw_left_deg)
    camera = forward_kinematics(robot.pose, CAMERA_LINK_CM)
    visual_height = (
        blob.height
        / VISUAL_REFERENCE_HEIGHT_PX
        * VISUAL_REFERENCE_BLOCK_HEIGHT_CM
    )
    visual_height = max(0.8, min(10.0, visual_height))
    return BlockEstimate(
        radius_cm=STOP_RADIUS_CM,
        lateral_left_cm=STOP_RADIUS_CM * math.sin(yaw),
        forward_cm=STOP_RADIUS_CM * math.cos(yaw),
        yaw_left_deg=yaw_left_deg,
        camera_radius_cm=camera.radius_cm,
        camera_height_cm=camera.height_cm + CAMERA_Z_OFFSET_CM,
        ray_pitch_deg=camera.pitch_deg,
        block_height_cm=visual_height,
        nx=0.5,
        ny=blob.ny,
    )


def _confirm_range_jump(
    robot,
    video,
    lock,
    gaze,
    flip_x: bool,
    flip_y: bool,
    previous_radius: float,
    limit_cm: float,
    *,
    stable_forward_floor_cm: float | None = None,
) -> float | None:
    """Re-sample a refused range jump exactly once.

    Mid-run transient triplets (settle blur, far floor-ray noise) can agree
    with each other yet disagree with truth, then self-resolve one sample set
    later (observed: 68.96cm vs 90.9cm truth, final frame re-read 87.7cm).
    Returns the confirmed radius when re-measurement lands within limit of the
    previous accepted radius.  For a forward pulse, the caller may additionally
    accept a stable monotonic decrease that exceeds the duration model while it
    remains outside a hard safety floor.  This keeps the model as an anomaly
    trigger rather than incorrectly treating it as odometry.  Bounded: one
    extra sample set, never a loop.
    """
    try:
        _gaze, confirm_samples, _blob = stationary_range_samples(
            robot, video, lock, gaze,
            count=3, flip_x=flip_x, flip_y=flip_y,
        )
    except Exception:
        return None
    if len(confirm_samples) < 2:
        return None
    radii = [sample.radius_cm for sample in confirm_samples]
    median = statistics.median(radii)
    if median_absolute_deviation(radii) > 1.5:
        return None
    if abs(median - previous_radius) <= limit_cm:
        print(
            f"range jump transient: re-measured {median:.2f}cm within "
            f"{limit_cm:.2f}cm of {previous_radius:.2f}cm; continuing",
            flush=True,
        )
        return median
    if (
        stable_forward_floor_cm is not None
        and median < previous_radius
        and median >= float(stable_forward_floor_cm)
    ):
        print(
            f"range jump stable-forward override: {previous_radius:.2f}->{median:.2f}cm "
            f"exceeds duration model {limit_cm:.2f}cm but remains above "
            f"safety floor {float(stable_forward_floor_cm):.2f}cm; continuing",
            flush=True,
        )
        return median
    return None


def approach_with_locked_gaze(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
    target_radius_cm: float = STOP_RADIUS_CM,
    allow_blind_arrival: bool = True,
    allow_body_realign: bool = True,
    allow_continuous_far: bool = True,
    max_motion_steps: int = 100,
    gaze_mode: str = GAZE_MODE_FIXATE,
    pursue_state: PursueState | None = None,
) -> ApproachResult:
    gaze_mode = normalize_gaze_mode(gaze_mode)
    if gaze_mode != GAZE_MODE_FIXATE and pursue_state is None:
        pursue_state = PursueState()
    previous_radius = None
    previous_motion_duration = None
    previous_motion_kind: str | None = None
    previous_pan_pulse: float | None = None
    previous_tilt_pulse: float | None = None
    previous_source: str | None = None
    previous_ray_pitch: float | None = None
    previous_camh: float | None = None
    visual_fallback_steps = 0
    max_motion_steps = max(1, int(max_motion_steps))
    # ``max_motion_steps`` limits actuator pulses, not observations. Always
    # take one final stopped measurement after the last allowed pulse; the old
    # loop performed pulse N and then raised without checking whether that
    # pulse had actually entered the target band.
    for approach_step in range(1, max_motion_steps + 2):
        hard_pan_boundary = (
            gaze.pan <= PAN_MIN + PAN_LIMIT_MARGIN
            or gaze.pan >= PAN_MAX - PAN_LIMIT_MARGIN
        )
        legacy_body_realign = abs(gaze.pan - BASE_CENTER) > BODY_REALIGN_LIMIT
        if hard_pan_boundary:
            if not allow_body_realign:
                robot.stop()
                raise RuntimeError(
                    f"arm-only near approach reached physical pan boundary ({gaze.pan}); "
                    "refusing chassis yaw"
                )
            print(
                f"gaze nearing pan limit ({gaze.pan}); verify clearance before body realign",
                flush=True,
            )
            gaze = retreat_for_body_alignment(
                robot, video, lock, gaze, flip_y=flip_y
            )
            gaze = align_body_to_gaze(
                robot,
                video,
                lock,
                gaze,
                flip_x=flip_x,
                flip_y=flip_y,
            )
        elif legacy_body_realign:
            print(
                f"approach keeps chassis heading fixed at pan={gaze.pan}; "
                "camera/servo tracks residual yaw until a physical pan boundary",
                flush=True,
            )

        gaze, samples, blob = stationary_range_samples(
            robot,
            video,
            lock,
            gaze,
            count=APPROACH_RANGE_SAMPLE_COUNT,
            flip_x=flip_x,
            flip_y=flip_y,
        )
        if len(samples) < APPROACH_RANGE_MIN_VALID:
            visual_ny = None
            if blob is not None:
                visual_ny = 1.0 - blob.ny if flip_y else blob.ny
            visually_far = (
                blob is not None
                and visual_ny is not None
                and visual_ny < VISUAL_FAR_MAX_NY
                and blob.area < VISUAL_FAR_MAX_AREA
                and blob.height < VISUAL_FAR_MAX_HEIGHT
            )
            if visually_far and visual_fallback_steps < VISUAL_FALLBACK_MAX_STEPS:
                visual_fallback_steps += 1
                previous_radius = None
                previous_motion_duration = None
                previous_motion_kind = None
                previous_pan_pulse = None
                previous_tilt_pulse = None
                previous_source = None
                previous_ray_pitch = None
                previous_camh = None
                print(
                    "metric range temporarily invalid; use verified far-view "
                    f"fallback {visual_fallback_steps}/{VISUAL_FALLBACK_MAX_STEPS} "
                    f"ny={visual_ny:.3f} area={blob.area} height={blob.height}",
                    flush=True,
                )
                try:
                    gaze = motion_pulse_with_gaze(
                        robot,
                        video,
                        lock,
                        gaze,
                        kind="forward",
                        speed=MOTOR_SPEED,
                        duration=VISUAL_FALLBACK_SECONDS,
                        flip_x=flip_x,
                        flip_y=flip_y,
                        gaze_mode=gaze_mode,
                        pursue_state=pursue_state,
                    )
                except LockedTargetLost:
                    robot.stop()
                    if not allow_blind_arrival:
                        raise RuntimeError(
                            "target lost before the face-alignment staging "
                            "radius; refusing to orbit without vision"
                        )
                    visually_near = (
                        blob.height >= VISUAL_BLIND_MIN_HEIGHT
                        or visual_ny >= VISUAL_BLIND_MIN_NY
                    )
                    if not visually_near:
                        raise RuntimeError(
                            "target lost during far visual approach; "
                            "refusing a blind grasp"
                        )
                    predicted = visual_blind_estimate(
                        robot,
                        gaze,
                        blob,
                        flip_x=flip_x,
                    )
                    print(
                        "VISUAL BLIND ARRIVAL LATCHED: close block left "
                        f"the camera; radius={predicted.radius_cm:.2f}cm",
                        flush=True,
                    )
                    return ApproachResult(
                        gaze=gaze,
                        block=predicted,
                        target_still_visible=False,
                    )
                continue
            pose_text = ",".join(
                f"{servo}:{robot.pose.get(servo)}" for servo in (3, 4, 5, 6)
            )
            blob_text = (
                "none"
                if blob is None
                else f"ny={visual_ny:.3f},area={blob.area},height={blob.height}"
            )
            raise RuntimeError(
                f"insufficient valid range samples ({len(samples)}/3); "
                f"pose=[{pose_text}] blob=[{blob_text}]"
            )
        visual_fallback_steps = 0
        radii = [sample.radius_cm for sample in samples]
        radius = statistics.median(radii)
        radius_mad = median_absolute_deviation(radii)
        if radius_mad > 1.5:
            raise RuntimeError(
                f"unstable approach range (MAD={radius_mad:.2f}cm); "
                "wheels remain stopped"
            )
        step_sources = [sample.source for sample in samples]
        step_source = max(set(step_sources), key=step_sources.count)
        step_blob = (
            f"nx={blob.nx:.3f} ny={blob.ny:.3f} w={blob.width} h={blob.height}"
            if blob is not None else "blob=none"
        )
        print(
            f"approach step={approach_step} stationary median={radius:.2f}cm "
            f"MAD={radius_mad:.2f}cm pan={gaze.pan} tilt={gaze.tilt} samples={len(samples)} "
            f"src={step_source} {step_blob}",
            flush=True,
        )
        # Track every sample-time viewpoint, even the first uncompared one:
        # the pulse recenters pan before the next sample, so the gate must
        # compare consecutive SAMPLE viewpoints, not post-pulse ones.
        pan_now = float(gaze.pan)
        pan_delta_deg = (
            abs(pan_now - previous_pan_pulse) / PULSE_PER_DEGREE
            if previous_pan_pulse is not None else 0.0
        )
        previous_pan_pulse = pan_now
        tilt_now = float(gaze.tilt)
        tilt_delta_deg = (
            abs(tilt_now - previous_tilt_pulse) / PULSE_PER_DEGREE
            if previous_tilt_pulse is not None else 0.0
        )
        previous_tilt_pulse = tilt_now
        # Floor triangulation breathes analytically with ray pitch:
        # d(forward)/d(pitch) = -h/sin^2(p).  The drift acts on the current
        # floor reading whatever the previous instrument was (size samples
        # carry the same ray-pitch/cam-height fields), so only the current
        # source gates the analytic term.
        if (
            step_source == "floor_ray"
            and previous_ray_pitch is not None
            and previous_camh is not None
        ):
            _p = max(5.0, abs(previous_ray_pitch))
            tilt_extra_cm = (
                previous_camh
                * math.radians(tilt_delta_deg)
                / (math.sin(math.radians(_p)) ** 2)
            )
        else:
            tilt_extra_cm = (
                VIEWPOINT_TILT_SLOPE_PER_DEG * (previous_radius or radius) * tilt_delta_deg
            )
        previous_source = step_source
        previous_ray_pitch = statistics.median(
            sample.ray_pitch_deg for sample in samples
        )
        previous_camh = statistics.median(
            sample.camera_height_cm for sample in samples
        )
        if previous_radius is not None and previous_motion_duration is not None:
            change = radius - previous_radius
            viewpoint_extra = min(
                VIEWPOINT_EXTRA_CAP_CM,
                VIEWPOINT_PAN_SLOPE_PER_DEG * previous_radius * pan_delta_deg
                + tilt_extra_cm,
            )
            if previous_motion_kind == "backward":
                allowed_rise = allowed_range_rise_cm(previous_motion_duration)
                allowed_rise += viewpoint_extra
                print(
                    f"range delta={change:+.2f}cm after backward "
                    f"{previous_motion_duration:.2f}s pulse "
                    f"(allowed -{RANGE_INCREASE_LIMIT_CM:.2f}..+{allowed_rise:.2f}cm) "
                    f"pan={gaze.pan} dpan={pan_delta_deg:.1f}deg tilt={gaze.tilt} dtilt={tilt_delta_deg:.1f}deg",
                    flush=True,
                )
                if change < -RANGE_INCREASE_LIMIT_CM or change > allowed_rise:
                    confirmed = _confirm_range_jump(
                        robot, video, lock, gaze, flip_x, flip_y,
                        previous_radius, max(RANGE_INCREASE_LIMIT_CM, allowed_rise),
                    )
                    if confirmed is not None:
                        radius = confirmed
                    else:
                        raise RuntimeError(
                            f"range changed implausibly from {previous_radius:.2f}cm "
                            f"to {radius:.2f}cm after a backward "
                            f"{previous_motion_duration:.2f}s pulse; "
                            f"allowed increase is {allowed_rise:.2f}cm; "
                            "refusing another wheel pulse"
                        )
            else:
                allowed_drop = allowed_range_drop_cm(previous_motion_duration)
                allowed_drop += viewpoint_extra
                print(
                    f"range delta={change:+.2f}cm after forward "
                    f"{previous_motion_duration:.2f}s pulse "
                    f"(allowed -{allowed_drop:.2f}..+{RANGE_INCREASE_LIMIT_CM:.2f}cm) "
                    f"pan={gaze.pan} dpan={pan_delta_deg:.1f}deg tilt={gaze.tilt} dtilt={tilt_delta_deg:.1f}deg",
                    flush=True,
                )
                if change > RANGE_INCREASE_LIMIT_CM or change < -allowed_drop:
                    confirmed = _confirm_range_jump(
                        robot, video, lock, gaze, flip_x, flip_y,
                        previous_radius, max(RANGE_INCREASE_LIMIT_CM, allowed_drop),
                        stable_forward_floor_cm=CHASSIS_HARD_LIMIT_CM + 1.0,
                    )
                    if confirmed is not None:
                        radius = confirmed
                    else:
                        raise RuntimeError(
                            f"range changed implausibly from {previous_radius:.2f}cm "
                            f"to {radius:.2f}cm after a forward "
                            f"{previous_motion_duration:.2f}s pulse; "
                            f"allowed decrease is {allowed_drop:.2f}cm; "
                            "refusing another wheel pulse"
                        )
        print(
            f"approach step={approach_step} accepted radius={radius:.2f}cm; "
            f"target={target_radius_cm:.2f}cm",
            flush=True,
        )
        # Coarse staging is a band, not a one-sided threshold. Being inside a
        # requested face-orbit radius is an overshoot and must be corrected by
        # a bounded backward pulse before any lateral orbit.
        tolerance = (
            FINAL_APPROACH_TOLERANCE_CM
            if target_radius_cm <= STOP_RADIUS_CM + 1e-6
            else COARSE_RADIUS_TOLERANCE_CM
        )
        error = radius - target_radius_cm
        if abs(error) <= tolerance:
            robot.stop()
            print(
                f"requested approach radius {target_radius_cm:.2f}cm reached "
                f"within ±{tolerance:.2f}cm tolerance; chassis remains stopped",
                flush=True,
            )
            return ApproachResult(
                gaze=gaze,
                block=median_block_estimate(samples),
                target_still_visible=True,
            )

        if approach_step > max_motion_steps:
            break

        motion_kind = "forward" if error > 0.0 else "backward"
        if motion_kind == "forward" and radius <= CHASSIS_HARD_LIMIT_CM:
            raise RuntimeError("chassis safety radius violated; refusing further forward motion")

        remaining = abs(error) - tolerance
        if (
            motion_kind == "forward"
            and allow_continuous_far
            and should_use_continuous_far_approach(radius, target_radius_cm)
        ):
            stream = continuous_far_approach(
                robot,
                video,
                lock,
                gaze,
                target_radius_cm=target_radius_cm,
                flip_x=flip_x,
                flip_y=flip_y,
            )
            gaze = stream.gaze
            previous_radius = radius
            previous_motion_duration = max(0.01, stream.duration_s)
            previous_motion_kind = "forward"
            continue
        if motion_kind == "backward":
            speed = MOTOR_SPEED
            if remaining > 5.0:
                duration = BACKWARD_FAR_SECONDS
            elif remaining > 2.0:
                duration = BACKWARD_NEAR_SECONDS
            else:
                duration = BACKWARD_FINE_SECONDS
            print(
                f"coarse staging overshoot: radius={radius:.2f}cm "
                f"target={target_radius_cm:.2f}cm -> {duration:.2f}s backward pulse",
                flush=True,
            )
        elif radius > 28.0:
            speed = FAR_FORWARD_SPEED
            # The mined gain table describes ROLLING pulses only: the first
            # pulse from standstill moved 2x the model (SIM: 25 cm in 0.8 s
            # vs 15 cm/s model) and tripped the plausibility gate, so the
            # first pulse of a run stays at the proven fixed value.
            if previous_motion_duration is not None:
                predicted = float(remaining) / PREDICTIVE_FORWARD_CM_S
                duration = min(
                    PREDICTIVE_FAR_PULSE_CAP_S,
                    max(FAR_FORWARD_SECONDS, predicted),
                )
            else:
                duration = FAR_FORWARD_SECONDS
            if duration > FAR_FORWARD_SECONDS:
                print(
                    f"predictive far pulse: remaining={remaining:.2f}cm "
                    f"-> {duration:.2f}s forward pulse "
                    f"(model {PREDICTIVE_FORWARD_CM_S:.1f}cm/s, "
                    f"floor {FAR_FORWARD_SECONDS:.2f}s cap "
                    f"{PREDICTIVE_FAR_PULSE_CAP_S:.2f}s)",
                    flush=True,
                )
        elif radius > 20.0:
            speed, duration = MOTOR_SPEED, MID_FORWARD_SECONDS
        elif remaining <= FINAL_CREEP_WINDOW_CM:
            speed, duration = NEAR_FORWARD_SPEED, FINAL_CREEP_SECONDS
            print(
                f"final creep: remaining={remaining:.2f}cm -> {duration:.2f}s forward pulse",
                flush=True,
            )
        else:
            speed, duration = NEAR_FORWARD_SPEED, NEAR_FORWARD_SECONDS
        measured_before_motion = median_block_estimate(samples)
        try:
            gaze = motion_pulse_with_gaze(
                robot,
                video,
                lock,
                gaze,
                kind=motion_kind,
                speed=speed,
                duration=duration,
                flip_x=flip_x,
                flip_y=flip_y,
                gaze_mode=gaze_mode,
                pursue_state=pursue_state,
            )
        except LockedTargetLost:
            robot.stop()
            if motion_kind == "backward":
                raise RuntimeError(
                    "target lost while backing out to the requested staging radius; "
                    "refusing blind retreat/re-alignment"
                )
            if not allow_blind_arrival:
                raise RuntimeError(
                    "target lost before the face-alignment staging radius; "
                    "refusing to orbit without vision"
                )
            predicted = predict_after_forward(
                measured_before_motion,
                FORWARD_PULSE_TRAVEL_EST_CM,
            )
            # The target can disappear *during* the final forward pulse.  The
            # old gate compared the pre-pulse radius and therefore rejected a
            # valid arrival merely because the previous frame was just outside
            # the blind-latch boundary.  Keep the same safety boundary, but
            # apply it to the controller's conservative post-pulse prediction.
            if predicted.radius_cm > BLIND_LATCH_MAX_RADIUS_CM:
                raise RuntimeError(
                    f"target lost with predicted radius {predicted.radius_cm:.2f}cm "
                    f"(last visible {radius:.2f}cm); refusing a blind grasp"
                )
            print(
                "BLIND ARRIVAL LATCHED: target left the camera at close "
                f"range; predicted radius={predicted.radius_cm:.2f}cm "
                f"left={predicted.lateral_left_cm:+.2f}cm",
                flush=True,
            )
            return ApproachResult(
                gaze=gaze,
                block=predicted,
                target_still_visible=False,
            )
        previous_radius = radius
        previous_motion_duration = duration
        previous_motion_kind = motion_kind
    raise RuntimeError(
        f"approach exceeded {max_motion_steps} measured motion pulses"
    )


def median_block_estimate(samples: list[BlockEstimate]) -> BlockEstimate:
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


def final_x_step_limit(error: float) -> int:
    """Return a bounded pan-step cap that tapers into the precision band.

    The requested step is still produced by the existing image-error
    proportional term and all existing servo limits/final confirmations remain
    authoritative. Only the globally-small output clamp becomes error dependent.
    """
    magnitude = abs(float(error))
    if magnitude > FINAL_X_VERY_COARSE_ERROR:
        return FINAL_X_VERY_COARSE_MAX_STEP_PULSE
    if magnitude > FINAL_X_COARSE_ERROR:
        return FINAL_X_COARSE_MAX_STEP_PULSE
    if magnitude > FINAL_X_FINE_ERROR:
        return FINAL_X_FINE_STEP_PULSE
    return FINAL_X_PRECISION_STEP_PULSE


def fine_align_horizontal(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    target_nx: float = 0.5,
    confirmations: int | None = None,
) -> Gaze:
    """Micro-align servo 6 to an explicit image-space target x."""
    required = FINAL_X_CONFIRMATIONS if confirmations is None else max(1, int(confirmations))
    stable = 0
    missing = 0
    previous_error: float | None = None
    last_command_nx: float | None = None
    wait_for_response = False
    for iteration in range(1, FINAL_X_MAX_ITERATIONS + 1):
        _, blob = read_locked(video, lock)
        if blob is None:
            missing += 1
            robot.stop()
            if missing >= LOCK_HOLD_FRAMES:
                raise LockedTargetLost(
                    "target disappeared during final horizontal alignment"
                )
            time.sleep(0.04)
            continue
        missing = 0
        nx = 1.0 - blob.nx if flip_x else blob.nx
        error = float(target_nx) - nx
        if abs(error) <= FINAL_X_DEADBAND:
            stable += 1
            wait_for_response = False
            print(
                f"fine-x iteration={iteration} nx={nx:.3f} "
                f"target-nx={float(target_nx):.3f} pan={gaze.pan} "
                f"stable={stable}/{required}",
                flush=True,
            )
            if stable >= required:
                return gaze
            time.sleep(0.025)
            continue

        stable = 0
        # Do not stack another correction on top of a command whose visual
        # response has not arrived yet.  The failed REAL trace showed two +48
        # pulses against the same nx=0.348 frame, which guaranteed overshoot.
        if (
            wait_for_response
            and last_command_nx is not None
            and abs(nx - last_command_nx) < FINAL_X_RESPONSE_EPSILON
        ):
            print(
                f"fine-x iteration={iteration} nx={nx:.3f} waiting-for-pan-response",
                flush=True,
            )
            wait_for_response = False
            time.sleep(CAMERA_DELAY_SECONDS)
            previous_error = error
            continue

        crossed = previous_error is not None and error * previous_error < 0.0
        gain = 0.30 if crossed else (0.45 if abs(error) <= FINAL_X_COARSE_ERROR else 0.55)
        requested = int(
            round(error * CAMERA_HFOV_DEG * PULSE_PER_DEGREE * gain)
        )
        if requested == 0:
            requested = 1 if error > 0.0 else -1
        step_limit = final_x_step_limit(error)
        if crossed:
            step_limit = min(step_limit, FINAL_X_OVERSHOOT_STEP_PULSE)
        delta = clamp(requested, -step_limit, step_limit)
        new_pan = clamp(gaze.pan + delta, PAN_MIN, PAN_MAX)
        if new_pan == gaze.pan:
            raise RuntimeError(
                "servo 6 reached its safe limit during final x alignment"
            )
        gaze = Gaze(pan=new_pan, tilt=gaze.tilt)
        robot.nudge_servos({PAN_SERVO: new_pan}, duration=0.10)
        last_command_nx = nx
        wait_for_response = True
        previous_error = error
        print(
            f"fine-x iteration={iteration} nx={nx:.3f} "
            f"target-nx={float(target_nx):.3f} pan-step={delta:+d} "
            f"limit={step_limit} -> {new_pan}",
            flush=True,
        )
        time.sleep(CAMERA_DELAY_SECONDS)
    raise RuntimeError("final horizontal alignment did not converge")


def final_measurement(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    flip_y: bool,
    expected_radius_cm: float,
) -> tuple[Gaze, BlockEstimate]:
    robot.stop()
    gaze = centre_gaze(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        flip_y=flip_y,
        confirmations=CENTER_CONFIRMATIONS,
        max_frames=60,
        # At final close range the floor target can sit below image centre while
        # tilt is already at its safe minimum.  Accept that saturated vertical
        # axis; fine_align_horizontal below still tightens the grasp-critical x.
        allow_limit_lock=True,
    )
    gaze = fine_align_horizontal(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
    )
    # Freeze the camera pose, wait out transport/servo delay, then sample.
    robot.stop()
    time.sleep(CAMERA_DELAY_SECONDS + SERVO_SETTLE_SECONDS)
    samples: list[BlockEstimate] = []
    last_frame = None
    last_blob = None
    for _ in range(FINAL_SAMPLE_COUNT * 3):
        frame, blob = read_locked(video, lock)
        if blob is None:
            continue
        estimate = estimate_block(robot.pose, blob)
        if estimate is None:
            continue
        samples.append(estimate)
        last_frame, last_blob = frame, blob
        if len(samples) >= FINAL_SAMPLE_COUNT:
            break
        time.sleep(0.025)
    if len(samples) < FINAL_MIN_VALID_SAMPLES:
        raise RuntimeError(
            f"only {len(samples)} valid final samples; refusing blind grasp"
        )
    result = median_block_estimate(samples)
    radius_mad = median_absolute_deviation(
        [item.radius_cm for item in samples]
    )
    lateral_mad = median_absolute_deviation(
        [item.lateral_left_cm for item in samples]
    )
    if radius_mad > 0.8 or lateral_mad > FINAL_LATERAL_MAD_MAX_CM:
        raise RuntimeError(
            f"final coordinate is unstable (range MAD={radius_mad:.2f}cm, "
            f"lateral MAD={lateral_mad:.2f}cm); refusing blind grasp"
        )
    if abs(result.radius_cm - expected_radius_cm) > 1.2:
        raise RuntimeError(
            f"final range {result.radius_cm:.2f}cm disagrees with stopped "
            f"approach range {expected_radius_cm:.2f}cm; refusing blind grasp"
        )
    vertical_limit_lock = (
        (gaze.tilt <= TILT_MIN and result.ny > 0.5)
        or (gaze.tilt >= TILT_MAX and result.ny < 0.5)
    )
    if (
        abs(result.nx - 0.5) > FINAL_X_DRIFT_LIMIT
        or (abs(result.ny - 0.5) > 0.08 and not vertical_limit_lock)
    ):
        raise RuntimeError(
            f"target drifted after gaze lock (nx={result.nx:.3f}, "
            f"ny={result.ny:.3f}); refusing blind grasp"
        )
    if last_frame is not None:
        save_debug_frame(last_frame, last_blob, DEBUG_PATH)
    print(
        "final block coordinate: "
        f"left={result.lateral_left_cm:+.2f}cm "
        f"forward={result.forward_cm:.2f}cm "
        f"radius={result.radius_cm:.2f}cm "
        f"block-height={result.block_height_cm:.2f}cm "
        f"yaw-left={result.yaw_left_deg:+.2f}deg "
        f"range-MAD={radius_mad:.2f}cm lateral-MAD={lateral_mad:.2f}cm "
        f"samples={len(samples)}",
        flush=True,
    )
    return gaze, result


def apply_radial_reach_correction(
    block: BlockEstimate,
    correction_cm: float,
) -> BlockEstimate:
    """Move a visual block estimate outward along its observed arm ray.

    The existing capture calibration separates visual floor-ray radius from the
    physical gripper reach.  After final tangent placement we retain the measured
    bearing and apply that same longitudinal correction radially, rather than
    pretending the chassis is still on its original centreline.
    """
    radius = float(block.radius_cm) + float(correction_cm)
    yaw = math.radians(float(block.yaw_left_deg))
    return BlockEstimate(
        radius_cm=radius,
        lateral_left_cm=radius * math.sin(yaw),
        forward_cm=radius * math.cos(yaw),
        yaw_left_deg=block.yaw_left_deg,
        camera_radius_cm=block.camera_radius_cm,
        camera_height_cm=block.camera_height_cm,
        ray_pitch_deg=block.ray_pitch_deg,
        block_height_cm=block.block_height_cm,
        nx=block.nx,
        ny=block.ny,
    )


def capture_block_estimate(
    gaze: Gaze,
    blob,
    *,
    block_radius_cm: float | None = None,
) -> BlockEstimate:
    """Build a grasp target from the calibrated fixed capture window.

    Do not feed the near-field camera ray range back into the blind grasp.  At
    this point the chassis has visually staged the cube at a repeatable image
    location with servos 3/4/5 fixed.  Only horizontal yaw comes from vision;
    forward radius is the calibrated MasterPi capture radius.
    """
    # Capture x is an explicit hand-eye calibration target.  Do not assume the
    # raw camera principal point is normalized image centre: measured fisheye
    # cameras can have cx != width/2, and SIM may patch CAPTURE_TARGET_NX to that
    # measured optical centre without changing the REAL default.
    pixel_left_deg = (CAPTURE_TARGET_NX - float(blob.nx)) * CAMERA_HFOV_DEG
    servo_left_deg = (int(gaze.pan) - BASE_CENTER) / PULSE_PER_DEGREE
    yaw_left_deg = servo_left_deg + pixel_left_deg
    yaw = math.radians(yaw_left_deg)
    block_radius = (
        CAPTURE_BLOCK_RADIUS_CM
        if block_radius_cm is None
        else float(block_radius_cm)
    )
    return BlockEstimate(
        radius_cm=block_radius,
        lateral_left_cm=block_radius * math.sin(yaw),
        forward_cm=block_radius * math.cos(yaw),
        yaw_left_deg=yaw_left_deg,
        camera_radius_cm=0.0,
        camera_height_cm=0.0,
        ray_pitch_deg=0.0,
        block_height_cm=DEFAULT_BLOCK_HEIGHT_CM,
        nx=float(blob.nx),
        ny=float(blob.ny),
    )


def make_capture_pick_plan(
    robot: Robot,
    gaze: Gaze,
    blob,
    *,
    block_radius_cm: float | None = None,
) -> PickPlan:
    """Create the blind-grasp plan, optionally after a final tangent side step.

    With no override this preserves the calibrated fixed capture radius exactly.
    A caller that translated laterally *after* reaching the capture window may
    provide the geometrically updated radial distance while preserving the same
    calibrated forward component.
    """
    block = capture_block_estimate(gaze, blob, block_radius_cm=block_radius_cm)
    plan = calculate_pick_plan(robot, gaze, block, capture_profile=True)
    if block_radius_cm is None and abs(plan.fingertip_radius_cm - CAPTURE_FINGERTIP_RADIUS_CM) > 1e-6:
        raise RuntimeError("capture grasp radius calibration drifted")
    return plan


def precapture_creep_seconds(ny: float) -> float:
    """Return a bounded coarse-to-fine pulse duration for far-view approach."""
    value = float(ny)
    if value < PRECAPTURE_FAST_MAX_NY:
        return PRECAPTURE_FAST_CREEP_SECONDS
    if value < PRECAPTURE_MID_MAX_NY:
        return PRECAPTURE_MID_CREEP_SECONDS
    return PRECAPTURE_NEAR_CREEP_SECONDS


def _sample_capture_with_optional_x_align(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool,
    target_nx: float,
    sample: tuple[object, float] | None = None,
    gaze_mode: str = GAZE_MODE_FIXATE,
    pursue_state: PursueState | None = None,
) -> tuple[Gaze, object, float, float]:
    """Use an existing 3-frame sample; pan-align only when it actually drifted.

    `_capture_sample` already requires two matching detections from three fresh
    frames.  Running `fine_align_horizontal` unconditionally before it used
    another three confirmation frames even when x was already centred.  Keep
    the same hard x gate in the caller, but pay the pan loop only when the
    measured x error exceeds a conservative 0.025 trigger.
    """
    blob, ny = _capture_sample(video, lock) if sample is None else sample
    nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
    if abs(nx - float(target_nx)) > CAPTURE_X_ALIGN_TRIGGER:
        if pursue_state is not None:
            gaze, outcome = pursue_trim_step(
                robot, gaze, pursue_state,
                blob=blob, flip_x=flip_x, target_nx=float(target_nx),
                deadband=CAPTURE_X_ALIGN_TRIGGER,
                motion_key=None, prenudge=False,
            )
            if outcome in ("centered", "trimmed"):
                blob, ny = _capture_sample(video, lock)
                nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
                return gaze, blob, float(ny), nx
            # lost/saturated: fall through to the legacy loop below.
        gaze = fine_align_horizontal(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=float(target_nx),
            confirmations=(1 if normalize_gaze_mode(gaze_mode) != GAZE_MODE_FIXATE else None),
        )
        blob, ny = _capture_sample(video, lock)
        nx = 1.0 - float(blob.nx) if flip_x else float(blob.nx)
    return gaze, blob, float(ny), nx


def visual_precapture_approach(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool = False,
    gaze_mode: str = GAZE_MODE_FIXATE,
    pursue_state: PursueState | None = None,
) -> tuple[Gaze, TargetLock, object]:
    """Bridge the far floor view into the close grasp-camera field of view.

    The loop remains stop-measure-move-stop.  It is faster only where the image
    itself provides large clearance: far pulses are longer, an already-centred
    three-frame sample is reused, and the guaranteed stop inside Robot.drive()
    is not redundantly issued a second time.
    """
    for servo, expected in PRECAPTURE_ARM_POSE.items():
        actual = int(robot.pose.get(servo, -9999))
        if abs(actual - int(expected)) > 25:
            raise RuntimeError(
                f"pre-capture camera pose is not trusted at servo {servo}: "
                f"expected {expected}, got {actual}"
            )
    previous_ny = None
    pending_sample: tuple[object, float] | None = None
    for pulse in range(PRECAPTURE_MAX_CREEP_PULSES + 1):
        gaze, blob, ny, nx = _sample_capture_with_optional_x_align(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=0.5,
            sample=pending_sample, gaze_mode=gaze_mode,
            pursue_state=pursue_state,
        )
        pending_sample = None
        print(
            f"pre-capture window pulse={pulse}/{PRECAPTURE_MAX_CREEP_PULSES} "
            f"nx={nx:.3f} ny={ny:.3f} area={blob.area} pan={gaze.pan}",
            flush=True,
        )
        if abs(nx - 0.5) > CAPTURE_X_DEADBAND:
            raise RuntimeError(
                f"pre-capture target x drifted (nx={nx:.3f}); refusing blind motion"
            )
        if ny >= PRECAPTURE_TOO_CLOSE_NY:
            robot.stop()
            raise RuntimeError(
                f"pre-capture target too close in far view (ny={ny:.3f})"
            )
        if ny >= PRECAPTURE_TARGET_NY - PRECAPTURE_TARGET_TOLERANCE_NY:
            robot.stop()
            print(
                f"pre-capture overlap ready at ny={ny:.3f} "
                f"(target={PRECAPTURE_TARGET_NY:.3f}±{PRECAPTURE_TARGET_TOLERANCE_NY:.3f}); "
                "switch to close camera pose",
                flush=True,
            )
            return gaze, lock, blob
        if pulse >= PRECAPTURE_MAX_CREEP_PULSES:
            break
        if previous_ny is not None and ny < previous_ny - 0.025:
            raise RuntimeError(
                f"pre-capture target moved away ({previous_ny:.3f}->{ny:.3f})"
            )
        duration = precapture_creep_seconds(ny)
        print(
            f"pre-capture coarse-to-fine pulse ny={ny:.3f} duration={duration:.2f}s",
            flush=True,
        )
        robot.drive("forward", MOTOR_SPEED, duration)
        # Robot.drive() stops all four motors in its finally block.  A second
        # stop here used four extra I2C writes after every tiny pulse.
        if not robot.dry_run:
            time.sleep(CAMERA_DELAY_SECONDS)
        next_blob, next_ny = _capture_sample(video, lock)
        if pursue_state is not None:
            # Feedforward trim on the just-measured median (no extra frame):
            # the next sample usually finds the target still centred, so the
            # x-align loop below never triggers.  Loss is ignored here; the
            # sample + legacy gate after it still refuse blind motion.
            gaze, _pc_outcome = pursue_trim_step(
                robot, gaze, pursue_state,
                blob=next_blob, flip_x=flip_x, target_nx=0.5,
                deadband=CAPTURE_X_ALIGN_TRIGGER,
                motion_key=("forward", MOTOR_SPEED, round(float(duration), 3)),
            )
        if next_ny - ny < CAPTURE_MIN_PROGRESS_NY:
            raise RuntimeError(
                f"pre-capture creep made insufficient visual progress ({ny:.3f}->{next_ny:.3f})"
            )
        previous_ny = ny
        pending_sample = (next_blob, next_ny)
    robot.stop()
    raise RuntimeError("could not bring red block into far/close camera overlap")


def establish_capture_pose(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool = False,
) -> tuple[Gaze, TargetLock]:
    """Freeze final camera geometry and re-confirm the same red target there."""
    robot.stop()
    # Preserve the already image-servoed pan across the far->close arm pose
    # transition whenever it is inside the physical servo envelope.  Snapping
    # a pursued pan (e.g. 1908) back to BASE_CENTER throws the tracked block
    # out of view and fails the confirmation below; keeping it removes that
    # artificial error without weakening any image confirmation after the
    # camera geometry changes.
    inherited_pan = int(gaze.pan)
    # Keep the pursued pan whenever it is inside the *physical* servo envelope,
    # including a valid hard-limit lock.  PAN_LIMIT_MARGIN is useful when
    # deciding whether more pursuit travel remains, but it is not an unsafe
    # region.  Seed 12 reached the far/close overlap centred at PAN_MIN=1050;
    # treating that valid lock as invalid snapped the camera to BASE_CENTER and
    # threw the red block out of the close-view FOV during the pose handoff.
    # The fine-align immediately below still fails closed if the close view
    # cannot be confirmed from the inherited pan.
    if PAN_MIN <= inherited_pan <= PAN_MAX:
        capture_start_pan = inherited_pan
    else:
        capture_start_pan = BASE_CENTER
    updates = dict(CAPTURE_ARM_POSE)
    updates[6] = capture_start_pan
    robot.nudge_servos(updates, duration=0.85)
    note_capture_motion = getattr(lock, "note_ego_motion", None)
    if callable(note_capture_motion):
        # Same arm-reprojection forgiveness as the precapture entry above.
        note_capture_motion(frames=5, allowance=0.75)
    if not robot.dry_run:
        time.sleep(0.95)
    gaze = Gaze(pan=capture_start_pan, tilt=CAPTURE_ARM_POSE[3])

    # Changing the camera pose can change image position/scale enough that the
    # previous geometric lock would reject the very same block.  Confirm a
    # stable red component in the new fixed view, then seed a fresh local lock.
    candidate = None
    for _ in range(6):
        frame = video.read()
        item = detect_red_blob(frame, min_area=TRACK_MIN_AREA, crop_left=0)
        if item is not None:
            candidate = confirmed_candidate(video, item)
            if candidate is not None:
                break
        time.sleep(0.04)
    if candidate is None:
        raise LockedTargetLost("red block not visible after entering fixed capture pose")
    capture_lock = TargetLock(candidate)
    target_nx, _target_ny = capture_target_pixel_from_hand_eye()
    gaze = fine_align_horizontal(
        robot, video, capture_lock, gaze, flip_x=flip_x, target_nx=target_nx
    )
    return gaze, capture_lock


def _capture_sample(video: LiveVideo, lock: TargetLock):
    hits = []
    last = None
    for _ in range(CAPTURE_SAMPLE_COUNT):
        frame = video.read()
        blob = detect_red_blob(frame, min_area=TRACK_MIN_AREA, crop_left=0)
        selected = lock.select(blob)
        if selected is not None:
            hits.append(selected)
            last = selected
        time.sleep(0.035)
    if len(hits) < CAPTURE_SAMPLE_MIN_HITS or last is None:
        raise LockedTargetLost("fixed-pose capture lost the locked red block")

    # The first frame(s) after a chassis pulse can still show the previous pose.
    # A three-frame median therefore lagged the real robot in the final close
    # approach: observed ny progressed 0.352 -> 0.390 -> 0.421, but median 0.390
    # triggered one unnecessary 60 ms forward pulse and the next view overshot
    # to ny~=0.51.  Keep the median as the noise floor, but never let it report
    # *farther* than the freshest matching frame.  This is conservative in both
    # directions: forward motion stops earlier; backward recovery does not claim
    # more retreat than the median supports.
    median_ny = statistics.median(float(item.ny) for item in hits)
    latest_ny = float(last.ny)
    control_ny = max(median_ny, latest_ny)
    return last, control_ny


def visual_capture_approach(
    robot: Robot,
    video: LiveVideo,
    lock: TargetLock,
    gaze: Gaze,
    *,
    flip_x: bool = False,
    target_nx: float | None = None,
    target_ny: float | None = None,
    pursue_state: PursueState | None = None,
) -> tuple[Gaze, object]:
    """Final fixed-camera look-and-move stage.

    Unlike far pre-capture, the physical pulse remains the proven 60 ms value:
    this stage is too sensitive to accelerate without a fresh physical
    calibration.  Latency is reduced only by reusing validated samples and by
    removing a redundant stop transaction.
    """
    nominal_target_nx, nominal_target_ny = capture_target_pixel_from_hand_eye()
    target_nx = nominal_target_nx if target_nx is None else float(target_nx)
    target_ny = nominal_target_ny if target_ny is None else float(target_ny)
    if not 0.05 <= target_nx <= 0.95:
        raise ValueError(f"unsafe capture target nx={target_nx:.3f}")
    if not 0.30 <= target_ny < CAPTURE_TOO_CLOSE_NY:
        raise ValueError(f"unsafe capture target ny={target_ny:.3f}")
    previous_ny = None
    pending_sample: tuple[object, float] | None = None
    retreat_pulses = 0
    creep_progress_anchor_ny: float | None = None
    low_progress_pulses = 0
    recovering_too_close = False
    # Creep and retreat draw from separate budgets.  Sharing one loop counter
    # meant an overshoot on the final creep pulse fired a single retreat and
    # then fell out of the loop (SIM: ny=0.408 vs window<=0.402 died with
    # "retreat 1/5" printed but never verified).  Recovery must run to its own
    # verdict: back inside the window or its own pulse cap.
    creep_pulses = 0
    # When recovering from a too-close entry, retreat to the actual requested
    # capture window rather than merely crossing below CAPTURE_TOO_CLOSE_NY.
    # This matters for the early face-staging target (~0.30), where ny~0.47 is
    # safe from collision but already too close for reliable floor-face geometry.
    retreat_target_max_ny = min(
        CAPTURE_TOO_CLOSE_NY - 0.005,
        target_ny + CAPTURE_TARGET_TOLERANCE_NY,
    )
    for pulse in range(CAPTURE_MAX_CREEP_PULSES + CAPTURE_MAX_RETREAT_PULSES + 1):
        gaze, blob, ny, nx = _sample_capture_with_optional_x_align(
            robot, video, lock, gaze, flip_x=flip_x, target_nx=target_nx,
            sample=pending_sample, pursue_state=pursue_state,
        )
        pending_sample = None
        print(
            f"capture window pulse={pulse}/{CAPTURE_MAX_CREEP_PULSES} "
            f"nx={nx:.3f} ny={ny:.3f} target=({target_nx:.3f},{target_ny:.3f}) "
            f"area={blob.area} pan={gaze.pan}",
            flush=True,
        )
        if abs(nx - target_nx) > CAPTURE_X_DEADBAND:
            raise RuntimeError(
                f"capture target x drifted (nx={nx:.3f}, target={target_nx:.3f}); "
                "refusing blind descent"
            )
        # A capture target is valid only inside the *requested* depth window.
        # The old lower-bound-only success check could accept an overshoot that
        # was still below the global collision limit (SIM reproduced
        # 0.258 -> 0.333 for the 0.300+/-0.025 face window).  That position is
        # already too close for floor-face projection, so recover backward
        # before declaring the hand-eye stage ready.
        if ny > retreat_target_max_ny + CAPTURE_WINDOW_OVERSHOOT_EPS_NY:
            if retreat_pulses >= CAPTURE_MAX_RETREAT_PULSES:
                robot.stop()
                raise RuntimeError(
                    f"could not retreat capture target into the requested hand-eye window "
                    f"(ny={ny:.3f}, target={target_ny:.3f}, "
                    f"retreats={retreat_pulses})"
                )
            print(
                f"capture target too close for requested window; bounded retreat "
                f"{retreat_pulses + 1}/{CAPTURE_MAX_RETREAT_PULSES} "
                f"ny={ny:.3f} -> target<={retreat_target_max_ny:.3f}",
                flush=True,
            )
            robot.drive("backward", MOTOR_SPEED, CAPTURE_RETREAT_SECONDS)
            note_ego_motion = getattr(lock, "note_ego_motion", None)
            if callable(note_ego_motion):
                note_ego_motion()
            if not robot.dry_run:
                time.sleep(CAMERA_DELAY_SECONDS)
            next_blob, next_ny = _capture_sample(video, lock)
            if next_ny > ny - CAPTURE_MIN_PROGRESS_NY:
                robot.stop()
                raise RuntimeError(
                    f"capture retreat made insufficient visual progress "
                    f"({ny:.3f}->{next_ny:.3f})"
                )
            retreat_pulses += 1
            recovering_too_close = True
            pending_sample = (next_blob, next_ny)
            continue
        recovering_too_close = False
        if ny >= target_ny - CAPTURE_TARGET_TOLERANCE_NY:
            robot.stop()
            print(
                f"visual hand-eye capture ready at pixel=({nx:.3f},{ny:.3f}) "
                f"TCP-target=({target_nx:.3f},{target_ny:.3f}) "
                f"y-tolerance=±{CAPTURE_TARGET_TOLERANCE_NY:.3f}; "
                "camera centre is not used as the grasp point",
                flush=True,
            )
            return gaze, blob
        if pulse >= CAPTURE_MAX_CREEP_PULSES + CAPTURE_MAX_RETREAT_PULSES:
            break
        if creep_pulses >= CAPTURE_MAX_CREEP_PULSES:
            # Creep budget spent.  A too-close state never reaches here (the
            # retreat branch above either recovers or raises on its own cap),
            # so anything left in-window simply stops pushing closer.
            break
        if previous_ny is not None and ny < previous_ny - 0.025:
            raise RuntimeError(
                f"capture target moved away ({previous_ny:.3f}->{ny:.3f}); refusing another creep"
            )
        creep_pulses += 1
        robot.drive("forward", MOTOR_SPEED, CAPTURE_CREEP_SECONDS)
        if not robot.dry_run:
            time.sleep(CAMERA_DELAY_SECONDS)
        next_blob, next_ny = _capture_sample(video, lock)
        if pursue_state is not None:
            # Same feedforward trim as pre-capture: keep the target centred
            # across the creep so the x-align loop above rarely triggers.
            gaze, _cc_outcome = pursue_trim_step(
                robot, gaze, pursue_state,
                blob=next_blob, flip_x=flip_x, target_nx=target_nx,
                deadband=CAPTURE_X_DEADBAND,
                motion_key=("forward", MOTOR_SPEED, CAPTURE_CREEP_SECONDS),
            )
        if creep_progress_anchor_ny is None:
            creep_progress_anchor_ny = ny
        pulse_progress = next_ny - ny
        cumulative_progress = next_ny - creep_progress_anchor_ny
        if pulse_progress >= CAPTURE_CREEP_MIN_PROGRESS_NY or cumulative_progress >= CAPTURE_CREEP_MIN_PROGRESS_NY:
            low_progress_pulses = 0
            creep_progress_anchor_ny = next_ny
        else:
            low_progress_pulses += 1
            if low_progress_pulses > CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES:
                raise RuntimeError(
                    f"capture creep made insufficient visual progress "
                    f"({ny:.3f}->{next_ny:.3f}, cumulative={cumulative_progress:+.3f})"
                )
        previous_ny = ny
        pending_sample = (next_blob, next_ny)
    robot.stop()
    raise RuntimeError("could not bring red block into calibrated visual capture window")


def grasp_descent_waypoints(plan: PickPlan) -> list[dict[int, int]]:
    """IK waypoints that keep the same radial TCP while descending in z."""
    target = forward_kinematics(plan.grasp_pose, GRIPPER_LINK)
    capture_plan = (
        abs(plan.fingertip_radius_cm - CAPTURE_FINGERTIP_RADIUS_CM) <= 0.15
        or abs(target.pitch_deg - CAPTURE_GRASP_PITCH_DEG) <= 3.0
    )
    preferred_pitch = CAPTURE_GRASP_PITCH_DEG if capture_plan else GRASP_PITCH_PREFERRED_DEG
    poses: list[dict[int, int]] = []
    for height in GRASP_DESCENT_HEIGHTS_CM:
        if height <= target.height_cm + 0.15:
            continue
        poses.append(
            solve_ik(
                plan.fingertip_radius_cm,
                height,
                preferred_pitch_deg=preferred_pitch,
            )
        )
    poses.append(dict(plan.grasp_pose))
    return poses


def calculate_pick_plan(
    robot: Robot,
    gaze: Gaze,
    block: BlockEstimate,
    *,
    capture_profile: bool = False,
) -> PickPlan:
    fingertip_radius = block.radius_cm + GRIPPER_TIP_PAST_CENTER_CM
    if not CALIBRATED_FINGERTIP_RADIUS_MIN_CM <= fingertip_radius <= CALIBRATED_FINGERTIP_RADIUS_MAX_CM:
        raise RuntimeError(
            f"measured fingertip radius {fingertip_radius:.2f}cm is outside "
            "the calibrated grasp envelope"
        )
    corrected_lateral = (
        block.lateral_left_cm - HAND_EYE_LATERAL_OFFSET_CM
    )
    coordinate_yaw_deg = math.degrees(
        math.atan2(corrected_lateral, max(0.1, block.forward_cm))
    )
    base_pulse = clamp_pulse(
        int(round(BASE_CENTER + coordinate_yaw_deg * PULSE_PER_DEGREE))
        + HAND_EYE_YAW_OFFSET_PULSE,
        PAN_MIN,
        PAN_MAX,
    )
    block_height = max(0.8, min(10.0, block.block_height_cm))
    grasp_reference_height = GRASP_REFERENCE_BLOCK_HEIGHT_CM
    if capture_profile or abs(block.radius_cm - CAPTURE_BLOCK_RADIUS_CM) <= 1e-6:
        grasp_height = CAPTURE_GRASP_HEIGHT_CM
    else:
        grasp_height = max(
            GRASP_HEIGHT_MIN_CM,
            min(
                GRASP_HEIGHT_MAX_CM,
                grasp_reference_height * GRASP_HEIGHT_RATIO + GRASP_HEIGHT_OFFSET_CM,
            ),
        )
    hover_height = max(
        HOVER_HEIGHT_CM,
        block_height + HOVER_CLEARANCE_ABOVE_BLOCK_CM,
    )
    hover_pose, hover_height = solve_hover_ik(
        fingertip_radius,
        hover_height,
    )
    capture_plan = capture_profile or abs(block.radius_cm - CAPTURE_BLOCK_RADIUS_CM) <= 1e-6
    grasp_pose = solve_ik(
        fingertip_radius,
        grasp_height,
        preferred_pitch_deg=(
            CAPTURE_GRASP_PITCH_DEG if capture_plan else GRASP_PITCH_PREFERRED_DEG
        ),
    )
    camera_point = forward_kinematics(robot.pose, CAMERA_LINK_CM)
    hover_point = forward_kinematics(hover_pose, GRIPPER_LINK)
    grasp_point = forward_kinematics(grasp_pose, GRIPPER_LINK)
    print(
        "camera FK at measurement: "
        f"radius={camera_point.radius_cm:.2f}cm "
        f"height={camera_point.height_cm + CAMERA_Z_OFFSET_CM:.2f}cm "
        f"pitch={camera_point.pitch_deg:.2f}deg",
        flush=True,
    )
    print(
        "arm IK plan: "
        f"measured-block-height={block_height:.2f}cm "
        f"grasp-reference-height={grasp_reference_height:.2f}cm "
        f"dynamic-grasp-z={grasp_height:.2f}cm "
        f"coordinate-yaw={coordinate_yaw_deg:+.2f}deg "
        f"base={base_pulse}, hover={hover_pose} "
        f"(r={hover_point.radius_cm:.2f},z={hover_point.height_cm:.2f}), "
        f"grasp={grasp_pose} "
        f"(r={grasp_point.radius_cm:.2f},z={grasp_point.height_cm:.2f})",
        flush=True,
    )
    return PickPlan(block, base_pulse, hover_pose, grasp_pose, fingertip_radius)


def move_arm_together(
    robot: Robot,
    pose: dict[int, int],
    *,
    duration: float,
) -> None:
    robot.nudge_servos({joint: pose[joint] for joint in (3, 4, 5)}, duration)
    if not robot.dry_run:
        time.sleep(duration + 0.10)


def capture_red_summary(
    video: LiveVideo,
    *,
    reference: RedSummary | None = None,
    floor_pose: dict[int, int] | None = None,
    decision: str | None = None,
) -> RedSummary:
    if decision not in {None, "reference", "floor_clear"}:
        raise ValueError(f"unsupported verification decision mode: {decision}")
    hits = []
    last_blob = None
    last_frame = None
    # Verification is a before/after still-image comparison. Use uStreamer's
    # latest /snapshot endpoint rather than the long-lived MJPEG reader here:
    # on the physical robot the streaming reader can briefly lag behind a large
    # arm move, while /snapshot immediately reflects the current hover pose.
    # Search/track/approach keep their continuous stream for control latency.
    time.sleep(VERIFY_SETTLE_SECONDS)
    sampled = 0
    for _ in range(VERIFY_FRAME_COUNT):
        frame = capture_bgr(quiet=True)
        sampled += 1
        blob = detect_red_blob(frame, min_area=TRACK_MIN_AREA, crop_left=0)
        if blob is not None and floor_pose is not None:
            # A held red cube can remain visible to the eye-in-hand camera.  It
            # is not a failed grasp unless its image is geometrically compatible
            # with a cube still sitting on the floor from this exact verification
            # pose.  This also keeps catching a miss that pushed the cube sideways.
            floor_estimate = estimate_block(floor_pose, blob)
            # Floor-clear verification must use *geometric floor evidence*.
            # ``estimate_block`` intentionally falls back to apparent-size range
            # when the bottom ray cannot hit the floor (for normal tracking), but
            # that fallback is not proof that the detected cube is on the floor.
            # A cube held close to the eye-in-hand camera is exactly such a case:
            # its bottom ray can point below the floor model while its apparent
            # size still yields a plausible short range.  Accepting that degraded
            # estimate turns a successful lift into MISS_RED_STILL_ON_FLOOR and
            # causes the caller to open the gripper.
            floor_compatible = (
                floor_estimate is not None
                and getattr(floor_estimate, "source", None) == "floor_ray"
            )
            if not floor_compatible:
                source = getattr(floor_estimate, "source", None)
                print(
                    "verification ignored red region without geometric floor ray "
                    f"(source={source or 'none'})",
                    flush=True,
                )
                blob = None
        if blob is not None and reference is not None:
            if reference.nx is None or reference.ny is None or reference.area is None:
                blob = None
            else:
                distance = math.hypot(blob.nx - reference.nx, blob.ny - reference.ny)
                area_ratio = blob.area / max(1.0, reference.area)
                if distance > VERIFY_MATCH_DISTANCE or not 0.15 <= area_ratio <= 6.0:
                    print(
                        "verification ignored unrelated red region "
                        f"distance={distance:.3f} area-ratio={area_ratio:.2f}",
                        flush=True,
                    )
                    blob = None
        if blob is not None:
            hits.append(blob)
            last_blob = blob
            last_frame = frame

        # Preserve the exact existing 9-frame decision thresholds while
        # avoiding frames that can no longer change the outcome.
        remaining = VERIFY_FRAME_COUNT - sampled
        if decision == "reference":
            if len(hits) >= VERIFY_MIN_REFERENCE_HITS:
                break
            if len(hits) + remaining < VERIFY_MIN_REFERENCE_HITS:
                break
        elif decision == "floor_clear":
            if len(hits) > VERIFY_MAX_SUCCESS_HITS:
                break
            if len(hits) + remaining <= VERIFY_MAX_SUCCESS_HITS:
                break
        time.sleep(0.025)
    if not hits:
        return RedSummary(0, sampled, None, None, None, None, None)
    return RedSummary(
        hits=len(hits),
        total=sampled,
        nx=statistics.median(blob.nx for blob in hits),
        ny=statistics.median(blob.ny for blob in hits),
        area=statistics.median(float(blob.area) for blob in hits),
        last_blob=last_blob,
        last_frame=last_frame,
    )


def make_verification_reference(
    robot: Robot,
    video: LiveVideo,
) -> VerificationReference:
    robot.stop()
    summary = capture_red_summary(video, decision="reference")
    print(
        f"pre-grasp camera reference: red hits={summary.hits}/{summary.total}",
        flush=True,
    )
    if summary.hits < VERIFY_MIN_REFERENCE_HITS or summary.last_blob is None:
        raise RuntimeError(
            "block is not reliably visible from the trusted pregrasp measurement pose; "
            "refusing an unverifiable blind descent"
        )
    if summary.last_frame is not None:
        save_debug_frame(summary.last_frame, summary.last_blob, DEBUG_PATH)
    pose = {servo: robot.pose[servo] for servo in (3, 4, 5, 6)}
    return VerificationReference(pose=pose, summary=summary)


def verify_grasp(
    robot: Robot,
    video: LiveVideo,
    reference: VerificationReference,
) -> tuple[bool, RedSummary]:
    """Reject visible misses; floor disappearance is only probable hold evidence.

    The previous verifier first matched only the original pickup-site position,
    then over-corrected to treating *any* visible red cube as failure. A held cube
    can remain visible to the eye-in-hand camera. Sample the whole frame, but
    count only detections geometrically compatible with the floor plane. This
    still rejects a cube pushed sideways while not rejecting a visible held cube.
    No floor-compatible red component is useful negative evidence,
    but with no gripper force/current sensor it is NOT positive proof of HELD.
    The caller therefore exposes this result as PROBABLE_HELD, never verified.
    """
    robot.move_servo(6, reference.pose[6], 0.35)
    move_arm_together(robot, reference.pose, duration=0.75)
    robot.stop()
    floor_red = capture_red_summary(
        video,
        reference=None,
        floor_pose=reference.pose,
        decision="floor_clear",
    )
    floor_clear = floor_red.hits <= VERIFY_MAX_SUCCESS_HITS
    print(
        f"post-grasp floor-compatible red hits={floor_red.hits}/{floor_red.total} -> "
        f"{'FLOOR_CLEAR_PROBABLE' if floor_clear else 'MISS_RED_STILL_ON_FLOOR'}",
        flush=True,
    )
    if floor_red.last_frame is not None:
        save_debug_frame(floor_red.last_frame, floor_red.last_blob, DEBUG_PATH)
    return floor_clear, floor_red


def execute_pick_to_hover(
    robot: Robot,
    video: LiveVideo,
    plan: PickPlan,
) -> VerificationReference:
    robot.stop()
    # Capture the verification reference before moving the eye-in-hand camera.
    # Approach just proved this exact stopped pose sees the target. The hover
    # pose may legitimately move the camera FOV away from the floor block, so
    # requiring visibility there prevented any descent (observed 0/9 hits).
    print("record block from trusted pregrasp measurement pose", flush=True)
    reference = make_verification_reference(robot, video)
    print("open gripper + aim base + move to calculated hover concurrently", flush=True)
    hover_updates = {joint: plan.hover_pose[joint] for joint in (3, 4, 5)}
    hover_updates[6] = plan.base_pulse
    hover_updates[GRIPPER_ID] = GRIPPER_OPEN
    robot.nudge_servos(hover_updates, duration=1.25)
    if not robot.dry_run:
        time.sleep(1.35)
    descent = grasp_descent_waypoints(plan)
    print(f"blind Cartesian-like descent waypoints={len(descent)}", flush=True)
    for index, pose in enumerate(descent, start=1):
        point = forward_kinematics(pose, GRIPPER_LINK)
        print(
            f"blind descent {index}/{len(descent)} r={point.radius_cm:.2f} z={point.height_cm:.2f}",
            flush=True,
        )
        move_arm_together(robot, pose, duration=0.45 if index < len(descent) else 0.60)
    print("close gripper", flush=True)
    robot.move_servo(GRIPPER_ID, GRIPPER_CLOSE, 0.85)
    if not robot.dry_run:
        time.sleep(0.45)
    print("lift while holding block", flush=True)
    for pose in reversed(descent[:-1]):
        move_arm_together(robot, pose, duration=0.40)
    move_arm_together(robot, plan.hover_pose, duration=0.70)
    return reference


def recalculate_capture_retry_plan(
    robot: Robot,
    video: LiveVideo,
    remaining: RedSummary,
    *,
    flip_x: bool,
) -> tuple[Gaze, TargetLock, PickPlan]:
    """After a clean miss, move the same fixed-view block slightly closer once."""
    if remaining.last_blob is None:
        raise RuntimeError("capture retry requested without a visible block")
    for servo, expected in CAPTURE_ARM_POSE.items():
        if abs(int(robot.pose.get(servo, -9999)) - int(expected)) > 25:
            raise RuntimeError(
                f"capture retry pose changed at servo {servo}; refusing chassis creep"
            )
    print(
        f"capture grasp missed at ny={remaining.ny:.3f}; "
        f"retry closer at ny>={CAPTURE_RETRY_TARGET_NY:.3f}",
        flush=True,
    )
    robot.move_servo(GRIPPER_ID, GRIPPER_OPEN, 0.55)
    lock = TargetLock(remaining.last_blob)
    gaze = Gaze(pan=int(robot.pose[6]), tilt=int(robot.pose[3]))
    gaze, blob = visual_capture_approach(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        target_ny=CAPTURE_RETRY_TARGET_NY,
    )
    return gaze, lock, make_capture_pick_plan(robot, gaze, blob)


def recalculate_retry_plan(
    robot: Robot,
    video: LiveVideo,
    remaining: RedSummary,
    *,
    flip_x: bool,
    flip_y: bool,
) -> tuple[Gaze, TargetLock, PickPlan]:
    if remaining.last_blob is None:
        raise RuntimeError("retry requested without a visible block")
    print("open gripper at hover before retry measurement", flush=True)
    robot.move_servo(GRIPPER_ID, GRIPPER_OPEN, 0.55)
    lock = TargetLock(remaining.last_blob)
    gaze = Gaze(pan=robot.pose[6], tilt=robot.pose[3])
    gaze = centre_gaze(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        flip_y=flip_y,
        confirmations=CENTER_CONFIRMATIONS,
        max_frames=60,
        allow_limit_lock=False,
    )
    gaze, samples, _ = stationary_range_samples(
        robot,
        video,
        lock,
        gaze,
        count=5,
        flip_x=flip_x,
        flip_y=flip_y,
    )
    if len(samples) < 3:
        raise RuntimeError(
            f"only {len(samples)} valid retry samples; refusing another descent"
        )
    expected = median_block_estimate(samples)
    gaze, block = final_measurement(
        robot,
        video,
        lock,
        gaze,
        flip_x=flip_x,
        flip_y=flip_y,
        expected_radius_cm=expected.radius_cm,
    )
    return gaze, lock, calculate_pick_plan(robot, gaze, block)


def run(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)
    video = None
    running = True

    def stop_signal(signum: int, _frame: object) -> None:
        nonlocal running
        running = False
        robot.stop()
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)

    try:
        state(State.BASIC_POSE, "set verified servo pulses; no buzzer command")
        robot.stop()
        robot.move_pose(BASIC_POSE, lowering=False)

        video = LiveVideo(args.stream_url)
        video.settle(CAMERA_DELAY_SECONDS)

        state(State.SCAN, "slow arm-camera scan across the complete safe envelope")
        robot.move_pose(SCAN_ARM_POSE, lowering=True)
        gaze = Gaze(pan=SCAN_ARM_POSE[6], tilt=SCAN_ARM_POSE[3])
        gaze, lock = scan_for_target(
            robot,
            video,
            gaze,
            timeout_seconds=args.scan_timeout,
        )

        state(State.LOCK_GAZE, "hold the confirmed red block at image centre")
        gaze = centre_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=args.flip_x,
            flip_y=args.flip_y,
        )

        state(State.ALIGN_BODY, "turn chassis while gaze remains locked")
        gaze = align_body_to_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=args.flip_x,
            flip_y=args.flip_y,
        )

        state(
            State.APPROACH_TO_FACE,
            "approach the safe face-alignment staging radius",
        )
        staging = approach_with_locked_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=args.flip_x,
            flip_y=args.flip_y,
            target_radius_cm=FACE_ALIGN_START_RADIUS_CM,
            allow_blind_arrival=False,
        )
        gaze = staging.gaze

        state(
            State.ALIGN_FACE,
            "orbit sideways and re-face the block until a face is square",
        )
        gaze = align_block_face(
            robot,
            video,
            lock,
            gaze,
            flip_x=args.flip_x,
            flip_y=args.flip_y,
        )

        state(
            State.FINAL_APPROACH,
            "continue straight while holding the face-aligned target lock",
        )
        approach = approach_with_locked_gaze(
            robot,
            video,
            lock,
            gaze,
            flip_x=args.flip_x,
            flip_y=args.flip_y,
        )
        gaze = approach.gaze

        state(
            State.STOP_AND_MEASURE,
            "stop chassis, flush camera delay, estimate coordinate from 7 frames",
        )
        if approach.target_still_visible:
            try:
                gaze, block = final_measurement(
                    robot,
                    video,
                    lock,
                    gaze,
                    flip_x=args.flip_x,
                    flip_y=args.flip_y,
                    expected_radius_cm=approach.block.radius_cm,
                )
            except LockedTargetLost:
                robot.stop()
                block = approach.block
                print(
                    "target left camera after arrival; use the last stopped "
                    "multi-frame coordinate for blind grasp",
                    flush=True,
                )
        else:
            robot.stop()
            block = approach.block
            print(
                "camera visibility ended at arrival; use latched coordinate "
                "for blind grasp",
                flush=True,
            )

        state(State.CALCULATE_ARM, "combine block coordinate with arm FK/IK")
        plan = calculate_pick_plan(robot, gaze, block)
        if args.no_pick:
            print("--no-pick: coordinate and IK complete; arm will not descend")
            return 0

        state(State.GRASP, "single blind grasp using calibrated hand-eye offset")
        reference = execute_pick_to_hover(robot, video, plan)
        state(
            State.VERIFY_GRASP,
            "check the whole returned floor view for any remaining red block",
        )
        floor_clear, remaining = verify_grasp(robot, video, reference)
        if not floor_clear:
            robot.move_servo(GRIPPER_ID, GRIPPER_OPEN, 0.55)
            raise RuntimeError(
                "red block remains visible after grasp; refusing an automatic retry"
            )
        print(
            "pickup floor is clear; without positive gripper sensing this is "
            "PROBABLE_HELD, not a verified grasp",
            flush=True,
        )
        robot.move_pose(CARRY_POSE, lowering=False)
        state(State.COMPLETE, "blind grasp sequence complete; hold remains probable")
        robot.stop()
        return 0
    finally:
        if video is not None:
            video.close()
        robot.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Face-aligned, camera-verified MasterPi red-block pickup controller"
        )
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-pick", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--scan-timeout", type=float, default=120.0)
    parser.add_argument("--stream-url", default=STREAM_URL)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 15.0 <= args.scan_timeout <= 300.0:
        raise ValueError("--scan-timeout must be between 15 and 300 seconds")
    return run(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("stopped by user", flush=True)
        raise SystemExit(130)
    except (RuntimeError, ValueError, FileNotFoundError) as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        raise SystemExit(1) from error
