"""Human-like gaze-hold (pursue mode) for coarse approach.

Implements the 2026-09-04 decision_log design "사람 같은注視 구조":
  eye (servo 6 pan + servo 3 tilt) pursues first, body takes over past limits,
  feedforward pre-compensation for known chassis motion (VOR-like),
  saccade to last-known bearing on target loss (with blink grace).

Only pure logic lives here. It reuses track.py's proven PID (tracking_step)
and geometry.py's PULSE_PER_DEGREE. No robot I/O: callers pass in a fresh
detection blob (or None) and apply the returned gaze via their own
``robot.nudge_servos`` path, so this module never touches actuators.

Gaze modes (toggle):
  fixate  -- legacy locked-gaze behaviour; pursue_step() is never called.
  pursue  -- per-pulse gaze correction + active re-acquire on loss.

Integration point: physical_state_machine_reference.approach_with_locked_gaze
motion-step loop calls pursue_step() after each pulse + fresh frame instead of
a full centre_gaze; SATURATED routes into the existing align_body_to_gaze
path. Default mode is fixate until SIM-validated.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from geometry import PULSE_PER_DEGREE
from poses import BASE_CENTER
from track import Gaze, tracking_step

# Gaze-hold never spends the full servo-6 corridor: face stage needs +/-34 deg
# usable, so pursuit caps itself below that and yields to the body instead.
GAZE_HOLD_MAX_DEG = 25.0
GAZE_HOLD_PAN_MIN = int(round(BASE_CENTER - GAZE_HOLD_MAX_DEG * PULSE_PER_DEGREE))
GAZE_HOLD_PAN_MAX = int(round(BASE_CENTER + GAZE_HOLD_MAX_DEG * PULSE_PER_DEGREE))

# Deadbeat trim gain.  Full geometric correction (1.0) oscillates under HFOV
# calibration error and servo latency (SIM: nx overshot left to 0.433); 0.6
# converges without measurable steady-state cost since the next pulse re-trims.
PURSUE_TRIM_GAIN = 0.6

# Human blink / microsaccade suppression: 1-2 lost frames hold gaze, only a
# longer loss triggers an active saccade to the remembered bearing.
BLINK_GRACE_FRAMES = 2

GAZE_MODE_FIXATE = "fixate"
GAZE_MODE_PURSUE = "pursue"
GAZE_MODES = (GAZE_MODE_FIXATE, GAZE_MODE_PURSUE)


def normalize_gaze_mode(value: object | None) -> str:
    text = str(value or GAZE_MODE_FIXATE).strip().lower()
    if text not in GAZE_MODES:
        raise ValueError(f"gaze_mode must be one of {GAZE_MODES}")
    return text


@dataclass
class PursueState:
    gaze: Gaze = field(default_factory=Gaze)
    misses: int = 0
    # Last known target bearing in degrees, + = target left of heading
    # (spatial_memory bearing convention). None until ever seen.
    memory_bearing_deg: float | None = None
    # Servo-space feedforward memory, keyed by motion identity
    # (kind, speed, pulse-duration).  A correction learned behind a 0.28 s
    # staging pulse must never replay before a 0.10 s creep pulse: replaying
    # a 3x correction oscillates the eye and wobbles the ny projection, which
    # the progress gate then misreads as backward motion (SIM).
    corr_by_motion: dict = field(default_factory=dict)
    last_motion_key: tuple | None = None


def recall_corr(state: PursueState, motion_key: tuple | None) -> int:
    """Correction to replay before a pulse of the given motion identity."""
    key = motion_key if motion_key is not None else state.last_motion_key
    if key is None:
        return 0
    return int(state.corr_by_motion.get(key, 0))


def store_corr(state: PursueState, motion_key: tuple | None, corr: int) -> None:
    """Remember a correction under its motion identity (and as most recent)."""
    if motion_key is None:
        motion_key = state.last_motion_key
    if motion_key is None:
        return
    state.corr_by_motion[motion_key] = int(corr)
    state.last_motion_key = motion_key


def feedforward_pan(pan: int, chassis_yaw_delta_deg: float) -> int:
    """Pre-compensate gaze for a KNOWN chassis yaw applied this pulse (VOR-like).

    Chassis turning left (+yaw) swings the camera view right, so the eye must
    lead left by the same angle. Caller handles image flip conventions; this
    keeps the geometric identity: pan_shift == -yaw_delta.
    """
    return int(round(pan - chassis_yaw_delta_deg * PULSE_PER_DEGREE))


def clamp_hold(pan: int) -> tuple[int, bool]:
    """Clamp pan into the pursue envelope. Returns (pan, saturated)."""
    if pan <= GAZE_HOLD_PAN_MIN:
        return GAZE_HOLD_PAN_MIN, True
    if pan >= GAZE_HOLD_PAN_MAX:
        return GAZE_HOLD_PAN_MAX, True
    return pan, False


def saccade_pan(current_pan: int, memory_bearing_deg: float) -> tuple[int, bool]:
    """Saccade toward the last-known bearing. Returns (pan, clipped).

    clipped=True means even eyes-full cannot cover it -> caller must transfer
    yaw to the body (existing align_body_to_gaze path).
    """
    target = int(round(current_pan + memory_bearing_deg * PULSE_PER_DEGREE))
    return clamp_hold(target)


def pursue_step(
    state: PursueState,
    blob,
    *,
    dt: float,
    flip_x: bool = False,
    flip_y: bool = False,
    memory_bearing_deg: float | None = None,
) -> str:
    """One pursue update. Returns status:

    TRACKING     blob visible, gaze following inside envelope
    SATURATED    blob visible but eyes at hold limit -> transfer yaw to body
    LOST_BLINK   blob missing within grace -> gaze held, wait one more frame
    LOST_SACCADE blob missing past grace -> gaze saccaded to memory bearing
    LOST_SEARCH  blob missing past grace and no memory -> escalate to search
    """
    if memory_bearing_deg is not None:
        state.memory_bearing_deg = memory_bearing_deg
    if blob is None:
        state.misses += 1
        if state.misses <= BLINK_GRACE_FRAMES:
            return "LOST_BLINK"
        if state.memory_bearing_deg is None:
            return "LOST_SEARCH"
        pan, _ = saccade_pan(state.gaze.pan, state.memory_bearing_deg)
        state.gaze.pan = pan
        return "LOST_SACCADE"
    state.misses = 0
    state.gaze = tracking_step(blob, state.gaze, dt=dt, flip_x=flip_x, flip_y=flip_y)
    state.gaze.pan, saturated = clamp_hold(state.gaze.pan)
    return "SATURATED" if saturated else "TRACKING"
