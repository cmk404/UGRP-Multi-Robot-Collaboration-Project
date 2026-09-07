"""Run the physically tuned REAL MasterPi stack on MuJoCo.

The task/control algorithms remain in scripts/red_block.  This module replaces
only the hardware boundary (I2C motors/servos, V4L2/MJPEG camera, wall clock)
with a MuJoCo-backed implementation having the same contract.
"""
from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
import importlib
import math
import os
from pathlib import Path
import sys
import threading
import time as wall_time
from types import ModuleType
from typing import Any, Mapping

import numpy as np

from sim.masterpi_dynamics_v2 import (
    TARGET_BLOCK_HALF_M,
    TARGET_BLOCK_LIFT_CENTER_M,
    TARGET_BLOCK_SIDE_M,
)
from sim.masterpi_camera_profile import CAMERA_CX_PX, CAMERA_NATIVE_WIDTH

# SIM-only hand-eye stop for the current centered measured-fisheye camera.
# X is the measured raw optical principal point.  Y was generated offline by
# placing the 30-mm cube under the production 15.0-cm grasp TCP, rendering the
# same raw-fisheye robot_cam used by the actor, and running the shared red-blob
# detector.  This is a digital-twin sensor/actuator calibration fixture, not a
# privileged runtime object lookup.  The resulting center was nx~=0.448,
# ny~=0.377; use measured cx for x and the detector fixture for y.
SIM_CAPTURE_TARGET_NX = float(CAMERA_CX_PX) / float(CAMERA_NATIVE_WIDTH)
SIM_CAPTURE_TARGET_NY = 0.377
# Actor-camera replay on seed 2101568344 measured a 60 ms close creep jumping
# from ny=0.258 to 0.333, skipping the early face-projection window
# (0.300+/-0.025).  A 40 ms replay then stopped at ny=0.248 before the 0.275 lower bound.
# Use the measured midpoint 50 ms in SIM so the existing seven-pulse budget can
# observe and stop inside that window.  REAL keeps its independently measured
# 60 ms pulse.
SIM_CAPTURE_CREEP_SECONDS = 0.05
# The 50 ms SIM creep above measured +0.006 normalized-y on seed 2101568344.
# The shared 0.008 watchdog was calibrated for the 60 ms REAL pulse, so use a
# duration-matched close-capture threshold while leaving pre-capture/retreat
# watchdogs and all REAL thresholds untouched. 0.005 remains above a two-pixel
# 480p contour displacement and still rejects zero/noise-only actuator motion.
SIM_CAPTURE_CREEP_MIN_PROGRESS_NY = 0.005
SIM_CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES = 1
# With the shorter 50 ms SIM pulse, seed 2101568344 advances monotonically from
# close-view ny=0.087 to 0.304 after the REAL-calibrated seven motions. One
# additional bounded pulse is required to enter the 0.377±0.025 capture window;
# the hard too-close guard at 0.505 and retreat path remain unchanged.
SIM_CAPTURE_MAX_CREEP_PULSES = 8
# A no-render drivetrain sweep of the shared dog-leg on this twin measured
# +7.38 deg residual yaw with symmetric 0.18/0.18 s turns. A 0.30 s restore
# leaves +0.31 deg, so use that SIM-only actuator calibration while REAL keeps
# its 0.18 s restore.
SIM_FACE_REPOSITION_RESTORE_TURN_SECONDS = 0.30
SIM_FAST_CAMERA_CONTROL = os.environ.get(
    "UGRP_SIM_FAST_CAMERA_CONTROL", "1"
).strip().lower() not in {"0", "false", "no", "off"}
# The sparse continuous-far controller is retained as an opt-in experiment.
# Combining it with the fast deterministic confirmation policy made seed 15
# lose centre lock; the default fast path therefore keeps bounded stopped
# pulses and accelerates perception/pick only.
SIM_FAST_CONTINUOUS_APPROACH = os.environ.get(
    "UGRP_SIM_FAST_CONTINUOUS_APPROACH", "0"
).strip().lower() in {"1", "true", "yes", "on"}

ROOT = Path(__file__).resolve().parents[1]
REAL_DIR = ROOT / "scripts" / "red_block"
SCRIPTS_DIR = ROOT / "scripts"
for _path in (REAL_DIR, SCRIPTS_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

# Import the physical stack from its real source.  These modules keep their
# deployed Pi-compatible flat imports, so the REAL directory is intentionally
# on sys.path rather than copied into a second SIM implementation.
poses = importlib.import_module("poses")
control_model = importlib.import_module("masterpi_control")
camera_mod = importlib.import_module("camera")
search_mod = importlib.import_module("search")
track_mod = importlib.import_module("track")
approach_mod = importlib.import_module("approach")
pick_mod = importlib.import_module("pick")
place_mod = importlib.import_module("place")
put_down_mod = importlib.import_module("put_down")
carry_handoff_mod = importlib.import_module("carry_handoff")
precision_mod = importlib.import_module("physical_state_machine_reference")
precision_handoff = importlib.import_module("precision_handoff")
near_look_handoff_mod = importlib.import_module("near_look_handoff")


# The REAL controller modules are resident in the multi-robot SIM process,
# unlike the one-skill-per-process REAL deployment.  These context-local
# dispatchers let each worker thread keep its own clock/camera/runtime while
# one shared set of module wrappers remains installed.  The originals are
# restored only after the final active SIM context exits.
_ACTIVE_RUNTIME: ContextVar[Any | None] = ContextVar("ugrp_active_sim_runtime", default=None)
_ACTIVE_PLACE_WRAPPERS: ContextVar[dict[str, Any] | None] = ContextVar(
    "ugrp_active_sim_place_wrappers", default=None
)
_PATCH_LOCK = threading.RLock()
_PATCH_REFCOUNT = 0
_PATCH_ORIGINALS: dict[tuple[ModuleType, str], Any] = {}
_PATCH_INSTALLED = False

_DISPATCH_KEYS = {
    (search_mod, "time"),
    (approach_mod, "time"),
    (pick_mod, "time"),
    (place_mod, "time"),
    (precision_mod, "time"),
    (search_mod, "LiveCamera"),
    (precision_mod, "LiveVideo"),
    (precision_mod, "capture_bgr"),
    (pick_mod, "capture_bgr"),
    (place_mod, "capture_bgr"),
    (precision_mod, "retreat_for_body_alignment"),
    (place_mod, "acquire_target"),
    (place_mod, "center_target"),
    (place_mod, "align_body_to_target"),
    (place_mod, "approach_target"),
}


class _RuntimeTimeProxy:
    """Route REAL module-local sleeps to the active SIM runtime."""

    def sleep(self, seconds: float) -> None:
        runtime = _ACTIVE_RUNTIME.get()
        if runtime is None:
            return wall_time.sleep(seconds)
        runtime.clock.sleep(seconds)

    def monotonic(self) -> float:
        runtime = _ACTIVE_RUNTIME.get()
        return runtime.clock.monotonic() if runtime is not None else wall_time.monotonic()

    def time(self) -> float:
        runtime = _ACTIVE_RUNTIME.get()
        return runtime.clock.time() if runtime is not None else wall_time.time()

    def __getattr__(self, name: str) -> Any:
        return getattr(wall_time, name)


_RUNTIME_TIME_PROXY = _RuntimeTimeProxy()


def _original(module: ModuleType, name: str) -> Any:
    with _PATCH_LOCK:
        return _PATCH_ORIGINALS.get((module, name), getattr(module, name))


def _dispatch(module: ModuleType, name: str, *args, **kwargs):
    wrappers = _ACTIVE_PLACE_WRAPPERS.get()
    runtime = _ACTIVE_RUNTIME.get()
    fn = wrappers.get(name) if runtime is not None and wrappers else None
    if fn is not None:
        return fn(*args, **kwargs)
    return _original(module, name)(*args, **kwargs)


def _install_dispatch_patches_locked() -> None:
    global _PATCH_INSTALLED
    if _PATCH_INSTALLED:
        return

    def install(module: ModuleType, name: str, value: Any) -> None:
        key = (module, name)
        _PATCH_ORIGINALS[key] = getattr(module, name)
        setattr(module, name, value)

    for module in (search_mod, approach_mod, pick_mod, place_mod, precision_mod):
        if hasattr(module, "time"):
            install(module, "time", _RUNTIME_TIME_PROXY)
    install(search_mod, "LiveCamera", lambda *a, **kw: (
        SimCamera(_ACTIVE_RUNTIME.get().world, _ACTIVE_RUNTIME.get().clock)
        if _ACTIVE_RUNTIME.get() is not None
        else _original(search_mod, "LiveCamera")(*a, **kw)
    ))
    install(precision_mod, "LiveVideo", lambda *a, **kw: (
        SimCamera(_ACTIVE_RUNTIME.get().world, _ACTIVE_RUNTIME.get().clock)
        if _ACTIVE_RUNTIME.get() is not None
        else _original(precision_mod, "LiveVideo")(*a, **kw)
    ))
    for module in (precision_mod, pick_mod, place_mod):
        install(module, "capture_bgr", lambda *a, _module=module, **kw: (
            _ACTIVE_RUNTIME.get().camera.read(*a, **kw)
            if _ACTIVE_RUNTIME.get() is not None
            else _original(_module, "capture_bgr")(*a, **kw)
        ))
    # ``detect_red_blob`` is now ContextVar-backed and needs no dispatch
    # wrapper.  Retain its baseline nevertheless so legacy callers that still
    # mutate the attribute during a SIM context are restored with the other
    # shared patches at final teardown.
    _PATCH_ORIGINALS[(precision_mod, "detect_red_blob")] = precision_mod.detect_red_blob
    install(
        precision_mod,
        "retreat_for_body_alignment",
        lambda *a, **kw: _dispatch(precision_mod, "retreat_for_body_alignment", *a, **kw),
    )
    for name in ("acquire_target", "center_target", "align_body_to_target", "approach_target"):
        install(
            place_mod,
            name,
            lambda *a, _name=name, **kw: _dispatch(place_mod, _name, *a, **kw),
        )
    _PATCH_INSTALLED = True


def _restore_dispatch_patches_locked() -> None:
    global _PATCH_INSTALLED
    if not _PATCH_INSTALLED:
        return
    for (module, name), original in reversed(list(_PATCH_ORIGINALS.items())):
        setattr(module, name, original)
    if hasattr(precision_mod, "_ugrp_original_detect_red_blob"):
        delattr(precision_mod, "_ugrp_original_detect_red_blob")
    _PATCH_ORIGINALS.clear()
    _PATCH_INSTALLED = False

MOTOR_PATTERNS = {
    "forward": np.asarray([1.0, 1.0, 1.0, 1.0], dtype=float),
    "backward": np.asarray([-1.0, -1.0, -1.0, -1.0], dtype=float),
    "left": np.asarray([-1.0, 1.0, 1.0, -1.0], dtype=float),
    "right": np.asarray([1.0, -1.0, -1.0, 1.0], dtype=float),
    "rotate-left": np.asarray([-1.0, 1.0, -1.0, 1.0], dtype=float),
    "rotate-right": np.asarray([1.0, -1.0, 1.0, -1.0], dtype=float),
}


# A carried-block search must not infer angular progress from a count of tiny
# wheel pulses.  The V2 drivetrain has a measured-form first-order response, so
# 45 ms start/stop pulses spend most of their time below steady-state speed and
# can add repeated tangential shocks at the gripper.  Use bounded, feedback-led
# body turns and re-read actor-visible odometry after every segment instead.
CARRY_SEARCH_HEADING_DEADBAND_RAD = math.radians(7.0)
CARRY_SEARCH_COARSE_ERROR_RAD = math.radians(24.0)
CARRY_SEARCH_COARSE_TURN_SECONDS = 0.34
CARRY_SEARCH_FINE_TURN_SECONDS = 0.16
CARRY_SEARCH_MAX_GUIDED_TURNS = 8
CARRY_SEARCH_MIN_PROGRESS_RAD = math.radians(1.0)
CARRY_SEARCH_MAX_STALLED_TURNS = 2


def carry_search_turn_for_heading_error(error_rad: float) -> tuple[str, float] | None:
    """Return one bounded chassis correction from actor-visible heading error.

    This intentionally does not convert pulse count into angle.  Direction comes
    from the observed sign and duration has only coarse/fine bounded tiers; the
    next correction is chosen from a fresh odometry error.
    """
    error = float(error_rad)
    if not math.isfinite(error):
        return None
    magnitude = abs(error)
    if magnitude <= CARRY_SEARCH_HEADING_DEADBAND_RAD:
        return None
    direction = "rotate-left" if error > 0.0 else "rotate-right"
    duration = (
        CARRY_SEARCH_COARSE_TURN_SECONDS
        if magnitude > CARRY_SEARCH_COARSE_ERROR_RAD
        else CARRY_SEARCH_FINE_TURN_SECONDS
    )
    return direction, duration


class SimClock:
    """REAL-compatible clock whose sleeps advance MuJoCo, not just wall time.

    Servo boards interpolate a commanded PWM over the supplied duration while
    the Python controller sleeps. The simulator therefore exposes an updater
    hook that is evaluated every 20 ms of simulated time before physics advances.
    """

    def __init__(self, world):
        self.world = world
        self.servo_updater = None

    def monotonic(self) -> float:
        return float(self.world.data.time)

    def time(self) -> float:
        return float(self.world.data.time)

    def sleep(self, seconds: float) -> None:
        seconds = max(0.0, float(seconds))
        if seconds <= 0:
            return
        deadline = self.monotonic() + seconds
        while True:
            now = self.monotonic()
            if now + 1e-9 >= deadline:
                break
            target_time = min(deadline, now + 0.020)
            if self.servo_updater is not None:
                self.servo_updater(target_time)
            if hasattr(self.world, "advance_to_sim_time"):
                self.world.advance_to_sim_time(target_time)
            elif hasattr(self.world, "step_realtime"):
                self.world.step_realtime(target_time - now)
            else:
                self.world.step(duration_s=target_time - now)
        if self.servo_updater is not None:
            self.servo_updater(self.monotonic())


class SimCamera:
    """BGR camera matching scripts/red_block/camera.py and LiveVideo.read()."""

    # A tenth of a micron / sub-microradian is far below one pixel at the
    # physical 640x480 sensor distance, but still large enough to absorb the
    # last floating-point settling residue from MuJoCo.  The cache is keyed by
    # the complete shared qpos, not just this controller's joints: a peer or a
    # free block moving must invalidate an actor-visible frame as well.
    _QPOS_TOLERANCE = 1e-7
    _CTRL_TOLERANCE = 1e-6

    def __init__(self, world, clock: SimClock):
        self.world = world
        self.clock = clock
        self.source = "mujoco:robot_cam"
        self._sequence = 0
        self._closed = False
        self._cached_bgr: np.ndarray | None = None
        self._cached_scene_signature: tuple[bytes, bytes, bytes] | None = None

    @staticmethod
    @contextmanager
    def _physics_guard(world):
        """Freeze the shared data while comparing and rendering one sample."""
        owner = getattr(world, "_owner", None)
        lock = getattr(owner, "physics_lock", None) or getattr(world, "physics_lock", None)
        if lock is None:
            yield
            return
        with lock:
            yield

    @classmethod
    def _quantized_bytes(cls, values, tolerance: float) -> bytes:
        array = np.asarray(values, dtype=np.float64)
        # Quantization gives the cache a documented, sensor-scale tolerance
        # without exposing any simulator-only state to the controller.
        return np.rint(array / tolerance).astype(np.int64, copy=False).tobytes()

    @classmethod
    def _scene_signature(cls, world) -> tuple[bytes, bytes, bytes] | None:
        """Return actor-visible scene inputs, or None for non-MuJoCo fakes."""
        data = getattr(world, "data", None)
        qpos = getattr(data, "qpos", None)
        if qpos is None:
            return None
        ctrl = getattr(data, "ctrl", ())
        eq_active = getattr(data, "eq_active", ())
        return (
            cls._quantized_bytes(qpos, cls._QPOS_TOLERANCE),
            cls._quantized_bytes(ctrl, cls._CTRL_TOLERANCE),
            np.asarray(eq_active, dtype=np.uint8).tobytes(),
        )

    def _publish_bgr(self, bgr: np.ndarray) -> None:
        """Publish only actor-visible pixels for the UI cache."""
        self.world._latest_robot_bgr = bgr
        self.world._latest_robot_frame_seq = self._sequence
        self.world._latest_robot_jpeg = None
        self.world._latest_robot_jpeg_seq = None
        self.world._latest_robot_jpeg_quality = None
        self.world._presentation_dirty = False
        # A reused BGR sample remains valid, but an old JPEG may have been
        # encoded at a different quality. Let the presentation layer decide
        # whether its encoded representation can be reused.

    @property
    def sequence(self) -> int:
        return self._sequence

    def read(self, timeout: float = 2.0, *, quiet: bool = False):
        del timeout, quiet
        if self._closed:
            raise RuntimeError("simulation camera is closed")
        # A physical read is a fresh sensor sample. Advance one tiny physics
        # quantum so repeated REAL confirmation reads cannot return a stale epoch.
        # The physical camera supplies a fresh frame at roughly 20 FPS. A 4 ms
        # synthetic read made the REAL PID observe before its 0.1 s servo nudge
        # had progressed, creating a SIM-only oscillation.
        self.clock.sleep(0.05)
        with self._physics_guard(self.world):
            signature = self._scene_signature(self.world)
            cache_hit = (
                self._cached_bgr is not None
                and signature is not None
                and signature == self._cached_scene_signature
            )
            bgr = self._cached_bgr if cache_hit else None
        if bgr is None:
            # Never hold the caller's physics lock while waiting for the shared
            # world's render broker: that broker acquires the same lock to take
            # one coherent MuJoCo snapshot. Tag the frame with the pre-render
            # signature; if a peer moved while this request queued, the next
            # read safely misses the cache instead of reusing stale pixels.
            rgb = self.world.render_rgb("robot_cam")
            bgr = np.ascontiguousarray(rgb[..., ::-1])  # REAL detector expects BGR.
            self._cached_bgr = bgr
            self._cached_scene_signature = signature
        self._sequence += 1
        # The browser's robot-camera view should reuse the exact frame that the
        # migrated REAL detector actually observed. Rendering a second 640x480
        # MuJoCo camera image only for UI transport is expensive on managed GPU
        # runtimes and can make controller timing computation-bound.
        self._publish_bgr(bgr)
        return bgr

    def settle(self, seconds: float = 0.0):
        self.clock.sleep(seconds)
        return self.read()

    def close(self) -> None:
        # Cameras are logical views of one persistent MuJoCo renderer. Closing a
        # REAL skill must not close the production world renderer.
        self._closed = True


def physical_motor_api_to_v2(raw: np.ndarray) -> np.ndarray:
    """Convert MasterPi MotorTransport IDs/polarities to v2 FL/FR/RL/RR.

    The mapping is derived from the REAL DRIVE_MAP basis, not guessed from
    visual wheel placement: REAL forward/left/yaw must map exactly onto the
    v2 forward/left/yaw basis.
    """
    raw = np.asarray(raw, dtype=float)
    if raw.shape != (4,):
        raise ValueError("expected four physical motor API values")
    return np.asarray([raw[2], -raw[0], -raw[3], raw[1]], dtype=float)


class SimMotorTransport:
    """Match masterpi_control.MotorTransport.write_motor used by precision.py."""

    def __init__(self, world):
        self.world = world
        self.values = np.zeros(4, dtype=float)

    def write_motor(self, motor: int, speed: int) -> None:
        idx = int(motor) - 1
        if not 0 <= idx < 4:
            raise ValueError(f"motor id must be 1..4, got {motor}")
        self.values[idx] = max(-1.0, min(1.0, float(speed) / 40.0))
        # No SIM-only response gain lives at the hardware adapter. The physical
        # motor API is converted 1:1 into the v2 wheel basis; measured response
        # differences belong in MasterPiDynamicsV2 calibration parameters.
        self.world.set_motor_commands(physical_motor_api_to_v2(self.values))

    def stop(self) -> None:
        self.values[:] = 0.0
        self.world.set_motor_commands(physical_motor_api_to_v2(self.values))


class SimServoTransport:
    """Servo-board boundary with duration-aware target interpolation."""

    def __init__(self, world, scheduler):
        self.world = world
        self.scheduler = scheduler

    def write_servo(self, servo: int, pulse: int, duration: float) -> None:
        self.scheduler(int(servo), poses.clamp_pulse(int(pulse)), float(duration))


class SimRobot:
    """Drop-in replacement for scripts/red_block/robot.Robot."""

    def __init__(self, world, clock: SimClock):
        self.world = world
        self.clock = clock
        self.dry_run = False
        self.motors = SimMotorTransport(world)
        self._servo_motions: dict[int, tuple[float, float, float, float]] = {}
        self.servos = SimServoTransport(world, self._queue_servo_motion)
        self.clock.servo_updater = self._advance_servo_motions
        self.pose: dict[int, int] = {
            int(k): int(v) for k, v in world.servo_command_pulses.items()
        }
        # Unlike physical PWM servos, the simulator owns the commanded state in
        # this same process, so its pose telemetry cannot be stale independently.
        self.pose_state_age_s: float | None = 0.0
        self._carry_contact_miss_since: dict[str, float] = {}

    def pose_state_fresh(self, max_age_s: float = 15.0) -> bool:
        del max_age_s
        self.pose_state_age_s = 0.0
        return True

    def carry_intact(self, target_color: str) -> bool:
        """Confirm sustained SIM carry loss instead of aborting on contact chatter.

        MuJoCo can briefly report neither finger in contact during a small chassis
        rotation even while the lifted cube remains between the jaws. Treat one
        such sample as transient. A real drop still fails immediately if the cube
        reaches the floor or if logical grasp identity is cleared, and sustained
        no-contact beyond the short grace window is rejected.
        """
        color = str(target_color)
        self.world._reconcile_grasp_state()
        if self.world.grasp_color != color:
            self._carry_contact_miss_since.pop(color, None)
            return False
        contact = self.world.finger_block_contact(color)
        block = np.asarray(self.world.body_xyz(f"{color}_block"), dtype=float)
        block_z = float(block[2])
        if block_z <= TARGET_BLOCK_LIFT_CENTER_M:
            self._carry_contact_miss_since.pop(color, None)
            return False
        if bool(contact.get("bilateral")):
            self._carry_contact_miss_since.pop(color, None)
            return True
        grip = np.asarray(self.world.site_xyz("grip_site"), dtype=float)
        separation = float(np.linalg.norm(block - grip))
        # The failed trace was still only 24.8 mm from the grip site. A cube that
        # has actually separated well outside its own 30 mm body envelope does
        # not receive grace.
        if separation > 0.040:
            self._carry_contact_miss_since.pop(color, None)
            return False
        now = float(self.world.data.time)
        since = self._carry_contact_miss_since.setdefault(color, now)
        return (now - since) < 0.22

    def remembered_destination_heading_error(self, color: str) -> float | None:
        """Return actor-visible spatial-memory heading error in radians.

        This deliberately uses the RGB/FK/odometry memory already available to
        the robot stack, never MuJoCo object truth.
        """
        entry = getattr(self.world, "spatial_memory", {}).get(str(color))
        xy = entry.get("position_xy") if isinstance(entry, dict) else None
        if not isinstance(xy, (list, tuple)) or len(xy) < 2:
            return None
        base = self.world.base_xyz()
        _, _, yaw = self.world.base_rpy()
        desired = math.atan2(float(xy[1]) - float(base[1]), float(xy[0]) - float(base[0]))
        arm_yaw = float(self.world._arm_qpos()[0])
        camera_heading = float(yaw) + arm_yaw
        return math.atan2(math.sin(desired-camera_heading), math.cos(desired-camera_heading))

    def _invalidate_precision_handoff(self) -> None:
        try:
            precision_handoff.invalidate_pick_plan()
        except Exception:
            pass

    def _advance_servo_motions(self, sim_time: float | None = None) -> None:
        """Update PWM actuator targets along queued hardware-duration ramps."""
        if not self._servo_motions:
            return
        now = float(self.world.data.time if sim_time is None else sim_time)
        updates: dict[int, int] = {}
        finished: list[int] = []
        for servo, (start_pulse, target_pulse, start_t, duration) in list(self._servo_motions.items()):
            u = 1.0 if duration <= 1e-9 else max(0.0, min(1.0, (now - start_t) / duration))
            pulse = int(round(start_pulse + (target_pulse - start_pulse) * u))
            updates[int(servo)] = poses.clamp_pulse(pulse)
            if u >= 1.0 - 1e-9:
                finished.append(int(servo))
        if updates:
            self.world.set_servo_pulses(updates)
        for servo in finished:
            self._servo_motions.pop(servo, None)

    def _queue_servo_motion(self, servo: int, pulse: int, duration: float) -> None:
        servo = int(servo)
        pulse = poses.clamp_pulse(int(pulse))
        duration = min(3.0, max(0.1, float(duration)))
        now = float(self.world.data.time)
        # Materialize any existing trajectory at the current time before
        # replacing it, exactly like sending a new command to a moving servo.
        self._advance_servo_motions(now)
        start_pulse = float(self.world.servo_command_pulses.get(servo, self.pose.get(servo, pulse)))
        deadband = max(0.0, float(getattr(self.world, "physical_params", {}).get("servo_deadband_pwm", 0.0)))
        rate = max(1.0, float(getattr(self.world, "physical_params", {}).get("servo_rate_pwm_per_s", 2000.0)))
        delta = abs(float(pulse) - start_pulse)
        # The controller still believes the requested PWM was sent (self.pose is
        # updated by move_servo/nudge_servos), while the simulated physical axis
        # may remain inside its measured deadband or lag behind a too-fast command.
        effective_target = start_pulse if delta <= deadband else float(pulse)
        effective_duration = duration if delta <= deadband else max(duration, delta / rate)
        self._servo_motions[servo] = (start_pulse, effective_target, now, effective_duration)

    def stop(self) -> None:
        self.motors.stop()

    def probe(self) -> dict[str, Any]:
        return {
            "ok": True,
            "controller": "mujoco-real-stack-adapter",
            "battery_mv": None,
            "model": self.world.state().get("model"),
        }

    def move_servo(self, servo: int, pulse: int, duration: float) -> None:
        servo = int(servo)
        pulse = poses.clamp_pulse(int(pulse))
        duration = min(3.0, max(0.1, float(duration)))
        if servo in {3, 4, 5, 6} and self.pose.get(servo) != pulse:
            self._invalidate_precision_handoff()
        self.servos.write_servo(servo, pulse, duration)
        # REAL move_servo blocks while the controller board interpolates the
        # command, then allows a short mechanical settling margin.
        self.pose[servo] = pulse
        self.clock.sleep(duration + 0.15)

    def move_pose(self, target: Mapping[int, int], *, lowering: bool) -> None:
        current = self.pose or None
        # This is the REAL servo ordering/duration function, not a SIM copy.
        for servo, pulse, duration in poses.servo_steps(current, target, lowering=lowering):
            self.move_servo(servo, pulse, duration)
            current = self.pose

    def drive(self, direction: str, speed: int, duration: float) -> None:
        self._invalidate_precision_handoff()
        carry_handoff_mod.invalidate_return_path()
        direction = str(direction)
        speed = int(speed)
        duration = float(duration)
        # REAL traces prove that pure mecanum strafe is not a trustworthy
        # positioning primitive, but they do not yet provide a calibrated
        # cross-axis drift model.  In the REAL-parity simulator fail closed
        # instead of granting an unrealistically perfect lateral translation.
        # Autonomous face repositioning uses the shared rotate+straight dog-leg.
        if direction in {"left", "right"}:
            raise RuntimeError(
                "REAL-parity SIM disables pure lateral wheel motion; "
                "use bounded rotate+straight repositioning"
            )
        if speed < 35:
            raise ValueError(f"wheel speed {speed} is ineffective on this chassis; use 35..40")
        # Use the REAL drive_speeds() mapping itself. The transport adapter then
        # converts physical motor IDs/polarities to the MuJoCo wheel basis.
        for motor, motor_speed in control_model.drive_speeds(direction, speed):
            self.motors.write_motor(motor, motor_speed)
        self.clock.sleep(duration)
        self.motors.stop()

    def move_servos_and_drive(
        self,
        updates: Mapping[int, int],
        *,
        servo_duration: float,
        direction: str,
        speed: int,
        drive_duration: float,
        settle: float = 0.0,
    ) -> None:
        """Mirror REAL's concurrent servo interpolation + bounded chassis move."""
        servo_duration = min(3.0, max(0.1, float(servo_duration)))
        drive_duration = min(2.0, max(0.05, float(drive_duration)))
        settle = max(0.0, float(settle))
        speed = int(speed)
        if speed < 35 or speed > 40:
            raise ValueError(f"parallel chassis speed must be 35..40, got {speed}")
        clean: dict[int, int] = {}
        for servo, pulse in updates.items():
            servo = int(servo)
            pulse = poses.clamp_pulse(int(pulse))
            if self.pose.get(servo) == pulse:
                continue
            if servo in {3, 4, 5, 6}:
                self._invalidate_precision_handoff()
            clean[servo] = pulse
        carry_handoff_mod.invalidate_return_path()
        for servo, pulse in clean.items():
            self.servos.write_servo(servo, pulse, servo_duration)
        self.pose.update(clean)
        for motor, motor_speed in control_model.drive_speeds(str(direction), speed):
            self.motors.write_motor(motor, motor_speed)
        try:
            self.clock.sleep(drive_duration)
        finally:
            self.motors.stop()
        remaining = servo_duration + settle - drive_duration
        if remaining > 0.0:
            self.clock.sleep(remaining)

    def nudge_servos(self, updates: Mapping[int, int], duration: float = 0.1) -> None:
        duration = min(3.0, max(0.1, float(duration)))
        clean: dict[int, int] = {}
        for servo, pulse in updates.items():
            servo = int(servo)
            pulse = poses.clamp_pulse(int(pulse))
            if self.pose.get(servo) == pulse:
                continue
            if servo in {3, 4, 5, 6}:
                self._invalidate_precision_handoff()
            clean[servo] = pulse
        if not clean:
            return
        # Queue all axes concurrently. The physical servo board interpolates
        # each target during `duration` while controller code continues after
        # only 50 ms; SimClock advances those pending ramps on subsequent sleeps.
        for servo, pulse in clean.items():
            self.servos.write_servo(servo, pulse, duration)
        self.pose.update(clean)
        self.clock.sleep(0.05)


@contextmanager
def _patched_modules(runtime: "MigratedRealStack"):
    """Install shared SIM wrappers and select one runtime for this thread.

    The REAL deployment gets process isolation for free; the SIM worker keeps
    these modules resident while public actions may overlap.  Module globals
    therefore must be patched once, while runtime-dependent behavior is routed
    through ContextVars instead of being rebound per action.
    """
    clock = runtime.clock
    camera = runtime.camera
    runtime_token = _ACTIVE_RUNTIME.set(runtime)
    detector_var = getattr(precision_mod, "_TARGET_DETECTOR", None)
    detector_token = detector_var.set(None) if detector_var is not None else None
    with _PATCH_LOCK:
        _install_dispatch_patches_locked()
        global _PATCH_REFCOUNT
        _PATCH_REFCOUNT += 1

    def patch(module: ModuleType, name: str, value: Any) -> None:
        # Dynamic dependencies are already installed as shared dispatchers.
        # Constants are common to all active SIM contexts and are restored only
        # when the last context exits; this prevents one worker from restoring
        # REAL defaults while another worker is still executing.
        key = (module, name)
        if key in _DISPATCH_KEYS:
            return
        with _PATCH_LOCK:
            if key not in _PATCH_ORIGINALS:
                _PATCH_ORIGINALS[key] = getattr(module, name)
                setattr(module, name, value)

    # Module-local time variables are safe to replace; Python's global time
    # module itself is never modified.
    for module in (search_mod, approach_mod, pick_mod, place_mod, precision_mod):
        if hasattr(module, "time"):
            patch(module, "time", clock)

    # Search owns its camera. Give it the same API backed by MuJoCo.
    patch(search_mod, "LiveCamera", lambda: SimCamera(runtime.world, clock))
    # Precision approach/pick instantiate LiveVideo from the controller module.
    patch(precision_mod, "LiveVideo", lambda _url=None: SimCamera(runtime.world, clock))
    # REAL currently carries a provisional 17.5-cm capture reach chosen from
    # physical probing.  The MuJoCo twin has separate, explicitly uncalibrated
    # contact geometry; its own prior sweep records bilateral grasp only around
    # 15.5--16.5 cm.  Do not let the old oversized SIM finger proxy hide that
    # mismatch.  Use the centre of the SIM-supported band for SIM execution only
    # while leaving every REAL constant untouched.
    # Re-derived after replacing the oversized legacy collision boxes with the
    # actual slim MasterPi pad envelope.  At the reproduced pre-grasp pose the
    # 14.75-cm TCP radius aligns longitudinally with the 30-mm cube instead of
    # contacting it early or closing in front of it.
    patch(precision_mod, "CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM", 2.50)
    patch(precision_mod, "CAPTURE_BLOCK_RADIUS_CM", 14.50)
    patch(precision_mod, "CAPTURE_FINGERTIP_RADIUS_CM", 15.00)
    # The centered measured-fisheye camera does not use normalized image centre
    # as optical centre (measured cx=287.689 px at 640 px width).  Couple the
    # final image-servo stop to the calibrated SIM grasp TCP rather than the
    # archived pre-recenter visual window.
    patch(precision_mod, "CAPTURE_TARGET_NX", SIM_CAPTURE_TARGET_NX)
    patch(precision_mod, "CAPTURE_TARGET_NY", SIM_CAPTURE_TARGET_NY)
    patch(precision_mod, "CAPTURE_CREEP_SECONDS", SIM_CAPTURE_CREEP_SECONDS)
    patch(
        precision_mod,
        "CAPTURE_CREEP_MIN_PROGRESS_NY",
        SIM_CAPTURE_CREEP_MIN_PROGRESS_NY,
    )
    patch(
        precision_mod,
        "CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES",
        SIM_CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES,
    )
    patch(
        precision_mod,
        "CAPTURE_MAX_CREEP_PULSES",
        SIM_CAPTURE_MAX_CREEP_PULSES,
    )
    patch(
        precision_mod,
        "FACE_REPOSITION_RESTORE_TURN_SECONDS",
        SIM_FACE_REPOSITION_RESTORE_TURN_SECONDS,
    )
    if SIM_FAST_CAMERA_CONTROL:
        # MuJoCo camera output is deterministic: keep independent stopped
        # samples and all final physical postconditions, but do not render the
        # same fixed view as many times as a noisy USB camera. REAL defaults are
        # restored when the last SIM action context exits.
        for name, value in {
            "ACQUIRE_CONFIRMATIONS": 1,
            "CENTER_CONFIRMATIONS": 1,
            "FINAL_X_CONFIRMATIONS": 1,
            "FINAL_SAMPLE_COUNT": 3,
            "FINAL_MIN_VALID_SAMPLES": 2,
            "APPROACH_RANGE_SAMPLE_COUNT": 1,
            "APPROACH_RANGE_MIN_VALID": 1,
            "CAPTURE_SAMPLE_COUNT": 1,
            "CAPTURE_SAMPLE_MIN_HITS": 1,
            "VERIFY_FRAME_COUNT": 3,
            "VERIFY_MIN_REFERENCE_HITS": 2,
            # Human-like SIM control: accept an already-safe face angle and use
            # larger far-view creeps. All close-view/hard-radius gates and the
            # physical grasp evaluator remain unchanged.
            "FACE_ROUTE_TARGET_ERROR_DEG": 10.0,
            "PRECAPTURE_FAST_CREEP_SECONDS": 0.18,
            "PRECAPTURE_MID_CREEP_SECONDS": 0.12,
            "PRECAPTURE_NEAR_CREEP_SECONDS": 0.06,
        }.items():
            patch(precision_mod, name, value)
        # The close-view verifier still aborts above 20 degrees. In the
        # deterministic SIM, an already observed <=18 degree face error keeps a
        # two-degree guard below that hard gate and does not justify repeated
        # chassis dog-legs that each back away and re-approach the cube.
        patch(approach_mod, "COARSE_FACE_ACCEPT_DEG", 18.0)
        if SIM_FAST_CONTINUOUS_APPROACH:
            for name, value in {
                "COARSE_STREAM_MAX_SECONDS": 0.75,
                "COARSE_STREAM_MAX_FRAMES": 6,
                "COARSE_STREAM_FRAME_DELAY_SECONDS": 0.10,
            }.items():
                patch(precision_mod, name, value)
    patch(
        precision_mod,
        "capture_target_pixel_from_hand_eye",
        lambda *args, **kwargs: (SIM_CAPTURE_TARGET_NX, SIM_CAPTURE_TARGET_NY),
    )
    # Production robot_cam is now mechanically centered with zero optical yaw.
    # The former +68-pulse (~5.4 deg) compensation belonged to the archived
    # off-centre one-pose camera fit and must not survive that mount change.
    patch(precision_mod, "HAND_EYE_YAW_OFFSET_PULSE", 0)
    # Keep the exact REAL calibrated controller envelope in simulation. A
    # digital twin must not widen IK limits merely to make a SIM grasp pass.
    # Grasp verification deliberately uses the physical /snapshot capture path
    # rather than LiveVideo. Route that same sensor boundary to MuJoCo as well;
    # otherwise migrated pick falls through to /dev/video0 on the Oracle host.
    patch(precision_mod, "capture_bgr", camera.read)
    patch(pick_mod, "capture_bgr", camera.read)

    # The measured fisheye close-view can pin a floor cube's bbox bottom against
    # the image boundary, so image-y may barely change during a genuine straight
    # retreat. Preserve the REAL visual guard, but in SIM accept the retreat when
    # the robot's own odometry proves >=3 cm backward travel before that guard
    # reports low visual progress. This is actor-available self-motion evidence,
    # not privileged object truth.
    original_retreat_for_body_alignment = _original(
        precision_mod, "retreat_for_body_alignment"
    )

    def _sim_retreat_for_body_alignment(robot, video, lock, gaze, **kwargs):
        start_xy = np.asarray(runtime.world.base_xyz()[:2], dtype=float).copy()
        try:
            return original_retreat_for_body_alignment(robot, video, lock, gaze, **kwargs)
        except RuntimeError as exc:
            if "straight retreat did not create visual body-alignment clearance" not in str(exc):
                raise
            end_xy = np.asarray(runtime.world.base_xyz()[:2], dtype=float)
            travelled = float(np.linalg.norm(end_xy - start_xy))
            if travelled < 0.030:
                raise
            robot.stop()
            print(
                f"SIM body-clearance fallback: odometry confirms {travelled:.3f}m straight retreat; "
                "continuing body alignment despite fisheye ny saturation",
                flush=True,
            )
            return gaze

    patch(precision_mod, "retreat_for_body_alignment", _sim_retreat_for_body_alignment)
    # REAL place uses capture_bgr directly.
    patch(place_mod, "capture_bgr", camera.read)
    # A carried block shares the arm with the camera. The REAL delivery scanner
    # sweeps shoulder/base joints, which can physically throw a carried block in
    # the current digital twin. In SIM only, keep the manipulator frozen while
    # searching/centering and use actor-visible spatial memory + chassis yaw.
    original_acquire_target = _original(place_mod, "acquire_target")
    original_center_target = _original(place_mod, "center_target")
    original_align_body = _original(place_mod, "align_body_to_target")

    def _sim_require_carry(robot, target_color: str) -> None:
        checker = getattr(robot, "carry_intact", None)
        if callable(checker) and not checker(target_color):
            robot.stop()
            raise place_mod.PlaceError(f"carried {target_color} block was lost during delivery; aborted immediately")

    def _sim_safe_acquire_target(robot, color: str):
        carried = str(getattr(runtime.world, "grasp_color", None) or "")
        if not carried:
            return original_acquire_target(robot, color)
        _sim_require_carry(robot, carried)

        # Carry search invariant: once a cube is physically grasped, freeze every
        # arm joint. The eye-in-hand camera therefore moves only with the chassis.
        # The reproduced failure showed shoulder/base scanning can unload both
        # contacts even while servo1 remains commanded closed.
        frame, blob = place_mod._detect(color)
        if blob is not None:
            return blob

        heading_error = getattr(robot, "remembered_destination_heading_error", None)
        if not callable(heading_error):
            raise place_mod.PlaceError(
                f"{color} destination has no actor-visible heading memory; refusing blind carry sweep"
            )

        stalled_turns = 0
        for _ in range(CARRY_SEARCH_MAX_GUIDED_TURNS):
            hint = heading_error(color)
            if hint is None:
                raise place_mod.PlaceError(
                    f"{color} destination heading memory became unavailable during carry search"
                )
            turn = carry_search_turn_for_heading_error(hint)
            if turn is None:
                # We reached the remembered bearing but still have no visual
                # confirmation.  Do not start an unbounded opposite-direction
                # sweep while carrying; that was the destructive old behavior.
                break
            direction, duration = turn
            robot.drive(direction, 31, duration)
            _sim_require_carry(robot, carried)
            frame, blob = place_mod._detect(color)
            if blob is not None:
                return blob

            new_hint = heading_error(color)
            if new_hint is None:
                raise place_mod.PlaceError(
                    f"{color} destination heading memory became unavailable after body turn"
                )
            if abs(new_hint) >= abs(hint) - CARRY_SEARCH_MIN_PROGRESS_RAD:
                stalled_turns += 1
                if stalled_turns >= CARRY_SEARCH_MAX_STALLED_TURNS:
                    raise place_mod.PlaceError(
                        f"{color} destination body turn made no odometry progress; refusing blind carry rotation"
                    )
            else:
                stalled_turns = 0

        frame, blob = place_mod._detect(color)
        if blob is not None:
            return blob
        final_hint = heading_error(color)
        degrees = None if final_hint is None else round(math.degrees(final_hint), 1)
        raise place_mod.PlaceError(
            f"{color} target not visible near remembered heading (error_deg={degrees}); "
            "refusing blind carry sweep"
        )

    def _sim_safe_center_target(robot, color: str, *, confirmations=place_mod.CENTER_CONFIRMATIONS):
        carried = str(getattr(runtime.world, "grasp_color", None) or "")
        stable = 0
        misses = 0
        last_blob = None
        for _ in range(place_mod.MAX_CENTER_FRAMES):
            if carried:
                _sim_require_carry(robot, carried)
            frame, blob = place_mod._detect(color)
            if blob is None:
                misses += 1
                if misses >= 5:
                    raise place_mod.PlaceError(f"{color} target lost while body-centering")
                continue
            misses = 0
            if place_mod.maybe_enter_near_delivery_view(robot, blob):
                if carried:
                    _sim_require_carry(robot, carried)
                stable = 0
                last_blob = None
                continue
            last_blob = blob
            estimate = place_mod.estimate_block(robot.pose, blob)
            if estimate is None:
                raise place_mod.PlaceError(f"{color} target has no valid calibrated bearing while body-centering")
            turn = place_mod.carry_body_turn_for_bearing_deg(estimate.yaw_left_deg)
            if turn is None:
                stable += 1
                if stable >= confirmations:
                    return blob
                continue
            stable = 0
            direction, speed, duration = turn
            previous_wrist = int(robot.pose.get(5, place_mod.CARRY_TRANSPORT_WRIST_PULSE))
            place_mod.prepare_carry_chassis_motion(robot)
            if carried:
                _sim_require_carry(robot, carried)
            robot.drive(direction, speed, duration)
            robot.stop()
            if carried:
                _sim_require_carry(robot, carried)
            place_mod.restore_post_motion_delivery_view(robot, previous_wrist)
            if carried:
                _sim_require_carry(robot, carried)
        raise place_mod.PlaceError(f"{color} target did not stabilize in body-centered camera view; last={last_blob}")

    def _sim_safe_align_body(robot, color: str):
        return _sim_safe_center_target(robot, color)

    def _sim_safe_approach_target(robot, color: str):
        carried = str(getattr(runtime.world, "grasp_color", None) or "")
        for pulse_number in range(place_mod.MAX_APPROACH_PULSES + 1):
            blob = _sim_safe_align_body(robot, color)
            measured = place_mod.stationary_measurement(robot, color)
            radius = measured.radius_cm
            if place_mod.TARGET_RADIUS_MIN_CM <= radius <= place_mod.TARGET_RADIUS_MAX_CM:
                place_mod.prepare_carry_chassis_motion(robot)
                if carried:
                    _sim_require_carry(robot, carried)
                return measured
            if pulse_number >= place_mod.MAX_APPROACH_PULSES:
                break
            motion = place_mod.carry_approach_motion(radius)
            if motion is None:
                place_mod.prepare_carry_chassis_motion(robot)
                if carried:
                    _sim_require_carry(robot, carried)
                return measured
            direction, speed, duration = motion
            place_mod.prepare_carry_chassis_motion(robot)
            if carried:
                _sim_require_carry(robot, carried)
            robot.drive(direction, speed, duration)
            if carried:
                _sim_require_carry(robot, carried)
            terminal_trim = direction == "forward" and radius <= place_mod.TERMINAL_TRIM_ENTRY_MAX_CM
            if terminal_trim:
                # Mirror REAL approach_target exactly here: after wheel stop,
                # let chassis velocity decay before starting the hover arm move.
                # The previous SIM override returned immediately and the base
                # coasted another ~1.8 cm during the first 50 ms of arm motion.
                clock.sleep(place_mod.CAMERA_SETTLE_SECONDS)
                if carried:
                    _sim_require_carry(robot, carried)
                # Do not perform the destructive close-view wrist transition.
                return place_mod.terminal_staged_estimate(measured)
            place_mod.set_post_motion_delivery_view(robot, close=False)
            if carried:
                _sim_require_carry(robot, carried)
        raise place_mod.PlaceError(
            f"could not enter calibrated place radius [{place_mod.TARGET_RADIUS_MIN_CM:.1f},{place_mod.TARGET_RADIUS_MAX_CM:.1f}]cm"
        )

    patch(place_mod, "acquire_target", _sim_safe_acquire_target)
    patch(place_mod, "center_target", _sim_safe_center_target)
    patch(place_mod, "align_body_to_target", _sim_safe_align_body)
    patch(place_mod, "approach_target", _sim_safe_approach_target)

    place_token = _ACTIVE_PLACE_WRAPPERS.set({
        "acquire_target": _sim_safe_acquire_target,
        "center_target": _sim_safe_center_target,
        "align_body_to_target": _sim_safe_align_body,
        "approach_target": _sim_safe_approach_target,
        "retreat_for_body_alignment": _sim_retreat_for_body_alignment,
    })
    try:
        yield
    finally:
        _ACTIVE_PLACE_WRAPPERS.reset(place_token)
        if detector_var is not None and detector_token is not None:
            detector_var.reset(detector_token)
        _ACTIVE_RUNTIME.reset(runtime_token)
        with _PATCH_LOCK:
            _PATCH_REFCOUNT -= 1
            if _PATCH_REFCOUNT == 0:
                _restore_dispatch_patches_locked()


class MigratedRealStack:
    """REAL task stack executed against a MuJoCo hardware adapter."""

    def __init__(self, world):
        self.world = world
        self.clock = SimClock(world)
        self.robot = SimRobot(world, self.clock)
        self.camera = SimCamera(world, self.clock)

    def sync_pose_from_world(self) -> None:
        self.robot._servo_motions.clear()
        self.robot.pose = {int(k): int(v) for k, v in self.world.servo_command_pulses.items()}

    def reset(self) -> None:
        self.sync_pose_from_world()
        try:
            precision_handoff.invalidate_pick_plan()
        except Exception:
            pass

    def primitive(self, motion: str, *, speed: int = 35, duration: float = 0.30) -> str:
        if motion == "stop":
            self.robot.stop()
            return "REAL primitive stop via MuJoCo adapter"
        self.robot.drive(motion, speed, duration)
        return f"REAL primitive {motion} speed={speed} duration={duration:.2f}s via MuJoCo adapter"

    @staticmethod
    def _target_detector(target_color: str):
        if target_color == "red":
            return search_mod.detect_red_blob
        return lambda frame, **kwargs: camera_mod.detect_target_blob(
            frame, target_color, **kwargs
        )

    def search(self, *, target_color: str = "red", seconds: float = 12.0) -> str:
        """Execute the shared REAL search implementation against MuJoCo I/O."""
        with _patched_modules(self):
            code = search_mod.run_search(
                self.robot,
                seconds=seconds,
                detector=self._target_detector(target_color),
                target_color=target_color,
            )
        if code != 0:
            raise RuntimeError(f"REAL search returned {code}")
        return f"REAL search.py confirmed {target_color} target on MuJoCo camera"

    def track(self, *, target_color: str = "red", seconds: float = 3.0) -> str:
        """Execute the shared REAL PID tracker against MuJoCo I/O."""
        code = track_mod.run_track(
            self.robot,
            capture=self.camera.read,
            seconds=seconds,
            clock=self.clock,
            target_color=target_color,
        )
        if code != 0:
            raise RuntimeError(f"REAL track returned {code}")
        return f"REAL track.py PID centered {target_color} target on MuJoCo camera"

    def approach(self, *, target_color: str = "red") -> str:
        """Execute the shared REAL coarse staging approach against MuJoCo I/O."""
        with _patched_modules(self):
            code = approach_mod.run_approach(
                self.robot,
                precision=precision_mod,
                target_color=target_color,
                fast_camera_control=SIM_FAST_CONTINUOUS_APPROACH,
            )
        if code != 0:
            raise RuntimeError(f"REAL coarse approach returned {code}")
        return f"REAL approach.py established stopped coarse staging for {target_color}"

    def _verify_sim_grasp_postcondition(self, target_color: str) -> None:
        """Evaluate REAL pick outcome with MuJoCo truth without changing motion."""
        self.clock.sleep(0.30)
        contact = self.world.finger_block_contact(target_color)
        block_z = float(self.world.body_xyz(f"{target_color}_block")[2])
        if not bool(contact.get("bilateral")) or block_z <= TARGET_BLOCK_LIFT_CENTER_M:
            self.world.grasp_color = None
            # REAL would only have PROBABLE_HELD here. The simulator knows the
            # physical outcome was false, so invalidate evidence but do not move
            # the robot or invent a SIM-only recovery motion.
            try:
                pick_mod.invalidate_carry_handoff()
            except Exception:
                pass
            raise RuntimeError(
                "shared REAL pick completed but MuJoCo postcondition failed "
                f"for {target_color} (bilateral={bool(contact.get('bilateral'))}, "
                f"z={block_z:.3f}m)"
            )
        self.world.grasp_color = target_color
        if target_color in getattr(self.world, "spatial_memory", {}):
            self.world.spatial_memory[target_color]["relation"] = "HELD"

    def pick(self, *, target_color: str = "red") -> str:
        """Run the shared near-field precision pick, then evaluate physics."""
        with _patched_modules(self):
            code = pick_mod.run_precision_pick(
                self.robot, precision=precision_mod, target_color=target_color
            )
        if code != 0:
            raise RuntimeError(f"REAL near-field precision pick returned {code}")
        self._verify_sim_grasp_postcondition(target_color)
        return (
            f"REAL pick.py near-field precision grasp completed for {target_color}; "
            "MuJoCo evaluator confirmed the physical grasp"
        )

    def put_down(self) -> str:
        """Reverse the shared REAL pickup path and verify the object was released."""
        before = self.world.grasp_color
        if before not in {"red", "blue", "yellow"}:
            # Actor state can be only PROBABLE_HELD, but the SIM evaluator knows
            # whether there is a physical object. Still run the shared safety gate
            # only when the physics says an object is actually carried.
            raise RuntimeError("shared REAL put_down requested while MuJoCo gripper carries no object")
        with _patched_modules(self):
            released = put_down_mod.execute_put_down(self.robot)
        self.clock.sleep(0.25)
        contact = self.world.finger_block_contact(before)
        block_z = float(self.world.body_xyz(f"{before}_block")[2])
        if bool(contact.get("bilateral")) or block_z > TARGET_BLOCK_LIFT_CENTER_M + 0.010:
            raise RuntimeError(
                "shared REAL put_down completed but MuJoCo release postcondition failed "
                f"for {before} (bilateral={bool(contact.get('bilateral'))}, z={block_z:.3f}m)"
            )
        self.world.grasp_color = None
        if before in getattr(self.world, "spatial_memory", {}):
            self.world.spatial_memory[before]["relation"] = "OBSERVED"
        return f"REAL put_down.py returned {released} to its pickup site; MuJoCo evaluator confirmed release"

    def _verify_sim_place_postcondition(
        self, target_color: str, destination_color: str
    ) -> None:
        """Evaluate the shared REAL place outcome without adding SIM motion."""
        target_xyz = self.world.body_xyz(f"{target_color}_block")
        destination_xyz = self.world.body_xyz(f"{destination_color}_block")
        planar = float(np.linalg.norm(target_xyz[:2] - destination_xyz[:2]))
        vertical = float(target_xyz[2] - destination_xyz[2])
        # The old evaluator still encoded the archived 50-mm-cube condition
        # (dz 38..70 mm) after production was switched to the measured 30-mm
        # task cubes.  Derive the evaluator from the actual twin geometry so it
        # cannot silently drift from the scene again.  A valid stack keeps the
        # upper center well inside the lower top face and one cube side above it.
        expected_vertical = float(TARGET_BLOCK_SIDE_M)
        vertical_tolerance = max(0.003, expected_vertical * 0.20)
        max_planar = float(TARGET_BLOCK_HALF_M) * 0.90
        if planar > max_planar or abs(vertical - expected_vertical) > vertical_tolerance:
            raise RuntimeError(
                "shared REAL place completed but MuJoCo stack postcondition failed "
                f"(xy={planar:.3f}m>{max_planar:.3f}m or "
                f"dz={vertical:.3f}m expected={expected_vertical:.3f}±{vertical_tolerance:.3f}m)"
            )
        self.world.grasp_color = None
        if target_color in getattr(self.world, "spatial_memory", {}):
            self.world.spatial_memory[target_color]["relation"] = (
                f"ON_{destination_color.upper()}"
            )

    def search_destination(self, *, target_color: str, destination_color: str) -> str:
        """Run the shared delivery-safe destination acquisition while carrying."""
        if self.world.grasp_color != target_color:
            raise place_mod.PlaceError(f"carried {target_color} block is not physically confirmed before destination search")
        with _patched_modules(self):
            place_mod._require_closed_gripper(self.robot, target_color)
            place_mod.acquire_target(self.robot, destination_color)
        return f"REAL place.py delivery search located {destination_color} while carrying {target_color}"

    def place(self, *, target_color: str, destination_color: str) -> str:
        """Execute the shared REAL place.py implementation against MuJoCo I/O."""
        with _patched_modules(self):
            place_mod.execute_place(self.robot, target_color, destination_color)
        self._verify_sim_place_postcondition(target_color, destination_color)
        return (
            f"REAL place.py visually verified {target_color}-on-{destination_color}; "
            "MuJoCo evaluator confirmed the stack"
        )

    # Compatibility wrappers contain no policy. They keep older internal tests
    # and callers working while all behavior comes from the generic REAL entrypoints.
    def search_red(self, *, seconds: float = 12.0) -> str:
        return self.search(target_color="red", seconds=seconds)

    def track_red(self, *, seconds: float = 3.0) -> str:
        return self.track(target_color="red", seconds=seconds)

    def approach_red(self) -> str:
        return self.approach(target_color="red")

    def pick_red(self) -> str:
        return self.pick(target_color="red")

    def place_red_on(self, color: str) -> str:
        return self.place(target_color="red", destination_color=color)
