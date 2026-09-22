"""Robot-local RGB camera and raw actuator boundary for planner runtimes."""

from __future__ import annotations

import base64
import hashlib
import math
from collections.abc import Mapping
from typing import Any

# Keep this capability boundary importable without the simulation runtime.
# ABAB mixing matches masterpi_dynamics_v2's reduced mecanum wrench basis.
_FORWARD_PATTERN = (1., 1., 1., 1.)
_LEFT_PATTERN = (-1., 1., 1., -1.)
_YAW_LEFT_PATTERN = (-1., 1., -1., 1.)


_STOP = (0.0, 0.0, 0.0, 0.0)
_ACTION_FIELDS = {
    "drive": frozenset({"kind", "forward", "turn", "duration_s"}),
    "mecanum": frozenset({"kind", "forward", "left", "turn", "duration_s"}),
    "look": frozenset({"kind", "pan_pulse"}),
    "arm": frozenset({"kind", "servo_id", "pulse"}),
    "wait": frozenset({"kind"}),
}


def validate_raw_action(action: Mapping[str, Any], *, allow_reverse: bool = False,
                        allow_mecanum: bool = False) -> None:
    """Validate the raw command contract without constructing a simulator."""
    if not isinstance(action, Mapping):
        raise ValueError("action must be an object")
    kind = action.get("kind")
    if not isinstance(kind, str) or kind not in _ACTION_FIELDS:
        raise ValueError("unknown action type")
    unknown = set(action) - _ACTION_FIELDS[kind]
    missing = _ACTION_FIELDS[kind] - set(action)
    if unknown:
        raise ValueError(f"unknown {kind} action fields: {sorted(unknown)}")
    if missing:
        raise ValueError(f"missing {kind} action fields: {sorted(missing)}")

    if kind in {"drive", "mecanum"}:
        if kind == "mecanum" and not allow_mecanum:
            raise ValueError("mecanum action requires allow_mecanum=True")
        _bounded_number("forward", action["forward"], -0.05 if allow_reverse else 0.0, 0.15)
        _bounded_number("turn", action["turn"], -.15 if kind == "mecanum" else -.2,
                        .15 if kind == "mecanum" else .2)
        _bounded_number("duration_s", action["duration_s"], 0.0, 1.0)
        if kind == "mecanum":
            _bounded_number("left", action["left"], -.10, .10)
    elif kind == "look":
        _bounded_int("pan_pulse", action["pan_pulse"], 500, 2500)
    elif kind == "arm":
        servo = _bounded_int("servo_id", action["servo_id"], 1, 5)
        if servo not in {1, 3, 4, 5}:
            raise ValueError("servo_id must be one of 1, 3, 4, or 5")
        _bounded_int("pulse", action["pulse"], 500, 2500)


class CameraRobotPort:
    """Expose one robot's RGB camera and command actuators only.

    This class is an object-capability boundary, not a process sandbox.  It
    deliberately provides no cargo, obstacle, peer, pose, inverse-kinematics,
    pickup, or weld API.  A caller given the underlying ``world`` can still
    bypass this boundary, so the runtime must give untrusted planners only this
    port (or serialized values returned by it).

    The physics owner must call :meth:`tick` as simulated time advances.  It
    should use ``world._physics_step_for(world.robot(rid))`` for the current
    multi-MasterPi implementation.  Rendering is brokered by the world and may
    wait on its render thread; callers must never hold ``world.physics_lock``
    while awaiting :meth:`capture`.
    """

    def __init__(self, world: Any, rid: str, *, allow_reverse: bool = False,
                 allow_mecanum: bool = False):
        self._world = world
        self.robot_id = str(rid)
        self._allow_reverse = bool(allow_reverse)
        self._allow_mecanum = bool(allow_mecanum)
        self._robot = world.robot(rid)
        self._frame_id = 0
        self._motor_commands = _STOP
        initial = {int(key): int(value) for key, value in self._robot.servo_command_pulses.items()}
        self._servo_pulses: dict[int, int] = dict(initial)
        self._servo_applied: dict[int, float] = {key: float(value) for key, value in initial.items()}
        self._servo_targets: dict[int, int] = {}
        self._servo_tick_time = self._clock_time()
        self._drive_expires_at: float | None = None
        self._busy_until = 0.0
        self._command_expires_at: float | None = None

    @property
    def busy_until(self) -> float:
        """Earliest simulated time at which the last command may be settled."""
        return self._busy_until

    def capture(self, camera: str = "robot_cam") -> dict[str, Any]:
        """Return a freshly owned JPEG and only this port's command state."""
        if camera not in {"robot_cam", "nav_cam"}:
            raise ValueError("unknown owned RGB camera")
        jpeg = bytes(
            self._world.render_jpeg(
                robot_id=self.robot_id, camera=camera
            )
        )
        self._frame_id += 1
        return {
            "robot_id": self.robot_id,
            "frame_id": self._frame_id,
            "sim_time": self._clock_time(),
            "image": base64.b64encode(jpeg).decode("ascii"),
            "sha256": hashlib.sha256(jpeg).hexdigest(),
            "camera": camera,
            "actuator_state": self._actuator_state(),
        }

    def validate_action(self, action: Mapping[str, Any]) -> None:
        """Check a command without touching actuators (for batch preflight)."""
        validate_raw_action(action, allow_reverse=self._allow_reverse,
                            allow_mecanum=self._allow_mecanum)

    def validate_bounded(self, action: Mapping[str, Any], duration_s: float) -> None:
        self.validate_action(action)
        duration = _bounded_number("duration_s", duration_s, 0.0, .25)
        if duration <= 0 or ("duration_s" in action and action["duration_s"] != duration):
            raise ValueError("positive lease and matching drive duration required")

    def apply_bounded(self, action: Mapping[str, Any], sim_time: float,
                      duration_s: float) -> dict[str, Any]:
        """Issue a short command; tick() also expires unfinished arm interpolation."""
        self.validate_bounded(action, duration_s)
        result = self.apply(action, sim_time)
        self._command_expires_at = float(sim_time) + float(duration_s)
        return result

    def apply(self, action: Mapping[str, Any], sim_time: float) -> dict[str, Any]:
        """Validate and issue one raw command, returning acknowledgement only."""
        now = _finite_number("sim_time", sim_time)
        self.validate_action(action)
        self._command_expires_at = None
        kind = action["kind"]

        if kind == "drive":
            forward = _bounded_number("forward", action["forward"], -0.05 if self._allow_reverse else 0.0, 0.15)
            turn = _bounded_number("turn", action["turn"], -0.2, 0.2)
            duration = _bounded_number("duration_s", action["duration_s"], 0.0, 1.0)
            # Simulator wheel order: front-left, front-right, rear-left, rear-right.
            command = (
                forward - turn,
                forward + turn,
                forward - turn,
                forward + turn,
            )
            self._set_motors(command)
            self._drive_expires_at = now + duration
            self._busy_until = self._drive_expires_at
        elif kind == "mecanum":
            if not self._allow_mecanum:
                raise ValueError("mecanum action requires allow_mecanum=True")
            forward = _bounded_number("forward", action["forward"], -0.05 if self._allow_reverse else 0.0, 0.15)
            left = _bounded_number("left", action["left"], -0.10, 0.10)
            turn = _bounded_number("turn", action["turn"], -0.15, 0.15)
            duration = _bounded_number("duration_s", action["duration_s"], 0.0, 1.0)
            mixed = tuple(f * forward + l * left + t * turn for f, l, t in
                          zip(_FORWARD_PATTERN, _LEFT_PATTERN, _YAW_LEFT_PATTERN))
            self._set_motors(tuple(float(value) for value in mixed))
            self._drive_expires_at = now + duration
            self._busy_until = self._drive_expires_at
        elif kind == "look":
            pan = _bounded_int("pan_pulse", action["pan_pulse"], 500, 2500)
            settle = self._queue_servos({6: pan}, now)
            self._busy_until = now + settle
        elif kind == "arm":
            servo = _bounded_int("servo_id", action["servo_id"], 1, 5)
            if servo not in {1, 3, 4, 5}:
                raise ValueError("servo_id must be one of 1, 3, 4, or 5")
            pulse = _bounded_int("pulse", action["pulse"], 500, 2500)
            settle = self._queue_servos({servo: pulse}, now)
            self._busy_until = now + settle
        else:
            self._set_motors(_STOP)
            self._busy_until = now

        return {
            "ok": True,
            "robot_id": self.robot_id,
            "kind": kind,
            "sim_time": now,
            "busy_until": self._busy_until,
            "actuator_state": self._actuator_state(),
        }

    def tick(self, sim_time: float) -> None:
        """Enforce leases on the physics clock, even without coordinator polling."""
        now = _finite_number("sim_time", sim_time)
        if self._command_expires_at is not None and now >= self._command_expires_at:
            self._advance_servos(self._command_expires_at)
            self.hold(now)
            return
        self._advance_servos(now)
        if self._drive_expires_at is not None and now >= self._drive_expires_at:
            self._set_motors(_STOP)

    def stop(self) -> None:
        """Immediately stop this robot's wheels."""
        self._set_motors(_STOP)

    def hold(self, sim_time: float) -> None:
        """Stop wheels and cancel queued arm motion at the last issued setpoint.

        This reads no measured joint state and does not prove stationary motion
        or load retention. Physics may coast or settle under position control.
        Do not advance pending interpolation after a revocation.
        """
        now = _finite_number("sim_time", sim_time)
        if now < self._servo_tick_time:
            raise ValueError("sim_time must not move backwards")
        self._set_motors(_STOP)
        self._servo_targets.clear()
        self._servo_pulses = {servo: int(round(pulse)) for servo, pulse in self._servo_applied.items()}
        self._servo_applied = {servo: float(pulse) for servo, pulse in self._servo_pulses.items()}
        self._servo_tick_time = now
        self._command_expires_at = None
        self._busy_until = now

    def _set_motors(self, command: tuple[float, float, float, float]) -> None:
        owned = tuple(float(value) for value in command)
        self._robot.set_motor_commands(list(owned))
        self._motor_commands = owned
        if owned == _STOP:
            self._drive_expires_at = None

    def _queue_servos(self, updates: dict[int, int], now: float) -> float:
        self._advance_servos(now)
        durations = []
        for servo, target in updates.items():
            current = self._servo_applied.get(servo, float(target))
            durations.append(abs(float(target) - current) / 2000.0)
            self._servo_targets[servo] = target
            self._servo_pulses[servo] = target
        return max(durations, default=0.0)

    def _advance_servos(self, now: float) -> None:
        if now < self._servo_tick_time:
            raise ValueError("sim_time must not move backwards")
        allowance = 2000.0 * (now - self._servo_tick_time)
        updates: dict[int, int] = {}
        for servo, target in tuple(self._servo_targets.items()):
            current = self._servo_applied.get(servo, float(target))
            delta = float(target) - current
            moved = current + max(-allowance, min(allowance, delta))
            pulse = int(round(moved))
            # Preserve fractional command progress. Rounding the accumulator
            # each physics tick can stall a servo when dt allows <=0.5 PWM.
            # Only the issued hardware setpoint is integer; no joint is read.
            self._servo_applied[servo] = moved
            updates[servo] = pulse
            if abs(moved - target) < 1e-9:
                self._servo_targets.pop(servo, None)
        if updates:
            # No forward_only: MuJoCo's physical position actuators own qpos.
            self._robot.set_servo_pulses(updates)
        self._servo_tick_time = now

    def _actuator_state(self) -> dict[str, Any]:
        return {
            "motor_commands": list(self._motor_commands),
            "servo_pulses": {str(key): value for key, value in self._servo_pulses.items()},
        }

    def _clock_time(self) -> float:
        return float(self._world.data.time)


def _finite_number(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _bounded_number(name: str, value: Any, lower: float, upper: float) -> float:
    result = _finite_number(name, value)
    if not lower <= result <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper}")
    return result


def _bounded_int(name: str, value: Any, lower: int, upper: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not lower <= value <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper}")
    return int(value)
