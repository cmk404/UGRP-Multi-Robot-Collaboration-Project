"""Robot-local RGB camera and raw actuator boundary for planner runtimes."""

from __future__ import annotations

import base64
import hashlib
import math
from collections.abc import Mapping
from typing import Any


_STOP = (0.0, 0.0, 0.0, 0.0)
_ACTION_FIELDS = {
    "drive": frozenset({"kind", "forward", "turn", "duration_s"}),
    "look": frozenset({"kind", "pan_pulse"}),
    "arm": frozenset({"kind", "servo_id", "pulse"}),
    "wait": frozenset({"kind"}),
}


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

    def __init__(self, world: Any, rid: str):
        self._world = world
        self.robot_id = str(rid)
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

    def apply(self, action: Mapping[str, Any], sim_time: float) -> dict[str, Any]:
        """Validate and issue one raw command, returning acknowledgement only."""
        now = _finite_number("sim_time", sim_time)
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

        if kind == "drive":
            forward = _bounded_number("forward", action["forward"], 0.0, 0.15)
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
        """Stop a drive whose bounded simulated-time lease has expired."""
        now = _finite_number("sim_time", sim_time)
        self._advance_servos(now)
        if self._drive_expires_at is not None and now >= self._drive_expires_at:
            self._set_motors(_STOP)

    def stop(self) -> None:
        """Immediately stop this robot's wheels."""
        self._set_motors(_STOP)

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
            self._servo_applied[servo] = float(pulse)
            updates[servo] = pulse
            if pulse == target:
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
