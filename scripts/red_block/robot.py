"""I2C motor/servo wrapper around masterpi_control.py."""

from __future__ import annotations

import importlib.util
import json
import os
import time
from pathlib import Path
from typing import Any, Mapping

from poses import clamp_pulse, servo_steps
from recorder import event as trace_event, record_pose


SCRIPT_DIR = Path(__file__).resolve().parent
REMOTE_DIR = Path("/home/ugrp1/MasterPi/tools")
CONTROL_CANDIDATES = (
    SCRIPT_DIR.parent / "masterpi_control.py",
    REMOTE_DIR / "masterpi_control.py",
)



POSE_TRUST_MAX_AGE_SECONDS = 15.0
LEGACY_CONTROLLER_BASENAME = "red_block_pick_state_machine.py"


def _legacy_controller_pids() -> list[int]:
    """Return active legacy raw-I2C controllers without touching them."""
    if not Path("/proc").is_dir():
        return []
    found: list[int] = []
    me = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == me:
            continue
        try:
            raw = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "ignore")
        except OSError:
            continue
        if LEGACY_CONTROLLER_BASENAME in raw and "python" in raw.lower():
            found.append(pid)
    return sorted(found)


def _pose_state_age_seconds(control: Any) -> float | None:
    path = getattr(control, "POSE_STATE_PATH", None)
    if path is None:
        return None
    try:
        raw = json.loads(Path(path).read_text())
        updated_at = float(raw["updated_at"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    return max(0.0, time.time() - updated_at)


def is_robot() -> bool:
    return Path("/dev/video0").exists() and any(path.is_file() for path in CONTROL_CANDIDATES)


def _invalidate_precision_handoff() -> None:
    try:
        from precision_handoff import invalidate_pick_plan
        invalidate_pick_plan()
    except Exception:
        # Motion safety must not depend on optional handoff bookkeeping.
        pass


def _invalidate_carry_handoff() -> None:
    try:
        from carry_handoff import invalidate_carry_handoff
        invalidate_carry_handoff()
    except Exception:
        # Releasing the gripper must never depend on optional bookkeeping.
        pass


def _invalidate_carry_return_path() -> None:
    try:
        from carry_handoff import invalidate_return_path
        invalidate_return_path()
    except Exception:
        # Chassis motion must never leave a stale absolute pickup-site path.
        pass


def _invalidate_near_look_handoff() -> None:
    try:
        from near_look_handoff import invalidate_near_look_handoff
        invalidate_near_look_handoff()
    except Exception:
        # Motion safety must not depend on optional handoff bookkeeping.
        pass


def load_control() -> Any:
    for path in CONTROL_CANDIDATES:
        if not path.is_file():
            continue
        spec = importlib.util.spec_from_file_location("masterpi_control", path)
        if spec is None or spec.loader is None:
            continue
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    raise FileNotFoundError("masterpi_control.py not found next to this package or on the Pi")


class Robot:
    def __init__(self, *, dry_run: bool = False):
        self.dry_run = dry_run
        self.control = load_control()
        self._actuator_lease = None
        if not dry_run:
            legacy_pids = _legacy_controller_pids() if is_robot() else []
            if legacy_pids:
                raise RuntimeError(
                    "legacy raw-I2C controller is already active "
                    f"(pid(s) {legacy_pids}); refusing concurrent actuation without stopping it"
                )
            acquire = getattr(self.control, "acquire_actuator_lease", None)
            if callable(acquire):
                self._actuator_lease = acquire(owner=f"red_block:{Path(os.path.basename(__file__)).stem}:{os.getpid()}")
        self.motors = self.control.MotorTransport(dry_run=dry_run)
        self.servos = self.control.ServoTransport(dry_run=dry_run)
        loader = getattr(self.control, "load_pose_state", None)
        self.pose: dict[int, int] = loader() if is_robot() and callable(loader) else {}
        self.pose_state_age_s = _pose_state_age_seconds(self.control) if is_robot() else None
        trace_event("robot_init", dry_run=bool(self.dry_run), is_robot=bool(is_robot()))
        record_pose(self.pose, reason="robot_init", pose_state_age_s=self.pose_state_age_s)

    def pose_state_fresh(self, max_age_s: float = POSE_TRUST_MAX_AGE_SECONDS) -> bool:
        age = _pose_state_age_seconds(self.control)
        self.pose_state_age_s = age
        return age is not None and age <= max_age_s

    def stop(self) -> None:
        trace_event("motor_stop", context="pick-red")
        self.control.stop_all(self.motors, context="pick-red")

    def probe(self) -> dict[str, Any]:
        if self.dry_run:
            return {"ok": True, "controller": "dry-run", "battery_mv": None}
        # The legacy controller frequently tears the two-byte battery read. On
        # real hardware we observe bursts such as 65433/65494/0 mV between
        # stable ~7.4 V samples, sometimes for 3-5 reads in a row. A single
        # valid sample is therefore not enough to prove power health, while a
        # fixed three-attempt retry can falsely reject a healthy robot. Require
        # two mutually-consistent valid samples inside a bounded window. Real
        # undervoltage still fails because probe_hardware rejects every sample
        # outside the physical 4.5..9.5 V range.
        max_attempts = 12
        consistency_mv = 500
        valid: list[dict[str, Any]] = []
        last_error: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            try:
                result = dict(self.control.probe_hardware())
                mv = int(result["battery_mv"])
                for previous in reversed(valid):
                    previous_mv = int(previous["battery_mv"])
                    if abs(mv - previous_mv) <= consistency_mv:
                        # Report the midpoint of two agreeing reads rather than
                        # privileging whichever byte pair happened to arrive last.
                        result["battery_mv"] = int(round((mv + previous_mv) / 2.0))
                        result["probe_attempts"] = attempt
                        result["probe_valid_samples"] = len(valid) + 1
                        trace_event("hardware_probe", result=result)
                        return result
                valid.append(result)
                last_error = RuntimeError(
                    "battery telemetry did not yet produce two consistent valid samples"
                )
            except Exception as exc:
                last_error = exc
            if attempt < max_attempts:
                time.sleep(0.03)
        if valid:
            values = [int(item["battery_mv"]) for item in valid]
            raise RuntimeError(
                f"battery telemetry unstable after {max_attempts} attempts; valid samples={values}"
            ) from last_error
        assert last_error is not None
        raise last_error

    def move_servo(self, servo: int, pulse: int, duration: float) -> None:
        pulse = clamp_pulse(pulse)
        duration = min(3.0, max(0.1, duration))
        if servo in {3, 4, 5, 6} and self.pose.get(servo) != pulse:
            _invalidate_precision_handoff()
        if servo in {4, 5} and self.pose.get(servo) != pulse:
            _invalidate_near_look_handoff()
        if servo == 1 and pulse > 1700:
            _invalidate_carry_handoff()
        print(f"servo {servo} -> {pulse} ({duration:.2f}s)", flush=True)
        trace_event("servo_command", servo=int(servo), from_pulse=self.pose.get(int(servo)), to_pulse=int(pulse), duration_s=float(duration), mode="blocking")
        self.servos.write_servo(servo, pulse, duration)
        if not self.dry_run:
            time.sleep(duration + 0.15)
        self.pose[servo] = pulse
        record_pose(self.pose, reason="move_servo")

    def move_pose(self, target: Mapping[int, int], *, lowering: bool) -> None:
        current = self.pose or None
        for servo, pulse, duration in servo_steps(current, target, lowering=lowering):
            self.move_servo(servo, pulse, duration)

    def reassert_pose_together(
        self,
        target: Mapping[int, int],
        *,
        duration: float = 1.25,
    ) -> None:
        """Re-command a known safe pose concurrently and wait for completion.

        This is for recovering trust after the commanded-pose timestamp becomes
        stale.  The previous search path cleared ``self.pose`` and then waited
        1.5 s *per servo*, even though the MasterPi PWM controller can start
        each joint move immediately and let the joints interpolate together.
        We still wait for the full requested motion before using camera geometry.
        """
        duration = min(3.0, max(0.3, float(duration)))
        normalized = {int(k): clamp_pulse(int(v)) for k, v in target.items()}
        if any(servo in {3, 4, 5, 6} for servo in normalized):
            _invalidate_precision_handoff()
        if any(servo in {4, 5} for servo in normalized):
            _invalidate_near_look_handoff()
        if normalized.get(1, 0) > 1700:
            _invalidate_carry_handoff()
        print(
            "reassert pose together "
            + " ".join(f"{servo}={pulse}" for servo, pulse in sorted(normalized.items()))
            + f" ({duration:.2f}s)",
            flush=True,
        )
        trace_event("servo_batch_command", updates=normalized, duration_s=float(duration), mode="reassert_concurrent")
        for servo, pulse in normalized.items():
            # Do not skip equal commanded values: the point of this method is
            # to regain trust when another/raw controller may have moved them.
            self.servos.write_servo(servo, pulse, duration)
            self.pose[servo] = pulse
        if not self.dry_run:
            time.sleep(duration + 0.15)
        record_pose(self.pose, reason="reassert_pose_together")

    def drive(self, direction: str, speed: int, duration: float) -> None:
        _invalidate_precision_handoff()
        _invalidate_near_look_handoff()
        _invalidate_carry_return_path()
        direction = str(direction)
        speed = int(speed)
        duration = float(duration)
        lateral = direction in {"left", "right"}
        max_speed = int(getattr(self.control, "MAX_LATERAL_SPEED", 70) if lateral else 40)
        if speed < 35 or speed > max_speed:
            raise ValueError(
                f"wheel speed {speed} is outside the {direction} chassis policy; use 35..{max_speed}"
            )
        if not (0.05 <= duration <= 2.0):
            raise ValueError(
                f"drive duration {duration:.2f}s is outside the bounded segment policy (0.05..2.0 s)"
            )
        # Mecanum rollers need noticeably more breakaway torque than straight or
        # rotational motion.  A weak lateral pulse often only twists the chassis
        # instead of translating it.  Make explicit left/right commands decisive
        # while leaving every non-lateral controller unchanged.
        requested_speed, requested_duration = speed, duration
        if lateral:
            speed = max(speed, int(getattr(self.control, "PREFERRED_LATERAL_SPEED", 65)))
            duration = max(duration, 0.65)
        print(
            f"drive {direction} speed={speed} duration={duration:.2f}s"
            + (f" (lateral boost from {requested_speed}@{requested_duration:.2f}s)" if lateral and (speed != requested_speed or duration != requested_duration) else ""),
            flush=True,
        )
        wheel_speeds = list(self.control.drive_speeds(direction, speed))
        trace_event(
            "chassis_command", direction=str(direction), speed=int(speed),
            duration_s=float(duration), wheel_speeds=wheel_speeds, dry_run=bool(self.dry_run),
        )
        if self.dry_run:
            return
        self.control._run_motion(
            self.motors,
            wheel_speeds,
            duration,
        )
        trace_event("chassis_command_complete", direction=str(direction), speed=int(speed), duration_s=float(duration))

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
        """Run one servo interpolation and one bounded chassis segment together.

        The PWM board continues servo interpolation after an I2C command is
        accepted, so we do not need threads (and should not race two writers on
        the same I2C bus).  Servo targets are issued first, wheel speeds are
        issued immediately afterwards, the wheels are stopped at their own
        deadline, and only then do we wait out any remaining servo interpolation.

        Callers must use this only when vision/geometry is not sampled during the
        overlap.  Precision grasp/descent and face/range measurement remain
        intentionally sequential.
        """
        servo_duration = min(3.0, max(0.1, float(servo_duration)))
        drive_duration = min(2.0, max(0.05, float(drive_duration)))
        speed = int(speed)
        if speed < 35 or speed > 40:
            raise ValueError(f"parallel chassis speed must be 35..40, got {speed}")
        normalized = {
            int(servo): clamp_pulse(int(pulse)) for servo, pulse in updates.items()
        }
        if any(
            servo in {3, 4, 5, 6} and self.pose.get(servo) != pulse
            for servo, pulse in normalized.items()
        ):
            _invalidate_precision_handoff()
        if any(
            servo in {4, 5} and self.pose.get(servo) != pulse
            for servo, pulse in normalized.items()
        ):
            _invalidate_near_look_handoff()
        if normalized.get(1, 0) > 1700:
            _invalidate_carry_handoff()
        # Any chassis displacement invalidates an arm-only inverse return path.
        _invalidate_carry_return_path()

        wheel_speeds = list(self.control.drive_speeds(direction, speed))
        trace_event(
            "parallel_servo_chassis_command",
            updates=normalized,
            servo_duration_s=servo_duration,
            direction=str(direction),
            speed=speed,
            drive_duration_s=drive_duration,
        )
        print(
            "parallel motion "
            + " ".join(f"s{servo}={pulse}" for servo, pulse in sorted(normalized.items()))
            + f" + {direction}@{speed} for {drive_duration:.2f}s "
            + f"(servo {servo_duration:.2f}s)",
            flush=True,
        )

        # Serialize the I2C writes, not the physical motions.  Once a servo
        # target is accepted the controller interpolates it independently.
        started = time.monotonic()
        for servo, pulse in normalized.items():
            if self.pose.get(servo) == pulse:
                continue
            self.servos.write_servo(servo, pulse, servo_duration)
            self.pose[servo] = pulse

        if self.dry_run:
            for motor, motor_speed in wheel_speeds:
                self.motors.write_motor(motor, motor_speed)
            for motor in range(1, 5):
                self.motors.write_motor(motor, 0)
            record_pose(self.pose, reason="parallel_servo_chassis")
            return

        self.control.stop_all(self.motors, context="parallel-pre-motion")
        try:
            for motor, motor_speed in wheel_speeds:
                self.motors.write_motor(motor, motor_speed)
            time.sleep(drive_duration)
        finally:
            self.control.stop_all(self.motors, context="parallel-motion-finally")

        elapsed = time.monotonic() - started
        remaining = servo_duration + max(0.0, float(settle)) - elapsed
        if remaining > 0.0:
            time.sleep(remaining)
        record_pose(self.pose, reason="parallel_servo_chassis")

    def nudge_servos(self, updates: Mapping[int, int], duration: float = 0.1) -> None:
        duration = min(3.0, max(0.1, duration))
        if any(
            int(servo) in {3, 4, 5, 6}
            and self.pose.get(int(servo)) != clamp_pulse(int(pulse))
            for servo, pulse in updates.items()
        ):
            _invalidate_precision_handoff()
        if any(
            int(servo) in {4, 5}
            and self.pose.get(int(servo)) != clamp_pulse(int(pulse))
            for servo, pulse in updates.items()
        ):
            _invalidate_near_look_handoff()
        changed = False
        requested = {int(servo): clamp_pulse(int(pulse)) for servo, pulse in updates.items()}
        if requested.get(1, 0) > 1700:
            _invalidate_carry_handoff()
        trace_event("servo_batch_command", updates=requested, duration_s=float(duration), mode="nudge_concurrent")
        for servo, pulse in updates.items():
            pulse = clamp_pulse(int(pulse))
            if self.pose.get(servo) == pulse:
                continue
            self.servos.write_servo(servo, pulse, duration)
            self.pose[servo] = pulse
            changed = True
        if changed and not self.dry_run:
            time.sleep(0.05)
        if changed:
            record_pose(self.pose, reason="nudge_servos")
