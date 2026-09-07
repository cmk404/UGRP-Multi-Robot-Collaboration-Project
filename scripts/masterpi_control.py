#!/usr/bin/env python3
"""Safe, bounded CLI for the legacy MasterPi I2C motor controller."""

from __future__ import annotations

import argparse
import atexit
import fcntl
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence


I2C_TRANSFER = "/usr/sbin/i2ctransfer"
I2C_BUS = 1
I2C_ADDRESS = 0x7A
MOTOR_REGISTER_BASE = 31
SERVO_REGISTER = 40
SERVO_IDS = frozenset({1, 3, 4, 5, 6})
SERVO_PULSE_MIN = 500
SERVO_PULSE_MAX = 2500
SERVO_DURATION_MIN = 0.1
SERVO_DURATION_MAX = 3.0
BATTERY_REGISTER = 0
BATTERY_MIN_MV = 4500
BATTERY_MAX_MV = 9500
PROBE_TIMEOUT = 2.0
# A healthy i2ctransfer write completes in milliseconds. Bound every actuator
# write so a wedged bus cannot trap the motion-finally stop path indefinitely.
I2C_WRITE_TIMEOUT = 0.5
CONTROLLER_ID = f"legacy-i2c-0x{I2C_ADDRESS:02x}"
STOP_RETRIES = 3
STOP_RETRY_DELAY = 0.02
POSE_STATE_PATH = Path(os.environ.get("UGRP_ROBOT_POSE_STATE", "/tmp/ugrp-masterpi-pose.json"))
ACTUATOR_LOCK_PATH = Path(os.environ.get("UGRP_ACTUATOR_LOCK", "/tmp/ugrp-masterpi-actuator.lock"))


def acquire_actuator_lease(
    *,
    owner: str,
    path: Path = ACTUATOR_LOCK_PATH,
):
    """Acquire the one-writer lease for physical actuators.

    The returned file object must stay alive for the whole high-level action.
    Kernel flock ownership disappears automatically if the process dies.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.seek(0)
        holder = handle.read().strip() or "unknown controller"
        handle.close()
        raise ControlError(f"MasterPi actuators are busy: {holder}") from exc
    handle.seek(0)
    handle.truncate()
    handle.write(json.dumps({"owner": owner, "pid": os.getpid(), "acquired_at": time.time()}, separators=(",", ":")))
    handle.flush()
    os.fsync(handle.fileno())
    return handle



def load_pose_state(path: Path = POSE_STATE_PATH) -> dict[int, int]:
    """Return the last successfully commanded servo pulses.

    PWM servos provide no position feedback, so this is commanded state only.
    Every control path uses this same file to prevent independent tools from
    silently disagreeing about the camera/arm pose.
    """
    try:
        raw = json.loads(path.read_text())
    except (OSError, ValueError, TypeError):
        return {}
    pose = raw.get("pose") if isinstance(raw, dict) else None
    if not isinstance(pose, dict):
        return {}
    out: dict[int, int] = {}
    for key, value in pose.items():
        try:
            servo = int(key)
            pulse = int(value)
        except (TypeError, ValueError):
            continue
        if servo in SERVO_IDS and SERVO_PULSE_MIN <= pulse <= SERVO_PULSE_MAX:
            out[servo] = pulse
    return out


def record_servo_command(
    servo: int,
    pulse: int,
    *,
    path: Path = POSE_STATE_PATH,
    writer: str = "masterpi_control",
) -> None:
    pose = load_pose_state(path)
    pose[int(servo)] = int(pulse)
    payload = {
        "pose": {str(k): v for k, v in sorted(pose.items())},
        "updated_at": time.time(),
        "last_writer": writer,
        "last_servo": int(servo),
        "last_pulse": int(pulse),
    }
    tmp = path.with_name(path.name + f".{os.getpid()}.tmp")
    try:
        tmp.write_text(json.dumps(payload, separators=(",", ":")))
        tmp.replace(path)
    except OSError:
        try:
            tmp.unlink(missing_ok=True)
        except OSError:
            pass


_active_motor_transport: "MotorTransport | None" = None


class ControlError(RuntimeError):
    """Raised when a motor command cannot be delivered."""


def _strict_int(text: str) -> int:
    if not re.fullmatch(r"-?(?:0|[1-9][0-9]*)", text):
        raise argparse.ArgumentTypeError(f"invalid integer: {text!r}")
    return int(text, 10)


def _bounded_int(low: int, high: int):
    def parse(text: str) -> int:
        value = _strict_int(text)
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(
                f"must be between {low} and {high}: {text!r}"
            )
        return value

    return parse


def _bounded_float(low: float, high: float):
    def parse(text: str) -> float:
        try:
            value = float(text)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"invalid number: {text!r}") from exc
        if not math.isfinite(value) or not low <= value <= high:
            raise argparse.ArgumentTypeError(
                f"must be between {low} and {high}: {text!r}"
            )
        return value

    return parse


def motor_register(motor: int) -> int:
    if motor not in range(1, 5):
        raise ValueError("motor must be 1..4")
    return MOTOR_REGISTER_BASE + motor - 1


def board_signed_speed(motor: int, speed: int) -> int:
    """Apply the legacy Board.py direction mapping before I2C encoding."""
    if motor not in range(1, 5):
        raise ValueError("motor must be 1..4")
    if not -100 <= speed <= 100:
        raise ValueError("speed must be -100..100")
    return -speed if motor in (1, 3) else speed


def i2c_argv(motor: int, speed: int) -> list[str]:
    """Build an argument list; never invoke a shell."""
    signed_speed = board_signed_speed(motor, speed)
    register = motor_register(motor)
    encoded_speed = signed_speed & 0xFF
    return [
        I2C_TRANSFER,
        "-y",
        "-a",
        str(I2C_BUS),
        f"w2@0x{I2C_ADDRESS:02x}",
        f"0x{register:02x}",
        f"0x{encoded_speed:02x}",
    ]


def _servo_duration_ms(duration: float) -> int:
    if not math.isfinite(duration) or not SERVO_DURATION_MIN <= duration <= SERVO_DURATION_MAX:
        raise ValueError(
            f"servo duration must be between {SERVO_DURATION_MIN} and {SERVO_DURATION_MAX} seconds"
        )
    return int(round(duration * 1000))


def servo_i2c_argv(servo: int, pulse: int, duration: float) -> list[str]:
    """Build the legacy MasterPi PWM servo frame without invoking a shell."""
    if servo not in SERVO_IDS:
        raise ValueError("servo must be one of 1, 3, 4, 5, 6")
    if not SERVO_PULSE_MIN <= pulse <= SERVO_PULSE_MAX:
        raise ValueError(f"servo pulse must be between {SERVO_PULSE_MIN} and {SERVO_PULSE_MAX}")
    duration_ms = _servo_duration_ms(duration)
    return [
        I2C_TRANSFER,
        "-y",
        "-a",
        str(I2C_BUS),
        f"w7@0x{I2C_ADDRESS:02x}",
        f"0x{SERVO_REGISTER:02x}",
        "0x01",
        f"0x{duration_ms & 0xFF:02x}",
        f"0x{(duration_ms >> 8) & 0xFF:02x}",
        f"0x{servo:02x}",
        f"0x{pulse & 0xFF:02x}",
        f"0x{(pulse >> 8) & 0xFF:02x}",
    ]


def probe_i2c_argv() -> tuple[list[str], list[str]]:
    """Build the board-compatible register-select and read transfers."""
    select = [
        I2C_TRANSFER,
        "-y",
        "-a",
        str(I2C_BUS),
        f"w1@0x{I2C_ADDRESS:02x}",
        f"0x{BATTERY_REGISTER:02x}",
    ]
    read = [
        I2C_TRANSFER,
        "-y",
        "-a",
        str(I2C_BUS),
        f"r2@0x{I2C_ADDRESS:02x}",
    ]
    return select, read


def parse_probe_output(stdout: str) -> int:
    """Parse two-byte little-endian hex output from i2ctransfer into battery mV."""
    tokens = stdout.strip().split()
    if len(tokens) != 2:
        raise ControlError(f"expected 2 bytes from probe, got {len(tokens)}: {stdout!r}")
    try:
        raw_bytes = [int(token, 16) for token in tokens]
    except ValueError as exc:
        raise ControlError(f"invalid hex byte in probe output: {stdout!r}") from exc
    for b in raw_bytes:
        if not 0 <= b <= 255:
            raise ControlError(f"byte out of range 0..255 in probe output: {stdout!r}")
    battery_mv = raw_bytes[0] | (raw_bytes[1] << 8)
    if not (BATTERY_MIN_MV <= battery_mv <= BATTERY_MAX_MV):
        raise ControlError(
            f"battery voltage {battery_mv} mV outside valid range ({BATTERY_MIN_MV}..{BATTERY_MAX_MV} mV)"
        )
    return battery_mv


def probe_hardware(dry_run: bool = False, timeout: float = PROBE_TIMEOUT) -> dict[str, Any]:
    """Execute a single read-only transaction and return controller probe info."""
    if dry_run:
        raise ControlError("probe requires live hardware; --dry-run is not supported")
    select_args, read_args = probe_i2c_argv()
    try:
        # This controller expects the same two-transfer sequence as the legacy
        # SDK. A combined repeated-start returns unrelated bytes on real HW.
        subprocess.run(
            select_args,
            check=True,
            shell=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        proc = subprocess.run(
            read_args,
            check=True,
            shell=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        raise ControlError(f"hardware probe timed out after {timeout}s") from exc
    except subprocess.CalledProcessError as exc:
        err = exc.stderr.strip() if exc.stderr else str(exc)
        raise ControlError(f"hardware probe failed: {err}") from exc
    except OSError as exc:
        raise ControlError(f"hardware probe execution error: {exc}") from exc

    battery_mv = parse_probe_output(proc.stdout)
    return {
        "ok": True,
        "controller": CONTROLLER_ID,
        "battery_mv": battery_mv,
    }


def _run_i2c_write(args: list[str], *, operation: str) -> None:
    try:
        subprocess.run(
            args,
            check=True,
            shell=False,
            timeout=I2C_WRITE_TIMEOUT,
        )
    except subprocess.TimeoutExpired as exc:
        raise ControlError(
            f"{operation} timed out after {I2C_WRITE_TIMEOUT:.2f}s"
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise ControlError(f"{operation} failed with exit code {exc.returncode}") from exc
    except OSError as exc:
        raise ControlError(f"{operation} execution error: {exc}") from exc


class MotorTransport:
    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run

    def write_motor(self, motor: int, speed: int) -> None:
        if speed != 0 and abs(int(speed)) < MIN_EFFECTIVE_WHEEL_SPEED:
            raise ValueError(
                f"wheel speed magnitude must be 0 (stop) or >= {MIN_EFFECTIVE_WHEEL_SPEED}; "
                "values below 35 are forbidden on this chassis"
            )
        if abs(int(speed)) > MAX_MOTOR_SPEED:
            raise ValueError(
                f"wheel speed magnitude must be <= {MAX_MOTOR_SPEED}; "
                "ordinary chassis motion stays capped at 40 while lateral mecanum motion may use 70"
            )
        args = i2c_argv(motor, speed)
        if self.dry_run:
            print("DRY-RUN " + shlex.join(args))
            return
        _run_i2c_write(args, operation=f"motor {motor} speed {speed} write")


class ServoTransport:
    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run

    def write_servo(self, servo: int, pulse: int, duration: float) -> None:
        args = servo_i2c_argv(servo, pulse, duration)
        if self.dry_run:
            print("DRY-RUN " + shlex.join(args))
            return
        _run_i2c_write(args, operation=f"servo {servo} pulse {pulse} write")
        record_servo_command(
            servo,
            pulse,
            writer=f"masterpi_control:{os.getpid()}",
        )


def stop_all(transport: MotorTransport, context: str = "stop") -> None:
    last_error: BaseException | None = None
    for attempt in range(1, STOP_RETRIES + 1):
        try:
            for motor in range(1, 5):
                transport.write_motor(motor, 0)
            if context:
                print(f"STOP {context} attempt={attempt}")
            return
        except BaseException as exc:
            last_error = exc
            if attempt < STOP_RETRIES:
                time.sleep(STOP_RETRY_DELAY)
    raise ControlError(f"failed to stop all motors after {STOP_RETRIES} attempts") from last_error


MIN_EFFECTIVE_WHEEL_SPEED = 35
MAX_CHASSIS_SPEED = 40
MAX_LATERAL_SPEED = 70
MAX_MOTOR_SPEED = MAX_LATERAL_SPEED
PREFERRED_CHASSIS_SPEED = 35
PREFERRED_LATERAL_SPEED = 65
# Longest single open-loop chassis segment any caller may request. Longer
# motions must be composed from bounded segments with observation in between.
MAX_CHASSIS_SEGMENT_S = 2.0


DRIVE_MAP: dict[str, tuple[int, int, int, int]] = {
    # Physical MasterPi patterns recovered from the known-good pickup code.
    # These are values at the MotorTransport API; board_signed_speed() then
    # applies the legacy motor 1/3 I2C direction encoding.
    "forward": (-1, 1, 1, -1),
    "backward": (1, -1, -1, 1),
    "left": (-1, -1, -1, -1),
    "right": (1, 1, 1, 1),
    "rotate-left": (-1, 1, -1, 1),
    "rotate-right": (1, -1, 1, -1),
}


def drive_speeds(direction: str, speed: int) -> list[tuple[int, int]]:
    if direction not in DRIVE_MAP:
        raise ValueError(f"unknown direction: {direction}")
    max_speed = MAX_LATERAL_SPEED if direction in {"left", "right"} else MAX_CHASSIS_SPEED
    if not MIN_EFFECTIVE_WHEEL_SPEED <= speed <= max_speed:
        raise ValueError(
            f"drive speed for {direction} must be {MIN_EFFECTIVE_WHEEL_SPEED}..{max_speed}; "
            "values below 35 are forbidden on this chassis"
        )
    return [(motor, DRIVE_MAP[direction][motor - 1] * speed) for motor in range(1, 5)]


def _add_dry_run(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--dry-run",
        action="store_true",
        default=argparse.SUPPRESS,
        help="print i2ctransfer commands without executing them",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="masterpi_control.py")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print i2ctransfer commands without executing them",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    stop_parser = commands.add_parser("stop", help="stop all four motors")
    _add_dry_run(stop_parser)

    motor_parser = commands.add_parser("motor", help="run one motor briefly")
    motor_parser.add_argument("motor", type=_bounded_int(1, 4))
    motor_parser.add_argument("speed", type=_bounded_int(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED))
    motor_parser.add_argument(
        "--duration", type=_bounded_float(0.05, 2.0), default=0.5
    )
    _add_dry_run(motor_parser)

    servo_parser = commands.add_parser("servo", help="move one PWM servo")
    servo_parser.add_argument("servo", type=_bounded_int(1, 6), choices=sorted(SERVO_IDS))
    servo_parser.add_argument("pulse", type=_bounded_int(SERVO_PULSE_MIN, SERVO_PULSE_MAX))
    servo_parser.add_argument(
        "--duration",
        type=_bounded_float(SERVO_DURATION_MIN, SERVO_DURATION_MAX),
        required=True,
    )
    _add_dry_run(servo_parser)

    probe_parser = commands.add_parser(
        "probe", help="read-only check of control board and battery voltage"
    )
    _add_dry_run(probe_parser)

    drive_parser = commands.add_parser("drive", help="run a bounded chassis motion")
    drive_parser.add_argument("direction", choices=tuple(DRIVE_MAP))
    drive_parser.add_argument("--speed", type=_bounded_int(MIN_EFFECTIVE_WHEEL_SPEED, MAX_LATERAL_SPEED), required=True)
    drive_parser.add_argument(
        "--duration", type=_bounded_float(0.05, MAX_CHASSIS_SEGMENT_S), default=0.5
    )
    _add_dry_run(drive_parser)
    return parser


def _signal_handler(signum: int, _frame: object) -> None:
    if _active_motor_transport is not None:
        try:
            stop_all(_active_motor_transport, context=f"signal-{signum}")
        finally:
            raise SystemExit(128 + signum)
    raise SystemExit(128 + signum)


def _atexit_stop() -> None:
    if _active_motor_transport is not None:
        try:
            stop_all(_active_motor_transport, context="atexit")
        except BaseException as exc:
            print(f"STOP atexit failed: {exc}", file=sys.stderr)


def _run_motion(
    transport: MotorTransport, speeds: Iterable[tuple[int, int]], duration: float
) -> None:
    duration = float(duration)
    if not (0.0 < duration <= MAX_CHASSIS_SEGMENT_S):
        raise ValueError(
            f"chassis segment duration must be in (0, {MAX_CHASSIS_SEGMENT_S}] s, got {duration}"
        )
    stop_all(transport, context="pre-motion")
    try:
        for motor, speed in speeds:
            transport.write_motor(motor, speed)
        time.sleep(duration)
    finally:
        stop_all(transport, context="motion-finally")


def main(argv: Sequence[str] | None = None) -> int:
    global _active_motor_transport
    args = build_parser().parse_args(argv)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    atexit.register(_atexit_stop)

    dry_run = bool(getattr(args, "dry_run", False))
    is_motor_command = args.command in {"motor", "drive"}
    lease = None
    if not dry_run and args.command in {"motor", "drive", "servo"}:
        lease = acquire_actuator_lease(owner=f"masterpi_control:{args.command}")
    transport = MotorTransport(dry_run=dry_run) if is_motor_command or args.command == "stop" else None
    if is_motor_command:
        _active_motor_transport = transport
    try:
        if args.command == "stop":
            assert transport is not None
            stop_all(transport)
        elif args.command == "motor":
            assert transport is not None
            _run_motion(transport, [(args.motor, args.speed)], args.duration)
        elif args.command == "drive":
            assert transport is not None
            _run_motion(transport, drive_speeds(args.direction, args.speed), args.duration)
        elif args.command == "servo":
            servo_transport = ServoTransport(dry_run=dry_run)
            servo_transport.write_servo(args.servo, args.pulse, args.duration)
            print(
                f"SERVO OK servo={args.servo} pulse={args.pulse} "
                f"duration_ms={_servo_duration_ms(args.duration)}"
            )
        elif args.command == "probe":
            result = probe_hardware(dry_run=dry_run)
            print(json.dumps(result, separators=(",", ":")))
        else:  # pragma: no cover - argparse enforces the command choices
            raise ControlError(f"unknown command: {args.command}")
        return 0
    finally:
        if is_motor_command:
            assert transport is not None
            try:
                stop_all(transport, context="main-finally")
            finally:
                _active_motor_transport = None
        if lease is not None:
            lease.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (argparse.ArgumentError, ControlError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
