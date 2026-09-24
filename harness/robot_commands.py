"""Shared MasterPi command execution and input bounds, independent of any UI."""

from __future__ import annotations

import math
import subprocess
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


MOTION_DIRECTIONS = frozenset(
    {"forward", "backward", "left", "right", "rotate-left", "rotate-right"}
)
SERVO_IDS = frozenset({1, 3, 4, 5, 6})
MIN_EFFECTIVE_WHEEL_SPEED = 31
MAX_CHASSIS_SPEED = 40


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""


class CommandRunner:
    """Execute argv directly without a shell."""

    def run(self, argv: Sequence[str], *, timeout: float) -> CommandResult:
        completed = subprocess.run(
            list(argv),
            shell=False,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return CommandResult(completed.returncode, completed.stdout, completed.stderr)

    def start(self, argv: Sequence[str]) -> subprocess.Popen[bytes]:
        return subprocess.Popen(
            list(argv),
            shell=False,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )


def validate_drive_payload(payload: Mapping[str, Any]) -> tuple[str, int, float]:
    if not isinstance(payload, Mapping):
        raise ValueError("JSON body must be an object")

    direction = payload.get("direction")
    if not isinstance(direction, str) or direction not in MOTION_DIRECTIONS:
        raise ValueError("direction must be one of the supported directions")

    speed = payload.get("speed")
    if isinstance(speed, bool) or not isinstance(speed, int) or not MIN_EFFECTIVE_WHEEL_SPEED <= speed <= MAX_CHASSIS_SPEED:
        raise ValueError(f"speed must be an integer from {MIN_EFFECTIVE_WHEEL_SPEED} to {MAX_CHASSIS_SPEED}; values <=30 do not move this chassis")

    duration = payload.get("duration")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("duration must be a number from 0.05 to 2.0")
    duration_float = float(duration)
    if not math.isfinite(duration_float) or not 0.05 <= duration_float <= 2.0:
        raise ValueError("duration must be a number from 0.05 to 2.0")
    return direction, speed, duration_float


def validate_servo_payload(payload: Mapping[str, Any]) -> tuple[int, int, float]:
    if not isinstance(payload, Mapping):
        raise ValueError("JSON body must be an object")

    servo = payload.get("servo")
    if isinstance(servo, bool) or not isinstance(servo, int) or servo not in SERVO_IDS:
        raise ValueError("servo must be one of 1, 3, 4, 5, or 6")

    pulse = payload.get("pulse")
    if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
        raise ValueError("pulse must be an integer from 500 to 2500")

    duration = payload.get("duration")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("duration must be a number from 0.1 to 3.0")
    duration_float = float(duration)
    if not math.isfinite(duration_float) or not 0.1 <= duration_float <= 3.0:
        raise ValueError("duration must be a number from 0.1 to 3.0")
    return servo, pulse, duration_float
