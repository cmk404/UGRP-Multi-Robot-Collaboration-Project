"""Pure chassis planner: drive toward a red blob until it is in front."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


TURN_DEADBAND = 0.12
PICK_CENTER_DEADBAND = 0.065
FAR_NY = 0.68
TOO_CLOSE_NY = 0.90
ARRIVE_AREA = 3500
TOO_CLOSE_AREA = 9000
TURN_SPEED = 35
FORWARD_SPEED = 35
BACK_SPEED = 35
STEP_DURATION = 0.35
TURN_DURATION = 0.45
PAN_CENTER = 1500
PAN_TURN_DEADBAND = 80
ALIGN_TURN_SPEED = 35
ALIGN_TURN_MIN_DURATION = 0.12
ALIGN_TURN_MAX_DURATION = 0.28
SEARCH_PANS = (1500, 1800, 1200, 1950, 1050, 1500)


@dataclass(frozen=True)
class ChassisCommand:
    kind: str
    speed: int = 0
    duration: float = 0.0


def _nx(blob: Any, flip_x: bool) -> float:
    return 1.0 - blob.nx if flip_x else blob.nx


def in_grasp_zone(blob: Any, *, flip_x: bool = False) -> bool:
    nx = _nx(blob, flip_x)
    return (
        abs(nx - 0.5) <= TURN_DEADBAND
        and blob.ny >= FAR_NY
        and blob.area >= ARRIVE_AREA
        and blob.ny < TOO_CLOSE_NY
    )


def in_pick_ready_zone(
    blob: Any, *, flip_x: bool = False, pan: int | None = None
) -> bool:
    """Tighter terminal condition used before the destructive pick skill.

    Image-center alone is insufficient with an eye-in-hand camera: the neck can
    be yawed while the target is centered.  When pan is known, require the
    camera to be nearly aligned with the chassis as well.
    """
    nx = _nx(blob, flip_x)
    if not (in_grasp_zone(blob, flip_x=flip_x) and abs(nx - 0.5) <= PICK_CENTER_DEADBAND):
        return False
    return pan is None or abs(int(pan) - PAN_CENTER) <= PAN_TURN_DEADBAND


def search_look(step: int, *, tilt_home: int) -> tuple[int, int]:
    """Next pan/tilt while hunting for a block that is not in view."""
    pan = SEARCH_PANS[step % len(SEARCH_PANS)]
    nod = 80 if step % 3 == 2 else 0
    return pan, tilt_home - nod


def search_drive(step: int) -> ChassisCommand | None:
    """After one head sweep, turn the body and keep looking."""
    if step < len(SEARCH_PANS):
        return None
    if step % 2 == 1:
        return None
    direction = "rotate-left" if (step // 2) % 2 == 0 else "rotate-right"
    return ChassisCommand(direction, TURN_SPEED, 0.40)


def approach_command(
    blob: Any | None, *, flip_x: bool = False, pan: int | None = None
) -> ChassisCommand:
    """Choose one bounded chassis step, or arrived/lost. Never uses the arm.

    When `pan` is set, a yawed neck means the body should turn toward the
    look direction. That matters once the camera is tracking: the blob stays
    near image-center even if the block is off to the side.
    """
    if blob is None:
        return ChassisCommand("lost")
    nx = _nx(blob, flip_x)
    if blob.ny >= TOO_CLOSE_NY and blob.area >= TOO_CLOSE_AREA:
        return ChassisCommand("backward", BACK_SPEED, STEP_DURATION)
    if pan is not None and abs(pan - PAN_CENTER) > PAN_TURN_DEADBAND:
        direction = "rotate-left" if pan > PAN_CENTER else "rotate-right"
        yaw = abs(int(pan) - PAN_CENTER)
        duration = min(
            ALIGN_TURN_MAX_DURATION,
            max(ALIGN_TURN_MIN_DURATION, yaw / 1000.0),
        )
        return ChassisCommand(direction, ALIGN_TURN_SPEED, duration)
    if abs(nx - 0.5) > TURN_DEADBAND:
        direction = "rotate-right" if nx > 0.5 else "rotate-left"
        return ChassisCommand(direction, TURN_SPEED, TURN_DURATION)
    if blob.ny < FAR_NY or blob.area < ARRIVE_AREA:
        return ChassisCommand("forward", FORWARD_SPEED, STEP_DURATION)
    return ChassisCommand("arrived")


def same_view(previous: Any | None, current: Any | None) -> bool:
    """True when the blob did not move enough to prove the robot actually turned."""
    if previous is None or current is None:
        return False
    return (
        abs(previous.nx - current.nx) < 0.02
        and abs(previous.ny - current.ny) < 0.02
        and abs(previous.area - current.area) / max(previous.area, 1) < 0.08
    )
