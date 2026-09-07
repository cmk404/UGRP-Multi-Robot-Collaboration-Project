"""Servo pulse constants and pose math for the red-block pickup."""

from __future__ import annotations

from typing import Mapping


GRIPPER_ID = 1
GRIPPER_CLOSED = 1500
GRIPPER_OPEN = 2000

# Observe/carry: camera up, used after a grasp. Do not use this to find a
# floor block — it looks at table height, not the ground.
# Last live target: ID3=960, ID4=2410, ID5=1215, ID6=1500.
POSE_OBSERVE = {3: 960, 4: 2410, 5: 1215}
# Search/approach: look at the floor in front of the robot.
POSE_SEARCH = {3: 740, 4: 2320, 5: 1320}
POSE_HOVER = {3: 650, 4: 2230, 5: 1500}
POSE_GRASP = {3: 650, 4: 2230, 5: 1920}
POSE_CARRY = dict(POSE_OBSERVE)

BASE_CENTER = 1500
BASE_MIN = 1100
BASE_MAX = 1900
BASE_SCALE = 600.0
SERVO_PULSE_MIN = 500
SERVO_PULSE_MAX = 2500


def clamp_pulse(pulse: int, low: int = SERVO_PULSE_MIN, high: int = SERVO_PULSE_MAX) -> int:
    return max(low, min(high, pulse))


def base_pulse_for_image_x(nx: float, *, flip_x: bool = False) -> int:
    """Map blob x in 0..1 to servo 6. Image-right lowers the pulse (yaw toward +X)."""
    if not 0.0 <= nx <= 1.0:
        raise ValueError(f"nx must be in 0..1, got {nx!r}")
    x = 1.0 - nx if flip_x else nx
    pulse = int(round(BASE_CENTER - (x - 0.5) * BASE_SCALE))
    return clamp_pulse(pulse, BASE_MIN, BASE_MAX)


def move_duration(from_pulse: int | None, to_pulse: int) -> float:
    if from_pulse is None:
        return 1.5
    return min(2.5, max(0.4, abs(to_pulse - from_pulse) / 800.0))


def pose_with_base(joints: Mapping[int, int], base: int, gripper: int) -> dict[int, int]:
    pose = dict(joints)
    pose[GRIPPER_ID] = gripper
    pose[6] = clamp_pulse(base, BASE_MIN, BASE_MAX)
    return pose


def lower_order(servo: int) -> int:
    return {1: 0, 6: 1, 3: 2, 4: 3, 5: 4}[servo]


def raise_order(servo: int) -> int:
    return {5: 0, 4: 1, 3: 2, 6: 3, 1: 4}[servo]


def servo_steps(
    current: Mapping[int, int] | None,
    target: Mapping[int, int],
    *,
    lowering: bool,
) -> list[tuple[int, int, float]]:
    order = lower_order if lowering else raise_order
    steps: list[tuple[int, int, float]] = []
    for servo in sorted(target, key=order):
        pulse = clamp_pulse(int(target[servo]))
        previous = None if current is None else current.get(servo)
        if previous == pulse:
            continue
        steps.append((servo, pulse, move_duration(previous, pulse)))
    return steps
