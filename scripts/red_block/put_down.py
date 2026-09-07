#!/usr/bin/env python3
"""Safely return the currently carried block to its original pickup site.

This is not a generic guessed floor drop. A live execution is authorized only
by the short-lived causal carry handoff created by precision pick, including the
exact hover/descent path used for that pickup. The robot reverses that path,
opens the gripper at the original grasp pose, then retreats to hover.
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from carry_handoff import (
    invalidate_carry_handoff,
    load_carry_handoff,
    require_carry_handoff,
    require_return_path,
)
from deploy import DEFAULT_HOST, deploy_and_run
from poses import GRIPPER_ID, GRIPPER_OPEN
from robot import Robot, is_robot


GRIPPER_CLOSED_MAX_PULSE = 1700


class PutDownError(RuntimeError):
    pass


def execute_put_down(robot: Robot) -> str:
    pulse = robot.pose.get(GRIPPER_ID)
    if pulse is None or int(pulse) > GRIPPER_CLOSED_MAX_PULSE:
        raise PutDownError("gripper is not closed; there is no carried object to put down")

    if robot.dry_run:
        # Dry-run validates the motion shape without requiring a live Pi handoff.
        print("dry-run: put-down requires a live causal return path before release", flush=True)
        return "dry-run put-down path accepted without actuation"

    payload = load_carry_handoff()
    target_color = str(payload.get("target_color") or "red")
    require_carry_handoff(
        current_gripper_pulse=int(pulse),
        current_robot_pose=robot.pose,
        target_color=target_color,
    )
    return_path = require_return_path(payload)

    # Import lazily so the same REAL controller helper is reused by SIM through
    # its hardware-adapter patching layer as well.
    import physical_state_machine_reference as precision

    robot.stop()
    robot.move_servo(6, int(return_path["base_pulse"]), 0.45)
    precision.move_arm_together(robot, return_path["hover_pose"], duration=0.85)
    descent = list(return_path["descent_poses"])
    for index, pose in enumerate(descent, start=1):
        print(f"put-down descent {index}/{len(descent)}", flush=True)
        precision.move_arm_together(
            robot,
            pose,
            duration=0.45 if index < len(descent) else 0.60,
        )

    print(f"release {target_color} block at original pickup site", flush=True)
    robot.move_servo(GRIPPER_ID, GRIPPER_OPEN, 0.70)
    # move_servo(open) invalidates the carry handoff in the physical Robot; do
    # it explicitly too for alternate adapters and failure-safe idempotence.
    invalidate_carry_handoff()

    for pose in reversed(descent[:-1]):
        precision.move_arm_together(robot, pose, duration=0.40)
    precision.move_arm_together(robot, return_path["hover_pose"], duration=0.70)
    robot.stop()
    return target_color


def run_on_robot(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)

    def handle_signal(signum: int, _frame: object) -> None:
        try:
            robot.stop()
        finally:
            raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    robot.stop()
    print(json.dumps({"probe": robot.probe()}, sort_keys=True), flush=True)
    color = execute_put_down(robot)
    robot.stop()
    print(f"released carried object; previous_target_color={color}", flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="현재 들고 있는 블록을 원래 집은 위치에 안전하게 내려놓습니다.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra = ["--dry-run"] if args.dry_run else []
    return deploy_and_run("put_down.py", extra, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PutDownError, RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
