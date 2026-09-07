#!/usr/bin/env python3
"""Small composable REAL chassis primitives for the agent.

Each call owns the actuator lease, probes the controller, executes exactly one
bounded motion, stops in a finally block, and returns. No primitive infers task
success from motor completion; the harness observes a fresh camera frame next.
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

from deploy import DEFAULT_HOST, deploy_and_run
from robot import Robot, is_robot

MOTIONS = ("forward", "backward", "left", "right", "rotate-left", "rotate-right", "stop")
MIN_SPEED = 35
MAX_SPEED = 40
MAX_LATERAL_SPEED = 70
LATERAL_DEFAULT_SPEED = 65
LATERAL_MIN_DURATION = 0.65
MIN_DURATION = 0.10
MAX_DURATION = 0.80
DEFAULT_SPEED = 35
DEFAULT_DURATION = 0.30


def validate_motion(motion: str, speed: int, duration: float) -> None:
    if motion not in MOTIONS:
        raise ValueError(f"unknown motion: {motion}")
    if motion != "stop":
        max_speed = MAX_LATERAL_SPEED if motion in {"left", "right"} else MAX_SPEED
        if not MIN_SPEED <= int(speed) <= max_speed:
            raise ValueError(f"speed for {motion} must be between {MIN_SPEED} and {max_speed}")
    if not MIN_DURATION <= float(duration) <= MAX_DURATION:
        raise ValueError(f"duration must be between {MIN_DURATION:.2f} and {MAX_DURATION:.2f} seconds")


def run_primitive(robot: Robot, motion: str, speed: int, duration: float) -> int:
    validate_motion(motion, speed, duration)
    if motion == "stop":
        robot.stop()
        print("primitive stop complete", flush=True)
        return 0
    requested_speed, requested_duration = int(speed), float(duration)
    if motion in {"left", "right"}:
        speed = max(requested_speed, LATERAL_DEFAULT_SPEED)
        duration = max(requested_duration, LATERAL_MIN_DURATION)
    robot.drive(motion, speed, duration)
    print(
        f"primitive complete motion={motion} speed={speed} duration={duration:.2f}",
        flush=True,
    )
    return 0


def run_on_robot(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)

    def handle_signal(signum: int, _frame: object) -> None:
        try:
            robot.stop()
        finally:
            raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        robot.stop()
        print(json.dumps({"probe": robot.probe()}, sort_keys=True), flush=True)
        return run_primitive(robot, args.motion, args.speed, args.duration)
    finally:
        robot.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="AI가 조합하는 짧고 제한된 MasterPi 기본 이동입니다.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--motion", choices=MOTIONS, required=True)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    validate_motion(args.motion, args.speed, args.duration)
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra = ["--motion", args.motion, "--speed", str(args.speed), "--duration", str(args.duration)]
    if args.dry_run:
        extra.append("--dry-run")
    return deploy_and_run("primitive.py", extra, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
