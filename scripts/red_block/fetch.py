#!/usr/bin/env python3
"""Compatibility macro: guarded precision approach -> precision pick -> carry.

This file intentionally contains no independent grasp policy. The historical
single-frame/fixed-pose grasp path was removed because it bypassed the active
hand-eye/TCP handoff used by pick.py.
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

from approach import run_approach
from carry import hold_until_interrupt
from deploy import DEFAULT_HOST, deploy_and_run
from pick import run_precision_pick
from robot import Robot, is_robot


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
    print("1/2 precision approach", flush=True)
    run_approach(robot, flip_x=args.flip_x)
    print("2/2 precision pick and carry", flush=True)
    run_precision_pick(robot, flip_x=args.flip_x, flip_y=args.flip_y)
    if args.no_hold:
        robot.stop()
        print("fetch done", flush=True)
        return 0
    hold_until_interrupt(robot)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="호환용 매크로: 정밀 approach와 검증된 pick을 연속 수행합니다."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    # Kept only so old operator commands fail explicitly instead of silently
    # changing the precision grasp pose.
    parser.add_argument("--grasp-pulse", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--no-hold", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.grasp_pulse is not None:
        raise ValueError(
            "legacy --grasp-pulse is disabled; precision approach owns the grasp plan"
        )
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra: list[str] = []
    if args.dry_run:
        extra.append("--dry-run")
    if args.flip_x:
        extra.append("--flip-x")
    if args.flip_y:
        extra.append("--flip-y")
    if args.no_hold:
        extra.append("--no-hold")
    return deploy_and_run("fetch.py", extra, host=args.host, tty=not args.no_hold)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
