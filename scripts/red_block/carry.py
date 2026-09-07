#!/usr/bin/env python3
"""Carry-pose hold helper.

The historical standalone `carry.py` also performed its own single-frame,
fixed-pose pickup. That was an unsafe second grasp implementation and is now
disabled. The supported physical path is `approach` -> `pick`; pick already
finishes in the carry pose. This module keeps only the hold helper used by the
legacy-compatible fetch CLI.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from deploy import DEFAULT_HOST


def hold_until_interrupt(robot) -> None:
    print("holding carry pose. Ctrl+C to stop.", flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("release hold", flush=True)
        robot.stop()


def run_on_robot(_args: argparse.Namespace) -> int:
    raise RuntimeError(
        "standalone carry pickup is disabled; run approach then pick (pick already ends in carry pose)"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="독립 carry 집기는 비활성화되었습니다. approach -> pick을 사용하세요."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--grasp-pulse", type=int, default=None)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    _args = build_parser().parse_args(argv)
    raise RuntimeError(
        "standalone carry pickup is disabled; run approach then pick (pick already ends in carry pose)"
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
