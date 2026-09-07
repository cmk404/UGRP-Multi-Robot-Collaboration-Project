#!/usr/bin/env python3
"""Bound one remote skill and stop its whole process group on timeout."""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys


TERM_GRACE_SECONDS = 3.0
STOP_AFTER_KILL_TIMEOUT_S = 6.0
# Deployed next to this file inside the same versioned package.
CONTROL_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "masterpi_control.py")


def _force_stop_actuators(reason: str) -> None:
    """Best-effort wheel stop after the skill process was killed.

    SIGTERM lets the skill run its own `finally: stop_all`, but a SIGKILL (or a
    skill stuck inside a blocking I2C write) skips that path and can leave the
    mecanum wheels spinning open-loop. Always issue an independent stop.
    """
    if not os.path.exists(CONTROL_SCRIPT):
        print(f"warning: watchdog cannot force-stop ({reason}): {CONTROL_SCRIPT} missing", file=sys.stderr, flush=True)
        return
    try:
        proc = subprocess.run(
            [sys.executable, CONTROL_SCRIPT, "stop"],
            capture_output=True, text=True, timeout=STOP_AFTER_KILL_TIMEOUT_S,
        )
        status = "ok" if proc.returncode == 0 else f"exit={proc.returncode} {proc.stderr.strip()[-200:]}"
    except (OSError, subprocess.TimeoutExpired) as exc:
        status = f"failed: {exc}"
    print(f"watchdog force-stop after {reason}: {status}", file=sys.stderr, flush=True)


def _terminate_child_group(child: subprocess.Popen, *, grace_s: float = TERM_GRACE_SECONDS, reason: str = "terminate") -> None:
    if child.poll() is not None:
        return
    killed = False
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        child.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        killed = True
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
    # A clean SIGTERM exit normally stopped the motors itself, but the skill may
    # have died between a motor write and its finally block; stop regardless.
    _force_stop_actuators(f"{reason} ({'SIGKILL' if killed else 'SIGTERM'})")


def run_guarded(command: list[str], timeout_s: float) -> int:
    if not command:
        raise ValueError("watchdog command is empty")
    if timeout_s <= 0:
        raise ValueError("watchdog timeout must be positive")
    child = subprocess.Popen(command, start_new_session=True)
    previous_handlers = {}

    def cancel_from_transport(signum: int, _frame) -> None:
        # When the Oracle-side action process group is cancelled, its SSH client
        # closes. sshd propagates that transport teardown as HUP/TERM; convert it
        # into a guarded child-group termination instead of orphaning the skill.
        _terminate_child_group(child, reason=f"transport signal {signum}")
        raise SystemExit(128 + signum)

    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, cancel_from_transport)
    try:
        try:
            return int(child.wait(timeout=timeout_s))
        except subprocess.TimeoutExpired:
            print(
                f"error: remote skill safety timeout after {timeout_s:.1f}s",
                file=sys.stderr,
                flush=True,
            )
            _terminate_child_group(child, reason=f"safety timeout {timeout_s:.1f}s")
            return 124
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    return run_guarded(command, args.timeout)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"error: watchdog could not start skill: {exc}", file=sys.stderr)
        raise SystemExit(125) from exc
