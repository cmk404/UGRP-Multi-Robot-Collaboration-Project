#!/usr/bin/env python3
"""Run UGRP entrypoints only for the lifetime of an owned work session."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


TERM_GRACE_SECONDS = 5.0


def session_dir() -> Path:
    configured = os.environ.get("UGRP_SESSION_DIR")
    return Path(configured) if configured else Path(tempfile.gettempdir()) / f"ugrp-{os.getuid()}" / "sessions"


def session_file(name: str) -> Path:
    return session_dir() / f"{name}.json"


def session_lock_file(name: str) -> Path:
    return session_dir() / f"{name}.lock"


def acquire_session_lock(name: str):
    directory = session_dir()
    directory.mkdir(parents=True, exist_ok=True)
    lock = session_lock_file(name).open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        return None
    return lock


def validate_name(name: str) -> str:
    if not name or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in name):
        raise ValueError("session name must contain only letters, numbers, '-' or '_'")
    return name


def process_group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def process_start_identity(pid: int) -> str | None:
    result = subprocess.run(
        ["/bin/ps", "-o", "lstart=", "-p", str(pid)],
        capture_output=True,
        text=True,
        check=False,
    )
    value = result.stdout.strip()
    return value or None


def verified_group(record: dict) -> int | None:
    try:
        leader = int(record["leader_pid"])
        pgid = int(record["pgid"])
        expected_start = str(record["leader_start"])
        return pgid if leader == pgid and os.getpgid(leader) == pgid and process_start_identity(leader) == expected_start else None
    except (KeyError, TypeError, ValueError, ProcessLookupError, PermissionError):
        return None


def load_session(name: str) -> dict | None:
    path = session_file(name)
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return None
    except (json.JSONDecodeError, OSError):
        return {"invalid": True}


def remove_session(name: str) -> None:
    try:
        session_file(name).unlink()
    except FileNotFoundError:
        pass


def remove_session_if_unlocked(name: str) -> None:
    lock = acquire_session_lock(name)
    if lock is None:
        return
    try:
        remove_session(name)
    finally:
        lock.close()


def write_session(name: str, record: dict) -> None:
    directory = session_dir()
    directory.mkdir(parents=True, exist_ok=True)
    path = session_file(name)
    temp = path.with_suffix(f".tmp-{os.getpid()}")
    temp.write_text(json.dumps(record, indent=2) + "\n")
    os.replace(temp, path)


def stop_group(pgid: int, grace: float = TERM_GRACE_SECONDS) -> None:
    if not process_group_alive(pgid):
        return
    os.killpg(pgid, signal.SIGTERM)
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        if not process_group_alive(pgid):
            return
        time.sleep(0.05)
    if process_group_alive(pgid):
        os.killpg(pgid, signal.SIGKILL)


def run_session(name: str, command: list[str], grace: float) -> int:
    if not command:
        raise ValueError("a command is required after --")
    lock = acquire_session_lock(name)
    if lock is None:
        print(f"UGRP session '{name}' is already starting or running", file=sys.stderr)
        return 2
    try:
        existing = load_session(name)
        if existing and not existing.get("invalid") and verified_group(existing) is not None:
            print(f"UGRP session '{name}' is already running (pgid {existing['pgid']})", file=sys.stderr)
            return 2
        remove_session(name)

        child = subprocess.Popen(command, start_new_session=True)
        # start_new_session makes the child a process-group leader. Using its PID
        # avoids racing os.getpgid() when a short command exits immediately.
        pgid = child.pid
        try:
            leader_start = process_start_identity(child.pid)
            if leader_start is None:
                try:
                    status = child.wait()
                    return 128 + (-status) if status < 0 else status
                finally:
                    # A short leader may have spawned a longer-lived descendant
                    # before exiting. The group remains this session's responsibility.
                    stop_group(pgid, grace)
            write_session(name, {
                "name": name,
                "owner_pid": os.getpid(),
                "leader_pid": child.pid,
                "leader_start": leader_start,
                "pgid": pgid,
                "command": command,
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            })
        except BaseException:
            try:
                stop_group(pgid, grace)
            finally:
                child.wait()
                remove_session(name)
            raise
        print(f"UGRP session '{name}' started (pgid {pgid})", file=sys.stderr)

        requested_signal: int | None = None
        signal_deadline: float | None = None

        def forward(signum: int, _frame: object) -> None:
            nonlocal requested_signal, signal_deadline
            if requested_signal is not None:
                return
            requested_signal = signum
            signal_deadline = time.monotonic() + grace
            try:
                os.killpg(pgid, signum)
            except ProcessLookupError:
                pass

        previous = {sig: signal.signal(sig, forward) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        try:
            while True:
                try:
                    status = child.wait(timeout=0.1)
                    break
                except subprocess.TimeoutExpired:
                    # Let a child finish its own result/receipt cleanup after
                    # SIGINT or SIGTERM before escalating to group termination.
                    if signal_deadline is not None and time.monotonic() >= signal_deadline:
                        stop_group(pgid, grace)
                        status = child.wait()
                        break
            return 128 + (-status) if status < 0 else status
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
            stop_group(pgid, grace)
            remove_session(name)
            print(f"UGRP session '{name}' stopped", file=sys.stderr)
    finally:
        lock.close()


def stop_session(name: str, grace: float) -> int:
    record = load_session(name)
    if record is None:
        print(f"UGRP session '{name}' is not running")
        return 0
    if record.get("invalid"):
        print(f"UGRP session '{name}' has an invalid record; refusing an unverified kill", file=sys.stderr)
        return 2
    pgid = verified_group(record)
    if pgid is None:
        print(f"UGRP session '{name}' no longer matches its recorded leader; refusing an unverified kill", file=sys.stderr)
        remove_session_if_unlocked(name)
        return 2
    stop_group(pgid, grace)
    remove_session_if_unlocked(name)
    print(f"UGRP session '{name}' stopped")
    return 0


def status_session(name: str) -> int:
    record = load_session(name)
    if not record or record.get("invalid"):
        print(f"{name}: stopped")
        return 1
    pgid = verified_group(record)
    alive = pgid is not None and process_group_alive(pgid)
    print(f"{name}: {'running' if alive else 'stopped'} (pgid {record['pgid']})")
    if not alive:
        remove_session_if_unlocked(name)
    return 0 if alive else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    run_parser = subparsers.add_parser("run", help="run a command as a foreground owned session")
    run_parser.add_argument("name", type=validate_name)
    run_parser.add_argument("command", nargs=argparse.REMAINDER)
    stop_parser = subparsers.add_parser("stop", help="stop one owned session and all descendants")
    stop_parser.add_argument("name", type=validate_name)
    status_parser = subparsers.add_parser("status", help="show whether an owned session is running")
    status_parser.add_argument("name", type=validate_name)
    for item in (run_parser, stop_parser):
        item.add_argument("--grace", type=float, default=TERM_GRACE_SECONDS, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.action == "run":
        command = args.command[1:] if args.command[:1] == ["--"] else args.command
        return run_session(args.name, command, max(0.0, args.grace))
    if args.action == "stop":
        return stop_session(args.name, max(0.0, args.grace))
    return status_session(args.name)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
