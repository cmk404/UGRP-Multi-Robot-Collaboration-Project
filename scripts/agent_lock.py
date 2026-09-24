#!/usr/bin/env python3
"""Exclusive host lock for timing-sensitive physics and training runs.

Several agents (Claude, Codex) share one Mac. Realtime native runs and wall-time
benchmarks are distorted by concurrent simulation or training, so such runs hold
this lock and other agents do not start new physics/training while it is held.
The lock is a directory created atomically under the primary checkout's
``outputs/agent-locks``; nothing here stops or signals another process.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import time

DEFAULT_ROOT = Path('/Users/changmin/projects/ugrp/outputs/agent-locks')


def _alive(pid):
    if not isinstance(pid, int) or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def status(root: Path, name: str = 'physics') -> dict | None:
    path = root / name / 'owner.json'
    if not path.is_file():
        return None
    owner = json.loads(path.read_text())
    owner['pid_alive'] = _alive(owner.get('pid'))
    return owner


def acquire(root: Path, *, owner: str, branch: str, purpose: str, pid: int,
            expected_minutes: float, name: str = 'physics') -> dict:
    root.mkdir(parents=True, exist_ok=True)
    lock = root / name
    try:
        lock.mkdir()
    except FileExistsError:
        held = status(root, name)
        raise RuntimeError(f'lock held: {json.dumps(held, ensure_ascii=False)}') from None
    value = {'owner': owner, 'branch': branch, 'purpose': purpose, 'pid': pid,
             'acquired_unix': time.time(), 'expected_end_unix': time.time() + 60 * expected_minutes,
             'loadavg_at_acquire': os.getloadavg()}
    (lock / 'owner.json').write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    return value


def release(root: Path, *, owner: str, name: str = 'physics', stale: bool = False) -> dict:
    held = status(root, name)
    if held is None:
        raise RuntimeError('lock is not held')
    if held['owner'] != owner and not (stale and not held['pid_alive']):
        raise RuntimeError('only the owner may release; --stale requires a dead recorded pid')
    record = root / 'released.jsonl'
    with record.open('a') as stream:
        stream.write(json.dumps({**held, 'released_by': owner, 'released_unix': time.time(),
                                 'stale_release': bool(stale)}, ensure_ascii=False) + '\n')
    shutil.rmtree(root / name)
    return held


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    get = sub.add_parser('acquire')
    get.add_argument('--owner', required=True, help='agent name, e.g. claude or codex')
    get.add_argument('--branch', required=True)
    get.add_argument('--purpose', required=True)
    get.add_argument('--pid', type=int, required=True, help='long-running driver PID that owns the runs')
    get.add_argument('--expected-minutes', type=float, required=True)
    put = sub.add_parser('release')
    put.add_argument('--owner', required=True)
    put.add_argument('--stale', action='store_true', help='release another owner only if its recorded pid is dead')
    args = parser.parse_args(argv)
    try:
        if args.command == 'status':
            value = status(args.root)
        elif args.command == 'acquire':
            value = acquire(args.root, owner=args.owner, branch=args.branch, purpose=args.purpose,
                            pid=args.pid, expected_minutes=args.expected_minutes)
        else:
            value = release(args.root, owner=args.owner, stale=args.stale)
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 1
    print(json.dumps(value, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
