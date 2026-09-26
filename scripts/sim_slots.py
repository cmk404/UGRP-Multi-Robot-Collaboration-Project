#!/usr/bin/env python3
"""Machine-wide sim slot queue: a counting file-lock semaphore so agents queue instead of oversubscribing.

N slot files live under the primary checkout's ``outputs/sim-slots``. A slot is
held by an exclusive ``fcntl.flock`` on its file, so the kernel releases it when
the holder exits or crashes (no stale slots, no PID bookkeeping). The holder
writes who/what/when into the file for ``status``. The child command inherits
the locked descriptor, so the slot stays held while either the wrapper or the
child is alive. Nothing here stops or signals another process.

Default N = cores - 1 (``UGRP_SIM_SLOTS`` or ``--slots`` override). Every agent
must use the same N for the queue to mean anything; the slot count is written
into each holder record and ``status`` reports mismatches.

Optional ``--ps-cap K`` also waits while K or more sim processes are running
machine-wide, queued or not (the user's machine-wide rule), counted from ``ps``
without the kiro-cli prompt text / TensorBoard / session-wrapper false positives
that the plain ``grep -c`` rule also counts.

Examples::

    python3 scripts/sim_slots.py status
    python3 scripts/ugrp_session.py run kiro-m1 -- \\
        python3 scripts/sim_slots.py run --owner kiro --label m1-s93 --ps-cap 6 -- \\
        .venv-sim-worker-mac/bin/python scripts/run_m1_owncam.py --prereg ... --speedups exact-v1

Python (runner side)::

    from scripts.sim_slots import sim_slot
    with sim_slot(owner='kiro', label='m1-s93'):
        run(...)
"""
from __future__ import annotations

import argparse
import contextlib
import fcntl
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_ROOT = Path('/Users/changmin/projects/ugrp/outputs/sim-slots')
POLL_S = 5.0
# A sim process: a python executable whose arguments name a runner/evaluator (the AGENTS count rule).
_SIM_ARGS = re.compile(r'(run_|scripts\.run|eval_|sim_profile)')
_EXCLUDE = ('kiro-cli', 'run_tensorboard', 'sim_slots.py', 'ugrp_session.py')


def default_slots() -> int:
    env = os.environ.get('UGRP_SIM_SLOTS')
    if env:
        return max(1, int(env))
    return max(1, (os.cpu_count() or 2) - 1)


def count_sim_processes(ps_text: str | None = None, exclude_pids: set[int] | None = None) -> int:
    """Running sim processes by the machine-wide rule, without kiro-cli prompt-text false positives."""
    if ps_text is None:
        ps_text = subprocess.run(['ps', '-Ao', 'pid=,command='], capture_output=True, text=True).stdout
    exclude_pids = exclude_pids or set()
    count = 0
    for line in ps_text.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) != 2 or not parts[0].isdigit() or int(parts[0]) in exclude_pids:
            continue
        exe, _, args = parts[1].partition(' ')
        if 'python' not in Path(exe).name.lower() or any(x in parts[1] for x in _EXCLUDE):
            continue
        if _SIM_ARGS.search(args):
            count += 1
    return count


class Slot:
    def __init__(self, path: Path, fd: int, index: int, record: dict):
        self.path, self.fd, self.index, self.record = path, fd, index, record

    def release(self) -> None:
        if self.fd >= 0:
            try:
                os.ftruncate(self.fd, 0)
            finally:
                fcntl.flock(self.fd, fcntl.LOCK_UN)
                os.close(self.fd)
                self.fd = -1


def _slot_paths(root: Path, slots: int) -> list[Path]:
    root.mkdir(parents=True, exist_ok=True)
    return [root / f'slot-{i:02d}.lock' for i in range(slots)]


def try_acquire(root: Path, slots: int, record: dict) -> Slot | None:
    for index, path in enumerate(_slot_paths(root, slots)):
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            continue
        value = {**record, 'slot': index, 'slots': slots, 'acquired_unix': round(time.time(), 3),
                 'loadavg_at_acquire': [round(v, 2) for v in os.getloadavg()]}
        os.ftruncate(fd, 0)
        os.pwrite(fd, (json.dumps(value, ensure_ascii=False) + '\n').encode(), 0)
        return Slot(path, fd, index, value)
    return None


def acquire(root: Path, slots: int, record: dict, *, ps_cap: int = 0, timeout_s: float = 0.,
            poll_s: float = POLL_S, log=None) -> Slot:
    """Block until a slot is free (and, with ps_cap, fewer than ps_cap unqueued sims run)."""
    started, last = time.time(), 0.
    while True:
        slot = None
        running = count_sim_processes(exclude_pids={os.getpid()}) if ps_cap else 0
        if not ps_cap or running < ps_cap:
            slot = try_acquire(root, slots, record)
        if slot is not None:
            slot.record['waited_s'] = round(time.time() - started, 1)
            slot.record['sim_processes_at_acquire'] = running if ps_cap else None
            return slot
        held = len(held_pids(root, slots))
        if timeout_s and time.time() - started >= timeout_s:
            raise TimeoutError(f'no sim slot within {timeout_s:.0f} s ({held}/{slots} held, {running} sims running)')
        if log and time.time() - last >= 60:
            log(f'sim_slots: waiting ({held}/{slots} slots held, {running} sims running, '
                f'load {os.getloadavg()[0]:.1f})')
            last = time.time()
        time.sleep(poll_s)


def held_pids(root: Path, slots: int) -> set[int]:
    return {int(h['pid']) for h in status(root, slots) if h.get('held') and isinstance(h.get('pid'), int)}


def status(root: Path, slots: int) -> list[dict]:
    rows = []
    for index, path in enumerate(_slot_paths(root, slots)):
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                held = False
                fcntl.flock(fd, fcntl.LOCK_UN)
            except BlockingIOError:
                held = True
            text = os.pread(fd, 65536, 0).decode(errors='replace').strip()
        finally:
            os.close(fd)
        row = {'slot': index, 'held': held}
        if held and text:
            try:
                row.update(json.loads(text))
            except ValueError:
                row['record'] = text[:200]
            row['slot'] = index
        rows.append(row)
    return rows


@contextlib.contextmanager
def sim_slot(*, owner: str, label: str = '', root: Path = DEFAULT_ROOT, slots: int | None = None,
             ps_cap: int = 0, timeout_s: float = 0.):
    slot = acquire(root, slots or default_slots(), {'owner': owner, 'label': label, 'pid': os.getpid(),
                                                    'command': ' '.join(sys.argv)[:500]},
                   ps_cap=ps_cap, timeout_s=timeout_s, log=lambda m: print(m, file=sys.stderr, flush=True))
    try:
        yield slot
    finally:
        slot.release()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--slots', type=int, default=None, help='slot count (default: UGRP_SIM_SLOTS or cores - 1)')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    run = sub.add_parser('run', help='wait for a slot, run the command holding it, return its exit code')
    run.add_argument('--owner', required=True, help='claude, codex or kiro')
    run.add_argument('--label', default='')
    run.add_argument('--ps-cap', type=int, default=0, help='also wait while this many unqueued sims run (0 = off)')
    run.add_argument('--timeout', type=float, default=0., help='give up after this many seconds (0 = wait forever)')
    run.add_argument('cmd', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    slots = args.slots or default_slots()
    if args.command == 'status':
        rows = status(args.root, slots)
        mism = sorted({r['slots'] for r in rows if r.get('held') and r.get('slots') not in (None, slots)})
        print(json.dumps({'slots': slots, 'held': sum(r['held'] for r in rows), 'slot_count_mismatch': mism,
                          'sim_processes_running': count_sim_processes(exclude_pids={os.getpid()}),
                          'loadavg': [round(v, 2) for v in os.getloadavg()], 'holders': [r for r in rows if r['held']]},
                         ensure_ascii=False, indent=2))
        return 0
    cmd = args.cmd[1:] if args.cmd[:1] == ['--'] else args.cmd
    if not cmd:
        parser.error('run needs a command after --')
    record = {'owner': args.owner, 'label': args.label, 'pid': os.getpid(), 'command': ' '.join(cmd)[:500]}
    try:
        slot = acquire(args.root, slots, record, ps_cap=args.ps_cap, timeout_s=args.timeout,
                       log=lambda m: print(m, file=sys.stderr, flush=True))
    except TimeoutError as exc:
        print(f'sim_slots: {exc}', file=sys.stderr)
        return 75
    print(f'sim_slots: slot {slot.index}/{slots} after {slot.record["waited_s"]} s', file=sys.stderr, flush=True)
    try:
        # the child inherits the locked descriptor: the slot is held while either process lives
        return subprocess.call(cmd, pass_fds=(slot.fd,))
    except KeyboardInterrupt:
        return 130
    finally:
        slot.release()


if __name__ == '__main__':
    sys.exit(main())
