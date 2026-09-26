#!/usr/bin/env python3
"""Machine-wide sim slot queue: at most N running sims, admitted atomically, counted by what actually runs.

N slot files live under the primary checkout's ``outputs/sim-slots``. A slot is
held by an exclusive ``fcntl.flock`` on its file, so the kernel releases it when
the holder exits or crashes (no stale slots, no PID bookkeeping). The holder
writes who/what/when into the file for ``status``. The child command inherits
the locked descriptor, so the slot stays held while either the wrapper or the
child is alive. Nothing here stops or signals another process.

Admission is one critical section under a global ``admission.lock`` flock:
count running sims, compare with N, take a free slot. Two wrappers can no
longer both read "5 running" and start the 6th and 7th sim (Codex review of
PR #209). Running sims = held slots + *unslotted* sims, where a sim is any
process that has MuJoCo (``libmujoco``) loaded - found from the process's
memory mappings (``lsof`` on macOS, ``/proc/<pid>/maps`` on Linux), not from
argv strings - and it is unslotted unless it or an ancestor holds a slot file
(holders and their children are already counted by their slot) or it is itself
a queued waiter (``queue.lock`` open, not running yet). When the
census cannot be taken (no ``lsof``/``/proc``), admission fails instead of
passing.

N defaults to 6, the user's machine-wide cap (``UGRP_SIM_SLOTS`` or ``--slots``
override; every agent should use the default). Held slots are counted over
every slot file present, so a different N cannot hide another holder.

Examples::

    python3 scripts/sim_slots.py status
    python3 scripts/ugrp_session.py run kiro-m1 -- \\
        python3 scripts/sim_slots.py run --owner kiro --label m1-s93 -- \\
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
import math
import os
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_ROOT = Path('/Users/changmin/projects/ugrp/outputs/sim-slots')
MACHINE_CAP = 6          # AGENTS/user rule: at most 6 sim processes machine-wide
POLL_S = 5.0
SIM_LIBRARY = 'libmujoco'
ADMISSION = 'admission.lock'
QUEUE = 'queue.lock'


def positive_int(value, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a positive integer, got {value!r}')
    try:
        number = int(str(value).strip())
    except ValueError:
        raise ValueError(f'{name} must be a positive integer, got {value!r}') from None
    if number < 1:
        raise ValueError(f'{name} must be a positive integer, got {value!r}')
    return number


def default_slots() -> int:
    env = os.environ.get('UGRP_SIM_SLOTS')
    return MACHINE_CAP if env is None or not env.strip() else positive_int(env, 'UGRP_SIM_SLOTS')


# ------------------------------------------------------------------ census
def _queue_role(path: str, prefix: str) -> str | None:
    """'holder' for a slot file, 'waiter' for the queue/admission files, None outside the slot directory."""
    if not path.startswith(prefix):
        return None
    name = path[len(prefix):]
    return 'holder' if name.startswith('slot-') and name.endswith('.lock') else 'waiter'


def parse_lsof(text: str, root: Path) -> tuple[set[int], set[int], set[int]]:
    """(pids with libmujoco mapped, slot holders, queue waiters) from ``lsof -F pfn`` (regular descriptors only)."""
    prefix = os.path.realpath(root) + os.sep
    sims, roles, pid, fd = set(), {'holder': set(), 'waiter': set()}, None, ''
    for line in text.splitlines():
        tag, value = line[:1], line[1:]
        if tag == 'p':
            pid, fd = (int(value) if value.isdigit() else None), ''
        elif tag == 'f':
            fd = value
        elif tag == 'n' and pid is not None:
            if fd in ('txt', 'mem') and SIM_LIBRARY in os.path.basename(value):
                sims.add(pid)
            elif fd.isdigit() and (role := _queue_role(value, prefix)):
                roles[role].add(pid)
    return sims, roles['holder'], roles['waiter']


def scan_proc(root: Path, proc: Path = Path('/proc')) -> tuple[set[int], set[int], set[int], dict[int, int]]:
    """Linux: the same census from ``/proc/<pid>/{stat,maps,fd}`` (unreadable processes are skipped)."""
    prefix = os.path.realpath(root) + os.sep
    sims, roles, parents = set(), {'holder': set(), 'waiter': set()}, {}
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            parents[pid] = int(fields[1])
        except (OSError, IndexError, ValueError):
            continue
        with contextlib.suppress(OSError):
            if SIM_LIBRARY in (entry/'maps').read_text():
                sims.add(pid)
        with contextlib.suppress(OSError):
            for fd in (entry/'fd').iterdir():
                with contextlib.suppress(OSError):
                    if role := _queue_role(os.readlink(fd), prefix):
                        roles[role].add(pid)
    return sims, roles['holder'], roles['waiter'], parents


def _scan(root: Path) -> tuple[set[int], set[int], set[int], dict[int, int]]:
    if sys.platform.startswith('linux'):
        try:
            return scan_proc(root)
        except OSError as exc:
            raise RuntimeError(f'cannot take the sim census (/proc): {exc}') from exc
    try:
        listing = subprocess.run(['lsof', '-n', '-P', '-w', '-F', 'pfn'], capture_output=True, text=True, timeout=60)
        table = subprocess.run(['ps', '-Ao', 'pid=,ppid='], capture_output=True, text=True, timeout=60, check=True)
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError(f'cannot take the sim census (lsof/ps): {exc}') from exc
    if not listing.stdout.strip():
        raise RuntimeError(f'cannot take the sim census: lsof returned nothing (exit {listing.returncode}) '
                           f'{listing.stderr.strip()[:200]}')
    sims, holders, waiters = parse_lsof(listing.stdout, root)
    parents = {}
    for line in table.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
            parents[int(parts[0])] = int(parts[1])
    return sims, holders, waiters, parents


def census(root: Path) -> dict:
    """Running MuJoCo processes and which of them are outside the slot queue.

    A sim is inside the queue when it or an ancestor holds a slot file (its slot counts it), or when it is
    itself waiting (queue/admission file open, not running yet). A waiter's children are not covered: the
    admitting process may have started sims of its own outside any slot.
    """
    sims, holders, waiters, parents = _scan(root)

    def under_holder(pid: int) -> bool:
        seen = set()
        while pid > 1 and pid not in seen:
            if pid in holders:
                return True
            seen.add(pid)
            pid = parents.get(pid, 0)
        return False
    return {'sim_pids': sorted(sims), 'holder_pids': sorted(holders), 'waiter_pids': sorted(waiters),
            'unslotted_sim_pids': sorted(p for p in sims if p not in waiters and not under_holder(p))}


# ------------------------------------------------------------------ slots
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


def _is_held(path: Path) -> tuple[bool, str]:
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            held = False
            fcntl.flock(fd, fcntl.LOCK_UN)
        except BlockingIOError:
            held = True
        return held, os.pread(fd, 65536, 0).decode(errors='replace').strip()
    finally:
        os.close(fd)


def held_count(root: Path) -> int:
    """Held slots over every slot file present (a holder with another N is still counted)."""
    return sum(_is_held(p)[0] for p in sorted(root.glob('slot-*.lock')))


def _try_slot(root: Path, slots: int, record: dict) -> Slot | None:
    for index, path in enumerate(_slot_paths(root, slots)):
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            continue
        value = {**record, 'slot': index, 'slots': slots, 'acquired_unix': round(time.time(), 3),
                 'loadavg_at_acquire': [round(v, 2) for v in os.getloadavg()]}
        _write_record(fd, value)
        return Slot(path, fd, index, value)
    return None


def _write_record(fd: int, record: dict) -> None:
    os.ftruncate(fd, 0)
    os.pwrite(fd, (json.dumps(record, ensure_ascii=False) + '\n').encode(), 0)


@contextlib.contextmanager
def _admission(root: Path):
    fd = os.open(root/ADMISSION, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)          # held only for one census + reservation
        yield
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)


def try_admit(root: Path, slots: int, record: dict) -> tuple[Slot | None, dict]:
    """One atomic admission attempt: (slot or None, the census it was decided on)."""
    slots = positive_int(slots, 'slots')
    _slot_paths(root, slots)
    with _admission(root):
        seen = census(root)
        held = held_count(root)
        seen = {'held_slots': held, 'unslotted_sims': len(seen['unslotted_sim_pids']),
                'unslotted_sim_pids': seen['unslotted_sim_pids'][:20]}
        seen['running'] = held + seen['unslotted_sims']
        slot = _try_slot(root, slots, record) if seen['running'] < slots else None
    return slot, seen


def acquire(root: Path, slots: int, record: dict, *, timeout_s: float = 0., poll_s: float = POLL_S,
            log=None) -> Slot:
    """Block until fewer than ``slots`` sims run machine-wide and a slot is free; timeout 0 = wait forever."""
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s < 0:
        raise ValueError(f'timeout_s must be a finite number >= 0, got {timeout_s!r}')
    slots = positive_int(slots, 'slots')
    _slot_paths(root, slots)
    started, last = time.time(), 0.
    queue_fd = os.open(root/QUEUE, os.O_RDWR | os.O_CREAT, 0o644)   # marks this process as waiting, not running
    try:
        while True:
            slot, seen = try_admit(root, slots, record)
            if slot is not None:
                slot.record.update(waited_s=round(time.time() - started, 1), census_at_acquire=seen)
                _write_record(slot.fd, slot.record)
                return slot
            if timeout_s and time.time() - started >= timeout_s:
                raise TimeoutError(f'no sim slot within {timeout_s:.0f} s ({seen["held_slots"]} slots held, '
                                   f'{seen["unslotted_sims"]} sims outside the queue, cap {slots})')
            if log and time.time() - last >= 60:
                log(f'sim_slots: waiting ({seen["held_slots"]} slots held + {seen["unslotted_sims"]} sims outside '
                    f'the queue, cap {slots}, load {os.getloadavg()[0]:.1f})')
                last = time.time()
            time.sleep(poll_s)
    finally:
        os.close(queue_fd)


def status(root: Path, slots: int) -> list[dict]:
    rows = []
    for index, path in enumerate(_slot_paths(root, slots)):
        held, text = _is_held(path)
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
             timeout_s: float = 0.):
    slot = acquire(root, default_slots() if slots is None else slots,
                   {'owner': owner, 'label': label, 'pid': os.getpid(), 'command': ' '.join(sys.argv)[:500]},
                   timeout_s=timeout_s, log=lambda m: print(m, file=sys.stderr, flush=True))
    try:
        yield slot
    finally:
        slot.release()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--slots', default=None,
                        help=f'machine-wide sim cap = slot count (default: UGRP_SIM_SLOTS or {MACHINE_CAP})')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    run = sub.add_parser('run', help='wait for a slot, run the command holding it, return its exit code')
    run.add_argument('--owner', required=True, help='claude, codex or kiro')
    run.add_argument('--label', default='')
    run.add_argument('--timeout', type=float, default=0., help='give up after this many seconds (0 = wait forever)')
    run.add_argument('cmd', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    try:
        slots = default_slots() if args.slots is None else positive_int(args.slots, '--slots')
    except ValueError as exc:
        parser.error(str(exc))
    if args.command == 'status':
        rows = status(args.root, slots)
        try:
            seen = census(args.root)
        except RuntimeError as exc:
            print(f'sim_slots: {exc}', file=sys.stderr)
            return 70
        mism = sorted({r['slots'] for r in rows if r.get('held') and r.get('slots') not in (None, slots)})
        print(json.dumps({'slots': slots, 'held': held_count(args.root), 'slot_count_mismatch': mism,
                          'sim_processes_running': len(seen['sim_pids']),
                          'unslotted_sim_pids': seen['unslotted_sim_pids'],
                          'loadavg': [round(v, 2) for v in os.getloadavg()], 'holders': [r for r in rows if r['held']]},
                         ensure_ascii=False, indent=2))
        return 0
    cmd = args.cmd[1:] if args.cmd[:1] == ['--'] else args.cmd
    if not cmd:
        parser.error('run needs a command after --')
    if not math.isfinite(args.timeout) or args.timeout < 0:
        parser.error(f'--timeout must be a finite number >= 0, got {args.timeout!r}')
    record = {'owner': args.owner, 'label': args.label, 'pid': os.getpid(), 'command': ' '.join(cmd)[:500]}
    try:
        slot = acquire(args.root, slots, record, timeout_s=args.timeout,
                       log=lambda m: print(m, file=sys.stderr, flush=True))
    except TimeoutError as exc:
        print(f'sim_slots: {exc}', file=sys.stderr)
        return 75
    except RuntimeError as exc:           # census unavailable: never start unchecked
        print(f'sim_slots: {exc}', file=sys.stderr)
        return 70
    print(f'sim_slots: slot {slot.index}/{slots} after {slot.record["waited_s"]} s '
          f'({slot.record["census_at_acquire"]["running"]} running before)', file=sys.stderr, flush=True)
    try:
        # the child inherits the locked descriptor: the slot is held while either process lives
        return subprocess.call(cmd, pass_fds=(slot.fd,))
    except KeyboardInterrupt:
        return 130
    finally:
        slot.release()


if __name__ == '__main__':
    sys.exit(main())
