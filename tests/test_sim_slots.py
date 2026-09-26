"""sim_slots: atomic admission against the machine-wide cap, census by loaded MuJoCo, crash release, CLI bounds.

MuJoCo itself is not needed here (the real-process census is in tests/test_sim_speed_runtime.py); the census is
patched wherever the machine's own running sims could change the outcome.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import sim_slots  # noqa: E402

SCRIPT = str(ROOT / 'scripts' / 'sim_slots.py')
MANY = '64'       # subprocess tests: a cap the machine's own sims cannot reach, so only this test's slots matter


def fake_census(unslotted=(), delay: float = 0.):
    def census(root):
        time.sleep(delay)
        return {'sim_pids': sorted(unslotted), 'holder_pids': [], 'waiter_pids': [],
                'unslotted_sim_pids': sorted(unslotted)}
    return census


class SimSlotsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / 'slots'
        patcher = mock.patch.object(sim_slots, 'census', fake_census())
        self.census = patcher.start()
        self.addCleanup(patcher.stop)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def admit(self, slots: int, owner: str = 'kiro', timeout: float = .3) -> sim_slots.Slot:
        return sim_slots.acquire(self.root, slots, {'owner': owner, 'pid': os.getpid()}, timeout_s=timeout,
                                 poll_s=.05)

    def test_default_is_the_machine_cap(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop('UGRP_SIM_SLOTS', None)
            self.assertEqual(sim_slots.default_slots(), 6)
            self.assertEqual(sim_slots.MACHINE_CAP, 6)
            os.environ['UGRP_SIM_SLOTS'] = ''
            self.assertEqual(sim_slots.default_slots(), 6)
            os.environ['UGRP_SIM_SLOTS'] = '3'
            self.assertEqual(sim_slots.default_slots(), 3)
            for bad in ('0', '-2', 'abc', '2.5', 'nan'):
                os.environ['UGRP_SIM_SLOTS'] = bad
                with self.subTest(bad=bad), self.assertRaises(ValueError):
                    sim_slots.default_slots()

    def test_counting_semaphore_and_release(self) -> None:
        a, b = self.admit(2), self.admit(2)
        self.assertEqual({a.index, b.index}, {0, 1})
        with self.assertRaises(TimeoutError):
            self.admit(2, owner='codex')
        rows = sim_slots.status(self.root, 2)
        self.assertEqual([r['held'] for r in rows], [True, True])
        self.assertEqual(rows[0]['owner'], 'kiro')
        self.assertEqual(rows[0]['census_at_acquire'], {'held_slots': 0, 'unslotted_sims': 0,
                                                        'unslotted_sim_pids': [], 'running': 0})
        a.release()
        c = self.admit(2, owner='codex')
        self.assertEqual(c.index, a.index)
        b.release(); c.release()
        self.assertEqual([r['held'] for r in sim_slots.status(self.root, 2)], [False, False])

    def test_unslotted_sims_count_toward_the_cap(self) -> None:
        self.census = fake_census({901, 902})
        with mock.patch.object(sim_slots, 'census', self.census):
            slot, seen = sim_slots.try_admit(self.root, 3, {'owner': 'kiro'})
            self.assertEqual(seen['running'], 2)
            self.assertIsNotNone(slot)
            second, seen = sim_slots.try_admit(self.root, 3, {'owner': 'kiro'})
            self.assertIsNone(second)
            self.assertEqual(seen, {'held_slots': 1, 'unslotted_sims': 2, 'unslotted_sim_pids': [901, 902],
                                    'running': 3})
            slot.release()

    def test_concurrent_admission_cannot_exceed_the_cap(self) -> None:
        """Codex review #2: five sims run, two wrappers ask at once -> exactly one starts (not 7 running)."""
        census = fake_census({901, 902, 903, 904, 905}, delay=.2)     # a slow census widens the race window
        got, errors, gate = [], [], threading.Barrier(2)

        def ask(owner):
            gate.wait()
            try:
                got.append(sim_slots.acquire(self.root, 6, {'owner': owner}, timeout_s=1.2, poll_s=.05))
            except TimeoutError as exc:
                errors.append(exc)
        with mock.patch.object(sim_slots, 'census', census):
            threads = [threading.Thread(target=ask, args=(o,)) for o in ('kiro', 'codex')]
            for t in threads:
                t.start()
            for t in threads:
                t.join(10)
        self.assertEqual((len(got), len(errors)), (1, 1), (got, errors))
        self.assertEqual(got[0].record['census_at_acquire']['running'], 5)
        got[0].release()

    def test_holders_with_another_slot_count_are_still_counted(self) -> None:
        held = [self.admit(3) for _ in range(3)]
        held[0].release()
        held[1].release()                  # slot-02 (outside a 2-slot view) stays held
        one = self.admit(2)
        self.assertEqual(one.index, 0)
        with self.assertRaises(TimeoutError):
            self.admit(2)                  # slot-01 is free, but 2 sims already run
        one.release()
        held[2].release()

    real_census = staticmethod(sim_slots.census)       # captured before setUp patches the module attribute

    def test_census_ancestry_and_cycles(self) -> None:
        # 20 holds a slot (child 11, grandchild 12 are its sim); 40 waits (itself not running, but its child 41
        # was started outside any slot); 30/31 form a parent cycle
        scan = ({10, 11, 12, 30, 40, 41}, {20}, {40},
                {10: 1, 11: 20, 12: 11, 20: 1, 30: 31, 31: 30, 40: 1, 41: 40})
        with mock.patch.object(sim_slots, '_scan', return_value=scan):
            seen = self.real_census(self.root)
        self.assertEqual(seen['unslotted_sim_pids'], [10, 30, 41])
        self.assertEqual((seen['holder_pids'], seen['waiter_pids']), ([20], [40]))

    def test_census_failure_refuses_admission(self) -> None:
        with mock.patch.object(sim_slots, 'census', self.real_census), \
                mock.patch.object(sim_slots, '_scan', side_effect=RuntimeError('no lsof')):
            with self.assertRaises(RuntimeError):
                self.admit(6)
        self.assertEqual(sim_slots.held_count(self.root), 0)

    @unittest.skipUnless(sys.platform == 'darwin', 'lsof census path')
    def test_missing_lsof_is_an_error(self) -> None:
        with mock.patch.object(sim_slots.subprocess, 'run', side_effect=FileNotFoundError('lsof')):
            with self.assertRaises(RuntimeError):
                self.real_census(self.root)
        empty = subprocess.CompletedProcess([], 1, stdout='', stderr='lsof: boom')
        with mock.patch.object(sim_slots.subprocess, 'run', return_value=empty):
            with self.assertRaises(RuntimeError):
                self.real_census(self.root)

    def test_missing_proc_is_an_error(self) -> None:
        with mock.patch.object(sim_slots.sys, 'platform', 'linux'), \
                mock.patch.object(sim_slots, 'scan_proc', side_effect=FileNotFoundError('/proc')):
            with self.assertRaises(RuntimeError):
                self.real_census(self.root)

    def test_parse_lsof_counts_loaded_mujoco_not_argv(self) -> None:
        self.root.mkdir(parents=True)
        root = os.path.realpath(self.root)
        text = '\n'.join([
            'p101', 'ftxt', 'n/venv/lib/python3.12/site-packages/mujoco/libmujoco.3.12.0.dylib',   # a sim
            'p102', 'ftxt', 'n/opt/python/bin/python3.12', 'f3', 'n/tmp/run_m1_owncam.log',        # argv-like, no sim
            'p103', 'fcwd', f'n{root}', 'ftxt', 'n/usr/lib/libz.dylib',                          # cwd only
            'p104', 'f7', f'n{root}/slot-00.lock', 'ftxt', 'n/x/mujoco/libmujoco.3.12.0.dylib',  # holder + sim
            'p105', 'f4', 'n/data/libmujoco.notes.txt',                                          # opened, not mapped
            'p106', 'fmem', 'n/usr/lib/libmujoco.so.3.12.0',                                     # Linux lsof style
            'p107', 'f5', f'n{root}/queue.lock', 'f6', f'n{root}/admission.lock',                # waiter
            'p108', 'f5', f'n{root}-other/slot-00.lock',                                         # another dir
            'pX', 'ftxt', 'n/x/libmujoco.dylib',
        ])
        self.assertEqual(sim_slots.parse_lsof(text, self.root), ({101, 104, 106}, {104}, {107}))

    def test_scan_proc_reads_maps_fds_and_parents(self) -> None:
        self.root.mkdir(parents=True)
        (self.root/'slot-00.lock').write_text('')
        proc = Path(self.tmp.name) / 'proc'

        def pid(n, ppid, maps='', fds=(), stat=True):
            d = proc/str(n)
            (d/'fd').mkdir(parents=True)
            if stat:
                (d/'stat').write_text(f'{n} (py thon) S {ppid} 1 1\n')
            (d/'maps').write_text(maps)
            for i, target in enumerate(fds):
                os.symlink(target, d/'fd'/str(i))
        lib = '7f00-7f10 r-xp 0 08:01 5 /venv/site-packages/mujoco/libmujoco.so.3.12.0\n'
        pid(100, 1, maps=lib)
        pid(101, 1, fds=[os.path.realpath(self.root/'slot-00.lock'), '/dev/null'])   # the kernel's canonical path
        pid(102, 101, maps=lib)
        pid(103, 1, maps=lib, stat=False)              # vanished / unreadable: skipped
        pid(104, 1, maps=lib, fds=[os.path.realpath(self.root) + '/queue.lock'])
        (proc/'self').mkdir()
        sims, holders, waiters, parents = sim_slots.scan_proc(self.root, proc)
        self.assertEqual((sims, holders, waiters), ({100, 102, 104}, {101}, {104}))
        self.assertEqual(parents, {100: 1, 101: 1, 102: 101, 104: 1})

    def test_killed_holder_frees_its_slot(self) -> None:
        proc = subprocess.Popen([sys.executable, SCRIPT, '--root', str(self.root), '--slots', MANY, 'run',
                                 '--owner', 'kiro', '--label', 'sleeper', '--', sys.executable, '-c',
                                 'import time; time.sleep(60)'], start_new_session=True)
        try:
            deadline = time.time() + 20
            while time.time() < deadline and not sim_slots.held_count(self.root):
                time.sleep(.1)
            row = sim_slots.status(self.root, 1)[0]
            self.assertTrue(row['held'])
            self.assertEqual(row['label'], 'sleeper')
            self.assertEqual(row['slots'], 64)
            self.assertIn('census_at_acquire', row)
        finally:
            os.killpg(proc.pid, signal.SIGKILL)       # our own test process group only
            proc.wait(10)
        deadline = time.time() + 10
        while time.time() < deadline and sim_slots.held_count(self.root):
            time.sleep(.1)
        self.assertEqual(sim_slots.held_count(self.root), 0)

    def test_run_returns_child_exit_code_and_releases(self) -> None:
        code = subprocess.call([sys.executable, SCRIPT, '--root', str(self.root), '--slots', MANY, 'run',
                                '--owner', 'kiro', '--', sys.executable, '-c', 'raise SystemExit(7)'],
                               stderr=subprocess.DEVNULL)
        self.assertEqual(code, 7)
        self.assertEqual(sim_slots.held_count(self.root), 0)
        out = subprocess.run([sys.executable, SCRIPT, '--root', str(self.root), '--slots', MANY, 'status'],
                             capture_output=True, text=True, check=True).stdout
        self.assertEqual(json.loads(out)['held'], 0)

    def test_cli_rejects_bad_values(self) -> None:
        cases = (['--slots', '0', 'status'], ['--slots', '-1', 'status'], ['--slots', 'x', 'status'],
                 ['run', '--owner', 'kiro'], ['run', '--owner', 'kiro', '--timeout', 'nan', '--', 'true'],
                 ['run', '--owner', 'kiro', '--timeout', '-1', '--', 'true'], ['run', '--owner', 'kiro', '--ps-cap',
                                                                               '6', '--', 'true'])
        for argv in cases:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as ctx:
                sim_slots.main(['--root', str(self.root), *argv])
            self.assertEqual(ctx.exception.code, 2)
        for bad in (-1., float('nan'), float('inf'), True):
            with self.subTest(timeout=bad), self.assertRaises(ValueError):
                sim_slots.acquire(self.root, 1, {}, timeout_s=bad)
        for bad in (0, -3, None, True, 'x'):
            with self.subTest(slots=bad), self.assertRaises(ValueError):
                sim_slots.acquire(self.root, bad, {}, timeout_s=.1)

    def test_context_manager(self) -> None:
        with sim_slots.sim_slot(owner='kiro', label='ctx', root=self.root, slots=1) as slot:
            self.assertTrue(sim_slots.status(self.root, 1)[0]['held'])
            self.assertEqual(slot.record['label'], 'ctx')
        self.assertFalse(sim_slots.status(self.root, 1)[0]['held'])


if __name__ == '__main__':
    unittest.main()
