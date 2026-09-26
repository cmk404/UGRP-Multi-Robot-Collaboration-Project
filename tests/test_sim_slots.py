"""sim_slots: counting flock semaphore, crash release, wrapper exit code, ps counting rule."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import sim_slots  # noqa: E402

SCRIPT = str(ROOT / 'scripts' / 'sim_slots.py')


class SimSlotsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / 'slots'

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_counting_semaphore_and_release(self) -> None:
        a = sim_slots.try_acquire(self.root, 2, {'owner': 'kiro', 'pid': os.getpid()})
        b = sim_slots.try_acquire(self.root, 2, {'owner': 'kiro', 'pid': os.getpid()})
        self.assertEqual({a.index, b.index}, {0, 1})
        self.assertIsNone(sim_slots.try_acquire(self.root, 2, {'owner': 'kiro'}))
        rows = sim_slots.status(self.root, 2)
        self.assertEqual([r['held'] for r in rows], [True, True])
        self.assertEqual(rows[0]['owner'], 'kiro')
        a.release()
        c = sim_slots.try_acquire(self.root, 2, {'owner': 'codex'})
        self.assertEqual(c.index, a.index)
        b.release(); c.release()
        self.assertEqual([r['held'] for r in sim_slots.status(self.root, 2)], [False, False])

    def test_timeout_when_full(self) -> None:
        held = sim_slots.try_acquire(self.root, 1, {'owner': 'kiro'})
        try:
            with self.assertRaises(TimeoutError):
                sim_slots.acquire(self.root, 1, {'owner': 'kiro'}, timeout_s=.3, poll_s=.1)
        finally:
            held.release()

    def test_killed_holder_frees_its_slot(self) -> None:
        proc = subprocess.Popen([sys.executable, SCRIPT, '--root', str(self.root), '--slots', '1', 'run',
                                 '--owner', 'kiro', '--label', 'sleeper', '--', sys.executable, '-c',
                                 'import time; time.sleep(60)'], start_new_session=True)
        try:
            deadline = time.time() + 20
            while time.time() < deadline and not sim_slots.status(self.root, 1)[0]['held']:
                time.sleep(.1)
            row = sim_slots.status(self.root, 1)[0]
            self.assertTrue(row['held'])
            self.assertEqual(row['label'], 'sleeper')
            self.assertIsNone(sim_slots.try_acquire(self.root, 1, {'owner': 'codex'}))
        finally:
            os.killpg(proc.pid, signal.SIGKILL)       # our own test process group only
            proc.wait(10)
        deadline = time.time() + 10
        while time.time() < deadline and sim_slots.status(self.root, 1)[0]['held']:
            time.sleep(.1)
        slot = sim_slots.try_acquire(self.root, 1, {'owner': 'codex'})
        self.assertIsNotNone(slot)
        slot.release()

    def test_run_returns_child_exit_code_and_releases(self) -> None:
        code = subprocess.call([sys.executable, SCRIPT, '--root', str(self.root), '--slots', '1', 'run',
                                '--owner', 'kiro', '--', sys.executable, '-c', 'raise SystemExit(7)'])
        self.assertEqual(code, 7)
        self.assertFalse(sim_slots.status(self.root, 1)[0]['held'])
        out = subprocess.run([sys.executable, SCRIPT, '--root', str(self.root), '--slots', '1', 'status'],
                             capture_output=True, text=True, check=True).stdout
        self.assertEqual(json.loads(out)['held'], 0)

    def test_context_manager(self) -> None:
        with sim_slots.sim_slot(owner='kiro', label='ctx', root=self.root, slots=1) as slot:
            self.assertTrue(sim_slots.status(self.root, 1)[0]['held'])
            self.assertEqual(slot.record['label'], 'ctx')
        self.assertFalse(sim_slots.status(self.root, 1)[0]['held'])

    def test_ps_count_excludes_false_positives(self) -> None:
        ps = '\n'.join([
            '101 /opt/homebrew/Cellar/python@3.12/3.12.13/bin/Python -m scripts.run_zone_teacher_fix --output o',
            '102 /path/.venv-sim-worker-mac/bin/python scripts/sim_profile.py m1 --episode m1dev-s93',
            '103 /path/.venv/bin/python scripts/eval_pair.py',
            '104 kiro-cli chat --no-interactive You are Kiro ... Python.*run_ ...',
            '105 /usr/bin/python3 scripts/run_tensorboard.py --logdir x',
            '106 /usr/bin/python3 scripts/ugrp_session.py run kiro-x -- python scripts/run_m1_owncam.py',
            '107 /usr/bin/python3 scripts/sim_slots.py run --owner kiro -- python scripts/run_m1_owncam.py',
            '108 /bin/zsh -c python scripts/run_m1_owncam.py',
            '109 /usr/bin/python3 scripts/other_tool.py',
        ])
        self.assertEqual(sim_slots.count_sim_processes(ps), 3)
        self.assertEqual(sim_slots.count_sim_processes(ps, exclude_pids={101}), 2)

    def test_default_slots_env(self) -> None:
        old = os.environ.get('UGRP_SIM_SLOTS')
        try:
            os.environ['UGRP_SIM_SLOTS'] = '3'
            self.assertEqual(sim_slots.default_slots(), 3)
            del os.environ['UGRP_SIM_SLOTS']
            self.assertEqual(sim_slots.default_slots(), max(1, (os.cpu_count() or 2) - 1))
        finally:
            if old is not None:
                os.environ['UGRP_SIM_SLOTS'] = old


if __name__ == '__main__':
    unittest.main()
