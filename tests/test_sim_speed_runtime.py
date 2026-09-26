"""Sim-speed tools against real MuJoCo processes (CI: the ubuntu-simulation-runtime job, which installs MuJoCo).

* sim_slots census: a process counts as a sim because it loaded libmujoco, not because of its argv; slot holders,
  their descendants and queued waiters are not counted twice; admission follows the real census.
* sim_profile: a failed preparation undoes every patch in the same process.
* pair_prof: the injected speed-up sources are pinned by the hash of the bytes executed (Codex review of PR #209).
"""
from __future__ import annotations

import contextlib
import hashlib
import io
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import sim_profile, sim_slots  # noqa: E402
from sim import exact_speedups  # noqa: E402

SLOTS = str(ROOT/'scripts'/'sim_slots.py')
PAIR_PROF = ROOT/'experiments'/'2026-09-26-sim-speed'/'pair_prof.py'
SIM = 'import mujoco, os, time; print(os.getpid(), flush=True); time.sleep(60)'
PARENT_OF_SIM = f'import subprocess, sys; subprocess.call([sys.executable, "-c", {SIM!r}])'
NOT_SIM = 'import os, time; print(os.getpid(), flush=True); time.sleep(60)'


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class RealCensusTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)/'slots'
        self.procs: list[subprocess.Popen] = []

    def tearDown(self) -> None:
        for proc in self.procs:                       # only the process groups this test started
            if proc.poll() is None:
                with contextlib.suppress(ProcessLookupError, PermissionError):
                    os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(10)
        self.tmp.cleanup()

    def spawn(self, code: str, *, root: Path | None = None, argv=()) -> int:
        cmd = [sys.executable, '-c', code, *argv]
        if root is not None:
            cmd = [sys.executable, SLOTS, '--root', str(root), '--slots', '64', 'run', '--owner', 'kiro', '--', *cmd]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                                start_new_session=True)
        self.procs.append(proc)
        line = proc.stdout.readline()               # the sim process prints its pid once MuJoCo is loaded
        self.assertTrue(line.strip().isdigit(), f'no pid from {cmd}')
        return int(line)

    def test_counts_loaded_mujoco_not_argv(self) -> None:
        outside = self.spawn(SIM)
        slotted = self.spawn(SIM, root=self.root)
        grandchild = self.spawn(PARENT_OF_SIM, root=self.root)      # did not inherit the slot descriptor
        decoy = self.spawn(NOT_SIM, argv=['scripts/run_m1_owncam.py', 'scripts.run_x', 'eval_y', 'kiro-cli'])
        seen = sim_slots.census(self.root)
        self.assertTrue({outside, slotted, grandchild} <= set(seen['sim_pids']), seen)
        self.assertNotIn(decoy, seen['sim_pids'])
        self.assertIn(outside, seen['unslotted_sim_pids'])
        self.assertNotIn(slotted, seen['unslotted_sim_pids'])
        self.assertNotIn(grandchild, seen['unslotted_sim_pids'])
        self.assertEqual(sim_slots.held_count(self.root), 2)

    def test_waiting_runner_is_not_running(self) -> None:
        self.root.mkdir(parents=True)
        blocker = sim_slots._try_slot(self.root, 1, {'owner': 'test'})          # the only slot, held here
        try:
            waiter = self.spawn(
                f'import mujoco, os, sys; sys.path.insert(0, {str(ROOT)!r}); from pathlib import Path; '
                f'from scripts import sim_slots; print(os.getpid(), flush=True); '
                f'sim_slots.acquire(Path({str(self.root)!r}), 1, {{"owner": "waiter"}}, poll_s=.1)')
            deadline = time.time() + 20
            while time.time() < deadline and waiter not in sim_slots.census(self.root)['waiter_pids']:
                time.sleep(.2)
            seen = sim_slots.census(self.root)
            self.assertIn(waiter, seen['sim_pids'])
            self.assertIn(waiter, seen['waiter_pids'])
            self.assertNotIn(waiter, seen['unslotted_sim_pids'])
        finally:
            blocker.release()

    def test_admission_follows_the_real_census(self) -> None:
        outside = self.spawn(SIM)             # a child of this (admitting) process, outside any slot
        slotted = self.spawn(SIM, root=self.root)
        real, ours = sim_slots.census, {outside, slotted}

        def only_ours(root):                      # other agents' sims on this machine must not decide the test
            seen = real(root)
            return {**seen, 'unslotted_sim_pids': [p for p in seen['unslotted_sim_pids'] if p in ours]}
        with mock.patch.object(sim_slots, 'census', only_ours):
            with self.assertRaises(TimeoutError):                 # 1 held + 1 outside = cap 2
                sim_slots.acquire(self.root, 2, {'owner': 'kiro'}, timeout_s=.6, poll_s=.1)
            os.kill(outside, signal.SIGKILL)
            deadline = time.time() + 20
            while time.time() < deadline and outside in real(self.root)['sim_pids']:
                time.sleep(.2)
            slot = sim_slots.acquire(self.root, 2, {'owner': 'kiro'}, timeout_s=10, poll_s=.1)
        self.assertEqual(slot.record['census_at_acquire']['running'], 1)
        slot.release()


class ProfileRestoreTests(unittest.TestCase):
    def test_failed_prepare_restores_every_patch(self) -> None:
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        step, close = mujoco.mj_step, MultiMasterPiProductionV2.close
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                sim_profile.main(['m1', '--episode', 'no-such-episode', '--output', str(Path(tmp)/'p')])
        self.assertIs(mujoco.mj_step, step)
        self.assertIs(MultiMasterPiProductionV2.close, close)


FAKE_WORLD = '''class MultiMasterPiProductionV2:
    def __init__(self):
        self.controllers, self._render_executor, self._fast_drive_kernel = {}, None, None

    def close(self):
        pass
'''
FAKE_DYNAMICS = '''import numpy as np
MAX_WHEEL_RAD_S = 12.0
FORWARD_PATTERN = np.array([1.0, 1.0, 1.0, 1.0])
LEFT_PATTERN = np.array([-1.0, 1.0, 1.0, -1.0])
YAW_LEFT_PATTERN = np.array([-1.0, 1.0, -1.0, 1.0])
'''
FAKE_RUNNER = '''import argparse, json, os
from pathlib import Path
import mujoco
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seed')
    p.add_argument('--output')
    a = p.parse_args()
    world = MultiMasterPiProductionV2()
    m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom size=".1"/></body></worldbody></mujoco>')
    d = mujoco.MjData(m)
    for _ in range(4000):
        mujoco.mj_step(m, d)
    world.close()
    if os.environ.get('FAKE_PAIR_FAIL'):
        raise RuntimeError('fake pair failure')
    Path(a.output).mkdir(parents=True)
    (Path(a.output)/'result.json').write_text(json.dumps({'seed': a.seed}))
'''


class PairProfSourceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.wt = Path(self.tmp.name)/'pair-wt'
        for rel, text in (('sim/__init__.py', ''), ('sim/multi_masterpi_production.py', FAKE_WORLD),
                          ('sim/masterpi_dynamics_v2.py', FAKE_DYNAMICS), ('scripts/run_m2_pair.py', FAKE_RUNNER)):
            (self.wt/rel).parent.mkdir(parents=True, exist_ok=True)
            (self.wt/rel).write_text(text)
        shutil.copy(ROOT/'sim'/'physics_drive_kernel.py', self.wt/'sim'/'physics_drive_kernel.py')
        git = ['git', '-C', str(self.wt), '-c', 'user.email=t@example.invalid', '-c', 'user.name=t']
        subprocess.run(['git', 'init', '-q', str(self.wt)], check=True)
        subprocess.run([*git, 'add', '-A'], check=True)
        subprocess.run([*git, 'commit', '-q', '-m', 'fake pair'], check=True)
        self.head = subprocess.run([*git, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def run_prof(self, mode: str, out: Path, **env) -> subprocess.CompletedProcess:
        return subprocess.run([sys.executable, str(PAIR_PROF), str(self.wt), mode, str(out), '701', '0',
                               '--speed-root', str(ROOT)], capture_output=True, text=True, timeout=300,
                              env={**os.environ, 'OMP_NUM_THREADS': '1', **env})

    def test_kernel_run_pins_injected_sources(self) -> None:
        out = Path(self.tmp.name)/'kernel'
        done = self.run_prof('kernel', out)
        self.assertEqual(done.returncode, 0, done.stderr[-2000:])
        prof = json.loads((out/'profile.json').read_text())
        injected = {row['module']: row for row in prof['injected_sources']}
        self.assertEqual(injected['kiro_sim_profile']['sha256'], sha(ROOT/'scripts'/'sim_profile.py'))
        self.assertEqual(injected['kiro_exact_speedups']['sha256'], sha(ROOT/'sim'/'exact_speedups.py'))
        self.assertEqual(injected['run_m2_pair_under_test']['sha256'], sha(self.wt/'scripts'/'run_m2_pair.py'))
        head = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True, text=True)
        self.assertEqual(prof['speed_source']['head'], head.stdout.strip())
        self.assertIsInstance(prof['speed_source']['dirty_paths'], list)
        self.assertEqual((prof['pair_worktree']['head'], prof['pair_worktree']['dirty_paths']), (self.head, []))
        self.assertEqual(prof['pair_worktree_head'], self.head)
        used = prof['pair_modules_used_by_kernel']['sim.physics_drive_kernel']
        self.assertEqual((Path(used['path']), used['sha256']),
                         ((self.wt/'sim'/'physics_drive_kernel.py').resolve(), sha(ROOT/'sim'/'physics_drive_kernel.py')))
        self.assertEqual(prof['kernel'], 'exact_drive_kernel' if exact_speedups.dot_order_matches()
                         else 'fallback_original_dot_order_mismatch')
        self.assertEqual((prof['error'], prof['mj_steps']), (None, 4000))
        self.assertEqual(len((out/'qpos_checkpoints.jsonl').read_text().splitlines()), 2)

    def test_failure_is_recorded_and_existing_output_kept(self) -> None:
        out = Path(self.tmp.name)/'none'
        done = self.run_prof('none', out, FAKE_PAIR_FAIL='1')
        self.assertEqual(done.returncode, 1, done.stderr[-2000:])
        prof = json.loads((out/'profile.json').read_text())
        self.assertEqual(prof['error'], 'RuntimeError: fake pair failure')
        self.assertNotIn('kiro_exact_speedups', {row['module'] for row in prof['injected_sources']})
        before = (out/'profile.json').read_bytes()
        again = self.run_prof('none', out)
        self.assertNotEqual(again.returncode, 0)
        self.assertIn('FileExistsError', again.stderr)
        self.assertEqual((out/'profile.json').read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
