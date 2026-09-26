"""Dev tools of the sim-speed work: the prefix slice hook, SIM limit restore and the equivalence checker (no sim)."""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import sim_equivalence, sim_profile  # noqa: E402


class FakeSkill:
    phase = None


class FakeController:
    def __init__(self):
        self.phase, self.skill, self.calls = 'search_leg', None, 0

    def decide(self, now):
        self.calls += 1
        if now >= 3:
            self.phase, self.skill = 'skill', FakeSkill()
            self.skill.phase = 'to_carry_posture'
        return {'mode': 'tick', 'now': now}


def runner_loop(runner, ctl, end: float = 10.) -> tuple[str, float]:
    """The shape of run_m1_owncam's decision loop: the module-level SIM_LIMIT_S is read before each decision."""
    now = 0.
    while True:
        if now > runner.SIM_LIMIT_S:
            return 'SIM_LIMIT', now
        if now >= end:
            return 'DONE', now
        ctl.decide(now)
        now += 1.


class PhaseStopTests(unittest.TestCase):
    def test_stops_only_at_target_and_restores(self) -> None:
        rec = sim_profile.Recorder()
        runner = types.SimpleNamespace(SIM_LIMIT_S=720.)
        sim_profile.install_phase_stop(rec, runner, FakeController, 'skill:to_carry_posture')
        ctl = FakeController()
        self.assertEqual(ctl.decide(1.0), {'mode': 'tick', 'now': 1.0})     # return value untouched
        ctl.phase, ctl.skill = 'skill', FakeSkill()
        ctl.skill.phase = 'grasp'
        ctl.decide(2.0)
        self.assertEqual(runner.SIM_LIMIT_S, 720.)
        ctl.skill.phase = 'to_carry_posture'
        ctl.decide(3.5)
        self.assertEqual(runner.SIM_LIMIT_S, -1.)
        self.assertEqual(rec.slice_stop, {'target': 'skill:to_carry_posture', 'sim_t': 3.5,
                                          'phase': 'skill:to_carry_posture'})
        self.assertEqual(ctl.calls, 3)
        rec.restore()
        self.assertFalse(hasattr(FakeController.decide, '__wrapped__'))
        self.assertEqual(runner.SIM_LIMIT_S, 720.)                              # Codex review #4: was left at -1

    def test_later_full_run_in_the_same_process_is_unaffected(self) -> None:
        runner = types.SimpleNamespace(SIM_LIMIT_S=720.)
        rec = sim_profile.Recorder()
        sim_profile.install_phase_stop(rec, runner, FakeController, 'skill:to_carry_posture')
        self.assertEqual(runner_loop(runner, FakeController()), ('SIM_LIMIT', 4.))    # the slice
        rec.restore()
        self.assertEqual(runner_loop(runner, FakeController()), ('DONE', 10.))        # the later full run

    def test_restore_without_a_stop_keeps_the_limit(self) -> None:
        runner = types.SimpleNamespace(SIM_LIMIT_S=0.)            # 0 must stay 0, not become a default
        rec = sim_profile.Recorder()
        sim_profile.install_phase_stop(rec, runner, FakeController, 'skill:never')
        runner_loop(runner, FakeController())
        rec.restore()
        self.assertEqual(runner.SIM_LIMIT_S, 0.)

    def test_m1_prepare_truncation_is_restored(self) -> None:
        from scripts import run_m1_owncam as runner
        before = runner.SIM_LIMIT_S
        rec = sim_profile.Recorder()
        args = argparse.Namespace(prereg=sim_profile.M1_PREREG, episode='m1dev-s93', sim_limit=50., sections=False,
                                  stop_at_phase='skill:to_carry_posture', speedups=None)
        try:
            sim_profile.m1_prepare(rec, args)
            self.assertEqual(runner.SIM_LIMIT_S, 50.)
        finally:
            rec.restore()
        self.assertEqual(runner.SIM_LIMIT_S, before)
        from harness.m1_owncam_delivery import M1OwnCamDelivery
        self.assertFalse(hasattr(M1OwnCamDelivery.decide, '__wrapped__'))

    def test_m1_prepare_rejects_non_dev_episode(self) -> None:
        rec = sim_profile.Recorder()
        args = argparse.Namespace(prereg=sim_profile.M1_PREREG, episode='no-such-episode', sim_limit=0.,
                                  sections=False, stop_at_phase='', speedups=None)
        with self.assertRaises(SystemExit):
            sim_profile.m1_prepare(rec, args)

    def test_cli_rejects_bad_numbers(self) -> None:
        for extra in (['--sim-limit', 'nan'], ['--sim-limit', '-1'], ['--sim-limit', 'inf'],
                      ['--cpu-mark-sim-s', 'nan'], ['--qpos-every', '-3']):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                sim_profile.parse_args(['m1', '--episode', 'm1dev-s93', '--output', '/nonexistent/x', *extra])
        args = sim_profile.parse_args(['m1', '--episode', 'm1dev-s93', '--output', '/x', '--sim-limit', '0'])
        self.assertEqual(args.sim_limit, 0.)


def write_run(root: Path, frames: list[bytes], commands: list[dict], result: dict, skill_events=(),
              frame_dt: float = .2) -> Path:
    (root/'inputs').mkdir(parents=True)
    (root/'frames').mkdir()
    (root/'eval_only').mkdir()
    rows = []
    for i, jpeg in enumerate(frames):
        (root/'frames'/f'{i:05d}.jpg').write_bytes(jpeg)
        rows.append({'frame': i, 't': frame_dt*i, 'file': f'frames/{i:05d}.jpg',
                     'sha256': hashlib.sha256(jpeg).hexdigest()})
    (root/'inputs'/'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (root/'inputs'/'commands.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in commands))
    (root/'skill_events.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in skill_events))
    (root/'result.json').write_text(json.dumps(result))
    (root/'manifest.json').write_text(json.dumps({'wall_s': 1.0, 'code': {'sha': 'x'}}))
    return root


def write_profile(root: Path, checkpoints: list[dict], *run_args, **run_kw) -> Path:
    write_run(root/'run', *run_args, **run_kw)
    (root/'run'/'attempt_started.json').write_text('{}')
    (root/'qpos_checkpoints.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in checkpoints))
    return root


class EquivalenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.cmds = [{'t': .1*i, 'kind': 'hold'} for i in range(10)]

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def assertInsufficient(self, report: dict, needle: str) -> None:
        self.assertFalse(report['equivalent'], report)
        self.assertEqual(report['verdict'], 'insufficient_evidence')
        self.assertTrue(any(needle in e for e in report['evidence_errors']), report['evidence_errors'])

    def test_identical_runs(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'OK', 'sim_s': 1})
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds, {'outcome': 'OK', 'sim_s': 1})
        report = sim_equivalence.compare(a, b)
        self.assertTrue(report['equivalent'], report)
        self.assertEqual(report['verdict'], 'equivalent')
        self.assertEqual(report['checks']['frame_jpeg_bytes']['compared'], 2)
        self.assertEqual(report['not_compared'], ['qpos_checkpoints: neither side is a sim_profile directory'])

    def test_frame_bytes_and_result_field_differences(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'OK', 'sim_s': 1})
        b = write_run(self.root/'b', [b'x', b'z'], self.cmds, {'outcome': 'OK', 'sim_s': 2})
        report = sim_equivalence.compare(a, b)
        self.assertFalse(report['equivalent'])
        self.assertEqual(report['verdict'], 'different')
        self.assertEqual(report['checks']['frame_jpeg_bytes']['mismatched'], [1])
        self.assertEqual(report['checks']['result.json']['differing_fields'], ['sim_s'])

    def test_truncated_prefix_and_untimed_rows(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y', b'q'], self.cmds, {'outcome': 'OK'}, skill_events=[{'k': 1}])
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds[:5], {'outcome': 'SIM_LIMIT'})
        report = sim_equivalence.compare(a, b, until=.15)        # B's frames run to 0.2 s: the window is reached
        self.assertTrue(report['equivalent'], report)
        self.assertEqual(report['not_compared_untimed_rows'], ['skill_events.jsonl'])
        self.assertEqual(report['checks']['inputs/commands.jsonl']['len_a'], 2)
        self.assertEqual(report['last_frame_sim_s'], {'A': .4, 'B': .2})

    # ---------------------------------------------------- Codex review #1: no evidence never passes
    def test_two_missing_paths_are_not_equivalent(self) -> None:
        report = sim_equivalence.compare(self.root/'nope-a', self.root/'nope-b', until=120)
        self.assertInsufficient(report, 'is not a directory')
        with contextlib.redirect_stdout(io.StringIO()):
            code = sim_equivalence.main([str(self.root/'nope-a'), str(self.root/'nope-b'), '--until-sim-s', '120'])
        self.assertEqual(code, sim_equivalence.EXIT_INSUFFICIENT)

    def test_empty_directories_are_not_equivalent(self) -> None:
        (self.root/'a').mkdir()
        (self.root/'b').mkdir()
        for until in (None, 120.):
            with self.subTest(until=until):
                self.assertInsufficient(sim_equivalence.compare(self.root/'a', self.root/'b', until=until),
                                        'missing')

    def test_missing_frame_logs_on_both_sides(self) -> None:
        a = write_run(self.root/'a', [b'x'], self.cmds, {'outcome': 'OK'})
        b = write_run(self.root/'b', [b'x'], self.cmds, {'outcome': 'OK'})
        for run in (a, b):
            (run/'inputs'/'frames.jsonl').unlink()
        report = sim_equivalence.compare(a, b)
        self.assertInsufficient(report, 'frames.jsonl')
        self.assertEqual(report['checks']['frame_jpeg_bytes']['compared'], 0)

    def test_empty_logs_on_both_sides(self) -> None:
        a = write_run(self.root/'a', [], [], {'outcome': 'OK'})
        b = write_run(self.root/'b', [], [], {'outcome': 'OK'})
        self.assertInsufficient(sim_equivalence.compare(a, b), 'no rows to compare')

    def test_window_not_reached(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'SIM_LIMIT'})
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds, {'outcome': 'SIM_LIMIT'})
        report = sim_equivalence.compare(a, b, until=120)     # both runs end at 0.2 s
        self.assertInsufficient(report, 'before the compared window')

    def test_bad_until_values(self) -> None:
        a = write_run(self.root/'a', [b'x'], self.cmds, {'outcome': 'OK'})
        for until in (0, 0., -1., math.nan, math.inf, True, '120'):
            with self.subTest(until=until), self.assertRaises(ValueError):
                sim_equivalence.compare(a, a, until=until)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            sim_equivalence.main([str(a), str(a), '--until-sim-s', 'nan'])

    def test_corrupted_rows(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'OK'})
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds, {'outcome': 'OK'})
        with (b/'inputs'/'commands.jsonl').open('a') as fh:
            fh.write('{"t": 1.0, "kind": \n')
        self.assertInsufficient(sim_equivalence.compare(a, b), 'is not JSON')
        (b/'inputs'/'commands.jsonl').write_text('[1, 2]\n')
        self.assertInsufficient(sim_equivalence.compare(a, b), 'not a JSON object')
        (b/'result.json').write_text('{')
        self.assertInsufficient(sim_equivalence.compare(a, b), 'result.json is not JSON')

    def test_missing_or_corrupted_jpeg(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'OK'})
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds, {'outcome': 'OK'})
        (b/'frames'/'00001.jpg').unlink()
        self.assertInsufficient(sim_equivalence.compare(a, b), 'missing frame file')
        (b/'frames'/'00001.jpg').write_bytes(b'')             # truncated frame: disagrees with its own log row
        report = sim_equivalence.compare(a, b)
        self.assertEqual(report['verdict'], 'different')
        self.assertEqual(report['checks']['frame_jpeg_bytes']['mismatched'], [1])

    def test_untimed_required_rows_cannot_be_cut(self) -> None:
        cmds = [{'kind': 'hold'}] * 3
        a = write_run(self.root/'a', [b'x', b'y'], cmds, {'outcome': 'OK'})
        b = write_run(self.root/'b', [b'x', b'y'], cmds, {'outcome': 'OK'})
        self.assertInsufficient(sim_equivalence.compare(a, b, until=.1), 'rows without SIM time')

    def test_profile_checkpoints_required_and_nonempty(self) -> None:
        cps = [{'step': 2000, 't': .1, 'sha256': 'h1'}, {'step': 4000, 't': .5, 'sha256': 'h2'}]
        a = write_profile(self.root/'a', cps, [b'x', b'y'], self.cmds, {'outcome': 'OK'})
        b = write_profile(self.root/'b', cps, [b'x', b'y'], self.cmds, {'outcome': 'OK'})
        report = sim_equivalence.compare(a, b)
        self.assertTrue(report['equivalent'], report)
        self.assertEqual(report['checks']['qpos_checkpoints']['len_a'], 2)
        (b/'qpos_checkpoints.jsonl').write_text('')
        self.assertInsufficient(sim_equivalence.compare(a, b), 'qpos checkpoints')
        (b/'qpos_checkpoints.jsonl').unlink()
        self.assertInsufficient(sim_equivalence.compare(a, b), 'missing')
        c = write_run(self.root/'c', [b'x', b'y'], self.cmds, {'outcome': 'OK'})     # a plain run directory
        report = sim_equivalence.compare(a, c)
        self.assertEqual(report['not_compared'], ['qpos_checkpoints: only A is a sim_profile directory'])
        self.assertEqual(report['verdict'], 'equivalent')

    def test_exit_codes(self) -> None:
        a = write_run(self.root/'a', [b'x'], self.cmds, {'outcome': 'OK'})
        b = write_run(self.root/'b', [b'z'], self.cmds, {'outcome': 'OK'})
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(sim_equivalence.main([str(a), str(a)]), sim_equivalence.EXIT_EQUIVALENT)
            self.assertEqual(sim_equivalence.main([str(a), str(b)]), sim_equivalence.EXIT_DIFFERENT)


if __name__ == '__main__':
    unittest.main()
