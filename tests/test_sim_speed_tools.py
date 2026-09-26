"""Dev tools of the sim-speed work: the prefix slice hook and the equivalence checker (no sim needed)."""
from __future__ import annotations

import hashlib
import json
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
        return {'mode': 'tick', 'now': now}


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


def write_run(root: Path, frames: list[bytes], commands: list[dict], result: dict, skill_events=()) -> Path:
    (root/'inputs').mkdir(parents=True)
    (root/'frames').mkdir()
    (root/'eval_only').mkdir()
    rows = []
    for i, jpeg in enumerate(frames):
        (root/'frames'/f'{i:05d}.jpg').write_bytes(jpeg)
        rows.append({'frame': i, 't': .2*i, 'file': f'frames/{i:05d}.jpg', 'sha256': hashlib.sha256(jpeg).hexdigest()})
    (root/'inputs'/'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (root/'inputs'/'commands.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in commands))
    (root/'skill_events.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in skill_events))
    (root/'result.json').write_text(json.dumps(result))
    (root/'manifest.json').write_text(json.dumps({'wall_s': 1.0, 'code': {'sha': 'x'}}))
    return root


class EquivalenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.cmds = [{'t': .1*i, 'kind': 'hold'} for i in range(10)]

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_identical_runs(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'OK', 'sim_s': 1})
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds, {'outcome': 'OK', 'sim_s': 1})
        self.assertTrue(sim_equivalence.compare(a, b)['equivalent'])

    def test_frame_bytes_and_result_field_differences(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y'], self.cmds, {'outcome': 'OK', 'sim_s': 1})
        b = write_run(self.root/'b', [b'x', b'z'], self.cmds, {'outcome': 'OK', 'sim_s': 2})
        report = sim_equivalence.compare(a, b)
        self.assertFalse(report['equivalent'])
        self.assertEqual(report['checks']['frame_jpeg_bytes']['mismatched'], [1])
        self.assertEqual(report['checks']['result.json']['differing_fields'], ['sim_s'])

    def test_truncated_prefix_and_untimed_rows(self) -> None:
        a = write_run(self.root/'a', [b'x', b'y', b'q'], self.cmds, {'outcome': 'OK'}, skill_events=[{'k': 1}])
        b = write_run(self.root/'b', [b'x', b'y'], self.cmds[:5], {'outcome': 'SIM_LIMIT'})
        report = sim_equivalence.compare(a, b, until=.35)
        self.assertTrue(report['equivalent'], report)
        self.assertEqual(report['not_compared_untimed_rows'], ['skill_events.jsonl'])
        self.assertEqual(report['checks']['inputs/commands.jsonl']['len_a'], 4)


if __name__ == '__main__':
    unittest.main()
