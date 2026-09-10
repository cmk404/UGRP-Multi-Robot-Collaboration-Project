"""Exercise wire audit against forged history and peer-image substitution."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from harness.camera_pair_policy import CameraPairPlanner
from harness.gemini_proxy import _to_gemini_multi_image_messages
from scripts.audit_camera_pair_inputs import audit


def jpeg(value):
    ok, data = cv2.imencode('.jpg', np.full((24, 32, 3), value, dtype=np.uint8))
    assert ok
    return data.tobytes()


class AuditTests(unittest.TestCase):
    def make_run(self, root, mode):
        calls = []
        for rid, value in [('r1', 30), ('r3', 90)]:
            folder = root / rid
            folder.mkdir()
            planner = CameraPairPlanner(rid, None, task='grasp', mode=mode)
            for index in range(3):
                own, top = jpeg(value + index * 10), jpeg(150 + index * 10)
                (folder / f'{index:03d}-own.jpg').write_bytes(own)
                (folder / f'{index:03d}-overhead.jpg').write_bytes(top)
                request = planner.prepare_request(own, top)
                payload = {'model': 'test', 'temperature': 0.2, 'max_tokens': 600,
                           'reasoning_effort': 'none', 'messages': _to_gemini_multi_image_messages(
                               request['messages'], request['images'])}
                (folder / f'wire-{index + 1:03d}.json').write_text(json.dumps(payload))
                action = {'kind': 'arm', 'servo_id': 5, 'pulse': 1500 + index * 20}
                planner.record_action(action)
                calls.append({'round': index, 'robot_id': rid, 'action': action})
        (root / 'result.json').write_text(json.dumps({'config': {'mode': mode, 'task': 'grasp'}, 'calls': calls}))

    def test_all_modes_replay_and_evaluator_is_not_required(self):
        for mode in ('baseline', 'memory', 'temporal', 'learned'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                self.make_run(root, mode)
                (root / 'evaluation-only.jsonl').write_text('UNREADABLE TRUTH MUST NOT BE LOADED')
                result = audit(root)
                self.assertTrue(result['ok'])
                self.assertEqual(result['requests'], 6)

    def test_forged_history_text_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.make_run(root, 'learned')
            path = root / 'r1/wire-003.json'
            payload = json.loads(path.read_text())
            payload['messages'][-1]['content'][0]['text'] += '\ncontact=true, beam_xyz=[1,2,3]'
            path.write_text(json.dumps(payload))
            with self.assertRaises(AssertionError):
                audit(root)

    def test_other_robot_image_substitution_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.make_run(root, 'temporal')
            path = root / 'r1/wire-003.json'
            payload = json.loads(path.read_text())
            peer = json.loads((root / 'r3/wire-003.json').read_text())
            payload['messages'][-1]['content'][2] = peer['messages'][-1]['content'][2]
            path.write_text(json.dumps(payload))
            with self.assertRaises(AssertionError):
                audit(root)

    def test_changed_issued_command_invalidates_later_wire(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.make_run(root, 'memory')
            path = root / 'result.json'
            report = json.loads(path.read_text())
            report['calls'][0]['action']['pulse'] += 100
            path.write_text(json.dumps(report))
            with self.assertRaises(AssertionError):
                audit(root)


if __name__ == '__main__':
    unittest.main()
