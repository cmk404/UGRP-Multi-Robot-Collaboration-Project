"""Replay allowed RGB/own-action derivations and verify exact recorded wire inputs."""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def audit(root):
    from harness.camera_pair_policy import CameraPairPlanner
    from harness.gemini_proxy import _to_gemini_multi_image_messages

    root = Path(root)
    metadata = root / 'result.json'
    if not metadata.exists():
        metadata = root / 'progress.json'
    report = json.loads(metadata.read_text())
    config = report.get('config', {})
    task, mode = config.get('task', 'carry'), config.get('mode', 'baseline')
    calls = {}
    for call in report.get('calls', []):
        key = (call['robot_id'], call['round'])
        assert key not in calls, 'duplicate action record'
        calls[key] = call
    result = []
    for rid in ('r1', 'r3'):
        # Rebuild solely from this robot's saved input RGB and issued commands.
        # No evaluator, simulator state or peer commands are loaded here.
        planner = CameraPairPlanner(rid, None, task=task, mode=mode)
        paths = sorted((root / rid).glob('wire-*.json'))
        for index, path in enumerate(paths):
            assert int(path.stem.split('-')[1]) == index + 1, 'missing request in sequence'
            payload = json.loads(path.read_text())
            assert set(payload) == {'model', 'messages', 'temperature', 'max_tokens', 'reasoning_effort'}
            own = (root / rid / f'{index:03d}-own.jpg').read_bytes()
            overhead = (root / rid / f'{index:03d}-overhead.jpg').read_bytes()
            expected = planner.prepare_request(own, overhead)
            expected_messages = _to_gemini_multi_image_messages(expected['messages'], expected['images'])
            assert payload['messages'] == expected_messages, f'input derivation mismatch: {path}'
            result.append({'robot_id': rid, 'round': index, 'wire': str(path.relative_to(root)),
                           'own': hashlib.sha256(own).hexdigest(),
                           'overhead': hashlib.sha256(overhead).hexdigest(),
                           'image_count': len(expected['images'])})
            call = calls.get((rid, index))
            if call is not None:
                planner.record_action(call['action'])
            else:
                assert index == len(paths) - 1, 'missing prior issued action'
    assert result, 'no audited requests'
    for row in result:
        peer = next((x for x in result if x['robot_id'] != row['robot_id'] and x['round'] == row['round']), None)
        if peer:
            assert peer['overhead'] == row['overhead'], 'different shared overhead'
    return {'ok': True, 'mode': mode, 'requests': len(result), 'audit': result,
            'checked': 'exact wire replay from original own/shared RGB and own issued actions only; '
                       'prior RGB and pixel-derived learning rebuilt deterministically per mode; '
                       'shared overhead matched; evaluator never read'}


if __name__ == '__main__':
    root = Path(sys.argv[1]).resolve()
    result = audit(root)
    (root / 'input-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'audit'}))
