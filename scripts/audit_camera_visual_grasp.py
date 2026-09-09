"""Replay raw RGB, model observations and own command feedback without referee data."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def audit(root):
    from harness.camera_visual_observer import CameraVisualObserver, parse_observation
    from harness.camera_grasp_controller import CameraGraspController, STARTUP_COMMANDS
    from harness.gemini_proxy import _to_gemini_multi_image_messages
    root = Path(root)
    report = json.loads((root / 'result.json').read_text())
    rows = []
    for rid in ('r1', 'r3'):
        assert report['startup_commands'] == list(STARTUP_COMMANDS)
        observer, controller = CameraVisualObserver(rid, None), CameraGraspController(STARTUP_COMMANDS)
        calls = [c for c in report['calls'] if c['robot_id'] == rid]
        for i, call in enumerate(calls):
            own = (root / rid / f'{call["round"]:03d}-own.jpg').read_bytes()
            top = (root / rid / f'{call["round"]:03d}-overhead.jpg').read_bytes()
            request = observer.prepare_request(own, top, controller.history)
            wire = json.loads((root / rid / f'wire-{i + 1:03d}.json').read_text())
            assert set(wire) == {'model', 'messages', 'temperature', 'max_tokens', 'reasoning_effort'}
            assert wire['messages'] == _to_gemini_multi_image_messages(request['messages'], request['images'])
            observation = observer.validate_response(call['response'], controller.history)
            assert observation == call['observation']
            scheduled = report['config']['active_robot']
            expected_active = scheduled == rid if scheduled != 'both' else ('r1', 'r3')[call['round'] % 2] == rid
            assert call['active'] == expected_active
            action = controller.step(observation, active=expected_active)
            assert action == call['action'], f'controller mismatch {rid}/{i}'
            assert json.loads(json.dumps(controller.last_decision)) == call['decision']
            rows.append({'robot_id': rid, 'round': call['round'],
                         'overhead_sha256': hashlib.sha256(top).hexdigest(),
                         'images': len(request['images'])})
    assert rows, 'no calls audited'
    for row in rows:
        peers = [r for r in rows if r['round'] == row['round']]
        assert len({r['overhead_sha256'] for r in peers}) == 1
    return {'ok': True, 'requests': len(rows), 'audit': rows,
            'checked': 'exact request and action replay from camera bytes, model response and own issued commands; no evaluator read'}


if __name__ == '__main__':
    root = Path(sys.argv[1])
    result = audit(root)
    (root / 'input-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'audit'}))
