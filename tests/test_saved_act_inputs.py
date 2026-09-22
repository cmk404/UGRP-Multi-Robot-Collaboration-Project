import hashlib
import json
import pytest

from harness.carry_input_history import window_indices, wire_request
from harness.pair_carry_act_contract import AXES, context
from scripts.audit_saved_act_inputs import audit


def fixture(root, temporal=True):
    def put(name, data):
        (root/name).write_text(json.dumps(data))
    put('episode-setup-only.json', {'static_map': {'docks': {'dock': {'slots': {'beam': {'center_m': [1., 1.]}}}}}})
    put('committed-plan.json', {'plan': {'dock': 'dock', 'tasks': [{'object': 'beam', 'route': 'north'}]}})
    put('skill-bindings.json', {'pair_model_slots': {'r1': 'physical-a', 'r3': 'physical-b'}})
    past = {'r1': [], 'r3': []}; calls = []
    for index in range(3):
        row = {'kind': 'act_carry', 'index': index, 'sim_time_s': index*.2, 'inputs': {}, 'actions': {}}
        for slot, physical in [('r1', 'physical-a'), ('r3', 'physical-b')]:
            images = {}
            for name in ['own', 'top']:
                data = f'{index}-{slot}-{name}'.encode(); filename = f'{index}-{slot}-{name}.jpg'
                (root/filename).write_bytes(data)
                images[name] = {'path': filename, 'sha256': hashlib.sha256(data).hexdigest()}
            ctx = context([1., 1.], 'north', slot, [.01, 0., 0.] if index else [0., 0., 0.])
            current = {'images': images, 'context': ctx, 'sim_time_s': row['sim_time_s'], 'frame_id': index}
            past[slot].append(current)
            history = [past[slot][i] for i in window_indices(index, 4)] if temporal else [current]
            frames = [{'own_rgb': (root/f['images']['own']['path']).read_bytes(),
                       'top_rgb': (root/f['images']['top']['path']).read_bytes(), 'context': f['context']} for f in history]
            wire = wire_request(frames, 4 if temporal else 1)
            payload = json.dumps(wire if temporal else wire['frames'][0])+'\n'
            inp = {'images': images, 'context': ctx, 'physical_robot_id': physical,
                   'wire_sha256': hashlib.sha256(payload.encode()).hexdigest()}
            if temporal:
                inp['history'] = history
            row['inputs'][slot] = inp
            row['actions'][slot] = dict(zip(AXES, [.01, 0., 0.]))
        calls.append(row)
    return calls


@pytest.mark.parametrize('temporal', [False, True])
def test_reconstructs_every_request_without_worker_or_simulator(tmp_path, temporal):
    result = audit(tmp_path, fixture(tmp_path, temporal))
    assert result['passed'] and result['requests'] == 6
    assert result['history'] == (4 if temporal else 1)
    assert audit(tmp_path, []) is None


@pytest.mark.parametrize('damage', ['wire', 'image', 'history', 'context', 'binding'])
def test_changed_inputs_cannot_be_admitted(tmp_path, damage):
    calls = fixture(tmp_path); inp = calls[-1]['inputs']['r1']
    if damage == 'wire': inp['wire_sha256'] = 'wrong'
    elif damage == 'image': (tmp_path/inp['images']['own']['path']).write_bytes(b'wrong')
    elif damage == 'history': inp['history'] = list(reversed(inp['history']))
    elif damage == 'context': inp['context'] = [0.]*8
    else: inp['physical_robot_id'] = 'other'
    with pytest.raises(ValueError): audit(tmp_path, calls)
