import hashlib
import json
import copy
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
    put('issued-commands.json', {rid: [{'stage': 'TRANSIT', 'issued_at_s': row['sim_time_s'],
        'action': row['actions'][slot]} for row in calls]
        for slot, rid in [('r1', 'physical-a'), ('r3', 'physical-b')]})
    return calls


@pytest.mark.parametrize('temporal', [False, True])
def test_reconstructs_every_request_without_worker_or_simulator(tmp_path, temporal):
    result = audit(tmp_path, fixture(tmp_path, temporal))
    assert result['passed'] and result['requests'] == 6
    assert result['history'] == (4 if temporal else 1)
    assert audit(tmp_path, []) is None


@pytest.mark.parametrize('damage', ['wire', 'image', 'history', 'context', 'binding', 'issued'])
def test_changed_inputs_cannot_be_admitted(tmp_path, damage):
    calls = fixture(tmp_path); inp = calls[-1]['inputs']['r1']
    if damage == 'wire': inp['wire_sha256'] = 'wrong'
    elif damage == 'image': (tmp_path/inp['images']['own']['path']).write_bytes(b'wrong')
    elif damage == 'history': inp['history'] = list(reversed(inp['history']))
    elif damage == 'context': inp['context'] = [0.]*8
    elif damage == 'binding': inp['physical_robot_id'] = 'other'
    else:
        path = tmp_path/'issued-commands.json'; issued = json.loads(path.read_text())
        issued['physical-a'][0]['action']['forward'] = .5; path.write_text(json.dumps(issued))
    with pytest.raises(ValueError): audit(tmp_path, calls)


def test_stale_retry_reconstructed_but_not_added_to_history_or_previous(tmp_path):
    calls = fixture(tmp_path)
    stale = copy.deepcopy(calls[1])
    stale.update(kind='act_stale_prediction', observed_at_s=.1,
                 received_at_s=.19, frame_id=101)
    stale.pop('sim_time_s')
    stale.pop('actions')
    for slot, inp in stale['inputs'].items():
        images = {}
        for name in ('own', 'top'):
            payload = f'stale-{slot}-{name}'.encode()
            filename = f'stale-{slot}-{name}.jpg'
            (tmp_path/filename).write_bytes(payload)
            images[name] = {'path': filename,
                            'sha256': hashlib.sha256(payload).hexdigest()}
        inp.update(images=images, frame_id=101, observed_at_s=.1)
        inp['history'][-1].update(images=images, sim_time_s=.1, frame_id=101)
        frames = [{'own_rgb': (tmp_path/f['images']['own']['path']).read_bytes(),
                   'top_rgb': (tmp_path/f['images']['top']['path']).read_bytes(),
                   'context': f['context']} for f in inp['history']]
        payload = json.dumps(wire_request(frames, 4)) + '\n'
        inp['wire_sha256'] = hashlib.sha256(payload.encode()).hexdigest()
    stale['decisions'] = {'r1': {'done': False}, 'r3': {'done': False}}
    result = audit(tmp_path, [calls[0], stale, calls[1], calls[2]])
    assert result['requests'] == 8
    assert result['accepted_decisions'] == 3
    assert result['stale_predictions'] == 1
    stale['inputs']['r1']['history'][-1]['frame_id'] = 999
    with pytest.raises(ValueError, match='history'):
        audit(tmp_path, [calls[0], stale, calls[1], calls[2]])


def test_capture_timestamp_is_distinct_from_decision_time(tmp_path):
    calls = fixture(tmp_path)
    row = calls[1]
    for inp in row['inputs'].values():
        inp['observed_at_s'] = .15
        inp['frame_id'] = 1
        inp['history'][-1]['sim_time_s'] = .15
    assert audit(tmp_path, calls)['passed']
    for inp in row['inputs'].values():
        inp['observed_at_s'] = .21
    with pytest.raises(ValueError, match='capture time'):
        audit(tmp_path, calls)


def test_pair_capture_provenance_must_be_coherent(tmp_path):
    calls = fixture(tmp_path)
    for inp in calls[1]['inputs'].values():
        inp['observed_at_s'] = .15
        inp['frame_id'] = 1
        inp['history'][-1]['sim_time_s'] = .15
    calls[1]['inputs']['r3']['frame_id'] = 2
    with pytest.raises(ValueError, match='not coherent'):
        audit(tmp_path, calls)


def test_issue_receipt_requires_actual_pair_commands(tmp_path):
    calls = fixture(tmp_path)
    receipt = {'kind': 'act_issue', 'index': 0, 'issued_at_s': 0.,
               'actions': calls[0]['actions']}
    assert audit(tmp_path, [calls[0], receipt, *calls[1:]])['passed']
    issued_path = tmp_path/'issued-commands.json'
    issued = json.loads(issued_path.read_text())
    issued['physical-b'].pop(0)
    issued_path.write_text(json.dumps(issued))
    with pytest.raises(ValueError, match='no issued commands'):
        audit(tmp_path, [calls[0], receipt, *calls[1:]])


def test_partial_inference_error_attempt_is_audited_without_adoption(tmp_path):
    calls = fixture(tmp_path)
    error = copy.deepcopy(calls[1])
    error.update(kind='act_inference_error', frame_id=1,
                 observed_at_s=.1, received_at_s=.2,
                 error='RuntimeError: worker exited')
    error.pop('sim_time_s')
    error.pop('actions')
    error['decisions'] = {'r1': {'done': False}}
    for slot, inp in error['inputs'].items():
        inp.update(frame_id=1, observed_at_s=.1,
                   response_received=slot=='r1')
        inp['history'][-1]['sim_time_s'] = .1
    result = audit(tmp_path, [calls[0], error])
    assert result['requests'] == 4
    assert result['completed_responses'] == 3
    assert result['accepted_decisions'] == 1
    assert result['inference_error_rows'] == 1
    error['inputs']['r3'].pop('wire_sha256')
    with pytest.raises(ValueError, match='wire hash'):
        audit(tmp_path, [calls[0], error])


def owner_abort(calls, known):
    row = copy.deepcopy(calls[1])
    row.update(kind='act_inference_error', frame_id=1, observed_at_s=.1,
               received_at_s=.2, discard_reason='owner_aborted',
               owner_error='RuntimeError: box lost', worker_completion='timeout',
               slot_failures={}, unconfirmed_slots=[slot for slot in ('r1', 'r3')
                                                    if slot not in known])
    row.pop('sim_time_s')
    row.pop('actions')
    row['inputs'] = {slot: row['inputs'][slot] for slot in known}
    row['decisions'] = {slot: {'done': False} for slot in known}
    for inp in row['inputs'].values():
        inp.update(frame_id=1, observed_at_s=.1, response_received=True)
        inp['history'][-1]['sim_time_s'] = .1
    return row


@pytest.mark.parametrize('known', [('r1',), ()])
def test_owner_abort_keeps_known_wire_and_marks_unknown_incomplete(tmp_path, known):
    calls = fixture(tmp_path)
    row = owner_abort(calls, known)
    result = audit(tmp_path, [calls[0], row])
    assert result['known_inputs_passed'] is True
    assert result['passed'] is False and result['complete'] is False
    assert result['requests'] == 2 + len(known)
    assert result['unconfirmed_slots'] == 2 - len(known)
    assert result['accepted_decisions'] == 1
    assert result['history'] == 4


def test_owner_abort_unknown_slots_and_known_wire_must_be_honest(tmp_path):
    calls = fixture(tmp_path)
    row = owner_abort(calls, ('r1',))
    row['inputs']['r1'].pop('wire_sha256')
    with pytest.raises(ValueError, match='wire hash'):
        audit(tmp_path, [calls[0], row])
    row = owner_abort(calls, ('r1',))
    row['unconfirmed_slots'] = ['r1']
    with pytest.raises(ValueError, match='unconfirmed slot'):
        audit(tmp_path, [calls[0], row])
    row = owner_abort(calls, ('r1',))
    row['unconfirmed_slots'] = []
    with pytest.raises(ValueError, match='aborted prediction accounting'):
        audit(tmp_path, [calls[0], row])


def test_aborted_prediction_does_not_advance_later_history(tmp_path):
    calls = fixture(tmp_path)
    row = owner_abort(calls, ('r1',))
    row['discard_reason'] = 'prediction_raised'
    result = audit(tmp_path, [calls[0], row, *calls[1:]])
    assert result['requests'] == 7
    assert result['accepted_decisions'] == 3
    assert result['unconfirmed_slots'] == 1
    assert result['passed'] is False


def test_only_unconfirmed_abort_is_incomplete_not_zero_request_proof(tmp_path):
    calls = fixture(tmp_path)
    row = owner_abort(calls, ())
    row['index'] = 0
    result = audit(tmp_path, [row])
    assert result['requests'] == 0
    assert result['unconfirmed_slots'] == 2
    assert result['schema'] == 'unknown'
    assert result['passed'] is False
