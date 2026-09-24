"""Read-only reconstruction of saved ACT requests; no inference or simulation."""
import hashlib
import json
import math
from pathlib import Path

from harness.carry_input_history import window_indices, wire_request
from harness.pair_carry_act_contract import AXES, context


def audit(root, decisions):
    root = Path(root).resolve()
    calls = [row for row in decisions
             if row.get('kind') in ('act_carry', 'act_stale_prediction', 'act_inference_error')]
    if not calls:
        return None
    setup = json.loads((root/'episode-setup-only.json').read_text())
    plan = json.loads((root/'committed-plan.json').read_text())['plan']
    bindings = json.loads((root/'skill-bindings.json').read_text())['pair_model_slots']
    commands = json.loads((root/'issued-commands.json').read_text())
    issued = {slot: {round(c['issued_at_s'], 6): c for c in commands[rid]
                     if c.get('stage') == 'TRANSIT' and 'issued_at_s' in c}
              for slot, rid in bindings.items()}
    receipts = {row['index']: row for row in decisions if row.get('kind') == 'act_issue'}
    goal = setup['static_map']['docks'][plan['dock']]['slots']['beam']['center_m']
    route = next(task['route'] for task in plan['tasks'] if task['object'] == 'beam')
    past = {slot: [] for slot in bindings}
    previous = {slot: [0., 0., 0.] for slot in bindings}
    images = {}; references = requests = completed = accepted = stale = errors = 0; length = None
    temporal = 'history' in next(iter(calls[0]['inputs'].values()))

    def require(ok, message):
        if not ok:
            raise ValueError(message)

    def image(ref):
        nonlocal references
        path = (root/ref['path']).resolve()
        require(path.is_relative_to(root), 'ACT image outside trial')
        if path not in images:
            data = path.read_bytes()
            images[path] = (data, hashlib.sha256(data).hexdigest())
        require(images[path][1] == ref['sha256'], 'ACT image hash mismatch')
        references += 1
        return images[path][0]

    for row in calls:
        accepted_row = row['kind'] == 'act_carry'
        error_row = row['kind'] == 'act_inference_error'
        index = accepted
        slots = set(row['inputs'])
        require(row['index'] == index and slots
                and (slots <= set(bindings) if error_row else slots == set(bindings)),
                'ACT decision sequence mismatch')
        if error_row:
            completed_slots = {slot for slot, inp in row['inputs'].items()
                               if inp.get('response_received') is True}
            require(all(type(inp.get('response_received')) is bool for inp in row['inputs'].values())
                    and set(row.get('decisions', {})) == completed_slots,
                    'ACT inference error response receipt mismatch')
        captures = {(inp.get('frame_id'), inp.get('observed_at_s'))
                    for inp in row['inputs'].values() if 'observed_at_s' in inp}
        require(len(captures) <= 1, 'ACT pair capture is not coherent')
        if not accepted_row:
            observed_at = row.get('observed_at_s', math.inf)
            received_at = row.get('received_at_s', -math.inf)
            require(received_at >= observed_at
                    and (error_row or received_at > observed_at),
                    'discarded ACT prediction chronology mismatch')
        for slot, inp in row['inputs'].items():
            require(inp['physical_robot_id'] == bindings[slot], 'ACT robot binding mismatch')
            require(inp['context'] == context(goal, route, slot, previous[slot]), 'ACT issued-command context mismatch')
            require(('history' in inp) == temporal, 'ACT request schema changed')
            observed = inp.get('observed_at_s')
            if not accepted_row:
                require('frame_id' in inp and type(observed) in (int, float),
                        'discarded ACT request missing capture provenance')
            if observed is not None:
                require(type(observed) in (int, float) and math.isfinite(observed)
                        and observed <= row.get('received_at_s', row.get('sim_time_s', math.inf)) + 1e-9,
                        'ACT capture time mismatch')
                if not accepted_row:
                    require(inp['frame_id'] == row['frame_id']
                            and observed == row['observed_at_s'],
                            'discarded ACT frame provenance mismatch')
            if temporal:
                history = inp['history']
                if length is None:
                    length = len(history)
                require(length in (1, 4) and len(history) == length, 'ACT history length mismatch')
                current = {'images': inp['images'], 'context': inp['context'],
                           'sim_time_s': observed if observed is not None else row['sim_time_s'],
                           'frame_id': inp.get('frame_id', history[-1]['frame_id'])}
                candidate = past[slot] + [current]
                expected = [candidate[i] for i in window_indices(index, length)]
                require(history == expected, 'ACT history is not the causal robot-local window')
                if accepted_row:
                    past[slot].append(current)
            else:
                length = 1
                expected = [inp]
            frames = [{'own_rgb': image(f['images']['own']), 'top_rgb': image(f['images']['top']),
                       'context': f['context']} for f in expected]
            request = wire_request(frames, length)
            payload = json.dumps(request if temporal else request['frames'][0]) + '\n'
            require(isinstance(inp.get('wire_sha256'), str)
                    and len(inp['wire_sha256']) == 64,
                    'ACT attempted request lacks a verifiable wire hash')
            require(hashlib.sha256(payload.encode()).hexdigest() == inp['wire_sha256'], 'ACT wire hash mismatch')
            requests += 1
            if not error_row or inp['response_received']:
                completed += 1
        if accepted_row:
            receipt = receipts.get(index)
            if receipt is not None:
                require(set(receipt.get('actions', {})) == set(bindings), 'ACT issue receipt slots mismatch')
                issue_at = receipt['issued_at_s']
                require(issue_at >= row['sim_time_s'] - 1e-9, 'ACT issue precedes decision')
            else:
                issue_at = row['sim_time_s']
            issued_pair = {slot: issued[slot].get(round(issue_at, 6)) for slot in bindings}
            if receipt is not None:
                require(all(issued_pair.values()), 'ACT issue receipt has no issued commands')
            if all(issued_pair.values()):
                for slot, command in issued_pair.items():
                    action = command.get('action', {})
                    require(all(action.get(axis) == row['actions'][slot][axis] for axis in AXES),
                            'ACT action differs from issued command')
                    if receipt is not None:
                        require(all(action.get(axis) == receipt['actions'][slot][axis] for axis in AXES),
                                'ACT issue receipt differs from issued command')
                    previous[slot] = [action[axis] for axis in AXES]
            else:
                require(not any(issued_pair.values()), 'partial ACT pair issue')
            accepted += 1
        else:
            if error_row:
                errors += 1
            else:
                stale += 1
    return {'passed': True, 'requests': requests, 'accepted_decisions': accepted,
            'completed_responses': completed, 'stale_predictions': stale,
            'inference_error_rows': errors, 'history': length,
            'schema': 'temporal' if temporal else 'single-frame',
            'image_references': references, 'unique_images': len(images),
            'scope': 'Accepted, stale and inference-error ACT attempts reconstructed from raw images, causal per-robot history and authored/actually-issued context. Discarded histories and actions never advance state. No model inference replay or physical rerun.'}
