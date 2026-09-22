"""Read-only reconstruction of saved ACT requests; no inference or simulation."""
import hashlib
import json
from pathlib import Path

from harness.carry_input_history import window_indices, wire_request
from harness.pair_carry_act_contract import AXES, context


def audit(root, decisions):
    root = Path(root).resolve()
    calls = [row for row in decisions if row.get('kind') == 'act_carry']
    if not calls:
        return None
    setup = json.loads((root/'episode-setup-only.json').read_text())
    plan = json.loads((root/'committed-plan.json').read_text())['plan']
    bindings = json.loads((root/'skill-bindings.json').read_text())['pair_model_slots']
    commands = json.loads((root/'issued-commands.json').read_text())
    issued = {slot: {round(c['issued_at_s'], 6): c for c in commands[rid]
                     if c['stage'] == 'TRANSIT'} for slot, rid in bindings.items()}
    goal = setup['static_map']['docks'][plan['dock']]['slots']['beam']['center_m']
    route = next(task['route'] for task in plan['tasks'] if task['object'] == 'beam')
    past = {slot: [] for slot in bindings}
    previous = {slot: [0., 0., 0.] for slot in bindings}
    images = {}; references = requests = 0; length = None
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

    for index, row in enumerate(calls):
        require(row['index'] == index and set(row['inputs']) == set(bindings), 'ACT decision sequence mismatch')
        for slot, inp in row['inputs'].items():
            require(inp['physical_robot_id'] == bindings[slot], 'ACT robot binding mismatch')
            require(inp['context'] == context(goal, route, slot, previous[slot]), 'ACT issued-command context mismatch')
            require(('history' in inp) == temporal, 'ACT request schema changed')
            if temporal:
                history = inp['history']
                if length is None:
                    length = len(history)
                require(length in (1, 4) and len(history) == length, 'ACT history length mismatch')
                current = {'images': inp['images'], 'context': inp['context'],
                           'sim_time_s': row['sim_time_s'], 'frame_id': history[-1]['frame_id']}
                past[slot].append(current)
                expected = [past[slot][i] for i in window_indices(index, length)]
                require(history == expected, 'ACT history is not the causal robot-local window')
            else:
                length = 1
                expected = [inp]
            frames = [{'own_rgb': image(f['images']['own']), 'top_rgb': image(f['images']['top']),
                       'context': f['context']} for f in expected]
            request = wire_request(frames, length)
            payload = json.dumps(request if temporal else request['frames'][0]) + '\n'
            require(hashlib.sha256(payload.encode()).hexdigest() == inp['wire_sha256'], 'ACT wire hash mismatch')
            if index < len(calls)-1:
                command = issued[slot].get(round(row['sim_time_s'], 6), {}).get('action', {})
                require(all(command.get(axis) == row['actions'][slot][axis] for axis in AXES),
                        'ACT history action differs from issued command')
            previous[slot] = [row['actions'][slot][axis] for axis in AXES]
            requests += 1
    return {'passed': True, 'requests': requests, 'history': length,
            'schema': 'temporal' if temporal else 'single-frame',
            'image_references': references, 'unique_images': len(images),
            'scope': 'All saved ACT requests reconstructed from raw images, causal per-robot history and authored/issued context. No model inference replay or physical rerun.'}
