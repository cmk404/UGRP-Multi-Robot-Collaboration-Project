"""Build ACT rows from RGB decisions and actually issued actions.

Realtime renewal commands repeat an accepted decision while its next image is
pending. Verify the renewals, but never turn them into synthetic observations.
"""

import argparse
import bisect
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.pair_carry_act_contract import AXES, SCALES, context

TIME_TOLERANCE_S = 1e-6
ACTION_TOLERANCE = 1e-9


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def command_at(actions, when):
    matches = [i for i, item in enumerate(actions)
               if abs(item['issued_at_s'] - when) <= TIME_TOLERANCE_S]
    if len(matches) != 1:
        raise ValueError(f'expected one issued command at {when}, got {len(matches)}')
    return matches[0]


def axes(command):
    action = command['action']
    if action['kind'] != 'mecanum' or action['duration_s'] <= 0:
        raise ValueError('invalid issued transit command')
    return [action[k] for k in AXES]


def aligned_actions(rows, renewals, commands):
    """Match original decisions exactly and account for every transit renewal."""
    actions = [item for item in commands if item['stage'] == 'TRANSIT']
    if not actions:
        raise ValueError('missing transit commands')
    modern = bool(renewals)
    if modern and len(actions) != len(rows) + len(renewals):
        raise ValueError('realtime decision/renewal/command count mismatch')
    if not modern and len(actions) not in (len(rows) - 1, len(rows)):
        raise ValueError('decision/command count mismatch')
    rotating = rows[0].get('kind') == 'rotating_carry'
    used, matched = set(), []
    for i, row in enumerate(rows):
        if not modern and i == len(rows) - 1 and len(actions) == len(rows) - 1:
            matched.append(None)
            continue
        # The older rotating controller records per-slot actions in decision
        # order but did not promise command times equal decision times.
        index = i if rotating else command_at(actions, row['sim_time_s'])
        if index in used:
            raise ValueError('one command matched multiple decisions')
        used.add(index)
        matched.append(actions[index])
    decision_times = [row['sim_time_s'] for row in rows] if renewals else []
    if decision_times != sorted(decision_times):
        raise ValueError('carry decisions out of order')
    for renewal in renewals:
        if renewal['kind'] != 'carry_pending_renewal' or renewal['reason'] != 'same_accepted_cruise_while_next_rgb_pending':
            raise ValueError('unknown renewal source')
        index = command_at(actions, renewal['issued_at_s'])
        if index in used:
            raise ValueError('renewal reused decision command')
        used.add(index)
        issued = actions[index]
        source = bisect.bisect_right(decision_times, renewal['issued_at_s'] + TIME_TOLERANCE_S) - 1
        if source < 0 or source >= len(rows) - 1:
            raise ValueError('renewal outside accepted carry interval')
        original = matched[source]
        if renewal['source_frame_ids'] != rows[source]['frame_ids']:
            raise ValueError('renewal changed source observation')
        if any(abs(a - b) > ACTION_TOLERANCE for a, b in zip(axes(issued), axes(original))):
            raise ValueError('renewal changed accepted action')
        if abs(issued['action']['duration_s'] - renewal['duration_s']) > TIME_TOLERANCE_S:
            raise ValueError('renewal duration mismatch')
    if len(used) != len(actions):
        raise ValueError('transit command without observed decision or renewal')
    return matched, {'decision_commands': sum(item is not None for item in matched),
                     'renewal_commands': len(renewals), 'transit_commands': len(actions)}


def extract(root):
    root = root.resolve()
    result = read(root / 'result.json')
    if not result['physical_success'] or not result['protocol_complete']:
        raise ValueError('unsuccessful teacher ' + str(root))
    if result.get('carry_policy'):
        raise ValueError('student is not teacher')
    decisions = read(root / 'pair-decisions.json')
    commands = read(root / 'issued-commands.json')
    bindings = read(root / 'skill-bindings.json')['pair_model_slots']
    plan = read(root / 'committed-plan.json')['plan']
    static = read(root / 'episode-setup-only.json')['static_map']
    goal = static['docks'][plan['dock']]['slots']['beam']['center_m']
    route = next(t['route'] for t in plan['tasks'] if t['object'] == 'beam')
    image_bind = {item['frame_id']: item for item in decisions if item['kind'] == 'image_binding'}
    rows = [item for item in decisions if item['kind'] in ('carry', 'rotating_carry')]
    if len({item['kind'] for item in rows}) != 1:
        raise ValueError('mixed/no carry trajectory')
    renewals = [item for item in decisions if item['kind'] == 'carry_pending_renewal']
    if renewals and rows[0]['kind'] != 'carry':
        raise ValueError('renewal unsupported for rotating carry')
    sequences, image_hashes, alignment = {}, {}, {}
    for slot, rid in bindings.items():
        matched, alignment[slot] = aligned_actions(rows, renewals, commands[rid])
        previous, sequence = [0., 0., 0.], []
        for i, row in enumerate(rows):
            done = row['control']['done'] if row['kind'] == 'carry' else all(d['done'] for d in row['decisions'].values())
            if done != (i == len(rows) - 1):
                raise ValueError('only terminal arrival expected')
            bound = image_bind[row['frame_ids'][slot]]
            refs = {'own': bound['own'][slot], 'top': bound['raw_top']}
            issued = matched[i]
            target = [0., 0., 0.] if issued is None else axes(issued)
            if done and any(abs(v) > ACTION_TOLERANCE for v in target):
                raise ValueError('terminal decision issued motion')
            if not done and issued is None:
                raise ValueError('missing decision action')
            if row['kind'] == 'rotating_carry' and not done and any(
                    abs(target[j] - row['decisions'][slot]['action'][axis]) > ACTION_TOLERANCE
                    for j, axis in enumerate(AXES)):
                raise ValueError('rotating label mismatch')
            for ref in refs.values():
                path = (root / ref['path']).resolve()
                if not path.is_relative_to(root) or sha(path) != ref['sha256']:
                    raise ValueError('image provenance')
                image_hashes[ref['path']] = ref['sha256']
            sequence.append({'id': f'{root}:{slot}:{i}', 'images': refs,
                             'context': context(goal, route, slot, previous),
                             'action': [v / scale for v, scale in zip(target, SCALES)] + [float(done)],
                             'done': done, 'frame_id': row['frame_ids'][slot]})
            previous = target
        sequences[slot] = sequence
    files = ['result.json', 'pair-decisions.json', 'issued-commands.json',
             'skill-bindings.json', 'committed-plan.json', 'episode-setup-only.json']
    return {'root': str(root), 'source_sha': result['source_sha'],
            'files': {name: sha(root / name) for name in files},
            'image_hashes': image_hashes, 'evidence_alignment': alignment,
            'sequences': sequences}


def merge_base(result, base_path, expected_sha, heldout_train_roots=None):
    if sha(base_path) != expected_sha:
        raise ValueError('base dataset hash mismatch')
    base = read(base_path)
    if heldout_train_roots is None:
        old_train, old_development = base['train'], base['development']
    else:
        # An archived dataset can retain its training raw episodes while
        # having incomplete development images. Re-split a complete whole
        # training episode; the missing originals are never relabelled.
        targets = {root.resolve() for root in heldout_train_roots}
        old_development = [ep for ep in base['train'] if Path(ep['root']).resolve() in targets]
        if len(old_development) != len(targets):
            raise ValueError('expected all base train episodes for development')
        old_train = [ep for ep in base['train'] if ep not in old_development]
    merged = {'train': old_train + result['train'],
              'development': old_development + result['development']}
    roots = [entry['root'] for split in merged for entry in merged[split]]
    if len(roots) != len(set(roots)):
        raise ValueError('episode shared or duplicated across splits')
    hashes = {split: {digest for ep in merged[split] for digest in ep['image_hashes'].values()}
              for split in merged}
    if hashes['train'] & hashes['development']:
        raise ValueError('identical source images cross train/development split')
    for split in merged:
        for ep in merged[split]:
            root = Path(ep['root'])
            for rel, expected in ep['files'].items():
                if sha(root / rel) != expected:
                    raise ValueError('base episode file changed')
            for rel, expected in ep['image_hashes'].items():
                if sha(root / rel) != expected:
                    raise ValueError('base episode image changed')
    return merged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, nargs='+', required=True)
    parser.add_argument('--development', type=Path, nargs='+', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--base-dataset', type=Path)
    parser.add_argument('--base-dataset-sha256')
    parser.add_argument('--base-development-train-episodes', type=Path, nargs='+',
                        help='move these complete base training episodes to development')
    args = parser.parse_args()
    if bool(args.base_dataset) != bool(args.base_dataset_sha256):
        raise ValueError('base dataset and expected SHA must be supplied together')
    if args.base_development_train_episodes and not args.base_dataset:
        raise ValueError('base development selection needs a base dataset')
    if {root.resolve() for root in args.train} & {root.resolve() for root in args.development}:
        raise ValueError('split overlap')
    result = {key: [extract(root) for root in getattr(args, key)]
              for key in ('train', 'development')}
    if args.base_dataset:
        result = merge_base(result, args.base_dataset, args.base_dataset_sha256,
                            args.base_development_train_episodes)
        result['composition'] = {
            'base_dataset': str(args.base_dataset.resolve()),
            'base_dataset_sha256': args.base_dataset_sha256,
            'base_development_selection': (
                'whole training episode moved to development; original base development omitted'
                if args.base_development_train_episodes else 'original base development retained'),
            'moved_episodes': ([str(root.resolve()) for root in args.base_development_train_episodes]
                               if args.base_development_train_episodes else []),
        }
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: {'episodes': len(result[key]),
                            'rows': sum(len(seq) for ep in result[key] for seq in ep['sequences'].values())}
                      for key in ('train', 'development')}))


if __name__ == '__main__':
    main()
