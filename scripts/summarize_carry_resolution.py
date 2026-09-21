"""Rank audited screening conditions without treating failures as a usable optimum."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rank(rows, arms, seeds, cases):
    expected = {(f"{a['id']}-s{s}", c['id']) for a in arms for s in seeds for c in cases}
    actual = [(r['condition'], r['case']) for r in rows]
    if len(actual) != len(set(actual)) or set(actual) != expected:
        raise ValueError('missing, duplicate, or unexpected screening rows')
    if not all(r.get('completed_report') for r in rows):
        raise ValueError('missing execution evidence')
    result = []
    for arm in arms:
        selected = [r for r in rows if r['condition'] in {f"{arm['id']}-s{s}" for s in seeds}]
        seed_successes = {str(s): sum(bool(r['whole_success']) for r in selected
                                     if r['condition'] == f"{arm['id']}-s{s}") for s in seeds}
        latencies = [r['inference_wall_s']['p95'] for r in selected if r.get('inference_wall_s')]
        result.append({'arm': arm, 'runs': len(selected), 'whole_successes_by_seed': seed_successes,
                       'minimum_seed_successes': min(seed_successes.values()),
                       'whole_successes': sum(seed_successes.values()),
                       'beam_successes': sum(bool(r['beam_success']) for r in selected),
                       'false_done_trials': sum(bool(r.get('false_done')) for r in selected),
                       'obstacle_contact_trials': sum(r.get('obstacle_contact_steps', 0) > 0 for r in selected),
                       'median_trial_p95_inference_s': statistics.median(latencies) if latencies else None,
                       'inference_trials': len(latencies),
                       'total_commands': sum(r.get('commands', 0) for r in selected),
                       'median_wall_s': statistics.median(r['wall_s'] for r in selected)})
    result.sort(key=lambda r: (-r['minimum_seed_successes'], -r['whole_successes'], -r['beam_successes'],
                              r['false_done_trials'], r['obstacle_contact_trials'],
                              r['median_trial_p95_inference_s'] if r['median_trial_p95_inference_s'] is not None else float('inf'),
                              r['arm']['size'], r['arm']['history']))
    return {'ranking': result,
            'selected_arm': result[0]['arm'] if result[0]['whole_successes'] else None,
            'decision': 'fresh_confirmation_required' if result[0]['whole_successes'] else 'no_usable_optimum',
            'scope': 'Best among the screened resolution/history conditions, not a global optimum or held-out success claim.'}


def combine(directories, selection):
    reports = [read(d/'report.json') for d in directories]
    audits = [read(d/'audit.json') for d in directories]
    first = reports[0]['protocol']
    arms, rows, teachers, provenance = [], [], [], []
    for directory, report, audit in zip(directories, reports, audits):
        protocol = report['protocol']
        if not report['complete'] or not audit['passed'] or report['source_sha'] != audit['source_sha']:
            raise ValueError('cohort is incomplete or unaudited')
        if sha(directory/'dataset.json') != protocol['dataset_sha256']:
            raise ValueError('dataset changed')
        for field in ('dataset_sha256', 'training', 'controls', 'test'):
            if protocol[field] != first[field]:
                raise ValueError('unmatched screening protocol: '+field)
        expected_models = {f"{a['id']}-s{s}" for a in protocol['arms'] for s in protocol['training']['seeds']}
        freeze = read(directory/'final-freeze.json')
        if (set(freeze['models']) != expected_models or freeze['source_sha'] != report['source_sha']
                or freeze['protocol_sha256'] != report['protocol_sha256']):
            raise ValueError('model freeze differs from executed protocol')
        for model in freeze['models'].values():
            if sha(Path(model['path'])/'model.safetensors') != model['sha256']:
                raise ValueError('audited model changed')
        expected_rows = {(condition, c['id']) for condition in {'teacher', *expected_models} for c in protocol['test']}
        actual_rows = [(r['condition'], r['case']) for r in audit['rows']]
        if len(actual_rows) != len(set(actual_rows)) or set(actual_rows) != expected_rows:
            raise ValueError('incomplete audit coverage')
        if not all(r.get('completed_report') for r in audit['rows']):
            raise ValueError('missing physical trial evidence')
        if audit['initial_states_matched'] != len(protocol['test']):
            raise ValueError('initial state audit incomplete')
        # Recheck raw hashes so an old passing audit cannot bless edited evidence.
        for row in audit['rows']:
            for name, digest in row['raw_hashes'].items():
                if sha(Path(row['raw'])/name) != digest:
                    raise ValueError('audited raw file changed')
        arms.extend(protocol['arms'])
        rows.extend(r for r in audit['rows'] if r['condition'] != 'teacher')
        teacher_rows = {r['case']: r for r in audit['rows'] if r['condition'] == 'teacher'}
        if teachers:
            for case, row in teacher_rows.items():
                old = teachers[0][case]
                for name in ('episode-setup-only.json', 'committed-plan.json'):
                    before, after = read(Path(old['raw'])/name), read(Path(row['raw'])/name)
                    if name == 'committed-plan.json':
                        before, after = before['plan'], after['plan']
                    if before != after:
                        raise ValueError('teacher setup/plan differs between cohorts')
                before = read(Path(old['raw'])/'result.json')['evaluation']
                after = read(Path(row['raw'])/'result.json')['evaluation']
                if before != after or old['whole_success'] != row['whole_success']:
                    raise ValueError('teacher physics differs between cohorts; inspect before pooling')
        teachers.append(teacher_rows)
        provenance.append({'directory': str(directory.resolve()), 'source_sha': report['source_sha'],
                           'report_sha256': sha(directory/'report.json'), 'audit_sha256': sha(directory/'audit.json')})
    expected_arms = {(size, history) for size in selection['candidate_resolutions'] for history in selection['candidate_histories']}
    if len(arms) != len(expected_arms) or {(a['size'], a['history']) for a in arms} != expected_arms:
        raise ValueError('candidate grid is incomplete or duplicated')
    if first['training']['seeds'] != selection['seeds'] or first['test'] != selection['screening_cases']:
        raise ValueError('selection declaration differs from executed screen')
    result = rank(rows, arms, selection['seeds'], selection['screening_cases'])
    result.update(provenance=provenance, teacher_physics_matched=True)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cohort', type=Path, action='append', required=True)
    p.add_argument('--selection', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--confirmation-protocol', type=Path)
    a = p.parse_args()
    selection = read(a.selection)
    result = combine(a.cohort, selection)
    result['selection_sha256'] = sha(a.selection)
    a.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    if a.confirmation_protocol and result['selected_arm']:
        protocol = read(a.cohort[0]/'report.json')['protocol']
        baseline = {'id': 'r128-h1', 'size': 128, 'history': 1}
        chosen = result['selected_arm']
        protocol.update(arms=[baseline] if baseline == chosen else [baseline, chosen],
                        test=selection['confirmation']['cases'],
                        scope='Fresh-offset confirmation after resolution/history screening; no further tuning on these cases')
        a.confirmation_protocol.write_text(json.dumps(protocol, indent=2)+'\n')
    print(json.dumps({'decision': result['decision'], 'selected_arm': result['selected_arm']}))


if __name__ == '__main__':
    main()
