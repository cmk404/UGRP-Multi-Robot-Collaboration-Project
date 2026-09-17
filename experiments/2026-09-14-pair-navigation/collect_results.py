#!/usr/bin/env python3
"""Posthoc record collection only; never imported by a robot or experiment."""
import csv
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
RAW = ROOT / 'outputs/pair-navigation'
EXECUTION_SHA = '13d833964335b04766cc8c7c8b72ddc5b70f4192'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def main():
    runs = []
    for folder in sorted(RAW.iterdir()):
        if not (folder / 'result.json').exists():
            continue
        source = json.loads((folder / 'result.json').read_text())
        final = source['source_sha'] == EXECUTION_SHA
        report = {k: v for k, v in source.items() if k not in ('steps', 'sync_events')}
        report['recorded_success'] = source.get('success', source.get('evaluation', {}).get('success'))
        report['case'] = folder.name
        report['cohort'] = 'final' if final else ('previous_final_candidate' if folder.name.startswith('final-') else 'development')
        report['raw_directory'] = str(folder.relative_to(ROOT))
        report['decision_rounds'] = len(source.get('steps', []))
        report['decision_robot_commands'] = 2 * report['decision_rounds']
        report['decision_command_scope'] = 'Both robots, including decision-loop zero commands; excludes grasp, post-arrival stop and placement'
        if source.get('steps'):
            report['final_actor_status'] = {r: d['status'] for r, d in source['steps'][-1]['decisions'].items()}
        if (folder / 'input-audit.json').exists():
            audit = json.loads((folder / 'input-audit.json').read_text())
            report['input_audit'] = {k: v for k, v in audit.items() if k != 'grasp'}
            report['grasp_input_audit_ok'] = audit['grasp']['ok']
        if (folder / 'diagnostic-audit.json').exists():
            report['diagnostic_audit'] = json.loads((folder / 'diagnostic-audit.json').read_text())
        if (folder / 'grasp-result.json').exists():
            report['grasp_prediction_calls'] = len(json.loads((folder / 'grasp-result.json').read_text())['calls'])
        if (folder / 'scene.xml').exists():
            report['archived_scene_xml_sha256'] = sha(folder / 'scene.xml')
        runs.append(report)
    assert len(runs) == 19 and sum(r['cohort'] == 'final' for r in runs) == 6
    write(DEST / 'results.json', {
        'execution_sha': EXECUTION_SHA,
        'final_scope': 'Three feasible fixed maps, one impossible map, one default-physics negative comparison, one hold/release diagnostic; one run each',
        'historical_scoring': 'Development and previous-final results retain their contemporaneous gates. In particular dev-wall-impratio10 passed older carry gates but failed later pre-departure stability review. Do not pool success rates.',
        'external_model_calls_all_runs': 0,
        'external_model_cost_usd_all_runs': 0,
        'runs': runs,
    })
    with (DEST / 'final-results.csv').open('w', newline='') as stream:
        fields = ['case', 'impratio', 'physical_success', 'refused_no_route', 'diagnostic_hold_success', 'air_release_success', 'hold_seconds', 'decision_rounds', 'decision_robot_commands', 'navigation_sim_seconds', 'wall_seconds', 'goal_position_error_m', 'goal_yaw_error_deg', 'min_lift_m', 'source_sha']
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for r in runs:
            if r['cohort'] != 'final':
                continue
            e = r.get('evaluation', {})
            row = {k: r.get(k, e.get(k)) for k in fields}
            row.update(impratio=r.get('contact_impratio', r.get('impratio')),
                       physical_success=r.get('evaluation', {}).get('success'),
                       refused_no_route=r.get('refused_no_route', False))
            if r.get('diagnostic_only'):
                row.update(diagnostic_hold_success=r['recorded_success'],
                           air_release_success=r.get('air_release_success'),
                           hold_seconds=r.get('seconds'),
                           min_lift_m=r.get('diagnostic_audit', {}).get('min_hold_lift_m'))
            writer.writerow(row)
    records, directories = [], []
    for folder in sorted(RAW.iterdir()):
        if not folder.is_dir() or folder.name == 'runtime':
            continue
        entries = []
        for path in sorted(folder.rglob('*')):
            if path.is_file():
                assert not path.is_symlink(), path
                item = {'path': str(path.relative_to(RAW)), 'bytes': path.stat().st_size, 'sha256': sha(path)}
                entries.append(item)
                records.append(item)
        encoded = ''.join(json.dumps(x, sort_keys=True) + '\n' for x in entries).encode()
        directories.append({'directory': folder.name, 'files': len(entries), 'bytes': sum(x['bytes'] for x in entries), 'ordered_file_index_sha256': hashlib.sha256(encoded).hexdigest()})
    index = DEST / 'raw-hashes.jsonl.gz'
    index.write_bytes(gzip.compress(''.join(json.dumps(x, sort_keys=True) + '\n' for x in records).encode(), mtime=0))
    write(DEST / 'raw-manifest.json', {
        'local_raw_root': str(RAW), 'raw_remote_backup': False,
        'raw_hash_index': {'path': index.name, 'sha256': sha(index), 'files': len(records), 'bytes': sum(x['bytes'] for x in records)},
        'directories': directories,
        'runtime_exclusion': 'Extracted grasp model duplicates the already-versioned models.zip; archive hash and original manifest verify it.',
        'model_archive': {'path': 'experiments/2026-09-10-rgb-varied-start/models.zip', 'sha256': sha(ROOT / 'experiments/2026-09-10-rgb-varied-start/models.zip')},
        'tracked_review_media': {'path': 'media/pair-navigation-demo.mp4', 'sha256': sha(DEST / 'media/pair-navigation-demo.mp4'), 'scope': 'Edited three-success review video only; raw RGB/logs/movies remain local'},
    })
    print(json.dumps({'runs': len(runs), 'raw_files': len(records), 'raw_bytes': sum(x['bytes'] for x in records)}))


if __name__ == '__main__':
    main()
