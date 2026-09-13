"""Fixed known-map development/held-out matrices; source stays fixed per cohort."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def cases(matrix):
    if matrix == 'development':
        starts = [{'case_id': 'development_r1', 'robot_id': 'r1', 'start_xy_m': [-.5,-2.55], 'start_yaw_deg': 0}]
    else:
        starts = [
            {'case_id': 'heldout_r1', 'robot_id': 'r1', 'start_xy_m': [-.48,-2.50], 'start_yaw_deg': 12},
            {'case_id': 'heldout_r3', 'robot_id': 'r3', 'start_xy_m': [-.54,-2.59], 'start_yaw_deg': -12}]
    return [{'name': f'{s["case_id"]}-{m}-{c}', 'case': s, 'map': m, 'condition': c}
            for s in starts for m in ('open', 'slalom', 'narrow')
            for c in (('map', 'direct') if m != 'narrow' and matrix == 'heldout' else ('map',))]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix', choices=['development', 'heldout'], required=True)
    p.add_argument('--python', required=True, help='mjpython on macOS; python on headless Linux')
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--only', choices=['open', 'slalom', 'narrow'])
    a = p.parse_args()
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True)
    if dirty.strip():
        raise RuntimeError('commit cohort source and protocol before running')
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    jobs = [j for j in cases(a.matrix) if a.only is None or j['map'] == a.only]
    a.out_dir.mkdir(parents=True, exist_ok=False)
    (a.out_dir / 'cohort.json').write_text(json.dumps({'source_sha': source, 'matrix': a.matrix, 'jobs': jobs}, indent=2)+'\n')
    results = []
    for job in jobs:
        out = a.out_dir / job['name']
        command = [a.python, '-m', 'scripts.run_known_map_navigation', '--map-file',
            str(ROOT / 'maps' / 'navigation' / (job['map'] + '.json')), '--case-json', json.dumps(job['case']),
            '--condition', job['condition'], '--out-dir', str(out), '--max-steps', '240']
        with (a.out_dir / (job['name'] + '.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        summary = {'name': job['name'], 'exit_code': result.returncode}
        if (out / 'result.json').exists():
            summary.update(json.loads((out / 'result.json').read_text()))
        else:
            # Some GUI Python launchers return zero even after a child traceback.
            summary.update(actor_status='runner_failed', error='No result.json; inspect retained process log')
        results.append(summary)
        (a.out_dir / 'results.json').write_text(json.dumps(results, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        current = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True)
        if current != source or dirty.strip():
            raise RuntimeError('cohort source changed; do not continue')


if __name__ == '__main__':
    main()
