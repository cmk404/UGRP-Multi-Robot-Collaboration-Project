"""Fixed known-map development/held-out matrices; source stays fixed per cohort."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def decision_budget(matrix, override=None):
    budget = override if override is not None else (400 if matrix == 'heading-comparison' else 240)
    if not 1 <= budget <= 400:
        raise ValueError('max_steps must be 1..400')
    return budget


def cases(matrix):
    if matrix == 'development':
        starts = [{'case_id': 'development_r1', 'robot_id': 'r1', 'start_xy_m': [-.5,-2.55], 'start_yaw_deg': 0}]
    else:
        starts = [
            {'case_id': 'heldout_r1', 'robot_id': 'r1', 'start_xy_m': [-.48,-2.50], 'start_yaw_deg': 12},
            {'case_id': 'heldout_r3', 'robot_id': 'r3', 'start_xy_m': [-.54,-2.59], 'start_yaw_deg': -12}]
    if matrix == 'heading-comparison':
        return [{'name': f'{s["case_id"]}-{m}-{style}', 'case': s, 'map': m,
                 'condition': 'map', 'motion_style': style}
                for s in starts for m in ('open', 'slalom', 'narrow')
                for style in ('heading', 'holonomic')]
    return [{'name': f'{s["case_id"]}-{m}-{c}', 'case': s, 'map': m, 'condition': c}
            for s in starts for m in ('open', 'slalom', 'narrow')
            for c in (('map', 'direct') if m != 'narrow' and matrix == 'heldout' else ('map',))]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix', choices=['development', 'heldout', 'heading-comparison'], required=True)
    p.add_argument('--python', required=True, help='mjpython on macOS; python on headless Linux')
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--only', choices=['open', 'slalom', 'narrow'])
    p.add_argument('--motion-style', choices=['heading', 'holonomic'], default='holonomic',
                   help='legacy cohort defaults to the original holonomic baseline')
    p.add_argument('--max-steps', type=int, help='default: 400 for heading-comparison, 240 for legacy cohorts')
    a = p.parse_args()
    a.max_steps = decision_budget(a.matrix, a.max_steps)
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True)
    if dirty.strip():
        raise RuntimeError('commit cohort source and protocol before running')
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    jobs = [j for j in cases(a.matrix) if a.only is None or j['map'] == a.only]
    a.out_dir.mkdir(parents=True, exist_ok=False)
    (a.out_dir / 'cohort.json').write_text(json.dumps({'source_sha': source, 'matrix': a.matrix, 'motion_style': a.motion_style, 'max_steps': a.max_steps, 'jobs': jobs}, indent=2)+'\n')
    results = []
    for job in jobs:
        out = a.out_dir / job['name']
        command = [a.python, '-m', 'scripts.run_known_map_navigation', '--map-file',
            str(ROOT / 'maps' / 'navigation' / (job['map'] + '.json')), '--case-json', json.dumps(job['case']),
            '--condition', job['condition'], '--motion-style', job.get('motion_style', a.motion_style),
            '--out-dir', str(out), '--max-steps', str(a.max_steps)]
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
