#!/usr/bin/env python3
"""Source/model-frozen paired short transport validation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.run_camera_approach_student import sha, write


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases-json', type=Path, required=True)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--transport-model-dir', type=Path, required=True)
    p.add_argument('--stage-model-dir', type=Path)
    p.add_argument('--replay-teacher-dir', type=Path, required=True)
    p.add_argument('--conditions', nargs='+', choices=('visual', 'playback'), default=['visual', 'playback'])
    p.add_argument('--mjpython', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    args = p.parse_args()
    if git('status', '--porcelain'):
        raise RuntimeError('commit source/protocol before cohort')
    source = git('rev-parse', 'HEAD')
    cases = json.loads(args.cases_json.read_text())['cases']
    if not cases or len({c['case_id'] for c in cases}) != len(cases):
        raise ValueError('unique cases required')
    roots = [args.grasp_model_dir, args.transport_model_dir]
    if args.stage_model_dir:
        roots.append(args.stage_model_dir)
    model_hashes = {str(p.resolve()): sha(p) for r in roots for p in r.glob('*.json')}
    teacher_path = args.replay_teacher_dir / 'teacher-report.json'
    model_hashes[str(teacher_path.resolve())] = sha(teacher_path)
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'case-inputs').mkdir()
    summary = {'source_sha': source, 'cases_sha256': sha(args.cases_json), 'model_sha256': model_hashes,
               'cases': cases, 'conditions': args.conditions, 'complete': False, 'runs': [],
               'cost_usd': 0, 'external_model_calls': 0}
    write(out / 'cohort-report.json', summary)
    started = time.monotonic()
    for case in cases:
        first_images = None
        setup = out / 'case-inputs' / f"{case['case_id']}.json"
        write(setup, case)
        for condition in args.conditions:
            if git('rev-parse', 'HEAD') != source or git('status', '--porcelain'):
                raise RuntimeError('source changed during cohort')
            if any(sha(Path(path)) != digest for path, digest in model_hashes.items()):
                raise RuntimeError('model or playback source changed during cohort')
            name = f"{case['case_id']}-{condition}"
            run_out = out / name
            command = [str(args.mjpython), 'scripts/run_camera_short_transport_student.py',
                '--grasp-model-dir', str(args.grasp_model_dir), '--transport-model-dir', str(args.transport_model_dir),
                '--case-json', str(setup), '--condition', condition, '--replay-teacher-dir', str(args.replay_teacher_dir),
                '--out-dir', str(run_out)]
            if args.stage_model_dir:
                command += ['--stage-model-dir', str(args.stage_model_dir)]
            with (out / f'{name}.log').open('w') as log:
                completed = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            result_path = run_out / 'result.json'
            result = json.loads(result_path.read_text()) if result_path.exists() else {}
            row = {'case_id': case['case_id'], 'condition': condition, 'directory': str(run_out),
                   'returncode': completed.returncode, 'success': result.get('success', False),
                   'error': result.get('error', 'missing result' if not result else None),
                   'evaluation': result.get('evaluation'), 'wall_time_s': result.get('wall_time_s'),
                   'sim_time_s': result.get('sim_time_s'), 'carry_decisions': len(result.get('carry_calls', [])),
                   'result_sha256': sha(result_path) if result_path.exists() else None}
            images = {r: {k: v['sha256'] for k, v in rec.items()}
                      for r, rec in result.get('actor_initial_images', {}).items()}
            row['paired_initial_carry_rgb_equal'] = None if first_images is None else images == first_images
            if first_images is None:
                first_images = images
            summary['runs'].append(row)
            write(out / 'cohort-report.json', summary)
            print(json.dumps({k: row[k] for k in ('case_id', 'condition', 'success', 'returncode')}), flush=True)
    summary['complete'] = True
    summary['wall_time_s'] = time.monotonic() - started
    summary['scores'] = {c: {'success': sum(r['success'] for r in summary['runs'] if r['condition'] == c),
                              'total': sum(r['condition'] == c for r in summary['runs'])} for c in args.conditions}
    write(out / 'cohort-report.json', summary)
    print(json.dumps(summary['scores']), flush=True)


if __name__ == '__main__':
    main()
