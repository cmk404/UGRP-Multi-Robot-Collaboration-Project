#!/usr/bin/env python3
"""Run a frozen, paired local grasp cohort and audit every saved RGB decision."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.audit_camera_grasp_student import audit
from scripts.collect_grasp_recovery_teacher import load_cases
from scripts.run_camera_pair_transport import evaluate_grasp_samples


def write(path, value):
    path.write_text(json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def compare_initial_rgb(first_root, first_images, current_root, current_images):
    """Allow tiny raster/JPEG noise only; physical initial state is checked separately."""
    metrics = {}
    for rid in first_images:
        for view in ('own', 'top'):
            a = first_images[rid][view]
            b = current_images[rid][view]
            if a['sha256'] == b['sha256']:
                metrics[f'{rid}/{view}'] = {'byte_exact': True, 'max_abs': 0, 'mean_abs': 0.0}
                continue
            left = cv2.imread(str(first_root / a['path']), cv2.IMREAD_COLOR)
            right = cv2.imread(str(current_root / b['path']), cv2.IMREAD_COLOR)
            if left is None or right is None:
                raise ValueError('paired initial RGB cannot be decoded')
            left, right = left.astype(np.int16), right.astype(np.int16)
            if left.shape != right.shape:
                raise ValueError('paired initial RGB shape differs')
            delta = np.abs(left - right)
            maximum, mean = int(delta.max()), float(delta.mean())
            if maximum > 3 or mean > 0.001:
                raise ValueError(f'paired initial RGB differs: {rid}/{view}: max={maximum}, mean={mean}')
            metrics[f'{rid}/{view}'] = {'byte_exact': False, 'max_abs': maximum, 'mean_abs': mean}
    return metrics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases-json', type=Path, required=True)
    p.add_argument('--model-dir', type=Path, required=True)
    p.add_argument('--baseline-model-dir', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--mjpython', type=Path, required=True)
    p.add_argument('--conditions', nargs='+', choices=['recovery', 'linear', 'playback'],
                   default=['recovery', 'linear', 'playback'])
    args = p.parse_args()
    out, cases_path = args.out_dir.resolve(), args.cases_json.resolve()
    candidate, baseline = args.model_dir.resolve(), args.baseline_model_dir.resolve()
    if out.exists():
        raise FileExistsError(out)
    if len(set(args.conditions)) != len(args.conditions):
        raise ValueError('duplicate conditions')
    if git('status', '--porcelain=v1'):
        raise ValueError('commit source and experiment protocol before cohort')
    source = git('rev-parse', 'HEAD')
    cases = load_cases(cases_path)
    skills = [json.loads((root / 'student-skill.json').read_text()) for root in (candidate, baseline)]
    for key in ('initialization_replay', 'close_pulses', 'lift_delta_pulses',
                'close_duration_s', 'close_settle_s', 'lift_duration_s', 'lift_settle_s', 'hold_s'):
        if skills[0][key] != skills[1][key]:
            raise ValueError(f'unmatched demonstration stage: {key}')
    if sha(candidate / 'evaluation-fixture.json') != sha(baseline / 'evaluation-fixture.json'):
        raise ValueError('unmatched evaluation fixtures')
    out.mkdir(parents=True)
    started = time.monotonic()
    rows = []
    summary = {'source_sha': source, 'cases_path': str(cases_path), 'cases_sha256': sha(cases_path),
               'model_dir': str(candidate), 'baseline_model_dir': str(baseline),
               'conditions': args.conditions, 'runs': rows, 'complete': False,
               'actor_boundary': 'own RGB, fixed shared top RGB, own issued commands; privileged evaluation output only'}
    write(out / 'cohort-report.json', summary)
    for case in cases:
        first_images = None
        first_time_range = None
        first_state = None
        first_destination = None
        for condition in args.conditions:
            if git('rev-parse', 'HEAD') != source or git('status', '--porcelain=v1'):
                raise RuntimeError('source changed during frozen cohort')
            models = candidate if condition == 'recovery' else baseline
            destination = out / f"{case['id']}-{condition}"
            command = [str(args.mjpython.resolve()), 'scripts/run_camera_grasp_student.py',
                       '--model-dir', str(models), '--out-dir', str(destination),
                       '--condition', 'playback' if condition == 'playback' else 'visual',
                       '--perturb', *map(str, case['perturb']['r1']),
                       '--perturb-r3', *map(str, case['perturb']['r3']),
                       '--rounds', '16', '--max-step', '25']
            completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            (out / f"{case['id']}-{condition}.stdout.txt").write_text(completed.stdout)
            (out / f"{case['id']}-{condition}.stderr.txt").write_text(completed.stderr)
            row = {'case_id': case['id'], 'condition': condition, 'command': command,
                   'raw_dir': str(destination), 'exit_code': completed.returncode, 'ok': False}
            rows.append(row)
            if completed.returncode:
                row['error'] = 'student runner failed; see saved stderr/result'
                write(out / 'cohort-report.json', summary)
                raise RuntimeError(f"runner failed: {destination}")
            report = json.loads((destination / 'result.json').read_text())
            replay = audit(destination, models)
            write(destination / 'audit.json', replay)
            evaluation_samples = [json.loads(line) for line in
                                  (destination / 'evaluation-only.jsonl').read_text().splitlines()]
            evaluation = evaluate_grasp_samples(evaluation_samples)
            if evaluation != report['evaluation'] or report['error'] is not None:
                raise ValueError('physical scoring replay differs from result')
            initial_images = {call['robot_id']: call['images'] for call in report['calls'][:2]}
            sim_range = [evaluation_samples[0]['sim_time_s'], evaluation_samples[-1]['sim_time_s']]
            if first_images is None:
                first_images, first_time_range = initial_images, sim_range
                first_state = report['evaluation_initial_state']
                first_destination = destination
            if report['evaluation_initial_state'] != first_state or sim_range != first_time_range:
                raise ValueError(f"unmatched paired initial physical state/timing: {case['id']}")
            image_match = compare_initial_rgb(first_destination, first_images, destination, initial_images)
            if report['applied_perturbation'] != case['perturb']:
                raise ValueError('requested perturbation clipped or changed')
            if report['source_sha'] != source:
                raise ValueError('runner used another source')
            row.update({'ok': True, 'evaluation': evaluation, 'audit_calls': replay['calls'],
                        'matched_initial_rgb': True, 'initial_rgb_comparison': image_match,
                        'matched_initial_physical_state': True, 'sim_time_range': sim_range,
                        'action_count': sum(len(c['actions']) for c in report['calls']),
                        'wall_elapsed_s': report['wall_elapsed_s'],
                        'result_sha256': sha(destination / 'result.json'),
                        'initial_decisions': {c['robot_id']: c['decision'] for c in report['calls'][:2]},
                        'final_decisions': report['final_visual_errors']})
            write(out / 'cohort-report.json', summary)
            print(json.dumps({'case': case['id'], 'condition': condition,
                              'success': evaluation['grasp_success'],
                              'hold_s': evaluation['longest_qualifying_duration_s'],
                              'lift_m': evaluation['max_lift_m'], 'audit_calls': replay['calls']}), flush=True)
    summary.update({'complete': True, 'wall_elapsed_s': time.monotonic() - started,
                    'success_counts': {condition: sum(row['evaluation']['grasp_success'] for row in rows
                                                      if row['condition'] == condition)
                                       for condition in args.conditions},
                    'case_count': len(cases)})
    write(out / 'cohort-report.json', summary)
    print(json.dumps({key: summary[key] for key in ('complete', 'case_count', 'success_counts', 'wall_elapsed_s')}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
