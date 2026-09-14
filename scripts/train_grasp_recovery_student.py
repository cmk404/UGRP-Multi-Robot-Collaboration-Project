#!/usr/bin/env python3
"""Fit nonlinear image-only corrections from successful privileged recoveries."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.camera_recovery_student import fit_recovery_model


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path: Path, value) -> None:
    path.write_text(json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n')


def rgb(root: Path, record: dict) -> bytes:
    path = (root / record['path']).resolve()
    if not path.is_relative_to(root) or sha(path) != record['sha256']:
        raise ValueError('teacher RGB path/hash mismatch')
    return path.read_bytes()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--teacher-dir', type=Path, required=True)
    parser.add_argument('--base-model-dir', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    teacher, base, out = (p.resolve() for p in (args.teacher_dir, args.base_model_dir, args.out_dir))
    if out.exists():
        raise FileExistsError(out)
    report = json.loads((teacher / 'report.json').read_text())
    actors = json.loads((teacher / 'actor_samples.json').read_text())
    label_rows = json.loads((teacher / 'privileged_labels.json').read_text())
    if len({x['sample_id'] for x in actors}) != len(actors):
        raise ValueError('duplicate teacher sample ids')
    labels = {x['sample_id']: x for x in label_rows}
    if len(labels) != len(label_rows) or set(labels) != {x['sample_id'] for x in actors}:
        raise ValueError('teacher label/sample pairing mismatch')
    successful = {x['id'] for x in report['cases'] if x['ok']}
    if not successful:
        raise ValueError('no physically successful expert recovery cases')
    # Verify all samples, including excluded failures, before fitting any model.
    loaded = {x['sample_id']: (rgb(teacher, x['observations']['own_rgb']),
                              rgb(teacher, x['observations']['shared_top_rgb'])) for x in actors}
    skill = json.loads((base / 'student-skill.json').read_text())
    started = time.monotonic()
    out.mkdir(parents=True)
    fit = {}
    records = {}
    for rid in ('r1', 'r3'):
        rows = [x for x in actors if x['robot_id'] == rid and x['case_id'] in successful]
        samples = [{'own_jpeg': loaded[x['sample_id']][0],
                    'top_jpeg': loaded[x['sample_id']][1],
                    'case_id': x['case_id'],
                    'correction_pulses': labels[x['sample_id']]['correction_pulses']} for x in rows]
        own_goal = (base / f'{rid}-goal-own.jpg').read_bytes()
        top_goal = (base / f'{rid}-goal-top.jpg').read_bytes()
        model = fit_recovery_model(own_goal, top_goal, samples)
        path = out / f'{rid}-model.json'
        write(path, model)
        records[rid] = {'path': path.name, 'sha256': sha(path)}
        fit[rid] = {'sample_count': len(samples), 'sample_ids': [x['sample_id'] for x in rows],
                    'diagnostics': model.get('diagnostics', {})}
        for view in ('own', 'top'):
            shutil.copy2(base / f'{rid}-goal-{view}.jpg', out / f'{rid}-goal-{view}.jpg')
    skill['models'] = records
    skill['scope'] = ('teacher-initialized local pregrasp; supervised nonlinear RGB recovery; '
                      'demonstrated close/lift playback; no navigation')
    write(out / 'student-skill.json', skill)
    shutil.copy2(base / 'evaluation-fixture.json', out / 'evaluation-fixture.json')
    training = {
        'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'teacher_dir': str(teacher), 'base_model_dir': str(base),
        'teacher_hashes': {name: sha(teacher / name) for name in
                           ('actor_samples.json', 'privileged_labels.json', 'report.json', 'manifest.json')},
        'base_skill_sha256': sha(base / 'student-skill.json'),
        'successful_teacher_cases': sorted(successful),
        'excluded_teacher_cases': [x['id'] for x in report['cases'] if not x['ok']],
        'models': fit, 'wall_elapsed_s': time.monotonic() - started,
        'runtime_input': 'own RGB and fixed shared top RGB to learned predictor; executor uses only own issued commands',
        'limitation': 'expert training fit is not independent rollout success; close/lift and initialization remain playback',
    }
    write(out / 'training-report.json', training)
    print(json.dumps({'out_dir': str(out), 'successful_cases': len(successful),
                      'excluded_cases': training['excluded_teacher_cases'],
                      'fit': {rid: x['diagnostics'] for rid, x in fit.items()},
                      'wall_elapsed_s': training['wall_elapsed_s']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
