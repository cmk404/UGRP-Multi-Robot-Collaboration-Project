#!/usr/bin/env python3
"""Refit local grasp features from preserved OFFLINE TEACHER RGB and labels.

No simulator is opened. Runtime failure frames are never training examples.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from harness.camera_recovery_student import fit_recovery_model, GRASP_TOP_ROI


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def refit(source, output):
    source = source.resolve()
    actors = json.loads((source/'actor-samples.json').read_text())
    labels = {r['sample_id']: r for r in json.loads((source/'teacher-labels-only.json').read_text())}
    if len({r['sample_id'] for r in actors}) != len(actors):
        raise ValueError('duplicate actor sample')
    records = []
    def views(row):
        result = []
        for key in ('own_rgb', 'top_rgb'):
            record = row[key]; path = (source/record['path']).resolve()
            if not path.is_relative_to(source) or sha(path) != record['sha256']:
                raise ValueError('teacher RGB path/hash mismatch')
            records.append(record)
            result.append(path.read_bytes())
        return result
    old = source/'models/grasp'
    manifest = copy.deepcopy(json.loads((old/'student-skill.json').read_text()))
    manifest['rgb_support_scope'] = 'local_grasp_top_v1'
    manifest.pop('constant_background_top_band', None)
    manifest.pop('constant_background_top_roi', None)
    output.mkdir(parents=True, exist_ok=False)
    diagnostics = {}
    for slot in ('r1', 'r3'):
        references = [r for r in actors if r['sample_id'] == 'grasp-reference-'+slot and r['model_slot'] == slot]
        if len(references) != 1: raise ValueError('missing unique teacher reference')
        reference = views(references[0]); samples = []
        for row in actors:
            if row['model_slot'] != slot or not row['sample_id'].startswith('grasp-'+slot+'-'): continue
            label = labels[row['sample_id']]
            if label['physical_teacher_recovery'] is not True: continue
            own, top = views(row)
            samples.append({'own_jpeg': own, 'top_jpeg': top,
                            'case_id': 'grasp-'+row['sample_id'].rsplit('-', 1)[1],
                            'correction_pulses': label['correction_pulses']})
        model = fit_recovery_model(*reference, samples, top_roi=GRASP_TOP_ROI)
        path = output/(slot+'-model.json'); write(path, model)
        manifest['models'][slot] = {'path': path.name, 'sha256': sha(path)}
        diagnostics[slot] = model['diagnostics']
        for name, data in zip(('own', 'top'), reference):
            (output/f'{slot}-goal-{name}.jpg').write_bytes(data)
    write(output/'student-skill.json', manifest)
    shutil.copy2(old/'evaluation-fixture.json', output/'evaluation-fixture.json')
    report = {'complete': True, 'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'teacher_source': str(source), 'teacher_report_sha256': sha(source/'report.json'),
              'teacher_actor_manifest_sha256': sha(source/'actor-samples.json'),
              'teacher_labels_sha256': sha(source/'teacher-labels-only.json'),
              'source_skill_sha256': sha(old/'student-skill.json'),
              'runtime_frames_used_for_training': False, 'trained_top_roi': GRASP_TOP_ROI,
              'input_records': records, 'diagnostics': diagnostics, 'models': manifest['models']}
    write(output/'refit-report.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        raise RuntimeError('commit source before refitting')
    result = refit(args.source, args.output)
    print(json.dumps({'complete': result['complete'], 'diagnostics': result['diagnostics']}))
