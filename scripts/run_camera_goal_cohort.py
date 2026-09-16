#!/usr/bin/env python3
"""Frozen full-transport coverage across setup-only start poses."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from scripts.run_camera_goal_transport import setup_poses
from scripts.run_camera_approach_student import sha, write
from scripts.audit_camera_goal_transport import audit


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases-json', type=Path, required=True)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--stage-model-dir', type=Path, required=True)
    p.add_argument('--reference-top', type=Path, required=True)
    p.add_argument('--mjpython', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    a = p.parse_args()
    cases = json.loads(a.cases_json.read_text())['cases']
    if not cases or len({c['case_id'] for c in cases}) != len(cases):
        raise ValueError('unique nonempty cases required')
    for c in cases:
        if not c['case_id'] or any(x not in 'abcdefghijklmnopqrstuvwxyz0123456789-' for x in c['case_id']):
            raise ValueError('invalid case_id')
        setup_poses(c['distance'], c['lateral'], c['yaw_deg'])
    if git('status', '--porcelain'): raise RuntimeError('commit source and protocol before cohort')
    source = git('rev-parse', 'HEAD')
    files = [a.cases_json, a.reference_top]
    for folder in (a.grasp_model_dir, a.stage_model_dir):
        files.extend(f for f in folder.rglob('*') if f.is_file())
    hashes = {str(f.resolve()): sha(f) for f in files}
    out = a.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = dict(schema='ugrp.goal_transport_cohort.v1', source_sha=source,
                  assets_sha256=hashes, cases=cases, runs=[], complete=False)
    write(out/'cohort-report.json', report)
    started = time.monotonic()
    for case in cases:
        for planner in ('local', 'llm'):
            if git('rev-parse', 'HEAD') != source or git('status', '--porcelain'):
                raise RuntimeError('source changed during cohort')
            if any(sha(Path(f)) != digest for f, digest in hashes.items()):
                raise RuntimeError('cohort assets changed')
            name = case['case_id']+'-'+planner
            run_out = out/name
            command = [str(a.mjpython.resolve()), 'scripts/run_camera_goal_transport.py',
                '--planner', planner, '--grasp-model-dir', str(a.grasp_model_dir.resolve()),
                '--stage-model-dir', str(a.stage_model_dir.resolve()),
                '--reference-top', str(a.reference_top.resolve()), '--out-dir', str(run_out)]
            for flag, key in (('--distance', 'distance'), ('--lateral', 'lateral'), ('--yaw-deg', 'yaw_deg')):
                command += [flag, *map(str, case[key])]
            print(json.dumps(dict(start=name, completed=len(report['runs']), total=2*len(cases))), flush=True)
            with (out/(name+'.log')).open('w') as log:
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            path = run_out/'result.json'
            data = json.loads(path.read_text()) if path.exists() else {}
            try: checked = audit(run_out)
            except Exception: checked = dict(ok=False, error=traceback.format_exc())
            run_out.mkdir(exist_ok=True)
            write(run_out/'audit.json', checked)
            trace = json.loads((run_out/'execution-trace.json').read_text()) if (run_out/'execution-trace.json').exists() else []
            llm = data.get('llm') or {}
            row = dict(case_id=case['case_id'], group=case['group'], planner=planner,
                directory=str(run_out), returncode=result.returncode,
                success=result.returncode == 0 and data.get('success') is True and checked['ok'],
                physical_success=data.get('success', False), audit_ok=checked['ok'],
                error=data.get('error', 'missing result'), evaluation=data.get('evaluation'),
                last_stage=trace[-1]['stage'] if trace else None,
                wall_s=data.get('wall_s'), sim_s=data.get('sim_s'),
                wheel_batches=sum('actions' in r for r in trace), arm_batches=sum('command' in r for r in trace),
                llm_calls=llm.get('call_count', 0), llm_usage=llm.get('usage', {}),
                cost_usd=None if planner == 'llm' else 0,
                result_sha256=sha(path) if path.exists() else None)
            report['runs'].append(row)
            write(out/'cohort-report.json', report)
            print(json.dumps({k:row[k] for k in ('case_id','planner','success','physical_success','audit_ok','last_stage','wall_s')}), flush=True)
    report['complete'] = True
    report['wall_s'] = time.monotonic()-started
    report['scores'] = {p: dict(success=sum(r['success'] for r in report['runs'] if r['planner']==p),
        total=sum(r['planner']==p for r in report['runs'])) for p in ('local','llm')}
    write(out/'cohort-report.json', report)
    print(json.dumps(report['scores']), flush=True)


if __name__ == '__main__': main()
