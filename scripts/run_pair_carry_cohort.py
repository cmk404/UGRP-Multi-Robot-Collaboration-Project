#!/usr/bin/env python3
"""Execute a registered paired matrix with frozen source/models and read-back."""
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
from scripts.audit_pair_carry_sync import audit


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases-json', type=Path, required=True)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--transport-model-dir', type=Path, required=True)
    p.add_argument('--mjpython', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    args = p.parse_args()
    if git('status','--porcelain'):
        raise RuntimeError('commit all source before cohort')
    source = git('rev-parse','HEAD')
    config = json.loads(args.cases_json.read_text())
    cases = config['cases']
    if not cases or len({c['case_id'] for c in cases}) != len(cases):
        raise ValueError('unique nonempty cases required')
    hashes = {str(p.resolve()):sha(p) for root in (args.grasp_model_dir,args.transport_model_dir)
              for p in root.glob('*.json')}
    hashes[str(args.cases_json.resolve())] = sha(args.cases_json)
    out = args.out_dir.resolve()
    out.mkdir(parents=True,exist_ok=False)
    (out/'case-inputs').mkdir()
    report = {'source_sha':source, 'input_hashes':hashes, 'cases':cases, 'runs':[],
              'complete':False,'external_model_calls':0,'cost_usd':0}
    write(out/'cohort-report.json',report)
    started = time.monotonic()
    for case in cases:
        first = None
        case_file = out/'case-inputs'/f"{case['case_id']}.json"
        write(case_file,case)
        for condition in config.get('conditions',['baseline','sync']):
            if git('rev-parse','HEAD') != source or git('status','--porcelain'):
                raise RuntimeError('source changed during cohort')
            if any(sha(Path(p))!=v for p,v in hashes.items()):
                raise RuntimeError('model/config changed during cohort')
            name = f"{case['case_id']}-{condition}"
            run = out/name
            command = [str(args.mjpython), 'scripts/run_pair_carry_sync.py',
                '--case-json',str(case_file),'--condition',condition,
                '--grasp-model-dir',str(args.grasp_model_dir.resolve()),
                '--transport-model-dir',str(args.transport_model_dir.resolve()),'--out-dir',str(run)]
            with (out/f'{name}.log').open('w') as log:
                # On timeout terminate this cohort; the session wrapper cleans its group.
                done = subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            result_file = run/'result.json'
            d = json.loads(result_file.read_text()) if result_file.exists() else {}
            checked = audit(run,args.transport_model_dir,args.grasp_model_dir)
            write(run/'input-audit.json',checked)
            images = {r:{k:v['sha256'] for k,v in q.items()} for r,q in d.get('anchor_images',{}).items()}
            paired = None if first is None else images == first
            first = images if first is None else first
            row = {'case_id':case['case_id'],'group':case['group'],'condition':condition,
                   'directory':str(run),'returncode':done.returncode,'audit_success':checked['success'],
                   'physical_success':d.get('success',False),'success':bool(d.get('success') and checked['success']),
                   'paired_initial_carry_rgb_equal':paired,'source_sha':d.get('source_sha'),
                   'wall_time_s':d.get('wall_time_s'),'sim_time_s':d.get('sim_time_s'),
                   'steps':len(d.get('steps',[])), 'recoveries':len(d.get('policy_events',[])),
                   'error':d.get('error'),'audit_errors':checked.get('errors'),
                   'evaluation':d.get('evaluation'), 'result_sha256':sha(result_file) if result_file.exists() else None}
            report['runs'].append(row)
            write(out/'cohort-report.json',report)
            print(json.dumps({k:row[k] for k in ('case_id','condition','success','audit_success','steps','recoveries')}),flush=True)
            if not checked['success'] or paired is False or d.get('source_sha') != source:
                raise RuntimeError('evidence/source/pairing failed; inspect preserved run before continuing')
            if case['group']=='fixed' and sum(r['group']=='fixed' for r in report['runs'])==20:
                if any(sum(r['success'] for r in report['runs'] if r['group']=='fixed' and r['condition']==c)<9
                       for c in ('baseline','sync')):
                    raise RuntimeError('fixed retention gate failed; do not expand cohort')
    report['complete'] = True
    report['wall_time_s'] = time.monotonic()-started
    report['scores'] = {g:{c:{'success':sum(r['success'] for r in report['runs'] if r['group']==g and r['condition']==c),
                              'total':sum(r['group']==g and r['condition']==c for r in report['runs'])}
                          for c in ('baseline','sync')} for g in sorted({r['group'] for r in report['runs']})}
    write(out/'cohort-report.json',report)
    print(json.dumps(report['scores']),flush=True)


if __name__=='__main__':
    main()
