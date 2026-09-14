"""Run unchanged terrain trials sequentially within an owned UGRP session."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]

def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mjpython',type=Path,required=True)
    p.add_argument('--grasp-model-dir',type=Path,required=True)
    p.add_argument('--out-dir',type=Path,required=True)
    args=p.parse_args()
    out=args.out_dir.resolve(); models=args.grasp_model_dir.resolve()
    if out.exists(): raise FileExistsError(out)
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit source and protocol first')
    sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    catalog=json.loads((ROOT/'maps/pair_navigation/catalog.json').read_text())
    out.mkdir(parents=True)
    record={'source_sha':sha,'impratio':10,'budget_per_trial':750,'wall_timeout_per_trial_s':600,
            'model_files':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(models.iterdir()) if f.is_file()},
            'trials':[]}
    write(out/'cohort.json',record)
    for e in catalog['entries']:
        current=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        if current!=sha: raise RuntimeError('execution source changed during cohort')
        command=[str(args.mjpython.resolve()),str(ROOT/'scripts/run_pair_navigation.py'),
                 '--map',str(ROOT/'maps/pair_navigation'/e['map']),
                 '--grasp-model-dir',str(models),'--out-dir',str(out/e['id']),
                 '--budget','750','--impratio','10']
        print('START '+e['id'],flush=True)
        started=time.monotonic()
        trial={'id':e['id'],'expected_route':e['expected_geometric_route'],'command':command}
        with (out/(e['id']+'.log')).open('w') as log:
            try:
                child=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=600)
                trial['returncode']=child.returncode
            except subprocess.TimeoutExpired:
                trial['timeout']=True
                record['trials'].append(trial);write(out/'cohort.json',record)
                raise  # Owned session wrapper cleans up every remaining child.
        trial['wall_seconds']=time.monotonic()-started
        result=out/e['id']/'result.json'
        if result.exists():
            report=json.loads(result.read_text())
            trial.update(success=report['success'],refused_no_route=report.get('refused_no_route',False),
                         error=report['error'],evaluation=report.get('evaluation'))
        else: trial['infrastructure_error']='result.json missing'
        record['trials'].append(trial);write(out/'cohort.json',record)
        print('END '+e['id']+' '+json.dumps({k:trial[k] for k in ('returncode','success','refused_no_route','wall_seconds') if k in trial}),flush=True)
    print('COHORT COMPLETE '+str(out),flush=True)

if __name__=='__main__': main()
