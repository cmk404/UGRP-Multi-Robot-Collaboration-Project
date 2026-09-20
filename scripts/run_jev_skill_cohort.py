#!/usr/bin/env python3
"""Frozen, fully enumerated skill cohorts with anonymous-pipe credentials."""
from __future__ import annotations
import argparse
import concurrent.futures
import getpass
import json
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.run_jev_skill_motion import DEVELOPMENT,HOLDOUT,run_trial,write
from scripts.run_jev_semantic_cohort import NEW_CASES


def plan(phase,policies=('rule','jev','gemini'),dev_cases=None):
    setup={**DEVELOPMENT,**HOLDOUT,**{f'legacy-{k}':{'pose':v,'target':[-.18,-2.65]} for k,v in NEW_CASES.items()}}
    jobs=[]
    if phase=='development':groups=[('full',dev_cases or list(DEVELOPMENT),1,policies)]
    elif phase=='holdout':groups=[('full',list(HOLDOUT),3,policies)]
    elif phase=='regression':groups=[('full',[k for k in setup if k.startswith('legacy-')],1,policies)]
    elif phase=='continuous':groups=[('full',['open01','known01','occluded01','temporary01'],2,[p for p in policies if p!='rule'])]
    elif phase=='ablation':
        models=[p for p in policies if p!='rule']
        groups=[('primitive',['open01','open02','yaw01','yaw02'],2,models),
                ('always',['yaw01','unknown01','occluded01','temporary01'],2,models),
                ('single',['yaw01','unknown01','occluded01','temporary01'],2,models),
                ('no_confidence',['yaw01','unknown01','occluded01','temporary01'],2,[p for p in models if p=='jev'])]
    else:raise ValueError('unknown phase')
    for arm,cases,repeats,actors in groups:
        for case in cases:
            for repeat in range(repeats):
                for policy in actors:
                    jobs.append({'case':case,'policy':policy,'arm':arm,'repeat':repeat,
                                 'trial_id':f'{case}-r{repeat}-{policy}-{arm}'})
    random.Random(210922).shuffle(jobs)
    return jobs,{j['case']:setup[j['case']] for j in jobs}


def worker(output,index):
    protocol=json.loads((output/'protocol.json').read_text())
    key=json.load(sys.stdin)['key']
    for job in protocol['jobs'][index::protocol['workers']]:
        args=SimpleNamespace(**job,**protocol['limits'],output=output/job['trial_id'],
            gemini_url='http://127.0.0.1:8391/v1/chat/completions')
        result=run_trial(args,protocol['case_setup'][job['case']],job['policy'],key if job['policy']=='jev' else None,protocol['source_sha'])
        result.update(repeat=job['repeat'],trial_id=job['trial_id'],partition=protocol['phase'])
        write(args.output/'result.json',result)
    return 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--mjpython',type=Path)
    p.add_argument('--worker',type=int);p.add_argument('--workers',type=int,default=4,choices=range(1,5))
    p.add_argument('--phase',choices=['development','holdout','ablation','regression','continuous'],default='development')
    p.add_argument('--policies',nargs='+',choices=['rule','jev','gemini'],default=['rule','jev','gemini'])
    p.add_argument('--dev-cases',nargs='+',choices=list(DEVELOPMENT))
    p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    if args.worker is not None:return worker(args.output,args.worker)
    if not args.execute or not args.mjpython:p.error('--execute and --mjpython required')
    if args.output.exists():p.error('output must be new')
    if args.dev_cases and args.phase!='development':p.error('case selection is development-only')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():p.error('commit source before experiments')
    jobs,cases=plan(args.phase,args.policies,args.dev_cases)
    protocol={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'phase':args.phase,'jobs':jobs,'case_setup':cases,'workers':args.workers,
        'limits':{'max_sim_s':90.,'max_wall_s':1200.,'max_calls':400,'max_input_tokens':1000000,
                  'clock':'continuous' if args.phase=='continuous' else 'paused'},
        'clock':'continuous with expired-skill braking' if args.phase=='continuous' else 'paused SIM during inference',
        'repeats':'identical physics; fresh API repeats, not independent layouts',
        'failures':'all enumerated jobs retained; no retries or holdout filtering',
        'cost_usd':None,'boundary':'RGB-only control, authored prior map, own commands; referee and event schedule output-only'}
    args.output.mkdir(parents=True);write(args.output/'protocol.json',protocol)
    key=getpass.getpass('Jev API key (hidden): ') if 'jev' in args.policies else None
    def launch(i):
        with (args.output/f'worker-{i}.log').open('w') as log:
            proc=subprocess.Popen([str(args.mjpython),str(Path(__file__).resolve()),'--output',str(args.output),'--worker',str(i)],
                stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,cwd=ROOT)
            proc.communicate(json.dumps({'key':key}))
            return [i,proc.returncode]
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        codes=list(pool.map(launch,range(args.workers)))
    results=[]
    for job in jobs:
        path=args.output/job['trial_id']/'result.json'
        if path.exists():results.append(json.loads(path.read_text()))
    write(args.output/'worker-exit-codes.json',codes);write(args.output/'results.json',results)
    print(json.dumps({'completed':len(results),'planned':len(jobs),'successes':sum(r['success'] for r in results),'workers':codes}),flush=True)
    return int(len(results)!=len(jobs) or any(x[1] for x in codes))

if __name__=='__main__':raise SystemExit(main())
