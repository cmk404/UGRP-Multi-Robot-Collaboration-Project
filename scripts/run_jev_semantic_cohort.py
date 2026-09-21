#!/usr/bin/env python3
"""Frozen 12-pose, three-repeat numeric/semantic closed-loop comparison."""
from __future__ import annotations
import argparse
import concurrent.futures
import contextlib
import getpass
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.run_jev_motion import CASES,trial,write
from harness.jev_motion import GOAL,VECTORS

NEW_CASES={f'new{i+1:02d}':pose for i,pose in enumerate([
    [-.82,-2.65,0],[-.72,-2.65,0],[-.90,-2.65,0],
    [-.83,-2.71,9],[-.77,-2.72,13],[-.89,-2.69,5],
    [-.83,-2.59,-9],[-.77,-2.58,-13],[-.89,-2.61,-5],
    [-.79,-2.68,-6],[-.79,-2.62,6],[-.87,-2.65,11]])}
ARMS=[('rule','numeric'),('jev','numeric'),('jev','semantic'),('gemini','numeric'),('gemini','semantic')]

def plan():
    jobs=[]
    for partition,cases,repeats in [('new',NEW_CASES,3),('regression',{k:CASES[k] for k in ('straight','left_offset','right_offset')},1)]:
        for case in cases:
            for repeat in range(repeats):
                for policy,variant in ARMS:
                    jobs.append(dict(partition=partition,case=case,repeat=repeat,policy=policy,state_variant=variant,
                        trial_id=f'{partition}-{case}-r{repeat}-{policy}-{variant}'))
    random.Random(210921).shuffle(jobs)
    return jobs

def worker(protocol,output,index):
    # Anonymous pipe only: never put credentials in argv, files, or environment.
    key=json.load(sys.stdin)['key']
    import mujoco,cv2
    write(output/f'environment-{index}.json',dict(python=sys.version,platform=platform.platform(),mujoco=mujoco.__version__,opencv=cv2.__version__))
    for job in protocol['jobs'][index::protocol['workers']]:
        args=SimpleNamespace(**protocol['limits'],**job,output=output,case_setup=protocol['case_setup'],
            jev_model='jev-1.13.0',gemini_model='gemini-3.8-flash',gemini_url='http://127.0.0.1:8391/v1/chat/completions')
        result=trial(args,job['policy'],job['case'],key if job['policy']=='jev' else None,protocol['source_sha'])
        result.update(partition=job['partition'],repeat=job['repeat'],trial_id=job['trial_id'])
        write(output/job['trial_id']/'result.json',result)
    return 0

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mjpython',type=Path)
    p.add_argument('--workers',type=int,default=4,choices=range(1,5))
    p.add_argument('--worker',type=int)
    p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    if args.worker is not None:return worker(json.loads((args.output/'protocol.json').read_text()),args.output,args.worker)
    if not args.execute or not args.mjpython:p.error('--execute and --mjpython required')
    if args.output.exists():p.error('output must be new')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():p.error('commit source first')
    sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    protocol=dict(source_sha=sha,jobs=plan(),workers=args.workers,case_setup={**CASES,**NEW_CASES},goal=GOAL,vectors=VECTORS,
        limits=dict(max_steps=70,max_input_tokens=180000,max_wall_s=360,timeout=30),
        scope='new start-pose generalization on same fixed map/object; three API repeats, identical physics',
        variant='semantic uses archived-state B exactly; includes matching wording, excludes explicit rule criteria',
        clock='SIM paused per episode during inference; concurrent wall time not comparable to earlier serial cohort',
        failures='all attempted episodes included; no automatic retries, no pose filtering or source changes')
    args.output.mkdir(parents=True);write(args.output/'protocol.json',protocol)
    key=getpass.getpass('Jev API key (hidden): ')
    def launch(index):
        with (args.output/f'worker-{index}.log').open('w') as log:
            proc=subprocess.Popen([str(args.mjpython),str(Path(__file__).resolve()),'--output',str(args.output),'--worker',str(index)],
                stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,cwd=ROOT)
            proc.communicate(json.dumps({'key':key}))
            return index,proc.returncode
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        codes=list(pool.map(launch,range(args.workers)))
    write(args.output/'worker-exit-codes.json',codes)
    results=[json.loads((args.output/j['trial_id']/'result.json').read_text()) for j in protocol['jobs'] if (args.output/j['trial_id']/'result.json').exists()]
    write(args.output/'results.json',results)
    print(json.dumps({'completed':len(results),'planned':len(protocol['jobs']),'workers':codes}),flush=True)
    return int(len(results)!=len(protocol['jobs']) or any(c for _,c in codes))

if __name__=='__main__':raise SystemExit(main())
