"""Finite sequential comparison with explicit, hash-frozen calibration and plans.

Each ordinary trial failure is recorded and the remaining declared trials run.
Operational skips stay unattempted. This runner does not train, tune or retry.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.carry_failure_metrics import schedule,outcome,aggregate
from scripts.cloud_progress import run_logged


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(path)


def command(protocol,job,output,mjpython,act_python):
    case=job['case'];controls=protocol['controls'];arm=protocol['conditions'][job['condition']]
    cmd=[mjpython,ROOT/'scripts/run_dispatch_e2e.py','--executor','skills',
         '--variant',case['variant'],'--seed',case['seed'],'--required-dock',case['dock'],
         '--plan-replay',case['plan'],'--grasp-model-dir',protocol['grasp'],
         '--stage-model-dir',protocol['stages'],'--output',output,
         '--max-wall-s',controls['max_wall_s'],'--carry-max-steps',controls['max_carry_steps'],
         '--video-fps',controls['video_fps'],'--spawn-offset',*case['offset']]
    if controls.get('efficient_capture'):cmd.append('--efficient-capture')
    if case.get('route_overlap',controls.get('route_overlap')):cmd.append('--route-overlap')
    if 'overlap_start' in controls:cmd+=['--overlap-start',controls['overlap_start']]
    if arm.get('model'):cmd+=['--carry-act-model',arm['model'],'--carry-act-python',act_python]
    if arm.get('stop_mode'):
        if not arm.get('model'):raise ValueError('stop mode requires ACT model')
        cmd+=['--carry-act-stop-mode',arm['stop_mode']]
    return list(map(str,cmd))


class CohortInterrupted(BaseException):
    def __init__(self,signum):self.signum=signum


def run():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','out','mjpython','act-python'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--tensorboard-dir',type=Path,help='new finite-cohort snapshot directory; export completed trials only')
    args=p.parse_args();protocol=json.loads(args.protocol.read_text())
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    protocol_sha=sha(args.protocol)
    if protocol.get('expected_source_sha',source)!=source:
        raise ValueError('protocol source SHA does not match checkout')
    def frozen():
        if sha(args.protocol)!=protocol_sha:raise RuntimeError('cohort protocol changed')
        if (subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=source
                or subprocess.check_output(['git','status','--porcelain'],cwd=ROOT)):
            raise RuntimeError('cohort source changed')
        for path,expected in protocol['asset_sha256'].items():
            if sha(path)!=expected:raise ValueError('frozen asset changed: '+path)
    frozen()
    fingerprints=[json.dumps({k:v for k,v in c.items() if k!='id'},sort_keys=True) for c in protocol['test']]
    if len(set(fingerprints))!=len(fingerprints):raise ValueError('duplicate physical cases; do not pretend repeated labels are new environments')
    jobs=schedule(protocol,list(protocol['conditions']))
    if len({j['trial_id'] for j in jobs})!=len(jobs):raise ValueError('duplicate trial IDs')
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=False)
    report={'source_sha':source,'protocol_sha256':sha(args.protocol),'protocol':protocol,
            'runs':[],'not_attempted':[],'complete':False,'jobs':jobs,'tensorboard_exports':[]}
    def save():
        report['failure_estimates']=aggregate(jobs,report['runs'])
        report['complete']=report['failure_estimates']['complete']
        write(args.out/'report.json',report)
    save()
    for job in jobs:
        frozen()
        if shutil.disk_usage(args.out).free/2**30<protocol['controls']['min_free_gib']:
            report['not_attempted'].append({**job,'reason':'disk_reserve'});save();continue
        output=args.out/'trials'/job['trial_id']
        cmd=command(protocol,job,output,args.mjpython,args.act_python)
        print(json.dumps({'trial':job['trial_id'],'state':'running'}),flush=True)
        try:
            row=run_logged(cmd,args.out/(job['trial_id']+'.log'),name=job['trial_id'],cwd=ROOT,
                           timeout=protocol['controls']['max_wall_s']+60,termination_grace_s=2)
        except OSError as error:
            row={'exit_code':127,'timed_out':False,'launch_error':type(error).__name__}
        except CohortInterrupted as error:
            # run_logged's finally has already reaped its separate child group.
            # Keep the interrupted attempt and leave remaining trials pending.
            row={'exit_code':128+error.signum,'timed_out':False,'interrupted_signal':error.signum}
        frozen()
        report['runs'].append({**job,**row,'output':str(output),'command':cmd,
                              'outcome':outcome(output,row['exit_code'],row['timed_out'])})
        save();print(json.dumps({'completed':len(report['runs']),'planned':len(jobs),'last':report['runs'][-1]['outcome']}),flush=True)
        if 'interrupted_signal' in row:
            report['interrupted_signal']=row['interrupted_signal'];save()
            return row['exit_code']
        if args.tensorboard_dir and (output/'result.json').exists():
            try:
                from scripts.tensorboard_tools.export import convert
                from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
                dest=args.tensorboard_dir/job['trial_id']
                manifest=convert(output,dest,max_images=3,media_port=6009)
                if not manifest['complete'] or manifest['warnings']:raise RuntimeError('dashboard export incomplete or warned')
                events=EventAccumulator(str(dest)).Reload()
                result=json.loads((output/'result.json').read_text())
                if events.Scalars('evaluation/reported_success')[0].value!=int(result['physical_success']):
                    raise RuntimeError('dashboard success readback mismatch')
                report['tensorboard_exports'].append({'trial_id':job['trial_id'],'snapshot':str(dest),
                    'manifest_sha256':sha(dest/'manifest.json'),'event_readback':True,'native_ui_verified':False})
            except Exception as error:
                report['tensorboard_exports'].append({'trial_id':job['trial_id'],'export_error':type(error).__name__+': '+str(error)})
            save()
    return 0 if report['complete'] else 2


def main():
    # ugrp_session forwards TERM to this runner's group; physical subprocesses
    # own separate groups. Unwind Python cleanup instead of orphaning them.
    def interrupt(signum,frame):raise CohortInterrupted(signum)
    previous={sig:signal.signal(sig,interrupt) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    try:return run()
    except CohortInterrupted as error:return 128+error.signum
    finally:
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':raise SystemExit(main())
