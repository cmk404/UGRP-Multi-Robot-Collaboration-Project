"""Finite reference reproduction before resuming an ACT input comparison."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.cloud_progress import run_logged
from scripts.carry_failure_metrics import outcome


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--act-python',type=Path,required=True)
    parser.add_argument('--mjpython',type=Path,required=True)
    args=parser.parse_args()
    protocol=json.loads(args.protocol.read_text());controls=protocol['controls']
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    def frozen():
        if (subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=source
                or subprocess.check_output(['git','status','--porcelain'],cwd=ROOT)):
            raise RuntimeError('revalidation source changed')
    frozen()
    original=Path(protocol['original_act_model'])
    if sha(original/'model.safetensors')!=protocol['original_act_model_sha256']:
        raise ValueError('original ACT checkpoint changed')
    calibration=Path(protocol['original_calibration'])
    for path,digest in protocol['original_calibration_sha256'].items():
        if sha(calibration/path)!=digest:raise ValueError('original calibration changed')
    candidate=Path(protocol['candidate_act_model'])
    if sha(candidate/'model.safetensors')!=protocol['candidate_act_model_sha256']:
        raise ValueError('candidate ACT checkpoint changed')
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=False)
    report={'complete':False,'source_sha':source,'protocol':protocol,
            'protocol_sha256':sha(args.protocol),'runs':[],'planned':4}
    write(args.out/'report.json',report)

    def trial(job):
        frozen()
        if shutil.disk_usage(args.out).free/2**30<controls['min_free_gib']:
            return {**job,'not_attempted':True,'reason':'disk_reserve'}
        case=next((c for c in protocol['cases'] if c['id']==job['case']),
                  protocol['release_regression'])
        root=calibration if job['calibration']=='original' else ROOT/protocol['native_calibration']
        output=args.out/job['condition']/job['case']
        plan=Path(protocol['teacher_roots'][case['teacher_case']])/'committed-plan.json'
        cmd=[args.mjpython,ROOT/'scripts/run_dispatch_e2e.py','--executor','skills',
             '--variant',case['variant'],'--seed','11','--required-dock','dock_a',
             '--plan-replay',plan,'--grasp-model-dir',root/'grasp',
             '--stage-model-dir',root/'varied','--output',output,
             '--max-wall-s',controls['max_wall_s'],'--carry-max-steps',controls['max_carry_steps'],
             '--video-fps',controls['video_fps'],'--spawn-offset',*case['offset']]
        if controls.get('efficient_capture',False):cmd.append('--efficient-capture')
        if not job['condition'].startswith('RGB'):
            model=original if job['condition']=='ACT-original-seed18' else candidate
            cmd+=['--carry-act-model',model,'--carry-act-python',args.act_python]
        run=run_logged(cmd,args.out/(job['condition']+'-'+job['case']+'.log'),
                       name=job['condition']+'-'+job['case'],cwd=ROOT,
                       timeout=controls['process_timeout_s'])
        frozen()
        return {**job,**run,'output':str(output),'command':list(map(str,cmd)),
                'outcome':outcome(output,run['exit_code'],run['timed_out'])}

    def batch(jobs):
        with ThreadPoolExecutor(max_workers=controls['workers']) as pool:
            for future in as_completed([pool.submit(trial,j) for j in jobs]):
                row=future.result();report['runs'].append(row)
                write(args.out/'report.json',report)
                print(json.dumps({'completed':len(report['runs']),'condition':row['condition'],
                                  'case':row['case'],'outcome':row.get('outcome'),
                                  'not_attempted':row.get('not_attempted',False)}),flush=True)
    batch(protocol['initial_jobs'][:2])
    if not all(r.get('outcome',{}).get('whole_success') for r in report['runs']):
        report['blocked']='original open reference did not reproduce; diagnose before new comparison'
        write(args.out/'report.json',report)
        return 1
    batch([protocol['initial_jobs'][2],protocol['release_regression']])
    report['complete']=not any(r.get('not_attempted') for r in report['runs'])
    write(args.out/'report.json',report)
    return 0 if report['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
