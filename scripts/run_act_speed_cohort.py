#!/usr/bin/env python3
"""Frozen matched starts for faster imitation and teacher-failure evaluation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.run_reference_approach_pilot import run_owned
from scripts.camera_approach_scene import validate_start_poses
from harness.reference_act_client import interpreter_path


def read(p):
    return json.loads(p.read_text())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def write(p, value):
    p.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','out-dir','mjpython','act-python','grasp-model-dir','teacher-dir','slow-model-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    for name in ('fast16-model-dir','fast17-model-dir'):
        p.add_argument('--'+name,type=Path)
    p.add_argument('--conditions',nargs='+',required=True,choices=('teacher2','slow_act','fast_act16','fast_act17'))
    args=p.parse_args()
    if len(set(args.conditions)) != len(args.conditions):
        raise ValueError('duplicate conditions')
    source=git('rev-parse','HEAD')
    if git('status','--porcelain'):
        raise ValueError('commit source and protocol before cohort')
    protocol=read(args.protocol)
    roots={'teacher2':args.slow_model_dir,'slow_act':args.slow_model_dir,
           'fast_act16':args.fast16_model_dir,'fast_act17':args.fast17_model_dir}
    hashes={}
    for condition in args.conditions:
        root=roots[condition]
        if root is None or not read(root/'report.json')['complete']:
            raise ValueError('missing completed training for '+condition)
        info=read(root/'report.json')
        subprocess.run(['git','diff','--exit-code',info['source_sha'],source,'--',
                        'harness/reference_act.py','scripts/reference_act_worker.py',
                        'harness/reference_act_client.py'],cwd=ROOT,check=True)
        for name,expected in info['artifacts'].items():
            file=(root/name).resolve()
            if not file.is_relative_to(root.resolve()) or sha(file)!=expected:
                raise ValueError('training artifact mismatch')
        hashes[condition]={str(root.resolve()/name):h for name,h in info['artifacts'].items()
                           if name.endswith(('safetensors','config.json','adapter.json'))}
    out=args.out_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    summary={'source_sha':source,'protocol_sha256':sha(args.protocol),'protocol':protocol,
             'model_hashes':hashes,'conditions':args.conditions,'cases':[], 'complete':False,
             'external_model_calls':0,'external_model_tokens':0}
    for index,case in enumerate(protocol['test_cases']):
        starts=validate_start_poses(case['start_poses'])
        fixture=out/(case['id']+'-fixture.json');write(fixture,starts)
        conditions=args.conditions[index % len(args.conditions):]+args.conditions[:index % len(args.conditions)]
        for condition in conditions:
            if git('rev-parse','HEAD')!=source or git('status','--porcelain'):
                raise ValueError('source changed during cohort')
            for path,expected in hashes[condition].items():
                if sha(Path(path))!=expected:raise ValueError('checkpoint changed')
            destination=out/(case['id']+'-'+condition)
            command=[str(args.mjpython.absolute()),str(ROOT/'scripts/run_camera_approach_student.py'),
                     '--grasp-model-dir',str(args.grasp_model_dir.resolve()),
                     '--approach-model-dir',str(roots[condition].resolve()),
                     '--start-poses-json',str(fixture),'--condition','visual','--out-dir',str(destination)]
            if condition=='teacher2':
                command+=['--teacher-reference-dir',str(args.teacher_dir.resolve()),'--teacher-speed','2']
            else:
                command+=['--act-python',str(interpreter_path(args.act_python)),
                          '--act-model-dir',str(roots[condition].resolve())]
            print(json.dumps({'case':case['id'],'condition':condition,'status':'running'}),flush=True)
            with (out/(destination.name+'.log')).open('w') as log:
                code,timeout=run_owned(command,log,timeout=240)
            result=read(destination/'result.json') if (destination/'result.json').exists() else {}
            row={'case_id':case['id'],'group':case['group'],'condition':condition,'start_poses':starts,
                 'raw_dir':str(destination),'returncode':code,'timeout':timeout,
                 'success':bool(code==0 and result.get('success')),
                 'approach_ok':result.get('approach_ok',False),'grasp_success':result.get('grasp_success',False),
                 'sim_s':result.get('approach_elapsed_sim_s'),'wall_s':result.get('wall_elapsed_s'),
                 'commands':len(result.get('approach_calls',[])),
                 'contact_steps':result.get('approach_payload_contact_steps'),
                 'stop_reason':result.get('stop_reason'),'error':result.get('error'),
                 'source_sha':result.get('source_sha')}
            summary['cases'].append(row)
            write(out/'summary.json',summary)
            print(json.dumps(row),flush=True)
    summary['complete']=True;write(out/'summary.json',summary)
    write(out/'raw-manifest.json',{str(f.relative_to(out)):sha(f) for f in sorted(out.rglob('*')) if f.is_file()})
    return int(any(r['returncode']!=0 or r['timeout'] for r in summary['cases']))


if __name__=='__main__':
    raise SystemExit(main())
