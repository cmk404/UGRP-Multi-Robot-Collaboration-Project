#!/usr/bin/env python3
"""Secondary diagnostic: original 1x teacher on EVERY failed 2x-teacher start.

This does not change or train students, and is not pooled into primary results.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.run_reference_approach_pilot import run_owned


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('baseline','teacher','slow-model','grasp-model','mjpython','out'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT):raise ValueError('commit before diagnostic')
    baseline=read(a.baseline/'summary.json')
    if not baseline['complete']:raise ValueError('baseline incomplete')
    cases=[r for r in baseline['cases'] if r['condition']=='teacher2' and not r['success']]
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    report={'source_sha':source,'baseline_sha256':sha(a.baseline/'summary.json'),
            'scope':'Post hoc mechanism diagnostic: original 1x teacher on all failed 2x teacher starts. No student retuning.',
            'selected_case_ids':[c['case_id'] for c in cases],'cases':[],'complete':False}
    for c in cases:
        if subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=source or subprocess.check_output(['git','status','--porcelain'],cwd=ROOT):
            raise ValueError('diagnostic source changed')
        name=c['case_id'];fixture=out/(name+'-fixture.json');write(fixture,c['start_poses'])
        destination=out/(name+'-teacher1')
        command=[str(a.mjpython.absolute()),str(ROOT/'scripts/run_camera_approach_student.py'),
                 '--grasp-model-dir',str(a.grasp_model.resolve()),'--approach-model-dir',str(a.slow_model.resolve()),
                 '--start-poses-json',str(fixture),'--condition','visual','--out-dir',str(destination),
                 '--teacher-reference-dir',str(a.teacher.resolve()),'--teacher-speed','1']
        with (out/(name+'.log')).open('w') as log:code,timeout=run_owned(command,log,240)
        r=read(destination/'result.json') if (destination/'result.json').exists() else {}
        original=read(Path(c['raw_dir'])/'result.json')
        if r.get('evaluation_initial_state')!=original['evaluation_initial_state']:raise ValueError('different diagnostic start')
        if any(r['approach_calls'][i]['images'][k]['sha256']!=original['approach_calls'][i]['images'][k]['sha256'] for i in (0,1) for k in ('own','top')):raise ValueError('different diagnostic RGB')
        row={'case_id':name,'group':c['group'],'returncode':code,'timeout':timeout,'raw_dir':str(destination),
             'success':bool(code==0 and r.get('success')),'approach_ok':r.get('approach_ok'),'grasp_success':r.get('grasp_success'),
             'sim_s':r.get('approach_elapsed_sim_s'),'error':r.get('error'),'stop_reason':r.get('stop_reason'),
             'initial_state_and_rgb_match':True}
        report['cases'].append(row);write(out/'summary.json',report);print(json.dumps(row),flush=True)
    report['complete']=True;write(out/'summary.json',report)
    write(out/'raw-manifest.json',{str(f.relative_to(out)):sha(f) for f in sorted(out.rglob('*')) if f.is_file()})
    return int(any(r['returncode']!=0 or r['timeout'] for r in report['cases']))


if __name__=='__main__':raise SystemExit(main())
