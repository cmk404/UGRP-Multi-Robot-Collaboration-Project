#!/usr/bin/env python3
"""Serial owned trial cohort; freeze source and preserve every attempted case."""
import argparse,json,subprocess,sys,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.run_reference_approach_pilot import run_owned

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
def main():
 p=argparse.ArgumentParser()
 for k in ('cases','out','grasp-model','mjpython'):p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--policy',required=True,choices=('recovery_teacher','nominal_teacher','act'))
 p.add_argument('--command-decoder',choices=('raw','calibrated'),default='raw')
 p.add_argument('--act-python',type=Path);p.add_argument('--model',type=Path);p.add_argument('--takeover',action='store_true')
 a=p.parse_args();git=lambda *x:subprocess.check_output(['git',*x],cwd=ROOT,text=True).strip();source=git('rev-parse','HEAD')
 if git('status','--porcelain'):raise ValueError('commit before cohort')
 a.out.mkdir(parents=True,exist_ok=False);cases=json.loads(a.cases.read_text());summary={'source_sha':source,'cases_sha256':sha(a.cases),'policy':a.policy,'cases':[],'complete':False}
 for c in cases:
  if git('rev-parse','HEAD')!=source or git('status','--porcelain'):raise ValueError('source changed during cohort')
  fixture=a.out/(c['id']+'-fixture.json');write(fixture,c);dest=a.out/c['id']
  cmd=[str(a.mjpython.absolute()),str(ROOT/'scripts/run_act_recovery_trial.py'),'--case',str(fixture.resolve()),'--out',str(dest.resolve()),'--grasp-model',str(a.grasp_model.resolve()),'--policy',a.policy]
  cmd+=['--command-decoder',a.command_decoder]
  if a.model:cmd+=['--model',str(a.model.resolve()),'--act-python',str(a.act_python.absolute())]
  if a.takeover:cmd+=['--takeover-step',str(c['takeover_step'])]
  print(json.dumps({'case':c['id'],'status':'running'}),flush=True)
  with (a.out/(c['id']+'.log')).open('w') as log:code,timeout=run_owned(cmd,log,300)
  r=json.loads((dest/'result.json').read_text()) if (dest/'result.json').exists() else {}
  row={'id':c['id'],'group':c.get('group'),'raw_dir':str(dest.resolve()),'returncode':code,'timeout':timeout,**{k:r.get(k) for k in ('success','approach_ok','alignment','training_eligible','contact_steps','approach_sim_s','wall_s','error')}}
  summary['cases'].append(row);write(a.out/'summary.json',summary);print(json.dumps(row),flush=True)
  if code or timeout:raise RuntimeError('trial runtime failure; retain partial cohort')
 summary['complete']=True;write(a.out/'summary.json',summary)
 write(a.out/'raw-manifest.json',{str(f.relative_to(a.out)):sha(f) for f in sorted(a.out.rglob('*')) if f.is_file()})
if __name__=='__main__':main()
