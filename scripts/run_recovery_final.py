#!/usr/bin/env python3
"""Evaluate all fixed models only after training is complete; one simulator at a time."""
import argparse,json,subprocess,sys,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser()
 for k in ('spec','mjpython','act-python','grasp-model'):p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();spec=json.loads(a.spec.read_text());source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
 for condition,e in spec['conditions'].items():
  if e.get('model'):
   root=Path(e['model']);r=json.loads((root/'report.json').read_text())
   if not r['complete']:raise ValueError('incomplete checkpoint '+condition)
   for rel,h in r['artifacts'].items():
    if sha(root/rel)!=h:raise ValueError('checkpoint changed')
 for condition,e in spec['conditions'].items():
  if subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=source:raise ValueError('source changed')
  cmd=[sys.executable,str(ROOT/'scripts/run_recovery_cases.py'),'--cases',spec['test_cases'],'--out',e['cohort'],'--mjpython',str(a.mjpython.absolute()),'--grasp-model',str(a.grasp_model.resolve()),'--policy',e['policy']]
  if e.get('model'):cmd+=['--model',e['model'],'--act-python',str(a.act_python.absolute())]
  subprocess.run(cmd,cwd=ROOT,check=True)
if __name__=='__main__':main()
