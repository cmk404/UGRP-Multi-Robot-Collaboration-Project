#!/usr/bin/env python3
"""Run the full teacher pilot, then preregistered development/training cohorts."""
import argparse,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser()
 for k in ('protocol-dir','out','mjpython','grasp-model'):p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
 stages=[('teacher-pilot','pilot'),('development-cases','development'),('nominal-cases','nominal'),('recovery-cases','recovery')]
 for fixture,name in stages:
  cmd=[sys.executable,str(ROOT/'scripts/run_recovery_cases.py'),'--cases',str((a.protocol_dir/(fixture+'.json')).resolve()),'--out',str((a.out/name).resolve()),'--mjpython',str(a.mjpython.absolute()),'--grasp-model',str(a.grasp_model.resolve()),'--policy','recovery_teacher']
  subprocess.run(cmd,check=True,cwd=ROOT)
  r=json.loads((a.out/name/'summary.json').read_text())
  if name=='pilot' and not all(c['success'] for c in r['cases']):raise RuntimeError('full teacher pilot failed; do not collect curriculum')
if __name__=='__main__':main()
