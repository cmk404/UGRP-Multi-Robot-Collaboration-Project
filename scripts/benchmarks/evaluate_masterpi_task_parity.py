#!/usr/bin/env python3
from __future__ import annotations
import sys
import argparse,json,math,os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    from scripts.benchmarks.masterpi_calibration_common import MANIFEST,finite,invalidate_manifest,load_jsonl,load_manifest,real_measurement_source,save_manifest
except ModuleNotFoundError:
    from masterpi_calibration_common import MANIFEST,finite,invalidate_manifest,load_jsonl,load_manifest,real_measurement_source,save_manifest
from sim.calibration_schema import CALIBRATABLE_DYNAMICS,CALIBRATABLE_HARDWARE
from sim.masterpi_production_v2 import MasterPiProductionV2

ACTIONS=('search','track','approach','pick')

def completed(path,split=None):
    out=[]
    for r in load_jsonl(path):
        if split is not None and r.get('split')!=split: continue
        if not isinstance(r.get('physical_success'),bool): continue
        if not all(finite(r.get(k)) for k in ('red_x_m','red_y_m','red_yaw_deg')): raise ValueError('bad physical scenario coordinates')
        if not real_measurement_source(r.get('measurement_source')): raise ValueError(f"{r.get('trial_id')}: non-physical source")
        out.append(r)
    return out

def manifest_params(m):
    p=m.get('parameters',{})
    dyn={k:float(p[k]) for k in CALIBRATABLE_DYNAMICS if finite(p.get(k))}
    hw={k:float(p[k]) for k in CALIBRATABLE_HARDWARE if finite(p.get(k))}
    return dyn,hw

def simulate_case(r,dynamics,hardware):
    w=MasterPiProductionV2(seed=7001,render=True,width=640,height=480,dynamics=dynamics,hardware=hardware,use_calibration_manifest=False)
    try:
        w.set_free_body_pose_for_reset('red_block',(float(r['red_x_m']),float(r['red_y_m']),0.015),math.radians(float(r['red_yaw_deg'])))
        # Give the externally placed block a brief physical settle before vision.
        w.step(duration_s=.12)
        detail=[]
        for action in ACTIONS:
            result=w.act(action); detail.append({'action':action,'ok':bool(result.ok),'reason':result.reason})
            if not result.ok: return False,detail
        state=w.state(); success=bool(state.get('bilateral_contact') and state.get('lifted') and state.get('stable'))
        return success,detail
    finally: w.close()

def metrics(rows,dyn,hw):
    outcomes=[]
    for r in rows:
        sim,detail=simulate_case(r,dyn,hw); physical=bool(r['physical_success'])
        outcomes.append({'trial_id':r['trial_id'],'physical_success':physical,'sim_success':sim,'match':sim==physical,'detail':detail})
    if not outcomes: return {'trials':0,'grasp_success_rate_gap':None,'task_outcome_disagreement_rate':None,'outcomes':[]}
    pr=sum(o['physical_success'] for o in outcomes)/len(outcomes); sr=sum(o['sim_success'] for o in outcomes)/len(outcomes)
    disagreement=sum(not o['match'] for o in outcomes)/len(outcomes)
    return {'trials':len(outcomes),'physical_success_rate':pr,'sim_success_rate':sr,'grasp_success_rate_gap':abs(sr-pr),'task_outcome_disagreement_rate':disagreement,'outcomes':outcomes}

def run(path,manifest_path=MANIFEST,split='holdout',write=False):
    rows=completed(path,split); m=load_manifest(manifest_path); dyn,hw=manifest_params(m)
    missing=[k for k in CALIBRATABLE_DYNAMICS if k not in dyn]+[k for k in CALIBRATABLE_HARDWARE if k not in hw]
    if missing: raise ValueError('manifest missing calibrated parameters: '+','.join(missing))
    out=metrics(rows,dyn,hw)
    if write:
        invalidate_manifest(m); res=m['results']; res['task_held_out_trials' if split=='holdout' else 'task_fit_trials']=out['trials']
        if split=='holdout': res['grasp_success_rate_gap']=out['grasp_success_rate_gap']; res['task_outcome_disagreement_rate']=out['task_outcome_disagreement_rate']
        m.setdefault('sources',{})['task_trials']=str(path); save_manifest(m,manifest_path)
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('trials',type=Path); ap.add_argument('--manifest',type=Path,default=MANIFEST); ap.add_argument('--split',choices=('fit','holdout'),default='holdout'); ap.add_argument('--write-manifest',action='store_true'); a=ap.parse_args()
    print(json.dumps(run(a.trials,a.manifest,a.split,a.write_manifest),ensure_ascii=False,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
