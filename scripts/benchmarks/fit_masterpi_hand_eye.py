#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import numpy as np
try:
    from scripts.benchmarks.masterpi_calibration_common import MANIFEST,finite,invalidate_manifest,load_jsonl,load_manifest,real_measurement_source,save_manifest
except ModuleNotFoundError:
    from masterpi_calibration_common import MANIFEST,finite,invalidate_manifest,load_jsonl,load_manifest,real_measurement_source,save_manifest

LINK1,LINK2,LINK3=9.30,6.50,6.20
PPD=2000.0/180.0
DEV={3:54,4:53,5:89,6:64}
VFOV=48.0
HFOV=math.degrees(2*math.atan(math.tan(math.radians(VFOV/2))*4/3))
BOUNDS={'camera_link_cm':(4.0,10.0),'camera_z_offset_cm':(0.0,6.0),'camera_pitch_offset_deg':(-12.0,12.0),'servo6_center_pwm':(1400.0,1600.0)}

def project(r,p):
    p3=float(r['servo3_pwm'])-DEV[3]; p4=float(r['servo4_pwm'])-DEV[4]; p5=float(r['servo5_pwm'])-DEV[5]
    t3=(p3-1500)/PPD; t4=(p4-1500)/PPD; t5=90-(p5-1500)/PPD
    pitch=t3+t5-t4
    shoulder=math.radians(t5); forearm=math.radians(t5-t4); tool=math.radians(pitch)
    radius=LINK2*math.cos(shoulder)+LINK3*math.cos(forearm)+p['camera_link_cm']*math.cos(tool)
    height=LINK1+LINK2*math.sin(shoulder)+LINK3*math.sin(forearm)+p['camera_link_cm']*math.sin(tool)+p['camera_z_offset_cm']
    pixel_down=(float(r['bottom_ny'])-.5)*VFOV
    ray=pitch+p['camera_pitch_offset_deg']-pixel_down
    if height<=.5 or not -88<ray<-3: return None
    ground=radius+height/math.tan(math.radians(-ray))
    bearing=(float(r['servo6_pwm'])-p['servo6_center_pwm'])/PPD + (.5-float(r['nx']))*HFOV
    a=math.radians(bearing)
    return np.array([ground*math.cos(a)/100,ground*math.sin(a)/100],dtype=float)

def rows_complete(path):
    out=[]
    for r in load_jsonl(path):
        if not all(finite(r.get(k)) for k in ('nx','bottom_ny','true_x_m','true_y_m')): continue
        if not real_measurement_source(r.get('measurement_source')): raise ValueError(f"{r.get('trial_id')}: non-physical source")
        if not (0<=float(r['nx'])<=1 and 0<=float(r['bottom_ny'])<=1): raise ValueError('normalized pixels out of range')
        out.append(r)
    return out

def metric(rows,p):
    errs=[]
    for r in rows:
        pred=project(r,p)
        if pred is None: errs.append(1.0); continue
        true=np.array([float(r['true_x_m']),float(r['true_y_m'])]); errs.append(float(np.linalg.norm(pred-true)))
    return float(np.mean(errs)) if errs else None

def clip(p): return {k:max(BOUNDS[k][0],min(BOUNDS[k][1],float(v))) for k,v in p.items()}

def fit(rows,initial):
    cur=clip(initial); best=metric(rows,cur)
    steps={'camera_link_cm':[1,.4,.15,.05],'camera_z_offset_cm':[1,.4,.15,.05],'camera_pitch_offset_deg':[3,1,.35,.1],'servo6_center_pwm':[40,15,5,1]}
    for stage in range(4):
        improved=True
        while improved:
            improved=False
            for k in cur:
                for sign in (-1,1):
                    cand=dict(cur); cand[k]+=sign*steps[k][stage]; cand=clip(cand); score=metric(rows,cand)
                    if score is not None and (best is None or score+1e-9<best): cur,best=cand,score; improved=True
    return cur

def run(path,manifest_path=MANIFEST,write=False):
    rows=rows_complete(path); fit_rows=[r for r in rows if r.get('split')!='holdout']; hold=[r for r in rows if r.get('split')=='holdout']
    if len(fit_rows)<12 or len(hold)<8: raise ValueError('need >=12 fit and >=8 held-out physical hand-eye observations')
    m=load_manifest(manifest_path); saved=m.get('parameters',{})
    initial={'camera_link_cm':saved.get('camera_link_cm') or 7.0,'camera_z_offset_cm':saved.get('camera_z_offset_cm') or 2.5,'camera_pitch_offset_deg':saved.get('camera_pitch_offset_deg') or 0.0,'servo6_center_pwm':saved.get('servo6_center_pwm') or 1500.0}
    params=fit(fit_rows,initial); fm=metric(fit_rows,params); hm=metric(hold,params)
    out={'parameters':params,'fit':{'trials':len(fit_rows),'hand_eye_position_mae_m':fm},'holdout':{'trials':len(hold),'hand_eye_position_mae_m':hm}}
    if write:
        invalidate_manifest(m); m['parameters'].update(params); m['results']['hand_eye_fit_trials']=len(fit_rows); m['results']['hand_eye_held_out_trials']=len(hold); m['results']['hand_eye_position_mae_m']=hm; m.setdefault('sources',{})['hand_eye_trials']=str(path); save_manifest(m,manifest_path)
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('trials',type=Path); ap.add_argument('--manifest',type=Path,default=MANIFEST); ap.add_argument('--write-manifest',action='store_true'); a=ap.parse_args(); print(json.dumps(run(a.trials,a.manifest,a.write_manifest),indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
