#!/usr/bin/env python3
"""Fit only gripper/contact plant parameters to physical FIT pick outcomes.

Held-out rows are deliberately never read here. Final task parity is evaluated
by evaluate_masterpi_task_parity.py after fitting.
"""
from __future__ import annotations
import sys
import argparse,json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    from scripts.benchmarks.masterpi_calibration_common import MANIFEST,invalidate_manifest,load_manifest,save_manifest
except ModuleNotFoundError:
    from masterpi_calibration_common import MANIFEST,invalidate_manifest,load_manifest,save_manifest
try:
    from scripts.benchmarks.evaluate_masterpi_task_parity import completed,manifest_params,metrics
except ModuleNotFoundError:
    from evaluate_masterpi_task_parity import completed,manifest_params,metrics

KP=(150.0,300.0,450.0,700.0,1000.0,1500.0)
MU=(0.6,1.2,2.0,3.4,5.0,7.0)

def run(path,manifest_path=MANIFEST,write=False):
    rows=completed(path,'fit')
    if len(rows)<10: raise ValueError('need at least 10 physical fit pick trials')
    m=load_manifest(manifest_path); dyn,base_hw=manifest_params(m)
    required_before=('wheel_radius_m','wheelbase_m','track_m','servo_deadband_pwm','servo_rate_pwm_per_s','camera_link_cm','camera_z_offset_cm','camera_pitch_offset_deg','servo6_center_pwm','block_mass_kg','block_floor_friction')
    missing=[k for k in required_before if k not in base_hw]
    if missing or len(dyn)<8: raise ValueError('calibrate chassis/static/servo/hand-eye before gripper fit')
    best=None
    for kp in KP:
        for mu in MU:
            hw=dict(base_hw); hw['gripper_position_kp']=kp; hw['gripper_finger_friction']=mu
            met=metrics(rows,dyn,hw); loss=met['task_outcome_disagreement_rate'] + .35*met['grasp_success_rate_gap']
            if best is None or loss < best[0]: best=(loss,kp,mu,met)
    _,kp,mu,met=best
    out={'parameters':{'gripper_position_kp':kp,'gripper_finger_friction':mu},'fit':{k:v for k,v in met.items() if k!='outcomes'}}
    if write:
        invalidate_manifest(m); m['parameters'].update(out['parameters']); m['results']['task_fit_trials']=met['trials']; m.setdefault('sources',{})['task_fit_trials']=str(path); save_manifest(m,manifest_path)
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('trials',type=Path); ap.add_argument('--manifest',type=Path,default=MANIFEST); ap.add_argument('--write-manifest',action='store_true'); a=ap.parse_args(); print(json.dumps(run(a.trials,a.manifest,a.write_manifest),indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
