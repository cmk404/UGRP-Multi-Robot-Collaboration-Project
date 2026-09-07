#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
try:
    from scripts.benchmarks.masterpi_calibration_common import MANIFEST, finite, invalidate_manifest, load_jsonl, load_manifest, real_measurement_source, save_manifest
except ModuleNotFoundError:
    from masterpi_calibration_common import MANIFEST, finite, invalidate_manifest, load_jsonl, load_manifest, real_measurement_source, save_manifest

PPD=2000.0/180.0

def complete_rows(path: Path):
    rows=[]
    for r in load_jsonl(path):
        if not (finite(r.get('measured_delta_deg')) and finite(r.get('settle_time_s'))): continue
        if not real_measurement_source(r.get('measurement_source')): raise ValueError(f"{r.get('trial_id')}: non-physical measurement source")
        rows.append(r)
    return rows

def predict(row, deadband, rate):
    delta=abs(float(row['target_pwm'])-float(row['start_pwm']))
    actual_pwm=0.0 if delta <= deadband else delta
    return actual_pwm/PPD, max(float(row['command_duration_s']), delta/max(rate,1e-6))

def metrics(rows, deadband, rate):
    e=[]; t=[]
    for r in rows:
        pe,pt=predict(r,deadband,rate); ae=abs(float(r['measured_delta_deg'])); at=float(r['settle_time_s'])
        e.append(abs(pe-ae)); t.append(abs(pt-at)/max(at,0.05))
    return {'trials':len(rows),'servo_endpoint_mae_deg':float(np.mean(e)) if e else None,
            'servo_settle_time_relative_error':float(np.mean(t)) if t else None}

def fit(rows):
    best=None
    for deadband in range(0,81):
        for rate in np.geomspace(200.0,3000.0,80):
            m=metrics(rows,float(deadband),float(rate))
            loss=(m['servo_endpoint_mae_deg']/3.0)**2+(m['servo_settle_time_relative_error']/0.2)**2
            if best is None or loss < best[0]: best=(loss,float(deadband),float(rate))
    assert best is not None
    return best[1],best[2]

def run(path:Path,manifest_path:Path=MANIFEST,write=False):
    rows=complete_rows(path); fit_rows=[r for r in rows if r.get('split')!='holdout']; hold=[r for r in rows if r.get('split')=='holdout']
    if len(fit_rows)<20 or len({int(r['servo']) for r in fit_rows})<4: raise ValueError('need >=20 physical fit trials spanning servos 3/4/5/6')
    deadband,rate=fit(fit_rows); fm=metrics(fit_rows,deadband,rate); hm=metrics(hold,deadband,rate)
    out={'parameters':{'servo_deadband_pwm':deadband,'servo_rate_pwm_per_s':rate},'fit':fm,'holdout':hm}
    if write:
        m=load_manifest(manifest_path); invalidate_manifest(m); m['parameters'].update(out['parameters'])
        m['results']['servo_fit_trials']=fm['trials']; m['results']['servo_held_out_trials']=hm['trials']
        m['results']['servo_endpoint_mae_deg']=hm['servo_endpoint_mae_deg']; m['results']['servo_settle_time_relative_error']=hm['servo_settle_time_relative_error']
        m.setdefault('sources',{})['servo_trials']=str(path); save_manifest(m,manifest_path)
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('trials',type=Path); ap.add_argument('--manifest',type=Path,default=MANIFEST); ap.add_argument('--write-manifest',action='store_true'); a=ap.parse_args()
    print(json.dumps(run(a.trials,a.manifest,a.write_manifest),indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
