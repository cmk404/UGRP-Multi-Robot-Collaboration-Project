#!/usr/bin/env python3
"""Create camera/arm hand-eye trials requiring externally measured floor truth."""
from __future__ import annotations
import argparse,json,random
from pathlib import Path
try:
    from scripts.benchmarks.masterpi_calibration_common import load_jsonl,save_jsonl
except ModuleNotFoundError:
    from masterpi_calibration_common import load_jsonl,save_jsonl

# Proven collision-free camera poses from the current REAL pickup envelope.
ARM_POSES=((740,2320,1320),(650,2250,1400),(550,2150,1500),(500,2050,1580))
PANS=(1350,1500,1650)

def make_plan(seed=20260830):
    rows=[]; i=0
    for pose in ARM_POSES:
        for pan in PANS:
            for rep in range(2):
                i+=1
                rows.append({'trial_id':f'handeye-{i:03d}','split':'holdout' if rep else 'fit',
                    'servo3_pwm':pose[0],'servo4_pwm':pose[1],'servo5_pwm':pose[2],'servo6_pwm':pan,
                    'nx':None,'bottom_ny':None,'true_x_m':None,'true_y_m':None,
                    'measurement_source':None,'notes':None})
    random.Random(seed).shuffle(rows); return rows

def status(rows):
    req=('nx','bottom_ny','true_x_m','true_y_m')
    done=[r for r in rows if all(r.get(k) is not None for k in req)]
    return {'total_trials':len(rows),'completed_trials':len(done),'fit_completed':sum(r['split']=='fit' for r in done),'holdout_completed':sum(r['split']=='holdout' for r in done),'next_trial':next((r for r in rows if r not in done),None)}

def main():
    ap=argparse.ArgumentParser(); sub=ap.add_subparsers(dest='cmd',required=True)
    c=sub.add_parser('create'); c.add_argument('output',type=Path); c.add_argument('--seed',type=int,default=20260830)
    st=sub.add_parser('status'); st.add_argument('plan',type=Path); a=ap.parse_args()
    if a.cmd=='create': save_jsonl(a.output,make_plan(a.seed)); rows=load_jsonl(a.output)
    else: rows=load_jsonl(a.plan)
    print(json.dumps(status(rows),ensure_ascii=False,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
