#!/usr/bin/env python3
"""Create physical pick cases reserved for gripper fit and final task hold-out."""
from __future__ import annotations
import argparse,json,random
from pathlib import Path
try:
    from scripts.benchmarks.masterpi_calibration_common import load_jsonl,save_jsonl
except ModuleNotFoundError:
    from masterpi_calibration_common import load_jsonl,save_jsonl

def make_plan(seed=20260830):
    rng=random.Random(seed); rows=[]
    # Fit cases estimate gripper/contact parameters; held-out cases are untouched
    # until the final promotion gate. Coordinates are robot-base floor truth from
    # an external grid/overhead reference before each physical trial.
    for i in range(32):
        split='fit' if i<12 else 'holdout'
        x=rng.uniform(.38,.58); y=rng.uniform(-.07,.07); yaw=rng.uniform(-35,35)
        rows.append({'trial_id':f'pick-{i+1:03d}','split':split,'red_x_m':round(x,4),'red_y_m':round(y,4),'red_yaw_deg':round(yaw,2),
                     'physical_success':None,'measurement_source':None,'notes':None})
    rng.shuffle(rows); return rows

def status(rows):
    done=[r for r in rows if isinstance(r.get('physical_success'),bool)]
    return {'total_trials':len(rows),'completed_trials':len(done),'fit_completed':sum(r['split']=='fit' for r in done),'holdout_completed':sum(r['split']=='holdout' for r in done),'next_trial':next((r for r in rows if r not in done),None)}

def main():
    ap=argparse.ArgumentParser(); sub=ap.add_subparsers(dest='cmd',required=True)
    c=sub.add_parser('create'); c.add_argument('output',type=Path); c.add_argument('--seed',type=int,default=20260830)
    st=sub.add_parser('status'); st.add_argument('plan',type=Path); a=ap.parse_args()
    if a.cmd=='create': save_jsonl(a.output,make_plan(a.seed)); rows=load_jsonl(a.output)
    else: rows=load_jsonl(a.plan)
    print(json.dumps(status(rows),ensure_ascii=False,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
