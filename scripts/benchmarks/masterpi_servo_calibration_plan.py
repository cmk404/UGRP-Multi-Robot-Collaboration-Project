#!/usr/bin/env python3
"""Create an external-measurement servo calibration plan.

This plan never drives hardware. Measure actual angular endpoint and settle time
from a fixed external camera/encoder after each commanded PWM step. Small steps
are included specifically to identify the physical deadband.
"""
from __future__ import annotations
import argparse, json, random
from pathlib import Path
try:
    from scripts.benchmarks.masterpi_calibration_common import load_jsonl, save_jsonl
except ModuleNotFoundError:
    from masterpi_calibration_common import load_jsonl, save_jsonl

SERVOS=(3,4,5,6)
DELTAS=(5,15,30,100,500)
REPEATS=3

def make_plan(seed=20260830):
    rows=[]; i=0
    for servo in SERVOS:
        for delta in DELTAS:
            for rep in range(REPEATS):
                i+=1; sign=1 if rep % 2 == 0 else -1
                rows.append({
                    'trial_id':f'servo-{i:03d}','split':'holdout' if rep==REPEATS-1 else 'fit',
                    'servo':servo,'start_pwm':1500,'target_pwm':1500+sign*delta,
                    'command_duration_s':max(0.10,delta/1200.0),
                    'measured_delta_deg':None,'settle_time_s':None,
                    'measurement_source':None,'notes':None,
                })
    random.Random(seed).shuffle(rows); return rows

def status(rows):
    done=[r for r in rows if r.get('measured_delta_deg') is not None and r.get('settle_time_s') is not None]
    return {'total_trials':len(rows),'completed_trials':len(done),
            'fit_completed':sum(r.get('split')=='fit' for r in done),
            'holdout_completed':sum(r.get('split')=='holdout' for r in done),
            'next_trial':next((r for r in rows if r not in done),None)}

def main():
    ap=argparse.ArgumentParser(); sub=ap.add_subparsers(dest='cmd',required=True)
    c=sub.add_parser('create'); c.add_argument('output',type=Path); c.add_argument('--seed',type=int,default=20260830)
    st=sub.add_parser('status'); st.add_argument('plan',type=Path); a=ap.parse_args()
    if a.cmd=='create': save_jsonl(a.output,make_plan(a.seed)); rows=load_jsonl(a.output)
    else: rows=load_jsonl(a.plan)
    print(json.dumps(status(rows),ensure_ascii=False,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
