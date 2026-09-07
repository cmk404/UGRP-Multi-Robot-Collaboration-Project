#!/usr/bin/env python3
"""Run exactly one planned physical chassis calibration pulse.

Default is preview-only. --execute sends one bounded primitive and then waits the
planned coast interval; it never chains trials because the physical robot must
be returned to the external start reference between measurements.
"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
RED=ROOT/'scripts'/'red_block'
if str(RED) not in sys.path: sys.path.insert(0,str(RED))
from deploy import DEFAULT_HOST, deploy_and_run  # noqa: E402

MOTION_MAP={
    'forward':'forward','backward':'backward','left':'left','right':'right',
    'rotate-left':'rotate-left','rotate-right':'rotate-right',
}

def load(path:Path):
    return [json.loads(x) for x in path.read_text(encoding='utf-8').splitlines() if x.strip()]

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('plan',type=Path)
    ap.add_argument('--trial-id')
    ap.add_argument('--host',default=DEFAULT_HOST)
    ap.add_argument('--execute',action='store_true')
    args=ap.parse_args()
    rows=load(args.plan)
    if args.trial_id:
        matches=[r for r in rows if r.get('trial_id')==args.trial_id]
        if len(matches)!=1: raise SystemExit('trial id not found uniquely')
        row=matches[0]
    else:
        row=next((r for r in rows if any(r.get(k) is None for k in ('dx_m','dy_m','dyaw_deg'))),None)
        if row is None: raise SystemExit('all trials already measured')
    motion=MOTION_MAP[row['direction']]
    payload={
        'trial_id':row['trial_id'],'split':row['split'],'motion':motion,
        'speed':int(row['command']),'drive_s':float(row['drive_s']),
        'coast_s':float(row['coast_s']),'execute':bool(args.execute),
    }
    print(json.dumps(payload,ensure_ascii=False,indent=2),flush=True)
    if not args.execute:
        return 0
    code=deploy_and_run(
        'primitive.py',
        ['--motion',motion,'--speed',str(int(row['command'])),'--duration',str(float(row['drive_s']))],
        host=args.host,
    )
    if code != 0:
        return int(code)
    time.sleep(float(row['coast_s']))
    print(json.dumps({'trial_id':row['trial_id'],'physical_command_completed':True,'measurement_still_required':True},ensure_ascii=False),flush=True)
    return 0
if __name__=='__main__': raise SystemExit(main())
