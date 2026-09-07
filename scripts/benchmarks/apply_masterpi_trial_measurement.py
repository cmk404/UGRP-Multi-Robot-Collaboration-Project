#!/usr/bin/env python3
"""Apply analyzer/manual measurements to one planned MasterPi chassis trial."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path

REQUIRED=("dx_m","dy_m","dyaw_deg")
OPTIONAL=("peak_speed_mps","stop_distance_m","measurement_source","notes")

def load_rows(path: Path)->list[dict]:
    return [json.loads(x) for x in path.read_text(encoding='utf-8').splitlines() if x.strip()]

def save_rows(path: Path, rows:list[dict])->None:
    with path.open('w',encoding='utf-8') as f:
        for r in rows: f.write(json.dumps(r,ensure_ascii=False,sort_keys=True)+'\n')

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('plan',type=Path)
    ap.add_argument('trial_id')
    ap.add_argument('--measurement-json',type=Path)
    for k in REQUIRED+OPTIONAL[:2]: ap.add_argument('--'+k.replace('_','-'),type=float)
    ap.add_argument('--measurement-source')
    ap.add_argument('--notes')
    ap.add_argument('--force',action='store_true')
    args=ap.parse_args()
    rows=load_rows(args.plan)
    matches=[r for r in rows if r.get('trial_id')==args.trial_id]
    if len(matches)!=1: raise SystemExit(f'trial_id {args.trial_id!r} not found uniquely')
    row=matches[0]
    if not args.force and any(row.get(k) is not None for k in REQUIRED):
        raise SystemExit('trial already has measurements; use --force to replace')
    values={}
    if args.measurement_json:
        values.update(json.loads(args.measurement_json.read_text(encoding='utf-8')))
    for k in REQUIRED+OPTIONAL[:2]:
        v=getattr(args,k)
        if v is not None: values[k]=v
    if args.measurement_source is not None: values['measurement_source']=args.measurement_source
    if args.notes is not None: values['notes']=args.notes
    for k in REQUIRED:
        if k not in values or not isinstance(values[k],(int,float)) or not math.isfinite(float(values[k])):
            raise SystemExit(f'missing/non-finite {k}')
    for k in REQUIRED+OPTIONAL:
        if k in values: row[k]=values[k]
    save_rows(args.plan,rows)
    print(json.dumps(row,ensure_ascii=False,indent=2))
    return 0
if __name__=='__main__': raise SystemExit(main())
