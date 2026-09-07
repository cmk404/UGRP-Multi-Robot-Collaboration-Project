#!/usr/bin/env python3
"""Safely record external physical measurements into a calibration JSONL row.

This command never touches the robot. It only edits one planned evidence row and
requires a physical measurement source. Values may be supplied as JSON so camera/
metrology tooling can pipe results in without hand-editing the dataset.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path
try:
    from scripts.benchmarks.masterpi_calibration_common import finite,load_jsonl,real_measurement_source,save_jsonl
except ModuleNotFoundError:
    from masterpi_calibration_common import finite,load_jsonl,real_measurement_source,save_jsonl

SCHEMAS={
 'servo':(('measured_delta_deg','settle_time_s'),()),
 'hand-eye':(('nx','bottom_ny','true_x_m','true_y_m'),()),
 'task':(('red_x_m','red_y_m','red_yaw_deg'),('physical_success',)),
}

def record(path:Path,trial_id:str,kind:str,values:dict,force=False):
    rows=load_jsonl(path); matches=[r for r in rows if r.get('trial_id')==trial_id]
    if len(matches)!=1: raise ValueError('trial_id not found uniquely')
    row=matches[0]; numeric,bool_fields=SCHEMAS[kind]
    source=values.get('measurement_source')
    if not real_measurement_source(source): raise ValueError('measurement_source must describe physical REAL evidence')
    for key in numeric:
        if key not in values or not finite(values[key]): raise ValueError(f'missing/non-finite {key}')
    for key in bool_fields:
        if not isinstance(values.get(key),bool): raise ValueError(f'{key} must be boolean')
    completed=all(row.get(k) is not None for k in numeric+bool_fields)
    if completed and not force: raise ValueError('trial already measured; use --force to replace')
    for key in numeric+bool_fields+('measurement_source','notes'):
        if key in values: row[key]=values[key]
    save_jsonl(path,rows); return row

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('kind',choices=tuple(SCHEMAS)); ap.add_argument('dataset',type=Path); ap.add_argument('trial_id'); ap.add_argument('--json',dest='json_path',type=Path,required=True); ap.add_argument('--force',action='store_true'); a=ap.parse_args()
    values=json.loads(a.json_path.read_text(encoding='utf-8')); print(json.dumps(record(a.dataset,a.trial_id,a.kind,values,a.force),ensure_ascii=False,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
