#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
try:
    from scripts.benchmarks.masterpi_calibration_common import MANIFEST, finite, load_manifest, save_manifest, invalidate_manifest, real_measurement_source
except ModuleNotFoundError:
    from masterpi_calibration_common import MANIFEST, finite, load_manifest, save_manifest, invalidate_manifest, real_measurement_source

FIELDS = {
    'wheel_radius_m': (0.015, 0.08),
    'wheelbase_m': (0.05, 0.30),
    'track_m': (0.05, 0.30),
    'block_mass_kg': (0.001, 0.5),
    'block_floor_friction': (0.05, 4.0),
}

def apply(path: Path, manifest_path: Path = MANIFEST) -> dict:
    evidence=json.loads(path.read_text(encoding='utf-8'))
    if evidence.get('hardware_unit') != 'ugrp1': raise ValueError('hardware_unit must be ugrp1')
    if not real_measurement_source(evidence.get('measurement_source')): raise ValueError('measurement_source must describe a physical measurement')
    vals={}
    for key,(lo,hi) in FIELDS.items():
        value=evidence.get(key)
        if not finite(value) or not lo <= float(value) <= hi: raise ValueError(f'{key} missing/out of bounds')
        vals[key]=float(value)
    m=load_manifest(manifest_path); invalidate_manifest(m)
    m.setdefault('parameters',{}).update(vals)
    m.setdefault('sources',{})['static_measurements']=str(path)
    save_manifest(m,manifest_path)
    return vals

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('measurements',type=Path); ap.add_argument('--manifest',type=Path,default=MANIFEST); a=ap.parse_args()
    print(json.dumps(apply(a.measurements,a.manifest),indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
