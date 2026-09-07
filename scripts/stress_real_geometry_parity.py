#!/usr/bin/env python3
"""Offline parity test: new REAL geometry vs the proven physical controller."""
from __future__ import annotations
import importlib.util, json, math, random, sys, types
from pathlib import Path
from harness.real_geometry import project_floor_pixel, forward_kinematics

ROOT=Path(__file__).resolve().parents[1]
# Geometry functions under test do not use cv2; stub it so Oracle's lean Python can import the reference.
sys.modules.setdefault('cv2', types.ModuleType('cv2'))
sys.path.insert(0, str(ROOT/'scripts'/'red_block'))
refp=ROOT/'scripts'/'red_block'/'physical_state_machine_reference.py'
spec=importlib.util.spec_from_file_location('ugrp_physical_ref',refp)
ref=importlib.util.module_from_spec(spec); sys.modules[spec.name]=ref; spec.loader.exec_module(ref)

rng=random.Random(20260829)
rows=[]; compared=0; max_xy=0.0; max_fk=0.0
for i in range(1000):
    pose={
        1:2000,
        3:rng.randint(600,1200),
        4:rng.randint(1300,2400),
        5:rng.randint(800,1900),
        6:rng.randint(900,2100),
    }
    nx=rng.uniform(.08,.92); ny=rng.uniform(.35,.95)
    a=project_floor_pixel(pose,nx=nx,ny=ny)
    b=ref.project_floor_pixel(pose,nx,ny)
    fka=forward_kinematics(pose); fkb=ref.forward_kinematics(pose,ref.CAMERA_LINK_CM)
    if fka is not None:
        fkerr=max(abs(fka.radius_cm-fkb.radius_cm),abs(fka.height_cm-fkb.height_cm),abs(fka.pitch_deg-fkb.pitch_deg))
        max_fk=max(max_fk,fkerr)
    if (a is None)!=(b is None):
        rows.append({'i':i,'kind':'validity_mismatch','pose':pose,'nx':nx,'ny':ny,'new':a is not None,'ref':b is not None})
        continue
    if a is None:
        continue
    compared+=1
    ref_left_cm,ref_forward_cm=b
    err=math.hypot(a.x_forward_m-ref_forward_cm/100.0,a.y_left_m-ref_left_cm/100.0)
    max_xy=max(max_xy,err)
    if err>1e-10:
        rows.append({'i':i,'kind':'xy_mismatch','err_m':err,'pose':pose,'nx':nx,'ny':ny})
print(json.dumps({'cases':1000,'valid_compared':compared,'failures':len(rows),'max_xy_error_m':max_xy,'max_fk_error_native_units':max_fk},indent=2))
if rows:
    print(json.dumps(rows[:5],indent=2)); raise SystemExit(1)
