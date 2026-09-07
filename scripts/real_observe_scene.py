#!/usr/bin/env python3
"""Read-only REAL scene observation skill. Never moves MasterPi."""
from __future__ import annotations
import argparse, json
from scripts.real_spatial_observe import once
from harness.real_carry import read_carry_evidence

def run(host:str='ugrp1')->dict:
    result=once(host,.22)
    metric={}
    for color,item in (result.get('metric') or {}).items():
        accepted=item.get('accepted') if isinstance(item,dict) else None
        if isinstance(accepted,dict): metric[color]=accepted
    carry=read_carry_evidence(host=host).public()
    return {
        'ok':True,
        'skill':'observe_scene',
        'command_status':'ACCEPTED',
        'execution_status':'COMPLETED',
        'outcome_status':'ACHIEVED',
        'detections':result.get('detections') or {},
        'metric_estimates':metric,
        'carry_evidence':carry,
        'pose_stable':bool(result.get('pose_stable')),
        'pose_writer':result.get('pose_writer'),
        'reference_frame':'robot_base_at_observation',
        'odometry_available':False,
        'note':'read-only camera/FK observation; no actuator command was sent',
    }

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--host',default='ugrp1'); args=ap.parse_args()
    print(json.dumps(run(args.host),ensure_ascii=False))
    return 0
if __name__=='__main__': raise SystemExit(main())
