#!/usr/bin/env python3
"""Read-only physical MasterPi spatial observation diagnostic.

Never moves the robot.  Reads one camera frame and the last commanded arm pose,
then reports image detections and robot-base-relative metric projections when
the pose is stable enough to trust.
"""
from __future__ import annotations
import argparse, json, time
from harness.pi_camera import grab_snapshot
from harness.perception import detect_scene_bytes
from harness.real_geometry import metric_memory_entry, project_detection
from harness.real_pose import read_commanded_pose


def once(host: str, settle_s: float) -> dict:
    p1=read_commanded_pose(host=host)
    time.sleep(max(0.0,settle_s))
    p2=read_commanded_pose(host=host)
    stable=bool(p1 and p2 and p1.pose==p2.pose and (p2.age_s is None or p2.age_s>=settle_s))
    jpeg=grab_snapshot(host=host)
    scene=detect_scene_bytes(jpeg) or {}
    out={
        'host':host,
        'jpeg_bytes':len(jpeg),
        'pose':None if p2 is None else p2.pose,
        'pose_age_s':None if p2 is None else p2.age_s,
        'pose_stable':stable,
        'pose_writer':None if p2 is None else p2.last_writer,
        'detections':scene,
        'metric':{},
    }
    if p2:
        for color in ('red','yellow','blue'):
            det=scene.get(color)
            proj=project_detection(p2.pose,det)
            entry=metric_memory_entry(p2.pose,det,pose_age_s=p2.age_s,pose_stable=stable)
            out['metric'][color]={
                'candidate':None if proj is None else {
                    'position_xy':[proj.x_forward_m,proj.y_left_m],
                    'distance_m':proj.range_m,
                    'bearing_deg':proj.bearing_deg,
                    'ray_pitch_deg':proj.ray_pitch_deg,
                },
                'accepted':entry,
            }
    return out


def main():
    ap=argparse.ArgumentParser(description='Read-only REAL MasterPi spatial observation')
    ap.add_argument('--host',default='ugrp1')
    ap.add_argument('--settle',type=float,default=.22)
    args=ap.parse_args()
    print(json.dumps(once(args.host,args.settle),ensure_ascii=False,indent=2))
if __name__=='__main__': main()
