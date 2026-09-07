#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,math,random,time
from pathlib import Path
import mujoco
import numpy as np
from sim.masterpi_physics import MasterPiPhysicsWorld, CARRY_POSE, SEARCH_POSE, ARM_JOINTS
from scripts.stress_semantic_map import set_scenario

def snap_pose(w, base_pose, yaw):
    pose=base_pose.copy();pose[0]=float(np.clip(yaw,-1.32,1.32))
    for n,v in zip(ARM_JOINTS,pose):w._set_joint_qpos(n,float(v))
    w._set_arm_target(pose);mujoco.mj_forward(w.model,w.data)

def one(seed,views_per_pose=9,width=320,height=240):
    rng=random.Random(seed*919393+41);w=MasterPiPhysicsWorld(seed=seed,width=width,height=height)
    try:
        set_scenario(w,rng)
        truth={c:w._body_pos(c+'_block')[:2] for c in ['red','blue','yellow']}
        seen={c:0 for c in truth};good={c:0 for c in truth};false_large=[];errors=[]
        yaws=np.linspace(-1.3,1.3,views_per_pose)
        for pose_name,pose in [('survey',CARRY_POSE),('object',SEARCH_POSE)]:
            for yaw in yaws:
                snap_pose(w,pose,float(yaw));d=w.scene_detections()
                for c,det in d.items():
                    if not det.get('visible'):continue
                    seen[c]+=1;xy=w._camera_pixel_ground_xy(det,plane_z=.025)
                    if xy is None:continue
                    err=float(np.linalg.norm(xy-truth[c]));errors.append((c,err,pose_name,float(yaw),det))
                    if err<=.055:good[c]+=1
                    if err>.14:false_large.append((c,err,pose_name,float(yaw),det))
        failures=[]
        if false_large:failures.append('SAME_COLOR_FALSE_POSITIVE')
        # Every object should have at least one geometrically correct view across the head range.
        for c in truth:
            if good[c]<1:failures.append(c.upper()+'_NO_GOOD_VIEW')
        return {'seed':seed,'ok':not failures,'failures':failures,'seen':seen,'good':good,
                'max_err':{c:round(max([e for cc,e,*_ in errors if cc==c],default=0),4) for c in truth},
                'false_large':false_large[:3]}
    finally:w.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--start',type=int,default=5000);ap.add_argument('--count',type=int,default=100);ap.add_argument('--out',required=True);ap.add_argument('--views',type=int,default=9);ap.add_argument('--width',type=int,default=320);ap.add_argument('--height',type=int,default=240);a=ap.parse_args()
    p=Path(a.out);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w') as f:
        for seed in range(a.start,a.start+a.count):
            r=one(seed,a.views,a.width,a.height);f.write(json.dumps(r,ensure_ascii=False)+'\n');f.flush()
            if not r['ok']:print(seed,'FAIL',r['failures'],r['seen'],r['good'],flush=True)
if __name__=='__main__':main()
