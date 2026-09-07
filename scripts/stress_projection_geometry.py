#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,math,random
from pathlib import Path
import mujoco
import numpy as np
from sim.masterpi_physics import MasterPiPhysicsWorld, CARRY_POSE, SEARCH_POSE, ARM_JOINTS

def project_pixel(w, point, camera='robot_cam'):
    cid=w._id(mujoco.mjtObj.mjOBJ_CAMERA,camera)
    pos=w.data.cam_xpos[cid].copy(); R=w.data.cam_xmat[cid].reshape(3,3).copy()
    local=R.T@(np.asarray(point)-pos)
    if local[2]>=-.03:return None
    fovy=math.radians(float(w.model.cam_fovy[cid])); ty=math.tan(fovy/2); tx=ty*w.width/w.height
    cx=.5 + (local[0]/(-local[2]))/(2*tx)
    cy=.5 - (local[1]/(-local[2]))/(2*ty)
    if not (0.01<cx<.99 and .01<cy<.99):return None
    return {'visible':True,'cx':float(cx),'cy':float(cy)}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--count',type=int,default=1000);ap.add_argument('--out',required=True);a=ap.parse_args()
    rng=random.Random(914221);w=MasterPiPhysicsWorld(seed=1,width=320,height=240,render=False)
    rows=[]
    try:
      attempts=0
      while len(rows)<a.count and attempts<a.count*20:
        attempts+=1
        rx=rng.uniform(-.2,.2);ry=rng.uniform(-.2,.2);byaw=rng.uniform(-math.pi,math.pi);hyaw=rng.uniform(-1.25,1.25)
        for n,v in [('base_x',rx),('base_y',ry),('base_yaw',byaw)]:w._set_joint_qpos(n,v)
        pose=(CARRY_POSE if rng.random()<.5 else SEARCH_POSE).copy();pose[0]=hyaw
        for n,v in zip(ARM_JOINTS,pose):w._set_joint_qpos(n,float(v))
        mujoco.mj_forward(w.model,w.data)
        # sample a ground object in front-ish world region; visibility is determined by projection
        p=np.array([rng.uniform(-.3,1.6),rng.uniform(-1.2,1.2),.025])
        det=project_pixel(w,p)
        if det is None:continue
        est=w._camera_pixel_ground_xy(det,plane_z=.025)
        if est is None:continue
        err=float(np.linalg.norm(est-p[:2]));rows.append({'err':err,'robot':[rx,ry,byaw],'head':hyaw,'point':p.tolist(),'pixel':[det['cx'],det['cy']]})
    finally:w.close()
    Path(a.out).write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
    errs=np.array([r['err'] for r in rows]);print('cases',len(rows),'max',float(errs.max()),'p99',float(np.quantile(errs,.99)),'mean',float(errs.mean()))
    if len(rows)<a.count or float(errs.max())>1e-6:raise SystemExit(1)
if __name__=='__main__':main()
