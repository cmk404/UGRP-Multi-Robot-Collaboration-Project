#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math, os, random, time
from pathlib import Path
import mujoco
import numpy as np
from sim.masterpi_physics import MasterPiPhysicsWorld


def body_free_qpos(world, body_name: str):
    bid=world._id(mujoco.mjtObj.mjOBJ_BODY, body_name)
    jid=int(world.model.body_jntadr[bid])
    qa=int(world.model.jnt_qposadr[jid])
    return qa, int(world.model.jnt_dofadr[jid])

def set_free_body(world, body_name: str, xyz):
    qa,da=body_free_qpos(world,body_name)
    world.data.qpos[qa:qa+3]=xyz
    world.data.qpos[qa+3:qa+7]=[1,0,0,0]
    world.data.qvel[da:da+6]=0

def set_scenario(world, rng: random.Random):
    # Valid workcell randomization: broad enough to alter bearings and ranges,
    # conservative enough to avoid spawning blocks inside furniture/walls.
    rx=rng.uniform(-.10,.10); ry=rng.uniform(-.10,.10); yaw=rng.uniform(math.radians(-28),math.radians(28))
    world._set_joint_qpos('base_x',rx); world._set_joint_qpos('base_y',ry); world._set_joint_qpos('base_yaw',yaw)
    for a,v in [('p_base_x',rx),('p_base_y',ry),('p_base_yaw',yaw),('a_base_x',0),('a_base_y',0),('a_base_yaw',0)]: world._set_actuator(a,v)
    world.odom_xy[:]=[rx,ry]; world.base_yaw=yaw
    red=(rng.uniform(.46,.64),rng.uniform(-.28,.28),.025)
    blue=(rng.uniform(.60,.88),rng.uniform(-.42,-.10),.025)
    yellow=(rng.uniform(.64,.94),rng.uniform(.10,.42),.025)
    set_free_body(world,'red_block',red); set_free_body(world,'blue_block',blue); set_free_body(world,'yellow_block',yellow)
    world.spatial_memory.clear(); world.obstacle_cells.clear(); world._memory_observation_id=0; world.blue_map_xy=None; world.yellow_map_xy=None
    mujoco.mj_forward(world.model,world.data); world.step_physics(35)
    return {'robot':[rx,ry,yaw],'red':red,'blue':blue,'yellow':yellow}

def static_obstacle_boxes(world):
    """All visible static box obstacles for evaluation only."""
    out=[]
    skip={'floor','delivery_blue','delivery_yellow'}
    for gid in range(world.model.ngeom):
        if int(world.model.geom_bodyid[gid]) != 0:
            continue
        name=mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_GEOM,gid) or f'geom{gid}'
        if name in skip or int(world.model.geom_type[gid]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            continue
        pos=world.data.geom_xpos[gid].copy(); size=world.model.geom_size[gid].copy()
        # Only geometry that rises enough above the floor to be an obstacle.
        if float(pos[2]+size[2]) < .065:
            continue
        out.append((name,pos,size))
    return out

def obstacle_error_to_static_boxes(xy, boxes):
    x,y=map(float,xy); best=float('inf')
    for _,pos,size in boxes:
        dx=max(abs(x-float(pos[0]))-float(size[0]),0.0)
        dy=max(abs(y-float(pos[1]))-float(size[1]),0.0)
        best=min(best,math.hypot(dx,dy))
    return best

def nearest_error(xy, truth):
    if truth.size==0:return None
    return float(np.min(np.linalg.norm(truth-np.asarray(xy,dtype=float),axis=1)))

def run_one(seed:int,width:int,height:int):
    rng=random.Random(seed*1000003+17)
    w=MasterPiPhysicsWorld(seed=seed,width=width,height=height); w.set_speed_multiplier(3)
    try:
        scenario=set_scenario(w,rng); start_yaw=w.base_yaw; t=time.perf_counter(); r=w.act('scan_world'); elapsed=time.perf_counter()-t
        st=w.state(); sm=st.get('semantic_map') or {}; mem=st.get('spatial_memory') or {}
        true={'red':w._body_pos('red_block')[:2],'blue':w._body_pos('blue_block')[:2],'yellow':w._body_pos('yellow_block')[:2]}
        object_err={}
        for c in ['red','blue','yellow']:
            m=mem.get(c); object_err[c]=None if not m or not m.get('position_xy') else float(np.linalg.norm(np.asarray(m['position_xy'])-true[c]))
        obs=sm.get('obstacles') or []; boxes=static_obstacle_boxes(w)
        obs_err=[obstacle_error_to_static_boxes(o['position_xy'],boxes) for o in obs]
        base_delta=abs((w.base_yaw-start_yaw+math.pi)%(2*math.pi)-math.pi)
        finite=all(np.isfinite(v).all() for v in [np.asarray(o.get('position_xy',[np.nan,np.nan])) for o in obs])
        # Criteria deliberately strict enough to expose regressions, not claim full SLAM accuracy.
        failures=[]
        if not r.ok: failures.append('SCAN_ACTION_FAIL')
        if base_delta>math.radians(1.5): failures.append('CHASSIS_MOVED_DURING_SCAN')
        if len(sm.get('drop_zones') or [])!=2: failures.append('DROP_ZONE_COUNT')
        if len(obs)<20: failures.append('OBSTACLE_COVERAGE_LOW')
        if not finite: failures.append('NONFINITE_MAP')
        if obs_err and float(np.quantile(obs_err,.95))>.055: failures.append('OBSTACLE_MAP_DRIFT')
        # At least blue/yellow should be mapped; red may be occluded/too close in a survey scan.
        for c in ['blue','yellow']:
            if object_err[c] is None: failures.append(f'{c.upper()}_MISSING')
            elif object_err[c]>.055: failures.append(f'{c.upper()}_MAP_ERROR')
        return {
            'seed':seed,'ok':not failures,'failures':failures,'elapsed_s':round(elapsed,4),'scenario':scenario,
            'object_error_m':{k:(None if v is None else round(v,4)) for k,v in object_err.items()},
            'objects':list(mem.keys()),'obstacles':len(obs),'zones':len(sm.get('drop_zones') or []),
            'obstacle_p50_error_m':None if not obs_err else round(float(np.median(obs_err)),4),
            'obstacle_p80_error_m':None if not obs_err else round(float(np.quantile(obs_err,.8)),4),
            'obstacle_p95_error_m':None if not obs_err else round(float(np.quantile(obs_err,.95)),4),
            'base_delta_deg':round(math.degrees(base_delta),3),
        }
    finally:w.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--start',type=int,default=1000);ap.add_argument('--count',type=int,default=50);ap.add_argument('--out',required=True);ap.add_argument('--width',type=int,default=320);ap.add_argument('--height',type=int,default=240);a=ap.parse_args()
    path=Path(a.out);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w') as f:
        for seed in range(a.start,a.start+a.count):
            row=run_one(seed,a.width,a.height);f.write(json.dumps(row,ensure_ascii=False)+'\n');f.flush()
            print(seed,'PASS' if row['ok'] else 'FAIL',row['failures'],'obs',row['obstacles'],'err',row['object_error_m'],flush=True)
if __name__=='__main__':main()
