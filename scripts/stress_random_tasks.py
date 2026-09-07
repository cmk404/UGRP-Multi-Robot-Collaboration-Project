#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,math,random,time
from pathlib import Path
import numpy as np
from sim.masterpi_physics import MasterPiPhysicsWorld
from scripts.stress_semantic_map import set_scenario

def run_one(seed:int,target:str,pre_scan:bool=False):
    rng=random.Random(seed*1000003+17)
    w=MasterPiPhysicsWorld(seed=seed,width=320,height=240);w.set_speed_multiplier(3)
    try:
        scenario=set_scenario(w,rng)
        actions=[]
        if pre_scan: actions.append('scan_world')
        actions += [f'map_{target}','search','track','approach','pick','carry',f'place_on_{target}']
        trace=[];failure=None
        for a in actions:
            t=time.perf_counter();r=w.act(a);dt=time.perf_counter()-t
            st=w.state();trace.append({'action':a,'ok':r.ok,'reason':r.reason,'dt':round(dt,4),'robot_xy':st.get('robot_xy'),'base_yaw':st.get('base_yaw'),'bilateral':st.get('bilateral_contact'),'lifted':st.get('lifted'),'stable':st.get('stable')})
            if not r.ok:
                failure={'action':a,'reason':r.reason};break
        st=w.state();red=np.asarray(st['red_xyz']);tar=np.asarray(st[target+'_xyz']);xyerr=float(np.linalg.norm(red[:2]-tar[:2]));zdiff=float(red[2]-tar[2]);stack=xyerr<=.04 and .038<=zdiff<=.066
        mem=st.get('spatial_memory') or {}; target_mem=mem.get(target)
        target_mem_err=None
        if target_mem and target_mem.get('position_xy'):
            target_mem_err=float(np.linalg.norm(np.asarray(target_mem['position_xy'])-tar[:2]))
        failures=[]
        if failure: failures.append('ACTION_FAIL:'+failure['action'])
        if not stack: failures.append('STACK_POSTCONDITION')
        if target_mem_err is not None and target_mem_err>.055: failures.append('TARGET_MEMORY_DRIFT')
        return {'seed':seed,'target':target,'pre_scan':pre_scan,'ok':not failures,'failures':failures,'failure':failure,
                'stack':stack,'xy_error_m':round(xyerr,4),'z_diff_m':round(zdiff,4),'target_memory_error_m':None if target_mem_err is None else round(target_mem_err,4),
                'scenario':scenario,'trace':trace,'memory':{k:{'position_xy':v.get('position_xy'),'relation':v.get('relation'),'confidence':v.get('confidence')} for k,v in mem.items()}}
    finally:w.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--start',type=int,default=7000);ap.add_argument('--count',type=int,default=20);ap.add_argument('--out',required=True);ap.add_argument('--pre-scan',action='store_true');a=ap.parse_args()
    p=Path(a.out);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w') as f:
        for i,seed in enumerate(range(a.start,a.start+a.count)):
            target='blue' if i%2==0 else 'yellow';r=run_one(seed,target,a.pre_scan);f.write(json.dumps(r,ensure_ascii=False)+'\n');f.flush()
            print(seed,target,'PASS' if r['ok'] else 'FAIL',r['failures'],r['failure'],'xy',r['xy_error_m'],flush=True)
if __name__=='__main__':main()
