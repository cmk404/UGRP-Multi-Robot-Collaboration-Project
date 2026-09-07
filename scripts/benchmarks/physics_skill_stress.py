#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, statistics, sys, time
from pathlib import Path

os.environ.setdefault('MUJOCO_GL','egl')
ROOT=Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))

SEQ=['map_yellow','search','track','approach','pick','carry','place_on_yellow']

def compact_state(st):
    keys=['robot_xy','red_xyz','yellow_xyz','grip_xyz','grip_error_m','bilateral_contact','lifted','stable','stable_steps','gripper_qpos','arm_qpos']
    return {k:st.get(k) for k in keys}

def run_episode(seed:int,speed:float,width:int,height:int):
    w=MasterPiPhysicsWorld(seed=seed,width=width,height=height,render=True)
    w.set_speed_multiplier(speed)
    rows=[]; ep0=time.perf_counter(); fail=None
    try:
        # Reset is implicit in construction. Optional recovery mirrors the
        # deterministic executive: one full reacquisition after a grasp failure.
        queue=list(SEQ); grasp_retries=0
        while queue:
            action=queue.pop(0)
            t=time.perf_counter(); r=w.act(action); dt=time.perf_counter()-t
            st=w.state()
            tobs=time.perf_counter(); red=w.camera_red_detection(); red_obs_s=time.perf_counter()-tobs
            tobs=time.perf_counter(); yellow=w.camera_yellow_detection(); yellow_obs_s=time.perf_counter()-tobs
            row={
                'seed':seed,'action':action,'elapsed_s':round(dt,4),'red_obs_s':round(red_obs_s,4),'yellow_obs_s':round(yellow_obs_s,4),'ok':bool(r.ok),'reason':r.reason,
                'red_visible':bool(red.get('visible')),'red_cx':red.get('cx'),'red_cy':red.get('cy'),'red_area':red.get('area_ratio'),
                'yellow_visible':bool(yellow.get('visible')),'yellow_cx':yellow.get('cx'),'yellow_cy':yellow.get('cy'),'yellow_area':yellow.get('area_ratio'),
                **compact_state(st),
            }
            rows.append(row)
            if not r.ok:
                grasp_fail = action in {'pick','carry'} and ('grasp postcondition failed' in r.reason or 'slipped from the gripper' in r.reason)
                if RECOVER_ON_GRASP and grasp_fail and grasp_retries < 1:
                    grasp_retries += 1
                    chain=['search','track','approach','pick','carry']
                    # If pick failed, original queue already begins with carry;
                    # replace it with a complete reacquisition + carry.
                    if action == 'pick' and queue and queue[0] == 'carry': queue.pop(0)
                    queue[:0]=chain
                    continue
                fail=action; break
        st=w.state(); red=st.get('red_xyz'); yellow=st.get('yellow_xyz')
        stack=False
        if red and yellow:
            dx=float(red[0])-float(yellow[0]); dy=float(red[1])-float(yellow[1]); dz=float(red[2])-float(yellow[2])
            stack=(dx*dx+dy*dy)**0.5 <= .040 and .040 <= dz <= .065
        return {
            'seed':seed,'ok':fail is None and stack,'failed_action':fail,'elapsed_s':round(time.perf_counter()-ep0,4),
            'stack_verified':stack,'final':compact_state(st),'steps':rows,
        }
    finally:
        w.close()

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--start-seed',type=int,default=0); ap.add_argument('--episodes',type=int,default=12)
    ap.add_argument('--module',default='sim.masterpi_physics')
    ap.add_argument('--recover-on-grasp',action='store_true')
    ap.add_argument('--speed',type=float,default=3); ap.add_argument('--width',type=int,default=640); ap.add_argument('--height',type=int,default=480)
    ap.add_argument('--out',default='outputs/physics_skill_stress.jsonl'); a=ap.parse_args()
    import importlib
    global MasterPiPhysicsWorld, RECOVER_ON_GRASP
    MasterPiPhysicsWorld=importlib.import_module(a.module).MasterPiPhysicsWorld
    RECOVER_ON_GRASP=bool(a.recover_on_grasp)
    out=Path(a.out); out.parent.mkdir(parents=True,exist_ok=True)
    results=[]
    with out.open('w') as f:
        for seed in range(a.start_seed,a.start_seed+a.episodes):
            r=run_episode(seed,a.speed,a.width,a.height); results.append(r)
            f.write(json.dumps(r,ensure_ascii=False)+'\n'); f.flush()
            print(json.dumps({'seed':seed,'ok':r['ok'],'failed':r['failed_action'],'elapsed_s':r['elapsed_s'],'stack':r['stack_verified']},ensure_ascii=False),flush=True)
    per={}
    for r in results:
        for s in r['steps']: per.setdefault(s['action'],[]).append(s['elapsed_s'])
    summary={
        'episodes':len(results),'successes':sum(r['ok'] for r in results),'success_rate':sum(r['ok'] for r in results)/max(1,len(results)),
        'failure_counts':{}, 'mean_episode_s':statistics.mean(r['elapsed_s'] for r in results),
        'action_mean_s':{k:statistics.mean(v) for k,v in per.items()},
        'action_p95_s':{k:sorted(v)[max(0,min(len(v)-1,int(.95*len(v))-1))] for k,v in per.items()},
        'mean_logging_observe_s':statistics.mean((s.get('red_obs_s',0)+s.get('yellow_obs_s',0)) for r in results for s in r['steps']),
    }
    for r in results:
        if r['failed_action']: summary['failure_counts'][r['failed_action']]=summary['failure_counts'].get(r['failed_action'],0)+1
        elif not r['stack_verified']: summary['failure_counts']['stack_verify']=summary['failure_counts'].get('stack_verify',0)+1
    print('SUMMARY '+json.dumps(summary,ensure_ascii=False),flush=True)
    Path(str(out)+'.summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
