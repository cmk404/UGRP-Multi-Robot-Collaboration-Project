"""One or three independent RGB actors in one physical arena; truth is referee-only."""
from pathlib import Path
import argparse
import base64
import hashlib
import json
import math

import mujoco

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.camera_robot_port import CameraRobotPort
from harness.visual_transport import VisualTransport
from harness.visual_macro_runtime import VisualMacroExecutor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True)
    ap.add_argument('--seed', type=int, default=41)
    ap.add_argument('--robots', type=int, choices=(1,3), default=1)
    ap.add_argument('--seconds', type=float, default=360)
    ap.add_argument('--impratio', type=float, default=10)
    ap.add_argument('--noslip-iterations', type=int, default=0)
    ap.add_argument('--record', action='store_true')
    args = ap.parse_args()
    out=Path(args.output); out.mkdir(parents=True, exist_ok=False)
    source={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for base in ('sim','harness','scripts','calibration')
            for p in sorted(Path(base).rglob('*')) if p.is_file() and p.suffix in ('.py','.xml','.json','.png','.yaml','.yml') and '__pycache__' not in p.parts}
    (out/'source-manifest.json').write_text(json.dumps(source,indent=2))
    source_hash=hashlib.sha256(json.dumps(source,sort_keys=True).encode()).hexdigest()
    active=['r1'] if args.robots==1 else ['r1','r2','r3']
    cargo_ids=tuple(f'small_box_0{i+1}' for i in range(args.robots))
    world=MultiMasterPiProductionV2(warehouse_layout='camera_team',seed=args.seed,render=True,
                                   warehouse_cargo_ids=cargo_ids)
    world.model.opt.impratio=args.impratio; world.model.opt.noslip_iterations=args.noslip_iterations
    for zone in 'abc':
        gid=mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_GEOM,'warehouse_zone_'+zone)
        world.model.geom_group[gid]=0
    initial=world.warehouse_state()
    # This is an operator task assignment of object identity and named region.
    # Geometric goal positions remain exclusively in the referee.
    goals={rid: min(world.warehouse_zones, key=lambda z: math.dist(world.warehouse_spec_by_id[cid].goal_xyz[:2], world.warehouse_zones[z].center_xy)) for rid,cid in zip(active,cargo_ids)}
    ports={rid:CameraRobotPort(world,rid) for rid in active}
    actors={rid:VisualTransport(rid,cid,goals[rid]) for rid,cid in zip(active,cargo_ids)}
    commands=(out/'commands.jsonl').open('w'); control=(out/'control.jsonl').open('w'); truth=(out/'evaluation-only.jsonl').open('w')
    def command_log(row):
        commands.write(json.dumps(row)+'\n')
    executors={rid:VisualMacroExecutor(ports[rid],log_callback=command_log) for rid in active}
    for rid in active: (out/'inputs'/rid).mkdir(parents=True)
    steps={rid:0 for rid in active}; reasons={rid:'TIME_BUDGET' for rid in active}; done=set()
    start=float(world.data.time); deadline=start+args.seconds; next_truth=start
    max_lifts={cid:0. for cid in cargo_ids}; peer_contact_s=0.; obstacle_contact_s=0.; concurrent_motion_s=0.; concurrent_carry_s=0.
    previous_cargo=None; previous_time=None; samples=[]; video=None; error=None
    try:
        if args.record:
            from scripts.record_visual_team import VisualTeamVideo
            video=VisualTeamVideo(world,out/'motion-1x.mp4',actors)
            video.capture(force=True)
        while world.data.time < deadline and len(done)<len(active):
            now=float(world.data.time)
            for rid in active:
                ex=executors[rid]; ex.tick(now)
                if rid in done or not ex.idle: continue
                wrist=ports[rid].capture(); nav=ports[rid].capture(camera='nav_cam')
                if video:video.update_inputs(rid,wrist,nav,now)
                index=steps[rid]; steps[rid]+=1
                for name,obs in [('wrist',wrist),('nav',nav)]:
                    (out/'inputs'/rid/f'{index:04d}-{name}.jpg').write_bytes(base64.b64decode(obs['image']))
                actor=actors[rid]; before=actor.phase
                action=actor.decide(wrist,nav)
                control.write(json.dumps({'robot_id':rid,'step':index,'time':now,'phase':before,
                    'wrist_sha256':wrist['sha256'],'nav_sha256':nav['sha256'],
                    'own_pose_commands':wrist['actuator_state']['servo_pulses'],
                    'estimated_target':actor.box.last_target,'attachment':actor.box.last_attachment,
                    'navigation':getattr(actor.navigator,'last_observation',None),'action':action})+'\n');control.flush()
                if index%20==0 or before not in ('approach','navigate'):
                    print(rid,index,round(now-start,1),before,action,flush=True)
                if action['kind']=='finish':
                    done.add(rid);reasons[rid]=action['reason'];ports[rid].stop()
                else:
                    # Motor state/pose source is always owned. Both source images are logged above.
                    ex.submit(action,wrist,before,now)
            now=float(world.data.time)
            if now>=next_truth:
                state=world.warehouse_state();samples.append(state)
                truth.write(json.dumps({'time':now,'state':state,
                    'robot_positions':{rid:world.robot(rid).base_xyz().tolist() for rid in active},
                    'grip_positions':{rid:world.robot(rid).site_xyz('grip_site').tolist() for rid in active}})+'\n');truth.flush()
                for cid in cargo_ids:
                    z=state['cargo'][cid]['position'][2]-initial['cargo'][cid]['position'][2]
                    max_lifts[cid]=max(max_lifts[cid],z)
                if previous_cargo is not None:
                    dt=now-previous_time
                    moving=sum(math.dist(state['cargo'][cid]['position'][:2],previous_cargo[cid]['position'][:2])>0.002 for cid in cargo_ids)
                    if moving>=2:concurrent_carry_s+=dt
                previous_cargo=state['cargo'];previous_time=now;next_truth=now+.25
            dt=float(world.model.opt.timestep)
            if sum(any(abs(v)>1e-5 for v in ports[r]._motor_commands) for r in active)>=2:concurrent_motion_s+=dt
            peer=False;obstacle=False
            for contact in world.data.contact:
                if contact.dist>=-.002:continue
                a=mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom1)) or ''
                b=mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom2)) or ''
                ra=next((r for r in world.robot_ids if a.startswith(r+'__')),None)
                rb=next((r for r in world.robot_ids if b.startswith(r+'__')),None)
                if ra and rb and ra!=rb:peer=True
                if (ra and 'barrier' in b) or (rb and 'barrier' in a):obstacle=True
            peer_contact_s+=dt*peer;obstacle_contact_s+=dt*obstacle
            world._physics_step_for(world.robot('r1'))
            if video and world.data.time>=video.next_frame:video.capture()
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
        for rid in active:
            if rid not in done: reasons[rid]='EXECUTION_ERROR:'+error
        print(error,flush=True)
        raise
    finally:
        for port in ports.values():port.stop()
        until=float(world.data.time)+1
        while world.data.time<until:
            for port in ports.values():port.tick(float(world.data.time))
            world._physics_step_for(world.robot('r1'))
            if video and world.data.time>=video.next_frame:video.capture()
        final=world.warehouse_state(); outcomes={}
        for rid,cid in zip(active,cargo_ids):
            box=final['cargo'][cid];zone=world.warehouse_zones[goals[rid]]
            p=box['position']; half=world.warehouse_spec_by_id[cid].dimensions_m
            inside=all(abs(p[i]-zone.center_xy[i])+half[i]/2 <= zone.half_extents_xy[i] for i in (0,1))
            constrained=any(any(s['cargo'][cid].get('constraints_active',{}).values()) for s in [initial,*samples,final])
            gates={'lift':max_lifts[cid]>=.04,'inside_destination':inside,'stable':box['stable'],
                   'no_attachment_constraint':not constrained,'visual_release':reasons[rid]=='VISUAL_RELEASE_CONFIRMED'}
            outcomes[rid]={'cargo_id':cid,'destination_zone':goals[rid],'reason':reasons[rid],'success':all(gates.values()),'gates':gates,'max_lift_m':max_lifts[cid],'decisions':steps[rid]}
        result={'seed':args.seed,'active_robots':active,'outcomes':outcomes,'success':all(o['success'] for o in outcomes.values()),
                'elapsed_sim_s':float(world.data.time)-start,'peer_penetration_gt2mm_s':peer_contact_s,
                'obstacle_penetration_gt2mm_s':obstacle_contact_s,'concurrent_drive_s':concurrent_motion_s,
                'concurrent_cargo_motion_s':concurrent_carry_s,'initial':initial,'final':final,'source_hash':source_hash,
                'physics':{'impratio':args.impratio,'noslip_iterations':args.noslip_iterations},
                'controller':'independent_deterministic_RGB','llm_calls':0,'messages':0,'error':error}
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ('success','outcomes','concurrent_drive_s','concurrent_cargo_motion_s')}),flush=True)
        commands.close();control.close();truth.close()
        if video:video.close()
        world.close()

if __name__=='__main__':main()
