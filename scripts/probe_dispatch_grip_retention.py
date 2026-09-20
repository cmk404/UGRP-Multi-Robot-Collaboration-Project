#!/usr/bin/env python3
"""OFFLINE privileged diagnostic: replay archived grip and inspect retention.

Source-run referee poses are teacher-only fixture setup. No state/contacts are
exported to a student. Compare solver friction refinement without welds, changes
to masses, friction coefficients, actuators, geometry, or cameras.
"""
from __future__ import annotations
import argparse,hashlib,json,math,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.research_dispatch_scene import DispatchScene
from scripts.camera_approach_scene import normalize_replay
from scripts.probe_dual_grasp_sync import _plain_beam_contact
from sim.research_dispatch_arena import episode


def main():
    import mujoco
    import numpy as np
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profiles',nargs='+',default=['legacy','global_noslip','local_contact'])
    p.add_argument('--hold-s',type=float,default=31.)
    p.add_argument('--motion',choices=['stationary','moving','both'],default='both')
    args=p.parse_args()
    if not 1<=args.hold_s<=300:raise ValueError('hold duration must be 1..300 seconds')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('commit source first')
    args.output.mkdir(parents=True,exist_ok=False)
    source=json.loads((args.source_run/'result.json').read_text())
    pair=source['bindings']['pair_model_slots']
    replay=json.loads((args.source_run/'pair-replay.json').read_text())
    commands=json.loads((args.source_run/'issued-commands.json').read_text())
    start=min(c['issued_at_s'] for rows in commands.values() for c in rows if c['stage']=='grasp_initialization')
    samples=[json.loads(x) for x in (args.source_run/'referee-only.jsonl').read_text().splitlines()]
    setup=min(samples,key=lambda x:abs(x['sim_time_s']-start))
    report={'teacher_only':True,'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'replayed_run':str(args.source_run.resolve()),'cases':[],
        'scope':'Privileged pose fixture plus archived grasp commands; not exact whole-run replay or independent RGB E2E.',
        'setup_referee_time_s':setup['sim_time_s'],
        'source_files':{name:hashlib.sha256((args.source_run/name).read_bytes()).hexdigest()
            for name in ('result.json','pair-replay.json','issued-commands.json','referee-only.jsonl')}}
    for profile in args.profiles:
        for moving in ([False,True] if args.motion=='both' else [args.motion=='moving']):
            case={'contact_solver_profile':profile,'moving':moving,'hold_s':args.hold_s,'samples':[]}
            out=args.output/f'{profile}-move-{int(moving)}'
            config=episode('open',11);config['contact_solver_profile']=profile
            for rid in pair.values():config['setup_only']['spawns'][rid]=[*setup['robots'][rid],0.]
            scene=DispatchScene(config,out);began=time.monotonic()
            try:
                scene.open();w=scene.world
                for item in replay:
                    if not item['stage'].startswith('grasp'):continue
                    cmd=item['command'];targets=normalize_replay(cmd)
                    w._team_joint_move_servos({pair[s]:v for s,v in targets.items()},cmd['duration_s'],settle_s=cmd.get('settle_s',0))
                scene.capture('lift-start')
                beam=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'team_beam_geom')
                floor=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
                case['hold_start_sim_s']=float(w.data.time)
                steps=round(args.hold_s/.1)
                for i in range(steps):
                    a={'kind':'mecanum','forward':.08 if moving and 35<=i<135 else 0.,
                        'left':.08 if moving and i<35 else 0.,'turn':0.,'duration_s':.1}
                    for rid in pair.values():scene.ports[rid].apply(a,float(w.data.time))
                    scene.step(.1)
                    floor_contact=False;friction_utilization=[]
                    for ci,c in enumerate(w.data.contact[:w.data.ncon]):
                        geoms={int(c.geom1),int(c.geom2)}
                        if beam not in geoms or c.dist>.001:continue
                        if floor in geoms:floor_contact=True
                        if not geoms.intersection(scene.robot_ids):continue
                        wrench=np.zeros(6);mujoco.mj_contactForce(w.model,w.data,ci,wrench)
                        if abs(wrench[0])>1e-7:
                            friction_utilization.append(float(math.hypot(wrench[1]/c.friction[0],wrench[2]/c.friction[1])/abs(wrench[0])))
                    case['samples'].append({'t':round((i+1)*.1,1),'sim_time_s':float(w.data.time),
                        'beam_position':w.data.geom_xpos[beam].tolist(),'floor_contact':floor_contact,
                        'max_friction_cone_utilization':max(friction_utilization,default=None),
                        'contacts':{rid:_plain_beam_contact(w,rid) for rid in pair.values()},'weld':bool(w.data.eq_active.any())})
                    if i in [21,99,steps-1] or (i+1)%500==0:
                        scene.capture('hold-'+str(i))
                        print(json.dumps({'profile':profile,'moving':moving,'held_s':round((i+1)*.1,1),
                            'beam_height':float(w.data.geom_xpos[beam][2])}),flush=True)
                for port in scene.ports.values():port.hold(float(w.data.time))
                w._team_joint_move_servos({rid:{1:2000} for rid in pair.values()},.65,settle_s=.5)
                scene.step(1.)
                case['opened_release_height']=float(w.data.geom_xpos[beam][2])
                scene.capture('opened-release')
                case['final_height']=case['samples'][-1]['beam_position'][2]
                case['min_height']=min(s['beam_position'][2] for s in case['samples'])
                case['floor_contact_samples']=sum(s['floor_contact'] for s in case['samples'])
                case['clock_continuous']=all(.099<y['sim_time_s']-x['sim_time_s']<.101
                    for x,y in zip(case['samples'],case['samples'][1:]))
                case['weld_steps']=scene.weld_steps
                case['obstacle_contact_steps']=scene.obstacle_contact_steps
                case['physics_warnings']={str(i):int(w.data.warning[i].number)
                    for i in range(len(w.data.warning)) if w.data.warning[i].number}
            finally:scene.close()
            case['wall_s']=time.monotonic()-began
            (out/'teacher-only.json').write_text(json.dumps(case,indent=2))
            report['cases'].append({k:v for k,v in case.items() if k!='samples'})
            print(json.dumps(report['cases'][-1]),flush=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
