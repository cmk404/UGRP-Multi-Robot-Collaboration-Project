#!/usr/bin/env python3
"""OFFLINE privileged diagnostic: replay archived grip and inspect retention.

Source-run referee poses are teacher-only fixture setup. No state/contacts are
exported to a student. Compare solver friction refinement without welds, changes
to masses, friction coefficients, actuators, geometry, or cameras.
"""
from __future__ import annotations
import argparse,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.research_dispatch_scene import DispatchScene
from scripts.camera_approach_scene import normalize_replay
from scripts.probe_dual_grasp_sync import _plain_beam_contact
from sim.research_dispatch_arena import episode


def main():
    import mujoco
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profiles',nargs='+',default=['legacy','global_noslip','local_friction'])
    args=p.parse_args()
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
        'replayed_run':str(args.source_run.resolve()),'cases':[]}
    for profile in args.profiles:
        for moving in [False,True]:
            case={'contact_solver_profile':profile,'moving':moving,'samples':[]}
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
                for i in range(310):
                    a={'kind':'mecanum','forward':.08 if moving and 35<=i<135 else 0.,
                        'left':.08 if moving and i<35 else 0.,'turn':0.,'duration_s':.1}
                    for rid in pair.values():scene.ports[rid].apply(a,float(w.data.time))
                    scene.step(.1)
                    case['samples'].append({'t':round((i+1)*.1,1),'beam_position':w.data.geom_xpos[beam].tolist(),
                        'contacts':{rid:_plain_beam_contact(w,rid) for rid in pair.values()},'weld':bool(w.data.eq_active.any())})
                    if i in [21,99,309]:scene.capture('hold-'+str(i))
                for port in scene.ports.values():port.hold(float(w.data.time))
                w._team_joint_move_servos({rid:{1:2000} for rid in pair.values()},.65,settle_s=.5)
                scene.step(1.)
                case['opened_release_height']=float(w.data.geom_xpos[beam][2])
                scene.capture('opened-release')
                case['final_height']=case['samples'][-1]['beam_position'][2]
                case['min_height']=min(s['beam_position'][2] for s in case['samples'])
                case['weld_steps']=scene.weld_steps
                case['obstacle_contact_steps']=scene.obstacle_contact_steps
            finally:scene.close()
            case['wall_s']=time.monotonic()-began
            (out/'teacher-only.json').write_text(json.dumps(case,indent=2))
            report['cases'].append({k:v for k,v in case.items() if k!='samples'})
            print(json.dumps(report['cases'][-1]),flush=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
