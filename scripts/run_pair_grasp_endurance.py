#!/usr/bin/env python3
"""Bounded RGB endurance experiment; all physical diagnostics are output-only."""
from pathlib import Path
import argparse,json,sys,subprocess,time,traceback,math,hashlib,platform
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.run_pair_navigation import PairNavigationScene
from scripts.run_camera_approach_student import models,write
from harness.grasp_student_inference import predict_student
from harness.pair_grasp_endurance import EnduranceActor,DT
from harness.pair_navigation import ROBOTS,authorize_pair,digest
from harness.pair_carry_sync import PairCarrySync
from scripts.evaluate_pair_navigation import evaluate_grasp_stability

class EnduranceScene(PairNavigationScene):
    def evaluation_snapshot(self,*,full_state=True):
        row=super().evaluation_snapshot(full_state=full_state)
        m,d=self.world.model,self.world.data
        row['finger_centers_m']={r:((d.geom_xpos[m.geom(r+'__left_finger').id]+d.geom_xpos[m.geom(r+'__right_finger').id])/2).tolist() for r in ROBOTS}
        row['beam_minus_finger_z_m']={r:row['position_m'][2]-row['finger_centers_m'][r][2] for r in ROBOTS}
        return row
    def execute_endurance(self,actions):
        self.phase='carry' if any(abs(a[k])>1e-12 for a in actions.values() for k in ('forward','left','turn')) else 'carry_stop'
        start=self.time()
        for r in ROBOTS:self.ports[r].apply(actions[r],start)
        self.tick(DT)
        self.trace.append({'stage':'endurance_command','actions':actions,'start_sim_time_s':start,'end_sim_time_s':self.time()})

def evaluate_endurance(rows, report):
    carry=[r for r in rows if r['phase'] in ('carry','carry_stop')]
    grasp=evaluate_grasp_stability(rows)
    if not carry:return {'success':False,'reason':'no endurance samples','grasp_stability':grasp}
    anchor=grasp.get('anchor_spacing_m');first=carry[0];start=report['endurance_start_s']
    checks={}
    for duration in (180,300):
        if duration>report['duration_s']:continue
        subset=[r for r in carry if r['sim_time_s']<=start+duration+.001]
        gates={'duration_completed':subset[-1]['sim_time_s']>=start+duration-.11,
            'full_grasp_stable':grasp['success'],'bilateral_every_sample':all(all(r['contacts'][rid]['bilateral'] for rid in ROBOTS) for r in subset),
            'lifted_every_sample':all(r['height_above_start_m']>=.03 for r in subset),
            'level_every_sample':all(r['tilt_deg']<=10 for r in subset),
            'spacing_preserved':anchor is not None and all(abs(math.dist(r['bases']['r1'][:2],r['bases']['r3'][:2])-anchor)<=.02 for r in subset),
            'height_retained':min(r['position_m'][2] for r in subset)>=first['position_m'][2]-.01,
            'continuous_samples':max((b['sim_time_s']-a['sim_time_s'] for a,b in zip(subset,subset[1:])),default=0)<=.15,
            'within_authored_bounds':all(r['within_authored_bounds'] for r in subset),
            'collision_free':report['wall_contact_ticks']==0 and report['unexpected_contact_ticks']==0,
            'weld_off':report['weld_active_ticks']==0,
            'invariants_unchanged':report['invariants_initial']==report['invariants_final']}
        checks[str(duration)]={'passed':all(gates.values()),'gates':gates,'last_sample_s':subset[-1]['sim_time_s']-start,
            'max_beam_drop_m':first['position_m'][2]-min(r['position_m'][2] for r in subset),
            'finger_height_change_m':{rid:subset[-1]['finger_centers_m'][rid][2]-first['finger_centers_m'][rid][2] for rid in ROBOTS},
            'beam_relative_to_finger_change_m':{rid:subset[-1]['beam_minus_finger_z_m'][rid]-first['beam_minus_finger_z_m'][rid] for rid in ROBOTS}}
    result={'success':bool(checks) and all(x['passed'] for x in checks.values()),'checkpoints':checks,'grasp_stability':grasp}
    if report.get('release_check_required'):
        release=[r for r in rows if r['phase']=='release_hold']
        released=len(release)>=10 and all(r['payload_floor_contact'] and all(not r['contacts'][rid][side] for rid in ROBOTS for side in ('left','right')) for r in release[-10:])
        result['released_on_floor']=released
        result['success'] &= released
    return result

def run(data,grasp_root,out,mode,duration_s=300,noslip_iterations=0,impratio=10):
    if out.exists():raise FileExistsError(out)
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('commit execution source and protocol first')
    skill,gm=models(grasp_root,'student-skill.json');scene=EnduranceScene(out,grasp_root,data,impratio=impratio,noslip_iterations=noslip_iterations)
    actors={r:EnduranceActor(data,r,mode) for r in ROBOTS};sync=PairCarrySync(data['map_id']+'-endurance')
    report={'schema':'ugrp.pair_grasp_endurance.v1','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'map':data,'map_sha256':digest(data),'mode':mode,'duration_s':duration_s,'close_pulse':1600,'impratio':impratio,'noslip_iterations':noslip_iterations,'endurance_trace_version':1,'release_check_required':True,
        'grasp_model_files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(grasp_root.iterdir()) if p.is_file()},
        'environment':{'python':sys.version,'platform':platform.platform()},'steps':[],'error':None,'external_model_calls':0,'cost_usd':0,'scope':'Fixed-start RGB guarded endurance, 180/300s checkpoints from the same trajectory'}
    started=time.monotonic()
    try:
        import mujoco
        report['environment']['mujoco']=mujoco.__version__
        scene.open();report['scene_xml_sha256']=scene.xml_sha;report['invariants_initial']=scene.invariant_record()
        scene.finish_grasp(predict_student,gm,after_close=scene.anchor_spacing,hold=scene.hold_spacing,close_pulse=1600)
        scene.hold_spacing(8.);scene.finish_spacing();report['endurance_start_s']=scene.time()
        for i in range(round(duration_s/DT)):
            frames=scene.capture(f'endurance-{i:04d}')
            decisions={r:actors[r].decide(frames[r]['own_bytes'],frames[r]['top_bytes']) for r in ROBOTS}
            permission=authorize_pair(sync,decisions,{r:frames[r]['frame_id'] for r in ROBOTS},i,interval_s=DT)
            actions={r:dict(decisions[r]['action']) for r in ROBOTS}
            if permission['phase']!='GO':
                for a in actions.values():a.update(forward=0.,left=0.,turn=0.)
            report['steps'].append({'index':i,'images':{r:{'own':frames[r]['own_rgb'],'top':frames[r]['shared_top_rgb']} for r in ROBOTS},
                'frame_ids':{r:frames[r]['frame_id'] for r in ROBOTS},'decisions':decisions,'permission':permission,'issued_actions':actions,'sim_time_s':scene.time()})
            scene.execute_endurance(actions)
            if not all(d['ready'] for d in decisions.values()):raise RuntimeError('RGB endurance stopped: '+str({r:d.get('error') for r,d in decisions.items()}))
        for port in scene.ports.values():port.stop()
        scene.capture('endurance-finished')
        scene.place()  # Fixed timer completion, never gated by evaluation/contact state.
    except Exception:report['error']=traceback.format_exc()
    finally:
        if scene.world:
            report.update(invariants_final=scene.invariant_record(),weld_active_ticks=scene.weld_active_ticks,wall_contact_ticks=scene.wall_contact_ticks,
                unexpected_contact_ticks=scene.unexpected_contact_ticks,sim_seconds=scene.time())
            report['evaluation']=evaluate_endurance(scene.evaluation_samples,report)
        report.update(spacing_steps=scene.spacing_steps,spacing_sync_events=scene.spacing_sync.events,sync_events=sync.events,wall_seconds=time.monotonic()-started)
        report['success']=not report['error'] and report.get('evaluation',{}).get('success',False)
        write(out/'grasp-result.json',scene.grasp_report);write(out/'contact-events-evaluation-only.json',scene.contact_events)
        try:scene.close()
        except Exception:report['cleanup_error']=traceback.format_exc();report['success']=False
        finally:
            out.mkdir(parents=True,exist_ok=True);write(out/'result.json',report)
    print(json.dumps({k:report[k] for k in ('success','error','evaluation','wall_seconds')},indent=2))
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map',type=Path,required=True);p.add_argument('--grasp-model-dir',type=Path,required=True)
    p.add_argument('--out-dir',type=Path,required=True);p.add_argument('--mode',choices=('stationary','shuttle'),required=True)
    p.add_argument('--impratio',type=int,choices=(1,10,100),default=10)
    p.add_argument('--noslip-iterations',type=int,choices=(0,3),default=0)
    a=p.parse_args();r=run(json.loads(a.map.read_text()),a.grasp_model_dir.resolve(),a.out_dir.resolve(),a.mode,noslip_iterations=a.noslip_iterations,impratio=a.impratio)
    return int(not r['success'])
if __name__=='__main__':raise SystemExit(main())
