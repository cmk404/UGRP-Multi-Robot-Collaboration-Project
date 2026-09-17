#!/usr/bin/env python3
"""RGB-only recovery student execution; privileged teacher is an explicit mode.

Post-run alignment scores cannot gate grasp. Takeover is a fixed training-only
step index; the scene continues physically without resetting robot or payload.
"""
import argparse,json,subprocess,sys,time,traceback,platform
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.camera_approach_scene import ApproachScene,ROBOTS,validate_start_poses
from scripts.collect_camera_approach_teacher import load_models,sha,write
from scripts.recovery_teacher import command,errors,heldout_region,score_alignment,AXES
from harness.recovery_commands import issued_command
from scripts.approach_speed_teacher import teacher_command
from scripts.run_camera_pair_transport import evaluate_grasp_samples

def execute(args):
    fixture=json.loads(args.case.read_text());starts=validate_start_poses(fixture['start_poses'])
    root=args.out.resolve();started=time.monotonic();scene=ApproachScene(root,args.grasp_model.resolve())
    models,skillhash,modelhashes=load_models(args.grasp_model.resolve())
    goals=scene.fixture['base_poses']
    rec={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
         'case':fixture,'policy':args.policy,'takeover_step':args.takeover_step,'config':{'max_rounds':160,'slice_s':.2,'weld':False,'confirmation_steps':3,'teacher_tail_budget':160 if args.takeover_step is not None else None,'command_decoder':args.command_decoder if args.policy=='act' else 'raw'},
         'grasp_skill_sha256':skillhash,'grasp_model_sha256':modelhashes,'calls':[],'approach_ok':False,'error':None,
         'input_boundary':'ACT process receives own_rgb and top_rgb only. Teacher/evaluation state is never passed to ACT.',
         'teacher_observations':[] if args.policy!='act' or args.takeover_step is not None else None}
    clients={};actors=[];labels=[];hist={r:[] for r in ROBOTS};takeover=None;prefix_declared_ready=False
    try:
        import mujoco
        from harness.grasp_student_inference import predict_student
        rec['environment']={'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__}
        if args.policy=='act':
            from harness.recovery_act_client import RecoveryClient
            rec['model_hashes']={}
            for r in ROBOTS:
                model=args.model.resolve()/r/'act';clients[r]=RecoveryClient(args.act_python,model)
                rec['model_hashes'][r]={p.name:sha(p) for p in model.iterdir() if p.is_file()}
        scene.open(start_poses=starts);rec['initial_state']=scene.evaluation_snapshot();begin=scene.time();confirm=0
        for i in range(160 + (args.takeover_step or 0)):
            frames=scene.capture(f'approach-{i:03d}')
            teaching=args.policy!='act' or (args.takeover_step is not None and i>=args.takeover_step)
            state=scene.evaluation_snapshot() # logging/referee only; no actor argument
            es={r:errors(state,goals,r) for r in ROBOTS}
            if teaching:
                if args.takeover_step is not None and takeover is None:
                    takeover={'index':i,'state':state,'prefix_contact_steps':scene.approach_payload_contact_steps,'prefix_declared_ready':prefix_declared_ready,'errors':es};confirm=0
                ds={r:command(es[r]) if args.policy!='nominal_teacher' else {**teacher_command(es[r]['x'],2.),'left':0.,'turn':0.} for r in ROBOTS}
                rec['teacher_observations'].append({'index':i,'state':state,'errors':es})
            else:ds={r:clients[r].predict(frames[r]['own_bytes'],frames[r]['top_bytes']) for r in ROBOTS}
            both=all(d['ok'] and d['ready'] for d in ds.values())
            confirm=confirm+1 if both else 0
            if not teaching and confirm>=3:prefix_declared_ready=True
            actions={r:issued_command(ds[r], 'raw' if teaching else args.command_decoder) for r in ROBOTS}
            for r in ROBOTS:
                obs={'own':frames[r]['own_rgb'],'top':frames[r]['shared_top_rgb']};sid=f'{fixture["id"]}:{i:03d}:{r}'
                rec['calls'].append({'index':i,'robot_id':r,'images':obs,'decision':ds[r],'action':actions[r],'teaching':teaching,'history':list(hist[r]),'wire_request_sha256':None if teaching else clients[r].last_request_sha256})
                if teaching:
                    actors.append({'id':sid,'case_id':fixture['id'],'robot_id':r,'images':obs})
                    labels.append({'id':sid,'case_id':fixture['id'],'robot_id':r,'target':{**actions[r],'stop':ds[r]['ready']},'heldout_region':heldout_region(es[r])})
                hist[r].append(actions[r])
            scene.drive_mecanum(actions,.2)
            if confirm>=3 and (args.takeover_step is None or teaching):rec['approach_ok']=True;break
        rec['approach_sim_s']=scene.time()-begin;rec['approach_end_state']=scene.evaluation_snapshot()
        rec['alignment']=score_alignment(scene.evaluation_samples,goals,scene.time()) # ONLY after actions finish
        rec['takeover']=takeover
        if rec['approach_ok'] and not args.skip_grasp:
            rec['grasp_calls']=scene.finish_grasp(predict_student,models)
        rec['evaluation']=evaluate_grasp_samples(scene.evaluation_samples)
        rec['final_state']=scene.evaluation_snapshot();rec['contact_steps']=scene.approach_payload_contact_steps
        rec['success']=bool(rec['approach_ok'] and rec['alignment']['success'] and rec['evaluation']['grasp_success'] and not rec['contact_steps'])
        rec['teacher_tail_contact_steps']=rec['contact_steps']-(takeover['prefix_contact_steps'] if takeover else 0)
        rec['training_eligible']=bool(rec['approach_ok'] and rec['alignment']['success'] and rec['evaluation']['grasp_success'] and rec['teacher_tail_contact_steps']==0 and not any(x['heldout_region'] for x in labels))
    except Exception as e:
        rec['error']=str(e);rec['traceback']=traceback.format_exc();rec['success']=False
    finally:
        scene.close()
        for c in clients.values():c.close()
        root.mkdir(exist_ok=True,parents=True)
        rec['wall_s']=time.monotonic()-started
        write(root/'actor_samples.json',actors);write(root/'teacher_labels.json',labels);write(root/'result.json',rec)
    print(json.dumps({k:rec.get(k) for k in ('success','approach_ok','alignment','training_eligible','error','wall_s')}),flush=True)
    return int(bool(rec['error']))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('case','out','grasp-model'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--policy',choices=('recovery_teacher','nominal_teacher','act'),required=True)
    p.add_argument('--command-decoder',choices=('raw','calibrated'),default='raw')
    p.add_argument('--model',type=Path);p.add_argument('--act-python',type=Path);p.add_argument('--takeover-step',type=int);p.add_argument('--skip-grasp',action='store_true')
    a=p.parse_args()
    if a.policy=='act' and (not a.model or not a.act_python):p.error('ACT needs model and interpreter')
    if a.takeover_step is not None and (a.policy!='act' or not 1<=a.takeover_step<160):p.error('takeover is training-only ACT step 1..159')
    raise SystemExit(execute(a))
