#!/usr/bin/env python3
"""OFFLINE TEACHER ONLY: collect new-arena RGB and fit existing skill backends.

This process may use setup poses/joints/contact labels under the approved teacher
exception. None of its simulator objects/labels are passed to the student runner.
The original demonstrated arm sequence and predictor schemas are retained.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.research_dispatch_scene import DispatchScene
from sim.research_dispatch_arena import episode
from scripts.run_camera_approach_student import models,sha,write
from harness.dispatch_skill_binding import canonical_pair_top
from harness.camera_varied_start_pose_student import fit_pose_stage_model
from harness.camera_recovery_student import fit_recovery_model
from scripts.camera_approach_scene import normalize_replay,image_record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--base-grasp',type=Path,required=True)
    p.add_argument('--reference-top',type=Path,default=ROOT/'tests/fixtures/camera_goal_transport/reference-top.jpg')
    p.add_argument('--approach-only',action='store_true')
    args=p.parse_args()
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('commit source first')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    skill,_=models(args.base_grasp.resolve(),'student-skill.json')
    reference=args.reference_top.read_bytes();config=episode('open',11)
    pair={'r1':'r3','r3':'r1'}
    # Teacher-only goal reset: reuse the demonstrated physical formation,
    # translated with the authored cargo. Never exported as student inputs.
    target={'r1':[-.18,-1.975,.032355118817659255], 'r3':[-.18,-1.325,.032355118817659255]}
    for slot,rid in pair.items():config['setup_only']['spawns'][rid]=[*target[slot],0.]
    scene=DispatchScene(config,out/'teacher-scene')
    actor_rows=[];teacher_rows=[];started=time.monotonic()
    report={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'teacher_only':True,'student_inputs':['own_rgb','fixed_top_rgb'],
        'base_skill_sha256':sha(args.base_grasp/'student-skill.json'),
        'model_provenance':'offline new-arena teacher; existing backend retraining; no online teacher correction',
        'complete':False,'error':None}
    try:
        import mujoco
        scene.open();w=scene.world
        def settle(t):
            for _ in range(round(t/w.model.opt.timestep)):w._physics_step_for(w.controllers['r1'])
        def arm(targets,duration=.35,settle_s=.10):
            w._team_joint_move_servos({pair[s]:v for s,v in targets.items()},duration,settle_s=settle_s)
        def snapshot():
            return (w.data.qpos.copy(),w.data.qvel.copy(),w.data.ctrl.copy(),float(w.data.time),
                    {r:copy.deepcopy(c.servo_command_pulses) for r,c in w.controllers.items()})
        def restore(s):
            w.data.qpos[:],w.data.qvel[:],w.data.ctrl[:],w.data.time=s[:4]
            w.data.qacc_warmstart[:]=0
            for r,c in w.controllers.items():c.servo_command_pulses=copy.deepcopy(s[4][r])
            w.data.eq_active[:]=0;mujoco.mj_forward(w.model,w.data)
        def capture(tag,slot):
            f=scene.capture(tag)[pair[slot]]
            top,tr=canonical_pair_top(f['top_bytes'],reference)
            rec=image_record(out/'student-rgb'/f'{tag}-top.jpg',out,top)
            own=image_record(out/'student-rgb'/f'{tag}-own.jpg',out,f['own_bytes'])
            actor_rows.append({'sample_id':tag,'model_slot':slot,'own_rgb':own,'top_rgb':rec,'transform':tr,
                               'raw_rgb_root':'teacher-scene','raw_top':f['shared_top_rgb']})
            return f['own_bytes'],top
        settle(.4);folded=snapshot()
        refs={s:capture('approach-reference-'+s,s) for s in pair}
        datasets={s:[] for s in pair}
        rng=np.random.default_rng(704)
        # Broad camera foreground/background variation plus dense final docking
        # samples. Error labels are measured after settling, in teacher only.
        cases=[(d,l,y) for d,l,y in itertools.product((0.,.04,.12,.22,.32),(-.055,0.,.055),(-9.,0.,9.))]
        cases += [(float(rng.uniform(-.009,.018)),float(rng.uniform(-.006,.006)),float(rng.uniform(-1.2,1.2))) for _ in range(120)]
        for s,rid in pair.items():
            for i,(distance,lateral,yaw) in enumerate(cases):
                restore(folded)
                pos=target[s].copy();pos[0]-=distance;pos[1]+=lateral
                w.controllers[rid].set_base_pose_for_test(tuple(pos),math.radians(yaw));settle(.3)
                try:
                    own,top=capture(f'approach-{s}-{i:04d}',s)
                except ValueError as exc:
                    teacher_rows.append({'sample_id':f'approach-{s}-{i:04d}','excluded':str(exc)})
                    continue
                actual=w.controllers[rid].base_xyz();angle=float(w.controllers[rid].base_rpy()[2])
                errors={'forward':target[s][0]-float(actual[0]),'lateral':target[s][1]-float(actual[1]),'yaw':-angle}
                row={'own_jpeg':own,'top_jpeg':top,'case_id':f'pose-{i:04d}','errors':errors,'broad':i<45}
                datasets[s].append(row);teacher_rows.append({'sample_id':f'approach-{s}-{i:04d}','privileged_errors':errors})
                if i%40==0:print(json.dumps({'teacher':'approach','slot':s,'sample':i,'total':len(cases)}),flush=True)
        write(out/'actor-samples.json',actor_rows);write(out/'teacher-labels-only.json',teacher_rows)
        # Fit without an active simulator, preserving saved RGB references.
        stage_out=out/'models'/'varied';stage_out.mkdir(parents=True)
        manifest={'schema':'ugrp.varied_start_skill.v1','models':{},'source':'offline dispatch RGB teacher'}
        for s in pair:
            manifest['models'][s]={}
            for axis in ('yaw','lateral','forward'):
                print(json.dumps({'fit':axis,'slot':s,'samples':len(datasets[s])}),flush=True)
                selected=[r for r in datasets[s] if r['broad']] if axis=='yaw' else datasets[s]
                rows=[{**r,'error':r['errors'][axis],'command':0.,'ready':False} for r in selected]
                m=fit_pose_stage_model(*refs[s],rows,s,axis)
                path=stage_out/f'model-{s}-{axis}.json';write(path,m)
                manifest['models'][s][axis]={'path':path.name,'sha256':sha(path)}
        write(stage_out/'varied-start-skill.json',manifest)
        if args.approach_only:
            report['complete']=True;report['scope']='approach transfer only; no new grasp model'
            return 0
        restore(folded)
        for cmd in skill['initialization_replay'][1:]:arm(normalize_replay(cmd),cmd['duration_s'],cmd.get('settle_s',0))
        neutral={s:{int(ch):int(v) for ch,v in w.controllers[rid].servo_command_pulses.items()} for s,rid in pair.items()}
        arm({s:{ch:neutral[s][ch] for ch in (3,4,5)} for s in pair})
        preclose=snapshot();grasp_refs={s:capture('grasp-reference-'+s,s) for s in pair}
        beam=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'team_beam_geom')
        floor=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
        def close_lift():
            arm({s:{1:skill['close_pulses'][s]} for s in pair},skill['close_duration_s'],skill['close_settle_s'])
            arm({s:{int(c):neutral[s][int(c)]+int(d) for c,d in skill['lift_delta_pulses'][s].items()} for s in pair},
                skill['lift_duration_s'],skill['lift_settle_s'])
            settled=[]
            for _ in range(22):
                settle(.1)
                contacts=[]
                for c in w.data.contact[:w.data.ncon]:
                    if int(c.geom1)==beam:contacts.append(int(c.geom2))
                    elif int(c.geom2)==beam:contacts.append(int(c.geom1))
                names=[mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,g) or '' for g in contacts]
                settled.append(bool(w.data.geom_xpos[beam][2]>.035 and floor not in contacts
                    and all(any(n.startswith(r+'__') for n in names) for r in pair.values()) and not w.data.eq_active.any()))
            return all(settled)
        report['teacher_nominal_grasp_success']=close_lift()
        scene.capture('teacher-nominal-lift')
        if not report['teacher_nominal_grasp_success']:raise RuntimeError('new-scene nominal teacher grasp failed; no grasp model exported')
        grasp_out=out/'models'/'grasp';grasp_out.mkdir(parents=True)
        grasp_manifest=copy.deepcopy(skill);grasp_manifest['models']={}
        grasp_manifest['task_domain']='dispatch_open_v1'
        grasp_manifest.pop('constant_background_top_band',None)
        for s in pair:
            rows=[]
            perturbations=[tuple(int(x) for x in v) for v in rng.integers(-50,51,size=(90,3))]
            for i,delta in enumerate(perturbations):
                restore(preclose)
                arm({s:{ch:neutral[s][ch]+d for ch,d in zip((3,4,5),delta)}})
                own,top=capture(f'grasp-{s}-{i:04d}',s)
                arm({s:{ch:neutral[s][ch] for ch in (3,4,5)}})
                ok=close_lift()
                teacher_rows.append({'sample_id':f'grasp-{s}-{i:04d}','correction_pulses':[-d for d in delta],'physical_teacher_recovery':ok})
                if ok:rows.append({'own_jpeg':own,'top_jpeg':top,'case_id':f'grasp-{i:04d}','correction_pulses':[-d for d in delta]})
                if i%15==0:print(json.dumps({'teacher':'grasp','slot':s,'sample':i,'accepted':len(rows)}),flush=True)
            model=fit_recovery_model(*grasp_refs[s],rows)
            path=grasp_out/f'{s}-model.json';write(path,model)
            grasp_manifest['models'][s]={'path':path.name,'sha256':sha(path)}
            for view,data in zip(('own','top'),grasp_refs[s]):(grasp_out/f'{s}-goal-{view}.jpg').write_bytes(data)
        write(grasp_out/'student-skill.json',grasp_manifest)
        shutil.copy2(args.base_grasp/'evaluation-fixture.json',grasp_out/'evaluation-fixture.json')
        report['complete']=True
    except Exception as exc:
        import traceback
        report['error']=str(exc);(out/'exception.txt').write_text(traceback.format_exc())
        raise
    finally:
        if scene.world:report['camera_geometry_unchanged']=scene.initial_invariants==scene.invariants()
        scene.close();report['wall_s']=time.monotonic()-started
        write(out/'actor-samples.json',actor_rows);write(out/'teacher-labels-only.json',teacher_rows);write(out/'report.json',report)
    return 0

if __name__=='__main__':raise SystemExit(main())
