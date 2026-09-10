#!/usr/bin/env python3
"""Fit a local RGB goal recovery model from privileged-teacher demonstrations.

Teacher coordinates/IK may produce labels; the exported correction model has
only image features and pulse-to-image derivatives. Close/lift remain explicit
learned demonstration-command playback in this first local curriculum stage.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from harness.camera_teacher_student import fit_visual_jacobian, predict_correction


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')
def load_rgb(root,record):
 p=(root/record['path']).resolve()
 if not p.is_relative_to(root.resolve()) or sha(p)!=record['sha256']:raise ValueError('RGB path/hash mismatch')
 return p.read_bytes()


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--teacher-dir',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True)
 args=p.parse_args();teacher=args.teacher_dir.resolve();out=args.out_dir.resolve()
 if out.exists():raise FileExistsError(out)
 report=json.loads((teacher/'run_report.json').read_text())
 if not report['ok']:raise ValueError('teacher failed; retain the dataset but do not export a successful skill')
 actors=json.loads((teacher/'actor_samples.json').read_text())
 manifest=json.loads((teacher/'manifest.json').read_text())
 out.mkdir(parents=True);started=time.monotonic();fit_reports={};model_records={}
 for rid in ('r1','r3'):
  reference=next(c for c in actors if c['robot_id']==rid and c['phase']=='preclose_open')
  ref_own=load_rgb(teacher,reference['observations']['own_rgb']);ref_top=load_rgb(teacher,reference['observations']['shared_top_rgb'])
  samples=[];sample_ids=[]
  for c in actors:
   if c['robot_id']!=rid or c['phase']!='preclose_probe_after' or not c['sample_id'].startswith('probe_'+rid+'_'):continue
   before_id=c['sample_id'].replace('_after:', '_before:')
   before=next(b for b in actors if b['sample_id']==before_id)
   ref_pulse=before['own_issued_command_snapshot'];pulse=c['own_issued_command_snapshot']
   samples.append({'before_own_jpeg':load_rgb(teacher,before['observations']['own_rgb']),
                   'before_top_jpeg':load_rgb(teacher,before['observations']['shared_top_rgb']),
                   'own_jpeg':load_rgb(teacher,c['observations']['own_rgb']),
                   'top_jpeg':load_rgb(teacher,c['observations']['shared_top_rgb']),
                   'delta_pulses':[int(pulse[str(ch)])-int(ref_pulse[str(ch)]) for ch in (3,4,5)]})
   sample_ids.append(c['sample_id'])
  if len(samples)<12:raise ValueError(f'{rid}: expected signed local probes on three servos')
  model=fit_visual_jacobian(ref_own,ref_top,samples)
  fit_reports[rid]={'diagnostics':model['diagnostics'],'reference_sample':reference['sample_id'],'probe_samples':sample_ids,
                   'in_sample_predictions':[{'delta_label':s['delta_pulses'],**predict_correction(model,s['own_jpeg'],s['top_jpeg'],max_step=100)} for s in samples]}
  write(out/f'{rid}-model.json',model)
  (out/f'{rid}-goal-own.jpg').write_bytes(ref_own);(out/f'{rid}-goal-top.jpg').write_bytes(ref_top)
  model_records[rid]={'path':f'{rid}-model.json','sha256':sha(out/f'{rid}-model.json')}
 # Export demonstrated actions, clearly distinct from the image regression.
 command_path=teacher/'teacher_commands.json'
 commands=json.loads(command_path.read_text())
 # The collector records preclose as the first open IK goal before any probes.
 first_probe=next((i for i,c in enumerate(commands) if c.get('phase')=='preclose_probe'),len(commands))
 close_index=next(i for i,c in enumerate(commands) if c.get('phase')=='closed')
 init_commands=commands[:min(first_probe,close_index)]
 if not init_commands:raise ValueError('missing teacher initialization commands')
 stage_ref={rid:next(c for c in actors if c['robot_id']==rid and c['phase']=='preclose_open') for rid in ('r1','r3')}
 stage_lift={rid:next(c for c in actors if c['robot_id']==rid and c['phase']=='lifted') for rid in ('r1','r3')}
 stage_close={rid:next(c for c in actors if c['robot_id']==rid and c['phase']=='closed') for rid in ('r1','r3')}
 learned_lift={rid:{str(ch):int(stage_lift[rid]['own_issued_command_snapshot'][str(ch)])-int(stage_close[rid]['own_issued_command_snapshot'][str(ch)]) for ch in (3,4,5)} for rid in ('r1','r3')}
 close_command=next(c for c in commands if c['phase']=='closed')
 lift_command=next(c for c in commands if c['phase']=='lifted')
 skill={'schema':'ugrp.local_rgb_grasp_skill.v1','scope':'teacher-initialized local pregrasp; RGB learned correction; demonstrated close/lift playback; no navigation',
        'models':model_records,'initialization_replay':init_commands,
        'close_pulses':{rid:int(stage_close[rid]['own_issued_command_snapshot']['1']) for rid in ('r1','r3')},
        'lift_delta_pulses':learned_lift,'close_duration_s':close_command['duration_s'],'close_settle_s':close_command['settle_s'],'lift_duration_s':lift_command['duration_s'],'lift_settle_s':lift_command['settle_s'],'hold_s':2.2}
 write(out/'student-skill.json',skill)
 write(out/'evaluation-fixture.json',{'schema':'ugrp.local_grasp_evaluation_fixture.v1','actor_access':False,
       'base_poses':report['setup_base_poses_m'],'seed':manifest['config']['seed'],'top_camera':manifest['config']['top_camera']})
 write(out/'training-report.json',{'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
       'teacher_dir':str(teacher),'teacher_manifest_sha256':sha(teacher/'manifest.json'),'teacher_actor_samples_sha256':sha(teacher/'actor_samples.json'),
       'teacher_success_verified':True,'models':fit_reports,'wall_elapsed_s':time.monotonic()-started,
       'runtime_input':'own RGB, fixed shared top RGB, own issued command history; no live teacher/IK/coordinates/contact',
       'limitation':'in-sample fit is not a rollout success; local initialization and close/lift playback are separate from learned RGB correction'})
 print(json.dumps({'out_dir':str(out),'fit':{rid:fit_reports[rid]['diagnostics'] for rid in fit_reports},'elapsed_s':time.monotonic()-started}))
 return 0
if __name__=='__main__':raise SystemExit(main())
