#!/usr/bin/env python3
"""Evaluate a teacher-trained local RGB correction, with command-playback control.

Fixture initialization replays demonstrated commands before the actor episode.
During the episode, the visual controller sees only RGB and its own issued
commands. Close and lift use demonstrated action playback in BOTH conditions;
this experiment isolates the contribution of learned image-based recovery.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from harness.camera_teacher_student import predict_correction

BOUNDARY='local pregrasp actor: own RGB, shared top RGB, own issued commands, fixed trained image model; no live teacher, IK, coordinates, joint measurements, contacts or evaluator feedback'


def write(p,value):p.write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def normcmd(c):
 return {rid:{int(k):int(v) for k,v in pose.items()} for rid,pose in c['targets'].items()}


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--model-dir',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True)
 p.add_argument('--condition',choices=('visual','playback'),required=True)
 p.add_argument('--perturb',type=int,nargs=3,required=True,metavar=('WRIST','ELBOW','SHOULDER'))
 p.add_argument('--rounds',type=int,default=16);p.add_argument('--max-step',type=int,default=25)
 p.add_argument('--video-fps',type=int,default=4)
 args=p.parse_args()
 if not 1<=args.rounds<=40 or not 1<=args.max_step<=50 or max(map(abs,args.perturb))>100:p.error('bounded local pilot only')
 out=args.out_dir.resolve();out.mkdir(parents=True,exist_ok=False);models_root=args.model_dir.resolve()
 skill=json.loads((models_root/'student-skill.json').read_text())
 fixture=json.loads((models_root/'evaluation-fixture.json').read_text())
 models={}
 for rid,rec in skill['models'].items():
  path=(models_root/rec['path']).resolve()
  if not path.is_relative_to(models_root) or sha(path)!=rec['sha256']:raise ValueError('trained model hash mismatch')
  models[rid]=json.loads(path.read_text())
 import mujoco
 import sim.multi_masterpi_production as production
 from scripts.probe_dual_grasp_sync import Video,_plain_beam_xml,_pose_metrics,_plain_beam_contact,_camera_look_at
 from scripts.run_camera_pair_transport import evaluate_grasp_samples
 started=time.monotonic();world=None;video=None;referee=None;calls=[];evals=[];commands={rid:{} for rid in models}
 report={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
         'input_boundary':BOUNDARY,'skill_sha256':sha(models_root/'student-skill.json'),
         'evaluation_fixture_sha256':sha(models_root/'evaluation-fixture.json'),
         'model_sha256':{rid:v['sha256'] for rid,v in skill['models'].items()},
         'config':{'condition':args.condition,'perturb':args.perturb,'rounds':args.rounds,'max_step':args.max_step,'seed':fixture['seed'],'weld':False},
         'scope':skill['scope'],'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__},
         'calls':calls,'error':None,'evaluation':None}
 next_sample=0.0;actor_started=False
 def evaluate_output():
  nonlocal next_sample
  now=float(world.data.time)
  if not actor_started or now+1e-9<next_sample:return
  sample={**_pose_metrics(world),'contacts':{rid:_plain_beam_contact(world,rid) for rid in models}}
  evals.append(sample);referee.write(json.dumps(sample,sort_keys=True)+'\n');next_sample=now+.1
 def frame_callback():
  evaluate_output()
  if video is not None:video.capture()
 def move(targets,duration_s=.35,settle_s=.10):
  if targets:world._team_joint_move_servos(targets,duration_s,settle_s=settle_s)
  else:
   for _ in range(round((duration_s+settle_s)/float(world.model.opt.timestep))):
    world._physics_step_for(world.controllers['r1']);frame_callback()
  for rid,updates in targets.items():commands[rid].update(updates)
 def snapshots(index):
  top=world.render_team_jpeg(camera='cctv_top',quality=95)
  top_path=f'rgb/{index:03d}-top.jpg';(out/top_path).write_bytes(top)
  rec={};own={}
  for rid in models:
   own[rid]=world.render_jpeg(robot_id=rid,camera='robot_cam',quality=95)
   op=f'rgb/{index:03d}-{rid}-own.jpg';(out/op).write_bytes(own[rid])
   rec[rid]={'own':{'path':op,'sha256':sha(out/op)},'top':{'path':top_path,'sha256':sha(out/top_path)}}
  return own,top,rec
 try:
  with patch.object(production,'build_multi_robot_xml',_plain_beam_xml(production.build_multi_robot_xml)):
   world=production.MultiMasterPiProductionV2(seed=fixture['seed'],width=960,height=720,render=True)
  for rid,pose in fixture['base_poses'].items():world.controllers[rid].set_base_pose_for_test(tuple(pose),0.0)
  topconf=fixture['top_camera']
  for name in ('cctv_top',):
   cid=mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_CAMERA,name)
   world.model.cam_pos[cid]=topconf['position_m'];world.model.cam_quat[cid]=topconf['quaternion_wxyz'];world.model.cam_fovy[cid]=topconf['fov_y_deg']
  mujoco.mj_forward(world.model,world.data)
  # Presentation-only video camera; neither actor RGB view uses this camera.
  _camera_look_at(world)
  # Explicit teacher-initialized curriculum. No inverse kinematics in this file.
  for c in skill['initialization_replay']:move(normcmd(c),c['duration_s'],c.get('settle_s',0.0))
  report['initialization_commands']={rid:dict(c) for rid,c in commands.items()}
  targets={rid:{ch:max(500,min(2500,commands[rid][ch]+args.perturb[i])) for i,ch in enumerate((3,4,5))} for rid in models}
  move(targets,.35,.10)
  report['actor_initial_issued_commands']={rid:dict(c) for rid,c in commands.items()}
  report['applied_perturbation']={rid:[commands[rid][ch]-report['initialization_commands'][rid][ch] for ch in (3,4,5)] for rid in models}
  (out/'rgb').mkdir();referee=(out/'evaluation-only.jsonl').open('w')
  video=Video(world,out/'motion.mp4',args.video_fps);world.frame_callback=frame_callback;actor_started=True
  for index in range(args.rounds):
   own,top,images=snapshots(index);targets={}
   for rid in models:
    before=dict(commands[rid]);decision=predict_correction(models[rid],own[rid],top,max_step=args.max_step)
    applied=decision['delta_pulses'] if args.condition=='visual' else [0,0,0]
    target={ch:max(500,min(2500,before[ch]+int(delta))) for ch,delta in zip(models[rid]['channels'],applied) if delta}
    actions=[{'kind':'arm','servo_id':ch,'pulse':pulse} for ch,pulse in target.items()]
    calls.append({'round':index,'robot_id':rid,'images':images[rid],'own_commands_before':before,
                  'decision':decision,'actions':actions,'condition':args.condition})
    if target:targets[rid]=target
   video.stage=f'{args.condition} recovery {index+1}/{args.rounds}'
   move(targets)
  own,top,images=snapshots(args.rounds)
  report['final_visual_errors']={rid:predict_correction(models[rid],own[rid],top,max_step=args.max_step) for rid in models}
  report['preclose_issued_commands']={rid:dict(c) for rid,c in commands.items()}
  report['post_recovery_images']=images
  video.stage='demonstrated close playback'
  move({rid:{1:int(skill['close_pulses'][rid])} for rid in models},skill['close_duration_s'],skill['close_settle_s'])
  video.stage='demonstrated lift delta playback'
  targets={rid:{int(ch):max(500,min(2500,commands[rid][int(ch)]+int(delta))) for ch,delta in skill['lift_delta_pulses'][rid].items()} for rid in models}
  move(targets,skill['lift_duration_s'],skill['lift_settle_s'])
  video.stage='physical hold evaluation only'
  move({},skill['hold_s'],0.0)
  snapshots(args.rounds+1)
  report['evaluation']=evaluate_grasp_samples(evals)
 except Exception as exc:
  import traceback
  report['error']=f'{type(exc).__name__}: {exc}';report['traceback']=traceback.format_exc()
 finally:
  if world is not None:world.frame_callback=None
  if referee is not None:referee.close()
  if video is not None:video.close()
  if world is not None:world.close()
  report['wall_elapsed_s']=time.monotonic()-started;write(out/'result.json',report)
 print(json.dumps({k:report[k] for k in ('config','error','evaluation','wall_elapsed_s')}))
 return 0 if report['error'] is None else 1
if __name__=='__main__':raise SystemExit(main())
