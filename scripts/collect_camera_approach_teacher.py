#!/usr/bin/env python3
"""Collect privileged labels for the bounded dual RGB approach policy."""
from __future__ import annotations
import argparse,hashlib,json,platform,subprocess,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.camera_approach_scene import ApproachScene,ROBOTS,MAX_APPROACH_ROUNDS
from scripts.run_camera_approach_cohort import load_cases
def write(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load_models(root):
 s=root/'student-skill.json';skill=json.loads(s.read_text());models={};hashes={}
 for rid in ROBOTS:
  rec=skill['models'][rid];p=(root/rec['path']).resolve()
  if not p.is_relative_to(root) or sha(p)!=rec['sha256']:raise ValueError(f'{rid} grasp model hash mismatch')
  models[rid]=json.loads(p.read_text());hashes[rid]=rec['sha256']
 return models,sha(s),hashes
def aggregate(out,cases):
 actors=[];labels=[]
 for c in cases:
  cid=c['id'];prefix=f'cases/{cid}/'
  for row in json.loads((out/'cases'/cid/'actor_samples.json').read_text()):
   row['observations']={k:{**v,'path':prefix+v['path']} for k,v in row['observations'].items()};actors.append(row)
  labels.extend(json.loads((out/'cases'/cid/'privileged_labels.json').read_text()))
 write(out/'actor_samples.json',actors);write(out/'privileged_labels.json',labels)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--grasp-model-dir',type=Path,required=True);p.add_argument('--cases-json',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True);a=p.parse_args()
 cp=a.cases_json.resolve();cases=load_cases(cp);gr=a.grasp_model_dir.resolve();models,skillhash,modelhashes=load_models(gr)
 import mujoco
 out=a.out_dir.resolve();out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
 report={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'cases_sha256':sha(cp),'grasp_skill_sha256':skillhash,'grasp_model_sha256':modelhashes,'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__},'config':{'seed':11,'weld':False,'slice_s':.2,'stop_dwell_s':.25,'teacher_forward_gain_per_m':.2,'teacher_min_moving_forward':.01,'max_rounds':MAX_APPROACH_ROUNDS},'successful_cases':[],'excluded_cases':[],'goal_references':{},'cases':[],'complete':False};write(out/'report.json',report)
 ref=ApproachScene(out/'reference',gr)
 try:
  ref.open({r:0 for r in ROBOTS});ref.stop_dwell();frames=ref.capture('goal')
  for rid in ROBOTS:
   own={**frames[rid]['own_rgb'],'path':'reference/'+frames[rid]['own_rgb']['path']};top={**frames[rid]['shared_top_rgb'],'path':'reference/'+frames[rid]['shared_top_rgb']['path']};report['goal_references'][rid]={'own_rgb':own,'shared_top_rgb':top,'base_x':float(ref.world.controllers[rid].base_xyz()[0])}
 finally:ref.close()
 for case in cases:
  cid=case['id'];caseout=out/'cases'/cid;scene=ApproachScene(caseout,gr);actors=[];labels=[];hist={r:[] for r in ROBOTS};rec={'id':cid,'requested_distance_m':case['distance_m'],'error':None,'approach_ok':False,'grasp_success':False};print(json.dumps({'case_id':cid,'status':'starting'}),flush=True)
  try:
   scene.open(case['distance_m']);rec['initial_physical_state']=scene.evaluation_snapshot();goals={r:float(report['goal_references'][r]['base_x']) for r in ROBOTS};rec['actual_base_x_offset_m']={r:goals[r]-float(scene.world.controllers[r].base_xyz()[0]) for r in ROBOTS}
   for i in range(MAX_APPROACH_ROUNDS):
    frames=scene.capture(f'approach-{i:03d}');speeds={};both=True
    for rid in ROBOTS:
     remaining=goals[rid]-float(scene.world.controllers[rid].base_xyz()[0]);stop=abs(remaining)<=.004;forward=0. if stop else (min(.15,max(.01,.2*remaining)) if remaining>0 else 0.);both&=stop;sid=f'{cid}:{i:03d}:{rid}';actors.append({'id':sid,'case_id':cid,'robot_id':rid,'observations':{'own_rgb':frames[rid]['own_rgb'],'shared_top_rgb':frames[rid]['shared_top_rgb']},'own_command_history':list(hist[rid])});labels.append({'sample_id':sid,'case_id':cid,'robot_id':rid,'forward':forward,'stop':bool(stop)});speeds[rid]=forward
    if both:rec['approach_ok']=True;break
    for rid in ROBOTS:hist[rid].append({'kind':'drive','forward':speeds[rid],'turn':0.,'duration_s':.2})
    scene.drive(speeds)
   scene.stop_dwell()
   for confirm in range(2):
    for rid in ROBOTS:hist[rid].append({'kind':'drive','forward':0.,'turn':0.,'duration_s':.25})
    frames=scene.capture(f'stationary-{confirm}')
    for rid in ROBOTS:
     remaining=goals[rid]-float(scene.world.controllers[rid].base_xyz()[0]);stop=abs(remaining)<=.004;sid=f'{cid}:stationary-{confirm}:{rid}'
     actors.append({'id':sid,'case_id':cid,'robot_id':rid,'observations':{'own_rgb':frames[rid]['own_rgb'],'shared_top_rgb':frames[rid]['shared_top_rgb']},'own_command_history':list(hist[rid])})
     labels.append({'sample_id':sid,'case_id':cid,'robot_id':rid,'forward':0. if stop else (min(.15,max(.01,.2*remaining)) if remaining>0 else 0.),'stop':bool(stop)})
     rec['approach_ok']=bool(rec['approach_ok'] and stop)
    if confirm==0:scene.stop_dwell()
   rec['approach_end_state']=scene.evaluation_snapshot()
   rec['approach_elapsed_sim_s']=float(scene.world.data.time)-rec['initial_physical_state']['sim_time_s']
   if rec['approach_ok']:
    from harness.grasp_student_inference import predict_student
    scene.finish_grasp(predict_student,models,rounds=16)
   from scripts.run_camera_pair_transport import evaluate_grasp_samples
   rec['evaluation']=evaluate_grasp_samples(scene.evaluation_samples);rec['grasp_success']=bool(rec['evaluation']['grasp_success']);rec['approach_payload_contact_steps']=scene.approach_payload_contact_steps;good=rec['approach_ok'] and rec['grasp_success'] and not rec['approach_payload_contact_steps'];(report['successful_cases'] if good else report['excluded_cases']).append(cid)
  except Exception as e:rec['error']=f'{type(e).__name__}: {e}';rec['traceback']=traceback.format_exc();report['excluded_cases'].append(cid)
  finally:
   scene.close();caseout.mkdir(parents=True,exist_ok=True);video=caseout/'motion.mp4';rec['video_sha256']=sha(video) if video.is_file() else None;write(caseout/'actor_samples.json',actors);write(caseout/'privileged_labels.json',labels);write(caseout/'result.json',rec);report['cases'].append(rec);aggregate(out,cases[:len(report['cases'])]);write(out/'report.json',report)
  print(json.dumps({'case_id':cid,'status':'complete','approach_ok':rec['approach_ok'],'grasp_success':rec['grasp_success'],'error':rec['error']}),flush=True)
 report['complete']=True;report['wall_elapsed_s']=time.monotonic()-started;write(out/'report.json',report);print(json.dumps({'complete':True,'successful_cases':report['successful_cases'],'excluded_cases':report['excluded_cases'],'wall_elapsed_s':report['wall_elapsed_s']}),flush=True);return 0
if __name__=='__main__':raise SystemExit(main())
