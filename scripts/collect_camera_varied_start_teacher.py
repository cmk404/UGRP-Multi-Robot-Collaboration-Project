#!/usr/bin/env python3
"""Collect static RGB calibration or privileged physical varied-start teachers."""
from __future__ import annotations
import argparse,hashlib,json,math,platform,re,subprocess,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.camera_approach_scene import ApproachScene,ROBOTS,validate_start_poses
STAGES=('yaw','lateral','forward');PHYSICAL_STAGES=('yaw','lateral','yaw','forward');LIMITS={'yaw':90,'lateral':110,'forward':160};TOLS={'yaw':.0035,'lateral':.0025,'forward':.004}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write_json(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False)+'\n')
def write_jsonl(p,rows):p.write_text(''.join(json.dumps(r,sort_keys=True,allow_nan=False)+'\n' for r in rows))
def load_cases(path):
 value=json.loads(path.read_text());rows=value.get('cases')
 if not isinstance(rows,list) or not rows:raise ValueError('cases must be a nonempty list')
 seen=set();out=[]
 for row in rows:
  cid=row.get('case_id') if isinstance(row,dict) else None
  if not isinstance(cid,str) or not re.fullmatch(r'[A-Za-z0-9_-]+',cid) or cid in seen:raise ValueError('case_id must be safe and unique')
  seen.add(cid);out.append({'case_id':cid,'start_poses':validate_start_poses(row.get('start_poses'))})
 return out
def clipped(value,low,high,min_abs):
 value=max(low,min(high,float(value)))
 if value and abs(value)<min_abs:value=math.copysign(min_abs,value)
 return value
def teacher_label(stage,rid,xyz,rpy,fixture):
 if stage=='yaw':error=-float(rpy[2]);command=clipped(error*.3,-.06,.06,.004)
 elif stage=='lateral':error=float(fixture[rid][1])-float(xyz[1]);command=clipped(error*.35,-.06,.06,.01)
 else:error=float(fixture[rid][0])-float(xyz[0]);command=clipped(error*.2,-.05,.15,.01)
 ready=abs(error)<=TOLS[stage]
 return {'command':0. if ready else command,'ready':ready,'teacher_truth':{'error':error,'tolerance':TOLS[stage]}}
def actor_and_label(scene,case_id,stage,index,fixture,actor,labels,prefix='',frames=None):
 frames=frames or scene.capture(f'{case_id}-{stage}-{index:03d}')
 for rid in ROBOTS:
  xyz=scene.world.controllers[rid].base_xyz();rpy=scene.world.controllers[rid].base_rpy();sid=f'{case_id}:{stage}:{index:03d}:{rid}'
  obs={k:{**frames[rid][src],'path':prefix+frames[rid][src]['path']} for k,src in (('own_rgb','own_rgb'),('shared_top_rgb','shared_top_rgb'))}
  actor.append({'id':sid,'case_id':case_id,'robot_id':rid,'stage':stage,'observations':obs})
  labels.append({'sample_id':sid,'case_id':case_id,'robot_id':rid,'stage':stage,**teacher_label(stage,rid,xyz,rpy,fixture)})
 return labels[-2:]
def reset(scene,qpos,qvel,start,fixture,case_id):
 import mujoco
 began=scene.time();setup={}
 scene.world.data.qpos[:]=qpos;scene.world.data.qvel[:]=qvel
 for port in scene.ports.values():port.stop()
 for rid in ROBOTS:
  spec=start[rid];base=fixture[rid];pose=(base[0]-spec['distance_m'],base[1]+spec['lateral_m'],base[2])
  scene.world.controllers[rid].set_base_pose_for_test(pose,math.radians(spec['yaw_deg']))
  setup[rid]={'xyz':list(map(float,pose)),'yaw_rad':math.radians(spec['yaw_deg'])}
 mujoco.mj_forward(scene.world.model,scene.world.data);scene.stop_dwell()
 scene.trace.append({'stage':'synthetic_calibration_reset','case_id':case_id,'start_sim_time_s':began,'end_sim_time_s':scene.time(),'setup':setup,'physical_trajectory':False})
def grasp_models(root):
 skillp=root/'student-skill.json';skill=json.loads(skillp.read_text());models={};hashes={}
 if set(skill.get('models',{}))!=set(ROBOTS):raise ValueError('grasp skill must contain r1 and r3')
 for rid in ROBOTS:
  rec=skill['models'][rid];p=(root/rec['path']).resolve()
  if not p.is_relative_to(root) or sha(p)!=rec['sha256']:raise ValueError('grasp model hash mismatch')
  models[rid]=json.loads(p.read_text());hashes[rid]=rec['sha256']
 return models,sha(skillp),hashes
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=('calibration','physical'),required=True);p.add_argument('--grasp-model-dir',type=Path,required=True);p.add_argument('--cases-json',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True);a=p.parse_args()
 casesp=a.cases_json.resolve();cases=load_cases(casesp);gr=a.grasp_model_dir.resolve();models,skillhash,modelhashes=grasp_models(gr)
 import mujoco
 out=a.out_dir.resolve();out.mkdir(parents=True,exist_ok=False);started=time.monotonic();actor=[];labels=[]
 report={'complete':False,'mode':a.mode,'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'cases_sha256':sha(casesp),'grasp_skill_sha256':skillhash,'grasp_model_sha256':modelhashes,'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__},'configuration':{'seed':11,'weld':False,'stages':list(STAGES),'physical_stage_order':list(PHYSICAL_STAGES),'limits':LIMITS,'slice_s':.2,'settled_confirmations':2,'label_source':'privileged base xyz/rpy output only; never actor observations','reset_procedure':'restore initial qpos/qvel; stop motors; set fixture-relative bases; mj_forward; 0.25s stop dwell'},'provenance':{'calibration':'independent synthetic resets; not trajectories or physical successes','physical':'actual mecanum motion followed by RGB grasp recovery'},'successful_cases':[],'excluded_cases':[],'goal_references':{},'cases':[]}
 write_json(out/'report.json',report)
 nominal={r:{'distance_m':.3,'lateral_m':0.,'yaw_deg':0.} for r in ROBOTS};scene=ApproachScene(out/'calibration' if a.mode=='calibration' else out/'reference',gr)
 try:
  scene.open(start_poses=nominal);qpos=scene.world.data.qpos.copy();qvel=scene.world.data.qvel.copy();fixture={r:list(map(float,scene.fixture['base_poses'][r])) for r in ROBOTS};report['reset_reference_state']={'qpos':qpos.tolist(),'qvel':qvel.tolist(),'source_start_poses':nominal}
  reset(scene,qpos,qvel,{r:{'distance_m':0.,'lateral_m':0.,'yaw_deg':0.} for r in ROBOTS},fixture,'goal');report['goal_setup_truth']=scene.evaluation_snapshot();goal=scene.capture('goal')
  for rid in ROBOTS:
   prefix='calibration/' if a.mode=='calibration' else 'reference/';report['goal_references'][rid]={k:{**goal[rid][k],'path':prefix+goal[rid][k]['path']} for k in ('own_rgb','shared_top_rgb')}
  if a.mode=='calibration':
   for case in cases:
    cid=case['case_id'];rec={'case_id':cid,'calibration_valid':False,'error':None}
    try:
     reset(scene,qpos,qvel,case['start_poses'],fixture,cid);rec['reset_truth']=scene.evaluation_snapshot();frames=scene.capture(f'{cid}-static')
     for stage in STAGES:actor_and_label(scene,cid,stage,0,fixture,actor,labels,prefix='calibration/',frames=frames)
     rec['calibration_valid']=True;report['successful_cases'].append(cid)
    except Exception as e:rec['error']=f'{type(e).__name__}: {e}';rec['traceback']=traceback.format_exc();report['excluded_cases'].append(cid)
    report['cases'].append(rec);write_jsonl(out/'actor-samples.jsonl',actor);write_jsonl(out/'teacher-labels.jsonl',labels);write_json(out/'report.json',report);print(json.dumps({'case_id':cid,'calibration_valid':rec['calibration_valid']}),flush=True)
 finally:scene.close()
 if a.mode=='physical':
  from harness.grasp_student_inference import predict_student
  for case in cases:
   cid=case['case_id'];caseout=out/'cases'/cid;s=ApproachScene(caseout,gr);rec={'case_id':cid,'physical_success':False,'error':None,'phase_results':[]}
   try:
    s.open(start_poses=case['start_poses']);fixture={r:list(map(float,s.fixture['base_poses'][r])) for r in ROBOTS};rec['initial_truth']=s.evaluation_snapshot()
    for phase_number,stage in enumerate(PHYSICAL_STAGES):
     settled=0;stationary=False;phase_start=s.time();steps=0
     for i in range(LIMITS[stage]):
      steps=i+1
      rows=actor_and_label(s,cid,stage,phase_number*1000+i,fixture,actor,labels,prefix=f'cases/{cid}/');commands={r:{'forward':0.,'left':0.,'turn':0.} for r in ROBOTS}
      both=all(row['ready'] for row in rows)
      if both and stationary:
       settled+=1
       if settled>=2:break
       s.drive_mecanum(commands,.25)
      elif both:
       settled=0;stationary=True;s.drive_mecanum(commands,.25)
      else:
       settled=0;stationary=False
       for row in rows:commands[row['robot_id']][{'yaw':'turn','lateral':'left','forward':'forward'}[stage]]=row['command']
       s.drive_mecanum(commands,.2)
     if settled<2:raise RuntimeError(f'{stage} teacher budget exhausted')
     rec['phase_results'].append({'phase_index':phase_number,'stage':stage,'steps':steps,'settled_confirmations':settled,'elapsed_sim_s':s.time()-phase_start,'end_truth':s.evaluation_snapshot()})
    rec['approach_end_truth']=s.evaluation_snapshot();rec['approach_elapsed_sim_s']=s.time()-rec['initial_truth']['sim_time_s'];rec['approach_payload_contact_steps']=s.approach_payload_contact_steps;rec['approach_collision_events']=s.approach_collision_events
    s.finish_grasp(predict_student,models,rounds=16);s.grasp_report['source_sha']=report['source_sha'];write_json(caseout/'grasp-result.json',s.grasp_report)
    from scripts.run_camera_pair_transport import evaluate_grasp_samples
    rec['evaluation']=evaluate_grasp_samples(s.evaluation_samples);rec['physical_success']=bool(rec['evaluation']['grasp_success'] and not s.approach_payload_contact_steps);rec['end_truth']=s.evaluation_snapshot()
    (report['successful_cases'] if rec['physical_success'] else report['excluded_cases']).append(cid)
   except Exception as e:
    rec['error']=f'{type(e).__name__}: {e}';rec['traceback']=traceback.format_exc();report['excluded_cases'].append(cid)
    if s.world is not None:
     rec['failure_final_truth']=s.evaluation_snapshot();rec['approach_payload_contact_steps']=s.approach_payload_contact_steps;rec['approach_collision_events']=s.approach_collision_events
   finally:s.close();video=caseout/'motion.mp4';rec['video_sha256']=sha(video) if video.is_file() else None;report['cases'].append(rec);write_jsonl(out/'actor-samples.jsonl',actor);write_jsonl(out/'teacher-labels.jsonl',labels);write_json(out/'report.json',report);print(json.dumps({'case_id':cid,'physical_success':rec['physical_success'],'error':rec['error']}),flush=True)
 report['complete']=all(case.get('error') is None for case in report['cases']);report['wall_elapsed_s']=time.monotonic()-started;write_json(out/'report.json',report);print(json.dumps({'complete':report['complete'],'successful_cases':report['successful_cases'],'excluded_cases':report['excluded_cases']}),flush=True);return 0 if report['complete'] else 1
if __name__=='__main__':raise SystemExit(main())
