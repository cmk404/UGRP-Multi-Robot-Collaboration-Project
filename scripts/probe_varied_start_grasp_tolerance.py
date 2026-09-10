#!/usr/bin/env python3
"""Measure the existing RGB grasp student's heading tolerance at setup only."""
import argparse,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.camera_approach_scene import ApproachScene,ROBOTS
from scripts.run_camera_approach_student import models,write
from scripts.run_camera_pair_transport import evaluate_grasp_samples
from harness.grasp_student_inference import predict_student

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out-dir',type=Path,required=True);p.add_argument('--grasp-model-dir',type=Path,required=True);a=p.parse_args()
 if a.out_dir.exists():raise FileExistsError(a.out_dir)
 a.out_dir.mkdir(parents=True);skill,ms=models(a.grasp_model_dir,'student-skill.json');source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip();results=[]
 for heading in (.25,.5,.75,1.):
  for sign in (-1,1):
   cid=f'yaw-{heading:g}-sign-{sign}';dest=a.out_dir/cid;scene=ApproachScene(dest,a.grasp_model_dir);starts={r:{'distance_m':.003,'lateral_m':(-1 if r=='r1' else 1)*.002,'yaw_deg':heading*sign*(-1 if r=='r1' else 1)}for r in ROBOTS};row={'case_id':cid,'source_sha':source,'setup_only':starts,'error':None}
   try:
    scene.open(start_poses=starts);row['initial_state']=scene.evaluation_snapshot();row['calls']=scene.finish_grasp(predict_student,ms);scene.grasp_report['source_sha']=source;write(dest/'grasp-result.json',scene.grasp_report);row['evaluation']=evaluate_grasp_samples(scene.evaluation_samples);row['success']=row['evaluation']['grasp_success'];row['final_state']=scene.evaluation_snapshot()
   except Exception as e:row['error']=repr(e);row['success']=False
   finally:scene.close()
   write(dest/'result.json',row);results.append(row);write(a.out_dir/'report.json',{'source_sha':source,'purpose':'setup perturbation tolerance of existing RGB grasp, not approach validation','complete':len(results)==8,'results':results});print(json.dumps({k:row[k]for k in ('case_id','success','error')}),flush=True)
 return 0
if __name__=='__main__':raise SystemExit(main())
