#!/usr/bin/env python3
"""Fixed raw mecanum probes; truth is recorded for calibration, never control."""
import argparse,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.camera_approach_scene import ApproachScene,ROBOTS

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--grasp-model-dir',type=Path,required=True)
 p.add_argument('--out-dir',type=Path,required=True)
 args=p.parse_args();out=args.out_dir.resolve();out.mkdir(parents=True,exist_ok=False)
 rows=[]
 for axis in ['forward','left','turn','r1_forward']:
  scene=ApproachScene(out/axis,args.grasp_model_dir)
  try:
   scene.open(start_poses={r:{'distance_m':.3,'lateral_m':0.,'yaw_deg':0.} for r in ROBOTS})
   start=scene.evaluation_snapshot();scene.capture('before')
   commands={r:{k:(.03 if (k==axis or axis=='r1_forward' and r=='r1' and k=='forward') else 0.) for k in ['forward','left','turn']} for r in ROBOTS}
   for _ in range(5):scene.drive_mecanum(commands,.2)
   driven=scene.evaluation_snapshot();scene.capture('after-drive')
   scene.drive_mecanum({r:{k:0. for k in ['forward','left','turn']} for r in ROBOTS},.5)
   final=scene.evaluation_snapshot();scene.capture('after-stop')
   row={'axis':axis,'command':commands,'start':start,'driven':driven,'after_stop':final,'contact_steps':scene.approach_payload_contact_steps}
   rows.append(row);print(json.dumps({'axis':axis,'start':start['bases'],'after_stop':final['bases'],'contact_steps':row['contact_steps']}),flush=True)
  finally:scene.close()
 report={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'trials':rows,'complete':True}
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 return 0
if __name__=='__main__':raise SystemExit(main())
