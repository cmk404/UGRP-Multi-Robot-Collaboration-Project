"""Run both endurance conditions sequentially from a frozen commit."""
from pathlib import Path
import argparse,json,subprocess,time
ROOT=Path(__file__).resolve().parents[2]
def main():
 p=argparse.ArgumentParser();p.add_argument('--mjpython',type=Path,required=True);p.add_argument('--grasp-model-dir',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True)
 p.add_argument('--impratio',type=int,choices=(1,10,100),default=10)
 p.add_argument('--noslip-iterations',type=int,choices=(0,3),default=0)
 p.add_argument('--spacing-integral',type=float,choices=(0.,2.),default=0.)
 p.add_argument('--finger-friction-damping',type=int,choices=(0,3000),default=0)
 a=p.parse_args();out=a.out_dir.resolve();out.mkdir(parents=True,exist_ok=False)
 sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip();rec={'source_sha':sha,'trials':[]}
 for mode in ['stationary','shuttle']:
  assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==sha
  assert not subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip()
  cmd=[str(a.mjpython.resolve()),str(ROOT/'scripts/run_pair_grasp_endurance.py'),'--map',str(ROOT/'maps/pair_navigation/narrow-door.json'),'--grasp-model-dir',str(a.grasp_model_dir.resolve()),'--out-dir',str(out/mode),'--mode',mode,'--noslip-iterations',str(a.noslip_iterations),'--impratio',str(a.impratio),'--spacing-integral',str(a.spacing_integral),'--finger-friction-damping',str(a.finger_friction_damping)]
  print('START',mode,flush=True);t=time.monotonic()
  with (out/(mode+'.log')).open('w') as f:r=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=900)
  result=json.loads((out/mode/'result.json').read_text())
  rec['trials'].append({'mode':mode,'command':cmd,'returncode':r.returncode,'wall_seconds':time.monotonic()-t,'success':result['success'],'error':result['error'],'evaluation':result['evaluation']})
  (out/'cohort.json').write_text(json.dumps(rec,indent=2)+'\n');print('END',mode,result['success'],flush=True)
if __name__=='__main__':main()
