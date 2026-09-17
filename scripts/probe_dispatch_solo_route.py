#!/usr/bin/env python3
"""Explicit fixture diagnostic: solo RGB transport after a preplaced beam job.

This is not a full three-robot E2E run. One-time authored reset places the beam
and two parked peers at a dock before any actor observations. The solo actor then
uses only its own RGB, common RGB, authored map and own issued command history.
"""
import argparse,json,subprocess,sys,time,traceback
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from sim.research_dispatch_arena import episode
from scripts.run_dispatch_skills import SkillScene
from scripts.run_dispatch_e2e import Referee
from scripts.probe_dual_grasp_sync import Video
from scripts.three_robot_runtime import write
from harness.dispatch_skill_binding import SkillBindings
from harness.three_robot_plan import digest


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--dock',choices=['dock_a','dock_b'],default='dock_a');args=p.parse_args()
 if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('commit source first')
 plan={'dock':args.dock,'tasks':[{'id':'beam_job','object':'beam','participants':['r1','r3'],'route':'north','after':[]},{'id':'box_job','object':'box','participants':['r2'],'route':'south','after':[]}]}
 committed={'plan':plan,'plan_hash':digest(plan),'proposal_id':'explicit-solo-diagnostic','version':1}
 config=episode('open',11);config['contact_solver_profile']='local_contact'
 slot=config['static_map']['docks'][args.dock]['slots']['beam']['center_m']
 config['setup_only']['cargo']['beam']=[*slot,.020]
 for rid,dy in [('r1',.325),('r3',-.325)]:config['setup_only']['spawns'][rid]=[slot[0],slot[1]+dy,.032355118817659255,0.]
 scene=SkillScene(config,args.output);began=time.monotonic();report={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'scope':'solo fixture diagnostic; beam preplaced and peers parked, not full E2E','dock':args.dock,'error':None}
 try:
  scene.open();scene.deadline=time.monotonic()+600
  write(args.output/'episode-setup-only.json',config);write(args.output/'scene-manifest.json',scene.manifest)
  scene.bindings=SkillBindings(committed,config['static_map']);scene.bindings.finish('beam')
  scene.team=SimpleNamespace(agreement=SimpleNamespace(committed=committed))
  scene.video=Video(scene.world,args.output/'execution.mp4',10);scene.referee=Referee(scene);scene.referee.sample();scene.pair_phase='PREPLACED FIXTURE'
  scene.start_solo()
  while not scene.solo.done:scene.step(.2)
 except (Exception,KeyboardInterrupt) as exc:
  report['error']=f'{type(exc).__name__}: {exc}';(args.output/'exception.txt').write_text(traceback.format_exc())
 finally:
  if scene.solo:report['solo_status']={'phase':scene.solo.phase,'reason':scene.solo.reason,'done':scene.solo.done}
  if scene.solo_executor:scene.solo_executor.cancel(scene.time(),'diagnostic_end')
  scene.solo=None;scene.deadline=None;scene.hold();scene.step(1.3);scene.capture('final')
  report['evaluation']=scene.referee.finish(plan);report['obstacle_contact_steps']=scene.obstacle_contact_steps
  report['camera_geometry_unchanged']=scene.initial_invariants==scene.invariants()
  write(args.output/'solo-decisions.json',scene.solo_rows);write(args.output/'solo-raw-actions.json',scene.solo_raw);write(args.output/'issued-commands.json',scene.command_history)
  scene.video.close();scene.close();report['wall_s']=time.monotonic()-began;write(args.output/'result.json',report)
 print(json.dumps(report),flush=True)
 return 0 if report['error'] is None and report['evaluation']['cargo']['box']['physical_success'] else 1

if __name__=='__main__':raise SystemExit(main())
