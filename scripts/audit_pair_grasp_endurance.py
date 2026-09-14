"""Replay saved RGB-only endurance inputs and separately check referee outputs."""
from pathlib import Path
import argparse,hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1]
if ROOT.name=='outputs':ROOT=ROOT.parent
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from harness.pair_grasp_endurance import EnduranceActor,DT
from harness.pair_grasp_spacing import PairGraspSpacing
from harness.pair_navigation import ROBOTS,authorize_pair,digest
from harness.pair_carry_sync import PairCarrySync
from scripts.audit_pair_carry_sync import _rgb,_same,_safe_file
from scripts.audit_camera_grasp_student import audit as audit_grasp
from scripts.run_pair_grasp_endurance import evaluate_endurance

def audit(root,models):
 root=Path(root).resolve();models=Path(models).resolve();r=json.loads(_safe_file(root,'result.json').read_text())
 if r['schema']!='ugrp.pair_grasp_endurance.v1' or digest(r['map'])!=r['map_sha256']:raise ValueError('schema/map mismatch')
 seen={rid:set() for rid in ROBOTS}
 for kind in ('spacing','endurance'):
  actors={rid:(PairGraspSpacing(r['map'],rid,integral_gain=r.get('spacing_integral',0.)) if kind=='spacing' else EnduranceActor(r['map'],rid,r['mode'],integral_gain=r.get('spacing_integral',0.))) for rid in ROBOTS}
  sync=PairCarrySync(r['map']['map_id']+('-grasp-spacing' if kind=='spacing' else '-endurance'))
  rows=r['spacing_steps'] if kind=='spacing' else r['steps']
  for i,row in enumerate(rows):
   expected={'index','images','frame_ids','decisions','permission','issued_actions','sim_time_s'}|({'executed'} if kind=='spacing' else set())
   if set(row)!=expected or row['index']!=i:raise ValueError('unexpected actor input or sequence')
   decisions={}
   for rid in ROBOTS:
    if set(row['images'][rid])!={'own','top'} or row['frame_ids'][rid] in seen[rid]:raise ValueError('image input/freshness mismatch')
    seen[rid].add(row['frame_ids'][rid]);decisions[rid]=actors[rid].decide(_rgb(root,row['images'][rid]['own']),_rgb(root,row['images'][rid]['top']))
    if not _same(decisions[rid],row['decisions'][rid]):raise ValueError(f'RGB decision mismatch {kind}/{i}/{rid}')
   if row['images']['r1']['top']!=row['images']['r3']['top']:raise ValueError('different common top frame')
   permission=authorize_pair(sync,decisions,row['frame_ids'],i,interval_s=DT)
   if permission!=row['permission']:raise ValueError('permission mismatch')
   for rid in ROBOTS:
    action=dict(decisions[rid]['action'])
    if permission['phase']!='GO':action.update(forward=0.,left=0.,turn=0.)
    if not _same(action,row['issued_actions'][rid]):raise ValueError('command mismatch')
  if not _same(sync.events,r['spacing_sync_events' if kind=='spacing' else 'sync_events']):raise ValueError('sync event mismatch')
 if not r['error'] and (len(r['steps'])!=round(r['duration_s']/DT) or len(r['spacing_steps'])!=104):raise ValueError('missing duration/initial hold')
 trace=json.loads(_safe_file(root,'execution-trace.json').read_text())
 close=[row for row in trace if row['stage']=='grasp_close'];skill=json.loads((models/'student-skill.json').read_text())
 if len(close)!=1 or close[0]['command']['targets']!={rid:{'1':r['close_pulse']} for rid in ROBOTS}:raise ValueError('undeclared fixed grasp command')
 if close[0]['command']['duration_s']!=skill['close_duration_s'] or close[0]['command']['settle_s']!=skill['close_settle_s']:raise ValueError('grasp timing changed')
 if r.get('endurance_trace_version')==1:
  drive=[x for x in trace if x['stage']=='endurance_command']
  if len(drive)!=len(r['steps']):raise ValueError('missing actuator trace')
  for cmd,row in zip(drive,r['steps']):
   if not _same(cmd['actions'],row['issued_actions']) or abs(cmd['end_sim_time_s']-cmd['start_sim_time_s']-DT)>1e-6:raise ValueError('actual actuator trace mismatch')
 for name,sha in r['grasp_model_files'].items():
  if hashlib.sha256((models/name).read_bytes()).hexdigest()!=sha:raise ValueError('model file changed')
 grasp=audit_grasp(root,models,report_name='grasp-result.json')
 if not grasp['ok']:raise ValueError('grasp input audit failed')
 rows=[json.loads(x) for x in _safe_file(root,'evaluation-only.jsonl').read_text().splitlines()]
 events=json.loads(_safe_file(root,'contact-events-evaluation-only.json').read_text())
 if sum(x['wall'] for x in events)!=r['wall_contact_ticks'] or sum(x['unexpected'] for x in events)!=r['unexpected_contact_ticks']:raise ValueError('collision event mismatch')
 evaluated=evaluate_endurance(rows,r)
 if not _same(evaluated,r['evaluation']):raise ValueError('output-only verdict mismatch')
 if r['success']!=bool(not r['error'] and not r.get('cleanup_error') and evaluated['success']):raise ValueError('false success')
 if hashlib.sha256(_safe_file(root,'scene.xml').read_bytes()).hexdigest()!=r['scene_xml_sha256']:raise ValueError('scene hash mismatch')
 if r.get('finger_friction_damping',0):
  pairs=r['invariants_initial']['explicit_contact_pairs']
  if pairs['pair_solreffriction'] != [[0.,-r['finger_friction_damping']]]*4:raise ValueError('finger friction profile mismatch')
 if r['invariants_initial']['contact_solver']['impratio']!=r['impratio']:raise ValueError('impedance declaration mismatch')
 if r['invariants_initial']['contact_solver']['noslip_iterations']!=r['noslip_iterations']:raise ValueError('solver declaration mismatch')
 return {'passed':True,'source_sha':r['source_sha'],'spacing_rounds':len(r['spacing_steps']),'endurance_rounds':len(r['steps']),'grasp':grasp,'physical_success':r['success'],'scope':'Exact saved RGB/command replay and output-only scoring; not a physical rerun'}
def main():
 p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--grasp-model-dir',type=Path,required=True);a=p.parse_args();r=audit(a.run_dir,a.grasp_model_dir)
 (a.run_dir/'input-audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:r[k] for k in ['passed','endurance_rounds','physical_success']}))
if __name__=='__main__':main()
