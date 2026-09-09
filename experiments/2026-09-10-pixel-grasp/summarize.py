"""Output-only evidence summarizer; never imported by camera controllers."""
from pathlib import Path
import json,sys,collections,math,gzip

def read_record(path):
 return path.read_text() if path.exists() else gzip.decompress(path.with_name(path.name+'.gz').read_bytes()).decode()

def summarize(root):
 root=Path(root);r=json.loads(read_record(root/'result.json'));all_calls=r['calls'];prefix=r.get('prefix_round_count',0);calls=[c for c in all_calls if c['round']>=prefix];a=[c for c in calls if c['active']]
 data={'root':str(root),'source_sha':r['source_sha'],'config':r['config'],'rounds_completed':r['rounds_completed'],'error':r['error'],'wall_elapsed_s':r['wall_elapsed_s'],'model_calls':0,'stages':dict(collections.Counter(c['decision']['stage'] for c in a)),'trial_closures':sum(c['decision']['stage']=='lift' and c['action'].get('servo_id')==1 and c['action'].get('pulse')==1500 for c in a),'identification_closures':sum(c['decision']['stage']!='lift' and c['action'].get('servo_id')==1 and c['action'].get('pulse')==1500 for c in a),'robots':{}}
 data.update({'prefix_round_count':prefix,'newly_executed_round_count':r.get('newly_executed_round_count',r['rounds_completed']),'evaluation_scope':r.get('evaluation_scope','full_run'),'metrics_scope':'newly_executed_calls_only','resume':r.get('resume'),'new_execution_wall_elapsed_s':r.get('new_execution_wall_elapsed_s',r['wall_elapsed_s'])})
 for rid in set(c['robot_id'] for c in calls):
  rc=[c for c in calls if c['robot_id']==rid];vals=[(c['round'],c['observation']['alignment']) for c in rc if c['observation'].get('alignment') is not None]
  pulse=2000;fresh=[]
  for previous in all_calls:
   if previous['round']>=prefix:break
   if previous['robot_id']==rid and previous['action'].get('servo_id')==1:pulse=previous['action']['pulse']
  for c in rc:
   observation=c['observation']
   if pulse==2000 and observation.get('alignment') is not None and observation['gripper']['source']=='isolated_gripper_motion':fresh.append((c['round'],observation['alignment']))
   if c['action'].get('servo_id')==1:pulse=c['action']['pulse']
  data['robots'][rid]={'minimum_fresh_open_alignment':min(fresh,key=lambda p:p[1]['cost']) if fresh else None,'first_alignment':vals[0] if vals else None,'minimum_alignment':min(vals,key=lambda p:p[1]['cost']) if vals else None,'last_alignment':vals[-1] if vals else None,'fresh_measurements':sum(c['observation']['gripper']['source']=='isolated_gripper_motion' for c in rc),'unavailable_reasons':dict(collections.Counter(c['observation']['gripper']['reason'] for c in rc if not c['observation']['gripper']['valid']))}
 samples=[json.loads(l) for l in read_record(root/'evaluation-only.jsonl').splitlines()];truth={'dual_grasp_success':r['grasp_success'],'max_lift_m':r['max_lift_m'],'longest_dual_hold_s':r['longest_qualifying_duration_s'],'contacts':{}}
 for rid in ('r1','r3'):
  truth['contacts'][rid]={key:sum(bool(e['contacts'][rid][key]) for e in samples) for key in ['left','right','bilateral']}
  truth['contacts'][rid]['max_bilateral_lift_m']=max((e['height_above_start_m'] for e in samples if e['contacts'][rid]['bilateral']),default=None)
  start=None;longest=0.
  for e in samples:
   valid=e['contacts'][rid]['bilateral'] and e['height_above_start_m']>=.03 and not any(e['constraints_active'].values())
   if valid:
    if start is None:start=e['sim_time_s']
    longest=max(longest,e['sim_time_s']-start)
   else:start=None
  truth['contacts'][rid]['longest_bilateral_3cm_hold_s']=longest
 data['output_only_evaluation']=truth
 return data
if __name__=='__main__':print(json.dumps([summarize(p) for p in sys.argv[1:]],indent=2))
