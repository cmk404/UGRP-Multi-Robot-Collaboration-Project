"""Audit completed RGB-only semantic motion cohort. Usage: COHORT OUTPUT."""
from pathlib import Path
import argparse,collections,gzip,hashlib,json,math,statistics,sys
import numpy as np
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('cohort',type=Path);parser.add_argument('output',type=Path)
parser.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1])
args=parser.parse_args()
ROOT=args.source_root
sys.path.insert(0,str(ROOT))
from harness.jev_motion import RGBObserver,policy_state,rule,at_goal,validate_jev,validate_gemini
from scripts.run_jev_motion import model_request,physical_goal_confirmed
from sim.research_dispatch_arena import FIXED_TOP
from harness.camera_motion_identity import ImageMotionIdentity
base=args.cohort;out=args.output;out.mkdir(parents=True,exist_ok=True)
protocol=json.loads((base/'protocol.json').read_text());results=[];audits=[];all_turns={};manifest=[]
for job in protocol['jobs']:
 folder=base/job['trial_id'];r=json.loads((folder/'result.json').read_text());rows=json.loads((folder/'turns.json').read_text()) if (folder/'turns.json').exists() else []
 assert r['source_sha']==protocol['source_sha']
 assert all(r[k]==job[k] for k in ('case','policy','state_variant','repeat','partition','trial_id'))
 if (folder/'identity.json').exists():
  identity=json.loads((folder/'identity.json').read_text())
  identify=ImageMotionIdentity();identify.update((folder/'rgb/probe-before-top.jpg').read_bytes(),None)
  anchor=identify.update((folder/'rgb/probe-after-top.jpg').read_bytes(),{'kind':'drive','forward':.1,'turn':0.,'duration_s':.2})
  assert anchor==identity['claim']
  obs=RGBObserver({'top_camera':FIXED_TOP},(folder/'rgb/probe-before-top.jpg').read_bytes(),(folder/'rgb/probe-after-top.jpg').read_bytes(),np.array(identity['claim']['center'])*[959,719])
  assert obs.probe_evidence==identity['probe']
 history=[];hashes=0;request_turns=[]
 for row in rows:
  images={}
  for k,image in row['images'].items():
   data=(folder/image['path']).read_bytes();assert hashlib.sha256(data).hexdigest()==image['sha256'];images[k]=data;hashes+=1
  observed=obs.observe(images['own_rgb'],images['shared_top_rgb'])
  assert observed==row['observation'];assert policy_state(observed,history)==row['state']
  if row['decision_source']=='common_RGB_goal_confirmation':assert at_goal(observed) and row['action']=='stop'
  elif row['decision_source']=='rule':assert rule(row['state'])==row['action']
  else:
   request_turns.append(row['turn'])
   req=json.loads((folder/f"{row['turn']:03d}-request.json").read_text());res=json.loads((folder/f"{row['turn']:03d}-response.json").read_text())
   assert req==row['request'] and res==row['response']
   assert req==model_request(row['state'],r['policy'],r['state_variant'],'jev-1.13.0' if r['policy']=='jev' else 'gemini-3.8-flash')
   assert row['action']==(validate_jev(res['body']) if r['policy']=='jev' else validate_gemini(res['body']))
  history.append(dict(action=row['action'],range_m=observed['range_m'],bearing_deg=observed['bearing_deg']))
 # A rejected API response can end an episode before a command row is appended.
 unexecuted=[]
 for p in sorted(folder.glob('*-request.json')):
  turn=int(p.name.split('-')[0])
  if turn in request_turns:continue
  assert turn==len(rows) and r['error'] is not None
  observed=obs.observe((folder/f'rgb/{turn:03d}-r2.jpg').read_bytes(),(folder/f'rgb/{turn:03d}-top.jpg').read_bytes())
  assert json.loads(p.read_text())==model_request(policy_state(observed,history),r['policy'],r['state_variant'],'jev-1.13.0' if r['policy']=='jev' else 'gemini-3.8-flash')
  unexecuted.append(turn)
 truth=json.loads((folder/'referee-only.json').read_text());tail=[x for x in truth if x['sim_s']>=truth[-1]['sim_s']-.4]
 stable_s=tail[-1]['sim_s']-tail[0]['sim_s']
 geometry=all(.26<=x['range_m']<=.30 and abs(x['bearing_deg'])<=6 for x in tail)
 physical=stable_s>=.35 and geometry
 physical_tolerant=(stable_s>=.35 or math.isclose(stable_s,.35,rel_tol=0,abs_tol=1e-9)) and geometry
 assert physical_tolerant==physical_goal_confirmed(truth)
 success=bool(r['stop_reason']=='RGB_goal_confirmed' and physical and not any(r[x] for x in ('weld_steps','cargo_contact_steps','peer_contact_steps','obstacle_contact_steps')) and r['camera_geometry_unchanged'])
 assert r['success']==success
 responses=[json.loads(p.read_text()) for p in sorted(folder.glob('*-response.json'))];assert len(responses)==r['model_calls']
 r['stable_tail_s']=stable_s;r['physical_goal_geometry_only']=geometry
 r['physical_goal_original']=physical;r['physical_goal_time_tolerant']=physical_tolerant
 r['audited_success']=bool(r['stop_reason']=='RGB_goal_confirmed' and physical_tolerant and not any(r[x] for x in ('weld_steps','cargo_contact_steps','peer_contact_steps','obstacle_contact_steps')) and r['camera_geometry_unchanged'])
 r['numerical_time_correction']=r['audited_success']!=r['success']
 r['actions']=dict(collections.Counter(x['action'] for x in rows))
 r['api_latencies_s']=[x['latency_s'] for x in responses]
 r['early_stop_count']=sum(x['action']=='stop' and not at_goal(x['observation']) for x in rows)
 model_rows=[x for x in rows if x['decision_source'] in ('jev','gemini')]
 r['reference_rule_agreement']=sum(x['action']==rule(x['state']) for x in model_rows)
 r['model_actions']=len(model_rows)
 r['turns_against_target_side']=sum(abs(x['observation']['bearing_deg'])>3 and x['action']==('turn_right' if x['observation']['bearing_deg']>0 else 'turn_left') for x in model_rows)
 r['choice_probability_mismatches']=sum(x.get('choice_probability_mismatch',False) for x in rows)
 audits.append(dict(trial_id=r['trial_id'],rgb_decisions=len(rows),image_hashes=hashes,unexecuted_request_turns=unexecuted,success_recomputed=success))
 results.append(r);all_turns[r['trial_id']]=rows
summary={}
for partition in ('new','regression'):
 summary[partition]={}
 for policy,variant in [('rule','numeric'),('jev','numeric'),('jev','semantic'),('gemini','numeric'),('gemini','semantic')]:
  arm=policy+'-'+variant;rs=[r for r in results if r['partition']==partition and r['policy']==policy and r['state_variant']==variant];lat=[x for r in rs for x in r['api_latencies_s']]
  summary[partition][arm]=dict(successes=sum(r['success'] for r in rs),audited_successes=sum(r['audited_success'] for r in rs),time_roundoff_corrections=[r['trial_id'] for r in rs if r['numerical_time_correction']],per_case_audited={case:sum(r['audited_success'] for r in rs if r['case']==case) for case in sorted({r['case'] for r in rs})},episodes=len(rs),per_case={case:sum(r['success'] for r in rs if r['case']==case) for case in sorted({r['case'] for r in rs})},audited_failure_reasons=dict(collections.Counter(r['error']['message'] if r['error'] else r['stop_reason'] for r in rs if not r['audited_success'])),failure_reasons=dict(collections.Counter(r['error']['message'] if r['error'] else r['stop_reason'] for r in rs if not r['success'])),wall_s=sum(r['wall_s'] for r in rs),sim_s=sum(r['sim_s'] for r in rs),commands=sum(r['commands'] for r in rs),model_calls=sum(r['model_calls'] for r in rs),input_tokens=sum(r['input_tokens'] for r in rs),output_tokens=sum(r['output_tokens'] for r in rs),latency_median_s=statistics.median(lat) if lat else None,latency_p95_s=sorted(lat)[math.ceil(.95*len(lat))-1] if lat else None,audited_success_only_median_commands=statistics.median([r['commands'] for r in rs if r['audited_success']]) if any(r['audited_success'] for r in rs) else None,success_only_median_commands=statistics.median([r['commands'] for r in rs if r['success']]) if any(r['success'] for r in rs) else None,final_physical_goal_without_completion=sum(r['physical_goal_geometry_only'] and r['stop_reason']!='RGB_goal_confirmed' for r in rs),early_stops=sum(r['early_stop_count'] for r in rs),reference_rule_agreement=sum(r['reference_rule_agreement'] for r in rs),model_actions=sum(r['model_actions'] for r in rs),turns_against_target_side=sum(r['turns_against_target_side'] for r in rs),choice_probability_mismatches=sum(r['choice_probability_mismatches'] for r in rs),cost_usd_billed=None)
starts=collections.defaultdict(set)
for r in results:
 rows=all_turns[r['trial_id']]
 if rows:starts[r['case']].add(tuple(rows[0]['images'][k]['sha256'] for k in ('own_rgb','shared_top_rgb')))
assert all(len(v)==1 for v in starts.values())
report=dict(source_sha=protocol['source_sha'],summary=summary,results=results,audits=audits,matched_start_rgb_pairs={k:[list(x) for x in v] for k,v in sorted(starts.items())},evaluation_correction={'kind':'post-hoc numerical audit, original success retained','min_stable_s':.35,'absolute_time_roundoff_tolerance_s':1e-9,'scope':'output-only elapsed-time comparison; geometry/contact/control criteria unchanged'},limitations=['12 unique new starts with 3 API repeats, fixed physics','same map and object; not new topology or multiple cargo','semantic state and wording jointly changed','SIM pauses during inference; 4 concurrent workers','classical RGB observer and common completion gate unchanged','reference rule agreement is exploratory, not optimal-action accuracy','raw images and video local only; actual billed cost unverified'])
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
(out/'turns.json.gz').write_bytes(gzip.compress((json.dumps(all_turns,indent=2)+'\n').encode(),mtime=0))
for p in sorted(base.rglob('*')):
 if p.is_file():
  data=p.read_bytes();assert b'apikey_' not in data
  manifest.append(dict(path=str(p),bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
(out/'raw-manifest.json.gz').write_bytes(gzip.compress((json.dumps(manifest,indent=2)+'\n').encode(),mtime=0))
print(json.dumps(summary,indent=2));print('Audited',len(results),'trials and',sum(x['rgb_decisions'] for x in audits),'RGB decisions')
