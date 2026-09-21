"""Replay stored RGB decisions and summarize a completed direct-motion comparison.

Usage: python scripts/audit_jev_motion.py COHORT OUTPUT
"""
from pathlib import Path
import collections
import hashlib
import json
import math
import statistics
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.jev_motion import RGBObserver,policy_state,at_goal,rule,validate_jev,validate_gemini
from sim.research_dispatch_arena import FIXED_TOP
import numpy as np

base=Path(sys.argv[1]);report_dir=Path(sys.argv[2]);report_dir.mkdir(parents=True,exist_ok=True)
policies=['rule','gemini','jev'];cases=['straight','left_offset','right_offset']
results=[];audits=[]
for case in cases:
 for policy in policies:
  folder=base/(policy+'-'+case)
  result=json.loads((folder/'result.json').read_text())
  rows=json.loads((folder/'turns.json').read_text()) if (folder/'turns.json').exists() else []
  identity=json.loads((folder/'identity.json').read_text()) if (folder/'identity.json').exists() else None
  if identity:
   observer=RGBObserver({'top_camera':FIXED_TOP},(folder/'rgb/probe-before-top.jpg').read_bytes(),
      (folder/'rgb/probe-after-top.jpg').read_bytes(),np.array(identity['claim']['center'])*[959,719])
  history=[];hashes=0;calls=[];max_rgb_range_error=0.;max_rgb_bearing_error=0.
  referee=json.loads((folder/'referee-only.json').read_text())
  for row in rows:
   content={}
   for name,v in row['images'].items():
    data=(folder/v['path']).read_bytes()
    assert hashlib.sha256(data).hexdigest()==v['sha256'];hashes+=1;content[name]=data
   observed=observer.observe(content['own_rgb'],content['shared_top_rgb'])
   assert observed==row['observation']
   assert policy_state(observed,history)==row['state']
   origin=row['decision_source']
   if origin=='rule':assert rule(row['state'])==row['action']
   elif origin=='common_RGB_goal_confirmation':assert at_goal(observed) and row['action']=='stop'
   else:
    req=json.loads((folder/f"{row['turn']:03d}-request.json").read_text())
    response=json.loads((folder/f"{row['turn']:03d}-response.json").read_text())
    assert req==row['request'] and response==row['response']
    if policy=='jev':assert req['state']==row['state'];expected=validate_jev(response['body'])
    else:assert json.loads(req['messages'][1]['content'])==row['state'];expected=validate_gemini(response['body'])
    assert expected==row['action'];calls.append(response)
   truth=min(referee,key=lambda x:abs(x['sim_s']-row['observed_at_sim_s']))
   max_rgb_range_error=max(max_rgb_range_error,abs(observed['range_m']-truth['range_m']))
   error=(observed['bearing_deg']-truth['bearing_deg']+180)%360-180
   max_rgb_bearing_error=max(max_rgb_bearing_error,abs(error))
   history.append({'action':row['action'],'range_m':observed['range_m'],'bearing_deg':observed['bearing_deg']})
  tail=[s for s in referee if s['sim_s']>=referee[-1]['sim_s']-.4]
  physical=(tail[-1]['sim_s']-tail[0]['sim_s']>=.35 and all(.26<=s['range_m']<=.30 and abs(s['bearing_deg'])<=6 for s in tail))
  expected=bool(result['stop_reason']=='RGB_goal_confirmed' and physical and not any(result[k] for k in ('cargo_contact_steps','peer_contact_steps','obstacle_contact_steps','weld_steps')) and result['camera_geometry_unchanged'])
  assert result['success']==expected
  audit={'policy':policy,'case':case,'rgb_replayed':len(rows),'image_hashes':hashes,'actions_verified':len(rows),
         'physical_success_recomputed':expected,'max_observed_range_error_m':max_rgb_range_error,
         'max_observed_bearing_error_deg':max_rgb_bearing_error}
  audits.append(audit)
  calls=[json.loads(p.read_text()) for p in sorted(folder.glob('*-response.json'))]
  assert len(calls)==result['model_calls']
  result['physical_goal_geometry_only']=physical
  result['choice_probability_mismatches']=sum(x.get('choice_probability_mismatch',False) for x in rows)
  latencies=[c['latency_s'] for c in calls]
  result['api_latency_s']=latencies
  result['action_counts']=dict(collections.Counter(x['action'] for x in rows))
  result['model_stop_before_RGB_goal']=sum(x['action']=='stop' and x['decision_source'] in ('jev','gemini') and not at_goal(x['observation']) for x in rows)
  result['decision_latency_s']=[x['observation_to_issue_wall_s'] for x in rows if x['decision_source']!='common_RGB_goal_confirmation']
  results.append(result)
summary={}
for policy in policies:
 rows=[r for r in results if r['policy']==policy];lat=[v for r in rows for v in r['api_latency_s']]
 summary[policy]={'successes':sum(r['success'] for r in rows),'episodes':len(rows),
  'wall_s':sum(r['wall_s'] for r in rows),'sim_s':sum(r['sim_s'] for r in rows),
  'commands':sum(r['commands'] for r in rows),'model_calls':sum(r['model_calls'] for r in rows),
  'input_tokens':sum(r['input_tokens'] for r in rows),'output_tokens':sum(r['output_tokens'] for r in rows),
  'api_latency_median_s':statistics.median(lat) if lat else None,
  'api_latency_p95_s':sorted(lat)[math.ceil(.95*len(lat))-1] if lat else None,
  'api_latency_max_s':max(lat) if lat else None,'cost_usd_billed':None,
  'choice_probability_mismatches':sum(r['choice_probability_mismatches'] for r in rows),
  'early_model_stops':sum(r['model_stop_before_RGB_goal'] for r in rows)}
report={'source_sha':results[0]['source_sha'],'summary':summary,'results':results,'audits':audits,
 'limitations':['one repetition per pose/policy; fixed policy order','fixed scene and limited yaw domain',
 'SIM paused during inference; not real-time','only approach/alignment, no grasp/carry',
 'classical RGB observation shared by policies, not direct image understanding',
 'API billing unverified; raw RGB/video local only']}
(report_dir/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
# All prior attempts retained and listed separately; never pooled with final success rates.
dev=[]
for p in sorted(base.parent.glob('jev-direct-motion-20260921-*/*/result.json')):
 if base in p.parents:continue
 d=json.loads(p.read_text());dev.append({'path':str(p),**d})
(report_dir/'prior-attempt-results.json').write_text(json.dumps(dev,ensure_ascii=False,indent=2)+'\n')
manifest=[]
for folder in sorted(base.parent.glob('jev-direct-motion-20260921-*')):
 if not folder.is_dir():continue
 for p in sorted(folder.rglob('*')):
  if p.is_file():
   b=p.read_bytes();assert b'apikey_' not in b
   manifest.append({'path':str(p),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()})
(report_dir/'raw-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(summary,indent=2));print('Audited RGB decisions:',sum(a['rgb_replayed'] for a in audits),'raw files:',len(manifest))
