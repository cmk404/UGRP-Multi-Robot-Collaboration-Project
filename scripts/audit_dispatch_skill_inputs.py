#!/usr/bin/env python3
"""Read-only reconstruction of saved RGB inputs, model predictions and LLM wires."""
from pathlib import Path
import sys,json,hashlib,base64
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.dispatch_skill_binding import canonical_pair_top
from harness.dispatch_plan import build_dispatch_request
from harness.camera_varied_start_student import predict_stage
from harness.grasp_student_inference import predict_student
from harness.gemini_proxy import _to_gemini_multi_image_messages
from scripts.run_camera_varied_start_student import load_stage_models
from scripts.run_camera_approach_student import models

def audit_translation_history(calls):
 previous=None;frozen=None;count=0
 for call in calls:
  if call['kind']!='image_binding':continue
  meta=call['transform'];shift=meta['translation_px']
  if meta['fixed_from_prior_rgb']:
   if frozen is None:frozen=previous
   assert frozen is not None and shift==frozen,'grasp image transform differs from prior RGB anchor'
   count+=1
  else:frozen=None
  previous=shift
 return count

def audit(p,*,replay_solo=False):
 p=Path(p);result=json.loads((p/'result.json').read_text());team=json.loads((p/'team/team.json').read_text());mission=json.loads((p/'actor-mission.json').read_text());identity=json.loads((p/'identity-evidence.json').read_text())
 def image(ref):
  f=(p/ref['path']).resolve();assert f.is_relative_to(p.resolve());b=f.read_bytes();assert hashlib.sha256(b).hexdigest()==ref['sha256'];return b
 inbox={r:[] for r in ['r1','r2','r3']};count=wire=0
 for row in team['rounds']:
  for rid in inbox:
   request_id=f"{team['run_id']}-{rid}-plan-{row['turn']}"
   ref=row['images'][rid]
   request=build_dispatch_request(rid,task=row.get('task_snapshot',mission),request_id=request_id,own_rgb=image(ref['own']),top_rgb=image(ref['top']),agreement=row['agreement'],inbox=inbox[rid],own_history=row['own_history'][rid],execution_pilot=True,identity_evidence=identity[rid])
   saved=json.loads((p/'team'/rid/(request_id+'-request.json')).read_text());assert saved==request,('request reconstruction',request_id);count+=1
  for rid,reply in row['replies'].items():
   if reply and reply['message']:
    for other in inbox:
     if rid!=other:inbox[other].append({'from_robot':rid,'turn':row['turn'],'message':reply['message']})
 for call in team['calls']:
  if call.get('wire_index'):
   saved=json.loads((p/'team'/call['request']).read_text());w=json.loads((p/'team'/call['robot_id']/f"wire-{call['wire_index']:03d}.json").read_text())
   assert w['messages']==_to_gemini_multi_image_messages(saved['messages'],saved['images']);wire+=1
 calls=json.loads((p/'pair-decisions.json').read_text());anchored=audit_translation_history(calls);bindings=result['bindings']['pair_model_slots'];reference=Path(result['config']['reference_top']).read_bytes()
 _,stage=load_stage_models(Path(result['config']['stage_model_dir']));_,grasp=models(p.with_name(p.name+'-grasp-models').resolve(),'student-skill.json')
 transforms=approach=grasp_count=0
 from harness.dispatch_beam_tracker import CarriedBeamTracker
 shaft=CarriedBeamTracker()
 for call in calls:
  if call['kind']=='image_binding':
   assert call['pair_binding']==bindings
   for slot,rid in bindings.items():assert call['own'][slot]['path'].endswith('-'+rid+'.jpg');image(call['own'][slot])
   meta=call['transform'];observed=shaft.observe(image(call['raw_top'])) if meta.get('tracked_carried_shaft') else None
   data,new=canonical_pair_top(image(call['raw_top']),reference,translation_px=meta['translation_px'] if meta['fixed_from_prior_rgb'] else None,hue_upper=meta.get('hue_upper',24),observed_beam=observed)
   assert data==image(call['derived_top']);assert new['translation_px']==meta['translation_px'];transforms+=1
  if call['kind']=='learned_approach':
   for a in call['report']['approach_calls']:
    prediction=predict_stage(stage[a['robot_id']][a['stage']],image(a['images']['own']),image(a['images']['top']))
    assert prediction==a['decision'],('stage',a['stage'],a['index']);approach+=1
 rec=json.loads((p/'pair-grasp.json').read_text())
 for a in rec.get('calls',[]):
  pred=predict_student(grasp[a['robot_id']],image(a['images']['own']),image(a['images']['top']),max_step=25)
  assert pred==a['decision'];grasp_count+=1
 solo=json.loads((p/'solo-decisions.json').read_text())
 for row in solo:
  own=image(row['images']['own']);image(row['images']['top']);obs=row['observation']
  assert set(obs)=={'robot_id','frame_id','sim_time','sha256','camera','actuator_state'}
  assert obs['robot_id']==result['bindings']['solo_robot'] and obs['sha256']==hashlib.sha256(own).hexdigest()
  assert set(obs['actuator_state'])=={'motor_commands','servo_pulses'}
 report={'passed':True,'scope':'saved input reconstruction and exact model prediction replay; not physical success', 'run':str(p.resolve()),'planning_requests':count,'actual_llm_wires':wire,'pair_image_bindings':transforms,'learned_approach_predictions':approach,'grasp_predictions':grasp_count,'solo_own_command_observations':len(solo)}
 report['prior_rgb_grasp_anchors_verified']=anchored
 if replay_solo:
  report['solo_decision_replay']=audit_solo_replay(p,json.loads((p/'committed-plan.json').read_text()),mission['static_map'])
 yield_vision,yield_report=audit_yield(p,mission['static_map'])
 report['yield_replay']=yield_report
 report['navigation_replay']=audit_navigation(p,calls,json.loads((p/'committed-plan.json').read_text()),mission['static_map'],yield_vision=yield_vision,identity=identity)
 (p/'input-audit.json').write_text(json.dumps(report,indent=2));print(report)
def audit_solo_replay(p,committed,static_map,*,return_policy=False):
 import copy,base64,json,hashlib
 from harness.dispatch_skill_binding import SkillBindings,ImageRoute
 from harness.solo_box_transport import SoloBoxTransport
 bindings=SkillBindings(committed,static_map)
 policy=SoloBoxTransport(robot_id=bindings.solo,navigator=ImageRoute(bindings,'box'),attachment_min_saturation=150,release_refine_ground_fit=True)
 rows=json.loads((p/'solo-decisions.json').read_text())
 for row in rows:
  obs=copy.deepcopy(row['observation']);raw=(p/row['images']['own']['path']).read_bytes();top=(p/row['images']['top']['path']).read_bytes()
  assert hashlib.sha256(raw).hexdigest()==obs['sha256']
  assert hashlib.sha256(top).hexdigest()==row['images']['top']['sha256']
  obs['image']=base64.b64encode(raw).decode()
  assert policy.phase==row['phase_before'],row['index']
  backup=copy.deepcopy(policy.box) if policy.phase=='approach' else None
  action,evidence=policy.decide(obs,top)
  if row['top_evidence'].get('waiting_before_grasp'):
   assert backup is not None and policy.phase=='lower'
   policy.box=backup;action={'kind':'wait','duration':.3}
  if row['top_evidence'].get('waiting_for_resource'):
   assert action['kind']=='mecanum'
   action={'kind':'wait','duration':.1}
  # JSON normalizes integer servo channel keys exactly as the archived wire.
  assert json.loads(json.dumps(action))==row['action'],('solo action',row['index'])
  assert policy.phase==row['phase_after'],('solo phase',row['index'])
 if return_policy:return policy
 return {'actions_replayed':len(rows),'source':'archived own RGB + TOP RGB + own command state + recorded resource wait signals','scope':'controller decision replay; resource ownership checked separately, not physical success'}

def audit_yield(p,static_map):
 from harness.dispatch_yield import SoloYield
 rows=json.loads((p/'solo-yield.json').read_text()) if (p/'solo-yield.json').exists() else []
 if not rows:return None,{'actions_replayed':0}
 committed=json.loads((p/'committed-plan.json').read_text())
 solo=audit_solo_replay(p,committed,static_map,return_policy=True)
 assert solo.done and solo.reason=='VISUAL_RELEASE_CONFIRMED'
 policy=SoloYield(static_map,solo.navigator.box_center)
 for index,row in enumerate(rows):
  for ref in row['images'].values():assert hashlib.sha256((p/ref['path']).read_bytes()).hexdigest()==ref['sha256']
  if index==0:
   assert row['action']=={'kind':'pose','pulses':{'1':2000,'3':740,'4':2320,'5':1320,'6':1500}}
  else:
   action,evidence=policy.decide((p/row['images']['top']['path']).read_bytes())
   evidence['initial_cargo_center_px']=solo.navigator.box_center.tolist()
   assert json.loads(json.dumps(action))==row['action'],('yield action',index)
   assert json.loads(json.dumps(evidence))==row['evidence'],('yield RGB evidence',index)
  assert row['box_job_finished']==policy.done
 return policy.vision,{'actions_replayed':len(rows),'visually_cleared_bay':policy.done,'source':'current RGB and authored staging goals; no referee input'}

def audit_navigation(p,calls,committed,static_map,*,yield_vision=None,identity=None):
 from harness.dispatch_navigation_map import navigation_map
 from harness.dispatch_pair_navigation import PairNavigator,authorize_pair
 from harness.dispatch_skill_binding import SkillBindings,BeamContinuity
 from harness.pair_carry_sync import PairCarrySync
 from harness.camera_goal_transport import own_payload
 from harness.dispatch_own_hold import OwnHoldContinuity
 import copy,math,json,hashlib
 bindings=SkillBindings(committed,static_map);continuity=BeamContinuity()
 agents=sync=None;last=None;anchor=None;count=0
 def raw(ref):
  value=(p/ref['path']).read_bytes();assert hashlib.sha256(value).hexdigest()==ref['sha256'];return value
 for call in calls:
  if call['kind']=='image_binding':
   last=call
   if call['transform']['hue_upper']==35:continuity.observe(call['transform']['observed_beam'])
  elif call['kind']=='navigation_map':
   top=raw(last['raw_top']);other=None
   if 'other_robot_observation' in call:
    from harness.dispatch_yield import WheelObserver
    from harness.camera_goal_transport import decode
    import numpy as np
    source=call['other_robot_source']
    if source['source']=='solo_yield_history':
     assert yield_vision is not None
     other=yield_vision.observe(top)
    else:
     frame=decode(top);claim=identity[bindings.solo]['claim'];assert claim['valid']
     hint=np.array(claim['center'])*[frame.shape[1]-1,frame.shape[0]-1]
     assert hint.tolist()==source['hint_px']
     other=WheelObserver(static_map,hint).observe(top)
    assert json.loads(json.dumps(other))==call['other_robot_observation']
   data=navigation_map(bindings,top,other_robot_center_px=other['center_px'] if other else None);assert data==call['map']
   agents={r:PairNavigator(data,r) for r in bindings.pair}
   sync=PairCarrySync('dispatch-'+bindings.committed['plan_hash']);anchor=copy.deepcopy(last['own'])
   own_guards={r:OwnHoldContinuity(raw(anchor[r])) for r in bindings.pair}
  elif call['kind']=='rotating_carry':
   assert agents is not None
   decisions={}
   for r in agents:
    own=raw(call['images'][r]['own']);top=raw(call['images'][r]['top'])
    value=agents[r].decide(own,top)
    value['own_attachment']=own_guards[r].observe(own)
    value['ready']=value['ready'] and value['own_attachment']['held_estimate'];decisions[r]=value
    assert json.loads(json.dumps(value))==call['decisions'][r],('rotation RGB replay',count,r)
   if 'frame_ids' in call:
    permission=authorize_pair(sync,decisions,call['frame_ids'],count)
    assert permission==call['permission'],('rotation sync',count)
   count+=1
 return {'rgb_rotation_decisions_replayed':count,'source':'archived RGB + authored map + exact plan; no referee input'}

if __name__=='__main__':
 import argparse
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('runs',nargs='+')
 parser.add_argument('--replay-solo',action='store_true',help='exact solo action replay using this source revision')
 args=parser.parse_args()
 for run_path in args.runs:audit(run_path,replay_solo=args.replay_solo)
