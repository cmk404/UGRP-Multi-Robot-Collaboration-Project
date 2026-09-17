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
 calls=json.loads((p/'pair-decisions.json').read_text());bindings=result['bindings']['pair_model_slots'];reference=Path(result['config']['reference_top']).read_bytes()
 _,stage=load_stage_models(Path(result['config']['stage_model_dir']));_,grasp=models(p.with_name(p.name+'-grasp-models').resolve(),'student-skill.json')
 transforms=approach=grasp_count=0
 for call in calls:
  if call['kind']=='image_binding':
   assert call['pair_binding']==bindings
   for slot,rid in bindings.items():assert call['own'][slot]['path'].endswith('-'+rid+'.jpg');image(call['own'][slot])
   meta=call['transform'];data,new=canonical_pair_top(image(call['raw_top']),reference,translation_px=meta['translation_px'] if meta['fixed_from_prior_rgb'] else None,hue_upper=meta.get('hue_upper',24))
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
 if replay_solo:
  setup=json.loads((p/'episode-setup-only.json').read_text())
  report['solo_decision_replay']=audit_solo_replay(p,json.loads((p/'committed-plan.json').read_text()),setup['static_map'])
 setup=json.loads((p/'episode-setup-only.json').read_text())
 report['navigation_replay']=audit_navigation(p,calls,json.loads((p/'committed-plan.json').read_text()),setup['static_map'])
 (p/'input-audit.json').write_text(json.dumps(report,indent=2));print(report)
def audit_solo_replay(p,committed,static_map):
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
 return {'actions_replayed':len(rows),'source':'archived own RGB + TOP RGB + own command state + recorded resource wait signals','scope':'controller decision replay; resource ownership checked separately, not physical success'}

def audit_navigation(p,calls,committed,static_map):
 from harness.dispatch_navigation_map import navigation_map
 from harness.dispatch_pair_navigation import PairNavigator,authorize_pair
 from harness.dispatch_skill_binding import SkillBindings,BeamContinuity
 from harness.pair_carry_sync import PairCarrySync
 from harness.camera_goal_transport import own_payload
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
   data=navigation_map(bindings,raw(last['raw_top']));assert data==call['map']
   agents={r:PairNavigator(data,r) for r in bindings.pair}
   sync=PairCarrySync('dispatch-'+bindings.committed['plan_hash']);anchor=copy.deepcopy(last['own'])
  elif call['kind']=='rotating_carry':
   assert agents is not None
   decisions={}
   for r in agents:
    own=raw(call['images'][r]['own']);top=raw(call['images'][r]['top'])
    value=agents[r].decide(own,top)
    current,initial=own_payload(own,hue_upper=35),own_payload(raw(anchor[r]),hue_upper=35)
    held=bool(current and initial and .25<=current[0]/initial[0]<=4 and math.dist(current[1:],initial[1:])<=.15)
    value['own_attachment']={'held_estimate':held,'current':current,'anchor':initial}
    value['ready']=value['ready'] and held;decisions[r]=value
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
