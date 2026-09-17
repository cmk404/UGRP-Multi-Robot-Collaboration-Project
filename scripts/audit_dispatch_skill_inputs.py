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

def audit(p):
 p=Path(p);result=json.loads((p/'result.json').read_text());team=json.loads((p/'team/team.json').read_text());mission=json.loads((p/'actor-mission.json').read_text());identity=json.loads((p/'identity-evidence.json').read_text())
 def image(ref):
  f=(p/ref['path']).resolve();assert f.is_relative_to(p.resolve());b=f.read_bytes();assert hashlib.sha256(b).hexdigest()==ref['sha256'];return b
 inbox={r:[] for r in ['r1','r2','r3']};count=wire=0
 for row in team['rounds']:
  for rid in inbox:
   request_id=f"{team['run_id']}-{rid}-plan-{row['turn']}"
   ref=row['images'][rid]
   request=build_dispatch_request(rid,task=mission,request_id=request_id,own_rgb=image(ref['own']),top_rgb=image(ref['top']),agreement=row['agreement'],inbox=inbox[rid],own_history=row['own_history'][rid],execution_pilot=True,identity_evidence=identity[rid])
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
 (p/'input-audit.json').write_text(json.dumps(report,indent=2));print(report)
if __name__=='__main__':
 for p in sys.argv[1:]:audit(p)
