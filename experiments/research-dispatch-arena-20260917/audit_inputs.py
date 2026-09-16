from pathlib import Path
import sys,json,base64,hashlib,re
root=Path(sys.argv[1]);sys.path.insert(0,str(root))
from sim.research_dispatch_arena import actor_task,digest
from harness.dispatch_plan import validate_dispatch_plan,compile_programs
from harness.three_robot_plan import TeamAgreement

def audit(src):
 result=json.loads((src/'result.json').read_text());cfg=json.loads((src/'episode-setup-only.json').read_text())
 assert result['error'] is None
 assert result['invariants_unchanged'] and result['weld_steps']==0 and result['obstacle_contact_steps']==0
 assert all(x>.01 for x in result['robot_displacement_m'].values())
 assert result['transport_success'] is None
 assert hashlib.sha256((src/'scene.xml').read_bytes()).hexdigest()==result['initial_invariants']['scene_xml_sha256']
 assert actor_task(cfg['static_map'])==json.loads((src/'actor-mission.json').read_text())
 team_path=src/'team/team.json';requests=wires=0;usage={'prompt_tokens':0,'completion_tokens':0,'total_tokens':0}
 if team_path.exists():
  team=json.loads(team_path.read_text());assert re.fullmatch(r'dispatch-[0-9a-f]{12}',team['run_id']);agreement=TeamAgreement(team['run_id'],plan_validator=validate_dispatch_plan)
  for row in team['rounds']:
   assert row['agreement']==agreement.context()
   for rid,refs in row['images'].items():
    req_id=f"{team['run_id']}-{rid}-plan-{row['turn']}"
    req=json.loads((src/'team'/rid/(req_id+'-request.json')).read_text());ctx=json.loads(req['messages'][1]['content']);requests+=1
    assert 'north_blocked' not in req['messages'][1]['content']
    assert set(ctx)=={'robot_id','request_id','reply_binding','reply_envelope','agreement','mission','peer_claims','own_issued_commands'}
    assert ctx['mission']==actor_task(cfg['static_map']) and ctx['agreement']==agreement.context()
    assert ctx['robot_id']==rid and ctx['own_issued_commands']==row['own_history'][rid]
    for label,ref in zip(('own','top'),req['images']):
     actual=base64.b64decode(ref['image'].split(',',1)[1]);record=refs[label]
     assert hashlib.sha256(actual).hexdigest()==record['sha256']
     assert actual==(src/record['path']).read_bytes()
    for call in (c for c in team['calls'] if c['request_id']==req_id and c.get('wire_index')):
     wire=json.loads((src/'team'/rid/f"wire-{call['wire_index']:03d}.json").read_text());wires+=1
     assert wire['messages'][0]==req['messages'][0]
     blocks=wire['messages'][1]['content'];assert blocks[0]['text']==req['messages'][1]['content']
     assert [b['image_url']['url'] for b in blocks if b['type']=='image_url']==[i['image'] for i in req['images']]
     for k in usage:usage[k]+=(call.get('usage') or {}).get(k,0)
   assert agreement.receive(row['replies'],row['turn'])==row['committed']
  assert agreement.committed==team['committed']==result['plan']
  assert not team['transport_roles_fixed_by_skill'] and team['planning_only']
  assert compile_programs(result['plan']['plan'],cfg['static_map'])==json.loads((src/'robot-programs.json').read_text())
 return {'run':str(src),'ok':True,'checked_requests':requests,'checked_wires':wires,'reported_usage':usage,'scope':'input/plan/scene integrity, not visual correctness or transport success'}
rows=[]
for name in ('open','shared','north_blocked','narrow_south','rough_south'):
 rows.append(audit(root/'outputs'/f'dispatch-{name}-v5'))
print(json.dumps(rows,indent=2))
