import copy
import pytest
from pathlib import Path
from types import SimpleNamespace
from harness.dispatch_feasibility import inspect_routes,negotiate_executable
from harness.dispatch_plan import fixture_plan
from harness.three_robot_plan import TeamAgreement,digest
from sim.research_dispatch_arena import authored_map

RAW=Path('tests/fixtures/dispatch_adaptive/north-barrier.jpg').read_bytes()
def commit(route):
 p=fixture_plan(dock='dock_a',route=route)
 if route=='south':p['tasks'][1]['route']='south'
 return {'plan':p,'plan_hash':digest(p),'version':1,'proposal_id':'test'}

def box_first(value):
 value=copy.deepcopy(value)
 beam,box=value['plan']['tasks']
 beam['after']=[box['id']];box['after']=[]
 value['plan_hash']=digest(value['plan'])
 return value

def test_same_prior_map_has_different_feasibility_from_actual_rgb():
 static=authored_map();a=inspect_routes(commit('north'),static,RAW)
 b=inspect_routes(commit('south'),static,RAW)
 c=inspect_routes(box_first(commit('south')),static,RAW)
 assert not a['feasible'] and not b['feasible'] and c['feasible']
 assert b['beam_routes']['south']['after_box_delivery_and_yield_feasible']
 assert c['beam_routes']['south']['conditional_on_box_delivery_and_yield']
 assert a['beam_routes']['north']['observed_barriers']
 assert 'swept' in a['beam_routes']['north']['reason']


def test_pickup_allocation_must_fit_observed_approach_not_fixed_robot_names():
 import json
 from harness.dispatch_feasibility import inspect_pickup_approach
 from harness.dispatch_skill_binding import SkillBindings
 root=Path('tests/fixtures/dispatch_adaptive')
 raw=(root/'blocked-pickup-top.jpg').read_bytes()
 value=json.loads((root/'blocked-pickup-input.json').read_text())
 reference=Path('tests/fixtures/camera_goal_transport/reference-top.jpg').read_bytes()
 c=value['committed'];identity=value['claims']
 report=inspect_pickup_approach(SkillBindings(c,authored_map('narrow_south')),raw,identity,reference)
 assert not report['feasible'] and report['robots']['r2']['blocking_robots']==['r3']
 c['plan']['tasks'][0]['participants']=['r1','r3'];c['plan']['tasks'][1]['participants']=['r2']
 c['plan_hash']=digest(c['plan'])
 assert inspect_pickup_approach(SkillBindings(c,authored_map()),raw,identity,reference)['feasible']
 # Renaming image-bound identities preserves feasibility; no hardcoded pair.
 names={'r1':'r2','r2':'r3','r3':'r1'}
 for task in c['plan']['tasks']:task['participants']=[names[r] for r in task['participants']]
 c['plan_hash']=digest(c['plan']);identity={names[r]:v for r,v in identity.items()}
 assert inspect_pickup_approach(SkillBindings(c,authored_map()),raw,identity,reference)['feasible']

@pytest.mark.parametrize('live',[False,True])
def test_rejected_route_requires_new_plan_and_three_new_exact_votes(tmp_path,live):
 task={};agreement=TeamAgreement('feedback',plan_validator=lambda p:p)
 class Team:
  def __init__(self):
   self.mode='fixture';self.plan_fixture={}
   self.output=tmp_path/'team';self.calls=[];self.rounds=[];self.events=[];self.agreement=agreement
  def save(self):pass
  def event(self,name,*args,**kwargs):self.events.append(name)
  def negotiate(self,frames,history,turn,sim_time):
   ctx=agreement.context();pending=ctx['proposal']
   proposal=fixture_plan(dock='dock_a',route='south' if 'execution_feedback' in task else 'north')
   if 'execution_feedback' in task:
    proposal['tasks'][1]['route']='south'
    proposal=box_first({'plan':proposal})['plan']
   replies={rid:{'request_id':f'feedback-{rid}-plan-{turn}','proposal_id':pending['proposal_id'] if pending else None,
      'plan_hash':pending['plan_hash'] if pending else None,'accept':True,'plan':pending['plan'] if pending else proposal,
      'reason':'protocol test policy','message':''} for rid in ('r1','r2','r3')}
   self.rounds.append({'turn':turn});return agreement.receive(replies,turn)
 team=Team();report=negotiate_executable(team,{'r1':{'top_bytes':RAW}},{},task,authored_map(),0,live_replan=live)
 assert report['feasible'] and len(team.rounds)==4
 assert team.events==['REPLAN_REQUIRED']+(['FIXTURE_TO_LIVE_REPLAN_DIAGNOSTIC'] if live else [])
 assert team.mode==('llm' if live else 'fixture')
 assert [x['event'] for x in agreement.events]==['PROPOSED','COMMITTED','INVALIDATED','PROPOSED','COMMITTED']
 assert agreement.committed['version']==2
 assert agreement.committed['plan']['tasks'][0]['route']=='south'
 assert agreement.committed['plan']['tasks'][0]['after']==['box_job']
 assert task['execution_feedback']['rejected_plan']['plan']['tasks'][0]['route']=='north'
