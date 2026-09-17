import copy
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

def test_same_prior_map_has_different_feasibility_from_actual_rgb():
 static=authored_map();a=inspect_routes(commit('north'),static,RAW)
 b=inspect_routes(commit('south'),static,RAW)
 assert not a['feasible'] and b['feasible']
 assert a['beam_routes']['north']['observed_barriers']
 assert 'swept' in a['beam_routes']['north']['reason']

def test_rejected_route_requires_new_plan_and_three_new_exact_votes(tmp_path):
 task={};agreement=TeamAgreement('feedback',plan_validator=lambda p:p)
 class Team:
  def __init__(self):
   self.output=tmp_path/'team';self.calls=[];self.rounds=[];self.events=[];self.agreement=agreement
  def save(self):pass
  def event(self,name,*args,**kwargs):self.events.append(name)
  def negotiate(self,frames,history,turn,sim_time):
   ctx=agreement.context();pending=ctx['proposal']
   proposal=fixture_plan(dock='dock_a',route='south' if 'execution_feedback' in task else 'north')
   if 'execution_feedback' in task:proposal['tasks'][1]['route']='south'
   replies={rid:{'request_id':f'feedback-{rid}-plan-{turn}','proposal_id':pending['proposal_id'] if pending else None,
      'plan_hash':pending['plan_hash'] if pending else None,'accept':True,'plan':pending['plan'] if pending else proposal,
      'reason':'protocol test policy','message':''} for rid in ('r1','r2','r3')}
   self.rounds.append({'turn':turn});return agreement.receive(replies,turn)
 team=Team();report=negotiate_executable(team,{'r1':{'top_bytes':RAW}},{},task,authored_map(),0)
 assert report['feasible'] and len(team.rounds)==4
 assert team.events==['REPLAN_REQUIRED']
 assert [x['event'] for x in agreement.events]==['PROPOSED','COMMITTED','INVALIDATED','PROPOSED','COMMITTED']
 assert agreement.committed['version']==2
 assert agreement.committed['plan']['tasks'][0]['route']=='south'
 assert task['execution_feedback']['rejected_plan']['plan']['tasks'][0]['route']=='north'
