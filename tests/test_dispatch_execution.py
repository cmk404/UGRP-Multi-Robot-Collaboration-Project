import copy
import json

import pytest

from harness.dispatch_execution import DispatchExecution, checks, validate_reply, build_request
from harness.dispatch_plan import fixture_plan, build_dispatch_request
from harness.three_robot_plan import digest, ROBOTS, TeamAgreement
from sim.research_dispatch_arena import authored_map, actor_task


def gate(solo='r2',after=False):
    plan=fixture_plan(solo=solo)
    if after:plan['tasks'][0]['after']=['box_job']
    return DispatchExecution({'plan':plan,'plan_hash':digest(plan)},authored_map())


def reply(g,r,status='READY',action=None):
    row=g.current(r);last=g.last_command[r]
    return {'request_id':'req','plan_hash':g.committed['plan_hash'],'stage':row['stage'],
        'status':status,'confidence':.9,'checks':sorted(checks(row['stage'],status=='DONE')),
        'command_id':last['command_id'] if last and status=='DONE' else None,
        'action':action or {'kind':'wait'},'reason':'fixture protocol claim, not physical evidence','message':''}


def batch(g,frame,status='READY',robots=ROBOTS):
    return g.batch({r:reply(g,r,status) for r in robots if g.current(r)},frame_id=frame,now_s=frame)


def test_all_role_assignments_bind_to_exact_plan():
    for solo in ROBOTS:
        g=gate(solo)
        assert g.current(solo)['object']=='box'
        assert all(g.current(r)['object']=='beam' for r in ROBOTS if r!=solo)
        assert all(g.current(r)['execution_adapter']=='experimental_rgb_raw_actions_v1' for r in ROBOTS)


def test_same_frame_and_done_without_current_command_do_not_advance():
    g=gate();batch(g,1,'DONE')
    assert all(g.index[r]==0 for r in ROBOTS)
    assert not batch(g,1)
    batch(g,2);batch(g,2,'DONE')
    assert all(g.index[r]==0 for r in ROBOTS)
    batch(g,3,'DONE')
    assert all(g.index[r]==1 for r in ROBOTS)
    # An old approach command cannot prove a GRASP transition.
    batch(g,4,'DONE')
    assert all(g.index[r]==1 for r in ROBOTS)


def test_independent_approach_and_pair_barrier():
    g=gate();batch(g,1,robots=['r1']);batch(g,2,'DONE',robots=['r1'])
    assert g.current('r1')['stage']=='GRASP' and g.current('r3')['stage']=='APPROACH'
    commands=batch(g,3)
    assert set(commands)=={'r2','r3'}


def test_full_job_dependency_never_bypassed_by_partial_grasp_ready():
    g=gate(after=True);batch(g,1);batch(g,2,'DONE')
    assert set(batch(g,3))=={'r2'}
    assert g.feedback['r1']['reason']=='waiting_for_job_dependency'


def test_fresh_uncertainty_holds_pair_but_solo_continues():
    g=gate();batch(g,1);batch(g,2,'DONE')
    values={r:reply(g,r) for r in ROBOTS};values['r3']=reply(g,'r3','UNCERTAIN')
    assert set(g.batch(values,frame_id=3,now_s=3))=={'r2'}
    assert set(batch(g,4))==set(ROBOTS)
    # Fresh readiness, not the preceding batch, owns permission.
    assert set(g.batch({'r1':reply(g,'r1')},frame_id=5,now_s=5))==set()


def test_full_protocol_resource_contention_and_release():
    g=gate()
    for n in range(3):batch(g,2*n+1);batch(g,2*n+2,'DONE')
    assert all(g.current(r)['stage']=='TRANSIT' for r in ROBOTS)
    assert set(batch(g,7))=={'r1','r3'}
    assert g.locks=={'north_gate':'beam_job','dispatch_apron':'beam_job'}
    for frame in (8,10,12):
        batch(g,frame,'DONE',robots=['r1','r3'])
        if frame<12:batch(g,frame+1,robots=['r1','r3'])
    assert not g.locks
    assert set(batch(g,13))=={'r2'}
    for frame in (14,16,18):
        batch(g,frame,'DONE',robots=['r2'])
        if frame<18:batch(g,frame+1,robots=['r2'])
    assert g.complete and not g.locks


def test_revoke_keeps_resource_locks_and_disables_all_actions():
    g=gate()
    for n in range(3):batch(g,2*n+1);batch(g,2*n+2,'DONE')
    batch(g,7);g.revoke('timeout while occupied')
    assert g.locks and not batch(g,8)


def test_raw_action_boundaries_and_binding_reject_unsafe_or_stale_values():
    g=gate();v=reply(g,'r1','WORKING',{'kind':'drive','forward':.1,'turn':0,'duration_s':1.})
    assert validate_reply(json.dumps(v),'req',plan_hash=v['plan_hash'],stage='APPROACH')==v
    for mutate in ('hash','request','close','long','nan','done_moves'):
        x=copy.deepcopy(v)
        if mutate=='hash':x['plan_hash']='0'*64
        if mutate=='request':x['request_id']='old'
        if mutate=='close':x['action']={'kind':'arm','servo_id':1,'pulse':1500}
        if mutate=='long':x['action']['duration_s']=2
        if mutate=='nan':x['confidence']=float('nan')
        if mutate=='done_moves':x['status']='DONE'
        with pytest.raises(ValueError):validate_reply(json.dumps(x),'req',plan_hash=v['plan_hash'],stage='APPROACH')


def test_optional_identity_echo_must_match_actual_endpoint():
    g=gate();v=reply(g,'r1');v['robot_id']='r1'
    kwargs={'plan_hash':v['plan_hash'],'stage':'APPROACH','robot_id':'r1'}
    assert validate_reply(json.dumps(v),'req',**kwargs)['robot_id']=='r1'
    v['robot_id']='r2'
    with pytest.raises(ValueError,match='endpoint'):validate_reply(json.dumps(v),'req',**kwargs)
    v['robot_id']='r1';v['new_field']='not allowed'
    with pytest.raises(ValueError,match='fields'):validate_reply(json.dumps(v),'req',**kwargs)


def test_execution_request_only_contains_allowlisted_own_observations():
    g=gate();own=[{'command_id':'own'}];inbox=[{'message':'claim'}]
    request=build_request('r1',request_id='opaque',committed=g.committed,program=g.current('r1'),
        task=actor_task(authored_map()),frame={'own_bytes':b'own','top_bytes':b'top'},previous=None,
        own_history=own,inbox=inbox,identity={'center':[.1,.2]},feedback={})
    own.append({'secret':'later'});inbox.append({'secret':'later'})
    text=json.dumps(request)
    assert all(x not in text for x in ('later','setup_only','spawns','qpos','contacts','evaluation-only'))
    assert len(request['images'])==2


def test_execution_planning_prompt_is_not_mislabeled_planning_only():
    request=build_dispatch_request('r1',request_id='opaque',task=actor_task(authored_map()),
        own_rgb=b'own',top_rgb=b'top',agreement=TeamAgreement('opaque').context(),execution_pilot=True)
    assert 'PLANNING ONLY' not in request['messages'][0]['content']
    assert 'physical execution pilot' in request['messages'][0]['content']
    assert 'participant[0] is end_a' in request['messages'][0]['content']
    assert 'UPPER beam endpoint' in request['messages'][0]['content']


def test_probe_readout_is_computed_only_from_pixel_motion_claim():
    request=build_dispatch_request('r1',request_id='opaque',task=actor_task(authored_map()),
        own_rgb=b'own',top_rgb=b'top',agreement=TeamAgreement('opaque').context(),
        identity_evidence={'claim':{'center':[.123,.789],'valid':True},'images':[]})
    context=json.loads(request['messages'][1]['content'])
    assert context['own_probe_image_readout']['vertical_percent_from_top']==78.9
    assert context['own_probe_image_readout']['horizontal_percent_from_left']==12.3
