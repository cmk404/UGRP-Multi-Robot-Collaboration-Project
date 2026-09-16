import copy
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from harness.dispatch_plan import (validate_dispatch_plan,fixture_plan,compile_programs,
                                  DispatchCoordinator,build_dispatch_request)
from harness.three_robot_plan import TeamAgreement,digest,ROBOTS
from sim.research_dispatch_arena import actor_task,authored_map,episode,build_scene_xml,VARIANTS,FIXED_TOP


def committed(plan):
    team=TeamAgreement('test',plan_validator=validate_dispatch_plan)
    for turn in (0,1):
        p=team.pending
        team.receive({r:{'request_id':f'test-{r}-plan-{turn}',
            'proposal_id':p['proposal_id'] if p else None,
            'plan_hash':p['plan_hash'] if p else None,'accept':True,
            'plan':plan,'reason':'fixture','message':''} for r in ROBOTS},turn)
    return team.committed


def report(c,r,status='READY'):
    row=c.current(r)
    c.report(r,plan_hash=c.committed['plan_hash'],stage=row['stage'],status=status,
             own_rgb_ref=f'{r}.jpg',top_rgb_ref='top.jpg',now_s=1.)


def test_arena_actor_inputs_do_not_reveal_seed_spawn_or_unannounced_barrier():
    a,b=episode('shared_crossing',11),episode('north_blocked',15)
    assert a['static_map']==b['static_map']
    assert actor_task(a['static_map'])==actor_task(b['static_map'])
    request=build_dispatch_request('r1',task=actor_task(a['static_map']),request_id='req',
        own_rgb=b'own',top_rgb=b'top',agreement=TeamAgreement('test').context())
    text=request['messages'][1]['content']
    assert all(k not in text for k in ('setup_only','unexpected_obstacles','spawns','seed','evaluation'))
    assert len(request['images'])==2


def test_request_identifiers_do_not_disclose_condition_or_seed():
    from scripts.run_research_dispatch import opaque_run_id
    run_id=opaque_run_id()
    assert re.fullmatch(r'dispatch-[0-9a-f]{12}',run_id)
    req=build_dispatch_request('r1',task=actor_task(authored_map('north_blocked')),
        request_id=run_id+'-r1-plan-0',own_rgb=b'own',top_rgb=b'top',
        agreement=TeamAgreement(run_id,plan_validator=validate_dispatch_plan).context())
    assert 'north_blocked' not in json.dumps(req)


@pytest.mark.parametrize('solo',ROBOTS)
def test_plan_changes_actual_robot_program_ownership(solo):
    plan=fixture_plan(solo=solo,dock='dock_b')
    programs=compile_programs(plan,authored_map())
    assert all(x['object']=='box' and x['goal_region']=='dock_b_box' for x in programs[solo])
    for rid in set(ROBOTS)-{solo}:
        assert all(x['object']=='beam' and x['goal_region']=='dock_b_beam' for x in programs[rid])
        assert programs[rid][0]['barrier'] is False
        assert programs[rid][1]['barrier'] is True


def test_invalid_plan_rejected_before_actuation():
    for mutation in ('duplicate','cycle','missing','extra','goal'):
        plan=fixture_plan()
        if mutation=='duplicate':plan['tasks'][1]['participants']=['r1']
        if mutation=='cycle':
            plan['tasks'][0]['after']=['box_job'];plan['tasks'][1]['after']=['beam_job']
        if mutation=='missing':plan['tasks'].pop()
        if mutation=='extra':plan['live_positions']={}
        if mutation=='goal':plan['dock']='anything'
        with pytest.raises(ValueError):validate_dispatch_plan(plan)


def test_local_preparation_independent_but_joint_lift_waits_for_partner():
    c=DispatchCoordinator(committed(fixture_plan()),authored_map())
    report(c,'r1');assert c.permission('r1');assert not c.permission('r3')
    report(c,'r1','DONE');assert c.current('r1')['stage']=='GRASP'
    report(c,'r1');assert not c.permission('r1')
    report(c,'r2');assert c.permission('r2')
    report(c,'r3');report(c,'r3','DONE');report(c,'r3')
    assert c.permission('r1') and c.permission('r3')


def test_revocation_keeps_occupied_resource_and_stale_plan_cannot_continue():
    c=DispatchCoordinator(committed(fixture_plan()),authored_map())
    # Exercise full reported stage progression, not hand-assigned internals.
    for stage in ('APPROACH','GRASP','LIFT'):
        for r in ROBOTS:report(c,r)
        for r in ROBOTS:report(c,r,'DONE')
    for r in ROBOTS:report(c,r)
    assert c.permission('r1')
    assert not c.permission('r2') # apron already reserved by beam task
    c.revoke('response timeout',now_s=2.)
    assert c.locks=={'north_gate':'beam_job','dispatch_apron':'beam_job'}
    assert all(not c.permission(r) for r in ROBOTS)
    with pytest.raises(ValueError):report(c,'r1','DONE')


def test_dependency_and_missing_visual_evidence_cannot_advance():
    plan=fixture_plan();plan['tasks'][0]['after']=['box_job']
    c=DispatchCoordinator(committed(plan),authored_map())
    report(c,'r1');assert not c.permission('r1')
    with pytest.raises(ValueError):
        c.report('r2',plan_hash=c.committed['plan_hash'],stage='APPROACH',status='READY',
                 own_rgb_ref='',top_rgb_ref='top.jpg',now_s=0.)


def test_environment_builder_retains_robots_camera_and_real_collision_geometry():
    source='<mujoco><worldbody><geom name="floor"/><camera name="cctv_top"/><body name="team_beam"><geom name="beam"/></body>'
    source+=''.join(f'<body name="{r}__robot"><camera name="{r}__robot_cam" pos="1 2 3"/></body>' for r in ROBOTS)
    source+='</worldbody><equality><weld name="test" active="true"/></equality></mujoco>'
    for variant in VARIANTS:
        xml,manifest=build_scene_xml(source,episode(variant))
        tree=ET.fromstring(xml)
        assert tree.find('.//weld').get('active')=='false'
        assert len(manifest['robot_xml_sha256'])==3
        assert tree.find('.//camera[@name="r2__robot_cam"]').get('pos')=='1 2 3'
        assert tree.find('.//geom[@name="dispatch_wall_north"]').get('contype')=='1'
        assert tree.find('.//geom[@name="dispatch_dock_a_beam"]').get('contype')=='0'
        assert (tree.find('.//geom[@name="dispatch_unannounced_north_barrier"]') is not None)==(variant=='north_blocked')


def test_saved_negotiation_history_cannot_gain_commands_issued_after_request(tmp_path):
    from functools import partial
    from harness.dispatch_plan import validate_dispatch_reply
    from scripts.three_robot_runtime import ThreeRobotRuntime
    frames={r:{'own_bytes':b'own','top_bytes':b'top','own_rgb':{'path':'own.jpg'},
               'shared_top_rgb':{'path':'top.jpg'},'frame_id':1} for r in ROBOTS}
    history={r:[] for r in ROBOTS}
    team=ThreeRobotRuntime(tmp_path/'team',run_id='history',mode='fixture',
        agreement=TeamAgreement('history',plan_validator=validate_dispatch_plan),
        request_builder=partial(build_dispatch_request,task=actor_task(authored_map())),
        reply_validator=validate_dispatch_reply,plan_fixture=fixture_plan(),
        roles_fixed_by_skill=False,planning_only=True)
    try:
        team.negotiate(frames,history,0,0.)
        history['r1'].append({'kind':'drive'})
        assert team.rounds[0]['own_history']['r1']==[]
    finally:team.close(1.)
