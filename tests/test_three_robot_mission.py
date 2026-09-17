import copy
import json

import pytest

from harness.three_robot_mission import (validate_mission, mission_fixture,
    validate_mission_reply, build_mission_request)
from harness.three_robot_plan import TeamAgreement, ROBOTS
from scripts.run_three_robot_mission import evaluate_solo


def test_stationary_third_robot_or_missing_goal_cannot_execute():
    for field, value in [('robot_id','r1'),('skill','inspect_goal_rgb_v1'),
                         ('object','orange_beam'),('goal','green_zone'),
                         ('completion','commands_issued')]:
        plan=mission_fixture();plan['solo'][field]=value
        with pytest.raises(ValueError):validate_mission(plan)
    with pytest.raises(ValueError):validate_mission({'transport':mission_fixture()['transport']})


def test_selected_goal_requires_identical_three_peer_ack():
    team=TeamAgreement('test',plan_validator=validate_mission)
    def reply(rid,turn,plan):
        pending=team.pending
        return {'request_id':f'test-{rid}-plan-{turn}',
                'proposal_id':pending['proposal_id'] if pending else None,
                'plan_hash':pending['plan_hash'] if pending else None,
                'accept':True,'plan':plan,'reason':'RGB','message':''}
    selected=mission_fixture('far_magenta')
    team.receive({'r1':reply('r1',0,selected)},0)
    wrong=reply('r2',1,mission_fixture('near_magenta'))
    with pytest.raises(ValueError):validate_mission_reply(json.dumps(wrong),wrong['request_id'],team.context())
    assert team.receive({r:reply(r,1,selected) for r in ('r1','r3')},1) is None
    assert not team.authorize(team.pending['proposal_id'],team.pending['plan_hash'])
    committed=team.receive({r:reply(r,2,selected) for r in ROBOTS},2)
    assert committed['plan']['solo']['goal']=='far_magenta'
    team.invalidate('cargo blocked')
    assert not team.authorize(committed['proposal_id'],committed['plan_hash'])


def test_plan_builder_uses_only_serialized_rgb_task_and_own_history():
    req=build_mission_request('r2',request_id='x',own_rgb=b'own',top_rgb=b'top',
        agreement=TeamAgreement('x').context(),own_history=[{'kind':'wait'}])
    context=json.loads(req['messages'][1]['content'])
    assert context['own_issued_commands']==[{'kind':'wait'}]
    assert len(req['images'])==2
    assert context['reply_binding']=={'request_id':'x','proposal_id':None,'plan_hash':None}
    assert set(context)=={'robot_id','request_id','reply_binding','agreement','pair_task',
                          'destination_preference','received_peer_claims','own_issued_commands'}


def test_referee_rejects_walking_without_cargo_and_wrong_destination():
    def row(x,z,robot,floor=False):
        return {'box_position':[x,-3.,z],'robot_position':[robot,-3.,.03],
                'floor_contact':floor,'robot_contact':not floor}
    samples=[row(.5,.016,0.,True),row(.85,.09,.7)]
    samples += [row(1.,.016,.85,True) for _ in range(11)]
    assert evaluate_solo(samples,'near_magenta')['success']
    assert not evaluate_solo(samples,'far_magenta')['success']
    dragged=copy.deepcopy(samples)
    for sample in dragged:sample['box_position'][2]=.016
    assert not evaluate_solo(dragged,'near_magenta')['success']
