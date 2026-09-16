import copy
import json

import cv2
import numpy as np
import pytest

from harness.gemini_proxy import GeminiProxyError
from harness.research_camera_actor import validate_reply
from harness.research_execution_recovery import RoleAgreement, request_with_recovery
from harness.research_visual_lease import VisualDriveLease


def proposal(inverse=False, **extra):
    roles={'r1':'bottom_end','r3':'top_end'}
    if inverse:roles={r:('top_end' if v=='bottom_end' else 'bottom_end') for r,v in roles.items()}
    return dict(roles=roles,accept=True,reason='visible ends',message='',**extra)


def ack(agreement, **changes):
    p=agreement.context()['proposal']
    return {**proposal(), 'roles':p['roles'],'proposal_id':p['proposal_id'],'plan_hash':p['plan_hash'],**changes}


def test_crossed_proposals_freeze_then_require_both_exact_acks():
    a=RoleAgreement()
    assert a.receive({'r1':proposal(),'r3':proposal(True)},0) is None
    frozen=copy.deepcopy(a.context())
    assert a.receive({'r1':ack(a),'r3':ack(a,roles=proposal(True)['roles'])},1) is None
    assert a.context()==frozen
    assert a.receive({'r1':ack(a),'r3':ack(a,proposal_id='old')},2) is None
    assert a.receive({'r1':ack(a),'r3':ack(a)},3)==proposal()['roles']
    assert [e['event'] for e in a.events]==['PROPOSED','CONFLICTING_ACK_REJECTED','STALE_ACK_REJECTED','COMMITTED']


def test_reject_rotates_proposer_and_invalidates_previous_acks():
    a=RoleAgreement();a.receive({'r1':proposal(),'r3':proposal(True)},0)
    old=ack(a)
    assert a.receive({'r1':ack(a),'r3':ack(a,accept=False)},1) is None
    assert a.context()=={'version':2,'proposer':'r3','proposal':None}
    a.receive({'r1':proposal(),'r3':proposal(True)},2)
    assert a.context()['proposal']['roles']==proposal(True)['roles']
    assert a.receive({'r1':old,'r3':old},3) is None
    with pytest.raises(ValueError):a.receive({'r1':ack(a)},4)


class Completer:
    last_usage=None;last_model='fake'
    def __init__(self,responses):self.responses=iter(responses)
    def complete(self,messages,images):
        value=next(self.responses)
        if isinstance(value,Exception):raise value
        return value


def valid_wait(request_id):
    return json.dumps(dict(reason='reobserve',message='',status='NOT_READY',confidence=.5,
                           checks=[],command_id=None,action={'kind':'wait'},request_id=request_id))


def recover(responses, **kwargs):
    records=[];requests=[]
    def make(attempt,hint):
        request=dict(messages=[{'hint':hint}],images=[],request_id=f'a{attempt}')
        requests.append(request);return request
    result=request_with_recovery(Completer(responses),make,
        lambda raw,req:validate_reply(raw,'PREPARE',request_id=req),records.append,**kwargs)
    return result,records,requests


def test_timeout_then_missing_action_kind_then_valid_yields_one_valid_reply():
    invalid=json.loads(valid_wait('a1'));invalid['action']={'forward':.1,'turn':0,'duration_s':.2}
    result,records,requests=recover([
        GeminiProxyError('timeout',error_kind='timeout',retryable=True),json.dumps(invalid),valid_wait('a2')])
    assert result[1] is None and result[0]['request_id']=='a2'
    assert len(records)==3 and sum('reply' in r for r in records)==1
    assert requests[1]['messages'][0]['hint']['kind']=='timeout'
    assert requests[2]['messages'][0]['hint']['previous_response']==json.dumps(invalid)
    assert all(r['latency_ms']>=0 for r in records)


def test_stale_reply_is_retried_and_fatal_or_budget_errors_stop():
    result,records,_=recover([valid_wait('old'),valid_wait('a1')])
    assert result[1] is None and records[0]['error_kind']=='reply_schema'
    result,records,_=recover([GeminiProxyError('auth',error_kind='auth',retryable=False)])
    assert result==(None,'auth') and len(records)==1
    assert recover([],can_request=lambda:False)[0]==(None,'request_budget')
    assert recover(['bad']*3)[0]==(None,'request_retries_exhausted')


def frame(x, *, beam=True, second_motion=None):
    rgb=np.full((480,640,3),220,np.uint8)
    if beam:cv2.rectangle(rgb,(450,180),(459,300),(0,140,255),-1)
    cv2.rectangle(rgb,(x,225),(x+24,250),(50,50,50),-1)
    if second_motion is not None:cv2.rectangle(rgb,(second_motion,50),(second_motion+24,75),(50,50,50),-1)
    return cv2.imencode('.jpg',rgb)[1].tobytes()


DRIVE={'kind':'drive','forward':.1,'turn':0,'duration_s':.2}


def test_local_lease_renews_only_fresh_isolated_motion_and_stops_at_budget():
    lease=VisualDriveLease(frame(180),DRIVE,max_steps=3)
    assert lease.after_step(frame(186))['renew']
    assert lease.after_step(frame(192))['renew']
    assert lease.after_step(frame(198))['reason']=='local_budget'
    assert not lease.after_step(frame(204))['renew']


@pytest.mark.parametrize('before,after,reason',[
    (frame(180),frame(180),'fresh_own_motion_unconfirmed'),
    (frame(180,beam=False),frame(186,beam=False),'beam_lost'),
    (frame(400),frame(406),'near_beam_reobserve_fine_alignment'),
    (frame(180,second_motion=400),frame(186,second_motion=406),'fresh_own_motion_unconfirmed'),
])
def test_visual_loss_ambiguity_and_close_clearance_stop(before,after,reason):
    result=VisualDriveLease(before,DRIVE).after_step(after)
    assert not result['renew'] and result['reason']==reason


def test_validated_ack_must_reference_this_request_and_frozen_plan():
    a=RoleAgreement();a.receive({'r1':proposal(),'r3':proposal(True)},0)
    good=ack(a,request_id='r1-turn1')
    assert validate_reply(json.dumps(good),'NEGOTIATE',request_id='r1-turn1',agreement=a.context())==good
    for change in ({'request_id':'old'},{'plan_hash':'other'},{'roles':proposal(True)['roles']}):
        with pytest.raises(ValueError):
            validate_reply(json.dumps({**good,**change}),'NEGOTIATE',request_id='r1-turn1',agreement=a.context())
