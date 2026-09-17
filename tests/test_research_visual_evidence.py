import base64
import copy
import json

import cv2
import numpy as np
import pytest

from harness.research_visual_evidence import build_review_request, detail_image, reviewed_reply, supports_claim, validate_review


def positive(phase='GRASP'):
    return dict(request_id='test',identity_resolved=True,target_visible=True,two_fingers_resolved=True,
                target_between_fingers=True,jaws_state='closed' if phase=='GRASP' else 'open',
                stationary=True,confidence=.9,reason='Synthetic resolved visual evidence, not a physical grasp result.')


@pytest.mark.parametrize('phase',['PREPARE','GRASP'])
def test_resolved_evidence_preserves_claim_and_missing_evidence_never_promotes(phase):
    answer=dict(status='DONE' if phase=='GRASP' else 'READY',action={'kind':'wait'},confidence=.9,
                checks=['grasp_observed'],command_id='own-1',reason='actor',message='')
    assert supports_claim(positive(phase),phase)
    assert reviewed_reply(answer,positive(phase),phase)==answer
    for field in ('identity_resolved','target_visible','two_fingers_resolved','target_between_fingers','stationary'):
        witness=positive(phase);witness[field]=False
        result=reviewed_reply(answer,witness,phase)
        assert result['status']=='UNCERTAIN' and result['checks']==[] and result['action']=={'kind':'wait'}
        assert answer['status']!='UNCERTAIN'
    for witness in (None,{**positive(phase),'confidence':.79},{**positive(phase),'jaws_state':'unknown'}):
        assert not supports_claim(witness,phase)


@pytest.mark.parametrize('change',[
    {'contacts':True},{'two_fingers_resolved':1},{'request_id':'old'},
    {'jaws_state':'probably_closed'},{'confidence':float('nan')},{'reason':''}])
def test_rejects_truth_extra_stale_and_untyped_witnesses(change):
    with pytest.raises(ValueError):validate_review(json.dumps({**positive(),**change}),'test')


def test_review_sees_only_original_camera_pixels_roles_and_retry():
    frame=np.zeros((48,64,3),np.uint8);frame[12:36,16:48]=[10,50,180]
    _,encoded=cv2.imencode('.jpg',frame)
    b64=base64.b64encode(encoded.tobytes()).decode()
    camera={'images':{k:{'jpeg_base64':b64} for k in ('own_rgb','top_rgb')},
            'contacts':'FORBIDDEN','plan':{'position':'FORBIDDEN'}}
    request=build_review_request('r1','GRASP',camera,roles={'r1':'bottom_end','r3':'top_end'},
                                 previous=camera,request_id='test')
    assert 'FORBIDDEN' not in json.dumps(request)
    assert len(request['images'])==6
    assert request['images'][0]['image'].split(',',1)[1]==b64
    assert request['images'][2]['image'].split(',',1)[1]==detail_image(b64)
    decoded=cv2.imdecode(np.frombuffer(base64.b64decode(detail_image(b64)),np.uint8),cv2.IMREAD_COLOR)
    assert decoded.shape==frame.shape and decoded[24,32,2]>160
    assert set(json.loads(request['messages'][1]['content']))=={'robot_id','phase','agreed_roles','current_request_id','retry'}
