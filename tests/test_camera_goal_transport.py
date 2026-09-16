import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.camera_goal_transport import coarse_approach, goal_carry, goal_features, dock_command, preclose_supported
from harness.camera_skill_actor import build_skill_request, validate_skill_reply, pair_skill_ready

FIXTURE=Path(__file__).parent/'fixtures/camera_goal_transport'


def rgb(name): return (FIXTURE/name).read_bytes()


def test_remembers_goal_when_robot_occludes_and_splits_marker():
    own,top=rgb('anchor-own.jpg'),rgb('anchor-top.jpg')
    assert goal_features(rgb('occluded-top.jpg')) is None
    d=goal_carry(own,rgb('occluded-top.jpg'),own,top)
    assert d['ok'] and not d['ready']
    assert d['features']['goal_x']==goal_features(top)['goal_x']
    assert .04<=d['forward']<=.10


def test_stationary_images_cannot_claim_progress_and_visible_goal_stops():
    own,top=rgb('anchor-own.jpg'),rgb('anchor-top.jpg')
    for _ in range(5):
        d=goal_carry(own,top,own,top)
        assert not d['ready'] and d['forward']==.1
    arrived=goal_carry(rgb('goal-own.jpg'),rgb('goal-top.jpg'),own,top)
    assert arrived['ready'] and arrived['forward']==0


def test_missing_payload_fails_closed():
    _,blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))
    d=goal_carry(blank.tobytes(),rgb('goal-top.jpg'),rgb('anchor-own.jpg'),rgb('anchor-top.jpg'))
    assert not d['ok'] and not d['ready'] and d['forward']==0


def test_overshooting_goal_never_drives_farther_away():
    top=cv2.imdecode(np.frombuffer(rgb('goal-top.jpg'),np.uint8),cv2.IMREAD_COLOR)
    shifted=cv2.warpAffine(top,np.float32([[1,0,20],[0,1,0]]),(960,720))
    _,jpeg=cv2.imencode('.jpg',shifted)
    d=goal_carry(rgb('goal-own.jpg'),jpeg.tobytes(),rgb('anchor-own.jpg'),rgb('anchor-top.jpg'))
    assert d['image_gap']<-.006
    assert not d['ok'] and not d['ready'] and d['forward']==0


@pytest.mark.parametrize('rid',['r1','r3'])
def test_far_approach_handoff_is_not_grasp_readiness(rid):
    far=coarse_approach(rgb('start-top.jpg'),rgb('reference-top.jpg'),rid)
    assert far['ok'] and not far['ready'] and far['forward']>0
    assert coarse_approach(rgb('reference-top.jpg'),rgb('reference-top.jpg'),rid)['ready']


def reply(**kwargs):
    return dict(request_id='r1-CLOSE',skill='CLOSE',confidence=.9,reason='visible jaws',message='',**kwargs)


@pytest.mark.parametrize('updates',[{'confidence':True},{'confidence':float('nan')},
    {'request_id':'stale'},{'skill':'LIFT'},{'contacts':True},{'reason':'x'*601}])
def test_skill_replies_reject_stale_truth_fields_and_invalid_confidence(updates):
    d=reply();d.update(updates)
    with pytest.raises(ValueError): validate_skill_reply(json.dumps(d),'r1-CLOSE','CLOSE')


def test_both_llms_must_select_same_skill_with_confidence():
    d=reply()
    assert pair_skill_ready({'r1':d,'r3':d},'CLOSE')
    for denied in (None,{**d,'skill':'HOLD'},{**d,'confidence':.79}):
        assert not pair_skill_ready({'r1':d,'r3':denied},'CLOSE')
    assert not pair_skill_ready({'r1':d},'CLOSE')


def test_skill_prompt_preserves_scope_and_only_declared_inputs():
    req=build_skill_request('r1','LIFT',request_id='x',own_rgb=b'own',top_rgb=b'top',
        own_commands=[{'kind':'close'}],peer_claims=[{'message':'claim'}])
    context=json.loads(req['messages'][1]['content'])
    assert set(context)=={'request_id','offered_skill','own_issued_commands','peer_visual_claims','retry'}
    assert len(req['images'])==2
    assert 'NOT guaranteed' in req['messages'][0]['content']
    assert 'not measured' in req['messages'][0]['content']


def test_previous_ready_band_is_not_precise_docking():
    # Failed asymmetric run had READY but still 2.54 mm of RGB-estimated error.
    d=dict(ok=True,precision='fine',ready=True,diagnostics=dict(image_derived_error=.00254))
    assert dock_command(d)['forward']>0
    assert not dock_command(d)['ready']
    for error in (.0007,-.0007):
        assert dock_command({**d,'diagnostics':dict(image_derived_error=error)})['forward']==0
    for error in (None,True,float('nan'),.02):
        v=dock_command({**d,'diagnostics':dict(image_derived_error=error)})
        assert not v['ok'] and v['forward']==0


def test_unsupported_grasp_cannot_be_closed_even_if_peer_is_ready():
    good=dict(observable=True,confidence=.99)
    assert preclose_supported(dict(r1=good,r3=good))
    assert not preclose_supported(dict(r1=dict(observable=False,confidence=0),r3=good))


def test_e2e_setup_supports_far_rotated_starts_without_expanding_old_near_domain():
    from scripts.run_camera_goal_transport import setup_poses
    from scripts.camera_approach_scene import validate_start_poses
    starts=setup_poses([.3,.7],[.06,-.06],[10,-10])
    assert starts['r3']==dict(distance_m=.7,lateral_m=-.06,yaw_deg=-10.)
    with pytest.raises(ValueError): validate_start_poses(starts)
    for distance,lateral,yaw in (([.3,.71],[0,0],[0,0]),([.3,.6],[0,float('nan')],[0,0]),
        ([.3,.6],[0,0],[0,True]),([.3,.6],[0,0],[0,21]),([.3],[0,0],[0,0])):
        with pytest.raises(ValueError): setup_poses(distance,lateral,yaw)


HEADING_FIXTURE=Path(__file__).parent/'fixtures/camera_goal_heading'


def test_heading_covers_saved_far_failures_without_forward_drift():
    import hashlib
    manifest=json.loads((HEADING_FIXTURE/'manifest.json').read_text())
    assert len(manifest)==19
    for name,source in manifest.items():
        top=(HEADING_FIXTURE/name).read_bytes()
        assert hashlib.sha256(top).hexdigest()==source['sha256']
        for rid,setup in source['setup_only'].items():
            decision=coarse_approach(top,rgb('reference-top.jpg'),rid)
            assert decision['ok']
            # Labels are used only here to evaluate saved input predictions.
            assert abs(decision['heading']['angle_deg']+setup['yaw_deg'])<1.5
            if setup['yaw_deg']:
                assert decision['turn']*setup['yaw_deg']<0
                assert decision['forward']==0 and not decision['ready']
            else:
                assert decision['turn']==0 and decision['forward']>0


def test_unchanged_heading_images_cannot_claim_rotation_succeeded():
    top=(HEADING_FIXTURE/'mixed-far-wide.jpg').read_bytes()
    for _ in range(10):
        d=coarse_approach(top,rgb('reference-top.jpg'),'r1')
        assert d['ok'] and d['turn']>0 and d['forward']==0 and not d['ready']


def test_missing_wheel_corners_fail_closed():
    from harness.camera_goal_transport import lane_heading
    frame=cv2.imdecode(np.frombuffer((HEADING_FIXTURE/'baseline.jpg').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    # Occlude the robot's lower wheel row while preserving the payload.
    frame[450:500,270:370]=0
    _,encoded=cv2.imencode('.jpg',frame)
    assert lane_heading(encoded.tobytes(),'r1') is None
    d=coarse_approach(encoded.tobytes(),rgb('reference-top.jpg'),'r1')
    assert not d['ok'] and not d['ready'] and d['forward']==0 and d['turn']==0
