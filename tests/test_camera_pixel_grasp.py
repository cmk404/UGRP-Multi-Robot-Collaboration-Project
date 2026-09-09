"""Behavioral checks for image-only end/axis alignment and trial grasp gates."""
import copy
from unittest.mock import patch

from harness.camera_pixel_grasp import PixelGraspController, alignment_features


def beam():
    return {'center':[.5,.5],'endpoints':[[.5,.3],[.5,.7]],'width_px':8.,
            'length_px':192.,'area_px':1536.,'image_size':[640,480]}


def grip(x=.5,y=.69,axis=(1.,0.)):
    return {'valid':True,'center':[x,y],'opening_axis':list(axis),'span_px':12.,'confidence':.9,'source':'isolated_gripper_motion'}


def own_beam(x=.5):
    return {'center':[x,.5],'endpoints':[[x,.4],[x,.6]],'width_px':20.,
            'length_px':100.,'area_px':2000.,'image_size':[640,480]}


def test_alignment_requires_direction_not_just_midpoint():
    b=beam();a=alignment_features(grip(),b)
    wrong=alignment_features(grip(axis=(0.,1.)),b)
    assert a['distance_px']<.01
    assert a['cost']<.01
    assert wrong['cost']>20
    assert abs(wrong['axis_error_rad'])>1.5


def test_endpoint_continuity_and_axis_sign_invariance():
    b=beam()
    a=alignment_features(grip(y=.2),b,endpoint=[.5,.7])
    assert a['endpoint']==[.5,.7]
    assert abs(alignment_features(grip(axis=(-1.,0.)),b)['cost'])<.01


def test_own_camera_endpoint_aim_contributes_to_cost():
    centered=alignment_features(grip(),beam(),own_beam=own_beam(.5))
    off_center=alignment_features(grip(),beam(),own_beam=own_beam(.75))
    assert centered['own_aim_error_px']==0
    assert off_center['own_aim_error_px']>30
    assert off_center['cost']>centered['cost']+30


def step_with(controller, gripper, candidate):
    with patch.object(controller.tracker,'update',return_value=copy.deepcopy(gripper)), patch('harness.camera_pixel_grasp.extract_beams',return_value=[copy.deepcopy(candidate)]):
        return controller.step(b'own',b'top')


def test_no_hidden_gripper_or_missing_target_cannot_claim_capture():
    c=PixelGraspController('r1')
    with patch.object(c.tracker,'update',return_value={'valid':False,'center':None,'opening_axis':None}),patch('harness.camera_pixel_grasp.extract_beams',return_value=[]):
        for _ in range(14):
            c.step(b'own',b'top')
    assert c.stage=='blocked'
    assert c.attempts==0
    assert c.last_action=={'kind':'wait'}


def test_two_aligned_observations_then_close_does_not_mean_success():
    c=PixelGraspController('r1');b=beam()
    first=step_with(c,grip(),b)
    second=step_with(c,grip(),b)
    assert first=={'kind':'wait'}
    assert second=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.stage=='lift'
    assert c.attempts==1
    # Unmoved beam cannot qualify as following the commanded hand.
    for _ in range(4):step_with(c,grip(),b)
    assert c.stage!='hold'
    assert c.last_action=={'kind':'arm','servo_id':1,'pulse':2000}


def test_observed_bad_step_schedules_real_reverse_not_position_reset():
    c=PixelGraspController('r1');b=beam()
    action=step_with(c,grip(x=.2),b)
    assert action['kind']=='drive' and action['forward']>0
    undo=step_with(c,grip(x=.19),b)
    assert undo['kind']=='drive' and undo['forward']<0
    assert 'forward' in c.tried


def test_tracked_candidate_is_remeasured_before_acceptance():
    c=PixelGraspController('r1');b=beam()
    step_with(c,grip(x=.2),b)
    propagated=grip(x=.3);propagated['source']='verified_optical_flow'
    action=step_with(c,propagated,b)
    assert action=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.stage=='measure' and c.pending is not None
    assert c.repeat is None


def test_candidate_losing_visibility_is_reversed_within_measurement_budget():
    c=PixelGraspController('r1');b=beam()
    step_with(c,grip(x=.2),b)
    invalid={'valid':False,'center':None,'opening_axis':None,'source':'unavailable'}
    actions=[step_with(c,invalid,b) for _ in range(7)]
    assert any(a['kind']=='drive' and a['forward']<0 for a in actions)
    assert c.pending is None and 'forward' in c.tried


def test_top_improvement_is_rejected_when_own_beam_disappears():
    c=PixelGraspController('r1');b=beam();own=own_beam(.8)
    with patch.object(c.tracker,'update',return_value=copy.deepcopy(grip(x=.2))), \
         patch('harness.camera_pixel_grasp.extract_beams',side_effect=[[copy.deepcopy(b)],[copy.deepcopy(own)]]):
        issued=c.step(b'own',b'top')
    assert issued['kind']=='drive' and issued['forward']>0
    with patch.object(c.tracker,'update',return_value=copy.deepcopy(grip(x=.45))), \
         patch('harness.camera_pixel_grasp.extract_beams',side_effect=[[copy.deepcopy(b)],[]]):
        rejected=c.step(b'own',b'top')
    assert rejected['kind']=='drive' and rejected['forward']<0
    assert c.repeat is None and 'forward' in c.tried
    assert 'previously visible own-camera beam' in c.last_decision['reason']


def test_own_endpoint_continuity_prevents_switch_to_opposite_end_during_pan():
    b=own_beam();b['endpoints']=[[.14,.386],[.91,.703]]
    b['length_px']=517.;b['width_px']=100.
    observed=alignment_features(grip(),beam(),own_beam=b,own_endpoint=[.76,.673])
    assert observed['own_endpoint']==[.91,.703]
    assert observed['own_aim_error_px']>0
