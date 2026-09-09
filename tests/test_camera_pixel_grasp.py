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


def step_with(controller, gripper, candidate, active=True):
    with patch.object(controller.tracker,'update',return_value=copy.deepcopy(gripper)), patch('harness.camera_pixel_grasp.extract_beams',return_value=[copy.deepcopy(candidate)]):
        return controller.step(b'own',b'top',active=active)


def exhaust_primitives(controller, alignment):
    controller.tried.update(label for label,_ in controller._candidates(alignment))


def start_lateral_composite(controller, b, initial=None):
    initial=initial or grip(x=.2)
    alignment=alignment_features(initial,b)
    exhaust_primitives(controller,alignment)
    action=step_with(controller,initial,b)
    assert action=={'kind':'drive','forward':0.,'turn':.1,'duration_s':1.}
    return copy.deepcopy(controller.pending['before'])


class StubJacobian:
    def __init__(self, plans=()):
        self.plans=list(plans)
        self.added=[]
        self.proposed=[]

    def add_sample(self,*args,**kwargs):
        self.added.append((args,kwargs))
        return True

    def propose(self,alignment,pulses):
        self.proposed.append((copy.deepcopy(alignment),copy.deepcopy(pulses)))
        return copy.deepcopy(self.plans.pop(0)) if self.plans else None


def test_no_hidden_gripper_or_missing_target_cannot_claim_capture():
    c=PixelGraspController('r1')
    with patch.object(c.tracker,'update',return_value={'valid':False,'center':None,'opening_axis':None}),patch('harness.camera_pixel_grasp.extract_beams',return_value=[]):
        for _ in range(40):
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


def test_tentative_lift_can_use_top_scale_when_centroid_does_not_move():
    c=PixelGraspController('r1')
    c.lift_start={'beam':beam(),'own_beam':own_beam()}
    observation=copy.deepcopy(c.lift_start)
    assert not c._visual_lift(observation)
    observation['beam']['area_px']*=1.08
    assert c._visual_lift(observation)
    observation['own_beam']['center'][0]+=.05
    assert not c._visual_lift(observation)


def test_ambiguous_baseline_tries_bounded_alternate_views_and_restores():
    c=PixelGraspController('r1');c.refresh_required=True
    invalid={'valid':False,'center':None,'opening_axis':None,'source':'unavailable'}
    actions=[step_with(c,invalid,beam()) for _ in range(40)]
    looks=[a['pan_pulse'] for a in actions if a['kind']=='look']
    assert looks==[1550,1450,1500]
    assert c.stage=='blocked' and c.attempts==0
    assert c.last_action=={'kind':'wait'}


def test_recovered_alternate_view_resumes_fresh_measurement_search():
    c=PixelGraspController('r1');c.refresh_required=True
    invalid={'valid':False,'center':None,'opening_axis':None,'source':'unavailable'}
    for _ in range(4):step_with(c,invalid,beam())
    assert c.last_action=={'kind':'look','pan_pulse':1550}
    # The view move itself is not accepted as geometric alignment. An open
    # command and its fresh isolated observation must follow it.
    step_with(c,grip(x=.2),beam())
    action=step_with(c,grip(x=.2),beam())
    assert action['kind']=='drive'
    assert c.stage=='align' and c.view_repair_index==0


def test_foreshortened_own_rectangle_does_not_invent_a_physical_endpoint():
    own=own_beam(.3);own['width_px']=65.
    a=alignment_features(grip(),beam(),own_beam=own,own_endpoint=[.1,.7])
    assert a['own_endpoint'] is None and a['own_aim_error_px'] is None
    assert a['cost']==a['top_cost']


def test_newly_observable_own_endpoint_does_not_add_an_unmatched_cost_penalty():
    c=PixelGraspController('r1');b=beam();ambiguous=own_beam(.9);ambiguous['width_px']=65.
    with patch.object(c.tracker,'update',return_value=grip(x=.44)),patch('harness.camera_pixel_grasp.extract_beams',side_effect=[[b],[ambiguous]]):
        c.step(b'own',b'top')
    with patch.object(c.tracker,'update',return_value=grip(x=.45)),patch('harness.camera_pixel_grasp.extract_beams',side_effect=[[b],[own_beam(.9)]]):
        action=c.step(b'own',b'top')
    assert action['kind']=='drive' and action['forward']>0
    assert c.repeat=='forward'


def test_offscreen_fitted_own_endpoint_is_not_observed_alignment():
    own=own_beam(.58);own['endpoints']=[[.58,1.074],[.05,.45]]
    a=alignment_features(grip(),beam(),own_beam=own,own_endpoint=[.58,1.06])
    assert a['own_endpoint'] is None and a['own_aim_error_px'] is None
    assert a['cost']==a['top_cost']


def test_composite_does_not_score_intermediate_temporary_worsening():
    c=PixelGraspController('r1');b=beam()
    before=start_lateral_composite(c,b)
    action=step_with(c,grip(x=.1),b)
    assert action=={'kind':'drive','forward':.05,'turn':0.,'duration_s':.8}
    assert c.pending['before']==before
    assert c.pending['label']=='lateral-left'
    assert 'lateral-left' not in c.tried
    assert len(c.composite_queue)==1


def test_composite_scores_only_after_endpoint_fresh_open_measurement():
    c=PixelGraspController('r1');b=beam();before=start_lateral_composite(c,b)
    step_with(c,grip(x=.1),b)
    final_raw=step_with(c,grip(x=.1),b)
    assert final_raw=={'kind':'drive','forward':0.,'turn':-.1,'duration_s':1.}
    propagated=grip(x=.3);propagated['source']='verified_optical_flow'
    assert step_with(c,propagated,b)=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.pending['before']==before and c.repeat is None
    assert step_with(c,grip(x=.3),b)=={'kind':'arm','servo_id':1,'pulse':2000}
    assert c.pending['before']==before and c.repeat is None
    next_action=step_with(c,grip(x=.3),b)
    assert c.repeat=='lateral-left'
    assert next_action=={'kind':'drive','forward':0.,'turn':.1,'duration_s':1.}


def test_failed_composite_rolls_back_every_raw_action_in_reverse_order():
    c=PixelGraspController('r1');b=beam();start_lateral_composite(c,b)
    step_with(c,grip(x=.1),b)
    step_with(c,grip(x=.1),b)
    propagated=grip(x=.1);propagated['source']='verified_optical_flow'
    step_with(c,propagated,b)
    step_with(c,grip(x=.1),b)
    first_undo=step_with(c,grip(x=.1),b)
    second_undo=step_with(c,grip(x=.1),b)
    third_undo=step_with(c,grip(x=.1),b)
    assert first_undo=={'kind':'drive','forward':0.,'turn':.1,'duration_s':1.}
    assert second_undo=={'kind':'drive','forward':-.05,'turn':0.,'duration_s':.8}
    assert third_undo=={'kind':'drive','forward':0.,'turn':-.1,'duration_s':1.}
    assert 'lateral-left' in c.tried


def test_inactive_peer_slice_does_not_advance_composite_queue():
    c=PixelGraspController('r1');b=beam();start_lateral_composite(c,b)
    queued=copy.deepcopy(c.composite_queue);pending=copy.deepcopy(c.pending)
    assert step_with(c,grip(x=.1),b,active=False)=={'kind':'wait'}
    assert c.composite_queue==queued and c.pending==pending
    assert step_with(c,grip(x=.1),b)==queued[0]


def test_jacobian_learns_only_after_fresh_open_primitive_endpoint():
    c=PixelGraspController('r1');c.jacobian=StubJacobian();b=beam()
    step_with(c,grip(x=.2),b)
    propagated=grip(x=.3);propagated['source']='verified_optical_flow'
    assert step_with(c,propagated,b)=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.jacobian.added==[]
    step_with(c,grip(x=.3),b)
    assert c.jacobian.added==[]
    step_with(c,grip(x=.3),b)
    assert len(c.jacobian.added)==1
    assert c.jacobian.added[0][1]=={
        'fresh':True,'same_endpoint':True,'primitive':'forward',
    }

    # A tracked before image can still be explored, but cannot seed learning.
    c=PixelGraspController('r1');c.jacobian=StubJacobian()
    tracked=grip(x=.2);tracked['source']='verified_optical_flow'
    step_with(c,tracked,b)
    step_with(c,grip(x=.3),b)
    assert c.jacobian.added==[]


def test_learned_sequence_has_heterogeneous_reverse_undo_and_inactive_hold():
    c=PixelGraspController('r1');b=beam()
    plan=[
        {'kind':'arm','servo_id':3,'pulse':790},
        {'kind':'look','pan_pulse':1550},
        {'kind':'drive','forward':.05,'turn':0.,'duration_s':1.},
    ]
    c.jacobian=StubJacobian([plan])
    exhaust_primitives(c,alignment_features(grip(x=.2),b))
    assert step_with(c,grip(x=.2),b)==plan[0]
    queued=copy.deepcopy(c.composite_queue);pending=copy.deepcopy(c.pending)
    assert step_with(c,grip(x=.1),b,active=False)=={'kind':'wait'}
    assert c.composite_queue==queued and c.pending==pending
    assert step_with(c,grip(x=.1),b)==plan[1]
    assert step_with(c,grip(x=.1),b)==plan[2]

    # The measured endpoint worsened, so unwind drive, look, then wrist.
    assert step_with(c,grip(x=.1),b)=={
        'kind':'drive','forward':-.05,'turn':-0.,'duration_s':1.,
    }
    assert step_with(c,grip(x=.1),b)=={'kind':'look','pan_pulse':1500}
    assert step_with(c,grip(x=.1),b)=={'kind':'arm','servo_id':3,'pulse':740}
    assert 'learned' in c.tried


def test_successful_learned_sequence_replans_from_the_fresh_endpoint():
    c=PixelGraspController('r1');b=beam()
    first=[{'kind':'arm','servo_id':3,'pulse':790}]
    second=[{'kind':'arm','servo_id':4,'pulse':2370}]
    c.jacobian=StubJacobian([first,second])
    exhaust_primitives(c,alignment_features(grip(x=.2),b))
    assert step_with(c,grip(x=.2),b)==first[0]
    assert step_with(c,grip(x=.3),b)==second[0]
    assert c.repeat=='learned'
    assert len(c.jacobian.proposed)==2
    assert c.jacobian.proposed[1][0]['offset_px'] != c.jacobian.proposed[0][0]['offset_px']


def test_unmeasurable_learned_sequence_records_failure_and_full_rollback():
    c=PixelGraspController('r1');b=beam()
    plan=[{'kind':'arm','servo_id':3,'pulse':790}]
    c.jacobian=StubJacobian([plan])
    exhaust_primitives(c,alignment_features(grip(x=.2),b))
    assert step_with(c,grip(x=.2),b)==plan[0]
    invalid={'valid':False,'center':None,'opening_axis':None,'source':'unavailable'}
    actions=[step_with(c,invalid,b) for _ in range(7)]
    assert actions[-1]=={'kind':'arm','servo_id':3,'pulse':740}
    assert c.last_learned_outcome['before_top_cost']>0
    assert c.last_learned_outcome['after_top_cost'] is None
    assert c.last_learned_outcome['accepted'] is False
    assert c.last_learned_outcome['measurement_unavailable'] is True
    assert c.last_decision['learned_outcome']==c.last_learned_outcome
    assert 'learned' in c.tried
