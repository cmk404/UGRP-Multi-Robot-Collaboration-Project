"""Behavioral checks for image-only end/axis alignment and trial grasp gates."""
import copy
import math
from unittest.mock import patch

from harness.camera_pixel_grasp import (
    BASIN_ACTIVE_BUDGET, BASIN_OFFSETS, SEARCH_SCALES,
    PixelGraspController, alignment_features,
)


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
    inside=alignment_features(grip(axis=(math.cos(.13),math.sin(.13))),b)
    wrong=alignment_features(grip(axis=(0.,1.)),b)
    assert a['distance_px']<.01
    assert a['cost']<.01
    assert inside['cost']<.01
    assert abs(inside['axis_error_rad'])<.16
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


def measured_alignment(distance, angle, width=24.55):
    angular_error=max(8.,10.*width)*max(0.,abs(angle)-.16)
    cost=math.hypot(distance,angular_error)
    return {'offset_px':[distance,0.], 'axis_error_rad':angle,
            'endpoint':[.5,.7], 'target':[.5,.7],
            'distance_px':distance, 'own_aim_error_px':None,
            'own_endpoint':None, 'top_cost':cost,
            'cost':cost, 'width_px':width}


def step_with_measured_alignment(controller, alignment):
    with patch.object(controller.tracker,'update',return_value=grip()), \
         patch('harness.camera_pixel_grasp.extract_beams',side_effect=[[beam()],[own_beam()]]), \
         patch('harness.camera_pixel_grasp.alignment_features',return_value=alignment):
        return controller.step(b'own',b'top')


def exhaust_primitives(controller, alignment):
    controller.tried.update(label for label,_ in controller._candidates(alignment))


def exhaust_current_scale(controller, alignment):
    exhaust_primitives(controller, alignment)
    controller.tried.update(('learned','lateral-left','lateral-right'))


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


def test_inside_cone_v11_distance_improvement_is_accepted():
    c=PixelGraspController('r1')
    before=measured_alignment(18.723,-.0161)
    after=measured_alignment(17.092,-.1307)
    c.pending={'before':before,'action':{'kind':'drive','forward':.05,'turn':0.,'duration_s':1.},
               'pulses_before':copy.deepcopy(c.pulses),'own_beam_visible':False,
               'learn':True,'label':'forward','undo':[]}
    step_with_measured_alignment(c,after)
    assert c.repeat=='forward'
    assert c.rollback==[]


def test_axis_outside_absolute_closure_gate_does_not_start_trial():
    c=PixelGraspController('r1')
    c.aligned=1
    outside=measured_alignment(0.,.23)
    action=step_with_measured_alignment(c,outside)
    assert action != {'kind':'arm','servo_id':1,'pulse':1500}
    assert c.aligned==0 and c.attempts==0
    inside=PixelGraspController('r1')
    inside.aligned=1
    assert step_with_measured_alignment(inside,measured_alignment(0.,.13)) == {
        'kind':'arm','servo_id':1,'pulse':1500}
    assert inside.attempts==1


def test_exhausted_coarse_search_advances_to_half_scale():
    c=PixelGraspController('r1');b=beam();current=alignment_features(grip(x=.47),b)
    exhaust_current_scale(c,current)

    action=step_with(c,grip(x=.47),b)

    assert action=={'kind':'drive','forward':.025,'turn':0.,'duration_s':.4}
    assert c.search_level==1
    assert c.last_decision['search_scale']==.5


def test_accepted_fine_step_continues_at_the_same_scale():
    c=PixelGraspController('r1');c.search_level=1;b=beam()
    first=step_with(c,grip(x=.47),b)
    repeated=step_with(c,grip(x=.48),b)

    assert first=={'kind':'drive','forward':.025,'turn':0.,'duration_s':.4}
    assert repeated==first
    assert c.repeat=='forward' and c.search_level==1


def test_exhausted_half_scale_advances_to_quarter_scale():
    c=PixelGraspController('r1');c.search_level=1;b=beam()
    exhaust_primitives(c,alignment_features(grip(x=.47),b))

    action=step_with(c,grip(x=.47),b)

    assert action=={'kind':'drive','forward':.0125,'turn':0.,'duration_s':.4}
    assert c.search_level==2


def test_finest_scale_exhaustion_moves_to_a_new_height_basin():
    c=PixelGraspController('r1');c.search_level=len(SEARCH_SCALES)-1;b=beam()
    exhaust_current_scale(c,alignment_features(grip(x=.47),b))
    origin=c.pulses[5]

    assert step_with(c,grip(x=.47),b)=={
        'kind':'arm','servo_id':5,'pulse':origin+BASIN_OFFSETS[0],
    }
    assert c.stage=='align' and c.search_level==0
    assert c.basin_origin==origin and c.basin_index==1
    assert c.refresh_required and c.tried==set()


def test_height_basin_targets_are_unique_legal_and_exhaust_boundedly():
    c=PixelGraspController('r1');obs={'alignment':measured_alignment(20,.3)}
    targets=[]
    for _ in range(len(BASIN_OFFSETS)):
        c._next_basin(obs)
        while c.basin_motion:
            c._issue(c.basin_motion.pop(0),'test transit',obs)
        targets.append(c.pulses[5])

    assert targets==[c.basin_origin+offset for offset in BASIN_OFFSETS]
    assert len(set(targets))==len(targets)
    assert all(500<=target<=2500 for target in targets)
    assert c._next_basin(obs)=={'kind':'wait'}
    assert c.stage=='blocked'
    assert c.last_decision['reason']=='bounded height-basin exploration exhausted without visual closure'


def test_basin_move_requires_fresh_remeasurement_and_cannot_close_early():
    c=PixelGraspController('r1');obs={'alignment':measured_alignment(20,.3)}
    c._next_basin(obs)
    tracked=grip();tracked['source']='verified_optical_flow'

    assert step_with(c,tracked,beam())=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.attempts==0 and c.aligned==0
    step_with(c,grip(),beam())
    assert c.attempts==0 and c.aligned==0
    step_with(c,grip(),beam())
    assert c.attempts==0 and c.aligned==1
    assert step_with(c,grip(),beam())=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.attempts==1


def test_basin_active_budget_ignores_inactive_and_does_not_preempt_rollback():
    c=PixelGraspController('r1');c.basin_active_calls=BASIN_ACTIVE_BUDGET-1
    before=c.basin_active_calls
    assert step_with(c,grip(x=.2),beam(),active=False)=={'kind':'wait'}
    assert c.basin_active_calls==before

    c.rollback=[{'kind':'drive','forward':-.05,'turn':0.,'duration_s':.4}]
    action=step_with(c,grip(x=.2),beam())
    assert action=={'kind':'drive','forward':-.05,'turn':0.,'duration_s':.4}
    assert c.basin_index==0


def test_basin_active_budget_escapes_before_local_scales_are_exhausted():
    c=PixelGraspController('r1');c.basin_active_calls=BASIN_ACTIVE_BUDGET-1
    origin=c.pulses[5]

    action=step_with(c,grip(x=.2),beam())

    assert action=={'kind':'arm','servo_id':5,'pulse':origin+50}
    assert c.basin_index==1 and c.basin_active_calls==0
    assert c.search_level==0 and c.refresh_required


def test_basin_escape_skips_a_target_already_reached_by_local_search():
    c=PixelGraspController('r1');obs={'alignment':measured_alignment(20,.3)}
    origin=c.pulses[5]
    c._next_basin(obs)
    c.pulses[5]=origin+BASIN_OFFSETS[1]
    c.basin_motion=[]

    action=c._next_basin(obs)

    assert action=={'kind':'arm','servo_id':5,'pulse':origin+BASIN_OFFSETS[0]}
    assert c.basin_origin==origin
    assert c.basin_target==origin+BASIN_OFFSETS[2]
    assert c.basin_index==3
    assert c.basin_motion==[{'kind':'arm','servo_id':5,
                             'pulse':origin+BASIN_OFFSETS[2]}]


def test_basin_targets_near_servo_limit_stay_legal_and_steps_stay_bounded():
    c=PixelGraspController('r1');obs={'alignment':measured_alignment(20,.3)}
    c.pulses[5]=2475
    issued=[];targets=[]
    while c.stage!='blocked':
        action=c._next_basin(obs)
        if action['kind']=='wait':
            break
        issued.append(action['pulse'])
        targets.append(c.basin_target)
        while c.basin_motion:
            issued.append(c.basin_motion.pop(0)['pulse'])
        c.pulses[5]=targets[-1]

    assert targets==[2425,2375,2325,2275]
    assert len(set(targets))==4
    assert all(500<=pulse<=2500 for pulse in issued)
    assert all(abs(after-before)<=100 for before,after in zip([2475,*issued],issued))


def test_basin_active_budget_does_not_preempt_pending_candidate_rollback():
    c=PixelGraspController('r1');c.basin_active_calls=BASIN_ACTIVE_BUDGET-1
    before=measured_alignment(10,.3)
    c.pending={'before':before,
               'action':{'kind':'drive','forward':.05,'turn':0.,'duration_s':.4},
               'pulses_before':copy.deepcopy(c.pulses),'own_beam_visible':False,
               'learn':False,'label':'forward',
               'undo':[{'kind':'drive','forward':-.05,'turn':0.,'duration_s':.4}]}

    action=step_with_measured_alignment(c,measured_alignment(12,.3))

    assert action=={'kind':'drive','forward':-.05,'turn':0.,'duration_s':.4}
    assert c.basin_index==0


def test_fine_near_and_far_candidates_scale_every_command_magnitude():
    c=PixelGraspController('r1');c.search_level=2
    near=dict(c._candidates({'distance_px':20}))
    far=dict(c._candidates({'distance_px':40}))

    assert near['forward']['forward']==.0125
    assert far['forward']['forward']==.0375
    assert near['back']['forward']==-.0125 and far['back']['forward']==-.0125
    assert near['left']['turn']==.025 and far['left']['turn']==.025
    assert near['wrist+']['pulse']-c.pulses[3]==12
    assert far['wrist+']['pulse']-c.pulses[3]==25
    assert all(500<=a.get('pulse',a.get('pan_pulse',1500))<=2500
               for a in (*near.values(),*far.values()) if a['kind'] in ('arm','look'))


def test_scale_change_waits_for_a_fresh_open_measurement():
    c=PixelGraspController('r1');b=beam();current=alignment_features(grip(x=.47),b)
    exhaust_current_scale(c,current)
    tracked=grip(x=.47);tracked['source']='verified_optical_flow'

    assert step_with(c,tracked,b)=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.search_level==0 and c.refresh_required


def test_failed_capture_recovery_restarts_coarse_search_at_new_height():
    c=PixelGraspController('r1');c.search_level=2;c.stage='lift';c.lift_steps=3
    c.lift_start={'beam':beam(),'own_beam':own_beam()};c.lift_pulse=c.pulses[5]
    c.pulses[5]-=150

    with patch.object(c,'_visual_lift',return_value=False):
        action=step_with(c,grip(),beam())

    assert action=={'kind':'arm','servo_id':1,'pulse':2000}
    assert c.search_level==0 and c.height_lock


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


def test_sweep_topology_needs_two_distinct_close_open_cycles_before_trial():
    c=PixelGraspController('r1');b=beam();b['width_px']=20.
    passed={"passed":True,"candidate_count":2,"candidates":[],"reason":"pass"}
    with patch('harness.camera_pixel_grasp.evaluate_sweep_topology',return_value=passed):
        first=step_with(c,grip(x=.47),b)
        assert first=={'kind':'arm','servo_id':1,'pulse':1500}
        assert c.sweep_passes==1 and c.attempts==0
        opened=step_with(c,grip(x=.47),b)
        assert opened=={'kind':'arm','servo_id':1,'pulse':2000}
        assert c.attempts==0
        trial=step_with(c,grip(x=.47),b)
    assert trial=={'kind':'arm','servo_id':1,'pulse':1500}
    assert c.stage=='lift' and c.attempts==1
    assert c.last_observation['sweep_topology']['cycle']==2
    assert 'never assume contact or success' in c.last_decision['reason']


def test_sweep_first_pass_is_reset_by_intervening_nonjaw_action():
    c=PixelGraspController('r1');b=beam();b['width_px']=20.
    passed={"passed":True,"candidate_count":1,"candidates":[],"reason":"pass"}
    with patch('harness.camera_pixel_grasp.evaluate_sweep_topology',return_value=passed):
        assert step_with(c,grip(x=.47),b)['pulse']==1500
        assert step_with(c,grip(x=.47),b,active=False)=={'kind':'wait'}
        assert c.sweep_passes==0 and c.sweep_phase is None
        assert step_with(c,grip(x=.47),b)['pulse']==2000
        # A later passing frame can only become a new first confirmation.
        assert step_with(c,grip(x=.47),b)['pulse']==1500
    assert c.sweep_passes==1 and c.attempts==0


def test_flow_only_frame_cannot_start_sweep_confirmation():
    c=PixelGraspController('r1');b=beam();b['width_px']=20.
    tracked=grip(x=.47);tracked['source']='verified_optical_flow'
    with patch('harness.camera_pixel_grasp.evaluate_sweep_topology') as topology:
        action=step_with(c,tracked,b)
    assert action['kind']!='arm' or action.get('servo_id')!=1
    topology.assert_not_called()
    assert c.sweep_passes==0 and c.attempts==0


def test_failed_second_sweep_returns_to_normal_policy_without_recounting_frame():
    c=PixelGraspController('r1');b=beam();b['width_px']=20.
    passed={"passed":True,"candidate_count":1,"candidates":[],"reason":"pass"}
    failed={"passed":False,"candidate_count":1,"candidates":[],"reason":"fail"}
    with patch('harness.camera_pixel_grasp.evaluate_sweep_topology',side_effect=[passed,failed]) as topology:
        step_with(c,grip(x=.47),b)
        step_with(c,grip(x=.47),b)
        action=step_with(c,grip(x=.47),b)
    assert topology.call_count==2
    assert c.sweep_passes==0 and c.sweep_phase is None and c.attempts==0
    assert action['kind']!='arm' or action.get('servo_id')!=1
    assert c.last_observation['sweep_topology']['passed'] is False
