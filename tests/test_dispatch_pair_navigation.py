import copy,math
import pytest
from pathlib import Path
import numpy as np
from harness.dispatch_skill_binding import SkillBindings
from harness.dispatch_navigation_map import navigation_map,visual_barriers
from harness.dispatch_pair_navigation import PairVision,plan_route,plan_placement_route,swept_clear,payload_coupled
from harness.dispatch_plan import fixture_plan
from harness.three_robot_plan import digest
from sim.research_dispatch_arena import authored_map

ROOT=Path('tests/fixtures/dispatch_adaptive')
def bindings(variant='shared_crossing',route='north'):
    plan=fixture_plan(dock='dock_a',route=route)
    return SkillBindings({'plan':plan,'plan_hash':digest(plan),'version':1},authored_map(variant))

@pytest.mark.parametrize('image',['carry-anchor.jpg','clutter-carry-anchor.jpg'])
def test_actual_carry_image_finds_both_wheel_envelopes_and_payload(image):
    raw=(ROOT/image).read_bytes();v=PairVision(navigation_map(bindings(),raw))
    obs=v.observe(raw,raw)
    assert np.allclose(obs['r1']['center_uv'],[275,354],atol=3)
    assert np.allclose(obs['r3']['center_uv'],[274,169],atol=3)
    center=(np.array(obs['r1']['xy_m'])+obs['r3']['xy_m'])/2
    assert payload_coupled(v.payload,center,0,0)

def test_north_passage_requires_rotation_and_every_swept_edge_is_clear():
    raw=(ROOT/'carry-anchor.jpg').read_bytes();data=navigation_map(bindings(),raw)
    start=[-.18,-1.65,0];goal=[*data['goal']['center_m'],0]
    assert not swept_clear(start,goal,data)
    route=plan_route(start,goal,data)
    assert route and any(abs(p[2])>.7 for p in route)
    assert all(swept_clear(a,b,data) for a,b in zip(route,route[1:]))
    assert all(p[1]>-2.0 for p in route)

def test_unannounced_barrier_comes_from_rgb_and_blocks_only_north():
    raw=(ROOT/'north-barrier.jpg').read_bytes()
    assert authored_map('north_blocked')==authored_map('shared_crossing')
    obstacles=visual_barriers(raw,authored_map())
    assert len(obstacles)==1 and obstacles[0]['source'].startswith('conservative TOP RGB')
    assert not visual_barriers((ROOT/'carry-anchor.jpg').read_bytes(),authored_map())
    for route,possible in [('north',False),('south',True)]:
        data=navigation_map(bindings(route=route),raw,planned_box_completion=True)
        result=plan_placement_route([-.18,-1.65,0],[*data['goal']['center_m'],0],data)
        assert bool(result)==possible

def test_pickup_resource_prevents_solo_from_waiting_in_pair_turn_space():
    b=bindings();assert b.permission('beam','APPROACH')
    assert not b.permission('box','APPROACH')
    b.finish('beam');assert b.permission('box','APPROACH')


def test_actual_painted_floor_does_not_erase_wheel_motion():
    import cv2,json
    from harness.dispatch_pair_navigation import track_wheel_motion
    before=cv2.imread(str(ROOT/'pair-267-rotate-carry-top.jpg'))
    after=cv2.imread(str(ROOT/'pair-268-rotate-carry-top.jpg'))
    prior=json.loads((ROOT/'wheel-prior.json').read_text())
    template=cv2.imread(str(ROOT/'wheel-prior-template.png'),0)
    center,angle,evidence=track_wheel_motion(before,after,prior['center'],prior['angle_deg'],template)
    assert np.linalg.norm(np.array(center)-prior['center'])<3
    assert .1<angle-prior['angle_deg']<2
    assert evidence['inlier_fraction']>.9 and evidence['max_cycle_error_px']<.2
    with pytest.raises(ValueError):
        track_wheel_motion(before,np.zeros_like(after),prior['center'],prior['angle_deg'],template)


def test_solo_corridor_respects_new_chicane_and_unvalidated_ridge():
    from harness.dispatch_navigation_map import solo_gate
    raw=(ROOT/'carry-anchor.jpg').read_bytes()
    gate=solo_gate(authored_map('narrow_south'),'south',raw)
    assert -2.70<=gate[1]<=-2.63
    with pytest.raises(RuntimeError,match='unvalidated terrain'):
        solo_gate(authored_map('rough_south'),'south',raw)


def test_navigation_uses_original_jpeg_without_lossy_second_encoding(monkeypatch):
    import cv2
    raw=(ROOT/'carry-anchor.jpg').read_bytes()
    data=navigation_map(bindings(),raw)
    monkeypatch.setattr(cv2,'imencode',lambda *a,**k:(_ for _ in ()).throw(AssertionError('raw actor image re-encoded')))
    obs=PairVision(data).observe(raw,raw)
    assert set(obs)=={'r1','r3'}


def test_waiting_cargo_remains_an_obstacle_until_dependency_is_completed():
    from harness.dispatch_navigation_map import visual_boxes
    raw=(ROOT/'north-barrier.jpg').read_bytes()
    boxes=visual_boxes(raw,authored_map())
    assert len(boxes)==1 and boxes[0]['center_m'][0]<0
    data=navigation_map(bindings(route='south'),raw,other_robot_center_px=[110,571])
    assert any(o['id']=='rgb_other_robot' for o in data['obstacles'])
    assert plan_route([-.18,-1.65,0],[*data['goal']['center_m'],0],data) is None
    future=navigation_map(bindings(route='south'),raw,planned_box_completion=True)
    assert all(o['id']!='rgb_other_robot' for o in future['obstacles'])
    assert any(o['id']=='planned_yield_pose' for o in future['obstacles'])
    assert plan_placement_route([-.18,-1.65,0],[*future['goal']['center_m'],0],future)


def test_box_first_reserves_pickup_until_visual_yield_completion():
    plan=fixture_plan(dock='dock_a',route='south')
    plan['tasks'][0]['after']=['box_job'];plan['tasks'][1]['after']=[]
    b=SkillBindings({'plan':plan,'plan_hash':digest(plan),'version':1},authored_map())
    assert not b.permission('beam','APPROACH')
    assert b.permission('box','APPROACH')
    assert not b.permission('beam','APPROACH')
    b.finish('box')
    assert b.permission('beam','APPROACH')


def test_release_yield_observes_real_wheels_and_does_not_assume_a_command_moved_them():
    from harness.dispatch_yield import SoloYield
    raw=(ROOT/'box-released-top.jpg').read_bytes()
    policy=SoloYield(authored_map(),[838.09,544.42])
    first,evidence=policy.decide(raw)
    assert first['forward']<0 and not policy.done
    for _ in range(4):
        action,again=policy.decide(raw)
        assert np.allclose(again['observation']['center_px'],evidence['observation']['center_px'],atol=.01)
        assert again['index']==0 and not policy.done
    import cv2
    black=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    with pytest.raises(ValueError):policy.decide(black)


def test_current_wheel_pixels_bound_accumulated_motion_drift():
    from harness.dispatch_pair_navigation import reanchor_wheels
    import cv2
    raw=(ROOT/'wheel-drift-anchor.jpg').read_bytes();v=PairVision(navigation_map(bindings(),raw));v.observe(raw,raw)
    mask=v._mask(cv2.imread(str(ROOT/'wheel-drift-top.jpg')))
    center=np.array([785.10121516,214.11153672])
    new,angle,evidence=reanchor_wheels(mask,v.templates['r1'],center,57.800334088)
    assert evidence['accepted'] and min(evidence['corner_pixels'])>=8
    assert new[0]<center[0] and new[1]>center[1]
    assert np.linalg.norm(np.array(new)-center)<=1.000001
    _,_,missing=reanchor_wheels(np.zeros_like(mask),v.templates['r1'],center,57.8)
    assert not missing['accepted']


def test_placement_can_shift_inside_slot_without_shrinking_loaded_envelope():
    from harness.dispatch_pair_navigation import plan_placement_route,footprint_clear
    raw=(ROOT/'north-barrier.jpg').read_bytes();data=navigation_map(bindings(route='south'),raw,planned_box_completion=True)
    start=[-.1274098649,-1.6584060607,.0027581691]
    goal=[1.5239043417,-1.3846056692,.0027581691]
    box=next(o for o in data['obstacles'] if o['id']=='planned_box_slot')
    box['center_m']=[1.7845643765,-1.3856339014]
    assert not footprint_clear(goal,data)
    route=plan_placement_route(start,goal,data)
    assert route and abs(route[-1][0]-goal[0])<=.06+1e-9
    assert abs(route[-1][1]-goal[1])<=.015+1e-9
    assert all(swept_clear(a,b,data) for a,b in zip(route,route[1:]))
    assert data['footprint']=={'half_forward_m':.20,'half_lateral_m':.47,'margin_m':.025}


def test_common_twist_does_not_pull_two_carriers_together_or_counter_rotate():
    from harness.dispatch_pair_navigation import rigid_pair_commands,rotate
    positions={'r1':np.array([1.49,-1.69]),'r3':np.array([1.31,-1.05])}
    headings={'r1':.346,'r3':.412}
    actions,evidence=rigid_pair_commands(positions,headings,[1.41,-1.36,.35],.32,[0.,0.],0.)
    assert actions['r1']['turn']==actions['r3']['turn']
    world={r:rotate([a['forward']*1.57,a['left']*1.18],headings[r]) for r,a in actions.items()}
    line=positions['r1']-positions['r3']
    assert abs(float((world['r1']-world['r3'])@line))<1e-10
    assert np.allclose(sum(world.values())/2,evidence['common_translation_m_s'])


def test_shared_heading_is_not_changed_by_one_wheel_appearance_bias():
    from harness.dispatch_pair_navigation import rigid_pair_commands
    positions={'r1':np.array([0.,-.325]),'r3':np.array([0.,.325])}
    actions,_=rigid_pair_commands(positions,{'r1':0.,'r3':.12},[0.,0.,0.],0.,[0.,0.],0.)
    assert all(a['turn']==a['forward']==a['left']==0. for a in actions.values())


def test_motor_saturation_preserves_the_common_twist_and_span():
    from harness.dispatch_pair_navigation import rigid_pair_commands,rotate
    positions={'r1':np.array([0.,-.325]),'r3':np.array([0.,.325])}
    headings={'r1':.4,'r3':-.3}
    actions,evidence=rigid_pair_commands(positions,headings,[0.,0.,0.],0.,[-.075,.075],.1)
    assert 0<evidence['shared_motor_scale']<1
    world={r:rotate([a['forward']*1.57,a['left']*1.18],headings[r]) for r,a in actions.items()}
    line=positions['r1']-positions['r3']
    assert abs(float((world['r1']-world['r3'])@line))<1e-10
    assert np.allclose(sum(world.values())/2,evidence['common_translation_m_s'])
    assert actions['r1']['turn']==actions['r3']['turn']
    for action in actions.values():
        assert -.050000001<=action['forward']<=.080000001
        assert abs(action['left'])<=.080000001 and abs(action['turn'])<=.100000001


def test_span_recovery_pushes_apart_when_one_carrier_catches_the_other():
    from harness.dispatch_pair_navigation import rigid_pair_commands,rotate
    positions={'r1':np.array([0.,-.31]),'r3':np.array([0.,.31])}
    headings={'r1':0.,'r3':0.}
    actions,evidence=rigid_pair_commands(positions,headings,[0.,0.,0.],0.,[0.,0.],0.,target_span=.65)
    assert actions['r1']['left']<0 and actions['r3']['left']>0
    assert actions['r1']['turn']==actions['r3']['turn']==0.
    assert evidence['radial_correction_m_s']<=.05
    nominal={r:p*(.644/.62) for r,p in positions.items()}
    quiet,evidence=rigid_pair_commands(nominal,headings,[0.,0.,0.],0.,[0.,0.],0.,target_span=.65)
    assert all(a['forward']==a['left']==a['turn']==0. for a in quiet.values())
    damping,evidence=rigid_pair_commands(nominal,headings,[0.,0.,0.],0.,[0.,0.],0.,target_span=.65,span_rate=-.02)
    assert damping['r1']['left']<0 and damping['r3']['left']>0


def test_current_body_direction_comes_from_complete_wheel_arrangement():
    import cv2
    from harness.dispatch_pair_navigation import reanchor_wheel_geometry
    frame=cv2.imread(str(ROOT/'compressed-pair-top.jpg'))
    mask=PairVision._mask(None,frame)
    # Pixel estimates deliberately biased from the visible upper chassis.
    center,angle,evidence=reanchor_wheel_geometry(mask,[744.,161.],5.)
    assert evidence['accepted'] and angle<5.
    assert abs(evidence['observed_center_px'][0]-742)<4
    assert all(8<=n<=150 for n in evidence['corner_pixels'])
    for unsupported in (np.zeros_like(mask),np.full_like(mask,255)):
        _,_,evidence=reanchor_wheel_geometry(unsupported,[744.,161.],5.)
        assert not evidence['accepted']


def test_chassis_motion_survives_paint_without_tracking_tread():
    import cv2,json
    from harness.dispatch_pair_navigation import track_wheel_motion
    before=cv2.imread(str(ROOT/'pair-267-rotate-carry-top.jpg'))
    after=cv2.imread(str(ROOT/'pair-268-rotate-carry-top.jpg'))
    prior=json.loads((ROOT/'wheel-prior.json').read_text())
    template=cv2.imread(str(ROOT/'wheel-prior-template.png'),0)
    center,angle,evidence=track_wheel_motion(before,after,prior['center'],prior['angle_deg'],template,chassis_only=True)
    assert np.linalg.norm(np.asarray(center)-prior['center'])<3
    assert 0<angle-prior['angle_deg']<2
    assert evidence['inlier_fraction']>.9
    with pytest.raises(ValueError):
        track_wheel_motion(before,np.zeros_like(after),prior['center'],prior['angle_deg'],template,chassis_only=True)


def test_box_delivery_uses_the_padded_region_before_point_chasing_hits_the_beam():
    from harness.dispatch_skill_binding import ImageRoute,pixel_from_map
    raw=(ROOT/'box-delivered-inside-slot.jpg').read_bytes();route=ImageRoute(bindings(),'box')
    route.box_center=np.array([835.,179.]);route.observe(raw);route.index=len(route.points)-1
    action,evidence=route.observe(raw)
    assert max(abs(e) for e in evidence['error_px'])>4
    assert evidence['ready'] and evidence['destination_region']['inside']
    assert all(action[k]==0 for k in ['forward','left','turn'])
    _,confirmed=route.observe(raw);assert confirmed['done']
    # The same image is outside a different authored destination.
    wrong=ImageRoute(bindings(),'box');wrong.dock='dock_b';wrong.box_center=np.array([835.,179.])
    wrong.observe(raw);wrong.index=len(wrong.points)-1
    _,evidence=wrong.observe(raw);assert not evidence['ready'] and not evidence['done']


def test_dim_box_reacquires_visible_silhouette_before_chasing_the_slot_center():
    from harness.dispatch_skill_binding import ImageRoute
    from harness.camera_goal_transport import decode
    raw=(ROOT/'box-inside-slot-stalled.jpg').read_bytes();route=ImageRoute(bindings(),'box')
    route.box_center=np.array([847.,179.]);route.box_previous=decode(raw)
    route.observe(raw);route.index=len(route.points)-1
    action,evidence=route.observe(raw)
    assert evidence['tracking']['method']=='cyan component'
    assert evidence['ready'] and evidence['destination_region']['inside']
    assert all(action[k]==0 for k in ['forward','left','turn'])


def test_small_dim_cargo_fragment_preserves_colour_identity_during_motion():
    import json
    from harness.dispatch_skill_binding import ImageRoute
    from harness.camera_goal_transport import decode
    prior=json.loads((ROOT/'dim-moving-box-prior.json').read_text())
    route=ImageRoute(bindings(),'box')
    route.box_center=np.array(prior['center']);route.box_delta=np.array(prior['delta'])
    route.box_previous=decode((ROOT/'dim-moving-box-prior.jpg').read_bytes())
    route.points=[np.array([841.87,176.93])]  # Route already selected before carrying.
    _,e=route.observe((ROOT/'dim-moving-box.jpg').read_bytes())
    assert e['tracking']['method']=='cyan component'
    assert np.allclose(e['cargo_center_px'],[463.5,130.],atol=2.)


def test_observed_static_floor_memory_prevents_apron_tracking_loss():
    import json
    from harness.dispatch_skill_binding import ImageRoute
    from harness.camera_goal_transport import decode
    prior=json.loads((ROOT/'box-background-prior.json').read_text())
    route=ImageRoute(bindings(),'box')
    route.box_center=np.array(prior['center']);route.box_delta=np.array(prior['delta'])
    route.box_previous=decode((ROOT/'box-background-prior.jpg').read_bytes())
    route.box_background=decode((ROOT/'box-background-reference.jpg').read_bytes())
    route.box_origin=np.array(prior['origin']);route.box_background_sha=prior['background_sha256']
    route.points=[np.array([637.17,359.5]),np.array([880.6,359.5])]
    _,e=route.observe((ROOT/'box-background-failure.jpg').read_bytes())
    assert e['tracking']['static_background']['active']
    assert e['tracking']['static_background']['reference_sha256']==prior['background_sha256']
    assert 2<np.linalg.norm(np.array(e['cargo_center_px'])-prior['center'])<5
    assert not e['ready'] and not e['done']


def test_visible_shaft_survives_a_narrow_consistent_contrast_band():
    import json
    from harness.dispatch_beam_tracker import CarriedBeamTracker
    tracker=CarriedBeamTracker()
    tracker.previous=json.loads((ROOT/'shaft-narrow-contrast-prior.json').read_text())
    b=tracker.observe((ROOT/'shaft-narrow-contrast.jpg').read_bytes())
    assert b['tracking']['threshold_support']>=3
    assert 78<=b['length_px']<=83 and b['width_px']<=20
    assert np.allclose(np.array(b['center'])*b['image_size'],[636,407],atol=3)


def test_persistent_rgb_span_error_increases_recovery_with_bounded_memory():
    from harness.dispatch_pair_navigation import update_span_bias
    bias=0.
    for _ in range(20):bias=update_span_bias(bias,.619,.639)
    assert bias==pytest.approx(.025)
    for _ in range(20):bias=update_span_bias(bias,.639,.639)
    assert abs(bias)<1e-6
    for _ in range(20):bias=update_span_bias(bias,.659,.639)
    assert bias==pytest.approx(-.025)


def test_release_yield_recovers_four_corners_when_last_cargo_hint_clips_the_chassis():
    import json,cv2
    from harness.dispatch_yield import SoloYield,acquire_wheel_geometry
    raw=(ROOT/'yield-offset-cargo-hint.jpg').read_bytes()
    hint=json.loads((ROOT/'yield-offset-cargo-hint.json').read_text())['cargo_hint_px']
    policy=SoloYield(authored_map(),hint)
    action,evidence=policy.decide(raw)
    obs=evidence['observation']
    assert action['forward']<0 and not policy.done
    assert 770<obs['center_px'][0]<781 and 195<obs['center_px'][1]<205
    assert obs['tracking']['candidate_count']>=4
    again=policy.decide(raw)[1]['observation']
    assert np.allclose(again['center_px'],obs['center_px'],atol=.01)
    frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    mask=PairVision._mask(None,frame)
    # Removing a complete lower row of wheels must not turn the arm into a base.
    mask[205:]=0
    with pytest.raises(ValueError):acquire_wheel_geometry(mask,np.array(hint)-[50,0])


def test_formation_footprint_uses_rgb_chassis_axis_not_mecanum_probe_drift():
    import json
    from harness.dispatch_pair_navigation import PairNavigator
    raw=json.loads((ROOT/'formation-route-after-yield.json').read_text())
    nav=PairNavigator(raw['map'],'r1')
    nav.phase='probe_settle';nav.confirmations=2;nav.heading=raw['motion_heading_rad']
    nav.vision.observe=lambda own,top:raw['observations']
    nav.vision.payload=raw['payload']
    decision=nav.decide(b'unused',b'unused')
    assert decision['status']=='track' and decision['route']
    assert abs(decision['formation_heading_rad'])<.01
    assert decision['heading_rad']>.09  # Distinct observed motor response retained.
    absolute=[[x,y,a+decision['formation_heading_rad']]for x,y,a in decision['route']]
    assert all(swept_clear(a,b,raw['map'])for a,b in zip(absolute,absolute[1:]))


def test_shared_rotation_keeps_body_headings_synchronized_with_unequal_wheel_response():
    from harness.dispatch_pair_navigation import rigid_pair_commands,rotate,wrap
    yaw={'r1':0.,'r3':0.};gains={'r1':.75,'r3':1.25};dt=.2
    for i in range(160):
        turn=.05*i*dt
        positions={'r1':rotate([0.,-.325],turn),'r3':rotate([0.,.325],turn)}
        errors={r:wrap(turn-yaw[r])for r in yaw}
        actions,e=rigid_pair_commands(positions,yaw,[0.,0.,turn],turn,[0.,0.],.05,
                                     target_span=.65,heading_errors=errors)
        for r in yaw:yaw[r]+=1.5*gains[r]*actions[r]['turn']*dt
        assert max(map(abs,e['individual_heading_correction_rad_s'].values()))<=.08
    assert abs(yaw['r1']-yaw['r3'])<math.radians(2)
    assert max(abs(v-1.6)for v in yaw.values())<math.radians(2)
