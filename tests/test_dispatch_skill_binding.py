import copy
import itertools
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import cv2
import numpy as np
import pytest
from harness.dispatch_skill_binding import SkillBindings, canonical_pair_top, beam_feature
from harness.dispatch_plan import validate_dispatch_plan
from harness.three_robot_plan import digest
from harness.solo_box_transport import SoloBoxTransport
from sim.research_dispatch_arena import authored_map
from scripts.dispatch_pair_skill import BoundPairSkill


def test_efficient_pair_capture_preserves_consumed_images_and_default(tmp_path):
    from scripts.run_dispatch_skills import SkillScene
    scene=SkillScene({},tmp_path)
    (tmp_path/'rgb').mkdir()
    scene.world=SimpleNamespace(
        render_team_jpeg=Mock(side_effect=lambda **kw:kw['camera'].encode()),
        render_jpeg=Mock(side_effect=lambda **kw:kw['robot_id'].encode()))
    selection={'own_robots':('r2','r3'),'overview':False}
    baseline=scene.capture('baseline',**selection)
    assert scene.world.render_team_jpeg.call_count==2
    assert scene.world.render_jpeg.call_count==3
    assert (tmp_path/'baseline-overview.jpg').exists()
    scene.world.render_team_jpeg.reset_mock();scene.world.render_jpeg.reset_mock()
    scene.efficient_capture=True
    optimized=scene.capture('optimized',**selection)
    assert scene.world.render_team_jpeg.call_count==1
    assert scene.world.render_jpeg.call_count==2
    assert not (tmp_path/'optimized-overview.jpg').exists()
    for rid in ('r2','r3'):
        for key in ('own_bytes','top_bytes'):
            assert optimized[rid][key]==baseline[rid][key]
        for key in ('own_rgb','shared_top_rgb'):
            assert optimized[rid][key]['sha256']==baseline[rid][key]['sha256']
    # Existing TOP-only consumers use r1 even when r1 is not in the pair.
    assert optimized['r1']['top_bytes']==baseline['r1']['top_bytes']
    assert 'own_bytes' not in optimized['r1']
    assert all(frame['frame_id']==2 for frame in optimized.values())
    scene.world.render_jpeg.reset_mock()
    assert all('own_bytes' in frame for frame in scene.capture('planning').values())
    assert scene.world.render_jpeg.call_count==3


def committed(order=('r1','r3','r2'),after=False):
    plan=validate_dispatch_plan({'dock':'dock_b','tasks':[
        {'id':'beam_job','object':'beam','participants':list(order[:2]),'route':'north','after':['box_job'] if after else []},
        {'id':'box_job','object':'box','participants':[order[2]],'route':'south','after':[]}]})
    return {'plan':plan,'plan_hash':digest(plan),'proposal_id':'plan-test','version':1}


@pytest.mark.parametrize('order',list(itertools.permutations(('r1','r2','r3'))))
def test_every_allocation_reaches_its_physical_endpoints(order):
    c=committed(order);b=SkillBindings(c,authored_map('open'))
    io=SimpleNamespace(out=Path('/tmp/unused'),pair_drive=Mock(),pair_arm=Mock())
    skill={'initialization_replay':[{'targets':{'r1':{1:2000},'r3':{1:2000}}}]}
    pair=BoundPairSkill(io,b,skill,{}, {},Path('/tmp/models'),b'')
    pair.drive({'r1':.04,'r3':.08})
    commands=io.pair_drive.call_args.args[0]
    assert commands[order[1]]['forward']==.04
    assert commands[order[0]]['forward']==.08
    assert order[2] not in commands
    pair.replay([{'targets':{'r1':{6:2500},'r3':{6:500}},'duration_s':1.}], 'initialization')
    assert io.pair_arm.call_args.args[0]=={order[1]:{6:2500},order[0]:{6:500}}
    solo=SoloBoxTransport(robot_id=b.solo,navigator=Mock())
    assert solo.box.robot_id==order[2]
    assert b.programs[order[0]][0]['role']=='end_a'
    assert b.programs[order[1]][0]['model_slot']=='r1'


def test_no_silent_route_change_or_revoked_plan_execution():
    c=committed();c['plan']['tasks'][0]['route']='south';c['plan_hash']=digest(c['plan'])
    b=SkillBindings(c,authored_map('narrow_south'))
    with pytest.raises(RuntimeError,match='PAIR_ROUTE_TOO_NARROW'):b.check_route()
    assert b.committed==c
    changed=copy.deepcopy(c);changed['plan']['dock']='dock_a'
    with pytest.raises(RuntimeError,match='replaced'):b.authorize(changed)
    b.revoked=True
    assert not b.permission('beam','APPROACH')


def test_task_dependencies_and_resource_occupancy_persist_on_revoke():
    b=SkillBindings(committed(after=True),authored_map('open'))
    assert b.permission('beam','APPROACH')
    assert not b.permission('beam','GRASP')
    assert b.permission('box','TRANSIT')
    b.finish('box')
    assert b.permission('beam','GRASP') and b.permission('beam','TRANSIT')
    locks=copy.deepcopy(b.locks);b.revoked=True
    assert not b.permission('box','TRANSIT') and b.locks==locks


def test_route_overlap_has_independent_travel_and_exclusive_unload():
    b=SkillBindings(committed(),authored_map('open'),route_overlap=True)
    assert b.permission('beam','GRASP')
    assert not b.permission('box','GRASP')
    b.note_transit_command('beam')
    assert b.permission('box','GRASP') and b.permission('box','TRANSIT')
    assert b.permission('beam','TRANSIT')
    assert set(b.locks)=={'north_gate','south_gate'}
    assert not b.permission('box','UNLOAD')
    assert b.permission('beam','UNLOAD')
    locks=copy.deepcopy(b.locks);b.revoked=True
    assert not b.permission('box','UNLOAD') and b.locks==locks
    b.revoked=False;b.finish('beam')
    assert b.permission('box','UNLOAD')
    assert b.locks=={'south_gate':'box_job','dispatch_apron':'box_job'}


def test_route_overlap_does_not_remove_agreed_dependencies_or_assume_moving_obstacles():
    with pytest.raises(ValueError,match='dependencies are never removed'):
        SkillBindings(committed(after=True),authored_map('open'),route_overlap=True)
    with pytest.raises(ValueError,match='moving-obstacle'):
        SkillBindings(committed(),authored_map('shared_crossing'),route_overlap=True)


def test_route_overlap_queue_is_at_rgb_staging_point_before_unload():
    from harness.dispatch_skill_binding import ImageRoute,pixel_from_map
    b=SkillBindings(committed(),authored_map('open'),route_overlap=True)
    raw=Path('tests/fixtures/dispatch_skill_transfer/box-top-held.jpg').read_bytes()
    route=ImageRoute(b,'box');_,e=route.observe(raw)
    assert len(route.points)==7
    west=b.static_map['regions']['dispatch_apron']['center_m'][0]-b.static_map['regions']['dispatch_apron']['half_extents_m'][0]
    assert route.points[1][0]==pytest.approx(pixel_from_map([west-.20,0],b.static_map,(720,960))[0])
    route.index=1;route.points[1]=np.array(e['cargo_center_px'])
    for _ in range(4):
        action,e=route.observe(raw)
        assert e['waiting_for_resource'] and route.index==1
        assert action['forward']==action['left']==0.
    b.finish('beam')
    route.observe(raw);route.observe(raw)
    assert route.index==2 and b.locks['dispatch_apron']=='box_job'


def test_resource_queue_does_not_exhaust_remaining_skill_decision(tmp_path):
    import base64
    from scripts.run_dispatch_skills import SkillScene
    class ReadyBox:
        phase='approach'
        last_attachment=None
        def decide(self, observation):
            self.phase='lower'
            return {'kind':'pose','pulses':{3:689}}
    raw=cv2.imencode('.jpg',np.zeros((480,640,3),np.uint8))[1].tobytes()
    b=SkillBindings(committed(),authored_map('open'))
    assert b.permission('beam','GRASP')
    solo=SoloBoxTransport(robot_id=b.solo,navigator=Mock())
    solo.initialized=True;solo.steps=599;solo.box=ReadyBox()
    scene=SkillScene.__new__(SkillScene)
    scene.solo=solo;scene.bindings=b;scene.solo_lease=0.;scene.solo_started=None
    scene.solo_rows=[];scene.out=tmp_path;scene.video=None
    scene.authorize=lambda:None;scene.time=lambda:1.
    scene.solo_executor=SimpleNamespace(idle=True,tick=Mock(),submit=Mock())
    scene.world=SimpleNamespace(render_team_jpeg=lambda **kw:raw)
    scene.ports={b.solo:SimpleNamespace(capture=lambda:{'image':base64.b64encode(raw).decode()})}
    for _ in range(3):scene._solo_tick()
    assert solo.steps==599 and solo.phase=='approach'
    assert all(r['top_evidence']['waiting_before_grasp'] for r in scene.solo_rows)
    b.finish('beam');scene._solo_tick()
    assert solo.steps==600 and solo.phase=='lower'
    with pytest.raises(RuntimeError,match='decision budget exhausted'):
        solo.decide({},raw)


def test_open_dock_b_does_not_drop_from_floor_plane_alignment():
    from harness.dispatch_skill_binding import ImageRoute
    raw=Path('tests/fixtures/dispatch_adaptive/dock-b-premature-carry-done.jpg').read_bytes()
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'beam')
    route.observe(raw);route.index=len(route.points)-1
    action,evidence=route.observe(raw)
    assert not evidence['ready'] and not evidence['done']
    # The failed carry stopped here. Continue south into the slot and west to
    # leave room for the box carrier; all evidence comes from this RGB frame.
    assert action['left']<0 and action['forward']<0


def test_rgb_transform_preserves_pixels_without_reference_substitution():
    reference=Path('tests/fixtures/camera_goal_transport/reference-top.jpg').read_bytes()
    image=cv2.imdecode(np.frombuffer(reference,np.uint8),cv2.IMREAD_COLOR)
    shifted=cv2.warpAffine(image,np.float32([[1,0,-150],[0,1,-80]]),(960,720))
    # Novelty must survive preprocessing rather than being filled from the model.
    shifted[400:440,100:140]=(255,0,255)
    raw=cv2.imencode('.jpg',shifted)[1].tobytes()
    aligned,evidence=canonical_pair_top(raw,reference)
    assert np.allclose(evidence['translation_px'],[150,80],atol=2)
    decoded=cv2.imdecode(np.frombuffer(aligned,np.uint8),cv2.IMREAD_COLOR)
    assert decoded[495,265,0]>220 and decoded[495,265,2]>220
    assert np.allclose(beam_feature(aligned)['center'],beam_feature(reference)['center'],atol=.003)


def test_solo_navigation_cannot_skip_existing_attachment_failure():
    navigator=Mock();skill=SoloBoxTransport(robot_id='r3',navigator=navigator)
    skill.initialized=True;skill.box.phase='carry'
    skill.box.decide=Mock(return_value={'kind':'finish','reason':'VISUAL_LOAD_DROPPED_OR_OCCLUDED'})
    action,_=skill.decide({},b'')
    assert action['kind']=='finish' and skill.done
    navigator.observe.assert_not_called()


def test_model_transfer_checks_source_hash_before_rewriting_manifest(tmp_path):
    import json
    from scripts.run_three_robot_mission import prepare_grasp_models
    source=tmp_path/'source';source.mkdir()
    (source/'bad.json').write_text('{"tampered":true}')
    (source/'student-skill.json').write_text(json.dumps({'models':{'r1':{'path':'bad.json','sha256':'wrong'}}}))
    with pytest.raises(ValueError,match='source model path/hash mismatch'):
        prepare_grasp_models(source,tmp_path/'export')
    assert not (tmp_path/'export').exists()


def test_grasp_retains_prior_image_alignment_when_visible_tip_changes():
    ref=Path('tests/fixtures/camera_goal_transport/reference-top.jpg').read_bytes()
    first,meta=canonical_pair_top(ref,ref)
    image=cv2.imdecode(np.frombuffer(ref,np.uint8),cv2.IMREAD_COLOR)
    # Cover one endpoint without moving the beam. The prior RGB transform must
    # remain fixed, so occlusion cannot translate the entire scene to a new goal.
    image[296:319,480:496]=0
    changed=cv2.imencode('.jpg',image)[1].tobytes()
    _,after=canonical_pair_top(changed,ref,translation_px=meta['translation_px'])
    assert after['translation_px']==meta['translation_px']
    assert after['fixed_from_prior_rgb']


def test_dispatch_carry_preserves_shape_under_observed_orange_yellow_lighting():
    import math
    from harness.camera_goal_transport import own_payload
    root=Path('tests/fixtures/dispatch_skill_transfer')
    a,b=(root/'beam-anchor.jpg').read_bytes(),(root/'beam-lit.jpg').read_bytes()
    old_a,old_b=own_payload(a),own_payload(b)
    assert math.dist(old_a[1:],old_b[1:])>.15
    current,anchor=own_payload(b,hue_upper=35),own_payload(a,hue_upper=35)
    assert .25<=current[0]/anchor[0]<=4 and math.dist(current[1:],anchor[1:])<=.15
    blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    assert own_payload(blank,hue_upper=35) is None


def test_loaded_top_lighting_keeps_shaft_geometry_and_rejects_floor():
    raw=(Path(__file__).parent/'fixtures/dispatch_skill_transfer/beam-top-lit.jpg').read_bytes()
    with pytest.raises(ValueError):beam_feature(raw)
    beam=beam_feature(raw,hue_upper=35)
    assert 90<beam['length_px']<120 and beam['width_px']<25
    assert beam['center'][0]<.4
    transformed,record=canonical_pair_top(raw,(Path(__file__).parent/"fixtures/camera_goal_transport/reference-top.jpg").read_bytes(),hue_upper=35)
    assert record['hue_upper']==35 and record['observed_beam']==beam


def test_dispatch_box_route_rejects_cyan_floor_distractors():
    from harness.dispatch_skill_binding import ImageRoute
    raw=(Path(__file__).parent/'fixtures/dispatch_skill_transfer/box-top-held.jpg').read_bytes()
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    action,evidence=route.observe(raw)
    assert np.allclose(evidence['cargo_center_px'],[274,545],atol=2)
    assert not evidence['done'] and action['kind']=='mecanum'


def test_route_cannot_stop_in_diagonal_dead_zone():
    from harness.dispatch_skill_binding import ImageRoute
    raw=Path('tests/fixtures/dispatch_skill_transfer/box-top-held.jpg').read_bytes()
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    _,evidence=route.observe(raw)
    center=np.array(evidence['cargo_center_px'])
    route.points=[center+[4.,5.],center+[50.,5.]]
    assert route.observe(raw)[1]['ready']
    route.observe(raw)
    action,_=route.observe(raw)
    assert route.index==1 and action['forward']>0


def test_contact_profile_preserves_robot_cargo_physics_and_cameras():
    mujoco=pytest.importorskip('mujoco')
    from sim.dispatch_contact_profile import contact_profile
    from sim.research_dispatch_arena import build_scene_xml,episode
    from sim.multi_masterpi_production import build_multi_robot_xml
    from scripts.probe_dual_grasp_sync import _plain_beam_xml
    xml,_=build_scene_xml(_plain_beam_xml(build_multi_robot_xml)(),episode('open',11))
    old=mujoco.MjModel.from_xml_string(xml)
    new=mujoco.MjModel.from_xml_string(contact_profile(xml,'local_contact'))
    for attr in ('geom_friction','body_mass','geom_size','geom_pos','cam_pos','cam_quat','cam_fovy','actuator_gainprm','actuator_forcerange'):
        assert np.array_equal(getattr(old,attr),getattr(new,attr)),attr
    assert new.opt.noslip_iterations==0 and new.opt.timestep==.0005
    assert new.npair-old.npair==12 and np.all(new.pair_solreffriction[:,1]==-3000)
    fine=mujoco.MjModel.from_xml_string(contact_profile(xml,'local_contact_fine'))
    for attr in ('geom_friction','body_mass','geom_size','geom_pos','cam_pos','cam_quat','cam_fovy',
                 'actuator_gainprm','actuator_forcerange','pair_friction','pair_solref','pair_solimp','pair_margin','pair_gap'):
        assert np.array_equal(getattr(new,attr),getattr(fine,attr)),attr
    assert fine.opt.noslip_iterations==0 and fine.opt.timestep==.00025
    assert fine.npair==new.npair and np.all(fine.pair_solreffriction[:,1]==-6000)


def test_grasp_reserves_transport_before_a_load_is_lifted():
    b=SkillBindings(committed(),authored_map('open'))
    assert b.permission('beam','GRASP')
    assert b.permission('box','APPROACH')
    assert not b.permission('box','GRASP')
    b.finish('beam')
    assert b.permission('box','GRASP')


def test_box_approaches_from_east_of_the_beam_slot():
    from harness.dispatch_skill_binding import ImageRoute,pixel_from_map
    raw=Path('tests/fixtures/dispatch_skill_transfer/box-top-held.jpg').read_bytes()
    b=SkillBindings(committed(),authored_map('open'));route=ImageRoute(b,'box')
    _,e=route.observe(raw)
    assert len(e['waypoints_px'])==6
    points=np.array(e['waypoints_px'])
    assert points[-2,0]>points[-1,0]>points[1,0]
    assert points[2,1]==points[3,1]


def test_box_tracking_survives_observed_lighting_change_without_global_reacquisition():
    from harness.dispatch_skill_binding import ImageRoute
    root=Path('tests/fixtures/dispatch_skill_transfer')
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    _,first=route.observe((root/'box-transit-274.jpg').read_bytes())
    _,second=route.observe((root/'box-transit-275.jpg').read_bytes())
    assert 3<np.linalg.norm(np.array(first['cargo_center_px'])-second['cargo_center_px'])<15
    with pytest.raises(RuntimeError):
        route.observe((root/'box-top-held.jpg').read_bytes())


def test_dispatch_attachment_does_not_merge_cyan_floor_with_held_box():
    import base64
    from harness.visual_attachment import compare_box_comotion
    root=Path('tests/fixtures/dispatch_skill_transfer')
    a,b=[base64.b64encode((root/f'box-own-{i}.jpg').read_bytes()).decode() for i in (158,159)]
    assert not compare_box_comotion(a,b)['attached']
    calibrated=compare_box_comotion(a,b,min_saturation=150)
    assert calibrated['attached'] and calibrated['thresholds']['min_iou']==.88


def test_box_tracking_uses_actual_prior_appearance_over_same_colour_floor():
    from harness.dispatch_skill_binding import ImageRoute
    root=Path('tests/fixtures/dispatch_skill_transfer')
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    route.observe((root/'box-floor-122.jpg').read_bytes())
    # Previous tracking result is itself reconstructed from archived RGB.
    route.box_center=np.array([395.,575.]);route.box_delta[:]=0
    route.observe((root/'box-floor-162.jpg').read_bytes())
    _,e=route.observe((root/'box-floor-163.jpg').read_bytes())
    assert np.allclose(e['cargo_center_px'],[402.,575.],atol=3)
    assert e['tracking']['method'] in {'cyan component','bidirectional RGB feature motion; own attachment independently required'}
    if 'consistent_features' in e['tracking']:assert e['tracking']['consistent_features']>=3
    _,after=route.observe((root/'box-floor-165.jpg').read_bytes())
    assert 409<after['cargo_center_px'][0]<423
    blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    with pytest.raises(RuntimeError):route.observe(blank)


def test_tracked_thin_cargo_keeps_identity_after_leaving_cyan_floor():
    from harness.dispatch_skill_binding import ImageRoute
    root=Path('tests/fixtures/dispatch_skill_transfer')
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    route.box_center=np.array([577.,574.]);route.box_delta=np.array([6.,0.])
    route.box_previous=cv2.imdecode(np.frombuffer((root/'box-edge-190.jpg').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    _,e=route.observe((root/'box-edge-191.jpg').read_bytes())
    assert np.allclose(e['cargo_center_px'],[583.,573.],atol=2)
    assert e['tracking']['method']=='cyan component'
    assert e['tracking']['min_saturation']<=125


def test_shadowed_cargo_requires_bidirectional_rgb_match():
    from harness.dispatch_skill_binding import ImageRoute
    root=Path('tests/fixtures/dispatch_skill_transfer')
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    route.box_center=np.array([632.4217760904792,491.7954463713328]);route.box_delta=np.array([0.,-5.])
    route.box_previous=cv2.imdecode(np.frombuffer((root/'box-shadow-237.jpg').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    _,e=route.observe((root/'box-shadow-238.jpg').read_bytes())
    assert e['tracking']['consistent_features']>=3 and e['tracking']['max_cycle_error_px']<1
    assert np.allclose(e['cargo_center_px'],[632.,487.],atol=2)
    blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    with pytest.raises(RuntimeError):route.observe(blank)


def test_cargo_motion_separates_stationary_floor_corners_at_boundary():
    from harness.dispatch_skill_binding import ImageRoute
    root=Path('tests/fixtures/dispatch_skill_transfer')
    route=ImageRoute(SkillBindings(committed(),authored_map('open')),'box')
    route.box_center=np.array([844.0717475615736,352.2126817154773]);route.box_delta=np.array([6.,0.])
    route.box_previous=cv2.imdecode(np.frombuffer((root/'box-apron-303.jpg').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    _,e=route.observe((root/'box-apron-304.jpg').read_bytes())
    assert np.allclose(e['cargo_center_px'],[850.6,351.7],atol=1)
    assert e['tracking']['consistent_features']>=6


def test_release_projection_refinement_removes_contact_seed_bias():
    import base64
    from harness.markerless_box import observe_ground_box
    root=Path('tests/fixtures/dispatch_skill_transfer');old=[];new=[]
    for i,pan in ((426,1465),(427,1525)):
        raw=base64.b64encode((root/f'box-release-{i}.jpg').read_bytes()).decode()
        pose={1:2000,3:500,4:2472,5:1320,6:pan}
        old.append(observe_ground_box(raw,pose)['estimated_box_center_base_m'])
        fit=observe_ground_box(raw,pose,refine_position=True)
        assert fit['floor_hypothesis_projection_iou']>.97
        new.append(fit['estimated_box_center_base_m'])
    assert np.linalg.norm(np.array(old[0])-old[1])>.010
    assert np.linalg.norm(np.array(new[0])-new[1])<.001
