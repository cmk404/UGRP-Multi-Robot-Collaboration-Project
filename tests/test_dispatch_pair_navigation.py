import copy,math
import pytest
from pathlib import Path
import numpy as np
from harness.dispatch_skill_binding import SkillBindings
from harness.dispatch_navigation_map import navigation_map,visual_barriers
from harness.dispatch_pair_navigation import PairVision,plan_route,swept_clear,payload_coupled
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
        result=plan_route([-.18,-1.65,0],[*data['goal']['center_m'],0],data)
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
    assert plan_route([-.18,-1.65,0],[*future['goal']['center_m'],0],future)


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
