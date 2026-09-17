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
        data=navigation_map(bindings(route=route),raw)
        result=plan_route([-.18,-1.65,0],[*data['goal']['center_m'],0],data)
        assert bool(result)==possible

def test_pickup_resource_prevents_solo_from_waiting_in_pair_turn_space():
    b=bindings();assert b.permission('beam','APPROACH')
    assert not b.permission('box','APPROACH')
    b.finish('beam');assert b.permission('box','APPROACH')
