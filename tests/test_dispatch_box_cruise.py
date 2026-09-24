"""Higher far-route authority preserves waypoint braking and resource stops."""
from pathlib import Path
import numpy as np
import pytest
from harness.dispatch_skill_binding import ImageRoute, SkillBindings
from sim.research_dispatch_arena import authored_map
from tests.test_dispatch_skill_binding import committed

RAW=Path('tests/fixtures/dispatch_skill_transfer/box-top-held.jpg').read_bytes()


def route_at(error, *, overlap=True):
    bindings=SkillBindings(committed(),authored_map('open'),route_overlap=overlap)
    route=ImageRoute(bindings,'box')
    _,evidence=route.observe(RAW)
    route.index=2
    route.points[2]=np.array(evidence['cargo_center_px'])+error
    return bindings,route


def test_far_box_cruise_respects_existing_port_limits():
    _,route=route_at(np.array([100.,100.]))
    action,evidence=route.observe(RAW)
    assert action['forward']==pytest.approx(.12)
    assert action['left']==pytest.approx(-.10)
    assert action['turn']==0 and action['duration_s']==.2
    assert evidence['cruise_command_limits']['reverse']==.05
    _,route=route_at(np.array([-100.,-100.]))
    action,_=route.observe(RAW)
    assert action['forward']==pytest.approx(-.05)
    assert action['left']==pytest.approx(.10)


def test_near_waypoint_brakes_with_original_gain_and_confirmations():
    _,route=route_at(np.array([20.,-18.]))
    action,_=route.observe(RAW)
    assert action['forward']==pytest.approx(.04,abs=.0005)
    assert action['left']==pytest.approx(.036,abs=.0005)
    _,route=route_at(np.array([4.,5.]))
    action,evidence=route.observe(RAW)
    assert evidence['ready'] and not evidence['done']
    assert action['forward']==action['left']==0
    assert route.index==2
    route.observe(RAW)
    assert route.index==3


def test_serial_route_keeps_original_cap():
    _,route=route_at(np.array([100.,100.]),overlap=False)
    action,evidence=route.observe(RAW)
    assert action['forward']==pytest.approx(.08)
    assert action['left']==pytest.approx(-.08)
    assert 'cruise_command_limits' not in evidence


def test_fast_route_still_waits_before_shared_apron():
    bindings,route=route_at(np.array([100.,100.]))
    route.index=1
    route.points[1]=route.box_center.copy()
    for _ in range(3):
        action,evidence=route.observe(RAW)
        assert evidence['waiting_for_resource'] and route.index==1
        assert action['forward']==action['left']==0
    bindings.finish('beam')
    route.observe(RAW);route.observe(RAW)
    assert route.index==2 and bindings.locks['dispatch_apron']=='box_job'


@pytest.mark.parametrize('extra', ['obstacle','terrain'])
def test_edited_open_map_does_not_inherit_fast_cruise(extra):
    _,route=route_at(np.array([100.,100.]))
    if extra=='obstacle':route.map['obstacles'].append({'id':'new_interior_obstacle'})
    else:route.map['terrain']=[{'id':'new_terrain'}]
    action,evidence=route.observe(RAW)
    assert action['forward']==pytest.approx(.08)
    assert action['left']==pytest.approx(-.08)
    assert 'cruise_command_limits' not in evidence
