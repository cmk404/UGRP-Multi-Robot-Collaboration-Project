import copy
import json
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from harness.pair_navigation import (PairVision, PairNavigator, validate_map, footprint_clear,
                                     swept_clear, plan_route, authorize_pair)
from harness.pair_carry_sync import PairCarrySync
from scripts.evaluate_pair_navigation import evaluate_samples
from scripts.audit_pair_carry_sync import _rgb

ROOT = Path(__file__).resolve().parents[1]
FIX = ROOT/'tests/fixtures/pair_navigation'


def data(name='open-left-90'):
    return json.loads((ROOT/'experiments/2026-09-14-pair-navigation/maps'/f'{name}.json').read_text())


def test_loaded_rectangle_rejects_rotation_whose_endpoints_are_clear():
    m = data()
    m['obstacles']=[{'id':'corner','center_m':[1.00,-2.38], 'half_extents_m':[.03,.03],'height_m':.3}]
    a,b=(.62,-2,0),(.62,-2,math.pi/2)
    assert footprint_clear(a,m) and footprint_clear(b,m)
    assert not swept_clear(a,b,m)


def test_wall_route_checks_both_chassis_payload_and_entire_swept_path():
    m=data('wall-detour');a,b=[.62,-2,0],[-.5,-2,0]
    assert not swept_clear(a,b,m)
    route=plan_route(a,b,m)
    assert route and len(route)>2
    assert all(swept_clear(x,y,m) for x,y in zip(route,route[1:]))
    assert max(abs(math.sin(p[2])) for p in route)>.9


def test_blocked_passage_is_refused_and_out_of_bounds_load_is_refused():
    m=data('blocked-passage')
    assert plan_route([.62,-2,0],[-.5,-2,0],m) is None
    assert plan_route([-1,-2,0],[.62,-2,0],data()) is None


@pytest.mark.parametrize('change', [
    lambda m:m.update(real_robot_xy=[0,0]),
    lambda m:m['top_camera'].update(fov_y_deg=70),
    lambda m:m['footprint'].update(half_lateral_m=.18),
    lambda m:m['goal'].update(center_m=[float('nan'),0]),
    lambda m:m['goal'].update(center_m=[100,0]),
])
def test_map_rejects_hidden_state_camera_changes_or_understated_load(change):
    m=data();change(m)
    with pytest.raises(ValueError):validate_map(m)


def test_actual_fixed_camera_recognizes_moved_chassis_and_partial_beam():
    vision=PairVision(data())
    obs=vision.observe((FIX/'start-own.jpg').read_bytes(),(FIX/'start-top.jpg').read_bytes())
    # Pixel wheel-envelope labels: no fixture base poses supplied to observer.
    assert np.allclose(obs['r1']['center_uv'], [488,437], atol=2)
    assert np.allclose(obs['r3']['center_uv'], [488,283], atol=2)
    assert vision.payload['feature_area_px'] >= 400


def test_issuing_probe_repeatedly_does_not_manufacture_observed_progress():
    actor=PairNavigator(data(),'r1')
    own,top=(FIX/'start-own.jpg').read_bytes(),(FIX/'start-top.jpg').read_bytes()
    for _ in range(21): decision=actor.decide(own,top)
    assert decision['status']=='probe_failed'
    assert decision['action']['forward']==0 and not decision['done']


def test_invalid_own_rgb_also_stops():
    decision=PairNavigator(data(),'r1').decide(b'not a jpeg',(FIX/'start-top.jpg').read_bytes())
    assert decision['status']=='vision_stop' and not decision['ready']


def test_payload_left_behind_stops_even_when_robot_pose_matches_plan():
    actor=PairNavigator(data(),'r1')
    actor.phase='track';actor.anchor_yaws={'r1':0.,'r3':0.}
    actor.anchor_payload={'relative_yaw_rad':0.};actor.vision.payload={'relative_yaw_rad':0.,'xy_m':[.62,-2]}
    obs={r:{'xy_m':[.62,y],'relative_yaw_rad':.2} for r,y in [('r1',-2.3),('r3',-1.7)]}
    with patch.object(actor.vision,'observe',return_value=obs):d=actor.decide(b'',b'')
    assert d['status']=='payload_decoupled'
    assert not d['ready'] and d['action']['turn']==0


def test_common_plan_mismatch_and_stale_reports_cannot_authorize_go():
    sync=PairCarrySync('case')
    reports={r:{'ready':True,'plan_hash':'same'} for r in ('r1','r3')}
    assert authorize_pair(sync,reports,{'r1':1,'r3':1},0)['phase']=='GO'
    reports['r3']['plan_hash']='different'
    assert authorize_pair(sync,reports,{'r1':2,'r3':2},1)['phase']=='HOLD'
    assert sync.authorize(1.0)['phase']=='HOLD'


def samples():
    template={'position_m':[.62,-2,.10],'height_above_start_m':.08,'yaw_rad':0.,'tilt_deg':0.,
              'constraints_active':{'r1':False,'r3':False}, 'payload_floor_contact':False,
              'within_authored_bounds':True,
              'contacts':{r:{'bilateral':True,'left':True,'right':True} for r in ('r1','r3')}}
    rows=[]
    for i in range(40):
        row=copy.deepcopy(template);row['sim_time_s']=i*.1
        row['phase']='grasp_hold' if i<20 else ('carry' if i<30 else 'release_hold')
        if i>=20:row['yaw_rad']=(i-20)/9*math.pi/2 if i<30 else math.pi/2
        if i>=30:
            row['payload_floor_contact']=True
            row['contacts']={r:{'bilateral':False,'left':False,'right':False} for r in ('r1','r3')}
        rows.append(row)
    return rows


def score(rows):
    return evaluate_samples(rows,data(),arrived=True,invariants_match=True,weld_ticks=0,wall_contact_ticks=0)


def test_full_success_requires_release_and_contact_in_every_carry_sample():
    rows=samples();assert score(rows)['success']
    rows[25]['contacts']['r3']['bilateral']=False
    assert not score(rows)['success']
    assert not score(rows)['gates']['bilateral_every_sample']


@pytest.mark.parametrize('mutation,gate',[
    (lambda rows:rows[29].update(yaw_rad=.5),'goal_orientation'),
    (lambda rows:rows[21].update(within_authored_bounds=False),'within_authored_bounds'),
    (lambda rows:rows[19]['contacts']['r1'].update(bilateral=False),'grasp_stable_before_departure'),
    (lambda rows:rows[-1]['contacts']['r1'].update(left=True),'released_on_floor'),
])
def test_referee_rejects_false_arrival_contact_or_release_claims(mutation,gate):
    rows=samples();mutation(rows)
    assert not score(rows)['gates'][gate]


def test_input_audit_rejects_extra_truth_fields_and_bad_hash():
    with pytest.raises(ValueError):_rgb(FIX,{'path':'start-top.jpg','sha256':'bad','qpos':[]})
    with pytest.raises(ValueError):_rgb(FIX,{'path':'start-top.jpg','sha256':'bad'})
