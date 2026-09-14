"""Recorded RGB geometry, own-view loss, and bounded motion recovery."""
import copy
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.pair_navigation import PairNavigator, ROBOTS, VisionUncertain, authorize_pair, retryable_visual_hold
from harness.pair_transport_vision import GeometryPairVision, OwnCarryMonitor, GripUncertain
from harness.pair_carry_sync import PairCarrySync

FIX=Path(__file__).parent/'fixtures/pair_transport_robustness'


def read(case,name):
    raw=(FIX/case/name).read_bytes()
    manifest=json.loads((FIX/'manifest.json').read_text())
    assert hashlib.sha256(raw).hexdigest()==manifest['files'][f'{case}/{name}']
    return raw


def observer(case):
    v=GeometryPairVision(json.loads(read(case,'map.json')))
    v.observe(read(case,'start-own.jpg'),read(case,'start-top.jpg'))
    state=json.loads(read(case,'state.json'))
    for key,value in state.items():setattr(v,key,value)
    return v


@pytest.mark.parametrize('case',['narrow-door','l-corner','s-bends','staggered-obstacles','blocked-branch'])
def test_recorded_partial_views_have_pixel_consistent_wheel_directions(case):
    v=observer(case)
    obs=v.observe(read(case,'target-own.jpg'),read(case,'target-top.jpg'))
    label=json.loads((FIX/'manifest.json').read_text())['cases'][case]
    for o in obs.values():
        assert abs(math.degrees(o['relative_yaw_rad'])-label['manual_wheel_axis_deg']) < label['angle_tolerance_deg']
        assert len(o['wheel_landmarks_uv'])==4
    assert v.carry_monitor.last['continuation_supported']


@pytest.mark.parametrize('case',['s-bends','staggered-obstacles'])
@pytest.mark.parametrize('rid',ROBOTS)
def test_own_camera_rejects_actual_drop_even_with_unchanged_top(case,rid):
    actor=PairNavigator(json.loads(read(case,'map.json')),rid,vision_mode='robust')
    actor.decide(read(case,'start-own.jpg'),read(case,'start-top.jpg'))
    d=actor.decide(read(case,f'dropped-{rid}-own.jpg'),read(case,'start-top.jpg'))
    assert d['status']=='grip_stop' and not d['ready'] and not d['done']
    assert all(d['action'][k]==0 for k in ('forward','left','turn'))
    assert actor.decide(read(case,'start-own.jpg'),read(case,'start-top.jpg'))['status']=='grip_stop'


def test_robot_positions_alone_cannot_supply_a_missing_beam():
    case='s-bends';v=observer(case)
    image=cv2.imdecode(np.frombuffer(read(case,'target-top.jpg'),np.uint8),1)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    image[cv2.inRange(hsv,(8,100,70),(19,255,255))>0]=0
    ok,jpeg=cv2.imencode('.jpg',image);assert ok
    before=copy.deepcopy(v.centers)
    with pytest.raises(VisionUncertain):v.observe(read(case,'target-own.jpg'),jpeg.tobytes())
    assert v.centers==before


def test_stale_replanned_version_never_authorizes_motion():
    sync=PairCarrySync('versions')
    sync.update_plan(2,0.)
    reports={r:{'ready':True,'plan_hash':'same-old-route','plan_version':1} for r in ROBOTS}
    assert authorize_pair(sync,reports,{r:1 for r in ROBOTS},1)['phase']!='GO'


def test_joint_replan_clears_previous_permission_and_can_resume():
    sync=PairCarrySync('replan')
    reports={r:{'ready':False,'status':'replan_hold','retryable':True,
                'plan_hash':'new-route','plan_version':2} for r in ROBOTS}
    assert retryable_visual_hold(reports)
    assert authorize_pair(sync,reports,{r:1 for r in ROBOTS},1)['phase']=='HOLD'
    assert sync.plan_version==2
    for d in reports.values():d.update(ready=True,status='track')
    assert authorize_pair(sync,reports,{r:2 for r in ROBOTS},2)['phase']=='GO'


def test_no_observed_progress_replans_then_stops_without_relaxing_arrival(monkeypatch):
    case='s-bends';actor=PairNavigator(json.loads(read(case,'map.json')),'r1',vision_mode='robust')
    own,top=read(case,'start-own.jpg'),read(case,'start-top.jpg')
    obs=actor.vision.observe(own,top)
    center=np.mean([obs[r]['xy_m'] for r in ROBOTS],axis=0)
    actor.phase='probe_settle';actor.confirmations=2;actor.heading=0.
    actor.decide(own,top)
    assert actor.phase=='track'
    monkeypatch.setattr(actor.vision,'observe',lambda a,b:copy.deepcopy(obs))
    replans=0
    for _ in range(250):
        decision=actor.decide(own,top)
        replans+=decision['status']=='replan_hold'
        assert not decision['done']
        if decision['status']=='motion_stalled':break
    assert replans==2 and decision['status']=='motion_stalled'
    assert all(decision['action'][k]==0 for k in ('forward','left','turn'))
