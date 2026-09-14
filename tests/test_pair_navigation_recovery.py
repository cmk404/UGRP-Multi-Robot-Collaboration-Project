"""Real failure frames plus no-motion/atomicity/timeout recovery regressions."""
import copy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.pair_navigation import (PairNavigator, TemporalPairVision, VisionUncertain,
                                    ROBOTS, authorize_pair, retryable_visual_hold)
from harness.pair_carry_sync import PairCarrySync

ROOT=Path(__file__).resolve().parents[1]
FIX=ROOT/'tests/fixtures/pair_navigation'
RECOVERY=ROOT/'tests/fixtures/pair_navigation_recovery'


def inputs():
    return (FIX/'start-own.jpg').read_bytes(),(FIX/'start-top.jpg').read_bytes()


def data():
    return json.loads((ROOT/'experiments/2026-09-14-pair-navigation/maps/open-left-90.json').read_text())


def blank():
    ok,encoded=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))
    assert ok
    return encoded.tobytes()


def test_temporary_occlusion_stops_both_until_three_good_observations():
    own,top=inputs()
    actors={r:PairNavigator(data(),r,vision_mode='temporal') for r in ROBOTS}
    sync=PairCarrySync('visual-recovery')
    def step(image,index):
        reports={r:a.decide(own,image) for r,a in actors.items()}
        permission=authorize_pair(sync,reports,{r:index for r in ROBOTS},index)
        return reports,permission
    reports,permission=step(top,0)
    assert permission['phase']=='GO'
    steps={r:a.probe_steps for r,a in actors.items()}
    for i,image in enumerate((blank(),top,top),1):
        reports,permission=step(image,i)
        assert permission['phase']=='HOLD' and retryable_visual_hold(reports)
        assert all(not d['ready'] and not d['done'] for d in reports.values())
        assert all(all(d['action'][k]==0 for k in ('forward','left','turn')) for d in reports.values())
        assert {r:a.probe_steps for r,a in actors.items()}==steps
    reports,permission=step(top,4)
    assert permission['phase']=='GO' and all(d['ready'] for d in reports.values())


def test_persistent_occlusion_times_out_and_never_resumes():
    own,top=inputs()
    actor=PairNavigator(data(),'r1',vision_mode='temporal')
    actor.decide(own,top)
    for _ in range(24):
        decision=actor.decide(own,blank())
        assert decision['status']=='vision_hold'
    decision=actor.decide(own,blank())
    assert decision['status']=='vision_stop' and not decision['retryable']
    assert actor.decide(own,top)['status']=='vision_stop'


def test_one_invalid_camera_is_terminal_for_the_pair():
    own,top=inputs()
    actors={r:PairNavigator(data(),r,vision_mode='temporal') for r in ROBOTS}
    reports={'r1':actors['r1'].decide(b'bad JPEG',top),'r3':actors['r3'].decide(own,top)}
    assert reports['r1']['status']=='vision_stop'
    assert not retryable_visual_hold(reports)
    assert authorize_pair(PairCarrySync('bad-own'),reports,{r:1 for r in ROBOTS},0)['phase']!='GO'


def test_second_robot_failure_does_not_commit_first_robot_or_payload(monkeypatch):
    own,top=inputs()
    vision=TemporalPairVision(data());vision.observe(own,top)
    before=copy.deepcopy(vision.__dict__)
    original=vision._match
    calls=0
    def fail_second(*args):
        nonlocal calls
        calls+=1
        if calls==2: raise VisionUncertain('second robot occluded')
        return original(*args)
    monkeypatch.setattr(vision,'_match',fail_second)
    with pytest.raises(VisionUncertain):vision.observe(own,top)
    for key in ('centers','angles','payload','payload_angle','payload_origin_angle','appearance'):
        assert vision.__dict__[key]==before[key]


def test_failed_initialization_leaves_no_partial_robot_tracks():
    own,top=inputs()
    frame=cv2.imdecode(np.frombuffer(top,np.uint8),cv2.IMREAD_COLOR)
    # A synthetic missing-upper-robot negative, not a modified actor camera.
    frame[200:310,420:550]=0
    ok,encoded=cv2.imencode('.jpg',frame);assert ok
    vision=TemporalPairVision(data())
    with pytest.raises(VisionUncertain):vision.observe(own,encoded.tobytes())
    assert not vision.templates and not vision.centers and vision.payload is None


def test_orange_blob_cannot_replace_the_tracked_beam():
    own,top=inputs()
    vision=TemporalPairVision(data());vision.observe(own,top)
    image=np.zeros((720,960,3),np.uint8)
    color=cv2.cvtColor(np.uint8([[[13,220,220]]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
    cv2.rectangle(image,(470,345),(500,375),color,-1)
    ok,encoded=cv2.imencode('.jpg',image);assert ok
    with pytest.raises(VisionUncertain):vision.observe(own,encoded.tobytes())


@pytest.mark.parametrize('case',['l-corner','s-bends','narrow-door','blocked-branch'])
def test_recorded_failure_is_tracked_without_identity_jump(case):
    root=RECOVERY/case
    manifest=json.loads((root/'manifest.json').read_text())
    def read(name):
        raw=(root/name).read_bytes()
        assert hashlib.sha256(raw).hexdigest()==manifest['files'][name]
        return raw
    state=json.loads(read('state.json'))
    vision=TemporalPairVision(json.loads((ROOT/manifest['map']).read_text()))
    for key in ('centers','angles','payload','payload_angle','payload_origin_angle'):
        setattr(vision,key,state[key])
    for rid in ROBOTS:
        vision.templates[rid]=cv2.imdecode(np.frombuffer(read(f'{rid}-initial.png'),np.uint8),0)
        vision.appearance[rid]=[{**entry,'image':cv2.imdecode(np.frombuffer(read(entry['image']),np.uint8),0)}
                                for entry in state['appearance'][rid]]
    for row in manifest['frames']:
        obs=vision.observe(read(row['own']),read(row['top']))
        # Independent wheel-envelope labels from visible image pixels, not
        # runtime simulator coordinates or candidate-generated target values.
        for rid in ROBOTS:
            assert np.linalg.norm(np.array(obs[rid]['center_uv'])-row['wheel_labels_uv'][rid]) < 8
    assert all(np.isfinite(o['relative_yaw_rad']) for o in obs.values())
