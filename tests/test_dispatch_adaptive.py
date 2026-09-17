from pathlib import Path
import cv2
import numpy as np
import pytest
from harness.dispatch_skill_binding import beam_feature
from harness.camera_beam_features import extract_beams
from harness.dispatch_skill_binding import BeamContinuity


def test_rotating_own_view_uses_recent_continuity_and_still_rejects_loss():
    import math
    from harness.dispatch_own_hold import OwnHoldContinuity
    from harness.camera_goal_transport import own_payload
    root=Path('tests/fixtures/dispatch_adaptive')
    anchor=(root/'own-hold-anchor.jpg').read_bytes()
    previous=(root/'own-hold-prior.jpg').read_bytes()
    current=(root/'own-hold-failure.jpg').read_bytes()
    guard=OwnHoldContinuity(anchor)
    # State fixture comes from the immediately previous actual own RGB.
    guard.previous=own_payload(previous,hue_upper=35)
    evidence=guard.observe(current)
    assert math.dist(evidence['current'][1:],evidence['anchor'][1:])>.15
    assert evidence['held_estimate'] and evidence['temporal_motion_norm']<.01
    black=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    assert not guard.observe(black)['held_estimate']
    assert not OwnHoldContinuity(anchor).observe(current)['held_estimate']


def test_actual_destination_b_failure_separates_beam_from_floor_paint():
    raw=Path('tests/fixtures/dispatch_adaptive/beam-floor-303.jpg').read_bytes()
    old=[b for b in extract_beams(raw,hue_upper=35)
         if 65<=b['length_px']<=180 and b['width_px']<=25]
    assert not old
    before=beam_feature(Path('tests/fixtures/dispatch_adaptive/beam-floor-302.jpg').read_bytes(),hue_upper=35)
    after=beam_feature(raw,hue_upper=35)
    assert np.linalg.norm((np.array(after['center'])-before['center'])*[960,720])<7
    assert 85<after['length_px']<110 and after['width_px']<25
    # Floor alone cannot become an accepted beam when the cargo disappears.
    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    image[175:289,616:650]=0
    with pytest.raises(ValueError,match='unresolved'):
        beam_feature(cv2.imencode('.jpg',image)[1].tobytes(),hue_upper=35)


def test_shaft_survives_brightness_change_over_painted_apron():
    raw=Path('tests/fixtures/dispatch_adaptive/beam-floor-r1-325.jpg').read_bytes()
    beam=beam_feature(raw,hue_upper=35)
    assert np.allclose(np.array(beam['center'])*[960,720],[633.5,345.5],atol=3)
    assert 85<beam['length_px']<110 and beam['width_px']<25
    tracker=BeamContinuity();tracker.observe(beam)
    moved={**beam,'center':[beam['center'][0]+.1,beam['center'][1]]}
    with pytest.raises(ValueError,match='continuity'):tracker.observe(moved)


def test_occluded_end_keeps_the_same_shaft_across_thresholds():
    b=beam_feature(Path('tests/fixtures/dispatch_adaptive/beam-floor-r2-311.jpg').read_bytes(),hue_upper=35)
    assert np.allclose(np.array(b['center'])*[960,720],[633.5,276],atol=3)
    assert 90<b['length_px']<105


def test_attached_gripper_pixels_are_not_treated_as_a_longer_beam():
    raw=Path('tests/fixtures/dispatch_adaptive/beam-gripper-bridge.jpg').read_bytes()
    original=extract_beams(raw,hue_upper=35)
    assert any(140<b['length_px']<155 and b['width_px']<25 for b in original)
    actual=beam_feature(raw,hue_upper=35)
    assert 75<actual['length_px']<110
    assert np.allclose(np.array(actual['center'])*[960,720],[689,174],atol=4)


@pytest.mark.parametrize('run',['r8','r9'])
def test_temporal_shaft_tracks_actual_failed_frames_and_rejects_missing_cargo(run):
    import json
    from harness.dispatch_beam_tracker import CarriedBeamTracker
    root=Path('tests/fixtures/dispatch_adaptive')
    tracker=CarriedBeamTracker()
    tracker.previous=json.loads((root/f'temporal-{run}-prior.json').read_text())['previous']
    raw=(root/f'temporal-{run}-failure.jpg').read_bytes()
    feature=tracker.observe(raw)
    assert feature['tracking']['threshold_support']>=3
    assert feature['tracking']['uses_issued_motion'] is False
    blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    with pytest.raises(ValueError,match='consistent RGB support'):tracker.observe(blank)
