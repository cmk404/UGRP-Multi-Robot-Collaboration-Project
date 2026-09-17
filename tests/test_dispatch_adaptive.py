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


def test_input_audit_rejects_a_changed_frozen_grasp_translation():
    import copy
    from scripts.audit_dispatch_skill_inputs import audit_translation_history
    calls=[{'kind':'image_binding','transform':{'translation_px':[12.,-6.],'fixed_from_prior_rgb':False}},
           {'kind':'image_binding','transform':{'translation_px':[12.,-6.],'fixed_from_prior_rgb':True}},
           {'kind':'image_binding','transform':{'translation_px':[12.,-6.],'fixed_from_prior_rgb':True}}]
    assert audit_translation_history(calls)==2
    tampered=copy.deepcopy(calls);tampered[2]['transform']['translation_px'][0]+=1
    with pytest.raises(AssertionError,match='prior RGB anchor'):audit_translation_history(tampered)
    with pytest.raises(AssertionError,match='prior RGB anchor'):audit_translation_history(calls[1:])


@pytest.mark.parametrize('frame',[252,259])
def test_translation_centerline_does_not_turn_shaded_edges_into_skew(frame):
    import json
    from harness.dispatch_translation_skew import translation_skew
    root=Path('tests/fixtures/dispatch_adaptive')
    beam=json.loads((root/f'translation-skew-{frame}.json').read_text())
    upper,lower=sorted(beam['endpoints'],key=lambda p:p[1])
    assert abs((lower[0]-upper[0])*960)>2.8
    skew,evidence=translation_skew((root/f'translation-skew-{frame}.jpg').read_bytes(),beam)
    assert abs(skew)<1.5  # Below half the unchanged 3-pixel recovery trigger.
    assert evidence['supported_sections']>=40


def test_translation_centerline_keeps_actual_skew_and_rejects_missing_beam():
    from harness.dispatch_translation_skew import translation_skew
    image=np.zeros((720,960,3),np.uint8)
    corners=cv2.boxPoints(((480,360),(16,90),8.)).astype(np.int32)
    cv2.fillConvexPoly(image,corners,(55,155,255))
    raw=cv2.imencode('.jpg',image)[1].tobytes()
    beam=beam_feature(raw,hue_upper=35)
    skew,_=translation_skew(raw,beam)
    assert abs(skew)>10
    blank=cv2.imencode('.jpg',np.zeros((720,960,3),np.uint8))[1].tobytes()
    with pytest.raises(ValueError,match='lacks current RGB support'):
        translation_skew(blank,beam)


def test_translation_centerline_excludes_adjacent_yellow_apron():
    import json
    from harness.dispatch_translation_skew import translation_skew
    root=Path('tests/fixtures/dispatch_adaptive')
    beam=json.loads((root/'translation-apron-contamination.json').read_text())
    skew,evidence=translation_skew((root/'translation-apron-contamination.jpg').read_bytes(),beam)
    assert abs(skew)<1.6
    assert evidence['min_saturation']==150


def test_exact_vertical_half_pixel_shaft_keeps_its_visible_row_coverage():
    import json
    from harness.dispatch_translation_skew import translation_skew
    root=Path('tests/fixtures/dispatch_adaptive')
    beam=json.loads((root/'translation-vertical-half-pixel.json').read_text())
    skew,evidence=translation_skew((root/'translation-vertical-half-pixel.jpg').read_bytes(),beam)
    assert evidence['supported_sections']>=50
    assert abs(skew)<1.5


def test_bright_cyan_probe_keeps_full_cargo_silhouette_without_floor_merge():
    import base64
    from harness.visual_attachment import compare_box_comotion
    root=Path('tests/fixtures/dispatch_adaptive')
    raw=[(root/f'bright-box-{n}.jpg').read_bytes() for n in [207,208,209,210]]
    frames=[base64.b64encode(b).decode() for b in raw]
    for a,b,pan in [(0,1,60),(0,2,-60),(1,2,-120),(2,3,60),(0,3,0)]:
        result=compare_box_comotion(frames[a],frames[b],camera_pan_delta_pwm=pan,min_saturation=150)
        assert result['attached']
        assert 140000<result['before_area_px']<180000
        assert 140000<result['after_area_px']<180000
        assert result['thresholds']['area_ratio_range']==[.9,1.1]
        assert result['thresholds']['min_iou']==.88
    # A broad saturation relaxation merges nearly the entire blue floor.
    broad=compare_box_comotion(frames[1],frames[2],camera_pan_delta_pwm=-120)
    assert broad['before_area_px']>240000
    image=cv2.imdecode(np.frombuffer(raw[1],np.uint8),cv2.IMREAD_COLOR)
    image[165:]=0
    floor_only=base64.b64encode(cv2.imencode('.jpg',image)[1]).decode()
    assert not compare_box_comotion(floor_only,floor_only,min_saturation=150)['attached']


def test_box_identity_does_not_switch_to_a_nearby_painted_floor_fragment():
    import json
    from harness.dispatch_skill_binding import ImageRoute,SkillBindings
    from sim.research_dispatch_arena import authored_map
    root=Path('tests/fixtures/dispatch_adaptive')
    prior=json.loads((root/'box-floor-identity-prior.json').read_text())
    route=ImageRoute(SkillBindings(prior['committed'],authored_map('narrow_south')),'box')
    route.box_center=np.array(prior['previous_center_px'])
    route.box_delta=np.array(prior['previous_delta_px'])
    route.box_previous=cv2.imread(str(root/'box-floor-identity-199.jpg'))
    _,evidence=route.observe((root/'box-floor-identity-200.jpg').read_bytes())
    assert np.linalg.norm(np.array(evidence['cargo_center_px'])-[268.25,204.66])<1
    assert np.linalg.norm(np.array(evidence['cargo_center_px'])-[256.30,218.37])>15
