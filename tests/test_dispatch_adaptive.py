from pathlib import Path
import cv2
import numpy as np
import pytest
from harness.dispatch_skill_binding import beam_feature
from harness.camera_beam_features import extract_beams
from harness.dispatch_skill_binding import BeamContinuity


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
