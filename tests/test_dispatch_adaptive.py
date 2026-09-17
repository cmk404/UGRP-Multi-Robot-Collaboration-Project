from pathlib import Path
import cv2
import numpy as np
import pytest
from harness.dispatch_skill_binding import beam_feature
from harness.camera_beam_features import extract_beams


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
