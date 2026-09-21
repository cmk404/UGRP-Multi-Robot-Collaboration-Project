from pathlib import Path
import cv2
import numpy as np
import pytest
from harness.dispatch_box_identity import PAN_PHASES, bind_attachment_pan
from harness.dispatch_skill_binding import ImageRoute, SkillBindings
from sim.research_dispatch_arena import authored_map
from tests.test_dispatch_skill_binding import committed


def test_recovered_failure_binds_cargo_among_five_same_colour_components():
    root = Path('tests/fixtures/dispatch_attachment_pan')
    frames = {phase: (root/(phase+'.jpg')).read_bytes() for phase in PAN_PHASES}
    center, evidence = bind_attachment_pan(frames)
    assert np.linalg.norm(center-[274.04,545.51]) < .02
    assert len(evidence['frames_sha256']) == 4
    route = ImageRoute(SkillBindings(committed(), authored_map('shared_crossing')), 'box')
    with pytest.raises(RuntimeError, match='unresolved or ambiguous'):
        route.observe(frames['attachment_home'])
    for phase in PAN_PHASES: route.record_attachment_top(phase,frames[phase])
    action, result = route.observe(frames['attachment_home'])
    assert np.allclose(result['cargo_center_px'], center)
    assert result['tracking']['initial_identity']['own_attachment_independently_required']
    assert action['kind'] == 'mecanum'


def sequence(positions, duplicate=False):
    frames = {}
    for phase,y in zip(PAN_PHASES, positions):
        image = np.zeros((160,240,3),np.uint8)
        # HSV hue 90; include a stationary distractor in every view.
        for x,cy in [(30,30),(100,y)]+([(180,y)] if duplicate else []):
            cv2.rectangle(image,(x,cy),(x+8,cy+8),(255,255,0),-1)
        frames[phase] = cv2.imencode('.png',image)[1].tobytes()
    return frames


@pytest.mark.parametrize('positions,duplicate', [([70,70,70,70],False),
    ([70,74,78,82],False), ([70,66,74,70],True)])
def test_static_one_way_and_ambiguous_motion_fail_closed(positions,duplicate):
    with pytest.raises(RuntimeError, match='unresolved or ambiguous'):
        bind_attachment_pan(sequence(positions,duplicate))


def test_complete_reversible_motion_required():
    frames = sequence([70,66,74,70])
    center,_ = bind_attachment_pan(frames)
    assert np.allclose(center,[104,74])
    del frames['verify_lift']
    with pytest.raises(RuntimeError,match='complete'):
        bind_attachment_pan(frames)
