"""Dynamic regrasp re-approach keeps the failed grasp's RGB translation when the beam is still."""
from pathlib import Path

from harness.dispatch_skill_binding import beam_feature, canonical_pair_top
from scripts.dispatch_pair_skill import regrasp_binding

FIX = Path(__file__).parent / 'fixtures' / 'dispatch_regrasp'
REFERENCE = Path(__file__).parent / 'fixtures' / 'camera_goal_transport' / 'reference-top.jpg'


def _anchor():
    before = (FIX/'w2-before-grasp-top.jpg').read_bytes()
    translation = canonical_pair_top(before, REFERENCE.read_bytes())[1]['translation_px']
    return {'translation_px': translation, 'beam35_center': beam_feature(before, hue_upper=35)['center']}


def test_truncated_hue24_shaft_with_a_still_beam_keeps_the_grasp_translation():
    anchor = _anchor()
    assert anchor['translation_px'] == [212.0, 96.0]
    raw = (FIX/'w2-regrasp-lateral-top.jpg').read_bytes()
    assert canonical_pair_top(raw, REFERENCE.read_bytes())[1]['translation_px'] == [212.0, 82.0]  # the recorded failure
    translation, evidence = regrasp_binding(raw, anchor)
    assert translation == [212.0, 96.0] and evidence['applied']
    assert round(evidence['hue24_length_px']) == 95 and evidence['hue35_moved_px'] <= .6


def test_a_full_hue24_shaft_or_a_moved_beam_keeps_the_per_frame_binding():
    anchor = _anchor()
    translation, evidence = regrasp_binding((FIX/'w2-before-grasp-top.jpg').read_bytes(), anchor)
    assert translation is None and not evidence['hue24_truncated'] and not evidence['applied']
    moved = {**anchor, 'beam35_center': [anchor['beam35_center'][0] + 5/960, anchor['beam35_center'][1]]}
    translation, evidence = regrasp_binding((FIX/'w2-regrasp-lateral-top.jpg').read_bytes(), moved)
    assert translation is None and evidence['hue24_truncated'] and evidence['hue35_moved_px'] > 4
