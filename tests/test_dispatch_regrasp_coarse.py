"""Dynamic regrasp re-approach anchors the coarse RGB target when the beam is still (v61)."""
from pathlib import Path
from types import SimpleNamespace

from harness.dispatch_skill_binding import PairCoarsePixels, beam_feature, still_truncated_beam

FIX = Path(__file__).parent / 'fixtures' / 'dispatch_regrasp'
REFERENCE = (Path(__file__).parent / 'fixtures' / 'camera_goal_transport' / 'reference-top.jpg').read_bytes()
FIRST = (FIX/'x1-first-approach-top.jpg').read_bytes()
GRASP_INIT = (FIX/'x1-grasp-init-top.jpg').read_bytes()
REGRASP = (FIX/'x1-regrasp-coarse-end-top.jpg').read_bytes()


def controller(r1_px, r3_px):
    claim = lambda px: {'claim': {'valid': True, 'center': [px[0]/960, px[1]/720]}}
    return PairCoarsePixels({'r1': claim(r1_px), 'r3': claim(r3_px)},
                            SimpleNamespace(pair={'r1': 'r1', 'r3': 'r3'}), REFERENCE)


def first_approach_anchor():
    first = controller((222.4, 356.9), (218.6, 166.7))
    assert all(first.decide(FIRST, slot)['ready'] for slot in ('r1', 'r3'))
    assert [round(v*s, 1) for v, s in zip(first.ready_beam24_center, (960, 720))] == [276.0, 263.5]
    return {'beam24_center': first.ready_beam24_center,
            'beam35_center': beam_feature(GRASP_INIT, hue_upper=35)['center']}


def test_truncated_still_beam_keeps_the_first_approach_coarse_target():
    anchor = first_approach_anchor()
    unanchored = controller((219.6, 370.3), (214.8, 180.2))  # recorded X1 coarse end, ~13 px south
    decisions = [unanchored.decide(REGRASP, slot) for slot in ('r1', 'r3')]
    assert all(d['ready'] and 'regrasp_beam' not in d for d in decisions)  # the recorded X1 failure
    regrasp = controller((219.6, 370.3), (214.8, 180.2))
    regrasp.regrasp_anchor = anchor
    for slot in ('r1', 'r3'):
        decision = regrasp.decide(REGRASP, slot)
        assert decision['regrasp_beam']['applied'] and decision['regrasp_beam']['hue24_truncated']
        assert decision['regrasp_beam']['hue35_moved_px'] <= .6
        assert not decision['ready'] and -.019 < decision['image_error'][1] < -.016
        assert decision['left'] > 0 and decision['forward'] == 0.
    assert regrasp.ready_beam24_center is None  # anchored frames never become a new anchor


def test_intact_shaft_ignores_the_anchor_and_recentering_never_records():
    anchor = first_approach_anchor()
    plain = controller((222.4, 356.9), (218.6, 166.7))
    anchored = controller((222.4, 356.9), (218.6, 166.7))
    anchored.regrasp_anchor = anchor
    for slot in ('r1', 'r3'):
        expected, decision = plain.decide(FIRST, slot), anchored.decide(FIRST, slot)
        assert not decision.pop('regrasp_beam')['applied']
        assert decision == expected
    recenter = controller((222.4, 356.9), (218.6, 166.7))
    assert recenter.decide(FIRST, 'r1', record_ready=False)['ready']
    assert recenter.ready_beam24_center is None


def test_still_truncated_beam_rejects_a_moved_beam():
    anchor = first_approach_anchor()
    applied, evidence = still_truncated_beam(REGRASP, anchor['beam35_center'])
    assert applied and round(evidence['hue24_length_px']) == 95
    moved = [anchor['beam35_center'][0] + 5/960, anchor['beam35_center'][1]]
    applied, evidence = still_truncated_beam(REGRASP, moved)
    assert not applied and evidence['hue24_truncated'] and evidence['hue35_moved_px'] > 4


def test_reset_for_regrasp_hands_the_coarse_anchor_to_the_rgb_controller():
    from scripts.dispatch_pair_skill import BoundPairSkill
    anchor = first_approach_anchor()
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.io = SimpleNamespace(realtime_control=False)
    pair.set_down = lambda: None
    pair.replay = lambda commands, stage: None
    pair.skill = {'initialization_replay': [{}]}
    pair.trace, pair.grasp_report = [], {}
    pair.grasp_translation = [212.0, 96.0]
    pair.grasp_beam35_center = anchor['beam35_center']
    pair.grasp_coarse_beam24 = anchor['beam24_center']
    pair.coarse = controller((219.6, 370.3), (214.8, 180.2))
    pair.reset_for_regrasp()
    assert pair.regrasp_anchor == {'translation_px': [212.0, 96.0], 'beam35_center': anchor['beam35_center']}
    assert pair.coarse.regrasp_anchor == anchor
    assert pair.phase == 'APPROACH' and not pair.transport_started
