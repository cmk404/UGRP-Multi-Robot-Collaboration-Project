"""Real spawned visual analysis, ordered ownership, and saved-RGB parity."""
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from harness.dispatch_pair_perception import (
    PairPerceptionProcess, PairPerceptionState, detached_beam_route)
from harness.dispatch_beam_tracker import CarriedBeamTracker
from harness.dispatch_skill_binding import (
    BeamContinuity, ImageRoute, SkillBindings, canonical_pair_top)
from harness.dispatch_plan import validate_dispatch_plan
from harness.three_robot_plan import digest
from sim.research_dispatch_arena import authored_map
from scripts.dispatch_pair_skill import BoundPairSkill


ROOT = Path(__file__).resolve().parents[1]
SAVED = Path('/Users/changmin/projects/ugrp/outputs/simulation-realtime-20260923/native-v17-repeat')


def _initialization(reference, anchor_top, anchor_own, bindings):
    carried = CarriedBeamTracker()
    continuity = BeamContinuity()
    _, transform = canonical_pair_top(anchor_top, reference, hue_upper=35,
                                      observed_beam=carried.observe(anchor_top))
    continuity.observe(transform['observed_beam'])
    return {'reference': reference, 'anchor_own': anchor_own,
            'carried_previous': carried.previous,
            'continuity_previous': continuity.previous,
            'route_state': detached_beam_route(ImageRoute(bindings, 'beam'))}


def _portable_input():
    fixture = ROOT / 'tests/fixtures/pair_grasp_spacing'
    top = (fixture / 'anchor-top.jpg').read_bytes()
    own = {slot: (fixture / f'anchor-{slot}-own.jpg').read_bytes()
           for slot in ('r1', 'r3')}
    plan = validate_dispatch_plan({'dock': 'dock_a', 'tasks': [
        {'id': 'beam_job', 'object': 'beam', 'participants': ['r1', 'r3'],
         'route': 'north', 'after': []},
        {'id': 'box_job', 'object': 'box', 'participants': ['r2'],
         'route': 'south', 'after': []}]})
    bindings = SkillBindings({'plan': plan, 'plan_hash': digest(plan)},
                             authored_map('open'))
    return _initialization(top, top, own, bindings), top, own


def _await(process, pending):
    deadline = time.monotonic() + 10
    while not pending.done():
        assert time.monotonic() < deadline
        time.sleep(.002)
    return pending.result()


def _same_analysis(expected, actual):
    for key in ('canonical_top', 'transform', 'carried_previous',
                'continuity_previous', 'motion', 'route', 'decisions',
                'skew', 'skew_evidence', 'frame_id', 'observed_at_s'):
        assert actual[key] == expected[key], key


def test_spawned_process_preserves_sequential_rgb_state_and_closes():
    initialization, top, own = _portable_input()
    inline = PairPerceptionState(**initialization)
    process = PairPerceptionProcess(initialization)
    try:
        process.wait_ready(lambda: time.sleep(.002))
        for frame_id in (10, 11):
            expected = inline.analyze(frame_id, frame_id / 10, top, own)
            pending = process.submit(frame_id, frame_id / 10, top, own)
            with pytest.raises(RuntimeError, match='one ordered RGB batch'):
                process.submit(frame_id + 1, frame_id / 10 + .1, top, own)
            _same_analysis(expected, _await(process, pending))
        with pytest.raises(RuntimeError, match='frames must be accepted in order'):
            _await(process, process.submit(11, 1.2, top, own))
    finally:
        process.close()
    assert not process.process.is_alive()


def test_spawned_worker_error_is_explicit_and_cleanup_bounded():
    initialization, _, own = _portable_input()
    process = PairPerceptionProcess(initialization)
    try:
        process.wait_ready(lambda: time.sleep(.002))
        pending = process.submit(1, 0., b'not a jpeg', own)
        with pytest.raises(RuntimeError, match='pair perception'):
            _await(process, pending)
    finally:
        process.close()
    assert not process.process.is_alive()


def test_owner_adopts_only_matching_rgb_and_retains_physical_slot_mapping(tmp_path):
    initialization, top, own = _portable_input()
    result = PairPerceptionState(**initialization).analyze(21, 1.25, top, own)
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.out = tmp_path
    pair.bindings = SimpleNamespace(pair={'r1': 'r3', 'r3': 'r1'})
    pair.carried_beam = CarriedBeamTracker()
    pair.beam_continuity = BeamContinuity()
    pair.calls = []
    pair.last_capture = None
    raw = {rid: {'frame_id': 21, 'observed_at_s': 1.25,
                 'top_bytes': top, 'shared_top_rgb': {'path': 'top.jpg'},
                 'own_bytes': own[slot], 'own_rgb': {'path': f'{rid}.jpg'}}
           for slot, rid in pair.bindings.pair.items()}
    wrong = dict(result, frame_id=20)
    with pytest.raises(RuntimeError, match='provenance mismatch'):
        pair._adopt_carry_analysis(raw, 1, wrong)
    assert pair.last_capture is None
    assert not (tmp_path / 'rgb').exists()
    mapped, motion, route, decisions, skew, skew_evidence, _ = (
        pair._adopt_carry_analysis(raw, 1, result))
    assert mapped['r1']['physical_robot_id'] == 'r3'
    assert mapped['r3']['physical_robot_id'] == 'r1'
    assert mapped['r1']['raw_top_bytes'] == top
    assert mapped['r1']['top_bytes'] == result['canonical_top']
    assert pair.carried_beam.previous == result['carried_previous']
    assert pair.beam_continuity.previous == result['continuity_previous']
    assert pair.calls[0]['derived_top']['sha256'] == hashlib.sha256(result['canonical_top']).hexdigest()
    assert (motion, route, decisions, skew, skew_evidence) == (
        result['motion'], result['route'], result['decisions'],
        result['skew'], result['skew_evidence'])


@pytest.mark.skipif(not (SAVED / 'pair-decisions.json').exists(),
                    reason='local preserved v17 RGB replay is unavailable')
def test_saved_v17_entire_carry_matches_original_record_and_spawned_process():
    rows = json.loads((SAVED / 'pair-decisions.json').read_text())
    anchor = next(row for row in rows if row.get('kind') == 'image_binding'
                  and row.get('transform', {}).get('tracked_carried_shaft'))
    committed = json.loads((SAVED / 'committed-plan.json').read_text())
    static_map = json.loads((SAVED / 'episode-setup-only.json').read_text())['static_map']
    bindings = SkillBindings(committed, static_map, route_overlap=False)
    reference = (ROOT / 'tests/fixtures/camera_goal_transport/reference-top.jpg').read_bytes()
    initialization = _initialization(reference,
        (SAVED / anchor['raw_top']['path']).read_bytes(),
        {slot: (SAVED / anchor['own'][slot]['path']).read_bytes()
         for slot in ('r1', 'r3')}, bindings)
    inline = PairPerceptionState(**initialization)
    process = PairPerceptionProcess(initialization)
    checked = 0
    try:
        process.wait_ready(lambda: time.sleep(.002))
        for index, row in enumerate(rows):
            if (row.get('kind') != 'image_binding' or row is anchor
                    or not row.get('transform', {}).get('tracked_carried_shaft')):
                continue
            top = (SAVED / row['raw_top']['path']).read_bytes()
            own = {slot: (SAVED / row['own'][slot]['path']).read_bytes()
                   for slot in ('r1', 'r3')}
            frame_id = row['frame_id']
            observed = float(index)  # saved sequence; only ordering is consumed
            expected = inline.analyze(frame_id, observed, top, own)
            actual = _await(process, process.submit(frame_id, observed, top, own))
            _same_analysis(expected, actual)
            assert hashlib.sha256(actual['canonical_top']).hexdigest() == row['derived_top']['sha256']
            assert actual['transform'] == row['transform']
            recorded = next(x for x in rows[index + 1:]
                            if x.get('kind') in ('carry', 'image_binding'))
            if recorded['kind'] != 'carry':
                break  # release/verification frames belong to another phase
            assert actual['route'] == recorded['route']
            assert actual['decisions'] == recorded['decisions']
            assert actual['skew_evidence'] == recorded['skew_evidence']
            checked += 1
    finally:
        process.close()
    assert checked == sum(row.get('kind') == 'carry' for row in rows) == 121
    assert not process.process.is_alive()
