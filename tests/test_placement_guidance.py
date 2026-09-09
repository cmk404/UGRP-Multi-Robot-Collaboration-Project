import base64
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from harness.transport_context import current_placement_guidance
from harness.visual_placement import _entry_guidance, inspect_placement
from harness.gemini_transport_policy import GeminiTransportPlanner


@pytest.mark.parametrize('angle', [0, 35, -75, 130])
def test_entry_bearing_tracks_rotated_visible_square_not_world_axes(angle):
    yaw = math.radians(angle)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    corners = np.array([[.4, -.41], [1.22, -.41], [1.22, .41], [.4, .41]]) @ rotation.T
    result = _entry_guidance((0, 0), corners, (.034, .04, .032), -.47)
    assert result['entry_bearing_deg'] == angle
    assert 48 <= result['estimated_entry_distance_cm'] <= 52


def fixture(name):
    root = Path(__file__).parent / 'fixtures/markerless_box/entry_guidance'
    metadata = json.loads((root / 'metadata.json').read_text())[name]
    observations = []
    for label, camera in [('wrist', 'robot_cam'), ('nav', 'nav_cam')]:
        data = (root / f'{name}-{label}.jpg').read_bytes()
        observations.append({'camera': camera, 'image': base64.b64encode(data).decode(),
            'sha256': hashlib.sha256(data).hexdigest(),
            'actuator_state': {'servo_pulses': metadata['servo_pulses']}})
    return observations, metadata['zone']


@pytest.mark.parametrize('name,status,has_guidance', [
    ('near', 'outside', True), ('inside', 'inside', False), ('hidden_edge', 'uncertain', False),
])
def test_real_m4_rgb_gate_unchanged_and_guidance_requires_supported_entry(name, status, has_guidance):
    observations, zone = fixture(name)
    evidence = inspect_placement(*observations, cargo_id='small_box_01',
                                destination_zone=zone, held_identity_confirmed=True)
    assert evidence['status'] == status
    hint = current_placement_guidance([{'placement_evidence': evidence}],
                                     observations[0]['sha256'], observations[1]['sha256'], observations[0]['actuator_state']['servo_pulses'])
    assert bool(hint) == has_guidance
    if has_guidance:
        assert hint['minimum_footprint_margin_cm'] == -11
        assert 12 <= hint['estimated_entry_distance_cm'] <= 15
        assert 0 < hint['entry_bearing_deg'] < 75  # observed C interior is forward-left
        for wrist_hash, nav_hash in [('older-wrist', observations[1]['sha256']),
                                     (observations[0]['sha256'], 'older-nav')]:
            assert current_placement_guidance([{'placement_evidence': evidence}], wrist_hash, nav_hash, observations[0]['actuator_state']['servo_pulses']) is None
        newest = {'placement_evidence': {**evidence, 'nav_sha256': 'different'}}
        assert current_placement_guidance([{'placement_evidence': evidence}, newest],
            observations[0]['sha256'], observations[1]['sha256'], observations[0]['actuator_state']['servo_pulses']) is None


def test_unconfirmed_target_cannot_supply_entry_guidance():
    observations, zone = fixture('near')
    evidence = inspect_placement(*observations, cargo_id='small_box_01',
                                destination_zone=zone, held_identity_confirmed=False)
    assert 'navigation_guidance' not in evidence


@pytest.mark.parametrize('state,current', [('carrying', True), ('carrying', False), ('grip_uncertain', True)])
def test_actual_model_context_only_gets_current_carrying_guidance(state, current):
    observations, zone = fixture('near')
    for index, observation in enumerate(observations):
        observation.update(robot_id='r1', frame_id=index + 1, sim_time=10.)
    evidence = inspect_placement(*observations, cargo_id='small_box_01',
                                destination_zone=zone, held_identity_confirmed=True)
    if not current:
        evidence['nav_sha256'] = 'older'

    class Completer:
        def complete(self, messages, *, images):
            self.context = json.loads(messages[-1]['content'])
            return json.dumps({'action': {'kind': 'wait', 'duration': .2}, 'reason': '관측',
                               'navigation_note': {'observed': '경계 확인', 'maneuver': 'wait', 'resume_when': '새 관측'}})

    completer = Completer()
    planner = GeminiTransportPlanner('r1', completer)
    planner.decide(*observations, [{'placement_evidence': evidence}], cargo_id='small_box_01',
                   destination_zone=zone, skill_state=state)
    assert ('current_placement_guidance' in completer.context) == (state == 'carrying' and current)


def test_same_image_bytes_with_changed_fk_pose_cannot_reuse_guidance():
    observations, zone = fixture('near')
    evidence = inspect_placement(*observations, cargo_id='small_box_01',
                                destination_zone=zone, held_identity_confirmed=True)
    pose = dict(observations[0]['actuator_state']['servo_pulses'])
    pose['6'] += 30
    assert current_placement_guidance([{'placement_evidence': evidence}],
        observations[0]['sha256'], observations[1]['sha256'], pose) is None
