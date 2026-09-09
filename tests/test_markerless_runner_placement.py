import base64
import hashlib
import json
from pathlib import Path

import pytest

from harness.llm_transport_skill import LLMTransportSkill
from scripts.evaluate_gemini_team import inspect_actor_placement


def released_observations():
    root = Path(__file__).parent / 'fixtures/markerless_box/released_finish'
    metadata = json.loads((root / 'metadata.json').read_text())
    observations = []
    for label in ('wrist', 'nav'):
        entry = metadata['files'][label]
        payload = (root / entry['file']).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == entry['sha256']
        observations.append({
            'camera': entry['camera'], 'sha256': entry['sha256'],
            'image': base64.b64encode(payload).decode(),
            'actuator_state': {'servo_pulses': metadata['servo_pulses']},
        })
    return observations


def test_fresh_finish_check_preserves_completed_release_continuity():
    actor = LLMTransportSkill('r1', 'small_box_01', 'B')
    actor.state = 'released'
    actor.box.reason = 'VISUAL_RELEASE_CONFIRMED'
    actor.box.held = False
    evidence = inspect_actor_placement(actor, *released_observations())
    assert evidence['status'] == 'inside', evidence
    assert evidence['identity']['confirmed'] is True
    assert actor.request({'kind': 'finish'}, placement_evidence=evidence) == {
        'kind': 'finish', 'reason': 'VISUAL_RELEASE_CONFIRMED',
    }
    assert actor.state == 'finished'


@pytest.mark.parametrize('reason', ['RUNNING', 'RELEASE_GROUND_TARGET_UNOBSERVABLE'])
def test_released_label_without_camera_sweep_proof_cannot_finish(reason):
    actor = LLMTransportSkill('r1', 'small_box_01', 'B')
    actor.state = 'released'
    actor.box.reason = reason
    actor.box.held = False
    evidence = inspect_actor_placement(actor, *released_observations())
    assert evidence['status'] == 'uncertain'
    assert evidence['identity']['confirmed'] is False
    with pytest.raises(ValueError, match='CAMERA_INSIDE_EVIDENCE_REQUIRED_BEFORE_FINISH'):
        actor.request({'kind': 'finish'}, placement_evidence=evidence)
