"""Replay v43 admission failure; hypothetical post-HOLD RGB is not physics."""
import base64
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from harness.markerless_box import observe_ground_box
from harness.rolling_visual_servo import ActiveViewRecovery, SettledViewRecovery
from scripts.run_dispatch_skills import SkillScene

FIXTURE = Path(__file__).parent / 'fixtures/settled_view_recovery'


def samples():
    manifest = json.loads((FIXTURE / 'manifest.json').read_text())
    result = []
    for item in manifest['items']:
        payload = (FIXTURE / item['fixture']).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == item['source_sha256']
        row = item['row']
        pose = row['observation']['actuator_state']['servo_pulses']
        box = observe_ground_box(base64.b64encode(payload).decode(), pose, 'small_box_01')
        result.append(dict(frame_id=row['frame_id'],
            observed_at_s=row['observed_at_s'], decision_at_s=row['sim_time_s'],
            capture_started_at_s=row['rolling_capture_evidence']['capture_started_at_s'],
            own_sha256=item['source_sha256'], top_sha256=row['images']['top']['sha256'],
            paired_top_sha256=row['images']['top']['sha256'], pose=pose, box=box,
            target=box['estimated_box_center_base_m'], phase='approach',
            cargo_id='small_box_01', plan_hash='saved-v43-plan'))
    return manifest, *result


def started():
    manifest, strong, weak = samples()
    recovery = SettledViewRecovery()
    assert recovery.remember(strong, manifest['completed_pose_receipts'], 10.0685)
    for _ in range(manifest['intervening_wheel_issue_count']):
        recovery.note_wheel_issue()
    event = recovery.start(weak)
    assert event['kind'] == 'hold_reobserve'
    assert recovery.anchor['wheel_generation'] == 0
    assert recovery.episode['held_wheel_generation'] == 82
    return recovery, strong, weak, event


def fresh_after_hold(sample, weak, event):
    result = copy.deepcopy(sample)
    result.update(frame_id=weak['frame_id'] + 10,
                  capture_started_at_s=event['capture_not_before_s'] + .01,
                  observed_at_s=event['capture_not_before_s'] + .12,
                  decision_at_s=event['capture_not_before_s'] + .30)
    return result


def test_real_v43_failure_is_wheel_invalidation_not_an_old_anchor():
    manifest, strong, weak = samples()
    original = ActiveViewRecovery()
    assert original.remember(strong, manifest['completed_pose_receipts'], 10.0685)
    assert weak['observed_at_s'] - strong['observed_at_s'] < 2
    for _ in range(82):
        original.note_wheel_issue()
    assert original.start(weak)['reason'] == 'anchor_stale_or_wheel_issued'
    recovery, _, _, event = started()
    assert recovery.poses_issued == 0
    assert event['capture_not_before_s'] == pytest.approx(weak['decision_at_s'] + .2)


def test_new_strong_capture_can_resume_without_a_camera_search():
    recovery, strong, weak, event = started()
    result = recovery.advance(fresh_after_hold(strong, weak, event))
    assert result['kind'] == 'recovered'
    assert result['reason'] == 'strong_settled_own_rgb'
    assert recovery.poses_issued == 0 and recovery.settled_observations == 1
    assert recovery.episode is None


def test_still_weak_new_capture_can_only_start_a_bounded_pose_search():
    recovery, _, weak, event = started()
    post = fresh_after_hold(weak, weak, event)
    # Identical JPEG pixels after an actual new stationary capture are valid;
    # this does not upgrade their confidence or permit a wheel command.
    result = recovery.advance(post)
    assert result['kind'] == 'pose_issued'
    assert result['issued_pose'] == {'3': 716}
    assert result['settled_search']['anchor_is_search_pose_only'] is True
    assert result['settled_search']['anchor_wheel_generation'] == 0
    assert result['settled_search']['held_wheel_generation'] == 82
    assert recovery.anchor['own_sha256'] != post['own_sha256']
    assert recovery.episode['started_at_s'] == weak['decision_at_s']


@pytest.mark.parametrize('damage,reason', [
    (lambda s,r,e: s.update(capture_started_at_s=e['capture_not_before_s']-.001),
     'missing_post_hold_settled_capture'),
    (lambda s,r,e: s.update(frame_id=97), 'missing_post_hold_settled_capture'),
    (lambda s,r,e: r.note_wheel_issue(), 'own_command_changed_during_settle'),
    (lambda s,r,e: s['pose'].update({'6':1600}), 'own_command_changed_during_settle'),
    (lambda s,r,e: s.update(plan_hash='other'), 'plan_changed_during_active_view'),
    (lambda s,r,e: s.update(phase='lower'), 'phase_exit_during_active_view'),
    (lambda s,r,e: s['box'].update(ambiguity_reason='second object'),
     'settled_view_identity_unresolved'),
])
def test_settle_requires_new_paired_rgb_and_no_intervening_own_command(damage, reason):
    recovery, strong, weak, event = started()
    post = fresh_after_hold(strong, weak, event)
    damage(post, recovery, event)
    assert recovery.advance(post)['reason'] == reason
    assert recovery.episode is None


def test_whole_budget_starts_at_hold_and_stale_search_anchor_is_not_relabelled():
    recovery, _, weak, event = started()
    post = fresh_after_hold(weak, weak, event)
    post.update(capture_started_at_s=weak['decision_at_s']+3.8,
                observed_at_s=weak['decision_at_s']+3.9,
                decision_at_s=weak['decision_at_s']+4.01)
    assert recovery.advance(post)['reason'] == 'active_view_budget_exhausted'
    recovery, _, weak, event = started()
    post = fresh_after_hold(weak, weak, event)
    recovery.anchor['observed_at_s'] -= 3
    assert recovery.advance(post)['reason'] == 'search_anchor_stale'


def test_actual_owner_hold_schedules_reobservation_without_submitting_drive():
    recovery, _, weak, event = started()
    port = SimpleNamespace(_motor_commands=(.1, .1, .1, .1))
    port.hold = lambda now: setattr(port, '_motor_commands', (0.,)*4)
    scene = SkillScene.__new__(SkillScene)
    scene.ports = {'r2': port}; scene.bindings = SimpleNamespace(solo='r2')
    scene.solo_raw = []; scene.solo_rows = []
    scene.solo_executor = SimpleNamespace(submit=Mock())
    row = {'action': {'kind':'drive','fwd':.1}}
    scene._issue_rolling_view_reobserve(event, row, weak['decision_at_s'])
    assert scene._solo_retry_at == event['capture_not_before_s']
    assert scene.solo_raw[-1]['motor_commands'] == [0.]*4
    assert row['issued_action']['kind'] == 'hold'
    scene.solo_executor.submit.assert_not_called()
