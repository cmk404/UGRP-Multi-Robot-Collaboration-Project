"""Offline regression fixtures; these never claim real Gemini delivery."""
import base64
import hashlib
import inspect

import cv2
import numpy as np

from harness.navigation_events import NavigationEvents


def image(textured=True):
    pixels = np.full((480, 640, 3), 70, np.uint8)
    if textured:
        pixels[::16, :, :] = 180
        pixels[:, ::16, :] = 180
    ok, encoded = cv2.imencode('.jpg', pixels)
    assert ok
    raw = encoded.tobytes()
    return {'camera': 'nav_cam', 'image': base64.b64encode(raw).decode(),
            'sha256': hashlib.sha256(raw).hexdigest()}


def begin(events, action):
    if hasattr(events, 'begin_command'):
        events.begin_command(action, ())
    else:
        events.reset(())  # The baseline runner resets on every new model action.


def observe(events, action, wall_sim_time, drive_control_s, *, textured=True):
    kwargs = ({'executed_drive_s': drive_control_s}
              if 'executed_drive_s' in inspect.signature(events.inspect).parameters else {})
    return events.inspect(image(textured), action, wall_sim_time, **kwargs)


def test_stagnation_survives_multiple_short_drives():
    events = NavigationEvents()
    action = {'kind': 'drive', 'fwd': .1, 'turn': .1, 'duration': 1.5}
    reasons = []
    for command in range(3):
        begin(events, action)
        for index in range(6):
            verdict = observe(events, action, command * 100 + index * .25,
                              command * 1.5 + index * .25)
            reasons.append(verdict.get('reason'))
    assert 'OWN_RGB_STAGNATION' in reasons


def test_flat_frame_is_unobservable_not_proof_of_stagnation():
    events = NavigationEvents()
    action = {'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': 4.0}
    begin(events, action)
    observe(events, action, 0, 0, textured=False)
    verdict = observe(events, action, 2.0, 2.0, textured=False)
    assert verdict['allowed']
    assert verdict['progress']['status'] == 'PROGRESS_UNOBSERVABLE'

import json
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import pytest

from harness.navigation_events import InferenceInputBudget
from harness.visual_macro_runtime import VisualMacroExecutor
from harness.gemini_transport_policy import GeminiTransportPlanner
from harness.inference_accounting import settle_pending
from scripts.evaluate_gemini_cohort import build_run_command


class Port:
    robot_id = 'r1'
    def __init__(self): self.applied = []; self.stops = 0
    def tick(self, now): pass
    def stop(self): self.stops += 1
    def apply(self, action, now): self.applied.append((now, dict(action)))


def macro_obs():
    return {'sha256': 'offline-fixture', 'actuator_state': {'servo_pulses': {}}}


def test_inference_wait_does_not_count_as_driving():
    events = NavigationEvents()
    action = {'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': 1.5}
    begin(events, action)
    assert observe(events, action, 0, 0)['allowed']
    assert observe(events, action, 1.5, 1.5)['allowed']
    begin(events, action)
    assert observe(events, action, 101.5, 1.5)['allowed']
    assert observe(events, action, 102.0, 2.0)['reason'] == 'OWN_RGB_STAGNATION'


def test_changed_direction_and_non_drive_reset_progress_not_all_short_commands():
    events = NavigationEvents()
    left = {'kind': 'drive', 'fwd': .1, 'turn': .2, 'duration': 1.5}
    right = {**left, 'turn': -.2}
    begin(events, left)
    observe(events, left, 0, 0)
    assert observe(events, left, 1.5, 1.5)['allowed']
    begin(events, right)
    assert observe(events, right, 2.0, 2.0)['allowed']
    begin(events, {'kind': 'pick'})
    assert events.anchor is None


def test_small_turn_adjustments_do_not_erase_same_family_progress():
    events = NavigationEvents()
    left = {'kind': 'drive', 'fwd': .1, 'turn': .18, 'duration': 1.5}
    begin(events, left); observe(events, left, 0, 0)
    adjusted = {**left, 'turn': .12}
    begin(events, adjusted)
    assert observe(events, adjusted, 100, 2.0)['reason'] == 'OWN_RGB_STAGNATION'


def test_elapsed_drive_lease_is_not_requested_duration_or_inference_idle():
    port = Port()
    ex = VisualMacroExecutor(port, drive_guard=lambda a, t: {'allowed': t < .5, 'reason': 'blocked'})
    ex.submit({'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': 3.5}, macro_obs(), 'navigate', 0)
    ex.tick(.25); ex.tick(.5)
    assert ex.last_execution['requested_duration_s'] == 3.5
    assert ex.last_execution['elapsed_drive_control_s'] == .5
    assert ex.last_execution['status'] == 'interrupted'
    assert ex.last_execution['motion_confirmed'] is False
    ex.tick(100)
    assert ex.total_drive_control_s == .5


def test_partial_cancel_counts_only_elapsed_part_of_current_lease():
    ex = VisualMacroExecutor(Port(), drive_guard=lambda a, t: {'allowed': True})
    ex.submit({'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': 4}, macro_obs(), 'navigate', 0)
    ex.cancel(.1, 'stop')
    ex.tick(10)
    assert ex.total_drive_control_s == pytest.approx(.1)
    assert ex.last_execution['elapsed_drive_control_s'] == pytest.approx(.1)


def test_late_tick_never_counts_unsent_or_expired_slices_as_executed():
    ex = VisualMacroExecutor(Port(), drive_guard=lambda a, t: {'allowed': True})
    ex.submit({'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': 1}, macro_obs(), 'navigate', 0)
    ex.tick(1.2)
    assert ex.last_execution['status'] == 'completed'
    assert ex.last_execution['elapsed_drive_control_s'] == .25
    assert ex.total_drive_control_s == .25


def test_executor_and_progress_guard_keep_clock_across_actual_short_macros():
    events = NavigationEvents(); port = Port()
    ex = VisualMacroExecutor(port, drive_guard=lambda a, t: events.inspect(
        image(), a, t, executed_drive_s=ex.total_drive_control_s))
    action = {'kind': 'drive', 'fwd': .1, 'turn': .1, 'duration': 1.5}
    events.begin_command(action)
    ex.submit(action, macro_obs(), 'navigate', 0)
    for t in np.arange(.25, 1.51, .25): ex.tick(float(t))
    ex.tick(1.7)
    assert ex.total_drive_control_s == pytest.approx(1.5)
    events.begin_command(action)
    ex.submit(action, macro_obs(), 'navigate', 100)
    ex.tick(100.25); ex.tick(100.5)
    assert ex.last_interruption['reason'] == 'OWN_RGB_STAGNATION'


def test_estimated_preflight_includes_inflight_capacity_and_is_idempotent():
    b = InferenceInputBudget(100, initial_request_estimate=40)
    assert b.reserve('a')
    assert b.remaining == 60
    assert b.reserve('b')
    assert b.remaining == 20
    assert not b.reserve('c')
    b.record('a', {'prompt_tokens': 35})
    b.record('a', {'prompt_tokens': 35})
    assert b.tokens == 35 and b.remaining == 25
    b.cancel_unstarted('b')
    assert b.remaining == 65 and b.verified_within_limit


def test_unknown_usage_is_not_free_and_prevents_verified_budget_claim():
    b = InferenceInputBudget(60000)
    assert b.reserve('a')
    b.record('a', None)
    assert b.tokens == 0 and b.calls_without_usage == 1
    assert b.estimated_unreported_tokens == 6000
    assert b.remaining == 54000 and not b.verified_within_limit


def test_underestimated_request_preserves_actual_overrun_and_disables_success():
    b = InferenceInputBudget(100, initial_request_estimate=80)
    assert b.reserve('a')
    b.record('a', {'prompt_tokens': 105})
    assert b.tokens == 105 and b.exhausted
    assert not b.verified_within_limit and not b.can_reserve


@pytest.mark.parametrize('invalid', [None, True, -1, float('nan'), 1.5])
def test_invalid_reported_token_values_are_unknown_not_zero(invalid):
    b = InferenceInputBudget(100, initial_request_estimate=20)
    assert b.reserve('a')
    b.record('a', {'prompt_tokens': invalid})
    assert b.estimated_unreported_tokens == 20
    assert b.calls_without_usage == 1
    assert not b.verified_within_limit


def test_robot_budgets_are_independent():
    a = InferenceInputBudget(100, initial_request_estimate=50)
    b = InferenceInputBudget(100, initial_request_estimate=50)
    a.reserve('r1-a'); a.record('r1-a', {'prompt_tokens': 100})
    assert a.exhausted and b.remaining == 100


def test_late_future_is_accounted_without_returning_an_action_for_execution():
    budget = InferenceInputBudget(100, initial_request_estimate=50)
    budget.reserve('a')
    planner = SimpleNamespace(last_audit={'usage': {'prompt_tokens': 55}})
    f = Future(); f.set_result({'action': {'kind': 'drive'}, 'reason': 'test'})
    pending = {'r1': {'future': f, 'call_id': 'a'}}
    rows = []
    settle_pending(pending, {'r1': planner}, {'r1': budget}, ThreadPoolExecutor(1), now=300, emit=rows.append)
    assert budget.tokens == 55 and not pending
    assert rows[0]['disposition'] == 'discarded_run_ended'
    assert 'apply' not in rows[0]


def test_not_started_future_releases_reservation_without_charging_fake_usage():
    budget = InferenceInputBudget(100, initial_request_estimate=50); budget.reserve('a')
    pending = {'r1': {'future': Future(), 'call_id': 'a'}}
    rows = []
    settle_pending(pending, {'r1': SimpleNamespace(last_audit={'usage': {'prompt_tokens': 90}})},
                   {'r1': budget}, ThreadPoolExecutor(1), now=300, emit=rows.append)
    assert budget.tokens == 0 and budget.remaining == 100
    assert budget.calls_without_usage == 0
    assert rows[0]['disposition'] == 'cancelled_before_start'


def test_cohort_passes_same_explicit_model_and_input_budget_to_all_modes(tmp_path):
    args = SimpleNamespace(output=tmp_path, seconds=300, max_calls=30,
                           max_input_tokens=60000, input_request_estimate=6000, model='gemini-3.8-flash')
    for mode in ('none', 'status', 'natural'):
        command = build_run_command(args, {'path': mode, 'seed': 41, 'communication': mode})
        assert command[command.index('--max-input-tokens') + 1] == '60000'
        assert command[command.index('--max-calls') + 1] == '30'
        assert command[command.index('--model') + 1] == 'gemini-3.8-flash'
        assert command[command.index('--input-request-estimate') + 1] == '6000'


class OfflineCompleter:
    model_name = 'OFFLINE_TEST_DOUBLE'
    last_model = 'OFFLINE_TEST_DOUBLE'
    last_usage = {'prompt_tokens': 123}
    def complete(self, messages, *, images):
        self.messages = messages; self.images = images
        return json.dumps({'action': {'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': 1.5},
                           'reason': '자기 화면에서 빈 통로를 확인함'}, ensure_ascii=False)


def actor_obs(camera):
    return {**image(), 'camera': camera, 'robot_id': 'r1', 'frame_id': 1, 'sim_time': 1.,
            'actuator_state': {'servo_pulses': {}}}


def test_model_receives_budget_and_actual_execution_without_rewriting_its_action():
    c = OfflineCompleter(); p = GeminiTransportPlanner('r1', c)
    b = InferenceInputBudget(60000); b.reserve('r1-a')
    snapshot = b.snapshot(calls_used=1, max_calls=30, remaining_sim_seconds=280)
    execution = {'last_decision': {'disposition': 'accepted'}, 'last_macro': {
        'status': 'interrupted', 'requested_duration_s': 3.5, 'elapsed_drive_control_s': .5}}
    result = p.decide(actor_obs('robot_cam'), actor_obs('nav_cam'), [],
                      cargo_id='small_box_01', destination_zone='A', skill_state='carrying',
                      budget=snapshot, execution_feedback=execution)
    context = json.loads(c.messages[-1]['content'])
    assert context['budget'] == snapshot and context['execution_feedback'] == execution
    assert result['action']['duration'] == 1.5  # Never silently upgrade to 4 seconds.
    assert len(c.images) == 2
    assert p.last_audit['model_context']['budget']['calls_remaining'] == 29


def test_spatial_oracle_in_new_context_is_rejected_before_any_model_call():
    c = OfflineCompleter(); p = GeminiTransportPlanner('r1', c)
    with pytest.raises(ValueError, match='SPATIAL_ORACLE'):
        p.decide(actor_obs('robot_cam'), actor_obs('nav_cam'), [],
                 cargo_id='small_box_01', destination_zone='A', skill_state='carrying',
                 execution_feedback={'last_macro': {'world_state': {}}})
    assert not hasattr(c, 'messages')


def test_pre_provider_validation_failure_clears_stale_usage_audit():
    p = GeminiTransportPlanner('r1', OfflineCompleter())
    p.last_audit = {'usage': {'prompt_tokens': 9999}}
    with pytest.raises(ValueError):
        p.decide(actor_obs('robot_cam'), actor_obs('nav_cam'), {'coordinates': [1, 2]},
                 cargo_id='small_box_01', destination_zone='A', skill_state='carrying')
    assert p.last_audit['usage'] is None


def test_running_done_robot_future_does_not_block_other_robots():
    from harness.inference_accounting import settle_completed
    future = Future(); future.set_running_or_notify_cancel()
    b = InferenceInputBudget(100, initial_request_estimate=50); b.reserve('a')
    assert settle_completed('r1', {'future': future, 'call_id': 'a'},
                            SimpleNamespace(last_audit=None), b, now=100) is None
    assert b.remaining == 50
    future.set_result({'action': {'kind': 'wait'}})
    row = settle_completed('r1', {'future': future, 'call_id': 'a'},
                           SimpleNamespace(last_audit={'usage': {'prompt_tokens': 40}}), b, now=101)
    assert row['disposition'] == 'discarded_run_ended'
    assert b.tokens == 40 and b.remaining == 60


def test_late_failed_response_counts_usage_without_action_execution():
    budget = InferenceInputBudget(100, initial_request_estimate=50); budget.reserve('a')
    future = Future(); future.set_exception(ValueError('OFFLINE_TEST_EXCEPTION'))
    rows = []
    pending = {'r1': {'future': future, 'call_id': 'a'}}
    settle_pending(pending, {'r1': SimpleNamespace(last_audit={'usage': {'prompt_tokens': 60}})},
                   {'r1': budget}, ThreadPoolExecutor(1), now=300, emit=rows.append)
    assert budget.tokens == 60
    assert rows[0]['decision'] is None and rows[0]['error_type'] == 'ValueError'
