from harness.camera_message_bus import CameraMessageBus
from harness.inference_recovery import InferenceRecovery
import pytest


def test_messages_are_other_agent_authored_expire_from_observation_time_and_copy():
    bus = CameraMessageBus(['r1','r2','r3'], 'status', ttl=10, limit=1)
    payload = {'observed':'주황 장벽이 앞에 보임','intent':'wait','request':'clear_path'}
    row = bus.publish('r1', payload, now=7, observed_at=1, decision_id='r1-call-1')
    payload['observed'] = 'changed'
    assert bus.inbox('r1', 8) == []
    assert bus.inbox('r2', 8)[0]['content']['observed'] != 'changed'
    assert bus.inbox('r2', 12) == []
    assert row['decision_id'] == 'r1-call-1'


def test_no_comm_has_no_delivery_or_storage():
    bus = CameraMessageBus(['r1','r2'])
    assert bus.publish('r1', 'hello', now=1, observed_at=1, decision_id='d') is None
    assert bus.inbox('r2', 2) == [] and not bus.sent


def test_message_schema_cannot_forward_extra_state():
    bus = CameraMessageBus(['r1','r2'], 'status')
    with pytest.raises(ValueError):
        bus.publish('r1', {'observed':'box','intent':'carry','request':'none','world_state':{}}, now=1, observed_at=1, decision_id='d')


def test_retry_backoff_is_bounded_and_reset_after_success():
    class Transient(Exception):
        retryable = True
        error_kind = 'timeout'
    r = InferenceRecovery(3)
    assert r.failure(Transient(), 10)['requires_fresh_observation']
    assert not r.ready(10.5) and r.ready(11)
    assert r.failure(Transient(), 11)['retry_delay_wall_s'] == 2
    assert not r.failure(Transient(), 13)['retry_scheduled']
    r.success()
    assert r.failure(Transient(), 20)['consecutive_errors'] == 1


def test_nonretryable_error_stops_without_hidden_retries():
    assert not InferenceRecovery().failure(ValueError('bad config'), 0)['retry_scheduled']


def test_rate_limit_honors_server_cooldown_and_fresh_capture_requirement():
    class Limited(Exception):
        retryable = True
        http_status = 429
        retry_after_s = 75
    recovery = InferenceRecovery()
    result = recovery.failure(Limited(), 10)
    assert result['retry_delay_wall_s'] == 75
    assert result['requires_fresh_observation']
    assert not recovery.ready(84) and recovery.ready(85)


def test_planner_wrapper_retains_retryable_transport_cause():
    from harness.inference_recovery import transport_error
    from harness.gemini_proxy import GeminiProxyError
    error = GeminiProxyError('timeout', error_kind='timeout', retryable=True)
    try:
        try:
            raise error
        except Exception as exc:
            raise RuntimeError('planning failed') from exc
    except RuntimeError as wrapped:
        assert transport_error(wrapped) is error
        assert InferenceRecovery().failure(transport_error(wrapped),0)['retry_scheduled']


def test_referee_rejects_rotated_corner_outside_square():
    from scripts.evaluate_gemini_team import footprint_inside
    import math
    assert footprint_inside((.38,0),0,(.04,.04),(0,0),(.41,.41))
    assert not footprint_inside((.385,0),math.pi/4,(.04,.04),(0,0),(.41,.41))
