"""Tick-failure regression: no physics, network, evaluator, or result rewrites."""
import json

import pytest

from harness.rgb_communication_async import (
    AsyncRuntimeLimits, OfflineDecisionPlanner, run_rgb_communication_async,
)
from harness.rgb_communication_runtime import RuntimeLimits, run_rgb_communication
from harness.rgb_communication_clock import ExecutionClock, sanitized_error
from test_rgb_communication_runtime import FixturePort, FIXTURE


SECRET = "PRIVATE_PROVIDER_BODY_api_key_token"


class FaultPort(FixturePort):
    def __init__(self, *, partial=False, close_error=False, snapshot_error=False,
                 extra_metadata=False, known=True):
        super().__init__()
        self.partial, self.close_error = partial, close_error
        self.snapshot_error, self.extra_metadata = snapshot_error, extra_metadata
        self.known, self.last_ack = known, None
        self.close_calls = []
        self.failed = False

    def tick(self, requested):
        if requested >= .8:
            self.failed = True
            if self.partial:
                self.now_s = .77
            try:
                raise ValueError("sim_time must not move backwards")
            except ValueError as cause:
                raise RuntimeError(SECRET) from cause
        super().tick(requested)
        self.last_ack = requested
        return {"secret_success_oracle": SECRET}

    def clock_snapshot(self):
        if self.snapshot_error and self.failed:
            raise RuntimeError(SECRET)
        value = {"schema": "ugrp.execution_clock.v1", "clock_domain": "sim",
                 "last_acknowledged_time_s": self.last_ack,
                 "actual_time_s": self.now_s if self.known else None}
        if self.extra_metadata:
            value["measured_pose"] = SECRET
        return value

    def close(self, now):
        self.close_calls.append(now)
        if self.close_error:
            raise OSError(SECRET)
        if now is not None and now != self.now_s:
            raise ValueError("close must not invent or reverse physical time")
        self.closed_at_s = self.now_s


class WaitPlanner:
    model_name = "offline-clock-test"
    evidence_kind = "fixture"

    def decide(self, request):
        return {"action": {"kind": "wait"}, "message": None}


def run(path, port, *, clock=False, max_ticks=20):
    common = dict(condition="none", common_task=FIXTURE["static_context"]["task"])
    if clock:
        common["clock_snapshot"] = port.clock_snapshot
    if path == "async":
        planners = {rid: OfflineDecisionPlanner(lambda req: {"action": {"kind": "wait"},
                    "message": None}) for rid in ("r1", "r2", "r3")}
        return run_rgb_communication_async(port, planners, limits=AsyncRuntimeLimits(
            max_ticks=max_ticks, max_calls_per_robot=50, tick_period_s=.05, poll_period_s=.001,
            decision_period_s=1.), **common)
    return run_rgb_communication(port, {rid: WaitPlanner() for rid in ("r1", "r2", "r3")},
        limits=RuntimeLimits(max_ticks=max_ticks, max_calls_per_robot=50, tick_period_s=.05), **common)


@pytest.mark.parametrize("path", ["async", "sync"])
def test_failed_requested_tick_is_not_terminal_or_close_time_without_ack(path):
    port = FaultPort(partial=True)
    result = run(path, port)
    assert result["events"][-1]["sim_time_s"] is None
    assert port.close_calls == [None]
    assert result["clock"]["requested_tick_s"] == .8
    assert result["clock"]["last_acknowledged_time_s"] == .75
    assert result["clock"]["terminal_time_s"] is None
    assert result["clock"]["terminal_time_status"] == "unknown"
    assert result["termination_reason"] == "RUNTIME_ERROR"


@pytest.mark.parametrize("path", ["async", "sync"])
@pytest.mark.parametrize("partial,actual", [(False, .75), (True, .77)])
def test_snapshot_distinguishes_no_advance_and_partial_advance(path, partial, actual):
    port = FaultPort(partial=partial)
    result = run(path, port, clock=True)
    assert port.close_calls == [actual]
    assert result["events"][-1]["sim_time_s"] == actual
    assert result["clock"]["requested_tick_s"] == .8
    assert result["clock"]["last_acknowledged_time_s"] == .75
    assert result["clock"]["terminal_time_s"] == actual
    assert result["clock"]["terminal_time_status"] == "verified"
    assert result["termination_reason"] == "RUNTIME_ERROR"
    assert SECRET not in json.dumps(result)


@pytest.mark.parametrize("path", ["async", "sync"])
def test_primary_and_close_errors_are_distinct_with_sanitized_cause_chain(path):
    result = run(path, FaultPort(partial=True, close_error=True), clock=True)
    assert result["termination_reason"] == "RUNTIME_ERROR"
    assert result["primary_error"]["error_type"] == "RuntimeError"
    assert result["close_error"]["error_type"] == "OSError"
    chain = result["primary_error"]["cause_chain"]
    assert [item["error_type"] for item in chain] == ["RuntimeError", "ValueError"]
    assert chain[1]["error_code"] == "CLOCK_MOVED_BACKWARDS"
    assert SECRET not in json.dumps(result)


@pytest.mark.parametrize("path", ["async", "sync"])
@pytest.mark.parametrize("settings", [{"snapshot_error": True}, {"extra_metadata": True}, {"known": False}])
def test_invalid_or_missing_actual_metadata_remains_unknown_without_leaking(path, settings):
    port = FaultPort(partial=True, **settings)
    result = run(path, port, clock=True)
    assert result["events"][-1]["sim_time_s"] is None
    assert result["clock"]["terminal_time_status"] == "unknown"
    assert result["clock"]["reason"]
    assert port.close_calls == [None]
    assert SECRET not in json.dumps(result)


@pytest.mark.parametrize("replacement", [
    {"schema": SECRET}, {"clock_domain": "monotonic"},
    {"actual_time_s": float("nan")}, {"actual_time_s": float("inf")},
    {"actual_time_s": True}, {"actual_time_s": -.1},
    {"actual_time_s": 10 ** 999},
    {"actual_time_s": .7}, {"last_acknowledged_time_s": .8},
    {"last_acknowledged_time_s": .5}, {"last_acknowledged_time_s": None},
])
def test_invalid_clock_snapshot_never_promotes_stale_actual(replacement):
    value = {"schema": "ugrp.execution_clock.v1", "clock_domain": "sim",
             "last_acknowledged_time_s": .75, "actual_time_s": .75}
    clock = ExecutionClock("sim", lambda: dict(value))
    clock.refresh()
    assert clock.actual == .75
    value.update(replacement)
    clock.refresh()
    assert clock.actual is None
    assert clock.reason == "CLOCK_SNAPSHOT_INVALID"
    assert clock.last_ack == .75
    assert SECRET not in json.dumps(clock.payload())


@pytest.mark.parametrize("path", ["async", "sync"])
@pytest.mark.parametrize("delta", [-4.9e-15, 4.9e-15])
def test_successful_tick_preserves_raw_clock_not_requested_time(path, delta):
    class RawClockPort(FaultPort):
        def tick(self, requested):
            self.now_s = requested + delta if requested else 0.
            self.last_ack = self.now_s
    port = RawClockPort()
    result = run(path, port, clock=True, max_ticks=3)
    assert result["clock"]["requested_tick_s"] == .1
    assert result["clock"]["last_successful_requested_tick_s"] == .1
    assert result["clock"]["terminal_time_s"] == .1 + delta
    assert result["clock"]["last_acknowledged_time_s"] == .1 + delta
    assert result["events"][-1]["sim_time_s"] == .1 + delta
    assert port.close_calls == [.1 + delta]


@pytest.mark.parametrize("path", ["async", "sync"])
def test_close_only_error_is_recorded_without_a_primary_error(path):
    result = run(path, FaultPort(close_error=True), clock=True, max_ticks=1)
    assert result["primary_error"] is None
    assert result["close_error"]["error_type"] == "OSError"
    assert result["termination_reason"] == "CLOSE_ERROR"
    errors = [e for e in result["events"] if e["event_type"] == "run_error"]
    assert errors[0]["payload"]["phase"] == "close"
    assert SECRET not in json.dumps(result)


def test_sanitizer_drops_custom_class_code_and_message_and_bounds_cycles():
    error = type(SECRET, (RuntimeError,), {})(SECRET)
    error.error_code = SECRET
    error.__cause__ = error
    result = sanitized_error(error)
    assert result["error_type"] == "Exception"
    assert result["error_code"] == "UNCLASSIFIED_ERROR"
    assert len(result["cause_chain"]) == 1
    assert result["chain_truncated"] is True
    assert SECRET not in json.dumps(result)


def test_backend_safe_error_codes_are_preserved_but_never_messages():
    error = RuntimeError(SECRET)
    error.error_code = "SIM_CLOCK_REVERSED"
    assert sanitized_error(error)["error_code"] == "SIM_CLOCK_REVERSED"
    assert SECRET not in json.dumps(sanitized_error(error))


@pytest.mark.parametrize("path", ["async", "sync"])
def test_supervisor_clock_does_not_enter_or_change_actor_requests(path):
    captures = []
    for hidden_offset in (0., 10.):
        captured = {}

        class SupervisorPort(FixturePort):
            def clock_snapshot(self):
                return {"schema": "ugrp.execution_clock.v1", "clock_domain": "sim",
                        "last_acknowledged_time_s": self.now_s + hidden_offset,
                        "actual_time_s": self.now_s + hidden_offset}

        class CapturePlanner(WaitPlanner):
            def __init__(self, rid):
                self.rid = rid

            def decide(self, request):
                captured[self.rid] = request
                return {"action": {"kind": "wait"}, "message": None}

        port = SupervisorPort()
        planners = {rid: CapturePlanner(rid) for rid in ("r1", "r2", "r3")}
        common = dict(run_id="clock-noninterference", condition="none",
                      common_task=FIXTURE["static_context"]["task"],
                      clock_snapshot=port.clock_snapshot)
        if path == "async":
            planners = {rid: OfflineDecisionPlanner(planner.decide)
                        for rid, planner in planners.items()}
            result = run_rgb_communication_async(port, planners,
                limits=AsyncRuntimeLimits(max_ticks=2, max_calls_per_robot=1,
                                          poll_period_s=.001), **common)
        else:
            result = run_rgb_communication(port, planners,
                limits=RuntimeLimits(max_ticks=1, max_calls_per_robot=1), **common)
        assert len(captured) == 3
        serialized = json.dumps(captured)
        assert "terminal_time_s" not in serialized
        assert "last_acknowledged_time_s" not in serialized
        assert "actual_time_s" not in serialized
        assert result["clock"]["terminal_time_status"] == "verified"
        captures.append(captured)
    assert captures[0] == captures[1]
