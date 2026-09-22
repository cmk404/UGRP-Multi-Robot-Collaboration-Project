"""Offline scheduling/causality tests. These are not physics or model results."""
import copy
from dataclasses import replace
import json
import threading
import time

import pytest

from harness.rgb_communication_async import (
    AsyncRuntimeLimits, OfflineDecisionPlanner, RuntimeControl,
    TokenLedger, run_rgb_communication_async,
)
from harness.rgb_communication_planner import PlannerCallError
from test_rgb_communication_runtime import FIXTURE, FixturePort


def run(callbacks, *, port=None, condition="none", limits=None, **kwargs):
    port = port or FixturePort()
    planners = {rid: OfflineDecisionPlanner(cb) for rid, cb in callbacks.items()}
    result = run_rgb_communication_async(port, planners, condition=condition,
        common_task=FIXTURE["static_context"]["task"], limits=limits or AsyncRuntimeLimits(
            max_ticks=60, poll_period_s=.002, decision_period_s=.05), **kwargs)
    return result, port


def finish(request):
    return {"action": {"kind": "finish", "claim": "mission_complete"}, "message": None}


def callbacks(callback=finish):
    return {rid: callback for rid in ("r1", "r2", "r3")}


def reasons(result):
    return [e["payload"].get("reason") for e in result["events"]]


def test_completion_is_actor_claim_not_physical_success_and_provenance_is_private():
    requests = []
    def cb(request):
        requests.append(request)
        return finish(request)
    result, port = run(callbacks(cb), provenance={"split": "HELD_OUT_TEST", "seed": 973})
    assert result["outcome"] == "completed"
    assert result["actor_finish_claims"] == dict.fromkeys(callbacks(), True)
    assert result["external_model_calls"] == 0
    assert result["planner_decisions"] == 3
    assert all("HELD_OUT_TEST" not in json.dumps(r) for r in requests)
    assert all("SECRET" not in json.dumps(r) for r in requests)
    assert port.closed_at_s is not None


def test_slow_robot_does_not_block_peer_decisions_or_clock_or_shutdown():
    release, other_done = threading.Event(), threading.Event()
    def slow(request):
        release.wait(2)
        return finish(request)
    def fast(request):
        other_done.set()
        return finish(request)
    try:
        started = time.monotonic()
        result, port = run({"r1": slow, "r2": fast, "r3": fast}, limits=AsyncRuntimeLimits(
            max_ticks=25, poll_period_s=.002, decision_period_s=.05, decision_timeout_s=.015))
        assert time.monotonic() - started < 1
        assert other_done.is_set()
        assert port.now_s > .5
        assert result["calls"]["r1"] == 1
        assert result["actor_finish_claims"] == {"r1": False, "r2": True, "r3": True}
        assert result["pending_planner_requests"] == {"r1": "r1-request-0001"}
        assert "LATE_REPLY" in reasons(result)
        assert result["peak_in_flight"] == 3
        before = copy.deepcopy(port.submissions)
        release.set()
        assert port.submissions == before  # worker has no port reference
    finally:
        release.set()


def test_cancel_reply_remains_one_inflight_and_never_finishes_actor():
    control = RuntimeControl()
    def cancel(request):
        control.cancel_inference("r1")
        return finish(request)
    result, _ = run({"r1": cancel, "r2": finish, "r3": finish}, control=control,
        limits=AsyncRuntimeLimits(max_calls_per_robot=1, max_ticks=30,
            poll_period_s=.002, decision_period_s=.05))
    assert result["calls"]["r1"] == 1
    assert not result["actor_finish_claims"]["r1"]
    assert "CANCELLED_REPLY" in reasons(result)


def test_cancelled_transport_can_replan_after_drain_without_api_retry():
    control = RuntimeControl()
    def interrupted(request):
        if request["request_id"].endswith("0001"):
            control.cancel_inference("r1")
            raise PlannerCallError("CANCELLED_BEFORE_SEND")
        return finish(request)
    result, _ = run({"r1": interrupted, "r2": finish, "r3": finish}, control=control)
    assert result["outcome"] == "completed"
    assert result["calls"]["r1"] == 2
    assert not result["retirement_reasons"]


@pytest.mark.parametrize("change", ["own_revision", "own_submission", "commands"])
def test_changed_own_evidence_rejects_old_reply(change):
    class ChangedPort(FixturePort):
        def observe(self, rid):
            obs = super().observe(rid)
            if change == "own_revision":
                obs["own_revision"] = int(self.now_s > 0)
            if change == "commands" and self.now_s > 0:
                obs["own_issued_commands"] = [{"task_id": "t", "lease_id": "l",
                    "command_id": "new", "stage": "RUN", "action": {"kind": "hold"},
                    "duration_s": .1, "issued_at_s": self.now_s, "meaning": "issued"}]
            return obs
        def local_status(self, rid):
            value = super().local_status(rid)
            if change == "own_revision":
                value["own_revision"] = int(self.now_s > 0)
            if change == "own_submission" and self.now_s > 0:
                value["own_submission_history"] = [{"request_id": "old", "status": "TERMINATED",
                    "reason": "lease_expired", "clock_domain": "sim", "timestamp_s": self.now_s}]
            return value
    result, _ = run(callbacks(), port=ChangedPort(), limits=AsyncRuntimeLimits(
        max_calls_per_robot=1, max_ticks=20, poll_period_s=.002, decision_period_s=.05))
    assert "OWN_EVIDENCE_CHANGED" in reasons(result)
    assert not any(result["actor_finish_claims"].values())


def test_stale_observation_and_wrong_echo_are_rejected():
    result, _ = run(callbacks(), limits=AsyncRuntimeLimits(max_calls_per_robot=1,
        max_ticks=20, poll_period_s=.002, decision_period_s=.05, max_observation_age_s=.001))
    assert "STALE_OBSERVATION" in reasons(result)
    def duplicate(request):
        return {**finish(request), "request_id": "r1-previous-request"}
    result, _ = run(callbacks(duplicate), limits=AsyncRuntimeLimits(max_calls_per_robot=1,
        max_ticks=20, poll_period_s=.002, decision_period_s=.05))
    assert "REPLY_CORRELATION_MISMATCH" in reasons(result)
    assert not any(result["actor_finish_claims"].values())


@pytest.mark.parametrize("condition", ["none", "structured", "natural"])
def test_same_scheduler_and_legal_actions_across_conditions(condition):
    result, _ = run(callbacks(), condition=condition)
    assert result["outcome"] == "completed"
    assert result["calls"] == {"r1": 1, "r2": 1, "r3": 1}
    assert result["events"][0]["payload"]["scheduler"] == "async_single_clock"


def test_unknown_usage_retains_reservation_and_violation_is_not_clamped():
    class Prepared:
        input_token_bound, output_token_bound = 7, 3
    ledger = TokenLedger(AsyncRuntimeLimits(max_input_tokens=10, max_output_tokens=5))
    ledger.reserve("first", Prepared())
    assert not ledger.settle("first", {})
    assert ledger.summary()["charged_input_tokens"] == 7
    with pytest.raises(PlannerCallError, match="TOKEN_BUDGET_EXHAUSTED"):
        ledger.reserve("second", Prepared())
    assert ledger.settle("first", {"input_tokens": 8, "output_tokens": 2})
    assert ledger.summary()["charged_input_tokens"] == 8


def test_concurrency_one_is_fair_and_failure_does_not_block_others():
    def error(request):
        raise PlannerCallError("OFFLINE_PROVIDER_FAILURE")
    result, _ = run({"r1": error, "r2": finish, "r3": finish},
        limits=AsyncRuntimeLimits(max_ticks=30, poll_period_s=.002,
            decision_period_s=.05, max_concurrent_requests=1))
    assert result["peak_in_flight"] == 1
    assert result["actor_finish_claims"] == {"r1": False, "r2": True, "r3": True}
    assert result["calls"]["r1"] == 1


def test_private_split_fields_fail_closed_before_planner():
    result, _ = run(callbacks(), port=FixturePort(leak_field="test_split"))
    assert result["termination_reason"] == "RUNTIME_ERROR"
    assert result["planner_decisions"] == 0


def test_decision_throttle_does_not_throttle_physics_clock():
    def wait(request):
        return {"action": {"kind": "wait"}, "message": None}
    result, port = run(callbacks(wait), limits=AsyncRuntimeLimits(max_ticks=21,
        poll_period_s=.001, decision_period_s=.5))
    assert port.now_s == 1.
    assert all(2 <= calls <= 3 for calls in result["calls"].values())


def test_received_partner_retract_invalidates_inflight_reply():
    release = threading.Event()
    class TimedPort(FixturePort):
        def tick(self, now_s):
            if now_s >= .3:
                release.set()
            return super().tick(now_s)
    def sender(request):
        return {"action": {"kind": "wait"}, "message": {"recipients": ["r2"],
            "content": {"message_type": "retract", "task_id": "joint-old"}, "ttl_s": 1.}}
    def slow(request):
        release.wait(2)
        return finish(request)
    result, _ = run({"r1": sender, "r2": slow, "r3": finish}, port=TimedPort(),
        condition="structured", limits=AsyncRuntimeLimits(max_calls_per_robot=1,
            max_ticks=25, poll_period_s=.002, decision_period_s=.05))
    assert not result["actor_finish_claims"]["r2"]
    assert "OWN_EVIDENCE_CHANGED" in reasons(result)
    received = [e for e in result["events"] if e["event_type"] == "message_received"]
    assert [e["robot_id"] for e in received] == ["r2"]


def test_report_expiration_invalidates_reply_that_used_it():
    release = threading.Event()
    class TimedPort(FixturePort):
        def tick(self, now_s):
            if now_s >= .8:
                release.set()
            return super().tick(now_s)
    def sender(request):
        if request["request_id"].endswith("0001"):
            return {"action": {"kind": "wait"}, "message": {"recipients": ["r2"],
                "content": {"message_type": "accept", "task_id": "joint-old"}, "ttl_s": .35}}
        return finish(request)
    used = []
    def receiver(request):
        if request["memory"]["received_messages"]:
            used.append(copy.deepcopy(request))
            release.wait(2)
            return finish(request)
        return {"action": {"kind": "wait"}, "message": None}
    result, _ = run({"r1": sender, "r2": receiver, "r3": finish}, port=TimedPort(),
        condition="structured", limits=AsyncRuntimeLimits(max_calls_per_robot=4,
            max_ticks=30, poll_period_s=.002, decision_period_s=.15))
    assert used
    expired = [e for e in result["events"] if e["event_type"] == "message_expired"]
    assert expired
    assert "OWN_EVIDENCE_CHANGED" in reasons(result)
    assert not result["actor_finish_claims"]["r2"]


def test_actor_retract_and_partner_change_are_new_independent_requests():
    def negotiate(request):
        call = int(request["request_id"].split("-")[-1])
        if call == 2:
            return {"action": {"kind": "cancel_pending", "task_id": "old", "reason": "retract"},
                "message": {"recipients": ["r2"], "content": {"message_type": "retract",
                    "task_id": "old"}, "ttl_s": 1.}}
        if call >= 4:
            return finish(request)
        partner = "r2" if call == 1 else "r3"
        return {"action": {"kind": "task_request", "task_id": "old" if call == 1 else "new",
            "object_id": "box-1", "skill": "carry", "participants": ["r1", partner],
            "resources": ["box-1"], "stage": "carry", "own_role": "lower",
            "expires_at_s": request["observation"]["observed_at_s"] + 5},
            "message": {"recipients": [partner], "content": {"message_type":
                "help_request" if call == 1 else "partner_change"}, "ttl_s": 1.}}
    result, port = run({"r1": negotiate, "r2": finish, "r3": finish}, condition="structured")
    actions = [a for rid, a in port.submissions if rid == "r1"]
    assert [a["kind"] for a in actions] == ["task_request", "cancel_pending", "task_request"]
    assert actions[0]["participants"] == ["r1", "r2"]
    assert actions[2]["participants"] == ["r1", "r3"]
    assert len({a["decision_id"] for a in actions}) == 3
    assert all(a["requested_at_s"] < e["sim_time_s"] for a, e in zip(actions,
        [e for e in result["events"] if e["event_type"] == "action_submitted"]))


@pytest.mark.parametrize("terminal", [False, True])
def test_strict_skill_generation_allows_expected_progress_but_not_terminal_change(terminal):
    class SkillPort(FixturePort):
        def observe(self, rid):
            obs = super().observe(rid)
            obs["own_revision"] = int(terminal and self.now_s > 0)
            obs["own_issued_commands"] = [{"task_id": "t", "lease_id": "l",
                "command_id": f"pulse-{self.now_s}", "stage": "RUN", "action": {"kind": "hold"},
                "duration_s": .1, "issued_at_s": self.now_s, "meaning": "issued"}]
            return obs
        def local_status(self, rid):
            status = super().local_status(rid)
            status["own_revision"] = int(terminal and self.now_s > 0)
            status["own_skill_status"] = {"task_id": "t", "state":
                "FINISHED_UNVERIFIED" if terminal and self.now_s > 0 else "RUNNING",
                "phase": str(self.now_s), "meaning": "own RGB phase, not physical success"}
            return status
    result, _ = run(callbacks(), port=SkillPort(), limits=AsyncRuntimeLimits(
        max_calls_per_robot=1, max_ticks=20, poll_period_s=.002, decision_period_s=.05))
    assert all(result["actor_finish_claims"].values()) == (not terminal)
    if terminal:
        assert "OWN_EVIDENCE_CHANGED" in reasons(result)


@pytest.mark.parametrize("kwargs", [{"poll_period_s": float("nan")},
    {"max_concurrent_requests": True}, {"max_input_tokens": -1}, {"max_ticks": 0}])
def test_invalid_limits_rejected(kwargs):
    with pytest.raises(ValueError):
        replace(AsyncRuntimeLimits(), **kwargs).validate()
