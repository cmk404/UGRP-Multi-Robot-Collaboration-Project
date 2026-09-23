"""C3 transport-budget probes. Injected byte transport; no provider capability proof."""
import copy
from dataclasses import replace
import hashlib
import io
import json
from pathlib import Path
import threading

import pytest

from harness.rgb_communication_async import AsyncRuntimeLimits, RuntimeControl, run_rgb_communication_async
from harness.rgb_communication_planner import GeminiRGBPlanner, ProviderSettings, make_rgb_planners
from harness.rgb_communication_study import provider_settings
from harness.rgb_communication_evaluation import ContractError
from test_rgb_communication_planner import request, settings, response_bytes
from test_rgb_communication_runtime import FixturePort, FIXTURE


def run_transport(tmp_path, opener, *, port=None, control=None, limits=None):
    planners = make_rgb_planners(settings(), tmp_path, http_open=opener, evidence_kind="fixture")
    return run_rgb_communication_async(port or FixturePort(), planners,
        condition="none", common_task=FIXTURE["static_context"]["task"], control=control,
        limits=limits or AsyncRuntimeLimits(max_ticks=30, max_calls_per_robot=1,
            poll_period_s=.005, decision_period_s=1.))


@pytest.mark.parametrize("history_pairs", [0, 1, 3])
def test_input_bound_and_transport_see_identical_original_image_body(tmp_path, history_pairs):
    counted, sent = [], []
    def bound(body):
        counted.append(body)
        return 100  # Offline artificial bound, never registered with D resolver.
    def opener(req, **kwargs):
        sent.append(req.data)
        return io.BytesIO(response_bytes(req))
    data = request()
    data["memory"]["own_observations"] = []
    for index in range(4):
        previous = copy.deepcopy(data["observation"])
        previous["observation_id"] = f"r1-prior-{index}"
        data["memory"]["own_observations"].append(previous)
    planner = GeminiRGBPlanner("r1", replace(settings(), history_image_pairs=history_pairs,
        input_token_bound=bound), tmp_path, http_open=opener, evidence_kind="fixture")
    prepared = planner.prepare(data)
    result = planner.complete_prepared(prepared, threading.Event())
    assert counted == sent == [prepared.body]
    parts = json.loads(sent[0])["messages"][-1]["content"]
    images = [p["image_url"]["url"] for p in parts if p["type"] == "image_url"]
    assert images == ["data:image/jpeg;base64," + FIXTURE["jpeg_base64"]] * (2 + 2*history_pairs)
    assert result["artifacts"][".request.json"]["sha256"] == hashlib.sha256(counted[0]).hexdigest()
    assert result["external_model_calls"] == 0


@pytest.mark.parametrize("with_usage", [False, True])
def test_missing_usage_reserves_both_budgets_and_present_usage_is_measured(tmp_path, with_usage):
    attempts = []
    def opener(req, **kwargs):
        attempts.append(req.data)
        return io.BytesIO(response_bytes(req, usage=with_usage))
    result = run_transport(tmp_path, opener)
    assert len(attempts) == 3
    assert result["tokens"]["charged_input_tokens"] == (60 if with_usage else 300)
    assert result["tokens"]["charged_output_tokens"] == (36 if with_usage else 1536)
    for key in ("input_tokens", "output_tokens"):
        assert result["tokens"]["unknown_" + key + "_calls"] == (0 if with_usage else 3)
    assert result["external_model_calls"] == 0


def test_transport_timeout_is_not_retried_or_counted_as_zero_usage(tmp_path):
    attempts = []
    def opener(req, **kwargs):
        attempts.append(req.data)
        raise TimeoutError("OFFLINE_TIMEOUT_NOT_A_PROVIDER_RESPONSE")
    result = run_transport(tmp_path, opener)
    assert len(attempts) == 3
    assert result["outcome"] == "api_error"
    assert result["tokens"]["charged_input_tokens"] == 300
    assert result["tokens"]["charged_output_tokens"] == 1536
    assert result["tokens"]["unknown_input_tokens_calls"] == 3
    assert result["tokens"]["unknown_output_tokens_calls"] == 3
    assert len(list(tmp_path.glob("*/*.request.json"))) == 3
    assert not list(tmp_path.glob("*/*.response.json"))


@pytest.mark.parametrize("cancel", [False, True])
def test_pending_late_or_cancelled_transport_keeps_slot_and_reservation_after_close(tmp_path, cancel):
    release, drained = threading.Event(), threading.Event()
    peer_drained = {rid: threading.Event() for rid in ("r2", "r3")}
    class CapturePort(FixturePort):
        def tick(self, requested):
            if requested == .05:
                assert all(event.wait(1) for event in peer_drained.values())
            return super().tick(requested)
    control, port = RuntimeControl(), CapturePort()
    attempts = []
    def opener(req, **kwargs):
        wire = json.loads(req.data)
        actor = json.loads(wire["messages"][-1]["content"][0]["text"])["robot_id"]
        attempts.append(actor)
        if actor == "r1":
            if cancel:
                control.cancel_inference("r1")
            assert release.wait(2)
        return io.BytesIO(response_bytes(req))
    try:
        planners = make_rgb_planners(settings(), tmp_path, http_open=opener, evidence_kind="fixture")
        for rid, event in peer_drained.items():
            peer_original = planners[rid].complete_prepared
            def tracked_peer(*args, original=peer_original, completed=event):
                try:
                    return original(*args)
                finally:
                    completed.set()
            planners[rid].complete_prepared = tracked_peer
        original = planners["r1"].complete_prepared
        def tracked(*args):
            try:
                return original(*args)
            finally:
                drained.set()
        planners["r1"].complete_prepared = tracked
        result = run_rgb_communication_async(port, planners, condition="none",
            common_task=FIXTURE["static_context"]["task"], control=control,
            # Deadline expiry has separate tests. Here the pending/drain
            # boundary must not depend on archive I/O taking less than 10 ms.
            wall_clock=lambda: 0.,
            limits=AsyncRuntimeLimits(max_ticks=15, poll_period_s=.005,
                max_calls_per_robot=3, decision_period_s=1., decision_timeout_s=.01))
        assert sorted(attempts) == ["r1", "r2", "r3"]
        assert result["calls"] == {"r1": 1, "r2": 1, "r3": 1}
        assert result["pending_planner_requests"] == {"r1": "r1-request-0001"}
        assert result["tokens"]["charged_input_tokens"] == 140
        assert result["tokens"]["charged_output_tokens"] == 536
        assert result["tokens"]["unknown_input_tokens_calls"] == 1
        assert result["tokens"]["unknown_output_tokens_calls"] == 1
        assert port.now_s >= .5
        assert result["actor_finish_details"] == {"r1": None, "r2": "cannot_continue", "r3": "cannot_continue"}
        before = copy.deepcopy(port.submissions)
        release.set()
        assert drained.wait(1)
        assert port.submissions == before
        # Returning after runtime close does not retroactively alter its usage
        # snapshot. D must reap the trial process before freezing raw hashes.
        assert result["tokens"]["unknown_input_tokens_calls"] == 1
        assert (tmp_path / "r1/r1-request-0001.response.json").is_file()
    finally:
        release.set()
        drained.wait(1)


@pytest.mark.parametrize("field", ["input_bound_evidence", "output_limit_evidence", "input_token_bound"])
def test_json_cannot_assert_or_inject_a_verified_capability(field):
    with pytest.raises(ContractError):
        provider_settings({"model": "gemini-3.7-flash", field: "ASSERTED_NOT_PROVEN"})
    assert not provider_settings({"model": "gemini-3.7-flash"}).readiness()["ready"]
    assert not ProviderSettings().readiness()["network_probed"]


def test_planning_packet_is_blocked_and_pins_current_c_policy_without_approving_it():
    root = Path(__file__).resolve().parents[1]
    packet = json.loads((root / "docs/research_parallel/c3-provider-packet.json").read_text())
    assert packet["artifact_kind"] == "planning_only_not_execution_manifest"
    assert not any(packet[key] for key in ("ready", "execution_admission", "live_readiness",
                                          "model_selection_approved", "budget_proposal_approved"))
    assert packet["approved_model_calls"] == 0
    assert packet["selected_model"] is None
    assert packet["six_trial_budget_proposal"]["monetary_cost_cap"] is None
    candidate = provider_settings(packet["unapproved_candidate_provider_config"])
    assert not candidate.readiness()["ready"]
    assert candidate.policy_manifest() == packet["candidate_policy_manifest"]
    for name, expected in packet["base_component_sha256"].items():
        # B/D may advance in the subsequent integrated candidate. Their hashes
        # here are explicitly historical BASE locks, not current attestations.
        if name.rsplit("/", 1)[-1] in {"rgb_communication_planner.py", "rgb_communication_async.py",
                "rgb_communication_runtime.py", "rgb_communication_clock.py", "gemini_proxy.py"}:
            assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected
    scheduler = dict(packet["scheduler_proposal"])
    assert scheduler.pop("scheduler_id") == "rgb-independent-async.v1"
    limits = AsyncRuntimeLimits(**scheduler)
    limits.validate()
    per_trial, cohort = packet["per_trial_budget_proposal"], packet["six_trial_budget_proposal"]
    assert limits.max_calls_per_robot * 3 == per_trial["model_calls"]
    assert limits.max_input_tokens == per_trial["input_tokens"]
    assert limits.max_output_tokens == per_trial["output_tokens"]
    assert limits.max_ticks * limits.tick_period_s <= per_trial["sim_time_s"]
    assert limits.wall_timeout_s <= per_trial["wall_time_s"] - 5
    for key in ("model_calls", "input_tokens", "output_tokens"):
        assert cohort[key] == 6 * per_trial[key]
    assert {(p["scenario_role"], p["condition"]) for p in packet["proposed_trials"]} == {
        (scenario, condition) for scenario in ("normal", "competition")
        for condition in ("none", "structured", "natural")}
