"""Real serializer/transport adapter with a byte-recording, network-free opener."""
import copy
from dataclasses import replace
import hashlib
import io
import json
import threading

import pytest

from harness.rgb_communication_planner import (
    GeminiRGBPlanner, PlannerCallError, ProviderSettings, make_rgb_planners,
)
from harness.rgb_communication_async import AsyncRuntimeLimits, run_rgb_communication_async
from test_rgb_communication_runtime import FIXTURE, FixturePort


def request():
    port = FixturePort()
    return {"robot_id": "r1", "request_id": "r1-request-0001", "evidence_revision": "abc",
            "condition": "none", "observation": port.observe("r1"),
            "local_status": port.local_status("r1"), "memory": {"own_observations": [port.observe("r1")]}}


def settings():
    return ProviderSettings(input_token_bound=lambda body: 100,
        input_bound_evidence="OFFLINE_TEST_BOUND_NOT_PROVIDER_PROOF",
        output_limit_evidence="OFFLINE_TEST_CAP_NOT_PROVIDER_PROOF")


def response_bytes(req, *, usage=True, invalid=False):
    wire = json.loads(req.data)
    prompt = json.loads(wire["messages"][-1]["content"][0]["text"])
    decision = {"request_id": prompt["request_id"], "evidence_revision": prompt["evidence_revision"],
                "action": {"kind": "finish", "claim": "cannot_continue"}, "message": None}
    body = {"choices": [{"message": {"content": "bad JSON" if invalid else json.dumps(decision)}}],
            "model": "fixture-endpoint-model"}
    if usage:
        body["usage"] = {"prompt_tokens": 20, "completion_tokens": 12}
    return json.dumps(body).encode()


def test_current_provider_capability_is_not_assumed_or_probed(tmp_path):
    calls = []
    planner = GeminiRGBPlanner("r1", ProviderSettings(), tmp_path,
                              http_open=lambda *a, **k: calls.append(a))
    assert ProviderSettings().readiness()["blockers"] == [
        "UNVERIFIED_INPUT_TOKEN_BOUND", "UNVERIFIED_OUTPUT_TOKEN_CAP"]
    with pytest.raises(PlannerCallError, match="PROVIDER_BOUNDS_UNAVAILABLE"):
        planner.prepare(request())
    assert not calls
    assert not list(tmp_path.iterdir())


def test_exact_original_two_images_and_raw_request_response_are_preserved(tmp_path):
    observed = []
    def opener(req, **kwargs):
        raw = response_bytes(req)
        observed.append((req.data, raw))
        return io.BytesIO(raw)
    planner = GeminiRGBPlanner("r1", settings(), tmp_path, http_open=opener, evidence_kind="fixture")
    prepared = planner.prepare(request())
    result = planner.complete_prepared(prepared, threading.Event())
    wire = json.loads(observed[0][0])
    parts = wire["messages"][-1]["content"]
    images = [x["image_url"]["url"] for x in parts if x["type"] == "image_url"]
    assert images == ["data:image/jpeg;base64," + FIXTURE["jpeg_base64"]] * 2
    assert [p["text"] for p in parts if p["type"] == "text"][1:] == ["own_rgb", "top_rgb"]
    assert "jpeg_base64" not in parts[0]["text"]
    assert result["usage"] == {"input_tokens": 20, "output_tokens": 12}
    for suffix, original in zip((".request.json", ".response.json"), observed[0]):
        assert (tmp_path / (prepared.request_id + suffix)).read_bytes() == original
        assert result["artifacts"][suffix]["sha256"] == hashlib.sha256(original).hexdigest()
    assert wire["max_tokens"] == settings().max_output_tokens


def test_cancel_before_send_and_duplicate_never_call_network(tmp_path):
    calls = []
    def opener(req, **kwargs):
        calls.append(req)
        return io.BytesIO(response_bytes(req))
    planner = GeminiRGBPlanner("r1", settings(), tmp_path, http_open=opener)
    prepared = planner.prepare(request())
    stop = threading.Event()
    stop.set()
    with pytest.raises(PlannerCallError, match="CANCELLED_BEFORE_SEND"):
        planner.complete_prepared(prepared, stop)
    with pytest.raises(PlannerCallError, match="DUPLICATE_REQUEST"):
        planner.complete_prepared(prepared, threading.Event())
    assert not calls


def test_serializer_drift_blocks_before_network(tmp_path):
    calls = []
    planner = GeminiRGBPlanner("r1", settings(), tmp_path, http_open=lambda *a, **k: calls.append(a))
    prepared = replace(planner.prepare(request()), body=b"changed")
    with pytest.raises(PlannerCallError, match="SERIALIZED_REQUEST_CHANGED"):
        planner.complete_prepared(prepared, threading.Event())
    assert not calls


def test_invalid_decision_still_preserves_raw_and_usage(tmp_path):
    planner = GeminiRGBPlanner("r1", settings(), tmp_path,
        http_open=lambda req, **kw: io.BytesIO(response_bytes(req, invalid=True)))
    with pytest.raises(PlannerCallError) as caught:
        planner.complete_prepared(planner.prepare(request()), threading.Event())
    assert caught.value.reason == "INVALID_DECISION_JSON"
    assert caught.value.usage["input_tokens"] == 20
    assert ".response.json" in caught.value.artifacts


@pytest.mark.parametrize("with_usage", [False, True])
def test_three_independent_planners_async_real_serializer_no_network(tmp_path, with_usage):
    bodies = []
    def opener(req, **kwargs):
        bodies.append(req.data)
        return io.BytesIO(response_bytes(req, usage=with_usage))
    planners = make_rgb_planners(settings(), tmp_path / "requests", http_open=opener,
                                 evidence_kind="fixture")
    assert len({id(p) for p in planners.values()}) == 3
    result = run_rgb_communication_async(FixturePort(), planners, condition="none",
        common_task=FIXTURE["static_context"]["task"], artifact_dir=tmp_path / "images",
        trace_path=tmp_path / "trace.jsonl", limits=AsyncRuntimeLimits(max_ticks=40,
            poll_period_s=.003, decision_period_s=.05))
    assert result["outcome"] == "completed"
    assert len(bodies) == 3
    assert result["external_model_calls"] == 0  # intercepted transport, evidence=fixture
    assert not any(result["actor_finish_claims"].values())
    assert result["tokens"]["charged_input_tokens"] == (60 if with_usage else 300)
    assert result["tokens"]["unknown_input_tokens_calls"] == (0 if with_usage else 3)
    for rid in planners:
        assert len(list((tmp_path / "requests" / rid).glob("*.json"))) == 2


def test_token_budget_blocks_transport_before_io(tmp_path):
    calls = []
    planners = make_rgb_planners(settings(), tmp_path,
        http_open=lambda *a, **k: calls.append(a), evidence_kind="fixture")
    result = run_rgb_communication_async(FixturePort(), planners, condition="none",
        common_task=FIXTURE["static_context"]["task"], limits=AsyncRuntimeLimits(max_input_tokens=50))
    assert not calls
    assert result["external_model_calls"] == 0
    assert result["planner_decisions"] == 0
    assert result["termination_reason"] == "TOKEN_BUDGET_EXHAUSTED"


def test_temporal_images_are_real_bytes_in_wire_and_count_is_policy_locked(tmp_path):
    data = request()
    previous = copy.deepcopy(data["observation"])
    previous["observation_id"] = "r1-before"
    data["memory"]["own_observations"].insert(0, previous)
    planner = GeminiRGBPlanner("r1", settings(), tmp_path)
    prepared = planner.prepare(data)
    parts = json.loads(prepared.body)["messages"][-1]["content"]
    assert len([p for p in parts if p["type"] == "image_url"]) == 4
    assert [p["text"] for p in parts if p["type"] == "text"][-2:] == [
        "history:r1-before:own_rgb", "history:r1-before:top_rgb"]
    assert settings().policy_manifest()["policy_sha256"] != replace(
        settings(), history_image_pairs=0).policy_manifest()["policy_sha256"]


@pytest.mark.parametrize("target", ["top", "observation", "status", "memory", "previous"])
def test_direct_serializer_rejects_split_and_evaluator_injection(tmp_path, target):
    data = request()
    where = {"top": data, "observation": data["observation"],
             "status": data["local_status"], "memory": data["memory"],
             "previous": data["memory"]["own_observations"][0]}[target]
    where["test_split"] = "TEST_B_WITH_TRUTH"
    with pytest.raises(ValueError):
        GeminiRGBPlanner("r1", settings(), tmp_path).prepare(data)
