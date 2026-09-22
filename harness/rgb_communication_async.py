"""One clock owner and at most one unresolved planner call per robot.

Workers never receive an execution port, evaluator or shared actor memory.
Cancellation is logical: an in-flight transport may drain until its bounded
timeout, continues to occupy its slot/budget, and can never actuate after close.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any, Mapping, Sequence
import uuid

from harness import rgb_communication_runtime as base
from harness.rgb_communication_planner import PlannerCallError
from harness.rgb_communication_clock import ExecutionClock, sanitized_error


@dataclass(frozen=True)
class AsyncRuntimeLimits(base.RuntimeLimits):
    tick_period_s: float = .05
    poll_period_s: float = .05
    decision_period_s: float = 1.0
    max_concurrent_requests: int = 3
    max_observation_age_s: float = 5.0
    max_input_tokens: int = 120_000
    max_output_tokens: int = 12_000
    max_input_tokens_per_call: int = 10_000
    max_output_tokens_per_call: int = 512

    def validate(self):
        super().validate()
        for value in (self.max_concurrent_requests, self.max_input_tokens,
                      self.max_output_tokens, self.max_input_tokens_per_call,
                      self.max_output_tokens_per_call):
            if type(value) is not int or value < 1:
                raise ValueError("INVALID_ASYNC_LIMITS")
        if self.max_concurrent_requests > 3:
            raise ValueError("INVALID_ASYNC_CONCURRENCY")
        for value in (self.poll_period_s, self.max_observation_age_s, self.decision_period_s):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError("INVALID_ASYNC_LIMITS")


class RuntimeControl:
    """External stop, or cancellation of a robot's *inference*, not its lease.

Task cancellation/reallocation remains an independent actor action. Whole-run
stop closes the port and holds all owned tasks through its normal lifecycle.
"""
    def __init__(self):
        self.stop = threading.Event()
        self.cancellations: queue.Queue[str] = queue.Queue()

    def cancel_inference(self, robot_id: str):
        self.cancellations.put(robot_id)


class OfflineDecisionPlanner:
    """Network-free replay adapter. callback sees only its own immutable request."""
    external_calls_per_decision = 0
    model_name = "deterministic-replay"

    def __init__(self, callback, *, evidence_kind="fixture"):
        self.callback, self.evidence_kind = callback, evidence_kind

    def prepare(self, request):
        return _OfflineRequest(copy.deepcopy(request))

    def complete_prepared(self, prepared, cancel):
        if cancel.is_set():
            raise PlannerCallError("CANCELLED_BEFORE_SEND")
        request = copy.deepcopy(prepared.request)
        reply = self.callback(request)
        return {"request_id": request["request_id"],
                "evidence_revision": request["evidence_revision"],
                **reply, "usage": {"input_tokens": 0, "output_tokens": 0},
                "external_model_calls": 0}


@dataclass(frozen=True)
class _OfflineRequest:
    request: dict
    input_token_bound: int = 0
    output_token_bound: int = 0


class TokenLedger:
    """Reserve proven worst-case cost before send; unknown usage stays reserved."""
    def __init__(self, limits):
        self.limits = limits
        self.entries = {}

    def reserve(self, request_id, prepared):
        bounds = (prepared.input_token_bound, prepared.output_token_bound)
        if any(type(v) is not int or v < 0 for v in bounds):
            raise PlannerCallError("INVALID_TOKEN_BOUND")
        caps = (self.limits.max_input_tokens_per_call, self.limits.max_output_tokens_per_call)
        if any(v > cap for v, cap in zip(bounds, caps)):
            raise PlannerCallError("PER_CALL_TOKEN_BUDGET_EXHAUSTED")
        totals = self.summary()
        if (totals["charged_input_tokens"] + bounds[0] > self.limits.max_input_tokens
                or totals["charged_output_tokens"] + bounds[1] > self.limits.max_output_tokens):
            raise PlannerCallError("TOKEN_BUDGET_EXHAUSTED")
        self.entries[request_id] = {"bounds": bounds, "measured": [None, None]}

    def settle(self, request_id, usage):
        entry = self.entries[request_id]
        violated = False
        for i, key in enumerate(("input_tokens", "output_tokens")):
            value = usage.get(key) if isinstance(usage, Mapping) else None
            if type(value) is int and value >= 0:
                entry["measured"][i] = value
                violated |= value > entry["bounds"][i]
        return violated

    def summary(self):
        result = {}
        for i, key in enumerate(("input_tokens", "output_tokens")):
            result["measured_" + key] = sum(e["measured"][i] or 0 for e in self.entries.values())
            result["unknown_" + key + "_calls"] = sum(e["measured"][i] is None
                                                      for e in self.entries.values())
            result["charged_" + key] = sum(e["bounds"][i] if e["measured"][i] is None
                                          else e["measured"][i] for e in self.entries.values())
        return result


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False, separators=(",", ":")).encode()).hexdigest()


def _state_key(observation, status, actor, epoch):
    local = copy.deepcopy({k: v for k, v in status.items() if k != "timestamp_s"})
    strict = "own_revision" in status
    # B2's revision is the high-level consent/task generation. Expected servo
    # interpolation within that consent must not starve all interrupt/pause
    # decisions. Terminal skill changes still invalidate, as do all new manual
    # commands, consent changes, pause/resume and lease lifecycle transitions.
    skill = local.get("own_skill_status", {})
    if strict and skill.get("state") == "RUNNING":
        skill.pop("phase", None)
    return _digest({"commands": None if strict else observation["own_issued_commands"],
                    "status": local,
                    "messages": actor.received_messages, "epoch": epoch})


@dataclass
class _Flight:
    request: dict
    state_key: str
    started: float
    cancel: threading.Event
    external_calls: int
    invalidated: str | None = None


def run_rgb_communication_async(
    port, planners: Mapping[str, Any], *, condition: str, common_task: Mapping[str, Any],
    run_id: str | None = None, robot_ids: Sequence[str] = base.DEFAULT_ROBOTS,
    limits: AsyncRuntimeLimits | None = None, trace_path: str | Path | None = None,
    artifact_dir: str | Path | None = None, control: RuntimeControl | None = None,
    provenance: Mapping[str, Any] | None = None, wall_clock=time.monotonic,
    clock_snapshot=None,
) -> dict[str, Any]:
    """Bounded async scheduler, common to none/structured/natural conditions.

provenance is trace-only (e.g. split/seed/source/prompt/controller hashes), never
an actor input. tick_period is SIM advancement; poll_period is minimum wall
pacing. One tick occurs per loop, so slow capture/prepare never causes a burst
of catch-up steps. Port tick/observe/submit must themselves be bounded.
clock_snapshot is a supervisor-only value callback for terminal/close evidence;
it must not step/render/read an evaluator, and is never given to a planner.
"""
    limits = limits or AsyncRuntimeLimits()
    limits.validate()
    if getattr(port, "clock_domain", None) != "sim":
        raise ValueError("ASYNC_REQUIRES_SIM_CLOCK")
    robot_ids = tuple(robot_ids)
    if condition not in base.CONDITIONS or len(robot_ids) != 3 or len(set(robot_ids)) != 3:
        raise ValueError("INVALID_CONDITION_OR_ROBOTS")
    if set(planners) != set(robot_ids) or len({id(p) for p in planners.values()}) != 3:
        raise ValueError("THREE_INDEPENDENT_PLANNERS_REQUIRED")
    if any(not callable(getattr(p, "prepare", None)) or
           not callable(getattr(p, "complete_prepared", None)) for p in planners.values()):
        raise ValueError("BOUNDED_PREPARED_PLANNERS_REQUIRED")
    run_id, control = run_id or uuid.uuid4().hex, control or RuntimeControl()
    common_task = copy.deepcopy(dict(common_task))
    started, ids = wall_clock(), base._IdFactory()
    trace = base.TraceRecorder(run_id=run_id, condition=condition, wall_clock=wall_clock,
                               wall_started_s=started, path=trace_path, ids=ids)
    actors = {rid: base._ActorState(rid) for rid in robot_ids}
    epochs, retired = {rid: 0 for rid in robot_ids}, set()
    retirement_reasons = {}
    next_decision_at = {rid: 0. for rid in robot_ids}
    channel = base.MessageChannel(condition, robot_ids, limits, trace, ids)
    flights, responses = {}, queue.Queue()
    ledger = TokenLedger(limits)
    now_s, ticks, peak = 0., 0, 0
    clock = ExecutionClock(port.clock_domain, clock_snapshot)
    primary_error = close_error = None
    outcome, reason = "aborted", "MAX_TICKS"
    external_calls = {rid: 0 for rid in robot_ids}
    measured_external_calls = {rid: 0 for rid in robot_ids}
    unknown_external_calls = set()
    static_hash = None
    trace.emit("run_started", robot_id=None, sim_time_s=0., payload={
        "robot_ids": list(robot_ids), "scheduler": "async_single_clock",
        "planner_models": {rid: p.model_name for rid, p in planners.items()},
        "evidence_kinds": {rid: p.evidence_kind for rid, p in planners.items()},
        "limits": copy.deepcopy(limits.__dict__), "provenance": copy.deepcopy(provenance or {}),
        "measured_metrics": ["high_level_actions", "messages", "message_bytes",
                             "planner_decisions", "external_model_calls", "model_response_time_s"]})

    def emit(kind, rid=None, payload=None, related=None, *, supervisor_clock=False):
        sim_time = clock.sim_time if supervisor_clock else now_s
        return trace.emit(kind, robot_id=rid, sim_time_s=sim_time,
                          payload=payload, related_ids=related)

    def capture(rid):
        nonlocal static_hash
        obs = base._copy_allowlisted(port.observe(rid), base.OBSERVATION_FIELDS,
                                     tuple(base.OBSERVATION_FIELDS - {"own_revision"}))
        recorded = base._validate_observation(obs, robot_id=rid,
                    artifact_dir=Path(artifact_dir) if artifact_dir is not None else None)
        status = base._copy_allowlisted(port.local_status(rid), base.LOCAL_STATUS_FIELDS,
                                        tuple(base.LOCAL_STATUS_FIELDS - {"own_revision", "own_skill_status"}))
        base._validate_local_status(status, robot_id=rid)
        base._clock_time(obs, status)
        if (obs.get("own_revision") != status.get("own_revision") or
                common_task != obs["static_context"]["task"]):
            raise ValueError("INCONSISTENT_ACTOR_CAPTURE")
        if static_hash is not None and static_hash != obs["static_context_sha256"]:
            raise ValueError("STATIC_CONTEXT_CHANGED")
        static_hash = obs["static_context_sha256"]
        return obs, status, recorded

    def worker(rid, planner, prepared, flight):
        try:
            reply, error = planner.complete_prepared(prepared, flight.cancel), None
        except Exception as exc:
            reply, error = None, exc
        responses.put((rid, flight.request["request_id"], reply, error, wall_clock()))

    try:
        for ticks in range(1, limits.max_ticks + 1):
            loop_started = wall_clock()
            if control.stop.is_set():
                reason = "CANCELLED"
                break
            if loop_started - started >= limits.wall_timeout_s:
                outcome, reason = "timeout", "WALL_TIMEOUT"
                break
            requested_tick_s = (ticks - 1) * limits.tick_period_s
            # Supervisor return is intentionally discarded, not used to wake.
            clock.tick(port, requested_tick_s)
            now_s = requested_tick_s
            while not control.cancellations.empty():
                rid = control.cancellations.get_nowait()
                if rid in epochs:
                    epochs[rid] += 1
                    if rid in flights:
                        flights[rid].cancel.set()
                        flights[rid].invalidated = "CANCELLED_REPLY"
            for rid, actor in actors.items():
                base._expire_actor_messages(actor, clock_time_s=now_s,
                    trace_sim_time_s=now_s, trace=trace)
                received, expired = channel.receive(rid, clock_time_s=now_s,
                    trace_sim_time_s=now_s, tick_index=ticks)
                actor.received_messages.extend(received)
                actor.expired_messages.extend(expired)
                flight = flights.get(rid)
                if flight and wall_clock() - flight.started >= limits.decision_timeout_s:
                    if flight.invalidated is None:
                        flight.invalidated = "LATE_REPLY"
                        flight.cancel.set()
                        emit("planner_request_cancelled", rid, {"reason": "LATE_REPLY"},
                             {"request_id": flight.request["request_id"]})

            while not responses.empty():
                rid, req_id, reply, error, ended = responses.get_nowait()
                flight = flights.get(rid)
                if flight is None or flight.request["request_id"] != req_id:
                    emit("planner_response_rejected", rid, {"reason": "DUPLICATE_REPLY"},
                         {"request_id": req_id})
                    continue
                del flights[rid]
                actor, request = actors[rid], flight.request
                usage = (reply.get("usage", {}) if isinstance(reply, Mapping)
                         else getattr(error, "usage", {}))
                sent = (reply.get("external_model_calls") if isinstance(reply, Mapping)
                        else getattr(error, "external_model_calls", None))
                if type(sent) is int and 0 <= sent <= flight.external_calls:
                    measured_external_calls[rid] += sent
                    unknown_external_calls.discard(req_id)
                breached = ledger.settle(req_id, usage)
                decision_id = ids.new("decision")
                decision = {"request_id": req_id, "decision_id": decision_id,
                            "response_latency_s": max(0., ended-flight.started),
                            "artifacts": copy.deepcopy(reply.get("artifacts", {})
                                if isinstance(reply, Mapping) else getattr(error, "artifacts", {}))}
                if isinstance(usage, Mapping):
                    decision.update({k: usage[k] for k in ("input_tokens", "output_tokens")
                                     if type(usage.get(k)) is int and usage[k] >= 0})
                if isinstance(reply, Mapping):
                    decision.update({k: copy.deepcopy(reply.get(k))
                                     for k in ("action", "message", "raw_text")})
                emit("planner_responded", rid, decision,
                     {"request_id": req_id, "decision_id": decision_id,
                      "observation_id": request["observation"]["observation_id"]})
                reject = flight.invalidated
                if breached:
                    reason = "PROVIDER_TOKEN_BOUND_VIOLATION"
                    raise PlannerCallError(reason)
                if error:
                    reject = reject or getattr(error, "reason", "PLANNER_ERROR")
                    # No implicit retries of failed provider transports.
                    if flight.invalidated is None:
                        retired.add(rid)
                        retirement_reasons[rid] = "API_ERROR"
                elif not isinstance(reply, Mapping):
                    reject = "INVALID_PLANNER_REPLY"
                elif (reply.get("request_id") != req_id or
                      reply.get("evidence_revision") != request["evidence_revision"]):
                    reject = "REPLY_CORRELATION_MISMATCH"
                elif ended-flight.started >= limits.decision_timeout_s:
                    reject = "LATE_REPLY"
                elif wall_clock()-started >= limits.wall_timeout_s:
                    reject = "WALL_TIMEOUT_REPLY"
                elif now_s-request["observation"]["observed_at_s"] > limits.max_observation_age_s:
                    reject = "STALE_OBSERVATION"
                if reject is None:
                    obs, status, _ = capture(rid)
                    if _state_key(obs, status, actor, epochs[rid]) != flight.state_key:
                        reject = "OWN_EVIDENCE_CHANGED"
                if reject:
                    emit("planner_response_rejected", rid, {"reason": reject},
                         {"request_id": req_id, "decision_id": decision_id})
                    continue
                actor.decisions.append(decision)
                proposal = reply.get("action")
                action_id = ids.new("action")
                try:
                    action = base._normalize_action(proposal, robot_id=rid,
                        robot_ids=robot_ids, observation_id=request["observation"]["observation_id"],
                        decision_id=decision_id, action_id=action_id, now_s=now_s
                    ) if proposal is not None else {"kind": "wait"}
                except ValueError as exc:
                    emit("action_rejected", rid, {"reason": str(exc)},
                         {"decision_id": decision_id, "action_id": action_id})
                    continue
                channel.send(rid, reply.get("message"), decision_id=decision_id,
                    clock_time_s=now_s, trace_sim_time_s=now_s, tick_index=ticks, actor=actor)
                if action["kind"] == "finish":
                    actor.finished, actor.finish_claim = True, action["claim"]
                    emit("actor_finished", rid, {"claim": action["claim"],
                         "meaning": "unverified_actor_claim"}, {"decision_id": decision_id,
                         "action_id": action_id, "observation_id": request["observation"]["observation_id"]})
                elif action["kind"] != "wait":
                    if actor.actions >= limits.max_actions_per_robot:
                        emit("budget_exhausted", rid, {"budget": "actions"})
                        continue
                    # Do not refresh the original capture time to launder staleness.
                    action["requested_at_s"] = request["observation"]["observed_at_s"]
                    if "own_revision" in request["local_status"]:
                        action["expected_revision"] = request["local_status"]["own_revision"]
                    related = {"decision_id": decision_id, "action_id": action_id,
                               "observation_id": request["observation"]["observation_id"]}
                    emit("action_submitted", rid, action, related)
                    actor.actions += 1
                    result = base._validate_submission_result(port.submit(rid, copy.deepcopy(action)),
                        expected_clock_domain=request["local_status"]["clock_domain"])
                    actor.action_results.append({"action_id": action_id, "result": result})
                    emit("action_result", rid, result, related)
                    epochs[rid] += 1

            if all(a.finished for a in actors.values()):
                outcome, reason = "completed", "ALL_ACTORS_FINISHED"
                break
            # Rotation gives equal admission when concurrency is below three.
            ordered = robot_ids[(ticks-1) % 3:] + robot_ids[:(ticks-1) % 3]
            for rid in ordered:
                actor = actors[rid]
                if (rid in flights or rid in retired or actor.finished or
                        actor.calls >= limits.max_calls_per_robot or
                        now_s < next_decision_at[rid] or
                        len(flights) >= limits.max_concurrent_requests):
                    continue
                obs, status, recorded = capture(rid)
                actor.observations.append(obs)
                actor.trace_observations.append(recorded)
                actor.statuses.append(status)
                actor.observations[:] = actor.observations[-4:]
                actor.trace_observations[:] = actor.trace_observations[-4:]
                actor.statuses[:] = actor.statuses[-4:]
                actor.decisions[:] = actor.decisions[-8:]
                actor.action_results[:] = actor.action_results[-8:]
                req_id = f"{rid}-request-{actor.calls+1:04d}"
                state_key = _state_key(obs, status, actor, epochs[rid])
                request = {"schema_version": base.REQUEST_SCHEMA_VERSION, "run_id": run_id,
                    "request_id": req_id, "robot_id": rid, "condition": condition,
                    "common_task": common_task, "observation": obs, "local_status": status,
                    "memory": actor.memory_snapshot(), "evidence_revision": _digest(
                        {"state": state_key, "observation_id": obs["observation_id"],
                         "own_issued_commands": obs["own_issued_commands"],
                         "images": {k: v["sha256"] for k, v in obs["images"].items()}}),
                    "budget": {"calls_remaining": limits.max_calls_per_robot-actor.calls,
                        "actions_remaining": limits.max_actions_per_robot-actor.actions,
                        "messages_remaining": limits.max_messages_per_robot-actor.sent_messages,
                        "message_bytes_remaining": limits.max_message_bytes_per_robot-actor.sent_message_bytes,
                        "max_message_ttl_s": limits.max_message_ttl_s}}
                # Human-readable run labels can themselves disclose test splits.
                request["run_id"] = _digest({"actor_run": run_id})
                try:
                    prepared = planners[rid].prepare(copy.deepcopy(request))
                    calls = getattr(planners[rid], "external_calls_per_decision", 1)
                    if calls not in (0, 1) or (calls == 0 and planners[rid].evidence_kind == "live_llm"):
                        raise PlannerCallError("INVALID_EXTERNAL_CALL_DECLARATION")
                    if calls and (prepared.input_token_bound < 1 or prepared.output_token_bound < 1):
                        raise PlannerCallError("MISSING_LIVE_TOKEN_BOUND")
                    ledger.reserve(req_id, prepared)
                except PlannerCallError as exc:
                    retired.add(rid)
                    retirement_reasons[rid] = exc.reason
                    emit("planner_admission_rejected", rid, {"reason": exc.reason},
                         {"request_id": req_id})
                    continue
                emit("observation_captured", rid, recorded, {"observation_id": obs["observation_id"]})
                emit("local_status_read", rid, status)
                traced = copy.deepcopy(request)
                traced["observation"] = recorded
                traced["memory"]["own_observations"] = actor.trace_observations[-4:]
                traced["external_model_calls"] = calls
                emit("planner_requested", rid, traced, {"request_id": req_id,
                     "observation_id": obs["observation_id"]})
                actor.calls += 1
                next_decision_at[rid] = now_s + limits.decision_period_s
                external_calls[rid] += calls
                if calls:
                    unknown_external_calls.add(req_id)
                flight = _Flight(request, state_key, wall_clock(), threading.Event(), calls)
                flights[rid] = flight
                threading.Thread(target=worker, args=(rid, planners[rid], prepared, flight),
                                 name=f"rgb-planner-{rid}", daemon=True).start()
                peak = max(peak, len(flights))
            if not flights and all(a.finished or rid in retired or a.calls >= limits.max_calls_per_robot
                                   for rid, a in actors.items()):
                reasons = set(retirement_reasons.values())
                if "API_ERROR" in reasons:
                    outcome, reason = "api_error", "API_ERROR"
                elif any("TOKEN_BUDGET_EXHAUSTED" in item for item in reasons):
                    reason = "TOKEN_BUDGET_EXHAUSTED"
                elif "PROVIDER_BOUNDS_UNAVAILABLE" in reasons:
                    reason = "PROVIDER_BOUNDS_UNAVAILABLE"
                else:
                    reason = "ADMISSION_REJECTED" if retired else "CALL_BUDGET_EXHAUSTED"
                break
            remaining = min(limits.poll_period_s-(wall_clock()-loop_started),
                            limits.wall_timeout_s-(wall_clock()-started))
            if remaining > 0:
                control.stop.wait(remaining)
    except Exception as exc:
        # The sole PlannerCallError raised to this scope has this fixed code.
        reason = ("PROVIDER_TOKEN_BOUND_VIOLATION" if isinstance(exc, PlannerCallError)
                  and exc.reason == "PROVIDER_TOKEN_BOUND_VIOLATION" else "RUNTIME_ERROR")
        outcome = "api_error" if isinstance(exc, PlannerCallError) else "aborted"
        primary_error = sanitized_error(exc)
        clock.refresh()
        emit("run_error", payload={"reason": reason, "phase": "runtime",
             **primary_error, "clock": clock.payload()}, supervisor_clock=True)
    finally:
        for flight in flights.values():
            flight.cancel.set()
        clock.refresh()
        try:
            port.close(clock.close_argument(failed=primary_error is not None))
        except Exception as exc:
            close_error = sanitized_error(exc)
            if primary_error is None:
                outcome, reason = "aborted", "CLOSE_ERROR"
            clock.refresh()
            emit("run_error", payload={"reason": "CLOSE_ERROR", "phase": "close",
                 **close_error, "clock": clock.payload()}, supervisor_clock=True)
        clock.refresh()
    if reason == "MAX_TICKS":
        outcome = "timeout"
    result = {"run_id": run_id, "condition": condition, "outcome": outcome,
              "termination_reason": reason, "ticks": ticks,
              "clock": clock.payload(), "primary_error": primary_error, "close_error": close_error,
              "actor_finish_claims": {rid: a.finish_claim == "mission_complete" for rid, a in actors.items()},
              "actor_finish_details": {rid: a.finish_claim for rid, a in actors.items()},
              "calls": {rid: a.calls for rid, a in actors.items()},
              "planner_decisions": sum(a.calls for a in actors.values()),
              "external_model_calls": sum(measured_external_calls.values()),
              "external_model_calls_by_robot": measured_external_calls,
              "external_model_call_upper_bound": sum(external_calls.values()),
              "unknown_external_call_requests": sorted(unknown_external_calls),
              "actions": {rid: a.actions for rid, a in actors.items()},
              "messages": {rid: a.sent_messages for rid, a in actors.items()},
              "message_bytes": {rid: a.sent_message_bytes for rid, a in actors.items()},
              "peak_in_flight": peak, "pending_planner_requests": {
                  rid: f.request["request_id"] for rid, f in flights.items()},
              "tokens": ledger.summary(), "trace_events": len(trace.events)+1}
    result["retirement_reasons"] = retirement_reasons
    emit("run_finished", payload=result, supervisor_clock=True)
    return {**result, "events": copy.deepcopy(trace.events)}
