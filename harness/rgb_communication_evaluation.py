"""Offline contracts and aggregation for the RGB communication pilot.

This module never starts a simulator, robot, or model.  It freezes a finite
schedule, validates runtime/evaluator artifacts, and keeps every planned run in
the denominator.  Runtime events contain controller-visible telemetry while
the evaluator artifact is deliberately separate and may contain referee data.
"""
from __future__ import annotations

from collections import Counter, defaultdict
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import random
import re
from statistics import median
from typing import Any, Iterable


MANIFEST_SCHEMA = "ugrp.rgb_communication_pilot_manifest.v1"
EVENT_SCHEMA = "rgb-communication-event.v1"
EVALUATOR_SCHEMA = "rgb-communication-evaluator.v1"
REPORT_SCHEMA = "ugrp.rgb_communication_evaluation_report.v1"

CONDITIONS = ("none", "structured", "natural")
ROBOTS = ("r1", "r2", "r3")
TERMINAL_OUTCOMES = (
    "success",
    "failure",
    "timeout",
    "api_error",
    "aborted",
)
REPORT_OUTCOMES = TERMINAL_OUTCOMES + (
    "unrun",
    "missing_artifact",
    "invalid_artifact",
)
RUNTIME_OUTCOMES = TERMINAL_OUTCOMES + ("completed",)
COUNT_METRICS = {
    "high_level_actions": "action_submitted",
    "messages": "message_sent",
    "model_calls": "planner_requested",
    "planner_decisions": "planner_requested",
}
SUM_METRICS = {
    "external_model_calls": ("planner_requested", "external_model_calls"),
    "message_bytes": ("message_sent", "bytes"),
    "message_tokens": ("message_sent", "token_count"),
    "input_tokens": ("planner_responded", "input_tokens"),
    "output_tokens": ("planner_responded", "output_tokens"),
    "model_response_time_s": ("planner_responded", "response_latency_s"),
}
RUNTIME_MEASUREMENT_NAMES = tuple(COUNT_METRICS) + tuple(SUM_METRICS)
MEASUREMENT_NAMES = RUNTIME_MEASUREMENT_NAMES + ("commands",)
SAFE_ID = re.compile(r"^[A-Za-z0-9._-]+$")


class ContractError(ValueError):
    """A manifest or evidence artifact does not satisfy the frozen contract."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _require_positive_number(value: Any, label: str) -> None:
    if not _is_number(value) or value <= 0:
        raise ContractError(f"{label} must be a positive number")


def _require_sha256(value: Any, label: str) -> None:
    if not isinstance(value, str) or len(value) != 64:
        raise ContractError(f"{label} must be a SHA-256 hex digest")
    try:
        int(value, 16)
    except ValueError as exc:
        raise ContractError(f"{label} must be a SHA-256 hex digest") from exc


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()


def build_manifest(protocol: dict[str, Any], *, source_sha: str, source_clean: bool) -> dict[str, Any]:
    """Build a deterministic two-scenario x three-condition pilot schedule."""
    scenarios = deepcopy(protocol.get("scenarios"))
    if not isinstance(scenarios, list) or len(scenarios) != 2:
        raise ContractError("the connection pilot requires exactly two scenarios")
    scenario_ids = [item.get("id") for item in scenarios if isinstance(item, dict)]
    if len(scenario_ids) != 2 or len(set(scenario_ids)) != 2 or not all(
        isinstance(value, str) and SAFE_ID.fullmatch(value) for value in scenario_ids
    ):
        raise ContractError("scenario ids must be two unique path-safe strings")

    mission = deepcopy(protocol.get("mission"))
    cargo = mission.get("cargo") if isinstance(mission, dict) else None
    object_ids = [item.get("object_id") for item in cargo if isinstance(item, dict)] \
        if isinstance(cargo, list) else []
    if not object_ids or len(object_ids) != len(set(object_ids)) or not all(
        isinstance(value, str) and SAFE_ID.fullmatch(value) for value in object_ids
    ):
        raise ContractError("mission cargo requires unique path-safe object ids")

    seed = protocol.get("seed")
    repeat = protocol.get("repeat", 0)
    randomization_seed = protocol.get("randomization_seed")
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise ContractError("seed must be an integer")
    if not isinstance(repeat, int) or repeat < 0:
        raise ContractError("repeat must be a non-negative integer")
    if not isinstance(randomization_seed, int) or isinstance(randomization_seed, bool):
        raise ContractError("randomization_seed must be an integer")

    schedule = [
        {
            "run_id": f"{scenario_id}--{condition}--s{seed}--r{repeat}",
            "scenario_id": scenario_id,
            "condition": condition,
            "seed": seed,
            "repeat": repeat,
            "object_ids": list(object_ids),
        }
        for scenario_id in scenario_ids
        for condition in CONDITIONS
    ]
    random.Random(randomization_seed).shuffle(schedule)
    for order, row in enumerate(schedule, start=1):
        row.update(
            order=order,
            artifact_relpath=f"runs/{row['run_id']}",
            required_artifacts={
                "runtime": "runtime.jsonl",
                "evaluator": "evaluator.json",
            },
        )

    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "scope": "provisional_rgb_communication_connection_pilot",
        "research_question": protocol.get("research_question"),
        "mission": mission,
        "stop_conditions": deepcopy(protocol.get("stop_conditions")),
        "statistical_claim": protocol.get("statistical_claim"),
        "conditions": list(CONDITIONS),
        "scenarios": scenarios,
        "seed": seed,
        "repeat": repeat,
        "randomization_seed": randomization_seed,
        "planned_denominator": len(schedule),
        "source": {
            "git_sha": source_sha,
            "clean": bool(source_clean),
        },
        "fixed_inputs": deepcopy(protocol.get("fixed_inputs")),
        "budgets": deepcopy(protocol.get("budgets")),
        "readiness": deepcopy(protocol.get("readiness")),
        "measurement_contract": {
            "runtime_event_schema": EVENT_SCHEMA,
            "evaluator_schema": EVALUATOR_SCHEMA,
            "optional_metrics": list(MEASUREMENT_NAMES),
            "metric_sources": {
                "commands": "evaluator.source_snapshot.coordination_audit unique LOCAL_COMMAND command_id",
                "high_level_actions": "runtime action_submitted event count",
                "messages": "runtime message_sent event count",
                "model_calls": "runtime planner_requested event count",
            },
            "unmeasured_value": None,
        },
        "schedule": schedule,
        "execution": {
            "strategy": "sequential_finite_schedule",
            "parallel_runs": 1,
            "automatic_retry": False,
            "stop_after_planned_runs": len(schedule),
        },
    }
    manifest["protocol_sha256"] = hashlib.sha256(_canonical_json(protocol)).hexdigest()
    validate_manifest(manifest)
    return manifest


def validate_manifest(manifest: dict[str, Any]) -> None:
    if manifest.get("schema_version") != MANIFEST_SCHEMA:
        raise ContractError("unsupported manifest schema")
    if manifest.get("conditions") != list(CONDITIONS):
        raise ContractError("conditions must be none, structured, natural in that order")
    scenarios = manifest.get("scenarios")
    if not isinstance(scenarios, list) or len(scenarios) != 2:
        raise ContractError("manifest must contain exactly two scenarios")
    scenario_ids = [item.get("id") for item in scenarios if isinstance(item, dict)]
    if (len(scenario_ids) != 2 or len(set(scenario_ids)) != 2
            or not all(isinstance(value, str) and SAFE_ID.fullmatch(value) for value in scenario_ids)):
        raise ContractError("scenario ids must be unique")

    mission = manifest.get("mission")
    if not isinstance(mission, dict):
        raise ContractError("mission must be an object")
    cargo = mission.get("cargo")
    object_ids = [item.get("object_id") for item in cargo if isinstance(item, dict)] \
        if isinstance(cargo, list) else []
    if not object_ids or len(object_ids) != len(set(object_ids)) or not all(
        isinstance(value, str) and SAFE_ID.fullmatch(value) for value in object_ids
    ):
        raise ContractError("mission cargo requires unique path-safe object ids")

    schedule = manifest.get("schedule")
    if not isinstance(schedule, list) or len(schedule) != 6:
        raise ContractError("manifest must contain exactly six planned runs")
    if manifest.get("planned_denominator") != len(schedule):
        raise ContractError("planned_denominator does not match the schedule")
    run_ids: set[str] = set()
    cells: set[tuple[Any, ...]] = set()
    orders: list[int] = []
    for row in schedule:
        if not isinstance(row, dict):
            raise ContractError("schedule rows must be objects")
        run_id = row.get("run_id")
        cell = (row.get("scenario_id"), row.get("condition"), row.get("seed"), row.get("repeat"))
        if not isinstance(run_id, str) or not run_id:
            raise ContractError("each run requires a run_id")
        if run_id in run_ids or cell in cells:
            raise ContractError("duplicate planned run")
        if row.get("scenario_id") not in scenario_ids or row.get("condition") not in CONDITIONS:
            raise ContractError("schedule row references an unknown scenario or condition")
        if not isinstance(row.get("order"), int):
            raise ContractError("schedule order must be an integer")
        required = row.get("required_artifacts")
        if required != {"runtime": "runtime.jsonl", "evaluator": "evaluator.json"}:
            raise ContractError("required artifacts must be runtime.jsonl and evaluator.json")
        expected_run_id = (f"{row.get('scenario_id')}--{row.get('condition')}--"
                           f"s{row.get('seed')}--r{row.get('repeat')}")
        if run_id != expected_run_id or row.get("artifact_relpath") != f"runs/{run_id}":
            raise ContractError("run_id or artifact path does not match the planned cell")
        if row.get("object_ids") != object_ids:
            raise ContractError("schedule object_ids do not match the mission")
        run_ids.add(run_id)
        cells.add(cell)
        orders.append(row["order"])
    expected_cells = {(scenario, condition, manifest.get("seed"), manifest.get("repeat"))
                      for scenario in scenario_ids for condition in CONDITIONS}
    if cells != expected_cells or sorted(orders) != list(range(1, 7)):
        raise ContractError("schedule is not a complete 2 x 3 randomized block")

    source = manifest.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("git_sha"), str) or not source["git_sha"]:
        raise ContractError("source.git_sha is required")
    if not isinstance(source.get("clean"), bool):
        raise ContractError("source.clean must be boolean")
    if not isinstance(manifest.get("fixed_inputs"), dict):
        raise ContractError("fixed_inputs must be an object")
    stop_conditions = manifest.get("stop_conditions")
    if not isinstance(stop_conditions, list) or not stop_conditions or not all(
        isinstance(value, str) and value for value in stop_conditions
    ):
        raise ContractError("stop_conditions must be a non-empty string list")
    if manifest.get("statistical_claim") != "none; integration and measurability check only":
        raise ContractError("the six-run connection pilot cannot make a statistical claim")
    if not isinstance(manifest.get("budgets"), dict):
        raise ContractError("budgets must be an object")
    if not isinstance(manifest.get("readiness"), dict):
        raise ContractError("readiness must be an object")
    try:
        _require_sha256(manifest.get("protocol_sha256"), "protocol_sha256")
    except ContractError as exc:
        raise ContractError("protocol_sha256 is invalid") from exc


def execution_blockers(manifest: dict[str, Any]) -> list[str]:
    """Return every reason the finite schedule must not be submitted yet."""
    try:
        validate_manifest(manifest)
    except ContractError as exc:
        return [f"manifest:{exc}"]
    blockers: list[str] = []
    source = manifest["source"]
    if not source["clean"]:
        blockers.append("source_not_clean")
    git_sha = source.get("git_sha", "")
    if len(git_sha) not in (40, 64):
        blockers.append("source_sha_not_fixed")

    fixed = manifest["fixed_inputs"]
    for key in ("model", "physics", "map", "calibration"):
        item = fixed.get(key)
        if not isinstance(item, dict):
            blockers.append(f"fixed_input_missing:{key}")
            continue
        try:
            _require_sha256(item.get("sha256"), f"fixed_inputs.{key}.sha256")
        except ContractError:
            blockers.append(f"fixed_input_hash_unresolved:{key}")

    per_run = manifest["budgets"].get("per_run")
    cohort = manifest["budgets"].get("cohort")
    per_run_fields = ("wall_time_s", "sim_time_s", "commands", "model_calls", "input_tokens",
                      "output_tokens", "messages", "message_bytes", "message_tokens")
    cohort_fields = ("wall_time_s", "model_calls", "total_tokens")
    for namespace, values, fields in (("per_run", per_run, per_run_fields),
                                      ("cohort", cohort, cohort_fields)):
        if not isinstance(values, dict):
            blockers.append(f"budget_missing:{namespace}")
            continue
        for field in fields:
            try:
                _require_positive_number(values.get(field), f"budgets.{namespace}.{field}")
            except ContractError:
                blockers.append(f"budget_missing_or_invalid:{namespace}.{field}")

    readiness = manifest["readiness"]
    for gate in (
        "r0_design_approved",
        "r1_boundary_audit_passed",
        "r2_common_runtime_verified",
        "r3_conditions_integrated",
        "event_schema_agreed",
        "single_submitter_named",
    ):
        if readiness.get(gate) is not True:
            blockers.append(f"readiness_gate_closed:{gate}")
    return blockers


def finite_schedule(manifest: dict[str, Any], *, require_ready: bool = False) -> list[dict[str, Any]]:
    validate_manifest(manifest)
    blockers = execution_blockers(manifest)
    if require_ready and blockers:
        raise ContractError("execution blocked: " + ", ".join(blockers))
    return [deepcopy(row) for row in sorted(manifest["schedule"], key=lambda item: item["order"])]


def validate_event(event: dict[str, Any], plan: dict[str, Any], seen_ids: set[str]) -> None:
    required = {
        "schema_version", "event_id", "run_id", "condition", "robot_id",
        "event_type", "sim_time_s", "wall_time_s", "related_ids", "payload",
    }
    missing = required - set(event)
    if missing:
        raise ContractError(f"runtime event missing fields: {sorted(missing)}")
    extra = set(event) - required
    if extra:
        raise ContractError(f"runtime event has unexpected fields: {sorted(extra)}")
    if event["schema_version"] != EVENT_SCHEMA:
        raise ContractError("runtime event schema mismatch")
    if event["run_id"] != plan["run_id"] or event["condition"] != plan["condition"]:
        raise ContractError("runtime event does not match planned run")
    event_id = event["event_id"]
    if not isinstance(event_id, str) or not event_id or event_id in seen_ids:
        raise ContractError("runtime event_id must be non-empty and unique")
    seen_ids.add(event_id)
    if event["robot_id"] is not None and event["robot_id"] not in ROBOTS:
        raise ContractError("runtime robot_id must be r1, r2, r3, or null")
    if not isinstance(event["event_type"], str) or not event["event_type"]:
        raise ContractError("runtime event_type must be non-empty")
    if event["sim_time_s"] is not None and (not _is_number(event["sim_time_s"]) or event["sim_time_s"] < 0):
        raise ContractError("sim_time_s must be null or non-negative")
    if not _is_number(event["wall_time_s"]) or event["wall_time_s"] < 0:
        raise ContractError("wall_time_s must be non-negative")
    if not isinstance(event["related_ids"], dict) or not all(
        isinstance(key, str) and bool(key) and isinstance(value, str) and bool(value)
        for key, value in event["related_ids"].items()
    ):
        raise ContractError("related_ids must map strings to strings")
    if not isinstance(event["payload"], dict):
        raise ContractError("payload must be an object")


def load_events(path: Path, plan: dict[str, Any]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    with path.open() as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ContractError(f"invalid JSONL at line {line_number}") from exc
            if not isinstance(event, dict):
                raise ContractError(f"runtime event at line {line_number} is not an object")
            validate_event(event, plan, seen_ids)
            events.append(event)
    if not events:
        raise ContractError("runtime artifact is empty")
    if sum(event["event_type"] == "run_started" for event in events) != 1:
        raise ContractError("runtime artifact requires exactly one run_started event")
    if sum(event["event_type"] == "run_finished" for event in events) != 1:
        raise ContractError("runtime artifact requires exactly one run_finished event")
    if events[0]["event_type"] != "run_started" or events[-1]["event_type"] != "run_finished":
        raise ContractError("runtime artifact must start and finish with boundary events")
    wall_times = [event["wall_time_s"] for event in events]
    if wall_times != sorted(wall_times):
        raise ContractError("runtime wall_time_s must be non-decreasing in JSONL order")
    terminal = [event for event in events if event["event_type"] == "run_finished"][0]
    if terminal["payload"].get("outcome") not in RUNTIME_OUTCOMES:
        raise ContractError("run_finished outcome is invalid")
    return events


def load_evaluator(path: Path, plan: dict[str, Any]) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError("invalid evaluator JSON") from exc
    if not isinstance(value, dict) or value.get("schema_version") != EVALUATOR_SCHEMA:
        raise ContractError("evaluator schema mismatch")
    if value.get("run_id") != plan["run_id"] or value.get("condition") != plan["condition"]:
        raise ContractError("evaluator does not match planned run")
    snapshot = value.get("source_snapshot")
    if not isinstance(snapshot, dict) or snapshot.get("schema") != "ugrp.rgb_evaluation_snapshot.v1":
        raise ContractError("evaluator must preserve the B evaluation snapshot")
    snapshot_fields = {
        "schema", "clock_domain", "timestamp_s", "external_evaluation",
        "coordination_audit", "active_task_ids", "pending_task_ids",
        "resource_owners", "meaning",
    }
    if set(snapshot) != snapshot_fields:
        raise ContractError("evaluator snapshot fields do not match the B contract")
    if snapshot.get("meaning") != "evaluation-only; forbidden as actor input or completion feedback":
        raise ContractError("evaluator snapshot boundary declaration is missing")
    if snapshot.get("clock_domain") not in {"sim", "monotonic"}:
        raise ContractError("evaluator snapshot clock_domain is invalid")
    # Unknown actual clock is preserved as missing raw evidence, not invented
    # from a request or last acknowledgement. Combined scoring rejects it below.
    if snapshot["timestamp_s"] is not None and (
            not _is_number(snapshot["timestamp_s"]) or snapshot["timestamp_s"] < 0):
        raise ContractError("evaluator snapshot timestamp_s is invalid")
    if not isinstance(snapshot.get("active_task_ids"), list) or not isinstance(
        snapshot.get("pending_task_ids"), list
    ) or not isinstance(snapshot.get("resource_owners"), dict):
        raise ContractError("evaluator snapshot task/resource fields are invalid")
    if snapshot["active_task_ids"] or snapshot["pending_task_ids"]:
        raise ContractError("evaluator snapshot was captured before runtime tasks were closed")
    if not isinstance(snapshot.get("coordination_audit"), list):
        raise ContractError("evaluator snapshot coordination_audit must be a list")
    external = snapshot.get("external_evaluation")
    if not isinstance(external, dict):
        raise ContractError("evaluator snapshot requires external_evaluation")
    if not isinstance(external.get("mission_complete"), bool):
        raise ContractError("external_evaluation.mission_complete must be boolean")
    objects = external.get("objects")
    if not isinstance(objects, dict):
        raise ContractError("evaluator objects must be an object")
    for object_id, record in objects.items():
        if not isinstance(object_id, str) or not isinstance(record, dict):
            raise ContractError("evaluator object records are invalid")
        if "stages" in record:
            if not isinstance(record["stages"], dict) or not all(
                isinstance(stage, str) and isinstance(done, bool) for stage, done in record["stages"].items()
            ):
                raise ContractError("object stages must map strings to booleans")
        elif type(record.get("physical_success")) is not bool or type(record.get("attempted")) is not bool:
            raise ContractError("each evaluator object requires stages or measured physical success/attempted")
    if set(objects) != set(plan["object_ids"]):
        raise ContractError("evaluator objects do not match the planned mission")
    recovery = external.get("recovery")
    if not isinstance(recovery, dict) or not all(
        isinstance(recovery.get(key), bool) for key in ("required", "reached", "succeeded")
    ):
        raise ContractError("evaluator recovery requires three boolean fields")
    if recovery["succeeded"] and not recovery["reached"]:
        raise ContractError("recovery cannot succeed before it is reached")
    return value


def _declared_measurements(events: Iterable[dict[str, Any]]) -> set[str]:
    starts = [event for event in events if event["event_type"] == "run_started"]
    declared = starts[0]["payload"].get("measured_metrics")
    if not isinstance(declared, list) or not all(isinstance(value, str) for value in declared):
        raise ContractError("run_started.payload.measured_metrics must be a string list")
    if len(declared) != len(set(declared)):
        raise ContractError("run_started measured_metrics contains duplicates")
    unknown = set(declared) - set(RUNTIME_MEASUREMENT_NAMES)
    if unknown:
        raise ContractError(f"unknown measured metrics: {sorted(unknown)}")
    return set(declared)


def extract_metrics(events: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, str]]:
    declared = _declared_measurements(events)
    metrics: dict[str, Any] = {
        "wall_time_s": max(event["wall_time_s"] for event in events),
        "sim_time_s": max(
            (event["sim_time_s"] for event in events if event["sim_time_s"] is not None),
            default=None,
        ),
    }
    statuses = {
        "wall_time_s": "measured",
        "sim_time_s": "measured" if metrics["sim_time_s"] is not None else "not_measured",
    }
    for name, kind in COUNT_METRICS.items():
        metrics[name] = sum(event["event_type"] == kind for event in events) if name in declared else None
        statuses[name] = "measured" if name in declared else "not_measured"
    for name, (kind, field) in SUM_METRICS.items():
        if name not in declared:
            metrics[name] = None
            statuses[name] = "not_measured"
            continue
        values = [event["payload"].get(field) for event in events if event["event_type"] == kind]
        if any(value is not None and (not _is_number(value) or value < 0) for value in values):
            raise ContractError(f"declared metric {name} has invalid values")
        known = [value for value in values if value is not None]
        complete = len(known) == len(values)
        metrics[name] = sum(known) if complete else None
        statuses[name] = "measured" if complete else "partially_measured"
    model_latencies = [event["payload"].get("response_latency_s") for event in events
                       if event["event_type"] == "planner_responded"]
    if "model_response_time_s" in declared:
        known_latencies = [value for value in model_latencies if value is not None]
        complete = len(known_latencies) == len(model_latencies)
        metrics["model_response_time_s_mean"] = (
            sum(known_latencies) / len(known_latencies) if complete and known_latencies
            else 0.0 if complete else None
        )
        statuses["model_response_time_s_mean"] = (
            "measured" if complete else "partially_measured"
        )
    else:
        metrics["model_response_time_s_mean"] = None
        statuses["model_response_time_s_mean"] = "not_measured"
    terminal = next((event["payload"] for event in reversed(events)
                     if event["event_type"] == "run_finished"), {})
    if "external_model_calls" in declared:
        actual = terminal.get("external_model_calls")
        upper = terminal.get("external_model_call_upper_bound")
        unknown = terminal.get("unknown_external_call_requests")
        measured = type(actual) is int and actual >= 0 and unknown == []
        metrics["external_model_calls"] = actual if measured else None
        statuses["external_model_calls"] = "measured" if measured else "partially_measured"
        metrics["external_model_call_upper_bound"] = upper if type(upper) is int else None
        statuses["external_model_call_upper_bound"] = "bounded" if type(upper) is int else "not_measured"
    tokens = terminal.get("tokens", {})
    if isinstance(tokens, dict) and tokens:
        for key in ("input_tokens", "output_tokens"):
            value, unknown = tokens.get("measured_" + key), tokens.get("unknown_" + key + "_calls")
            measured = type(value) is int and value >= 0 and unknown == 0
            metrics[key] = value if measured else None
            statuses[key] = "measured" if measured else "partially_measured"
    evidence_kinds = next((event["payload"].get("evidence_kinds", {}) for event in events
                           if event["event_type"] == "run_started"), {})
    metrics["planner_response_time_s"] = metrics["model_response_time_s"]
    statuses["planner_response_time_s"] = statuses["model_response_time_s"]
    metrics["pending_planner_requests"] = len(terminal.get("pending_planner_requests", {})) \
        if isinstance(terminal.get("pending_planner_requests"), dict) else None
    if evidence_kinds and set(evidence_kinds.values()) == {"deterministic_physical_replay"}:
        for key in ("model_response_time_s", "model_response_time_s_mean"):
            metrics[key] = None
            statuses[key] = "not_applicable_offline_replay"
    elif terminal.get("pending_planner_requests"):
        statuses["model_response_time_s_mean"] = "completed_responses_only_pending_excluded"
    return metrics, statuses


def extract_evaluator_metrics(evaluator: dict[str, Any]) -> tuple[dict[str, Any], dict[str, str]]:
    snapshot = evaluator["source_snapshot"]
    audit = snapshot.get("coordination_audit")
    if audit is None:
        return {"commands": None}, {"commands": "not_measured"}
    if not isinstance(audit, list):
        raise ContractError("coordination_audit must be a list when present")
    command_ids = []
    for event in audit:
        if not isinstance(event, dict):
            raise ContractError("coordination_audit events must be objects")
        if event.get("event") != "LOCAL_COMMAND":
            continue
        required = {
            "event", "robot_id", "timestamp_s", "task_id", "lease_id", "command_id",
            "stage", "action", "duration_s", "issued_at_s", "meaning",
        }
        if not required <= set(event):
            raise ContractError("LOCAL_COMMAND is missing issuance provenance")
        if event.get("robot_id") not in ROBOTS:
            raise ContractError("LOCAL_COMMAND robot_id is invalid")
        if event.get("meaning") != "issued command, not measured state or success":
            raise ContractError("LOCAL_COMMAND evidence boundary is invalid")
        command_id = event.get("command_id")
        if not isinstance(command_id, str) or not command_id:
            raise ContractError("LOCAL_COMMAND requires a unique command_id")
        command_ids.append(command_id)
    if len(command_ids) != len(set(command_ids)):
        raise ContractError("duplicate LOCAL_COMMAND command_id")
    return {"commands": len(command_ids)}, {"commands": "measured"}


def score_termination(runtime_terminal: dict[str, Any], external: dict[str, Any]) -> dict[str, Any]:
    """Score after shutdown without altering either actor or referee evidence.

    A completed actor loop is only a claim.  Conversely a timeout/abort can
    coexist with a positive referee measurement; that does not erase the
    system termination.  Legacy success/failure fields remain readable.
    """
    runtime_outcome = runtime_terminal.get("outcome")
    physical = external.get("mission_complete")
    if runtime_outcome not in RUNTIME_OUTCOMES or type(physical) is not bool:
        raise ContractError("invalid termination or evaluator verdict")
    claims = runtime_terminal.get("actor_finish_claims", {})
    if not isinstance(claims, dict) or not set(claims) <= set(ROBOTS) \
            or any(type(value) is not bool for value in claims.values()):
        raise ContractError("actor_finish_claims must map actor IDs to booleans")
    normal = runtime_outcome in {"completed", "success", "failure"}
    outcome = ("success" if physical else "failure") if normal else runtime_outcome
    return {
        "outcome": outcome,
        "mission_complete": outcome == "success",
        "physical_mission_complete": physical,
        "runtime_outcome": runtime_outcome,
        "runtime_termination": deepcopy(runtime_terminal),
        "evaluator_verdict": deepcopy(external),
        "actor_finish_claims": deepcopy(claims),
        "false_finish_claim": not physical and (
            runtime_outcome == "success" or any(claims.values())
        ),
        "false_finish_claim_by_robot": {rid: claim and not physical for rid, claim in claims.items()},
        "termination_verdict_disagreement": (normal and runtime_outcome == "success" and not physical)
            or (not normal and physical),
    }


def interaction_metrics(events: list[dict[str, Any]], snapshot: dict[str, Any]) -> dict[str, Any]:
    """Measured command overlap and event-linked replanning, never causal proof."""
    audit = snapshot.get("coordination_audit", [])
    commands = [row for row in audit if row.get("event") == "LOCAL_COMMAND"]
    intervals = []
    for row in commands:
        begin, duration = row.get("issued_at_s"), row.get("duration_s")
        if _is_number(begin) and _is_number(duration) and duration >= 0:
            intervals.append((begin, begin + duration, row["robot_id"], row.get("task_id")))
    points = sorted({point for begin, end, _, _ in intervals for point in (begin, end)})
    concurrent_robot_s = independent_task_s = 0.0
    busy = {robot: 0.0 for robot in ROBOTS}
    for left, right in zip(points, points[1:]):
        active = [(robot, task) for begin, end, robot, task in intervals if begin <= left < end]
        robots = {robot for robot, _ in active}
        for robot in robots:
            busy[robot] += right - left
        if len(robots) >= 2:
            concurrent_robot_s += right - left
        if len({task for _, task in active}) >= 2:
            independent_task_s += right - left
    waits = {robot: 0 for robot in ROBOTS}
    prior_actions: dict[str, Any] = {}
    received_since_action: dict[str, set[str]] = {robot: set() for robot in ROBOTS}
    linked_changes = []
    for event in events:
        robot = event["robot_id"]
        if robot not in ROBOTS:
            continue
        payload, related = event["payload"], event["related_ids"]
        if event["event_type"] == "message_received":
            if related.get("message_id"):
                received_since_action[robot].add(related["message_id"])
        if event["event_type"] == "planner_responded":
            action = payload.get("action")
            if isinstance(action, dict) and action.get("kind") == "wait":
                waits[robot] += 1
            if action is not None and robot in prior_actions and action != prior_actions[robot] \
                    and received_since_action[robot]:
                linked_changes.append({"robot_id": robot, "decision_id": related.get("decision_id"),
                                       "message_ids": sorted(received_since_action[robot]),
                                       "before": prior_actions[robot], "after": deepcopy(action)})
            if action is not None:
                prior_actions[robot] = deepcopy(action)
                received_since_action[robot].clear()
    return {
        "issued_command_overlap_sim_s": concurrent_robot_s,
        "independent_task_command_overlap_sim_s": independent_task_s,
        "issued_command_busy_sim_s_by_robot": busy,
        "wait_decisions_by_robot": waits,
        "message_preceded_action_changes": linked_changes,
        "deadlock_observed": snapshot.get("external_evaluation", {}).get("deadlock_observed"),
        "interpretation": "Issued command intervals are not measured motion; temporal message/action links are not causal effects.",
    }


def validate_terminal_clock(terminal: dict[str, Any], snapshot: dict[str, Any]) -> None:
    """Validate new supervisor clocks without rewriting legacy evidence.

    Requested time is never a substitute for actual time, including partial
    progress. Backend numerical clock checks own tiny requested/actual roundoff.
    """
    clock = terminal["payload"].get("clock")
    if "clock" not in terminal["payload"]:
        return
    fields = {"schema", "clock_domain", "requested_tick_s", "last_acknowledged_time_s",
              "last_successful_requested_tick_s", "terminal_time_s", "terminal_time_status", "reason"}
    if not isinstance(clock, dict) or set(clock) != fields:
        raise ContractError("runtime terminal clock schema is invalid")
    if (clock["schema"] != "rgb-runtime-clock.v1" or clock["clock_domain"] != "sim"
            or snapshot["clock_domain"] != clock["clock_domain"]):
        raise ContractError("runtime terminal clock domain or schema is invalid")
    if clock["terminal_time_status"] != "verified":
        raise ContractError("runtime terminal actual clock is unknown")
    actual = clock["terminal_time_s"]
    if (not _is_number(actual) or actual < 0 or actual != terminal["sim_time_s"]
            or not isinstance(clock["reason"], str) or not clock["reason"]):
        raise ContractError("runtime terminal actual clock is inconsistent")
    for field in ("requested_tick_s", "last_acknowledged_time_s", "last_successful_requested_tick_s"):
        if clock[field] is not None and (not _is_number(clock[field]) or clock[field] < 0):
            raise ContractError("runtime terminal clock has invalid numeric metadata")
    acknowledged = clock["last_acknowledged_time_s"]
    requested = clock["requested_tick_s"]
    successful_request = clock["last_successful_requested_tick_s"]
    if acknowledged is not None and acknowledged > actual:
        raise ContractError("runtime terminal actual clock precedes acknowledged clock")
    if successful_request is not None and (requested is None or successful_request > requested):
        raise ContractError("runtime terminal requested clock order is inconsistent")


def evaluate_run(plan: dict[str, Any], artifact_root: Path) -> dict[str, Any]:
    run_dir = artifact_root / plan["artifact_relpath"]
    base = {
        "run_id": plan["run_id"],
        "order": plan["order"],
        "scenario_id": plan["scenario_id"],
        "condition": plan["condition"],
        "seed": plan["seed"],
        "repeat": plan["repeat"],
        "artifact_relpath": plan["artifact_relpath"],
    }
    if not run_dir.exists():
        return {**base, "outcome": "unrun", "reason": "planned_run_directory_absent",
                "mission_complete": False, "artifacts": {}, "metrics": {},
                "measurement_status": {}}
    required = plan["required_artifacts"]
    runtime_path = run_dir / required["runtime"]
    evaluator_path = run_dir / required["evaluator"]
    if (not run_dir.resolve().is_relative_to(artifact_root.resolve()) or any(
        path.is_symlink() or not path.resolve().is_relative_to(run_dir.resolve())
        for path in (runtime_path, evaluator_path)
    )):
        return {**base, "outcome": "invalid_artifact", "reason": "artifact_path_escape_or_symlink",
                "mission_complete": False, "artifacts": {}, "metrics": {}, "measurement_status": {}}
    present = {"runtime": runtime_path.is_file(), "evaluator": evaluator_path.is_file()}
    if not all(present.values()):
        return {**base, "outcome": "missing_artifact", "reason": "required_artifact_absent",
                "mission_complete": False, "artifacts": present, "metrics": {},
                "measurement_status": {}}
    try:
        original_hashes = {"runtime": _sha256(runtime_path), "evaluator": _sha256(evaluator_path)}
        events = load_events(runtime_path, plan)
        evaluator = load_evaluator(evaluator_path, plan)
        terminal = [event for event in events if event["event_type"] == "run_finished"][0]
        metrics, measurement_status = extract_metrics(events)
        evaluator_metrics, evaluator_status = extract_evaluator_metrics(evaluator)
        metrics.update(evaluator_metrics)
        measurement_status.update(evaluator_status)
        external = evaluator["source_snapshot"]["external_evaluation"]
        score = score_termination(terminal["payload"], external)
        snapshot = evaluator["source_snapshot"]
        validate_terminal_clock(terminal, snapshot)
        if snapshot["timestamp_s"] is None:
            raise ContractError("evaluator snapshot actual clock is unknown")
        if (snapshot["clock_domain"] == "sim" and terminal["sim_time_s"] is not None
                and snapshot["timestamp_s"] < terminal["sim_time_s"]):
            raise ContractError("evaluator snapshot predates the runtime terminal event")
        if "clock" in terminal["payload"]:
            metrics["sim_time_s"] = terminal["payload"]["clock"]["terminal_time_s"]
            measurement_status["sim_time_s"] = "verified_supervisor_terminal_clock"
        if original_hashes != {"runtime": _sha256(runtime_path), "evaluator": _sha256(evaluator_path)}:
            raise ContractError("source artifact changed during evaluation")
    except ContractError as exc:
        return {**base, "outcome": "invalid_artifact", "reason": str(exc),
                "mission_complete": False,
                "artifacts": {"runtime": _sha256(runtime_path), "evaluator": _sha256(evaluator_path)},
                "metrics": {}, "measurement_status": {}}
    return {
        **base,
        **score,
        "reason": terminal["payload"].get("termination_reason", terminal["payload"].get("reason")),
        "objects": external["objects"],
        "recovery": external["recovery"],
        "metrics": metrics,
        "measurement_status": measurement_status,
        "artifacts": original_hashes,
        "evaluator_scope": evaluator.get("scope"),
        "interactions": interaction_metrics(events, snapshot),
        "evaluator_snapshot_sha256": hashlib.sha256(
            _canonical_json(evaluator["source_snapshot"])
        ).hexdigest(),
    }


def _successful_metric(rows: list[dict[str, Any]], name: str) -> dict[str, Any]:
    values = [row["metrics"].get(name) for row in rows
              if row["outcome"] == "success" and row["metrics"].get(name) is not None]
    return {
        "population": "successful_runs_only",
        "n": len(values),
        "values": values,
        "median": median(values) if values else None,
    }


def _all_outcome_metric(rows: list[dict[str, Any]], name: str) -> dict[str, Any]:
    values = [row["metrics"].get(name) for row in rows if row["metrics"].get(name) is not None]
    return {
        "population": "all_planned_runs_with_measurement",
        "planned_n": len(rows),
        "measured_n": len(values),
        "unmeasured_or_unavailable_n": len(rows) - len(values),
        "values": values,
        "total": sum(values) if values else None,
    }


def _stage_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    measured = [row for row in rows if isinstance(row.get("objects"), dict)]
    object_stages: dict[str, dict[str, dict[str, int]]] = {}
    keys = sorted({
        (object_id, stage)
        for row in measured
        for object_id, record in row["objects"].items()
        for stage in record.get("stages", {})
    })
    for object_id, stage in keys:
        records = [row["objects"].get(object_id, {}).get("stages", {}).get(stage)
                   for row in measured]
        available = [value for value in records if isinstance(value, bool)]
        object_stages.setdefault(object_id, {})[stage] = {
            "completed_n": sum(available),
            "evaluator_measured_n": len(available),
            "planned_n": len(rows),
        }
    recoveries = [row["recovery"] for row in rows if isinstance(row.get("recovery"), dict)]
    return {
        "objects": object_stages,
        "recovery": {
            "required_n": sum(value["required"] for value in recoveries),
            "reached_n": sum(value["reached"] for value in recoveries),
            "succeeded_n": sum(value["succeeded"] for value in recoveries),
            "evaluator_measured_n": len(recoveries),
            "planned_n": len(rows),
        },
    }


def evaluate_manifest(manifest: dict[str, Any], artifact_root: Path) -> dict[str, Any]:
    validate_manifest(manifest)
    rows = [evaluate_run(plan, artifact_root) for plan in finite_schedule(manifest)]
    by_condition: dict[str, Any] = {}
    for condition in CONDITIONS:
        condition_rows = [row for row in rows if row["condition"] == condition]
        counts = Counter(row["outcome"] for row in condition_rows)
        by_condition[condition] = {
            "planned_denominator": len(condition_rows),
            "mission_complete_numerator": sum(row["mission_complete"] for row in condition_rows),
            "mission_completion_rate": (
                sum(row["mission_complete"] for row in condition_rows) / len(condition_rows)
            ),
            "outcome_counts": {outcome: counts.get(outcome, 0) for outcome in REPORT_OUTCOMES},
            "successful_wall_time_s": _successful_metric(condition_rows, "wall_time_s"),
            "successful_sim_time_s": _successful_metric(condition_rows, "sim_time_s"),
            "cost_metrics_all_outcomes": {
                name: _all_outcome_metric(condition_rows, name)
                for name in (
                    "commands", "high_level_actions", "messages", "message_bytes",
                    "message_tokens", "model_calls", "input_tokens", "output_tokens",
                    "model_response_time_s",
                )
            },
            "stage_summary": _stage_summary(condition_rows),
            "note": "Failure durations are retained in run_rows and excluded from successful-only speed summaries.",
        }

    groups: dict[tuple[str, int, int], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[(row["scenario_id"], row["seed"], row["repeat"])].append(row)
    paired_rows = []
    for (scenario_id, seed, repeat), group in sorted(groups.items()):
        values = {row["condition"]: {
            "run_id": row["run_id"],
            "outcome": row["outcome"],
            "mission_complete": row["mission_complete"],
            "metrics": row["metrics"],
        } for row in group}
        paired_rows.append({
            "scenario_id": scenario_id,
            "seed": seed,
            "repeat": repeat,
            "complete_condition_triplet": set(values) == set(CONDITIONS),
            "conditions": values,
        })

    expected_dirs = {Path(row["artifact_relpath"]) for row in manifest["schedule"]}
    runs_dir = artifact_root / "runs"
    unexpected = []
    if runs_dir.is_dir():
        unexpected = sorted(str(path.relative_to(artifact_root)) for path in runs_dir.iterdir()
                            if path.is_dir() and path.relative_to(artifact_root) not in expected_dirs)
    counts = Counter(row["outcome"] for row in rows)
    return {
        "schema_version": REPORT_SCHEMA,
        "manifest_schema_version": manifest["schema_version"],
        "manifest_sha256": hashlib.sha256(_canonical_json(manifest)).hexdigest(),
        "manifest_protocol_sha256": manifest["protocol_sha256"],
        "planned_denominator": manifest["planned_denominator"],
        "mission_complete_numerator": sum(row["mission_complete"] for row in rows),
        "outcome_counts": {outcome: counts.get(outcome, 0) for outcome in REPORT_OUTCOMES},
        "all_planned_runs_accounted_for": len(rows) == manifest["planned_denominator"],
        "unexpected_artifact_directories": unexpected,
        "condition_summary": by_condition,
        "paired_raw_rows": paired_rows,
        "run_rows": rows,
        "uncertainty_analysis": {
            "status": "not_applicable_to_six-run_connection_pilot",
            "future_unit": "paired scenario/map block with predeclared repeats",
            "raw_rows_preserved": True,
        },
        "interpretation_boundary": (
            "Connection-pilot artifact audit only; no statistical superiority claim. "
            "Synthetic fixtures are not research results."
        ),
    }
