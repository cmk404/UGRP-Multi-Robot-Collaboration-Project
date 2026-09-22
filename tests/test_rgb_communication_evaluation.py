import copy
import json
from pathlib import Path
import subprocess
import sys

import pytest

from harness.rgb_communication_evaluation import (
    ContractError,
    EVALUATOR_SCHEMA,
    EVENT_SCHEMA,
    build_manifest,
    evaluate_manifest,
    execution_blockers,
    finite_schedule,
    validate_manifest,
)


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "rgb_communication_evaluation"


def protocol():
    return json.loads((FIXTURE / "pilot_protocol.json").read_text())


def manifest():
    return build_manifest(protocol(), source_sha="a" * 40, source_clean=True)


def event(plan, event_id, event_type, wall_time_s, *, sim_time_s=0.0, robot_id=None, payload=None,
          related_ids=None):
    return {
        "schema_version": EVENT_SCHEMA,
        "event_id": event_id,
        "run_id": plan["run_id"],
        "condition": plan["condition"],
        "robot_id": robot_id,
        "event_type": event_type,
        "sim_time_s": sim_time_s,
        "wall_time_s": wall_time_s,
        "related_ids": related_ids or {},
        "payload": payload or {},
    }


def write_artifacts(root, plan, outcome, *, measured=True, duplicate_event=False):
    run = root / plan["artifact_relpath"]
    run.mkdir(parents=True)
    measured_metrics = [
        "high_level_actions", "messages", "message_bytes", "model_calls",
        "input_tokens", "output_tokens", "model_response_time_s",
    ] if measured else []
    rows = [event(plan, "start", "run_started", 0.0,
                  payload={"measured_metrics": measured_metrics})]
    if measured:
        rows.extend([
            event(plan, "request", "planner_requested", 0.9, sim_time_s=0.5, robot_id="r1",
                  related_ids={"request_id": "req-1"}),
            event(plan, "model", "planner_responded", 1.0, sim_time_s=0.5, robot_id="r1",
                  related_ids={"request_id": "req-1", "decision_id": "decision-1"},
                  payload={"input_tokens": 20, "output_tokens": 5, "response_latency_s": 0.25}),
            event(plan, "message", "message_sent", 1.1, sim_time_s=0.5, robot_id="r1",
                  related_ids={"message_id": "message-1"},
                  payload={"bytes": 12, "token_count": 3}),
            event(plan, "action", "action_submitted", 1.2, sim_time_s=0.6, robot_id="r1",
                  related_ids={"action_id": "action-1"}),
        ])
    terminal_wall = 8.0 if outcome == "success" else 2.0
    rows.append(event(plan, "finish", "run_finished", terminal_wall, sim_time_s=4.0,
                      payload={"outcome": outcome, "reason": outcome.upper()}))
    if duplicate_event:
        rows.insert(-1, copy.deepcopy(rows[1]))
    (run / "runtime.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    if outcome == "missing_artifact":
        return
    recovery_required = plan["scenario_id"] == "local-visual-misalignment-recovery"
    completed = outcome == "success"
    evaluator = {
        "schema_version": EVALUATOR_SCHEMA,
        "run_id": plan["run_id"],
        "condition": plan["condition"],
        "scope": "synthetic fixture only",
        "source_snapshot": {
            "schema": "ugrp.rgb_evaluation_snapshot.v1",
            "clock_domain": "sim",
            "timestamp_s": 4.0,
            "external_evaluation": {
                "mission_complete": completed,
                "objects": {
                    "beam": {"stages": {"picked": completed, "delivered": completed}},
                    "box": {"stages": {"picked": completed, "delivered": completed}}
                },
                "recovery": {
                    "required": recovery_required,
                    "reached": recovery_required and outcome not in {"api_error"},
                    "succeeded": recovery_required and completed
                }
            },
            "coordination_audit": [
                {"event": "LOCAL_COMMAND", "robot_id": "r1", "timestamp_s": 1.2,
                 "task_id": "task-1", "lease_id": "lease-1", "command_id": "command-1",
                 "stage": "APPROACH", "action": {"kind": "drive"}, "duration_s": 0.2,
                 "issued_at_s": 1.2,
                 "meaning": "issued command, not measured state or success"}
            ],
            "active_task_ids": [],
            "pending_task_ids": [],
            "resource_owners": {},
            "meaning": "evaluation-only; forbidden as actor input or completion feedback"
        }
    }
    (run / "evaluator.json").write_text(json.dumps(evaluator))


def materialize_fixture(root, value):
    outcomes = json.loads((FIXTURE / "outcomes.json").read_text())
    for plan in value["schedule"]:
        write_artifacts(root, plan, outcomes[plan["scenario_id"]][plan["condition"]])


def test_manifest_is_a_complete_randomized_six_run_block_with_provenance():
    value = manifest()
    validate_manifest(value)
    assert value["planned_denominator"] == 6
    assert [row["order"] for row in finite_schedule(value)] == list(range(1, 7))
    assert {(row["scenario_id"], row["condition"]) for row in value["schedule"]} == {
        (scenario, condition)
        for scenario in ("normal-supported-mission", "local-visual-misalignment-recovery")
        for condition in ("none", "structured", "natural")
    }
    assert value["source"] == {"git_sha": "a" * 40, "clean": True}
    assert value["budgets"]["per_run"]["wall_time_s"] == 900
    assert value["budgets"]["per_run"]["model_calls"] == 72
    assert value["fixed_inputs"]["physics"]["id"] == "weld-off-current-physics"


def test_duplicate_schedule_and_missing_budget_are_never_admitted():
    value = manifest()
    blockers = execution_blockers(value)
    assert "budget_missing_or_invalid:per_run.message_tokens" in blockers
    assert "fixed_input_hash_unresolved:model" in blockers
    assert "readiness_gate_closed:r1_boundary_audit_passed" in blockers
    with pytest.raises(ContractError, match="execution blocked"):
        finite_schedule(value, require_ready=True)

    ready = copy.deepcopy(value)
    ready["budgets"]["per_run"]["message_tokens"] = 1000
    for item in ready["fixed_inputs"].values():
        item["sha256"] = "b" * 64
    ready["readiness"] = {key: True for key in ready["readiness"]}
    assert execution_blockers(ready) == []
    assert len(finite_schedule(ready, require_ready=True)) == 6

    duplicate = copy.deepcopy(value)
    duplicate["schedule"][1]["run_id"] = duplicate["schedule"][0]["run_id"]
    with pytest.raises(ContractError, match="duplicate planned run"):
        validate_manifest(duplicate)


def test_all_planned_outcomes_stay_in_denominator_and_speed_uses_successes_only(tmp_path):
    value = manifest()
    materialize_fixture(tmp_path, value)
    report = evaluate_manifest(value, tmp_path)
    assert report["planned_denominator"] == 6
    assert report["mission_complete_numerator"] == 1
    assert report["outcome_counts"] == {
        "success": 1,
        "failure": 1,
        "timeout": 1,
        "api_error": 1,
        "aborted": 1,
        "unrun": 0,
        "missing_artifact": 1,
        "invalid_artifact": 0,
    }
    none = report["condition_summary"]["none"]
    assert none["planned_denominator"] == 2
    assert none["mission_completion_rate"] == 0.5
    assert none["successful_wall_time_s"]["values"] == [8.0]
    assert none["cost_metrics_all_outcomes"]["model_calls"]["planned_n"] == 2
    assert none["cost_metrics_all_outcomes"]["model_calls"]["measured_n"] == 2
    assert none["stage_summary"]["objects"]["beam"]["delivered"] == {
        "completed_n": 1, "evaluator_measured_n": 2, "planned_n": 2
    }
    success = next(row for row in report["run_rows"] if row["outcome"] == "success")
    assert success["metrics"]["commands"] == 1
    assert success["metrics"]["high_level_actions"] == 1
    assert success["metrics"]["model_calls"] == 1
    assert success["metrics"]["input_tokens"] == 20
    assert success["metrics"]["model_response_time_s"] == 0.25
    assert len(report["paired_raw_rows"]) == 2
    assert all(row["complete_condition_triplet"] for row in report["paired_raw_rows"])
    assert report["uncertainty_analysis"]["raw_rows_preserved"] is True


def test_absent_directory_is_unrun_but_partial_directory_is_missing_artifact(tmp_path):
    value = manifest()
    plan = value["schedule"][0]
    write_artifacts(tmp_path, plan, "missing_artifact")
    report = evaluate_manifest(value, tmp_path)
    row = next(item for item in report["run_rows"] if item["run_id"] == plan["run_id"])
    assert row["outcome"] == "missing_artifact"
    assert report["outcome_counts"]["unrun"] == 5
    assert report["mission_complete_numerator"] == 0


def test_duplicate_event_and_success_truth_disagreement_are_invalid_artifacts(tmp_path):
    value = manifest()
    first, second = value["schedule"][:2]
    write_artifacts(tmp_path, first, "failure", duplicate_event=True)
    write_artifacts(tmp_path, second, "success")
    evaluator_path = tmp_path / second["artifact_relpath"] / "evaluator.json"
    evaluator = json.loads(evaluator_path.read_text())
    evaluator["source_snapshot"]["external_evaluation"]["mission_complete"] = False
    evaluator_path.write_text(json.dumps(evaluator))
    report = evaluate_manifest(value, tmp_path)
    outcomes = {row["run_id"]: row["outcome"] for row in report["run_rows"]}
    assert outcomes[first["run_id"]] == "invalid_artifact"
    assert outcomes[second["run_id"]] == "invalid_artifact"
    assert report["mission_complete_numerator"] == 0


def test_unmeasured_metrics_remain_null(tmp_path):
    value = manifest()
    plan = value["schedule"][0]
    write_artifacts(tmp_path, plan, "failure", measured=False)
    report = evaluate_manifest(value, tmp_path)
    row = next(item for item in report["run_rows"] if item["run_id"] == plan["run_id"])
    assert row["metrics"]["messages"] is None
    assert row["metrics"]["input_tokens"] is None
    assert row["measurement_status"]["messages"] == "not_measured"
    assert row["metrics"]["commands"] == 1


def test_partially_measured_provider_usage_stays_null_without_losing_outcome(tmp_path):
    value = manifest()
    plan = value["schedule"][0]
    write_artifacts(tmp_path, plan, "failure", measured=True)
    runtime_path = tmp_path / plan["artifact_relpath"] / "runtime.jsonl"
    rows = [json.loads(line) for line in runtime_path.read_text().splitlines()]
    response = next(row for row in rows if row["event_type"] == "planner_responded")
    response["payload"].pop("input_tokens")
    runtime_path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    report = evaluate_manifest(value, tmp_path)
    row = next(item for item in report["run_rows"] if item["run_id"] == plan["run_id"])
    assert row["outcome"] == "failure"
    assert row["metrics"]["input_tokens"] is None
    assert row["measurement_status"]["input_tokens"] == "partially_measured"


def test_cli_admit_blocks_provisional_manifest(tmp_path):
    value = manifest()
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(value))
    result = subprocess.run(
        [sys.executable, "scripts/evaluate_rgb_communication.py", "admit", "--manifest", str(path)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 3
    response = json.loads(result.stdout)
    assert response["go"] is False
    assert "do not submit" in response["next_step"]
