"""TensorBoard adapter for completed study trials with separate raw evidence."""
from __future__ import annotations

from harness.rgb_communication_evaluation import (
    evaluate_run, load_events, load_evaluator, extract_metrics, extract_evaluator_metrics,
    interaction_metrics,
)


RUN_SCHEMA = "rgb-communication-trial-result.v2"
EXTRA_METRICS = (
    "result/high_level_actions", "result/messages", "result/message_bytes", "result/message_tokens",
    "result/model_response_time_s", "result/planner_response_time_s", "result/pending_planner_requests",
    "result/external_model_calls", "result/planner_decisions",
    "evaluation/physical_mission_complete", "evaluation/false_finish_claim",
    "evaluation/evidence_valid", "result/process_wall_s",
    "clock/runtime_requested_sim_s", "clock/evaluator_snapshot_sim_s",
    "execution/issued_command_overlap_sim_s", "execution/independent_task_command_overlap_sim_s",
)


def export_communication(src, writer, result):
    """Re-score original runtime/evaluator files instead of trusting summary success."""
    plan = src.read("plan.json", required=True)
    src.read("evaluator.json", required=True)
    src.read_jsonl("runtime.jsonl")
    for name in ("runtime.jsonl", "evaluator.json", "plan.json"):
        expected = result.get("source_artifact_hashes", {}).get(name)
        if expected != src.files[name]["sha256"]:
            raise ValueError("Communication source artifact changed: " + name)
    scored = evaluate_run({**plan, "artifact_relpath": "."}, src.root)
    if scored["outcome"] in {"missing_artifact", "unrun"}:
        raise ValueError("Communication source not verifiable: " + scored["outcome"])
    valid = scored["outcome"] != "invalid_artifact"
    # A clock-consistency failure must remain visible as a failure diagnostic,
    # never disappear from the dashboard or be upgraded to a scored success.
    # Parse each raw source separately; malformed JSON/schema still fails closed.
    if not valid:
        events = load_events(src.root / "runtime.jsonl", plan)
        evaluation = load_evaluator(src.root / "evaluator.json", plan)
        values, statuses = extract_metrics(events)
        issued, issued_status = extract_evaluator_metrics(evaluation)
        values.update(issued)
        statuses.update(issued_status)
        terminal = events[-1]["payload"]
        external = evaluation["source_snapshot"]["external_evaluation"]
        scored.update(metrics=values, measurement_status=statuses, runtime_termination=terminal,
            evaluator_verdict=external, actor_finish_claims=terminal.get("actor_finish_claims", {}),
            physical_mission_complete=external["mission_complete"], false_finish_claim=None,
            interactions=interaction_metrics(events, evaluation["source_snapshot"]))
        writer.text("validation/failure", {"reason": scored["reason"],
            "scope": "Invalid combined evidence. Separate raw-source measurements below do not establish a final scored success."})
        clock = terminal.get("clock")
        requested = clock.get("requested_tick_s") if isinstance(clock, dict) else events[-1]["sim_time_s"]
        writer.scalar("clock/runtime_requested_sim_s", requested)
        writer.scalar("clock/evaluator_snapshot_sim_s", evaluation["source_snapshot"]["timestamp_s"])
        # Do not disguise an attempted tick as measured final SIM progress.
        values["sim_time_s"] = None
        statuses["sim_time_s"] = "inconsistent_clock_sources"
    values = scored["metrics"]
    metrics = {"result/wall_s": values.get("wall_time_s"),
               "result/sim_s": values.get("sim_time_s"),
               "evaluation/reported_success": int(scored["mission_complete"]),
               "evaluation/evidence_valid": int(valid),
               "evaluation/physical_mission_complete": int(scored["physical_mission_complete"]),
               "evaluation/false_finish_claim": int(scored["false_finish_claim"])
                   if type(scored["false_finish_claim"]) is bool else None}
    for field in ("commands", "model_calls", "input_tokens", "output_tokens", "high_level_actions",
                  "messages", "message_bytes", "message_tokens", "model_response_time_s",
                  "external_model_calls", "planner_decisions", "planner_response_time_s", "pending_planner_requests"):
        metrics["result/" + field] = values.get(field)
    for field in ("issued_command_overlap_sim_s", "independent_task_command_overlap_sim_s"):
        metrics["execution/" + field] = scored["interactions"].get(field)
    process = src.read("process.json")
    if isinstance(process, dict):
        metrics["result/process_wall_s"] = process.get("wall_time_s")
    for tag, value in metrics.items():
        writer.scalar(tag, value)
    writer.text("result/termination", scored["runtime_termination"])
    writer.text("evaluation/referee_only", scored["evaluator_verdict"])
    writer.text("claims/actor_finish", scored["actor_finish_claims"])
    writer.text("result/measurements", scored["measurement_status"])
    writer.text("execution/message_action_links_not_causal", scored["interactions"])
    return {"family": "rgb-communication", "policy": plan["condition"],
            "case": plan["scenario_id"], "run_id": plan["run_id"],
            "condition": plan["condition"], "source_sha": result["source_sha"],
            "seed": plan["seed"], "outcome": scored["outcome"],
            "clock": "sim+wall", "scope": result["evidence_kind"],
            "setup_sha256": result["config_sha256"],
            "evidence_valid": valid,
            "success_source_field": "independent_evaluator_after_runtime_close" if valid
                else "invalid_artifact_no_confirmed_success"}, metrics
