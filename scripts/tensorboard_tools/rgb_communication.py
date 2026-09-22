"""TensorBoard adapter for completed study trials with separate raw evidence."""
from __future__ import annotations

from harness.rgb_communication_evaluation import evaluate_run


RUN_SCHEMA = "rgb-communication-trial-result.v2"
EXTRA_METRICS = (
    "result/high_level_actions", "result/messages", "result/message_bytes", "result/message_tokens",
    "result/model_response_time_s", "result/planner_response_time_s", "result/pending_planner_requests",
    "result/external_model_calls", "result/planner_decisions",
    "evaluation/physical_mission_complete", "evaluation/false_finish_claim",
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
    if scored["outcome"] in {"missing_artifact", "invalid_artifact", "unrun"}:
        raise ValueError("Communication source not verifiable: " + scored["outcome"])
    values = scored["metrics"]
    metrics = {"result/wall_s": values.get("wall_time_s"),
               "result/sim_s": values.get("sim_time_s"),
               "evaluation/reported_success": int(scored["mission_complete"]),
               "evaluation/physical_mission_complete": int(scored["physical_mission_complete"]),
               "evaluation/false_finish_claim": int(scored["false_finish_claim"])}
    for field in ("commands", "model_calls", "input_tokens", "output_tokens", "high_level_actions",
                  "messages", "message_bytes", "message_tokens", "model_response_time_s",
                  "external_model_calls", "planner_decisions", "planner_response_time_s", "pending_planner_requests"):
        metrics["result/" + field] = values.get(field)
    for field in ("issued_command_overlap_sim_s", "independent_task_command_overlap_sim_s"):
        metrics["execution/" + field] = scored["interactions"].get(field)
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
            "success_source_field": "independent_evaluator_after_runtime_close"}, metrics
