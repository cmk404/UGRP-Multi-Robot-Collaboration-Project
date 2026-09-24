#!/usr/bin/env python3
"""Read-only, compact audit of a managed real-time dispatch run.

No RGB files or simulator state are read. An output file is written only with
--output and must live outside the raw run directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path


CORE = (
    "result.json", "evaluation-only.json", "issued-commands.json",
    "solo-decisions.json", "solo-raw-actions.json", "solo-renewals.json",
    "pair-decisions.json", "referee-only.jsonl", "execution.frames.json",
    "execution.mp4", "scene-manifest.json",
)
ESSENTIAL = (
    "result.json", "evaluation-only.json", "issued-commands.json",
    "solo-decisions.json", "pair-decisions.json", "referee-only.jsonl",
    "execution.frames.json", "execution.mp4",
)
MOTION_KEYS = ("forward", "left", "turn")


def read_json(path: Path, default=None):
    return json.loads(path.read_text()) if path.is_file() else default


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def summary(values):
    values = sorted(float(value) for value in values if isinstance(value, (int, float)) and math.isfinite(value))
    if not values:
        return {"count": 0}
    return {"count": len(values), "min": values[0], "median": statistics.median(values),
            "p95": values[math.ceil(.95 * len(values)) - 1], "max": values[-1], "sum": sum(values)}


def intervals(commands, *, renewal=None):
    rows = []
    for row in commands:
        if row.get("stage") != "TRANSIT" or not isinstance(row.get("action"), dict):
            continue
        if renewal is not None and bool(row.get("pending_renewal")) != renewal:
            continue
        action = row["action"]
        if not any(abs(float(action.get(axis, 0) or 0)) > 1e-9 for axis in MOTION_KEYS):
            continue
        start, end = row.get("issued_at_s"), row.get("valid_until_s")
        if isinstance(start, (int, float)) and isinstance(end, (int, float)) and end > start:
            rows.append((float(start), float(end)))
    return sorted(rows)


def coverage(rows):
    if not rows:
        return {"moving_command_count": 0, "moving_command_union_s": 0, "consecutive_positive_gap_s": summary([])}
    merged = [list(rows[0])]
    gaps = []
    for start, end in rows[1:]:
        if start > merged[-1][1] + 1e-9:
            gaps.append(start - merged[-1][1])
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)
    return {"moving_command_count": len(rows),
            "moving_command_union_s": sum(end - start for start, end in merged),
            "consecutive_positive_gap_s": summary(gaps)}


def audit_renewals(receipts, commands, decisions):
    issued = [row for row in commands if row.get("pending_renewal")]
    errors = []
    if len(receipts) != len(issued):
        errors.append(f"receipt_count={len(receipts)} issued_count={len(issued)}")
    for index, receipt in enumerate(receipts):
        row = next((candidate for candidate in issued
                    if candidate.get("issued_at_s") == receipt.get("issued_at_s")
                    and candidate.get("source_frame_id") == receipt.get("source_frame_id")), None)
        tag = f"renewal[{index}]"
        if row is None:
            errors.append(f"{tag}: no matching issued command")
            continue
        if receipt.get("kind") != "solo_carry_pending_renewal":
            errors.append(f"{tag}: wrong kind")
        if receipt.get("phase") != "carry" or row.get("stage") != "TRANSIT":
            errors.append(f"{tag}: outside carry/TRANSIT")
        issued_at = receipt.get("issued_at_s")
        until = receipt.get("valid_until_s")
        previous = receipt.get("previous_valid_until_s")
        observed = receipt.get("source_observed_at_s")
        duration = receipt.get("duration_s")
        if not all(isinstance(v, (int, float)) for v in (issued_at, until, previous, observed, duration)):
            errors.append(f"{tag}: missing time")
            continue
        if not (0 < duration <= .25 + 1e-8 and abs(until - issued_at - duration) <= 1e-5):
            errors.append(f"{tag}: invalid duration")
        if not issued_at < previous - 1e-9:
            errors.append(f"{tag}: previous lease expired before renewal")
        if until > observed + .6 + 1e-8:
            errors.append(f"{tag}: beyond original RGB deadline")
        if isinstance(receipt.get("original_rgb_deadline_s"), (int, float)) and until > receipt["original_rgb_deadline_s"] + 1e-8:
            errors.append(f"{tag}: beyond recorded RGB deadline")
        if abs(float(row.get("action", {}).get("duration_s", -1)) - duration) > 1e-5:
            errors.append(f"{tag}: issued action duration differs from receipt")
        for key in ("valid_until_s", "original_rgb_deadline_s", "source_decision_sim_time_s", "plan_hash"):
            if row.get(key) != receipt.get(key):
                errors.append(f"{tag}: {key} differs from issued command")
        if row.get("observed_at_s") != observed:
            errors.append(f"{tag}: changed source observation time")
        if row.get("renewal_reason") != receipt.get("reason"):
            errors.append(f"{tag}: reason differs from issued command")
        permissions = {tuple(permission) for permission in (receipt.get("permission_requests") or [])
                       if isinstance(permission, (list, tuple)) and len(permission) == 2}
        if ("box", "TRANSIT") not in permissions:
            errors.append(f"{tag}: TRANSIT permission absent")
        apron_required = ("box", "UNLOAD") in permissions
        if receipt.get("apron_permission_required") is not apron_required:
            errors.append(f"{tag}: apron permission flag differs from UNLOAD membership")
        if row.get("apron_permission_required") is not apron_required:
            errors.append(f"{tag}: issued apron permission flag differs from UNLOAD membership")
        if receipt.get("permission_verified_at_s") != issued_at or row.get("permission_verified_at_s") != issued_at:
            errors.append(f"{tag}: permission was not verified at issue time")
        if {tuple(permission) for permission in (row.get("permission_requests") or [])
            if isinstance(permission, (list, tuple)) and len(permission) == 2} != permissions:
            errors.append(f"{tag}: issued permission requests differ from receipt")
        source = next((candidate for candidate in commands if not candidate.get("pending_renewal")
                       and candidate.get("observed_at_s") == observed
                       and candidate.get("issued_at_s", -math.inf) <= issued_at
                       and candidate.get("stage") == "TRANSIT"), None)
        decision = next((candidate for candidate in decisions
                         if candidate.get("frame_id") == receipt.get("source_frame_id")
                         and candidate.get("observed_at_s") == observed), None)
        if source is None or decision is None:
            errors.append(f"{tag}: original decision/command cannot be linked")
        else:
            if receipt.get("source_decision_sim_time_s") != decision.get("sim_time_s"):
                errors.append(f"{tag}: source decision time differs")
            for axis in MOTION_KEYS:
                if abs(float(row["action"].get(axis, 0)) - float(source["action"].get(axis, 0))) > 1e-9:
                    errors.append(f"{tag}: {axis} changed from accepted action")
                if abs(float(row["action"].get(axis, 0)) - float(decision.get("action", {}).get(axis, 0))) > 1e-9:
                    errors.append(f"{tag}: {axis} changed from RGB decision")
    return {"receipts": len(receipts), "issued_renewals": len(issued), "errors": errors,
            "status": "pass" if not errors else "fail"}


def audit(run: Path, manager_path: Path):
    if not (run / "result.json").is_file():
        raise ValueError(f"missing result.json: {run}")
    if not manager_path.is_file():
        raise ValueError(f"missing manager manifest: {manager_path}")
    result = read_json(run / "result.json") or {}
    evaluation = read_json(run / "evaluation-only.json") or {}
    decisions = read_json(run / "solo-decisions.json", [])
    commands = read_json(run / "issued-commands.json", {})
    receipts = read_json(run / "solo-renewals.json", [])
    manager = read_json(manager_path) or {}
    errors = []
    receipt_files = {row.get("path"): row for row in (manager.get("output_receipt") or {}).get("files", [])}
    core = {}
    for name in CORE:
        path = run / name
        if not path.is_file():
            core[name] = {"present": False}
            if name in ESSENTIAL:
                errors.append(f"essential raw artifact missing: {name}")
            continue
        digest = sha256(path)
        recorded = receipt_files.get(name)
        core[name] = {"present": True, "bytes": path.stat().st_size, "sha256": digest,
                      "matches_manager_receipt": digest == recorded.get("sha256") if recorded else None}
        if recorded and core[name]["matches_manager_receipt"] is False:
            errors.append(f"manager output receipt mismatch: {name}")
    source = manager.get("source") or {}
    tree = source.get("execution_tree") or {}
    after = manager.get("source_after") or {}
    inputs_before = manager.get("inputs_before")
    inputs_after = manager.get("inputs_after")
    manager_summary = {
        "path": str(manager_path), "sha256": sha256(manager_path), "run_id": manager.get("run_id"),
        "status": manager.get("status"), "exit_code": manager.get("exit_code"),
        "output_matches_run": Path(manager.get("output", "")).resolve() == run.resolve(),
        "source_sha": source.get("source_sha"), "source_dirty": source.get("source_dirty"),
        "source_tree_before_sha256": tree.get("sha256"), "source_tree_after_sha256": after.get("sha256"),
        "source_unchanged": manager.get("source_changed_during_run") is False and tree.get("sha256") == after.get("sha256"),
        "inputs_unchanged": manager.get("inputs_changed_during_run") is False and inputs_before == inputs_after,
        "finalization_errors": manager.get("finalization_errors", []),
        "runtime_s": manager.get("runtime_s"),
    }
    if not manager_summary["output_matches_run"]:
        errors.append("manager output path differs from run path")
    if manager_summary["source_dirty"] is not False:
        errors.append("manager source was dirty before run")
    if manager_summary["finalization_errors"]:
        errors.append("manager finalization errors present")
    if not manager_summary["source_unchanged"] or not manager_summary["inputs_unchanged"]:
        errors.append("source or inputs changed during run")
    if result.get("source_sha") != source.get("source_sha"):
        errors.append("result source SHA differs from manager")
    if manager.get("status") != "process_completed" or manager.get("exit_code") != 0:
        errors.append("manager process did not complete successfully")
    if result.get("physical_success") != evaluation.get("physical_success"):
        errors.append("reported and evaluation physical success differ")
    bundles = {row["path"]: row["sha256"] for row in tree.get("files", [])
               if row.get("path", "").startswith("config/rgb_execution_bundles/rgb-standard-dispatch-v")}
    cargo = evaluation.get("cargo") or {}
    clearance = {name: {"physical_success": row.get("physical_success"),
                        "max_lift_m": row.get("max_lift_m"), "displacement_m": row.get("displacement_m"),
                        "inside_slot_at_end": row.get("inside_slot_at_end"),
                        "released_supported_stable": row.get("released_supported_stable"),
                        "carry_clearance": row.get("carry_clearance")}
                 for name, row in cargo.items()}
    concurrent = evaluation.get("concurrent_transport") or {}
    resource_events = result.get("resource_events") or []
    apron_holder = None
    apron_errors = []
    apron_order = []
    for event in resource_events:
        if event.get("event") == "acquire" and event.get("resource") == "dispatch_apron":
            if apron_holder is not None:
                apron_errors.append(f"overlapping apron holders: {apron_holder}, {event.get('object')}")
            apron_holder = event.get("object")
            apron_order.append(apron_holder)
        if "dispatch_apron" in (event.get("released") or []):
            if apron_holder != event.get("object"):
                apron_errors.append("apron release by non-holder")
            apron_holder = None
    if apron_holder is not None:
        apron_errors.append("apron never released")
    if apron_errors:
        errors.extend(f"resource: {error}" for error in apron_errors)
    solo = commands.get("r2", []) if isinstance(commands, dict) else []
    carry_decisions = [row for row in decisions if row.get("phase_before") == "carry"]
    carry_times = [row.get("sim_time_s") for row in carry_decisions if isinstance(row.get("sim_time_s"), (int, float))]
    attachments = [row.get("own_attachment_evidence") for row in decisions if isinstance(row.get("own_attachment_evidence"), dict)]
    held = Counter(str(row.get("attached")) for row in attachments)
    renewal_audit = audit_renewals(receipts, solo, decisions)
    if renewal_audit["errors"]:
        errors.append("solo renewal invariants failed")
    return {
        "schema": "ugrp.faster_dispatch_run_audit.v1", "run": str(run), "manager": manager_summary,
        "source_bundle": {"active_bundle": "not identified by run result or manager; inspect pinned source separately",
                          "recorded_bundle_hashes": bundles,
                          "plan_replay_sha256": result.get("plan_replay_sha256"),
                          "scene_manifest_sha256": core.get("scene-manifest.json", {}).get("sha256")},
        "outcome": {"phase": result.get("phase"), "error": result.get("error"),
                    "protocol_complete": result.get("protocol_complete"),
                    "physical_success_reported": result.get("physical_success"),
                    "physical_success_evaluation_only": evaluation.get("physical_success"),
                    "llm_calls": result.get("llm_calls"), "scope": result.get("scope"),
                    "interpretation": "referee samples are evaluation-only; issued commands are not measured motion or physical success"},
        "timing": {"wall_s": result.get("wall_s"), **(result.get("timing") or {})},
        "physical_evaluation_only": {"cargo": clearance, "weld_steps": evaluation.get("weld_steps"),
                                     "robot_robot_contact_samples": evaluation.get("robot_robot_contact_samples"),
                                     "obstacle_contact_steps": result.get("obstacle_contact_steps"),
                                     "simultaneous_loaded_motion_s": concurrent.get("simultaneous_loaded_motion_s"),
                                     "both_loaded_intervals_count": len(concurrent.get("intervals_s") or []),
                                     "all_three_body_motion_sim_s": evaluation.get("all_three_body_motion_sim_s")},
        "resources": {"events": resource_events, "apron_acquire_order": apron_order,
                      "apron_exclusion_errors": apron_errors,
                      "solo_resource_wait_sim_s": (result.get("realtime_control_stats") or {}).get("solo_resource_wait_sim_s")},
        "solo": {"decision_count": len(decisions), "carry_decision_count": len(carry_decisions),
                 "carry_decision_span_s": max(carry_times) - min(carry_times) if carry_times else None,
                 "phase_counts": dict(Counter(str(row.get("phase_before")) for row in decisions)),
                 "moving_command_coverage": coverage(intervals(solo)),
                 "original_moving_command_coverage": coverage(intervals(solo, renewal=False)),
                 "pending_renewal": renewal_audit,
                 "own_rgb_attachment_evidence_rows": len(attachments),
                 "own_rgb_attachment_attached_counts": dict(held),
                 "own_rgb_attachment_scope": "recorded image-derived estimates, not a contact sensor"},
        "core_files": core, "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", "--run", dest="run", type=Path, required=True)
    parser.add_argument("--manager", dest="manager_manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="write audit JSON outside the raw run")
    args = parser.parse_args()
    run = args.run.resolve()
    if args.output and (args.output.resolve() == run or run in args.output.resolve().parents):
        parser.error("audit output must be outside the raw run")
    report = audit(run, args.manager_manifest.resolve())
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    else:
        print(payload, end="")
    if report["errors"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
