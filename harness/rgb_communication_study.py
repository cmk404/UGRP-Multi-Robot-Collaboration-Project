"""Assembly and bounded execution of the RGB communication study.

Every trial is a disposable subprocess with an independently enforced wall
deadline. Actor/runtime evidence and referee evidence are written separately.
Factories below are explicit project adapters, never arbitrary import paths
from a manifest. Importing this module does not start a model or simulator.
"""
from __future__ import annotations

import hashlib
from collections import Counter, defaultdict
from copy import deepcopy
import importlib
import inspect
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from harness.rgb_execution_bundle import environment_fingerprint
from harness.rgb_communication_evaluation import (
    ContractError, EVALUATOR_SCHEMA, CONDITIONS, evaluate_run,
)


STUDY_SCHEMA = "rgb-communication-study.v2"
RESULT_SCHEMA = "rgb-communication-study-result.v2"
BACKEND_MODULE = "harness.rgb_skill_execution"
RUNTIME_MODULE = "harness.rgb_communication_runtime"
PLANNER_MODULE = "harness.rgb_communication_planner"
SCHEDULER_ID = "rgb-independent-async.v1"
PHYSICAL_BUDGETS = {
    "sim_time_s": 300, "wall_time_s": 600, "job_wall_time_s": 1800,
    "model_calls": 0, "input_tokens": 0, "output_tokens": 0,
    "commands": 6000,
}
TRIAL_SCHEMA = "rgb-communication-trial-result.v2"


def finite(value: Any, *, minimum: float = 0) -> bool:
    return type(value) in (int, float) and math.isfinite(value) and value >= minimum


def digest_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def digest_json(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError, UnicodeError) as exc:
        raise ContractError(f"unreadable JSON: {path.name}") from exc
    if not isinstance(value, dict):
        raise ContractError(f"expected JSON object: {path.name}")
    return value


def write_new_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def source_state(root: Path) -> dict:
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip()
    return {"git_sha": sha, "clean": not status}


def checked_reference(root: Path, reference: dict) -> Path:
    if not isinstance(reference, dict) or set(reference) != {"path", "sha256"}:
        raise ContractError("artifact reference must contain path and sha256")
    if not isinstance(reference["path"], str) or not isinstance(reference["sha256"], str):
        raise ContractError("artifact reference path/hash must be strings")
    relative = Path(reference["path"])
    path = root / relative
    if relative.is_absolute() or ".." in relative.parts or path.is_symlink() \
            or not path.resolve().is_relative_to(root.resolve()) or not path.is_file():
        raise ContractError("artifact reference is missing or outside its root")
    if digest_file(path) != reference["sha256"]:
        raise ContractError(f"artifact hash mismatch: {relative}")
    return path.resolve()


def source_file_hashes(root: Path) -> dict[str, str]:
    """Pin execution code/configuration, including imported existing RGB skills."""
    names = subprocess.check_output(["git", "ls-files", "-z", "harness", "sim", "scripts",
                                     "maps", "calibration", "config", "configs",
                                     "requirements-sim.txt"], cwd=root).decode().split("\0")
    return {name: digest_file(root / name) for name in sorted(filter(None, names))
            if (root / name).is_file() and (root / name).suffix in {".py", ".json", ".xml", ".txt"}}


def verify_local_assets(catalog: dict, descriptor: dict) -> None:
    """Explicit local-only hash binding; never relax generic evidence paths."""
    if set(catalog) != {"schema", "mode", "files"} \
            or catalog["schema"] != "rgb-local-assets.v1" \
            or catalog["mode"] != "read_only_reference" \
            or not isinstance(catalog["files"], dict) or not catalog["files"]:
        raise ContractError("invalid read-only local asset catalog")
    for name, expected in catalog["files"].items():
        if not isinstance(name, str) or not isinstance(expected, str):
            raise ContractError("invalid local asset path/hash")
        path = Path(name)
        if not path.is_absolute() or path.is_symlink() or not path.is_file() \
                or str(path.resolve()) != name or digest_file(path) != expected:
            raise ContractError("local asset changed or unavailable")
    hashes = descriptor.get("input_hashes")
    if not isinstance(hashes, dict) or not hashes \
            or any(catalog["files"].get(name) != expected for name, expected in hashes.items()):
        raise ContractError("local catalog does not bind every backend input")


def prepare_manifest(config: dict, root: Path) -> dict:
    stage = config.get("stage")
    if stage not in {"physical_replay", "llm_smoke", "pilot"}:
        raise ContractError("stage must be physical_replay, llm_smoke, or pilot")
    backend_config = config.get("backend")
    selected_bundle = backend_config.get("execution_bundle_id") if isinstance(backend_config, dict) else None
    if not isinstance(selected_bundle, str) or not selected_bundle:
        raise ContractError("explicit backend execution_bundle_id required")
    state = source_state(root)
    if not state["clean"]:
        raise ContractError("commit all source/configuration before preparing the study")
    trials = config.get("trials")
    if not isinstance(trials, list) or not trials:
        raise ContractError("a finite, explicit trial list is required")
    ids = [row.get("run_id") for row in trials if isinstance(row, dict)]
    if len(ids) != len(trials) or not all(
        isinstance(value, str) and value and value.isascii()
        and all(c.isalnum() or c in "-_" for c in value) for value in ids
    ) or len(set(ids)) != len(ids):
        raise ContractError("trial IDs must be unique path-safe strings")
    manifest = {"schema_version": STUDY_SCHEMA, "stage": stage, "source": state,
                "source_files": source_file_hashes(root), "scheduler_id": SCHEDULER_ID,
                "execution_bundle_id": selected_bundle,
                "config": config, "config_sha256": digest_json(config),
                "planned_denominator": len(trials), "automatic_retry": False}
    # Structural validity is not a declaration of live readiness.
    return manifest


def provider_settings(config: dict):
    """No approved input-bound capability exists yet: never invent one.

    JSON cannot register executable callbacks or assert verified provider caps.
    Adding a real resolver requires separately tested, source-pinned evidence.
    """
    allowed = {"model", "max_output_tokens", "timeout_s", "temperature", "reasoning_effort"}
    if not isinstance(config, dict) or set(config) - allowed or not config.get("model"):
        raise ContractError("provider requires an explicit model and supported settings")
    module = importlib.import_module(PLANNER_MODULE)
    return module.ProviderSettings(**config)


def assess_environment(config: dict, evidence: dict, *, root: Path, evidence_root: Path,
                       descriptor: dict, source_sha: str) -> dict:
    """Bind A's split/exposure review to B's actual support descriptor.

    The full 22-case suite is inspected offline, not rolled out or trained on.
    A reviewed legacy development scene remains a smoke, not a new test map.
    """
    blockers = []
    if evidence.get("schema_version") != "rgb-study-environment.v1":
        raise ContractError("unknown environment evidence schema")
    if evidence.get("source_sha") != source_sha:
        blockers.append("environment_source_mismatch")
    suite = importlib.import_module("sim.act_map_suite")
    _, cases = suite.load_suite(checked_reference(root, evidence["suite"]))
    scenarios = importlib.import_module("harness.rgb_communication_scenarios")
    provenance = read_json(checked_reference(evidence_root, evidence["provenance"]))
    assignments = evidence["scenario_assignments"]
    exposure = scenarios.audit_exposure(cases, provenance, assignments,
        allow_legacy_dispatch_open=config["stage"] == "physical_replay"
            and evidence.get("map", {}).get("map_id") == "dispatch_open",
        allow_unknown_diagnostic_origins=config["stage"] == "physical_replay")
    if not exposure.get("valid"):
        blockers.extend("exposure:" + value for value in exposure.get("blockers", ["invalid"]))
    # The map ID/version and authored geometry must match the actual builder,
    # not just an unrelated suite entry or an "open" fallback.
    map_record = evidence.get("map", {})
    for key in ("map_id", "map_version", "map_instance_sha256", "map_group_sha256"):
        if not map_record.get(key) or descriptor.get(key) != map_record[key]:
            blockers.append("environment_backend_mismatch:" + key)
    if map_record.get("stratum") not in {"same_map_new_layout", "test_a", "test_b", "development", "regression"}:
        blockers.append("environment_stratum_missing")
    if map_record.get("stratum") in {"test_a", "test_b"}:
        blockers.append("holdout_rollout_not_authorized_in_connection_smoke")
    for trial in config["trials"]:
        if any(trial.get(key) != map_record.get(key) for key in
               ("map_instance_sha256", "map_group_sha256", "stratum")):
            blockers.append("trial_environment_identity_mismatch:" + trial["run_id"])
    # Reviewed source/data/checkpoint/prompt identities are mandatory even for
    # a legacy development scene. Foundation provenance may remain unknown.
    for kind in ("data", "checkpoint", "prompt"):
        records = evidence.get("provenance_artifacts", {}).get(kind)
        if not isinstance(records, list) or not records:
            blockers.append("provenance_missing:" + kind)
            continue
        for reference in records:
            checked_reference(evidence_root, reference)
    environment_review = read_json(checked_reference(evidence_root, evidence["review"]))
    checker = getattr(scenarios, "assess_environment_readiness", None)
    if checker is None:
        blockers.append("independent_environment_checker_unavailable")
        reviewed = None
    else:
        reviewed = checker(environment_review, cases, expected_source_sha=source_sha,
                           artifact_root=evidence_root, expected_backend_id=config["backend_id"],
                           purpose="physical_replay" if config["stage"] == "physical_replay" else "live",
                           allow_legacy_dispatch_open=config["stage"] == "physical_replay"
                               and map_record.get("map_id") == "dispatch_open")
        if not reviewed.get("ready"):
            blockers.extend("environment:" + value for value in reviewed.get("blockers", ["not_ready"]))
    return {"ready": not blockers, "blockers": blockers, "map": map_record,
            "exposure": exposure, "independent_environment": reviewed,
            "suite_case_count": len(cases), "claim_scope": config["claim_scope"]}


def stratified_report(rows: list[dict], trials: list[dict]) -> list[dict]:
    """Preserve every assigned run and distinguish failure labels from guesses."""
    by_id = {row["run_id"]: row for row in rows}
    groups = defaultdict(list)
    for trial in trials:
        groups[(trial.get("map_group_sha256"), trial.get("stratum", "unknown"),
                trial["condition"])].append(by_id.get(trial["run_id"], {"outcome": "unrun"}))
    result = []
    for (group, stratum, condition), members in sorted(groups.items(), key=lambda pair: str(pair[0])):
        outcomes = Counter(row.get("outcome", "unrun") for row in members)
        causes = Counter()
        reached = measured = 0
        successful_wall = []
        for row in members:
            scored = row.get("result", {})
            external = scored.get("evaluator_verdict", {})
            cause = external.get("failure_category")
            causes[cause if cause in {"environment", "controller", "negotiation"} else "unclassified"] += 1
            recovery = external.get("recovery", {})
            if type(recovery.get("reached")) is bool:
                reached += recovery["reached"]
                measured += 1
            wall = scored.get("metrics", {}).get("wall_time_s")
            if row.get("outcome") == "success" and finite(wall):
                successful_wall.append(wall)
        result.append({"map_group_sha256": group, "stratum": stratum, "condition": condition,
                       "planned_n": len(members), "outcomes": dict(outcomes),
                       "failure_categories": dict(causes), "event_reached_n": reached,
                       "event_measured_n": measured, "successful_wall_time_s": successful_wall})
    return result


def preflight(manifest: dict, *, root: Path, evidence_root: Path) -> dict:
    blockers, checked = [], []
    if manifest.get("schema_version") != STUDY_SCHEMA:
        return {"ready": False, "blockers": ["unsupported_study_schema"], "checked": []}
    state = source_state(root)
    if manifest.get("source") != state or not state["clean"]:
        blockers.append("source_sha_or_clean_state_mismatch")
    config = manifest.get("config", {})
    if not isinstance(config, dict):
        return {"ready": False, "blockers": ["configuration_invalid"], "checked": []}
    if digest_json(config) != manifest.get("config_sha256"):
        blockers.append("configuration_hash_mismatch")
    if not isinstance(config.get("backend"), dict) or not config["backend"].get("execution_bundle_id") \
            or manifest.get("execution_bundle_id") != config["backend"].get("execution_bundle_id"):
        blockers.append("execution_bundle_selection_missing_or_mismatched")
    files = manifest.get("source_files")
    if not isinstance(files, dict) or not files:
        blockers.append("source_file_hashes_missing")
    else:
        for path, sha in files.items():
            try:
                checked_reference(root, {"path": path, "sha256": sha})
            except ContractError:
                blockers.append(f"source_file_changed:{path}")
        if files != source_file_hashes(root):
            blockers.append("source_dependency_set_mismatch")
    trials = config.get("trials", [])
    if not isinstance(trials, list) or any(not isinstance(row, dict) for row in trials):
        return {"ready": False, "blockers": blockers + ["trials_invalid"], "checked": checked}
    if not trials or manifest.get("planned_denominator") != len(trials):
        blockers.append("planned_denominator_mismatch")
    if manifest.get("scheduler_id") != SCHEDULER_ID or manifest.get("automatic_retry") is not False:
        blockers.append("scheduler_or_retry_policy_mismatch")
    stage, budgets = manifest.get("stage"), config.get("budgets", {})
    if stage != config.get("stage"):
        blockers.append("stage_mismatch")
    if not isinstance(budgets, dict):
        return {"ready": False, "blockers": blockers + ["budgets_invalid"], "checked": checked}
    ids = [row.get("run_id") for row in trials]
    if not all(isinstance(value, str) and value and value.isascii()
               and all(c.isalnum() or c in "-_" for c in value) for value in ids) \
            or len(set(map(str, ids))) != len(ids):
        blockers.append("trial_id_invalid_or_duplicate")
    for row in trials:
        if row.get("condition") not in CONDITIONS:
            blockers.append("condition_invalid")
        if type(row.get("seed")) is not int:
            blockers.append("seed_not_fixed")
        if not isinstance(row.get("common_task"), dict):
            blockers.append("actor_static_task_missing")
        if row.get("seed") != config.get("backend", {}).get("seed"):
            blockers.append("trial_reset_seed_differs_from_frozen_backend")
    if stage == "physical_replay":
        if len(trials) != 2 or {row.get("replay_kind") for row in trials} != {"solo", "joint"}:
            blockers.append("physical_replay_requires_solo_and_joint_exactly_once")
        for key, maximum in PHYSICAL_BUDGETS.items():
            value = budgets.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0 \
                    or value > maximum or (maximum > 0 and value == 0):
                blockers.append(f"physical_budget_invalid:{key}")
        if any(row.get("condition") != "none" for row in trials):
            blockers.append("physical_replay_requires_no_model_no_communication")
    elif stage not in {"llm_smoke", "pilot"}:
        blockers.append("unsupported_stage")
    else:
        for key in ("model_calls", "input_tokens", "output_tokens", "messages", "message_bytes"):
            if type(budgets.get(key)) is not int or budgets[key] <= 0:
                blockers.append(f"live_budget_required:{key}")
        if stage == "llm_smoke" and len(trials) != 1:
            blockers.append("llm_smoke_requires_exactly_one_trial")
        if stage == "pilot":
            pairs = [(row.get("scenario_id"), row.get("condition")) for row in trials]
            names = {pair[0] for pair in pairs}
            if len(trials) != 6 or len(names) != 2 or set(pairs) != {
                (name, condition) for name in names for condition in CONDITIONS
            }:
                blockers.append("pilot_requires_two_scenarios_three_conditions")
    for key in ("sim_time_s", "wall_time_s", "job_wall_time_s", "commands"):
        value = budgets.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            blockers.append(f"finite_budget_required:{key}")
    for field in ("map", "physics", "camera", "calibration"):
        reference = config.get("inputs", {}).get(field)
        try:
            checked_reference(root, reference)
            checked.append({"input": field, **reference})
        except (ContractError, TypeError):
            blockers.append(f"fixed_input_unverified:{field}")
    if not isinstance(config.get("submitter"), str) or not config["submitter"].strip():
        blockers.append("single_submitter_not_named")
    if config.get("claim_scope") != "development_connection_smoke":
        blockers.append("expanded_cohort_not_authorized")
    descriptor = {}
    try:
        backend = importlib.import_module(BACKEND_MODULE)
        descriptor = backend.backend_descriptor(config.get("backend", {}))
        if descriptor.get("execution_bundle_id") != manifest.get("execution_bundle_id") \
                or not descriptor.get("execution_bundle_sha256"):
            blockers.append("execution_bundle_descriptor_mismatch")
        if descriptor.get("synthetic") is not False or descriptor.get("weld") is not False:
            blockers.append("backend_is_synthetic_or_weld_not_off")
        if descriptor.get("camera_fov_changed") is not False:
            blockers.append("backend_camera_geometry_not_preserved")
        if descriptor.get("clock_owner") != "single_simulator":
            blockers.append("backend_clock_owner_not_unique")
        if descriptor.get("supervisor_clock_schema") != "ugrp.execution_clock.v1":
            blockers.append("backend_supervisor_clock_unavailable")
        if descriptor.get("backend_id") != config.get("backend_id"):
            blockers.append("backend_identity_mismatch")
        public_task = backend.public_static_context(config.get("backend", {}))["task"]
        if any(row.get("common_task") != public_task for row in trials):
            blockers.append("actor_task_differs_from_backend_public_task")
        for field, cap in (("max_sim_s", "sim_time_s"), ("max_commands", "commands")):
            if not finite(config.get("backend", {}).get(field), minimum=1) \
                    or config["backend"][field] > budgets.get(cap, 0):
                blockers.append("backend_budget_not_enforced:" + cap)
        checked.append({"backend": descriptor})
    except (ImportError, AttributeError, ValueError, RuntimeError, OSError) as exc:
        blockers.append(f"backend_unavailable:{type(exc).__name__}")
    if "backend_descriptor_evidence" in config or config.get("submitter") == "D3":
        try:
            frozen = read_json(checked_reference(evidence_root, config.get("backend_descriptor_evidence")))
            if frozen != descriptor:
                blockers.append("backend_descriptor_changed")
        except (ContractError, TypeError) as exc:
            blockers.append(f"backend_descriptor_evidence_invalid:{type(exc).__name__}")
    if "local_assets" in config or config.get("submitter") == "D3":
        try:
            catalog = read_json(checked_reference(evidence_root, config.get("local_assets")))
            verify_local_assets(catalog, descriptor)
        except (ContractError, OSError, TypeError) as exc:
            blockers.append(f"local_assets_invalid:{type(exc).__name__}")
    try:
        runtime = importlib.import_module("harness.rgb_communication_async")
        limits = runtime.AsyncRuntimeLimits(**config.get("runtime_limits", {}))
        limits.validate()
        expected_timing = descriptor.get("execution_contract", {})
        for attr, expected_key in (("tick_period_s", "study_tick_period_s"),
                                   ("poll_period_s", "study_poll_period_s")):
            if getattr(limits, attr, None) != expected_timing.get(expected_key):
                blockers.append("execution_bundle_runtime_timing_mismatch:" + attr)
        if "clock_snapshot" not in inspect.signature(runtime.run_rgb_communication_async).parameters:
            blockers.append("async_supervisor_clock_not_connected")
        if limits.wall_timeout_s > budgets["wall_time_s"] - 5:
            blockers.append("runtime_wall_deadline_must_allow_cleanup")
        if limits.max_ticks * limits.tick_period_s > budgets["sim_time_s"] + 1e-6:
            blockers.append("runtime_sim_budget_exceeded")
        if limits.max_concurrent_requests > 3:
            blockers.append("too_many_concurrent_actor_calls")
        if stage != "physical_replay":
            if limits.max_calls_per_robot * 3 > budgets["model_calls"]:
                blockers.append("runtime_budget_not_enforced:model_calls")
            for attr, budget in (("max_messages_per_robot", "messages"),
                                 ("max_message_bytes_per_robot", "message_bytes")):
                if getattr(limits, attr, float("inf")) * 3 > budgets[budget]:
                    blockers.append(f"runtime_budget_not_enforced:{budget}")
            for attr, budget in (("max_input_tokens", "input_tokens"),
                                 ("max_output_tokens", "output_tokens")):
                if getattr(limits, attr, float("inf")) > budgets[budget]:
                    blockers.append(f"runtime_budget_not_enforced:{budget}")
    except (ImportError, AttributeError, TypeError, ValueError, KeyError) as exc:
        blockers.append(f"async_limits_unavailable:{type(exc).__name__}")
    # E0 is evidence-bearing: a declared split or boolean is never enough.
    try:
        environment = read_json(checked_reference(evidence_root, config.get("environment_evidence")))
        gate = assess_environment(config, environment, root=root, evidence_root=evidence_root,
                                  descriptor=descriptor, source_sha=state["git_sha"])
        blockers.extend(gate["blockers"])
        checked.append({"environment": gate})
    except (ImportError, AttributeError, ContractError, KeyError, TypeError, ValueError) as exc:
        blockers.append(f"environment_evidence_invalid:{type(exc).__name__}")
    try:
        scenarios = importlib.import_module("harness.rgb_communication_scenarios")
        boundary = read_json(checked_reference(evidence_root, config.get("offline_boundary_evidence")))
        if config.get("submitter") == "D3" and (
            boundary.get("schema_version") != "rgb-offline-boundary-review.v2"
            or boundary.get("independent_reviewer") != "A3"
        ):
            blockers.append("d3_requires_independent_a3_boundary_v2")
        verifier = getattr(scenarios, "assess_offline_boundary", None)
        if verifier is None:
            blockers.append("offline_boundary_checker_unavailable")
        else:
            audit = verifier(boundary, expected_source_sha=state["git_sha"],
                             expected_config_sha256=digest_json({k: v for k, v in config.items()
                                                                 if k != "offline_boundary_evidence"}),
                             expected_components=config.get("components", {}),
                             artifact_root=evidence_root, source_root=root,
                             **({"required_schema": "rgb-offline-boundary-review.v2"}
                                if config.get("submitter") == "D3" else {}))
            if not audit.get("ready"):
                blockers.extend("offline_boundary:" + b for b in audit.get("blockers", ["not_ready"]))
            checked.append({"offline_boundary": audit})
    except (ImportError, AttributeError, ContractError, KeyError, TypeError, ValueError) as exc:
        blockers.append(f"offline_boundary_evidence_invalid:{type(exc).__name__}")
    if stage != "physical_replay":
        try:
            runtime = importlib.import_module(RUNTIME_MODULE)
            if not callable(runtime.run_rgb_communication_async):
                blockers.append("async_runtime_not_connected")
            settings = provider_settings(config.get("provider", {}))
            provider = settings.readiness()
            policy = read_json(checked_reference(evidence_root, config.get("provider_policy")))
            if settings.policy_manifest() != policy:
                blockers.append("provider_policy_hash_or_model_mismatch")
            if not provider.get("ready"):
                blockers.extend(f"provider:{value}" for value in provider.get("blockers", ["not_ready"]))
            checked.append({"provider": provider})
        except (ImportError, AttributeError, ValueError) as exc:
            blockers.append(f"planner_or_runtime_unavailable:{type(exc).__name__}")
        try:
            scenarios = importlib.import_module("harness.rgb_communication_scenarios")
            for trial in trials:
                spec = read_json(checked_reference(root, trial["scenario"]))
                evidence = read_json(checked_reference(evidence_root, trial["readiness_evidence"]))
                readiness = scenarios.assess_readiness(spec, evidence,
                    expected_source_sha=state["git_sha"], artifact_root=evidence_root, source_root=root)
                if not readiness["ready"]:
                    blockers.extend(f"scenario:{trial['run_id']}:{value}" for value in readiness["blockers"])
                checked.append({"scenario": readiness})
        except (ImportError, AttributeError, KeyError, ValueError, OSError) as exc:
            blockers.append(f"independent_readiness_evidence_invalid:{type(exc).__name__}")
    return {"schema_version": "rgb-study-preflight.v2", "ready": not blockers,
            "blockers": blockers, "checked": checked, "stage": stage,
            "source_sha": state["git_sha"], "manifest_sha256": digest_json(manifest)}


class ReplayDecision:
    """One actor's explicit fixed diagnostic script, not role negotiation.

    Scripts contain only allowed action envelopes. There is no referee access,
    truth-based repair, success claim, automatic replanning, or retry.
    """
    def __init__(self, actions: list[dict]):
        self.actions = deepcopy(actions)
        self.index = 0

    def __call__(self, request: dict) -> dict:
        if self.index < len(self.actions):
            action = self.actions[self.index]
            self.index += 1
        else:
            status = request.get("local_status", {})
            own_skill = status.get("own_skill_status", {})
            # Even RGB skill terminal is not measured delivery. End the fixed
            # diagnostic with cannot_continue, never manufacture a success claim.
            terminal = own_skill.get("state") in {"FINISHED_UNVERIFIED", "STOPPED"}
            action = {"kind": "finish", "claim": "cannot_continue"} if terminal else {"kind": "wait"}
        return {"action": action, "message": None}


def supervisor_clock_callback(bundle):
    """Only pass B's clock-only callback; never derive it from evaluator state."""
    callback = getattr(bundle, "clock_snapshot", None)
    if not callable(callback):
        raise ContractError("backend supervisor clock callback is unavailable")
    return callback


def run_trial(manifest: dict, *, run_id: str, root: Path, evidence_root: Path, output: Path) -> dict:
    """Only invoked in the parent's wall-bounded disposable subprocess."""
    readiness = preflight(manifest, root=root, evidence_root=evidence_root)
    if not readiness["ready"]:
        raise ContractError("trial blocked: " + ", ".join(readiness["blockers"]))
    if source_state(root) != manifest["source"] or source_file_hashes(root) != manifest["source_files"]:
        raise ContractError("trial source drift")
    config = manifest["config"]
    if digest_json(config) != manifest["config_sha256"]:
        raise ContractError("trial configuration drift")
    matches = [(i, row) for i, row in enumerate(config["trials"], 1) if row["run_id"] == run_id]
    if len(matches) != 1:
        raise ContractError("trial not uniquely present in fixed manifest")
    order, trial = matches[0]
    plan = {**deepcopy(trial), "order": order, "repeat": trial.get("repeat", 0),
            "artifact_relpath": ".", "required_artifacts": {
                "runtime": "runtime.jsonl", "evaluator": "evaluator.json"}}
    write_new_json(output / "plan.json", plan)
    backend_module = importlib.import_module(BACKEND_MODULE)
    async_module = importlib.import_module("harness.rgb_communication_async")
    runtime = importlib.import_module(RUNTIME_MODULE)
    limits = async_module.AsyncRuntimeLimits(**config["runtime_limits"])
    limits.validate()
    backend_config = {**config["backend"], "output_dir": str(output / "backend"),
                      "seed": trial["seed"]}
    bundle = backend_module.build_rgb_skill_backend(backend_config)
    bundle_provenance = bundle.provenance
    if bundle_provenance.get("execution_bundle_id") != manifest["execution_bundle_id"] \
            or not bundle_provenance.get("execution_bundle_sha256") \
            or not isinstance(bundle_provenance.get("effective_execution"), dict) \
            or not isinstance(bundle_provenance.get("diff_from_historical_f1"), dict) \
            or bundle_provenance.get("execution_source") != {"git_sha": manifest["source"]["git_sha"], "dirty": False}:
        bundle.close()
        raise ContractError("backend execution bundle evidence missing or mismatched")
    evidence_kind = "deterministic_physical_replay" if manifest["stage"] == "physical_replay" else "live_llm"
    try:
        clock_snapshot = supervisor_clock_callback(bundle)
        write_new_json(output / "backend-provenance.json", bundle_provenance)
        if manifest["stage"] == "physical_replay":
            scripts = trial["replay_actions"]
            if set(scripts) != {"r1", "r2", "r3"}:
                raise ContractError("replay requires three explicit actor scripts")
            planners = {rid: async_module.OfflineDecisionPlanner(ReplayDecision(actions),
                        evidence_kind=evidence_kind) for rid, actions in scripts.items()}
        else:
            settings = provider_settings(config["provider"])
            if not settings.readiness()["ready"]:
                raise ContractError("provider readiness not established")
            planners = importlib.import_module(PLANNER_MODULE).make_rgb_planners(
                settings, output / "requests", evidence_kind=evidence_kind)
        # Only the explicitly authored public task enters actors. The manifest,
        # split, map seed, evaluator, exposure ledger and referee NEVER do.
        runtime.run_rgb_communication_async(bundle.actor_port, planners,
            clock_snapshot=clock_snapshot,
            condition=trial["condition"], common_task=trial["common_task"], run_id=run_id,
            limits=limits, trace_path=output / "runtime.jsonl", artifact_dir=output / "runtime-inputs",
            provenance={"source_sha": manifest["source"]["git_sha"],
                        "manifest_sha256": digest_json(manifest), "evidence_kind": evidence_kind,
                        "execution_bundle_id": bundle_provenance["execution_bundle_id"],
                        "execution_bundle_sha256": bundle_provenance["execution_bundle_sha256"]})
        snapshot = bundle.evaluation_snapshot()
        write_new_json(output / "evaluator.json", {
            "schema_version": EVALUATOR_SCHEMA, "run_id": run_id, "condition": trial["condition"],
            "scope": evidence_kind + "; evaluation-only after runtime shutdown",
            "source_snapshot": snapshot})
    except Exception as exc:
        # Preserve partial logs; do not manufacture a terminal or evaluator.
        write_new_json(output / "trial-error.json", {"error_type": type(exc).__name__,
                                                     "raw_error_omitted": "may contain provider secrets"})
    finally:
        bundle.close()
    scored = evaluate_run(plan, output)
    result = {**scored, "schema_version": TRIAL_SCHEMA,
              "source_sha": manifest["source"]["git_sha"], "config_sha256": manifest["config_sha256"],
              "execution_bundle_id": bundle_provenance["execution_bundle_id"],
              "execution_bundle_sha256": bundle_provenance["execution_bundle_sha256"],
              "effective_execution": bundle_provenance["effective_execution"],
              "diff_from_historical_f1": bundle_provenance["diff_from_historical_f1"],
              "execution_environment_sha256": bundle_provenance["execution_environment"]["sha256"],
              "evidence_kind": evidence_kind, "claim_scope": config["claim_scope"],
              "source_artifact_hashes": {name: digest_file(output / name)
                  for name in ("plan.json", "runtime.jsonl", "evaluator.json") if (output / name).is_file()}}
    if manifest["stage"] == "physical_replay":
        targets = trial.get("replay_target_objects", [])
        objects = scored.get("evaluator_verdict", {}).get("objects", {})
        measured = bool(targets) and all(type(objects.get(obj, {}).get("physical_success")) is bool
                                        for obj in targets)
        result["replay_target_objects"] = targets
        result["replay_goal_complete"] = all(objects[obj]["physical_success"] for obj in targets) if measured else None
        result["replay_goal_scope"] = "selected diagnostic objects only; does not override full mission_complete"
    write_new_json(output / "result.json", result)
    return result


def bounded_process(command: list[str], *, cwd: Path, log_path: Path, timeout_s: float,
                    cleanup_grace_s: float = 5) -> dict:
    """Terminate only the child process group owned by this trial on timeout."""
    if not finite(timeout_s, minimum=.001) or not finite(cleanup_grace_s, minimum=.001) \
            or cleanup_grace_s > 10:
        raise ContractError("finite timeout and cleanup grace <=10 seconds required")
    started = time.monotonic()
    def group_alive(pid):
        try:
            os.killpg(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            # A transient/denied probe is not evidence that a group is gone.
            return True

    def finalize(child):
        deadline = time.monotonic() + cleanup_grace_s
        if group_alive(child.pid):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        # Reserve part of the existing grace for KILL and its actual drain.
        soft_end = deadline - min(.5, cleanup_grace_s / 2)
        while time.monotonic() < soft_end:
            child.poll()
            if not group_alive(child.pid):
                break
            time.sleep(.01)
        if group_alive(child.pid):
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        while time.monotonic() < deadline:
            child.poll()
            if not group_alive(child.pid):
                break
            time.sleep(.01)
        child.poll()
        return child.returncode is not None, not group_alive(child.pid)

    if threading.current_thread() is not threading.main_thread():
        raise ContractError("owned process supervision requires the main signal-handling thread")
    requested_interrupt = False
    child = None
    reaped = group_gone = False
    cleanup_attempted = False

    def interrupted(_signal, _frame):
        nonlocal requested_interrupt
        requested_interrupt = True
        # Never raise asynchronously across Popen/handle assignment or cleanup.
        # The bounded wait polls this flag; repeated INT/TERM cannot tear reap.

    previous = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        with log_path.open("x") as log:
            child = subprocess.Popen(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            timed_out = False
            try:
                deadline = started + timeout_s
                while True:
                    if requested_interrupt:
                        raise KeyboardInterrupt
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise subprocess.TimeoutExpired(command, timeout_s)
                    try:
                        code = child.wait(timeout=min(.1, remaining))
                        break
                    except subprocess.TimeoutExpired:
                        continue
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                timed_out = isinstance(exc, subprocess.TimeoutExpired)
                cleanup_attempted = True
                reaped, group_gone = finalize(child)
                code = 124 if timed_out else 130
            else:
                reaped, group_gone = True, not group_alive(child.pid)
                if not group_gone:
                    # A dead leader is not proof that its descendants stopped.
                    cleanup_attempted = True
                    reaped, group_gone = finalize(child)
                    code = code or 125
    finally:
        if child is not None and not reaped and not cleanup_attempted:
            cleanup_attempted = True
            reaped, group_gone = finalize(child)
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if requested_interrupt and code == 0:
        code = 130
    return {"exit_code": code, "timed_out": timed_out,
            "wall_time_s": time.monotonic() - started, "process_group": child.pid,
            "cleanup_grace_s": cleanup_grace_s, "child_reaped": reaped,
            "process_group_gone": group_gone}


def run_study(manifest: dict, *, root: Path, evidence_root: Path, output: Path,
              allocation_wall_s: float | None = None) -> dict:
    readiness = preflight(manifest, root=root, evidence_root=evidence_root)
    if not readiness["ready"]:
        raise ContractError("study blocked: " + ", ".join(readiness["blockers"]))
    backend_check = next((row["backend"] for row in readiness["checked"] if "backend" in row), {})
    if not backend_check.get("execution_bundle_sha256"):
        raise ContractError("preflight omitted execution bundle identity")
    output.mkdir(parents=True, exist_ok=False)
    # Exclusive directory creation is also the no-duplicate-submission lock.
    write_new_json(output / "manifest.json", manifest)
    write_new_json(output / "preflight.json", readiness)
    write_new_json(output / "environment.json", {**environment_fingerprint(),
        "source_sha": manifest["source"]["git_sha"],
        "execution_bundle_id": manifest["execution_bundle_id"],
        "execution_bundle_sha256": backend_check.get("execution_bundle_sha256")})
    started = time.monotonic()
    config = manifest["config"]
    allowed_wall = config["budgets"]["job_wall_time_s"]
    if allocation_wall_s is not None:
        if not finite(allocation_wall_s, minimum=1) or allocation_wall_s > allowed_wall:
            raise ContractError("allocation can only reduce the frozen wall budget")
        allowed_wall = allocation_wall_s
    rows = []
    halt_reason = None
    finalized = True
    for index, trial in enumerate(config["trials"]):
        remaining = allowed_wall - (time.monotonic() - started)
        trial_dir = output / "runs" / trial["run_id"]
        if halt_reason or remaining <= 5:
            rows.append({"run_id": trial["run_id"], "outcome": "unrun",
                         "reason": halt_reason or "job_wall_budget"})
            continue
        trial_dir.mkdir(parents=True, exist_ok=False)
        command = [sys.executable, "-m", "scripts.run_rgb_communication_study", "trial",
                   "--manifest", str((output / "manifest.json").resolve()),
                   "--run-id", trial["run_id"], "--output", str(trial_dir.resolve()),
                   "--evidence-root", str(evidence_root.resolve())]
        process = bounded_process(command, cwd=root, log_path=trial_dir / "process.log",
                                  timeout_s=min(config["budgets"]["wall_time_s"], remaining) - 5)
        interrupted = process["exit_code"] == 130 or process["exit_code"] < 0
        finalized = finalized and process.get("child_reaped") is True \
            and process.get("process_group_gone") is True
        if not finalized:
            halt_reason = "child_cleanup_unconfirmed"
        elif interrupted:
            halt_reason = "study_interrupted"
        elif process["timed_out"]:
            halt_reason = "previous_trial_wall_timeout"
        elif process["exit_code"]:
            halt_reason = "previous_trial_process_failed"
        write_new_json(trial_dir / "process.json", process)
        if not finalized:
            result = {"outcome": "aborted", "reason": "child_cleanup_unconfirmed"}
        elif (trial_dir / "result.json").is_file():
            result = read_json(trial_dir / "result.json")
            if result.get("run_id") != trial["run_id"] or result.get("schema_version") != TRIAL_SCHEMA:
                result = {"outcome": "invalid_artifact", "reason": "run_identity_mismatch"}
            else:
                try:
                    for name, sha in result["source_artifact_hashes"].items():
                        checked_reference(trial_dir, {"path": name, "sha256": sha})
                except (KeyError, ContractError):
                    result = {"outcome": "invalid_artifact", "reason": "source_artifact_hash_mismatch"}
            if process["timed_out"]:
                result = {**result, "outcome": "timeout", "reason": "parent_wall_deadline",
                          "child_outcome": result.get("outcome"), "mission_complete": False}
            elif process["exit_code"]:
                result = {**result, "outcome": "aborted", "reason": "child_process_failed",
                          "child_outcome": result.get("outcome"), "mission_complete": False}
        else:
            result = {"outcome": "timeout" if process["timed_out"] else
                      "aborted" if process["exit_code"] else "missing_artifact",
                      "reason": "study_interrupted" if interrupted else "trial_did_not_produce_result"}
        rows.append({"run_id": trial["run_id"], "condition": trial["condition"],
                     "process": process, "result": result, "outcome": result.get("outcome")})
        write_new_json(output / f"progress-{index + 1:03d}.json", {"run_rows": rows,
                                                                "planned_denominator": len(config["trials"])})
    report = {"schema_version": RESULT_SCHEMA, "source_sha": manifest["source"]["git_sha"],
              "manifest_sha256": digest_json(manifest), "stage": manifest["stage"],
              "execution_bundle_id": manifest["execution_bundle_id"],
              "planned_denominator": len(config["trials"]), "run_rows": rows,
              "successes": sum(row.get("outcome") == "success" for row in rows),
              "map_strata": stratified_report(rows, config["trials"]),
              "wall_s": time.monotonic() - started,
              "allocated_job_wall_s": allowed_wall,
              "child_cleanup_confirmed": finalized,
              "scope": "physical replay is capability evidence; communication pilot is not a superiority test"}
    write_new_json(output / "report.json", report)
    if finalized:
        write_new_json(output / "artifact-hashes.json", {
            str(path.relative_to(output)): digest_file(path) for path in sorted(output.rglob("*"))
            if path.is_file() and not path.is_symlink()})
        # Only this final receipt attests completed hashing. report.json does
        # not predeclare success while the inventory may still be written.
        write_new_json(output / "artifact-finalization.json", {
            "schema": "rgb-study-artifact-finalization.v1", "complete": True,
            "source_sha": manifest["source"]["git_sha"],
            "inventory": {"path": "artifact-hashes.json", "sha256": digest_file(output / "artifact-hashes.json")}})
    return {**report, "artifact_hashes_finalized": finalized}
