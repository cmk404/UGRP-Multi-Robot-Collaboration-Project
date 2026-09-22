"""Executable, evaluator-side scenarios and evidence gates for the RGB study.

Nothing here sends scenario events, annotations, or evaluator state to actors.
Readiness is a file-backed audit gate, not a physical-success detector.  Passing
it cannot replace independently reviewing the referenced images and source.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import random
import re
from typing import Any, Mapping, Sequence


SCENARIO_SCHEMA = "rgb-communication-scenario.v1"
READINESS_SCHEMA = "rgb-scenario-readiness.v1"
LINK_SCHEMA = "rgb-scenario-links.v1"
CONDITIONS = ("none", "structured", "natural")
FAMILIES = ("normal", "contention", "recovery")
ROBOTS = ("r1", "r2", "r3")
AUDIT_CASES = (
    "actor_source_allowlist", "evaluator_noninterference", "recipient_isolation",
    "queued_and_delivered_ttl", "slow_peer_progress", "cancelled_late_reply",
    "duplicate_joint_intent", "lease_expiry_and_reuse", "queued_command_expiry",
    "finish_claim_separation", "request_decision_command_provenance",
)
A3_AUDIT_CASES = AUDIT_CASES + (
    "worker_total_wall_and_sim_age_at_consumption", "completed_worker_cancel_and_revision",
    "local_child_finalization_before_hashing", "scene_backend_definition_binding",
    "asset_and_parent_map_exposure_integrity",
    "observation_cache_capture_identity_and_isolation", "single_clock_service_fairness",
)
SOURCE_FILES = (
    "harness/rgb_execution_contract.py", "harness/rgb_execution_port.py",
    "harness/rgb_skill_execution.py", "harness/rgb_communication_runtime.py",
    "harness/rgb_communication_async.py",
    "harness/rgb_communication_planner.py", "harness/rgb_communication_study.py",
    "scripts/run_rgb_communication_study.py", "harness/rgb_communication_scenarios.py",
)
COMPONENT_KEYS = {"serializer_id", "backend_id", "scheduler_id"}
PIN_KEYS = {"setup_sha256", "map_sha256", "physics_sha256", "camera_sha256"}


class ScenarioError(ValueError):
    """Malformed or unverifiable research evidence (never actor feedback)."""


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _require(test: Any, message: str) -> None:
    if not test:
        raise ScenarioError(message)


def _text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _hex(value: Any, length: int = 64) -> bool:
    return isinstance(value, str) and bool(re.fullmatch(rf"[0-9a-f]{{{length}}}", value))


def _time(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and value >= 0)


def _exact(value: Any, keys: set[str], label: str) -> None:
    _require(isinstance(value, dict) and set(value) == keys, f"{label}: fields mismatch")


def validate_scenario(spec: Mapping[str, Any]) -> None:
    """Validate an executable *candidate*. Null execution pins remain NO-GO."""
    _exact(spec, {"schema_version", "scenario_id", "family", "pilot_candidate",
                  "selection_reason", "robot_ids", "mission", "event", "execution"}, "scenario")
    _require(spec["schema_version"] == SCENARIO_SCHEMA, "scenario schema mismatch")
    _require(_text(spec["scenario_id"]) and re.fullmatch(r"[a-z0-9-]+", spec["scenario_id"]),
             "unsafe scenario id")
    _require(spec["family"] in FAMILIES, "unknown scenario family")
    _require(type(spec["pilot_candidate"]) is bool, "pilot_candidate must be boolean")
    _require(_text(spec["selection_reason"]), "selection reason required")
    _require(spec["robot_ids"] == list(ROBOTS), "three equal robots required")
    _require(spec["mission"] == {"beam": 1, "box": 1, "beam_carriers": 2, "box_carriers": 1},
             "scenario must keep the 2+1 mission")
    event = spec["event"]
    _exact(event, {"event_id", "trigger", "visibility", "recognition", "minimum_frames",
                   "requires_revision", "physical_injection"}, "event")
    _require(_text(event["event_id"]) and _text(event["recognition"]), "event identity/criterion required")
    triggers = {"normal": "initial_scene", "contention": "competing_actor_intents",
                "recovery": "command_without_visual_progress"}
    _require(event["trigger"] == triggers[spec["family"]], "family trigger mismatch")
    _require(event["visibility"] in {"public_top", "own_rgb"}, "visibility requires an RGB channel")
    _require(event["physical_injection"] is None, "unvalidated physical injection is not supported")
    _require(type(event["requires_revision"]) is bool, "requires_revision must be boolean")
    _require(event["requires_revision"] == (spec["family"] != "normal"), "revision criterion mismatch")
    _require(type(event["minimum_frames"]) is int and event["minimum_frames"] >= 1,
             "positive minimum_frames required")
    if spec["family"] == "recovery":
        _require(event["minimum_frames"] >= 2, "stall recognition requires before/after RGB")
    execution = spec["execution"]
    _exact(execution, {"components", "pins", "required_source_files"}, "execution")
    _exact(execution["components"], COMPONENT_KEYS, "components")
    _require(all(v is None or _text(v) for v in execution["components"].values()), "invalid component id")
    _exact(execution["pins"], PIN_KEYS, "pins")
    _require(all(v is None or _hex(v) for v in execution["pins"].values()), "invalid execution hash")
    files = execution["required_source_files"]
    _require(isinstance(files, list) and all(_text(p) for p in files), "source paths required")
    _require(len(files) == len(set(files)) and set(SOURCE_FILES) <= set(files),
             "source audit must include execution, serializer, runtime and study runner")


def load_scenarios(path: str | Path) -> list[dict[str, Any]]:
    specs = json.loads(Path(path).read_text(encoding="utf-8"))
    _require(isinstance(specs, list) and len(specs) == 3, "catalog must have three scenario families")
    for spec in specs:
        validate_scenario(spec)
    _require({s["family"] for s in specs} == set(FAMILIES), "catalog families must be unique")
    _require(len({s["scenario_id"] for s in specs}) == 3, "duplicate scenario id")
    _require(sum(s["pilot_candidate"] for s in specs) == 2, "connection pilot needs two candidates")
    return specs


def _verified_file(ref: Any, root: Path, checked: list[str]) -> Path:
    _exact(ref, {"path", "sha256"}, "artifact reference")
    _require(_text(ref["path"]) and _hex(ref["sha256"]), "invalid artifact reference")
    relative = Path(ref["path"])
    _require(not relative.is_absolute() and ".." not in relative.parts, "artifact path escapes root")
    root = root.resolve()
    path = (root / relative).resolve()
    _require(path.is_relative_to(root) and path.is_file(), f"artifact missing or escaped: {relative}")
    _require(hashlib.sha256(path.read_bytes()).hexdigest() == ref["sha256"], f"artifact hash mismatch: {relative}")
    checked.append(str(relative))
    return path


def assess_offline_boundary(evidence: Mapping[str, Any], *, expected_source_sha: str,
                            expected_config_sha256: str, expected_components: Mapping[str, str],
                            artifact_root: Path, source_root: Path,
                            required_schema: str | None = None) -> dict[str, Any]:
    """Versioned pre-physics audit, without circular physical-pass prerequisites.

    Config hash excludes only the offline_boundary_evidence reference itself,
    avoiding a self-referential hash. D still pins the complete final manifest.
    This verifier does not issue a review or promote offline tests to live proof.
    """
    checked: list[str] = []
    blockers: list[str] = []
    try:
        _exact(evidence, {"schema_version", "scope", "verdict", "independent_reviewer",
                           "source_sha", "source_files", "config_sha256", "components",
                           "cases", "artifacts"}, "offline boundary review")
        versions = {"rgb-offline-boundary-review.v1": ("A2", AUDIT_CASES),
                    "rgb-offline-boundary-review.v2": ("A3", A3_AUDIT_CASES)}
        schema = evidence["schema_version"]
        _require(schema in versions and (required_schema is None or schema == required_schema),
                 "offline review schema mismatch")
        reviewer, required_cases = versions[schema]
        _require(evidence["scope"] == "offline" and evidence["verdict"] == "pass"
                 and evidence["independent_reviewer"] == reviewer, "independent offline pass missing")
        _require(_hex(expected_source_sha, 40) and evidence["source_sha"] == expected_source_sha, "offline source mismatch")
        _require(_hex(expected_config_sha256) and evidence["config_sha256"] == expected_config_sha256, "offline config mismatch")
        _exact(dict(expected_components), COMPONENT_KEYS, "expected components")
        _require(all(_text(v) for v in expected_components.values())
                 and evidence["components"] == expected_components, "offline component mismatch")
        _require(isinstance(evidence["source_files"], dict) and set(SOURCE_FILES) <= set(evidence["source_files"]), "offline source coverage missing")
        for path, sha in evidence["source_files"].items():
            _verified_file({"path": path, "sha256": sha}, source_root, checked)
        _require(isinstance(evidence["cases"], dict) and
                 all(evidence["cases"].get(case) == "pass" for case in required_cases), "offline boundary cases incomplete")
        _require(isinstance(evidence["artifacts"], list) and evidence["artifacts"], "offline raw audit evidence missing")
        for ref in evidence["artifacts"]:
            _verified_file(ref, artifact_root, checked)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        blockers.append(str(exc))
    return {"schema_version": "rgb-offline-boundary-result.v1", "ready": not blockers,
            "blockers": blockers, "checked_artifacts": checked, "live_readiness": False,
            "scope": "pre-physics offline boundary only"}


def assess_readiness(
    spec: Mapping[str, Any], evidence: Mapping[str, Any], *, expected_source_sha: str,
    artifact_root: Path, source_root: Path,
) -> dict[str, Any]:
    """Fail closed for live LLM use; physical diagnostic admission is D's job.

    Review artifacts are JSON records bound to the same source/spec/components.
    Their hashes, source files and RGB frames are read here. Review verdicts are
    still assertions by the named independent reviewer, not cryptographic trust.
    Do not fabricate review files merely to make this gate green.
    """
    checked: list[str] = []
    blockers: list[str] = []
    try:
        validate_scenario(spec)
        _require(_hex(expected_source_sha, 40), "expected source must be a full Git SHA")
        _exact(evidence, {"schema_version", "evidence_kind", "source_sha", "scenario_sha256",
                           "components", "pins", "source_files", "boundary_audit",
                           "physical_replays", "visual_witnesses"}, "readiness evidence")
        _require(evidence["schema_version"] == READINESS_SCHEMA, "readiness schema mismatch")
        _require(evidence["evidence_kind"] == "independent_review", "synthetic or unaudited evidence cannot admit live use")
        _require(evidence["source_sha"] == expected_source_sha, "source SHA mismatch")
        _require(evidence["scenario_sha256"] == canonical_sha256(spec), "scenario hash mismatch")
        for name in ("components", "pins"):
            expected = spec["execution"][name]
            _require(all(value is not None for value in expected.values()), f"unfrozen {name}")
            _require(evidence[name] == expected, f"{name} mismatch")
        source_files = evidence["source_files"]
        _require(isinstance(source_files, dict) and
                 set(spec["execution"]["required_source_files"]) <= set(source_files), "source audit files missing")
        for name, digest in source_files.items():
            _verified_file({"path": name, "sha256": digest}, source_root, checked)
        binding = {name: evidence[name] for name in
                   ("source_sha", "scenario_sha256", "components", "pins", "source_files")}

        def review(ref: Any, scope: str) -> dict[str, Any]:
            path = _verified_file(ref, artifact_root, checked)
            value = json.loads(path.read_text(encoding="utf-8"))
            _require(isinstance(value, dict), "review must be a JSON object")
            _require(value.get("binding") == binding, "review execution binding mismatch")
            _require(value.get("scope") == scope and value.get("verdict") == "pass", f"{scope} review not passed")
            _require(value.get("evidence_kind") == "independent_review", "fixture review is not admissible")
            return value

        audit = review(evidence["boundary_audit"], "offline")
        _require(audit.get("independent_reviewer") == "A2", "independent A2 source audit required")
        cases = audit.get("cases", {})
        _require(isinstance(cases, dict) and all(cases.get(case) == "pass" for case in AUDIT_CASES),
                 "boundary audit incomplete or failed")
        _require(isinstance(audit.get("artifacts"), list) and audit["artifacts"], "audit raw evidence absent")
        for ref in audit["artifacts"]:
            _verified_file(ref, artifact_root, checked)
        replays = evidence["physical_replays"]
        _require(isinstance(replays, list) and len(replays) == 2, "solo and joint physical replays required")
        kinds = []
        for ref in replays:
            replay = review(ref, "physical")
            kinds.append(replay.get("kind"))
            _require(replay.get("model_calls") == 0 and type(replay.get("model_calls")) is int,
                     "physical replay must use zero model calls")
            _require(replay.get("weld_enabled") is False and replay.get("replay_goal_complete") is True
                     and type(replay.get("mission_complete")) is bool,
                     "physical replay needs target success, preserved raw mission verdict and weld OFF")
            target = {"solo": ["box"], "joint": ["beam"]}.get(replay.get("kind"))
            _require(replay.get("target_object_ids") == target, "replay goal must match solo/joint target")
            _require(isinstance(replay.get("artifacts"), list) and replay["artifacts"], "replay raw artifacts absent")
            for ref in replay["artifacts"]:
                _verified_file(ref, artifact_root, checked)
        _require(sorted(kinds) == ["joint", "solo"], "physical replay kinds mismatch")
        witnesses = evidence["visual_witnesses"]
        _require(isinstance(witnesses, list) and witnesses, "visual witness absent")
        seen = set()
        for witness in witnesses:
            _exact(witness, {"event_id", "robot_id", "observation_id", "visibility",
                             "observed_at_s", "frame", "review"}, "visual witness")
            _require(witness["event_id"] == spec["event"]["event_id"], "witness event mismatch")
            _require(witness["robot_id"] in ROBOTS and _text(witness["observation_id"]), "invalid witness actor")
            _require(witness["visibility"] == spec["event"]["visibility"], "witness visibility mismatch")
            _require(_time(witness["observed_at_s"]), "invalid witness time")
            key = (witness["robot_id"], witness["observation_id"])
            _require(key not in seen, "duplicate visual witness")
            seen.add(key)
            frame = _verified_file(witness["frame"], artifact_root, checked).read_bytes()
            _require(frame.startswith(b"\xff\xd8") and frame.endswith(b"\xff\xd9"), "witness must preserve raw JPEG")
            record = review(witness["review"], "visual")
            expected = {k: v for k, v in witness.items() if k != "review"}
            _require(record.get("witness") == expected and record.get("criterion_visible") is True,
                     "visual reviewer did not verify this exact witness")
            _require(record.get("independent_reviewer") == "A2", "independent image inspection required")
            _require(record.get("private_information_claim") is False,
                     "no private-information claim established by this gate")
        counts = Counter(w["robot_id"] for w in witnesses)
        _require(max(counts.values(), default=0) >= spec["event"]["minimum_frames"],
                 "before/after witnesses must belong to the same actor")
    except (ScenarioError, OSError, ValueError, TypeError, KeyError) as exc:
        blockers.append(str(exc))
    return {"schema_version": "rgb-scenario-readiness-result.v1",
            "scenario_id": spec.get("scenario_id"), "ready": not blockers,
            "blockers": blockers, "checked_artifacts": checked,
            "scope": "live-admission evidence integrity; not a research outcome"}


def validate_episode(spec: Mapping[str, Any], events: Sequence[Mapping[str, Any]],
                     links: Mapping[str, Any], *, condition: str,
                     artifact_root: Path | None = None,
                     require_wire_images: bool = False) -> dict[str, Any]:
    """Verify reviewer-labelled event→RGB→decision→report/revision→issued command.

    References are original runtime event IDs, not prose similarities. Physical
    event/reached/success are separate evaluator annotations, never evidence of
    agent recognition. Empty recognition is valid and stays in the denominator.
    """
    validate_scenario(spec)
    _require(condition in CONDITIONS, "unknown condition")
    _exact(links, {"schema_version", "scenario_id", "condition", "physical_event",
                   "recognitions"}, "episode links")
    _require(links["schema_version"] == LINK_SCHEMA and links["scenario_id"] == spec["scenario_id"]
             and links["condition"] == condition, "episode binding mismatch")
    physical = links["physical_event"]
    _exact(physical, {"event_id", "reached", "recovered", "evaluator_artifact_sha256"}, "physical event")
    _require(physical["event_id"] == spec["event"]["event_id"], "physical event mismatch")
    _require(type(physical["reached"]) is bool and type(physical["recovered"]) is bool,
             "physical event verdicts must be boolean")
    _require(not physical["recovered"] or physical["reached"], "recovery cannot precede event")
    _require(_hex(physical["evaluator_artifact_sha256"]), "separate evaluator artifact required")
    index = {}
    for position, row in enumerate(events):
        _require(isinstance(row, Mapping) and _text(row.get("event_id")), "event ID required")
        _require(row["event_id"] not in index, "duplicate event ID")
        _require(row.get("condition") == condition, "mixed condition trace")
        index[row["event_id"]] = (position, row)
    if condition == "none":
        _require(not any(e.get("event_type") in {"message_sent", "message_received"} for e in events),
                 "none condition contains delivered communication")

    def event(event_id: str, kind: str, robot: str | None = None) -> tuple[int, Mapping[str, Any]]:
        _require(event_id in index, f"missing referenced event: {event_id}")
        pos, row = index[event_id]
        _require(row.get("event_type") == kind, f"wrong event type: {event_id}")
        if robot is not None:
            _require(row.get("robot_id") == robot, f"cross-actor evidence: {event_id}")
        return pos, row

    def decision(event_id: str, robot: str) -> tuple[int, Mapping[str, Any], int, Mapping[str, Any]]:
        pos, row = event(event_id, "planner_responded", robot)
        request_id = row["related_ids"]["request_id"]
        requests = [(p, e) for p, e in index.values() if e.get("event_type") == "planner_requested"
                    and e.get("robot_id") == robot and e.get("related_ids", {}).get("request_id") == request_id]
        _require(len(requests) == 1 and requests[0][0] < pos, "decision lacks unique prior request")
        _require(_text(row["related_ids"].get("decision_id")), "decision ID missing")
        _require(not any(e.get("event_type") == "planner_response_rejected" and
                         e.get("related_ids", {}).get("request_id") == request_id
                         and e.get("robot_id") == robot for e in events), "rejected reply is not an accepted decision")
        return pos, row, *requests[0]

    def transmitted_images(decision_row: Mapping[str, Any], request_row: Mapping[str, Any]) -> set[str]:
        _require(artifact_root is not None, "wire inspection needs artifact_root")
        artifact = decision_row["payload"].get("artifacts", {}).get(".request.json")
        _require(isinstance(artifact, dict), "actual provider request archive missing")
        # C archives absolute paths; accept only descendants of the supplied
        # retrieved root, then apply the same strict hash/path verification.
        path = Path(artifact.get("path", ""))
        if path.is_absolute():
            _require(path.resolve().is_relative_to(artifact_root.resolve()), "wire artifact escaped root")
            path = path.resolve().relative_to(artifact_root.resolve())
        ref = {"path": str(path), "sha256": artifact.get("sha256")}
        body = json.loads(_verified_file(ref, artifact_root, []).read_text())
        images = set()
        request_matches = []

        def walk(value):
            if isinstance(value, dict):
                # Gemini proxy accepts OpenAI image_url parts, converted by the
                # existing project serializer. Hash strings elsewhere are not images.
                if value.get("type") == "image_url":
                    image_url = value.get("image_url", {})
                    url = image_url.get("url") if isinstance(image_url, dict) else image_url
                    if isinstance(url, str) and url.startswith("data:image/jpeg;base64,"):
                        images.add(hashlib.sha256(base64.b64decode(url.split(",", 1)[1], validate=True)).hexdigest())
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)
            elif isinstance(value, str):
                try:
                    decoded = json.loads(value)
                except ValueError:
                    return
                if isinstance(decoded, dict) and decoded.get("request_id") == request_row["related_ids"]["request_id"]:
                    request_matches.append(decoded.get("robot_id") == request_row["robot_id"])
        walk(body)
        _require(request_matches and all(request_matches), "wire request/actor identity mismatch")
        return images

    _require(isinstance(links["recognitions"], list), "recognitions must be a list")
    chains = []
    seen_recognitions = set()
    for link in links["recognitions"]:
        _exact(link, {"robot_id", "observation_event_ids", "decision_event_id", "report_event_id",
                      "revision_event_id", "cancel_event_id", "prior_command_event_id",
                      "command_event_id"}, "recognition chain")
        robot = link["robot_id"]
        _require(robot in ROBOTS, "unknown recognizing actor")
        _require(link["decision_event_id"] not in seen_recognitions, "duplicate recognition decision")
        seen_recognitions.add(link["decision_event_id"])
        dpos, dec, reqpos, req = decision(link["decision_event_id"], robot)
        wire_hashes = transmitted_images(dec, req) if require_wire_images else None
        obs_ids = link["observation_event_ids"]
        _require(isinstance(obs_ids, list) and len(set(obs_ids)) == len(obs_ids)
                 and len(obs_ids) >= spec["event"]["minimum_frames"], "insufficient distinct RGB observations")
        request_observations = [req["payload"].get("observation", {})]
        request_observations += req["payload"].get("memory", {}).get("own_observations", [])
        seen_observations = {o.get("observation_id"): o for o in request_observations}
        obs_times = []
        for obs_id in obs_ids:
            opos, obs = event(obs_id, "observation_captured", robot)
            _require(opos < reqpos, "recognition uses future observation")
            key = obs["related_ids"].get("observation_id")
            _require(key in seen_observations, "RGB evidence was not in the actual model request")
            channel = "top_rgb" if spec["event"]["visibility"] == "public_top" else "own_rgb"
            image = obs["payload"].get("images", {}).get(channel, {})
            actual_image = seen_observations[key].get("images", {}).get(channel, {})
            _require(_hex(image.get("sha256")) and actual_image.get("sha256") == image["sha256"],
                     "model RGB evidence hash mismatch")
            if wire_hashes is not None:
                _require(image["sha256"] in wire_hashes,
                         "historical RGB hash/ref was not an image in the actual provider request")
            _require(_time(obs["payload"].get("observed_at_s")), "observation time required")
            obs_times.append(obs["payload"]["observed_at_s"])
        if spec["family"] == "recovery":
            _require(len(set(obs_times)) >= 2, "stall requires temporally distinct observations")
            _require(req["payload"].get("observation", {}).get("own_issued_commands"),
                     "stall recognition lacks own issued command evidence")
        reported = link["report_event_id"] is not None
        if reported:
            mpos, msg = event(link["report_event_id"], "message_sent", robot)
            _require(condition != "none" and mpos > dpos and
                     msg["related_ids"].get("decision_id") == dec["related_ids"]["decision_id"],
                     "report does not belong to recognizing decision")
        revised = link["revision_event_id"] is not None
        current_dec = dec
        current_pos = dpos
        if revised:
            rpos, rev, revreqpos, revreq = decision(link["revision_event_id"], robot)
            _require(rpos > dpos and rev["related_ids"]["decision_id"] != dec["related_ids"]["decision_id"],
                     "revision must be a later distinct decision")
            _require(rev["payload"].get("action") != dec["payload"].get("action"), "revision has no action change")
            current_dec, current_pos = rev, rpos
            # Only explicitly received, still-valid peer claims count as message exposure.
            for claim in revreq["payload"].get("memory", {}).get("received_messages", []):
                mid = claim.get("message_id")
                received = [(p, e) for p, e in index.values() if e.get("event_type") == "message_received"
                            and e.get("robot_id") == robot and e.get("related_ids", {}).get("message_id") == mid]
                _require(len(received) == 1 and received[0][0] < revreqpos, "revision contains undelivered peer claim")
                sent = [e for e in events if e.get("event_type") == "message_sent" and
                        e.get("related_ids", {}).get("message_id") == mid]
                _require(len(sent) == 1 and robot in sent[0]["payload"].get("recipients", []),
                         "revision uses a message addressed to another robot")
                now = revreq["payload"]["observation"]["observed_at_s"]
                _require(_time(claim.get("expires_at_s")) and now < claim["expires_at_s"], "revision uses expired peer claim")
        cancelled = link["cancel_event_id"] is not None
        if cancelled:
            cpos, cancel = event(link["cancel_event_id"], "action_submitted", robot)
            _require(dpos < cpos <= current_pos and cancel["payload"].get("kind") in {"interrupt", "cancel_pending"},
                     "cancellation not between recognition and revised decision")
        changed_command = False
        if link["command_event_id"] is not None:
            cpos, command = event(link["command_event_id"], "command_issued", robot)
            _require(cpos > current_pos and command["related_ids"].get("decision_id") ==
                     current_dec["related_ids"]["decision_id"], "issued command lacks decision provenance")
            _require(_text(command["payload"].get("command_id")), "actual issued command ID missing")
            if link["prior_command_event_id"] is not None:
                ppos, prior = event(link["prior_command_event_id"], "command_issued", robot)
                _require(ppos < dpos, "prior command must precede recognition")
                changed_command = prior["payload"].get("action") != command["payload"].get("action")
        elif link["prior_command_event_id"] is not None:
            raise ScenarioError("prior command without subsequent issued command")
        chains.append({"robot_id": robot, "decision_id": dec["related_ids"]["decision_id"],
                       "reported": reported, "silent": not reported, "revised": revised,
                       "cancelled": cancelled, "issued_command_changed": changed_command})
    return {"scenario_id": spec["scenario_id"], "condition": condition,
            "physical_event_reached": physical["reached"], "physical_recovered": physical["recovered"],
            "recognized_n": len(chains), "chains": chains,
            "complete_recovery_chain_n": sum(c["revised"] and c["issued_command_changed"] for c in chains),
            "model_image_exposure_verified": require_wire_images and bool(chains),
            "causal_effect_established": False,
            "meaning": "ID/hash linkage of reviewer-labelled recognition; not proof of semantic correctness or causality"}


def paired_schedule(specs: Sequence[Mapping[str, Any]], *, seeds: Sequence[int], repeats: int,
                    order_seed: int, ablation: str = "primary", pilot: bool = True) -> list[dict[str, Any]]:
    """Matched triplets; ablations get separate blocks, never replace a condition."""
    for spec in specs:
        validate_scenario(spec)
    _require(len({s["scenario_id"] for s in specs}) == len(specs) and bool(specs), "duplicate/empty scenarios")
    _require(seeds and len(set(seeds)) == len(seeds) and all(type(s) is int for s in seeds), "unique integer seeds required")
    _require(type(repeats) is int and repeats > 0 and type(order_seed) is int, "invalid repeat/randomization seed")
    _require(ablation in {"primary", "message_delay", "message_limit", "memory_limit", "scheduler"}, "unknown ablation")
    if pilot:
        _require(len(specs) == 2 and all(s["pilot_candidate"] for s in specs) and
                 len(seeds) == 1 and repeats == 1 and ablation == "primary", "pilot is exactly 2 x 3 trials")
    rows = [{"scenario_id": s["scenario_id"], "scenario_sha256": canonical_sha256(s),
             "seed": seed, "repeat": repeat, "condition": condition, "ablation": ablation,
             "paired_block": f"{s['scenario_id']}:s{seed}:r{repeat}:{ablation}"}
            for s in specs for seed in seeds for repeat in range(repeats) for condition in CONDITIONS]
    random.Random(order_seed).shuffle(rows)
    return [dict(row, order=i) for i, row in enumerate(rows, 1)]


def summarize_episodes(schedule: Sequence[Mapping[str, Any]], rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Keep unrun/fail/unknown and event-not-reached trials in planned denominators."""
    def key(row):
        return row["paired_block"], row["condition"]
    planned = {key(row): row for row in schedule}
    _require(len(planned) == len(schedule) and bool(planned), "duplicate/empty schedule")
    observed = {}
    for row in rows:
        _require(key(row) in planned and key(row) not in observed, "unexpected/duplicate run")
        _require(row.get("outcome") in {"success", "failure", "timeout", "api_error", "aborted", "unrun"}, "invalid outcome")
        for name in ("mission_complete", "event_reached", "recovered"):
            _require(row.get(name) is None or type(row[name]) is bool, f"invalid {name}")
        _require(not row.get("recovered") or row.get("event_reached") is True, "recovery without reach")
        _require(isinstance(row.get("costs"), dict), "cost measurements required")
        _require(all(v is None or _time(v) for v in row["costs"].values()), "invalid cost measurement")
        observed[key(row)] = row
    summary = {}
    for condition in CONDITIONS:
        keys = [k for k in planned if k[1] == condition]
        values = [observed.get(k, {"outcome": "unrun", "costs": {}}) for k in keys]
        costs = {name for row in values for name in row["costs"]}
        summary[condition] = {
            "planned_n": len(keys), "outcomes": dict(Counter(v["outcome"] for v in values)),
            "mission_complete_n": sum(v.get("mission_complete") is True for v in values),
            "event_reached_n": sum(v.get("event_reached") is True for v in values),
            "event_unmeasured_n": sum(v.get("event_reached") is None for v in values),
            "recovered_n": sum(v.get("recovered") is True for v in values),
            "costs_all_outcomes": {name: {
                "measured_n": sum(v["costs"].get(name) is not None for v in values),
                "total": sum(v["costs"].get(name) or 0 for v in values)
                if any(v["costs"].get(name) is not None for v in values) else None,
            } for name in sorted(costs)},
        }
    return {"planned_n": len(schedule), "by_condition": summary,
            "causal_effect_established": False, "raw_rows": list(rows)}


def audit_exposure(cases: Sequence[Mapping[str, Any]], provenance: Mapping[str, Any],
                   scenario_assignments: Sequence[Mapping[str, Any]], *,
                   allow_legacy_dispatch_open: bool = False,
                   allow_unknown_diagnostic_origins: bool = False) -> dict[str, Any]:
    """E0: reuse the suite split validator, then audit all transitive exposure.

    Authored geometry QA alone is not policy exposure. Diagnostics, training,
    examples and tuning on a held-out map demote that map to regression. Unknown
    foundation pretraining is disclosed, never certified uncontaminated.
    """
    from sim.act_map_suite import layout_digest, validate_splits
    from sim.research_dispatch_arena import authored_map, digest

    blockers: list[str] = []
    effective: dict[str, str] = {}
    unknown: list[str] = []
    lineage: dict[str, set[str]] = {}
    try:
        validate_splits(cases)
        by_map = {case["id"]: case for case in cases}
        if allow_legacy_dispatch_open:
            by_map["dispatch_open"] = {"id": "dispatch_open", "split": "regression",
                                       "topology_id": "legacy-dispatch-room", "map": authored_map("open")}
        effective = {mid: case["split"] for mid, case in by_map.items()}
        _exact(provenance, {"schema_version", "records", "exposures", "freeze_order", "final_test_ids"}, "provenance")
        _require(provenance["schema_version"] == "rgb-map-exposure.v1", "exposure schema mismatch")
        _require(type(provenance["freeze_order"]) is int and provenance["freeze_order"] >= 0, "freeze order required")
        records = provenance["records"]
        _require(isinstance(records, list) and records, "nonempty provenance records required")
        by_id = {}
        for record in records:
            _exact(record, {"id", "kind", "parents", "map_refs", "declared_splits", "origin"}, "provenance record")
            _require(_text(record["id"]) and record["id"] not in by_id, "duplicate/empty lineage id")
            _require(record["kind"] in {"episode", "frame", "augmentation", "dataset", "checkpoint",
                                         "prompt", "foundation_model", "scenario"}, "unknown provenance kind")
            _require(record["origin"] in {"known", "unknown"}, "origin knowledge status required")
            _require(isinstance(record["parents"], list) and all(_text(p) for p in record["parents"])
                     and len(set(record["parents"])) == len(record["parents"]), "invalid lineage parents")
            _require(isinstance(record["map_refs"], list), "map_refs required")
            _require(isinstance(record["declared_splits"], list), "declared_splits required")
            by_id[record["id"]] = record
        visiting = set()

        def ancestors(rid: str) -> set[str]:
            _require(rid in by_id and rid not in visiting, "missing parent or cyclic provenance")
            if rid in lineage:
                return lineage[rid]
            visiting.add(rid)
            record = by_id[rid]
            maps = set()
            for ref in record["map_refs"]:
                _exact(ref, {"map_id", "map_sha256", "layout_sha256"}, "parent map reference")
                mid = ref["map_id"]
                _require(mid in by_map, "unknown parent map")
                _require(ref["map_sha256"] == digest(by_map[mid]["map"])
                         and ref["layout_sha256"] == layout_digest(by_map[mid]["map"]), "parent map hash mismatch")
                maps.add(mid)
            for parent in record["parents"]:
                maps.update(ancestors(parent))
            _require(sorted(record["declared_splits"]) == sorted({by_map[mid]["split"] for mid in maps}),
                     "child must inherit every parent map split")
            diagnostic_unknown = allow_unknown_diagnostic_origins and record["origin"] == "unknown"
            _require(maps or record["kind"] == "foundation_model" or record["parents"] or diagnostic_unknown,
                     "artifact lacks provenance; unknown local origin requires diagnostic-only admission")
            if record["origin"] == "unknown":
                unknown.append(rid)
            visiting.remove(rid)
            lineage[rid] = maps
            return maps

        for rid in by_id:
            ancestors(rid)
        _require(isinstance(provenance["exposures"], list), "exposure history required")
        uses = {"geometry_qa", "training", "prompt_tuning", "checkpoint_selection", "demo",
                "policy_diagnosis", "final_evaluation"}
        test_seen_orders = []
        for exposure in provenance["exposures"]:
            _exact(exposure, {"record_id", "use", "order", "artifact_sha256"}, "exposure")
            _require(exposure["record_id"] in by_id and exposure["use"] in uses, "invalid exposure source/use")
            _require(type(exposure["order"]) is int and exposure["order"] >= 0
                     and _hex(exposure["artifact_sha256"]), "exposure order/artifact hash required")
            affected = lineage[exposure["record_id"]]
            heldout = {mid for mid in affected if by_map[mid]["split"] in {"test_a", "test_b"}}
            if exposure["use"] != "geometry_qa" and heldout:
                test_seen_orders.append((exposure["order"], heldout))
                if exposure["use"] != "final_evaluation" or exposure["order"] <= provenance["freeze_order"]:
                    for mid in heldout:
                        effective[mid] = "regression"
            if exposure["use"] in {"training", "prompt_tuning", "checkpoint_selection", "demo"}:
                # Looking at final outcomes then retuning anywhere invalidates that
                # previously inspected test as an untouched comparison for the revision.
                for order, mids in test_seen_orders:
                    if order < exposure["order"]:
                        for mid in mids:
                            effective[mid] = "regression"
        # History order must not change the result: repeat the temporal relation
        # over all tuning events, including those listed before test observations.
        tuning_orders = [e["order"] for e in provenance["exposures"] if e["use"] in
                         {"training", "prompt_tuning", "checkpoint_selection", "demo"}]
        for order, mids in test_seen_orders:
            if any(t > order for t in tuning_orders):
                for mid in mids:
                    effective[mid] = "regression"
        final = provenance["final_test_ids"]
        _require(isinstance(final, list) and len(final) == len(set(final)) and
                 all(mid in by_map and by_map[mid]["split"] in {"test_a", "test_b"} for mid in final),
                 "invalid final test selection")
        # Declared final test must be the whole frozen A/B suite, not success-filtered.
        expected = {mid for mid, case in by_map.items() if case["split"] in {"test_a", "test_b"}}
        _require(set(final) == expected, "final test silently excludes a held-out map")
        contaminated = sorted(mid for mid in final if effective[mid] == "regression")
        if contaminated:
            blockers.append("exposed test maps require a new holdout: " + ", ".join(contaminated))
        seen_assignments = set()
        for assignment in scenario_assignments:
            _exact(assignment, {"scenario_id", "parent_map_id", "map_sha256", "layout_sha256", "split"}, "scenario parent")
            mid = assignment["parent_map_id"]
            _require(_text(assignment["scenario_id"]) and mid in by_map, "scenario parent missing")
            pair = (assignment["scenario_id"], mid)
            _require(pair not in seen_assignments, "duplicate scenario parent assignment")
            seen_assignments.add(pair)
            _require(assignment["map_sha256"] == digest(by_map[mid]["map"])
                     and assignment["layout_sha256"] == layout_digest(by_map[mid]["map"]), "scenario map hash mismatch")
            _require(assignment["split"] == effective[mid], "scenario must inherit effective parent split")
    except (ValueError, TypeError, KeyError) as exc:
        blockers.append(str(exc))
    return {"schema_version": "rgb-map-exposure-result.v1", "valid": not blockers,
            "blockers": blockers, "effective_splits": effective,
            "map_topologies": {case["id"]: case["topology_id"] for case in cases},
            "legacy_development_registry": ["dispatch_open"] if allow_legacy_dispatch_open else [],
            "lineage_maps": {rid: sorted(maps) for rid, maps in lineage.items()},
            "unknown_origins": sorted(set(unknown)),
            "diagnostic_only": allow_unknown_diagnostic_origins,
            "heldout_claim_ready": not blockers and not allow_unknown_diagnostic_origins,
            "uncontaminated_pretraining_claim": False,
            "provenance_sha256": canonical_sha256(provenance),
            "scope": "declared lineage/exposure integrity, not policy performance or exhaustive pretraining knowledge"}


def validate_training_comparison(plan: Mapping[str, Any], exposure_report: Mapping[str, Any]) -> None:
    """T1/T2 change distribution only; T3 is an explicitly separate size study."""
    _exact(plan, {"schema_version", "arms", "fixed_test_ids", "training_seeds", "selection_rule_sha256"}, "training design")
    _require(plan["schema_version"] == "rgb-training-comparison.v1", "training design schema mismatch")
    _require(exposure_report.get("valid") is True, "invalid split/exposure audit")
    _require(exposure_report.get("heldout_claim_ready") is True, "diagnostic unknown provenance cannot approve training comparison")
    _require(isinstance(plan["arms"], list), "training arms required")
    arms = {arm["id"]: arm for arm in plan["arms"]}
    _require(len(arms) == len(plan["arms"]) and set(arms) in ({"T1", "T2"}, {"T1", "T2", "T3"}), "T1/T2 and optional T3 required")
    required = {"id", "map_ids", "episodes", "transitions", "frames", "updates", "batch_size",
                "architecture_sha256", "input_sha256", "initial_checkpoint_sha256", "device", "precision"}
    for arm in arms.values():
        _exact(arm, required, "training arm")
        _require(isinstance(arm["map_ids"], list) and arm["map_ids"] and len(set(arm["map_ids"])) == len(arm["map_ids"]), "training map IDs required")
        _require(all(exposure_report["effective_splits"].get(mid) == "train" for mid in arm["map_ids"]), "non-training map in training arm")
        _require(all(type(arm[k]) is int and arm[k] > 0 for k in
                     ("episodes", "transitions", "frames", "updates", "batch_size")), "positive data/update counts required")
        _require(all(_hex(arm[k]) for k in ("architecture_sha256", "input_sha256", "initial_checkpoint_sha256")), "training inputs must be pinned")
        _require(_text(arm["device"]) and _text(arm["precision"]), "device/precision required")
    same = required - {"id", "map_ids"}
    _require(all(arms["T1"][k] == arms["T2"][k] for k in same), "T1/T2 data amount and training budget/config must match")
    _require(set(arms["T1"]["map_ids"]) < set(arms["T2"]["map_ids"]), "T2 must expand map distribution, not repeat seeds")
    if "T3" in arms:
        _require(set(arms["T3"]["map_ids"]) == set(arms["T2"]["map_ids"]), "T3 keeps the diverse distribution")
        _require(all(arms["T3"][k] == arms["T2"][k] for k in
                     ("architecture_sha256", "input_sha256", "initial_checkpoint_sha256", "device", "precision", "batch_size")), "T3 changes amount/budget only")
        _require(all(arms["T3"][k] > arms["T2"][k] for k in ("episodes", "transitions", "frames")), "T3 must disclose larger data amount")
    heldout = {mid for mid, split in exposure_report["effective_splits"].items() if split in {"test_a", "test_b"}}
    _require(isinstance(plan["fixed_test_ids"], list) and len(plan["fixed_test_ids"]) == len(heldout)
             and set(plan["fixed_test_ids"]) == heldout, "same complete frozen test set required")
    seeds = plan["training_seeds"]
    _require(isinstance(seeds, list) and len(seeds) >= 2 and len(set(seeds)) == len(seeds)
             and all(type(s) is int and s >= 0 for s in seeds), "repeated independent training seeds required")
    _require(_hex(plan["selection_rule_sha256"]), "development-only selection rule must be frozen")


def scene_support_matrix(*, seed: int = 11) -> dict[str, Any]:
    """Resolve native scene *definitions* without XML generation or physics.

    This is an evaluator-side inventory, not an actor input or execution permit.
    Geometry groups deliberately exclude goal/reset repetitions, but include
    terrain and unexpected setup obstacles (unlike B's older map_group digest).
    No catalogue entry is promoted to a transport or safe-stop success.
    """
    from sim.session_scenes import Scene, catalog
    from sim.session_config import validate_config
    from sim.act_map_suite import load_suite
    from sim.research_dispatch_arena import digest
    from harness.rgb_skill_execution import BACKEND_ID, SKILLS

    _require(type(seed) is int and 0 <= seed <= 1_000_000, "invalid matrix reset seed")
    root = Path(__file__).resolve().parents[1]
    # Hash the complete finite definition dependency set, including assets that
    # Scene.sources does not enumerate (e.g. generated corner source maps).
    paths = {p for folder in ("harness", "sim", "scripts", "maps", "calibration") for p in (root/folder).rglob("*")
             if p.is_file() and p.suffix in {".py", ".json", ".xml", ".png", ".stl", ".obj"}}
    sources = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in sorted(paths)}
    _, cases = load_suite()
    case_by_id = {c["id"]: c for c in cases}
    rows = []
    for entry in catalog():
        config = validate_config({"version": 1, "scene": {"layout": entry["id"], "seed": seed}})
        resolved = Scene(config["scene"], root)
        setup = resolved.config["setup_only"] if resolved.config else None
        physical_map = resolved.config["static_map"] if resolved.config else resolved.map
        parent_map = resolved.map if resolved.map is not None else physical_map
        case = case_by_id.get(entry["id"].removeprefix("act/")) if resolved.family == "act" else None
        if resolved.family == "multi_object":
            from sim.act_map_suite import generate_case
            layout = json.loads((root/"maps/act_generalization/multi_object_layout_v1.json").read_text())
            case = generate_case(layout["source_case"])
            parent_map = case["map"]
        camera = physical_map["top_camera"] if physical_map else None
        goals = ({"parent_goal": parent_map.get("goal"),
                  **{key: physical_map[key] for key in ("frame", "docks", "zones", "destinations") if key in physical_map}}
                 if physical_map else None)
        geometry = ({key: physical_map.get(key, []) for key in
                     ("bounds_m", "obstacles", "terrain")} if physical_map else None)
        if geometry is not None:
            geometry["unexpected_obstacles"] = (setup or {}).get("unexpected_obstacles", [])
        supported = entry["id"] == "dispatch/open"
        row = {
            "selection": entry["id"], "family": entry["family"], "catalog_scope": entry["scope"],
            "parent_case_id": case["id"] if case else None,
            "declared_split": case["split"] if case else "unassigned_or_exposed_legacy",
            "map_id": parent_map.get("map_id") if parent_map else None,
            "parent_map_sha256": digest(parent_map) if parent_map else None,
            "physical_map_sha256": digest(physical_map) if physical_map else None,
            "physical_geometry_sha256": digest(geometry) if geometry else None,
            "definition_sha256": digest({"selection": entry["id"], "scene": config["scene"],
                                         "resolved_config": resolved.config, "parent_map": resolved.map}),
            "reset_definition_sha256": digest(setup) if setup else None,
            "top_camera_sha256": digest(camera) if camera else None,
            "own_camera_source_sha256": canonical_sha256({p: sources[p] for p in (
                "sim/masterpi_camera_profile.py", "sim/masterpi_scene_v2.xml",
                "sim/masterpi_production_v2.py", "sim/multi_masterpi_production.py",
                "sim/masterpi_dynamics_v2.py", "sim/masterpi_geometry.py", "harness/real_geometry.py")}),
            "native_rgb_size": [config["camera"]["width"], config["camera"]["height"]],
            "backend_rgb_size": [960, 720] if supported else None,
            "goal_definition_sha256": digest(goals) if goals else None,
            "definition_sources_sha256": canonical_sha256(sources),
            "backend_id": BACKEND_ID,
            "transport_support": "experimental_fixed_dispatch_routes" if supported else "unsupported",
            "supported_skills": sorted(SKILLS) if supported else [],
            "physical_safe_stop_support": "unverified",
            "evaluator": "scripts.run_dispatch_e2e.Referee" if supported else None,
            "evaluator_source_sha256": sources["scripts/run_dispatch_e2e.py"] if supported else None,
            "runtime_scene_xml_sha256": None, "runtime_camera_invariants": None,
            "physical_success": False,
            "reason": ("matching map/reset definitions only; native and backend recordings/resolution differ"
                       if supported else "native scene availability does not connect RGB transport or evaluator"),
        }
        rows.append(row)
    groups: dict[str, list[str]] = {}
    for row in rows:
        if row["physical_geometry_sha256"]:
            groups.setdefault(row["physical_geometry_sha256"], []).append(row["selection"])
    return {"schema_version": "rgb-scene-support-matrix.v1", "seed": seed,
            "entry_count": len(rows), "family_counts": dict(Counter(r["family"] for r in rows)),
            "rows": rows, "definition_source_files": sources,
            "repeated_physical_geometry_groups": [g for g in groups.values() if len(g) > 1],
            "catalog_entries_are_independent_maps": False,
            "physical_controls_verified": False, "execution_admission": False,
            "scope": "read-only definitions; no reset, render, model or physical execution"}


def assess_scene_backend_selection(selection: str, descriptor: Mapping[str, Any], *,
                                   purpose: str = "connection_diagnostic") -> dict[str, Any]:
    """A3 exact selection handshake. Never alias an unsupported map to open.

    B's descriptor is independently recomputed by D against immutable assets;
    this complementary gate binds its map/reset/camera/skills to native metadata.
    Native contact/geometry/robot overrides are not inputs to this contract.
    """
    from sim.research_dispatch_arena import episode, digest
    from harness.rgb_skill_execution import BACKEND_ID, SKILLS
    blockers = []
    binding = {}
    try:
        _require(purpose == "connection_diagnostic", "physical development/control admission is not implemented")
        _require(selection == "dispatch/open", "unsupported native transport selection; no map substitution")
        _require(isinstance(descriptor, Mapping), "backend descriptor must be an object")
        _require(descriptor.get("schema") == "ugrp.rgb_skill_backend_descriptor.v1"
                 and descriptor.get("backend_id") == BACKEND_ID, "backend identity mismatch")
        seed = descriptor.get("seed")
        _require(type(seed) is int and 0 <= seed <= 1_000_000, "invalid backend reset seed")
        expected = episode("open", seed)
        static = expected["static_map"]
        binding = {"map_id": static["map_id"], "map_sha256": digest(static),
                   "map_instance_sha256": digest(static),
                   "map_group_sha256": digest({k: static[k] for k in ("bounds_m", "obstacles", "top_camera")}),
                   "camera_sha256": digest(static["top_camera"]),
                   "reset_sha256": digest(expected["setup_only"]), "goal_frame": static["frame"],
                   "capabilities": SKILLS, "synthetic": False, "weld": False,
                   "camera_fov_changed": False, "clock_owner": "single_simulator", "clock_domain": "sim",
                   "skill_image_max_age_s": 1., "skill_worker_wall_limit_s": 2.}
        _require(all(descriptor.get(k) == v for k, v in binding.items()), "scene/backend definition or boundary mismatch")
        _require(all(descriptor.get(k) is False for k in ("synthetic", "weld", "camera_fov_changed")),
                 "backend boundary flags must be booleans")
        _require(all(_time(descriptor.get(k)) for k in ("skill_image_max_age_s", "skill_worker_wall_limit_s")),
                 "backend timing limits must be finite numbers, not booleans")
        _require(canonical_sha256(descriptor.get("capabilities")) == canonical_sha256(SKILLS),
                 "backend skill definition type/value mismatch")
        _require(isinstance(descriptor.get("map_to_scene"), Mapping), "map-to-scene binding must be an object")
        _require(descriptor["map_to_scene"].get("source_map_sha256") == digest(static)
                 and descriptor["map_to_scene"].get("builder") == "sim.research_dispatch_arena.build_scene_xml",
                 "map-to-scene definition mismatch")
    except (ValueError, TypeError, KeyError) as exc:
        blockers.append(str(exc))
    return {"schema_version": "rgb-scene-backend-selection.v1", "ready": not blockers,
            "purpose": purpose, "selection": selection, "blockers": blockers,
            "definition_binding_sha256": canonical_sha256(binding),
            "requires_runtime_evidence": ["scene.xml", "initial_invariants", "RGB dimensions and original capture times",
                                          "evaluator-only result", "post-child-exit artifact hashes"],
            "physical_controls_verified": False, "execution_admission": False}


ENVIRONMENT_CHECKS = (
    "map_scene_goal_correspondence", "initial_nonpenetration", "loaded_clearance",
    "fixed_camera_visibility", "reset_seed_consistency", "observation_timestamp_alignment",
    "easy_development_success", "impossible_safe_stop", "single_physics_clock", "weld_off",
)
ENVIRONMENT_PREFLIGHT_CHECKS = (
    "map_scene_goal_correspondence", "fixed_camera_configuration", "reset_seed_contract",
    "single_physics_clock", "weld_off",
)


def assess_environment_readiness(record: Mapping[str, Any], cases: Sequence[Mapping[str, Any]], *,
                                 artifact_root: Path, expected_source_sha: str,
                                 expected_backend_id: str, purpose: str = "live",
                                 allow_legacy_dispatch_open: bool = False) -> dict[str, Any]:
    """Verify E0 artifact bindings; geometry alone never proves policy transport."""
    from sim.act_map_suite import layout_digest, validate_splits
    from sim.research_dispatch_arena import authored_map, digest
    checked: list[str] = []
    blockers: list[str] = []
    try:
        validate_splits(cases)
        _require(purpose in {"live", "physical_replay"}, "invalid environment admission purpose")
        diagnostic = purpose == "physical_replay"
        _exact(record, {"schema_version", "source_sha", "backend_id", "scope", "maps", "checks"}, "environment review")
        _require(record["schema_version"] == "rgb-environment-readiness.v1", "environment schema mismatch")
        _require(_hex(expected_source_sha, 40) and record["source_sha"] == expected_source_sha,
                 "environment source mismatch")
        _require(_text(expected_backend_id) and record["backend_id"] == expected_backend_id, "environment backend mismatch")
        scope = "static_reset_preflight" if diagnostic else "development_physical_controls"
        _require(record["scope"] == scope, "environment scope does not match admission purpose")
        _require(isinstance(record["maps"], list) and record["maps"], "environment maps required")
        by_map = {case["id"]: case for case in cases}
        if allow_legacy_dispatch_open:
            _require(diagnostic, "legacy dispatch registry is physical-replay only, not live or heldout")
            # Explicit legacy source definition, never an alias for a suite map.
            # Its map/group digests are recomputed from the frozen source.
            by_map["dispatch_open"] = {"id": "dispatch_open", "split": "regression",
                                       "map": authored_map("open")}
        seen = set()
        for row in record["maps"]:
            _exact(row, {"map_id", "map_sha256", "layout_sha256", "scene", "camera", "reset", "evaluator_config", "capability"}, "environment map")
            mid = row["map_id"]
            _require(mid in by_map and mid not in seen, "unknown/duplicate environment map")
            seen.add(mid)
            allowed = {"train", "dev", "control", "regression"} if diagnostic else {"dev", "control"}
            _require(by_map[mid]["split"] in allowed, "readiness controls must not consume final test policies")
            _require(row["map_sha256"] == digest(by_map[mid]["map"])
                     and row["layout_sha256"] == layout_digest(by_map[mid]["map"]), "environment map identity mismatch")
            _require(row["capability"] in {"transport", "safe_stop"}, "unsupported map cannot silently become an open map")
            for key in ("scene", "camera", "reset", "evaluator_config"):
                _verified_file(row[key], artifact_root, checked)
        if not diagnostic:
            _require(any(by_map[mid]["split"] == "dev" for mid in seen)
                     and any(by_map[mid]["split"] == "control" for mid in seen), "easy and impossible controls required")
        required_checks = ENVIRONMENT_PREFLIGHT_CHECKS if diagnostic else ENVIRONMENT_CHECKS
        _exact(record["checks"], set(required_checks), "environment checks")
        for name, ref in record["checks"].items():
            path = _verified_file(ref, artifact_root, checked)
            check = json.loads(path.read_text())
            _require(check.get("check_id") == name and check.get("verdict") == "pass"
                     and check.get("source_sha") == expected_source_sha
                     and check.get("backend_id") == expected_backend_id
                     and check.get("maps_sha256") == canonical_sha256(record["maps"]), "environment check binding/verdict mismatch")
            kind = "static_contract_check" if diagnostic else "physical_diagnostic"
            _require(check.get("evidence_kind") == kind, "check evidence level does not match admission purpose")
            _require(isinstance(check.get("artifacts"), list) and check["artifacts"], "environment raw evidence missing")
            for ref in check["artifacts"]:
                _verified_file(ref, artifact_root, checked)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        blockers.append(str(exc))
    return {"schema_version": "rgb-environment-readiness-result.v1", "ready": not blockers,
            "purpose": purpose, "blockers": blockers, "checked_artifacts": checked,
            "legacy_development_registry": ["dispatch_open"] if allow_legacy_dispatch_open else [],
            "physical_controls_verified": purpose == "live" and not blockers,
            "heldout_policy_performance_verified": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--scenario-id")
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--artifact-root", type=Path, default=Path("."))
    parser.add_argument("--source-root", type=Path, default=Path("."))
    parser.add_argument("--expected-source-sha")
    args = parser.parse_args()
    specs = load_scenarios(args.catalog)
    if args.evidence:
        if not (args.scenario_id and args.expected_source_sha):
            parser.error("--scenario-id and --expected-source-sha are required with --evidence")
        selected = [s for s in specs if s["scenario_id"] == args.scenario_id]
        if not selected:
            parser.error("scenario-id is not in catalog")
        report = assess_readiness(selected[0], json.loads(args.evidence.read_text()),
                                  expected_source_sha=args.expected_source_sha,
                                  artifact_root=args.artifact_root, source_root=args.source_root)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ready"] else 2
    print(json.dumps({"schema_valid": True, "scenario_ids": [s["scenario_id"] for s in specs],
                      "live_readiness": "not_assessed"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
