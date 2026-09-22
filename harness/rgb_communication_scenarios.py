"""Executable, evaluator-side scenarios and evidence gates for the RGB study.

Nothing here sends scenario events, annotations, or evaluator state to actors.
Readiness is a file-backed audit gate, not a physical-success detector.  Passing
it cannot replace independently reviewing the referenced images and source.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
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
SOURCE_FILES = (
    "harness/rgb_execution_contract.py", "harness/rgb_execution_port.py",
    "harness/rgb_skill_execution.py", "harness/rgb_communication_runtime.py",
    "harness/rgb_communication_planner.py", "harness/rgb_communication_study.py",
    "scripts/run_rgb_communication_study.py",
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
            _require(replay.get("weld_enabled") is False and replay.get("mission_complete") is True,
                     "physical replay needs real success with weld OFF")
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
                     links: Mapping[str, Any], *, condition: str) -> dict[str, Any]:
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
        return pos, row, *requests[0]

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
            rpos, rev, _, revreq = decision(link["revision_event_id"], robot)
            _require(rpos > dpos and rev["related_ids"]["decision_id"] != dec["related_ids"]["decision_id"],
                     "revision must be a later distinct decision")
            _require(rev["payload"].get("action") != dec["payload"].get("action"), "revision has no action change")
            current_dec, current_pos = rev, rpos
            # Only explicitly received, still-valid peer claims count as message exposure.
            for claim in revreq["payload"].get("memory", {}).get("received_messages", []):
                mid = claim.get("message_id")
                received = [(p, e) for p, e in index.values() if e.get("event_type") == "message_received"
                            and e.get("robot_id") == robot and e.get("related_ids", {}).get("message_id") == mid]
                _require(len(received) == 1 and received[0][0] < rpos, "revision contains undelivered peer claim")
                now = revreq["payload"]["observation"]["observed_at_s"]
                _require(_time(claim.get("expires_at_s")) and now < claim["expires_at_s"], "revision uses expired peer claim")
        cancelled = link["cancel_event_id"] is not None
        if cancelled:
            cpos, cancel = event(link["cancel_event_id"], "action_submitted", robot)
            _require(dpos < cpos <= current_pos and cancel["payload"].get("kind") in {"interrupt", "cancel"},
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
        parser.error("--scenario-id and --expected-source-sha are required with --evidence") if not (
            args.scenario_id and args.expected_source_sha) else None
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
