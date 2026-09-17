"""Offline integrity audit for one or more saved pick-match episodes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from harness.gemini_proxy import _to_gemini_multi_image_messages
from harness.semantic_pick_policy import PickMatchPlanner


def _load(path: Path) -> Any:
    return json.loads(path.read_text())


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _issue(report: dict[str, Any], code: str, detail: str) -> None:
    report["issues"].append({"code": code, "detail": detail})


def _classification(config: dict[str, Any], result: dict[str, Any]) -> str:
    if config.get("capture_only") or result.get("reason") == "CAPTURE_ONLY":
        return "capture_only"
    declared = config.get("trial_kind") or config.get("execution_kind")
    if config.get("scripted_units") or result.get("scripted_steps") or declared in {"scripted", "scripted_smoke", "smoke"}:
        return "scripted_smoke"
    if int(result.get("wire_requests", 0) or 0) > 0:
        return "live_model_episode"
    return "non_live_no_model_requests"


def _check_constraints(value: Any, location: str, report: dict[str, Any]) -> int:
    checked = 0
    if isinstance(value, dict):
        for key, child in value.items():
            child_location = f"{location}.{key}"
            if key == "constraints_active":
                if not isinstance(child, dict):
                    _issue(report, "INVALID_CONSTRAINT_FLAGS", child_location)
                else:
                    checked += len(child)
                    for name, active in child.items():
                        if not isinstance(active, bool):
                            _issue(report, "INVALID_CONSTRAINT_FLAG", f"{child_location}.{name}")
                        elif active:
                            _issue(report, "CONSTRAINT_ACTIVE", f"{child_location}.{name}")
            else:
                checked += _check_constraints(child, child_location, report)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            checked += _check_constraints(child, f"{location}[{index}]", report)
    return checked


def audit_episode(path: Path) -> dict[str, Any]:
    root = path.resolve()
    report: dict[str, Any] = {"path": str(root), "issues": [], "calls_checked": 0,
                              "constraint_flags_checked": 0}
    required = ["config.json", "fixture-evaluation-only.json", "result.json"]
    missing = [name for name in required if not (root / name).is_file()]
    for name in missing:
        _issue(report, "MISSING_FILE", name)
    if missing:
        report.update({"classification": "incomplete", "ok": False})
        return report

    try:
        config, fixture, result = (_load(root / name) for name in required)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        _issue(report, "INVALID_JSON", str(exc))
        report.update({"classification": "incomplete", "ok": False})
        return report
    report["classification"] = _classification(config, result)
    calls_path = root / "calls.json"
    if calls_path.is_file():
        try:
            calls = _load(calls_path)
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            _issue(report, "INVALID_CALLS_JSON", str(exc))
            calls = []
    else:
        calls = []
        if report["classification"] not in {"capture_only", "scripted_smoke"}:
            _issue(report, "MISSING_FILE", "calls.json")
    condition = config.get("condition")
    if condition not in {"skill", "semantic"}:
        _issue(report, "INVALID_CONDITION", repr(condition))
        report["ok"] = False
        return report
    if result.get("condition") != condition or fixture.get("seed") != config.get("seed") or result.get("seed") != config.get("seed"):
        _issue(report, "EPISODE_IDENTITY_MISMATCH", "condition or seed differs across saved records")
    if result.get("git_sha") != config.get("git_sha"):
        _issue(report, "GIT_SHA_MISMATCH", "config and result git_sha differ")
    if result.get("calls") is not None and result.get("calls") != len(calls):
        _issue(report, "RESULT_CALL_COUNT_MISMATCH", f"result={result.get('calls')} calls.json={len(calls)}")
    if not isinstance(calls, list):
        _issue(report, "INVALID_CALLS", "calls.json must be a list")
        calls = []

    class NoCallCompleter:
        def complete(self, *_args: Any, **_kwargs: Any) -> str:
            raise AssertionError("offline audit must not call a model")

    planner = PickMatchPlanner(NoCallCompleter(), condition)
    wire_files = sorted((root / "wire").glob("*.json")) if (root / "wire").is_dir() else []
    if len(wire_files) != len(calls):
        _issue(report, "WIRE_CALL_COUNT_MISMATCH", f"wire={len(wire_files)} calls={len(calls)}")
    for position, call in enumerate(calls, 1):
        index = call.get("call") if isinstance(call, dict) else None
        if index != position:
            _issue(report, "CALL_INDEX_MISMATCH", f"position={position} call={index!r}")
        own_path = root / "inputs" / f"call-{position:03d}-own.jpg"
        top_path = root / "inputs" / f"call-{position:03d}-top.jpg"
        if not own_path.is_file() or not top_path.is_file():
            _issue(report, "MISSING_CALL_IMAGE", f"call {position}")
            continue
        own, top = own_path.read_bytes(), top_path.read_bytes()
        if call.get("own_sha256") != _sha(own) or call.get("top_sha256") != _sha(top):
            _issue(report, "CALL_IMAGE_HASH_MISMATCH", f"call {position}")
        try:
            request = planner.prepare_request(own, top)
            expected_messages = _to_gemini_multi_image_messages(request["messages"], request["images"])
        except Exception as exc:
            _issue(report, "REQUEST_RECONSTRUCTION_FAILED", f"call {position}: {exc}")
            continue
        if position <= len(wire_files):
            try:
                wire = _load(wire_files[position - 1])
            except Exception as exc:
                _issue(report, "INVALID_WIRE_JSON", f"call {position}: {exc}")
            else:
                expected_wire = {"model": config.get("model"), "messages": expected_messages,
                                 "temperature": 0.2, "max_tokens": 650,
                                 "reasoning_effort": "none"}
                if wire != expected_wire:
                    _issue(report, "WIRE_REQUEST_MISMATCH", f"call {position}")
                try:
                    content = wire["messages"][-1]["content"]
                    labels = [item.get("text") for item in content if item.get("type") == "text"][1:]
                    context = json.loads(content[0]["text"])
                    if labels != ["OWN_CAMERA", "SHARED_TOP_CAMERA"]:
                        _issue(report, "IMAGE_LABEL_MISMATCH", f"call {position}: {labels!r}")
                    if set(context) != {"task", "mode", "own_model_selected_actions"}:
                        _issue(report, "EXTRA_ACTOR_STATE_IN_WIRE", f"call {position}: {sorted(context)}")
                except Exception as exc:
                    _issue(report, "INVALID_WIRE_STRUCTURE", f"call {position}: {exc}")
        action = call.get("action")
        accepted = action is not None and not call.get("dispatch_rejected") and not call.get("error")
        if accepted:
            try:
                planner.record_issued(action)
            except (TypeError, ValueError) as exc:
                _issue(report, "INVALID_ACCEPTED_ACTION", f"call {position}: {exc}")
        report["calls_checked"] += 1

    report["constraint_flags_checked"] += _check_constraints(fixture, "fixture", report)
    referee = root / "evaluation-only.jsonl"
    referee_rows = 0
    if referee.is_file():
        for line_no, line in enumerate(referee.read_text().splitlines(), 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                _issue(report, "INVALID_REFEREE_JSON", f"line {line_no}: {exc}")
                continue
            referee_rows += 1
            flags = row.get("constraints_active") if isinstance(row, dict) else None
            if not isinstance(flags, dict) or not flags:
                _issue(report, "MISSING_REFEREE_CONSTRAINT_FLAGS", f"line {line_no}")
            report["constraint_flags_checked"] += _check_constraints(row, f"referee[{line_no}]", report)
    if report["classification"] in {"live_model_episode", "scripted_smoke"} and referee_rows == 0:
        _issue(report, "NO_CONSTRAINT_EVIDENCE", "executed episode has no referee samples")
    final_fixture_path = root / "final-fixture-evaluation-only.json"
    if final_fixture_path.is_file():
        try:
            final_fixture = _load(final_fixture_path)
            for field in ("camera_parameters_sha256", "geometry_sha256"):
                if final_fixture.get(field) != fixture.get(field):
                    _issue(report, "FIXTURE_MUTATED", field)
        except Exception as exc:
            _issue(report, "INVALID_FINAL_FIXTURE", str(exc))
    elif report["classification"] in {"live_model_episode", "scripted_smoke"}:
        _issue(report, "MISSING_FINAL_FIXTURE", "cannot verify unchanged camera and geometry hashes")
    report["ok"] = not report["issues"]
    return report


def audit_paths(paths: list[Path]) -> dict[str, Any]:
    episodes = [audit_episode(path) for path in paths]
    report: dict[str, Any] = {"schema": "ugrp.pick_match_audit.v1", "episodes": episodes,
                              "pair_checks": [], "issues": []}
    by_seed: dict[Any, list[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]]] = {}
    for episode, path in zip(episodes, paths):
        if episode.get("classification") == "incomplete":
            continue
        config = _load(path / "config.json")
        fixture = _load(path / "fixture-evaluation-only.json")
        by_seed.setdefault(config.get("seed"), []).append((episode, config, fixture))
    pair_fields = ("git_sha", "seed", "seconds", "max_calls", "max_input_tokens", "model",
                   "timeout", "physics_paused_during_inference", "physics", "seconds_is_sim_budget")
    fixture_fields = ("initial_qpos_sha256", "camera_parameters_sha256", "geometry_sha256")
    for seed, group in by_seed.items():
        if len(paths) == 1:
            break
        conditions = {config.get("condition") for _, config, _ in group}
        check = {"seed": seed, "conditions": sorted(str(v) for v in conditions), "issues": []}
        if conditions != {"skill", "semantic"} or len(group) != 2:
            check["issues"].append("expected exactly one skill and one semantic episode")
        else:
            left, right = group
            for field in pair_fields:
                if left[1].get(field) != right[1].get(field):
                    check["issues"].append(f"config mismatch: {field}")
            for field in fixture_fields:
                if left[2].get(field) != right[2].get(field):
                    check["issues"].append(f"fixture mismatch: {field}")
            classes = {left[0].get("classification"), right[0].get("classification")}
            if classes != {"live_model_episode"}:
                check["issues"].append(f"not a live cohort: {sorted(classes)}")
        check["ok"] = not check["issues"]
        report["pair_checks"].append(check)
    report["ok"] = all(item["ok"] for item in episodes) and all(item["ok"] for item in report["pair_checks"])
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit_paths(args.episodes)
    rendered = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
