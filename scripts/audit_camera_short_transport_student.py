#!/usr/bin/env python3
"""Audit saved RGB carry inputs/actions, then independently replay referee output."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path, PurePath
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.camera_approach_scene import ROBOTS
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_short_transport_student import choose_carry_actions

CALL_KEYS = {"index", "robot_id", "frame_id", "stationary", "images",
             "own_command_history", "decision", "action"}
IMAGE_KEYS = {"own", "top"}
RGB_KEYS = {"path", "sha256"}
ACTION_KEYS = {"kind", "forward", "turn", "duration_s"}


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _safe_file(root: Path, relative: Any) -> Path:
    if not isinstance(relative, str) or not relative or PurePath(relative).is_absolute():
        raise ValueError("artifact path must be a non-empty relative path")
    parts = PurePath(relative).parts
    if any(part in ("", ".", "..") for part in parts):
        raise ValueError("artifact path traversal refused")
    current = root
    for part in parts:
        current = current / part
        if current.is_symlink():
            raise ValueError("symlink artifact path refused")
    resolved = current.resolve()
    if not resolved.is_relative_to(root) or not resolved.is_file():
        raise ValueError("artifact path escapes root or is not a file")
    return resolved


def _rgb(run_dir: Path, record: Any) -> tuple[bytes, str]:
    if not isinstance(record, dict) or set(record) != RGB_KEYS:
        raise ValueError("RGB record schema mismatch")
    path = _safe_file(run_dir, record["path"])
    data = path.read_bytes()
    digest = _digest(data)
    if not isinstance(record["sha256"], str) or digest != record["sha256"]:
        raise ValueError(f"RGB hash mismatch: {record.get('path')}")
    return data, digest


def _close(expected: Any, saved: Any, *, tolerance: float = 1e-9) -> bool:
    if isinstance(expected, bool) or isinstance(saved, bool):
        return expected is saved
    if isinstance(expected, (int, float)) and isinstance(saved, (int, float)):
        return (math.isfinite(float(expected)) and math.isfinite(float(saved))
                and math.isclose(float(expected), float(saved), rel_tol=tolerance, abs_tol=tolerance))
    if isinstance(expected, dict) and isinstance(saved, dict):
        return set(expected) == set(saved) and all(_close(expected[k], saved[k], tolerance=tolerance) for k in expected)
    if isinstance(expected, list) and isinstance(saved, list):
        return len(expected) == len(saved) and all(_close(a, b, tolerance=tolerance) for a, b in zip(expected, saved))
    return expected == saved


def _action(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != ACTION_KEYS or value.get("kind") != "drive":
        raise ValueError(f"{label} action schema mismatch")
    forward, turn, duration = value["forward"], value["turn"], value["duration_s"]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v))
           for v in (forward, turn, duration)):
        raise ValueError(f"{label} action is non-finite")
    if not 0.0 <= float(forward) <= .10 or float(turn) != 0.0 or not 0.0 < float(duration) <= .25:
        raise ValueError(f"{label} action exceeds carry bounds")
    return value


def _models(root: Path, report: dict[str, Any]) -> dict[str, dict[str, Any]]:
    manifest_path = _safe_file(root, "short-transport-skill.json")
    if report.get("transport_skill_sha256") != _digest(manifest_path.read_bytes()):
        raise ValueError("transport manifest hash mismatch")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != "ugrp.camera_short_transport_skill.v1":
        raise ValueError("transport manifest schema mismatch")
    records, reported = manifest.get("models"), report.get("transport_model_sha256")
    if (not isinstance(records, dict) or set(records) != set(ROBOTS)
            or not isinstance(reported, dict) or set(reported) != set(ROBOTS)):
        raise ValueError("transport model coverage mismatch")
    result = {}
    for rid in ROBOTS:
        record = records[rid]
        if not isinstance(record, dict) or set(record) != {"path", "sha256"}:
            raise ValueError(f"transport model record schema mismatch: {rid}")
        path = _safe_file(root, record["path"])
        digest = _digest(path.read_bytes())
        if digest != record["sha256"] or digest != reported[rid]:
            raise ValueError(f"transport model hash mismatch: {rid}")
        model = json.loads(path.read_text())
        if model.get("schema") != "ugrp.camera_short_transport_model.v1" or model.get("robot_id") != rid:
            raise ValueError(f"transport model identity mismatch: {rid}")
        result[rid] = model
    return result


def _predict(model, own, top, initial_own, initial_top, history):
    from harness.camera_short_transport_student import predict_transport
    return predict_transport(model, own, top, initial_own, initial_top,
                             own_command_history=history)


def _playback(report: dict[str, Any]) -> list[dict[str, Any]]:
    saved, source = report.get("playback_actions"), report.get("playback_teacher_source")
    if not isinstance(saved, list) or not isinstance(source, dict) or set(source) != {"path", "sha256"}:
        raise ValueError("playback audit unsupported: replay actions or teacher provenance are missing")
    root = Path(source["path"])
    if not root.is_absolute() or root.is_symlink():
        raise ValueError("playback teacher source must be an accessible absolute directory")
    teacher_path = root / "teacher-report.json"
    if teacher_path.is_symlink() or not teacher_path.is_file():
        raise ValueError("playback audit unsupported: teacher-report.json is inaccessible")
    if _digest(teacher_path.read_bytes()) != source["sha256"]:
        raise ValueError("playback teacher source hash mismatch")
    teacher = json.loads(teacher_path.read_text())
    calls = teacher.get("calls")
    if not isinstance(calls, list):
        raise ValueError("invalid playback teacher calls")
    indexes = sorted(set(row.get("index") for row in calls if isinstance(row, dict)))
    if indexes != list(range(len(indexes))):
        raise ValueError("playback teacher indexes are not contiguous")
    expected = []
    for index in indexes:
        actions = {}
        for rid in ROBOTS:
            matches = [row for row in calls if row.get("index") == index and row.get("robot_id") == rid]
            if len(matches) != 1 or not isinstance(matches[0].get("action"), dict):
                raise ValueError("playback teacher robot/index coverage mismatch")
            actions[rid] = matches[0]["action"]
        expected.append(actions)
    if not _close(expected, saved):
        raise ValueError("archived playback actions differ from exact teacher source")
    return saved


def _audit_actor(run_dir: Path, model_root: Path, report: dict[str, Any]) -> dict[str, Any]:
    if report.get("schema") != "ugrp.short_transport_student.v1":
        raise ValueError("result schema mismatch")
    config = report.get("config")
    if not isinstance(config, dict) or config.get("condition") not in ("visual", "playback"):
        raise ValueError("invalid saved condition")
    if config.get("maximum_carry_slices") != 100 or config.get("stationary_confirmations") != 2 or config.get("weld") is not False:
        raise ValueError("saved carry protocol mismatch")
    condition = config["condition"]
    replay = _playback(report) if condition == "playback" else None
    models = _models(model_root, report)

    anchors = report.get("actor_initial_images")
    arm_archive = report.get("actor_initial_issued_arm_commands")
    if (not isinstance(anchors, dict) or set(anchors) != set(ROBOTS)
            or not isinstance(arm_archive, dict) or set(arm_archive) != set(ROBOTS)
            or any(not isinstance(arm_archive[r], dict) for r in ROBOTS)):
        raise ValueError("initial actor archive schema mismatch")
    anchor_bytes, anchor_records = {}, {}
    for rid in ROBOTS:
        if not isinstance(anchors[rid], dict) or set(anchors[rid]) != IMAGE_KEYS:
            raise ValueError(f"initial image schema mismatch: {rid}")
        own, _ = _rgb(run_dir, anchors[rid]["own"])
        top, _ = _rgb(run_dir, anchors[rid]["top"])
        anchor_bytes[rid] = (own, top)
        anchor_records[rid] = anchors[rid]

    calls = report.get("carry_calls")
    if not isinstance(calls, list) or not calls or len(calls) % len(ROBOTS):
        raise ValueError("invalid paired carry calls")
    histories = {r: [] for r in ROBOTS}
    last_frame = {r: 0 for r in ROBOTS}
    confirming, consecutive, carry_ready = False, 0, False
    pairs = len(calls) // len(ROBOTS)
    if pairs > 100:
        raise ValueError("carry call budget exceeded")
    for index in range(pairs):
        pair = calls[index * 2:index * 2 + 2]
        if [row.get("robot_id") if isinstance(row, dict) else None for row in pair] != list(ROBOTS):
            raise ValueError(f"missing ordered robot pair at carry index {index}")
        decisions, top_hash = {}, None
        for row in pair:
            if set(row) != CALL_KEYS:
                raise ValueError("carry call schema or unexpected actor input keys")
            rid, frame_id = row["robot_id"], row["frame_id"]
            if row["index"] != index:
                raise ValueError("carry indexes are not sequential")
            if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id <= 0 or frame_id <= last_frame[rid]:
                raise ValueError("carry frame is not fresh and positive")
            last_frame[rid] = frame_id
            if row["stationary"] is not confirming or row["own_command_history"] != histories[rid]:
                raise ValueError("stationary flag or own command history mismatch")
            if not isinstance(row["images"], dict) or set(row["images"]) != IMAGE_KEYS:
                raise ValueError("carry image schema mismatch")
            own, _ = _rgb(run_dir, row["images"]["own"])
            top, digest = _rgb(run_dir, row["images"]["top"])
            if top_hash is not None and top_hash != digest:
                raise ValueError("robots do not share the same top RGB at carry index")
            top_hash = digest
            if index == 0 and row["images"] != anchor_records[rid]:
                raise ValueError("first carry images do not match archived anchors")
            initial_own, initial_top = anchor_bytes[rid]
            decision = _predict(models[rid], own, top, initial_own, initial_top,
                                list(histories[rid]))
            if not _close(decision, row["decision"]):
                raise ValueError(f"RGB decision replay mismatch: {rid}/{index}")
            decisions[rid] = decision
        if pair[0]["frame_id"] != pair[1]["frame_id"]:
            raise ValueError("paired carry calls do not share a frame id")
        if replay is None:
            control = choose_carry_actions(decisions, confirming)
        else:
            ready = index >= len(replay)
            control = {"valid": True, "ready": ready,
                       "duration_s": .25 if ready else .2,
                       "forwards": {r: 0.0 if ready else float(replay[index][r]["forward"])
                                    for r in ROBOTS}}
        for row in pair:
            rid = row["robot_id"]
            expected = {"kind": "drive", "forward": control["forwards"][rid],
                        "turn": 0.0, "duration_s": control["duration_s"]}
            saved = _action(row["action"], f"{rid}/{index}")
            if not _close(expected, saved):
                raise ValueError(f"carry action replay mismatch: {rid}/{index}")
            histories[rid].append(saved)
        if not control["valid"]:
            if index != pairs - 1:
                raise ValueError("carry calls continue after invalid RGB decision")
            break
        if confirming:
            consecutive = consecutive + 1 if control["ready"] else 0
            if consecutive >= 2:
                carry_ready = True
                if index != pairs - 1:
                    raise ValueError("carry calls continue after stationary qualification")
                break
            if not control["ready"]:
                confirming = False
        elif control["ready"]:
            confirming = True
    if not carry_ready and pairs < 100 and control["valid"]:
        raise ValueError("carry calls terminate before qualification or budget")
    if bool(report.get("carry_ready")) != carry_ready:
        raise ValueError("saved carry_ready differs from replay")
    return {"passed": True, "condition": condition, "carry_ready": carry_ready,
            "carry_call_count": len(calls), "carry_frame_count": pairs,
            "initial_arm_command_archives": len(arm_archive),
            "checked": "exact model and RGB hashes; current and anchor RGB; own issued-action histories; prediction, action, bounds, and stationary transition replay"}


def audit(run_dir: Path, transport_model_dir: Path) -> dict[str, Any]:
    run_dir, model_root = Path(run_dir).resolve(), Path(transport_model_dir).resolve()
    errors: list[str] = []
    actor = None
    try:
        report = json.loads(_safe_file(run_dir, "result.json").read_text())
        actor = _audit_actor(run_dir, model_root, report)
    except Exception as exc:
        return {"success": False, "rgb_action_audit_passed": False,
                "physics_evaluation_matches": False, "experiment_success": False,
                "errors": [f"{type(exc).__name__}: {exc}"], "counts": {}}

    physics_matches = False
    evaluation = None
    try:
        samples = [json.loads(line) for line in _safe_file(run_dir, "evaluation-only.jsonl").read_text().splitlines() if line.strip()]
        evaluation = evaluate_transport_samples(samples)
        if not _close(evaluation, report.get("evaluation")):
            raise ValueError("saved physics evaluation differs from fresh evaluation")
        if report.get("weld_active_ticks") != 0:
            raise ValueError("weld_active_ticks is not zero")
        if report.get("invariants_initial") != report.get("invariants_final"):
            raise ValueError("geometry or policy camera invariants changed")
        physics_matches = True
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    experiment_success = bool(report.get("success"))
    expected_success = bool(report.get("error") is None and report.get("approach_ok")
        and actor["carry_ready"] and evaluation and evaluation.get("success")
        and report.get("weld_active_ticks") == 0
        and report.get("approach_payload_contact_steps") == 0
        and report.get("invariants_initial") == report.get("invariants_final"))
    if experiment_success != expected_success:
        errors.append("ValueError: saved success predicate mismatch")
    return {"success": actor["passed"] and physics_matches and not errors,
            "rgb_action_audit_passed": actor["passed"],
            "physics_evaluation_matches": physics_matches,
            "experiment_success": experiment_success, "actor": actor,
            "evaluation": evaluation, "errors": errors,
            "counts": {"carry_calls": actor["carry_call_count"],
                       "carry_frames": actor["carry_frame_count"],
                       "evaluation_samples": evaluation.get("metrics", {}).get("sample_count") if evaluation else None},
            "limitations": "Recomputation validates archived evidence and controller consistency; it does not prove process isolation or OS sandboxing."}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--transport-model-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = audit(args.run_dir, args.transport_model_dir)
    value = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.out:
        args.out.write_text(value)
    else:
        print(value, end="")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
