#!/usr/bin/env python3
"""Replay a grasp student's decisions from stored RGB, model, and commands only."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.grasp_student_inference import predict_student as predict_correction


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha(path: Path) -> str:
    return _sha_bytes(path.read_bytes())


def _inside(root: Path, relative: Any) -> Path:
    if not isinstance(relative, str) or not relative:
        raise ValueError("artifact path must be a non-empty relative string")
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"artifact path escapes its root: {relative}")
    return path


def _image(run_dir: Path, record: Any) -> bytes:
    if not isinstance(record, dict) or not isinstance(record.get("sha256"), str):
        raise ValueError("invalid RGB record")
    path = _inside(run_dir, record.get("path"))
    value = path.read_bytes()
    if _sha_bytes(value) != record["sha256"]:
        raise ValueError(f"RGB hash mismatch: {record.get('path')}")
    return value


def _commands(value: Any, label: str) -> dict[int, int]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a command mapping")
    try:
        result = {int(k): int(v) for k, v in value.items()}
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} has invalid command values") from exc
    if any(isinstance(v, bool) or not 500 <= v <= 2500 for v in result.values()):
        raise ValueError(f"{label} has out-of-range commands")
    return result


def _canonical(value: Any) -> Any:
    return json.loads(json.dumps(value, sort_keys=True, allow_nan=False))


def _calls(report: dict[str, Any]) -> list[dict[str, Any]]:
    rows = report.get("calls")
    if not isinstance(rows, list):
        raise ValueError("missing result.json calls")
    return rows


def audit(run_dir: Path | str, model_dir: Path | str, *, report_name: str = "result.json") -> dict[str, Any]:
    run_dir, model_dir = Path(run_dir).resolve(), Path(model_dir).resolve()
    report_path = _inside(run_dir, report_name)
    report = json.loads(report_path.read_text())
    config = report.get("config")
    if not isinstance(config, dict):
        raise ValueError("missing run config")
    condition, max_step = config.get("condition"), config.get("max_step")
    if condition not in ("visual", "playback"):
        raise ValueError("unexpected student condition")
    if isinstance(max_step, bool) or not isinstance(max_step, int):
        raise ValueError("invalid saved max_step")

    skill_path = model_dir / "student-skill.json"
    if report.get("skill_sha256") != _sha(skill_path):
        raise ValueError("student skill hash mismatch")
    skill = json.loads(skill_path.read_text())
    records = skill.get("models")
    if not isinstance(records, dict) or not records:
        raise ValueError("skill has no student models")
    models: dict[str, dict[str, Any]] = {}
    for rid, record in records.items():
        if not isinstance(record, dict):
            raise ValueError(f"invalid model record: {rid}")
        path = _inside(model_dir, record.get("path"))
        digest = _sha(path)
        if digest != record.get("sha256"):
            raise ValueError(f"trained model hash mismatch: {rid}")
        if report.get("model_sha256", {}).get(rid) != digest:
            raise ValueError(f"run model hash mismatch: {rid}")
        models[rid] = json.loads(path.read_text())

    state_record = report.get("actor_initial_issued_commands")
    if not isinstance(state_record, dict) or set(state_record) != set(models):
        raise ValueError("actor initial command history does not match model robots")
    states = {rid: _commands(state_record[rid], f"initial commands for {rid}") for rid in models}
    calls = _calls(report)
    seen: set[tuple[int, str]] = set()
    replay = []
    shared_top: dict[int, str] = {}
    for call in calls:
        if not isinstance(call, dict):
            raise ValueError("invalid student call record")
        rid, round_index = call.get("robot_id"), call.get("round")
        if rid not in models or isinstance(round_index, bool) or not isinstance(round_index, int):
            raise ValueError("invalid robot or round in student call")
        key = (round_index, rid)
        if key in seen:
            raise ValueError(f"duplicate student call: {key}")
        seen.add(key)
        if call.get("condition") != condition:
            raise ValueError(f"condition mismatch: {rid} round {round_index}")
        before = _commands(call.get("own_commands_before"), "own_commands_before")
        if before != states[rid]:
            raise ValueError(f"own command history mismatch: {rid} round {round_index}")
        images = call.get("images")
        if not isinstance(images, dict):
            raise ValueError("missing RGB records")
        own = _image(run_dir, images.get("own"))
        top = _image(run_dir, images.get("top"))
        top_digest = _sha_bytes(top)
        if round_index in shared_top and shared_top[round_index] != top_digest:
            raise ValueError(f"shared top RGB mismatch: round {round_index}")
        shared_top[round_index] = top_digest
        decision = predict_correction(models[rid], own, top, max_step=max_step)
        if _canonical(decision) != call.get("decision"):
            raise ValueError(f"student decision replay mismatch: {rid} round {round_index}")
        applied = decision["delta_pulses"] if condition == "visual" else [0, 0, 0]
        channels = models[rid]["channels"]
        targets = {
            int(ch): max(500, min(2500, before[int(ch)] + int(delta)))
            for ch, delta in zip(channels, applied) if int(delta)
        }
        actions = [
            {"kind": "arm", "servo_id": ch, "pulse": pulse}
            for ch, pulse in targets.items()
        ]
        if actions != call.get("actions"):
            raise ValueError(f"final raw action replay mismatch: {rid} round {round_index}")
        states[rid].update(targets)
        replay.append({"round": round_index, "robot_id": rid,
                       "own_sha256": images["own"]["sha256"],
                       "top_sha256": images["top"]["sha256"], "actions": actions})

    rounds = config.get("rounds")
    if isinstance(rounds, bool) or not isinstance(rounds, int) or rounds < 1:
        raise ValueError("invalid saved round count")
    expected = {(index, rid) for index in range(rounds) for rid in models}
    if seen != expected:
        raise ValueError("student call coverage differs from saved round schedule")
    final_commands = report.get("preclose_issued_commands")
    if not isinstance(final_commands, dict) or any(
            _commands(final_commands.get(rid), f"preclose commands for {rid}") != states[rid]
            for rid in models):
        raise ValueError("final own command history mismatch")

    post_images = report.get("post_recovery_images")
    final_decisions = report.get("final_visual_errors")
    if not isinstance(post_images, dict) or not isinstance(final_decisions, dict):
        raise ValueError("missing post-recovery RGB or decisions")
    for rid in models:
        image_records = post_images.get(rid)
        if not isinstance(image_records, dict):
            raise ValueError(f"missing post-recovery RGB: {rid}")
        final = predict_correction(
            models[rid], _image(run_dir, image_records.get("own")),
            _image(run_dir, image_records.get("top")), max_step=max_step,
        )
        if _canonical(final) != final_decisions.get(rid):
            raise ValueError(f"final visual decision mismatch: {rid}")

    return {
        "ok": True, "condition": condition, "rounds": rounds,
        "calls": len(calls), "models": report["model_sha256"],
        "calls_source": "result.json:calls", "replay": replay,
        "checked": (
            "skill/model and RGB hashes; exact RGB-only correction, condition-specific pulse "
            "deltas, raw arm actions, and own-command history; evaluator data unused"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run_dir, args.model_dir)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, sort_keys=True, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "replay"}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
