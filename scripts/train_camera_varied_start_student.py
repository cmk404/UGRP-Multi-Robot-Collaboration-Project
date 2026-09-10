#!/usr/bin/env python3
"""Train per-robot staged RGB models for bounded varied starts."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.camera_varied_start_student import (COMMAND_BOUNDS, STAGES,
                                                  fit_stage_model)

ROBOTS = ("r1", "r3")
REGRESSION_CAP = 600


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _image(root: Path, record: Any) -> bytes:
    if not isinstance(record, dict) or not isinstance(record.get("path"), str):
        raise ValueError("invalid teacher RGB record")
    path = (root / record["path"]).resolve()
    if not path.is_relative_to(root) or not path.is_file() or _sha(path) != record.get("sha256"):
        raise ValueError("teacher RGB path/hash mismatch")
    return path.read_bytes()


def _labels(value: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError("privileged labels must be a list")
    result = {}
    for row in value:
        sample_id = row.get("sample_id") if isinstance(row, dict) else None
        if not isinstance(sample_id, str) or not sample_id or sample_id in result:
            raise ValueError("invalid or duplicate label sample_id")
        result[sample_id] = row
    return result


def _jsonl(path: Path) -> list[Any]:
    rows = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise ValueError(f"blank JSONL row: {path.name}:{line_number}")
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL row: {path.name}:{line_number}") from exc
    return rows


def _validate_pair(actor: Any, label: Any, declared: set[str]) -> None:
    if not isinstance(actor, dict) or not isinstance(label, dict):
        raise ValueError("actor/label row must be an object")
    for key in ("case_id", "robot_id", "stage"):
        if actor.get(key) != label.get(key):
            raise ValueError(f"actor/label {key} mismatch")
    if actor.get("case_id") not in declared or actor.get("robot_id") not in ROBOTS or actor.get("stage") not in STAGES:
        raise ValueError("actor sample is outside the declared varied-start scope")
    command, ready = label.get("command"), label.get("ready")
    if (isinstance(command, bool) or not isinstance(command, (int, float))
            or not math.isfinite(command) or not COMMAND_BOUNDS[actor["stage"]][0] <= command <= COMMAND_BOUNDS[actor["stage"]][1]
            or not isinstance(ready, bool)):
        raise ValueError("invalid stage command or readiness label")


def _balanced(rows: list[dict[str, Any]], cap: int = REGRESSION_CAP) -> list[dict[str, Any]]:
    if len(rows) <= cap:
        return rows
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(row["case_id"], []).append(row)
    ordered = sorted(groups, key=lambda x: hashlib.sha256(x.encode()).hexdigest())
    selected = [row for case in ordered for row in groups[case]
                if row["ready"] or abs(row["command"]) <= .003]
    if len(selected) > cap:
        raise ValueError("ready/near-zero stage samples exceed regression cap")
    selected_ids = {row["sample_id"] for row in selected}
    remaining = {case: [row for row in groups[case] if row["sample_id"] not in selected_ids]
                 for case in ordered}
    missing = [case for case in ordered if not any(row["case_id"] == case for row in selected)]
    if len(selected) + len(missing) > cap:
        raise ValueError("regression cap cannot represent every stage trajectory")
    quotas = {case: int(case in missing) for case in ordered}
    capacity = cap - len(selected) - len(missing)
    while capacity:
        progressed = False
        for case in ordered:
            if quotas[case] < len(remaining[case]):
                quotas[case] += 1
                capacity -= 1
                progressed = True
                if not capacity:
                    break
        if not progressed:
            break
    for case in ordered:
        values, count = remaining[case], quotas[case]
        indices = ([len(values) // 2] if count == 1 else
                   [round(i * (len(values) - 1) / (count - 1)) for i in range(count)] if count else [])
        selected.extend(values[index] for index in indices)
    return selected


def train(teacher_dir: Path | str, out_dir: Path | str) -> dict[str, Any]:
    teacher, out = Path(teacher_dir).resolve(), Path(out_dir).resolve()
    if out.exists():
        raise FileExistsError(out)
    report_path, actors_path, labels_path = (teacher / "report.json", teacher / "actor-samples.jsonl",
                                             teacher / "teacher-labels.jsonl")
    report = json.loads(report_path.read_text())
    if report.get("complete") is not True:
        raise ValueError("teacher report must be complete before training")
    successful, excluded = report.get("successful_cases"), report.get("excluded_cases")
    if (not isinstance(successful, list) or not successful or not all(isinstance(x, str) for x in successful)
            or not isinstance(excluded, list) or not all(isinstance(x, str) for x in excluded)
            or set(successful) & set(excluded)):
        raise ValueError("invalid successful/excluded teacher cases")
    declared = set(successful) | set(excluded)
    actors = _jsonl(actors_path)
    labels = _labels(_jsonl(labels_path))
    if not isinstance(actors, list):
        raise ValueError("actor samples must be a list")
    actor_by_id = {}
    loaded = {}
    for actor in actors:
        sample_id = actor.get("id") if isinstance(actor, dict) else None
        if not isinstance(sample_id, str) or not sample_id or sample_id in actor_by_id:
            raise ValueError("invalid or duplicate actor sample id")
        actor_by_id[sample_id] = actor
    if set(actor_by_id) != set(labels):
        raise ValueError("actor and privileged label IDs differ")
    for sample_id, actor in actor_by_id.items():
        label = labels[sample_id]
        _validate_pair(actor, label, declared)
        observations = actor.get("observations")
        if not isinstance(observations, dict):
            raise ValueError("actor observations are missing")
        loaded[sample_id] = (_image(teacher, observations.get("own_rgb")),
                             _image(teacher, observations.get("shared_top_rgb")))
    goals = report.get("goal_references")
    if not isinstance(goals, dict) or set(goals) != set(ROBOTS):
        raise ValueError("teacher goal references must cover both robots")
    out.mkdir(parents=True)
    records, diagnostics = {r: {} for r in ROBOTS}, {r: {} for r in ROBOTS}
    for rid in ROBOTS:
        goal = goals[rid]
        ref_own = _image(teacher, goal.get("own_rgb"))
        ref_top = _image(teacher, goal.get("shared_top_rgb"))
        for stage in STAGES:
            domain = []
            for sample_id, actor in actor_by_id.items():
                if actor["robot_id"] != rid or actor["stage"] != stage or actor["case_id"] not in successful:
                    continue
                label = labels[sample_id]
                domain.append({"sample_id": sample_id, "case_id": actor["case_id"],
                    "own_jpeg": loaded[sample_id][0], "top_jpeg": loaded[sample_id][1],
                    "command": label["command"], "ready": label["ready"]})
            selected = _balanced(domain)
            if len({row["case_id"] for row in selected}) < 4:
                raise ValueError(f"{rid}/{stage} requires at least four successful cases")
            model = fit_stage_model(ref_own, ref_top, selected, rid, stage,
                                    domain_samples=domain)
            path = out / f"model-{rid}-{stage}.json"
            _write(path, model)
            records[rid][stage] = {"path": path.name, "sha256": _sha(path)}
            diagnostics[rid][stage] = {"available_sample_count": len(domain),
                "selected_sample_count": len(selected),
                "subsampled_sample_count": len(domain) - len(selected),
                "selected_case_count": len({row["case_id"] for row in selected}),
                "selected_sample_ids": [row["sample_id"] for row in selected],
                "model": model["diagnostics"]}
    skill = {"schema": "ugrp.rgb_varied_start_skill.v1",
             "scope": {"forward_distance_m": [.15, .40], "lateral_m": [-.06, .06],
                       "heading_deg": [-10, 10], "robots": list(ROBOTS)},
             "runtime_inputs": ["own_rgb", "fixed_top_rgb"], "stage_order": ["yaw", "lateral", "yaw", "forward"],
             "readiness": {"score": .65, "absolute_command_max": .003,
                           "fresh_stationary_confirmations": 2},
             "models": records}
    _write(out / "varied-start-skill.json", skill)
    result = {"source_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "teacher_dir": str(teacher), "successful_cases": successful,
              "excluded_cases": excluded, "regression_cap_per_robot_stage": REGRESSION_CAP,
              "input_hashes": {"report.json": _sha(report_path), "actor-samples.jsonl": _sha(actors_path),
                               "teacher-labels.jsonl": _sha(labels_path)},
              "models": records, "diagnostics": diagnostics,
              "runtime_boundary": "own RGB and fixed-top RGB ROI only; teacher pose and labels are absent at runtime"}
    _write(out / "training-report.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    result = train(args.teacher_dir, args.out_dir)
    print(json.dumps({"models": result["models"], "diagnostics": result["diagnostics"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
