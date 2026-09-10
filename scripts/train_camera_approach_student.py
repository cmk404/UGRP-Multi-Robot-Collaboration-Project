#!/usr/bin/env python3
"""Train per-robot RGB straight-approach models from privileged teacher labels."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.camera_approach_student import fit_approach_model


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, value) -> None:
    path.write_text(json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n")


def _load_rgb(root: Path, record: dict) -> bytes:
    if not isinstance(record, dict) or not isinstance(record.get("path"), str):
        raise ValueError("invalid RGB record")
    path = (root / record["path"]).resolve()
    if not path.is_relative_to(root) or _sha(path) != record.get("sha256"):
        raise ValueError("teacher RGB path/hash mismatch")
    return path.read_bytes()


def _labels(value):
    if isinstance(value, list):
        rows = value
    elif isinstance(value, dict):
        rows = [{"sample_id": key, **label} for key, label in value.items()]
    else:
        raise ValueError("privileged_labels.json must be a list or mapping")
    result = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("invalid privileged label row")
        sample_id = row.get("sample_id", row.get("id"))
        if not isinstance(sample_id, str) or not sample_id or sample_id in result:
            raise ValueError("invalid or duplicate privileged label sample_id")
        result[sample_id] = row
    return result


def _balanced(rows: list[dict], cap: int = 400) -> list[dict]:
    if len(rows) <= cap:
        return rows
    groups: dict[str, list[dict]] = {}
    for row in rows:
        groups.setdefault(row["case_id"], []).append(row)
    ordered = sorted(groups, key=lambda value: hashlib.sha256(value.encode()).hexdigest())
    # Preserve every narrow stop/near-stop example before subsampling approach
    # frames.  Silently dropping this band would make false readiness likelier.
    selected = [row for case_id in ordered for row in groups[case_id]
                if row["stop"] or row["forward"] <= 0.003]
    if len(selected) > cap:
        raise ValueError("near-stop samples alone exceed deterministic training cap")
    selected_ids = {row["sample_id"] for row in selected}
    remaining = {case_id: [row for row in groups[case_id]
                           if row["sample_id"] not in selected_ids] for case_id in ordered}
    missing = [case_id for case_id in ordered
               if not any(row["case_id"] == case_id for row in selected)]
    if len(selected) + len(missing) > cap:
        raise ValueError("training cap cannot represent every trajectory")
    quotas = {case_id: int(case_id in missing) for case_id in ordered}
    capacity = cap - len(selected) - len(missing)
    while capacity:
        progressed = False
        for case_id in ordered:
            if quotas[case_id] < len(remaining[case_id]):
                quotas[case_id] += 1;capacity -= 1
                progressed = True
                if capacity == 0:
                    break
        if not progressed:
            break
    for case_id in ordered:
        values, count = remaining[case_id], quotas[case_id]
        if count == 1:
            indices = [len(values) // 2]
        elif count > 1:
            indices = [round(i * (len(values) - 1) / (count - 1)) for i in range(count)]
        else:
            indices = []
        selected.extend(values[index] for index in indices)
    return selected


def _validate_pair(actor: dict, label: dict, sample_id: str,
                   declared_cases: set[str]) -> None:
    rid, case_id = actor.get("robot_id"), actor.get("case_id")
    if rid not in ("r1", "r3"):
        raise ValueError(f"invalid actor robot_id: {sample_id}")
    if not isinstance(case_id, str) or case_id not in declared_cases:
        raise ValueError(f"actor case is not declared successful or excluded: {sample_id}")
    if "robot_id" in label and label.get("robot_id") != rid:
        raise ValueError(f"actor/label robot mismatch: {sample_id}")
    if "case_id" in label and label.get("case_id") != case_id:
        raise ValueError(f"actor/label case mismatch: {sample_id}")
    forward, stop = label.get("forward"), label.get("stop")
    if (isinstance(forward, bool) or not isinstance(forward, (int, float))
            or not math.isfinite(float(forward)) or not 0 <= float(forward) <= 0.15):
        raise ValueError(f"invalid privileged forward label: {sample_id}")
    if not isinstance(stop, bool):
        raise ValueError(f"invalid privileged stop label: {sample_id}")


def train(teacher_dir: Path | str, out_dir: Path | str) -> dict:
    started = time.monotonic()
    teacher, out = Path(teacher_dir).resolve(), Path(out_dir).resolve()
    if out.exists():
        raise FileExistsError(out)
    report = json.loads((teacher / "report.json").read_text())
    if report.get("complete") is not True:
        raise ValueError("teacher report must be complete before training")
    actors = json.loads((teacher / "actor_samples.json").read_text())
    labels = _labels(json.loads((teacher / "privileged_labels.json").read_text()))
    successful = report.get("successful_cases")
    excluded = report.get("excluded_cases", report.get("excluded"))
    if not isinstance(successful, list) or not successful or not all(isinstance(x, str) for x in successful):
        raise ValueError("report successful_cases must be a non-empty case-id list")
    if not isinstance(excluded, list) or not all(isinstance(x, str) for x in excluded):
        raise ValueError("report excluded_cases must be a case-id list")
    if set(successful) & set(excluded):
        raise ValueError("successful and excluded cases overlap")
    goals = report.get("goal_references")
    if not isinstance(actors, list) or not isinstance(goals, dict):
        raise ValueError("teacher report or actor samples are invalid")
    actor_by_id = {}
    for row in actors:
        sample_id = row.get("id") if isinstance(row, dict) else None
        if not isinstance(sample_id, str) or not sample_id or sample_id in actor_by_id:
            raise ValueError("invalid or duplicate actor sample id")
        actor_by_id[sample_id] = row
    if set(actor_by_id) != set(labels):
        raise ValueError("actor and privileged label IDs differ")
    declared = set(successful) | set(excluded)
    loaded = {}
    for sample_id, actor in actor_by_id.items():
        label = labels[sample_id]
        _validate_pair(actor, label, sample_id, declared)
        observations = actor.get("observations")
        if not isinstance(observations, dict):
            raise ValueError(f"missing actor observations: {sample_id}")
        loaded[sample_id] = (
            _load_rgb(teacher, observations.get("own_rgb")),
            _load_rgb(teacher, observations.get("shared_top_rgb")),
        )
    out.mkdir(parents=True)
    model_records, diagnostics = {}, {}
    for rid in ("r1", "r3"):
        goal = goals.get(rid)
        if not isinstance(goal, dict):
            raise ValueError(f"missing goal reference for {rid}")
        ref_own = _load_rgb(teacher, goal.get("own_rgb"))
        ref_top = _load_rgb(teacher, goal.get("shared_top_rgb"))
        joined = []
        available_count = 0
        for sample_id, actor in actor_by_id.items():
            label = labels[sample_id]
            case_id = actor.get("case_id")
            if actor.get("robot_id") != rid or case_id not in successful:
                continue
            available_count += 1
            joined.append({"sample_id": sample_id, "case_id": case_id,
                           "own_jpeg": loaded[sample_id][0],
                           "top_jpeg": loaded[sample_id][1],
                           "forward": label.get("forward"), "stop": label.get("stop")})
        domain_samples = joined
        joined = _balanced(joined)
        if len({row["case_id"] for row in joined}) < 4:
            raise ValueError(f"{rid} requires at least four successful teacher trajectories")
        model = fit_approach_model(ref_own, ref_top, joined,
                                   domain_samples=domain_samples)
        path = out / f"model-{rid}.json";_write(path, model)
        model_records[rid] = {"path": path.name, "sha256": _sha(path)}
        diagnostics[rid] = {"available_sample_count": available_count,
                            "selected_sample_count": len(joined),
                            "subsampled_sample_count": available_count - len(joined),
                            "selected_case_count": len({row['case_id'] for row in joined}),
                            "selected_sample_ids": [row["sample_id"] for row in joined],
                            "model": model["diagnostics"]}
    skill = {"schema": "ugrp.rgb_short_approach_skill.v1",
             "scope": "20-30cm straight physical-wheel approach to learned RGB stop; executor confirms two stationary frames",
             "runtime_inputs": ["own_rgb", "fixed_top_rgb"],
             "models": model_records,
             "readiness": {"executor_fresh_stationary_confirmations": 2,
                           "model_stop_score": 0.65, "model_forward_max": 0.003}}
    _write(out / "approach-skill.json", skill)
    result = {"source_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "teacher_dir": str(teacher),
              "input_hashes": {name: _sha(teacher / name) for name in
                               ("actor_samples.json", "privileged_labels.json", "report.json")},
              "successful_cases": successful, "excluded_cases": excluded,
              "diagnostics": diagnostics, "models": model_records,
              "runtime_boundary": "model receives own RGB and fixed top RGB only; no pose, distance, IK, contact, evaluator, or command target",
              "wall_elapsed_s": time.monotonic() - started}
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
