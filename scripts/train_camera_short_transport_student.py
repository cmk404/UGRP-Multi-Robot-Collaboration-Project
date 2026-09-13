#!/usr/bin/env python3
"""Train per-robot fixed-scene RGB short-transport regressors from teacher records."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.camera_short_transport_student import (FEATURE_NAMES, REGRESSION_NAMES,
    extract_transport_features, regression_vector)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_image(root: Path, record: dict, dataset_images: dict[str, dict]) -> bytes:
    if not isinstance(record, dict) or set(record) != {"path", "sha256"}:
        raise ValueError("image record requires path and sha256")
    path = (root / record["path"]).resolve(strict=True)
    if not path.is_relative_to(root.resolve()) or path.is_symlink():
        raise ValueError("image path escapes teacher directory")
    digest = sha(path)
    if digest != record["sha256"]:
        raise ValueError(f"image SHA mismatch: {record['path']}")
    key = f"{root}:{Path(record['path']).as_posix()}"
    dataset_images[key] = {"teacher_root": str(root), "path": record["path"], "sha256": digest}
    return path.read_bytes()


def train(teacher_dirs: list[Path], out_dir: Path) -> dict:
    roots = [path.resolve() for path in teacher_dirs]
    all_reports, dataset_images = [], {}
    for root in roots:
        report_path = root / "teacher-report.json"
        report = json.loads(report_path.read_text())
        if not re.fullmatch(r"[0-9a-f]{40}", str(report.get("source_sha", ""))):
            raise ValueError(f"invalid teacher source SHA: {root}")
        evaluation = report.get("evaluation")
        evaluation_path = root / "evaluation.json"
        if evaluation is None and evaluation_path.exists():
            evaluation = json.loads(evaluation_path.read_text())
        if report.get("error") is None and (not isinstance(evaluation, dict)
                                             or evaluation.get("success") is not True):
            raise ValueError(f"completed teacher lacks passing evaluation: {root}")
        all_reports.append((root, report_path, report))
        for row in list(report.get("calls", [])) + list(report.get("state_samples", [])):
            images = row.get("images") if isinstance(row, dict) else None
            if not isinstance(images, dict) or set(images) != {"own", "top"}:
                raise ValueError(f"teacher input lacks exact own/top image records: {root}")
            checked_image(root, images["own"], dataset_images)
            checked_image(root, images["top"], dataset_images)
    reports = [item for item in all_reports if item[2].get("error") is None
               and item[2].get("calls")]
    if not reports:
        raise ValueError("no completed teacher demonstrations")
    out_dir.mkdir(parents=True, exist_ok=False)
    training_source_sha = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    manifest = {"schema": "ugrp.camera_short_transport_skill.v1", "models": {},
                "training_code_source_sha": training_source_sha,
                "scope": "fixed demonstrated scene; own RGB + fixed top RGB + first-carry RGB anchor + own issued history",
                "training_sources": [{"path": str(path), "sha256": sha(path),
                                      "source_sha": report["source_sha"],
                                      "completed_demo": report.get("error") is None}
                                     for _, path, report in all_reports]}
    training_report = {"training_code_source_sha": training_source_sha,
                       "models": {}, "negative_samples": 0, "negative_confusions": 0}
    for rid in ("r1", "r3"):
        x_rows, y_rows, raw, ready_true, ready_false = [], [], [], [], []
        negative_features, demo_rows = [], {}
        for root, _path, report in reports:
            demo_rows[str(root)] = []
            calls = [row for row in report["calls"] if row.get("robot_id") == rid]
            useful = [row for row in calls if isinstance(row.get("teacher_labels"), dict)
                      and row["teacher_labels"].get("held") is True]
            if not useful:
                continue
            first = useful[0]
            anchor = extract_transport_features(
                checked_image(root, first["images"]["own"], dataset_images),
                checked_image(root, first["images"]["top"], dataset_images), rid)
            if not anchor["ok"]:
                continue
            for row in useful:
                feat = extract_transport_features(
                    checked_image(root, row["images"]["own"], dataset_images),
                    checked_image(root, row["images"]["top"], dataset_images), rid)
                if not feat["ok"]:
                    continue
                history = row.get("own_command_history", [])
                issued = sum(float(a["forward"]) * float(a["duration_s"]) for a in history)
                progress = float(row["teacher_labels"]["payload_progress_m"])
                x_rows.append(regression_vector(feat["features"], anchor["features"], issued))
                y_rows.append(progress); raw.append(feat["features"])
                demo_rows[str(root)].append((x_rows[-1], progress))
                (ready_true if row["teacher_labels"].get("ready") is True else ready_false).append(progress)
        for root, _path, report in all_reports:
            for row in report.get("state_samples", []):
                if row.get("robot_id") != rid or row.get("teacher_labels", {}).get("held") is not False:
                    continue
                feat = extract_transport_features(
                    checked_image(root, row["images"]["own"], dataset_images),
                    checked_image(root, row["images"]["top"], dataset_images), rid)
                if feat["ok"]:
                    negative_features.append(feat["features"])
        if len(x_rows) < 3:
            raise ValueError(f"insufficient useful carry samples for {rid}")
        coefficients = np.linalg.lstsq(np.asarray(x_rows), np.asarray(y_rows), rcond=None)[0]
        predictions = np.asarray(x_rows) @ coefficients
        def cross_validation(columns):
            if len([rows for rows in demo_rows.values() if rows]) < 2:
                return {"available": False, "reason": "requires_at_least_two_demonstrations"}
            errors = []
            for held_out, held_rows in demo_rows.items():
                train_rows = [row for key, rows in demo_rows.items() if key != held_out for row in rows]
                if not train_rows or not held_rows:
                    continue
                train_x = np.asarray([[row[0][i] for i in columns] for row in train_rows])
                beta = np.linalg.lstsq(train_x, np.asarray([row[1] for row in train_rows]), rcond=None)[0]
                errors.extend(float(np.dot([row[0][i] for i in columns], beta) - row[1])
                              for row in held_rows)
            return {"available": True, "rmse_m": float(np.sqrt(np.mean(np.square(errors)))),
                    "max_abs_error_m": float(max(map(abs, errors))), "samples": len(errors)}
        cv_payload = cross_validation((0, 1))
        cv_payload_robot = cross_validation((0, 1, 2))
        support = {}
        for name in FEATURE_NAMES:
            values = [row[name] for row in raw]
            width = max(values) - min(values)
            pad = max(.005, width * .20)
            support[name] = [float(min(values) - pad), float(max(values) + pad)]
        orange_values = [row["own_orange_fraction"] for row in raw]
        held_floor = max(0.0, min(orange_values) - .05)
        positive_array = np.asarray([[row[name] for name in FEATURE_NAMES] for row in raw])
        negative_array = np.asarray([[row[name] for name in FEATURE_NAMES]
                                     for row in negative_features]) if negative_features else None
        positive_center = np.mean(positive_array, axis=0)
        scale = np.maximum(np.ptp(positive_array, axis=0), .005)
        negative_center = (np.mean(negative_array, axis=0) if negative_array is not None
                           else positive_center + scale * 10.0)
        # Fixed pre-registered cutoff for the 20 cm task. It intentionally
        # anticipates residual coast instead of learning a later stop point.
        ready_at = .19
        model = {"schema": "ugrp.camera_short_transport_model.v1", "robot_id": rid,
                 "feature_names": list(FEATURE_NAMES), "regression_names": list(REGRESSION_NAMES),
                 "progress_coefficients": [float(v) for v in coefficients],
                 "feature_support": support, "held_own_orange_min": held_floor,
                 "held_positive_center": [float(v) for v in positive_center],
                 "held_negative_center": [float(v) for v in negative_center],
                 "held_feature_scale": [float(v) for v in scale],
                 "ready_progress_m": float(ready_at), "slow_progress_m": float(min(.17, ready_at - .02)),
                 "ready_threshold_source": "fixed_0.19m_for_0.20m_task_to_allow_coast",
                 "target_distance_m": .20,
                 "limitations": "fixed demonstrated camera/appearance domain; held is an RGB estimate, not bilateral-contact proof"}
        path = out_dir / f"{rid}-model.json"
        path.write_text(json.dumps(model, indent=2, sort_keys=True) + "\n")
        manifest["models"][rid] = {"path": path.name, "sha256": sha(path)}
        def predicted_held(feature):
            vector = np.asarray([feature[name] for name in FEATURE_NAMES])
            return (np.linalg.norm((vector - positive_center) / scale)
                    <= np.linalg.norm((vector - negative_center) / scale))
        confusions = int(sum(bool(predicted_held(feature)) for feature in negative_features))
        positive_misses = int(sum(not bool(predicted_held(feature)) for feature in raw))
        training_report["negative_samples"] += len(negative_features)
        training_report["negative_confusions"] += confusions
        training_report["models"][rid] = {"samples": len(y_rows),
            "rmse_m": float(np.sqrt(np.mean((predictions - y_rows) ** 2))),
            "max_abs_error_m": float(np.max(np.abs(predictions - y_rows))),
            "negative_samples": len(negative_features), "negative_confusions": confusions,
            "positive_samples": len(raw), "positive_confusions": positive_misses}
        training_report["models"][rid]["leave_one_demo_out"] = {
            "payload_dx_only": cv_payload, "payload_plus_own_robot_dx": cv_payload_robot,
            "selected": "payload_plus_own_robot_dx"}
    manifest["dataset_images"] = [dataset_images[key] for key in sorted(dataset_images)]
    manifest_path = out_dir / "short-transport-skill.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    (out_dir / "training-report.json").write_text(json.dumps(training_report, indent=2, sort_keys=True) + "\n")
    return training_report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher-dir", type=Path, nargs="+", required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(train(args.teacher_dir, args.out_dir), sort_keys=True))


if __name__ == "__main__":
    main()
