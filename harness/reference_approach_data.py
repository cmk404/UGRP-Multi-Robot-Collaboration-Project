"""Teacher data joins for reference comparisons; never used by the live actor."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from scripts.train_camera_approach_student import _labels, _load_rgb, _validate_pair


def load_teacher(root: Path) -> tuple[dict, dict, dict]:
    root = root.resolve()
    report = json.loads((root / "report.json").read_text())
    if report.get("complete") is not True:
        raise ValueError("teacher collection is incomplete")
    successful, excluded = report["successful_cases"], report["excluded_cases"]
    if (not successful or len(set(successful)) != len(successful)
            or set(successful) & set(excluded)):
        raise ValueError("invalid teacher case sets")
    labels = _labels(json.loads((root / "privileged_labels.json").read_text()))
    actors = json.loads((root / "actor_samples.json").read_text())
    ids = [row["id"] for row in actors]
    if len(set(ids)) != len(ids) or set(ids) != set(labels):
        raise ValueError("duplicate or mismatched sample IDs")
    trajectories = {rid: {} for rid in ("r1", "r3")}
    for row in actors:
        label = labels[row["id"]]
        _validate_pair(row, label, row["id"], set(successful) | set(excluded))
        if set(row["observations"]) != {"own_rgb", "shared_top_rgb"}:
            raise ValueError("actor observation must contain exactly two RGB views")
        own = _load_rgb(root, row["observations"]["own_rgb"])
        top = _load_rgb(root, row["observations"]["shared_top_rgb"])
        if row["case_id"] not in successful:
            continue
        trajectories[row["robot_id"]].setdefault(row["case_id"], []).append({
            "sample_id": row["id"], "case_id": row["case_id"],
            "own_jpeg": own, "top_jpeg": top,
            "forward": label["forward"], "stop": label["stop"],
        })
    for cases in trajectories.values():
        if set(cases) != set(successful):
            raise ValueError("every successful case must have both robots")
    references = {}
    for rid in trajectories:
        goal = report["goal_references"][rid]
        # Deliberately discard teacher base_x and all other privileged fields.
        references[rid] = (_load_rgb(root, goal["own_rgb"]),
                           _load_rgb(root, goal["shared_top_rgb"]))
    return trajectories, references, report


def split_cases(cases: list[str], heldout: list[str]) -> tuple[list[str], list[str]]:
    if not heldout or len(set(heldout)) != len(heldout) or not set(heldout) < set(cases):
        raise ValueError("heldout must be a nonempty proper subset without duplicates")
    training = sorted(set(cases) - set(heldout))
    if len(training) < 4:
        raise ValueError("kernel grouped training requires four training cases")
    return training, sorted(heldout)


def action_chunks(trajectories: dict, cases: list[str], size: int):
    """Preserve source chronology, and pad only within each case/robot."""
    if isinstance(size, bool) or not isinstance(size, int) or size < 1:
        raise ValueError("chunk size must be a positive integer")
    rows, targets, padding = [], [], []
    for case in cases:
        sequence = trajectories[case]
        for index, row in enumerate(sequence):
            rows.append(row)
            targets.append([[sequence[min(index + k, len(sequence) - 1)]["forward"] / .15,
                             float(sequence[min(index + k, len(sequence) - 1)]["stop"])]
                            for k in range(size)])
            padding.append([index + k >= len(sequence) for k in range(size)])
    return rows, targets, padding


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
