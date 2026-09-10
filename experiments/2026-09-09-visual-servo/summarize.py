#!/usr/bin/env python3
"""Summarize completed camera visual-servo artifacts without replay or model calls."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any


LOCAL_MODEL_REASON = "local current-image model predicts reduced alignment error"


def _raw_response(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, str):
        return None
    text = value.strip()
    if text.startswith("```") and text.endswith("```"):
        lines = text.splitlines()
        text = "\n".join(lines[1:-1]).strip()
    try:
        parsed = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return None
    return parsed if isinstance(parsed, dict) else None


def _trusted_goal_error(observation: Any) -> float | None:
    if not isinstance(observation, dict):
        return None
    view = observation.get("view")
    if view not in {"own", "overhead"}:
        return None
    confidence = observation.get("confidence")
    identity = observation.get("identity_confidence")
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or confidence < 0.7:
        return None
    if view == "overhead" and (
        isinstance(identity, bool) or not isinstance(identity, (int, float)) or identity < 0.8
    ):
        return None
    jaws, target = observation.get("jaws"), observation.get("target")
    if not isinstance(jaws, list) or len(jaws) != 2 or not isinstance(target, list) or len(target) != 2:
        return None
    try:
        points = [[float(v) for v in point] for point in (jaws[0], jaws[1], target)]
    except (TypeError, ValueError):
        return None
    if any(len(point) != 2 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in point) for point in points):
        return None
    midpoint = ((points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2)
    return math.hypot(points[2][0] - midpoint[0], points[2][1] - midpoint[1])


def _output_evaluation(root: Path, report: dict[str, Any]) -> dict[str, Any] | None:
    path = root / "evaluation-only.jsonl"
    if not path.is_file():
        return None
    samples = []
    for line in path.read_text().splitlines():
        if line.strip():
            value = json.loads(line)
            if isinstance(value, dict):
                samples.append(value)
    if not samples:
        return {"source": "output-only referee", "sample_count": 0}
    robot_ids = sorted({rid for sample in samples for rid in sample.get("bases", {})})
    displacement = {}
    contacts = {}
    for rid in robot_ids:
        first = samples[0].get("bases", {}).get(rid, {}).get("xyz")
        last = samples[-1].get("bases", {}).get(rid, {}).get("xyz")
        if isinstance(first, list) and isinstance(last, list) and len(first) >= 2 and len(last) >= 2:
            displacement[rid] = math.hypot(float(last[0]) - float(first[0]), float(last[1]) - float(first[1]))
        contacts[rid] = {
            key: sum(bool(sample.get("contacts", {}).get(rid, {}).get(key)) for sample in samples)
            for key in ("left", "right", "bilateral")
        }
    return {
        "source": "output-only referee; excluded from controller/input metrics",
        "sample_count": len(samples),
        "planar_base_displacement_m_first_to_last": displacement,
        "contact_sample_counts": contacts,
        "reported_grasp_success": report.get("grasp_success"),
        "reported_longest_qualifying_duration_s": report.get("longest_qualifying_duration_s"),
        "reported_max_lift_m": report.get("max_lift_m"),
    }


def summarize(root: Path, include_evaluation: bool) -> dict[str, Any]:
    result_path = root / "result.json"
    report = json.loads(result_path.read_text())
    calls = report.get("calls", [])
    if not isinstance(calls, list):
        raise ValueError("calls must be a list")

    stages = Counter(str(call.get("decision", {}).get("stage", "missing")) for call in calls)
    close_commands = sum(
        call.get("action", {}).get("kind") == "arm"
        and call.get("action", {}).get("servo_id") == 1
        and call.get("action", {}).get("pulse") == 1500
        for call in calls
    )
    trusted = sum(_trusted_goal_error(call.get("observation")) is not None for call in calls)
    token_sums = {
        key: sum(
            value for call in calls
            if isinstance((value := call.get("usage", {}).get(key)), (int, float)) and not isinstance(value, bool)
        )
        for key in ("prompt_tokens", "completion_tokens", "total_tokens")
    }

    raw_high = consistent = rejected = 0
    for call in calls:
        raw = _raw_response(call.get("response"))
        observation = call.get("observation", {})
        if not raw or raw.get("view") != "overhead":
            continue
        raw_identity = raw.get("identity_confidence")
        if isinstance(raw_identity, bool) or not isinstance(raw_identity, (int, float)) or raw_identity < 0.8:
            continue
        raw_high += 1
        motion = observation.get("motion_identity", {})
        if motion.get("consistent") is True and observation.get("identity_confidence", 0) >= 0.8:
            consistent += 1
        else:
            rejected += 1

    local_indices = [
        index for index, call in enumerate(calls)
        if call.get("decision", {}).get("reason") == LOCAL_MODEL_REASON
    ]
    comparisons = []
    for index in local_indices:
        current = calls[index]
        next_call = next(
            (candidate for candidate in calls[index + 1:]
             if candidate.get("robot_id") == current.get("robot_id")),
            None,
        )
        if next_call is None:
            continue
        before_obs, after_obs = current.get("observation", {}), next_call.get("observation", {})
        before, after = _trusted_goal_error(before_obs), _trusted_goal_error(after_obs)
        if before is None or after is None or before_obs.get("view") != after_obs.get("view"):
            continue
        comparisons.append({
            "robot_id": current.get("robot_id"),
            "round": current.get("round"),
            "next_round": next_call.get("round"),
            "view": before_obs.get("view"),
            "before_error": before,
            "after_error": after,
            "change": after - before,
        })

    final_learning = {}
    for call in calls:
        learning = call.get("learning") or call.get("decision", {}).get("learning") or {}
        final_learning[str(call.get("robot_id", "unknown"))] = {
            "sample_count": learning.get("sample_count"),
            "channel_sample_counts": learning.get("channel_sample_counts"),
        }
    result = {
        "root": str(root),
        "git_sha": report.get("git_sha"),
        "rounds_completed": report.get("rounds_completed"),
        "call_count": len(calls),
        "error": report.get("error"),
        "token_sums": token_sums,
        "stage_counts": dict(sorted(stages.items())),
        "close_command_count": close_commands,
        "trusted_landmarks": {
            "count": trusted,
            "total": len(calls),
            "fraction": trusted / len(calls) if calls else None,
        },
        "motion_identity_raw_high_confidence": {
            "count": raw_high,
            "consistent_and_accepted": consistent,
            "rejected_or_not_consistent": rejected,
        },
        "final_learning": final_learning,
        "local_model_decision_count": len(local_indices),
        "local_model_valid_next_same_view": {
            "count": len(comparisons),
            "error_decreased": sum(item["change"] < 0 for item in comparisons),
            "error_increased": sum(item["change"] > 0 for item in comparisons),
            "error_unchanged": sum(item["change"] == 0 for item in comparisons),
            "comparisons": comparisons,
        },
    }
    if include_evaluation:
        result["output_only_evaluation"] = _output_evaluation(root, report)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", type=Path, nargs="+", help="completed run directories containing result.json")
    parser.add_argument("--include-evaluation", action="store_true", help="also summarize output-only referee data")
    args = parser.parse_args()
    summaries = [summarize(root.expanduser().resolve(), args.include_evaluation) for root in args.roots]
    print(json.dumps(summaries, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
