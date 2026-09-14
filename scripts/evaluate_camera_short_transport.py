#!/usr/bin/env python3
"""Pure sampled-output evaluator for camera-only short transport trials."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

ROBOTS = ("r1", "r3")
REQUIRED_PHASES = (
    "grasp_hold", "carry", "carry_stop", "place_lower", "place_open",
    "place_retract", "release_hold",
)
MAX_SAMPLE_GAP_S = 0.15


def _finite(value: Any) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def _position(sample: Mapping[str, Any]) -> tuple[float, float, float] | None:
    value = sample.get("position_m")
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or len(value) != 3 or not all(_finite(item) for item in value)):
        return None
    return tuple(float(item) for item in value)


def _weld_off(sample: Mapping[str, Any]) -> bool:
    value = sample.get("constraints_active")
    return (isinstance(value, Mapping) and set(value) == set(ROBOTS)
            and all(value[rid] is False for rid in ROBOTS))


def _bilateral(sample: Mapping[str, Any]) -> bool:
    contacts = sample.get("contacts")
    return (isinstance(contacts, Mapping) and set(contacts) == set(ROBOTS)
            and all(isinstance(contacts[rid], Mapping)
                    and contacts[rid].get("bilateral") is True for rid in ROBOTS))


def _finger_clear(sample: Mapping[str, Any]) -> bool:
    contacts = sample.get("contacts")
    if not isinstance(contacts, Mapping) or set(contacts) != set(ROBOTS):
        return False
    for rid in ROBOTS:
        contact = contacts[rid]
        if not isinstance(contact, Mapping):
            return False
        if not all(contact.get(key) is False for key in ("left", "right", "bilateral")):
            return False
    return True


def _continuous(rows: Sequence[Mapping[str, Any]]) -> bool:
    return bool(rows) and all(
        0.0 < float(b["sim_time_s"]) - float(a["sim_time_s"]) <= MAX_SAMPLE_GAP_S + 1e-12
        for a, b in zip(rows, rows[1:])
    )


def _duration(rows: Sequence[Mapping[str, Any]]) -> float | None:
    if not rows:
        return None
    return float(rows[-1]["sim_time_s"]) - float(rows[0]["sim_time_s"])


def _forms_block(samples: Sequence[Mapping[str, Any]], phases: set[str]) -> bool:
    indexes = [index for index, row in enumerate(samples) if row["phase"] in phases]
    return bool(indexes) and indexes == list(range(indexes[0], indexes[-1] + 1))


def _span_3d(rows: Sequence[Mapping[str, Any]]) -> float | None:
    positions = [_position(row) for row in rows]
    if not positions or any(position is None for position in positions):
        return None
    return max(
        math.dist(a, b) for index, a in enumerate(positions) for b in positions[index:]
    )


def evaluate_transport_samples(
    samples: list[dict], *, target_distance_m: float = .20,
    tolerance_m: float = .03, minimum_hold_s: float = 2.0,
    release_stable_s: float = 1.0,
) -> dict[str, Any]:
    """Evaluate recorded referee samples without consulting runtime state/actions."""
    gates = {
        "valid_samples": False, "required_phases": False, "weld_off": False,
        "grasp_hold": False, "carry_continuity": False, "carry_progress": False,
        "transport_endpoint": False, "lateral_drift": False,
        "release_continuity": False, "release_grounded_clear": False,
        "release_stable": False,
    }
    metrics: dict[str, Any] = {
        "target_distance_m": target_distance_m, "tolerance_m": tolerance_m,
        "minimum_hold_s": minimum_hold_s, "release_stable_s": release_stable_s,
        "max_sample_gap_s": MAX_SAMPLE_GAP_S, "sample_count": len(samples) if isinstance(samples, list) else None,
    }
    reasons: list[str] = []

    if not all(_finite(value) and float(value) >= 0.0 for value in
               (target_distance_m, tolerance_m, minimum_hold_s, release_stable_s)):
        reasons.append("configuration values must be finite and non-negative")
        return {"success": False, "gates": gates, "reasons": reasons, "metrics": metrics,
                "scope": "sampled referee continuity only"}
    if not isinstance(samples, list) or not samples:
        reasons.append("samples must be a non-empty list")
        return {"success": False, "gates": gates, "reasons": reasons, "metrics": metrics,
                "scope": "sampled referee continuity only"}

    valid = True
    previous_time = None
    for index, sample in enumerate(samples):
        if (not isinstance(sample, Mapping) or not isinstance(sample.get("phase"), str)
                or not _finite(sample.get("sim_time_s")) or _position(sample) is None
                or not _finite(sample.get("height_above_start_m"))
                or not _weld_off(sample)):
            valid = False
            reasons.append(f"sample {index} has missing, non-finite, or invalid core fields")
            break
        now = float(sample["sim_time_s"])
        if previous_time is not None and now <= previous_time:
            valid = False
            reasons.append(f"sample {index} time is not strictly increasing")
            break
        previous_time = now
    gates["valid_samples"] = valid
    gates["weld_off"] = valid and all(_weld_off(sample) for sample in samples)
    if not valid:
        return {"success": False, "gates": gates, "reasons": reasons, "metrics": metrics,
                "scope": "sampled referee continuity only"}

    by_phase = {phase: [sample for sample in samples if sample["phase"] == phase]
                for phase in REQUIRED_PHASES}
    missing = [phase for phase, rows in by_phase.items() if not rows]
    gates["required_phases"] = not missing
    if missing:
        reasons.append("missing required phases: " + ", ".join(missing))

    grasp = by_phase["grasp_hold"]
    grasp_duration = _duration(grasp)
    metrics["grasp_hold_duration_s"] = grasp_duration
    gates["grasp_hold"] = bool(
        grasp and _forms_block(samples, {"grasp_hold"}) and _continuous(grasp)
        and grasp_duration is not None
        and grasp_duration + 1e-12 >= minimum_hold_s
        and all(float(row["height_above_start_m"]) >= .03 and _bilateral(row)
                and _weld_off(row) for row in grasp)
    )
    if not gates["grasp_hold"]:
        reasons.append("grasp_hold is not continuous, long enough, lifted, bilateral, and weld-free")

    carry = [sample for sample in samples if sample["phase"] in {"carry", "carry_stop"}]
    gates["carry_continuity"] = bool(
        by_phase["carry"] and by_phase["carry_stop"]
        and _forms_block(samples, {"carry", "carry_stop"}) and _continuous(carry)
        and all(float(row["height_above_start_m"]) >= .03 and _bilateral(row)
                and _weld_off(row) for row in carry)
    )
    if not gates["carry_continuity"]:
        reasons.append("carry samples are missing, discontinuous, unlifted, non-bilateral, or welded")
    carry_progress = None
    if carry:
        carry_progress = _position(carry[-1])[0] - _position(carry[0])[0]
    metrics["carry_start_to_end_progress_m"] = carry_progress
    gates["carry_progress"] = carry_progress is not None and carry_progress + 1e-12 >= .15
    if not gates["carry_progress"]:
        reasons.append("carry start-to-end progress is below 0.15 m")

    displacement = lateral = None
    if grasp and by_phase["release_hold"]:
        origin, final = _position(grasp[-1]), _position(by_phase["release_hold"][-1])
        displacement = final[0] - origin[0]
        lateral = abs(final[1] - origin[1])
    metrics["final_displacement_m"] = displacement
    metrics["displacement_error_m"] = None if displacement is None else displacement - target_distance_m
    metrics["lateral_drift_m"] = lateral
    gates["transport_endpoint"] = (displacement is not None
        and abs(displacement - target_distance_m) <= tolerance_m + 1e-12)
    gates["lateral_drift"] = lateral is not None and lateral <= .03 + 1e-12
    if not gates["transport_endpoint"]:
        reasons.append("final displacement is outside target tolerance")
    if not gates["lateral_drift"]:
        reasons.append("final lateral drift exceeds 0.03 m")

    release = by_phase["release_hold"]
    release_duration = _duration(release)
    metrics["release_hold_duration_s"] = release_duration
    gates["release_continuity"] = bool(
        release and samples[-len(release):] == release and _continuous(release)
        and release_duration is not None and release_duration + 1e-12 >= release_stable_s
    )
    if not gates["release_continuity"]:
        reasons.append("final release_hold is not contiguous and long enough")
    gates["release_grounded_clear"] = bool(release) and all(
        row.get("payload_floor_contact") is True
        and row.get("payload_robot_contact") is False
        and _finger_clear(row) and _weld_off(row) for row in release
    )
    if not gates["release_grounded_clear"]:
        reasons.append("release_hold is not grounded and clear of robots/fingers")
    release_span = _span_3d(release)
    metrics["release_position_span_3d_m"] = release_span
    gates["release_stable"] = release_span is not None and release_span <= .005 + 1e-12
    if not gates["release_stable"]:
        reasons.append("release position span exceeds 0.005 m")

    success = all(gates.values())
    return {"success": success, "gates": gates, "reasons": reasons, "metrics": metrics,
            "scope": "sampled referee continuity only"}
