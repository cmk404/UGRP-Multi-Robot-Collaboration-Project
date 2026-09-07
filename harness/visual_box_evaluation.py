"""Offline outcome scoring for the single-box camera-only skill demo.

These metrics establish lift, transfer, settling, and absence of simulator
cargo constraints.  They do not prove cooperation, perception causality,
physical grasp force, or that a controller lacked other privileged inputs.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any


MIN_LIFT_M = 0.04
MIN_TRANSFER_M = 0.25
MODES = frozenset({"short_transfer", "destination_zone"})


def evaluate_visual_box_transfer(
    initial_warehouse_state: Mapping[str, Any],
    final_warehouse_state: Mapping[str, Any],
    evaluation_logs: Sequence[Mapping[str, Any]],
    *,
    cargo_id: str | None = None,
    mode: str = "short_transfer",
) -> dict[str, Any]:
    """Score one cargo body from evaluation-only warehouse snapshots.

    Log entries may be warehouse states directly or may contain one under the
    ``warehouse_state`` key.  ``cargo_id`` may be omitted only when exactly one
    cargo entry exists in both endpoint snapshots.
    """
    if mode not in MODES:
        raise ValueError(f"mode must be one of {sorted(MODES)}")
    initial = _snapshot(initial_warehouse_state)
    final = _snapshot(final_warehouse_state)
    resolved_id = _cargo_id(initial, final, cargo_id)
    initial_box = _box(initial, resolved_id)
    final_box = _box(final, resolved_id)

    snapshots = [initial]
    for entry in evaluation_logs:
        snapshots.append(_snapshot(entry))
    snapshots.append(final)
    boxes = [_box(snapshot, resolved_id) for snapshot in snapshots]

    initial_position = _position(initial_box)
    final_position = _position(final_box)
    heights = [_position(box)[2] for box in boxes]
    max_height = max(heights)
    max_lift = max_height - initial_position[2]
    planar_displacement = math.hypot(
        final_position[0] - initial_position[0],
        final_position[1] - initial_position[1],
    )
    final_stable = final_box.get("stable") is True
    constraint_samples = [_active_constraints(box) for box in boxes]
    constraints_during_episode = any(any(sample.values()) for sample in constraint_samples)
    destination_success = (
        _destination_success(final_box) if mode == "destination_zone" else None
    )

    gates = {
        "lift": max_lift >= MIN_LIFT_M,
        "transfer": planar_displacement >= MIN_TRANSFER_M,
        "final_stable": final_stable,
        "constraint_free": not constraints_during_episode,
    }
    if mode == "destination_zone":
        gates["destination_zone"] = bool(destination_success)

    return {
        "success": all(gates.values()),
        "mode": mode,
        "cargo_id": resolved_id,
        "max_box_height_m": max_height,
        "max_lift_above_initial_m": max_lift,
        "planar_displacement_m": planar_displacement,
        "final_stable": final_stable,
        "active_constraints_during_episode": constraints_during_episode,
        "constraint_samples": constraint_samples,
        "destination_evaluation_success": destination_success,
        "gates": gates,
        "thresholds": {
            "minimum_lift_m": MIN_LIFT_M,
            "minimum_planar_displacement_m": MIN_TRANSFER_M,
        },
        "claim_scope": "single_box_lift_transfer_outcome_not_cooperation_proof",
    }


def _snapshot(value: Mapping[str, Any]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("warehouse state/log entry must be an object")
    nested = value.get("warehouse_state")
    snapshot = nested if nested is not None else value
    if not isinstance(snapshot, Mapping) or not isinstance(snapshot.get("cargo"), Mapping):
        raise ValueError("warehouse snapshot must contain a cargo object")
    return snapshot


def _cargo_id(initial: Mapping[str, Any], final: Mapping[str, Any], requested: str | None) -> str:
    if requested is not None:
        result = str(requested)
        _box(initial, result)
        _box(final, result)
        return result
    common = set(initial["cargo"]) & set(final["cargo"])
    if len(common) != 1:
        raise ValueError("cargo_id is required unless exactly one cargo is present")
    return str(next(iter(common)))


def _box(snapshot: Mapping[str, Any], cargo_id: str) -> Mapping[str, Any]:
    box = snapshot["cargo"].get(cargo_id)
    if not isinstance(box, Mapping):
        raise ValueError(f"warehouse snapshot is missing cargo {cargo_id!r}")
    return box


def _position(box: Mapping[str, Any]) -> tuple[float, float, float]:
    value = box.get("position")
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("cargo position must contain x, y, z")
    result = tuple(_finite("cargo position", item) for item in value)
    return result  # type: ignore[return-value]


def _active_constraints(box: Mapping[str, Any]) -> dict[str, bool]:
    value = box.get("constraints_active", {})
    if not isinstance(value, Mapping):
        raise ValueError("constraints_active must be an object")
    result: dict[str, bool] = {}
    for key, active in value.items():
        if not isinstance(active, bool):
            raise ValueError("constraint activity values must be booleans")
        result[str(key)] = active
    return result


def _destination_success(box: Mapping[str, Any]) -> bool:
    evaluation = box.get("evaluation")
    if not isinstance(evaluation, Mapping) or not isinstance(evaluation.get("success"), bool):
        raise ValueError("destination mode requires cargo evaluation.success")
    return bool(evaluation["success"])


def _finite(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} values must be finite numbers")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} values must be finite numbers")
    return result
