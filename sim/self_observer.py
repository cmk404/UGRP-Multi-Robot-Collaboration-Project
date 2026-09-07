from __future__ import annotations

"""Privileged SIM-only observer for automatic simulator debugging.

This module is deliberately outside the actor/perception contract.  It may use
MuJoCo ground truth because its job is to audit the simulator itself, not to
help the robot complete a task.  The bridge persists its output separately and
never injects it into planner WorldState.
"""

import math
from typing import Any

COLORS = ("red", "blue", "yellow")
# Public pick owns only the final, bounded near-field chassis correction.  A
# larger translation means the coarse approach handoff was not actually in the
# arm-reachable corridor and must not be blessed as a clean pick.
PICK_MAX_BASE_SHIFT_M = 0.20


def _xyz(state: dict, key: str) -> tuple[float, float, float] | None:
    value = state.get(key)
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        return float(value[0]), float(value[1]), float(value[2])
    except (TypeError, ValueError):
        return None


def _dist(a, b) -> float | None:
    if a is None or b is None:
        return None
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def diagnose_action(
    *,
    action: str,
    params: dict[str, Any] | None,
    before: dict[str, Any],
    after: dict[str, Any],
    result_ok: bool,
    result_reason: str = "",
    before_vision: dict[str, Any] | None = None,
    after_vision: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return deterministic, actionable anomalies for one simulated action."""
    params = dict(params or {})
    action = str(action or "").lower()
    target = str(params.get("target_color") or before.get("grasp_color") or "red")
    dest = str(params.get("destination_color") or "")
    issues: list[dict[str, Any]] = []

    def issue(code: str, severity: str, summary: str, **evidence: Any) -> None:
        issues.append({"code": code, "severity": severity, "summary": summary, "evidence": evidence})

    before_base = _xyz(before, "base_xyz")
    after_base = _xyz(after, "base_xyz")
    base_shift = _dist(before_base, after_base)
    before_yaw = float(before.get("base_yaw") or 0.0)
    after_yaw = float(after.get("base_yaw") or 0.0)
    yaw_change = abs(_angle_delta(before_yaw, after_yaw))

    metrics: dict[str, Any] = {
        "base_shift_m": None if base_shift is None else round(base_shift, 4),
        "base_yaw_change_deg": round(math.degrees(yaw_change), 2),
        "result_ok": bool(result_ok),
    }

    # Global physics sanity. These are simulator-debug assertions, not robot evidence.
    rpy = after.get("base_rpy")
    if isinstance(rpy, (list, tuple)) and len(rpy) >= 2:
        tilt_deg = math.degrees(math.hypot(float(rpy[0]), float(rpy[1])))
        metrics["base_tilt_deg"] = round(tilt_deg, 2)
        if tilt_deg > 12.0:
            issue("ROBOT_EXCESSIVE_TILT", "critical", "Robot chassis tilted far beyond normal floor operation.", tilt_deg=round(tilt_deg, 2))
    if after_base is not None and (after_base[2] < 0.015 or after_base[2] > 0.12):
        issue("BASE_HEIGHT_IMPLAUSIBLE", "critical", "Robot base height is physically implausible.", base_z_m=round(after_base[2], 4))

    # Record object displacements for every command and detect destructive side effects.
    object_moves: dict[str, float] = {}
    for color in COLORS:
        d = _dist(_xyz(before, f"{color}_xyz"), _xyz(after, f"{color}_xyz"))
        if d is not None:
            object_moves[color] = round(d, 4)
    metrics["object_displacement_m"] = object_moves

    if action in {"search", "track", "observe_scene", "scan_world", "map_red", "map_blue", "map_yellow"}:
        if base_shift is not None and base_shift > 0.04:
            issue("PERCEPTION_ACTION_MOVED_CHASSIS", "warning", "A perception/localization action translated the chassis substantially.", shift_m=round(base_shift, 4))
        if action in {"track", "observe_scene"} and yaw_change > math.radians(12):
            issue("LOCAL_OBSERVATION_ROTATED_CHASSIS", "warning", "A local observation/track action rotated the chassis substantially.", yaw_deg=round(math.degrees(yaw_change), 2))

    if action == "approach":
        reason_lower = str(result_reason or "").lower()
        if not result_ok and "body alignment exceeded" in reason_lower:
            issue(
                "BODY_ALIGNMENT_BUDGET_EXHAUSTED",
                "warning",
                "One-direction body alignment was still converging when its bounded turn budget expired.",
                yaw_change_deg=round(math.degrees(yaw_change), 2),
            )
        if not result_ok and "could not hold the red block at image centre" in reason_lower:
            issue(
                "GAZE_RECENTER_BUDGET_EXHAUSTED",
                "warning",
                "Target recovery/recentering consumed the bounded image-centering frame budget.",
                yaw_change_deg=round(math.degrees(yaw_change), 2),
            )
        tb = _xyz(before, f"{target}_xyz")
        ta = _xyz(after, f"{target}_xyz")
        db = _dist(before_base, tb)
        da = _dist(after_base, ta)
        if db is not None and da is not None:
            metrics["target_base_distance_before_m"] = round(db, 4)
            metrics["target_base_distance_after_m"] = round(da, 4)
            if result_ok and da > db + 0.015:
                issue("APPROACH_MOVED_AWAY", "critical", "Approach reported success although the robot ended farther from the target.", before_m=round(db, 4), after_m=round(da, 4))
        moved = object_moves.get(target, 0.0)
        if moved > 0.035:
            issue("APPROACH_PUSHED_TARGET", "warning", "Approach displaced the target block before grasping it.", target=target, displacement_m=moved)

    if action in {"pick", "pick_red", "pick_blue", "pick_yellow"}:
        reason_lower = str(result_reason or "").lower()
        if not result_ok and "precision pick plan is missing" in reason_lower:
            issue(
                "PRECISION_PICK_HANDOFF_MISSING",
                "warning",
                "Pick was requested without a valid pregrasp handoff from a successful approach.",
                reason=str(result_reason or ""),
            )
        if not result_ok and (
            "valid block-face samples" in reason_lower
            or "valid arm-face range samples" in reason_lower
        ):
            issue(
                "FACE_GEOMETRY_UNAVAILABLE",
                "warning",
                "Pick could not obtain enough valid near-field visual face/range geometry samples.",
                reason=str(result_reason or ""),
            )
        if not result_ok and "could not bring red block into calibrated visual capture window" in reason_lower:
            issue(
                "CAPTURE_WINDOW_BUDGET_EXHAUSTED",
                "warning",
                "Pick close-view image servo exhausted its bounded creep budget before entering the calibrated capture window.",
                reason=str(result_reason or ""),
            )
        if not result_ok and "arm-relative face error did not improve" in reason_lower:
            issue(
                "ARM_FACE_REPOSITION_STALLED",
                "warning",
                "Pick face placement made insufficient visual progress across bounded base repositions.",
                reason=str(result_reason or ""),
            )
        if not result_ok and "approach exceeded" in reason_lower and "motion pulses" in reason_lower:
            issue(
                "PICK_NEARFIELD_MOTION_BUDGET_EXHAUSTED",
                "warning",
                "Pick remained outside the measured near-field target band after its bounded chassis pulses.",
                reason=str(result_reason or ""),
            )
        if not result_ok and (
            "target disappeared" in reason_lower
            or "target lost" in reason_lower
            or "locked block not found" in reason_lower
        ):
            issue(
                "PICK_TARGET_LOST_NEAR_FIELD",
                "warning",
                "Pick lost the target during near-field alignment before a verified grasp.",
                reason=str(result_reason or ""),
            )
        held = after.get("grasp_color")
        stable = bool(after.get("stable"))
        bilateral = bool(after.get("bilateral_contact"))
        lifted = bool(after.get("lifted"))
        metrics.update({"held_color": held, "stable": stable, "bilateral_contact": bilateral, "lifted": lifted})
        metrics["pick_base_shift_limit_m"] = PICK_MAX_BASE_SHIFT_M
        if base_shift is not None and base_shift > PICK_MAX_BASE_SHIFT_M:
            issue(
                "PICK_BASE_MOTION_CONTRACT_EXCEEDED",
                "critical",
                "Pick translated the chassis beyond its bounded near-field correction contract.",
                shift_m=round(base_shift, 4),
                limit_m=PICK_MAX_BASE_SHIFT_M,
            )
        if result_ok and not (held == target and stable and bilateral and lifted):
            issue("PICK_FALSE_SUCCESS", "critical", "Pick reported success without a stable bilateral lifted grasp in MuJoCo truth.", target=target, held=held, stable=stable, bilateral=bilateral, lifted=lifted)
        if not result_ok and held == target and stable:
            issue("PICK_FALSE_FAILURE", "critical", "Pick reported failure although MuJoCo truth shows a stable grasp.", target=target)
        moved = object_moves.get(target, 0.0)
        if not result_ok and moved > 0.045:
            issue("DESTRUCTIVE_GRASP_MISS", "warning", "Failed pick substantially displaced the target block, changing the retry initial condition.", target=target, displacement_m=moved)
        elif not result_ok and held != target and not lifted and moved > 0.006:
            issue(
                "GRASP_MISS_DISPLACED_TARGET",
                "warning",
                "Failed pick contacted/displaced the target but did not retain a lifted grasp.",
                target=target,
                displacement_m=moved,
                bilateral_at_end=bilateral,
            )
        if bool(after.get("left_contact")) != bool(after.get("right_contact")):
            issue("ASYMMETRIC_FINGER_CONTACT", "info", "Pick ended with one-sided finger contact.", left=bool(after.get("left_contact")), right=bool(after.get("right_contact")))

    delivery_actions = {"search_destination", "place", "place_on_blue", "place_on_yellow", "place_on_red"}
    if action in delivery_actions:
        if not dest and action.startswith("place_on_"):
            dest = action.removeprefix("place_on_")
        target_after = _xyz(after, f"{target}_xyz")
        dest_after = _xyz(after, f"{dest}_xyz") if dest in COLORS else None
        separation = _dist(target_after, dest_after)
        if separation is not None:
            metrics["target_destination_center_distance_m"] = round(separation, 4)
        still_held = after.get("grasp_color") == target
        started_held = before.get("grasp_color") == target and bool(before.get("stable"))
        try:
            grip_error_m = float(after.get("grip_error_m"))
        except (TypeError, ValueError):
            grip_error_m = None
        physically_separated = grip_error_m is not None and grip_error_m > 0.040
        if (
            not result_ok
            and started_held
            and still_held
            and not physically_separated
            and bool(after.get("lifted"))
            and not bool(after.get("bilateral_contact"))
        ):
            issue(
                "CARRY_CONTACT_TRANSIENT_AT_PLACE_ABORT", "warning",
                "Delivery aborted while the target remained within the gripper envelope but bilateral contact was transiently absent.",
                target=target, destination=dest, grip_error_m=grip_error_m,
            )
        reason_lower = str(result_reason or "").lower()
        if (
            action == "search_destination"
            and not result_ok
            and before.get("grasp_color") != target
            and "not physically confirmed before destination search" in reason_lower
        ):
            issue(
                "DESTINATION_SEARCH_AFTER_CARRIED_OBJECT_LOSS", "warning",
                "Destination search was attempted after the carry state had already been lost.",
                target=target,
                destination=dest,
                before_grasp_color=before.get("grasp_color"),
                after_grasp_color=after.get("grasp_color"),
            )
        if started_held and not result_ok and (not still_held or physically_separated):
            code = (
                "CARRIED_OBJECT_DROPPED_DURING_DESTINATION_SEARCH"
                if action == "search_destination"
                else "CARRIED_OBJECT_DROPPED_DURING_PLACE"
            )
            summary = (
                "Destination search physically separated the carried block from the gripper before visual acquisition."
                if action == "search_destination"
                else "Placement physically lost the carried block before a verified release."
            )
            issue(
                code, "critical", summary, target=target, destination=dest,
                final_z_m=None if target_after is None else round(target_after[2], 4),
                grip_error_m=grip_error_m, logical_grasp_color=after.get("grasp_color"),
            )
        if action != "search_destination" and result_ok and still_held:
            issue("PLACE_FALSE_SUCCESS_STILL_HELD", "critical", "Place reported success while the target remains logically/physically held.", target=target, destination=dest)
        if action != "search_destination" and result_ok and separation is not None and separation > 0.09:
            issue("PLACE_FALSE_SUCCESS_FAR_FROM_DESTINATION", "critical", "Place reported success but target and destination remain far apart.", separation_m=round(separation, 4), destination=dest)

        before_dest_vision = (before_vision or {}).get(dest, {}) if dest else {}
        after_dest_vision = (after_vision or {}).get(dest, {}) if dest else {}
        destination_was_visible = bool(before_dest_vision.get("visible"))
        destination_visible_after = bool(after_dest_vision.get("visible"))
        destination_move = object_moves.get(dest) if dest else None
        if (
            action != "search_destination"
            and not result_ok
            and "target lost while body-centering" in reason_lower
            and started_held
            and still_held
            and bool(after.get("lifted"))
            and bool(after.get("bilateral_contact"))
            and destination_was_visible
            and not destination_visible_after
            and (destination_move is None or destination_move <= 0.005)
        ):
            issue(
                "PLACE_DESTINATION_VISIBILITY_LOST_DURING_ALIGNMENT",
                "warning",
                "Placement lost the visual destination during body alignment while the carried block remained securely held and the destination stayed physically stationary.",
                target=target,
                destination=dest,
                destination_displacement_m=destination_move,
                carried_bilateral_contact=True,
                destination_visible_before=True,
                destination_visible_after=False,
            )

    # A missing remote runtime module means the command failed before reaching
    # MuJoCo physics. Classify this infrastructure fault explicitly so a frozen
    # UI is not misdiagnosed as a controller/physics anomaly.
    reason_lower = str(result_reason or "").lower()
    if result_ok is False and not issues and (
        "no module named" in reason_lower
        or "modulenotfounderror" in reason_lower
        or "importerror" in reason_lower
    ):
        issue(
            "SIM_WORKER_RUNTIME_DEPENDENCY_MISSING",
            "critical",
            "Remote SIM worker could not load a required runtime module; the action never reached physics.",
            reason=str(result_reason or ""),
        )

    if result_ok is False and not issues:
        issue("ACTION_FAILED_UNCLASSIFIED", "warning", "Action failed but the deterministic observer has not yet classified the physical cause.", reason=str(result_reason or ""))

    rank = {"info": 1, "warning": 2, "critical": 3}
    worst = max((rank.get(x["severity"], 0) for x in issues), default=0)
    status = {0: "CLEAN", 1: "NOTICE", 2: "SUSPICIOUS", 3: "BROKEN"}[worst]
    return {
        "schema": "ugrp.sim.self_observer.v1",
        "status": status,
        "action": action,
        "target_color": target,
        "destination_color": dest or None,
        "result_reason": str(result_reason or ""),
        "metrics": metrics,
        "issues": issues,
        "vision_before": before_vision or {},
        "vision_after": after_vision or {},
        "privileged_sim_debug_only": True,
    }
