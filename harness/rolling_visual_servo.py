"""Fail-closed evidence and time bounds for an opt-in moving RGB approach.

This module receives only RGB-derived skill state and issued command metadata.
It does not read simulation pose, joints, contacts, or referee results.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from collections.abc import Mapping
from typing import Any

from harness.markerless_box import _PROVENANCE as GROUND_BOX_PROVENANCE


FINAL_ENTRY_HANDOFF_M = 0.22
NEAR_MOTION_REOBSERVE_BUFFER_M = 0.03
FINAL_ENTRY_SETTLE_S = 0.2
RGB_TTL_S = 0.6
MAX_LEASE_S = 0.25


def _sha256(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def approach_drive_support(*, action: Mapping[str, Any], box: Mapping[str, Any] | None,
                           target: tuple[float, ...] | None, cargo_id: str, frame_id: int,
                           observed_at_s: float, decision_at_s: float,
                           capture_started_at_s: float, own_sha256: str,
                           top_sha256: str, paired_top_sha256: str,
                           prior_frame_id: int | None = None,
                           prior_observed_at_s: float | None = None,
                           prior_own_sha256: str | None = None) -> dict[str, Any]:
    """Require a distinct paired RGB sample and a directly seen cyan target.

    A rejection is a stop/reobserve or final-entry handoff, never a reason to
    extend a previous motor lease. The range is an own-RGB estimate, not GT.
    """
    if action.get("kind") != "drive":
        return {"allowed": False, "reason": "not_approach_drive"}
    values = (observed_at_s, decision_at_s, capture_started_at_s)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return {"allowed": False, "reason": "invalid_rgb_time"}
    if (observed_at_s < capture_started_at_s - 1e-6
            or observed_at_s > decision_at_s + 1e-6
            or decision_at_s - observed_at_s >= RGB_TTL_S):
        return {"allowed": False, "reason": "stale_or_noncausal_rgb"}
    if (isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id <= 0
            or not _sha256(own_sha256) or not _sha256(top_sha256)
            or top_sha256 != paired_top_sha256):
        return {"allowed": False, "reason": "unpaired_rgb_identity"}
    if prior_frame_id is not None and (frame_id <= prior_frame_id
                                      or prior_observed_at_s is None
                                      or observed_at_s <= prior_observed_at_s
                                      or own_sha256 == prior_own_sha256):
        return {"allowed": False, "reason": "reused_rgb_evidence"}
    if not isinstance(box, Mapping) or box.get("visible") is not True:
        return {"allowed": False, "reason": "no_direct_cyan_rgb"}
    confidence = box.get("confidence")
    if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
            or not math.isfinite(confidence) or confidence < 0.75
            or box.get("target_id") != cargo_id
            or box.get("reason") not in {
                "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
                "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED"}
            or box.get("ambiguity_reason") is not None
            or box.get("identity_source") != "task_catalog_reference_only_not_visually_decoded"
            or box.get("provenance") not in {
                GROUND_BOX_PROVENANCE,
                GROUND_BOX_PROVENANCE + "+measured_full_top_rectangle"}):
        return {"allowed": False, "reason": "weak_direct_cyan_rgb"}
    if (not isinstance(target, (tuple, list)) or len(target) < 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in target[:2])):
        return {"allowed": False, "reason": "invalid_rgb_target"}
    radial_m = math.hypot(*target[:2])
    if radial_m <= FINAL_ENTRY_HANDOFF_M:
        return {"allowed": False, "reason": "final_entry_handoff", "radial_m": radial_m}
    duration = action.get("duration")
    if (isinstance(duration, bool) or not isinstance(duration, (int, float))
            or not math.isfinite(duration) or not 0 < duration <= 1.0):
        return {"allowed": False, "reason": "invalid_original_motion_horizon"}
    return {"allowed": True, "reason": "fresh_direct_cyan_rgb",
            "radial_m": radial_m, "confidence": float(confidence),
            "source_frame_id": frame_id, "source_observed_at_s": float(observed_at_s),
            "source_own_sha256": own_sha256, "source_top_sha256": top_sha256,
            "original_duration_s": float(duration)}


@dataclass
class RollingApproachLease:
    """One RGB decision's absolute motor horizon; never reset by renewal."""

    action: dict[str, Any]
    evidence: dict[str, Any]
    decision_at_s: float
    first_issued_at_s: float
    plan_hash: str
    valid_until_s: float
    motor_commands: tuple[float, ...]
    permission_requests: tuple[tuple[str, str], ...]
    issued_windows: list[tuple[float, float]] = field(default_factory=list)
    hold_at_s: float | None = None

    @property
    def horizon_s(self) -> float:
        return self.first_issued_at_s + self.evidence["original_duration_s"]

    @property
    def rgb_deadline_s(self) -> float:
        return self.evidence["source_observed_at_s"] + RGB_TTL_S

    def remaining_s(self, now: float) -> float:
        if not math.isfinite(now):
            return 0.0
        remaining=min(self.horizon_s, self.rgb_deadline_s) - now
        return remaining if remaining > 1e-9 else 0.0

    def next_duration_s(self, now: float) -> float:
        return min(MAX_LEASE_S, self.remaining_s(now))

    def frame_within_issued_command_window(self, observed_at_s: float) -> bool:
        return any(start - 1e-9 <= observed_at_s < min(end, self.hold_at_s or end) - 1e-9
                   for start, end in self.issued_windows)

    def last_command_end_s(self) -> float:
        end=max((end for _, end in self.issued_windows), default=self.first_issued_at_s)
        return min(end, self.hold_at_s) if self.hold_at_s is not None else end

    def issued_forward_command_integral_m(self, observed_at_s: float, now: float) -> float:
        """Issued forward speed × lease time since RGB, not measured travel."""
        forward=float(self.action.get("forward", 0.0))
        if forward <= 0 or now <= observed_at_s:
            return 0.0
        cap=min(now, self.hold_at_s) if self.hold_at_s is not None else now
        spans=sorted((max(start, observed_at_s), min(end, cap))
                     for start,end in self.issued_windows)
        total=0.0
        last_end=None
        for start,end in spans:
            if end <= start:
                continue
            effective_start=max(start, last_end) if last_end is not None else start
            total+=max(0.0, end-effective_start)
            last_end=max(end, last_end) if last_end is not None else end
        return forward*total


def guard_near_entry_from_issued_commands(support: dict[str, Any],
                                          previous: RollingApproachLease | None,
                                          observed_at_s: float,
                                          decision_at_s: float) -> dict[str, Any]:
    """Require a settled new RGB view before leaving rolling mode near cargo.

    The command integral only determines when to request another image. It is
    not a bound on actual displacement or proof that the robot stopped.
    """
    result=dict(support)
    issued=(previous.issued_forward_command_integral_m(observed_at_s,decision_at_s)
            if previous else 0.0)
    settle_until=(previous.last_command_end_s()+FINAL_ENTRY_SETTLE_S
                  if previous else None)
    radial=result.get("radial_m")
    near=(isinstance(radial,(int,float)) and
          radial<=FINAL_ENTRY_HANDOFF_M+NEAR_MOTION_REOBSERVE_BUFFER_M+issued)
    if previous and near:
        if observed_at_s < settle_until-1e-9:
            result.update(allowed=False,
                          reason="moving_rgb_near_entry_requires_settled_reobservation")
        elif result.get("allowed"):
            result.update(allowed=False,reason="final_entry_handoff")
    result.update(post_capture_issued_forward_integral_m=issued,
                  issued_integral_is_not_measured_travel=True,
                  previous_command_settle_until_s=settle_until)
    return result
