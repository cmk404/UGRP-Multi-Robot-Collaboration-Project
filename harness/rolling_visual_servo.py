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
VIEW_RECOVERY_MIN_CONFIDENCE = 0.50
VIEW_RECOVERY_MAX_ANCHOR_AGE_S = 2.0
VIEW_RECOVERY_MAX_ELAPSED_S = 4.0
VIEW_RECOVERY_MAX_POSES = 3
VIEW_RECOVERY_MAX_EPISODES = 3
VIEW_RECOVERY_WRIST_STEP_PWM = 24
VIEW_RECOVERY_MAX_PAN_DELTA_PWM = 45
VIEW_RECOVERY_MIN_RADIAL_M = 0.35


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


def _direct_view(sample: Mapping[str, Any], minimum_confidence: float) -> bool:
    """A direct own-RGB cuboid, with no identity borrowed from a command."""
    box = sample.get("box")
    target = sample.get("target")
    if not isinstance(box, Mapping) or box.get("visible") is not True:
        return False
    confidence = box.get("confidence")
    area = box.get("area_px")
    return bool(
        isinstance(confidence, (int, float)) and not isinstance(confidence, bool)
        and math.isfinite(confidence) and confidence >= minimum_confidence
        and isinstance(area, (int, float)) and not isinstance(area, bool)
        and math.isfinite(area) and area > 0
        and box.get("target_id") == sample.get("cargo_id")
        and box.get("ambiguity_reason") is None
        and box.get("identity_source") == "task_catalog_reference_only_not_visually_decoded"
        and box.get("reason") in {
            "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
            "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED"}
        and box.get("provenance") in {
            GROUND_BOX_PROVENANCE,
            GROUND_BOX_PROVENANCE + "+measured_full_top_rectangle"}
        and isinstance(target, (tuple, list)) and len(target) >= 2
        and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) for v in target[:2])
        and math.hypot(*target[:2]) >= VIEW_RECOVERY_MIN_RADIAL_M)


def _fresh_view(sample: Mapping[str, Any]) -> bool:
    times = [sample.get(k) for k in
             ("capture_started_at_s", "observed_at_s", "decision_at_s")]
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in times):
        return False
    started, observed, decided = times
    return bool(
        started <= observed <= decided and decided-observed < RGB_TTL_S
        and isinstance(sample.get("frame_id"), int)
        and not isinstance(sample["frame_id"], bool) and sample["frame_id"] > 0
        and _sha256(sample.get("own_sha256"))
        and _sha256(sample.get("top_sha256"))
        and sample["top_sha256"] == sample.get("paired_top_sha256")
        and isinstance(sample.get("pose"), Mapping)
        and all(isinstance(sample["pose"].get(str(servo)), int)
                and not isinstance(sample["pose"].get(str(servo)), bool)
                for servo in (3, 6)))


def _same_cargo_view(anchor: Mapping[str, Any], sample: Mapping[str, Any]) -> bool:
    """Compare two actual RGB fits, not their camera servo commands alone."""
    if not _direct_view(sample, VIEW_RECOVERY_MIN_CONFIDENCE):
        return False
    old_box, box = anchor["box"], sample["box"]
    old_target, target = anchor["target"], sample["target"]
    area_ratio = float(box["area_px"]) / float(old_box["area_px"])
    old_bbox, bbox = old_box.get("pixel_bbox"), box.get("pixel_bbox")
    if (not isinstance(old_bbox, (tuple, list)) or not isinstance(bbox, (tuple, list))
            or len(old_bbox) != 4 or len(bbox) != 4
            or any(not isinstance(v, (int, float)) or isinstance(v, bool)
                   or not math.isfinite(v) for v in [*old_bbox, *bbox])
            or min(old_bbox[2:]) <= 0 or min(bbox[2:]) <= 0):
        return False
    old_center = (old_bbox[0]+old_bbox[2]/2, old_bbox[1]+old_bbox[3]/2)
    center = (bbox[0]+bbox[2]/2, bbox[1]+bbox[3]/2)
    old_pose, pose = anchor["pose"], sample["pose"]
    pan_delta = abs(int(pose["6"])-int(old_pose["6"]))
    wrist_delta = abs(int(pose["3"])-int(old_pose["3"]))
    width_ratio, height_ratio = bbox[2]/old_bbox[2], bbox[3]/old_bbox[3]
    return bool(
        old_box.get("reason") == box.get("reason")
        and old_box.get("provenance") == box.get("provenance")
        and 0.65 <= area_ratio <= 1.50
        and 0.75 <= width_ratio <= 1.33
        and 0.75 <= height_ratio <= 1.33
        and abs(center[0]-old_center[0]) <= 12+1.5*pan_delta
        and abs(center[1]-old_center[1]) <= 12+1.5*wrist_delta
        and math.dist(old_target[:2], target[:2]) <= 0.06)


@dataclass
class ActiveViewRecovery:
    """Finite active observation after a visible but weak rolling drive view.

    Camera posture is an *issued* completed pose receipt plus the port's issued
    PWM cache; neither is a measured joint position. This class issues no motor
    action and cannot turn a weak view into rolling drive support.
    """

    anchor: dict[str, Any] | None = None
    episode: dict[str, Any] | None = None
    wheel_generation: int = 0
    episodes_started: int = 0
    poses_issued: int = 0

    def note_wheel_issue(self) -> None:
        self.wheel_generation += 1

    def remember(self, sample: Mapping[str, Any], completed_poses: Mapping[str, Any] | None,
                 last_rolling_command_end_s: float | None) -> bool:
        if self.episode is not None or not _fresh_view(sample) or not _direct_view(sample, 0.75):
            return False
        if last_rolling_command_end_s is not None and (
                sample["observed_at_s"] < last_rolling_command_end_s + FINAL_ENTRY_SETTLE_S):
            return False
        if not isinstance(completed_poses, Mapping):
            return False
        for servo in (3, 6):
            receipt = completed_poses.get(str(servo))
            if not isinstance(receipt, Mapping) or receipt.get("status") != "completed":
                return False
            macro = receipt.get("macro")
            if not isinstance(macro, Mapping) or macro.get("kind") != "pose":
                return False
            ended = receipt.get("ended_at")
            if not isinstance(ended, (int, float)) or isinstance(ended, bool) or (
                    not math.isfinite(ended) or ended > sample["capture_started_at_s"]):
                return False
            pulses = macro.get("pulses")
            if (not isinstance(pulses, Mapping)
                    or sample["pose"][str(servo)] != pulses.get(servo, pulses.get(str(servo)))):
                return False
        self.anchor = {"frame_id": sample["frame_id"],
                       "observed_at_s": sample["observed_at_s"],
                       "own_sha256": sample["own_sha256"],
                       "top_sha256": sample["top_sha256"],
                       "box": dict(sample["box"]),
                       "target": tuple(sample["target"]),
                       "pose": dict(sample["pose"]),
                       "plan_hash": sample.get("plan_hash"),
                       "completed_pose_receipts": {key: dict(value)
                            for key, value in completed_poses.items()},
                       "wheel_generation": self.wheel_generation}
        return True

    def _next_pose(self, sample: Mapping[str, Any]) -> dict[str, int] | None:
        anchor = self.anchor
        assert anchor is not None and self.episode is not None
        old, current = anchor["pose"], sample["pose"]
        pan = int(old["6"])
        wrist = int(old["3"])
        delta_pan = abs(pan-int(current["6"]))
        if delta_pan > VIEW_RECOVERY_MAX_PAN_DELTA_PWM:
            return None
        candidates = ([{"6": pan}] if delta_pan else [])
        candidates.extend({"3": wrist+offset} for offset in
                          (-VIEW_RECOVERY_WRIST_STEP_PWM, VIEW_RECOVERY_WRIST_STEP_PWM)
                          if 500 <= wrist+offset <= 2200)
        used = self.episode["attempted_poses"]
        for pose in candidates:
            if pose not in used and all(current[key] != value for key, value in pose.items()):
                return pose
        return None

    def _fail(self, reason: str, sample: Mapping[str, Any]) -> dict[str, Any]:
        event = {"kind": "failed", "reason": reason,
                 "frame_id": sample.get("frame_id"),
                 "own_sha256": sample.get("own_sha256"),
                 "episode": self.episodes_started,
                 "poses_issued": len(self.episode["attempted_poses"]) if self.episode else 0,
                 "wheels_held": True}
        self.episode = None
        return event

    def _issue_view_pose(self, sample: Mapping[str, Any]) -> dict[str, Any]:
        assert self.episode is not None and self.anchor is not None
        if (sample["decision_at_s"]-self.episode["started_at_s"] > VIEW_RECOVERY_MAX_ELAPSED_S
                or len(self.episode["attempted_poses"]) >= VIEW_RECOVERY_MAX_POSES):
            return self._fail("active_view_budget_exhausted", sample)
        pose = self._next_pose(sample)
        if pose is None:
            return self._fail("no_distinct_bounded_issued_pose", sample)
        delta = max(abs(value-int(sample["pose"][key])) for key,value in pose.items())
        estimated_settle_s = max(0.25, delta/600.0) + 0.3
        if (sample["decision_at_s"]+estimated_settle_s
                >= self.episode["started_at_s"]+VIEW_RECOVERY_MAX_ELAPSED_S):
            return self._fail("pose_cannot_settle_within_budget", sample)
        self.episode["attempted_poses"].append(pose)
        self.episode["expected_pose"] = pose
        self.episode["last_view"] = (sample["frame_id"], sample["observed_at_s"],
                                     sample["own_sha256"])
        self.poses_issued += 1
        return {"kind": "pose_issued", "reason": "weak_visible_direct_cyan",
                "episode": self.episodes_started,
                "attempt": len(self.episode["attempted_poses"]),
                "issued_pose": pose, "frame_id": sample["frame_id"],
                "own_sha256": sample["own_sha256"],
                "top_sha256": sample["top_sha256"],
                "box_confidence": sample["box"]["confidence"],
                "anchor_frame_id": self.anchor["frame_id"],
                "anchor_own_sha256": self.anchor["own_sha256"],
                "anchor_completed_pose_receipts": self.anchor["completed_pose_receipts"],
                "anchor_target_xy_m": list(self.anchor["target"][:2]),
                "current_target_xy_m": list(sample["target"][:2]),
                "wheel_generation": self.wheel_generation,
                "wheels_held": True,
                "issued_pwm_not_measured_joint": True}

    def start(self, sample: Mapping[str, Any]) -> dict[str, Any]:
        anchor = self.anchor
        if not _fresh_view(sample):
            return self._fail("stale_or_unpaired_rgb", sample)
        if anchor is None or self.episodes_started >= VIEW_RECOVERY_MAX_EPISODES:
            return self._fail("no_bounded_strong_anchor", sample)
        if (sample["frame_id"] <= anchor["frame_id"]
                or sample["observed_at_s"] <= anchor["observed_at_s"]
                or sample["own_sha256"] == anchor["own_sha256"]):
            return self._fail("no_distinct_current_own_rgb", sample)
        if sample.get("plan_hash") != anchor["plan_hash"]:
            return self._fail("anchor_plan_changed", sample)
        if (sample["observed_at_s"]-anchor["observed_at_s"] > VIEW_RECOVERY_MAX_ANCHOR_AGE_S
                or self.wheel_generation != anchor["wheel_generation"]):
            return self._fail("anchor_stale_or_wheel_issued", sample)
        if not _same_cargo_view(anchor, sample):
            return self._fail("weak_view_identity_unresolved", sample)
        self.episodes_started += 1
        self.episode = {"started_at_s": sample["decision_at_s"],
                        "plan_hash": sample.get("plan_hash"),
                        "attempted_poses": [], "last_view": None,
                        "expected_pose": None}
        return self._issue_view_pose(sample)

    def advance(self, sample: Mapping[str, Any]) -> dict[str, Any] | None:
        episode, anchor = self.episode, self.anchor
        if episode is None:
            return None
        if sample.get("phase") != "approach":
            return self._fail("phase_exit_during_active_view", sample)
        if not _fresh_view(sample):
            return self._fail("stale_or_unpaired_reobservation", sample)
        if sample["decision_at_s"]-episode["started_at_s"] > VIEW_RECOVERY_MAX_ELAPSED_S:
            return self._fail("active_view_budget_exhausted", sample)
        if sample.get("plan_hash") != episode["plan_hash"]:
            return self._fail("plan_changed_during_active_view", sample)
        receipt = sample.get("completed_pose")
        expected_pose = episode["expected_pose"]
        if (not isinstance(receipt, Mapping) or receipt.get("status") != "completed"
                or not isinstance(receipt.get("macro"), Mapping)
                or receipt["macro"].get("kind") != "pose"
                or receipt["macro"].get("pulses") != expected_pose
                or not isinstance(receipt.get("ended_at"), (int, float))
                or receipt["ended_at"] > sample["capture_started_at_s"]
                or any(sample["pose"][key] != value
                       for key, value in expected_pose.items())):
            return self._fail("active_view_pose_receipt_missing", sample)
        prior_frame, prior_time, prior_sha = episode["last_view"]
        if (sample["frame_id"] <= prior_frame or sample["observed_at_s"] <= prior_time
                or sample["own_sha256"] == prior_sha):
            return self._fail("no_distinct_post_pose_rgb", sample)
        expected_generation = episode.get("held_wheel_generation", anchor["wheel_generation"])
        if self.wheel_generation != expected_generation or not _same_cargo_view(anchor, sample):
            return self._fail("reobservation_identity_or_wheel_mismatch", sample)
        if _direct_view(sample, 0.75):
            event = {"kind": "recovered", "reason": "strong_distinct_own_rgb",
                     "episode": self.episodes_started,
                     "poses_issued": len(episode["attempted_poses"]),
                     "frame_id": sample["frame_id"],
                     "own_sha256": sample["own_sha256"],
                     "anchor_frame_id": anchor["frame_id"],
                     "box_confidence": sample["box"]["confidence"],
                     "locked_pan_pwm": int(sample["pose"]["6"]),
                     "wheels_held_until_fresh_rgb": True}
            self.episode = None
            return event
        return self._issue_view_pose(sample)


@dataclass
class SettledViewRecovery(ActiveViewRecovery):
    """Reobserve a weak moving capture after an exact wheel HOLD first.

    The earlier strong view can suggest a camera search pose. It never becomes
    current drive evidence by adjusting its timestamp or wheel generation.
    """

    holds_started: int = 0
    settled_observations: int = 0

    def start(self, sample: Mapping[str, Any]) -> dict[str, Any]:
        if not _fresh_view(sample):
            return self._fail("stale_or_unpaired_rgb", sample)
        if sample.get("phase") != "approach" or not _direct_view(sample, 0.50):
            return self._fail("weak_view_identity_unresolved", sample)
        if self.episodes_started >= VIEW_RECOVERY_MAX_EPISODES:
            return self._fail("active_view_episode_budget_exhausted", sample)
        self.episodes_started += 1
        self.holds_started += 1
        held_at = sample["decision_at_s"]
        self.episode = {
            "started_at_s": held_at, "plan_hash": sample.get("plan_hash"),
            "attempted_poses": [], "expected_pose": None, "last_view": None,
            "stage": "hold_reobserve", "held_wheel_generation": self.wheel_generation,
            "capture_not_before_s": held_at + FINAL_ENTRY_SETTLE_S,
            "weak_view": {**sample, "box": dict(sample["box"]),
                          "target": tuple(sample["target"]), "pose": dict(sample["pose"])},
        }
        return {"kind": "hold_reobserve", "reason": "weak_capture_requires_settled_rgb",
                "episode": self.episodes_started, "issued_at_s": held_at,
                "capture_not_before_s": self.episode["capture_not_before_s"],
                "frame_id": sample["frame_id"], "own_sha256": sample["own_sha256"],
                "top_sha256": sample["top_sha256"],
                "wheel_generation": self.wheel_generation,
                "wheels_held": True, "issued_hold_not_measured_stop": True}

    def _issue_view_pose(self, sample: Mapping[str, Any]) -> dict[str, Any]:
        # Keep explicit proof that intervening wheel issues were not erased.
        episode = self.episode
        event = super()._issue_view_pose(sample)
        if episode is not None and event["kind"] == "pose_issued":
            event["settled_search"] = {
                "hold_at_s": episode["started_at_s"],
                "capture_not_before_s": episode["capture_not_before_s"],
                "held_wheel_generation": episode["held_wheel_generation"],
                "anchor_wheel_generation": self.anchor["wheel_generation"],
                "weak_frame_id": episode["weak_view"]["frame_id"],
                "weak_own_sha256": episode["weak_view"]["own_sha256"],
                "settled_frame_id": episode["settled_frame_id"],
                "settled_own_sha256": episode["settled_own_sha256"],
                "anchor_is_search_pose_only": True,
            }
        return event

    def advance(self, sample: Mapping[str, Any]) -> dict[str, Any] | None:
        episode = self.episode
        if episode is None or episode.get("stage") != "hold_reobserve":
            return super().advance(sample)
        if sample.get("phase") != "approach":
            return self._fail("phase_exit_during_active_view", sample)
        if not _fresh_view(sample):
            return self._fail("stale_or_unpaired_reobservation", sample)
        if sample["decision_at_s"] >= episode["started_at_s"] + VIEW_RECOVERY_MAX_ELAPSED_S:
            return self._fail("active_view_budget_exhausted", sample)
        if sample.get("plan_hash") != episode["plan_hash"]:
            return self._fail("plan_changed_during_active_view", sample)
        weak = episode["weak_view"]
        if (sample["capture_started_at_s"] < episode["capture_not_before_s"]
                or sample["frame_id"] <= weak["frame_id"]
                or sample["observed_at_s"] <= weak["observed_at_s"]):
            return self._fail("missing_post_hold_settled_capture", sample)
        if (self.wheel_generation != episode["held_wheel_generation"]
                or sample["pose"] != weak["pose"]):
            return self._fail("own_command_changed_during_settle", sample)
        if not _same_cargo_view(weak, sample):
            return self._fail("settled_view_identity_unresolved", sample)
        self.settled_observations += 1
        episode["settled_frame_id"] = sample["frame_id"]
        episode["settled_own_sha256"] = sample["own_sha256"]
        if _direct_view(sample, 0.75):
            self.episode = None
            return {"kind": "recovered", "reason": "strong_settled_own_rgb",
                    "episode": self.episodes_started, "poses_issued": 0,
                    "frame_id": sample["frame_id"], "own_sha256": sample["own_sha256"],
                    "top_sha256": sample["top_sha256"],
                    "hold_at_s": episode["started_at_s"],
                    "capture_not_before_s": episode["capture_not_before_s"],
                    "weak_frame_id": weak["frame_id"], "weak_own_sha256": weak["own_sha256"],
                    "box_confidence": sample["box"]["confidence"],
                    "wheels_held_until_fresh_rgb": True}
        anchor = self.anchor
        if anchor is None:
            return self._fail("no_bounded_strong_anchor", sample)
        if sample.get("plan_hash") != anchor["plan_hash"]:
            return self._fail("anchor_plan_changed", sample)
        if (sample["observed_at_s"] - anchor["observed_at_s"] > VIEW_RECOVERY_MAX_ANCHOR_AGE_S
                or anchor["frame_id"] >= weak["frame_id"]):
            return self._fail("search_anchor_stale", sample)
        if not _same_cargo_view(anchor, weak) or not _same_cargo_view(anchor, sample):
            return self._fail("weak_view_identity_unresolved", sample)
        episode["stage"] = "pose_search"
        return self._issue_view_pose(sample)
