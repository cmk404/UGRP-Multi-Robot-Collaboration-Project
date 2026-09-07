"""Versioned participant-only recovery decisions; no automatic plan selection."""
from dataclasses import dataclass, field
import time
import uuid


@dataclass
class RecoveryEvent:
    participants: tuple[str, ...]
    reason: str
    evidence: dict
    event_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    created_at: float = field(default_factory=time.monotonic)
    decisions: dict = field(default_factory=dict)
    resolved: bool = False

    def submit(self, robot_id, event_id, action):
        if event_id != self.event_id or self.resolved:
            raise ValueError("STALE_RECOVERY_EVENT")
        if robot_id not in self.participants or action not in {"resume", "replan", "cancel", "request_help"}:
            raise ValueError("INVALID_RECOVERY_DECISION")
        self.decisions[robot_id] = action
        if set(self.decisions) == set(self.participants) and len(set(self.decisions.values())) == 1:
            self.resolved = True
            return action
        return None

    def public(self):
        return {"event_id":self.event_id,"reason":self.reason,"evidence":dict(self.evidence),
                "participants":list(self.participants),"resolved":self.resolved}


def measurement_confidence(measurement, *, age_s=0., max_age_s=2.):
    """No precision/geometry is fabricated when only identity memory survives."""
    depth = measurement.get("range_m")
    confidence = measurement.get("confidence")
    fresh = measurement.get("current_view", True) and age_s <= max_age_s
    valid = isinstance(depth,(int,float)) and 0.08 <= depth <= 4.
    return {"usable_for_motion": bool(fresh and valid and isinstance(confidence,(int,float)) and confidence >= .7),
            "identity_only":not (fresh and valid),"age_s":age_s,
            "range_std_m":measurement.get("range_std_m") if fresh and valid else None,
            "reason":"CURRENT_RGBD" if fresh and valid else "REOBSERVATION_REQUIRED"}


def handling_decision(*, width_m=None, mass_kg=None, mass_std_kg=None,
                      solo_limit_kg=.10, grip_limit_m=.06, solo_failure=None):
    """Conservative policy over supplied estimates, never simulator body truth."""
    if solo_failure in {"OVERLOAD", "GRIP_UNSTABLE", "SOLO_LIFT_FAILED"}:
        return {"action":"request_help","reason":solo_failure}
    if width_m is None or mass_kg is None or mass_std_kg is None:
        return {"action":"observe","reason":"CAPACITY_UNCERTAIN"}
    if width_m > grip_limit_m or mass_kg+2*mass_std_kg > solo_limit_kg:
        return {"action":"request_help","reason":"SOLO_CAPACITY_EXCEEDED"}
    return {"action":"solo","reason":"CONSERVATIVE_CAPACITY_CHECK"}
