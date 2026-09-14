"""Small, deterministic barriers for paired execution.

This module deliberately contains no transport, planner, or simulator integration.
Callers supply timestamps from one monotonic clock domain.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class _ReadyReport:
    plan_version: int
    epoch: int
    sequence: int
    ready: bool
    observed_at_s: float
    received_at_s: float
    frame_id: str
    reason: str


class PairCarrySync:
    """In-process two-party execution barrier with revocable permissions."""

    def __init__(
        self,
        task_id: str,
        participants: Iterable[str] = ("r1", "r3"),
        plan_version: int = 1,
        report_ttl_s: float = 0.6,
    ) -> None:
        participant_tuple = tuple(participants)
        if not task_id or len(participant_tuple) < 2:
            raise ValueError("task_id and at least two participants are required")
        if len(set(participant_tuple)) != len(participant_tuple) or any(not p for p in participant_tuple):
            raise ValueError("participants must be unique non-empty identifiers")
        if not isinstance(plan_version, int) or isinstance(plan_version, bool) or plan_version < 1:
            raise ValueError("plan_version must be a positive integer")
        if not _finite(report_ttl_s) or report_ttl_s <= 0:
            raise ValueError("report_ttl_s must be finite and positive")

        self.task_id = task_id
        self.participants: Tuple[str, ...] = participant_tuple
        self.plan_version = plan_version
        self.report_ttl_s = float(report_ttl_s)
        self.epoch = 0
        self.events: List[Dict[str, Any]] = []
        self._phase = "WAIT"
        self._reason = "waiting_for_readiness"
        self._last_clock_s = -math.inf
        self._reports: Dict[str, _ReadyReport] = {}
        self._last_sequence: Dict[str, int] = {}
        self._last_observed: Dict[str, float] = {}
        self._last_received: Dict[str, float] = {}
        self._seen_frames: Dict[str, set[str]] = {p: set() for p in self.participants}

    def report(
        self,
        robot_id: str,
        *,
        plan_version: int,
        epoch: int,
        sequence: int,
        ready: bool,
        observed_at_s: float,
        received_at_s: float,
        frame_id: str,
        reason: str = "",
    ) -> bool:
        rejection = self._validate_report(
            robot_id, plan_version, epoch, sequence, ready,
            observed_at_s, received_at_s, frame_id,
        )
        if rejection:
            self._event("REPORT_REJECTED", received_at_s, robot_id=robot_id, reason=rejection)
            return False

        report = _ReadyReport(
            plan_version, epoch, sequence, ready, float(observed_at_s),
            float(received_at_s), frame_id, reason,
        )
        self._reports[robot_id] = report
        self._last_sequence[robot_id] = sequence
        self._last_observed[robot_id] = float(observed_at_s)
        self._last_received[robot_id] = float(received_at_s)
        self._seen_frames[robot_id].add(frame_id)
        self._last_clock_s = float(received_at_s)
        self._event(
            "REPORT_ACCEPTED", received_at_s, robot_id=robot_id, ready=ready,
            sequence=sequence, frame_id=frame_id, reason=reason,
        )
        return True

    def authorize(self, now_s: float) -> Dict[str, Any]:
        self._accept_clock(now_s)
        if self._phase == "ABORT":
            return self._decision()

        if self._phase == "GO":
            invalid = self._invalid_readiness_reason(now_s)
            if invalid:
                return self.hold(invalid, now_s)
            return self._decision()

        if self._all_ready(now_s):
            previous = self._phase
            self._phase = "GO"
            self._reason = "all_participants_ready" if previous == "WAIT" else "all_participants_ready_to_resume"
            self._event("AUTHORIZED", now_s, phase="GO", reason=self._reason)
        return self._decision()

    def hold(self, reason: str, now_s: float) -> Dict[str, Any]:
        self._accept_clock(now_s)
        if self._phase == "ABORT":
            return self._decision()
        if self._phase != "HOLD":
            self.epoch += 1
            self._reports.clear()
            self._phase = "HOLD"
            self._reason = reason or "hold_requested"
            self._event("HELD", now_s, phase="HOLD", reason=self._reason)
        return self._decision()

    def abort(self, reason: str, now_s: float) -> Dict[str, Any]:
        self._accept_clock(now_s)
        if self._phase != "ABORT":
            self._reports.clear()
            self._phase = "ABORT"
            self._reason = reason or "abort_requested"
            self._event("ABORTED", now_s, phase="ABORT", reason=self._reason)
        return self._decision()

    def update_plan(self, plan_version: int, now_s: float, reason: str = "plan_updated") -> Dict[str, Any]:
        self._accept_clock(now_s)
        if not isinstance(plan_version, int) or isinstance(plan_version, bool) or plan_version <= self.plan_version:
            raise ValueError("new plan_version must be a larger integer")
        if self._phase == "ABORT":
            raise RuntimeError("cannot update an aborted synchronization task")
        self.plan_version = plan_version
        self.epoch += 1
        self._reports.clear()
        self._phase = "HOLD"
        self._reason = reason
        self._event("PLAN_UPDATED", now_s, phase="HOLD", reason=reason, plan_version=plan_version)
        return self._decision()

    def _validate_report(self, robot_id, plan_version, epoch, sequence, ready,
                         observed_at_s, received_at_s, frame_id) -> str:
        if robot_id not in self.participants:
            return "unknown_participant"
        if self._phase == "ABORT":
            return "task_aborted"
        if not isinstance(plan_version, int) or isinstance(plan_version, bool):
            return "invalid_plan_version"
        if plan_version != self.plan_version:
            return "plan_version_mismatch"
        if not isinstance(epoch, int) or isinstance(epoch, bool):
            return "invalid_epoch"
        if epoch != self.epoch:
            return "epoch_mismatch"
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
            return "invalid_sequence"
        if not isinstance(ready, bool):
            return "invalid_ready"
        if not _finite(observed_at_s) or not _finite(received_at_s):
            return "non_finite_timestamp"
        if observed_at_s > received_at_s:
            return "future_observation"
        if received_at_s - observed_at_s > self.report_ttl_s:
            return "stale_on_arrival"
        if received_at_s < self._last_clock_s:
            return "non_monotonic_clock"
        if not isinstance(frame_id, str) or not frame_id:
            return "invalid_frame_id"
        if sequence <= self._last_sequence.get(robot_id, -1):
            return "stale_sequence"
        if observed_at_s < self._last_observed.get(robot_id, -math.inf):
            return "non_monotonic_observation"
        if received_at_s < self._last_received.get(robot_id, -math.inf):
            return "non_monotonic_receipt"
        if frame_id in self._seen_frames[robot_id]:
            return "duplicate_frame"
        return ""

    def _all_ready(self, now_s: float) -> bool:
        return all(
            (report := self._reports.get(robot_id)) is not None
            and report.ready
            and now_s >= report.observed_at_s
            and now_s - report.observed_at_s <= self.report_ttl_s
            for robot_id in self.participants
        )

    def _invalid_readiness_reason(self, now_s: float) -> str:
        for robot_id in self.participants:
            report = self._reports.get(robot_id)
            if report is None:
                return f"missing_report:{robot_id}"
            if not report.ready:
                return f"not_ready:{robot_id}"
            if now_s < report.observed_at_s:
                return f"clock_before_observation:{robot_id}"
            if now_s - report.observed_at_s > self.report_ttl_s:
                return f"report_expired:{robot_id}"
        return ""

    def _decision(self) -> Dict[str, Any]:
        return {"phase": self._phase, "epoch": self.epoch, "reason": self._reason}

    def _event(self, kind: str, timestamp_s: float, **fields: Any) -> None:
        event = {
            "event": kind,
            "timestamp_s": float(timestamp_s) if _finite(timestamp_s) else None,
            "task_id": self.task_id,
            "plan_version": self.plan_version,
            "epoch": self.epoch,
        }
        event.update(fields)
        self.events.append(event)

    @staticmethod
    def _require_time(value: float) -> None:
        if not _finite(value):
            raise ValueError("timestamp must be finite")

    def _accept_clock(self, value: float) -> None:
        self._require_time(value)
        if value < self._last_clock_s:
            raise ValueError("timestamp must not move backwards")
        self._last_clock_s = float(value)


@dataclass
class _ResourceState:
    generation: int
    task_id: str
    plan_version: int
    reserved_at_s: float
    expires_at_s: float
    reservations: set[str] = field(default_factory=set)
    occupied: bool = False
    release_acks: set[str] = field(default_factory=set)


class SharedResourceLedger:
    """A bounded local ownership ledger; occupied entries expire only by release."""

    def __init__(self, required_participants: Iterable[str] = ("r1", "r3"), reservation_ttl_s: float = 0.6) -> None:
        self.required_participants = tuple(required_participants)
        if not self.required_participants or len(set(self.required_participants)) != len(self.required_participants):
            raise ValueError("required_participants must be unique")
        if not _finite(reservation_ttl_s) or reservation_ttl_s <= 0:
            raise ValueError("reservation_ttl_s must be finite and positive")
        self.reservation_ttl_s = float(reservation_ttl_s)
        self.events: List[Dict[str, Any]] = []
        self._resources: Dict[str, _ResourceState] = {}
        self._next_generation = 1
        self._last_clock_s = -math.inf

    def reserve(self, resource_id: str, *, task_id: str, plan_version: int,
                participant: str, now_s: float, generation: Optional[int] = None) -> Optional[int]:
        self._validate_request(resource_id, task_id, plan_version, participant, now_s)
        if generation is not None:
            self._validate_generation(generation)
        state = self._resources.get(resource_id)
        if state and not state.occupied and now_s > state.expires_at_s:
            del self._resources[resource_id]
            state = None
        if state is None:
            if generation is not None:
                return None
            generation = self._next_generation
            self._next_generation += 1
            state = _ResourceState(generation, task_id, plan_version, now_s,
                                   now_s + self.reservation_ttl_s)
            self._resources[resource_id] = state
        elif generation != state.generation:
            return None
        if state.task_id != task_id or state.plan_version != plan_version:
            return None
        state.reservations.add(participant)
        state.expires_at_s = max(state.expires_at_s, now_s + self.reservation_ttl_s)
        self._record("RESERVED", resource_id, state, now_s, participant)
        return state.generation

    def occupy(self, resource_id: str, *, task_id: str, plan_version: int,
               generation: int, now_s: float) -> bool:
        self._validate_identity(resource_id, task_id, plan_version, now_s)
        self._validate_generation(generation)
        state = self._resources.get(resource_id)
        if (not state or state.generation != generation or state.task_id != task_id
                or state.plan_version != plan_version):
            return False
        if not state.occupied and now_s > state.expires_at_s:
            del self._resources[resource_id]
            return False
        if state.occupied:
            return True
        if set(self.required_participants) != state.reservations:
            return False
        state.occupied = True
        self._record("OCCUPIED", resource_id, state, now_s)
        return True

    def release(self, resource_id: str, *, task_id: str, plan_version: int,
                participant: str, generation: int, now_s: float) -> bool:
        self._validate_request(resource_id, task_id, plan_version, participant, now_s)
        self._validate_generation(generation)
        state = self._resources.get(resource_id)
        if (not state or not state.occupied or state.generation != generation
                or state.task_id != task_id or state.plan_version != plan_version):
            return False
        if participant in state.release_acks:
            return False
        state.release_acks.add(participant)
        self._record("RELEASE_ACK", resource_id, state, now_s, participant)
        if set(self.required_participants) == state.release_acks:
            self._record("RELEASED", resource_id, state, now_s)
            del self._resources[resource_id]
            return True
        return False

    def state(self, resource_id: str, now_s: float) -> Optional[Dict[str, Any]]:
        self._accept_clock(now_s)
        state = self._resources.get(resource_id)
        if state and not state.occupied and now_s > state.expires_at_s:
            del self._resources[resource_id]
            state = None
        if state is None:
            return None
        return {
            "resource_id": resource_id, "generation": state.generation, "task_id": state.task_id,
            "plan_version": state.plan_version, "occupied": state.occupied,
            "reservations": sorted(state.reservations),
            "release_acks": sorted(state.release_acks),
            "expires_at_s": state.expires_at_s,
        }

    def _validate_request(self, resource_id, task_id, plan_version, participant, now_s) -> None:
        self._validate_identity(resource_id, task_id, plan_version, now_s)
        if participant not in self.required_participants:
            raise ValueError("unknown participant")

    def _validate_identity(self, resource_id, task_id, plan_version, now_s) -> None:
        if not resource_id or not task_id:
            raise ValueError("resource_id and task_id are required")
        if not isinstance(plan_version, int) or isinstance(plan_version, bool) or plan_version < 1:
            raise ValueError("plan_version must be a positive integer")
        self._accept_clock(now_s)

    @staticmethod
    def _validate_generation(generation: int) -> None:
        if not isinstance(generation, int) or isinstance(generation, bool) or generation < 1:
            raise ValueError("generation must be a positive integer")

    def _accept_clock(self, now_s: float) -> None:
        PairCarrySync._require_time(now_s)
        if now_s < self._last_clock_s:
            raise ValueError("timestamp must not move backwards")
        self._last_clock_s = float(now_s)

    def _record(self, event, resource_id, state, timestamp_s, participant=None) -> None:
        item = {"event": event, "timestamp_s": float(timestamp_s), "resource_id": resource_id,
                "generation": state.generation, "task_id": state.task_id,
                "plan_version": state.plan_version}
        if participant is not None:
            item["participant"] = participant
        self.events.append(item)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
