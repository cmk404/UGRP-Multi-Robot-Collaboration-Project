"""Referee for mixed solo and joint warehouse task claims.

This module is deliberately a referee, rather than a planner.  It never picks
the next cargo, assigns roles, or completes a missing participant proposal.
Robots submit claims for a manifest item and the referee only checks the
claims against fresh observations and the current reservation epoch.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Iterable
from uuid import uuid4


@dataclass(frozen=True)
class Observation:
    robot_id: str
    cargo_ids: tuple[str, ...]
    revision: int
    observation_id: str

    @classmethod
    def from_value(cls, value: "Observation | Mapping[str, Any]") -> "Observation":
        if isinstance(value, cls):
            return value
        return cls(str(value.get("robot_id", "")),
                   tuple(str(x) for x in value.get("cargo_ids", ())),
                   int(value.get("revision", -1)), str(value.get("observation_id", "")))


@dataclass(frozen=True)
class TaskClaim:
    robot_id: str
    cargo_id: str
    participants: tuple[str, ...]
    destination: str
    revision: int
    observation_id: str

    @classmethod
    def from_value(cls, value: "TaskClaim | Mapping[str, Any]") -> "TaskClaim":
        if isinstance(value, cls):
            return value
        return cls(str(value.get("robot_id", "")), str(value.get("cargo_id", "")),
                   tuple(str(x) for x in value.get("participants", ())),
                   str(value.get("destination", "")), int(value.get("revision", -1)),
                   str(value.get("observation_id", "")))


# A proposal is the public vocabulary used by callers; TaskClaim is the more
# descriptive name used in this module.
Proposal = TaskClaim


@dataclass(frozen=True)
class AcceptedAssignment:
    assignment_id: str
    cargo_id: str
    participants: tuple[str, ...]
    destination: str
    revision: int

    def as_dict(self) -> dict[str, Any]:
        return {"assignment_id": self.assignment_id, "cargo_id": self.cargo_id,
                "participants": self.participants, "destination": self.destination,
                "revision": self.revision}


@dataclass(frozen=True)
class Rejection:
    proposal: TaskClaim
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {"proposal": self.proposal, "reason": self.reason}


@dataclass(frozen=True)
class BatchResult:
    accepted: tuple[AcceptedAssignment, ...] = ()
    rejected: tuple[Rejection, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {"accepted": [a.as_dict() for a in self.accepted],
                "rejected": [r.as_dict() for r in self.rejected]}

    def __getitem__(self, key: str) -> Any:
        return self.as_dict()[key]


@dataclass
class _Assignment:
    value: AcceptedAssignment
    completed: set[str] = field(default_factory=set)


class MixedWarehouseReferee:
    """Validate independent solo/joint claims and atomically reserve them."""

    def __init__(self, manifest: Mapping[str, Any] | Iterable[Mapping[str, Any]],
                 robot_ids: Iterable[str] = ("r1", "r2", "r3"), revision: int = 0):
        self.robot_ids = tuple(robot_ids)
        self.revision = int(revision)
        self.required_carriers = self._parse_manifest(manifest)
        self.observations: dict[str, Observation] = {}
        self.peer_reports: dict[str, set[str]] = {rid: set() for rid in self.robot_ids}
        self._reservations: dict[str, _Assignment] = {}
        self._cargo_reservations: dict[str, str] = {}
        self._robot_reservations: dict[str, str] = {}
        self._seen_proposals: set[tuple[Any, ...]] = set()
        self._seen_completions: set[tuple[str, str, int]] = set()
        self._released: set[tuple[str, int]] = set()
        self.pending: dict[str, TaskClaim] = {}

    @staticmethod
    def _parse_manifest(manifest: Mapping[str, Any] | Iterable[Mapping[str, Any]]) -> dict[str, int]:
        if isinstance(manifest, Mapping):
            result = {}
            for cid, value in manifest.items():
                if isinstance(value, Mapping):
                    value = value.get("required_carriers", value.get("required_participants"))
                result[str(cid)] = int(value)
            return result
        return {str(item["cargo_id"]): int(item.get("required_carriers", item.get("required_participants")))
                for item in manifest}

    def observe(self, observation: Observation | Mapping[str, Any]) -> None:
        obs = Observation.from_value(observation)
        if obs.robot_id not in self.robot_ids or not obs.observation_id or obs.revision != self.revision:
            raise ValueError("INVALID_OBSERVATION")
        self.observations[obs.robot_id] = obs

    record_observation = observe

    def report_peer_observation(self, reporter_id: str, cargo_id: str,
                                recipients: Iterable[str] | None = None) -> None:
        if reporter_id not in self.robot_ids or cargo_id not in self.required_carriers:
            raise ValueError("INVALID_PEER_REPORT")
        obs = self.observations.get(reporter_id)
        if obs is None or cargo_id not in obs.cargo_ids:
            raise ValueError("UNSUPPORTED_PEER_REPORT")
        targets = self.robot_ids if recipients is None else tuple(recipients)
        for rid in targets:
            if rid not in self.robot_ids:
                raise ValueError("INVALID_PEER_REPORT")
            if rid != reporter_id:
                self.peer_reports[rid].add(cargo_id)

    def submit(self, proposals: Iterable[TaskClaim | Mapping[str, Any]]) -> BatchResult:
        """Accept every currently compatible ready claim; do not wait for all robots."""
        claims = [TaskClaim.from_value(p) for p in proposals]
        rejected: list[Rejection] = []
        candidates: dict[tuple[str, tuple[str, ...], str, int], list[TaskClaim]] = {}
        repeated = {p.robot_id for p in claims if sum(q.robot_id == p.robot_id for q in claims)>1}
        for claim in claims:
            reason = "MULTIPLE_ROBOT_CLAIMS" if claim.robot_id in repeated else self._validate(claim)
            if reason:
                rejected.append(Rejection(claim, reason))
                continue
            self.pending[claim.robot_id] = claim
        for rid, claim in tuple(self.pending.items()):
            obs = self.observations.get(rid)
            if obs is None or obs.observation_id != claim.observation_id or claim.revision != self.revision:
                self.pending.pop(rid, None)
                continue
            key = (claim.cargo_id, tuple(sorted(claim.participants)), claim.destination, claim.revision)
            candidates.setdefault(key, []).append(claim)
        accepted: list[AcceptedAssignment] = []
        # Each group is decided independently.  A rejected/conflicting group
        # cannot partially reserve a cargo or robot.
        for key, group in candidates.items():
            # An earlier independently-ready group in this batch may have
            # invalidated and pruned these proposals.
            group = [p for p in group if self.pending.get(p.robot_id) == p]
            if not group:
                continue
            cargo, participants, destination, revision = key
            by_robot = {p.robot_id: p for p in group}
            if set(by_robot) != set(participants):
                continue  # Pending consent never blocks an unrelated solo job.
            if cargo in self._cargo_reservations:
                for p in group:
                    self.pending.pop(p.robot_id, None)
                    rejected.append(Rejection(p, "RESERVED"))
                continue
            if any(r in self._robot_reservations for r in participants):
                # A live author may consent now to work that starts after a
                # partner's independent assignment. No reservation is made
                # until the entire proposed crew is free.
                continue
            aid = uuid4().hex
            assignment = AcceptedAssignment(aid, cargo, participants, destination, revision)
            self._reservations[aid] = _Assignment(assignment)
            self._cargo_reservations[cargo] = aid
            for rid in participants:
                self._robot_reservations[rid] = aid
                self.pending.pop(rid, None)
            accepted.append(assignment)
            # Pending consent remains valid only while its cargo and every
            # proposed participant are unreserved. Give each affected author
            # explicit own-action feedback instead of stranding the claim.
            for author, pending in tuple(self.pending.items()):
                if (pending.cargo_id in self._cargo_reservations or
                        author in self._robot_reservations):
                    self.pending.pop(author, None)
                    rejected.append(Rejection(pending, "RESERVED"))
        return BatchResult(tuple(accepted), tuple(rejected))

    accept = submit
    evaluate = submit

    def _validate(self, p: TaskClaim) -> str | None:
        if p.robot_id not in self.robot_ids:
            return "UNKNOWN_ROBOT"
        if not p.cargo_id or p.cargo_id not in self.required_carriers:
            return "UNKNOWN_CARGO"
        required = self.required_carriers[p.cargo_id]
        if required not in (1, 2):
            return "INVALID_MANIFEST"
        if len(p.participants) != required:
            return "OVERSIZED_SOLO_CLAIM" if required == 2 and len(p.participants) == 1 else "PARTICIPANT_COUNT"
        if len(set(p.participants)) != len(p.participants) or any(r not in self.robot_ids for r in p.participants):
            return "DISTINCT_PARTICIPANTS_REQUIRED"
        if p.robot_id not in p.participants:
            return "NONPARTICIPANT_PROPOSAL"
        if not p.destination:
            return "INVALID_DESTINATION"
        if p.revision != self.revision:
            return "STALE_EPOCH"
        obs = self.observations.get(p.robot_id)
        if obs is None or obs.revision != self.revision or obs.observation_id != p.observation_id:
            return "STALE_OBSERVATION"
        if p.cargo_id not in obs.cargo_ids and p.cargo_id not in self.peer_reports[p.robot_id]:
            return "UNSUPPORTED_CARGO_OBSERVATION"
        marker = (p.robot_id, p.cargo_id, tuple(sorted(p.participants)), p.destination, p.revision, p.observation_id)
        if marker in self._seen_proposals:
            return "REPLAYED_PROPOSAL"
        self._seen_proposals.add(marker)
        if (p.cargo_id in self._cargo_reservations or
                p.robot_id in self._robot_reservations):
            return "RESERVED"
        return None

    def complete(self, assignment_id: str, robot_id: str, revision: int | None = None) -> dict[str, Any]:
        self._check_assignment(assignment_id, revision)
        assignment = self._reservations[assignment_id]
        if robot_id not in assignment.value.participants:
            raise ValueError("NONPARTICIPANT_COMPLETION")
        marker = (assignment_id, robot_id, assignment.value.revision)
        if marker in self._seen_completions:
            raise ValueError("REPLAYED_COMPLETION")
        self._seen_completions.add(marker)
        assignment.completed.add(robot_id)
        return {"ok": True, "assignment_id": assignment_id,
                "complete": assignment.completed == set(assignment.value.participants)}

    def release(self, assignment_id: str, revision: int | None = None) -> dict[str, Any]:
        self._check_assignment(assignment_id, revision)
        assignment = self._reservations[assignment_id]
        marker = (assignment_id, assignment.value.revision)
        if marker in self._released:
            raise ValueError("REPLAYED_RELEASE")
        if assignment.completed != set(assignment.value.participants):
            raise ValueError("INCOMPLETE_ASSIGNMENT")
        self._released.add(marker)
        del self._reservations[assignment_id]
        self._cargo_reservations.pop(assignment.value.cargo_id, None)
        for rid in assignment.value.participants:
            self._robot_reservations.pop(rid, None)
        return {"ok": True, "assignment_id": assignment_id, "released": True}

    def abort(self, assignment_id: str, reason: str):
        self._check_assignment(assignment_id, self.revision)
        assignment = self._reservations.pop(assignment_id).value
        self._cargo_reservations.pop(assignment.cargo_id, None)
        for rid in assignment.participants:
            self._robot_reservations.pop(rid, None)
        return {"aborted": True, "reason": reason}

    def _check_assignment(self, assignment_id: str, revision: int | None) -> None:
        if revision is not None and revision != self.revision:
            raise ValueError("STALE_EPOCH")
        if assignment_id not in self._reservations:
            raise ValueError("STALE_OR_UNKNOWN_ASSIGNMENT")

    def advance_revision(self, revision: int) -> None:
        if revision <= self.revision:
            raise ValueError("STALE_EPOCH")
        if self._reservations:
            raise ValueError("ACTIVE_ASSIGNMENTS_REQUIRE_ABORT")
        self.pending.clear()
        self.revision = revision
        self.observations.clear()
        self.peer_reports = {rid: set() for rid in self.robot_ids}

    @property
    def active_assignments(self) -> tuple[AcceptedAssignment, ...]:
        return tuple(item.value for item in self._reservations.values())


__all__ = ["Observation", "TaskClaim", "Proposal", "AcceptedAssignment", "Rejection",
           "BatchResult", "MixedWarehouseReferee"]
