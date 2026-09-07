"""A small, referee-only protocol for three-robot warehouse role agreement.

The protocol transports and checks proposals.  It deliberately does not contain
an auction, tie breaker, role compiler, or fallback policy: every decision must
come from the robot that submitted it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping
from uuid import uuid4
import hashlib
import json

ROBOT_IDS = ("r1", "r2", "r3")
ROLES = ("carrier_0", "carrier_1", "scout")


@dataclass(frozen=True)
class LocalObservation:
    """Facts available to one robot at the start of a negotiation round."""

    robot_id: str
    cargo_ids: tuple[str, ...]
    destinations: tuple[str, ...]
    revision: int = 0
    observation_id: str = ""
    facts: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_value(cls, value: "LocalObservation | Mapping[str, Any]") -> "LocalObservation":
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping):
            raise TypeError("observation must be LocalObservation or a mapping")
        return cls(
            robot_id=str(value.get("robot_id", "")),
            cargo_ids=tuple(str(x) for x in value.get("cargo_ids", ())),
            destinations=tuple(str(x) for x in value.get("destinations", ())),
            revision=int(value.get("revision", 0)),
            observation_id=str(value.get("observation_id", "")),
            facts=dict(value.get("facts", {})),
        )


@dataclass(frozen=True)
class Proposal:
    robot_id: str
    role: str
    cargo_id: str
    destination: str
    revision: int
    raw_message: str
    parsed_decision: Mapping[str, Any] = field(default_factory=dict)
    assignments: Mapping[str, str] = field(default_factory=dict)
    observation_id: str = ""

    @classmethod
    def from_value(cls, value: "Proposal | Mapping[str, Any]", *, raw_message: str | None = None) -> "Proposal":
        if isinstance(value, cls):
            if raw_message is None:
                return value
            return Proposal(value.robot_id, value.role, value.cargo_id, value.destination,
                            value.revision, raw_message, value.parsed_decision,
                            value.assignments, value.observation_id)
        if not isinstance(value, Mapping):
            raise TypeError("proposal must be Proposal or a mapping")
        return cls(
            robot_id=str(value.get("robot_id", "")), role=str(value.get("role", "")),
            cargo_id=str(value.get("cargo_id", "")), destination=str(value.get("destination", "")),
            revision=int(value.get("revision", -1)),
            raw_message=str(value.get("raw_message", value.get("message", raw_message or ""))),
            parsed_decision=dict(value.get("parsed_decision", {})),
            assignments=dict(value.get("assignments", {})),
            observation_id=str(value.get("observation_id", "")),
        )

    @property
    def plan_digest(self) -> str:
        payload = {"assignments": dict(sorted(self.assignments.items())),
                   "cargo_id": self.cargo_id, "destination": self.destination,
                   "revision": self.revision}
        return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


@dataclass(frozen=True)
class DecisionLogEntry:
    event: str
    robot_id: str | None
    raw_message: str
    parsed_decision: Mapping[str, Any]
    round_id: str
    reason: str | None = None


@dataclass(frozen=True)
class Evaluation:
    accepted: bool
    round_id: str
    reason_codes: tuple[str, ...]
    assignments: Mapping[str, str] = field(default_factory=dict)
    cargo_id: str | None = None
    destination: str | None = None


@dataclass
class _Round:
    round_id: str
    cargo_id: str
    destination: str
    revision: int
    peer_comm: bool
    observations: dict[str, LocalObservation]
    proposals: dict[str, Proposal] = field(default_factory=dict)
    inboxes: dict[str, list[str]] = field(default_factory=dict)
    log: list[DecisionLogEntry] = field(default_factory=list)
    evaluation: Evaluation | None = None
    reports: dict[str, set[str]] = field(default_factory=dict)


class WarehouseProtocol:
    """Transport and referee for exactly three independent robot proposals."""

    def __init__(self, robot_ids: tuple[str, ...] = ROBOT_IDS) -> None:
        if tuple(robot_ids) != ROBOT_IDS:
            raise ValueError("warehouse protocol requires exactly r1, r2, r3")
        self.robot_ids = ROBOT_IDS
        self._rounds: dict[str, _Round] = {}
        self._latest_revision: dict[str, int] = {}

    def start_round(self, observations: Mapping[str, LocalObservation | Mapping[str, Any]],
                    cargo_id: str | None, destination: str, revision: int = 0,
                    peer_comm: bool = True) -> str:
        """Create a fresh round; no proposal is synthesized by this method."""
        if set(observations) != set(self.robot_ids):
            raise ValueError("all three local observations are required")
        if not destination or revision < 0:
            raise ValueError("destination and non-negative revision are required")
        parsed = {rid: LocalObservation.from_value(observations[rid]) for rid in self.robot_ids}
        if any(obs.robot_id != rid or not obs.observation_id for rid, obs in parsed.items()):
            raise ValueError("observation robot_id does not match its key")
        previous = self._latest_revision.get(cargo_id)
        if previous is not None and revision < previous:
            raise ValueError("stale goal revision")
        self._latest_revision[cargo_id] = max(revision, previous or revision)
        rid = uuid4().hex
        self._rounds[rid] = _Round(rid, cargo_id or "", destination, revision, bool(peer_comm), parsed,
                                    inboxes={robot: [] for robot in self.robot_ids},
                                    reports={robot: set() for robot in self.robot_ids})
        return rid

    def publish_message(self, round_id: str, sender_id: str, message: str) -> None:
        """Deliver raw text to peers only when peer communication is enabled."""
        round_ = self._get(round_id)
        if round_.evaluation is not None:
            raise ValueError("round already evaluated")
        if sender_id not in self.robot_ids or not isinstance(message, str) or not message:
            raise ValueError("invalid sender or empty message")
        if not round_.peer_comm:
            return
        recipients = self.robot_ids
        for rid in recipients:
            if rid != sender_id or not round_.peer_comm:
                round_.inboxes[rid].append(message)

    def inbox(self, round_id: str, robot_id: str) -> tuple[str, ...]:
        round_ = self._get(round_id)
        if robot_id not in self.robot_ids:
            raise ValueError("unknown robot")
        return tuple(round_.inboxes[robot_id])

    def submit_proposal(self, round_id: str, proposal: Proposal | Mapping[str, Any],
                        raw_message: str | None = None) -> None:
        round_ = self._get(round_id)
        if round_.evaluation is not None:
            raise ValueError("round already evaluated")
        p = Proposal.from_value(proposal, raw_message=raw_message)
        parsed = dict(p.parsed_decision) or {"role": p.role, "cargo_id": p.cargo_id,
                                             "destination": p.destination, "revision": p.revision}
        if p.robot_id not in self.robot_ids:
            raise ValueError("unknown robot")
        if p.robot_id in round_.proposals:
            raise ValueError("duplicate proposal")
        round_.proposals[p.robot_id] = p
        round_.log.append(DecisionLogEntry("proposal", p.robot_id, p.raw_message, parsed, round_id))

    def publish_observation_report(self, round_id: str, sender_id: str,
                                   cargo_id: str, message: str) -> None:
        """Forward an explicit peer observation; isolated rounds reject it."""
        round_ = self._get(round_id)
        if round_.evaluation is not None:
            raise ValueError("round already evaluated")
        if sender_id not in self.robot_ids or not cargo_id or not message:
            raise ValueError("invalid observation report")
        if cargo_id not in round_.observations[sender_id].cargo_ids:
            raise ValueError("sender observation does not contain cargo")
        if not round_.peer_comm:
            raise ValueError("peer communication disabled")
        for recipient in self.robot_ids:
            if recipient != sender_id:
                round_.reports[recipient].add(cargo_id)
                round_.inboxes[recipient].append(message)

    def evaluate(self, round_id: str) -> Evaluation:
        round_ = self._get(round_id)
        if round_.evaluation is not None:
            return round_.evaluation
        errors: list[str] = []
        latest = self._latest_revision.get(round_.cargo_id)
        newer_round_exists = any(other.round_id != round_id and
                                 other.destination == round_.destination and
                                 other.revision > round_.revision
                                 for other in self._rounds.values())
        if (latest is not None and round_.revision < latest) or newer_round_exists:
            errors.append("STALE_REVISION")
        if set(round_.proposals) != set(self.robot_ids):
            errors.append("MISSING_PROPOSAL")
        for rid, p in round_.proposals.items():
            obs = round_.observations[rid]
            if not p.raw_message.strip():
                errors.append("INVALID_MESSAGE")
            if p.role not in ROLES:
                errors.append("INVALID_ROLE")
            if (round_.cargo_id and p.cargo_id != round_.cargo_id) or p.destination != round_.destination:
                errors.append("DISAGREEMENT")
            if p.revision != round_.revision or p.revision != obs.revision:
                errors.append("STALE_REVISION")
            if not p.observation_id or p.observation_id != obs.observation_id:
                errors.append("STALE_OBSERVATION")
            if p.cargo_id not in obs.cargo_ids and p.cargo_id not in round_.reports[rid]:
                errors.append("OBSERVATION_MISMATCH")
            if dict(p.assignments) != {"carrier_0": p.assignments.get("carrier_0"),
                                       "carrier_1": p.assignments.get("carrier_1"),
                                       "scout": p.assignments.get("scout")} or \
                    set(p.assignments.values()) != set(self.robot_ids) or \
                    p.assignments.get(p.role) != rid:
                errors.append("INVALID_PLAN")
        digests = {p.plan_digest for p in round_.proposals.values()}
        if len(digests) > 1:
            errors.append("PLAN_DISAGREEMENT")
        roles = [p.role for p in round_.proposals.values()]
        if len(roles) != len(set(roles)) or set(roles) != set(ROLES):
            errors.append("ROLE_COVERAGE")
        accepted = not errors and len(round_.proposals) == 3
        codes = tuple(dict.fromkeys(errors))
        assignments = dict(next(iter(round_.proposals.values())).assignments) if accepted else {}
        agreed_cargo = next(iter({p.cargo_id for p in round_.proposals.values()}), None)
        agreed_destination = next(iter({p.destination for p in round_.proposals.values()}), None)
        result = Evaluation(accepted, round_id, codes, assignments,
                            agreed_cargo if accepted else None,
                            agreed_destination if accepted else None)
        round_.evaluation = result
        round_.log.append(DecisionLogEntry("evaluation", None, "", {"accepted": accepted}, round_id,
                                           ",".join(codes) or None))
        return result

    def decision_log(self, round_id: str) -> tuple[DecisionLogEntry, ...]:
        return tuple(self._get(round_id).log)

    def _get(self, round_id: str) -> _Round:
        try:
            return self._rounds[round_id]
        except KeyError as exc:
            raise KeyError("unknown round") from exc
