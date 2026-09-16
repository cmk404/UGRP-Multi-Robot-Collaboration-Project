"""Local task-stage coordination; no planner, perception, actuator or simulator.

All public calls must be serialized by one coordinator. Report timestamps use
that coordinator's declared clock domain. See docs/task_stage_sync_contract.md.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time
from typing import Any, Callable
from uuid import uuid4

from harness.pair_carry_sync import PairCarrySync


STAGES = ("GRASP", "LIFT", "CARRY", "LOWER", "RELEASE")
READY_CHECKS = {
    "GRASP": frozenset({"at_grasp_pose", "stopped"}),
    "LIFT": frozenset({"grasp_observed", "stopped"}),
    "CARRY": frozenset({"load_lift_observed"}),
    "LOWER": frozenset({"arrived_observed", "stopped"}),
    "RELEASE": frozenset({"support_observed", "stopped"}),
}
DONE_CHECKS = {
    "GRASP": frozenset({"grasp_observed", "stopped"}),
    "LIFT": frozenset({"load_lift_observed", "stopped"}),
    "CARRY": frozenset({"arrived_observed", "stopped"}),
    "LOWER": frozenset({"support_observed", "stopped"}),
    "RELEASE": frozenset({"released_observed", "stopped"}),
}
CHECKS = frozenset().union(*READY_CHECKS.values(), *DONE_CHECKS.values(), {"motion_observed"})
STATUSES = {"READY", "NOT_READY", "UNCERTAIN", "FAILED", "OBSERVED_MOTION", "DONE"}


def _text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _integer(value: Any, minimum: int = 0) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= minimum


def _keys(value: Any, expected: set[str]) -> None:
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError("missing or unexpected contract fields")


@dataclass(frozen=True)
class TaskPlan:
    task_id: str
    plan_version: int
    object_id: str
    roles: tuple[tuple[str, str], ...]
    goal_region: str
    map_id: str
    map_version: str
    map_sha256: str

    def __post_init__(self) -> None:
        if not all(_text(v) for v in (self.task_id, self.object_id, self.goal_region,
                                     self.map_id, self.map_version)):
            raise ValueError("task, object, goal and static map identity are required")
        if not _integer(self.plan_version, 1):
            raise ValueError("plan_version must be a positive integer")
        if (not isinstance(self.roles, tuple) or len(self.roles) < 2
                or any(not isinstance(p, tuple) or len(p) != 2
                       or not all(_text(v) for v in p) for p in self.roles)
                or len({p[0] for p in self.roles}) != len(self.roles)):
            raise ValueError("at least two unique robot identities and roles are required")
        if (not isinstance(self.map_sha256, str) or len(self.map_sha256) != 64
                or any(c not in "0123456789abcdef" for c in self.map_sha256)):
            raise ValueError("map_sha256 must be a lowercase SHA256 digest")

    @property
    def participants(self) -> tuple[str, ...]:
        return tuple(p[0] for p in self.roles)

    def to_dict(self) -> dict:
        return {"schema": "ugrp.task_plan.v1", "task_id": self.task_id,
                "plan_version": self.plan_version, "object_id": self.object_id,
                "participants": [{"robot_id": r, "role": role} for r, role in self.roles],
                "goal_region": self.goal_region,
                "static_map": {"id": self.map_id, "version": self.map_version,
                               "sha256": self.map_sha256}}

    @classmethod
    def from_dict(cls, value: dict) -> TaskPlan:
        _keys(value, {"schema", "task_id", "plan_version", "object_id", "participants",
                      "goal_region", "static_map"})
        if value["schema"] != "ugrp.task_plan.v1" or not isinstance(value["participants"], list):
            raise ValueError("invalid plan schema or participants")
        for participant in value["participants"]:
            _keys(participant, {"robot_id", "role"})
        _keys(value["static_map"], {"id", "version", "sha256"})
        m = value["static_map"]
        return cls(value["task_id"], value["plan_version"], value["object_id"],
                   tuple((p["robot_id"], p["role"]) for p in value["participants"]),
                   value["goal_region"], m["id"], m["version"], m["sha256"])


@dataclass(frozen=True)
class StageReport:
    task_id: str
    run_id: str
    plan_version: int
    stage: str
    epoch: int
    robot_id: str
    sequence: int
    status: str
    observed_at_s: float
    decided_at_s: float
    sent_at_s: float
    observation_id: str
    own_rgb_ref: str
    top_rgb_ref: str
    confidence: float
    checks: tuple[str, ...]
    command_id: str | None
    reason: str

    def __post_init__(self) -> None:
        if not all(_text(v) for v in (self.task_id, self.run_id, self.robot_id,
                                     self.observation_id, self.own_rgb_ref, self.top_rgb_ref)):
            raise ValueError("identities and both RGB references are required")
        if not _integer(self.plan_version, 1) or not _integer(self.epoch) or not _integer(self.sequence):
            raise ValueError("invalid version, epoch or sequence")
        if not isinstance(self.stage, str) or self.stage not in STAGES:
            raise ValueError("invalid stage")
        if not isinstance(self.status, str) or self.status not in STATUSES:
            raise ValueError("invalid report status")
        if (not all(_number(v) and v >= 0 for v in
                    (self.observed_at_s, self.decided_at_s, self.sent_at_s))
                or not self.observed_at_s <= self.decided_at_s <= self.sent_at_s):
            raise ValueError("invalid report timestamp order")
        if not _number(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be in [0, 1]")
        if (not isinstance(self.checks, tuple) or any(not isinstance(c, str) or c not in CHECKS
                                                   for c in self.checks)
                or len(set(self.checks)) != len(self.checks)):
            raise ValueError("invalid or duplicate visual checks")
        if self.command_id is not None and not _text(self.command_id):
            raise ValueError("command_id must be a nonempty string or null")
        if not isinstance(self.reason, str):
            raise ValueError("reason must be text")

    def to_dict(self) -> dict:
        result = asdict(self)
        result.update(schema="ugrp.stage_report.v1", evidence_source="own_rgb+top_rgb",
                      checks=list(self.checks))
        return result

    @classmethod
    def from_dict(cls, value: dict) -> StageReport:
        names = set(cls.__dataclass_fields__)
        _keys(value, names | {"schema", "evidence_source"})
        if value["schema"] != "ugrp.stage_report.v1" or value["evidence_source"] != "own_rgb+top_rgb":
            raise ValueError("only the RGB report schema is accepted")
        if not isinstance(value["checks"], list):
            raise ValueError("checks must be a JSON array")
        fields = {key: value[key] for key in names}
        fields["checks"] = tuple(fields["checks"])
        return cls(**fields)


@dataclass(frozen=True)
class Permission:
    run_id: str
    task_id: str
    plan_version: int
    stage: str
    epoch: int
    token: str
    issued_at_s: float
    expires_at_s: float


class TaskStageSync:
    """Five serial barriers using the existing PairCarrySync readiness core.

    dispatch() is the integration seam for a nonblocking, bounded local command
    submitter. The submitter must honor duration_s and implement STOP/hold on its
    own watchdog. This coordinator cannot stop an already dispatched action.
    """

    def __init__(self, plan: TaskPlan, *, report_ttl_s: float = 0.6,
                 max_command_s: float = 0.25, min_confidence: float = 0.8,
                 clock_domain: str = "monotonic", now_s: float = 0.0) -> None:
        if not isinstance(plan, TaskPlan):
            raise ValueError("validated TaskPlan required")
        if (not _number(max_command_s) or max_command_s <= 0
                or not _number(min_confidence) or not 0 <= min_confidence <= 1
                or clock_domain not in ("monotonic", "sim")):
            raise ValueError("invalid command limit, confidence or clock domain")
        self.plan = plan
        self.run_id = uuid4().hex
        self.max_command_s = float(max_command_s)
        self.min_confidence = float(min_confidence)
        self.clock_domain = clock_domain
        self._barrier = PairCarrySync(plan.task_id, plan.participants, plan.plan_version, report_ttl_s)
        self._index = 0
        self._finished = False
        self._aborted = False
        self._last_clock_s = -math.inf
        self._epoch_started_s = now_s
        self._latest: dict[str, StageReport] = {}
        self._commands: dict[str, dict] = {}
        self._last_command: dict[str, str] = {}
        self._seen_rgb: dict[str, set[tuple[str, str]]] = {p: set() for p in plan.participants}
        self._permits: dict[str, Permission] = {}
        self._wall_origin = time.monotonic()
        self.events: list[dict] = []
        self._clock(now_s)
        self._event("REQUEST", now_s, plan=plan.to_dict())

    @property
    def stage(self) -> str:
        return STAGES[self._index]

    @property
    def epoch(self) -> int:
        return self._barrier.epoch

    def _clock(self, now_s: float) -> None:
        if not _number(now_s) or now_s < 0 or now_s < self._last_clock_s:
            raise ValueError("coordinator time must be finite, nonnegative and nondecreasing")
        self._last_clock_s = float(now_s)

    def _event(self, event: str, now_s: float, **fields: Any) -> None:
        self.events.append({"event": event, "task_id": self.plan.task_id, "run_id": self.run_id,
                            "plan_version": self.plan.plan_version, "stage": self.stage,
                            "epoch": self.epoch, "timestamp_s": now_s,
                            "clock_domain": self.clock_domain,
                            "wall_elapsed_s": time.monotonic() - self._wall_origin, **fields})

    def _clear_epoch(self, now_s: float) -> None:
        self._latest.clear()
        self._permits.clear()
        self._epoch_started_s = now_s

    def _refresh(self, now_s: float) -> dict:
        self._clock(now_s)
        if self._finished:
            return {"phase": "FINISH", "epoch": self.epoch, "reason": "all_stages_reported_done"}
        old_epoch = self.epoch
        decision = self._barrier.authorize(now_s)
        if self.epoch != old_epoch:
            self._clear_epoch(now_s)
            self._event("HOLD", now_s, reason=decision["reason"])
        self._permits = {key: value for key, value in self._permits.items() if value.expires_at_s > now_s}
        return decision

    def receive(self, robot_id: str, payload: dict, *, now_s: float) -> bool:
        """Receive from a caller-authenticated robot; reject extra/GT fields.

        Schema validation proves shape, not that referenced images support the
        producer's interpretation. Raw images and producer inputs need an audit.
        """
        self._refresh(now_s)
        try:
            report = StageReport.from_dict(payload)
            if self._finished or self._aborted:
                raise ValueError("task_terminal")
            if robot_id not in self.plan.participants or report.robot_id != robot_id:
                raise ValueError("source_identity_mismatch")
            if (report.task_id, report.run_id, report.plan_version, report.stage, report.epoch) != (
                    self.plan.task_id, self.run_id, self.plan.plan_version, self.stage, self.epoch):
                raise ValueError("task_run_plan_stage_or_epoch_mismatch")
            if report.sent_at_s > now_s or report.observed_at_s < self._epoch_started_s:
                raise ValueError("future_report_or_pre_epoch_observation")
            if (report.own_rgb_ref, report.top_rgb_ref) in self._seen_rgb[robot_id]:
                raise ValueError("reused_rgb_pair")
            if report.status in ("DONE", "OBSERVED_MOTION"):
                command = self._commands.get(report.command_id)
                if (not command or command["robot_id"] != robot_id or command["stage"] != self.stage
                        or command["plan_version"] != self.plan.plan_version
                        or report.command_id != self._last_command.get(robot_id)
                        or command["failed"] or report.observed_at_s <= command["issued_at_s"]
                        or (report.status == "DONE" and report.observed_at_s < command["end_at_s"])):
                    raise ValueError("post_command_visual_evidence_required")
        except (ValueError, TypeError) as exc:
            self._event("REPORT_REJECTED", now_s, robot_id=robot_id, reason=str(exc))
            return False

        required = (DONE_CHECKS[self.stage] if report.status == "DONE" else
                    {"motion_observed"} if report.status == "OBSERVED_MOTION" else READY_CHECKS[self.stage])
        ready = (report.status in ("READY", "OBSERVED_MOTION", "DONE")
                 and report.confidence >= self.min_confidence and required <= set(report.checks))
        accepted = self._barrier.report(
            robot_id, plan_version=report.plan_version, epoch=report.epoch, sequence=report.sequence,
            ready=ready, observed_at_s=report.observed_at_s, received_at_s=now_s,
            frame_id=report.observation_id, reason=report.reason or report.status)
        if not accepted:
            self._event("REPORT_REJECTED", now_s, robot_id=robot_id,
                        reason=self._barrier.events[-1]["reason"])
            return False
        previous = self._latest.get(robot_id)
        self._latest[robot_id] = report
        self._seen_rgb[robot_id].add((report.own_rgb_ref, report.top_rgb_ref))
        self._event("REPORT", now_s, robot_id=robot_id, report=report.to_dict(), ready=ready,
                    observation_to_decision_s=report.decided_at_s-report.observed_at_s,
                    sender_queue_s=report.sent_at_s-report.decided_at_s,
                    delivery_s=now_s-report.sent_at_s)
        if not ready or (previous and previous.status == "DONE" and report.status != "DONE"):
            self.hold("participant_not_ready_or_completion_revoked:" + robot_id, now_s=now_s, renew=True)
        return True

    def status(self, *, now_s: float) -> dict:
        """Refresh revocation/expiry without minting a new permission per physics tick."""
        return {**self._refresh(now_s), "run_id": self.run_id, "task_id": self.plan.task_id,
                "plan_version": self.plan.plan_version, "stage": self.stage, "permission": None}

    def authorize(self, *, now_s: float) -> dict:
        decision = self._refresh(now_s)
        permit = None
        if decision["phase"] == "GO":
            expires = min(r.observed_at_s + self._barrier.report_ttl_s for r in self._latest.values())
            expires = min(expires, now_s + self.max_command_s)
            if expires > now_s:
                grant = Permission(self.run_id, self.plan.task_id, self.plan.plan_version,
                                   self.stage, self.epoch, uuid4().hex, now_s, expires)
                self._permits[grant.token] = grant
                permit = asdict(grant)
                self._event("GO", now_s, permission=permit)
        return {**decision, "run_id": self.run_id, "task_id": self.plan.task_id,
                "plan_version": self.plan.plan_version, "stage": self.stage, "permission": permit}

    def dispatch(self, robot_id: str, permission: dict | None, command_id: str, *,
                 now_s: float, duration_s: float, submit: Callable[[], Any]) -> bool:
        """Gate one nonblocking local submission. Return False without submitting on denial."""
        decision = self._refresh(now_s)
        grant = self._permits.get(permission.get("token")) if isinstance(permission, dict) and isinstance(permission.get("token"), str) else None
        latest = self._latest.get(robot_id)
        preceding = self._commands.get(self._last_command.get(robot_id))
        if (decision["phase"] != "GO" or grant is None or permission != asdict(grant)
                or robot_id not in self.plan.participants or latest is None or latest.status == "DONE"
                or not _text(command_id) or command_id in self._commands
                or not _number(duration_s) or not 0 < duration_s <= self.max_command_s
                or (preceding is not None and now_s < preceding["end_at_s"])
                or now_s + duration_s > grant.expires_at_s or not callable(submit)):
            self._event("COMMAND_DENIED", now_s, robot_id=robot_id)
            return False
        self._commands[command_id] = {"robot_id": robot_id, "stage": self.stage,
                                      "plan_version": self.plan.plan_version,
                                      "issued_at_s": now_s, "end_at_s": now_s + duration_s, "failed": False}
        self._last_command[robot_id] = command_id
        self._event("COMMAND_ISSUED", now_s, robot_id=robot_id, command_id=command_id,
                    duration_s=duration_s, permission_token=grant.token)
        try:
            submit()
        except Exception:
            self._commands[command_id]["failed"] = True
            self._event("COMMAND_ERROR", now_s, robot_id=robot_id, command_id=command_id)
            self.hold("local_submission_failed:" + robot_id, now_s=now_s)
            raise
        return True

    def hold(self, reason: str, *, now_s: float, renew: bool = False) -> dict:
        self._clock(now_s)
        if self._finished or self._aborted:
            return self._refresh(now_s)
        old_epoch = self.epoch
        result = self._barrier.hold(reason, now_s, renew=renew)
        if self.epoch != old_epoch:
            self._clear_epoch(now_s)
            self._event("HOLD", now_s, reason=reason)
        return result

    def abort(self, reason: str, *, now_s: float) -> dict:
        self._clock(now_s)
        if self._finished:
            return self._refresh(now_s)
        self._aborted = True
        self._barrier.abort(reason, now_s)
        self._clear_epoch(now_s)
        self._event("ABORT", now_s, reason=reason)
        return self._refresh(now_s)

    def advance(self, *, now_s: float) -> bool:
        decision = self._refresh(now_s)
        if decision["phase"] != "GO" or not all(
                p in self._latest and self._latest[p].status == "DONE" for p in self.plan.participants):
            return False
        self._event("STAGE_REPORTED_DONE", now_s)
        self._barrier.hold("stage_transition", now_s)
        self._clear_epoch(now_s)
        if self._index == len(STAGES) - 1:
            self._finished = True
            self._event("FINISH", now_s, physical_success_evaluated=False)
        else:
            self._index += 1
            self._event("STAGE_REQUEST", now_s)
        return True

    def update_plan(self, plan: TaskPlan, *, now_s: float) -> None:
        self._clock(now_s)
        if self._finished or self._aborted:
            raise ValueError("cannot update a terminal task")
        if (not isinstance(plan, TaskPlan) or plan.task_id != self.plan.task_id
                or plan.object_id != self.plan.object_id or plan.roles != self.plan.roles):
            raise ValueError("task, object and participant roles cannot change within a run")
        self._barrier.update_plan(plan.plan_version, now_s)
        self.plan = plan
        self._clear_epoch(now_s)
        self._event("PLAN_UPDATED", now_s, plan=plan.to_dict())
