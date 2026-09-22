"""Common RGB execution adapter for matched none/structured/natural studies.

The adapter coordinates independently submitted task requests and bounded
robot-local commands.  It never chooses cargo, partners, roles, task order, or
recovery actions.  Joint work begins only after every named participant submits
the same task core.  A joint command batch similarly waits for one independently
submitted command from each participant, while unrelated tasks keep advancing.

This module is coordination and endpoint wiring, not evidence that any RGB
skill succeeds physically.  Callers must provide physical endpoints, a strict
RGB frame source, and explicit skill capabilities.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import json
from dataclasses import dataclass, field
from functools import wraps
import threading
from typing import Any, Iterable, Mapping
from uuid import uuid4

from harness.rgb_execution_contract import (
    COMMAND_KIND,
    INTERRUPT_KIND,
    RELEASE_KIND,
    REQUEST_KIND,
    EvaluationSource,
    FrameSource,
    MotionEndpoint,
    SkillCapability,
    finite_number,
    nonempty_text,
    validate_static_context,
)


def _json_copy(value: Any) -> Any:
    """Detach untrusted payloads and reject NaN/non-JSON objects."""
    return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))


def _exact_keys(value: Any, expected: set[str]) -> None:
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError("missing or unexpected contract fields")


def _text_list(value: Any, *, allow_empty: bool = False) -> tuple[str, ...]:
    if (not isinstance(value, list) or (not value and not allow_empty)
            or any(not nonempty_text(item) for item in value)
            or len(set(value)) != len(value)):
        raise ValueError("expected unique nonempty text list")
    return tuple(value)


def _serialized(method):
    """Serialize the shared clock/lease state while preserving method metadata."""
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return locked


@dataclass(frozen=True)
class _TaskRequest:
    request_id: str
    task_id: str
    object_id: str
    skill: str
    participants: tuple[str, ...]
    resources: tuple[str, ...]
    stage: str
    own_role: str
    observation_id: str
    decision_id: str
    requested_at_s: float
    expires_at_s: float

    @property
    def core(self) -> tuple[Any, ...]:
        return (self.task_id, self.object_id, self.skill, self.participants,
                self.resources, self.stage)


@dataclass
class _Lease:
    lease_id: str
    task_id: str
    object_id: str
    skill: str
    participants: tuple[str, ...]
    resources: tuple[str, ...]
    stage: str
    roles: dict[str, str]
    expires_at_s: float
    queued: dict[str, dict] = field(default_factory=dict)


class RGBExecutionPort:
    """Actor-safe facade over robot-local bounded motion endpoints.

    ``tick`` owns the declared clock.  Every actor call observes the most recent
    tick value; actor payloads cannot advance time.  For SIM, call ``tick``
    before every physics step.  For a real-time owner, pass the same monotonic
    clock used by endpoint watchdogs.
    """

    def __init__(self, endpoints: Mapping[str, MotionEndpoint], *,
                 frame_source: FrameSource,
                 static_context: Mapping[str, Any],
                 capabilities: Iterable[SkillCapability],
                 clock_domain: str,
                 now_s: float = 0.,
                 evaluation_source: EvaluationSource | None = None,
                 command_history_limit: int = 16,
                 max_request_age_s: float = 5.):
        if not endpoints or any(not nonempty_text(robot_id) for robot_id in endpoints):
            raise ValueError("at least one named endpoint is required")
        if any(getattr(endpoint, "robot_id", None) != robot_id
               for robot_id, endpoint in endpoints.items()):
            raise ValueError("endpoint identities must match mapping keys")
        if clock_domain not in {"sim", "monotonic"}:
            raise ValueError("clock_domain must be sim or monotonic")
        if not finite_number(now_s, minimum=0.):
            raise ValueError("now_s must be a nonnegative finite number")
        if not isinstance(command_history_limit, int) or command_history_limit < 1:
            raise ValueError("command_history_limit must be positive")
        if not finite_number(max_request_age_s, minimum=0.) or max_request_age_s == 0:
            raise ValueError("max_request_age_s must be positive")
        declared = list(capabilities)
        if not declared or len({item.skill for item in declared}) != len(declared):
            raise ValueError("capabilities must uniquely name at least one skill")

        detached_static_context = _json_copy(static_context)
        validate_static_context(detached_static_context)
        declared_names = {item.skill for item in declared}
        if set(detached_static_context["task"]["allowed_skills"]) != declared_names:
            raise ValueError("static allowed_skills must match declared capabilities")

        self._endpoints = dict(endpoints)
        self._lock = threading.RLock()
        self._frame_source = frame_source
        self._static_context = detached_static_context
        self._static_context_sha256 = hashlib.sha256(
            json.dumps(self._static_context, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")).encode()).hexdigest()
        self._capabilities = {item.skill: item for item in declared}
        self._clock_domain = clock_domain
        self._now_s = float(now_s)
        self._evaluation_source = evaluation_source
        self._history_limit = command_history_limit
        self._max_request_age_s = float(max_request_age_s)
        self._observation_sequence = {robot_id: 0 for robot_id in endpoints}
        self._own_commands = {robot_id: [] for robot_id in endpoints}
        self._own_events = {robot_id: [] for robot_id in endpoints}
        self._own_results = {robot_id: [] for robot_id in endpoints}
        self._pending: dict[str, dict[str, _TaskRequest]] = {}
        self._active: dict[str, _Lease] = {}
        self._resource_owners: dict[str, str] = {}
        self._terminal_tasks: set[str] = set()
        self._request_ids = {robot_id: set() for robot_id in endpoints}
        self._command_ids = {robot_id: set() for robot_id in endpoints}
        self._audit: list[dict] = []
        self._closed = False

    @property
    def robot_ids(self) -> tuple[str, ...]:
        return tuple(self._endpoints)

    @property
    def clock_domain(self) -> str:
        return self._clock_domain

    def _require_open_robot(self, robot_id: str) -> None:
        if self._closed:
            raise RuntimeError("execution port is closed")
        if robot_id not in self._endpoints:
            raise ValueError("unknown robot_id")

    def _record(self, robot_id: str, event: str, **fields: Any) -> None:
        row = {"event": event, "robot_id": robot_id,
               "timestamp_s": self._now_s, **_json_copy(fields)}
        self._own_events[robot_id].append(row)
        self._audit.append(copy.deepcopy(row))

    def _result(self, robot_id: str, request_id: str, status: str,
                reason: str, **fields: Any) -> dict:
        row = {"request_id": request_id, "status": status, "reason": reason,
               "clock_domain": self._clock_domain, "timestamp_s": self._now_s,
               **_json_copy(fields)}
        self._own_results[robot_id].append(copy.deepcopy(row))
        self._record(robot_id, "SUBMISSION_" + status, **row)
        return copy.deepcopy(row)

    def _hold_participants(self, participants: Iterable[str], reason: str) -> list[dict]:
        errors = []
        for robot_id in participants:
            try:
                self._endpoints[robot_id].hold(self._now_s)
            except Exception as exc:  # still attempt every named participant
                errors.append({"robot_id": robot_id,
                               "error_type": type(exc).__name__})
            self._record(robot_id, "LOCAL_HOLD", reason=reason)
        return errors

    def _release_lease(self, lease: _Lease, reason: str) -> None:
        for resource in lease.resources:
            if self._resource_owners.get(resource) == lease.task_id:
                self._resource_owners.pop(resource)
        self._active.pop(lease.task_id, None)
        self._terminal_tasks.add(lease.task_id)
        public_reason = {
            "actor_released_without_measured_success": "task_released_unverified",
            "actor_interrupted_task": "task_interrupted",
            "task lease expired": "lease_expired",
            "endpoint_tick_failed": "execution_unavailable",
            "command_batch_failed": "execution_unavailable",
            "command_validation_failed": "execution_unavailable",
            "execution_port_closed": "execution_port_closed",
        }.get(reason, "task_terminated")
        for robot_id in lease.participants:
            self._own_results[robot_id].append({
                "request_id": "lifecycle:" + lease.lease_id,
                "status": "TERMINATED",
                "reason": public_reason,
                "clock_domain": self._clock_domain,
                "timestamp_s": self._now_s,
                "task_id": lease.task_id,
                "lease_id": lease.lease_id,
            })
            self._record(robot_id, "TASK_TERMINATED", task_id=lease.task_id,
                         lease_id=lease.lease_id, reason=reason,
                         meaning="coordination lifecycle, not measured task success")

    def _pending_for_robot(self, robot_id: str) -> list[_TaskRequest]:
        return [requests[robot_id] for requests in self._pending.values()
                if robot_id in requests]

    def _active_for_robot(self, robot_id: str) -> list[_Lease]:
        return [lease for lease in self._active.values()
                if robot_id in lease.participants]

    @_serialized
    def observe(self, robot_id: str) -> dict:
        """Return only permitted RGB, static context and own issued commands."""
        self._require_open_robot(robot_id)
        try:
            packet = self._frame_source(robot_id, self._now_s)
        except Exception as exc:
            self._hold_participants((robot_id,), "rgb_source_failed")
            raise RuntimeError("rgb source unavailable") from exc
        if not isinstance(packet, Mapping) or set(packet) != {"own_rgb", "top_rgb"}:
            self._hold_participants((robot_id,), "invalid_rgb_packet")
            raise ValueError("frame source must return only own_rgb and top_rgb")
        images = {}
        for name in ("own_rgb", "top_rgb"):
            data = packet[name]
            if (not isinstance(data, bytes) or not data.startswith(b"\xff\xd8")
                    or not data.endswith(b"\xff\xd9")):
                self._hold_participants((robot_id,), "invalid_rgb_packet")
                raise ValueError("original JPEG bytes required")
            digest = hashlib.sha256(data).hexdigest()
            images[name] = {"ref": f"memory://{robot_id}/{self._observation_sequence[robot_id] + 1}/{name}.jpg",
                            "sha256": digest,
                            "jpeg_base64": base64.b64encode(data).decode("ascii")}
        self._observation_sequence[robot_id] += 1
        observation_id = f"{robot_id}-{self._observation_sequence[robot_id]:06d}"
        return _json_copy({
            "schema": "ugrp.rgb_actor_observation.v1",
            "robot_id": robot_id,
            "observation_id": observation_id,
            "observed_at_s": self._now_s,
            "clock_domain": self._clock_domain,
            "images": images,
            "static_context": self._static_context,
            "static_context_sha256": self._static_context_sha256,
            "own_issued_commands": self._own_commands[robot_id][-self._history_limit:],
        })

    @_serialized
    def local_status(self, robot_id: str) -> dict:
        """Return only this robot's request/command lifecycle, never peer state."""
        self._require_open_robot(robot_id)
        active = []
        for lease in self._active_for_robot(robot_id):
            active.append({"task_id": lease.task_id, "object_id": lease.object_id,
                           "skill": lease.skill, "stage": lease.stage,
                           "participants": list(lease.participants),
                           "own_role": lease.roles[robot_id],
                           "lease_id": lease.lease_id,
                           "expires_at_s": lease.expires_at_s,
                           "own_command_pending": robot_id in lease.queued})
        pending = []
        for request in self._pending_for_robot(robot_id):
            pending.append({"request_id": request.request_id,
                            "task_id": request.task_id,
                            "object_id": request.object_id,
                            "skill": request.skill,
                            "stage": request.stage,
                            "participants": list(request.participants),
                            "own_role": request.own_role,
                            "expires_at_s": request.expires_at_s,
                            "status": "PENDING_CONSENT"})
        return _json_copy({
            "schema": "ugrp.rgb_actor_status.v1",
            "robot_id": robot_id,
            "clock_domain": self._clock_domain,
            "timestamp_s": self._now_s,
            "active": active,
            "pending": pending,
            "own_submission_history": self._own_results[robot_id][-self._history_limit:],
        })

    @_serialized
    def submit(self, robot_id: str, submission: Mapping[str, Any]) -> dict:
        """Validate and enqueue one actor-owned request without planner repair."""
        self._require_open_robot(robot_id)
        try:
            payload = _json_copy(submission)
            if not isinstance(payload, dict) or payload.get("kind") not in {
                    REQUEST_KIND, COMMAND_KIND, INTERRUPT_KIND, RELEASE_KIND}:
                raise ValueError("unknown submission kind")
            if payload["kind"] == REQUEST_KIND:
                return self._submit_task(robot_id, payload)
            if payload["kind"] == COMMAND_KIND:
                return self._submit_command(robot_id, payload)
            return self._submit_termination(robot_id, payload)
        except (TypeError, ValueError, KeyError) as exc:
            request_id = (submission.get("request_id") if isinstance(submission, Mapping)
                          and nonempty_text(submission.get("request_id")) else "invalid")
            return self._result(robot_id, request_id, "REJECTED", str(exc))

    def _validate_correlation(self, payload: dict) -> None:
        for name in ("observation_id", "decision_id"):
            if not nonempty_text(payload[name]):
                raise ValueError(f"{name} must be nonempty text")
        requested = payload["requested_at_s"]
        if not finite_number(requested, minimum=0.):
            raise ValueError("requested_at_s must be a nonnegative finite number")
        if requested > self._now_s:
            raise ValueError("future actor request")
        if self._now_s - requested > self._max_request_age_s:
            raise ValueError("stale actor request")

    def _submit_task(self, robot_id: str, payload: dict) -> dict:
        _exact_keys(payload, {"kind", "request_id", "task_id", "object_id", "skill",
                              "participants", "resources", "stage", "own_role",
                              "observation_id", "decision_id", "requested_at_s",
                              "expires_at_s"})
        for name in ("request_id", "task_id", "object_id", "skill", "stage", "own_role"):
            if not nonempty_text(payload[name]):
                raise ValueError(f"{name} must be nonempty text")
        participants = tuple(sorted(_text_list(payload["participants"])))
        resources = tuple(sorted(_text_list(payload["resources"])))
        self._validate_correlation(payload)
        if robot_id not in participants or set(participants) - set(self._endpoints):
            raise ValueError("participants must name the submitting endpoint and known peers")
        if not finite_number(payload["expires_at_s"], minimum=0.) or payload["expires_at_s"] <= self._now_s:
            raise ValueError("task request is expired")
        if payload["request_id"] in self._request_ids[robot_id]:
            raise ValueError("duplicate request_id")
        self._request_ids[robot_id].add(payload["request_id"])
        capability = self._capabilities.get(payload["skill"])
        if capability is None:
            raise ValueError("unsupported skill")
        if payload["stage"] not in capability.stages or len(participants) not in capability.team_sizes:
            raise ValueError("unsupported skill stage or team size")
        if payload["task_id"] in self._terminal_tasks or payload["task_id"] in self._active:
            raise ValueError("task_id is already active or terminal")
        if self._active_for_robot(robot_id) or self._pending_for_robot(robot_id):
            raise ValueError("robot already has an active or pending task")
        if any(resource in self._resource_owners for resource in resources):
            raise ValueError("resource conflict")

        request = _TaskRequest(payload["request_id"], payload["task_id"], payload["object_id"],
                               payload["skill"], participants, resources, payload["stage"],
                               payload["own_role"], payload["observation_id"],
                               payload["decision_id"], float(payload["requested_at_s"]),
                               float(payload["expires_at_s"]))
        group = self._pending.setdefault(request.task_id, {})
        if group and next(iter(group.values())).core != request.core:
            raise ValueError("joint request core mismatch")
        group[robot_id] = request
        self._record(robot_id, "TASK_REQUESTED", request_id=request.request_id,
                     task_id=request.task_id, participants=list(request.participants),
                     own_role=request.own_role)
        if set(group) != set(request.participants):
            return self._result(robot_id, request.request_id, "PENDING", "awaiting_independent_consent",
                                task_id=request.task_id)

        if any(self._active_for_robot(participant) for participant in request.participants):
            return self._reject_group(request.task_id, "participant already active")[robot_id]
        if any(resource in self._resource_owners for resource in request.resources):
            return self._reject_group(request.task_id, "resource conflict")[robot_id]
        roles = {participant: group[participant].own_role for participant in request.participants}
        if capability.distinct_roles and len(set(roles.values())) != len(roles):
            return self._reject_group(request.task_id, "joint roles must be independently distinct")[robot_id]
        lease = _Lease(uuid4().hex, request.task_id, request.object_id, request.skill,
                       request.participants, request.resources, request.stage, roles,
                       min(item.expires_at_s for item in group.values()))
        self._active[lease.task_id] = lease
        self._pending.pop(lease.task_id)
        for resource in lease.resources:
            self._resource_owners[resource] = lease.task_id
        results = {}
        for participant in lease.participants:
            own = group[participant]
            results[participant] = self._result(
                participant, own.request_id, "ACCEPTED", "independent_consent_complete",
                task_id=lease.task_id, lease_id=lease.lease_id,
                expires_at_s=lease.expires_at_s)
        return results[robot_id]

    def _reject_group(self, task_id: str, reason: str) -> dict[str, dict]:
        group = self._pending.pop(task_id, {})
        return {robot_id: self._result(robot_id, request.request_id, "REJECTED", reason,
                                       task_id=task_id)
                for robot_id, request in group.items()}

    def _submit_command(self, robot_id: str, payload: dict) -> dict:
        _exact_keys(payload, {"kind", "request_id", "task_id", "lease_id", "command_id",
                              "stage", "action", "duration_s", "observation_id",
                              "decision_id", "requested_at_s"})
        for name in ("request_id", "task_id", "lease_id", "command_id", "stage"):
            if not nonempty_text(payload[name]):
                raise ValueError(f"{name} must be nonempty text")
        self._validate_correlation(payload)
        if payload["request_id"] in self._request_ids[robot_id]:
            raise ValueError("duplicate request_id")
        self._request_ids[robot_id].add(payload["request_id"])
        if payload["command_id"] in self._command_ids[robot_id]:
            raise ValueError("duplicate command_id")
        lease = self._active.get(payload["task_id"])
        if (lease is None or robot_id not in lease.participants
                or payload["lease_id"] != lease.lease_id):
            raise ValueError("unknown task lease")
        if payload["stage"] != lease.stage:
            raise ValueError("command stage does not match task stage")
        if robot_id in lease.queued:
            raise ValueError("robot already has a queued command")
        if not isinstance(payload["action"], dict) or not nonempty_text(payload["action"].get("kind")):
            raise ValueError("action must contain a kind")
        capability = self._capabilities[lease.skill]
        if payload["action"]["kind"] not in capability.action_kinds:
            raise ValueError("action kind is unsupported for this skill")
        if (not finite_number(payload["duration_s"], minimum=0.)
                or payload["duration_s"] <= 0
                or payload["duration_s"] > capability.max_duration_s):
            raise ValueError("command duration is outside the capability bound")
        try:
            self._endpoints[robot_id].validate_bounded(payload["action"], payload["duration_s"])
        except Exception as exc:
            self._hold_participants(lease.participants, "command_validation_failed")
            self._release_lease(lease, "command_validation_failed")
            raise ValueError(f"command validation failed: {type(exc).__name__}") from exc
        self._command_ids[robot_id].add(payload["command_id"])
        lease.queued[robot_id] = copy.deepcopy(payload)
        reason = ("ready_for_tick" if len(lease.participants) == 1
                  or set(lease.queued) == set(lease.participants)
                  else "awaiting_participant_command")
        return self._result(robot_id, payload["request_id"], "QUEUED", reason,
                            task_id=lease.task_id, command_id=payload["command_id"])

    def _submit_termination(self, robot_id: str, payload: dict) -> dict:
        _exact_keys(payload, {"kind", "request_id", "task_id", "lease_id", "reason",
                              "observation_id", "decision_id", "requested_at_s"})
        for name in ("request_id", "task_id", "lease_id", "reason"):
            if not nonempty_text(payload[name]):
                raise ValueError(f"{name} must be nonempty text")
        self._validate_correlation(payload)
        if payload["request_id"] in self._request_ids[robot_id]:
            raise ValueError("duplicate request_id")
        self._request_ids[robot_id].add(payload["request_id"])
        lease = self._active.get(payload["task_id"])
        if (lease is None or robot_id not in lease.participants
                or payload["lease_id"] != lease.lease_id):
            raise ValueError("unknown task lease")
        meaning = ("actor_released_without_measured_success" if payload["kind"] == RELEASE_KIND
                   else "actor_interrupted_task")
        errors = self._hold_participants(lease.participants, meaning)
        self._release_lease(lease, meaning)
        return self._result(robot_id, payload["request_id"], "ACCEPTED", meaning,
                            task_id=payload["task_id"], hold_failed=bool(errors))

    @_serialized
    def tick(self, now_s: float) -> dict:
        """Advance watchdogs, expiries and ready queues without a global barrier."""
        if self._closed:
            raise RuntimeError("execution port is closed")
        if not finite_number(now_s, minimum=0.) or now_s < self._now_s:
            raise ValueError("clock must be finite, nonnegative and monotonic")
        self._now_s = float(now_s)
        rows = []
        endpoint_errors = []
        for robot_id, endpoint in self._endpoints.items():
            try:
                endpoint.tick(self._now_s)
            except Exception as exc:
                endpoint_errors.append({"robot_id": robot_id,
                                        "error_type": type(exc).__name__})
                for lease in list(self._active_for_robot(robot_id)):
                    self._hold_participants(lease.participants, "endpoint_tick_failed")
                    self._release_lease(lease, "endpoint_tick_failed")

        for task_id, group in list(self._pending.items()):
            if any(request.expires_at_s <= self._now_s for request in group.values()):
                rejected = self._reject_group(task_id, "task request expired")
                rows.extend(rejected.values())

        for lease in list(self._active.values()):
            if lease.expires_at_s <= self._now_s:
                errors = self._hold_participants(lease.participants, "task lease expired")
                self._release_lease(lease, "task lease expired")
                rows.append({"task_id": lease.task_id, "status": "EXPIRED",
                             "hold_errors": errors})

        for lease in list(self._active.values()):
            if set(lease.queued) != set(lease.participants):
                continue
            batch = lease.queued
            try:
                for robot_id in lease.participants:
                    command = batch[robot_id]
                    self._endpoints[robot_id].apply_bounded(
                        command["action"], self._now_s, command["duration_s"])
                    issued = {"task_id": lease.task_id, "lease_id": lease.lease_id,
                              "command_id": command["command_id"], "stage": lease.stage,
                              "action": copy.deepcopy(command["action"]),
                              "duration_s": command["duration_s"],
                              "issued_at_s": self._now_s,
                              "meaning": "issued command, not measured state or success"}
                    self._own_commands[robot_id].append(issued)
                    self._record(robot_id, "LOCAL_COMMAND", **issued)
                rows.append({"task_id": lease.task_id, "status": "DISPATCHED",
                             "participants": list(lease.participants)})
                lease.queued = {}
            except Exception as exc:
                errors = self._hold_participants(lease.participants, "command_batch_failed")
                self._release_lease(lease, "command_batch_failed")
                rows.append({"task_id": lease.task_id, "status": "FAILED",
                             "error_type": type(exc).__name__,
                             "hold_errors": errors})
        return _json_copy({"clock_domain": self._clock_domain,
                           "timestamp_s": self._now_s,
                           "events": rows, "endpoint_errors": endpoint_errors})

    @_serialized
    def evaluation_snapshot(self) -> dict:
        """Trusted evaluator-only snapshot; never returned by actor methods."""
        if self._evaluation_source is None:
            raise RuntimeError("evaluation source is not connected")
        external = _json_copy(self._evaluation_source())
        return _json_copy({
            "schema": "ugrp.rgb_evaluation_snapshot.v1",
            "clock_domain": self._clock_domain,
            "timestamp_s": self._now_s,
            "external_evaluation": external,
            "coordination_audit": self._audit,
            "active_task_ids": sorted(self._active),
            "pending_task_ids": sorted(self._pending),
            "resource_owners": self._resource_owners,
            "meaning": "evaluation-only; forbidden as actor input or completion feedback",
        })

    @_serialized
    def close(self, now_s: float) -> None:
        """Hold this port's active participants and preserve evaluator separation."""
        if self._closed:
            return
        if not finite_number(now_s, minimum=0.) or now_s < self._now_s:
            raise ValueError("clock must be finite, nonnegative and monotonic")
        self._now_s = float(now_s)
        held: set[str] = set()
        for lease in list(self._active.values()):
            participants = [robot_id for robot_id in lease.participants if robot_id not in held]
            self._hold_participants(participants, "execution_port_closed")
            held.update(participants)
            self._release_lease(lease, "execution_port_closed")
        pending_robots = {robot_id for group in self._pending.values() for robot_id in group} - held
        self._hold_participants(sorted(pending_robots), "execution_port_closed")
        self._pending.clear()
        self._closed = True
