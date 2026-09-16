"""Serialized SIM execution adapter for the five-stage synchronization contract.

Camera packets and a robot-local producer's JSON replies cross the boundary.
The adapter does not infer grasp, support, or completion. No producer means
UNCERTAIN, never READY. The physics owner must call tick() before every step.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import json
from pathlib import Path

from harness.task_stage_sync import StageReport, TaskPlan, TaskStageSync


def _write(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def uncertain_reply(request: dict) -> dict:
    """Default when the execution owner's visual stage producer is absent."""
    return {"request_id": request["request_id"], "status": "UNCERTAIN", "confidence": 0.,
            "checks": [], "command_id": None, "reason": "visual_stage_producer_not_connected",
            "decided_at_s": request["observed_at_s"]}


class TaskStageExecution:
    """One coordinator, local CameraRobotPorts, and one shared simulation clock.

    Only the trusted physics owner receives this object/ports. A visual producer
    receives a detached JSON camera request and returns a detached JSON reply.
    This is a local integration seam, not network authentication or a process
    sandbox. Use receive()'s robot_id from the bound endpoint, not reply text.
    """

    def __init__(self, plan: TaskPlan, ports: dict, *, map_path: Path,
                 output: Path, now_s: float = 0., report_ttl_s: float = .6):
        if set(ports) != set(plan.participants) or any(p.robot_id != r for r, p in ports.items()):
            raise ValueError("plan and local port identities must match")
        if hashlib.sha256(map_path.read_bytes()).hexdigest() != plan.map_sha256:
            raise ValueError("static map bytes do not match the plan")
        self.sync = TaskStageSync(plan, clock_domain="sim", now_s=now_s, report_ttl_s=report_ttl_s)
        self.ports = dict(ports)
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output / "rgb").mkdir()
        self._pending: dict[str, dict] = {}
        self._sequence = {r: 0 for r in ports}
        self._history = {r: [] for r in ports}
        self._requests = 0
        self._last_context = None
        self._held = False
        self.events: list[dict] = []
        _write(self.output / "plan.json", plan.to_dict())
        (self.output / "static-map.json").write_bytes(map_path.read_bytes())
        self.tick(now_s)

    def _stop_all(self, now_s: float, reason: str) -> None:
        errors = []
        # One broken endpoint must not prevent attempting to stop every peer.
        for rid, port in self.ports.items():
            try:
                port.hold(now_s)
            except Exception as exc:
                errors.append({"robot_id": rid, "error": f"{type(exc).__name__}: {exc}"})
        if not self._held or errors:
            self.events.append({"event": "LOCAL_HOLD", "timestamp_s": now_s,
                                "reason": reason, "errors": errors})
        self._held = True
        if errors:
            self.sync.abort("local_stop_failed", now_s=now_s)
            raise RuntimeError("local stop failed: " + json.dumps(errors))

    def tick(self, now_s: float) -> dict:
        """Run before each physics step, including while reports are absent."""
        status = self.sync.status(now_s=now_s)
        context = (status["run_id"], status["plan_version"], status["stage"], status["epoch"])
        if status["phase"] != "GO" or context != self._last_context:
            self._stop_all(now_s, status["reason"])
        self._last_context = context
        try:
            for port in self.ports.values():
                port.tick(now_s)
        except Exception:
            self.hold("local_tick_failed", now_s=now_s)
            raise
        return status

    def capture(self, robot_id: str, *, own_rgb: bytes, top_rgb: bytes, now_s: float) -> dict:
        """Persist the exact permitted producer input. No world/state is accepted."""
        status = self.tick(now_s)
        if robot_id not in self.ports:
            raise ValueError("unknown endpoint")
        self._requests += 1
        request_id = f"{robot_id}-{self._requests:06d}"
        images = {}
        for label, data in (("own_rgb", own_rgb), ("top_rgb", top_rgb)):
            if not isinstance(data, bytes) or not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
                self.hold("invalid_rgb_packet", now_s=now_s)
                raise ValueError("original JPEG bytes required")
            relative = f"rgb/{request_id}-{label}.jpg"
            (self.output / relative).write_bytes(data)
            images[label] = {"ref": relative, "sha256": hashlib.sha256(data).hexdigest(),
                             "jpeg_base64": base64.b64encode(data).decode("ascii")}
        request = {"schema": "ugrp.stage_camera_request.v1", "request_id": request_id,
                   "robot_id": robot_id, "task_id": status["task_id"], "run_id": status["run_id"],
                   "plan_version": status["plan_version"], "stage": status["stage"],
                   "epoch": status["epoch"], "observed_at_s": now_s, "images": images,
                   "plan": self.sync.plan.to_dict(),
                   "own_issued_commands": copy.deepcopy(self._history[robot_id][-16:])}
        self._pending[robot_id] = copy.deepcopy(request)
        _write(self.output / f"{request_id}-request.json", request)
        return copy.deepcopy(request)

    def receive(self, robot_id: str, reply: dict, *, now_s: float) -> bool:
        """Bind a producer reply to its actual capture/context, not fresh timestamps."""
        self.tick(now_s)
        request = self._pending.get(robot_id)
        try:
            # Enforce a real JSON boundary and reject unexpected state/metadata.
            reply = json.loads(json.dumps(reply, allow_nan=False))
            fields = {"request_id", "status", "confidence", "checks", "command_id", "reason", "decided_at_s"}
            if not isinstance(reply, dict) or set(reply) != fields:
                raise ValueError("invalid producer reply fields")
            if not request or reply["request_id"] != request["request_id"]:
                raise ValueError("unknown or superseded camera request")
            _write(self.output / (request["request_id"] + "-reply.json"), reply)
            self._pending.pop(robot_id)
            self._sequence[robot_id] += 1
            report = StageReport.from_dict({
                **{k: request[k] for k in ("task_id", "run_id", "plan_version", "stage", "epoch", "robot_id", "observed_at_s")},
                **{k: reply[k] for k in ("status", "confidence", "checks", "command_id", "reason", "decided_at_s")},
                "schema": "ugrp.stage_report.v1", "evidence_source": "own_rgb+top_rgb",
                "sequence": self._sequence[robot_id], "sent_at_s": now_s,
                "observation_id": request["request_id"],
                "own_rgb_ref": request["images"]["own_rgb"]["ref"],
                "top_rgb_ref": request["images"]["top_rgb"]["ref"]})
            accepted = self.sync.receive(robot_id, report.to_dict(), now_s=now_s)
        except (TypeError, ValueError) as exc:
            accepted = False
            self.events.append({"event": "PRODUCER_REPLY_REJECTED", "robot_id": robot_id,
                                "timestamp_s": now_s, "reason": str(exc)})
        if not accepted:
            self.hold("producer_reply_rejected:" + robot_id, now_s=now_s)
        self.tick(now_s)
        return accepted

    def authorize(self, *, now_s: float) -> dict:
        self.tick(now_s)
        return self.sync.authorize(now_s=now_s)

    def dispatch_pair(self, permission: dict | None, commands: dict, *, now_s: float) -> bool:
        """Validate both commands before issuing either; stop both on partial failure.

        Submissions share a frozen SIM instant: no physics step occurs between
        them. This does not guarantee simultaneous actuation on physical robots.
        """
        self.tick(now_s)
        try:
            commands = json.loads(json.dumps(commands, allow_nan=False))
            if not isinstance(commands, dict) or set(commands) != set(self.ports):
                raise ValueError("one bounded command per participant required")
            for rid, command in commands.items():
                if not isinstance(command, dict) or set(command) != {"command_id", "action", "duration_s"}:
                    raise ValueError("invalid command envelope")
                self.ports[rid].validate_bounded(command["action"], command["duration_s"])
                kind = command["action"]["kind"]
                allowed = {"drive", "mecanum", "wait"} if self.sync.stage == "CARRY" else {"arm", "wait"}
                if kind not in allowed:
                    raise ValueError("action type does not belong to the current stage")
        except (TypeError, ValueError):
            self.hold("invalid_command_batch", now_s=now_s)
            raise
        try:
            for rid, command in commands.items():
                accepted = self.sync.dispatch(rid, permission, command["command_id"],
                    now_s=now_s, duration_s=command["duration_s"],
                    submit=lambda r=rid, c=command: self.ports[r].apply_bounded(c["action"], now_s, c["duration_s"]))
                if not accepted:
                    self.hold("command_batch_denied", now_s=now_s)
                    return False
                self._held = False
                self._history[rid].append({**command, "stage": self.sync.stage,
                    "plan_version": self.sync.plan.plan_version, "issued_at_s": now_s,
                    "meaning": "issued command, not measured state or execution success"})
                self.events.append({"event": "LOCAL_COMMAND", "robot_id": rid,
                                    "timestamp_s": now_s, **copy.deepcopy(command)})
        except Exception:
            self.hold("local_submission_failed", now_s=now_s)
            raise
        return True

    def hold(self, reason: str, *, now_s: float) -> None:
        self.sync.hold(reason, now_s=now_s)
        self._stop_all(now_s, reason)

    def advance(self, *, now_s: float) -> bool:
        self.tick(now_s)
        advanced = self.sync.advance(now_s=now_s)
        self.tick(now_s)
        return advanced

    def update_plan(self, plan: TaskPlan, *, map_path: Path, now_s: float) -> None:
        if hashlib.sha256(map_path.read_bytes()).hexdigest() != plan.map_sha256:
            self.hold("replacement_map_hash_mismatch", now_s=now_s)
            raise ValueError("replacement static map mismatch")
        self.sync.update_plan(plan, now_s=now_s)
        self.tick(now_s)
        _write(self.output / f"plan-{plan.plan_version}.json", plan.to_dict())
        (self.output / f"static-map-{plan.plan_version}.json").write_bytes(map_path.read_bytes())

    def close(self, *, now_s: float) -> None:
        """Release only this adapter's actuators; preserve all execution evidence."""
        try:
            self._stop_all(now_s, "execution_closed")
            self.sync.abort("execution_closed", now_s=now_s)
        finally:
            _write(self.output / "events.json", {"sync": self.sync.events, "execution": self.events})
