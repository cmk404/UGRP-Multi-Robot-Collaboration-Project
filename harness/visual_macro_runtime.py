"""Nonblocking execution of visual-skill macros on one camera robot port."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from typing import Any


class VisualMacroExecutor:
    """Schedule one robot's raw actions without owning or stepping physics."""

    def __init__(
        self,
        port: Any,
        log_callback: Callable[[dict[str, Any]], None] | None = None,
        drive_guard: Callable[[Mapping[str, Any], float], Mapping[str, Any]] | None = None,
    ):
        self.port = port
        self._log_callback = log_callback
        self._drive_guard = drive_guard
        self._events: list[tuple[float, dict[str, Any] | None]] = []
        self._completion_time: float | None = None
        self._source_hash = ""
        self._macro: dict[str, Any] | None = None
        self._last_tick: float | None = None
        self.last_interruption: dict[str, Any] | None = None
        self.last_execution: dict[str, Any] | None = None
        self.total_drive_control_s = 0.0
        self._macro_drive_control_s = 0.0
        self._lease_end: float | None = None
        self._lease_accounted_until: float | None = None
        self._macro_started_at = 0.0

    @property
    def idle(self) -> bool:
        return self._completion_time is None

    def submit(
        self,
        action: Mapping[str, Any],
        observation: Mapping[str, Any],
        phase: str,
        now: float,
    ) -> None:
        """Validate and schedule a drive, pose, wait, or safe finish macro."""
        timestamp = _finite("now", now)
        if not self.idle:
            raise RuntimeError("VISUAL_MACRO_BUSY")
        self.last_interruption = None
        if not isinstance(action, Mapping):
            raise ValueError("visual macro must be an object")
        if not isinstance(observation, Mapping):
            raise ValueError("observation must be an object")
        source_hash = observation.get("sha256")
        if not isinstance(source_hash, str) or not source_hash:
            raise ValueError("observation sha256 is required")
        kind = action.get("kind")
        if kind not in {"drive", "pose", "wait", "finish"}:
            raise ValueError("UNKNOWN_SKILL_MACRO")

        requested = dict(action)
        events: list[tuple[float, dict[str, Any] | None]] = []
        if kind == "drive":
            _exact_fields(action, {"kind", "fwd", "turn", "duration"})
            duration = _finite("duration", action["duration"])
            upper = 4.0 if self._drive_guard is not None else 1.0
            if duration < 0.0 or duration > upper or (self._drive_guard is not None and duration == 0.0):
                raise ValueError("duration is outside allowed range")
            forward = _bounded("fwd", action["fwd"], 0.0, 0.15)
            turn = _bounded("turn", action["turn"], -0.2, 0.2)
            if self._drive_guard is None:
                events.append((timestamp, {
                    "kind": "drive", "forward": forward, "turn": turn,
                    "duration_s": duration,
                }))
            else:
                offset = 0.0
                while offset < duration - 1e-12:
                    slice_duration = min(0.25, duration - offset)
                    events.append((timestamp + offset, {
                        "kind": "drive", "forward": forward, "turn": turn,
                        "duration_s": slice_duration,
                    }))
                    offset += slice_duration
            completion = timestamp + duration + 0.2
            events.append((completion, None))
        elif kind == "pose":
            _exact_fields(action, {"kind", "pulses"})
            targets = _pose_targets(action["pulses"])
            actuator = observation.get("actuator_state")
            starts_raw = actuator.get("servo_pulses") if isinstance(actuator, Mapping) else None
            if not isinstance(starts_raw, Mapping):
                raise ValueError("observation servo_pulses are required for pose")
            starts = {}
            for servo in targets:
                value = starts_raw.get(str(servo))
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError(f"observation missing own PWM for servo {servo}")
                starts[servo] = value
            delta = max((abs(targets[s] - starts[s]) for s in targets), default=0)
            duration = max(0.25, delta / 600.0)
            count = max(5, math.ceil(duration / 0.05))
            interval = duration / count
            for sample in range(1, count + 1):
                u = sample / count
                ease = u * u * (3.0 - 2.0 * u)
                for servo, end in targets.items():
                    pulse = round(starts[servo] + ease * (end - starts[servo]))
                    raw = (
                        {"kind": "look", "pan_pulse": pulse}
                        if servo == 6 else
                        {"kind": "arm", "servo_id": servo, "pulse": pulse}
                    )
                    events.append((timestamp + sample * interval, raw))
            settle = (0.5 if str(phase) in {"verify_lift", "attachment_left", "attachment_right", "attachment_home"}
                      else 0.3 if str(phase) == "approach" else 0.08)
            completion = timestamp + duration + settle
        elif kind == "wait":
            _exact_fields(action, {"kind", "duration"})
            duration = _bounded("duration", action["duration"], 0.0, math.inf)
            events.append((timestamp, {"kind": "wait"}))
            completion = timestamp + max(0.05, duration)
        else:
            if set(action) - {"kind", "reason"}:
                raise ValueError("unknown finish action fields")
            self.port.stop()
            self._emit("explicit_stop", timestamp, source_hash, requested)
            self._emit("macro_finished", timestamp, source_hash, requested)
            return

        self._source_hash = source_hash
        self._macro_started_at = timestamp
        self._macro_drive_control_s = 0.0
        self._macro = requested
        self._events = events
        self._completion_time = completion
        self._emit("macro_submitted", timestamp, source_hash, requested)
        self._dispatch_due(timestamp)

    def cancel(self, now: float, reason: str) -> None:
        """Stop the port and discard every pending action in the active macro."""
        timestamp = _finite("now", now)
        if not isinstance(reason, str) or not reason:
            raise ValueError("cancel reason is required")
        self._accrue_drive(timestamp)
        if self.idle:
            self._events = []
            self.port.stop()
            return
        self._interrupt(timestamp, reason, {"source": "cancel"})

    def tick(self, now: float) -> None:
        """Advance this executor at shared simulated time; never step physics."""
        timestamp = _finite("now", now)
        if self._last_tick is not None and timestamp < self._last_tick:
            raise ValueError("now must not move backwards")
        self._last_tick = timestamp
        self._accrue_drive(timestamp)
        self.port.tick(timestamp)
        if self.idle:
            return
        self._dispatch_due(timestamp)
        if self._completion_time is not None and timestamp + 1e-12 >= self._completion_time:
            macro = self._macro or {}
            if macro.get("kind") == "drive":
                self.port.stop()
                self._emit("explicit_stop", timestamp, self._source_hash, macro)
            self.last_execution = self._execution_summary(timestamp, "completed")
            self._emit("macro_finished", timestamp, self._source_hash, macro,
                       details={"execution": self.last_execution})
            self._events = []
            self._completion_time = None
            self._macro = None
            self._source_hash = ""

    def _dispatch_due(self, now: float) -> None:
        pending = []
        guarded_drive_dispatched = False
        for scheduled, raw in self._events:
            if scheduled > now + 1e-12:
                pending.append((scheduled, raw))
                continue
            if raw is not None:
                if self._drive_guard is not None and (self._macro or {}).get("kind") == "drive":
                    # A slightly late shared-physics tick may use only the
                    # remainder of the current lease window. Fully elapsed
                    # windows are discarded and at most one slice is launched
                    # per tick, so delayed ticks cannot replay a burst.
                    interval_end = scheduled + float(raw["duration_s"])
                    if interval_end <= now + 1e-12 or guarded_drive_dispatched:
                        continue
                    try:
                        verdict = self._drive_guard(dict(self._macro or {}), now)
                    except Exception as exc:
                        self._interrupt(now, "DRIVE_GUARD_ERROR", {
                            "error": f"{type(exc).__name__}: {exc}",
                        })
                        return
                    if not isinstance(verdict, Mapping) or not isinstance(verdict.get("allowed"), bool):
                        self._interrupt(now, "INVALID_DRIVE_GUARD_RESULT", {"verdict": verdict})
                        return
                    if not verdict["allowed"]:
                        reason = verdict.get("reason")
                        if not isinstance(reason, str) or not reason:
                            reason = "DRIVE_GUARD_REJECTED"
                        evidence = verdict.get("evidence", verdict)
                        self._interrupt(now, reason, evidence)
                        return
                    raw = dict(raw)
                    raw["duration_s"] = min(float(raw["duration_s"]), interval_end - now)
                    guarded_drive_dispatched = True
                # A shared runner can tick just after a non-grid sample time.
                # The port clock was already advanced to ``now`` above, so a
                # command must use that same actual time rather than backdate
                # itself to the ideal schedule.
                self.port.apply(raw, now)
                if raw.get("kind") == "drive":
                    self._accrue_drive(now)
                    self._lease_accounted_until = now
                    self._lease_end = now + float(raw["duration_s"])
                self._emit("raw_action", now, self._source_hash,
                           self._macro or {}, raw, scheduled_time=scheduled)
        self._events = pending

    def _interrupt(self, now: float, reason: str, evidence: Any) -> None:
        macro = dict(self._macro or {})
        interruption = {
            "time": float(now),
            "reason": reason,
            "evidence": evidence,
            "source_frame_sha256": self._source_hash,
            "macro": macro,
        }
        self._accrue_drive(now)
        self._lease_end = self._lease_accounted_until = None
        self.last_interruption = interruption
        self.last_execution = self._execution_summary(now, "interrupted", reason)
        self.port.stop()
        self._emit("macro_interrupted", now, self._source_hash, macro,
                   details={"reason": reason, "evidence": evidence,
                            "execution": self.last_execution})
        self._events = []
        self._completion_time = None
        self._macro = None
        self._source_hash = ""

    def _accrue_drive(self, now: float) -> None:
        """Count elapsed applied command leases, not requested or future time."""
        if self._lease_end is None or self._lease_accounted_until is None:
            return
        until = min(now, self._lease_end)
        elapsed = max(0.0, until - self._lease_accounted_until)
        self.total_drive_control_s += elapsed
        self._macro_drive_control_s += elapsed
        self._lease_accounted_until = max(self._lease_accounted_until, until)
        if now >= self._lease_end:
            self._lease_end = self._lease_accounted_until = None

    def _execution_summary(self, now: float, status: str, reason: str | None = None) -> dict[str, Any]:
        return {"scope": "raw_skill_macro", "macro": dict(self._macro or {}),
                "status": status, "reason": reason,
                "requested_duration_s": (self._macro or {}).get("duration"),
                "elapsed_drive_control_s": round(self._macro_drive_control_s, 6),
                "started_at": self._macro_started_at, "ended_at": now,
                "motion_confirmed": False}

    def _emit(self, event: str, now: float, source_hash: str,
              macro: Mapping[str, Any], raw: Mapping[str, Any] | None = None,
              scheduled_time: float | None = None,
              details: Mapping[str, Any] | None = None) -> None:
        if self._log_callback is None:
            return
        item = {
            "event": event,
            "time": float(now),
            "robot_id": str(getattr(self.port, "robot_id", "")),
            "source_frame_sha256": source_hash,
            "macro": dict(macro),
        }
        if raw is not None:
            item["raw_action"] = dict(raw)
        if scheduled_time is not None:
            item["scheduled_time"] = float(scheduled_time)
        if details is not None:
            item.update(details)
        self._log_callback(item)


def _finite(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _bounded(name: str, value: Any, lower: float, upper: float) -> float:
    result = _finite(name, value)
    if not lower <= result <= upper:
        raise ValueError(f"{name} is outside allowed range")
    return result


def _exact_fields(action: Mapping[str, Any], fields: set[str]) -> None:
    if set(action) != fields:
        raise ValueError(f"invalid {action.get('kind')} action fields")


def _pose_targets(value: Any) -> dict[int, int]:
    if not isinstance(value, Mapping) or not value:
        raise ValueError("pose pulses must be a nonempty object")
    targets = {}
    for raw_servo, pulse in value.items():
        if isinstance(raw_servo, bool):
            raise ValueError("invalid pose servo")
        try:
            servo = int(raw_servo)
        except (TypeError, ValueError):
            raise ValueError("invalid pose servo") from None
        if servo not in {1, 3, 4, 5, 6} or isinstance(pulse, bool) or not isinstance(pulse, int):
            raise ValueError("invalid pose servo or pulse")
        if not 500 <= pulse <= 2500:
            raise ValueError("pose pulse is outside allowed range")
        targets[servo] = pulse
    return targets


__all__ = ["VisualMacroExecutor"]
