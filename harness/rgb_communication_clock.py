"""Supervisor-only clock evidence and redacted exception metadata.

Requested ticks are not evidence that physics advanced. This callback never
enters actor inputs, scheduling decisions, or physical success assessment.
"""
from __future__ import annotations

import math


_CLOCK_FIELDS = {"schema", "clock_domain", "last_acknowledged_time_s", "actual_time_s"}
_ERROR_TYPES = {"Exception", "RuntimeError", "ValueError", "TypeError", "OSError",
                "TimeoutError", "ConnectionError", "KeyError", "AssertionError"}
_ERROR_CODES = {"PHYSICS_OWNER_UNAVAILABLE", "SIM_CLOCK_INVALID",
                "SIM_CLOCK_REVERSED", "SIM_CLOCK_MISMATCH"}


def sanitized_error(exc):
    """No exception messages, arguments, arbitrary class names, or tracebacks."""
    chain, seen, relation = [], set(), "primary"
    while exc is not None and id(exc) not in seen and len(chain) < 8:
        seen.add(id(exc))
        name = type(exc).__name__
        code = vars(exc).get("error_code")
        code = code if type(code) is str and code in _ERROR_CODES else "UNCLASSIFIED_ERROR"
        if (type(exc) is ValueError and len(exc.args) == 1 and type(exc.args[0]) is str
                and exc.args[0] == "sim_time must not move backwards"):
            code = "CLOCK_MOVED_BACKWARDS"
        chain.append({"error_type": name if name in _ERROR_TYPES else "Exception",
                      "error_code": code, "relation": relation})
        if exc.__cause__ is not None:
            exc, relation = exc.__cause__, "cause"
        elif not exc.__suppress_context__:
            exc, relation = exc.__context__, "context"
        else:
            exc = None
    return {"error_type": chain[0]["error_type"], "error_code": chain[0]["error_code"],
            "cause_chain": chain, "chain_truncated": exc is not None}


class ExecutionClock:
    def __init__(self, domain, snapshot=None):
        self.domain = domain if domain in ("sim", "monotonic") else "unknown"
        self.snapshot = snapshot
        self.requested = self.last_successful_requested = self.last_ack = None
        self.actual = self._last_verified_actual = None
        self.reason = "CLOCK_SNAPSHOT_UNAVAILABLE"

    def refresh(self):
        self.actual = None
        self.reason = "CLOCK_SNAPSHOT_UNAVAILABLE"
        if self.snapshot is None:
            return
        try:
            value = self.snapshot()
        except Exception:
            self.reason = "CLOCK_SNAPSHOT_FAILED"
            return
        self.reason = "CLOCK_SNAPSHOT_INVALID"
        # An exact, tiny value-only contract, not an evaluator/actor snapshot.
        if (type(value) is not dict or set(value) != _CLOCK_FIELDS
                or value["schema"] != "ugrp.execution_clock.v1"
                or value["clock_domain"] != self.domain):
            return
        ack, actual = value["last_acknowledged_time_s"], value["actual_time_s"]
        try:
            invalid_time = any(v is not None and (type(v) not in (int, float)
                               or not math.isfinite(v) or v < 0) for v in (ack, actual))
        except OverflowError:
            invalid_time = True
        if invalid_time:
            return
        if (ack is not None and actual is not None and ack > actual
                or self.last_ack is not None and (ack is None or ack < self.last_ack)
                or actual is not None and self._last_verified_actual is not None
                and actual < self._last_verified_actual):
            return
        self.last_ack = ack
        self.actual = actual
        if actual is not None:
            self._last_verified_actual = actual
        self.reason = "VERIFIED_CLOCK_SNAPSHOT" if actual is not None else "ACTUAL_CLOCK_UNKNOWN"

    def tick(self, port, requested):
        self.requested = requested
        # Return may contain supervisor truth. Do not read or propagate it.
        port.tick(requested)
        self.last_successful_requested = requested
        if self.snapshot is None:
            self.last_ack = requested  # Logical acknowledgement, NOT actual time.
        self.refresh()

    def close_argument(self, *, failed):
        if self.actual is not None:
            return self.actual
        # Only the successful legacy path retains its old acknowledged hint.
        # None asks the backend to close at its own clock after uncertain work.
        if not failed and self.snapshot is None:
            return self.last_successful_requested
        return None

    @property
    def sim_time(self):
        return self.actual if self.domain == "sim" else None

    def payload(self):
        return {"schema": "rgb-runtime-clock.v1", "clock_domain": self.domain,
                "requested_tick_s": self.requested,
                "last_successful_requested_tick_s": self.last_successful_requested,
                "last_acknowledged_time_s": self.last_ack,
                "terminal_time_s": self.actual,
                "terminal_time_status": "verified" if self.actual is not None else "unknown",
                "reason": self.reason}
