"""Opt-in far-field command schedule for the paired fine RGB alignment stages.

The saved pose models map an image-derived alignment error to a small
proportional wheel command (gain .2-.35, floor .01). Far from the target this
corrects only a few percent of the error per .2 s slice. The schedule raises
the command far from the target and hands back to the unchanged model command
well before it.

Version 2 follows a failed synchronous cohort (protocol-v2 B1): the earlier
cap ignored motion still owed to the previous command, the robot overshot, and
beyond the target the pose estimate was not monotonic. This version

* subtracts an upper bound of the previous command's carry-over motion,
* sizes the command so its own upper-bound travel (this slice plus carry-over)
  stops at least ``HANDOVER_TOLERANCES`` outer tolerances before the target,
* returns the model decision unchanged inside that hand-over distance, and
* is latched off for the rest of a stage once the error sign flips.

Response constants are least-squares fits of image-derived error change per
unit command over two recorded synchronous runs (protocol-v2 A1/B1:
forward .021 current + .156 previous; lateral .006 + .068; yaw .002 + .118),
inflated by 1.5 for the upper bound. They are a controller calibration from
RGB estimates only, valid for synchronous execution, not a physical model.
"""
from __future__ import annotations

import math

from harness.camera_varied_start_pose_student import SCHEMA as POSE_SCHEMA, TOLERANCES
from harness.camera_varied_start_student import COMMAND_BOUNDS

SCHEMA = "ugrp.fine_gain_schedule.v2"
HANDOVER_TOLERANCES = 8.
# Upper-bound error reduction per unit command: total over the issuing and the
# following slice, and the part still owed by the previous command.
TOTAL_RESPONSE_HIGH = {"yaw": .18, "lateral": .11, "forward": .27}
CARRY_RESPONSE_HIGH = {"yaw": .18, "lateral": .10, "forward": .24}


def settings() -> dict:
    return {"schema": SCHEMA, "handover_tolerances": HANDOVER_TOLERANCES,
            "total_response_high": dict(TOTAL_RESPONSE_HIGH),
            "carry_response_high": dict(CARRY_RESPONSE_HIGH),
            "tolerances": dict(TOLERANCES), "execution": "synchronous only",
            "command_bounds": {k: list(v) for k, v in COMMAND_BOUNDS.items()}}


def image_error(decision: dict) -> float | None:
    if decision.get("ok") is not True:
        return None
    try:
        error = float(decision["diagnostics"]["image_derived_error"])
    except (KeyError, TypeError, ValueError):
        return None
    return error if math.isfinite(error) else None


def schedule_decision(model: dict, decision: dict, *, previous_command: float = 0.,
                      latched: bool = False) -> dict:
    """Return a copy with a far-field command; keep the model output for audit.

    ``previous_command`` is this robot's own last issued command on the same
    axis in the current stage (zero after a stationary slice).
    """
    stage = model.get("stage")
    if model.get("schema") != POSE_SCHEMA or stage not in TOLERANCES:
        raise ValueError("fine gain schedule requires a visual pose stage model")
    result = dict(decision)
    result["model_command"] = decision.get("command")
    result["gain_schedule"] = {"schema": SCHEMA, "applied": False, "latched": bool(latched)}
    error = image_error(decision)
    if (latched or error is None or decision.get("ready") is not False
            or decision.get("precision", "fine") != "fine"):
        return result
    try:
        command = float(decision["command"])
        previous = float(previous_command)
    except (KeyError, TypeError, ValueError):
        return result
    if not math.isfinite(command) or not math.isfinite(previous):
        return result
    if command == 0. or math.copysign(1., command) != math.copysign(1., error):
        # A model command against its own error is not ours to reinterpret.
        return result
    tolerance = TOLERANCES[stage]
    handover = HANDOVER_TOLERANCES * tolerance
    # Motion toward the target still owed by the previous command.
    owed = max(0., math.copysign(1., error) * previous) * CARRY_RESPONSE_HIGH[stage]
    room = abs(error) - owed - handover
    info = {"schema": SCHEMA, "applied": False, "latched": False, "image_derived_error": error,
            "previous_command": previous, "owed_high": owed, "handover": handover, "room": room}
    result["gain_schedule"] = info
    if room <= 0.:
        return result
    low, high = COMMAND_BOUNDS[stage]
    bound = high if error > 0 else -low
    magnitude = max(abs(command), min(room / TOTAL_RESPONSE_HIGH[stage], bound))
    scheduled = math.copysign(magnitude, error)
    result["command"] = scheduled
    info.update(applied=scheduled != command, bound=bound)
    return result


class StageScheduler:
    """Per-stage state: own previous command and the sign-flip latch."""

    def __init__(self, robots):
        self.previous = {r: 0. for r in robots}
        self.sign = {r: None for r in robots}
        self.latched = {r: False for r in robots}

    def decide(self, rid, model, decision):
        error = image_error(decision)
        if (error is not None and error != 0. and self.sign[rid] is not None
                and math.copysign(1., error) != self.sign[rid]):
            self.latched[rid] = True
        out = schedule_decision(model, decision, previous_command=self.previous[rid],
                                latched=self.latched[rid])
        if out["gain_schedule"].get("applied") and self.sign[rid] is None:
            self.sign[rid] = math.copysign(1., error)
        return out

    def issued(self, rid, command):
        self.previous[rid] = float(command)
