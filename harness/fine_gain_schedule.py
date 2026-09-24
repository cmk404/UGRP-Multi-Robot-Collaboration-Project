"""Opt-in far-field command schedule for the paired fine RGB alignment stages.

The saved pose models map an image-derived alignment error to a small
proportional wheel command (gain .2-.35, floor .01). In the recorded open
dispatch runs this corrected only a few percent of the error per .2 s slice,
so the forward stage spent about 25 SIM seconds on 17 cm. Far from the
existing tolerance this schedule raises the command toward a fixed fraction of
the same image-derived error per slice. It never lowers the model command,
never changes readiness, tolerances, bounds, stationary confirmations, final
alignment checks or dock commands, and returns the model decision unchanged
within ``FAR_TOLERANCE_MULTIPLE`` tolerances of the target.

Response constants come from RGB-derived error changes between consecutive
moving slices of two recorded native v19 runs (no referee or simulator state).
They are a controller calibration, not a measurement of the physical robot.
"""
from __future__ import annotations

import math

from harness.camera_varied_start_pose_student import SCHEMA as POSE_SCHEMA, TOLERANCES
from harness.camera_varied_start_student import COMMAND_BOUNDS

SCHEMA = "ugrp.fine_gain_schedule.v1"
STEP_FRACTION = .35
FAR_TOLERANCE_MULTIPLE = 3.
# Median image-error reduction per unit command in one moving slice. A larger
# value yields a smaller command; lateral had no usable far samples, so it
# shares the forward value instead of its weak .01 deadband response.
NOMINAL_RESPONSE = {"yaw": .12, "lateral": .15, "forward": .15}
# Upper-quartile response: the command may not be expected to cross the outer
# tolerance even if this slice responds as strongly as the recorded p75.
HIGH_RESPONSE = {"yaw": .17, "lateral": .24, "forward": .24}


def settings() -> dict:
    return {"schema": SCHEMA, "step_fraction": STEP_FRACTION,
            "far_tolerance_multiple": FAR_TOLERANCE_MULTIPLE,
            "nominal_response": dict(NOMINAL_RESPONSE), "high_response": dict(HIGH_RESPONSE),
            "tolerances": dict(TOLERANCES),
            "command_bounds": {k: list(v) for k, v in COMMAND_BOUNDS.items()}}


def schedule_decision(model: dict, decision: dict) -> dict:
    """Return a copy with a far-field command; keep the model output for audit."""
    stage = model.get("stage")
    if model.get("schema") != POSE_SCHEMA or stage not in TOLERANCES:
        raise ValueError("fine gain schedule requires a visual pose stage model")
    result = dict(decision)
    result["model_command"] = decision.get("command")
    result["gain_schedule"] = {"schema": SCHEMA, "applied": False}
    if (decision.get("ok") is not True or decision.get("ready") is not False
            or decision.get("precision", "fine") != "fine"):
        return result
    try:
        error = float(decision["diagnostics"]["image_derived_error"])
        command = float(decision["command"])
    except (KeyError, TypeError, ValueError):
        return result
    if not math.isfinite(error) or not math.isfinite(command):
        return result
    tolerance = TOLERANCES[stage]
    if abs(error) <= FAR_TOLERANCE_MULTIPLE * tolerance:
        return result
    if command == 0. or math.copysign(1., command) != math.copysign(1., error):
        # A model command against its own error is not ours to reinterpret.
        return result
    desired = STEP_FRACTION * abs(error) / NOMINAL_RESPONSE[stage]
    no_crossing = (abs(error) - tolerance) / HIGH_RESPONSE[stage]
    low, high = COMMAND_BOUNDS[stage]
    bound = high if error > 0 else -low
    magnitude = max(abs(command), min(desired, no_crossing, bound))
    scheduled = math.copysign(magnitude, error)
    result["command"] = scheduled
    result["gain_schedule"] = {"schema": SCHEMA, "applied": scheduled != command,
                               "image_derived_error": error, "tolerance": tolerance,
                               "desired": desired, "no_crossing_cap": no_crossing,
                               "bound": bound}
    return result
