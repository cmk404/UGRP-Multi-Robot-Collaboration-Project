"""Learned visual alignment errors mapped to calibrated wheel commands.

Teacher state is accepted during fitting only. Runtime receives RGB bytes and
saved coefficients; the returned error is an image-derived estimate.
"""
from __future__ import annotations

import math

from harness.camera_varied_start_student import COMMAND_BOUNDS, MIN_MOVING_COMMAND, STAGES

SCHEMA = "ugrp.rgb_varied_start_pose.v1"
GAINS = {"yaw": .3, "lateral": .35, "forward": .2}
# A separate setup-only physical probe retained full RGB grasp success at
# headings +/-0.25, 0.5, 0.75, and 1 degree, with 3 mm forward/2 mm lateral
# offsets (8/8). This 0.344-degree visual stop band leaves estimation margin.
TOLERANCES = {"yaw": .006, "lateral": .0025, "forward": .004}


def fit_pose_stage_model(reference_own, reference_top, samples, rid, stage,
                         *, domain_samples=None):
    from harness.camera_varied_start_geometry import fit_geometry_model
    if stage not in STAGES:
        raise ValueError("unsupported stage")
    if stage == "yaw":
        from harness.camera_varied_start_heading import fit_heading_model
        fit = fit_heading_model
    else:
        fit = fit_geometry_model
    rows = samples if domain_samples is None else domain_samples
    geometry = fit(reference_top, rows, rid, stage)
    return {"schema": SCHEMA, "robot_id": rid, "stage": stage,
            "runtime_inputs": ["fixed_top_rgb"], "geometry": geometry,
            "settings": {"gain": GAINS[stage], "tolerance": TOLERANCES[stage],
                         "command_bounds": list(COMMAND_BOUNDS[stage]),
                         "minimum_moving_command": MIN_MOVING_COMMAND},
            "diagnostics": geometry["diagnostics"]}


def predict_pose_stage(model, own_jpeg, top_jpeg):
    from harness.camera_varied_start_geometry import predict_geometry
    stage = model.get("stage")
    if (model.get("schema") != SCHEMA or stage not in STAGES
            or model.get("robot_id") not in ("r1", "r3")):
        raise ValueError("invalid RGB pose model")
    expected = {"gain": GAINS[stage], "tolerance": TOLERANCES[stage],
                "command_bounds": list(COMMAND_BOUNDS[stage]),
                "minimum_moving_command": MIN_MOVING_COMMAND}
    if model.get("settings") != expected:
        raise ValueError("invalid pose control calibration")
    geometry = model.get("geometry")
    if (not isinstance(geometry, dict) or geometry.get("robot_id") != model["robot_id"]
            or geometry.get("stage") != stage):
        raise ValueError("pose geometry identity mismatch")
    if stage == "yaw":
        from harness.camera_varied_start_heading import predict_heading
        result = predict_heading(geometry, top_jpeg)
    else:
        result = predict_geometry(geometry, top_jpeg)
    if not result["ok"]:
        return {"ok": False, "command": 0., "ready_score": 0., "ready": False,
                "reason": "rgb_pose_outside_support", "diagnostics": result["diagnostics"]}
    error = float(result["error"])
    if not math.isfinite(error):
        raise ValueError("nonfinite image-derived alignment error")
    ready = abs(error) <= TOLERANCES[stage]
    command = 0. if ready else math.copysign(max(MIN_MOVING_COMMAND, abs(error * GAINS[stage])), error)
    low, high = COMMAND_BOUNDS[stage]
    command = max(low, min(high, command))
    return {"ok": True, "command": command, "ready_score": float(ready), "ready": ready,
            "precision": result.get("precision", "fine"),
            "reason": "visual_pose_ready" if ready else "visual_pose_command",
            "diagnostics": {**result["diagnostics"], "image_derived_error": error}}
