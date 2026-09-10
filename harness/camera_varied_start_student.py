"""Staged RGB-only controller for bounded varied robot starts."""
from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from harness.camera_recovery_student import _compact, _select_kernel
from harness.camera_teacher_student import OWN_SIZE, TOP_SIZE, _decode_gray, _view_features


SCHEMA = "ugrp.rgb_varied_start_stage_kernel.v1"
STAGES = ("yaw", "lateral", "forward")
MAX_COMPONENTS = 32
READY_SCORE = .65
READY_COMMAND = .003
COMMAND_BOUNDS = {"yaw": (-.06, .06), "lateral": (-.06, .06),
                  "forward": (-.05, .15)}
# Pixel calibration on the fixed 960x720 top camera. r1 is the lower lane.
TOP_ROIS = {"r1": (250, 345, 580, 550), "r3": (250, 160, 580, 345)}
TOP_REFERENCE_SIZE = (960, 720)


def _finite_command(value: Any, stage: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("command must be a finite stage-bounded number")
    result = float(value)
    low, high = COMMAND_BOUNDS[stage]
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError("command must be a finite stage-bounded number")
    return result


def _encode(own_jpeg: bytes, top_jpeg: bytes, rid: str) -> np.ndarray:
    if rid not in TOP_ROIS:
        raise ValueError("robot_id must be r1 or r3")
    own = _decode_gray(own_jpeg, OWN_SIZE, "own_jpeg")
    if not isinstance(top_jpeg, bytes) or len(top_jpeg) < 4:
        raise ValueError("top_jpeg must be non-empty JPEG bytes")
    top_raw = cv2.imdecode(np.frombuffer(top_jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    if top_raw is None or min(top_raw.shape[:2]) < 2:
        raise ValueError("top_jpeg is not a decodable image")
    height, width = top_raw.shape[:2]
    x1, y1, x2, y2 = TOP_ROIS[rid]
    ref_width, ref_height = TOP_REFERENCE_SIZE
    sx, sy = width / ref_width, height / ref_height
    left, right = round(x1 * sx), round(x2 * sx)
    top_y, bottom = round(y1 * sy), round(y2 * sy)
    if not (0 <= left < right <= width and 0 <= top_y < bottom <= height):
        raise ValueError("fixed top ROI is outside the decoded image")
    top = cv2.resize(top_raw[top_y:bottom, left:right], TOP_SIZE,
                     interpolation=cv2.INTER_AREA).astype(np.float64) / 255.0
    encoded = np.concatenate((_view_features(own, 1.0), _view_features(top, 2.0)))
    return _compact(encoded)


def _sample(sample: Any, stage: str) -> tuple[bytes, bytes, float, bool, str]:
    required = ("own_jpeg", "top_jpeg", "command", "ready", "case_id")
    if not isinstance(sample, dict) or not all(key in sample for key in required):
        raise ValueError("stage sample requires RGB, command, ready, and case_id")
    ready, case_id = sample["ready"], sample["case_id"]
    if not isinstance(ready, bool) or not isinstance(case_id, str) or not case_id:
        raise ValueError("stage sample ready/case_id is invalid")
    return (sample["own_jpeg"], sample["top_jpeg"],
            _finite_command(sample["command"], stage), ready, case_id)


def fit_stage_model(reference_own: bytes, reference_top: bytes,
                    samples: list[dict[str, Any]], rid: str, stage: str,
                    *, domain_samples: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    if rid not in TOP_ROIS or stage not in STAGES:
        raise ValueError("unsupported robot or varied-start stage")
    if not isinstance(samples, list) or not samples:
        raise ValueError("samples must be a non-empty list")
    reference = _encode(reference_own, reference_top, rid)
    features, targets, groups = [reference], [[0.0, 1.0]], []
    scale = max(abs(value) for value in COMMAND_BOUNDS[stage])
    for row in samples:
        own, top, command, ready, case_id = _sample(row, stage)
        features.append(_encode(own, top, rid))
        targets.append([command / scale, float(ready)])
        groups.append(case_id)
    delta = np.stack(features) - reference
    target_array = np.asarray(targets, dtype=np.float64)
    _, singular, vt = np.linalg.svd(delta, full_matrices=False)
    tolerance = max(delta.shape) * np.finfo(float).eps * (singular[0] if len(singular) else 0.0)
    rank = int(np.sum(singular > tolerance))
    if rank < 1:
        raise ValueError("stage RGB samples contain no observable variation")
    components = vt[:min(MAX_COMPONENTS, rank)]
    coordinates = delta @ components.T
    regression_residuals = np.linalg.norm(delta - coordinates @ components, axis=1)
    median = float(np.median(regression_residuals))
    mad = float(np.median(np.abs(regression_residuals - median)))
    domain = samples if domain_samples is None else domain_samples
    if not isinstance(domain, list) or not domain:
        raise ValueError("domain_samples must be a non-empty list")
    domain_features = []
    for row in domain:
        own, top, _command, _ready, _case_id = _sample(row, stage)
        domain_features.append(_encode(own, top, rid))
    domain_delta = np.stack(domain_features) - reference
    domain_coordinates = domain_delta @ components.T
    domain_residuals = np.linalg.norm(domain_delta - domain_coordinates @ components, axis=1)
    residual_limit = max(1e-8, float(np.max(domain_residuals)),
                         median + 4 * max(mad, 1e-8)) * 1.25
    bandwidth, regularization, alpha, cv_mse, selection = _select_kernel(
        coordinates, target_array, groups)
    return {"schema": SCHEMA, "robot_id": rid, "stage": stage,
            "runtime_inputs": ["own_rgb", "fixed_top_rgb"],
            "top_roi": {"reference_size": list(TOP_REFERENCE_SIZE),
                        "xyxy": list(TOP_ROIS[rid])},
            "reference_compact": reference.tolist(),
            "pca_components": components.tolist(),
            "support_coordinates": coordinates.tolist(),
            "kernel_alpha_scaled": alpha.tolist(), "bandwidth": bandwidth,
            "regularization": regularization, "pca_residual_limit": residual_limit,
            "settings": {"command_bounds": list(COMMAND_BOUNDS[stage]),
                         "command_scale": scale, "ready_score": READY_SCORE,
                         "ready_command_max": READY_COMMAND},
            "diagnostics": {"sample_count": len(samples), "support_count": len(coordinates),
                "case_count": len(set(groups)), "components": len(components),
                "feature_rank": rank, "hyperparameter_selection": selection,
                "cross_validation_mse_scaled_outputs": cv_mse,
                "residual_domain_sample_count": len(domain),
                "residual_domain_max": float(np.max(domain_residuals)),
                "residual_limit_source": "all_validated_stage_training_rgb"}}


def predict_stage(model: dict[str, Any], own_jpeg: bytes, top_jpeg: bytes) -> dict[str, Any]:
    if not isinstance(model, dict) or model.get("schema") != SCHEMA:
        raise ValueError("unsupported varied-start stage model")
    rid, stage = model.get("robot_id"), model.get("stage")
    if rid not in TOP_ROIS or stage not in STAGES:
        raise ValueError("invalid varied-start model robot/stage")
    if model.get("top_roi") != {"reference_size": list(TOP_REFERENCE_SIZE),
                                "xyxy": list(TOP_ROIS[rid])}:
        raise ValueError("model top ROI differs from fixed robot calibration")
    reference = np.asarray(model.get("reference_compact"), dtype=np.float64)
    components = np.asarray(model.get("pca_components"), dtype=np.float64)
    support = np.asarray(model.get("support_coordinates"), dtype=np.float64)
    alpha = np.asarray(model.get("kernel_alpha_scaled"), dtype=np.float64)
    bandwidth, limit = model.get("bandwidth"), model.get("pca_residual_limit")
    settings = model.get("settings")
    if (reference.ndim != 1 or components.ndim != 2 or components.shape[1:] != reference.shape
            or not 1 <= len(components) <= MAX_COMPONENTS or support.shape != (len(alpha), len(components))
            or alpha.shape != (len(support), 2) or not isinstance(settings, dict)
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
                   for v in (bandwidth, limit))
            or not all(np.all(np.isfinite(v)) for v in (reference, components, support, alpha))):
        raise ValueError("invalid varied-start model arrays or settings")
    difference = _encode(own_jpeg, top_jpeg, rid) - reference
    coordinate = difference @ components.T
    orthogonal = float(np.linalg.norm(difference - coordinate @ components))
    distances = np.linalg.norm(support - coordinate, axis=1)
    nearest = float(np.min(distances))
    weights = np.exp(-.5 * distances * distances / (float(bandwidth) ** 2))
    effective = float(weights.sum() ** 2 / max(1e-12, np.sum(weights * weights)))
    diagnostics = {"nearest_support_distance": nearest, "orthogonal_residual": orthogonal,
                   "effective_support": effective, "training_case_count": model["diagnostics"].get("case_count")}
    if nearest > 2 * bandwidth or orthogonal > limit or effective < 1.25:
        return {"ok": False, "command": 0.0, "ready_score": 0.0, "ready": False,
                "reason": "rgb_state_outside_stage_support", "diagnostics": diagnostics}
    prediction = weights @ alpha
    low, high = settings["command_bounds"]
    command = float(np.clip(prediction[0] * settings["command_scale"], low, high))
    ready_score = float(np.clip(prediction[1], 0.0, 1.0))
    ready = ready_score >= settings["ready_score"] and abs(command) <= settings["ready_command_max"]
    return {"ok": True, "command": command, "ready_score": ready_score,
            "ready": bool(ready), "reason": "learned_stage_ready" if ready else "learned_stage_command",
            "diagnostics": diagnostics}
