"""Teacher-labelled straight-wheel approach from own and fixed-top RGB only."""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from harness.camera_recovery_student import MAX_COMPONENTS, _compact, _kernel, _select_kernel
from harness.camera_teacher_student import OWN_SIZE, TOP_SIZE, encode_views


SCHEMA = "ugrp.rgb_straight_approach_kernel.v1"
READY_STOP_SCORE = 0.65
READY_FORWARD = 0.003


def _forward(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("forward must be a finite number in 0..0.15")
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 0.15:
        raise ValueError("forward must be a finite number in 0..0.15")
    return result


def fit_approach_model(reference_own: bytes, reference_top: bytes,
                       samples: list[dict[str, Any]], *,
                       domain_samples: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """Fit RGB-to-forward/stop interpolation with grouped trajectory CV."""
    if not isinstance(samples, list) or not samples:
        raise ValueError("samples must be a non-empty list")
    reference = _compact(encode_views(reference_own, reference_top))
    features, labels, case_ids = [reference], [[0.0, 1.0]], []
    for sample in samples:
        if not isinstance(sample, dict) or not all(key in sample for key in
                ("own_jpeg", "top_jpeg", "forward", "stop", "case_id")):
            raise ValueError("each sample requires RGB, forward, stop, and case_id")
        case_id = sample["case_id"]
        if not isinstance(case_id, str) or not case_id:
            raise ValueError("case_id must be a non-empty string")
        stop = sample["stop"]
        if not isinstance(stop, bool):
            raise ValueError("stop must be boolean")
        features.append(_compact(encode_views(sample["own_jpeg"], sample["top_jpeg"])))
        # Scale velocity to the same numeric range as the binary stop target
        # while selecting kernel hyperparameters.  Runtime converts it back.
        labels.append([_forward(sample["forward"]) / 0.15, float(stop)])
        case_ids.append(case_id)
    delta = np.stack(features) - reference
    targets = np.asarray(labels, dtype=np.float64)
    _, singular, vt = np.linalg.svd(delta, full_matrices=False)
    tolerance = max(delta.shape) * np.finfo(float).eps * (singular[0] if len(singular) else 0.0)
    feature_rank = int(np.sum(singular > tolerance))
    if feature_rank < 1:
        raise ValueError("approach RGB samples contain no observable variation")
    components = vt[:min(MAX_COMPONENTS, feature_rank)]
    coordinates = delta @ components.T
    reconstruction = coordinates @ components
    residuals = np.linalg.norm(delta - reconstruction, axis=1)
    median = float(np.median(residuals));mad = float(np.median(np.abs(residuals - median)))
    domain_residuals = residuals
    if domain_samples is not None:
        if not isinstance(domain_samples, list) or not domain_samples:
            raise ValueError("domain_samples must be a non-empty list when provided")
        domain_features = []
        for sample in domain_samples:
            if not isinstance(sample, dict) or not all(key in sample for key in
                    ("own_jpeg", "top_jpeg", "forward", "stop", "case_id")):
                raise ValueError("each domain sample requires RGB, forward, stop, and case_id")
            _forward(sample["forward"])
            if not isinstance(sample["stop"], bool):
                raise ValueError("domain sample stop must be boolean")
            if not isinstance(sample["case_id"], str) or not sample["case_id"]:
                raise ValueError("domain sample case_id must be a non-empty string")
            domain_features.append(_compact(encode_views(sample["own_jpeg"], sample["top_jpeg"])))
        domain_delta = np.stack(domain_features) - reference
        domain_coordinates = domain_delta @ components.T
        domain_residuals = np.linalg.norm(
            domain_delta - domain_coordinates @ components, axis=1)
    residual_limit = max(1e-8, float(np.max(domain_residuals)),
                         median + 4 * max(mad, 1e-8)) * 1.25
    bandwidth, regularization, alpha, cv_mse, selection = _select_kernel(
        coordinates, targets, case_ids
    )
    arrays = (reference, components, coordinates, alpha, singular, targets)
    if not all(np.all(np.isfinite(value)) for value in arrays):
        raise ValueError("fitted approach model is not finite")
    return {
        "schema": SCHEMA,
        "runtime_inputs": ["own_rgb", "fixed_top_rgb"],
        "feature_spec": {"source": "encode_views", "pool": 4,
                         "components": int(len(components))},
        "reference_compact": reference.tolist(),
        "pca_components": components.tolist(),
        "support_coordinates": coordinates.tolist(),
        "kernel_alpha_scaled": alpha.tolist(),
        "bandwidth": bandwidth, "regularization": regularization,
        "pca_residual_limit": residual_limit,
        "settings": {"ready_stop_score": READY_STOP_SCORE,
                     "ready_forward_max": READY_FORWARD,
                     "max_forward": 0.15},
        "diagnostics": {
            "sample_count": len(samples), "support_count": len(coordinates),
            "case_count": len(set(case_ids)), "goal_anchor_included": True,
            "feature_rank": feature_rank,
            "singular_values": singular[:len(components)].tolist(),
            "hyperparameter_selection": selection,
            "cross_validation_mse_scaled_outputs": cv_mse,
            "representation_cv_limitation": (
                "PCA is fit once on all included training cases; grouped CV refits only kernel regression"
            ),
            "pca_residual_median": median,
            "residual_domain_sample_count": int(len(domain_residuals)),
            "residual_domain_max": float(np.max(domain_residuals)),
            "residual_limit_source": (
                "all_validated_training_rgb" if domain_samples is not None
                else "regression_samples"
            ),
        },
    }


def _validated(model: Any):
    if not isinstance(model, dict) or model.get("schema") != SCHEMA:
        raise ValueError("unsupported approach model schema")
    reference = np.asarray(model.get("reference_compact"), dtype=np.float64)
    components = np.asarray(model.get("pca_components"), dtype=np.float64)
    support = np.asarray(model.get("support_coordinates"), dtype=np.float64)
    alpha = np.asarray(model.get("kernel_alpha_scaled"), dtype=np.float64)
    bandwidth, residual_limit = model.get("bandwidth"), model.get("pca_residual_limit")
    settings, diagnostics = model.get("settings"), model.get("diagnostics")
    compact_size = 2 * (OWN_SIZE[0] // 4) * (OWN_SIZE[1] // 4) + 2 * (TOP_SIZE[0] // 4) * (TOP_SIZE[1] // 4)
    if (reference.shape != (compact_size,) or components.ndim != 2
            or components.shape[1] != compact_size or not 1 <= len(components) <= MAX_COMPONENTS
            or support.ndim != 2 or support.shape[1] != len(components)
            or alpha.shape != (len(support), 2)
            or not isinstance(settings, dict) or not isinstance(diagnostics, dict)
            or isinstance(bandwidth, bool) or not isinstance(bandwidth, (int, float))
            or isinstance(residual_limit, bool) or not isinstance(residual_limit, (int, float))
            or not math.isfinite(float(bandwidth)) or float(bandwidth) <= 0
            or not math.isfinite(float(residual_limit)) or float(residual_limit) <= 0
            or not all(np.all(np.isfinite(v)) for v in (reference, components, support, alpha))):
        raise ValueError("approach model has invalid shapes or values")
    stop_threshold = settings.get("ready_stop_score")
    forward_threshold = settings.get("ready_forward_max")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in (stop_threshold, forward_threshold)):
        raise ValueError("approach readiness settings are invalid")
    return (reference, components, support, alpha, float(bandwidth),
            float(residual_limit), float(stop_threshold), float(forward_threshold), diagnostics)


def predict_approach(model: dict[str, Any], own_jpeg: bytes, top_jpeg: bytes) -> dict[str, Any]:
    """Predict straight forward velocity and readiness; fail closed off support."""
    (reference, components, support, alpha, bandwidth, residual_limit,
     stop_threshold, forward_threshold, training) = _validated(model)
    difference = _compact(encode_views(own_jpeg, top_jpeg)) - reference
    feature_error = float(np.linalg.norm(difference) / math.sqrt(len(difference)))
    coordinate = difference @ components.T
    orthogonal = float(np.linalg.norm(difference - coordinate @ components))
    distances = np.linalg.norm(support - coordinate, axis=1)
    nearest = float(np.min(distances))
    weights = np.exp(-0.5 * distances * distances / (bandwidth * bandwidth))
    effective = float(weights.sum() ** 2 / max(1e-12, np.sum(weights * weights)))
    diagnostics = {"feature_error": feature_error, "nearest_support_distance": nearest,
                   "orthogonal_residual": orthogonal, "effective_support": effective,
                   "training_case_count": training.get("case_count")}
    if feature_error <= 1e-9:
        return {"ok": True, "forward": 0.0, "stop_score": 1.0,
                "ready": True, "reason": "learned_rgb_goal_anchor",
                "diagnostics": diagnostics}
    if nearest > 2 * bandwidth or orthogonal > residual_limit or effective < 1.25:
        return {"ok": False, "forward": 0.0, "stop_score": 0.0,
                "ready": False, "reason": "rgb_state_outside_approach_support",
                "diagnostics": diagnostics}
    prediction = weights @ alpha
    if prediction.shape != (2,) or not np.all(np.isfinite(prediction)):
        raise ValueError("approach prediction is not finite")
    forward = float(np.clip(prediction[0] * 0.15, 0.0, 0.15))
    stop_score = float(np.clip(prediction[1], 0.0, 1.0))
    ready = stop_score >= stop_threshold and forward <= forward_threshold
    return {"ok": True, "forward": forward, "stop_score": stop_score,
            "ready": bool(ready), "reason": "learned_rgb_stop" if ready else "learned_rgb_forward",
            "diagnostics": diagnostics}
