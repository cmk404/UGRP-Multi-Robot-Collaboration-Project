"""Teacher-trained, runtime RGB-only local visual Jacobian.

The teacher may choose the calibration probes, but the serialized model retains
only a reference image feature vector and image derivatives with respect to the
three issued pulse channels.  It is a local image-goal recovery model; it does
not assert contact, grasp success, or a world-space pose.
"""
from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np


SCHEMA = "ugrp.rgb_visual_jacobian.v1"
OWN_SIZE = (96, 72)
TOP_SIZE = (128, 96)
_TOP_WEIGHT = 2.0
_RIDGE = 1e-6


def _decode_gray(jpeg: bytes, size: tuple[int, int], name: str) -> np.ndarray:
    if not isinstance(jpeg, bytes) or len(jpeg) < 4:
        raise ValueError(f"{name} must be non-empty JPEG bytes")
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    if frame is None or min(frame.shape[:2]) < 2:
        raise ValueError(f"{name} is not a decodable image")
    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA).astype(np.float64) / 255.0


def _view_features(gray: np.ndarray, weight: float) -> np.ndarray:
    # Intensity preserves signed appearance changes.  Gradient magnitude gives
    # thin robot/object edges comparable influence to broad floor intensity.
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    edge = np.hypot(gx, gy) / 4.0
    return weight * np.concatenate((gray.ravel(), edge.ravel()))


def encode_views(own_jpeg: bytes, top_jpeg: bytes) -> np.ndarray:
    """Return a deterministic two-view intensity/edge thumbnail feature."""
    own = _view_features(_decode_gray(own_jpeg, OWN_SIZE, "own_jpeg"), 1.0)
    top = _view_features(_decode_gray(top_jpeg, TOP_SIZE, "top_jpeg"), _TOP_WEIGHT)
    encoded = np.concatenate((own, top)).astype(np.float64, copy=False)
    if encoded.ndim != 1 or not np.all(np.isfinite(encoded)):
        raise ValueError("encoded image features are not finite")
    return encoded


def _channels(value: Any) -> tuple[int, int, int]:
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) for v in value)):
        raise ValueError("channels must contain exactly three integer servo ids")
    result = tuple(int(v) for v in value)
    if len(set(result)) != 3:
        raise ValueError("channels must be unique")
    return result


def _delta(value: Any) -> np.ndarray:
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != 3:
        raise ValueError("delta_pulses must contain exactly three values")
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError("delta_pulses must be finite")
    return array


def fit_visual_jacobian(reference_own: bytes, reference_top: bytes,
                        samples: list[dict[str, Any]], channels=(3, 4, 5)) -> dict[str, Any]:
    """Fit ``delta_feature = J @ (delta_pulses / 50)`` from local probes."""
    channel_ids = _channels(channels)
    if not isinstance(samples, list) or not samples:
        raise ValueError("samples must be a non-empty list")
    reference = encode_views(reference_own, reference_top)
    commands, responses = [], []
    for sample in samples:
        if not isinstance(sample, dict) or set(("own_jpeg", "top_jpeg", "delta_pulses")) - set(sample):
            raise ValueError("each sample requires own_jpeg, top_jpeg, and delta_pulses")
        commands.append(_delta(sample["delta_pulses"]) / 50.0)
        feature = encode_views(sample["own_jpeg"], sample["top_jpeg"])
        if feature.shape != reference.shape:
            raise ValueError("sample feature shape differs from reference")
        before = reference
        if "before_own_jpeg" in sample or "before_top_jpeg" in sample:
            if not all(k in sample for k in ("before_own_jpeg", "before_top_jpeg")):
                raise ValueError("both before-view images are required for paired probes")
            before = encode_views(sample["before_own_jpeg"], sample["before_top_jpeg"])
        responses.append(feature - before)
    u = np.stack(commands)
    y = np.stack(responses)
    if not np.all(np.isfinite(u)) or not np.all(np.isfinite(y)):
        raise ValueError("training arrays must be finite")
    gram = u.T @ u + _RIDGE * np.eye(3)
    try:
        jacobian = (np.linalg.solve(gram, u.T @ y)).T
    except np.linalg.LinAlgError as exc:
        raise ValueError("visual regression is numerically invalid") from exc
    singular = np.linalg.svd(jacobian, compute_uv=False)
    tolerance = max(jacobian.shape) * np.finfo(float).eps * (singular[0] if len(singular) else 0.0)
    rank = int(np.sum(singular > tolerance))
    residual = y - u @ jacobian.T
    noise_rms = float(np.sqrt(np.mean(residual * residual)))
    response_rms = float(np.sqrt(np.mean(y * y)))
    values = (reference, jacobian, singular, np.asarray((noise_rms, response_rms)))
    if not all(np.all(np.isfinite(v)) for v in values):
        raise ValueError("fitted visual model is not finite")
    condition = (float(singular[0] / singular[-1])
                 if len(singular) == 3 and singular[-1] > tolerance else None)
    return {
        "schema": SCHEMA,
        "channels": list(channel_ids),
        "feature_spec": {
            "own_size": list(OWN_SIZE), "top_size": list(TOP_SIZE),
            "planes": ["grayscale", "sobel_magnitude"], "top_weight": _TOP_WEIGHT,
        },
        "reference_features": reference.tolist(),
        "feature_derivative": jacobian.tolist(),
        "diagnostics": {
            "sample_count": len(samples), "rank": rank,
            "singular_values": singular.tolist(), "condition_number": condition,
            "noise_rms": noise_rms, "response_rms": response_rms,
        },
    }


def _validated_model(model: Any) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    if not isinstance(model, dict) or model.get("schema") != SCHEMA:
        raise ValueError("unsupported visual Jacobian model schema")
    _channels(model.get("channels"))
    reference = np.asarray(model.get("reference_features"), dtype=np.float64)
    jacobian = np.asarray(model.get("feature_derivative"), dtype=np.float64)
    diagnostics = model.get("diagnostics")
    expected = 2 * OWN_SIZE[0] * OWN_SIZE[1] + 2 * TOP_SIZE[0] * TOP_SIZE[1]
    if (reference.shape != (expected,) or jacobian.shape != (expected, 3)
            or not isinstance(diagnostics, dict)
            or not np.all(np.isfinite(reference)) or not np.all(np.isfinite(jacobian))):
        raise ValueError("visual Jacobian model has invalid shapes or values")
    rank = diagnostics.get("rank")
    if isinstance(rank, bool) or not isinstance(rank, int) or not 0 <= rank <= 3:
        raise ValueError("visual Jacobian model has invalid rank diagnostics")
    return reference, jacobian, diagnostics


def predict_correction(model: dict[str, Any], own_jpeg: bytes, top_jpeg: bytes,
                       max_step: int = 25) -> dict[str, Any]:
    """Return a bounded pulse correction toward the learned RGB reference."""
    if isinstance(max_step, bool) or not isinstance(max_step, int) or not 1 <= max_step <= 100:
        raise ValueError("max_step must be an integer in 1..100")
    reference, jacobian, diagnostics = _validated_model(model)
    error = encode_views(own_jpeg, top_jpeg) - reference
    error_norm = float(np.linalg.norm(error))
    rank = int(diagnostics["rank"])
    singular = np.linalg.svd(jacobian, compute_uv=False)
    numerical_rank = int(np.linalg.matrix_rank(jacobian))
    observable = rank == 3 and numerical_rank == 3 and singular[-1] > 1e-12
    if not observable:
        return {
            "delta_pulses": [0, 0, 0], "feature_error": error_norm,
            "predicted_remaining_error": error_norm, "observable": False,
            "rank": min(rank, numerical_rank), "confidence": 0.0,
            "reason": "rank_deficient_visual_jacobian",
        }
    damping = max(1e-8, float(diagnostics.get("noise_rms", 0.0)))
    system = jacobian.T @ jacobian + damping * damping * np.eye(3)
    try:
        units = np.linalg.solve(system, -jacobian.T @ error)
    except np.linalg.LinAlgError as exc:
        raise ValueError("visual correction solve failed") from exc
    pulses = np.clip(np.rint(units * 50.0), -max_step, max_step).astype(int)
    applied = pulses.astype(np.float64) / 50.0
    remaining = error + jacobian @ applied
    remaining_norm = float(np.linalg.norm(remaining))
    if not np.all(np.isfinite(units)) or not math.isfinite(remaining_norm):
        raise ValueError("visual correction is not finite")
    condition = float(singular[0] / singular[-1])
    noise = max(0.0, float(diagnostics.get("noise_rms", 0.0)))
    signal = max(1e-12, float(diagnostics.get("response_rms", 0.0)))
    confidence = float(np.clip((singular[-1] / singular[0]) * math.exp(-noise / signal), 0.0, 1.0))
    return {
        "delta_pulses": pulses.tolist(), "feature_error": error_norm,
        "predicted_remaining_error": remaining_norm, "observable": True,
        "rank": numerical_rank, "confidence": confidence,
        "condition_number": condition, "reason": "local_rgb_goal_recovery",
    }
