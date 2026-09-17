"""Nonlinear, teacher-labelled recovery using only two RGB views at runtime.

The exported model contains compressed image features and corrective pulse
labels.  It contains no pose, command state, IK, contact, or success signal.
Predictions are bounded local interpolation and fail closed away from support.
"""
from __future__ import annotations

import hashlib
import math
from typing import Any

import numpy as np

from harness.camera_teacher_student import OWN_SIZE, TOP_SIZE, encode_views


SCHEMA = "ugrp.rgb_recovery_kernel.v1"
CHANNELS = (3, 4, 5)
POOL = 4
MAX_COMPONENTS = 24


def _compact(encoded: np.ndarray) -> np.ndarray:
    expected = 2 * OWN_SIZE[0] * OWN_SIZE[1] + 2 * TOP_SIZE[0] * TOP_SIZE[1]
    value = np.asarray(encoded, dtype=np.float64)
    if value.shape != (expected,) or not np.all(np.isfinite(value)):
        raise ValueError("encoded RGB feature has invalid shape or values")
    result, offset = [], 0
    for width, height in (OWN_SIZE, OWN_SIZE, TOP_SIZE, TOP_SIZE):
        count = width * height
        plane = value[offset:offset + count].reshape(height, width)
        offset += count
        if height % POOL or width % POOL:
            raise ValueError("feature plane is incompatible with fixed pooling")
        result.append(plane.reshape(height // POOL, POOL, width // POOL, POOL).mean((1, 3)).ravel())
    return np.concatenate(result)


def _label(value: Any) -> np.ndarray:
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != 3:
        raise ValueError("correction_pulses must contain three values")
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)) or np.any(np.abs(result) > 500):
        raise ValueError("correction_pulses must be finite and bounded")
    return result


def _kernel(left: np.ndarray, right: np.ndarray, bandwidth: float) -> np.ndarray:
    distances2 = np.maximum(
        np.sum(left * left, axis=1)[:, None] + np.sum(right * right, axis=1)[None, :]
        - 2.0 * left @ right.T, 0.0,
    )
    return np.exp(-0.5 * distances2 / (bandwidth * bandwidth))


def _select_kernel(x: np.ndarray, y: np.ndarray, groups: list[str] | None = None
                   ) -> tuple[float, float, np.ndarray, float, str]:
    pair = np.sqrt(np.maximum(
        np.sum(x * x, axis=1)[:, None] + np.sum(x * x, axis=1)[None, :] - 2 * x @ x.T,
        0.0,
    ))
    nonzero = pair[np.triu_indices(len(x), 1)]
    nonzero = nonzero[nonzero > 1e-9]
    if not len(nonzero):
        raise ValueError("training RGB features contain no distinct states")
    median = float(np.median(nonzero))
    best = None
    folds: list[np.ndarray] = []
    if groups is not None:
        unique = sorted(set(groups), key=lambda value: hashlib.sha256(value.encode()).hexdigest())
        if len(unique) < 4:
            raise ValueError("grouped selection requires at least four distinct case_id values")
        assignments = {value: index % 4 for index, value in enumerate(unique)}
        group_array = np.asarray(["__goal_anchor__", *groups], dtype=object)
        folds = [np.asarray([assignments.get(value) == fold for value in group_array])
                 for fold in range(4)]
        selection = "group_4fold"
    else:
        selection = "leave_one_sample_out"
    # Group folds keep correlated frames from one recovery trajectory together.
    # The synthetic/standalone API retains exact leave-one-sample-out selection.
    for bandwidth in (median * 0.5, median, median * 2.0):
        k = _kernel(x, x, bandwidth)
        eigmax = max(1e-12, float(np.linalg.eigvalsh(k)[-1]))
        for regularization in (eigmax * 1e-6, eigmax * 1e-4,
                               eigmax * 1e-2, eigmax):
            try:
                if folds:
                    squared, count = 0.0, 0
                    for validation in folds:
                        training = ~validation
                        k_train = k[np.ix_(training, training)]
                        alpha_fold = np.linalg.solve(
                            k_train + regularization * np.eye(int(training.sum())), y[training]
                        )
                        prediction = k[np.ix_(validation, training)] @ alpha_fold
                        residual = prediction - y[validation]
                        squared += float(np.sum(residual * residual));count += int(residual.size)
                    score = squared / count
                else:
                    inverse = np.linalg.inv(k + regularization * np.eye(len(x)))
                    alpha_loo = inverse @ y
                    diagonal = np.diag(inverse)
                    if np.any(np.abs(diagonal) < 1e-12):
                        continue
                    loo_residual = alpha_loo / diagonal[:, None]
                    score = float(np.mean(loo_residual * loo_residual))
                alpha = np.linalg.solve(k + regularization * np.eye(len(x)), y)
            except np.linalg.LinAlgError:
                continue
            candidate = (score, bandwidth, regularization, alpha)
            if best is None or candidate[0] < best[0]:
                best = candidate
    if best is None or not all(math.isfinite(v) for v in best[:3]):
        raise ValueError("kernel regression selection failed")
    return best[1], best[2], best[3], best[0], selection


def fit_recovery_model(reference_own: bytes, reference_top: bytes,
                       samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Fit a compact Gaussian-kernel inverse controller from RGB-labelled examples."""
    if not isinstance(samples, list) or not samples:
        raise ValueError("samples must be a non-empty list")
    reference = _compact(encode_views(reference_own, reference_top))
    features, labels = [reference], [np.zeros(3)]  # explicit goal anchor
    case_ids: list[str] = []
    any_case_id = any(isinstance(sample, dict) and "case_id" in sample for sample in samples)
    for sample in samples:
        if not isinstance(sample, dict) or not all(
                key in sample for key in ("own_jpeg", "top_jpeg", "correction_pulses")):
            raise ValueError("each sample requires two RGB views and correction_pulses")
        features.append(_compact(encode_views(sample["own_jpeg"], sample["top_jpeg"])))
        labels.append(_label(sample["correction_pulses"]))
        if any_case_id:
            case_id = sample.get("case_id")
            if not isinstance(case_id, str) or not case_id:
                raise ValueError("all samples require a non-empty string case_id for grouped selection")
            case_ids.append(case_id)
    delta = np.stack(features) - reference
    y = np.stack(labels)
    if not np.all(np.isfinite(delta)) or not np.all(np.isfinite(y)):
        raise ValueError("training data are not finite")
    _, singular, vt = np.linalg.svd(delta, full_matrices=False)
    tolerance = max(delta.shape) * np.finfo(float).eps * (singular[0] if len(singular) else 0.0)
    feature_rank = int(np.sum(singular > tolerance))
    components = vt[:min(MAX_COMPONENTS, feature_rank)]
    if len(components) < 3:
        raise ValueError("RGB recovery samples have fewer than three observable directions")
    coordinates = delta @ components.T
    reconstructed = coordinates @ components
    reconstruction_residuals = np.linalg.norm(delta - reconstructed, axis=1)
    residual_median = float(np.median(reconstruction_residuals))
    residual_mad = float(np.median(np.abs(reconstruction_residuals - residual_median)))
    # A query can project near a support point while being radically different
    # in discarded pixels.  Preserve a data-derived orthogonal novelty limit.
    residual_limit = max(1e-8, float(np.max(reconstruction_residuals)),
                         residual_median + 4.0 * max(residual_mad, 1e-8)) * 1.25
    label_rank = int(np.linalg.matrix_rank(y))
    rank = min(feature_rank, label_rank, 3)
    bandwidth, regularization, alpha, cv_mse, selection = _select_kernel(
        coordinates, y, case_ids if any_case_id else None
    )
    arrays = (reference, components, coordinates, alpha, singular, y)
    if not all(np.all(np.isfinite(value)) for value in arrays):
        raise ValueError("fitted recovery model is not finite")
    return {
        "schema": SCHEMA, "channels": list(CHANNELS),
        "feature_spec": {"source": "encode_views", "pool": POOL,
                         "components": int(len(components))},
        "reference_compact": reference.tolist(),
        "pca_components": components.tolist(),
        "support_coordinates": coordinates.tolist(),
        "kernel_alpha": alpha.tolist(),
        "bandwidth": bandwidth, "regularization": regularization,
        "pca_residual_limit": residual_limit,
        "diagnostics": {
            "sample_count": len(samples), "support_count": len(coordinates),
            "goal_anchor_included": True, "feature_rank": feature_rank,
            "label_rank": label_rank, "rank": rank,
            "singular_values": singular[:len(components)].tolist(),
            "hyperparameter_selection": selection,
            "cross_validation_mse": cv_mse,
            "pca_residual_median": residual_median,
        },
    }


def _validated(model: Any) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float, dict]:
    if not isinstance(model, dict) or model.get("schema") != SCHEMA or model.get("channels") != list(CHANNELS):
        raise ValueError("unsupported recovery model schema or channels")
    reference = np.asarray(model.get("reference_compact"), dtype=np.float64)
    components = np.asarray(model.get("pca_components"), dtype=np.float64)
    support = np.asarray(model.get("support_coordinates"), dtype=np.float64)
    alpha = np.asarray(model.get("kernel_alpha"), dtype=np.float64)
    bandwidth = model.get("bandwidth")
    residual_limit = model.get("pca_residual_limit")
    diagnostics = model.get("diagnostics")
    compact_size = 2 * (OWN_SIZE[0] // POOL) * (OWN_SIZE[1] // POOL) + 2 * (TOP_SIZE[0] // POOL) * (TOP_SIZE[1] // POOL)
    if (reference.shape != (compact_size,) or components.ndim != 2
            or components.shape[1] != compact_size or not 3 <= len(components) <= MAX_COMPONENTS
            or support.ndim != 2 or support.shape[1] != len(components)
            or alpha.shape != (len(support), 3) or not isinstance(diagnostics, dict)
            or isinstance(bandwidth, bool) or not isinstance(bandwidth, (int, float))
            or not math.isfinite(bandwidth) or bandwidth <= 0
            or isinstance(residual_limit, bool) or not isinstance(residual_limit, (int, float))
            or not math.isfinite(residual_limit) or residual_limit <= 0
            or not all(np.all(np.isfinite(v)) for v in (reference, components, support, alpha))):
        raise ValueError("recovery model has invalid shapes or values")
    return (reference, components, support, alpha, float(bandwidth),
            float(residual_limit), diagnostics)


def predict_recovery(model: dict[str, Any], own_jpeg: bytes, top_jpeg: bytes,
                     max_step: int = 25) -> dict[str, Any]:
    """Interpolate a bounded correction from current RGB, failing closed off support."""
    if isinstance(max_step, bool) or not isinstance(max_step, int) or not 1 <= max_step <= 100:
        raise ValueError("max_step must be an integer in 1..100")
    reference, components, support, alpha, bandwidth, residual_limit, diagnostics = _validated(model)
    difference = _compact(encode_views(own_jpeg, top_jpeg)) - reference
    band = model.get('constant_background_top_band')
    if band is not None:
        # Optional task-local transfer of an existing model: discard only TOP
        # pixels that were constant in every learned direction. Never suppress
        # own-camera novelty or a varying/trained feature to pass a support gate.
        if band != [6, 19]:
            raise ValueError('unsupported constant-background feature band')
        mask = np.ones(len(difference), dtype=bool)
        own_count = 2 * (OWN_SIZE[0]//POOL) * (OWN_SIZE[1]//POOL)
        rows = np.arange(TOP_SIZE[1]//POOL)
        mask[own_count:] = np.tile(np.repeat((rows>=band[0]) & (rows<band[1]), TOP_SIZE[0]//POOL), 2)
        if np.max(np.abs(components[:,~mask])) > 1e-8:
            raise ValueError('background band removes a learned visual direction')
        difference[~mask] = 0.
    feature_error = float(np.linalg.norm(difference) / math.sqrt(len(difference)))
    coordinate = difference @ components.T
    orthogonal_residual = float(np.linalg.norm(difference - coordinate @ components))
    distances = np.linalg.norm(support - coordinate, axis=1)
    rank = diagnostics.get("rank")
    if isinstance(rank, bool) or not isinstance(rank, int) or not 0 <= rank <= 3:
        raise ValueError("recovery model has invalid rank diagnostics")
    if feature_error <= 1e-9:
        return {"delta_pulses": [0, 0, 0], "feature_error": feature_error,
                "predicted_remaining_error": 0.0, "observable": rank == 3,
                "rank": rank, "confidence": 1.0 if rank == 3 else 0.0,
                "reason": "at_learned_rgb_goal" if rank == 3 else "rank_deficient_recovery_model"}
    nearest = float(np.min(distances))
    kernels = np.exp(-0.5 * distances * distances / (bandwidth * bandwidth))
    effective = float(kernels.sum() ** 2 / max(1e-12, np.sum(kernels * kernels)))
    observable = (rank == 3 and nearest <= 2.0 * bandwidth and effective >= 1.25
                  and orthogonal_residual <= residual_limit)
    coverage = math.exp(-0.5 * (nearest / bandwidth) ** 2)
    confidence = float(np.clip(coverage * min(1.0, effective / 3.0), 0.0, 1.0)) if observable else 0.0
    if not observable:
        reason = "rank_deficient_recovery_model" if rank < 3 else "rgb_state_outside_kernel_support"
        return {"delta_pulses": [0, 0, 0], "feature_error": feature_error,
                "predicted_remaining_error": feature_error, "observable": False,
                "rank": rank, "confidence": 0.0, "reason": reason,
                "nearest_support_distance": nearest, "effective_support": effective,
                "orthogonal_residual": orthogonal_residual}
    predicted = kernels @ alpha
    if predicted.shape != (3,) or not np.all(np.isfinite(predicted)):
        raise ValueError("recovery prediction is not finite")
    pulses = np.clip(np.rint(predicted), -max_step, max_step).astype(int)
    return {"delta_pulses": pulses.tolist(), "feature_error": feature_error,
            # The inverse model has no image forward dynamics, so it cannot
            # claim that issuing a pulse will reduce the measured image error.
            "predicted_remaining_error": feature_error,
            "prediction_error_available": False,
            "observable": True, "rank": rank, "confidence": confidence,
            "reason": "local_rgb_kernel_recovery",
            "nearest_support_distance": nearest, "effective_support": effective,
            "orthogonal_residual": orthogonal_residual}
