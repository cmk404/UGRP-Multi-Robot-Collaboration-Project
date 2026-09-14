"""RGB-only heading estimation for the varied-start camera harness.

The estimator registers a soft yellow-wheel field against a goal image.  A
full foreground field is retained as a lower-precision fallback.  Both maps
are derived from RGB pixels; no simulator pose is used by ``predict``.
"""

from __future__ import annotations

import base64
import hashlib
import math
from typing import Any

import cv2
import numpy as np


TOP_ROIS = {"r1": (250, 345, 580, 550), "r3": (250, 160, 580, 345)}
SCHEMA = "ugrp.rgb_varied_start_heading.v1"
THRESHOLDS = {"soft_yellow": 0.70, "full_delta": 0.65}
_PATCH_SIZE = (96, 96)
_INITIAL_ANGLES_DEG = (-12.0, 0.0, 12.0)


def _decode_jpeg(value: bytes | bytearray | str) -> np.ndarray:
    if isinstance(value, str):
        value = base64.b64decode(value)
    if not isinstance(value, (bytes, bytearray)) or len(value) < 4:
        raise ValueError("invalid top JPEG")
    image = cv2.imdecode(np.frombuffer(value, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (720, 960, 3):
        raise ValueError("invalid top JPEG")
    return image


def _encode_png(image: np.ndarray) -> str:
    ok, data = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("could not encode heading model image")
    return base64.b64encode(data.tobytes()).decode("ascii")


def _decode_png(value: str) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(base64.b64decode(value), np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("invalid heading model image")
    return image


def _crop(image: np.ndarray, roi: tuple[int, int, int, int]) -> np.ndarray:
    x1, y1, x2, y2 = roi
    if image.shape[0] < y2 or image.shape[1] < x2:
        raise ValueError("top image is smaller than configured heading ROI")
    return image[y1:y2, x1:x2]


def _yellow_center(image: np.ndarray, background: np.ndarray) -> tuple[float, float]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    delta = np.mean(np.abs(image.astype(np.float32) - background.astype(np.float32)), axis=2)
    mask = (
        (hsv[:, :, 0] > 12)
        & (hsv[:, :, 0] < 45)
        & (hsv[:, :, 1] > 110)
        & (hsv[:, :, 2] > 90)
        & (delta > 12)
    ).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    clean = np.zeros_like(mask)
    for index in range(1, count):
        if stats[index, cv2.CC_STAT_AREA] >= 2:
            clean[labels == index] = 1
    yy, xx = np.where(clean)
    if len(xx) < 20:
        raise ValueError("too few foreground wheel pixels")
    return float(xx.mean()), float(yy.mean())


def _field(image: np.ndarray, background: np.ndarray, mode: str) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    delta = np.mean(np.abs(image.astype(np.float32) - background.astype(np.float32)), axis=2)
    if mode == "soft_yellow":
        value = (
            np.clip((hsv[:, :, 1] - 70) / 100, 0, 1)
            * np.clip((hsv[:, :, 2] - 50) / 150, 0, 1)
            * np.clip((delta - 6) / 24, 0, 1)
            * ((hsv[:, :, 0] > 12) & (hsv[:, :, 0] < 45))
        )
    elif mode == "full_delta":
        value = np.clip((delta - 8) / 100, 0, 1)
    else:
        raise ValueError(f"unknown heading field mode: {mode}")
    return cv2.GaussianBlur(value.astype(np.float32), (5, 5), 0.8)


def _measure(
    image: np.ndarray,
    background: np.ndarray,
    template: np.ndarray,
    goal_center: tuple[float, float],
    mode: str,
    threshold: float,
) -> tuple[np.ndarray, float] | None:
    try:
        center = _yellow_center(image, background)
        patch = cv2.getRectSubPix(_field(image, background, mode), _PATCH_SIZE, center)
    except (ValueError, cv2.error):
        return None
    candidates: list[tuple[float, np.ndarray, float]] = []
    for angle in _INITIAL_ANGLES_DEG:
        warp = cv2.getRotationMatrix2D((47.5, 47.5), -angle, 1.0).astype(np.float32)
        try:
            score, warp = cv2.findTransformECC(
                template,
                patch,
                warp,
                cv2.MOTION_EUCLIDEAN,
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-6),
                None,
                5,
            )
        except cv2.error:
            continue
        theta = math.degrees(math.atan2(float(warp[1, 0]), float(warp[0, 0])))
        if abs(theta) < 16:
            candidates.append((float(score), warp, theta))
    if not candidates:
        return None
    score, warp, theta = max(candidates, key=lambda item: item[0])
    if score < threshold:
        return None
    pivot = warp @ np.array([47.5, 47.5, 1.0])
    x = center[0] + pivot[0] - 47.5 - goal_center[0]
    y = center[1] + pivot[1] - 47.5 - goal_center[1]
    return np.array([1, x, y, theta, x * theta, y * theta, x * x, y * y]), score


def _fold(case_id: str) -> int:
    return int(hashlib.sha256(case_id.encode()).hexdigest(), 16) % 5


def fit_heading_model(
    reference_top: bytes,
    samples: list[dict[str, Any]],
    rid: str,
    stage: str = "yaw",
) -> dict[str, Any]:
    """Fit a JSON-serializable heading model from calibration RGB and labels."""
    if rid not in TOP_ROIS or stage != "yaw":
        raise ValueError(f"unsupported robot id: {rid}")
    selected = [sample for sample in samples if sample.get("stage", stage) == stage]
    if len(selected) < 10:
        raise ValueError("at least 10 heading calibration samples are required")
    roi = TOP_ROIS[rid]
    images = [_crop(_decode_jpeg(sample["top_jpeg"]), roi) for sample in selected]
    background = np.median(np.stack(images), axis=0).astype(np.uint8)
    goal = _crop(_decode_jpeg(reference_top), roi)
    goal_center = _yellow_center(goal, background)
    encoded_templates: dict[str, str] = {}
    measurements: dict[str, list[tuple[int, np.ndarray, float]]] = {}
    thresholds = dict(THRESHOLDS)
    for mode, threshold in thresholds.items():
        goal_patch = cv2.getRectSubPix(_field(goal, background, mode), _PATCH_SIZE, goal_center)
        encoded_templates[mode] = _encode_png(np.clip(goal_patch * 255, 0, 255).astype(np.uint8))
        template = _decode_png(encoded_templates[mode]).astype(np.float32) / 255.0
        rows = []
        for index, image in enumerate(images):
            value = _measure(image, background, template, goal_center, mode, threshold)
            if value is not None:
                rows.append((index, value[0], value[1]))
        measurements[mode] = rows
    coefficients: dict[str, list[float]] = {}
    diagnostics: dict[str, Any] = {"sample_count": len(selected), "methods": {},
        "cv_feature_pipeline": "case-held-out regression conditional on a background learned from all training RGB; final heldout images are excluded"}
    domains = {}
    targets = np.array([float(sample["error"]) for sample in selected])
    groups = np.array([_fold(str(sample["case_id"])) for sample in selected])
    for mode, rows in measurements.items():
        if len(rows) < 8:
            raise ValueError(f"insufficient supported calibration samples for {mode}")
        indices = np.array([row[0] for row in rows])
        features = np.stack([row[1] for row in rows])
        coordinates = features[:, 1:4]
        margin = np.maximum(np.ptp(coordinates, axis=0) * .1, [3., 3., .5])
        domains[mode] = {"minimum": (coordinates.min(axis=0) - margin).tolist(),
                         "maximum": (coordinates.max(axis=0) + margin).tolist()}
        values = targets[indices]
        supported_groups = groups[indices]
        cv_prediction = np.full(len(rows), np.nan)
        for fold in range(5):
            train = supported_groups != fold
            test = supported_groups == fold
            if train.sum() >= features.shape[1] and test.any():
                fold_coef = np.linalg.lstsq(features[train], values[train], rcond=None)[0]
                cv_prediction[test] = features[test] @ fold_coef
        coef = np.linalg.lstsq(features, values, rcond=None)[0]
        coefficients[mode] = coef.tolist()
        valid = np.isfinite(cv_prediction)
        error_deg = np.degrees(cv_prediction[valid] - values[valid])
        diagnostics["methods"][mode] = {
            "supported": len(rows),
            "threshold": thresholds[mode],
            "cv_rmse_deg": float(np.sqrt(np.mean(error_deg * error_deg))),
            "cv_p95_deg": float(np.quantile(np.abs(error_deg), 0.95)),
        }
    return {
        "version": 1, "schema": SCHEMA,
        "rid": rid,
        "robot_id": rid,
        "stage": stage,
        "roi": list(roi),
        "background_png": _encode_png(background),
        "templates_png": encoded_templates,
        "goal_center": list(goal_center),
        "thresholds": thresholds,
        "coefficients": coefficients,
        "domains": domains,
        "diagnostics": diagnostics,
    }


def predict_heading(model: dict[str, Any], top_jpeg: bytes) -> dict[str, Any]:
    """Estimate heading error, preferring the calibrated fine measurement."""
    if (not isinstance(model, dict) or model.get("schema") != SCHEMA
            or model.get("robot_id") not in TOP_ROIS or model.get("stage") != "yaw"
            or model.get("roi") != list(TOP_ROIS[model["robot_id"]])
            or model.get("thresholds") != THRESHOLDS):
        raise ValueError("invalid heading model identity/calibration")
    image = _crop(_decode_jpeg(top_jpeg), tuple(model["roi"]))
    background = _decode_png(model["background_png"])
    goal_center = tuple(float(value) for value in model["goal_center"])
    if background.shape != image.shape or len(goal_center) != 2 or not all(map(math.isfinite, goal_center)):
        raise ValueError("invalid heading background/center")
    attempts = []
    for mode, precision in (("soft_yellow", "fine"), ("full_delta", "coarse")):
        template = _decode_png(model["templates_png"][mode]).astype(np.float32) / 255.0
        coefficient = np.asarray(model["coefficients"][mode], dtype=float)
        low = np.asarray(model["domains"][mode]["minimum"], dtype=float)
        high = np.asarray(model["domains"][mode]["maximum"], dtype=float)
        if (template.shape != (96, 96) or coefficient.shape != (8,)
                or low.shape != (3,) or high.shape != (3,) or np.any(low >= high)
                or not all(np.all(np.isfinite(x)) for x in (template, coefficient, low, high))):
            raise ValueError("invalid heading model arrays")
        measured = _measure(
            image,
            background,
            template,
            goal_center,
            mode,
            float(model["thresholds"][mode]),
        )
        if measured is None:
            attempts.append({"method": mode, "ok": False})
            continue
        features, score = measured
        if not np.all((features[1:4] >= low) & (features[1:4] <= high)):
            attempts.append({"method": mode, "ok": False, "reason": "outside_training_domain"})
            continue
        error = float(features @ coefficient)
        return {
            "ok": True,
            "error": error,
            "precision": precision,
            "diagnostics": {
                "method": mode,
                "precision": precision,
                "ecc_score": score,
                "attempts": attempts,
            },
        }
    return {
        "ok": False,
        "error": None,
        "precision": "unavailable",
        "diagnostics": {
            "method": None,
            "precision": "unavailable",
            "reason": "heading_registration_unsupported",
            "attempts": attempts,
        },
    }
