"""RGB-only fixed-top foreground geometry regression for varied starts.

Training may consume offline teacher errors.  The saved predictor contains only
image-derived state and explicit ExtraTrees nodes, so inference has no sklearn
or privileged-state dependency.
"""
from __future__ import annotations

import base64
import hashlib
import math
from typing import Any

import cv2
import numpy as np


SCHEMA = "ugrp.rgb_varied_start_geometry.v1"
STAGES = ("lateral", "forward")
TOP_ROIS = {"r1": (250, 345, 580, 550), "r3": (250, 160, 580, 345)}
TOP_REFERENCE_SIZE = (960, 720)
BACKGROUND_SAMPLES = 65
FEATURE_GRID = (8, 6)
MIN_FOREGROUND_PIXELS = 20


def _decode_top(jpeg: bytes) -> np.ndarray:
    if not isinstance(jpeg, bytes) or len(jpeg) < 4:
        raise ValueError("top_jpeg must be non-empty JPEG bytes")
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None or min(image.shape[:2]) < 2:
        raise ValueError("top_jpeg is not a decodable image")
    return image


def _roi(image: np.ndarray, rid: str) -> np.ndarray:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = TOP_ROIS[rid]
    rw, rh = TOP_REFERENCE_SIZE
    left, right = round(x1 * width / rw), round(x2 * width / rw)
    top, bottom = round(y1 * height / rh), round(y2 * height / rh)
    if not (0 <= left < right <= width and 0 <= top < bottom <= height):
        raise ValueError("fixed top ROI is outside the decoded image")
    return cv2.resize(image[top:bottom, left:right], (x2 - x1, y2 - y1),
                      interpolation=cv2.INTER_AREA)


def _moments(mask: np.ndarray, weight: np.ndarray | None = None) -> list[float]:
    yy, xx = np.nonzero(mask)
    if len(xx) < MIN_FOREGROUND_PIXELS:
        return [0.0] * 9
    values = np.ones(len(xx)) if weight is None else weight[yy, xx].astype(float) + 1.0
    total = float(values.sum())
    cx, cy = float(values @ xx / total), float(values @ yy / total)
    dx, dy = xx - cx, yy - cy
    x2 = float(values @ (dx * dx) / total)
    y2 = float(values @ (dy * dy) / total)
    xy = float(values @ (dx * dy) / total)
    angle = 0.5 * math.atan2(2.0 * xy, x2 - y2)
    h, w = mask.shape
    return [cx / w, cy / h, x2 / (w * w), y2 / (h * h), xy / (w * h),
            math.sin(2.0 * angle), math.cos(2.0 * angle), len(xx) / mask.size,
            total / mask.size]


def _components(mask: np.ndarray) -> list[float]:
    count, _labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    h, w = mask.shape
    values = sorted(((int(stats[i, 4]), float(centers[i, 0]) / w,
                      float(centers[i, 1]) / h, float(stats[i, 2]) / w,
                      float(stats[i, 3]) / h) for i in range(1, count)), reverse=True)[:8]
    values += [(0, 0.0, 0.0, 0.0, 0.0)] * (8 - len(values))
    return np.asarray(values, dtype=float).ravel().tolist()


def _quantiles(mask: np.ndarray) -> list[float]:
    yy, xx = np.nonzero(mask)
    if not len(xx):
        return [0.0] * 14
    h, w = mask.shape
    result = []
    for quantile in (.05, .1, .25, .5, .75, .9, .95):
        result.extend((float(np.quantile(xx, quantile)) / w,
                       float(np.quantile(yy, quantile)) / h))
    return result


def _features(roi: np.ndarray, background: np.ndarray) -> np.ndarray:
    difference = np.max(cv2.absdiff(roi, background), axis=2)
    foreground = (difference > 18).astype(np.uint8)
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN,
                                  np.ones((3, 3), np.uint8))
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_CLOSE,
                                  np.ones((5, 5), np.uint8))
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow = ((hsv[:, :, 0] >= 18) & (hsv[:, :, 0] <= 38)
              & (hsv[:, :, 1] > 80) & (hsv[:, :, 2] > 70)
              & (foreground > 0)).astype(np.uint8)
    result = _moments(foreground, difference) + _moments(yellow)
    for mask in (foreground, yellow):
        result.extend(_components(mask))
        result.extend(_quantiles(mask))
    for mask in (foreground, yellow):
        small = cv2.resize(mask.astype(np.float32), FEATURE_GRID,
                           interpolation=cv2.INTER_AREA)
        result.extend(small.ravel().astype(float).tolist())
    return np.asarray(result, dtype=np.float64)


def _background(rois: list[np.ndarray]) -> np.ndarray:
    # Hash ordering makes the capped median independent of caller ordering.
    ordered = sorted(rois, key=lambda x: hashlib.sha256(x.tobytes()).digest())
    if len(ordered) > BACKGROUND_SAMPLES:
        indices = np.linspace(0, len(ordered) - 1, BACKGROUND_SAMPLES).round().astype(int)
        ordered = [ordered[index] for index in indices]
    return np.median(np.stack(ordered), axis=0).astype(np.uint8)


def _fit_tree(features: np.ndarray, targets: np.ndarray, rng: np.random.Generator,
              *, min_leaf: int = 2, max_depth: int = 18) -> dict[str, Any]:
    tree = {key: [] for key in ("left", "right", "feature", "threshold", "value")}

    def grow(indices: np.ndarray, depth: int) -> int:
        node = len(tree["left"])
        for key, value in (("left", -1), ("right", -1), ("feature", -1),
                           ("threshold", 0.0), ("value", float(targets[indices].mean()))):
            tree[key].append(value)
        if depth >= max_depth or len(indices) < 2 * min_leaf or np.ptp(targets[indices]) <= 1e-12:
            return node
        candidates = rng.choice(features.shape[1], min(16, features.shape[1]), replace=False)
        best = None
        for feature in candidates:
            values = features[indices, feature]
            low, high = float(values.min()), float(values.max())
            if high - low <= 1e-12:
                continue
            threshold = float(rng.uniform(low, high))
            select = values <= threshold
            left, right = indices[select], indices[~select]
            if len(left) < min_leaf or len(right) < min_leaf:
                continue
            loss = float(np.var(targets[left]) * len(left) + np.var(targets[right]) * len(right))
            if best is None or loss < best[0]:
                best = loss, int(feature), threshold, left, right
        if best is None:
            return node
        _loss, feature, threshold, left_indices, right_indices = best
        tree["feature"][node], tree["threshold"][node] = feature, threshold
        tree["left"][node] = grow(left_indices, depth + 1)
        tree["right"][node] = grow(right_indices, depth + 1)
        return node

    grow(np.arange(len(targets)), 0)
    return tree


def _fit_forest(features: np.ndarray, targets: np.ndarray, count: int,
                seed: int) -> list[dict[str, Any]]:
    rng = np.random.default_rng(seed)
    return [_fit_tree(features, targets, rng) for _ in range(count)]


def _forest_predict(trees: list[dict[str, Any]], features: np.ndarray) -> np.ndarray:
    return np.asarray([np.mean([_tree_predict(tree, row) for tree in trees])
                       for row in features], dtype=float)


def _tree_predict(tree: dict[str, Any], features: np.ndarray) -> float:
    node = 0
    for _step in range(len(tree["left"]) + 1):
        if not 0 <= node < len(tree["left"]):
            raise ValueError("geometry tree child index is out of bounds")
        if tree["left"][node] == -1:
            return float(tree["value"][node])
        node = (tree["left"][node] if features[tree["feature"][node]]
                <= tree["threshold"][node] else tree["right"][node])
    raise ValueError("geometry tree contains a cycle")


def _validate_tree(tree: Any, feature_count: int) -> None:
    keys = ("left", "right", "feature", "threshold", "value")
    if not isinstance(tree, dict) or any(not isinstance(tree.get(key), list) for key in keys):
        raise ValueError("invalid geometry tree data")
    lengths = {len(tree[key]) for key in keys}
    if len(lengths) != 1 or not lengths or next(iter(lengths)) < 1:
        raise ValueError("invalid geometry tree arrays")
    count = len(tree["left"])
    for node in range(count):
        left, right, feature = tree["left"][node], tree["right"][node], tree["feature"][node]
        threshold, value = tree["threshold"][node], tree["value"][node]
        if (isinstance(left, bool) or not isinstance(left, int)
                or isinstance(right, bool) or not isinstance(right, int)
                or isinstance(feature, bool) or not isinstance(feature, int)
                or isinstance(threshold, bool) or not isinstance(threshold, (int, float))
                or isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(threshold) or not math.isfinite(value)):
            raise ValueError("invalid geometry tree node")
        leaf = left == right == -1
        if not leaf and not (0 <= left < count and 0 <= right < count and 0 <= feature < feature_count):
            raise ValueError("invalid geometry tree split")


def fit_geometry_model(reference_top: bytes, samples: list[dict[str, Any]],
                       rid: str, stage: str) -> dict[str, Any]:
    """Fit a deterministic top-RGB error model from offline teacher errors."""
    if rid not in TOP_ROIS or stage not in STAGES:
        raise ValueError("unsupported robot or geometry stage")
    if not isinstance(samples, list) or len(samples) < 10:
        raise ValueError("geometry training requires at least ten samples")
    reference_roi = _roi(_decode_top(reference_top), rid)
    parsed = []
    for sample in samples:
        if not isinstance(sample, dict) or set(("top_jpeg", "error", "case_id")) - set(sample):
            raise ValueError("geometry sample requires top_jpeg, error, and case_id")
        error, case_id = sample["error"], sample["case_id"]
        if (isinstance(error, bool) or not isinstance(error, (int, float))
                or not math.isfinite(error) or not isinstance(case_id, str) or not case_id):
            raise ValueError("geometry sample error/case_id is invalid")
        parsed.append((_roi(_decode_top(sample["top_jpeg"]), rid), float(error), case_id))
    parsed.sort(key=lambda row: (row[2], hashlib.sha256(row[0].tobytes()).digest(), row[1]))
    if len(set(row[2] for row in parsed)) < 5:
        raise ValueError("geometry training requires at least five cases")
    targets = np.asarray([row[1] for row in parsed], dtype=np.float64)
    groups = np.asarray([row[2] for row in parsed])
    if np.ptp(targets) <= 0:
        raise ValueError("geometry samples contain no finite target variation")
    folds = min(5, len(set(groups)))
    case_fold = {case: int.from_bytes(hashlib.sha256(case.encode()).digest()[:8], "big") % folds
                 for case in set(groups)}
    # Ensure hash collisions do not accidentally leave a fold empty.
    if len(set(case_fold.values())) < folds:
        case_fold = {case: index % folds for index, case in enumerate(sorted(set(groups)))}
    cv = np.empty(len(targets), dtype=float)
    for fold in range(folds):
        held = np.asarray([case_fold[case] == fold for case in groups])
        fold_background = _background([reference_roi] + [parsed[index][0]
                                      for index in np.flatnonzero(~held)])
        train_features = np.stack([_features(parsed[index][0], fold_background)
                                   for index in np.flatnonzero(~held)])
        held_features = np.stack([_features(parsed[index][0], fold_background)
                                  for index in np.flatnonzero(held)])
        trees = _fit_forest(train_features, targets[~held], 24, 1700 + fold)
        cv[held] = _forest_predict(trees, held_features)
    errors = np.abs(cv - targets)
    background = _background([reference_roi] + [row[0] for row in parsed])
    features = np.stack([_features(row[0], background) for row in parsed])
    if not np.all(np.isfinite(features)):
        raise ValueError("geometry samples produced non-finite features")
    forest = _fit_forest(features, targets, 96, 17)
    mean, scale = features.mean(axis=0), features.std(axis=0)
    scale[scale < 1e-8] = 1.0
    standardized = (features - mean) / scale
    _u, _s, vt = np.linalg.svd(standardized, full_matrices=False)
    components = vt[:min(16, len(vt))]
    coordinates = standardized @ components.T
    residuals = np.linalg.norm(standardized - coordinates @ components, axis=1)
    nearest_other_case = []
    for index, coordinate in enumerate(coordinates):
        eligible = groups != groups[index]
        nearest_other_case.append(float(np.min(np.linalg.norm(
            coordinates[eligible] - coordinate, axis=1))))
    ok, encoded = cv2.imencode(".png", background)
    if not ok:
        raise RuntimeError("failed to encode learned RGB background")
    return {"schema": SCHEMA, "robot_id": rid, "stage": stage,
            "runtime_inputs": ["fixed_top_rgb"],
            "top_roi": {"reference_size": list(TOP_REFERENCE_SIZE),
                        "xyxy": list(TOP_ROIS[rid])},
            "background_png_base64": base64.b64encode(encoded.tobytes()).decode("ascii"),
            "feature_mean": mean.tolist(), "feature_scale": scale.tolist(),
            "ood_components": components.tolist(),
            "ood_support_coordinates": coordinates.tolist(),
            "ood_nearest_limit": max(1e-8, float(np.quantile(nearest_other_case, .99))) * 1.5,
            # Permit ordinary JPEG/pixel jitter around the learned manifold;
            # grossly different masks remain orders of magnitude farther away.
            "ood_residual_limit": max(math.sqrt(features.shape[1]) * .25,
                                      float(np.quantile(residuals, .99)) * 1.5),
            "trees": forest,
            "diagnostics": {"sample_count": len(samples),
                "case_count": len(set(groups)), "cv_folds": folds,
                "cv_mae": float(errors.mean()),
                "cv_rmse": float(np.sqrt(np.mean((cv - targets) ** 2))),
                "cv_p95_absolute_error": float(np.quantile(errors, .95)),
                "cv_feature_pipeline": "fold-local background and foreground extraction",
                "background_sample_count": min(BACKGROUND_SAMPLES, len(parsed) + 1),
                "inference_boundary": "fixed top RGB ROI only"}}


def predict_geometry(model: dict[str, Any], top_jpeg: bytes) -> dict[str, Any]:
    """Predict continuous axis error using only the current fixed-top RGB."""
    if not isinstance(model, dict) or model.get("schema") != SCHEMA:
        raise ValueError("unsupported geometry model")
    rid, stage = model.get("robot_id"), model.get("stage")
    if rid not in TOP_ROIS or stage not in STAGES:
        raise ValueError("invalid geometry model robot/stage")
    if model.get("top_roi") != {"reference_size": list(TOP_REFERENCE_SIZE),
                                "xyxy": list(TOP_ROIS[rid])}:
        raise ValueError("geometry model top ROI differs from fixed calibration")
    try:
        background_bytes = base64.b64decode(model["background_png_base64"], validate=True)
        background = cv2.imdecode(np.frombuffer(background_bytes, np.uint8), cv2.IMREAD_COLOR)
        mean = np.asarray(model["feature_mean"], dtype=float)
        scale = np.asarray(model["feature_scale"], dtype=float)
        components = np.asarray(model["ood_components"], dtype=float)
        support = np.asarray(model["ood_support_coordinates"], dtype=float)
        trees = model["trees"]
        nearest_limit = float(model["ood_nearest_limit"])
        residual_limit = float(model["ood_residual_limit"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid geometry model data") from exc
    feature_count = len(mean)
    expected_height = TOP_ROIS[rid][3] - TOP_ROIS[rid][1]
    expected_width = TOP_ROIS[rid][2] - TOP_ROIS[rid][0]
    if (background is None or background.shape != (expected_height, expected_width, 3)
            or mean.ndim != 1 or not feature_count or scale.shape != mean.shape
            or components.ndim != 2 or components.shape[1] != feature_count
            or support.ndim != 2 or not len(support) or support.shape[1] != len(components)
            or not isinstance(trees, list) or not trees
            or not math.isfinite(nearest_limit) or not math.isfinite(residual_limit)
            or nearest_limit <= 0 or residual_limit <= 0
            or not all(np.all(np.isfinite(x)) for x in (mean, scale, components, support))
            or np.any(scale <= 0)):
        raise ValueError("invalid geometry model arrays")
    for tree in trees:
        _validate_tree(tree, feature_count)
    roi = _roi(_decode_top(top_jpeg), rid)
    features = _features(roi, background)
    if features.shape != mean.shape or not np.all(np.isfinite(features)):
        raise ValueError("invalid geometry feature vector")
    standardized = (features - mean) / scale
    coordinate = standardized @ components.T
    residual = float(np.linalg.norm(standardized - coordinate @ components))
    nearest = float(np.min(np.linalg.norm(support - coordinate, axis=1)))
    diagnostics = {"nearest_support_distance": nearest,
                   "orthogonal_residual": residual,
                   "support_count": len(support)}
    if nearest > nearest_limit or residual > residual_limit:
        return {"ok": False, "error": 0.0,
                "reason": "rgb_foreground_outside_geometry_support",
                "diagnostics": diagnostics}
    try:
        error = float(np.mean([_tree_predict(tree, features) for tree in trees]))
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError("invalid geometry tree data") from exc
    if not math.isfinite(error):
        raise ValueError("geometry model produced a non-finite error")
    return {"ok": True, "error": error, "reason": "learned_rgb_foreground_geometry",
            "diagnostics": diagnostics}
