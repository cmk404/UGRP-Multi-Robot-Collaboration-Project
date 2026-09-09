"""Image-only gripper motion calibration and bounded temporal tracking."""
from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np


def _result(*, valid: bool = False, center=None, axis=None, span=None,
            confidence: float = 0.0, source: str = "unavailable",
            reason: str, tracked_points: int = 0) -> dict[str, Any]:
    return {
        "valid": bool(valid),
        "center": center,
        "opening_axis": axis,
        "span_px": span,
        "confidence": float(max(0.0, min(1.0, confidence))),
        "source": source,
        "reason": reason,
        "tracked_points": int(tracked_points),
    }


def _decode(jpeg: bytes) -> np.ndarray | None:
    if not isinstance(jpeg, bytes) or len(jpeg) < 4:
        return None
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if frame is None or min(frame.shape[:2]) < 16:
        return None
    return frame


class GripperMotionTracker:
    """Learn a gripper anchor from isolated servo-1 motion, then track pixels."""

    def __init__(self, initial_gripper_pulse: int | None = 2000,
                 max_track_age: int = 4) -> None:
        self._prev: np.ndarray | None = None
        self._gripper_pulse = initial_gripper_pulse
        self._max_track_age = max(1, int(max_track_age))
        self._track: dict[str, Any] | None = None

    @staticmethod
    def _normalized(point: np.ndarray, shape: tuple[int, ...]) -> list[float]:
        h, w = shape[:2]
        return [float(point[0] / (w - 1)), float(point[1] / (h - 1))]

    @staticmethod
    def _gripper_change(action: dict | None, prior: int | None) -> tuple[bool, int | None]:
        if not isinstance(action, dict) or action.get("kind") != "arm" or action.get("servo_id") != 1:
            return False, prior
        pulse = action.get("pulse")
        if isinstance(pulse, bool) or not isinstance(pulse, int) or not 500 <= pulse <= 2500:
            return False, prior
        return prior is not None and pulse != prior, pulse

    def _calibrate(self, old: np.ndarray, new: np.ndarray) -> dict[str, Any]:
        if old.shape != new.shape:
            self._track = None
            return _result(reason="frame shape changed during gripper calibration")
        h, w = new.shape[:2]
        color_delta = np.max(cv2.absdiff(old, new), axis=2)
        mask = (color_delta > 15).astype(np.uint8)
        raw_y, raw_x = np.nonzero(mask)
        if len(raw_x):
            raw_width = int(raw_x.max() - raw_x.min() + 1)
            raw_height = int(raw_y.max() - raw_y.min() + 1)
            if (raw_width >= 0.45 * w or raw_height >= 0.45 * h
                    or raw_width * raw_height >= 0.15 * h * w
                    or len(raw_x) >= 0.05 * h * w):
                self._track = None
                return _result(reason="motion is not a compact gripper region")
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        if count <= 1:
            self._track = None
            return _result(reason="no measurable isolated gripper motion")
        largest = int(np.max(stats[1:, cv2.CC_STAT_AREA]))
        minimum = max(8, int(round(h * w * 0.000025)), int(math.ceil(largest * 0.12)))
        ids = [i for i in range(1, count) if int(stats[i, cv2.CC_STAT_AREA]) >= minimum]
        if len(ids) < 2:
            self._track = None
            return _result(reason="fewer than two meaningful motion lobes")
        if len(ids) > 8:
            self._track = None
            return _result(reason="ambiguous multi-region motion")

        selected = np.isin(labels, ids).astype(np.uint8)
        ys, xs = np.nonzero(selected)
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        bbox_area = (x1 - x0 + 1) * (y1 - y0 + 1)
        if (bbox_area >= 0.15 * h * w or (x1 - x0 + 1) >= 0.45 * w
                or (y1 - y0 + 1) >= 0.45 * h
                or len(xs) >= 0.05 * h * w):
            self._track = None
            return _result(reason="motion is not a compact gripper region")

        component_centers = centroids[ids].astype(np.float64)
        centered = component_centers - component_centers.mean(axis=0)
        covariance = centered.T @ centered / max(1, len(ids) - 1)
        values, vectors = np.linalg.eigh(covariance)
        if values[-1] < 9.0 or values[-1] < 1.8 * max(values[-2], 1e-6):
            self._track = None
            return _result(reason="motion lobes have no unambiguous opening axis")
        axis = vectors[:, -1]
        axis /= np.linalg.norm(axis)
        if axis[1] < 0 or (abs(axis[1]) < 1e-9 and axis[0] < 0):
            axis = -axis
        projections = component_centers @ axis
        span = float(projections.max() - projections.min())
        if not math.isfinite(span) or span < 5.0:
            self._track = None
            return _result(reason="motion lobe span is too small")

        points = np.column_stack((xs, ys)).astype(np.float64)
        center = points.mean(axis=0)
        # One-pixel support around the changed pixels admits edge corners while
        # excluding nearby static chassis texture from the gripper track.
        feature_mask = cv2.dilate(selected * 255, np.ones((3, 3), np.uint8), iterations=1)
        features = cv2.goodFeaturesToTrack(
            cv2.cvtColor(new, cv2.COLOR_BGR2GRAY), maxCorners=80,
            qualityLevel=0.005, minDistance=2, mask=feature_mask, blockSize=3,
        )
        tracked_points = 0 if features is None else len(features)
        if tracked_points < 3:
            self._track = None
            return _result(reason="gripper motion region lacks trackable image features")

        balance = min(stats[i, cv2.CC_STAT_AREA] for i in ids) / max(
            stats[i, cv2.CC_STAT_AREA] for i in ids
        )
        contrast = min(1.0, float(np.mean(color_delta[selected > 0])) / 35.0)
        confidence = min(0.95, 0.55 + 0.20 * float(balance) + 0.20 * contrast)
        self._track = {
            "center": center, "axis": axis, "span": span,
            "features": features.astype(np.float32), "mask": feature_mask,
            "age": 0, "confidence": confidence, "seed_confidence": confidence,
        }
        return _result(
            valid=True, center=self._normalized(center, new.shape), axis=axis.tolist(),
            span=span, confidence=confidence, source="isolated_gripper_motion",
            reason="compact two-or-more-lobe servo-1 image motion calibrated",
            tracked_points=tracked_points,
        )

    def _track_frame(self, old: np.ndarray, new: np.ndarray) -> dict[str, Any]:
        track = self._track
        if track is None:
            return _result(reason="gripper motion has not been calibrated")
        if track["age"] >= self._max_track_age:
            self._track = None
            return _result(reason="gripper image track expired")
        if old.shape != new.shape or float(np.std(new)) < 2.0:
            self._track = None
            return _result(reason="gripper region occluded or frame unusable")

        old_gray = cv2.cvtColor(old, cv2.COLOR_BGR2GRAY)
        new_gray = cv2.cvtColor(new, cv2.COLOR_BGR2GRAY)
        p0 = track["features"]
        params = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        p1, status1, _ = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **params)
        if p1 is None or status1 is None:
            self._track = None
            return _result(reason="forward optical flow failed")
        p0_back, status2, _ = cv2.calcOpticalFlowPyrLK(new_gray, old_gray, p1, None, **params)
        if p0_back is None or status2 is None:
            self._track = None
            return _result(reason="backward optical flow failed")
        good = ((status1.reshape(-1) == 1) & (status2.reshape(-1) == 1)
                & (np.linalg.norm(p0_back.reshape(-1, 2) - p0.reshape(-1, 2), axis=1) <= 1.5))
        before, after = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
        if len(before) < 3:
            self._track = None
            return _result(reason="too few forward-backward-consistent gripper points")
        affine, inliers = cv2.estimateAffinePartial2D(
            before, after, method=cv2.RANSAC, ransacReprojThreshold=2.0,
            maxIters=1000, confidence=0.99,
        )
        if affine is None or inliers is None:
            self._track = None
            return _result(reason="robust gripper affine fit failed")
        keep = inliers.reshape(-1).astype(bool)
        inlier_count = int(keep.sum())
        inlier_ratio = inlier_count / len(before)
        residual = np.linalg.norm(
            cv2.transform(before.reshape(-1, 1, 2), affine).reshape(-1, 2) - after,
            axis=1,
        )
        linear = affine[:, :2].astype(np.float64)
        scale = math.sqrt(abs(float(np.linalg.det(linear))))
        if (inlier_count < 3 or inlier_ratio < 0.6 or not np.all(np.isfinite(affine))
                or not 0.7 <= scale <= 1.3 or float(np.median(residual[keep])) > 1.5):
            self._track = None
            return _result(reason="gripper affine fit is ambiguous or inconsistent")

        center = linear @ track["center"] + affine[:, 2]
        axis = linear @ track["axis"]
        norm = float(np.linalg.norm(axis))
        axis = axis / norm if norm > 1e-9 else axis
        h, w = new.shape[:2]
        if (norm <= 1e-9 or not np.all(np.isfinite(center)) or center[0] < 0
                or center[0] > w - 1 or center[1] < 0 or center[1] > h - 1):
            self._track = None
            return _result(reason="tracked gripper geometry left finite image bounds")
        span = float(track["span"] * scale)
        age = int(track["age"]) + 1
        confidence = min(float(track["confidence"]),
                         float(track["seed_confidence"]) - 0.07 * age,
                         0.45 + 0.5 * inlier_ratio)
        if confidence < 0.6:
            self._track = None
            return _result(reason="gripper image track confidence fell below threshold")
        next_features = after[keep].reshape(-1, 1, 2).astype(np.float32)
        warped_mask = cv2.warpAffine(track["mask"], affine, (w, h), flags=cv2.INTER_NEAREST)
        self._track = {
            "center": center, "axis": axis, "span": span, "features": next_features,
            "mask": warped_mask, "age": age, "confidence": confidence,
            "seed_confidence": track["seed_confidence"],
        }
        return _result(
            valid=True, center=self._normalized(center, new.shape), axis=axis.tolist(),
            span=span, confidence=confidence, source="verified_optical_flow",
            reason="forward-backward gripper-region flow with robust affine fit",
            tracked_points=inlier_count,
        )

    def update(self, top_jpeg: bytes, previous_action: dict | None) -> dict[str, Any]:
        frame = _decode(top_jpeg)
        if frame is None:
            self._prev = None
            self._track = None
            return _result(reason="top_jpeg is not a decodable image")
        changed, pulse = self._gripper_change(previous_action, self._gripper_pulse)
        self._gripper_pulse = pulse
        if self._prev is None:
            result = _result(reason="waiting for a prior top-camera frame")
        elif changed:
            result = self._calibrate(self._prev, frame)
        else:
            result = self._track_frame(self._prev, frame)
        self._prev = frame
        return result
