"""RGB-only relative evidence that a previously identified cyan box co-moves."""
from __future__ import annotations

import cv2
import numpy as np

from harness.monocular_box import _decode_jpeg


MIN_CLOSE_AREA_PX = 6000
MIN_IOU = .88
MAX_CENTROID_DELTA_PX = 8.0
# Wrist-camera yaw changes the apparent centroid of a box held below the
# camera.  Calibration frames from the 60-PWM attachment probe bound that
# deterministic motion at about 13 px.  Apply the allowance only when the
# caller supplies its own commanded pan displacement; ordinary monitoring
# keeps the tighter stationary-camera gate.
MAX_PAN_CENTROID_PX_PER_PWM = 0.10
# Contour centroids come from integer mask pixels.  Reserve half a pixel for
# boundary quantization on commanded camera-motion comparisons only.
PAN_CENTROID_QUANTIZATION_PX = 0.5
AREA_RATIO_RANGE = (.90, 1.10)


def _cyan_object_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Include both illuminated top and darker vertical cyan faces.
    mask = cv2.inRange(hsv, np.asarray((74, 65, 45)), np.asarray((108, 255, 255)))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return np.zeros_like(mask), 0.0, None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    owned = np.zeros_like(mask)
    cv2.drawContours(owned, [contour], -1, 255, thickness=cv2.FILLED)
    moments = cv2.moments(contour)
    if moments["m00"] <= 0:
        return owned, area, None
    centroid = np.asarray((moments["m10"]/moments["m00"],
                           moments["m01"]/moments["m00"]), dtype=np.float64)
    return owned, area, centroid


def compare_box_comotion(before_image, after_image, *, camera_pan_delta_pwm=0):
    """Compare two controlled-arm RGB frames for relative attachment evidence.

    The cyan color does not identify a box. The caller must have established
    target identity by marker tracking before the intervention. A positive
    result is evidence of visual attachment/co-motion only, not force sensing,
    absolute height, or guaranteed grasp success.
    """
    if isinstance(camera_pan_delta_pwm, bool) or not isinstance(camera_pan_delta_pwm, (int, float)):
        raise ValueError("camera_pan_delta_pwm must be numeric")
    pan_delta = abs(float(camera_pan_delta_pwm))
    if not np.isfinite(pan_delta) or pan_delta > 120:
        raise ValueError("camera_pan_delta_pwm outside calibrated range")
    before = _decode_jpeg(before_image)
    after = _decode_jpeg(after_image)
    if before.shape != after.shape:
        raise ValueError("CAMERA_FRAME_SIZE_CHANGED")
    before_mask, before_area, before_centroid = _cyan_object_mask(before)
    after_mask, after_area, after_centroid = _cyan_object_mask(after)
    base = {
        "evidence": "visual_attachment",
        "attached": False,
        "identity_source": "caller_prior_marker_tracking_required",
        "color_is_identity_evidence": False,
        "provenance": "two_own_rgb_jpegs+controlled_arm_intervention+cyan_mask_comotion",
        "thresholds": {"min_close_area_px": MIN_CLOSE_AREA_PX, "min_iou": MIN_IOU,
            "max_centroid_delta_px": MAX_CENTROID_DELTA_PX,
            "pan_centroid_px_per_pwm": MAX_PAN_CENTROID_PX_PER_PWM,
            "area_ratio_range": list(AREA_RATIO_RANGE)},
        "camera_pan_delta_pwm": float(camera_pan_delta_pwm),
        "before_area_px": before_area,
        "after_area_px": after_area,
    }
    if before_centroid is None or after_centroid is None or min(before_area, after_area) < MIN_CLOSE_AREA_PX:
        return {**base, "reason": "CLOSE_CYAN_OBJECT_NOT_VISIBLE_IN_BOTH_FRAMES"}
    intersection = int(np.count_nonzero((before_mask > 0) & (after_mask > 0)))
    union = int(np.count_nonzero((before_mask > 0) | (after_mask > 0)))
    iou = float(intersection/union) if union else 0.0
    centroid_delta = float(np.linalg.norm(after_centroid-before_centroid))
    area_ratio = float(after_area/before_area)
    metrics = {"mask_iou": iou, "centroid_delta_px": centroid_delta,
               "area_ratio": area_ratio,
               "before_centroid_px": [float(v) for v in before_centroid],
               "after_centroid_px": [float(v) for v in after_centroid]}
    centroid_limit = (MAX_CENTROID_DELTA_PX + pan_delta * MAX_PAN_CENTROID_PX_PER_PWM
                      + (PAN_CENTROID_QUANTIZATION_PX if pan_delta else 0.0))
    metrics["effective_centroid_limit_px"] = centroid_limit
    passed = (iou >= MIN_IOU and centroid_delta <= centroid_limit
              and AREA_RATIO_RANGE[0] <= area_ratio <= AREA_RATIO_RANGE[1])
    return {**base, **metrics, "attached": bool(passed),
            "reason": "VISUAL_ATTACHMENT_SUPPORTED" if passed else "CYAN_OBJECT_DID_NOT_COMOVE_WITH_CAMERA"}
