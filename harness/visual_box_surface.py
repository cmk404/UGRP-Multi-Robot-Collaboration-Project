"""RGB-only close-range height estimate for a known upright cyan box top."""
from __future__ import annotations

import cv2
import numpy as np

from harness.monocular_box import _decode_jpeg
from harness.visual_arm import camera_extrinsics
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix


BOX_TOP_DIMS_M = {"small_box_01": (0.034, 0.040), "small_box_02": (0.036, 0.038), "small_box_03": (0.034, 0.040)}
BOX_HEIGHT_M = {"small_box_01": 0.032, "small_box_02": 0.034, "small_box_03": 0.032}


def _invisible(target_id, reason, image_size=None):
    result = {"target_id": target_id, "visible": False, "reason": reason,
              "provenance": "own_rgb_cyan_top_surface+known_upright_box_geometry"}
    if image_size is not None:
        result["image_size_px"] = list(image_size)
    return result


def _cyan_top_quad(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # The manufactured cyan top is the high-value face; the vertical body face
    # is deliberately darker under the same light.
    mask = cv2.inRange(hsv, np.asarray((75, 70, 152)), np.asarray((105, 255, 220)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    height, width = frame.shape[:2]
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 180:
            continue
        hull = cv2.convexHull(contour)
        perimeter = float(cv2.arcLength(hull, True))
        quad = cv2.approxPolyDP(hull, .02 * perimeter, True).reshape(-1, 2)
        if len(quad) != 4 or not cv2.isContourConvex(quad.astype(np.float32)):
            continue
        if np.any(quad[:, 0] <= 2) or np.any(quad[:, 0] >= width-3) or np.any(quad[:, 1] <= 2) or np.any(quad[:, 1] >= height-3):
            continue
        solidity = area / max(float(cv2.contourArea(hull)), 1.)
        if solidity < .90:
            continue
        candidates.append((area, solidity, quad.astype(np.float64)))
    return max(candidates, default=None, key=lambda item: item[0])


def observe_known_box_top(image, servo_pose, target_id="small_box_01"):
    """Estimate an already-identified upright box centre from its cyan top.

    This color surface is not an identity signal. Callers may use it only after
    a unique visual target was bound during the same approach.
    Base axes are +x forward, +y left, +z up.
    """
    if target_id not in BOX_TOP_DIMS_M:
        raise ValueError("UNKNOWN_BOX_ID")
    frame = _decode_jpeg(image)
    height, width = frame.shape[:2]
    candidate = _cyan_top_quad(frame)
    if candidate is None:
        return _invisible(target_id, "CYAN_TOP_QUAD_NOT_VISIBLE", (width, height))
    area, solidity, quad = candidate
    k = scaled_camera_matrix(width, height)
    d = np.asarray(CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    rays = cv2.fisheye.undistortPoints(quad.reshape(1, 4, 2), k, d).reshape(4, 2)
    optical = np.column_stack((rays, np.ones(4)))
    origin, axes = camera_extrinsics(servo_pose)
    origin = np.asarray(origin, np.float64)
    directions = optical @ np.asarray(axes, np.float64)
    if not np.all(np.isfinite(directions)) or np.any(directions[:, 2] >= -.02):
        return _invisible(target_id, "TOP_RAYS_NOT_DOWNWARD", (width, height))
    slopes = directions[:, :2] / directions[:, 2, None]
    edge_coeff = np.linalg.norm(np.roll(slopes, -1, axis=0)-slopes, axis=1)
    if np.any(edge_coeff <= 1e-6):
        return _invisible(target_id, "DEGENERATE_TOP_QUAD", (width, height))
    a, b = BOX_TOP_DIMS_M[target_id]
    # Illumination separates the high-value top as a bright strip: its two
    # short photometric edges need not be the physical top boundary. The two
    # long opposite edges remain the full manufactured long dimension.
    pixel_edges = np.linalg.norm(np.roll(quad, -1, axis=0)-quad, axis=1)
    pair = (0, 2) if np.mean(pixel_edges[[0, 2]]) >= np.mean(pixel_edges[[1, 3]]) else (1, 3)
    long_dimension = max(a, b)
    independent_heights = long_dimension / edge_coeff[list(pair)]
    camera_above = float(np.mean(independent_heights))
    edge_rmse = float(np.std(independent_heights) * long_dimension / max(camera_above, 1e-9))
    pair_disagreement = float(abs(independent_heights[0]-independent_heights[1]) / camera_above)
    top_z = float(origin[2]-camera_above)
    if pair_disagreement > .18 or not .0 < camera_above < 1.0 or not -.01 <= top_z <= .35:
        return _invisible(target_id, "TOP_GEOMETRY_AMBIGUOUS", (width, height))
    scales = (top_z-origin[2]) / directions[:, 2]
    points = origin + directions * scales[:, None]
    surface_patch = np.mean(points, axis=0)
    surface_patch[2] = top_z
    box_center_height = top_z - BOX_HEIGHT_M[target_id] / 2.0
    edge_lengths = np.linalg.norm(np.roll(points[:, :2], -1, axis=0)-points[:, :2], axis=1)
    confidence = float(np.clip(solidity * (1-pair_disagreement/.18), 0., 1.))
    return {
        "target_id": target_id,
        "visible": True,
        "pixel_corners": [[float(x), float(y)] for x, y in quad],
        "pixel_centroid": [float(v) for v in np.mean(quad, axis=0)],
        "area_px": area,
        "estimated_top_height_base_m": top_z,
        "estimated_surface_patch_base_m": [float(v) for v in surface_patch],
        "estimated_box_center_height_m": float(box_center_height),
        "measured_top_edge_lengths_m": [float(v) for v in edge_lengths],
        "known_top_dimensions_m": [a, b],
        "edge_fit_rmse_m": edge_rmse,
        "opposite_long_edge_height_disagreement_ratio": pair_disagreement,
        "measurement_scope": "height from two full long top edges; XY is bright surface-patch center, not full box center",
        "confidence": confidence,
        "identity_source": "caller_prior_visual_target_binding_required",
        "provenance": "own_rgb_cyan_top_quad+raw_fisheye_rays+own_camera_fk+known_upright_box_dimensions",
    }
