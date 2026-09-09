"""Conservative marker-free observation of one cyan floor box from own RGB."""
from __future__ import annotations

import base64
import math
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from harness.monocular_box import _decode_jpeg
from harness.visual_arm import camera_extrinsics
from harness.visual_box_surface import BOX_HEIGHT_M, BOX_TOP_DIMS_M
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

_PROVENANCE = "own_rgb_cyan_silhouette+raw_fisheye+own_camera_fk+known_floor_cuboid_projection"
_MIN_AREA_PX = 90.0
_BORDER_PX = 3
_MIN_SOLIDITY = 0.82
_MIN_PROJECTION_IOU = 0.58
_MIN_TOP_PROJECTION_IOU = 0.70


def _invisible(target_id: str, reason: str, image_size=None, **details):
    result = {"target_id": target_id, "visible": False, "reason": reason,
              "ambiguity_reason": reason, "provenance": _PROVENANCE,
              "identity_source": "task_catalog_reference_only_not_visually_decoded"}
    if image_size is not None:
        result["image_size_px"] = list(image_size)
    result.update(details)
    return result


def _cyan_components(frame: np.ndarray):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Saved seed-46 release views put the cyan cargo at H>=73.  Exact-H72
    # pixels occur along the dark lower/reflection boundary and change the
    # fitted silhouette with camera pan.  Exclude that measured edge while
    # retaining the existing H104 blue-floor boundary. Projection, ambiguity,
    # and ground-stationarity checks remain independent and unchanged.
    mask = cv2.inRange(hsv, np.asarray((73, 65, 45)), np.asarray((104, 255, 245)))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = frame.shape[:2]
    accepted = []
    clipped = False
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < _MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= _BORDER_PX or y <= _BORDER_PX or x + w >= width-_BORDER_PX or y + h >= height-_BORDER_PX:
            clipped = True
            continue
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        solidity = area / max(hull_area, 1.0)
        if solidity < _MIN_SOLIDITY or w < 7 or h < 7:
            continue
        accepted.append({"contour": contour, "area": area, "solidity": solidity,
                         "bbox": (x, y, w, h), "mask": mask})
    return accepted, clipped


def _bright_top_quads(frame: np.ndarray):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width = frame.shape[:2]
    quads = []
    # A fixed bright threshold can join the top to a lit front face.  Sweep a
    # small calibrated set; every result still has to pass the same full top
    # geometry, height, and projection checks below.
    for value_floor in (152, 170, 185, 200):
        mask = cv2.inRange(
            hsv, np.asarray((75, 70, value_floor)), np.asarray((105, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 180:
                continue
            hull = cv2.convexHull(contour)
            perimeter = float(cv2.arcLength(hull, True))
            quad = cv2.approxPolyDP(hull, .02*perimeter, True).reshape(-1, 2)
            if len(quad) != 4 or not cv2.isContourConvex(quad.astype(np.float32)):
                continue
            if (np.any(quad[:, 0] <= 2) or np.any(quad[:, 0] >= width-3) or
                    np.any(quad[:, 1] <= 2) or np.any(quad[:, 1] >= height-3)):
                continue
            solidity = area/max(float(cv2.contourArea(hull)), 1.0)
            if solidity >= .90:
                quads.append((contour, quad.astype(np.float64), solidity,
                              value_floor))
    return quads


def _fit_floor_top_quad(contour, quad, solidity, origin, axes, k, d,
                        dimensions, image_shape):
    rays = cv2.fisheye.undistortPoints(quad.reshape(1, 4, 2), k, d).reshape(4, 2)
    directions = np.column_stack((rays, np.ones(4))) @ axes
    if not np.all(np.isfinite(directions)) or np.any(directions[:, 2] >= -.02):
        return None
    slopes = directions[:, :2]/directions[:, 2, None]
    coeff = np.linalg.norm(np.roll(slopes, -1, axis=0)-slopes, axis=1)
    if np.any(coeff <= 1e-6):
        return None
    a, b, box_height = dimensions
    patterns = (np.asarray((a, b, a, b)), np.asarray((b, a, b, a)))
    fits = []
    for expected in patterns:
        camera_above = float(coeff @ expected / (coeff @ coeff))
        measured = camera_above*coeff
        relative_errors = abs(measured-expected)/expected
        rmse = float(np.sqrt(np.mean((measured-expected)**2)))
        fits.append((rmse, camera_above, expected, relative_errors))
    rmse, camera_above, expected, relative_errors = min(fits, key=lambda item: item[0])
    top_z = float(origin[2]-camera_above)
    if abs(top_z-box_height) > .008 or rmse > .003 or np.max(relative_errors) > .12:
        return None
    scales = (top_z-origin[2])/directions[:, 2]
    points = origin+directions*scales[:, None]
    edges = np.roll(points[:, :2], -1, axis=0)-points[:, :2]
    lengths = np.linalg.norm(edges, axis=1)
    cosines = [abs(float(edges[i] @ edges[(i+1) % 4]) /
                   max(lengths[i]*lengths[(i+1) % 4], 1e-9)) for i in range(4)]
    if max(cosines) > .15:
        return None
    center = np.mean(points[:, :2], axis=0)
    first_dim = float(expected[0])
    yaw = math.atan2(float(edges[0, 1]), float(edges[0, 0]))
    axis = np.asarray((math.cos(yaw), math.sin(yaw)))
    cross = np.asarray((-axis[1], axis[0]))
    other_dim = float(expected[1])
    ideal = np.asarray([[center[0]+su*axis[0]*first_dim/2+sv*cross[0]*other_dim/2,
                         center[1]+su*axis[1]*first_dim/2+sv*cross[1]*other_dim/2,
                         box_height]
                        for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    projected = _project_points(ideal, origin, axes, k, d)
    if projected is None:
        return None
    projected_cuboid = _project_points(
        _cuboid_corners(center, yaw, dimensions), origin, axes, k, d)
    if projected_cuboid is None:
        return None
    iou = _polygon_iou(contour, projected, image_shape)
    if iou < _MIN_TOP_PROJECTION_IOU:
        return None
    moments = cv2.moments(contour)
    pixel_centroid = (np.mean(quad, axis=0) if moments["m00"] == 0 else
                      np.asarray((moments["m10"]/moments["m00"],
                                  moments["m01"]/moments["m00"])))
    return {"center": center, "yaw": yaw % math.pi, "iou": iou,
            "top_z": top_z, "edge_rmse": rmse,
            "edge_relative_errors": relative_errors, "right_angle_cosines": cosines,
            "projected": projected, "projected_cuboid": projected_cuboid,
            "solidity": solidity,
            "pixel_centroid": pixel_centroid,
            "pixel_bbox": cv2.boundingRect(contour),
            "area_px": float(cv2.contourArea(contour))}


def _observe_floor_top_face(frame, origin, axes, k, d, dimensions):
    fits = []
    for contour, quad, solidity, value_floor in _bright_top_quads(frame):
        fit = _fit_floor_top_quad(contour, quad, solidity, origin, axes, k, d,
                                  dimensions, frame.shape)
        if fit is not None:
            fit["value_floor"] = value_floor
            fits.append(fit)
    clusters = []
    for fit in sorted(fits, key=lambda item: item["iou"], reverse=True):
        matching = []
        for cluster in clusters:
            representative = cluster[0]
            center_close = np.linalg.norm(
                fit["center"]-representative["center"]) <= .012
            footprint_iou = _polygon_iou(
                cv2.convexHull(fit["projected"].astype(np.float32)),
                representative["projected"], frame.shape)
            if center_close and footprint_iou >= .55:
                matching.append(cluster)
        if len(matching) == 1:
            matching[0].append(fit)
        else:
            # Zero matches is a distinct object. Multiple matches are kept
            # ambiguous rather than joining spatially separate objects.
            clusters.append([fit])
    if len(clusters) != 1:
        return None, len(clusters)
    # Preserve the original V152 estimate whenever that path validates. The
    # additional thresholds recover only frames where the original topology
    # cannot form a geometry-valid top.
    original = [fit for fit in clusters[0] if fit["value_floor"] == 152]
    selected = max(original or clusters[0], key=lambda item: item["iou"])
    return selected, 1


def _pixel_ground_point(pixel, origin, axes, k, d):
    undistorted = cv2.fisheye.undistortPoints(
        np.asarray(pixel, np.float64).reshape(1, 1, 2), k, d).reshape(2)
    optical = np.asarray((undistorted[0], undistorted[1], 1.0), np.float64)
    direction = optical @ axes
    if not np.all(np.isfinite(direction)) or direction[2] >= -0.02:
        return None
    scale = -origin[2] / direction[2]
    if not math.isfinite(float(scale)) or scale <= 0:
        return None
    return origin + direction * scale


def _project_points(points_base, origin, axes, k, d):
    optical = (np.asarray(points_base, np.float64) - origin) @ np.linalg.inv(axes)
    if np.any(optical[:, 2] <= 1e-5) or not np.all(np.isfinite(optical)):
        return None
    normalized = (optical[:, :2] / optical[:, 2, None]).reshape(-1, 1, 2)
    pixels = cv2.fisheye.distortPoints(normalized, k, d).reshape(-1, 2)
    return pixels if np.all(np.isfinite(pixels)) else None


def _cuboid_corners(center_xy, yaw, dimensions):
    a, b, height = dimensions
    c, s = math.cos(yaw), math.sin(yaw)
    u = np.asarray((c, s)) * (a/2.0)
    v = np.asarray((-s, c)) * (b/2.0)
    center = np.asarray(center_xy, np.float64)
    return np.asarray([[(center + su*u + sv*v)[0], (center + su*u + sv*v)[1], z]
                       for z in (0.0, height) for su in (-1, 1) for sv in (-1, 1)],
                      np.float64)


def _polygon_iou(contour, projected, image_shape):
    observed = cv2.convexHull(contour).reshape(-1, 2).astype(np.int32)
    predicted = cv2.convexHull(projected.astype(np.float32)).reshape(-1, 2).astype(np.int32)
    all_points = np.vstack((observed, predicted))
    x, y, w, h = cv2.boundingRect(all_points)
    height, width = image_shape[:2]
    x0, y0, x1, y1 = max(0, x), max(0, y), min(width, x+w), min(height, y+h)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    om = np.zeros((y1-y0, x1-x0), np.uint8)
    pm = np.zeros_like(om)
    offset = np.asarray((x0, y0))
    cv2.fillConvexPoly(om, observed-offset, 1)
    cv2.fillConvexPoly(pm, predicted-offset, 1)
    union = int(np.count_nonzero(om | pm))
    return float(np.count_nonzero(om & pm) / union) if union else 0.0


def _fit_floor_cuboid(component, origin, axes, k, d, dimensions, image_shape):
    contour = component["contour"].reshape(-1, 2)
    max_y = float(np.max(contour[:, 1]))
    band = contour[contour[:, 1] >= max_y-max(2.0, component["bbox"][3]*0.08)]
    contacts = [_pixel_ground_point(pixel, origin, axes, k, d) for pixel in band]
    contacts = np.asarray([point for point in contacts if point is not None])
    if len(contacts) < 1:
        return None
    contact = np.median(contacts[:, :2], axis=0)
    radial = contact - origin[:2]
    radial_norm = float(np.linalg.norm(radial))
    # At the calibrated 14.5 cm grasp radius the arm-mounted camera can be
    # only about 7.5 cm from the near floor contact.  Do not confuse that
    # camera-relative distance with the target's base-frame grasp radius.
    if radial_norm <= 1e-6 or radial_norm > 1.2:
        return None
    radial /= radial_norm
    lateral = np.asarray((-radial[1], radial[0]))
    a, b, height = dimensions
    candidates = []
    for yaw_deg in range(0, 180, 5):
        yaw = math.radians(yaw_deg)
        axis_a = np.asarray((math.cos(yaw), math.sin(yaw)))
        axis_b = np.asarray((-math.sin(yaw), math.cos(yaw)))
        support = abs(float(axis_a @ radial))*a/2 + abs(float(axis_b @ radial))*b/2
        base_center = contact + radial*support
        for radial_delta in (-0.008, 0.0, 0.008):
            for lateral_delta in (-0.006, 0.0, 0.006):
                center = base_center + radial*radial_delta + lateral*lateral_delta
                corners = _cuboid_corners(center, yaw, (a, b, height))
                pixels = _project_points(corners, origin, axes, k, d)
                if pixels is None:
                    continue
                iou = _polygon_iou(component["contour"], pixels, image_shape)
                candidates.append((iou, center, yaw, pixels,
                                   radial_delta, lateral_delta))
    if not candidates:
        return None

    # Keep the calibrated coarse search as the global initializer.  Refine
    # only its best hypothesis in a bounded local neighborhood: at most
    # 9 * 9 * 9 = 729 additional projections.  The +/-4 mm neighborhood
    # also lets a coarse edge hypothesis at +/-6 mm reach +/-10 mm without
    # widening the accepted projection or stationarity thresholds.
    refined = list(candidates)
    for _, _, coarse_yaw, _, coarse_radial, coarse_lateral in sorted(
            candidates, key=lambda item: item[0], reverse=True)[:1]:
        coarse_yaw_deg = round(math.degrees(coarse_yaw))
        for yaw_delta_deg in range(-4, 5):
            yaw = math.radians((coarse_yaw_deg + yaw_delta_deg) % 180)
            axis_a = np.asarray((math.cos(yaw), math.sin(yaw)))
            axis_b = np.asarray((-math.sin(yaw), math.cos(yaw)))
            support = (abs(float(axis_a @ radial))*a/2
                       + abs(float(axis_b @ radial))*b/2)
            base_center = contact + radial*support
            for radial_mm in range(-4, 5):
                radial_delta = coarse_radial + radial_mm/1000.0
                for lateral_mm in range(-4, 5):
                    lateral_delta = coarse_lateral + lateral_mm/1000.0
                    center = (base_center + radial*radial_delta
                              + lateral*lateral_delta)
                    pixels = _project_points(
                        _cuboid_corners(center, yaw, (a, b, height)),
                        origin, axes, k, d)
                    if pixels is None:
                        continue
                    iou = _polygon_iou(
                        component["contour"], pixels, image_shape)
                    refined.append((iou, center, yaw, pixels,
                                    radial_delta, lateral_delta))
    best = max(refined, key=lambda item: item[0])
    return best[:4]


def observe_ground_box(image, servo_pose: Mapping[int | str, int | float],
                       target_id: str = "small_box_01") -> dict[str, Any]:
    """Estimate one upright cyan cuboid center conditional on a floor hypothesis.

    ``target_id`` selects known catalog dimensions; cyan pixels do not decode
    identity. Base axes are +x forward, +y left, +z up. A successful result is
    a projection-validated floor hypothesis, not independent proof of contact.
    """
    if target_id not in BOX_TOP_DIMS_M or target_id not in BOX_HEIGHT_M:
        raise ValueError("UNKNOWN_BOX_ID")
    frame = _decode_jpeg(image)
    height, width = frame.shape[:2]
    components, clipped = _cyan_components(frame)
    origin, axes = camera_extrinsics(servo_pose)
    origin = np.asarray(origin, np.float64)
    axes = np.asarray(axes, np.float64)
    k = scaled_camera_matrix(width, height)
    d = np.asarray(CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    a, b = BOX_TOP_DIMS_M[target_id]
    box_height = BOX_HEIGHT_M[target_id]
    dimensions = (a, b, box_height)
    plausible_components = []
    for candidate in components:
        candidate_fit = _fit_floor_cuboid(
            candidate, origin, axes, k, d, dimensions, frame.shape)
        if candidate_fit is not None and candidate_fit[0] >= _MIN_PROJECTION_IOU:
            plausible_components.append((candidate, candidate_fit))
    if len(plausible_components) > 1:
        return _invisible(target_id, "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES",
                          (width, height), candidate_count=len(plausible_components))
    if plausible_components:
        component, fit = plausible_components[0]
    else:
        component = components[0] if len(components) == 1 else None
        fit = (_fit_floor_cuboid(component, origin, axes, k, d, dimensions, frame.shape)
               if component is not None else None)
    iou = fit[0] if fit is not None else 0.0
    top_fit, top_count = _observe_floor_top_face(frame, origin, axes, k, d, dimensions)
    if top_count > 1:
        return _invisible(target_id, "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES",
                          (width, height), candidate_count=max(len(components), top_count))
    # Preserve the established silhouette result when it already validates.
    # Extra photometric thresholds are recovery views, not a reason to replace
    # an otherwise valid estimate. The original V152 top path keeps priority.
    if (top_fit is not None and top_fit["value_floor"] != 152
            and fit is not None and iou >= _MIN_PROJECTION_IOU):
        top_fit = None
    if top_fit is not None or fit is None or iou < _MIN_PROJECTION_IOU:
        if top_fit is not None:
            envelope = cv2.convexHull(
                top_fit["projected_cuboid"].astype(np.float32)).reshape(-1, 2)
            other_plausible = 0
            for candidate in components:
                moments = cv2.moments(candidate["contour"])
                candidate_centroid = (
                    np.mean(candidate["contour"].reshape(-1, 2), axis=0)
                    if moments["m00"] == 0 else
                    np.asarray((moments["m10"]/moments["m00"],
                                moments["m01"]/moments["m00"])))
                # A disconnected photometric face belongs to this top only
                # when its centroid lies in the calibrated known-cuboid
                # projection. The small tolerance is rasterization only.
                if cv2.pointPolygonTest(envelope, tuple(candidate_centroid), True) >= -2.0:
                    continue
                candidate_fit = _fit_floor_cuboid(
                    candidate, origin, axes, k, d, dimensions, frame.shape)
                if candidate_fit is not None and candidate_fit[0] >= _MIN_PROJECTION_IOU:
                    other_plausible += 1
            if other_plausible:
                return _invisible(
                    target_id, "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES",
                    (width, height), candidate_count=1+other_plausible)
            center, yaw = top_fit["center"], top_fit["yaw"]
            return {"target_id": target_id, "visible": True,
                    "reason": "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED",
                    "ambiguity_reason": None,
                    "pixel_centroid": [float(v) for v in top_fit["pixel_centroid"]],
                    "pixel_bbox": [int(v) for v in top_fit["pixel_bbox"]],
                    "area_px": float(top_fit["area_px"]),
                    "estimated_box_center_base_m": [float(center[0]), float(center[1]),
                                                    float(top_fit["top_z"]-box_height/2)],
                    "estimated_yaw_mod_pi_rad": float(yaw),
                    "known_box_dimensions_m": [a, b, box_height],
                    "measured_top_height_base_m": float(top_fit["top_z"]),
                    "expected_floor_top_height_base_m": float(box_height),
                    "floor_top_height_residual_m": float(abs(top_fit["top_z"]-box_height)),
                    "top_edge_fit_rmse_m": float(top_fit["edge_rmse"]),
                    "top_edge_relative_errors": [float(v) for v in top_fit["edge_relative_errors"]],
                    "top_right_angle_abs_cosines": [float(v) for v in top_fit["right_angle_cosines"]],
                    "floor_hypothesis_projection_iou": float(top_fit["iou"]),
                    "floor_hypothesis_projection_residual": float(1-top_fit["iou"]),
                    "projection_scope": "full known-size top rectangle at calibrated expected floor-top height",
                    "projected_top_face_pixels": [[float(x), float(y)] for x, y in top_fit["projected"]],
                    "confidence": float(np.clip(top_fit["solidity"]*top_fit["iou"], 0., 1.)),
                    "identity_source": "task_catalog_reference_only_not_visually_decoded",
                    "assumptions": ["exactly one fully visible bright cyan top rectangle",
                                    "upright known-size cuboid", "measured top plane agrees with floor contact"],
                    "measurement_scope": "full top rectangle center and height; floor contact inferred from measured top height",
                    "provenance": _PROVENANCE+"+measured_full_top_rectangle"}
        if len(components) != 1:
            if not components and top_count == 0:
                reason = "CYAN_SILHOUETTE_FRAME_CLIPPED" if clipped else "CYAN_SILHOUETTE_NOT_VISIBLE"
                return _invisible(target_id, reason, (width, height))
            return _invisible(target_id, "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES",
                              (width, height), candidate_count=max(len(components), top_count))
        if fit is None:
            return _invisible(target_id, "FLOOR_CUBOID_GEOMETRY_UNRESOLVED", (width, height))
        return _invisible(target_id, "FLOOR_HYPOTHESIS_PROJECTION_MISMATCH", (width, height),
                          floor_hypothesis_projection_iou=iou,
                          floor_hypothesis_projection_residual=1.0-iou)
    iou, center, yaw, projected = fit
    residual = 1.0-iou
    moments = cv2.moments(component["contour"])
    centroid = [moments["m10"]/moments["m00"], moments["m01"]/moments["m00"]]
    confidence = float(np.clip(component["solidity"] * iou, 0.0, 1.0))
    return {"target_id": target_id, "visible": True, "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
            "ambiguity_reason": None, "pixel_centroid": [float(v) for v in centroid],
            "pixel_bbox": [int(v) for v in component["bbox"]], "area_px": component["area"],
            "estimated_box_center_base_m": [float(center[0]), float(center[1]), float(box_height/2)],
            "estimated_yaw_mod_pi_rad": float(yaw), "known_box_dimensions_m": [a, b, box_height],
            "floor_hypothesis_projection_iou": float(iou),
            "floor_hypothesis_projection_residual": float(residual),
            "projected_silhouette_pixels": [[float(x), float(y)] for x, y in cv2.convexHull(projected.astype(np.float32)).reshape(-1, 2)],
            "confidence": confidence,
            "identity_source": "task_catalog_reference_only_not_visually_decoded",
            "assumptions": ["single fully visible cyan silhouette", "upright known-size cuboid",
                            "box bottom lies on calibrated robot-base floor plane"],
            "measurement_scope": "center estimate conditional on floor cuboid projection fit; not independent floor-contact proof",
            "provenance": _PROVENANCE}


__all__ = ["observe_ground_box"]
