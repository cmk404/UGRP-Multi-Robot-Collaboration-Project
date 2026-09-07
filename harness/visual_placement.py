"""Conservative camera-only evidence for small-box placement.

The result is evidence for a caller's release/finish decision.  This module
does not select navigation actions and imports no simulator state or control.
"""
from __future__ import annotations

import base64
import hashlib
import math
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from harness.monocular_box import BOX_MARKER_IDS, observe_box
from harness.visual_arm import camera_to_base, forward_grip
from sim.navigation_camera_profile import (
    NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M, NAV_CX_PX, NAV_CY_PX, NAV_FX_PX,
    NAV_FY_PX, NAV_MOUNT_XYZ_M, NAV_PITCH_DOWN_DEG,
)


_ZONE_HSV = {
    "A": ((100, 105, 35), (132, 255, 255)),
    "B": ((40, 85, 30), (84, 255, 255)),
    "C": ((22, 105, 55), (38, 255, 255)),
}
_BOX_DIMENSIONS = {
    "small_box_01": (0.034, 0.040, 0.032),
    "small_box_02": (0.036, 0.038, 0.034),
    "small_box_03": (0.034, 0.040, 0.032),
}
_PLACEMENT_MARGIN_M = 0.050
_ZONE_SIDE_M = 0.82


def _decode(obs: Mapping[str, Any], expected_cameras: set[str]) -> tuple[np.ndarray, str, str]:
    if not isinstance(obs, Mapping):
        raise ValueError("missing observation")
    camera = str(obs.get("camera", ""))
    if camera not in expected_cameras:
        raise ValueError("unexpected camera")
    image = obs.get("image"); claimed = obs.get("sha256")
    if not isinstance(image, str) or not isinstance(claimed, str):
        raise ValueError("missing image or hash")
    try:
        payload = base64.b64decode(image, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError("invalid image") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if digest != claimed:
        raise ValueError("image hash mismatch")
    frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("invalid image")
    return frame, digest, image


def _servo_pose(obs: Mapping[str, Any]) -> Mapping[str | int, int | float] | None:
    state = obs.get("actuator_state")
    if not isinstance(state, Mapping):
        return None
    for key in ("servo_pulses", "commanded_pwm", "own_pose_commands"):
        pose = state.get(key)
        if isinstance(pose, Mapping):
            return pose
    # Some recorded wrist observations carry the owned PWM map directly.
    if all(str(key) in state or key in state for key in (3, 4, 5, 6)):
        return state
    return None


def _project_ground(point: tuple[float, float], width: int, height: int) -> tuple[float, float] | None:
    x, y = point
    pitch = math.radians(NAV_PITCH_DOWN_DEG)
    dx = x - NAV_MOUNT_XYZ_M[0]
    right = -y
    down = -math.sin(pitch) * dx + math.cos(pitch) * NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M
    forward = math.cos(pitch) * dx + math.sin(pitch) * NAV_CAMERA_HEIGHT_ABOVE_FLOOR_M
    if forward <= 1e-8:
        return None
    sx, sy = width / 640.0, height / 480.0
    return ((width - 1) / 2.0 + NAV_FX_PX * sx * right / forward,
            (height - 1) / 2.0 + NAV_FY_PX * sy * down / forward)


def _footprint(center: tuple[float, float], dimensions: tuple[float, float, float]) -> list[tuple[float, float]]:
    hx = dimensions[0] / 2.0 + _PLACEMENT_MARGIN_M
    hy = dimensions[1] / 2.0 + _PLACEMENT_MARGIN_M
    x, y = center
    return [(x, y), (x-hx, y-hy), (x-hx, y+hy), (x+hx, y-hy), (x+hx, y+hy)]


def _zone_mask(frame: np.ndarray, zone: str) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low, high = _ZONE_HSV[zone]
    mask = cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    return cv2.erode(mask, np.ones((5, 5), np.uint8))


def _reconstruct_zone_square(frame: np.ndarray, mask: np.ndarray) -> dict[str, Any] | None:
    """Fit the known 0.82 m square only from visible projected RGB boundaries."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < frame.shape[0] * frame.shape[1] * .01:
        return None
    ground = []
    for u, v in contour[:, 0, :]:
        from sim.navigation_camera_profile import ground_point_robot
        point = ground_point_robot(float(u), float(v), frame.shape[1], frame.shape[0])
        if point is not None and 0.0 < point[0] < 5.0 and abs(point[1]) < 5.0:
            ground.append(point)
    if len(ground) < 80:
        return None
    points = np.asarray(ground, np.float32)
    rect = cv2.minAreaRect(points)
    measured = sorted(float(v) for v in rect[1])
    if measured[0] < .20 or measured[1] > .96 or measured[1] < .68:
        return None
    observed_box = cv2.boxPoints(rect).astype(np.float32)
    edge0 = observed_box[1] - observed_box[0]
    edge1 = observed_box[2] - observed_box[1]
    axis0 = edge0 / max(np.linalg.norm(edge0), 1e-9)
    axis1 = edge1 / max(np.linalg.norm(edge1), 1e-9)
    axes = (axis0, axis1)
    intervals = []
    for axis in axes:
        values = points @ axis
        low, high = float(values.min()), float(values.max())
        span = high - low
        if span >= .68:
            mid = (low + high) / 2.0
            intervals.append((mid - _ZONE_SIDE_M / 2.0, mid + _ZONE_SIDE_M / 2.0))
        else:
            other_axis = axes[1] if axis is axis0 else axes[0]
            other_mid = float(np.mean(points @ other_axis))
            low_point = axis * low + other_axis * other_mid
            high_point = axis * high + other_axis * other_mid
            # The reliably visible boundary is the far edge; extend the known
            # square side from it toward the camera to reconstruct a clipped
            # near edge.
            if np.linalg.norm(high_point) >= np.linalg.norm(low_point):
                intervals.append((high - _ZONE_SIDE_M, high))
            else:
                intervals.append((low, low + _ZONE_SIDE_M))
    corners = np.asarray([
        axes[0] * a + axes[1] * b
        for a, b in ((intervals[0][0], intervals[1][0]),
                     (intervals[0][1], intervals[1][0]),
                     (intervals[0][1], intervals[1][1]),
                     (intervals[0][0], intervals[1][1]))
    ], dtype=np.float32)
    # Require RGB boundary support on the far edge and both adjacent side
    # edges. The near edge may be outside the frame or hidden by carried cargo.
    edge_support = []
    edge_ranges = []
    for index in range(4):
        a, b = corners[index], corners[(index + 1) % 4]
        edge = b - a
        delta = points - a
        distance = np.abs(edge[0] * delta[:, 1] - edge[1] * delta[:, 0]) / max(np.linalg.norm(edge), 1e-9)
        projection = np.dot(points - a, edge) / max(float(np.dot(edge, edge)), 1e-9)
        edge_support.append(int(np.count_nonzero((distance < .045) & (projection > -.08) & (projection < 1.08))))
        edge_ranges.append(float(np.linalg.norm((a + b) / 2.0)))
    far = int(np.argmax(edge_ranges))
    required = {far, (far - 1) % 4, (far + 1) % 4}
    if any(edge_support[index] < 8 for index in required):
        return None
    inside_fraction = float(np.mean([
        cv2.pointPolygonTest(corners, (float(p[0]), float(p[1])), True) >= -.065
        for p in points
    ]))
    if inside_fraction < .90:
        return None
    return {
        "calibrated_zone_square_corners_m": [[float(x), float(y)] for x, y in corners],
        "measured_boundary_extents_m": measured,
        "known_zone_side_m": _ZONE_SIDE_M,
        "edge_support_counts": edge_support,
        "required_supported_edges": sorted(required),
        "observed_boundary_fit_fraction": inside_fraction,
        "method": "nav_rgb_boundary_ground_projection+known_square_side",
    }


def _identity(wrist_encoded: str, cargo_id: str) -> dict[str, Any]:
    try:
        result = observe_box(wrist_encoded, cargo_id)
    except (ValueError, KeyError) as exc:
        return {"confirmed": False, "reason": str(exc)}
    return {
        "confirmed": bool(result.get("visible")
                          and result.get("known_marker_geometry", {}).get("marker_id") == BOX_MARKER_IDS[cargo_id]
                          and float(result.get("reprojection_rmse_px", math.inf)) <= 3.0),
        "marker_id": BOX_MARKER_IDS[cargo_id],
        "visible": bool(result.get("visible")),
        "reprojection_rmse_px": result.get("reprojection_rmse_px"),
        "_face_inset_camera_m": result.get("box_face_inset_camera_m"),
    }


def inspect_placement(wrist_obs: Mapping[str, Any], nav_obs: Mapping[str, Any], *,
                      cargo_id: str, destination_zone: str,
                      stage: str = "before_release",
                      held_identity_confirmed: bool = False) -> dict[str, Any]:
    """Return serializable inside/outside/uncertain placement evidence."""
    if cargo_id not in _BOX_DIMENSIONS:
        raise ValueError("unknown cargo_id")
    zone = str(destination_zone).upper()
    if zone not in _ZONE_HSV:
        raise ValueError("destination_zone must be A, B, or C")
    if stage == "carrying":
        stage = "before_release"
    if stage not in {"before_release", "released"}:
        raise ValueError("stage must be before_release or released")
    base = {"status": "uncertain", "reason": "", "stage": stage,
            "cargo_id": cargo_id, "destination_zone": zone,
            "wrist_sha256": None, "nav_sha256": None}
    try:
        wrist_frame, wrist_hash, wrist_encoded = _decode(wrist_obs, {"robot_cam", "wrist"})
        nav_frame, nav_hash, _ = _decode(nav_obs, {"nav_cam", "nav"})
    except ValueError as exc:
        return {**base, "reason": f"INVALID_VISUAL_EVIDENCE:{exc}"}
    base.update({"wrist_sha256": wrist_hash, "nav_sha256": nav_hash})
    identity = _identity(wrist_encoded, cargo_id)
    fresh_identity = bool(identity["confirmed"])
    continuity_identity = bool(stage == "before_release" and held_identity_confirmed)
    identity["confirmed"] = bool(fresh_identity or continuity_identity)
    identity["provenance"] = ("fresh_target_marker_in_wrist_rgb" if fresh_identity else
                              "caller_confirmed_tracked_id_grasp_plus_continuous_visual_grip"
                              if continuity_identity else "unconfirmed")
    pose = _servo_pose(wrist_obs)
    if pose is None:
        return {**base, "reason": "OWN_PWM_UNAVAILABLE", "identity": identity}

    center = None
    center_source = None
    if stage == "before_release":
        try:
            grip = forward_grip(pose)
            center = (float(grip[0]), float(grip[1]))
            center_source = "predicted_lowering_center_from_owned_pwm_fk"
        except (ValueError, KeyError) as exc:
            return {**base, "reason": f"OWN_PWM_INVALID:{exc}", "identity": identity}
    elif identity["confirmed"]:
        point = identity.get("_face_inset_camera_m")
        if isinstance(point, (list, tuple)) and len(point) == 3:
            try:
                box = camera_to_base(point, pose)
                center = (float(box[0]), float(box[1]))
                center_source = "released_target_marker_pose_plus_owned_pwm_fk"
            except (ValueError, KeyError):
                center = None

    mask = _zone_mask(nav_frame, zone)
    visible_area = int(np.count_nonzero(mask))
    square = _reconstruct_zone_square(nav_frame, mask)
    if square is None:
        # Conservative fallback for a nearby zone whose third edge is outside
        # the camera view.  The experimental fitter still requires two real
        # perpendicular RGB boundaries and visible interior color support.
        from harness.visual_placement_corner import reconstruct_zone_from_corner
        square = reconstruct_zone_from_corner(nav_frame, zone)
    public_identity = {key: value for key, value in identity.items() if not key.startswith("_")}
    calibrated_center_key = ("calibrated_lowering_center_m" if stage == "before_release"
                             else "calibrated_released_box_center_m")
    evidence = {**base, "identity": public_identity, calibrated_center_key: center,
                "calibration_estimate_source": center_source,
                "box_dimensions_m": list(_BOX_DIMENSIONS[cargo_id]),
                "placement_margin_m": _PLACEMENT_MARGIN_M, "zone_visible_pixels": visible_area}
    evidence["zone_square_reconstruction"] = square
    if center is None:
        return {**evidence, "reason": "TARGET_POSITION_UNOBSERVABLE"}
    pixels = [_project_ground(point, nav_frame.shape[1], nav_frame.shape[0])
              for point in _footprint(center, _BOX_DIMENSIONS[cargo_id])]
    evidence["predicted_footprint_pixels"] = [None if p is None else [float(p[0]), float(p[1])] for p in pixels]
    if visible_area < nav_frame.shape[0] * nav_frame.shape[1] * 0.01:
        return {**evidence, "reason": "DESTINATION_ZONE_NOT_VISIBLE"}
    if not identity["confirmed"]:
        # Held boxes commonly fill the wrist view. That occlusion must not be
        # misreported as outside, but it also cannot prove target identity.
        return {**evidence, "reason": "TARGET_MARKER_OCCLUDED_OR_UNCONFIRMED"}
    if square is None:
        return {**evidence, "reason": "DESTINATION_SQUARE_UNDERCONSTRAINED"}
    square_corners = np.asarray(square["calibrated_zone_square_corners_m"], np.float32)
    metric_footprint = _footprint(center, _BOX_DIMENSIONS[cargo_id])
    signed = [float(cv2.pointPolygonTest(square_corners, point, True)) for point in metric_footprint]
    evidence["calibrated_footprint_signed_margin_m"] = signed
    if min(signed) >= 0.0:
        return {**evidence, "status": "inside", "reason": "TARGET_FOOTPRINT_INSIDE_VISIBLE_ZONE_WITH_MARGIN"}
    if min(signed) <= -.02:
        return {**evidence, "status": "outside", "reason": "TARGET_FOOTPRINT_OUTSIDE_VISIBLE_ZONE_MARGIN"}
    return {**evidence, "reason": "ZONE_BOUNDARY_TOO_CLOSE_FOR_RELEASE"}


__all__ = ["inspect_placement"]
