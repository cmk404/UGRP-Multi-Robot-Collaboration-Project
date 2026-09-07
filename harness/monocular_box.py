"""Monocular box fiducial observation from one raw robot-camera JPEG.

This module deliberately imports no simulator world, data, depth, or object
state. Metric scale comes only from the manufactured 75 mm printed marker.
"""
from __future__ import annotations

import base64
import math

import cv2
import numpy as np

from sim.masterpi_camera_profile import (
    CAMERA_CALIBRATION_ID,
    CAMERA_FISHEYE_D,
    CAMERA_MOUNT_STATUS,
    scaled_camera_matrix,
)


MARKER_DICTIONARY = "DICT_4X4_50"
PRINTED_MARKER_SIDE_M = 0.075  # 100 mm plate * 96/128 marker ink extent.
TAG_PLATE_SIDE_M = 0.100
BOX_MARKER_IDS = {"small_box_01": 14, "small_box_02": 15, "small_box_03": 16}
# Fixed manufactured mount: marker centre is 60 mm above the box centre and
# its face is just outside the box half-width. Both opposite faces reuse an ID.
BOX_FACE_DEPTH_M = {"small_box_01": 0.019, "small_box_02": 0.020, "small_box_03": 0.019}
MARKER_CENTER_HEIGHT_M = 0.060


def _metadata(target_id):
    return {
        "dictionary": MARKER_DICTIONARY,
        "marker_id": BOX_MARKER_IDS[target_id],
        "printed_marker_side_m": PRINTED_MARKER_SIDE_M,
        "tag_plate_side_m": TAG_PLATE_SIDE_M,
        "mount": "duplicate upright tags on opposite box faces",
        "marker_center_above_box_center_m": MARKER_CENTER_HEIGHT_M,
        "marker_plane_to_box_center_depth_m": BOX_FACE_DEPTH_M[target_id],
        "face_identity_observable": False,
        "scale_source": "known printed marker geometry",
        "camera_calibration_id": CAMERA_CALIBRATION_ID,
        "camera_mount_status": CAMERA_MOUNT_STATUS,
    }


def _decode_jpeg(image):
    if not isinstance(image, str) or not image:
        raise ValueError("INVALID_BASE64_JPEG")
    try:
        payload = base64.b64decode(image, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError("INVALID_BASE64_JPEG") from exc
    if len(payload) < 4 or not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
        raise ValueError("INVALID_BASE64_JPEG")
    frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.ndim != 3 or frame.shape[0] < 8 or frame.shape[1] < 8:
        raise ValueError("INVALID_BASE64_JPEG")
    return frame


def _object_corners():
    half = PRINTED_MARKER_SIDE_M / 2.0
    # Required IPPE_SQUARE order: top-left, top-right, bottom-right,
    # bottom-left in a marker frame whose +y points to printed top.
    return np.asarray(((-half, half, 0), (half, half, 0),
                       (half, -half, 0), (-half, -half, 0)), dtype=np.float64)


def _pose_candidates(raw_corners, width, height):
    k = scaled_camera_matrix(width, height)
    d = np.asarray(CAMERA_FISHEYE_D, dtype=np.float64).reshape(4, 1)
    raw = np.asarray(raw_corners, dtype=np.float64).reshape(1, 4, 2)
    ideal = cv2.fisheye.undistortPoints(raw, k, d, P=k).reshape(4, 2)
    result = cv2.solvePnPGeneric(
        _object_corners(), ideal, k, np.zeros((4, 1), dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not result[0]:
        return []
    candidates = []
    for rvec, tvec in zip(result[1], result[2]):
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(tvec)) or tvec[2] <= 0:
            continue
        projected, _ = cv2.fisheye.projectPoints(
            _object_corners().reshape(1, -1, 3),
            np.asarray(rvec, dtype=np.float64).reshape(3, 1), tvec.reshape(3, 1), k, d,
        )
        error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(4, 2) - raw.reshape(4, 2)) ** 2, axis=1))))
        if math.isfinite(error):
            candidates.append((error, np.asarray(rvec).reshape(3), tvec))
    return candidates


def _detect_target_corners(gray, wanted):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    default_detector = cv2.aruco.ArucoDetector(
        dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _rejected = default_detector.detectMarkers(gray)
    if ids is None or wanted not in ids.reshape(-1):
        fallback_parameters = cv2.aruco.DetectorParameters()
        fallback_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        fallback_detector = cv2.aruco.ArucoDetector(dictionary, fallback_parameters)
        corners, ids, _rejected = fallback_detector.detectMarkers(gray)
    if ids is None:
        return []
    return [np.asarray(value, dtype=np.float64).reshape(4, 2)
            for value, marker_id in zip(corners, ids.reshape(-1))
            if int(marker_id) == wanted]


def _visible_result(base, raw_candidates, target_id, provenance, confirmed_age):
    width, height = base["image_size_px"]
    solutions = []
    for raw in raw_candidates:
        for error, rvec, tvec in _pose_candidates(raw, width, height):
            solutions.append((error, -abs(float(cv2.contourArea(raw.astype(np.float32)))), raw, rvec, tvec))
    if not solutions:
        return None
    error, _neg_area, raw, rvec, tvec = min(solutions, key=lambda item: (item[0], item[1]))
    rotation, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    face_inset = tvec - rotation[:, 2] * BOX_FACE_DEPTH_M[target_id]
    centroid = np.mean(raw, axis=0)
    return {
        **base,
        "visible": True,
        "provenance": provenance,
        "frames_since_id_confirmed": int(confirmed_age),
        "pixel_centroid": [float(centroid[0]), float(centroid[1])],
        "pixel_corners": [[float(x), float(y)] for x, y in raw],
        "marker_pose_camera": {
            "translation_m": [float(v) for v in tvec],
            "rotation_rvec_rad": [float(v) for v in rvec],
            "range_m": float(np.linalg.norm(tvec)),
            "frame": "opencv_camera_x_right_y_down_z_forward",
        },
        "box_face_inset_camera_m": [float(v) for v in face_inset],
        "upright_box_center_conversion": {
            "available_in_camera_frame": False,
            "procedure": "transform box_face_inset_camera_m to gravity-aligned robot base, then subtract marker_center_above_box_center_m from base Z",
            "base_z_offset_m": -MARKER_CENTER_HEIGHT_M,
            "reason": "printed marker rotation does not identify gravity in the wrist-camera frame",
            "opposite_face_identity_observable": False,
        },
        "reprojection_rmse_px": float(error),
    }


def observe_box(image, target_id="small_box_01"):
    """Observe a tagged box from a base64 JPEG using calibrated monocular PnP.

    Returned metric coordinates use the OpenCV camera frame: x right, y down,
    z forward. Because the same ID is installed on both opposite faces, the
    physical face identity remains unknown. The marker texture's in-plane
    rotation also does not establish gravity, so the 60 mm vertical offset is
    intentionally not applied in camera coordinates.
    """
    if target_id not in BOX_MARKER_IDS:
        raise ValueError("UNKNOWN_BOX_ID")
    frame = _decode_jpeg(image)
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    metadata = _metadata(target_id)
    base = {
        "target_id": target_id,
        "visible": False,
        "image_size_px": [int(width), int(height)],
        "provenance": "raw_robot_camera_rgb_jpeg+aruco_4x4_50+fisheye_calibration+marker_size_pnp",
        "known_marker_geometry": metadata,
    }
    result = _visible_result(base, _detect_target_corners(gray, BOX_MARKER_IDS[target_id]),
                             target_id, base["provenance"], 0)
    return result if result is not None else base


class CameraBoxTracker:
    """Bounded RGB-only marker tracking between direct identity detections."""

    def __init__(self, target_id="small_box_01", max_tracking_frames=20):
        if target_id not in BOX_MARKER_IDS:
            raise ValueError("UNKNOWN_BOX_ID")
        if isinstance(max_tracking_frames, bool) or not isinstance(max_tracking_frames, int) or not 1 <= max_tracking_frames <= 40:
            raise ValueError("INVALID_TRACKING_BUDGET")
        self.target_id = target_id
        self.max_tracking_frames = max_tracking_frames
        self._gray = None
        self._corners = None
        self._age = 0

    def _clear(self):
        self._gray = None
        self._corners = None
        self._age = 0

    @staticmethod
    def _valid_geometry(previous, current, gray):
        height, width = gray.shape
        if not np.all(np.isfinite(current)):
            return False
        if np.any(current[:, 0] < 1) or np.any(current[:, 0] >= width-1) or np.any(current[:, 1] < 1) or np.any(current[:, 1] >= height-1):
            return False
        contour = current.astype(np.float32)
        area = abs(float(cv2.contourArea(contour)))
        old_area = abs(float(cv2.contourArea(previous.astype(np.float32))))
        if area < 70 or old_area < 70 or not .35 <= area / old_area <= 2.8 or not cv2.isContourConvex(contour):
            return False
        edges = np.linalg.norm(current - np.roll(current, -1, axis=0), axis=1)
        if float(np.min(edges)) < 4:
            return False
        # A blank or occluded quadrilateral must not coast on LK inertia.
        dst = np.asarray(((0, 0), (95, 0), (95, 95), (0, 95)), np.float32)
        warp = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(contour, dst), (96, 96))
        # Absolute brightness varies strongly with the rendered/physical wrist
        # angle. Require bimodal local contrast rather than white pixels at a
        # fixed intensity; a blank floor/occlusion still fails this gate.
        p10, p90 = np.percentile(warp, (10, 90))
        return float(np.std(warp)) >= 18 and float(p90-p10) >= 40

    def observe(self, image):
        frame = _decode_jpeg(image)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        base = {"target_id": self.target_id, "visible": False,
                "image_size_px": [int(width), int(height)],
                "provenance": "raw_robot_camera_rgb_jpeg+bounded_lk_tracking",
                "known_marker_geometry": _metadata(self.target_id)}
        direct = _detect_target_corners(gray, BOX_MARKER_IDS[self.target_id])
        result = _visible_result(base, direct, self.target_id,
            "raw_robot_camera_rgb_jpeg+aruco_id_confirmed+fisheye_marker_size_pnp", 0)
        if result is not None:
            self._gray, self._corners, self._age = gray, np.asarray(result["pixel_corners"], np.float32), 0
            return result
        if self._gray is None or self._corners is None or self._age >= self.max_tracking_frames:
            self._clear()
            return base
        previous = self._corners.reshape(-1, 1, 2)
        current, status, _error = cv2.calcOpticalFlowPyrLK(self._gray, gray, previous, None,
            winSize=(31, 31), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        if current is None or status is None or not np.all(status):
            self._clear(); return base
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self._gray, current, None,
            winSize=(31, 31), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        if backward is None or back_status is None or not np.all(back_status):
            self._clear(); return base
        tracked = current.reshape(4, 2).astype(np.float64)
        fb = np.linalg.norm(backward.reshape(4, 2)-previous.reshape(4, 2), axis=1)
        # At extreme perspective the bottom corner can have a weak reverse
        # match near the frame edge even while the other three and the planar
        # PnP geometry remain stable. Bound both the robust median and the lone
        # worst corner rather than requiring four equally strong reverse hits.
        if float(np.median(fb)) > 1.5 or float(np.max(fb)) > 25 or not self._valid_geometry(self._corners, tracked, gray):
            self._clear(); return base
        age = self._age + 1
        result = _visible_result(base, [tracked], self.target_id,
            "raw_robot_camera_rgb_jpeg+lk_tracked_from_confirmed_aruco+fisheye_marker_size_pnp", age)
        if result is None or result["reprojection_rmse_px"] > 4.0:
            self._clear(); return base
        self._gray, self._corners, self._age = gray, tracked.astype(np.float32), age
        return result


def camera_point_to_robot_recommendation():
    """Describe the safe next calibration step without fabricating a transform."""
    return {
        "available": False,
        "reason": "camera mount is provisional and depends on current arm servo pose",
        "required_inputs": [
            "current commanded servos 3, 4, 5, and camera pan servo 6",
            "validated arm forward kinematics",
            "held-out hand-eye calibration for the physical wrist camera",
        ],
        "camera_frame": "opencv_camera_x_right_y_down_z_forward",
        "warning": "do not use the provisional static mount as a global or robot-base pose",
    }
