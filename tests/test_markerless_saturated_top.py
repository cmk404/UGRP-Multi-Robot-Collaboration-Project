from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from harness.visual_arm import camera_extrinsics
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

from harness import markerless_box as DETECTOR

ROOT = Path(__file__).parent / "fixtures/markerless_box/saturated_release"

IMAGE_SIZE = (640, 480)
POSE = {3: 740, 4: 2320, 5: 1320, 6: 1500}
DIMS = (0.034, 0.040, 0.032)
SATURATED_CYAN = (255, 255, 0)
DIM_CYAN = (100, 100, 15)


def jpeg(frame):
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
    assert ok
    return base64.b64encode(encoded.tobytes()).decode()


def projected(points, pose=POSE):
    origin, axes = camera_extrinsics(pose)
    return DETECTOR._project_points(
        np.asarray(points, np.float64), np.asarray(origin), np.asarray(axes),
        scaled_camera_matrix(*IMAGE_SIZE),
        np.asarray(CAMERA_FISHEYE_D).reshape(4, 1))


def draw_box(frame, center=(.35, 0.), yaw=0., bottom_z=0., reflection=False):
    corners = DETECTOR._cuboid_corners(center, yaw, DIMS)
    corners[:, 2] += bottom_z
    body = projected(corners)
    cv2.fillConvexPoly(frame, cv2.convexHull(body.astype(np.float32)).astype(np.int32), DIM_CYAN)
    a, b, height = DIMS
    c, s = np.cos(yaw), np.sin(yaw)
    u, v = np.asarray((c, s))*a/2, np.asarray((-s, c))*b/2
    center = np.asarray(center)
    top = [[*(center+su*u+sv*v), bottom_z+height]
           for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    top_pixels = projected(top)
    cv2.fillConvexPoly(frame, np.rint(top_pixels).astype(np.int32), SATURATED_CYAN)
    if reflection:
        hull = cv2.convexHull(body.astype(np.float32)).reshape(-1, 2)
        bottom = hull[np.argmax(hull[:, 1])]
        tail = np.asarray((bottom, bottom+(42, 25), bottom+(8, 18)), np.int32)
        cv2.fillConvexPoly(frame, tail, DIM_CYAN)


def test_actual_release_frames_recover_one_consistent_floor_top_from_own_rgb_pwm():
    metadata = json.loads((ROOT / "metadata.json").read_text())
    for sequence in metadata["sequences"]:
        centers = []
        for fixture in sequence["frames"]:
            payload = (ROOT / fixture["file"]).read_bytes()
            assert hashlib.sha256(payload).hexdigest() == fixture["sha256"]
            result = DETECTOR.observe_ground_box(
                base64.b64encode(payload).decode(), fixture["servo_pulses"], "small_box_01")
            assert result["visible"] is True, (fixture["file"], result)
            assert result["reason"] == "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED"
            assert result["floor_hypothesis_projection_iou"] >= .70
            assert result["floor_top_height_residual_m"] <= .008
            centers.append(np.asarray(result["estimated_box_center_base_m"][:2]))
        assert max(np.linalg.norm(a-b) for a in centers for b in centers) < .003


def test_saturated_nonrectangle_does_not_become_a_floor_box():
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.fillPoly(frame, [np.asarray(((210, 180), (410, 220), (250, 330)), np.int32)], SATURATED_CYAN)
    result = DETECTOR.observe_ground_box(jpeg(frame), POSE)
    assert result["visible"] is False


def test_saturated_clipped_quad_fails_closed():
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(frame, (0, 160), (180, 330), SATURATED_CYAN, -1)
    result = DETECTOR.observe_ground_box(jpeg(frame), POSE)
    assert result["visible"] is False


def test_two_valid_saturated_tops_remain_ambiguous():
    frame = np.zeros((480, 640, 3), np.uint8)
    draw_box(frame, center=(.35, -.07))
    draw_box(frame, center=(.35, .07))
    result = DETECTOR.observe_ground_box(jpeg(frame), POSE)
    assert result["visible"] is False
    assert result["reason"] == "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"
    assert result["candidate_count"] >= 2


def test_elevated_saturated_top_cannot_pass_floor_height_geometry():
    frame = np.zeros((480, 640, 3), np.uint8)
    # Isolate the newly admitted saturated top band: a known-size rectangle
    # at the wrong height must still fail the unchanged measured-height gate.
    a, b, height = DIMS
    top = [[.35+su*a/2, sv*b/2, .08+height]
           for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    cv2.fillConvexPoly(frame, np.rint(projected(top)).astype(np.int32), SATURATED_CYAN)
    result = DETECTOR.observe_ground_box(jpeg(frame), POSE)
    assert result["visible"] is False
