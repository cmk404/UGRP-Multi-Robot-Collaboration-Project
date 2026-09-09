import base64
from unittest import mock

import cv2
import numpy as np

from harness.markerless_box import (
    _cuboid_corners,
    _project_points,
    observe_ground_box,
)
from harness.visual_arm import camera_extrinsics
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix


POSE = {3: 740, 4: 2320, 5: 1320, 6: 1500}
IMAGE_SIZE = (640, 480)
CYAN_BGR = (180, 180, 25)
SMALL_BOX_DIMS = (0.034, 0.040, 0.032)


def _jpeg(frame):
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
    assert ok
    return base64.b64encode(encoded.tobytes()).decode()


def _blank():
    return np.zeros((IMAGE_SIZE[1], IMAGE_SIZE[0], 3), np.uint8)


def _projected_box(center=(0.35, 0.0), yaw=0.0, bottom_z=0.0):
    origin, axes = camera_extrinsics(POSE)
    corners = _cuboid_corners(center, yaw, SMALL_BOX_DIMS)
    corners[:, 2] += bottom_z
    return _project_points(
        corners,
        np.asarray(origin, np.float64),
        np.asarray(axes, np.float64),
        scaled_camera_matrix(*IMAGE_SIZE),
        np.asarray(CAMERA_FISHEYE_D, np.float64).reshape(4, 1),
    )


def _draw_projected_box(frame, **kwargs):
    pixels = _projected_box(**kwargs)
    assert pixels is not None
    hull = cv2.convexHull(pixels.astype(np.float32)).astype(np.int32)
    cv2.fillConvexPoly(frame, hull, CYAN_BGR)


def _draw_box_with_bright_top(frame, center=(.35, 0.), yaw=0., bottom_z=0., reflection=False):
    pixels = _projected_box(center=center, yaw=yaw, bottom_z=bottom_z)
    cv2.fillConvexPoly(frame, cv2.convexHull(pixels.astype(np.float32)).astype(np.int32),
                       (100, 100, 15))
    origin, axes = camera_extrinsics(POSE)
    a, b, height = SMALL_BOX_DIMS
    c, s = np.cos(yaw), np.sin(yaw)
    u, v = np.asarray((c, s))*a/2, np.asarray((-s, c))*b/2
    center_xy = np.asarray(center)
    top = np.asarray([[*(center_xy+su*u+sv*v), bottom_z+height]
                      for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    top_pixels = _project_points(top, np.asarray(origin), np.asarray(axes),
                                 scaled_camera_matrix(*IMAGE_SIZE),
                                 np.asarray(CAMERA_FISHEYE_D).reshape(4, 1))
    quad = np.rint(top_pixels).astype(np.int32)
    cv2.fillConvexPoly(frame, quad, CYAN_BGR)
    if reflection:
        hull = cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2)
        bottom = hull[np.argmax(hull[:, 1])]
        tail = np.asarray((bottom, bottom+(42, 25), bottom+(8, 18)), np.int32)
        cv2.fillConvexPoly(frame, tail, (100, 100, 15))
    return quad


def test_known_floor_cuboid_returns_only_conditional_metric_center():
    frame = _blank()
    _draw_projected_box(frame, center=(0.35, 0.0), yaw=0.0)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is True
    assert result["known_box_dimensions_m"] == [0.034, 0.040, 0.032]
    assert np.allclose(result["estimated_box_center_base_m"], [0.35, 0.0, 0.016], atol=0.012)
    assert result["floor_hypothesis_projection_iou"] >= 0.58
    assert "conditional" in result["measurement_scope"]
    assert "own_rgb" in result["provenance"]
    assert result["identity_source"] == "task_catalog_reference_only_not_visually_decoded"


def test_near_grasp_radius_is_not_rejected_by_camera_relative_distance():
    pose = {3: 500, 4: 2448, 5: 1320, 6: 1500}
    origin, axes = camera_extrinsics(pose)
    corners = _cuboid_corners((0.145, 0.0), 0.0, SMALL_BOX_DIMS)
    pixels = _project_points(
        corners, np.asarray(origin), np.asarray(axes),
        scaled_camera_matrix(*IMAGE_SIZE),
        np.asarray(CAMERA_FISHEYE_D).reshape(4, 1),
    )
    frame = _blank()
    cv2.fillConvexPoly(
        frame, cv2.convexHull(pixels.astype(np.float32)).astype(np.int32), CYAN_BGR,
    )

    result = observe_ground_box(_jpeg(frame), pose)

    assert result["visible"] is True
    assert np.allclose(result["estimated_box_center_base_m"][:2], [0.145, 0.0], atol=0.012)


def test_elevated_cyan_cuboid_is_rejected_by_floor_projection_extent():
    frame = _blank()
    _draw_projected_box(frame, center=(0.35, 0.0), bottom_z=0.08)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "FLOOR_HYPOTHESIS_PROJECTION_MISMATCH"
    assert result["floor_hypothesis_projection_iou"] < 0.58
    assert result["floor_hypothesis_projection_residual"] > 0.42


def test_multiple_indistinguishable_cyan_candidates_fail_closed():
    frame = _blank()
    cv2.rectangle(frame, (100, 200), (150, 250), CYAN_BGR, -1)
    cv2.rectangle(frame, (400, 200), (450, 250), CYAN_BGR, -1)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"
    assert result["candidate_count"] == 2


def test_frame_clipped_silhouette_fails_closed():
    frame = _blank()
    cv2.rectangle(frame, (0, 180), (80, 280), CYAN_BGR, -1)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "CYAN_SILHOUETTE_FRAME_CLIPPED"


def test_low_pixel_candidate_fails_closed():
    frame = _blank()
    cv2.rectangle(frame, (300, 200), (306, 206), CYAN_BGR, -1)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "CYAN_SILHOUETTE_NOT_VISIBLE"


def test_occluded_nonconvex_silhouette_fails_closed():
    frame = _blank()
    polygon = np.asarray(((200, 180), (300, 180), (300, 220),
                          (235, 220), (235, 300), (200, 300)), np.int32)
    cv2.fillPoly(frame, [polygon], CYAN_BGR)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "CYAN_SILHOUETTE_NOT_VISIBLE"


def test_full_measured_top_recovers_when_floor_reflection_distorts_silhouette():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.29, 0.), reflection=True)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is True
    assert result["reason"] == "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED"
    assert np.allclose(result["estimated_box_center_base_m"], [.29, 0., .016], atol=.008)
    assert result["floor_top_height_residual_m"] < .008
    assert result["floor_hypothesis_projection_iou"] >= .70
    assert result["projection_scope"] == "full known-size top rectangle at calibrated expected floor-top height"
    assert len(result["pixel_centroid"]) == 2
    assert len(result["pixel_bbox"]) == 4
    assert result["area_px"] > 180


def test_elevated_full_top_cannot_bypass_floor_height_gate():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.35, 0.), bottom_z=.08, reflection=True)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False


def test_partial_bright_strip_cannot_supply_box_center():
    frame = _blank()
    quad = _draw_box_with_bright_top(frame, center=(.29, 0.), reflection=True)
    # Retain only a narrow strip of the bright top while leaving the dark body.
    mask = np.zeros(frame.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, quad, 255)
    ys = np.where(mask)[0]
    frame[(mask > 0) & (np.indices(mask.shape)[0] < int(np.percentile(ys, 70)))] = (100, 100, 15)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert not (result["visible"] and result["reason"] == "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED")


def test_two_full_top_rectangles_are_rejected_as_ambiguous():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.34, -.07), reflection=True)
    _draw_box_with_bright_top(frame, center=(.34, .07), reflection=True)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"
    assert result["candidate_count"] >= 2


def test_multiple_valid_tops_override_an_otherwise_valid_silhouette_fit():
    frame = _blank()
    _draw_projected_box(frame, center=(.35, 0.))

    with mock.patch("harness.markerless_box._observe_floor_top_face",
                    return_value=(None, 2)):
        result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"
    assert result["candidate_count"] == 2


def test_bright_top_cannot_hide_second_valid_box_with_dim_top():
    frame = _blank()
    _draw_box_with_bright_top(frame, center=(.34, -.07))
    dim_quad = _draw_box_with_bright_top(frame, center=(.34, .07))
    cv2.fillConvexPoly(frame, dim_quad, (140, 140, 20))

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is False
    assert result["reason"] == "MULTIPLE_INDISTINGUISHABLE_CYAN_CANDIDATES"
    assert result["candidate_count"] == 2


def test_one_valid_box_ignores_spatially_separate_implausible_cyan_fragment():
    frame = _blank()
    _draw_projected_box(frame, center=(.35, 0.))
    cv2.rectangle(frame, (48, 128), (63, 135), CYAN_BGR, -1)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is True
    assert result["reason"] == "FLOOR_CUBOID_HYPOTHESIS_VALIDATED"
    assert np.allclose(result["estimated_box_center_base_m"], [.35, 0., .016], atol=.012)


def test_disconnected_body_patch_is_grouped_only_inside_projected_cuboid():
    frame = _blank()
    pixels = _projected_box(center=(.29, 0.))
    origin, axes = camera_extrinsics(POSE)
    a, b, height = SMALL_BOX_DIMS
    top = np.asarray([[.29+su*a/2, sv*b/2, height]
                      for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    top_pixels = _project_points(
        top, np.asarray(origin), np.asarray(axes), scaled_camera_matrix(*IMAGE_SIZE),
        np.asarray(CAMERA_FISHEYE_D).reshape(4, 1))
    cv2.fillConvexPoly(frame, np.rint(top_pixels).astype(np.int32), CYAN_BGR)
    body_pixel = np.rint(np.mean(pixels[:4], axis=0)).astype(int)
    cv2.rectangle(frame, tuple(body_pixel-6), tuple(body_pixel+6), (100, 100, 15), -1)

    result = observe_ground_box(_jpeg(frame), POSE)

    assert result["visible"] is True
    assert result["reason"] == "MEASURED_TOP_FACE_FLOOR_HYPOTHESIS_VALIDATED"
