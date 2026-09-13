import inspect
import math

import cv2
import numpy as np

import harness.heading_map_navigation as heading


def map_data(obstacles=(), goal=(1.55, -2.55)):
    return {"schema": "ugrp.authored_navigation_map.v1", "map_id": "heading_test", "version": 1,
            "frame": "warehouse_xy_m", "bounds_m": [-.85, 1.95, -3.1, -.9],
            "grid_resolution_m": .025,
            "top_camera": {"name": "cctv_top", "position_m": [.55, -2, 2.5],
                           "quaternion_wxyz": [1, 0, 0, 0], "fov_y_deg": 55},
            "footprint": {"unloaded_radius_m": .18, "safety_margin_m": .04},
            "zones": {"start": {"center_m": [-.50, -2.55], "radius_m": .20},
                      "goal": {"center_m": list(goal), "radius_m": .18}},
            "obstacles": list(obstacles)}


def obstacle(center=(.5, -2.0), half=(.12, 1.0)):
    return {"id": "wall", "kind": "box", "center_m": center, "half_extents_m": half,
            "height_m": .4, "traversable": False, "cost_multiplier": 1}


def jpeg_at(point, data):
    h, w = 480, 640
    camera = data["top_camera"]
    distance = camera["position_m"][2] - .09
    visible_h = 2 * distance * np.tan(np.deg2rad(camera["fov_y_deg"]) / 2)
    visible_w = visible_h * w / h
    u = round((point[0] - camera["position_m"][0]) * w / visible_w + (w - 1) / 2)
    v = round((camera["position_m"][1] - point[1]) * h / visible_h + (h - 1) / 2)
    image = np.zeros((h, w, 3), np.uint8)
    cv2.circle(image, (u, v), 12, (0, 255, 255), -1)
    cv2.circle(image, (u + 8, v - 4), 3, (0, 180, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


def asymmetric_patch(angle=0.0):
    patch = np.zeros((96, 96), np.uint8)
    cv2.rectangle(patch, (27, 31), (68, 61), 255, -1)
    cv2.rectangle(patch, (59, 24), (72, 40), 255, -1)
    cv2.circle(patch, (62, 35), 7, 0, -1)
    matrix = cv2.getRotationMatrix2D((47.5, 47.5), angle, 1)
    return cv2.warpAffine(patch, matrix, (96, 96), flags=cv2.INTER_NEAREST)


def calibrate(navigator, data, end=(-.44, -2.55)):
    own = jpeg_at((-.5, -2.55), data)
    assert navigator.decide(own, own, 0)["status"] == "calibrating_forward"
    moved = jpeg_at(end, data)
    assert navigator.decide(own, moved, 1)["status"] == "settling_forward_probe"
    return navigator.decide(own, moved, 2)


def test_rotation_match_tracks_both_signs_from_pixels():
    base = asymmetric_patch()
    positive, diag = heading._estimate_patch_rotation_deg(base, asymmetric_patch(10), 1)
    negative, negative_diag = heading._estimate_patch_rotation_deg(base, asymmetric_patch(-10), -1)
    assert diag["ok"] and positive == 10
    assert negative_diag["ok"] and negative == -10


def test_rotation_match_rejects_symmetric_or_lost_appearance():
    circle = np.zeros((96, 96), np.uint8)
    cv2.circle(circle, (48, 48), 20, 255, -1)
    value, diag = heading._estimate_patch_rotation_deg(circle, circle, 1)
    assert value is None and diag["reason"] == "heading_rotation_unresolved"
    value, diag = heading._estimate_patch_rotation_deg(circle, np.zeros_like(circle), 1)
    assert value is None and diag["reason"] == "heading_patch_sparse"


def test_rotation_match_detects_opposite_response_instead_of_hiding_it():
    value, diag = heading._estimate_patch_rotation_deg(
        asymmetric_patch(), asymmetric_patch(-10), expected_sign=1)
    assert value is None
    assert diag["reason"] == "heading_rotation_opposite_command"


def test_rotation_match_tolerates_local_roller_appearance_changes():
    base = asymmetric_patch()
    changed = asymmetric_patch(10)
    cv2.circle(changed, (31, 54), 4, 0, -1)
    cv2.circle(changed, (68, 56), 3, 255, -1)
    value, diag = heading._estimate_patch_rotation_deg(base, changed, 1)
    assert diag["ok"]
    assert value is not None and abs(value - 10) <= 1


def test_calibration_establishes_front_then_aligns_before_forward():
    data = map_data(goal=(-.44, -1.3))
    navigator = heading.HeadingMapNavigator(data, "r1")
    result = calibrate(navigator, data)
    assert navigator._heading_rad == 0.0
    assert result["status"] == "aligning_heading"
    assert result["action"]["forward"] == 0
    assert result["action"]["left"] == 0
    assert result["action"]["turn"] > 0


def test_aligned_navigation_is_forward_only_with_no_lateral_slide():
    data = map_data()
    navigator = heading.HeadingMapNavigator(data, "r3")
    result = calibrate(navigator, data)
    assert result["status"] == "navigating_forward"
    assert result["action"]["forward"] == .10
    assert .25 <= result["action"]["duration_s"] <= .6
    assert result["action"]["left"] == result["action"]["turn"] == 0


def test_forward_rgb_response_shortens_later_lease_without_pose_feedback():
    data = map_data()
    navigator = heading.HeadingMapNavigator(data, "r3")
    first = calibrate(navigator, data)
    initial_duration = first["action"]["duration_s"]
    own = jpeg_at((-.5, -2.55), data)
    # The next raw top frame shows 8 cm travel from the issued straight lease.
    moved = jpeg_at((-.36, -2.55), data)
    result = navigator.decide(own, moved, 3)
    assert result["status"] == "navigating_forward"
    assert result["action"]["duration_s"] < initial_duration
    assert navigator._forward_gain_m_per_impulse > 1.0


def test_blocked_map_refuses_before_motion_and_bad_jpeg_is_rejected():
    data = map_data([obstacle()])
    own = jpeg_at((-.5, -2.55), data)
    navigator = heading.HeadingMapNavigator(data, "r1")
    refused = navigator.decide(own, own, 0)
    assert refused["status"] == "no_map_route" and refused["done"]
    assert refused["action"]["forward"] == refused["action"]["left"] == 0
    try:
        heading.HeadingMapNavigator(map_data(), "r1").decide(b"bad", own, 0)
    except ValueError as error:
        assert "own_jpeg" in str(error)
    else:
        raise AssertionError("invalid own RGB must fail closed")


def test_four_consecutive_heading_patch_losses_are_terminal():
    data = map_data()
    image = jpeg_at((-.5, -2.55), data)
    navigator = heading.HeadingMapNavigator(data, "r1")
    navigator._patch = lambda *_: None
    results = [navigator.decide(image, image, frame) for frame in range(4)]
    assert [result["done"] for result in results] == [False, False, False, True]
    assert results[-1]["status"] == "heading_visual_lost"


def test_source_has_no_forbidden_runtime_state_and_all_emitted_actions_are_non_lateral():
    source = inspect.getsource(heading)
    assert "mujoco" not in source and "world_state" not in source
    assert "joint" not in source and "contact" not in source
    data = map_data(goal=(-.44, -1.3))
    navigator = heading.HeadingMapNavigator(data, "r1")
    result = calibrate(navigator, data)
    assert navigator._issued
    assert all(action["kind"] == "mecanum" and action["left"] == 0
               and action["forward"] >= 0 for action in navigator._issued)
    assert math.isfinite(result["action"]["turn"])
