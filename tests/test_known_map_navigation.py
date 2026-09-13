import inspect

import cv2
import numpy as np
import pytest

import harness.known_map_navigation as nav


def map_data(obstacles=()):
    return {"schema": "ugrp.authored_navigation_map.v1", "map_id": "test", "version": 1,
            "frame": "warehouse_xy_m", "bounds_m": [-.85, 1.95, -3.1, -.9],
            "grid_resolution_m": .025,
            "top_camera": {"name": "cctv_top", "position_m": [.55, -2, 2.5],
                           "quaternion_wxyz": [1, 0, 0, 0], "fov_y_deg": 55},
            "footprint": {"unloaded_radius_m": .18, "safety_margin_m": .04},
            "zones": {"start": {"center_m": [-.50, -2.55], "radius_m": .20},
                      "goal": {"center_m": [1.55, -2.55], "radius_m": .18}},
            "obstacles": list(obstacles)}


def obstacle(center, half):
    return {"id": "wall", "kind": "box", "center_m": center, "half_extents_m": half,
            "height_m": .4, "traversable": False, "cost_multiplier": 1}


def jpeg_at(point, data, shape=(480, 640)):
    h, w = shape
    camera = data["top_camera"]
    distance = camera["position_m"][2] - .09
    visible_h = 2 * distance * np.tan(np.deg2rad(camera["fov_y_deg"]) / 2)
    visible_w = visible_h * w / h
    u = round((point[0] - camera["position_m"][0]) * w / visible_w + (w - 1) / 2)
    v = round((camera["position_m"][1] - point[1]) * h / visible_h + (h - 1) / 2)
    image = np.zeros((h, w, 3), np.uint8)
    cv2.circle(image, (u, v), 10, (0, 255, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


def jpeg_with_pixels(points, shape=(720, 960), radius=6):
    image = np.zeros((*shape, 3), np.uint8)
    for point in points:
        cv2.circle(image, point, radius, (0, 255, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return encoded.tobytes()


def test_projection_round_trip_center_and_offset():
    data = map_data()
    assert np.allclose(nav.pixel_to_world((319.5, 239.5), (480, 640, 3), data["top_camera"]), (.55, -2))
    image = jpeg_at((-.5, -2.55), data)
    decoded = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(decoded, cv2.COLOR_BGR2HSV)
    ys, xs = np.where(cv2.inRange(hsv, (20, 70, 50), (40, 255, 255)) > 0)
    assert np.allclose(nav.pixel_to_world((xs.mean(), ys.mean()), decoded.shape, data["top_camera"]),
                       (-.5, -2.55), atol=.015)


def test_astar_detours_and_fully_blocked_narrow_map_has_no_path():
    detour = map_data([obstacle((.5, -2.55), (.12, .25))])
    path = nav.plan_grid_path(detour, (-.5, -2.55), (1.55, -2.55))
    assert path and len(path) > 2
    wall = map_data([obstacle((.5, -2.0), (.12, 1.0))])
    assert nav.plan_grid_path(wall, (-.5, -2.55), (1.55, -2.55)) is None


def test_inflated_footprint_cannot_plan_with_centre_near_map_boundary():
    data = map_data()
    # Centre is within the authored rectangle, but the 0.22 m inflated chassis is not.
    assert nav.plan_grid_path(data, (-.75, -2.55), (1.55, -2.55)) is None


def test_uncertain_localization_stops_and_own_rgb_is_decoded():
    data = map_data()
    blank = jpeg_at((10, 10), data)
    result = nav.KnownMapNavigator(data, "r1").decide(blank, blank, 1)
    assert result["action"]["forward"] == result["action"]["left"] == 0
    assert result["status"] == "localization_uncertain"
    assert result["diagnostics"]["own_rgb"]["decoded"] is True


def test_four_consecutive_image_localization_losses_are_terminal():
    data = map_data()
    blank = jpeg_at((10, 10), data)
    navigator = nav.KnownMapNavigator(data, "r1")
    results = [navigator.decide(blank, blank, frame) for frame in range(4)]
    assert [item["done"] for item in results] == [False, False, False, True]
    assert results[-1]["status"] == "localization_lost"


def test_real_r1_top_image_component_layout_groups_as_one_chassis():
    # Derived from development_r1-open-map/rgb/0000-top.jpg, source SHA-256
    # 6e0b89bc4cff83f1bf9f0ca0e422dd16837780eef09bcae368bc744ccdcb4192.
    # These are the four measured yellow-component centroids, retained as a
    # compact pixel-only regression fixture rather than simulator/referee data.
    data = map_data()
    top = jpeg_with_pixels([(168, 495), (201, 496), (166, 525), (201, 533)])
    own = jpeg_at((-.5, -2.55), data)
    result = nav.KnownMapNavigator(data, "r1").decide(own, top, 0)
    assert result["status"] == "calibrating_forward"
    assert np.allclose(result["diagnostics"]["position_estimate_m"], (-.5, -2.55), atol=.08)
    assert result["diagnostics"]["localization"]["group_component_count"] == 4


def test_two_separate_robot_sized_yellow_groups_are_ambiguous():
    data = map_data()
    # Two compact groups are 0.40 m apart and equally near the start centre.
    points = []
    for world in [(-.5, -2.35), (-.5, -2.75)]:
        camera = data["top_camera"]; h, w = 720, 960
        distance = camera["position_m"][2] - .09
        visible_h = 2 * distance * np.tan(np.deg2rad(camera["fov_y_deg"]) / 2)
        visible_w = visible_h * w / h
        u = round((world[0] - camera["position_m"][0]) * w / visible_w + (w - 1) / 2)
        v = round((camera["position_m"][1] - world[1]) * h / visible_h + (h - 1) / 2)
        points.extend([(u - 12, v - 10), (u + 12, v - 10), (u - 12, v + 10), (u + 12, v + 10)])
    top = jpeg_with_pixels(points, radius=4)
    result = nav.KnownMapNavigator(data, "r1").decide(top, top, 0)
    assert result["status"] == "localization_uncertain"
    assert result["diagnostics"]["localization"]["reason"] == "ambiguous_yellow_groups"


def test_same_image_after_probe_is_not_treated_as_measured_motion():
    data = map_data()
    own = jpeg_at((-.5, -2.55), data)
    navigator = nav.KnownMapNavigator(data, "r3")
    first = navigator.decide(own, own, 1)
    assert first["status"] == "calibrating_forward"
    frame = 2
    result = None
    for _ in range(6):
        assert navigator.decide(own, own, frame)["status"] == "settling_forward_probe"
        frame += 1
        result = navigator.decide(own, own, frame)
        frame += 1
        if result["done"]:
            break
        assert result["status"] == "calibrating_forward_repeat"
    assert result["status"] == "calibration_insufficient_visual_signal"
    assert result["done"] is True
    assert result["action"]["forward"] == result["action"]["left"] == 0
    assert result["diagnostics"]["calibration"]["uses_issued_commands_as_measurement"] is False


def test_two_visual_probes_create_jacobian_and_bounded_command():
    data = map_data()
    own = jpeg_at((-.5, -2.55), data)
    navigator = nav.KnownMapNavigator(data, "r1")
    navigator.decide(own, own, 1)
    p2 = jpeg_at((-.44, -2.55), data)
    assert navigator.decide(own, p2, 2)["status"] == "settling_forward_probe"
    assert navigator.decide(own, p2, 3)["status"] == "calibrating_lateral"
    p3 = jpeg_at((-.44, -2.49), data)
    assert navigator.decide(own, p3, 4)["status"] == "settling_lateral_probe"
    result = navigator.decide(own, p3, 5)
    assert result["status"] == "navigating"
    assert -.05 <= result["action"]["forward"] <= .10
    assert -.08 <= result["action"]["left"] <= .08
    assert result["action"]["turn"] == 0
    assert result["diagnostics"]["calibration"]["visual_displacement_jacobian"]


def test_calibration_waits_through_visual_coast_before_measuring_total_probe_motion():
    data = map_data(); own = jpeg_at((-.5, -2.55), data)
    navigator = nav.KnownMapNavigator(data, "r1")
    navigator.decide(own, own, 0)
    moving = jpeg_at((-.47, -2.55), data)
    assert navigator.decide(own, moving, 1)["status"] == "settling_forward_probe"
    coasting = jpeg_at((-.44, -2.55), data)
    assert navigator.decide(own, coasting, 2)["status"] == "settling_forward_probe"
    settled = jpeg_at((-.44, -2.55), data)
    result = navigator.decide(own, settled, 3)
    assert result["status"] == "calibrating_lateral"
    # Column uses full displacement from the pre-probe stationary origin.
    assert np.allclose(navigator._forward_delta, (.06 / .048, 0.0), atol=.15)


def test_short_visual_signal_repeats_same_axis_and_uses_accumulated_impulse():
    data = map_data(); own = jpeg_at((-.5, -2.55), data)
    navigator = nav.KnownMapNavigator(data, "r1")
    navigator.decide(own, own, 0)
    short = jpeg_at((-.485, -2.55), data)
    navigator.decide(own, short, 1)
    repeated = navigator.decide(own, short, 2)
    assert repeated["status"] == "calibrating_forward_repeat"
    assert repeated["action"]["forward"] == .08
    enough = jpeg_at((-.445, -2.55), data)
    navigator.decide(own, enough, 3)
    result = navigator.decide(own, enough, 4)
    assert result["status"] == "calibrating_lateral"
    assert navigator._probe_pulses["forward"] == 2
    assert np.allclose(navigator._forward_delta, (.055 / .096, 0.0), atol=.12)


def test_uniform_saturation_preserves_control_direction():
    forward, left = nav._uniformly_bound_control(.25, -.10)
    assert forward == pytest.approx(.10)
    assert left == pytest.approx(-.04)
    assert left / forward == pytest.approx(-.10 / .25)


def test_probe_fails_closed_when_route_exists_but_unknown_yaw_disk_is_not_clear():
    data = map_data([obstacle((-.22, -2.55), (.01, .08))])
    own = jpeg_at((-.5, -2.55), data)
    result = nav.KnownMapNavigator(data, "r1").decide(own, own, 1)
    assert nav.plan_grid_path(data, (-.5, -2.55), (1.55, -2.55)) is not None
    assert result["status"] == "calibration_unsafe_clearance"
    assert result["done"] is True
    assert result["action"]["forward"] == result["action"]["left"] == 0


def test_module_input_purity_has_no_simulator_or_world_state_dependency():
    source = inspect.getsource(nav)
    assert "import mujoco" not in source
    assert "from sim.authored_navigation_map import validate_map" in source
    assert "world_state" not in source
    assert "render-label" not in source
    assert "nav_cam" not in source
