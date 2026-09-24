"""Opt-in cargo carrier revalidation under bounded TOP occlusion."""

import base64
import hashlib
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import ImageRoute, SkillBindings
from harness.solo_box_transport import SoloBoxTransport
from harness.visual_attachment import compare_box_comotion
from scripts.run_dispatch_skills import SkillScene
from test_dispatch_skill_binding import authored_map, committed


SAVED = Path(__file__).parent / "fixtures/bounded_carrier_relink"


def _cloud(x=30., y=30.):
    return np.array([[[x + dx, y + dy]]
                     for dy in (0., 10., 20.) for dx in (0., 10., 20.)],
                    dtype=np.float32)


def _image(x=25, *, orange=True):
    image = np.zeros((112, 112, 3), np.uint8)
    if orange:
        image[25:56, x:x + 36] = (0, 140, 255)
    return image


def _attachment():
    raw = b"current-own-camera-jpeg"
    own = base64.b64encode(raw).decode()
    return ({"evidence": "visual_attachment", "attached": True,
             "camera_pan_delta_pwm": 0, "mask_iou": .99}, own, own), hashlib.sha256(raw).hexdigest()


def _route(*, enabled=True):
    route = ImageRoute(SkillBindings(committed(), authored_map("open")), "box",
                       time_aware_box_reacquisition=enabled,
                       bounded_carrier_relink=enabled)
    route.box_previous = _image()
    route.box_previous_top_sha256 = "a" * 64
    route.box_background = np.zeros_like(route.box_previous)
    route.box_center = np.array([60., 60.])
    route.box_carrier_points = _cloud()
    route.box_carrier_occluded_frames = 3
    route.box_carrier_occlusion_started_at_s = 1.
    route.box_carrier_occlusion_last_at_s = 1.45
    route.box_carrier_occlusion_last_frame_id = 3
    route.box_carrier_cumulative_motion_px = 9.
    route.box_carrier_link_source = {"cargo_support_method": "cyan component",
                                     "frame_id": 0, "top_sha256": "0" * 64}
    return route


def _flow(points, shift=(3., 0.)):
    previous = points.reshape(-1, 2)
    delta = np.tile(np.array(shift, np.float32), (len(previous), 1))
    return delta, np.zeros(len(previous)), previous, previous + delta


def _observe_proxy(route, *, frame=None, attachment=None, own_sha=None,
                   at=1.65, frame_id=4):
    if attachment is None:
        attachment, valid_sha = _attachment()
        own_sha = valid_sha if own_sha is None else own_sha
    return route._track_carrier_under_occlusion(
        _image(x=28) if frame is None else frame, attachment,
        observed_at_s=at, frame_id=frame_id, own_sha256=own_sha,
        top_sha256="b" * 64)


def test_relink_needs_new_corners_on_the_previously_linked_carrier(monkeypatch):
    route = _route()
    route._tracked_points = lambda old, new, points: _flow(points)
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: _cloud(x=10., y=10.))
    center, tracking = _observe_proxy(route)
    audit = tracking["bounded_carrier_relink"]
    assert np.allclose(center, [63., 60.])
    assert audit["status"] == "accepted"
    assert audit["last_cargo_carrier_visual_link"]["cargo_support_method"] == "cyan component"
    assert audit["occluded_frames_before"] == 3
    assert audit["occluded_frames_after"] == 4
    assert audit["cumulative_visual_motion_px_after"] == pytest.approx(12.)
    assert audit["same_capture_own_attachment_validated"] is True
    assert audit["current_top_sha256"] == "b" * 64
    assert len(route.box_carrier_points) == 9


def test_nearby_different_orange_object_cannot_become_fresh_cloud(monkeypatch):
    route = _route()
    route._tracked_points = lambda old, new, points: _flow(points)
    # Simulate orange corners on a nearby different chassis. Co-motion alone
    # is not enough: their old pixels do not overlap the original linked cloud.
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: _cloud(x=50., y=10.))
    with pytest.raises(RuntimeError, match="fresh_carrier_identity_mismatch"):
        _observe_proxy(route)
    assert route.box_carrier_relink_audit["old_cloud_overlap_fraction"] < .6
    assert route.box_carrier_occluded_frames == 3


@pytest.mark.parametrize("invalid", ["detached", "pan", "wrong_capture_sha",
                                    "old_capture", "stale_capture"])
def test_relink_rejects_bad_current_own_or_stale_capture(invalid):
    route = _route()
    attachment, own_sha = _attachment()
    evidence, own, validated = attachment
    args = {}
    if invalid == "detached":
        attachment = ({**evidence, "attached": False}, own, validated)
    elif invalid == "pan":
        attachment = ({**evidence, "camera_pan_delta_pwm": 20}, own, validated)
    elif invalid == "wrong_capture_sha":
        own_sha = "0" * 64
    elif invalid == "old_capture":
        args["frame_id"] = 3
    else:
        args["at"] = 2.1
    with pytest.raises(RuntimeError, match="dispatch bounded carrier relink"):
        _observe_proxy(route, attachment=attachment, own_sha=own_sha, **args)
    assert route.box_carrier_occluded_frames == 3


def test_first_carrier_only_capture_must_follow_recent_cargo_visual_link():
    route = _route()
    route.box_carrier_occluded_frames = 0
    route.box_carrier_link_source.update(observed_at_s=.4, frame_id=2)
    with pytest.raises(RuntimeError, match="stale_first_carrier_capture_after_cargo_link"):
        _observe_proxy(route, at=1.65, frame_id=4)
    assert route.box_carrier_occluded_frames == 0
    route.box_carrier_link_source.update(observed_at_s=1.5, frame_id=4)
    with pytest.raises(RuntimeError, match="stale_first_carrier_capture_after_cargo_link"):
        _observe_proxy(route, at=1.65, frame_id=4)
    assert route.box_carrier_occluded_frames == 0


def test_lost_cargo_never_reappears_then_total_proxy_budget_fails():
    route = _route()
    route.box_carrier_occluded_frames = 12
    route.box_carrier_occlusion_last_at_s = 1.65
    route.box_carrier_occlusion_last_frame_id = 4
    with pytest.raises(RuntimeError, match="occlusion_frame_or_time_budget_exhausted"):
        _observe_proxy(route, at=1.85, frame_id=5)
    assert route.box_carrier_relink_audit["occluded_frames_before"] == 12
    assert route.box_carrier_relink_audit["max_occluded_frames"] == 12


@pytest.mark.parametrize("budget", ["elapsed", "motion"])
def test_relinked_carrier_cannot_reset_elapsed_or_visual_motion_budget(monkeypatch, budget):
    route = _route()
    route._tracked_points = lambda old, new, points: _flow(points)
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: _cloud(x=10., y=10.))
    kwargs = {}
    if budget == "elapsed":
        route.box_carrier_occlusion_last_at_s = 4.1
        kwargs = {"at": 4.3}
        reason = "occlusion_frame_or_time_budget_exhausted"
    else:
        route.box_carrier_cumulative_motion_px = 104.
        reason = "cumulative_visual_motion_budget_exhausted"
    with pytest.raises(RuntimeError, match=reason):
        _observe_proxy(route, **kwargs)
    assert route.box_carrier_occluded_frames == 3


def test_fresh_orange_motion_disagreement_rejects_identity_switch(monkeypatch):
    route = _route()
    flows = iter(((3., 0.), (8., 0.)))
    route._tracked_points = lambda old, new, points: _flow(points, next(flows))
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: _cloud(x=10., y=10.))
    with pytest.raises(RuntimeError, match="fresh_carrier_identity_mismatch"):
        _observe_proxy(route)
    assert route.box_carrier_relink_audit["motion_disagreement_px"] > 1.5


def test_relink_rejects_partial_carrier_appearance(monkeypatch):
    route = _route()
    route._tracked_points = lambda old, new, points: _flow(points)
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: _cloud(x=10., y=10.))
    partially_visible = _image(x=28)
    partially_visible[25:56, 42:64] = 0
    with pytest.raises(RuntimeError, match="linked_appearance_unresolved|fresh_carrier_identity_mismatch"):
        _observe_proxy(route, frame=partially_visible)
    assert route.box_carrier_occluded_frames == 3


def test_legacy_cap_remains_three_without_flag():
    with pytest.raises(RuntimeError, match="dispatch box unresolved or ambiguous"):
        _observe_proxy(_route(enabled=False))


def test_final_delivery_refuses_proxy_even_when_current_carrier_is_visible(monkeypatch):
    route = _route()
    route.box_carrier_occluded_frames = 1
    route.points = [np.array([63., 60.])]
    route.index = 0
    route._track_carrier_under_occlusion = lambda *args, **kwargs: (
        np.array([63., 60.]),
        {"method": "linked TOP RGB carrier motion during cargo occlusion",
         "motion_px": [3., 0.]})
    jpeg = cv2.imencode(".jpg", _image(x=28))[1].tobytes()
    attachment, own_sha = _attachment()
    with pytest.raises(RuntimeError, match="strong current direct TOP cargo silhouette"):
        route.observe(jpeg, own_attachment=attachment, own_sha256=own_sha,
                      observed_at_s=1.65, frame_id=4)


def test_small_current_cyan_fragment_cannot_reset_total_proxy_budget():
    route = _route()
    route.box_delta = np.zeros(2)
    route._track_carrier_under_occlusion = Mock(return_value=(
        np.array([63., 60.]),
        {"method": "linked TOP RGB carrier motion during cargo occlusion",
         "motion_px": [3., 0.]}))
    frame = _image(x=28)
    frame[57:60, 57:63] = (255, 255, 0)  # Strong colour, only 18 pixels.
    jpeg = cv2.imencode(".png", frame)[1].tobytes()
    attachment, own_sha = _attachment()
    _, evidence = route.observe(jpeg, own_attachment=attachment,
                                own_sha256=own_sha,
                                observed_at_s=1.65, frame_id=4)
    assert route._track_carrier_under_occlusion.call_count == 1
    assert route.box_carrier_occluded_frames == 3
    assert evidence["tracking"]["weak_cyan_fragment_not_direct_identity"]["area_px"] == 18


@pytest.mark.parametrize("cyan_slices", [((54, 60), (61, 67)),
                                         ((58, 64), (65, 71))])
def test_competing_strong_cyan_near_proxy_fails_before_identity_switch(cyan_slices):
    route = _route()
    route.box_delta = np.zeros(2)
    frame = _image(x=28)
    for low, high in cyan_slices:
        frame[57:63, low:high] = (255, 255, 0)
    jpeg = cv2.imencode(".png", frame)[1].tobytes()
    attachment, own_sha = _attachment()
    with pytest.raises(RuntimeError, match="competing_current_strong_cyan"):
        route.observe(jpeg, own_attachment=attachment, own_sha256=own_sha,
                      observed_at_s=1.65, frame_id=4)
    assert route.box_carrier_relink_audit["strong_candidate_count"] == 2
    assert route.box_carrier_occluded_frames == 3


def _two_view_route():
    route = _route()
    route.box_delta = np.zeros(2)
    route.points = [np.array([100., 100.]), np.array([90., 90.])]
    route.index = 0
    route.box_direct_history = [{"area_px": 36, "hue_median": 90.,
                                 "bounds_px": (57, 57, 6, 6),
                                 "top_sha256": "0" * 64,
                                 "frame_id": 0, "observed_at_s": 0.}]
    def track(frame, attachment, **kwargs):
        route.box_carrier_occluded_frames += 1
        route.box_carrier_cumulative_motion_px += 3.
        return route.box_center + np.array([3., 0.]), {
            "method": "linked TOP RGB carrier motion during cargo occlusion",
            "motion_px": [3., 0.],
            "occluded_frames": route.box_carrier_occluded_frames}
    route._track_carrier_under_occlusion = Mock(side_effect=track)
    return route


def _strong_candidate_jpeg(carrier_x, cyan_x):
    frame = _image(x=carrier_x)
    frame[57:63, cyan_x:cyan_x + 6] = (255, 255, 0)
    return cv2.imencode(".png", frame)[1].tobytes()


def test_one_strong_cyan_view_is_provisional_then_consistent_second_restores_direct():
    route = _two_view_route()
    attachment, own_sha = _attachment()
    _, first = route.observe(_strong_candidate_jpeg(28, 58),
        own_attachment=attachment, own_sha256=own_sha,
        observed_at_s=1.65, frame_id=4)
    assert first["tracking"]["method"].startswith("linked TOP RGB carrier")
    assert first["tracking"]["bounded_direct_reappearance"]["status"] == "provisional"
    assert route.box_carrier_occluded_frames == 4
    assert route.box_direct_recovery_pending is not None
    _, second = route.observe(_strong_candidate_jpeg(31, 61),
        own_attachment=attachment, own_sha256=own_sha,
        observed_at_s=1.9, frame_id=5)
    assert second["tracking"]["method"] == (
        "bounded two-view direct cyan recovery from TOP RGB")
    assert second["tracking"]["bounded_direct_reappearance"]["status"] == (
        "confirmed_two_view_direct_cargo")
    assert route.box_carrier_occluded_frames == 0
    assert route.box_direct_recovery_pending is None


def test_nearby_different_cyan_object_cannot_reset_budget_on_second_view():
    route = _two_view_route()
    attachment, own_sha = _attachment()
    route.observe(_strong_candidate_jpeg(28, 58),
        own_attachment=attachment, own_sha256=own_sha,
        observed_at_s=1.65, frame_id=4)
    with pytest.raises(RuntimeError, match="cyan_candidate_two_view_mismatch"):
        route.observe(_strong_candidate_jpeg(31, 66),
            own_attachment=attachment, own_sha256=own_sha,
            observed_at_s=1.9, frame_id=5)
    assert route.box_carrier_relink_audit["previous_provisional_rejected"]
    assert route.box_carrier_occluded_frames == 5
    assert route.box_carrier_cumulative_motion_px == pytest.approx(15.)


def test_provisional_cyan_at_final_slot_stops_for_new_view_without_done():
    route = _two_view_route()
    route.points = [np.array([63., 60.])]
    route.index = 0
    attachment, own_sha = _attachment()
    action, evidence = route.observe(_strong_candidate_jpeg(28, 58),
        own_attachment=attachment, own_sha256=own_sha,
        observed_at_s=1.65, frame_id=4)
    assert action["kind"] == "mecanum"
    assert action["forward"] == action["left"] == 0.
    assert evidence["waiting_for_second_direct_cargo"]
    assert not evidence["ready"] and not evidence["done"]
    assert route.confirmations == 0


def _stationary_final_followup(monkeypatch):
    route = _two_view_route()
    del route._track_carrier_under_occlusion  # Exercise the real carrier gate.
    route.map = _release_route().map
    route.points = [np.array([63., 60.])]
    route.index = 0
    route.box_direct_final_hold_pending = True
    route.box_carrier_occluded_frames = 4
    route.box_carrier_occlusion_last_at_s = 1.65
    route.box_carrier_occlusion_last_frame_id = 4
    route.box_carrier_points = _cloud(x=33.)
    route.box_center = np.array([63., 60.])
    route.box_delta = np.array([3., 0.])
    jpeg = _strong_candidate_jpeg(28, 58)
    route.box_previous = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    route.box_previous_top_sha256 = hashlib.sha256(jpeg).hexdigest()
    route.box_direct_recovery_pending = {
        "center_px": [60.5, 59.5], "bounds_px": [[58, 57], [64, 63]],
        "area_px": 36, "hue_median": 90., "aspect_ratio": 1.,
        "foreground_fraction": 1., "observed_at_s": 1.65,
        "frame_id": 4, "top_sha256": route.box_previous_top_sha256}
    route._tracked_points = lambda old, new, points: _flow(points, (0., 0.))
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: _cloud(x=10., y=10.))
    return route, jpeg


def test_final_hold_confirms_same_top_pixels_from_new_frame_and_own_capture(monkeypatch):
    route, jpeg = _stationary_final_followup(monkeypatch)
    attachment, own_sha = _attachment()
    _, evidence = route.observe(jpeg, own_attachment=attachment,
        own_sha256=own_sha, observed_at_s=1.9, frame_id=5)
    tracking = evidence["tracking"]
    assert tracking["method"] == "bounded two-view direct cyan recovery from TOP RGB"
    assert tracking["bounded_direct_reappearance"]["stationary_final_hold"]
    assert tracking["bounded_direct_reappearance"]["status"] == (
        "confirmed_two_view_direct_cargo")
    assert route.box_carrier_occluded_frames == 0
    assert route.box_direct_final_hold_pending is False


def test_stationary_final_hold_rejects_reused_capture_identity(monkeypatch):
    route, jpeg = _stationary_final_followup(monkeypatch)
    attachment, own_sha = _attachment()
    with pytest.raises(RuntimeError, match="stale_or_reversed_carrier_capture"):
        route.observe(jpeg, own_attachment=attachment,
            own_sha256=own_sha, observed_at_s=1.9, frame_id=4)


def _release_route():
    route = _route()
    route.map = {"top_camera": {"quaternion_wxyz": [1, 0, 0, 0],
                                 "position_m": [0, 0, 1], "fov_y_deg": 90},
                 "docks": {"dock_b": {"slots": {"box": {
                     "center_m": [0, 0], "half_extents_m": [.3, .3]}}}}}
    route.points = [np.array([55.5, 55.5])]
    route.index = 0
    previous = np.zeros((112, 112, 3), np.uint8)
    previous[50:59, 50:59] = (255, 255, 0)
    prior_jpeg = cv2.imencode(".jpg", previous)[1].tobytes()
    route.box_background = np.zeros_like(previous)
    route.box_previous_top_sha256 = hashlib.sha256(prior_jpeg).hexdigest()
    route.box_direct_history = [{"center_px": (54., 54.),
                                 "top_sha256": route.box_previous_top_sha256,
                                 "observed_at_s": 1., "frame_id": 1}]
    return route


def _current_release_jpeg(*, cargo=True):
    image = np.zeros((112, 112, 3), np.uint8)
    if cargo:
        image[51:60, 51:60] = (255, 255, 0)
    return cv2.imencode(".jpg", image)[1].tobytes()


def test_release_pose_requires_new_direct_cargo_and_current_own_capture():
    route = _release_route()
    attachment, own_sha = _attachment()
    gate = route.verify_release_visual(
        _current_release_jpeg(), attachment, observed_at_s=1.2,
        frame_id=2, own_sha256=own_sha)
    assert gate["component_area_px"] >= 25
    assert gate["top_sha256"] == route.box_previous_top_sha256
    assert gate["own_sha256"] == own_sha
    # A stationary box may make identical JPEG bytes on a new, paired capture.
    repeated = route.verify_release_visual(
        _current_release_jpeg(), attachment, observed_at_s=1.4,
        frame_id=3, own_sha256=own_sha)
    assert repeated["top_sha256"] == gate["top_sha256"]
    with pytest.raises(RuntimeError, match="recent final direct TOP cargo"):
        route.verify_release_visual(_current_release_jpeg(), attachment,
                                    observed_at_s=1.4, frame_id=3,
                                    own_sha256=own_sha)


@pytest.mark.parametrize("invalid", ["no_cargo", "detached", "stale", "own_sha"])
def test_release_pose_fails_when_current_evidence_is_missing(invalid):
    route = _release_route()
    attachment, own_sha = _attachment()
    evidence, own, validated = attachment
    jpeg = _current_release_jpeg()
    at = 1.2
    if invalid == "no_cargo":
        jpeg = _current_release_jpeg(cargo=False)
    elif invalid == "detached":
        attachment = ({**evidence, "attached": False}, own, validated)
    elif invalid == "stale":
        at = 1.7
    else:
        own_sha = "0" * 64
    with pytest.raises(RuntimeError, match="dispatch release"):
        route.verify_release_visual(jpeg, attachment, observed_at_s=at,
                                    frame_id=2, own_sha256=own_sha)


def test_transport_checks_new_release_rgb_once_before_first_lowering_pose():
    route = _release_route()
    gate = Mock(return_value={"support_source": "current direct TOP cargo"})
    route.verify_release_visual = gate
    transport = SoloBoxTransport(robot_id="r2", navigator=route)
    transport.initialized = True
    transport.box.phase = "release"
    transport.box._carry_previous_image = _attachment()[0][1]
    transport.box._compare_attachment = Mock(return_value=_attachment()[0][0])
    transport.box.decide = Mock(return_value={"kind": "pose", "pulses": {1: 1500}})
    attachment, own_sha = _attachment()
    own = {"image": attachment[1], "sha256": own_sha,
           "sim_time": 1.2, "frame_id": 2}
    action, evidence = transport.decide(own, _current_release_jpeg())
    assert action["kind"] == "pose"
    assert evidence["release_direct_gate"]["support_source"] == "current direct TOP cargo"
    assert gate.call_count == 1
    transport.decide(own, _current_release_jpeg())
    assert gate.call_count == 1


def test_release_gate_failure_prevents_arm_pose_command():
    route = _release_route()
    route.verify_release_visual = Mock(side_effect=RuntimeError(
        "dispatch release current direct TOP cargo unresolved or ambiguous"))
    transport = SoloBoxTransport(robot_id="r2", navigator=route)
    transport.initialized = True
    transport.box.phase = "release"
    transport.box._carry_previous_image = _attachment()[0][1]
    transport.box._compare_attachment = Mock(return_value=_attachment()[0][0])
    transport.box.decide = Mock(return_value={"kind": "pose", "pulses": {1: 1500}})
    attachment, own_sha = _attachment()
    own = {"image": attachment[1], "sha256": own_sha,
           "sim_time": 1.2, "frame_id": 2}
    with pytest.raises(RuntimeError, match="dispatch release"):
        transport.decide(own, _current_release_jpeg(cargo=False))
    transport.box.decide.assert_not_called()


def test_opt_in_requires_rolling_realtime_open_scope(tmp_path, capsys):
    from scripts.run_dispatch_e2e import main
    with pytest.raises(SystemExit):
        main(["--output", str(tmp_path / "out"), "--bounded-carrier-relink"])
    assert "requires --rolling-visual-servo" in capsys.readouterr().err
    scene = SkillScene.__new__(SkillScene)
    scene.rolling_visual_servo = True
    scene.bounded_carrier_relink = True
    scene.realtime_control = True
    scene.bindings = type("Bindings", (), {"static_map": {
        "map_id": "dispatch_open", "terrain": [],
        "obstacles": [{"id": "service_island"}]}})()
    with pytest.raises(ValueError, match="without internal obstacles"):
        scene.start_solo()


@pytest.mark.parametrize("run", ["diag1", "diag2"])
def test_original_rolling_failure_boundary_has_new_current_rgb_support(
        run):
    manifest = json.loads((SAVED / "manifest.json").read_text())
    info = manifest["runs"][run]
    state = info["prior_state"]
    def rgb(index, kind):
        return (SAVED / f"{run}-{index}-{kind}.jpg").read_bytes()
    def image(index):
        return cv2.imdecode(np.frombuffer(rgb(index, "top"), np.uint8),
                            cv2.IMREAD_COLOR)
    route = ImageRoute(SkillBindings(committed(), authored_map("open")), "box",
                       time_aware_box_reacquisition=True,
                       bounded_carrier_relink=True)
    route.box_previous = image(info["prior_index"])
    route.box_previous_top_sha256 = hashlib.sha256(
        rgb(info["prior_index"], "top")).hexdigest()
    route.box_background = image(info["background_index"])
    route.box_background_sha = hashlib.sha256(
        rgb(info["background_index"], "top")).hexdigest()
    route.box_center = np.asarray(state["box_center_px"])
    route.box_delta = np.asarray(state["box_delta_px"])
    route.box_origin = np.asarray(state["box_origin_px"])
    route.points = [np.asarray(point) for point in state["waypoints_px"]]
    route.index = state["waypoint_index"]
    first, last = info["first_replay_index"], info["failed_original_index"]
    for index in range(first, last + 1):
        capture = (info["captures"].get(str(index))
                   or info["failed_capture"])
        attachment = own_sha = None
        if index > first:
            previous = base64.b64encode(rgb(index - 1, "own")).decode()
            own_raw = rgb(index, "own")
            current = base64.b64encode(own_raw).decode()
            visual = compare_box_comotion(previous, current, min_saturation=150)
            assert visual["attached"] is True
            attachment = (visual, current, current)
            own_sha = hashlib.sha256(own_raw).hexdigest()
        _, evidence = route.observe(
            rgb(index, "top"), own_attachment=attachment,
            observed_at_s=capture["observed_at_s"],
            frame_id=capture["frame_id"], own_sha256=own_sha)
        if index == first:
            assert evidence["tracking"]["method"].startswith(
                "bidirectional RGB feature motion")
        elif index < last:
            assert evidence["tracking"]["method"] == (
                "linked TOP RGB carrier motion during cargo occlusion")
    tracking = evidence["tracking"]
    audit = tracking["bounded_carrier_relink"]
    assert tracking["occluded_frames"] == 4
    assert audit["fresh_linked_corner_count"] >= 8
    assert audit["old_cloud_overlap_fraction"] >= .6
    assert audit["current_orange_foreground_fraction"] >= .6
    assert audit["same_capture_own_attachment_validated"]
    assert audit["current_top_sha256"] == hashlib.sha256(rgb(last, "top")).hexdigest()
    assert audit["current_own_sha256"] == hashlib.sha256(rgb(last, "own")).hexdigest()
    assert audit["occluded_frames_after"] == 4
    assert audit["cumulative_visual_motion_px_after"] < audit["max_visual_motion_px"]


def test_saved_failure_boundary_fixture_preserves_original_jpeg_bytes():
    manifest = json.loads((SAVED / "manifest.json").read_text())
    assert len(manifest["files_sha256"]) == 24
    for name, expected in manifest["files_sha256"].items():
        assert hashlib.sha256((SAVED / name).read_bytes()).hexdigest() == expected
