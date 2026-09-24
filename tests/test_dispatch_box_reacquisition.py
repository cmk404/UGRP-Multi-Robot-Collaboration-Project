"""Saved-camera tests for opt-in, time-aware direct TOP cargo recovery."""

import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import ImageRoute, SkillBindings
from harness.visual_attachment import compare_box_comotion
from test_dispatch_skill_binding import authored_map, committed


FRAMES = Path(__file__).parent / "fixtures/dispatch_box_reacquisition"


def _top(index):
    return (FRAMES / f"a-{index}-top.jpg").read_bytes()


def _frame(index):
    return cv2.imdecode(np.frombuffer(_top(index), np.uint8), cv2.IMREAD_COLOR)


def _attachment():
    prior = base64.b64encode((FRAMES / "a-144-own.jpg").read_bytes()).decode()
    raw = (FRAMES / "a-145-own.jpg").read_bytes()
    current = base64.b64encode(raw).decode()
    evidence = compare_box_comotion(prior, current, min_saturation=150)
    assert evidence["attached"] is True
    return (evidence, current, current), hashlib.sha256(raw).hexdigest()


def _route(*, enabled=True):
    route = ImageRoute(SkillBindings(committed(), authored_map("open")), "box",
                       time_aware_box_reacquisition=enabled)
    route.box_center = np.array([333.66101694915255, 575.9830508474577])  # saved #141
    route.box_delta = route.box_center - np.array([324.74242424242425, 577.1060606060606])
    route.box_origin = np.array([274.0131578947368, 545.4605263157895])
    route.box_background = _frame(117)
    route.box_background_sha = hashlib.sha256(_top(117)).hexdigest()
    route.box_previous = _frame(141)
    route.points = [np.array(x) for x in (
        [274.0131578947368, 586.3295695527553], [576.3174991993468, 586.3295695527553],
        [637.1742129817933, 586.3295695527553], [637.1742129817933, 359.5],
        [880.6010681115795, 359.5], [880.6010681115795, 176.92985865266039],
        [841.8740684318409, 176.92985865266039])]
    route.index = 1
    for index, when, frame_id, method in (
        (142, 102.4352499993057, 1013, "cyan component"),
        (143, 102.82549999929647, 1016, "bidirectional RGB feature motion"),
        (144, 103.03524999929151, 1018, "cyan component"),
    ):
        _, evidence = route.observe(_top(index), observed_at_s=when, frame_id=frame_id)
        assert evidence["tracking"]["method"].startswith(method)
    return route


def _observe_145(route, *, top=None, attachment=None, own_sha=None,
                 observed_at=103.40024999928288, frame_id=1021):
    if attachment is None:
        attachment, valid_sha = _attachment()
        if own_sha is None:
            own_sha = valid_sha
    return route.observe(_top(145) if top is None else top,
                         own_attachment=attachment, own_sha256=own_sha,
                         observed_at_s=observed_at, frame_id=frame_id)


def test_saved_holdout_a_reacquires_current_visible_cargo_after_legacy_failure():
    route = _route()
    assert len(route.box_direct_history) == 2  # #143 proxy was not direct evidence.
    _, evidence = _observe_145(route)
    tracking = evidence["tracking"]
    assert tracking["method"] == "time-aware direct cyan reacquisition from TOP RGB"
    assert np.allclose(evidence["cargo_center_px"], [397.2926829268293, 571.8048780487804])
    assert tracking["previous_direct_frame_ids"] == [1013, 1018]
    assert tracking["candidate_count_in_corridor"] == 1
    assert tracking["longitudinal_residual_px"] < tracking["longitudinal_limit_px"]
    assert tracking["lateral_residual_px"] < tracking["lateral_limit_px"]
    assert tracking["own_attachment"] == "fresh validated same-capture own RGB required"
    assert route.box_carrier_points is None
    assert route.confirmations == 0


def test_default_preserves_legacy_fail_closed_behavior():
    with pytest.raises(RuntimeError, match="linked carrier motion ambiguous"):
        _observe_145(_route(enabled=False))


@pytest.mark.parametrize("change", ["detached", "mismatched", "pan", "own_hash"])
def test_reacquisition_needs_current_own_attachment(change):
    route = _route()
    attachment, own_sha = _attachment()
    evidence, current, validated = attachment
    if change == "detached":
        attachment = ({**evidence, "attached": False}, current, validated)
    elif change == "mismatched":
        attachment = (evidence, current, base64.b64encode(b"other image").decode())
    elif change == "pan":
        attachment = ({**evidence, "camera_pan_delta_pwm": 60}, current, validated)
    else:
        own_sha = "0" * 64
    with pytest.raises(RuntimeError, match="own RGB attachment|linked carrier motion ambiguous"):
        _observe_145(route, attachment=attachment, own_sha=own_sha)


@pytest.mark.parametrize("observed_at,frame_id", [
    (104.0, 1021),  # Direct evidence is now more than 0.6 s old.
    (103.40024999928288, 1018),  # Reused or reversed capture ID.
    (103.03524999929151, 1021),  # Reused or reversed capture time.
])
def test_reacquisition_rejects_stale_or_reversed_capture(observed_at, frame_id):
    with pytest.raises(RuntimeError, match="linked carrier motion ambiguous"):
        _observe_145(_route(), observed_at=observed_at, frame_id=frame_id)


def test_reacquisition_rejects_background_only_cyan():
    route = _route()
    current = _frame(145)
    route.box_background[562:584, 391:405] = current[562:584, 391:405]
    with pytest.raises(RuntimeError, match="linked carrier motion ambiguous"):
        _observe_145(route)


def test_reacquisition_rejects_repeated_top_rgb_with_new_capture_metadata():
    # Existing narrow-gate cyan may accept this old image; the opt-in broad
    # reacquisition must never treat it as a new direct visual recovery.
    _, evidence = _observe_145(_route(), top=_top(144))
    assert evidence["tracking"]["method"] != "time-aware direct cyan reacquisition from TOP RGB"


def test_reacquisition_rejects_large_cyan_jump():
    route = _route()
    shifted = _frame(145)
    reference = _frame(117)
    shifted[560:586, 390:410] = reference[560:586, 390:410]
    cv2.rectangle(shifted, (497, 565), (504, 578), (200, 200, 0), -1)
    modified = cv2.imencode(".png", shifted)[1].tobytes()
    with pytest.raises(RuntimeError, match="linked carrier motion ambiguous"):
        _observe_145(route, top=modified)


def test_reacquisition_rejects_competing_cyan_in_visual_corridor():
    route = _route()
    mixed = _frame(145)
    cv2.rectangle(mixed, (387, 565), (392, 578), (200, 200, 0), -1)
    modified = cv2.imencode(".png", mixed)[1].tobytes()
    with pytest.raises(RuntimeError, match="linked carrier motion ambiguous"):
        _observe_145(route, top=modified)


def _route_with_inconsistent_patch_flow(monkeypatch):
    route = _route()
    route.box_carrier_points = None
    route.box_carrier_occluded_frames = 0
    # Four valid but mutually inconsistent KLT displacements reproduce the
    # branch where the legacy feature count passes yet no common motion does.
    monkeypatch.setattr(cv2, "goodFeaturesToTrack",
                        lambda *args, **kwargs: np.zeros((4, 1, 2), np.float32))
    delta = np.array([[0, 0], [4, 0], [0, 4], [-4, 0]], np.float32)
    route._tracked_points = lambda *_: (
        delta, np.zeros(4), np.zeros((4, 2)), np.zeros((4, 2)))
    assert route._consistent_group(delta, minimum=3) is None
    return route


def test_inconsistent_klt_without_carrier_can_reacquire_fresh_direct_cyan(monkeypatch):
    route = _route_with_inconsistent_patch_flow(monkeypatch)
    _, evidence = _observe_145(route)
    assert evidence["tracking"]["method"] == "time-aware direct cyan reacquisition from TOP RGB"
    assert np.allclose(evidence["cargo_center_px"], [397.2926829268293, 571.8048780487804])


@pytest.mark.parametrize("invalid", ["competing_cyan", "detached", "stale_time"])
def test_inconsistent_klt_without_carrier_still_rejects_ambiguous_or_stale_evidence(
        monkeypatch, invalid):
    route = _route_with_inconsistent_patch_flow(monkeypatch)
    kwargs = {}
    if invalid == "competing_cyan":
        mixed = _frame(145)
        cv2.rectangle(mixed, (387, 565), (392, 578), (200, 200, 0), -1)
        kwargs["top"] = cv2.imencode(".png", mixed)[1].tobytes()
    elif invalid == "detached":
        attachment, own_sha = _attachment()
        evidence, current, validated = attachment
        kwargs.update(attachment=({**evidence, "attached": False}, current, validated),
                      own_sha=own_sha)
    else:
        kwargs["observed_at"] = 103.635251  # More than 0.6 s after direct #144.
    with pytest.raises(RuntimeError, match="dispatch box unresolved or ambiguous in TOP RGB"):
        _observe_145(route, **kwargs)


def test_saved_holdout_b_dim_fragment_remains_unresolved():
    """A broad motion corridor alone does not make a dim fragment an identity."""
    route = ImageRoute(SkillBindings(committed(), authored_map("open")), "box",
                       time_aware_box_reacquisition=True)
    route.box_center = np.array([508.8, 564.4])  # saved direct #154
    route.box_delta = route.box_center - np.array([497.5, 566.625])
    route.box_origin = np.array([274.0921052631579, 545.4868421052631])
    route.box_background = cv2.imread(str(FRAMES / "b-121-top.jpg"))
    route.box_background_sha = hashlib.sha256((FRAMES / "b-121-top.jpg").read_bytes()).hexdigest()
    route.box_previous = cv2.imread(str(FRAMES / "b-154-top.jpg"))
    for index, when, frame_id in (
        (155, 115.49149999899696, 1072),
        (156, 115.74024999899108, 1074),
    ):
        _, evidence = route.observe((FRAMES / f"b-{index}-top.jpg").read_bytes(),
                                    observed_at_s=when, frame_id=frame_id)
        assert evidence["tracking"]["method"] == "cyan component"
        assert evidence["tracking"]["min_saturation"] == 125
    assert len(route.box_direct_history) == 2

    before = base64.b64encode((FRAMES / "b-156-own.jpg").read_bytes()).decode()
    own_raw = (FRAMES / "b-157-own.jpg").read_bytes()
    own = base64.b64encode(own_raw).decode()
    attachment = compare_box_comotion(before, own, min_saturation=150)
    assert attachment["attached"] is True
    assert attachment["mask_iou"] > .99

    current = cv2.imread(str(FRAMES / "b-157-top.jpg"))
    hsv = cv2.cvtColor(current, cv2.COLOR_BGR2HSV)
    foreground = cv2.absdiff(current, route.box_background).max(axis=2) > 20
    # The two strong-colour fragments near the visual prediction have areas 7
    # and 5, each below the existing 8-pixel component floor. Lowering the
    # saturation joins them, but this would expand identity at reacquisition.
    for saturation, expected_area in ((125, 7), (105, 14)):
        mask = cv2.inRange(hsv, np.array((80, saturation, 35), np.uint8),
                           np.array((102, 255, 255), np.uint8))
        mask[~foreground] = 0
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
        near = [int(stats[i, 4]) for i in range(1, count)
                if np.linalg.norm(centers[i] - [552, 567]) < 5]
        assert max(near) == expected_area
    with pytest.raises(RuntimeError, match="dispatch box unresolved or ambiguous in TOP RGB"):
        route.observe((FRAMES / "b-157-top.jpg").read_bytes(),
                      own_attachment=(attachment, own, own),
                      own_sha256=hashlib.sha256(own_raw).hexdigest(),
                      observed_at_s=116.00024999898493, frame_id=1077)


def test_saved_camera_fixture_hashes_match_manifest():
    manifest = json.loads((FRAMES / "manifest.json").read_text())
    for name, expected in manifest["files_sha256"].items():
        assert hashlib.sha256((FRAMES / name).read_bytes()).hexdigest() == expected
