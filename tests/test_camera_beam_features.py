import cv2
import numpy as np
import pytest

from harness.camera_beam_features import extract_beams, select_beam


ORANGE = (0, 140, 255)


def _jpeg(rectangles, size=(640, 480)):
    width, height = size
    image = np.full((height, width, 3), 45, dtype=np.uint8)
    for center, extent, angle in rectangles:
        box = cv2.boxPoints((center, extent, angle)).astype(np.int32)
        cv2.fillConvexPoly(image, box, ORANGE)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return encoded.tobytes()


def test_extracts_rotated_rectangle_geometry():
    candidates = extract_beams(_jpeg([((320, 240), (180, 30), 32)]))

    assert len(candidates) == 1
    beam = candidates[0]
    assert np.allclose(beam["center"], [0.5, 0.5], atol=0.01)
    assert beam["length_px"] == pytest.approx(180, abs=5)
    assert beam["width_px"] == pytest.approx(30, abs=6)
    assert len(beam["endpoints"]) == 2
    assert len(beam["corners4"]) == 4
    assert beam["image_size"] == [640, 480]
    assert not beam["touches_border"]


def test_returns_multiple_blobs_and_rejects_tiny_noise():
    jpeg = _jpeg([((120, 150), (90, 24), 0), ((500, 330), (130, 28), 70), ((20, 20), (8, 8), 0)])

    candidates = extract_beams(jpeg)

    assert len(candidates) == 2
    assert candidates[0]["area_px"] > candidates[1]["area_px"]


def test_selection_prefers_continuity_then_anchor():
    candidates = extract_beams(_jpeg([((120, 240), (100, 24), 0), ((500, 240), (100, 24), 0)]))

    anchored = select_beam(candidates, anchor=[0.2, 0.5], previous_center=None)
    continuous = select_beam(candidates, anchor=[0.2, 0.5], previous_center=[0.8, 0.5])

    assert anchored["center"][0] < 0.3
    assert continuous["center"][0] > 0.7


def test_requires_anchor_or_previous_center_and_handles_empty():
    candidates = extract_beams(_jpeg([((320, 240), (100, 24), 0)]))

    assert select_beam(candidates, anchor=None, previous_center=None) is None
    assert select_beam([], anchor=[0.5, 0.5], previous_center=None) is None
    assert extract_beams(b"") == []
    assert extract_beams(b"not a jpeg") == []


def test_clipped_blob_is_retained_and_flagged():
    candidates = extract_beams(_jpeg([((20, 220), (130, 55), 15)]))

    assert len(candidates) == 1
    assert candidates[0]["touches_border"] is True


def test_missing_previous_target_does_not_switch_to_distant_distractor():
    assert select_beam([{"center": [.8, .1], "area_px": 1000}], [.8, .1], [.2, .8]) is None
