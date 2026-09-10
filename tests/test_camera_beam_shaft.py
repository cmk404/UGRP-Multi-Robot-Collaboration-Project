"""Image-only regression tests for stable beam-shaft geometry."""
from pathlib import Path
import hashlib
import json
import math

import cv2
import numpy as np
import pytest

from harness.camera_beam_features import extract_beams
from harness.camera_beam_shaft import robust_shaft_geometry


ORANGE = (0, 140, 255)
FIXTURES = Path(__file__).parent / "fixtures" / "camera_beam_shaft"


def _encoded_scene(center, length, width, angle, *, lobe=None, noise=True, size=(720, 540)):
    image = np.full((size[1], size[0], 3), 45, dtype=np.uint8)
    box = cv2.boxPoints((center, (length, width), angle)).astype(np.int32)
    cv2.fillConvexPoly(image, box, ORANGE)
    radians = math.radians(angle)
    axis = np.asarray((math.cos(radians), math.sin(radians)))
    if lobe:
        end = np.asarray(center) + axis * length / 2
        lobe_angle = angle + (90 if lobe == "perpendicular" else 45)
        direction = np.asarray((math.cos(math.radians(lobe_angle)),
                                math.sin(math.radians(lobe_angle))))
        tip = end + direction * 25
        cv2.line(image, np.rint(end).astype(int), np.rint(tip).astype(int), ORANGE, 7)
        cv2.circle(image, np.rint(tip).astype(int), 7, ORANGE, -1)
    if noise:
        cv2.circle(image, (35, 45), 4, ORANGE, -1)
        cv2.circle(image, (680, 500), 3, ORANGE, -1)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return encoded.tobytes()


def _axis_error_degrees(beam, expected):
    endpoints = np.asarray(beam["endpoints"])
    delta = endpoints[1] - endpoints[0]
    measured = math.degrees(math.atan2(delta[1] * beam["image_size"][1],
                                       delta[0] * beam["image_size"][0]))
    return abs((measured - expected + 90) % 180 - 90)


@pytest.mark.parametrize("angle,length,width", [(-37, 170, 18), (23, 240, 27), (71, 125, 14)])
def test_rotated_and_scaled_shafts_preserve_image_geometry(angle, length, width):
    jpeg = _encoded_scene((360, 270), length, width, angle)
    baseline = extract_beams(jpeg)[0]
    beam = extract_beams(jpeg, robust_shaft=True)[0]

    assert beam["length_px"] == pytest.approx(baseline["length_px"], abs=4)
    assert beam["width_px"] == pytest.approx(baseline["width_px"], abs=1)
    assert _axis_error_degrees(beam, angle) < 1.0
    assert np.allclose(beam["center"], [.5, .5], atol=.01)


@pytest.mark.parametrize("angle,lobe", [(-31, "perpendicular"), (38, "diagonal")])
@pytest.mark.parametrize("end_sign", [-1, 1])
def test_attached_end_lobes_do_not_warp_the_shaft(angle, lobe, end_sign):
    # Reversing the shaft angle attaches the same lobe generator at either
    # physical end without giving the estimator an image-coordinate prior.
    effective_angle = angle if end_sign > 0 else angle + 180
    clean = extract_beams(
        _encoded_scene((360, 270), 210, 20, effective_angle), robust_shaft=True,
    )[0]
    beam = extract_beams(
        _encoded_scene((360, 270), 210, 20, effective_angle, lobe=lobe),
        robust_shaft=True,
    )[0]

    assert beam["length_px"] == pytest.approx(clean["length_px"], abs=5)
    assert beam["width_px"] == pytest.approx(clean["width_px"], abs=1)
    assert _axis_error_degrees(beam, angle) < 1.0
    assert np.allclose(beam["center"], [.5, .5], atol=.012)


def test_disconnected_noise_does_not_change_candidate_area_or_geometry():
    clean = extract_beams(_encoded_scene((360, 270), 190, 22, 14, noise=False), robust_shaft=True)[0]
    noisy = extract_beams(_encoded_scene((360, 270), 190, 22, 14, noise=True), robust_shaft=True)[0]

    assert noisy["area_px"] == pytest.approx(clean["area_px"], abs=2)
    assert noisy["width_px"] == pytest.approx(clean["width_px"], abs=.2)
    assert np.allclose(noisy["endpoints"], clean["endpoints"], atol=2 / 540)


def test_unsupported_compact_component_is_rejected_only_in_robust_mode():
    image = np.full((300, 400, 3), 45, dtype=np.uint8)
    cv2.circle(image, (200, 150), 35, ORANGE, -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok

    assert len(extract_beams(encoded.tobytes())) == 1
    assert extract_beams(encoded.tobytes(), robust_shaft=True) == []
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2.rectangle(mask, (30, 30), (70, 70), 255, -1)
    assert robust_shaft_geometry(mask) is None


def test_v12_clean_and_attached_tail_frames_share_the_same_shaft():
    metadata = json.loads((FIXTURES / "SOURCE.json").read_text())
    beams = []
    for name, expected_hash in metadata["files"].items():
        payload = (FIXTURES / name).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == expected_hash
        candidates = extract_beams(payload, robust_shaft=True)
        beams.append(min(candidates, key=lambda item: abs(item["center"][0] - .508)))

    clean, attached = beams
    for beam in beams:
        endpoints = np.asarray(beam["endpoints"]) * beam["image_size"]
        assert beam["width_px"] == pytest.approx(18, abs=1)
        assert endpoints[:, 0] == pytest.approx([650.5, 650.5], abs=1)
        assert endpoints[:, 1] == pytest.approx([396, 563], abs=2)
        assert _axis_error_degrees(beam, 90) < 1.0
    assert attached["area_px"] > clean["area_px"] + 100
