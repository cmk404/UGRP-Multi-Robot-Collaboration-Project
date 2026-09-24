"""Exact arithmetic and pixel-preservation checks for beam scan acceleration."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.camera_beam_shaft import _slices, robust_shaft_geometry


FIXTURES = Path(__file__).parent / "fixtures" / "camera_beam_shaft"


def _reference_slices(points, origin, axis):
    normal = np.asarray((-axis[1], axis[0]))
    axial = (points - origin) @ axis
    lateral = (points - origin) @ normal
    indices = np.floor(axial).astype(int)
    rows = []
    for index in range(int(indices.min()), int(indices.max()) + 1):
        values = lateral[indices == index]
        if len(values) >= 2:
            rows.append((index, float(values.min()), float(values.max()),
                         float(np.median(values)), len(values)))
    return axial, lateral, rows


@pytest.mark.parametrize("seed", range(10))
def test_grouped_slices_match_reference_exactly(seed):
    random = np.random.default_rng(seed)
    points = random.integers(-120, 120, size=(900, 2)).astype(float)
    angle = seed * .21
    axis = np.asarray((np.cos(angle), np.sin(angle)))
    origin = np.median(points, axis=0)
    expected = _reference_slices(points, origin, axis)
    actual = _slices(points, origin, axis)
    assert np.array_equal(actual[0], expected[0])
    assert np.array_equal(actual[1], expected[1])
    assert actual[2] == expected[2]


@pytest.mark.parametrize("name", ["v12-r000-top.jpg", "v12-r374-top.jpg"])
@pytest.mark.parametrize("saturation", [105, 135, 170, 190])
def test_cropped_contours_preserve_exact_full_frame_masks_and_geometry(name, saturation):
    frame = cv2.imdecode(np.frombuffer((FIXTURES / name).read_bytes(), np.uint8),
                         cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray((3, saturation, 45), np.uint8),
                       np.asarray((35, 255, 255), np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    checked = 0
    for contour in contours:
        if cv2.contourArea(contour) < max(100., mask.size * .00045):
            continue
        full = np.zeros_like(mask)
        cv2.drawContours(full, [contour], -1, 255, thickness=cv2.FILLED)
        full = cv2.bitwise_and(full, mask)
        x, y, width, height = cv2.boundingRect(contour)
        cropped = np.zeros((height, width), np.uint8)
        cv2.drawContours(cropped, [contour], -1, 255,
                         thickness=cv2.FILLED, offset=(-x, -y))
        cropped = cv2.bitwise_and(cropped, mask[y:y + height, x:x + width])
        assert np.array_equal(cropped, full[y:y + height, x:x + width])
        baseline = robust_shaft_geometry(full)
        accelerated = robust_shaft_geometry(cropped, offset_xy=(x, y))
        assert (baseline is None) == (accelerated is None)
        if baseline is not None:
            for key, value in baseline.items():
                other = accelerated[key]
                if isinstance(value, np.ndarray):
                    assert np.array_equal(value, other)
                else:
                    assert value == other
        checked += 1
    assert checked >= 1


@pytest.mark.parametrize("x,y,length", [(0, 1, 180), (290, 250, 109)])
def test_crop_coordinates_at_image_boundary_are_exact(x, y, length):
    full = np.zeros((300, 400), np.uint8)
    cv2.rectangle(full, (x, y), (x + length, y + 21), 255, -1)
    baseline = robust_shaft_geometry(full)
    cropped = robust_shaft_geometry(full[y:y + 22, x:x + length + 1],
                                    offset_xy=(x, y))
    assert baseline is not None and cropped is not None
    for key, value in baseline.items():
        if isinstance(value, np.ndarray):
            assert np.array_equal(value, cropped[key])
        else:
            assert value == cropped[key]
