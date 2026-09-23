"""Coarse RGB preparation may be shared without sharing wheel identity state."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import (
    PairCoarsePixels, _cached_coarse_beam, _cached_coarse_raw_mask,
    _cached_coarse_reference_lane, _coarse_beam, _coarse_raw_mask,
    _coarse_reference_lane, _coarse_small_components,
)


FIXTURES = Path(__file__).parent / 'fixtures'
REFERENCE = (FIXTURES / 'camera_goal_transport' / 'reference-top.jpg').read_bytes()
TOP = (FIXTURES / 'dispatch_adaptive' / 'blocked-pickup-top.jpg').read_bytes()
IDENTITY = {
    'r1': {'claim': {'valid': True, 'center': [.113, .2]}},
    'r3': {'claim': {'valid': True, 'center': [.109, .499]}},
}
BINDINGS = SimpleNamespace(pair={'r1': 'r1', 'r3': 'r3'})


def controller():
    return PairCoarsePixels(IDENTITY, BINDINGS, REFERENCE)


def test_component_lookup_preserves_exact_300_pixel_boundary_and_background():
    yellow = np.zeros((40, 60), dtype=np.uint8)
    yellow[1:11, 1:31] = 255  # Exactly 300 pixels: included.
    yellow[15:22, 1:44] = 255  # 301 pixels: excluded.
    yellow[30, 55] = 255
    n, labels, stats, _ = cv2.connectedComponentsWithStats(yellow)
    original = np.zeros_like(yellow)
    count = 0
    for i in range(1, n):
        if stats[i, 4] <= 300:
            original[labels == i] = 255
            count += 1
    clean, selected, pixels = _coarse_small_components(yellow)
    assert np.array_equal(clean, original)
    assert selected == count == 2
    assert pixels == 301
    assert clean[0, 0] == clean[15, 1] == 0
    assert clean[1, 1] == clean[30, 55] == 255


def test_exact_jpeg_key_and_mutable_results_cannot_contaminate_cache():
    for cached in (_cached_coarse_raw_mask, _cached_coarse_beam,
                   _cached_coarse_reference_lane):
        cached.cache_clear()
    first, *metrics = _coarse_raw_mask(TOP)
    again, *same_metrics = _coarse_raw_mask(bytes(bytearray(TOP)))
    assert np.array_equal(first, again) and metrics == same_metrics
    assert _cached_coarse_raw_mask.cache_info().hits == 1
    first[0, 0] = 255
    assert _coarse_raw_mask(TOP)[0][0, 0] == 0
    _coarse_raw_mask(TOP + b'\x00')
    assert _cached_coarse_raw_mask.cache_info().misses == 2

    beam = _coarse_beam(TOP)
    expected_center = _coarse_beam(TOP)['center'][:]
    beam['center'][0] = -1
    assert _coarse_beam(TOP)['center'] == expected_center
    assert _cached_coarse_beam.cache_info().hits >= 2
    lane = _coarse_reference_lane(REFERENCE, 'r1')
    expected_x = _coarse_reference_lane(REFERENCE, 'r1')['beam_x']
    lane['beam_x'] = -1
    assert _coarse_reference_lane(REFERENCE, 'r1')['beam_x'] == expected_x


def test_bounded_cache_evicts_changed_rgb_inputs():
    _cached_coarse_raw_mask.cache_clear()
    images = []
    for value in range(5):
        frame = np.full((720, 960, 3), value * 20, dtype=np.uint8)
        images.append(cv2.imencode('.jpg', frame)[1].tobytes())
    for image in images:
        _coarse_raw_mask(image)
    info = _cached_coarse_raw_mask.cache_info()
    assert info.currsize == info.maxsize == 4
    misses = info.misses
    _coarse_raw_mask(images[0])
    assert _cached_coarse_raw_mask.cache_info().misses == misses + 1


def test_invalid_image_inputs_keep_fail_closed_boundary():
    coarse = controller()
    for raw in (b'', bytearray(TOP)):
        with pytest.raises(ValueError, match='nonempty JPEG required'):
            coarse.decide(raw, 'r1')
    with pytest.raises(ValueError, match='invalid JPEG'):
        coarse.decide(b'not a jpeg', 'r1')
    small = cv2.imencode('.jpg', np.zeros((40, 60, 3), np.uint8))[1].tobytes()
    with pytest.raises(ValueError, match='calibrated 960x720'):
        coarse.decide(small, 'r1')


def test_cached_rgb_does_not_share_wheel_centers_or_change_decisions():
    a, b = controller(), controller()
    first = a.decide(TOP, 'r1')
    assert first['ok'] is True
    initial_b = b.centers['r1'].copy()
    a.decide(TOP, 'r1')
    assert np.array_equal(b.centers['r1'], initial_b)
    assert b.decide(TOP, 'r1') == first
    assert np.array_equal(b.centers['r1'], np.asarray(first['wheel_center_px']))


def test_parallel_callers_receive_independent_masks_and_decisions():
    expected = controller().decide(TOP, 'r1')

    def classify(_):
        clean, *_ = _coarse_raw_mask(TOP)
        clean[0, 0] = 255
        return controller().decide(TOP, 'r1')

    with ThreadPoolExecutor(max_workers=8) as pool:
        assert all(row == expected for row in pool.map(classify, range(16)))
    assert _coarse_raw_mask(TOP)[0][0, 0] == 0
