"""Shared beam segmentation must leave each RGB tracker's prior independent."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import copy

import cv2
import numpy as np

from harness.camera_beam_features import (
    _cached_carried_beam_scans, carried_beam_scans, extract_beams,
)
from harness.dispatch_beam_tracker import CarriedBeamTracker


FIXTURES = Path(__file__).parent / 'fixtures' / 'dispatch_adaptive'
SATURATIONS = tuple(range(105, 191, 5))


def test_actual_rgb_threshold_candidates_equal_uncached_extraction():
    for name in ('carry-anchor.jpg', 'beam-floor-303.jpg',
                 'temporal-r8-failure.jpg', 'shaft-narrow-contrast.jpg'):
        jpeg = (FIXTURES / name).read_bytes()
        expected = tuple(tuple(extract_beams(jpeg, robust_shaft=True,
            hue_upper=35, min_saturation=s)) for s in SATURATIONS)
        assert carried_beam_scans(jpeg) == expected
        assert carried_beam_scans(jpeg) == expected


def test_same_rgb_different_tracker_priors_keep_independent_identity():
    frame = np.zeros((720, 960, 3), dtype=np.uint8)
    cv2.rectangle(frame, (220, 260), (233, 349), (0, 105, 255), -1)
    cv2.rectangle(frame, (680, 260), (693, 349), (0, 105, 255), -1)
    jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    candidates = carried_beam_scans(jpeg)[0]
    assert len(candidates) == 2
    left, right = sorted(candidates, key=lambda b: b['center'][0])
    trackers = [CarriedBeamTracker(), CarriedBeamTracker()]
    trackers[0].previous, trackers[1].previous = copy.deepcopy(left), copy.deepcopy(right)
    observed = [tracker.observe(jpeg) for tracker in trackers]
    assert observed[0]['center'] == left['center']
    assert observed[1]['center'] == right['center']
    assert observed[0]['tracking']['prior_center'] == left['center']
    assert observed[1]['tracking']['prior_center'] == right['center']
    assert observed[0]['tracking']['selected_saturation'] in SATURATIONS
    assert observed[1]['tracking']['selected_saturation'] in SATURATIONS


def test_cache_returns_independent_values_and_evicts_old_images():
    _cached_carried_beam_scans.cache_clear()
    jpeg = (FIXTURES / 'carry-anchor.jpg').read_bytes()
    original = carried_beam_scans(jpeg)
    changed = carried_beam_scans(jpeg)
    assert changed == original
    assert _cached_carried_beam_scans.cache_info().hits == 1
    first = next(b for level in changed for b in level)
    first['center'][0] = -1
    assert carried_beam_scans(jpeg) == original

    images = []
    for shade in range(5):
        frame = np.full((20, 20, 3), shade * 20, dtype=np.uint8)
        images.append(cv2.imencode('.jpg', frame)[1].tobytes())
    for image in images:
        carried_beam_scans(image)
    info = _cached_carried_beam_scans.cache_info()
    assert info.maxsize == info.currsize == 4
    misses = info.misses
    carried_beam_scans(images[0])
    assert _cached_carried_beam_scans.cache_info().misses == misses + 1
    assert carried_beam_scans((FIXTURES / 'beam-floor-303.jpg').read_bytes()) != original


def test_cached_scans_are_safe_to_use_in_parallel_threads():
    jpeg = (FIXTURES / 'carry-anchor.jpg').read_bytes()
    reference = carried_beam_scans(jpeg)

    def read_and_mutate(_):
        result = carried_beam_scans(jpeg)
        assert result == reference
        next(b for level in result for b in level)['center'][0] = -1

    with ThreadPoolExecutor(max_workers=8) as workers:
        list(workers.map(read_and_mutate, range(16)))
    assert carried_beam_scans(jpeg) == reference
