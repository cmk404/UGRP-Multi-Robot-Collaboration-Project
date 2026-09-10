import cv2
import numpy as np

from harness.camera_landmark_tracker import CameraLandmarkTracker


def frame(offset=(0, 0), occluded=False):
    image = np.zeros((100, 120, 3), np.uint8)
    if occluded:
        image[:] = 127
    else:
        dx, dy = offset
        for x, y in ((30, 35), (30, 55), (55, 45)):
            cv2.rectangle(image, (x-5+dx, y-5+dy), (x+5+dx, y+5+dy), (80, 180, 250), -1)
            cv2.line(image, (x-5+dx, y-5+dy), (x+5+dx, y+5+dy), (255, 20, 10), 2)
            cv2.line(image, (x-5+dx, y+5+dy), (x+5+dx, y-5+dy), (10, 255, 30), 2)
    ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 100])
    assert ok
    return data.tobytes()


def obs(view='own', present=True, confidence=.9, identity=.9, offset=(0, 0)):
    dx, dy = offset
    points = [[(30+dx)/119, (35+dy)/99], [(30+dx)/119, (55+dy)/99]]
    return {'view': view, 'jaws': points if present else None,
            'target': [(55+dx)/119, (45+dy)/99] if present else None,
            'confidence': confidence, 'identity_confidence': identity,
            'capture_visible': True, 'lift_visible': True, 'reason': 'pixels'}


def test_translated_texture_bridges_null_landmarks_and_clears_success_flags():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs())
    result = tracker.update(frame((3, 2)), frame(), obs(present=False))
    assert result['landmark_tracking']['source'] == 'optical_flow'
    assert result['landmark_tracking']['age'] == 1
    assert result['target'] == pytest.approx([58/119, 47/99], abs=.015)
    assert result['confidence'] == pytest.approx(.87)
    assert not result['capture_visible'] and not result['lift_visible']


def test_occlusion_and_null_first_observation_are_unavailable():
    tracker = CameraLandmarkTracker()
    assert tracker.update(frame(), frame(), obs(present=False))['jaws'] is None
    tracker.update(frame(), frame(), obs())
    result = tracker.update(frame(occluded=True), frame(), obs(present=False))
    assert result['landmark_tracking']['source'] == 'unavailable'
    assert result['jaws'] is None


def test_bridge_expires_after_three_frames():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs())
    for i in range(1, 4):
        result = tracker.update(frame((i, 0)), frame(), obs(present=False))
        assert result['landmark_tracking']['source'] == 'optical_flow'
        assert result['landmark_tracking']['age'] == i
        assert result['landmark_tracking']['confidence'] == pytest.approx(.9 - .03 * i)
    expired = tracker.update(frame((4, 0)), frame(), obs(present=False))
    assert expired['landmark_tracking']['source'] == 'unavailable'


def test_overhead_low_identity_never_seeds_and_views_are_separate():
    tracker = CameraLandmarkTracker()
    low = tracker.update(frame(), frame(), obs(view='overhead', identity=.79))
    assert low['landmark_tracking']['source'] == 'unavailable'
    tracker.update(frame(), frame(), obs(view='own'))
    overhead = tracker.update(frame((2, 0)), frame((2, 0)), obs(view='overhead', present=False))
    assert overhead['landmark_tracking']['source'] == 'unavailable'


def test_large_model_jump_prefers_verified_flow():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs())
    jumped = tracker.update(frame((2, 0)), frame(), obs(offset=(30, 0)))
    assert jumped['landmark_tracking']['source'] == 'optical_flow'
    assert jumped['target'][0] < .6


def test_bridge_never_exceeds_low_seed_confidence():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs(confidence=.7))
    bridged = tracker.update(frame((2, 0)), frame(), obs(present=False))
    assert bridged['confidence'] == pytest.approx(.67)


def test_agreeing_model_and_flow_are_fused_toward_flow():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs())
    current = obs(offset=(4, 2))  # Flow displacement is (2, 2), within agreement limit.
    result = tracker.update(frame((2, 2)), frame(), current)
    assert result['landmark_tracking']['source'] == 'fused'
    assert result['target'][0] == pytest.approx((.25 * 59 + .75 * 57) / 119, abs=.01)
    assert result['capture_visible'] and result['lift_visible']


def test_view_switch_invalidates_unpropagated_track():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs(view='own'))
    tracker.update(frame((1, 0)), frame(), obs(view='overhead'))
    back = tracker.update(frame((2, 0)), frame(), obs(view='own', present=False))
    assert back['landmark_tracking']['source'] == 'unavailable'


def test_overhead_flow_bridge_requires_current_identity_confidence():
    tracker = CameraLandmarkTracker()
    tracker.update(frame(), frame(), obs(view='overhead'))
    low = tracker.update(frame(), frame((2, 0)), obs(view='overhead', present=False, identity=.79))
    assert low['landmark_tracking']['source'] == 'unavailable'


def test_flat_pixel_landmarks_cannot_seed():
    tracker = CameraLandmarkTracker()
    blank = frame(occluded=True)
    result = tracker.update(blank, blank, obs())
    assert result['landmark_tracking']['source'] == 'unavailable'
    assert result['jaws'] is None


import pytest
