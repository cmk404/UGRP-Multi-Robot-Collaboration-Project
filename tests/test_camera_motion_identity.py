import cv2
import numpy as np
import pytest

from harness.camera_motion_identity import ImageMotionIdentity


MOVE = {"kind": "drive", "forward": 0.03, "turn": 0.0, "duration_s": 0.2}


def jpeg(regions=(), shape=(100, 100)):
    image = np.zeros((*shape, 3), dtype=np.uint8)
    for x1, y1, x2, y2, color in regions:
        image[y1:y2, x1:x2] = color
    okay, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 100])
    assert okay
    return encoded.tobytes()


def test_stationary_or_settling_pixels_do_not_establish_identity():
    identity = ImageMotionIdentity()
    base = jpeg()
    assert identity.update(base, None)["center"] is None
    stationary = identity.update(base, MOVE)
    assert stationary["valid"] is False
    settling = identity.update(jpeg([(49, 49, 51, 51, (255, 255, 255))]), MOVE)
    assert settling["changed_pixels"] < 20
    assert settling["center"] is None


def test_compact_motion_identifies_normalized_motion_anchor():
    identity = ImageMotionIdentity()
    identity.update(jpeg(), None)
    result = identity.update(jpeg([(30, 55, 40, 65, (255, 255, 255))]), MOVE)
    assert result["valid"] and result["fresh"] and result["age"] == 0
    assert result["changed_pixels"] >= 20
    assert result["center"] == pytest.approx([34.5 / 99, 59.5 / 99], abs=0.02)


def test_far_second_region_and_broad_motion_are_rejected_without_anchor_swap():
    identity = ImageMotionIdentity()
    identity.update(jpeg(), None)
    first = identity.update(jpeg([(25, 55, 35, 65, (255, 255, 255))]), MOVE)
    anchor = first["center"]
    # Establish the next comparison baseline without attributing its visual change.
    identity.update(jpeg(), {"kind": "wait"})
    far = identity.update(jpeg([(75, 15, 85, 25, (255, 255, 255))]), MOVE)
    assert not far["valid"] and far["center"] == anchor

    other = ImageMotionIdentity()
    other.update(jpeg(), None)
    broad = other.update(jpeg([(5, 10, 95, 20, (255, 255, 255))]), MOVE)
    assert not broad["valid"] and broad["center"] is None


def test_anchor_expires_after_six_frames_without_valid_motion():
    identity = ImageMotionIdentity()
    identity.update(jpeg(), None)
    assert identity.update(jpeg([(40, 40, 50, 50, (255, 255, 255))]), MOVE)["valid"]
    for _ in range(6):
        retained = identity.update(jpeg([(40, 40, 50, 50, (255, 255, 255))]), {"kind": "wait"})
    assert retained["center"] is not None and retained["age"] == 6
    assert retained["valid"] and not retained["fresh"]
    expired = identity.update(jpeg([(40, 40, 50, 50, (255, 255, 255))]), {"kind": "wait"})
    assert expired["center"] is None and expired["age"] == 7
    assert not expired["valid"] and not expired["fresh"]


def test_stationary_frame_retains_existing_anchor_as_valid():
    identity = ImageMotionIdentity()
    identity.update(jpeg(), None)
    identity.update(jpeg([(40, 40, 50, 50, (255, 255, 255))]), MOVE)
    retained = identity.update(jpeg([(40, 40, 50, 50, (255, 255, 255))]), MOVE)
    assert retained["changed_pixels"] < 20
    assert retained["valid"] and not retained["fresh"] and retained["age"] == 1


@pytest.mark.parametrize("bad", [b"", b"not jpeg", b"\xff\xd8bad\xff\xd9", "jpeg"])
def test_invalid_jpeg_is_rejected_without_destroying_previous_frame(bad):
    identity = ImageMotionIdentity()
    identity.update(jpeg(), None)
    with pytest.raises(ValueError):
        identity.update(bad, MOVE)
    assert identity.update(jpeg([(45, 45, 55, 55, (255, 255, 255))]), MOVE)["valid"]
