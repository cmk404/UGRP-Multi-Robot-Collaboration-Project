"""USB camera capture and red-block detection for MasterPi."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

from recorder import record_camera_frame, record_detection


STREAM_SNAPSHOT_URLS = (
    "http://127.0.0.1:8080/snapshot",
    "http://127.0.0.1:8080/?action=snapshot",
)
RED_LAB_MIN = (31, 158, 130)
RED_LAB_MAX = (255, 255, 255)
MIN_BLOB_AREA = 2500
FRAME_PATH = Path("/tmp/ugrp1-pick-red.jpg")
DEBUG_PATH = Path("/tmp/ugrp1-pick-red-debug.jpg")
# Blue/yellow delivery targets are 30-mm blocks.  A long, narrow colour strip
# can be exposed by the eye-in-hand camera from the robot itself, but cannot
# provide a safe floor-target centre/range observation.  Keep this separate
# from the calibrated red detector.
COLOR_BLOB_MAX_ASPECT = 2.4
COLOR_BLOB_SHORT_SIDE_FRACTION = 0.90


@dataclass(frozen=True)
class ColorBlob:
    cx: int
    cy: int
    area: int
    width: int
    height: int
    nx: float
    ny: float
    # Oriented contour geometry retained for block-face alignment.  Defaults
    # preserve compatibility with callers/tests that only need centroid+bbox.
    box_points: tuple[tuple[float, float], ...] = ()
    rectangularity: float = 0.0


# Backward-compatible name used by the physically tuned red pickup code.
RedBlob = ColorBlob


def capture_from_stream(url: str):
    import cv2
    import numpy as np
    from urllib.request import urlopen

    with urlopen(url, timeout=2.0) as response:
        data = response.read()
    if not data:
        return None
    return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)


def _open_v4l2():
    import cv2

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def _read_v4l2(cap, warmup: int):
    frame = None
    ok = False
    for _ in range(max(1, warmup)):
        ok, frame = cap.read()
        time.sleep(0.02)
    if not ok or frame is None:
        return None
    return frame


def capture_from_v4l2(warmup: int = 8):
    cap = _open_v4l2()
    if cap is None:
        return None
    try:
        return _read_v4l2(cap, warmup)
    finally:
        cap.release()


class LiveCamera:
    """Keep V4L2 open so each grab is a new frame, not the same buffered still."""

    def __init__(self):
        self._cap = None
        self.source = None

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def read(self, *, quiet: bool = False):
        for url in STREAM_SNAPSHOT_URLS:
            try:
                frame = capture_from_stream(url)
            except Exception:
                continue
            if frame is not None:
                if self.source != url and not quiet:
                    print(f"camera via {url}", flush=True)
                self.source = url
                record_camera_frame(frame, source=url)
                return frame

        if self._cap is None:
            self._cap = _open_v4l2()
            if self._cap is None:
                raise RuntimeError(
                    "could not read a camera frame. ustreamer is likely holding "
                    "/dev/video0; expected a JPEG at http://127.0.0.1:8080/snapshot"
                )
            warmup = 12
            self.source = "/dev/video0"
            if not quiet:
                print("camera via /dev/video0", flush=True)
        else:
            warmup = 3
        frame = _read_v4l2(self._cap, warmup)
        if frame is None:
            self.close()
            raise RuntimeError("camera opened but returned no frame")
        record_camera_frame(frame, source=self.source)
        return frame


def capture_bgr(warmup: int = 8, *, quiet: bool = False):
    for url in STREAM_SNAPSHOT_URLS:
        try:
            frame = capture_from_stream(url)
        except Exception:
            continue
        if frame is not None:
            if not quiet:
                print(f"camera via {url}", flush=True)
            record_camera_frame(frame, source=url)
            return frame

    frame = capture_from_v4l2(warmup=warmup)
    if frame is not None:
        if not quiet:
            print("camera via /dev/video0", flush=True)
        record_camera_frame(frame, source="/dev/video0")
        return frame

    raise RuntimeError(
        "could not read a camera frame. ustreamer is likely holding "
        "/dev/video0; expected a JPEG at http://127.0.0.1:8080/snapshot"
    )


def is_small_edge_blob(
    nx: float,
    area: float,
    *,
    edge_frac: float = 0.15,
    min_edge_area: int = 3000,
) -> bool:
    """Ignore tiny red blobs glued to the left/right of the frame.

    Those are usually the robot body, a cable, or a stale crop of the
    same still. A real block that fills the edge is kept.
    """
    return (nx < edge_frac or nx > 1.0 - edge_frac) and area < min_edge_area


def is_block_like_color_component(
    area: float,
    width: int,
    height: int,
    *,
    min_area: int,
) -> bool:
    """Whether a generic blue/yellow component can safely steer delivery.

    The minimum short side scales with the caller's area confidence floor,
    rather than hard-coding a particular camera distance.  This rejects a
    self/cable stripe even when its colour and contour fill are otherwise good.
    Red deliberately keeps its independently calibrated detector path.
    """
    short_side = min(int(width), int(height))
    long_side = max(int(width), int(height))
    if short_side <= 0 or long_side <= 0:
        return False
    minimum_short_side = math.sqrt(max(1.0, float(min_area))) * COLOR_BLOB_SHORT_SIDE_FRACTION
    return bool(
        short_side >= minimum_short_side
        and (float(long_side) / float(short_side)) <= COLOR_BLOB_MAX_ASPECT
    )


def _best_blob_from_mask(
    mask,
    *,
    min_area: int,
    crop_left: int,
    edge_frac: float,
    min_edge_area: int,
) -> ColorBlob | None:
    import cv2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = mask.shape[:2]
    best = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area <= best_area:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        # Eye-in-hand self/cable fragments are frequently clipped by the top
        # or side of the image. A floor cube used for navigation must be a
        # bounded component before it is allowed to steer the chassis.
        if (
            y <= 1
            or x <= max(1, crop_left + 1)
            or x + bw >= width - 2
            or y + bh >= height - 2
        ):
            continue
        aspect = bw / max(1.0, float(bh))
        fill = area / max(1.0, float(bw * bh))
        if not 0.25 <= aspect <= 4.0 or fill < 0.14:
            continue
        nx = (moments["m10"] / moments["m00"]) / width
        if is_small_edge_blob(nx, area, edge_frac=edge_frac, min_edge_area=min_edge_area):
            continue
        best = contour
        best_area = area
    if best is None:
        return None
    moments = cv2.moments(best)
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    _x, _y, bw, bh = cv2.boundingRect(best)
    rect = cv2.minAreaRect(best)
    rw, rh = rect[1]
    rect_area = max(1.0, float(rw) * float(rh))
    rectangularity = max(0.0, min(1.0, float(best_area) / rect_area))
    box = cv2.boxPoints(rect)
    box_points = tuple(
        (round(float(px) / width, 6), round(float(py) / height, 6))
        for px, py in box
    )
    return ColorBlob(
        cx=cx,
        cy=cy,
        area=int(best_area),
        width=int(bw),
        height=int(bh),
        nx=round(cx / width, 3),
        ny=round(cy / height, 3),
        box_points=box_points,
        rectangularity=round(rectangularity, 4),
    )


def _largest_red_suspicion_from_mask(mask, *, min_area: int = 120) -> ColorBlob | None:
    """Return the largest red component even when it is clipped by the frame.

    This is intentionally NOT a navigation/grasp detector. It exists so search
    never converts user-visible peripheral red into "nothing" merely because
    the component touches an image edge or lies inside the normal self-artifact
    crop. Search may use this only to move the camera for a better look; a
    normal bounded detection is still required before handoff to track.
    """
    import cv2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = mask.shape[:2]
    best = None
    best_area = 0.0
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < float(min_area) or area <= best_area:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        _x, _y, bw, bh = cv2.boundingRect(contour)
        if bw < 3 or bh < 3:
            continue
        fill = area / max(1.0, float(bw * bh))
        if fill < 0.08:
            continue
        best = contour
        best_area = area
    if best is None:
        return None
    moments = cv2.moments(best)
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    _x, _y, bw, bh = cv2.boundingRect(best)
    rect = cv2.minAreaRect(best)
    rw, rh = rect[1]
    rect_area = max(1.0, float(rw) * float(rh))
    rectangularity = max(0.0, min(1.0, float(best_area) / rect_area))
    box = cv2.boxPoints(rect)
    box_points = tuple(
        (round(float(px) / width, 6), round(float(py) / height, 6))
        for px, py in box
    )
    return ColorBlob(
        cx=cx, cy=cy, area=int(best_area), width=int(bw), height=int(bh),
        nx=round(cx / width, 3), ny=round(cy / height, 3),
        box_points=box_points, rectangularity=round(rectangularity, 4),
    )


def detect_red_suspicion(frame, *, min_area: int = 120) -> ColorBlob | None:
    """Peripheral-red observation for search gaze steering.

    Unlike detect_red_blob, this deliberately keeps components touching any
    frame border and does not crop the robot-side 100 px. The result is only
    a direction cue; it is never sufficient for track/approach/pick.
    """
    import cv2
    import numpy as np

    blurred = cv2.GaussianBlur(frame, (3, 3), 3)
    kernel = np.ones((3, 3), np.uint8)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    lab_mask = cv2.inRange(
        lab, np.array(RED_LAB_MIN, np.uint8), np.array(RED_LAB_MAX, np.uint8)
    )
    lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_OPEN, kernel)
    lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_CLOSE, kernel)

    b = blurred[:, :, 0].astype(np.int16)
    g = blurred[:, :, 1].astype(np.int16)
    r = blurred[:, :, 2].astype(np.int16)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    selected = (
        (r > 65)
        & ((r - g) > 20)
        & ((r - b) > 20)
        & (r * 100 > g * 118)
        & (r * 100 > b * 118)
        & (((h < 16) | (h > 168)))
        & (sat > 60)
        & (val > 55)
    )
    hsv_mask = selected.astype(np.uint8) * 255
    hsv_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_OPEN, kernel)
    hsv_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel)

    candidates = [
        _largest_red_suspicion_from_mask(lab_mask, min_area=min_area),
        _largest_red_suspicion_from_mask(hsv_mask, min_area=min_area),
    ]
    valid = [item for item in candidates if item is not None]
    if not valid:
        return None
    return max(valid, key=lambda item: float(item.area))


def red_blob_quality(blob: ColorBlob | None) -> float:
    """Rank a bounded red component by how much it resembles the target cube.

    Area remains the strongest cue, but a compact rectangular component is
    preferred over a thin/irregular red reflection.  This is intentionally a
    soft score: a rotated cube or a partly foreshortened face must still win.
    """
    if blob is None:
        return float("-inf")
    aspect = float(blob.width) / max(1.0, float(blob.height))
    aspect_quality = min(aspect, 1.0 / max(aspect, 1e-6))  # 1=square, falls smoothly
    shape_quality = 0.65 + 0.20 * float(blob.rectangularity) + 0.15 * aspect_quality
    return float(blob.area) * shape_quality


def choose_red_blob(*candidates: ColorBlob | None) -> ColorBlob | None:
    """Choose across independent colour models instead of trusting LAB first.

    The old detector returned immediately on any LAB hit.  On the physical
    robot that allowed a small unrelated red patch to hide a much larger cube
    that HSV found in the same frame.  Scoring all colour-model candidates
    fixes that failure without widening the calibrated colour thresholds.
    """
    valid = [item for item in candidates if item is not None]
    if not valid:
        return None
    return max(valid, key=red_blob_quality)


def detect_red_blob(
    frame,
    *,
    min_area: int = MIN_BLOB_AREA,
    crop_left: int = 100,
    edge_frac: float = 0.15,
    min_edge_area: int = 3000,
) -> ColorBlob | None:
    """Detect the physical red block with a calibrated + lighting fallback.

    The official MasterPi LAB calibration and a conservative BGR+HSV model are
    evaluated in parallel.  The best bounded, block-like component wins.  This
    prevents a small LAB-only red patch from masking a larger true cube while
    keeping the original calibrated thresholds and self-artifact rejection.
    """
    import cv2
    import numpy as np

    blurred = cv2.GaussianBlur(frame, (3, 3), 3)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    lab_mask = cv2.inRange(
        lab,
        np.array(RED_LAB_MIN, np.uint8),
        np.array(RED_LAB_MAX, np.uint8),
    )
    kernel = np.ones((3, 3), np.uint8)
    lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_OPEN, kernel)
    lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_CLOSE, kernel)
    if crop_left > 0:
        lab_mask[:, :crop_left] = 0
    lab_blob = _best_blob_from_mask(
        lab_mask,
        min_area=min_area,
        crop_left=crop_left,
        edge_frac=edge_frac,
        min_edge_area=min_edge_area,
    )

    b = blurred[:, :, 0].astype(np.int16)
    g = blurred[:, :, 1].astype(np.int16)
    r = blurred[:, :, 2].astype(np.int16)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    selected = (
        (r > 70)
        & ((r - g) > 25)
        & ((r - b) > 25)
        & (r * 100 > g * 125)
        & (r * 100 > b * 125)
        & (((h < 14) | (h > 170)))
        & (sat > 70)
        & (val > 60)
    )
    mask = selected.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if crop_left > 0:
        mask[:, :crop_left] = 0
    hsv_blob = _best_blob_from_mask(
        mask,
        min_area=max(300, int(min_area * 0.45)),
        crop_left=crop_left,
        edge_frac=edge_frac,
        min_edge_area=min_edge_area,
    )
    result = choose_red_blob(lab_blob, hsv_blob)
    record_detection(
        frame, color="red", blob=result, detector="red_lab+hsv",
        min_area=min_area, crop_left=crop_left,
    )
    return result


def detect_color_blob(
    frame,
    color: str,
    *,
    min_area: int = 800,
    crop_left: int = 0,
    edge_frac: float = 0.10,
    min_edge_area: int = 1200,
) -> ColorBlob | None:
    """Detect a blue/yellow block with the same RGB rules used by REAL memory.

    Red remains on the separately tuned LAB detector above. Keeping the two
    paths separate avoids silently weakening the pickup detector that has
    already been tuned on the physical robot.
    """
    import cv2
    import numpy as np

    name = str(color).strip().lower()
    if name not in {"blue", "yellow"}:
        raise ValueError("color must be 'blue' or 'yellow'")

    blurred = cv2.GaussianBlur(frame, (3, 3), 1)
    b = blurred[:, :, 0].astype(np.int16)
    g = blurred[:, :, 1].astype(np.int16)
    r = blurred[:, :, 2].astype(np.int16)
    if name == "yellow":
        selected = (
            (r > 100)
            & (g > 65)
            & (g * 100 > r * 45)
            & (g * 100 < r * 105)
            & (b < 100)
            & ((g - b) > 35)
        )
    else:
        selected = (
            (b > 70)
            & (b * 10 > r * 17)
            & (b * 10 > g * 15)
            & ((b - r) > 40)
            & ((b - g) > 30)
        )
    mask = selected.astype(np.uint8) * 255
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if crop_left > 0:
        mask[:, 0:crop_left] = 0

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = frame.shape[:2]
    best = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area <= best_area:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        _x, _y, bw, bh = cv2.boundingRect(contour)
        if not is_block_like_color_component(area, bw, bh, min_area=min_area):
            continue
        nx = (moments["m10"] / moments["m00"]) / width
        if is_small_edge_blob(nx, area, edge_frac=edge_frac, min_edge_area=min_edge_area):
            continue
        best = contour
        best_area = area
    if best is None:
        record_detection(
            frame, color=name, blob=None, detector=f"{name}_rgb",
            min_area=min_area, crop_left=crop_left,
        )
        return None
    moments = cv2.moments(best)
    if moments["m00"] == 0:
        return None
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    _x, _y, bw, bh = cv2.boundingRect(best)
    rect = cv2.minAreaRect(best)
    rw, rh = rect[1]
    rect_area = max(1.0, float(rw) * float(rh))
    rectangularity = max(0.0, min(1.0, float(best_area) / rect_area))
    box = cv2.boxPoints(rect)
    box_points = tuple(
        (round(float(px) / width, 6), round(float(py) / height, 6))
        for px, py in box
    )
    result = ColorBlob(
        cx=cx,
        cy=cy,
        area=int(best_area),
        width=int(bw),
        height=int(bh),
        nx=round(cx / width, 3),
        ny=round(cy / height, 3),
        box_points=box_points,
        rectangularity=round(rectangularity, 4),
    )
    record_detection(
        frame, color=name, blob=result, detector=f"{name}_rgb",
        min_area=min_area, crop_left=crop_left,
    )
    return result


def detect_target_blob(frame, color: str, **kwargs) -> ColorBlob | None:
    """Detect a selected block without weakening the tuned red path."""
    name = str(color).strip().lower()
    if name == "red":
        return detect_red_blob(frame, **kwargs)
    return detect_color_blob(frame, name, **kwargs)


def save_debug_frame(
    frame,
    blob: ColorBlob | None,
    path: Path,
    *,
    label: str = "red",
) -> None:
    import cv2

    record_camera_frame(frame, source="debug", force=True, label=label)
    vis = frame.copy()
    if blob is not None:
        x = blob.cx - blob.width // 2
        y = blob.cy - blob.height // 2
        cv2.rectangle(vis, (x, y), (x + blob.width, y + blob.height), (0, 255, 0), 2)
        cv2.circle(vis, (blob.cx, blob.cy), 6, (0, 255, 0), -1)
        if len(blob.box_points) == 4:
            import numpy as np
            pts = np.asarray(
                [[int(round(nx * vis.shape[1])), int(round(ny * vis.shape[0]))]
                 for nx, ny in blob.box_points],
                dtype=np.int32,
            )
            cv2.polylines(vis, [pts], True, (255, 255, 0), 2)
        cv2.putText(
            vis,
            f"{label} {blob.area}px",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
    cv2.imwrite(str(path), vis)
