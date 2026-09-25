"""Colour-generalised box detection for the zone benchmark (own RGB + TOP RGB).

Robot input only: a robot's own RGB JPEG with its own *commanded* arm pulses,
and the fixed TOP RGB JPEGs with the authored TOP calibration. No simulator
pose, segmentation or contact is read here.

Own RGB (``detect_own``)
    Per colour, silhouette components are fitted to the known upright box
    cuboid on the floor with the existing marker-free projection fit
    (``harness.markerless_box``, not modified). Profiles:

    * ``OWN_PROFILE_PRODUCTION`` (default): cyan components come from the
      production extractor ``markerless_box._cyan_components`` itself and only
      the production near-range floor fit (<= 1.2 m contact) is used.
    * ``OWN_PROFILE_ZONE``: opt-in. A narrower cyan range that excludes the
      blue zone pickup paint, plus a coarse far-range floor fit (1.2-4 m) that
      reports far boxes with ``range_class='far_coarse'``.

TOP RGB (``detect_top``)
    * ``TOP_PROFILE_BASELINE`` (default) is exactly
      ``harness.zone_perception.detect_boxes`` (used by the ZC1/ZC2 zone runs).
    * ``TOP_PROFILE_ZONE``: opt-in. Rotation-invariant box gates
      (``minAreaRect`` fill/aspect), a 3x3 close and ranges chosen on the dev
      split.

``harness.markerless_box.observe_ground_box`` (single cyan target, open-map
skill) and ``harness.zone_perception`` are untouched and keep their behaviour
byte for byte. Colour identifies a box *kind*, never an individual box.
"""
from __future__ import annotations

import base64
import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness import markerless_box as _mb
from harness import zone_perception as _zp

KINDS = ('cyan', 'green', 'red', 'yellow')
# Same prototype as the dispatch box (sim.zone_arena.BOX_HALF); catalog size.
BOX_HALF_M = (.017, .020, .016)
BOX_DIMS_M = (2*BOX_HALF_M[0], 2*BOX_HALF_M[1], 2*BOX_HALF_M[2])
PROVENANCE_OWN = 'own_rgb_colour_silhouette+raw_fisheye+own_commanded_camera_fk+known_floor_cuboid_projection'

OWN_PROFILE_PRODUCTION = 'own_production_v1'
OWN_PROFILE_ZONE = 'own_zone_v2'
TOP_PROFILE_BASELINE = 'zone_perception_v1'
TOP_PROFILE_ZONE = 'top_zone_v2'
PROFILE_BASELINE = TOP_PROFILE_BASELINE   # backwards-compatible name

# Own-RGB HSV ranges (OpenCV H 0..179), chosen on the dev split only
# (experiments/2026-09-25-zone-rgb-color). ``cyan`` under the production
# profile is not listed: it calls markerless_box._cyan_components.
OWN_HSV = {
    # red box H 178..3; zone A paint sits at H 10..13.
    'red': (((0, 150, 35), (6, 255, 255)), ((172, 150, 35), (179, 255, 255))),
    # green box H 60..67; dispatch beam-dock paint sits at H 71..74.
    'green': (((56, 140, 30), (70, 255, 255)),),
    # yellow box H 23..30, S >= 190; the orange beam and apron paint sit at H 13..23.
    'yellow': (((23, 170, 50), (33, 255, 255)),),
}
# Zone profile only: cyan box H 90..95 (S >= 135); the zone pickup paint sits
# at H 100..110 and leaks into the production H <= 104 range.
OWN_ZONE_CYAN_HSV = (((85, 120, 40), (98, 255, 255)),)
OWN_MIN_AREA_PX = _mb._MIN_AREA_PX
OWN_BORDER_PX = _mb._BORDER_PX
OWN_MIN_SOLIDITY = _mb._MIN_SOLIDITY
OWN_MIN_PROJECTION_IOU = _mb._MIN_PROJECTION_IOU
# Coarse far-range fit (zone profile): the production fit refuses a floor
# contact farther than 1.2 m from the camera.
FAR_MIN_RANGE_M, FAR_MAX_RANGE_M = 1.2, 4.0
FAR_MIN_PROJECTION_IOU = .50

# TOP zone profile (OpenCV H 0..179); chosen on the dev split only.
TOP_ZONE_HSV = {
    'cyan': (((84, 90, 40), (96, 255, 255)),),
    'red': (((0, 120, 50), (6, 255, 255)), ((172, 120, 50), (179, 255, 255))),
    'green': (((55, 100, 40), (72, 255, 255)),),
    'yellow': (((22, 140, 80), (33, 255, 255)),),
}
# Min area 50 px and rect fill 0.70 reject the striped yellow mecanum rollers
# (merged by the close) that pass every colour gate; box tops are ~100-190 px.
TOP_ZONE_AREA_PX = (50, 300)
TOP_ZONE_MIN_RECT_FILL = .70
TOP_ZONE_MAX_ASPECT = 2.2


def _frame(image) -> np.ndarray:
    if isinstance(image, np.ndarray):
        return image
    if isinstance(image, str):
        image = base64.b64decode(image)
    frame = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('invalid JPEG')
    return frame


def _mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
    return mask


def _colour_components(frame: np.ndarray, kind: str, profile: str = OWN_PROFILE_PRODUCTION):
    """Same morphology and gates as the production cyan extractor."""
    if kind == 'cyan' and profile == OWN_PROFILE_PRODUCTION:
        return _mb._cyan_components(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = _mask(hsv, OWN_ZONE_CYAN_HSV if kind == 'cyan' else OWN_HSV[kind])
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = frame.shape[:2]
    accepted, clipped = [], False
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < OWN_MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= OWN_BORDER_PX or y <= OWN_BORDER_PX or x+w >= width-OWN_BORDER_PX or y+h >= height-OWN_BORDER_PX:
            clipped = True
            continue
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area/max(hull_area, 1.)
        if solidity < OWN_MIN_SOLIDITY or w < 7 or h < 7:
            continue
        accepted.append({'contour': contour, 'area': area, 'solidity': solidity,
                         'bbox': (x, y, w, h), 'mask': mask})
    return accepted, clipped


def _far_coarse_fit(component, origin, axes, k, d, image_shape):
    """Known-cuboid projection for a floor contact 1.2-4 m away (coarse yaw)."""
    contour = component['contour'].reshape(-1, 2)
    max_y = float(np.max(contour[:, 1]))
    band = contour[contour[:, 1] >= max_y-max(2., component['bbox'][3]*.08)]
    contacts = [_mb._pixel_ground_point(p, origin, axes, k, d) for p in band]
    contacts = np.asarray([p for p in contacts if p is not None])
    if not len(contacts):
        return None
    contact = np.median(contacts[:, :2], axis=0)
    radial = contact-origin[:2]
    norm = float(np.linalg.norm(radial))
    if not FAR_MIN_RANGE_M < norm <= FAR_MAX_RANGE_M:
        return None
    radial /= norm
    a, b, _ = BOX_DIMS_M
    best = None
    for yaw_deg in range(0, 180, 10):
        yaw = math.radians(yaw_deg)
        support = (abs(math.cos(yaw)*radial[0]+math.sin(yaw)*radial[1])*a/2
                   + abs(-math.sin(yaw)*radial[0]+math.cos(yaw)*radial[1])*b/2)
        center = contact+radial*support
        pixels = _mb._project_points(_mb._cuboid_corners(center, yaw, BOX_DIMS_M), origin, axes, k, d)
        if pixels is None:
            continue
        iou = _mb._polygon_iou(component['contour'], pixels, image_shape)
        if best is None or iou > best[0]:
            best = (iou, center, yaw, pixels)
    return best


def detect_own(image, servo_pose: Mapping[int | str, int | float], kinds: Sequence[str] = KINDS,
               *, profile: str = OWN_PROFILE_PRODUCTION) -> dict[str, Any]:
    """All box-coloured floor cuboids in one own-RGB frame.

    ``servo_pose`` is the robot's own commanded arm pulses (servos 3-6). Each
    detection passed the colour silhouette gates and a known-cuboid floor
    projection gate. Positions are in the robot base frame (+x forward,
    +y left) and are conditional on the floor hypothesis; ``far_coarse``
    positions come from a coarse yaw search and are less accurate.
    """
    if profile not in (OWN_PROFILE_PRODUCTION, OWN_PROFILE_ZONE):
        raise ValueError(f'unknown own-RGB profile: {profile}')
    frame = _frame(image)
    height, width = frame.shape[:2]
    origin, axes = _mb.camera_extrinsics(servo_pose)
    origin, axes = np.asarray(origin, np.float64), np.asarray(axes, np.float64)
    k = _mb.scaled_camera_matrix(width, height)
    d = np.asarray(_mb.CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    detections, clipped_kinds = [], []
    for kind in kinds:
        if kind not in KINDS:
            raise ValueError(f'unknown box kind: {kind}')
        components, clipped = _colour_components(frame, kind, profile)
        if clipped:
            clipped_kinds.append(kind)
        for comp in components:
            fit = _mb._fit_floor_cuboid(comp, origin, axes, k, d, BOX_DIMS_M, frame.shape)
            range_class = 'near'
            if fit is None or fit[0] < OWN_MIN_PROJECTION_IOU:
                if profile != OWN_PROFILE_ZONE:
                    continue
                fit = _far_coarse_fit(comp, origin, axes, k, d, frame.shape)
                if fit is None or fit[0] < FAR_MIN_PROJECTION_IOU:
                    continue
                range_class = 'far_coarse'
            m = cv2.moments(comp['contour'])
            detections.append({
                'kind': kind, 'range_class': range_class,
                'pixel_centroid': [m['m10']/m['m00'], m['m01']/m['m00']],
                'pixel_bbox': [int(v) for v in comp['bbox']], 'area_px': comp['area'],
                'solidity': comp['solidity'], 'floor_hypothesis_projection_iou': float(fit[0]),
                'estimated_box_center_base_m': [float(fit[1][0]), float(fit[1][1]), BOX_HALF_M[2]],
                'estimated_yaw_mod_pi_rad': float(fit[2] % math.pi)})
    return {'detections': detections, 'clipped_kinds': clipped_kinds, 'image_size_px': [width, height],
            'profile': profile, 'provenance': PROVENANCE_OWN,
            'identity_source': 'colour kind only; individual box identity is not visually decoded'}


# ------------------------------------------------------------------ TOP RGB

def _detect_top_zone(jpeg, camera, kinds):
    frame = _zp._decode(jpeg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    found = []
    kernel = np.ones((3, 3), np.uint8)
    for kind in kinds:
        mask = cv2.morphologyEx(_mask(hsv, TOP_ZONE_HSV[kind]), cv2.MORPH_CLOSE, kernel)
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        for i in range(1, count):
            area = int(stats[i][4])
            if not TOP_ZONE_AREA_PX[0] <= area <= TOP_ZONE_AREA_PX[1]:
                continue
            points = np.column_stack(np.nonzero(labels == i)[::-1]).astype(np.float32)
            (_, _), (rw, rh), _ = cv2.minAreaRect(points)
            rw, rh = rw+1., rh+1.   # pixel extent, not centre-to-centre
            if area < TOP_ZONE_MIN_RECT_FILL*rw*rh or max(rw, rh) > TOP_ZONE_MAX_ASPECT*min(rw, rh):
                continue
            u, v = (float(c) for c in centers[i])
            fx, fy = _zp.pixel_to_floor(u, v, camera, frame.shape)
            found.append({'kind': kind, 'pixel': [round(u/frame.shape[1], 4), round(v/frame.shape[0], 4)],
                          'floor_xy_m': [round(fx, 3), round(fy, 3)], 'area_px': area,
                          'camera': camera['name']})
    return found


def detect_top(jpeg: bytes, camera: Mapping[str, Any], kinds: Sequence[str] = KINDS,
               *, profile: str = TOP_PROFILE_BASELINE) -> list[dict[str, Any]]:
    """TOP colour blobs; the baseline profile is ``zone_perception.detect_boxes``."""
    if profile == TOP_PROFILE_BASELINE:
        return _zp.detect_boxes(jpeg, camera, kinds)
    if profile == TOP_PROFILE_ZONE:
        return _detect_top_zone(jpeg, camera, kinds)
    raise ValueError(f'unknown TOP profile: {profile}')


__all__ = ['KINDS', 'BOX_HALF_M', 'OWN_HSV', 'OWN_PROFILE_PRODUCTION', 'OWN_PROFILE_ZONE',
           'TOP_PROFILE_BASELINE', 'TOP_PROFILE_ZONE', 'detect_own', 'detect_top']
