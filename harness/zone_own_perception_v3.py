"""Wrist-RGB-only zone judgments, version 3 (package B follow-up to v2).

``harness/zone_own_perception.py`` (v1) and ``harness/zone_own_perception_v2.py``
(v2) are imported and **never modified**; their judgments keep answering byte for
byte (``tests/test_zone_own_perception_v3.py`` asserts it). v3 closes the three
gaps that the v2 test split and its report left open
(``experiments/2026-09-26-zone-own-perception-v2``):

1. **W7 - a held-item region that sees the background.** v1/v2 score a fixed
   image rectangle. In the CARRY posture a held can is not in that rectangle at
   all, so the rectangle showed a cyan box standing behind the robot and v2
   answered "holding cyan" at 0.82. v3 computes **where the item at my own grip
   must appear** from the issued arm pulses (forward kinematics of servos 3-6,
   the fixed camera calibration and the static cargo catalogue), and only pixels
   inside that predicted silhouette may count. If the ordered kind's silhouette is
   not in the rendered frame for the current posture the answer is ``unknown``.
   A colour blob only counts as held when it *is* the silhouette (overlap ratio),
   not when it merely covers part of it.
2. **A carried ``heavy_crate`` has no colour.** In the arm's shadow the crate
   renders at S 0 / V 7-13, so the v2 colour windows never fire. v3 identifies a
   carried team item from the predicted at-grip silhouette of each team kind: the
   kind whose silhouette the occupied (dark or cargo-coloured) pixels match best,
   by a margin, is the kind at the grip. Colour still wins when it is there.
3. **``tri_frame`` is not identified at the grasp stage.** The v2 pre-grasp look
   posture (tool pitch -54.9 deg) frames the floor of the reach band but only a
   sliver of the pale frame edges. v3 adds a measured grasp-stage look posture
   that frames both, so kind and "my handle is here" come from one frame.

Allowed inputs are exactly v1's and v2's: one own wrist fisheye frame, the
robot's own *issued* arm pulses, the static catalogue/map and the scenario order
sheet. The at-grip silhouette is a *prediction* from issued pulses and static
geometry; it is never checked against simulator state at runtime. ``import
mujoco`` never appears (the tests assert it).

Every judgment returns ``yes`` / ``no`` / ``unknown`` with a confidence.
``unknown`` stays first-class; a ``no`` always names the positive evidence.

Thresholds marked ``PREREGISTERED`` were fixed on the dev split before the
held-out split was scored (``experiments/2026-09-26-zone-own-perception-v3``).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from functools import lru_cache
from typing import Any

import cv2
import numpy as np

from harness import visual_arm as _arm
from harness import zone_color_boxes as _zcb
from harness import zone_own_perception as _v1
from harness import zone_own_perception_v2 as _v2
from sim import zone_cargo as _catalogue

SCHEMA = 'ugrp.zone_own_perception.v3'
PROFILE = 'zone_own_perception_v3'
ANSWERS = _v1.ANSWERS
JUDGMENTS = ('held_item_at_grip', 'team_cargo_at_grip', 'team_cargo_grasp_stage')
PROVENANCE = _v1.PROVENANCE
TEAM_KINDS = _v1.TEAM_KINDS
SOLO_KINDS = _v2.SOLO_KINDS

# --- at-grip geometry (PREREGISTERED) -----------------------------------------
# Hold convention of the calibrated grasp (sim/zone_cargo.py): the grip site is
# GRASP_Z_M above the item's floor contact and the robot faces the grasp role's
# approach heading. Between the jaws the item keeps its world-upright attitude in
# the recorded fixtures; for a physical hold the item is rigid in the gripper, and
# the caller passes the issued pose *at the grasp* as ``grasp_pose`` (camera and
# item are then one rigid body, so the silhouette is the one at grasp time).
# The first grasp role of every kind is used; each catalogue kind is symmetric
# across its roles (beam ends, crate lugs, frame vertices), so the silhouette is
# the same for every role.
# Render near clip: the simulator's pinhole wrist render (``robot_cam``) draws
# nothing nearer than znear x model extent = 0.0020 x 14.86 m = 0.0297 m along the
# optical axis, and faces cut by that plane vanish. Parts of a held item nearer
# than this are background in the frame, which is exactly how the v2 CARRY region
# came to show the arena behind a held can. This is a fixed property of the
# camera as rendered, like its FOV; a physical lens has its own near limit.
RENDER_NEAR_CLIP_M = .0297
CYLINDER_SIDES = 24
# Hold tolerance: a real hold is not exactly on the convention, and 4 mm at 3 cm
# from the lens moves the silhouette by tens of pixels (dev: a held can's lit band
# left the nominal silhouette in 2 of 3 ticks under the fixture's 4 mm / 3 mm hold
# jitter). The *band* is the union of the silhouettes over these offsets along the
# tool heading and in height; colour is counted in the band, sizes against the
# nominal silhouette.
HOLD_TOLERANCE_M = ((0., 0.), (.006, 0.), (-.006, 0.), (0., .006), (0., -.006))
HELD_MIN_SILHOUETTE_PX = 5000       # nominal silhouette of the ordered kind in the frame
HELD_FILL_MIN = .15                 # ordered colour in the band over the nominal silhouette -> yes
HELD_OTHER_FILL_MIN = .45           # another kind must fill its own silhouette to be claimed
HELD_CONTAINMENT_MIN = .85          # the colour blobs touching the band must lie in the band
HELD_EMPTY_MAX = .02                # ordered colour in the (dilated) band at most this share -> absent
HELD_EMPTY_DILATE_PX = 25
HELD_MARGIN = 1.6                   # ordered colour over any rival colour inside the band
HELD_LIT_MIN = .30                  # band 75th-percentile V over the frame's, before absence
# Held-item hue windows that differ from v1's (``hue_mask`` itself is untouched):
# the pickup-bay floor paint renders at OpenCV H 98-100 / S 95-109 under the
# ceiling light, inside v1's cyan band H 84-100 - that paint is what v2 read as
# "holding cyan" in the W7 failure (segmentation of the v2 test frame: 91,092 of
# the cyan pixels were ``zone_pickup`` paint, 6 were a box). A cyan box at the
# grip renders at H 93 (P5-P95 93-94, dev), so the held-item window stops at 97.
HELD_HUE_WINDOW = {'cyan': (84, 97)}
# Carried team item from its silhouette.
TEAM_MIN_SILHOUETTE_PX = 5000
TEAM_COLOUR_FILL_MIN = .25
TEAM_CONTAINMENT_MIN = .75          # dev: a carried beam 0.79-0.82 in 3 ticks (the blob runs to the frame edge)
TEAM_SHAPE_COLOUR_MAX = .02         # the shape cue is used only when no team colour is at the grip
# "Occupied, unlit" is an *absolute* V ceiling: a relative one fails exactly when
# the unlit item fills most of the frame (the frame's own 75th percentile is then
# the item). Dev: v2's dark crate rendered at V 7-13; over 437 dev-scan frames at
# most 13 % of a frame's floor/paint pixels (segmentation, eval only) fell at or
# below V 40, in the robot's own shadow - below the shape and emptiness gates.
DARK_MAX_VALUE = 40
SHAPE_IOU_MIN = .55                 # occupied pixels vs one kind's silhouette
SHAPE_MARGIN = 1.25                 # best kind's IoU over the runner-up
TEAM_EMPTY_LIT_SHARE = .85          # the ordered kind's band is lit floor ...
TEAM_EMPTY_OCCUPIED_MAX = .08       # ... and the union of all team bands is unoccupied -> nothing held

# --- grasp-stage look posture (PREREGISTERED after the dev scan) ---------------
# Dev scan (``scripts/eval_zone_own_perception_v3.py render --grasp-posture ...``,
# five candidates, same robot/item/tick jitter per view): v2's pre-grasp posture
# (tool pitch -54.9 deg) identified ``tri_frame`` in 1/4 grasp views; raising only
# the wrist (-50 / -44 deg) gave 4/4 but -44 deg made two confident "handle
# elsewhere" errors. Calibrated IK with the grip site 0.10 m ahead at 0.16 m and
# the tool at -39 deg sees the floor from 0.22 m to 0.88 m on the centre column:
# ``tri_frame`` 4/4, ``heavy_crate`` 4/4, handles 12/12 decided correct, 0
# confident errors (``long_beam`` identity 2/4, the rest unknown).
GRASP_LOOK_POSTURE = {1: 2000, 3: 699, 4: 2400, 5: 1321, 6: 1500}

# Handle contact plane (PREREGISTERED). v2 projects the lowest dark pixel of a
# handle onto the floor, but seen from above that pixel is on the handle, not
# under it: the beam's grip band lies on top of the 32 mm bar and a 46 mm lug's
# floor edge is hidden by its own top. Dev (``dev-frames``, v2 detections, contact
# pixel intersected with the plane z, error to the true grip point): z = 0 is
# +3.7 to +6.7 cm too far (the v2 test "7 mm outside the band" error), z = 0.032 m
# is +0.3 to +1.2 cm for all three kinds in both look postures. 0.032 m is the
# catalogue handle height (``sim.zone_cargo.HANDLE_HEIGHT_M``).
HANDLE_CONTACT_PLANE_Z_M = .032

CONFIDENCE = {
    'held_kind_matched': .88,
    'held_kind_conflict': .82,
    'held_absent': .75,
    'held_unknown': .0,
    'team_colour_matched': .88,
    'team_colour_conflict': .82,
    'team_shape_matched': .70,
    'team_shape_conflict': .70,
    'team_nothing_at_grip': .75,
    'team_unknown': .0,
}


# --------------------------------------------------------------------------- geometry

@lru_cache(maxsize=None)
def held_parts(kind: str) -> tuple[tuple[Any, ...], tuple[float, float, float], float]:
    """Static catalogue geometry of one kind for the at-grip prediction.

    Returns ``(parts, grip_xyz, approach_yaw)`` in the item's own frame (origin on
    the floor under the centroid). Each part is ``('box', centre, half, yaw)`` or
    ``('cylinder', centre, (radius, half_height), 0.)``. Non-colliding paint bands
    are skipped: they lie on the surface of a body part.
    """
    if kind in _zcb.KINDS:
        half = tuple(float(v) for v in _zcb.BOX_HALF_M)
        parts = (('box', (0., 0., half[2]), half, 0.),)
        return parts, (0., 0., float(_catalogue.GRASP_Z_M)), 0.
    spec = _catalogue.kind(kind)
    parts = tuple((p.shape, tuple(float(v) for v in p.center), tuple(float(v) for v in p.size),
                   float(p.yaw)) for p in spec.parts if p.collision)
    grasp = spec.grasps[0]
    return parts, tuple(float(v) for v in grasp.grip_xyz), float(grasp.approach_yaw)


def _pixel_rays(width: int, height: int):
    return _pixel_rays_cached(int(width), int(height))


@lru_cache(maxsize=4)
def _pixel_rays_cached(width, height):
    """Optical-frame ray of every raw fisheye pixel, scaled to unit optical depth."""
    from harness import markerless_box as _mb
    k = _mb.scaled_camera_matrix(width, height)
    d = np.asarray(_mb.CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    ys, xs = np.mgrid[0:height, 0:width]
    pixels = np.column_stack((xs.ravel(), ys.ravel())).astype(np.float64).reshape(-1, 1, 2)
    norm = cv2.fisheye.undistortPoints(pixels, k, d).reshape(-1, 2)
    rays = np.column_stack((norm, np.ones(len(norm))))
    rays.setflags(write=False)
    return rays


def _grip_frame(pose):
    """Grip site and tool yaw in the robot base frame, from issued pulses only."""
    grip = np.asarray(_arm.forward_grip(pose), np.float64)
    yaw = math.radians(_arm.tool_pose(pose).yaw_left_deg)
    return grip, yaw


def _ray_box(origin, rays, centre, half, rot):
    """Entry/exit depth of every ray through an oriented box (slab test)."""
    o = (origin-centre) @ rot                       # box-local ray origin
    dirs = rays @ rot
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1./dirs
        t1 = (-np.asarray(half)-o)*inv
        t2 = (np.asarray(half)-o)*inv
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    return tmin, tmax


def _ray_cylinder(origin, rays, centre, radius, half_height):
    """Entry/exit depth of every ray through an upright cylinder (base-frame z axis)."""
    o = origin-centre
    a = rays[:, 0]**2+rays[:, 1]**2
    b = 2*(o[0]*rays[:, 0]+o[1]*rays[:, 1])
    c = o[0]**2+o[1]**2-radius**2
    disc = b*b-4*a*c
    with np.errstate(divide='ignore', invalid='ignore'):
        root = np.sqrt(np.where(disc > 0, disc, np.nan))
        s1, s2 = (-b-root)/(2*a), (-b+root)/(2*a)
        z1 = (-half_height-o[2])/rays[:, 2]
        z2 = (half_height-o[2])/rays[:, 2]
    zmin, zmax = np.minimum(z1, z2), np.maximum(z1, z2)
    tmin = np.fmax(s1, zmin)
    tmax = np.fmin(s2, zmax)
    return tmin, tmax


def _rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def held_silhouette(kind: str, servo_pose: Mapping[int | str, int | float], size: tuple[int, int], *,
                    grasp_pose: Mapping[int | str, int | float] | None = None,
                    offset_m: tuple[float, float] = (0., 0.),
                    near_clip_m: float = RENDER_NEAR_CLIP_M) -> tuple[np.ndarray, dict[str, Any]]:
    """Where an item of ``kind`` held at my grip must appear in my own wrist frame.

    Forward kinematics of the issued pulses gives the grip site and the tool yaw;
    the item is placed by the catalogue hold convention, shifted by ``offset_m``
    (along the tool heading, in height); every raw fisheye pixel ray is intersected
    with its parts. A pixel belongs to the silhouette when the ray's first surface
    is at least ``near_clip_m`` in front of the lens. With a ``grasp_pose`` (a
    physical hold, rigid in the gripper) the geometry is the one at the grasp,
    because camera and item then move together.
    """
    geometry_pose = _v1._pose(grasp_pose if grasp_pose is not None else servo_pose)
    key = tuple(sorted((int(k), int(v)) for k, v in geometry_pose.items() if int(k) in (3, 4, 5, 6)))
    mask, info = _silhouette_cached(kind, key, int(size[0]), int(size[1]),
                                    (round(float(offset_m[0]), 5), round(float(offset_m[1]), 5)),
                                    round(float(near_clip_m), 5))
    info = dict(info, geometry_from='grasp_pose' if grasp_pose is not None else 'issued_pose')
    return mask.copy(), info


@lru_cache(maxsize=256)
def _silhouette_cached(kind, pose_key, width, height, offset, near_clip_m):
    pose = dict(pose_key)
    origin, axes = _arm.camera_extrinsics(pose)
    origin = np.asarray(origin, np.float64)
    axes = np.asarray(axes, np.float64)             # rows: optical x, y, z in the base frame
    rays = _pixel_rays(width, height) @ axes         # base-frame direction per unit optical depth
    grip, tool_yaw = _grip_frame(pose)
    heading = np.asarray((math.cos(tool_yaw), math.sin(tool_yaw), 0.))
    grip = grip+offset[0]*heading+np.asarray((0., 0., offset[1]))
    parts, grip_obj, approach = held_parts(kind)
    obj_yaw = tool_yaw-approach
    rot = _rot_z(obj_yaw)
    obj_origin = grip-rot @ np.asarray(grip_obj)
    first = np.full(len(rays), np.inf)
    for shape, centre, dims, part_yaw in parts:
        c = obj_origin+rot @ np.asarray(centre)
        if shape == 'box':
            tmin, tmax = _ray_box(origin, rays, c, dims, rot @ _rot_z(part_yaw))
        else:
            tmin, tmax = _ray_cylinder(origin, rays, c, dims[0], dims[1])
        hit = np.isfinite(tmin) & np.isfinite(tmax) & (tmax >= tmin) & (tmin >= near_clip_m)
        first = np.where(hit, np.minimum(first, tmin), first)
    mask = np.isfinite(first).reshape(height, width)
    mask.setflags(write=False)
    depth = first[np.isfinite(first)]
    info = {'kind': kind, 'pixels': int(mask.sum()),
            'depth_m': [round(float(depth.min()), 4), round(float(depth.max()), 4)] if depth.size else None,
            'grip_base_m': [round(float(v), 4) for v in grip], 'item_yaw_deg': round(math.degrees(obj_yaw), 2),
            'offset_m': list(offset), 'near_clip_m': near_clip_m}
    return mask, info


def held_band(kind: str, servo_pose: Mapping[int | str, int | float], size: tuple[int, int], *,
              grasp_pose: Mapping[int | str, int | float] | None = None) -> tuple[np.ndarray, np.ndarray,
                                                                                     dict[str, Any]]:
    """Nominal at-grip silhouette and its hold-tolerance band (union over offsets)."""
    nominal, info = held_silhouette(kind, servo_pose, size, grasp_pose=grasp_pose)
    band = nominal.copy()
    for offset in HOLD_TOLERANCE_M[1:]:
        band |= held_silhouette(kind, servo_pose, size, grasp_pose=grasp_pose, offset_m=offset)[0]
    return nominal, band, info


# --------------------------------------------------------------------------- shared

def _answer(judgment, answer, confidence, reason, **extra):
    if answer not in ANSWERS:
        raise ValueError(f'answer must be one of {ANSWERS}')
    row = {'schema': SCHEMA, 'profile': PROFILE, 'judgment': judgment, 'answer': answer,
           'confidence': round(float(confidence), 3), 'reason': reason, 'provenance': PROVENANCE}
    row.update(extra)
    return row


def _prepare(image, servo_pose):
    frame = _v1._frame(image)
    pose = _v1._pose(servo_pose)
    height, width = frame.shape[:2]
    usable = _v1._usable_region(width, height)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    reference = _v1._frame_reference(hsv)
    return frame, pose, hsv, reference, usable


def _kind_mask(hsv, kind, reference):
    if kind in _v2.TEAM_MASK:
        raw = _v2.team_mask(hsv, kind, reference)
    else:
        raw = _v1.hue_mask(hsv, kind, reference=reference)
        if kind in HELD_HUE_WINDOW:
            low, high = HELD_HUE_WINDOW[kind]
            raw = raw & (((hsv[..., 0] >= low) & (hsv[..., 0] <= high)).astype(np.uint8)*255)
    return cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)


def _band_colour(mask, nominal, band):
    """Colour inside the band (share of the nominal silhouette) and containment.

    Containment is the share of the colour blobs touching the band that lies
    inside it: a held item's colour ends at its silhouette, while floor paint or a
    distant object of the same colour runs on past it.
    """
    nominal_px = int(nominal.sum())
    inside = mask & band
    area = int(inside.sum())
    if not area:
        return 0., 0., 0
    _, labels = cv2.connectedComponents(mask.astype(np.uint8))
    touching = np.unique(labels[inside])
    touching = touching[touching > 0]
    blob_px = int(np.isin(labels, touching).sum())
    return area/max(nominal_px, 1), area/max(blob_px, 1), area


def _dark_mask(hsv, reference, usable):
    return (hsv[..., 2] <= DARK_MAX_VALUE) & usable


# --------------------------------------------------------------------------- held item at my grip

def judge_held_item(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                    candidate_kinds: Sequence[str] = SOLO_KINDS,
                    grasp_pose: Mapping[int | str, int | float] | None = None) -> dict[str, Any]:
    """Is the item at my own grip the ordered kind? (any posture)

    * ``yes``      the ordered kind's colour lies in its predicted at-grip band,
                   covers enough of the silhouette, and does not run on past the
                   band (so it is not floor paint or an object behind the grip).
    * ``no`` / other kind   another kind fills *its* silhouette the same way and
                   the ordered kind's band shows none of its colour.
    * ``no`` / ``expected_kind_absent``   the ordered kind's silhouette is in the
                   frame, the band is lit and carries no cargo colour at all.
    * ``unknown``  the ordered kind's silhouette is not (or barely) in the rendered
                   frame in this posture - the v2 W7 case - or the evidence is mixed.
    """
    if expected_kind not in _v1.APPEARANCE:
        raise ValueError(f'unknown cargo kind: {expected_kind}')
    frame, pose, hsv, reference, usable = _prepare(image, servo_pose)
    size = (frame.shape[1], frame.shape[0])
    kinds = [k for k in candidate_kinds if k in _v1.APPEARANCE]
    if expected_kind not in kinds:
        kinds.append(expected_kind)
    per, bands, masks, nominals = {}, {}, {}, {}
    for kind in kinds:
        nominal, band, info = held_band(kind, pose, size, grasp_pose=grasp_pose)
        nominal &= usable
        band &= usable
        mask = _kind_mask(hsv, kind, reference) & usable
        fill, containment, _ = _band_colour(mask, nominal, band)
        bands[kind], masks[kind], nominals[kind] = band, mask, nominal
        per[kind] = {'silhouette_px': int(nominal.sum()), 'band_px': int(band.sum()), 'fill': round(fill, 4),
                     'containment': round(containment, 4), 'depth_m': info['depth_m']}
    band_e = bands[expected_kind]
    nominal_px = per[expected_kind]['silhouette_px']
    band_px = max(int(band_e.sum()), 1)
    in_band = {k: float((masks[k] & band_e).sum())/band_px for k in kinds}
    common = {'expected_kind': expected_kind, 'per_kind': per,
              'colour_in_expected_band': {k: round(v, 4) for k, v in in_band.items()},
              'grasp_pose_given': grasp_pose is not None,
              'posture_pwm': {str(k): int(v) for k, v in pose.items()}}
    if nominal_px < HELD_MIN_SILHOUETTE_PX:
        return _answer('held_item_at_grip', 'unknown', CONFIDENCE['held_unknown'],
                       'EXPECTED_KIND_NOT_FRAMED_AT_GRIP', observed='not_observed', **common)
    row = per[expected_kind]
    rival = max((in_band[k] for k in kinds if k != expected_kind), default=0.)
    if (row['fill'] >= HELD_FILL_MIN and row['containment'] >= HELD_CONTAINMENT_MIN
            and in_band[expected_kind] >= HELD_MARGIN*max(rival, 1e-6)):
        return _answer('held_item_at_grip', 'yes', CONFIDENCE['held_kind_matched'],
                       'EXPECTED_KIND_IN_AT_GRIP_BAND', observed=expected_kind, **common)
    near = cv2.dilate(band_e.astype(np.uint8), np.ones((HELD_EMPTY_DILATE_PX, HELD_EMPTY_DILATE_PX), np.uint8)
                      ).astype(bool) & usable
    near_px = max(int(near.sum()), 1)
    own_near = float((masks[expected_kind] & near).sum())/near_px
    common['expected_colour_near_band'] = round(own_near, 4)
    others = sorted((k for k in kinds if k != expected_kind and per[k]['silhouette_px'] >= HELD_MIN_SILHOUETTE_PX
                     and per[k]['fill'] >= HELD_OTHER_FILL_MIN
                     and per[k]['containment'] >= HELD_CONTAINMENT_MIN),
                    key=lambda k: -per[k]['fill'])
    if others and own_near <= HELD_EMPTY_MAX:
        return _answer('held_item_at_grip', 'no', CONFIDENCE['held_kind_conflict'],
                       'OTHER_KIND_IN_ITS_AT_GRIP_BAND', observed=others[0], **common)
    any_near = max(float((masks[k] & near).sum())/near_px for k in kinds)
    common['any_colour_near_band'] = round(any_near, 4)
    if own_near <= HELD_EMPTY_MAX and any_near <= HELD_EMPTY_MAX:
        band_value = float(np.percentile(hsv[..., 2][band_e], _v1.FRAME_PERCENTILE))
        common['band_value_p75'] = round(band_value, 1)
        if band_value < max(_v1.ABS_MIN_VALUE, HELD_LIT_MIN*reference[1]):
            return _answer('held_item_at_grip', 'unknown', CONFIDENCE['held_unknown'],
                           'AT_GRIP_BAND_TOO_DARK', observed='not_observed', **common)
        return _answer('held_item_at_grip', 'no', CONFIDENCE['held_absent'],
                       'EXPECTED_KIND_ABSENT_FROM_AT_GRIP_BAND', observed='expected_kind_absent', **common)
    return _answer('held_item_at_grip', 'unknown', CONFIDENCE['held_unknown'], 'AT_GRIP_BAND_AMBIGUOUS',
                   observed='not_observed', **common)


# --------------------------------------------------------------------------- carried team item

def judge_team_cargo_at_grip(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                             grasp_pose: Mapping[int | str, int | float] | None = None,
                             candidate_kinds: Sequence[str] = TEAM_KINDS) -> dict[str, Any]:
    """Is the team item at my grip (while carrying) the ordered kind?

    Colour first: a team kind whose colour lies in its own at-grip band and covers
    enough of its silhouette. Only when *no* team colour is in the grip bands at
    all (an unlit item) the shape: the occupied (dark) pixels in the union of the
    team bands are compared with each kind's nominal silhouette, and the kind that
    matches best by a margin is reported with the weaker shape confidence. When
    the ordered kind's band is lit floor and no team band is occupied, nothing is
    at the grip.
    """
    if expected_kind not in _v2.TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame, pose, hsv, reference, usable = _prepare(image, servo_pose)
    size = (frame.shape[1], frame.shape[0])
    kinds = [k for k in candidate_kinds if k in _v2.TEAM_MASK]
    if expected_kind not in kinds:
        kinds.append(expected_kind)
    dark = _dark_mask(hsv, reference, usable)
    colours = {k: _kind_mask(hsv, k, reference) & usable for k in kinds}
    any_colour = np.zeros_like(dark)
    for mask in colours.values():
        any_colour |= mask
    nominals, bands, per = {}, {}, {}
    union = np.zeros_like(dark)
    for kind in kinds:
        nominal, band, info = held_band(kind, pose, size, grasp_pose=grasp_pose)
        nominal &= usable
        band &= usable
        nominals[kind], bands[kind] = nominal, band
        union |= band
        fill, containment, _ = _band_colour(colours[kind], nominal, band)
        per[kind] = {'silhouette_px': int(nominal.sum()), 'band_px': int(band.sum()),
                     'colour_fill': round(fill, 4), 'containment': round(containment, 4),
                     'depth_m': info['depth_m']}
    union_px = max(int(union.sum()), 1)
    colour_in_union = float((any_colour & union).sum())/union_px
    dark_u = dark & union
    for kind in kinds:
        sil = nominals[kind]
        inter = int((dark_u & sil).sum())
        per[kind]['shape_iou'] = round(inter/max(int((dark_u | sil).sum()), 1), 4)
    common = {'expected_kind': expected_kind, 'per_kind': per, 'colour_in_union': round(colour_in_union, 4),
              'grasp_pose_given': grasp_pose is not None,
              'posture_pwm': {str(k): int(v) for k, v in pose.items()}}
    if per[expected_kind]['silhouette_px'] < TEAM_MIN_SILHOUETTE_PX:
        return _answer('team_cargo_at_grip', 'unknown', CONFIDENCE['team_unknown'],
                       'EXPECTED_KIND_NOT_FRAMED_AT_GRIP', observed='not_observed', **common)
    coloured = sorted((k for k in kinds if per[k]['silhouette_px'] >= TEAM_MIN_SILHOUETTE_PX
                       and per[k]['colour_fill'] >= TEAM_COLOUR_FILL_MIN
                       and per[k]['containment'] >= TEAM_CONTAINMENT_MIN),
                      key=lambda k: -per[k]['colour_fill'])
    if coloured:
        best = coloured[0]
        if best == expected_kind:
            return _answer('team_cargo_at_grip', 'yes', CONFIDENCE['team_colour_matched'],
                           'EXPECTED_KIND_COLOUR_IN_AT_GRIP_BAND', observed=best, cue='colour', **common)
        return _answer('team_cargo_at_grip', 'no', CONFIDENCE['team_colour_conflict'],
                       'OTHER_KIND_COLOUR_IN_AT_GRIP_BAND', observed=best, cue='colour', **common)
    if colour_in_union <= TEAM_SHAPE_COLOUR_MAX:
        ranked = sorted(kinds, key=lambda k: -per[k]['shape_iou'])
        best = ranked[0]
        runner = per[ranked[1]]['shape_iou'] if len(ranked) > 1 else 0.
        common['shape_margin'] = round(per[best]['shape_iou']/max(runner, 1e-6), 3)
        if (per[best]['shape_iou'] >= SHAPE_IOU_MIN
                and per[best]['shape_iou'] >= SHAPE_MARGIN*max(runner, 1e-6)):
            if best == expected_kind:
                return _answer('team_cargo_at_grip', 'yes', CONFIDENCE['team_shape_matched'],
                               'EXPECTED_KIND_SHAPE_AT_GRIP', observed=best, cue='shape', **common)
            return _answer('team_cargo_at_grip', 'no', CONFIDENCE['team_shape_conflict'],
                           'OTHER_KIND_SHAPE_AT_GRIP', observed=best, cue='shape', **common)
    band_e = bands[expected_kind]
    lit_share = float((band_e & ~dark & ~any_colour).sum())/max(int(band_e.sum()), 1)
    occupied_union = float(((dark | any_colour) & union).sum())/union_px
    common['expected_band_lit_share'] = round(lit_share, 4)
    common['union_occupied_share'] = round(occupied_union, 4)
    if lit_share >= TEAM_EMPTY_LIT_SHARE and occupied_union <= TEAM_EMPTY_OCCUPIED_MAX:
        return _answer('team_cargo_at_grip', 'no', CONFIDENCE['team_nothing_at_grip'],
                       'NOTHING_AT_GRIP_BANDS_ARE_LIT_FLOOR', observed='empty', cue='shape', **common)
    return _answer('team_cargo_at_grip', 'unknown', CONFIDENCE['team_unknown'], 'AT_GRIP_EVIDENCE_AMBIGUOUS',
                   observed='not_observed', **common)


# --------------------------------------------------------------------------- my handle is here

def _plane_point(pixel, pose, z, size):
    frame = np.zeros((size[1], size[0], 3), np.uint8)
    origin, axes, k, d, _ = _v1._optics(frame, pose)
    norm = cv2.fisheye.undistortPoints(np.asarray(pixel, np.float64).reshape(1, 1, 2), k, d).reshape(2)
    direction = np.asarray((norm[0], norm[1], 1.)) @ axes
    if not np.all(np.isfinite(direction)) or direction[2] >= -1e-3:
        return None
    t = (z-origin[2])/direction[2]
    return origin+t*direction if t > 0 else None


def judge_team_cargo_handle(image, servo_pose: Mapping[int | str, int | float], *, expected_kind: str,
                            reach_band_m: tuple[float, float] = _v2.HANDLE_REACH_BAND_M,
                            lateral_m: float = _v2.HANDLE_LATERAL_M) -> dict[str, Any]:
    """"My handle is here", with the handle contact on the handle plane.

    Detection is v2's (dark component touching the kind's colour); only the
    range of each handle changes: its contact pixel is intersected with the plane
    ``HANDLE_CONTACT_PLANE_Z_M`` instead of the floor. The yes/no/unknown rules are
    v2's.
    """
    base = _v2.judge_team_cargo_handle(image, servo_pose, expected_kind=expected_kind,
                                       reach_band_m=reach_band_m, lateral_m=lateral_m)
    base = {**base, 'schema': SCHEMA, 'profile': PROFILE, 'v2_answer': base['answer'],
            'v2_reason': base['reason'], 'contact_plane_z_m': HANDLE_CONTACT_PLANE_Z_M}
    handles = base.get('handles') or []
    if not handles:
        return base
    frame = _v1._frame(image)
    pose = _v1._pose(servo_pose)
    size = (frame.shape[1], frame.shape[0])
    corrected = []
    for row in handles:
        point = _plane_point(row['contact_px'], pose, HANDLE_CONTACT_PLANE_Z_M, size)
        if point is None:
            continue
        x, y = float(point[0]), float(point[1])
        inside = reach_band_m[0] <= x <= reach_band_m[1] and abs(y) <= lateral_m
        corrected.append({**row, 'estimated_base_m': [round(x, 4), round(y, 4)],
                          'floor_projection_base_m': row['estimated_base_m'],
                          'range_m': round(math.hypot(x, y), 4), 'in_reach': bool(inside)})
    base['handles'] = corrected
    if not corrected:
        return {**base, 'answer': 'unknown', 'confidence': _v2.CONFIDENCE['handle_unknown'],
                'reason': 'NO_HANDLE_FEATURE_ON_THE_ITEM', 'observed': 'not_observed'}
    in_reach = [row for row in corrected if row['in_reach']]
    if in_reach:
        nearest = min(in_reach, key=lambda row: abs(row['estimated_base_m'][1]))
        return {**base, 'answer': 'yes', 'confidence': _v2.CONFIDENCE['handle_in_reach'],
                'reason': 'HANDLE_IN_REACH_BAND', 'observed': 'handle_here', 'nearest': nearest}
    nearest = min(corrected, key=lambda row: row['range_m'])
    return {**base, 'answer': 'no', 'confidence': _v2.CONFIDENCE['handle_out_of_reach'],
            'reason': 'HANDLES_FOUND_ALL_OUTSIDE_REACH', 'observed': 'handle_elsewhere', 'nearest': nearest}


# --------------------------------------------------------------------------- grasp stage

def judge_team_cargo_grasp_stage(image, servo_pose: Mapping[int | str, int | float], *,
                                 expected_kind: str) -> dict[str, Any]:
    """Kind and "my handle is here" from one frame in the grasp-stage look posture.

    The kind is v2's judgment, unchanged; the handle is v2's detection with the
    v3 handle contact plane. Both come from one frame taken in
    ``GRASP_LOOK_POSTURE``, which is what v3 changes. The frame must belong
    to that posture (issued pulses within v1's tolerance), otherwise both answers
    are ``unknown``.
    """
    if expected_kind not in _v2.TEAM_MASK:
        raise ValueError(f'not a team cargo kind: {expected_kind}')
    frame = _v1._frame(image)
    pose = _v1._pose(servo_pose)
    off = {s: abs(pose[s]-v) for s, v in GRASP_LOOK_POSTURE.items() if s in pose and s != 6}
    posture_ok = all(v <= _v1.POSTURE_TOLERANCE_PWM for v in off.values())
    common = {'expected_kind': expected_kind, 'posture_offset_pwm': off, 'posture_ok': posture_ok,
              'posture_pwm': {str(k): int(v) for k, v in GRASP_LOOK_POSTURE.items()}}
    if not posture_ok:
        unknown = {'answer': 'unknown', 'confidence': 0., 'reason': 'NOT_IN_GRASP_LOOK_POSTURE',
                   'observed': 'not_observed'}
        return _answer('team_cargo_grasp_stage', 'unknown', 0., 'NOT_IN_GRASP_LOOK_POSTURE',
                       observed='not_observed', identity=dict(unknown), handle=dict(unknown), **common)
    identity = _v2.judge_team_cargo_identity(frame, pose, expected_kind=expected_kind)
    handle = judge_team_cargo_handle(frame, pose, expected_kind=expected_kind)
    return _answer('team_cargo_grasp_stage', identity['answer'], identity['confidence'], identity['reason'],
                   observed=identity.get('observed'), identity=identity, handle=handle, **common)


__all__ = ['SCHEMA', 'PROFILE', 'ANSWERS', 'JUDGMENTS', 'PROVENANCE', 'TEAM_KINDS', 'SOLO_KINDS',
           'RENDER_NEAR_CLIP_M', 'HOLD_TOLERANCE_M', 'GRASP_LOOK_POSTURE', 'held_parts', 'held_silhouette',
           'held_band',
           'judge_held_item', 'judge_team_cargo_at_grip', 'judge_team_cargo_handle',
           'judge_team_cargo_grasp_stage', 'HANDLE_CONTACT_PLANE_Z_M', 'HELD_HUE_WINDOW']
