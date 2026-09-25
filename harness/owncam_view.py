"""Wrist-camera (robot_cam) view geometry for carry/look postures.

Pure functions over ISSUED servo PWM only (never measured joints, never a
simulator). They answer: with a box held in the gripper, which part of the
floor and of a vertical wall plane can the arm-tip fisheye see in a given
posture, and how large does a wall tag of a given size appear?

The SIM wrist image is a pinhole render remapped to the measured fisheye
(``sim.masterpi_camera_profile.raw_fisheye_remap``); raw pixels whose ideal
pinhole pixel falls outside the render are black. ``valid_pixel_mask`` repeats
that rule, so the metrics describe the SIM image. The physical lens sees more.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from functools import lru_cache

import cv2
import numpy as np

from harness.visual_arm import SAFE_PULSE_MAX, SAFE_PULSE_MIN, _ik_at_pitch, camera_extrinsics, tool_pose
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

WIDTH, HEIGHT = 640, 480
ROBOT_BASE_FLOOR_HEIGHT_CM = 3.25
_D = np.asarray(CAMERA_FISHEYE_D, np.float64).reshape(4, 1)


def posture(radius_m: float, height_m: float, pitch_deg: float, *, pan: int = 1500,
            grip: int = 1500) -> dict[int, int]:
    """Issued PWM for a grip-site radius/height (robot floor frame) and tool pitch.

    Unlike ``solve_grip_site_ik`` this is not restricted to the calibrated
    floor-grasp envelope: carry/look postures keep the box in the air.
    """
    for name, value in (('radius_m', radius_m), ('height_m', height_m), ('pitch_deg', pitch_deg)):
        if not math.isfinite(float(value)):
            raise ValueError(f'{name} must be finite')
    arm = _ik_at_pitch(float(radius_m) * 100., float(height_m) * 100. - ROBOT_BASE_FLOOR_HEIGHT_CM,
                       float(pitch_deg))
    if arm is None:
        raise ValueError('posture has no safe arm IK solution')
    if not SAFE_PULSE_MIN <= int(pan) <= SAFE_PULSE_MAX or not SAFE_PULSE_MIN <= int(grip) <= SAFE_PULSE_MAX:
        raise ValueError('pan/grip outside the physical PWM range')
    return {1: int(grip), **arm, 6: int(pan)}


@lru_cache(maxsize=4)
def _pixel_rays(step: int):
    """Normalised fisheye rays of a raw-pixel grid and the SIM validity mask."""
    k = scaled_camera_matrix(WIDTH, HEIGHT)
    ys, xs = np.mgrid[step // 2:HEIGHT:step, step // 2:WIDTH:step].astype(np.float64)
    raw = np.stack((xs, ys), axis=-1).reshape(-1, 1, 2)
    ideal = cv2.fisheye.undistortPoints(raw, k, _D, R=np.eye(3), P=k).reshape(-1, 2)
    valid = ((ideal[:, 0] >= 0) & (ideal[:, 0] <= WIDTH - 1) & (ideal[:, 1] >= 0) & (ideal[:, 1] <= HEIGHT - 1))
    normal = cv2.fisheye.undistortPoints(raw, k, _D).reshape(-1, 2)
    return xs.reshape(-1), ys.reshape(-1), normal, valid


def valid_pixel_mask(step: int = 1) -> np.ndarray:
    xs, ys, _, valid = _pixel_rays(step)
    return valid.reshape(len(range(step // 2, HEIGHT, step)), len(range(step // 2, WIDTH, step)))


def base_rays(pose: Mapping[int | str, int | float], step: int = 8):
    """Camera origin and unit base-frame rays for the sampled raw pixels."""
    xs, ys, normal, valid = _pixel_rays(step)
    origin, axes = camera_extrinsics(pose)
    origin = np.asarray(origin, float)
    axes = np.asarray(axes, float)
    rays = normal[:, :1] * axes[0] + normal[:, 1:2] * axes[1] + axes[2]
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return origin, rays, xs, ys, valid


def _unoccluded(xs, ys, valid, occlusion):
    keep = valid.copy()
    if occlusion is not None:
        occ = np.asarray(occlusion, bool)
        keep &= ~occ[ys.astype(int).clip(0, occ.shape[0] - 1), xs.astype(int).clip(0, occ.shape[1] - 1)]
    return keep


def view_metrics(pose: Mapping[int | str, int | float], *, occlusion: np.ndarray | None = None,
                 wall_distances_m: Sequence[float] = (.3, .6, 1., 1.5), step: int = 4) -> dict:
    """Visible floor range and wall bands for one posture.

    ``occlusion`` is an optional HxW bool mask (e.g. held-box pixels of a render).
    Distances on the floor are forward x in the robot base frame; wall planes
    are x = camera_x + d (d ahead of the lens), reported as visible z ranges on
    the centre line (|y| <= 0.05 m) and lateral y range at 0.05 m height.
    """
    origin, rays, xs, ys, valid = base_rays(pose, step)
    keep = _unoccluded(xs, ys, valid, occlusion)
    axis = np.asarray(camera_extrinsics(pose)[1][2], float)
    result = {'camera_height_m': round(float(origin[2]), 4),
              'camera_x_m': round(float(origin[0]), 4),
              'optical_axis_pitch_deg': round(math.degrees(math.asin(max(-1., min(1., axis[2])))), 2),
              'tool_pitch_deg': round(tool_pose(pose).pitch_deg, 2),
              'valid_fraction': round(float(valid.mean()), 4),
              'unoccluded_fraction_of_valid': round(float(keep.sum() / max(valid.sum(), 1)), 4)}
    r = rays[keep]
    elev = np.degrees(np.arcsin(r[:, 2]))
    azim = np.degrees(np.arctan2(r[:, 1], r[:, 0]))
    result['elevation_deg'] = [round(float(elev.min()), 2), round(float(elev.max()), 2)] if len(r) else None
    result['azimuth_deg'] = [round(float(azim.min()), 2), round(float(azim.max()), 2)] if len(r) else None
    result['sees_above_horizon'] = bool(len(r) and elev.max() > 0)
    down = r[:, 2] < -1e-6
    floor = {}
    if down.any():
        s = -origin[2] / r[down, 2]
        pts = origin + s[:, None] * r[down]
        ahead = pts[(np.abs(pts[:, 1]) <= .05) & (pts[:, 0] > 0)]
        floor['centre_line_x_m'] = ([round(float(ahead[:, 0].min()), 3), round(float(min(ahead[:, 0].max(), 9.99)), 3)]
                                    if len(ahead) else None)
        for d in (.5, 1.):
            band = pts[np.abs(pts[:, 0] - d) <= .03]
            floor[f'lateral_y_at_{d:.1f}m'] = ([round(float(band[:, 1].min()), 3), round(float(band[:, 1].max()), 3)]
                                              if len(band) else None)
    result['floor'] = floor
    walls = {}
    for d in wall_distances_m:
        plane_x = origin[0] + float(d)
        fwd = r[:, 0] > 1e-6
        s = (plane_x - origin[0]) / r[fwd, 0]
        pts = origin + s[:, None] * r[fwd]
        centre = pts[(np.abs(pts[:, 1]) <= .05) & (pts[:, 2] >= 0)]
        low = pts[(np.abs(pts[:, 2] - .05) <= .02)]
        walls[f'{d:.1f}'] = {
            'z_range_centre_m': ([round(float(centre[:, 2].min()), 3), round(float(centre[:, 2].max()), 3)]
                                 if len(centre) else None),
            'y_range_at_z0.05_m': ([round(float(low[:, 1].min()), 3), round(float(low[:, 1].max()), 3)]
                                   if len(low) else None)}
    result['wall_plane'] = walls
    return result


def project_base_points(pose: Mapping[int | str, int | float], points_base: np.ndarray) -> np.ndarray:
    """Raw fisheye pixels (NaN when behind the camera) of robot-frame points."""
    origin, axes = camera_extrinsics(pose)
    origin = np.asarray(origin, float)
    axes = np.asarray(axes, float)
    rel = np.asarray(points_base, float).reshape(-1, 3) - origin
    cam = rel @ axes.T
    out = np.full((len(cam), 2), np.nan)
    ahead = cam[:, 2] > 1e-6
    if ahead.any():
        norm = (cam[ahead, :2] / cam[ahead, 2:3]).reshape(-1, 1, 2)
        k = scaled_camera_matrix(WIDTH, HEIGHT)
        out[ahead] = cv2.fisheye.distortPoints(norm, k, _D).reshape(-1, 2)
    return out


def tag_pixel_side(pose: Mapping[int | str, int | float], *, distance_m: float, height_m: float,
                   size_m: float, lateral_m: float = 0.) -> dict:
    """Projected side length (px) of a wall tag facing the robot, d ahead of the lens.

    Reports whether all four corners fall on valid SIM pixels. Occlusion by the
    held box is not modelled here (see ``view_metrics``/renders).
    """
    origin, _ = camera_extrinsics(pose)
    x = float(origin[0]) + float(distance_m)
    h = float(size_m) / 2.
    corners = np.array([[x, lateral_m - h, height_m + h], [x, lateral_m + h, height_m + h],
                        [x, lateral_m + h, height_m - h], [x, lateral_m - h, height_m - h]])
    px = project_base_points(pose, corners)
    if np.isnan(px).any():
        return {'in_view': False, 'side_px': None}
    side = float(np.mean([np.linalg.norm(px[i] - px[(i + 1) % 4]) for i in range(4)]))
    k = scaled_camera_matrix(WIDTH, HEIGHT)
    inside = []
    for u, v in px:
        if not (0 <= u < WIDTH and 0 <= v < HEIGHT):
            inside.append(False)
            continue
        ideal = cv2.fisheye.undistortPoints(np.array([[[u, v]]], float), k, _D, R=np.eye(3), P=k).reshape(2)
        inside.append(bool(0 <= ideal[0] <= WIDTH - 1 and 0 <= ideal[1] <= HEIGHT - 1))
    return {'in_view': all(inside), 'side_px': round(side, 1), 'corners_px': px.round(1).tolist()}
