"""Wall AprilTag detection and per-tag PnP on the robot's own wrist fisheye RGB.

Robot inputs only: the raw wrist RGB, the measured camera intrinsics/fisheye
distortion (``sim.masterpi_camera_profile``, fixed calibration) and the robot's
own ISSUED servo pulses. Camera->base extrinsics come from
``harness.visual_arm.camera_extrinsics(commanded_pose)``: forward kinematics of
the commanded PWM, never measured joints. The static map gives each tag's
surveyed pose. Nothing here imports or reads the simulator.

Frames: base = robot floor frame (+x forward, +y left, +z up, origin on the
floor under the chassis origin). Camera = OpenCV optical (+x right, +y down,
+z forward). Tag = OpenCV IPPE_SQUARE (+x right, +y up, +z out of the tag
towards the viewer); tag corners are TL, TR, BR, BL as detected.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import cv2
import numpy as np

from harness.visual_arm import camera_extrinsics
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
EXTRINSICS_FUNCTION = 'harness.visual_arm.camera_extrinsics(commanded servo PWM)'


def camera_in_base(commanded_pose: Mapping[int | str, int | float]) -> tuple[np.ndarray, np.ndarray]:
    """(origin (3,), R_bc (3x3, columns = optical axes in base)) from issued PWM."""
    origin, axes = camera_extrinsics(commanded_pose)
    return np.asarray(origin, float), np.asarray(axes, float).T


def tag_world_frame(tag: Mapping) -> tuple[np.ndarray, np.ndarray]:
    """(centre (3,), R_wt) of a map landmark."""
    nx, ny = tag['normal_xy']
    rot = np.array([[-ny, 0., nx], [nx, 0., ny], [0., 1., 0.]], float)
    return np.asarray(tag['center_m'], float), rot


class TagDetector:
    """AprilTag 36h11 detector + fisheye undistortion + IPPE_SQUARE PnP."""

    def __init__(self, width: int = 640, height: int = 480, *, sizes: Mapping[int, float] | None = None,
                 default_size_m: float = .072, min_side_px: float = 8.):
        self.width, self.height = int(width), int(height)
        self.K = scaled_camera_matrix(self.width, self.height)
        self.D = np.asarray(CAMERA_FISHEYE_D, float).reshape(4, 1)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY), params)
        self.sizes = dict(sizes or {})
        self.default_size_m = float(default_size_m)
        self.min_side_px = float(min_side_px)

    @classmethod
    def for_map(cls, static_map: Mapping, width: int = 640, height: int = 480):
        tags = static_map['landmarks']['tags']
        return cls(width, height, sizes={int(t['id']): float(t['size_m']) for t in tags})

    def undistort(self, corners_px: np.ndarray) -> np.ndarray:
        """Raw fisheye pixels -> normalized pinhole coordinates (x/z, y/z)."""
        pts = np.asarray(corners_px, np.float64).reshape(-1, 1, 2)
        return cv2.fisheye.undistortPoints(pts, self.K, self.D).reshape(-1, 2)

    def detect(self, image: np.ndarray) -> list[dict]:
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        else:
            gray = image
        if gray.shape != (self.height, self.width):
            raise ValueError('image size differs from the camera calibration size')
        corners, ids, _ = self.detector.detectMarkers(gray)
        out = []
        if ids is None:
            return out
        for quad, tag_id in zip(corners, ids.ravel()):
            tag_id = int(tag_id)
            if self.sizes and tag_id not in self.sizes:
                continue  # not a map landmark: ignore
            px = quad.reshape(4, 2).astype(float)
            side = float(np.mean(np.linalg.norm(px - np.roll(px, 1, axis=0), axis=1)))
            if side < self.min_side_px:
                continue
            norm = self.undistort(px)
            size = self.sizes.get(tag_id, self.default_size_m)
            solutions = self.solve_pnp(norm, size)
            if not solutions:
                continue
            out.append({'id': tag_id, 'corners_px': px.tolist(), 'side_px': side,
                        'corners_norm': norm.tolist(), 'size_m': size, 'solutions': solutions})
        return out

    def solve_pnp(self, norm: np.ndarray, size: float) -> list[dict]:
        """IPPE_SQUARE: both mirror solutions, lowest reprojection error first.

        Works on normalized coordinates scaled by the focal length (K = diag(f,
        f, 1)); reprojection errors are in pixels. No LM refinement: for these
        small tags the two poses are nearly ambiguous and LM can drift to the
        mirror minimum (tested: it moved the translation 0.28% vs 0.08%).
        """
        h = size/2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
        f = float(self.K[0, 0])
        kf = np.diag([f, f, 1.])
        img = (np.asarray(norm, np.float64)*f).reshape(-1, 1, 2)
        try:
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, kf, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return []
        sols = []
        for i in range(int(n)):
            rvec, tvec = rvecs[i], tvecs[i]
            proj, _ = cv2.projectPoints(obj, rvec, tvec, kf, None)
            err = float(np.sqrt(np.mean(np.sum((proj - img).reshape(-1, 2)**2, axis=1))))
            rot, _ = cv2.Rodrigues(rvec)
            t = tvec.reshape(3)
            if t[2] <= 0:
                continue
            sols.append({'R_ct': rot.tolist(), 't_ct': t.tolist(), 'reproj_px': err})
        sols.sort(key=lambda s: s['reproj_px'])
        return sols


def implied_base_poses(detection: Mapping, tag: Mapping, commanded_pose: Mapping,
                       *, max_height_m: float = .06, max_tilt_deg: float = 20.) -> list[dict]:
    """Planar robot poses implied by one tag's PnP solutions (for initialization).

    Solutions whose implied base is off the floor or tilted are rejected: the
    robot stands on the floor, which removes most IPPE mirror ambiguities.
    """
    o_bc, r_bc = camera_in_base(commanded_pose)
    c_w, r_wt = tag_world_frame(tag)
    out = []
    for sol in detection['solutions']:
        r_ct, t_ct = np.asarray(sol['R_ct']), np.asarray(sol['t_ct'])
        r_wc = r_wt @ r_ct.T
        o_wc = c_w - r_wc @ t_ct
        r_wb = r_wc @ r_bc.T
        o_wb = o_wc - r_wb @ o_bc
        tilt = math.degrees(math.acos(max(-1., min(1., r_wb[2, 2]))))
        if abs(o_wb[2]) > max_height_m or tilt > max_tilt_deg:
            continue
        out.append({'x': float(o_wb[0]), 'y': float(o_wb[1]),
                    'yaw': math.atan2(r_wb[1, 0], r_wb[0, 0]), 'range_m': float(np.linalg.norm(t_ct)),
                    'reproj_px': sol['reproj_px'], 'height_m': float(o_wb[2]), 'tilt_deg': tilt})
    return out


def predicted_tag_in_camera(poses: np.ndarray, tag: Mapping, commanded_pose: Mapping,
                            ) -> tuple[np.ndarray, np.ndarray]:
    """Tag centre and normal in the camera frame for N planar poses (x, y, yaw).

    Returns (p_c (N,3), n_c (N,3)).
    """
    o_bc, r_bc = camera_in_base(commanded_pose)
    c_w, r_wt = tag_world_frame(tag)
    poses = np.asarray(poses, float).reshape(-1, 3)
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    # world -> base for each pose
    d = c_w[None, :2] - poses[:, :2]
    p_b = np.stack((c*d[:, 0] + s*d[:, 1], -s*d[:, 0] + c*d[:, 1], np.full(len(poses), c_w[2])), axis=1)
    n_w = r_wt[:, 2]
    n_b = np.stack((c*n_w[0] + s*n_w[1], -s*n_w[0] + c*n_w[1], np.full(len(poses), n_w[2])), axis=1)
    p_c = (p_b - o_bc[None, :]) @ r_bc
    n_c = n_b @ r_bc
    return p_c, n_c


def observed_tag_in_camera(detection: Mapping) -> tuple[np.ndarray, np.ndarray]:
    """(t (3,), normals (S,3)): centre of the lowest-reprojection solution and
    every solution's face normal (IPPE mirror ambiguity is resolved later)."""
    sols = detection['solutions']
    t = np.asarray(min(sols, key=lambda s: s.get('reproj_px', 0.))['t_ct'], float)
    normals = np.array([np.asarray(s['R_ct'])[:, 2] for s in sols])
    return t, normals


def angle_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a/np.linalg.norm(a, axis=-1, keepdims=True)
    b = b/np.linalg.norm(b, axis=-1, keepdims=True)
    return np.arccos(np.clip(np.sum(a*b, axis=-1), -1., 1.))


def tags_by_id(static_map: Mapping) -> dict[int, Mapping]:
    return {int(t['id']): t for t in static_map['landmarks']['tags']}


def servo_pose_from(values: Mapping[int | str, int | float] | Sequence) -> dict[int, int]:
    return {int(k): int(v) for k, v in dict(values).items()}
