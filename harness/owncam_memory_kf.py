"""Static 2-D object tracks for the own-camera memory: Kalman predict/update and gated association.

Adapted (copied and trimmed, not imported) from two MIT-licensed sources:

* filterpy 1.4.5, ``filterpy/kalman/kalman_filter.py`` (file sha256
  67be531e655dca69a6a6a36d788a5b50801b272d1740350ebdc72fae736018d3): the module
  functions ``predict`` (x = Fx + Bu, P = alpha^2 F P F' + Q) and ``update``
  (residual, system uncertainty S = HPH' + R, gain K = PH'S^-1, Joseph-form
  covariance P = (I-KH)P(I-KH)' + KRK' and the measurement log-likelihood).
  filterpy is not installed in the project environment (it needs scipy); here
  only numpy is used and ``filterpy.stats.logpdf`` is a closed-form Gaussian.
* PythonRobotics commit b2020cd, ``SLAM/EKFSLAM/ekf_slam.py`` (file sha256
  8014f978a7b044213c46670334b98cbb5ef18935ceef4ebda0ddf6a266196575):
  ``search_correspond_landmark_id`` (nearest landmark by Mahalanobis distance,
  a new landmark when every distance exceeds a threshold) and
  ``calc_landmark_position`` (landmark = robot pose + rotated relative
  observation). The robot pose comes from the own-camera particle filter, so
  every track is its own 2-D filter instead of one augmented SLAM state, the
  pose covariance is propagated into the measurement noise, and the
  association is one-to-one per frame with an ambiguity refusal (the
  ``harness/multi_object_tracking.py`` rule: never silently pick between two
  plausible tracks).

filterpy license (MIT):
  Copyright (c) 2015 Roger R. Labbe Jr
PythonRobotics license (MIT):
  Copyright (c) 2016 - now Atsushi Sakai and other contributors:
  https://github.com/AtsushiSakai/PythonRobotics/contributors
For both: Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including without
limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom
the Software is furnished to do so, subject to the following conditions: The
above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS
IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

SCHEMA = 'ugrp.owncam_memory_kf.v1'
# chi-square 99% quantile with 2 degrees of freedom: the association gate on the
# squared Mahalanobis distance (PythonRobotics uses a fixed M_DIST_TH).
CHI2_2DOF_99 = 9.21
# A detection whose two best tracks are both inside the gate and closer than
# this in squared Mahalanobis distance is ambiguous and updates nothing.
AMBIGUITY_MARGIN = 2.0
SOURCES = {
    'filterpy': {'url': 'https://github.com/rlabbe/filterpy', 'version': '1.4.5', 'license': 'MIT',
                 'file': 'filterpy/kalman/kalman_filter.py',
                 'file_sha256': '67be531e655dca69a6a6a36d788a5b50801b272d1740350ebdc72fae736018d3',
                 'reused': 'predict, update (Joseph form), log-likelihood'},
    'PythonRobotics': {'url': 'https://github.com/AtsushiSakai/PythonRobotics', 'commit': 'b2020cd',
                       'license': 'MIT', 'file': 'SLAM/EKFSLAM/ekf_slam.py',
                       'file_sha256': '8014f978a7b044213c46670334b98cbb5ef18935ceef4ebda0ddf6a266196575',
                       'reused': 'search_correspond_landmark_id, calc_landmark_position'},
}


def kf_predict(x: np.ndarray, P: np.ndarray, Q: np.ndarray, F: np.ndarray | None = None,
               alpha: float = 1.) -> tuple[np.ndarray, np.ndarray]:
    """filterpy ``predict`` without control input: x = Fx, P = alpha^2 F P F' + Q."""
    F = np.eye(len(x)) if F is None else np.asarray(F, float)
    x = F @ x
    P = (alpha*alpha)*(F @ P @ F.T) + Q
    return x, P


def gaussian_logpdf(y: np.ndarray, S: np.ndarray) -> float:
    """log N(y; 0, S) (replaces ``filterpy.stats.logpdf``)."""
    k = len(y)
    sign, logdet = np.linalg.slogdet(S)
    if sign <= 0:
        return -math.inf
    return float(-.5*(k*math.log(2*math.pi) + logdet + y @ np.linalg.solve(S, y)))


def kf_update(x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray,
              H: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, dict]:
    """filterpy ``update``: residual, S, K, Joseph-form P; returns (x, P, info)."""
    H = np.eye(len(x)) if H is None else np.asarray(H, float)
    y = np.asarray(z, float) - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    I_KH = np.eye(len(x)) - K @ H
    P = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x, P, {'residual': y, 'S': S, 'loglik': gaussian_logpdf(y, S),
                  'mahalanobis2': float(y @ np.linalg.solve(S, y))}


def mahalanobis2(x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray) -> float:
    y = np.asarray(z, float) - x
    return float(y @ np.linalg.solve(P + R, y))


def observation_to_map(pose_xyyaw: Sequence[float], pose_cov: np.ndarray, base_xy: Sequence[float],
                       sigma_det_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Map-frame point and covariance of a detection given in the robot base frame.

    ``calc_landmark_position`` of PythonRobotics (range/bearing there, base x/y
    here) plus first-order propagation of the pose covariance:
    R = sigma_det^2 I + J Sigma_pose J', J = d(point)/d(x, y, yaw).
    """
    x, y, yaw = (float(v) for v in pose_xyyaw)
    bx, by = (float(v) for v in base_xy)
    c, s = math.cos(yaw), math.sin(yaw)
    z = np.array([x + c*bx - s*by, y + s*bx + c*by])
    J = np.array([[1., 0., -s*bx - c*by], [0., 1., c*bx - s*by]])
    cov = np.asarray(pose_cov, float).reshape(3, 3)
    R = (sigma_det_m**2)*np.eye(2) + J @ cov @ J.T
    return z, .5*(R + R.T)


def associate(track_states: Sequence[tuple[np.ndarray, np.ndarray]], measurements: Sequence[tuple[np.ndarray, np.ndarray]],
              *, gate: float = CHI2_2DOF_99, ambiguity_margin: float = AMBIGUITY_MARGIN) -> list[dict]:
    """One-to-one gated association (greedy by squared Mahalanobis distance).

    Returns one row per measurement: ``{'measurement': i, 'track': j | None,
    'decision': 'update' | 'new' | 'ambiguous', 'd2': ...}``. ``new`` follows
    the PythonRobotics rule (every distance beyond the gate); ``ambiguous``
    (two tracks inside the gate within ``ambiguity_margin``) updates nothing.
    """
    d2 = np.full((len(measurements), len(track_states)), np.inf)
    for i, (z, R) in enumerate(measurements):
        for j, (x, P) in enumerate(track_states):
            d2[i, j] = mahalanobis2(x, P, z, R)
    rows: dict[int, dict] = {}
    for i in range(len(measurements)):
        inside = sorted((float(d2[i, j]), j) for j in range(len(track_states)) if d2[i, j] <= gate)
        if len(inside) >= 2 and inside[1][0] - inside[0][0] < ambiguity_margin:
            rows[i] = {'measurement': i, 'track': None, 'decision': 'ambiguous',
                       'd2': [round(v, 3) for v, _ in inside[:2]]}
    used_tracks: set[int] = set()
    pairs = sorted((float(d2[i, j]), i, j) for i in range(len(measurements)) for j in range(len(track_states))
                   if d2[i, j] <= gate and i not in rows)
    for value, i, j in pairs:
        if i in rows or j in used_tracks:
            continue
        rows[i] = {'measurement': i, 'track': j, 'decision': 'update', 'd2': round(value, 3)}
        used_tracks.add(j)
    for i in range(len(measurements)):
        if i not in rows:
            best = float(d2[i].min()) if len(track_states) else None
            # inside the gate of an already-claimed track only: not a new object either
            claimed = best is not None and best <= gate
            rows[i] = {'measurement': i, 'track': None, 'decision': 'ambiguous' if claimed else 'new',
                       'd2': None if best is None else round(best, 3)}
    return [rows[i] for i in range(len(measurements))]
