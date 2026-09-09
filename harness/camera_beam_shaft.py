"""Robust image-only shaft geometry for elongated connected masks."""

from __future__ import annotations

import numpy as np


def _principal_axis(points: np.ndarray) -> np.ndarray | None:
    covariance = np.cov(points, rowvar=False)
    if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
        return None
    values, vectors = np.linalg.eigh(covariance)
    if values[-1] <= 0 or values[-1] < 4 * max(values[0], 1e-9):
        return None
    axis = vectors[:, -1]
    if axis[0] < 0 or (abs(axis[0]) < 1e-12 and axis[1] < 0):
        axis = -axis
    return axis


def _slices(points: np.ndarray, origin: np.ndarray, axis: np.ndarray):
    normal = np.asarray((-axis[1], axis[0]))
    axial = (points - origin) @ axis
    lateral = (points - origin) @ normal
    indices = np.floor(axial).astype(int)
    rows = []
    for index in range(int(indices.min()), int(indices.max()) + 1):
        values = lateral[indices == index]
        if len(values) >= 2:
            rows.append((index, float(values.min()), float(values.max()),
                         float(np.median(values)), len(values)))
    return axial, lateral, rows


def robust_shaft_geometry(component_mask: np.ndarray) -> dict | None:
    """Fit the longest stable-width shaft in one connected component.

    The provisional PCA axis is refined from the central 60 percent of the
    component.  End support must remain close to that core's width and center,
    which excludes narrow bridges and attached perpendicular/diagonal lobes.
    """
    if component_mask.ndim != 2:
        return None
    ys, xs = np.nonzero(component_mask)
    if len(xs) < 100:
        return None
    points = np.column_stack((xs, ys)).astype(float)
    origin = np.median(points, axis=0)
    axis = _principal_axis(points)
    if axis is None:
        return None

    for _ in range(2):
        axial, lateral, rows = _slices(points, origin, axis)
        if len(rows) < 10:
            return None
        low, high = np.quantile(axial, (.2, .8))
        central = [row for row in rows if low <= row[0] + .5 <= high]
        if len(central) < 5:
            return None
        widths = np.asarray([row[2] - row[1] + 1.0 for row in central])
        shaft_width = float(np.median(widths))
        shaft_center = float(np.median([row[3] for row in central]))
        if not np.isfinite(shaft_width) or shaft_width < 2:
            return None
        core = ((axial >= low) & (axial <= high)
                & (np.abs(lateral - shaft_center) <= .65 * shaft_width))
        if np.count_nonzero(core) < 30:
            return None
        refined = _principal_axis(points[core])
        if refined is None:
            return None
        if np.dot(refined, axis) < 0:
            refined = -refined
        axis = refined
        origin = np.median(points[core], axis=0)

    axial, lateral, rows = _slices(points, origin, axis)
    low, high = np.quantile(axial, (.2, .8))
    central = [row for row in rows if low <= row[0] + .5 <= high]
    widths = np.asarray([row[2] - row[1] + 1.0 for row in central])
    shaft_width = float(np.median(widths))
    shaft_center = float(np.median([row[3] for row in central]))
    if np.median(np.abs(widths - shaft_width)) > .2 * shaft_width:
        return None

    compatible = [row for row in rows
                  if .70 * shaft_width <= row[2] - row[1] + 1.0 <= 1.30 * shaft_width
                  and abs(row[3] - shaft_center) <= .25 * shaft_width
                  and row[4] >= .55 * shaft_width]
    runs: list[list[tuple]] = []
    for row in compatible:
        if not runs or row[0] != runs[-1][-1][0] + 1:
            runs.append([])
        runs[-1].append(row)
    if not runs:
        return None
    run = max(runs, key=lambda item: (len(item), -abs(item[0][0] + item[-1][0])))
    # A wide attachment can spoil a slice's total width while the shaft still
    # crosses it. Extend only where a shaft-width center corridor stays dense;
    # a narrow connector or diagonal tail cannot satisfy this support gate.
    indices = np.floor(axial).astype(int)
    corridor = np.abs(lateral - shaft_center) <= .55 * shaft_width
    supported = {index for index in range(int(indices.min()), int(indices.max()) + 1)
                 if np.count_nonzero((indices == index) & corridor) >= .75 * shaft_width}
    start_index, end_index = run[0][0], run[-1][0]
    while start_index - 1 in supported:
        start_index -= 1
    while end_index + 1 in supported:
        end_index += 1
    length = float(end_index - start_index + 1)
    if length < max(12.0, 3.0 * shaft_width):
        return None

    normal = np.asarray((-axis[1], axis[0]))
    start_t, end_t = start_index, end_index + 1.0
    center_s = float(np.median([row[3] for row in run]))
    start = origin + axis * start_t + normal * center_s
    end = origin + axis * end_t + normal * center_s
    center = (start + end) / 2.0
    half_width = shaft_width / 2.0
    corners = np.asarray((start - normal * half_width,
                          start + normal * half_width,
                          end + normal * half_width,
                          end - normal * half_width))
    return {
        "center_px": center,
        "endpoints_px": np.asarray((start, end)),
        "corners_px": corners,
        "width_px": shaft_width,
        "length_px": length,
    }
