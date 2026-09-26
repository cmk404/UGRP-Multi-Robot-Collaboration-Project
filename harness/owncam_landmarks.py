"""Static-map landmarks for the own-camera memory, and the landmark-provider interface.

User decision 2026-09-26: fiducial tags are being removed from the study environment
(zone-entrance tags now; wall and door-post tags later, replaced by vision recognition of
doors, door posts and wall corners on branch ``kiro/zone-vision-loc``). The memory
(``harness.owncam_memory``) therefore never depends on tags:

* the landmark catalogue below is derived from the static map GEOMETRY only: the wall
  rectangles, the door passages and physical door posts (``landmarks.door_posts`` when
  a map has them). It never reads ``landmarks.tags``;
* the memory stores generic landmark observations (landmark id and type from this
  catalogue, bearing / range, the robot's own pose estimate and sigma at that time,
  frame id, provider) and plans "where to look" from this catalogue's geometry;
* observations come from a ``LandmarkProvider``. Today the only provider with a working
  detector is the INTERIM tag provider (``harness.owncam_landmark_tags``): it re-labels
  the pose source's tag detections as observations of the catalogue landmarks the tags
  are mounted next to. Every result obtained with it is labelled "interim, tag provider".
  ``GeometricLandmarkProvider`` is the tag-free provider; its detector hook is where the
  vision detector of ``kiro/zone-vision-loc`` plugs in. Without a detector it observes
  nothing and supports nothing, and the memory falls back to full looks.

Landmark types (all from walls / passages / door posts)

  wall_corner  concave corner where two wall faces meet; vertical edge seen from one quadrant
  wall_end     convex free wall end away from any door; seen from three quadrants
  door_post    convex wall end / door-post edge bounding a door passage
  door_gap     the door opening: the floor point at the passage centre
  wall_face    one free face of a wall (its floor line), sampled every FACE_SAMPLE_M for planning

Corners come from a quadrant-occupancy test at every intersection of the wall faces'
lines (exact on the rectilinear zone maps). Rejected alternative: rasterising the map and
``cv2.findContours`` + ``cv2.approxPolyDP`` (1 cm snapping error and no face identity).

Nothing here imports the simulator.
"""
from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass, field

import numpy as np

SCHEMA = 'ugrp.owncam_landmarks.v1'
EPS_M = .002                     # quadrant probe offset (smaller than any wall / post gap in the maps)
DOOR_POST_RADIUS_M = .12         # a convex corner this close to a door passage bounds that door
FACE_SAMPLE_M = .25              # planning samples along a free wall face
FACE_MIN_M = .10                 # shorter free face runs are not landmarks (wall ends are corners)
FACE_STEP_M = .01
POINT_TYPES = ('wall_corner', 'wall_end', 'door_post', 'door_gap')
TYPES = POINT_TYPES + ('wall_face',)
QUADRANTS = ((1, 1), (-1, 1), (-1, -1), (1, -1))


def _qname(q) -> str:
    return ('+x' if q[0] > 0 else '-x') + ('+y' if q[1] > 0 else '-y')


@dataclass(frozen=True)
class Landmark:
    id: str
    type: str
    xy: tuple[float, float]                              # floor point (point types) / face midpoint
    height_m: float
    source: tuple[str, ...]                              # static-map element ids
    free_quadrants: tuple[tuple[int, int], ...] = ()     # corners: quadrants of free space around the point
    normal_xy: tuple[float, float] | None = None         # wall_face: out of the wall; door_gap: crossing axis
    segment: tuple[tuple[float, float], tuple[float, float]] | None = None

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class LandmarkObservation:
    """One provider observation of one catalogue landmark in one own frame (memory record)."""
    t: float
    frame_id: int
    landmark_id: str
    landmark_type: str
    provider: str
    interim: bool
    azimuth_rad: float
    elevation_rad: float
    range_m: float | None
    pose_xyyaw: tuple[float, float, float]
    std_xy_m: float
    std_yaw_rad: float
    posture: str
    settled: bool
    feature_id: str | None = None                        # provider feature (e.g. an interim tag id)
    supports: tuple[str, ...] = field(default=())        # every catalogue landmark this feature stands for

    def as_dict(self) -> dict:
        d = asdict(self)
        for k in ('azimuth_rad', 'elevation_rad', 'std_xy_m', 'std_yaw_rad'):
            d[k] = round(float(d[k]), 5)
        d['range_m'] = None if d['range_m'] is None else round(float(d['range_m']), 4)
        d['pose_xyyaw'] = [round(float(v), 4) for v in d['pose_xyyaw']]
        d['supports'] = list(d['supports'])
        return d


def _rects(static_map: Mapping) -> list[tuple[str, float, float, float, float, float]]:
    out = []
    for o in static_map.get('obstacles', []):
        if o.get('kind') == 'wall':
            (cx, cy), (hx, hy) = o['center_m'], o['half_extents_m']
            out.append((str(o['id']), float(cx), float(cy), float(hx), float(hy), float(o.get('height_m', .10))))
    for p in (static_map.get('landmarks') or {}).get('door_posts', []) or []:
        (cx, cy), (hx, hy) = p['center_m'], p['half_extents_m']
        out.append((str(p['id']), float(cx), float(cy), float(hx), float(hy), float(p['height_m'])))
    return out


class LandmarkCatalogue:
    """Landmarks of one static map (geometry only; ``landmarks.tags`` is never read)."""

    def __init__(self, static_map: Mapping):
        self.map_id = str(static_map.get('map_id', ''))
        self.rects = _rects(static_map)
        x0, x1, y0, y1 = (float(v) for v in static_map['bounds_m'])
        self.bounds = (x0, x1, y0, y1)
        self.doors = [p for p in static_map.get('passages', []) if p.get('kind') == 'door']
        geometry = {'bounds_m': list(self.bounds), 'rects': [list(r) for r in self.rects],
                    'doors': [{k: p[k] for k in ('id', 'center_m', 'half_extents_m', 'axis')} for p in self.doors]}
        self.geometry_sha256 = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
        self.landmarks: list[Landmark] = self._corners() + self._gaps() + self._faces()
        self.by_id = {lm.id: lm for lm in self.landmarks}
        if len(self.by_id) != len(self.landmarks):
            raise ValueError('duplicate landmark ids')
        self.sha256 = hashlib.sha256(json.dumps([lm.as_dict() for lm in self.landmarks], sort_keys=True)
                                     .encode()).hexdigest()
        self._plan = self._plan_points()

    # ------------------------------------------------------------ geometry
    def occupied(self, x: float, y: float) -> bool:
        x0, x1, y0, y1 = self.bounds
        if not (x0 < x < x1 and y0 < y < y1):
            return True
        return any(abs(x - cx) <= hx and abs(y - cy) <= hy for _, cx, cy, hx, hy, _ in self.rects)

    def _on_boundary(self, x, y) -> list[tuple]:
        out = []
        for r in self.rects:
            _, cx, cy, hx, hy, _ = r
            inside = abs(x - cx) <= hx + 1e-9 and abs(y - cy) <= hy + 1e-9
            if inside and (abs(abs(x - cx) - hx) < 1e-9 or abs(abs(y - cy) - hy) < 1e-9):
                out.append(r)
        return out

    def _door_of(self, x, y):
        for p in self.doors:
            (cx, cy), (hx, hy) = p['center_m'], p['half_extents_m']
            dx, dy = max(abs(x - cx) - hx, 0.), max(abs(y - cy) - hy, 0.)
            if math.hypot(dx, dy) <= DOOR_POST_RADIUS_M:
                return p
        return None

    def _corners(self) -> list[Landmark]:
        xs = sorted({round(cx + s*hx, 6) for _, cx, _, hx, _, _ in self.rects for s in (-1, 1)})
        ys = sorted({round(cy + s*hy, 6) for _, _, cy, _, hy, _ in self.rects for s in (-1, 1)})
        out = []
        for x in xs:
            for y in ys:
                on = self._on_boundary(x, y)
                if not on:
                    continue
                free = tuple(q for q in QUADRANTS if not self.occupied(x + EPS_M*q[0], y + EPS_M*q[1]))
                if len(free) not in (1, 3):
                    continue
                src = tuple(sorted({r[0] for r in on}))
                height = max(r[5] for r in on)
                if len(free) == 1:
                    out.append(Landmark(f"corner:{'+'.join(src)}:{_qname(free[0])}", 'wall_corner', (x, y), height, src,
                                        free_quadrants=free))
                    continue
                blocked = next(q for q in QUADRANTS if q not in free)
                door = self._door_of(x, y)
                if door is None:
                    out.append(Landmark(f"end:{'+'.join(src)}:{_qname(blocked)}", 'wall_end', (x, y), height, src,
                                        free_quadrants=free))
                    continue
                (cx, cy) = door['center_m']
                cross, lat = (0, 1) if door.get('axis', 'x') == 'x' else (1, 0)
                side = ('+' if (x, y)[cross] > (cx, cy)[cross] else '-') + 'xy'[cross]
                end = 'hi' if (x, y)[lat] > (cx, cy)[lat] else 'lo'
                out.append(Landmark(f"{door['id']}:post_{end}:{side}", 'door_post', (x, y), height,
                                    src + (str(door['id']),), free_quadrants=free))
        return out

    def _gaps(self) -> list[Landmark]:
        out = []
        for p in self.doors:
            axis = (1., 0.) if p.get('axis', 'x') == 'x' else (0., 1.)
            out.append(Landmark(f"{p['id']}:gap", 'door_gap', tuple(float(v) for v in p['center_m']), 0.,
                                (str(p['id']),), normal_xy=axis))
        return out

    def _faces(self) -> list[Landmark]:
        out = []
        for wid, cx, cy, hx, hy, height in self.rects:
            for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
                n = (float(sign), 0.) if axis == 0 else (0., float(sign))
                if axis == 0:
                    fixed, lo, hi = cx + sign*hx, cy - hy, cy + hy
                else:
                    fixed, lo, hi = cy + sign*hy, cx - hx, cx + hx
                steps = np.arange(lo + FACE_STEP_M/2, hi, FACE_STEP_M)
                free = [not self.occupied(*((fixed + sign*EPS_M, s) if axis == 0 else (s, fixed + sign*EPS_M)))
                        for s in steps]
                runs, start = [], None
                for s, f in zip(steps, free):
                    if f and start is None:
                        start = s
                    if not f and start is not None:
                        runs.append((start, prev))
                        start = None
                    prev = s
                if start is not None:
                    runs.append((start, steps[-1]))
                k = 0
                for a, b in runs:
                    a, b, fixed = float(a - FACE_STEP_M/2), float(b + FACE_STEP_M/2), float(fixed)
                    if b - a < FACE_MIN_M:
                        continue
                    p0 = (fixed, a) if axis == 0 else (a, fixed)
                    p1 = (fixed, b) if axis == 0 else (b, fixed)
                    mid = ((p0[0] + p1[0])/2, (p0[1] + p1[1])/2)
                    name = ('+' if sign > 0 else '-') + 'xy'[axis]
                    out.append(Landmark(f'face:{wid}:{name}:{k}', 'wall_face', (round(mid[0], 6), round(mid[1], 6)),
                                        height, (wid,), normal_xy=n,
                                        segment=((round(p0[0], 6), round(p0[1], 6)), (round(p1[0], 6), round(p1[1], 6)))))
                    k += 1
        return out

    # ------------------------------------------------------------ planning points
    def _plan_points(self) -> dict:
        ids, types, xy, face_dir, idx, height = [], [], [], [], [], []
        for i, lm in enumerate(self.landmarks):
            if lm.type == 'wall_face':
                (ax, ay), (bx, by) = lm.segment
                length = math.hypot(bx - ax, by - ay)
                n = max(1, int(round(length/FACE_SAMPLE_M)))
                for k in range(n):
                    f = (k + .5)/n
                    ids.append(lm.id), types.append(lm.type), idx.append(i), height.append(lm.height_m)
                    xy.append((ax + f*(bx - ax), ay + f*(by - ay)))
                    face_dir.append(((bx - ax)/length, (by - ay)/length))
            else:
                ids.append(lm.id), types.append(lm.type), idx.append(i), height.append(lm.height_m)
                xy.append(lm.xy)
                face_dir.append((0., 0.))
        return {'ids': ids, 'types': types, 'index': np.array(idx, int), 'xy': np.array(xy, float).reshape(-1, 2),
                'face_dir': np.array(face_dir, float).reshape(-1, 2), 'height': np.array(height, float)}

    def plan_points(self) -> dict:
        return self._plan

    def seen_from(self, points_index: np.ndarray, cam_xy: Sequence[float]) -> np.ndarray:
        """Is the camera on a free side of each planning point (corner quadrants, face normal, door axis)?"""
        cam = np.asarray(cam_xy, float)
        out = np.zeros(len(points_index), bool)
        for k, i in enumerate(points_index):
            lm = self.landmarks[int(self._plan['index'][i])]
            d = cam - self._plan['xy'][i]
            if lm.type == 'wall_face':
                out[k] = float(np.dot(d, lm.normal_xy)) > .02
            elif lm.type == 'door_gap':
                out[k] = abs(float(np.dot(d, lm.normal_xy))) > .05
            else:
                out[k] = any(d[0]*q[0] > .02 and d[1]*q[1] > .02 for q in lm.free_quadrants)
        return out

    def describe(self) -> dict:
        counts = {t: sum(lm.type == t for lm in self.landmarks) for t in TYPES}
        return {'schema': SCHEMA, 'map_id': self.map_id, 'sha256': self.sha256, 'geometry_sha256': self.geometry_sha256,
                'counts': counts, 'plan_points': len(self._plan['ids']),
                'source': 'static map geometry: obstacles kind=wall, passages kind=door, landmarks.door_posts; '
                          'landmarks.tags is not read'}


class LandmarkProvider:
    """Interface between the memory and a landmark detector (see the module docstring).

    ``bind`` gives the provider the memory's camera model (``harness.owncam_memory.ViewModel``)
    and the catalogue. ``observe`` returns partial observation rows for one own frame:
    ``{'landmark_id', 'landmark_type', 'feature_id', 'supports', 'azimuth_rad',
    'elevation_rad', 'range_m'}``. ``support`` returns, for planning rows produced by
    ``ViewModel.visible_landmarks``, the probability that this provider observes each row
    (and the expected number of provider features, for the PF's tempering).
    """
    name = 'abstract'
    interim = False
    face_points_identified = False      # True if features along a wall face are individually identified

    def bind(self, view, catalogue: LandmarkCatalogue) -> None:
        self.view, self.catalogue = view, catalogue

    def observe(self, *, image, servo: Mapping[int, int], loaded: bool, **inputs) -> list[dict]:
        raise NotImplementedError

    def support(self, rows: Sequence[Mapping], pose, servo: Mapping[int, int], loaded: bool, *,
                strict: bool = False, sigma=None) -> dict:
        """``{'w': P(observed) per row, 'n_expected': expected features, 'p_any': P(any), 'features': ids}``."""
        raise NotImplementedError

    def noise(self, loaded: bool) -> dict:
        raise NotImplementedError

    def describe(self) -> dict:
        return {'name': self.name, 'interim': self.interim}


# Placeholder detection probabilities / bearing noise of a future vision detector. Not used
# by any run in this experiment; kiro/zone-vision-loc will report the measured values.
NOMINAL_P = {'door_post': .8, 'wall_corner': .7, 'wall_end': .7, 'door_gap': .8, 'wall_face': .6}
NOMINAL_NOISE = {'azimuth_std_rad': .01, 'elevation_std_rad': .02, 'temper': 0.}


class GeometricLandmarkProvider(LandmarkProvider):
    """Tag-free provider. ``detector(image, servo, catalogue, view) -> rows`` is the vision hook."""
    name = 'geometric'
    interim = False

    def __init__(self, detector: Callable | None = None, p_detect: Mapping[str, float] | None = None,
                 noise: Mapping[str, float] | None = None):
        self.detector = detector
        self.p = dict(NOMINAL_P if p_detect is None else p_detect)
        self._noise = dict(NOMINAL_NOISE if noise is None else noise)

    def observe(self, *, image, servo, loaded, **inputs):
        if self.detector is None:
            return []
        return list(self.detector(image, servo, self.catalogue, self.view))

    def support(self, rows, pose, servo, loaded, *, strict=False, sigma=None):
        if self.detector is None or not len(rows):
            return {'w': np.zeros(len(rows)), 'n_expected': 0., 'p_any': 0., 'features': []}
        w = np.array([1. if strict else self.p.get(r['type'], 0.) for r in rows], float)
        return {'w': w, 'n_expected': float(w.sum()), 'p_any': float(1. - np.prod(1. - np.minimum(w, 1.))),
                'features': [r['id'] for r in rows]}

    def noise(self, loaded):
        return dict(self._noise)

    def describe(self):
        return {'name': self.name, 'interim': False,
                'detector': None if self.detector is None else getattr(self.detector, '__name__', 'callable'),
                'status': 'no detector attached: observes nothing (vision detector pending, kiro/zone-vision-loc)'
                if self.detector is None else 'detector attached', 'p_detect': self.p, 'noise': self._noise}
