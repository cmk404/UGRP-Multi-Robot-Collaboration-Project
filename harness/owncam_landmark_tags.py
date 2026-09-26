"""INTERIM landmark provider: AprilTag detections re-labelled as static-map landmark observations.

Status (user decision 2026-09-26): tags are being removed from the study environment
(zone-entrance tags now; wall and door-post tags later, replaced by vision recognition of
doors, door posts and wall corners on branch ``kiro/zone-vision-loc``). This provider
exists only so that the memory can be run today on the tagged maps; every result obtained
with it is labelled "interim, tag provider". Nothing in ``harness.owncam_memory`` depends
on it, and tags are never a requirement: without this provider the memory keeps its box
and floor memories and plans full looks.

What it does

* ``observe``: takes the tag detections the own-camera pose source already computed for
  this frame (``harness.wall_tags.TagDetector``; no second detection) and returns one
  landmark observation per tag, labelled with the static-map catalogue landmark the tag is
  mounted next to (a door post, a wall corner, or else the wall face), its bearing and
  range from the tag's PnP solution, the provider feature id (``tag:<id>``) and every
  catalogue landmark the tag stands for (``supports``).
* ``support``: for planning rows of the static-map catalogue (``ViewModel.visible_landmarks``)
  the probability that a tag next to that landmark is detected from the given view: the
  memory_v1 tag visibility model (projected side, obliquity, wall occlusion, held-box band;
  table fitted on the own inputs of the M1 test episodes s102/s105), aggregated per
  landmark within ``SUPPORT_RADIUS_M``.
* ``noise``: the own-camera PF's tag measurement noise (fixed calibration), so the look
  planner's information matches the filter that actually uses the tags.

Nothing here imports the simulator.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from harness.owncam_landmarks import LandmarkProvider
from harness.owncam_memory import camera_in_base
from harness.wall_tags import tag_world_frame

SCHEMA = 'ugrp.owncam_landmark_tags.v1'
INTERIM_LABEL = 'interim, tag provider'
SUPPORT_RADIUS_M = .30
FACE_NORMAL_DOT = .9
# Planning visibility (memory_v1 values). The margin may be slightly negative: tags touching
# the render edge are still detected (M1 test own frames).
PLAN_MIN_SIDE_PX, PLAN_MAX_RANGE_M, PLAN_MAX_OBLIQUE_DEG, PLAN_MARGIN_PX = 10., 3.5, 70., -2.
PLAN_LOADED_ROW_LIMIT_PX = 172.      # loaded detections reach row 169 at the frame sides
# Detection probability by projected side and obliquity (M1 test s102/s105 own frames,
# predicted vs detected, settled LOOK_P20 frames).
DETECT_P_BY_SIDE = ((12., .03), (14., .40), (17., .72), (22., .97), (1e9, .95))
DETECT_P_BY_OBLIQUE = ((30., 1.), (50., .85), (60., .50), (1e9, .15))
# Strict expectation (the "expected view is missing" trigger): near, large, well inside the frame.
EXPECT_MIN_SIDE_PX, EXPECT_MAX_RANGE_M, EXPECT_MAX_OBLIQUE_DEG, EXPECT_MARGIN_PX = 20., 1.8, 55., 18.
EXPECT_LOADED_ROW_LIMIT_PX = 132.
CONFIG = {k: v for k, v in dict(globals()).items() if k.isupper() and isinstance(v, (int, float, tuple, str))}


def tag_detect_prob(side_px: float, oblique_deg: float) -> float:
    p = next(v for lim, v in DETECT_P_BY_SIDE if side_px < lim)
    return p*next(v for lim, v in DETECT_P_BY_OBLIQUE if oblique_deg < lim)


class TagLandmarkProvider(LandmarkProvider):
    name = 'tags_interim'
    interim = True
    face_points_identified = True        # each tag is an identified point on its wall face

    def __init__(self, static_map: Mapping, params: Mapping):
        tags = (static_map.get('landmarks') or {}).get('tags') or []
        self.tag_ids = np.array([int(t['id']) for t in tags], int)
        self.tag_centre = np.array([t['center_m'] for t in tags], float).reshape(-1, 3)
        corners, normals = [], []
        for t in tags:
            c_w, rot = tag_world_frame(t)
            h = float(t['size_m'])/2
            corners.append(c_w + np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ rot.T)
            normals.append(rot[:, 2])
        self.tag_corners = np.array(corners, float).reshape(-1, 4, 3)
        self.tag_normal = np.array(normals, float).reshape(-1, 3)
        self._index = {int(t): i for i, t in enumerate(self.tag_ids)}
        mp = dict(params['measurement'])
        self.meas = {False: mp, True: {**mp, **(params.get('measurement_loaded') or {})}}

    # ------------------------------------------------------------ catalogue link
    def bind(self, view, catalogue) -> None:
        super().bind(view, catalogue)
        plan = catalogue.plan_points()
        self._plan_xy = plan['xy']
        self._plan_ids = plan['ids']
        self._plan_types = plan['types']
        self._plan_dirs = self._row_dirs(range(len(plan['ids'])))
        # tag x planning point: the tag stands for this landmark sample
        self._link = self._links(self._plan_xy, self._plan_dirs, self._plan_types)
        self.supports, self.anchor = [], []
        for i in range(len(self.tag_ids)):
            rows = np.flatnonzero(self._link[i])
            ids = sorted({self._plan_ids[r] for r in rows})
            self.supports.append(tuple(ids))
            points = [r for r in rows if self._plan_types[r] != 'wall_face']
            pick = points or list(rows)
            if pick:
                d = np.hypot(*(self._plan_xy[pick] - self.tag_centre[i, :2]).T)
                r = pick[int(np.argmin(d))]
                self.anchor.append((self._plan_ids[r], self._plan_types[r]))
            else:
                self.anchor.append((f'unanchored:tag:{int(self.tag_ids[i])}', 'unanchored'))

    def _row_dirs(self, rows) -> list:
        """Per planning row: acceptable tag normals (unit xy vectors, or None = any facing tag)."""
        out = []
        for r in rows:
            lm = self.catalogue.by_id[self._plan_ids[r]]
            if lm.type == 'wall_face':
                out.append(('face', np.asarray(lm.normal_xy, float)))
            elif lm.type == 'door_gap':
                out.append(('any', None))
            else:
                out.append(('quad', [np.array(q, float)/math.sqrt(2.) for q in lm.free_quadrants]))
        return out

    def _links(self, xy, dirs, types) -> np.ndarray:
        n_xy = self.tag_normal[:, :2]
        d = np.hypot(self.tag_centre[:, None, 0] - xy[None, :, 0], self.tag_centre[:, None, 1] - xy[None, :, 1])
        link = d <= SUPPORT_RADIUS_M
        for r, (kind, v) in enumerate(dirs):
            if kind == 'face':
                link[:, r] &= (n_xy @ v) >= FACE_NORMAL_DOT
            elif kind == 'quad':
                link[:, r] &= np.max(np.stack([n_xy @ q for q in v]), axis=0) > 0.
        return link

    # ------------------------------------------------------------ visibility (memory_v1 model)
    def visible_tags(self, pose: Sequence[float], servo: Mapping[int, int], loaded: bool, *, strict: bool = False,
                     sigma: tuple[float, float] | None = None) -> list[dict]:
        if not len(self.tag_ids):
            return []
        view = self.view
        pose = np.asarray(pose, float)
        corners_c = view.to_camera(self.tag_corners.reshape(-1, 3), pose, servo, loaded)[0].reshape(-1, 4, 3)
        centre_c = corners_c.mean(axis=1)
        px, ideal, ok = view.project(corners_c)
        rng = np.linalg.norm(centre_c, axis=1)
        n_b = view._to_base(self.tag_normal, np.array([[0., 0., pose[2]]]))[0]
        _, r_bc = camera_in_base(servo)
        n_c = n_b @ r_bc
        cos_oblique = -np.sum(n_c*centre_c, axis=1)/np.maximum(rng, 1e-9)
        side = np.linalg.norm(px - np.roll(px, 1, axis=1), axis=2).mean(axis=1)
        if strict:
            sx, syaw = sigma if sigma is not None else (0., 0.)
            mx = EXPECT_MARGIN_PX + view.f*(2*syaw + 2*sx/np.maximum(rng, .3))
            my = np.full(len(rng), EXPECT_MARGIN_PX)
            row_limit = EXPECT_LOADED_ROW_LIMIT_PX
            min_side, max_range, max_obl = EXPECT_MIN_SIDE_PX, EXPECT_MAX_RANGE_M, EXPECT_MAX_OBLIQUE_DEG
        else:
            mx = my = np.full(len(rng), PLAN_MARGIN_PX)
            row_limit = PLAN_LOADED_ROW_LIMIT_PX
            min_side, max_range, max_obl = PLAN_MIN_SIDE_PX, PLAN_MAX_RANGE_M, PLAN_MAX_OBLIQUE_DEG
        inside = np.all(ok, axis=1) & np.all(view.in_view(px, ideal, loaded, mx[:, None], my[:, None], row_limit), axis=1)
        keep = inside & (side >= min_side) & (rng <= max_range) & (cos_oblique >= math.cos(math.radians(max_obl)))
        idx = np.flatnonzero(keep)
        if len(idx):
            cam = view.camera_world(pose, servo)
            towards = cam[None, :] - self.tag_centre[idx]
            towards /= np.maximum(np.linalg.norm(towards, axis=1, keepdims=True), 1e-9)
            idx = idx[~view.occluded(cam, self.tag_centre[idx] + .01*towards)]
        out = []
        for i in idx:
            obl = math.degrees(math.acos(min(1., float(cos_oblique[i]))))
            out.append({'id': int(self.tag_ids[i]), 'index': int(i), 'side_px': round(float(side[i]), 1),
                        'range_m': round(float(rng[i]), 3), 'oblique_deg': round(obl, 1),
                        'p_detect': 1. if strict else round(tag_detect_prob(float(side[i]), obl), 3)})
        return out

    # ------------------------------------------------------------ provider interface
    def support(self, rows, pose, servo, loaded, *, strict=False, sigma=None):
        """Per planning row: P(some tag next to that landmark is detected); expected tag count; P(any tag)."""
        vis = self.visible_tags(pose, servo, loaded, strict=strict, sigma=sigma)
        w = np.zeros(len(rows))
        if not vis or not len(rows):
            return {'w': w, 'n_expected': 0., 'p_any': 0., 'features': []}
        tag_idx = np.array([v['index'] for v in vis])
        p = np.array([v['p_detect'] for v in vis])
        plan_rows = np.array([r['plan_index'] for r in rows], int)
        link = self._link[np.ix_(tag_idx, plan_rows)]                       # (visible tags, rows)
        w = 1. - np.prod(np.where(link, 1. - p[:, None], 1.), axis=0)
        return {'w': w, 'n_expected': float(p.sum()), 'p_any': float(1. - np.prod(1. - p)),
                'features': [f"tag:{v['id']}" for v in vis]}

    def observe(self, *, image, servo, loaded, tag_detections: Sequence[Mapping] = (), **inputs) -> list[dict]:
        out = []
        for d in tag_detections:
            i = self._index.get(int(d['id']))
            if i is None or not d.get('solutions'):
                continue
            t = np.asarray(d['solutions'][0]['t_ct'], float)
            lid, ltype = self.anchor[i]
            out.append({'landmark_id': lid, 'landmark_type': ltype, 'feature_id': f"tag:{int(d['id'])}",
                        'supports': self.supports[i], 'azimuth_rad': float(math.atan2(t[0], t[2])),
                        'elevation_rad': float(math.atan2(t[1], t[2])), 'range_m': float(np.linalg.norm(t))})
        return out

    def noise(self, loaded: bool) -> dict:
        mp = self.meas[bool(loaded)]
        return {'azimuth_std_rad': max(mp['azimuth_std_rad'], mp.get('azimuth_floor_rad', 0.)),
                'elevation_std_rad': max(mp['elevation_std_rad'], mp.get('elevation_floor_rad', 0.)),
                'range_log_std': max(mp['range_log_std'], mp.get('range_floor', 0.)),
                'temper': float(mp.get('tag_temper', 0.))}

    def describe(self) -> dict:
        anchored = {}
        for lid, ltype in self.anchor:
            anchored[ltype] = anchored.get(ltype, 0) + 1
        return {'name': self.name, 'interim': True, 'label': INTERIM_LABEL, 'schema': SCHEMA, 'tags': len(self.tag_ids),
                'tags_anchored_by_type': anchored, 'support_radius_m': SUPPORT_RADIUS_M, 'config': dict(CONFIG)}
