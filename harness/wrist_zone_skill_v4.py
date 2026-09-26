"""``wrist_zone_skill_v4``: v3 plus a self-occlusion check on N7's low-top hand-off.

Same input boundary as v1-v3 (one own ``robot_cam`` observation + one injected
``PoseEstimate`` per step; static map; order sheet). v1, v2 and v3 stay
byte-identical as recorded results of experiments/2026-09-25-zone-owncam-skill.

Recorded failure (v3 cohort 521-530, P/529; also once in P/523 and v2 519):
N7's carry check handed off ``TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`` three times
while the box was held (GT z 0.171 m), each grip check passed, and the third
hand-off hit the v3 cap. Root cause (replayed on the recorded frames):

* During carry N7 calls ``visual_box_surface.observe_known_box_top`` on every
  frame. Its top mask keeps value 152-220 ("the bright manufactured top") and
  it accepts a bright STRIP as a top, computing height only from the two long
  edges, which it assumes are the full 40 mm long dimension.
* At a few positions/headings (all on or next to zone C in the recorded runs)
  the zone light puts the HELD box face at value 147-153, straddling 152. A
  3 px band of the held face passes the mask (area 186 px >= the 180 px
  minimum); read as a 40 mm edge it lies "far away", i.e. at floor height
  (z -0.02..0.01 m), with confidence 0.93-0.98.
* The zone C paint is not in these frames at all; the held face fills the
  lower 54 % of the image. The alarm is a photometric strip of the held box,
  not a floor object.

v4 fix: on that hand-off only, test whether the reported top quad lies inside
the held-box silhouette of the current own frame (cyan-hue component touching
the image bottom, holes filled) AND that silhouette is still the held box
(area 0.8-1.25 of the carry anchor's). A floor object cannot be seen through
the held box, so such a quad is a self-occluded strip: carry resumes without
an arm probe and the rejection is logged. Otherwise (no held silhouette, e.g.
after a real drop, or a quad outside it) the v3 path runs unchanged: stop and
the N7 left/right/home attachment probe. N7's co-motion drop check, which runs
before the surface check on every carry frame, and the VISUAL_GRASP_DRIFT and
anchor-warning re-seat paths are unchanged.
"""
from __future__ import annotations

import base64
from typing import Any

import cv2
import numpy as np

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v2 as v2
from harness import wrist_zone_skill_v3 as v3
from harness.visual_box_skill import _pose

PROFILE = 'wrist_zone_skill_v4'
HELD_HUE_LO = (75, 70, 40)            # cyan hue band of the held box, any lit face
HELD_HUE_HI = (105, 255, 255)
HELD_AREA_RATIO = (.80, 1.25)         # N7 carry anchor area_ratio range
CORNER_MARGIN_PX = 5
BOTTOM_MARGIN_PX = 40


def _decode(image) -> np.ndarray:
    data = base64.b64decode(image) if isinstance(image, str) else bytes(image)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('INVALID_JPEG')
    return frame


def held_silhouette(frame: np.ndarray) -> np.ndarray | None:
    """Filled cyan-hue component touching the image bottom (the held box in carry_p30)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(HELD_HUE_LO), np.asarray(HELD_HUE_HI))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    height = mask.shape[0]
    # The rectified fisheye frame has a black margin, so "touching the bottom"
    # means reaching the last BOTTOM_MARGIN_PX rows.
    bottom = [i for i in range(1, count)
              if stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] >= height - BOTTOM_MARGIN_PX]
    if not bottom:
        return None
    label = max(bottom, key=lambda i: stats[i, cv2.CC_STAT_AREA])
    component = (labels == label).astype(np.uint8)
    contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(component)
    cv2.drawContours(filled, contours, -1, 1, thickness=cv2.FILLED)
    return filled.astype(bool)


def self_occlusion_check(surface: dict[str, Any] | None, image, anchor_image) -> dict[str, Any]:
    """Is N7's low 'top' quad inside the still-present held-box silhouette?"""
    corners = (surface or {}).get('pixel_corners')
    if not corners or len(corners) != 4:
        return {'self_occluded': False, 'reason': 'NO_TOP_QUAD'}
    frame = _decode(image)
    held = held_silhouette(frame)
    anchor = held_silhouette(_decode(anchor_image)) if anchor_image is not None else None
    if held is None or anchor is None or not anchor.any():
        return {'self_occluded': False, 'reason': 'NO_HELD_SILHOUETTE'}
    ratio = float(held.sum() / anchor.sum())
    if not HELD_AREA_RATIO[0] <= ratio <= HELD_AREA_RATIO[1]:
        return {'self_occluded': False, 'reason': 'HELD_SILHOUETTE_CHANGED', 'area_ratio_to_anchor': round(ratio, 3)}
    inner = cv2.erode(held.astype(np.uint8), np.ones((2 * CORNER_MARGIN_PX + 1,) * 2, np.uint8)).astype(bool)
    h, w = inner.shape
    inside = [bool(inner[min(h - 1, max(0, int(round(y)))), min(w - 1, max(0, int(round(x))))]) for x, y in corners]
    return {'self_occluded': all(inside), 'reason': 'QUAD_INSIDE_HELD_SILHOUETTE' if all(inside) else 'QUAD_OUTSIDE_HELD_SILHOUETTE',
            'corners_inside': inside, 'area_ratio_to_anchor': round(ratio, 3),
            'held_fraction': round(float(held.mean()), 3),
            'quad_area_px': (surface or {}).get('area_px'),
            'quad_top_height_m': (surface or {}).get('estimated_top_height_base_m'),
            'quad_edge_lengths_m': (surface or {}).get('measured_top_edge_lengths_m')}


class WristOnlyBoxSkillV4(v3.WristOnlyBoxSkillV3):
    def resume_carry_after_self_occluded_top(self) -> None:
        """Undo N7's low-top hand-off when the 'top' is a strip of the held box itself."""
        if not (self.phase == 'finished' and self.reason == v3.GRIP_CHECK_REASON and self.held):
            raise RuntimeError('resume needs the N7 low-top hand-off with a held box')
        self.phase, self.reason = 'carry', 'RUNNING'
        self._probe_results = []
        self._probe_origin_phase = None


class WristZoneDeliveryV4(v3.WristZoneDeliveryV3):
    """v3 delivery; N7 low-top hand-offs inside the held silhouette resume carry."""

    def __init__(self, order, **kwargs):
        super().__init__(order, **kwargs)
        self.self_occluded_rejections = []

    def _new_box(self):
        return WristOnlyBoxSkillV4(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    def _nav_preplace(self, obs, est):
        check = self.box.decide(obs)
        if check['kind'] == 'finish':
            if check['reason'] == v3.GRIP_CHECK_REASON and self.box.held:
                evidence = self_occlusion_check(self.box.last_surface, obs['image'], self.box._attachment_image)
                if evidence['self_occluded']:
                    self.self_occluded_rejections.append(
                        {'estimate': [round(est.x_m, 4), round(est.y_m, 4), round(est.yaw_rad, 4)], **evidence})
                    self._event('low_top_self_occluded', est, evidence=evidence)
                    self.box.resume_carry_after_self_occluded_top()
                    return self._nav_preplace_after_check(est)
                if len(self.grip_checks) < v3.MAX_GRIP_CHECKS:
                    self.grip_checks.append({'trigger': v3.GRIP_CHECK_REASON, 'occlusion_check': evidence,
                                             'estimate': [round(est.x_m, 4), round(est.y_m, 4)], 'result': None})
                    self._event('grip_check_start', est, check=len(self.grip_checks), occlusion_check=evidence)
                    self.phase = 'grip_check'
                    return self.box.begin_check_grip(obs)
            if check['reason'] == 'VISUAL_GRASP_DRIFT' and self.reseats < v2.MAX_RESEATS:
                return self._start_reseat(est, 'n7_drift_stop')
            return self._finish('CARRY_' + check['reason'])
        return self._nav_preplace_after_check(est)

    def summary(self):
        return {**super().summary(), 'self_occluded_rejections': len(self.self_occluded_rejections),
                'self_occluded_samples': self.self_occluded_rejections[:5]}
