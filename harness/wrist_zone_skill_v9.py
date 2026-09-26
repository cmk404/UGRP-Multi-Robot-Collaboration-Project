"""``wrist_zone_skill_v9``: v8 + two approach-stage fixes from the v8 cohort (583, 587). Nothing else.

v1-v8 stay byte-identical; ``WristZoneDeliveryV9`` is a drop-in for v6/v7/v8 (same public API).
Both fixes read only the robot's own RGB fit and its own published commands.

(a) v8 583 (``SIM_LIMIT``): at 0.29 m N7's fit alternated with the wrist between the
    measured top face (``pixel_centroid`` = top-rectangle centroid) and the whole floor cuboid
    (silhouette centroid). The estimated 3-D centre agreed (0.295 m in both), but the
    vertical error that v2's approach chases flipped -31 <-> +39 px, so servo 3 dithered
    545 <-> 561 for ~600 steps; v2's forced-forward escape is disabled in face approach.
    v9: one fit model for the vertical error -- the projection of the estimated 3-D box
    centre (same point whichever N7 fit produced it) -- and a face-approach no-progress
    guard: after ``FACE_NO_PROGRESS_CORRECTIONS`` wrist-only corrections with a valid fit
    and no base motion, one short forward step (only when the box is ahead and far enough).

(b) v8 587 (``GRASP_TARGET_NOT_VISIBLE``): the grasp stage started ~0.41 m from the box and
    fitted it; one N7 approach macro (0.15 m/s x 1 s) took the robot to 0.27-0.30 m, where
    no N7 fit validated at the search elbow (servo 4 = 2320): approaching from the south on
    the blue floor, the lower half of the box's near face is pale (floor reflection), so the
    silhouette is cut (IoU ~0.5) and the bright-top quad merges with the upper face band.
    v9 dev 424 (same layout) showed that backing off alone does not help: the fit returns at
    ~0.36 m and is lost again at ~0.34 m. A diagnostic sweep of the same layout (setup-only
    placement, fit on robot_cam frames) found a valid fit (<= 1 cm, eval only) at every range
    0.34-0.18 m except 0.32 m with a steeper, look-down elbow (servo 4 = 2480) and one of a
    few wrist values -- the top face dominates the view.
    v9: when the fit is lost on the first frame after a forward macro that started inside
    ``NEAR_FIT_ZONE_M`` (or later while in look-down mode), enter look-down mode: elbow
    ``LOOK_DOWN_ELBOW`` and a short wrist sweep, one frame per posture, until the fit
    validates. A sweep without a fit is followed by one short reverse step (v1/v2's retreat
    step) and a new sweep, at most ``MAX_BACKOFF_STEPS`` reverse steps per box; then the
    last valid wrist/pan/elbow command is restored once and N7's own search applies.
    While in look-down mode forward macros are capped to N7's short creep (re-fit after
    every step) and the posture is held: the vertical error is not chased while the
    projected box centre stays within ``LOOK_DOWN_VERTICAL_HOLD_PX`` of the target row.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v2 as v2
from harness import wrist_zone_skill_v8 as v8
from harness.owncam_view import project_base_points
from harness.visual_box_skill import _pose

PROFILE = 'wrist_zone_skill_v9'
TARGET_ROW_PX = 218.7                     # v2's vertical target row
FACE_NO_PROGRESS_CORRECTIONS = 6
FACE_FORCE_STEP = (.08, 0., .3)          # v2's forced-forward macro
FACE_FORCE_MIN_RADIAL_M = .20
FACE_FORCE_MAX_BEARING_RAD = v2.FORCE_MAX_BEARING_RAD
NEAR_FIT_ZONE_M = .45
LOOK_DOWN_ELBOW = 2480
LOOK_DOWN_WRIST_SWEEP = (560, 530, 500, 620, 680)
LOOK_DOWN_VERTICAL_HOLD_PX = 80.          # = v2 FORCE_MAX_VERTICAL_PX
MAX_BACKOFF_STEPS = 6
BACKOFF_STEP = {'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0., 'duration': 1.}   # v1/v2 retreat step
CAPPED_FORWARD = (.10, .6)               # N7's own short forward macro


def projected_centre_px(box: dict, pose) -> list[float] | None:
    """Image position of the fit's estimated 3-D box centre (independent of which N7 fit produced it)."""
    centre = box.get('estimated_box_center_base_m') if box.get('visible') else None
    if not isinstance(centre, (list, tuple)) or len(centre) != 3 or not np.all(np.isfinite(centre)):
        return None
    px = project_base_points({int(k): v for k, v in pose.items()}, np.asarray([centre], float))[0]
    return [float(px[0]), float(px[1])] if np.all(np.isfinite(px)) else None


class WristOnlyBoxSkillV9(v8.WristOnlyBoxSkillV8):
    """v8 box skill with a fit-model-independent vertical error and a close-range look-down re-fit."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._last_valid_fit: dict[str, Any] | None = None
        self._last_forward: tuple[float, float, float | None] | None = None   # (fwd, duration, fx at start)
        self._look_down = False
        self._look_down_done = False              # gave up once: N7 search from then on
        self._sweep_index = 0
        self._reverse_steps = 0
        self._face_pose_corrections = 0
        self.v9_stats = {'projected_centre_frames': 0, 'face_forced_steps': 0, 'look_down_entries': 0,
                         'sweep_frames': 0, 'backoff_steps': 0, 'pose_restores': 0, 'capped_forwards': 0,
                         'vertical_holds': 0}
        self.v9_events: list[dict[str, Any]] = []

    # ---- base motion bookkeeping ----
    def _drive_macro(self, fwd, turn, duration):
        if self._look_down and fwd > 0 and fwd * duration > CAPPED_FORWARD[0] * CAPPED_FORWARD[1]:
            fwd, duration = CAPPED_FORWARD
            self.v9_stats['capped_forwards'] += 1
        start_fx = (self._last_valid_fit or {}).get('fx')
        self._last_forward = (float(fwd), float(duration), start_fx) if fwd > 0 and abs(turn) < 1e-9 else None
        self._face_pose_corrections = 0
        return super()._drive_macro(fwd, turn, duration)

    # ---- approach ----
    def _approach(self, box, target, pose):
        if target is not None:
            if self._look_down and self._sweep_index:
                self.v9_events.append({'event': 'look_down_fit_valid', 'sweep_frames': self._sweep_index,
                                       'fx_m': round(float(target[0]), 4), 'wrist': int(pose['3'])})
            self._sweep_index = 0
            self._last_valid_fit = {'fx': float(target[0]), 'fy': float(target[1]),
                                    'pose': {int(k): int(v) for k, v in pose.items()}}
            projected = projected_centre_px(box, pose)
            if projected is not None:
                self.v9_stats['projected_centre_frames'] += 1
                if self._look_down and abs(projected[1] - TARGET_ROW_PX) <= LOOK_DOWN_VERTICAL_HOLD_PX:
                    projected = [projected[0], TARGET_ROW_PX]           # hold the look-down posture
                    self.v9_stats['vertical_holds'] += 1
                box = {**box, 'pixel_centroid': projected, 'pixel_centroid_fit_model': box.get('pixel_centroid')}
        else:
            action = self._close_range_refit(pose)
            if action is not None:
                return action
        action = super()._approach(box, target, pose)
        return self._face_no_progress_guard(action, target)

    def _close_range_refit(self, pose):
        if self._look_down_done or self._last_valid_fit is None:
            return None
        if not self._look_down:
            fwd = self._last_forward
            if fwd is None or self._fused or fwd[2] is None or fwd[2] >= NEAR_FIT_ZONE_M:
                return None
            self._look_down = True
            self.v9_stats['look_down_entries'] += 1
            self.v9_events.append({'event': 'close_range_fit_lost_look_down', 'after_macro': list(fwd[:2]),
                                   'last_valid_fx_m': round(fwd[2], 4)})
        elif self._fused and self._sweep_index == 0:
            return None                            # v2 bridges a transient miss with the fused estimate first
        self._last_forward = None
        if self._sweep_index < len(LOOK_DOWN_WRIST_SWEEP):
            wrist = LOOK_DOWN_WRIST_SWEEP[self._sweep_index]
            self._sweep_index += 1
            self.v9_stats['sweep_frames'] += 1
            pan = int(self._last_valid_fit['pose'].get(6, pose.get('6', 1500)))
            return _pose({4: LOOK_DOWN_ELBOW, 3: wrist, 6: pan})
        self._sweep_index = 0
        if self._reverse_steps < MAX_BACKOFF_STEPS:
            self._reverse_steps += 1
            self.v9_stats['backoff_steps'] += 1
            self._on_base_motion()                 # the base moves: fused estimates are stale
            return dict(BACKOFF_STEP)
        self._look_down, self._look_down_done = False, True
        self.v9_stats['pose_restores'] += 1
        last = self._last_valid_fit['pose']
        self.v9_events.append({'event': 'look_down_gave_up_restore_last_valid_pose', 'pose': last})
        return _pose({k: last[k] for k in (3, 4, 6) if k in last})

    def _face_no_progress_guard(self, action, target):
        if not (self._face_approach and target is not None and isinstance(action, dict) and action.get('kind') == 'pose'):
            return action
        self._face_pose_corrections += 1
        fused = self.last_fused or {}
        fx, fy = fused.get('x'), fused.get('y')
        if self._face_pose_corrections < FACE_NO_PROGRESS_CORRECTIONS or fx is None:
            return action
        radial, bearing = math.hypot(fx, fy), math.atan2(fy, fx)
        if radial <= FACE_FORCE_MIN_RADIAL_M or abs(bearing) > FACE_FORCE_MAX_BEARING_RAD:
            return action
        self.v9_stats['face_forced_steps'] += 1
        self.v9_events.append({'event': 'face_approach_no_progress_step', 'radial_m': round(radial, 4),
                               'bearing_rad': round(bearing, 4)})
        return self._drive_macro(*FACE_FORCE_STEP)


class WristZoneDeliveryV9(v8.WristZoneDeliveryV8):
    """v8 delivery with the v9 box skill (same public API as v6/v7/v8)."""

    def _new_box(self):
        return WristOnlyBoxSkillV9(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    def summary(self):
        return {**super().summary(), 'profile': PROFILE, 'v9_stats': dict(self.box.v9_stats),
                'v9_events': self.box.v9_events[-10:]}
