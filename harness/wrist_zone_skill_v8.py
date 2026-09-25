"""``wrist_zone_skill_v8``: v7 + a frame-relative cyan saturation gate for the approach stage.

v1-v7 stay byte-identical (v6 is the M1 interface, v7's cohort source is frozen).
v8 keeps v7's public API (``WristZoneDeliveryV8`` is a drop-in for v6/v7).

Problem (M1 dev-a6, PR #181 comment 2026-09-25T22:16Z): N7's approach stage fits
the floor box with ``observe_ground_box(min_saturation=65)``. The west pickup
floor of zone_wide_door is a light blue whose hue (~103) lies inside the cyan
band with saturation ~94-118, so at 65 the floor merges with the box: s93 6/6
approach frames missed, s95 frame 371 took the floor for the box at (0.98, -0.32)
m. v6's top-edge yaw mask (S >= 70) has the same leak.

Fix, following the illumination policy of Kiro B's own-perception package
(PR #193, ``harness/zone_own_perception.py`` @ fb44c2c: colour gates are
relative to the frame and to the local floor/paint, absolute floors only reject
grey): per frame, estimate the floor's own colour from the lower 70 % of the
valid view -- its dominant hue and the 90th percentile saturation of the pixels
within ``FLOOR_HUE_TOL`` of that hue -- and require box pixels to exceed that
floor saturation by ``FLOOR_SAT_MARGIN``:

    min_saturation = clip(max(65, s_floor_p90 + 35), 65, 150)

On the grey east floor the floor saturation is low, so the gate stays at v6's
65 (east behaviour unchanged). On the blue west floor it rises to the top of the
supported range. The same per-frame floor is used for the top-edge yaw: pixels
below the gate are neutralised to grey before v6's edge estimator runs (grey,
not black, so v6's vignette clip test is not triggered). No new fixed colour
number is introduced for a particular floor; only the margin (35) and the
supported range of ``observe_ground_box`` (65-150).

Second fix (v7 cohort seed 570, evaluation-only diagnosis): after a keep-out
guard stop late in the grasp the arm is still in the lowered grasp posture
(own commands 3:500 / 4:2384). v7 starts the fresh box skill there, so the
wrist camera looks at the floor under the gripper and never sees the box 0.44 m
ahead (``GRASP_TARGET_NOT_VISIBLE``). v8 re-commands N7's SEARCH posture when the
keep-out backoff ends (before the re-plan drive) and, if its own last command is
still not SEARCH on arrival, once more before the fresh box skill starts. The
check reads only the robot's own published servo commands.
"""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from harness import visual_box_skill as n7
from harness import wrist_zone_skill_v5 as v5
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v6 as v6
from harness import wrist_zone_skill_v7 as v7

PROFILE = 'wrist_zone_skill_v8'
FLOOR_ROW_START = .30               # lower 70 % of the view is floor at the approach postures
FLOOR_COLOURED_MIN_S = 30           # the floor counts as coloured if most of it is at least this saturated
FLOOR_COLOURED_SHARE = .40
FLOOR_HUE_TOL = 8                   # floor hue cluster (OpenCV H)
FLOOR_SAT_PERCENTILE = 90
FLOOR_SAT_MARGIN = 35
MIN_SAT_RANGE = (65, 150)           # observe_ground_box's supported range; 65 = v6 default
NEUTRAL_GREY = (120, 120, 120)


def floor_relative_min_saturation(frame: np.ndarray) -> dict[str, Any]:
    """Frame-relative cyan saturation gate from the floor's own colour in this frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h = frame.shape[0]
    valid = v6.valid_pixel_mask(1).astype(bool)
    region = np.zeros(frame.shape[:2], bool)
    region[int(FLOOR_ROW_START * h):] = True
    region &= valid
    s = hsv[..., 1][region]
    hue = hsv[..., 0][region]
    coloured = s >= FLOOR_COLOURED_MIN_S
    if coloured.mean() >= FLOOR_COLOURED_SHARE:
        floor_hue = float(np.median(hue[coloured]))
        cluster = coloured & (np.abs(hue.astype(int) - floor_hue) <= FLOOR_HUE_TOL)
        s_ref = float(np.percentile(s[cluster], FLOOR_SAT_PERCENTILE)) if cluster.any() else float(np.percentile(s, FLOOR_SAT_PERCENTILE))
        kind = 'coloured_floor'
    else:
        floor_hue = None
        s_ref = float(np.percentile(s, FLOOR_SAT_PERCENTILE))
        kind = 'grey_floor'
    gate = int(round(min(MIN_SAT_RANGE[1], max(MIN_SAT_RANGE[0], s_ref + FLOOR_SAT_MARGIN))))
    return {'min_saturation': gate, 'floor_kind': kind, 'floor_hue': None if floor_hue is None else round(floor_hue, 1),
            'floor_s_p90': round(s_ref, 1), 'coloured_share': round(float(coloured.mean()), 3)}


def neutralise_below(frame: np.ndarray, min_saturation: int) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    out = frame.copy()
    out[hsv[..., 1] < min_saturation] = NEUTRAL_GREY
    return out


class FloorRelativeEdgeYawAligner(v6.EdgeYawFaceAligner):
    """v6 top-edge yaw vote; the frame is first gated with the floor-relative saturation."""

    def observe(self, box: Mapping[str, Any], target_xy: Sequence[float]) -> dict[str, Any]:
        frame = self.frame
        gated = None
        if isinstance(frame, Mapping) and isinstance(frame.get('image'), str):
            decoded = v6.decode_jpeg_b64(frame['image'])
            if decoded is not None:
                gate = floor_relative_min_saturation(decoded)
                gated = (neutralise_below(decoded, gate['min_saturation']), gate)
        if gated is None:
            return super().observe(box, target_xy)
        original = v6.decode_jpeg_b64
        image, gate = gated
        try:
            v6.decode_jpeg_b64 = lambda _b64: image          # scoped: this call only (single-threaded skill)
            result = super().observe(box, target_xy)
        finally:
            v6.decode_jpeg_b64 = original
        result.setdefault('evidence', {})['floor_gate'] = gate
        return result


class WristOnlyBoxSkillV8(v6.WristOnlyBoxSkillV6):
    """v6 box skill whose approach-stage ground fit uses the floor-relative saturation gate."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._face_aligner = FloorRelativeEdgeYawAligner()
        self.last_floor_gate: dict[str, Any] | None = None

    def decide(self, observation):
        if self.phase != 'approach' or not isinstance(observation, Mapping) or not isinstance(observation.get('image'), str):
            return super().decide(observation)
        frame = v6.decode_jpeg_b64(observation['image'])
        if frame is None:
            return super().decide(observation)            # N7 validation rejects it
        gate = floor_relative_min_saturation(frame)
        self.last_floor_gate = gate
        original = n7.observe_ground_box

        def gated(image, pose, target_id='small_box_01', **options):
            options.setdefault('min_saturation', gate['min_saturation'])
            return original(image, pose, target_id, **options)
        try:
            n7.observe_ground_box = gated                   # scoped: this decide only (single-threaded skill)
            return super().decide(observation)
        finally:
            n7.observe_ground_box = original


def _own_pose_is_search(obs) -> bool:
    state = obs.get('actuator_state') if isinstance(obs, Mapping) else None
    pulses = state.get('servo_pulses') if isinstance(state, Mapping) else None
    if not isinstance(pulses, Mapping):
        return False
    own = {int(k): int(v) for k, v in pulses.items()}
    return all(own.get(k) == v for k, v in n7.SEARCH.items())


class WristZoneDeliveryV8(v7.WristZoneDeliveryV7):
    """v7 delivery with the floor-relative approach gate (same public API as v6/v7)."""

    search_pose_commands = 0

    def _search_pose(self, est, where):
        self.search_pose_commands += 1
        self._event('replan_search_pose', est, where=where)
        return n7._pose(n7.SEARCH)

    def _keepout_backoff(self, obs, est):
        action = super()._keepout_backoff(obs, est)
        if self.phase == 'replan_nav' and not _own_pose_is_search(obs):
            return self._search_pose(est, 'backoff_done')    # raise the wrist camera before the re-plan drive
        return action

    def _replan_nav(self, obs, est):
        # v7's _replan_nav with one change: on arrival, SEARCH posture first if the own command is not SEARCH.
        choice = self.replan['choice']
        action = self._navigate(est, tuple(choice['approach_xy_m']), float(choice['heading_rad']), carrying=False)
        if action is not None:
            return action
        if not _own_pose_is_search(obs):
            return self._search_pose(est, 'approach_reached')
        self._event('replan_approach_reached', est, replan=self.replan['number'], choice=choice)
        self.box = self._new_box()
        self.phase = 'grasp'
        return v5._wait(.1)

    def _new_box(self):
        return WristOnlyBoxSkillV8(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    def summary(self):
        return {**super().summary(), 'profile': PROFILE, 'last_floor_gate': self.box.last_floor_gate,
                'replan_search_pose_commands': self.search_pose_commands}
