"""Package F executor: the M1 delivery adapter (split from ``zone_own_executor``, issue #221).

``_DeliverController`` is ``harness.m1_owncam_delivery.M1OwnCamDelivery`` (PR #201, read-only) with:
the executor's shared localizer, uncertainty gate and sweep guard; every leg on
``zone_own_guards.GuardedDriver``; M1 sweeps restricted to the collision-free pan interval; the own-RGB
search limited to the order sheet's coarse pickup slot; lane viewpoints for a far bay; and a retreat
re-look when a cyan region is clipped at the bottom edge of a search frame (too close to fit).
"""
from __future__ import annotations

import base64
import math
from collections.abc import Mapping, Sequence

import numpy as np

from harness import visual_arm as va
from harness import zone_own_guards as guards
from harness.m1_owncam_delivery import NEAR_MIN_DETECTIONS, M1OwnCamDelivery
from harness.owncam_drive import CARRY_POSTURE
from harness.owncam_pose_source import OwnCamPoseSource
from harness.zone_own_contract import SLOT_SEARCH_MARGIN_M, lane_viewpoints

# Near-range truncation (Codex review 2 of PR #206, P2-6): a cyan region clipped at the bottom edge of a
# search frame while nothing fits = "too close to fit", not "absent". Then look again from a retreat viewpoint.
NEAR_CLIP_RIM_PX = 4            # cyan within 4 px above the black fisheye rim
NEAR_CLIP_DARK_MAX = 12         # outside the fisheye image circle (max BGR channel)
NEAR_CLIP_LOWER_FRACTION = .6   # only the lower 40 % of the frame (floor right ahead)
NEAR_CLIP_MIN_PX = 40           # fixtures: clipped 311/337 px, not clipped 0 px, noise <= 7 px
NEAR_CLIP_MIN_FRAMES = 2
NEAR_CLIP_AHEAD_M = .28         # the bottom band of a SEARCH_POSE frame is ~0.2-0.3 m ahead
NEAR_RETREAT_M = .20
LANE_RETREAT_M = .40
CHASSIS_HALF_M = .15


def bottom_clipped_cyan_px(image_b64: str) -> int:
    """Cyan pixels (own-RGB zone profile HSV) in the lower image that touch the fisheye image edge below them.

    The wrist fisheye leaves black corners/rim; a box too close to fit is cut by that rim, not by the
    rectangular image border that ``zone_color_boxes`` checks (fixture: smoke v1 s700 r3 frame 172).
    """
    import cv2

    from harness.zone_color_boxes import OWN_ZONE_CYAN_HSV
    try:
        frame = cv2.imdecode(np.frombuffer(base64.b64decode(image_b64, validate=True), np.uint8), cv2.IMREAD_COLOR)
    except (ValueError, TypeError):
        return 0
    if frame is None or frame.ndim != 3:
        return 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cyan = np.zeros(hsv.shape[:2], bool)
    for low, high in OWN_ZONE_CYAN_HSV:
        cyan |= cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8)) > 0
    outside = frame.max(axis=2) < NEAR_CLIP_DARK_MAX
    rim_below = np.zeros_like(outside)
    for k in range(1, NEAR_CLIP_RIM_PX + 1):
        rim_below[:-k] |= outside[k:]
    hit = cyan & rim_below
    hit[:int(frame.shape[0] * NEAR_CLIP_LOWER_FRACTION)] = False
    return int(np.count_nonzero(hit))


class _DeliverController(M1OwnCamDelivery):
    """M1 delivery on the executor's localizer, gate and sweep guard; own-RGB search limited to the ordered slot."""

    def __init__(self, *args, shared_pose: OwnCamPoseSource, servo: Mapping[int, int],
                 slot_rect: tuple[tuple[float, float], tuple[float, float]], all_rows_y: Sequence[float] = (),
                 gate: guards.UncertaintyGate, guard: guards.SweepGuard, static_keepouts: Sequence[Mapping] = (),
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.all_rows_y = tuple(float(y) for y in all_rows_y)
        self.pose = shared_pose                 # one localizer per robot for the whole episode
        self.servo = dict(servo)                # own issued servo state at job start
        self.slot_rect = slot_rect
        self.gate, self.guard = gate, guard
        self.static_discs = [dict(k) for k in static_keepouts]
        self.near_clipped: list[dict] = []
        self._retreated: set[int] = set()
        self.__dict__.setdefault('legs', [])

    def _init(self, now):
        """M1 init plus lane viewpoints for a far bay (dev-s703 plumb3: P2 boxes are a few pixels from x=-0.47)."""
        decision = super()._init(now)
        if self.phase == 'search_leg' and not getattr(self, '_lanes_added', False):
            self._lanes_added = True
            extra = lane_viewpoints(self.slot_rect, self.all_rows_y, self.pose.report(now).y_m)
            if extra:
                self.viewpoints += extra
                self._event(now, 'lane_viewpoints', viewpoints=self.viewpoints)
        return decision

    @property
    def leg(self):
        return self.__dict__.get('_leg')

    @leg.setter
    def leg(self, value):
        """M1 replaces or drops its leg driver; the replaced leg's guard log is archived first."""
        old = self.__dict__.get('_leg')
        if old is not None and old is not value:
            self._archive(old)
        self.__dict__['_leg'] = value

    def archive_leg(self):
        if self.leg is not None:
            self._archive(self.leg)

    def _archive(self, leg):
        if not getattr(leg, '_archived', False):
            leg._archived = True
            self.__dict__.setdefault('legs', []).append({'goal': list(leg.goal), 'loaded': leg.loaded, 'outcome': leg.outcome, 'looks': leg.looks,
                              'guard_log': leg.guard_log, 'stall_keepouts': leg.stall_keepouts,
                              'gate_looks': leg.gate_looks, 'recoveries': leg.recoveries})

    def _start_leg(self, goal, *, loaded):
        """Every M1 leg on the guarded driver (uncertainty gate, sweep guard, progress monitor)."""
        self.leg = guards.GuardedDriver(self.pose.loc, self.map, self.params, loaded=loaded, goal_xy=goal,
                                        door_xy=self.door_xy, keepouts=self._keepouts(), initial_servo=dict(self.servo),
                                        seed=self.seed, gate=self.gate, guard=self.guard)
        if loaded:
            self.leg.drive_pose = dict(CARRY_POSTURE)
        self.leg_goal = tuple(goal)

    def _start_sweep(self, now, purpose, pose, pans, restore, reason):
        """M1 sweeps keep only the collision-free pan interval (no back-off inside the M1 chain)."""
        rep = self.pose.report(now)
        plan = self.guard.plan(self.servo, pose, pans, guards.OwnPose.from_report(rep),
                               loaded=bool(getattr(getattr(self.skill, 'box', None), 'held', False)))
        kept = plan['pans'] or [int(self.servo.get(6, 1500))]
        if plan['reason'] != 'clear':
            self._event(now, 'sweep_guard', purpose=purpose, requested=list(pans), kept=kept, dropped=plan['dropped'],
                        guard_reason=plan['reason'])
        if purpose == 'search':
            self.near_clipped = []
        super()._start_sweep(now, purpose, pose, kept, restore, reason)

    def _in_slot(self, xy) -> bool:
        (x0, x1), (y0, y1) = self.slot_rect
        m = SLOT_SEARCH_MARGIN_M
        return x0 - m <= xy[0] <= x1 + m and y0 - m <= xy[1] <= y1 + m

    def _search_detect(self, obs, report):
        n = len(self.cyan)
        super()._search_detect(obs, report)
        # Order sheet: the item stands in this coarse pickup slot, so a cyan box seen elsewhere is not it.
        self.cyan[n:] = [d for d in self.cyan[n:] if self._in_slot(d['map_xy'])]
        if len(self.cyan) == n and report.initialized:
            pan = math.radians((int(obs['actuator_state']['servo_pulses'].get('6', va.BASE_CENTER)) - va.BASE_CENTER)
                               / va.PULSE_PER_DEGREE)
            ahead = (report.x_m + NEAR_CLIP_AHEAD_M * math.cos(report.yaw_rad + pan),
                     report.y_m + NEAR_CLIP_AHEAD_M * math.sin(report.yaw_rad + pan))
            if self._in_slot(ahead):
                px = bottom_clipped_cyan_px(obs['image'])
                if px >= NEAR_CLIP_MIN_PX:
                    self.near_clipped.append({'t': round(report.t_est, 3), 'frame_id': int(obs['frame_id']), 'px': px})

    def _retreat_goal(self, vx, vy):
        (sy0, sy1) = self.slot_rect[1]
        x0, _, y0, y1 = self.map['bounds_m']
        candidates = [(vx - NEAR_RETREAT_M, vy)] + [(vx, vy + d) for d in (LANE_RETREAT_M, -LANE_RETREAT_M)
                                                    if sy0 <= vy + d <= sy1]
        for gx, gy in candidates:
            if gx - CHASSIS_HALF_M <= x0 + .05 or not y0 + .2 < gy < y1 - .2:
                continue
            if all(math.hypot(gx - d['center_m'][0], gy - d['center_m'][1]) >= d['radius_m'] + CHASSIS_HALF_M + .02
                   for d in self.static_discs):
                return gx, gy
        return None

    def _after_sweep(self, now):
        if self.phase == 'search_sweep':
            near, n_near = self._cluster('near')
            clipped = len(self.near_clipped)
            if (near is None or n_near < NEAR_MIN_DETECTIONS) and clipped >= NEAR_CLIP_MIN_FRAMES \
                    and self.view_index not in self._retreated and self.view_index < len(self.viewpoints):
                self._retreated.add(self.view_index)
                goal = self._retreat_goal(*self.viewpoints[self.view_index])
                self._event(now, 'near_clipped', frames=clipped, retreat_goal=goal)
                if goal is not None:
                    self.phase = 'search_leg'
                    self._start_leg(goal, loaded=False)
                    return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        return super()._after_sweep(now)
