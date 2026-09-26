"""``wrist_zone_skill_v7``: v6 + re-plan to another box face after a static keep-out guard stop.

v1-v6 stay byte-identical (v6 is the interface the M1 integration, PR #201, adopts;
hash-pinned in the tests). v7 only changes what happens after
``STATIC_KEEPOUT_GUARD`` fires while grasping.

v6 cohort (551-560): 552 and 557 stopped with ``STATIC_KEEPOUT_GUARD`` next to a
parked robot. The approach point was clear, but the box sat so that squaring to
the face in view (and the grasp itself, base ~0.165 m from the box centre) would
have put r1 inside the parked robot's disc; v6 then had no way to try another face.

v7, on a guard stop in the grasp phase:
1. Box pose on the map from OWN RGB only: the N7 ground-fit box centre (base
   frame) of the latest frame that saw it, transformed with the own pose
   estimate, and the face yaw of the latest ready top-edge vote (base-frame yaw +
   own heading). If the box was never located, the run ends as in v6.
2. The face being approached (the one facing the chassis) is marked blocked.
   Every other face k of the box (normal n_k = yaw + k * 90 deg) gives an
   approach point ``box + n_k * REPLAN_APPROACH_STANDOFF_M`` facing the box. A
   face is usable when the straight approach from that point to the grasp base
   position ``box + n_k * GRASP_BASE_STANDOFF_M`` keeps ROBOT_RADIUS +
   disc radius + KEEPOUT_MARGIN from every static keep-out disc and stays inside
   the static bounds.
3. If no face is usable: ``NO_GRASPABLE_FACE_CLEAR_OF_KEEPOUTS`` (clean abort, no
   motion toward any disc). Otherwise the robot backs off (retreat only, still
   under the guard), drives to the nearest usable approach point (planner with an
   own-RGB box disc instead of the whole bay, see ``planner_discs``), and restarts
   the grasp with a fresh box skill. At most ``MAX_KEEPOUT_REPLANS`` re-plans;
   then ``KEEPOUT_REPLAN_LIMIT``.
No live peer pose, no GT, no map face convention is used; keep-outs remain the
static-layout ``StaticKeepout`` discs of v6.
"""
from __future__ import annotations

import math
from typing import Any, Mapping

from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v5 as v5
from harness import wrist_zone_skill_v6 as v6
from harness.wrist_zone_skill_v6 import (KEEPOUT_MARGIN_M, ROBOT_RADIUS_M, StaticKeepout,  # noqa: F401 (re-export)
                                         _segment_point_distance)

PROFILE = 'wrist_zone_skill_v7'
GRASP_BASE_STANDOFF_M = .165        # base centre to box centre at the grasp (v6 cohort 551/556/559: 0.162-0.167 m)
REPLAN_APPROACH_STANDOFF_M = .45
REPLAN_BACKOFF_M = .12
REPLAN_BACKOFF_STEP_LIMIT = 6
MAX_KEEPOUT_REPLANS = 2
BLOCKED_FACE_TOL_RAD = math.radians(45.)
BOX_PLANNER_DISC_M = .08            # own-RGB box footprint for the path planner (+ robot radius inside the planner)
# v6's sides plus the east side (facing west), tried last: in boxes-near-parked-peer layouts the east face can be
# the only clear one. v6 itself is unchanged.
APPROACH_SIDES = v6.APPROACH_SIDES + (('east', math.pi),)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class WristZoneDeliveryV7(v6.WristZoneDeliveryV6):
    """v6 delivery; a keep-out guard stop while grasping re-plans to another clear box face."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.box_map_estimate: dict[str, Any] | None = None
        self.blocked_normals: list[float] = []
        self.keepout_replans = 0
        self.replan: dict[str, Any] | None = None

    # ---------------- bay approach point (v6 + east side) ----------------
    def approach_point(self) -> dict[str, Any]:
        (cx, cy), (hx, hy) = self.order.pickup_bay_center_m, self.order.pickup_bay_half_m
        rejected = []
        index = 0
        for side, heading in APPROACH_SIDES:
            fx, fy = math.cos(heading), math.sin(heading)
            lx, ly = -fy, fx
            half = hx if side in ('west', 'east') else hy
            end = (cx - v6.APPROACH_GRASP_STANDOFF_M * fx, cy - v6.APPROACH_GRASP_STANDOFF_M * fy)
            for clearance, lateral in v6.APPROACH_OFFSETS:
                back = half + clearance
                goal = (cx - back * fx + lateral * lx, cy - back * fy + lateral * ly)
                worst = None
                if self.static_bounds_m is not None:
                    x0, x1, y0, y1 = self.static_bounds_m
                    edge = min(goal[0] - x0, x1 - goal[0], goal[1] - y0, y1 - goal[1]) - ROBOT_RADIUS_M - KEEPOUT_MARGIN_M
                    worst = ('static_bounds', edge)
                for k in self.static_keepouts:
                    need = ROBOT_RADIUS_M + float(k.radius_m) + KEEPOUT_MARGIN_M
                    gap = min(math.hypot(goal[0] - k.xy_m[0], goal[1] - k.xy_m[1]),
                              _segment_point_distance(goal, end, k.xy_m)) - need
                    if worst is None or gap < worst[1]:
                        worst = (k.keepout_id, gap)
                if worst is None or worst[1] >= 0.:
                    return {'blocked': False, 'goal_xy_m': [round(goal[0], 4), round(goal[1], 4)],
                            'heading_rad': round(heading, 6), 'side': side, 'candidate': index,
                            'clearance_m': clearance, 'lateral_offset_m': lateral, 'is_v5_point': index == 0,
                            'rejected': rejected, 'keepouts': [k.record() for k in self.static_keepouts]}
                rejected.append({'candidate': index, 'side': side, 'nearest_keepout': worst[0],
                                 'gap_m': round(worst[1], 4)})
                index += 1
        return {'blocked': True, 'rejected': rejected, 'keepouts': [k.record() for k in self.static_keepouts]}

    # ---------------- own-RGB box pose on the map ----------------
    def _track_box(self, est: v1.PoseEstimate) -> None:
        target = self.box.last_target
        if target is None:
            return
        c, s = math.cos(est.yaw_rad), math.sin(est.yaw_rad)
        bx, by = float(target[0]), float(target[1])
        record = dict(self.box_map_estimate or {})
        record.update({'xy_m': [est.x_m + c * bx - s * by, est.y_m + s * bx + c * by],
                       'source': 'own_rgb_ground_fit+own_pose_estimate', 'pose_source': est.source})
        alignment = self.box.last_face_alignment or {}
        if alignment.get('ready'):
            record['yaw_rad'] = _wrap(est.yaw_rad + math.radians(float(alignment['evidence']['yaw_mod90_deg'])))
            record['yaw_source'] = alignment.get('normal_source')
        self.box_map_estimate = record

    def face_candidates(self, robot_xy) -> list[dict[str, Any]]:
        """Usable faces of the own-RGB box estimate, nearest approach point first."""
        box = self.box_map_estimate
        if not box:
            return []
        bx, by = box['xy_m']
        yaw = box.get('yaw_rad')
        if yaw is None:        # no ready face vote: treat the side facing the robot as a face
            yaw = math.atan2(robot_xy[1] - by, robot_xy[0] - bx)
        out = []
        for k in range(4):
            normal = _wrap(yaw + k * math.pi / 2)
            if any(abs(_wrap(normal - b)) < BLOCKED_FACE_TOL_RAD for b in self.blocked_normals):
                continue
            nx, ny = math.cos(normal), math.sin(normal)
            start = (bx + nx * REPLAN_APPROACH_STANDOFF_M, by + ny * REPLAN_APPROACH_STANDOFF_M)
            end = (bx + nx * GRASP_BASE_STANDOFF_M, by + ny * GRASP_BASE_STANDOFF_M)
            gap, nearest = math.inf, None
            for k_out in self.static_keepouts:
                need = ROBOT_RADIUS_M + float(k_out.radius_m) + KEEPOUT_MARGIN_M
                g = _segment_point_distance(start, end, k_out.xy_m) - need
                if g < gap:
                    gap, nearest = g, k_out.keepout_id
            if self.static_bounds_m is not None:
                x0, x1, y0, y1 = self.static_bounds_m
                g = min(min(p[0] - x0, x1 - p[0], p[1] - y0, y1 - p[1]) for p in (start, end)) - ROBOT_RADIUS_M - KEEPOUT_MARGIN_M
                if g < gap:
                    gap, nearest = g, 'static_bounds'
            out.append({'normal_rad': round(normal, 5), 'approach_xy_m': [round(start[0], 4), round(start[1], 4)],
                        'grasp_base_xy_m': [round(end[0], 4), round(end[1], 4)],
                        'heading_rad': round(_wrap(normal + math.pi), 5), 'gap_m': round(gap, 4),
                        'nearest': nearest, 'usable': gap >= 0.,
                        'travel_m': round(math.hypot(start[0] - robot_xy[0], start[1] - robot_xy[1]), 4)})
        return sorted(out, key=lambda c: (not c['usable'], c['travel_m']))

    def planner_discs(self) -> list[tuple[float, float, float]]:
        """Obstacle discs the runner's planner should add while re-planning (own-RGB box footprint)."""
        if self.phase in ('keepout_backoff', 'replan_nav') and self.box_map_estimate:
            bx, by = self.box_map_estimate['xy_m']
            return [(bx, by, BOX_PLANNER_DISC_M)]
        return []

    # ---------------- guard handling ----------------
    def _replan_after_guard(self, est: v1.PoseEstimate, hit: str, action: Mapping[str, Any]) -> dict[str, Any]:
        box = self.box_map_estimate
        if not box:
            return self._finish('STATIC_KEEPOUT_GUARD')
        if self.keepout_replans >= MAX_KEEPOUT_REPLANS:
            self._event('keepout_replan_limit', est, keepout=hit, replans=self.keepout_replans)
            return self._finish('KEEPOUT_REPLAN_LIMIT')
        bx, by = box['xy_m']
        toward_robot = math.atan2(est.y_m - by, est.x_m - bx)
        yaw = box.get('yaw_rad')
        blocked = toward_robot if yaw is None else min(
            (_wrap(yaw + k * math.pi / 2) for k in range(4)), key=lambda n: abs(_wrap(n - toward_robot)))
        self.blocked_normals.append(blocked)
        candidates = self.face_candidates((est.x_m, est.y_m))
        usable = [c for c in candidates if c['usable']]
        self._event('keepout_replan_candidates', est, keepout=hit, blocked_normal_rad=round(blocked, 5),
                    box_map_estimate=dict(box), candidates=candidates, blocked_action=dict(action))
        if not usable:
            return self._finish('NO_GRASPABLE_FACE_CLEAR_OF_KEEPOUTS')
        self.keepout_replans += 1
        self.replan = {'choice': usable[0], 'origin': (est.x_m, est.y_m), 'steps': 0, 'number': self.keepout_replans}
        self.phase = 'keepout_backoff'
        return v5._wait(.05)

    def _keepout_backoff(self, obs, est):
        rp = self.replan
        moved = math.hypot(est.x_m - rp['origin'][0], est.y_m - rp['origin'][1])
        if moved < REPLAN_BACKOFF_M and rp['steps'] < REPLAN_BACKOFF_STEP_LIMIT:
            rp['steps'] += 1
            return self._retreat_action(est)
        self._event('keepout_backoff_done', est, moved_m=round(moved, 3), steps=rp['steps'])
        self.phase = 'replan_nav'
        return v5._wait(.1)

    def _retreat_action(self, est: v1.PoseEstimate) -> dict[str, Any]:
        """Translate away from the own-RGB box and from every nearby keep-out disc (never toward one)."""
        bx, by = self.box_map_estimate['xy_m']
        vx, vy = est.x_m - bx, est.y_m - by
        norm = math.hypot(vx, vy) or 1.
        vx, vy = vx / norm, vy / norm
        near = []
        for k in self.static_keepouts:
            dx, dy = est.x_m - k.xy_m[0], est.y_m - k.xy_m[1]
            d = math.hypot(dx, dy)
            if d < ROBOT_RADIUS_M + float(k.radius_m) + KEEPOUT_MARGIN_M + .10 and d > 1e-9:
                near.append((dx / d, dy / d))
                vx, vy = vx + 2 * dx / d, vy + 2 * dy / d
        norm = math.hypot(vx, vy) or 1.
        vx, vy = vx / norm, vy / norm
        if any(vx * ax + vy * ay < 0 for ax, ay in near):
            vx, vy = near[0]                                     # straight away from the nearest disc
        c, s = math.cos(est.yaw_rad), math.sin(est.yaw_rad)
        forward, left = .05 * (c * vx + s * vy), .05 * (-s * vx + c * vy)
        return {'kind': 'mecanum', 'forward': float(forward), 'left': float(left), 'turn': 0., 'duration': 1.}

    def _replan_nav(self, obs, est):
        choice = self.replan['choice']
        action = self._navigate(est, tuple(choice['approach_xy_m']), float(choice['heading_rad']), carrying=False)
        if action is None:
            self._event('replan_approach_reached', est, replan=self.replan['number'], choice=choice)
            self.box = self._new_box()
            self.phase = 'grasp'
            return v5._wait(.1)
        return action

    # ---------------- public ----------------
    def decide(self, observation: Mapping[str, Any], estimate: v1.PoseEstimate) -> dict[str, Any]:
        action = v5.WristZoneDeliveryV5.decide(self, observation, estimate)
        if self.phase == 'grasp':
            self._track_box(estimate)
        if self.phase == 'finished':
            return action
        hit = self._guard(action, estimate)
        if hit is None:
            return action
        self.keepout_guard_stops += 1
        self._event('static_keepout_guard', estimate, keepout=hit, blocked_action=dict(action))
        if self.phase == 'grasp':
            return self._replan_after_guard(estimate, hit, action)
        return self._finish('STATIC_KEEPOUT_GUARD')

    def summary(self):
        return {**super().summary(), 'profile': PROFILE, 'keepout_replans': self.keepout_replans,
                'box_map_estimate': self.box_map_estimate,
                'blocked_normals_rad': [round(b, 5) for b in self.blocked_normals]}
