"""Pre-run team-route feasibility gate for the GT TEACHER (zone teacher fix, 2026-09-26).

TEACHER-ONLY. This reads the scenario config (setup-only item poses, the
static map, the goal's landing layout) and never the simulator. Its verdict
decides whether a teacher-feasibility run is started at all; it never reaches a
robot, a prompt or a claim. A teacher result is never a robot success.

Blockers it answers (PR #169, experiments/2026-09-25-zone-team-a2 section 7):

- B2 (tri_frame x 0.5 m door): the team footprint cannot reach any landing
  area of its zone even with every solo item removed -> ``infeasible``. The
  teacher's executor would only reject the route at every commit
  (``no_team_route``, 332 times in the record).
- B8 (red box in front of the wide door): a route exists only if some solo
  items are moved first -> ``order_constrained`` with the minimal blockers.
  The teacher executes claims; it cannot reorder them, so such a scenario
  needs a claim order the scripted fixture never produces.

It uses the same planner, footprint, keep-outs, obstacle shapes and goal yaws
as ``scripts.zone_team_teacher.ZoneTeamExecutor._commit`` (read-only reuse).
"""
from __future__ import annotations

import itertools
import math
import time

from harness.static_keepouts import keepout_rects
from harness.zone_goal_v2 import formation, landing_layout, required_carriers
from harness.zone_team_footprint import TeamFootprint, circle, item_polygons, transform
from harness.zone_team_route import plan_team_route

SCHEMA = 'ugrp.zone_teacher_gate.v1'
# Same shapes as ZoneTeamExecutor.item_obstacles (solo items are a 3.5 cm disc there).
SOLO_OBSTACLE_R_M = .035
# Yaw symmetry of team items, as in ZoneTeamExecutor._commit.
SYMMETRY = {'long_beam': math.pi, 'heavy_crate': math.pi, 'tri_frame': 2*math.pi/3}
VERDICTS = ('ok', 'order_constrained', 'infeasible')


def _items(config):
    """[(item_id, kind, (x, y, yaw))] of every physical item from the setup-only config."""
    out = []
    for oid, o in config['setup_only']['objects'].items():
        x, y = o['position_m'][:2]
        out.append((oid, o['kind'], (float(x), float(y), 0.)))
    for it in config.get('cargo_items', ()):
        out.append((it['item_id'], it['kind'], tuple(float(v) for v in it['pose'])))
    return out


def _obstacle(kind, pose):
    if required_carriers(kind) == 1:
        return [circle(pose[0], pose[1], SOLO_OBSTACLE_R_M)]
    return [transform(q, pose) for q in item_polygons(kind)]


def _route(kind, start, area, rects, obstacles, bounds):
    goal = tuple(area['item_pose'])
    sym = SYMMETRY[kind]
    yaws = [goal[2] + k*sym for k in range(int(round(2*math.pi/sym)))]
    footprint = TeamFootprint(kind, tuple(sorted(formation(kind))))
    return plan_team_route(footprint, start, goal, rects=rects, obstacles=obstacles, bounds=bounds, goal_yaws=yaws)


def team_route_feasibility(config, *, max_blocker_set=2):
    """{'schema', 'verdict', 'items': [...]} for every team item of the config.

    Per team item and each landing area of its kind in the goal: route with all
    other items as obstacles -> ok; else with only team cargo as obstacles ->
    order_constrained (smallest solo-item subsets, up to ``max_blocker_set``,
    whose removal alone opens a route); else infeasible. The scenario verdict
    is the worst item verdict.
    """
    static = config['static_map']
    goal = config['goal']
    rects = keepout_rects(static)
    bounds = static['bounds_m']
    areas = {}
    for zone, rows in landing_layout(goal, static_map=static).items():
        for a in rows:
            areas.setdefault(a['kind'], []).append((zone, a))
    items = _items(config)
    rows = []
    started = time.monotonic()
    for iid, kind, pose in items:
        if required_carriers(kind) == 1:
            continue
        others = [(oid, k, p) for oid, k, p in items if oid != iid]
        solo = [(oid, k, p) for oid, k, p in others if required_carriers(k) == 1]
        team = [(oid, k, p) for oid, k, p in others if required_carriers(k) > 1]
        for zone, area in areas.get(kind, ()):
            everything = [q for _, k, p in others for q in _obstacle(k, p)]
            r = _route(kind, pose, area, rects, everything, bounds)
            row = {'item_id': iid, 'kind': kind, 'start_pose': [round(v, 4) for v in pose], 'zone': zone,
                   'landing_item_pose': [round(v, 4) for v in area['item_pose']],
                   'with_all_items': {'ok': r['ok'], 'reason': r['reason'], 'expansions': r['expansions']}}
            if r['ok']:
                row['verdict'] = 'ok'
                row['legs'] = max(0, len(r['poses']) - 1)
                rows.append(row)
                continue
            only_team = [q for _, k, p in team for q in _obstacle(k, p)]
            r2 = _route(kind, pose, area, rects, only_team, bounds)
            row['without_solo_items'] = {'ok': r2['ok'], 'reason': r2['reason'], 'expansions': r2['expansions']}
            if not r2['ok']:
                row['verdict'] = 'infeasible'
                rows.append(row)
                continue
            blockers = []
            for size in range(1, max_blocker_set + 1):
                for subset in itertools.combinations(solo, size):
                    gone = {s[0] for s in subset}
                    obs = [q for oid, k, p in others if oid not in gone for q in _obstacle(k, p)]
                    if _route(kind, pose, area, rects, obs, bounds)['ok']:
                        blockers.append(sorted(gone))
                if blockers:
                    break
            row['verdict'] = 'order_constrained'
            row['blocking_solo_items'] = blockers
            row['blocking_solo_item_poses'] = {oid: [round(v, 4) for v in p[:2]] for oid, _, p in solo
                                               if any(oid in b for b in blockers)}
            rows.append(row)
    rank = {v: i for i, v in enumerate(VERDICTS)}
    verdict = max((r['verdict'] for r in rows), key=rank.get, default='ok')
    return {'schema': SCHEMA, 'verdict': verdict, 'items': rows,
            'source': 'scenario config only (setup-only poses, static map, goal landing layout); '
                      'never simulator state; teacher-only, never robot input',
            'planner': 'harness.zone_team_route.plan_team_route (same as the team teacher commit)',
            'wall_s': round(time.monotonic() - started, 2)}


__all__ = ['SCHEMA', 'VERDICTS', 'team_route_feasibility']
