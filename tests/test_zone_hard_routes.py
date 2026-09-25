"""Hard-route zone maps: static rectangle keep-outs, passages and standoffs."""
import hashlib
import json
import math
import random
from pathlib import Path
from types import SimpleNamespace

import pytest

from harness import zone_coordination as zc
from harness import zone_solo as zs
from harness.static_keepouts import (disc_footprint, inside_rect, keepout_rects, passage_zones, pose_clear,
                                     rect_distance, swept_clear)
from scripts import zone_teacher as zt
from sim import zone_arena as za

ROUTES = ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor')
G8 = {'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}


def _battery():
    """Planner queries on the v1 maps (golden digest from origin/main 3cbf4e4)."""
    out = []
    goal = {'A': {'red': 3}, 'B': {'cyan': 3}, 'C': {'green': 2, 'yellow': 1}}
    for variant in ('zone_open', 'zone_wide'):
        for seed in (11, 12, 13):
            g = goal if variant == 'zone_wide' else {'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}
            cfg = za.episode(variant, seed, goal=g, extra_boxes={'red': 1} if variant == 'zone_wide' else {})
            static = cfg['static_map']
            kw = {'rects': keepout_rects(static)}
            boxes = [o['position_m'][:2] for o in cfg['setup_only']['objects'].values()]
            slots = [s['center_m'] for z in static['zone_slots'].values() for s in z]
            rng = random.Random(seed)
            x0, x1, y0, y1 = static['bounds_m']
            for k in range(12):
                start = (rng.uniform(x0+.2, x1-.2), rng.uniform(y0+.2, y1-.2))
                bx, by = boxes[k % len(boxes)]
                sx, sy = slots[k % len(slots)]
                peers = [zt.PeerDisc((rng.uniform(x0, x1), rng.uniform(y0, y1), zt.PEER_CLEARANCE_M)) for _ in range(2)]
                discs = [(x, y, zt.BOX_CLEARANCE_M) for x, y in boxes] + peers
                p1 = zt.plan_path(start, (bx-zt.GRASP_RADIUS_M-.10, by), static['bounds_m'], discs, **kw)
                p2 = zt.plan_path(start, (sx-zt.GRASP_RADIUS_M-.08, sy), static['bounds_m'], discs,
                                  radius=zt.CARRY_RADIUS_M, **kw)
                r = zt.retreat_point(start, [(bx, by)], static['bounds_m'], discs, clear=.5, **kw)
                out.append([p1 and [[round(a, 6) for a in q] for q in p1],
                            p2 and [[round(a, 6) for a in q] for q in p2], r and [round(a, 6) for a in r]])
    return hashlib.sha256(json.dumps(out).encode()).hexdigest()


def test_v1_maps_plans_and_prompts_are_byte_identical_to_main():
    for name, sha in (('zone_open', '2057e40536f0f71077ff362ee127b36c6a4a486fca409d92cfb348a3a8c1c1e0'),
                      ('zone_wide', '6b7b9eb671e5114c78b52e1ee0a4cc02288038aa04edfe37ab4f908bbac6f852')):
        assert hashlib.sha256((Path(za.MAP_DIR)/f'{name}.json').read_bytes()).hexdigest() == sha
        static = za.authored_map(name)
        assert keepout_rects(static) == () and passage_zones(static) == [] and 'passages' not in static
    assert _battery() == 'ee5961d94ad6c143d50cf3ec2fee7969440215d9527570a2b2f987307d88f455'
    expected = {'zone_open': 'fa14c5955772c9c62deef8a71f8719319bb76d79cb9e2f0fe6c7dc1fd547a3bf',
                'zone_wide': '8f5a934c47efe2cd4aa248f71d348063146bc569aceade2bad373dadb067c427'}
    for variant, sha in expected.items():
        cfg = za.episode(variant, 12, goal=G8)
        task = za.actor_task(cfg['static_map'], cfg['goal'])
        assert 'static_map_text' not in task
        views = za.top_views(cfg['static_map'])
        frame = {'own': b'o', **{w[2]: w[0].encode() for w in views}}
        ctx = zc.context('r1', task=task, labels={}, view={}, board={}, own_jobs=[], inbox=[])
        requests = [task, zc.build_claim_request('r1', request_id='q', task=task, frame=frame, ctx=ctx, views=views),
                    zc.build_plan_request('r1', request_id='q', task=task, frame=frame, agreement={}, ctx=ctx,
                                          views=views),
                    zs.build_solo_request('r1', request_id='q', task=task, frame=frame,
                                          ctx=zs.solo_context('r1', labels={}, view={}, own_jobs=[]), views=views)]
        assert hashlib.sha256(json.dumps(requests, sort_keys=True).encode()).hexdigest() == sha


def test_rect_keepouts_rotated_and_footprint_interface():
    rect = (1., 1., .5, .1, math.pi/2)  # rotated: long along y
    assert rect_distance((1., 1.), rect) == 0.
    assert rect_distance((1.3, 1.), rect) == pytest.approx(.2)
    assert rect_distance((1., 1.7), rect) == pytest.approx(.2)
    # Disc robot goes around the rotated wall and keeps its radius from it.
    path = zt.plan_path((.3, 1.), (1.7, 1.), [0., 2., -.5, 2.8], [], rects=(rect,))
    assert path and all(rect_distance(q, rect) >= zt.ROBOT_RADIUS_M - 1e-9 for q in path[1:-1])
    assert zt.plan_path((.3, 1.), (1.7, 1.), [0., 2., 0., 2.], [], rects=((1., 1., 1.2, .05, math.pi/2),)) is None
    # Convex footprint (e.g. a load rectangle) against the same rectangles.
    load = [(.3, .1), (-.3, .1), (-.3, -.1), (.3, -.1)]
    assert pose_clear((.5, 1., 0.), load, (rect,)) and not pose_clear((.8, 1., 0.), load, (rect,))
    assert not pose_clear((1., .2, math.pi/2), load, (rect,), bounds=[0., 2., 0., 2.])  # sticks out of bounds
    assert swept_clear((.5, .2, 0.), (1.5, .2, 0.), load, (rect,))  # passes south of the wall end
    assert not swept_clear((.5, 1., 0.), (1.5, 1., 0.), load, (rect,))
    assert not swept_clear((.7, .33, 0.), (.7, .33, math.pi/2), load, (rect,))  # turning into its corner
    assert pose_clear((.7, .33, 0.), load, (rect,)) and pose_clear((.7, .33, math.pi/2), load, (rect,))
    disc = disc_footprint(.2)
    assert all(math.hypot(*v) >= .2 for v in disc)
    for variant in ROUTES:
        static = za.authored_map(variant)
        rects = keepout_rects(static)
        assert rects and len(rects) == len(static['obstacles']) - 4
        assert len(keepout_rects(static, perimeter=True)) == len(static['obstacles'])


def _floor_seen(point, camera, rects, height=.10):
    """Analytic TOP check: in the image and the line of sight clears every wall."""
    cx, cy, cz = camera['position_m']
    half_y = cz*math.tan(math.radians(camera['fov_y_deg'])/2)
    if abs(point[0]-cx) > half_y*960/720 or abs(point[1]-cy) > half_y:
        return False
    # Below the wall top the ray covers the last height/cz of its horizontal run.
    for i in range(41):
        t = height/cz*i/40
        q = (point[0]+(cx-point[0])*t, point[1]+(cy-point[1])*t)
        if any(inside_rect(q, r) for r in rects):
            return False
    return True


def test_route_maps_keep_the_wide_tops_and_every_free_floor_point_and_doorway_is_seen():
    wide = za.authored_map('zone_wide')
    for variant in ROUTES:
        static = za.authored_map(variant)
        assert static == json.loads((Path(za.MAP_DIR)/f'{variant}.json').read_text())
        for key in ('top_cameras', 'bounds_m', 'regions', 'zone_slots', 'box_kinds'):
            assert static[key] == wide[key]
        assert static['obstacles'][:4] == wide['obstacles'] and za.top_views(static) == za.top_views(wide)
        assert all(o['height_m'] == .10 for o in static['obstacles'])
        rects = keepout_rects(static, perimeter=True)
        x0, x1, y0, y1 = static['bounds_m']
        hidden = []
        for i in range(129):
            for j in range(93):
                p = (x0+.03+(x1-x0-.06)*i/128, y0+.03+(y1-y0-.06)*j/92)
                gap = min(rect_distance(p, r) for r in rects)
                if gap < .005:
                    continue  # under a wall
                if not any(_floor_seen(p, c, rects) for c in static['top_cameras']):
                    hidden.append(gap)
        # A 0.10 m wall hides a thin floor strip behind it from the TOPs that
        # see that side; robot centres keep >= 0.17 m from walls.
        assert not hidden or max(hidden) < .08, (variant, max(hidden))
        for passage in static['passages']:
            (px, py), (hx, hy) = passage['center_m'], passage['half_extents_m']
            seen = 0
            for u in (-1., -.8, -.5, 0., .5, .8, 1.):
                for v in (-1., -.8, -.5, 0., .5, .8, 1.):
                    q = (px+u*hx, py+v*hy)
                    if min(rect_distance(q, r) for r in rects) < (.005 if passage['kind'] == 'door' else .08):
                        continue  # a wall face or the thin shadow strip along it
                    assert any(_floor_seen(q, c, rects) for c in static['top_cameras']), (variant, passage['id'], q)
                    seen += 1
            assert seen >= 9, (variant, passage['id'])


def test_route_maps_door_widths_and_single_lane_geometry():
    for variant in ROUTES:
        static = za.authored_map(variant)
        rects = keepout_rects(static)
        for pid, core, zone in passage_zones(static):
            p = next(q for q in static['passages'] if q['id'] == pid)
            assert p['width_m'] == pytest.approx(.50)
            # A loaded robot fits on the lane centre; two robots never fit side by side.
            centre = core[:2]
            assert all(rect_distance(centre, r) >= zt.CARRY_RADIUS_M for r in rects)
            # Side by side the planner needs radius + (peer clearance + radius) + radius.
            assert p['width_m'] < 3*zt.ROBOT_RADIUS_M + zt.PEER_CLEARANCE_M
            assert p['width_m'] >= 2*zt.CARRY_RADIUS_M + .05
    wide = next(p for p in za.authored_map('zone_wide_two_doors')['passages'] if p['id'] == 'door_wide')
    assert wide['lanes'] == 2 and wide['width_m'] >= 2*zt.CARRY_RADIUS_M + zt.PEER_CLEARANCE_M + zt.CARRY_RADIUS_M
    bay = next(p for p in za.authored_map('zone_wide_corridor')['passages'] if p['kind'] == 'passing_bay')
    lane = next(p for p in za.authored_map('zone_wide_corridor')['passages'] if p['id'] == 'corridor_1')
    # A robot in the bay is clear of a loaded robot on the lane centre line.
    assert lane['center_m'][1] - bay['center_m'][1] >= zt.PASSAGE_CLEAR_M
    assert min(bay['half_extents_m']) >= zt.CARRY_RADIUS_M


def test_route_maps_paths_exist_through_the_passages_for_loaded_robots():
    for variant in ROUTES:
        cfg = za.episode(variant, 12, goal=G8, extra_boxes={'red': 1, 'cyan': 1})
        static = cfg['static_map']
        rects = keepout_rects(static)
        boxes = {oid: o['position_m'][:2] for oid, o in cfg['setup_only']['objects'].items()}
        for oid, (bx, by) in list(boxes.items())[:3]:
            others = [(x, y, zt.BOX_CLEARANCE_M) for k, (x, y) in boxes.items() if k != oid]
            pre = (bx-zt.GRASP_RADIUS_M-.10, by)
            for spawn in cfg['setup_only']['spawns'].values():
                assert zt.plan_path(tuple(spawn[:2]), pre, static['bounds_m'],
                                    others + [(bx, by, zt.BOX_CLEARANCE_M)], rects=rects)
            for zone in 'ABC':
                sx, sy = static['zone_slots'][zone][0]['center_m']
                path = zt.plan_path(pre, (sx-zt.GRASP_RADIUS_M-.08, sy), static['bounds_m'], others,
                                    radius=zt.CARRY_RADIUS_M, rects=rects)
                assert path and all(rect_distance(q, r) >= zt.CARRY_RADIUS_M - 1e-9 for q in path[1:-1] for r in rects)
                # Every trip crosses the divider through a declared passage.
                cross = [q for a, q in zip(path, path[1:]) if (a[0]-za.DIVIDER_X)*(q[0]-za.DIVIDER_X) <= 0]
                assert cross and all(any(inside_rect(q, (*p['center_m'], p['half_extents_m'][0]+.1, p['half_extents_m'][1], 0.))
                                         for p in static['passages'] if p['kind'] != 'passing_bay') for q in cross)


def test_route_prompts_include_the_walls_and_doors_as_static_text_only():
    for variant in ROUTES:
        cfg = za.episode(variant, 12, goal=G8)
        task = za.actor_task(cfg['static_map'], G8)
        text = task['static_map_text']
        for p in cfg['static_map']['passages']:
            assert p['id'] in text
        for o in cfg['static_map']['obstacles'][4:]:
            assert o['id'] in text
        views = za.top_views(cfg['static_map'])
        frame = {'own': b'o', **{w[2]: w[0].encode() for w in views}}
        ctx = zc.context('r1', task=task, labels={}, view={}, board={}, own_jobs=[], inbox=[])
        for request in (zc.build_claim_request('r1', request_id='q', task=task, frame=frame, ctx=ctx, views=views),
                        zs.build_solo_request('r1', request_id='q', task=task, frame=frame,
                                              ctx=zs.solo_context('r1', labels={}, view={}, own_jobs=[]),
                                              views=views)):
            assert request['messages'][0]['content'].endswith('\nStatic map: ' + text)
        # Static map only: no setup poses, spawns or box positions.
        assert 'spawn' not in text and 'box_' not in text and 'position_m' not in json.dumps(task)
    assert 'one robot at a time' in za.actor_task(za.authored_map('zone_wide_door'), G8)['static_map_text']
    assert '2 robots side by side' in za.actor_task(za.authored_map('zone_wide_two_doors'), G8)['static_map_text']
    assert 'passing bay' in za.actor_task(za.authored_map('zone_wide_corridor'), G8)['static_map_text']


def test_route_variants_are_registered_where_variants_are_validated():
    from scripts.run_zone_dispatch import parser
    from sim.zone_scene import catalog
    assert [r['id'] for r in catalog()] == ['zones/'+v for v in ('zone_wide',) + ROUTES]
    for variant in ROUTES:
        assert parser().parse_args(['--output', 'x', '--variant', variant]).variant == variant
        assert za.episode(variant, 11, goal=G8)['static_map']['map_id'] == variant


class _KinematicWorld:
    """Poses integrated from issued mecanum commands (no physics), for the
    teacher's traffic rules. Boxes are floor points unless lifted."""
    def __init__(self, robots, boxes):
        self.poses = {r: list(p) for r, p in robots.items()}
        self.boxes = {b: list(p) for b, p in boxes.items()}
        self.cmd = {r: (0., 0., 0.) for r in robots}
        self.data = SimpleNamespace(body=lambda name: SimpleNamespace(xpos=tuple(self.boxes[name])))

    def robot(self, rid):
        pose = self.poses[rid]
        return SimpleNamespace(base_xyz=lambda: (pose[0], pose[1], .03), base_rpy=lambda: (0., 0., pose[2]))

    def port(self, rid):
        def apply(cmd, now):
            if cmd.get('kind') == 'mecanum':
                self.cmd[rid] = (cmd['forward'], cmd['left'], cmd['turn'])
        def hold(now):
            self.cmd[rid] = (0., 0., 0.)
        return SimpleNamespace(apply=apply, hold=hold)

    def step(self, dt):
        for rid, (fwd, left, turn) in self.cmd.items():
            x, y, yaw = self.poses[rid]
            self.poses[rid] = [x + (math.cos(yaw)*fwd - math.sin(yaw)*left)*dt,
                               y + (math.sin(yaw)*fwd + math.cos(yaw)*left)*dt, yaw + turn*dt]


def _drive_team(variant, robots, jobs, boxes, *, until_phase='align_box', limit_s=150.):
    world = _KinematicWorld(robots, boxes)
    log = []
    team = zt.ZoneTeacherExecutor(world, {r: world.port(r) for r in robots}, za.authored_map(variant),
                                  {b: {'body_name': b} for b in boxes},
                                  lambda kind, rid, now, **d: log.append({'event': kind, 'robot_id': rid,
                                                                          'sim_time_s': round(now, 2), **d}))
    for rid, box in jobs.items():
        team.robots[rid].assign({'job_id': f'{rid}-1', 'box_body': box, 'slot_xy': (4.6, .4)}, 0.)
    now, arrived = 0., {}
    while now < limit_s and len(arrived) < len(jobs):
        team.tick(now)
        world.step(zt.CONTROL_S)
        now = round(now + zt.CONTROL_S, 3)
        for rid in jobs:
            if rid not in arrived and team.robots[rid].phase in (until_phase, 'failed'):
                arrived[rid] = (team.robots[rid].phase, now)
    return team, log, arrived


def test_narrow_door_head_on_resolves_by_yielding_not_mutual_path_blocked():
    # r1 west of the door heads east, r2 east of it heads west: they meet in
    # the one-robot door. Previously each only waited for the other.
    boxes = {'east_box': (3.40, .05, .02), 'west_box': (.80, .05, .02)}
    robots = {'r1': (1.55, .05, 0.), 'r2': (2.85, .05, math.pi), 'r3': (-.85, -2.25, 0.)}
    team, log, arrived = _drive_team('zone_wide_door', robots, {'r1': 'east_box', 'r2': 'west_box'}, boxes)
    assert arrived.get('r1', ('',))[0] == 'align_box' and arrived.get('r2', ('',))[0] == 'align_box', arrived
    assert not any(e.get('outcome') == 'teacher_path_blocked' for e in log)
    standoffs = [e for e in log if e['event'] == 'passage_standoff']
    assert standoffs and standoffs[0]['passage'] == 'door_1'
    # Neither is in the opening nor holds a box: the larger id backs out.
    assert standoffs[0]['robot_id'] == 'r2' and not standoffs[0]['fallback']
    assert any(e['event'] == 'passage_wait' for e in log)


def test_corridor_head_on_resolves_through_the_bay_or_by_backing_out():
    boxes = {'east_box': (4.95, 1.10, .02), 'west_box': (.50, .60, .02)}
    robots = {'r1': (2.40, 1.175, 0.), 'r2': (3.85, 1.175, math.pi), 'r3': (-.85, -2.25, 0.)}
    team, log, arrived = _drive_team('zone_wide_corridor', robots, {'r1': 'east_box', 'r2': 'west_box'}, boxes,
                                     limit_s=240.)
    assert arrived.get('r1', ('',))[0] == 'align_box' and arrived.get('r2', ('',))[0] == 'align_box', arrived
    assert not any(e.get('outcome') == 'teacher_path_blocked' for e in log)
    assert any(e['event'] == 'passage_standoff' and e['passage'] == 'corridor_1' for e in log)


def test_passage_priority_reads_physical_state_only():
    # Same positions, stillness and lifted boxes -> same robot backs out, no
    # matter which jobs (boxes, goals) the robots were given.
    robots = {'r1': (2.05, .05, 0.), 'r2': (2.45, .05, math.pi), 'r3': (-.85, -2.25, 0.)}
    boxes = {'a': (3.4, .05, .02), 'b': (.8, .05, .02), 'c': (.8, -1., .02), 'held': (2.61, .05, .02)}
    chosen = set()
    for jobs in ({'r1': 'a', 'r2': 'b'}, {'r1': 'c', 'r2': 'a'}, {'r1': 'b', 'r2': 'c'}):
        world = _KinematicWorld(robots, boxes)
        log = []
        team = zt.ZoneTeacherExecutor(world, {r: world.port(r) for r in robots}, za.authored_map('zone_wide_door'),
                                      {b: {'body_name': b} for b in boxes}, lambda kind, rid, now, **d: log.append((kind, rid, d)))
        for rid, box in jobs.items():
            team.robots[rid].assign({'job_id': rid, 'box_body': box, 'slot_xy': (4.6, .4)}, 0.)
            team.robots[rid].phase = 'to_box'
        for now in (0., 1., 2.5):
            team.track_motion(now)
        team.resolve_passages(2.5)
        chosen.add(next(rid for kind, rid, _ in log if kind == 'passage_standoff'))
    assert chosen == {'r2'}
    # A box lifted next to r2 (it holds one) gives r2 priority; r1 backs out.
    boxes['held'] = (2.61, .05, .08)
    world = _KinematicWorld(robots, boxes)
    log = []
    team = zt.ZoneTeacherExecutor(world, {r: world.port(r) for r in robots}, za.authored_map('zone_wide_door'),
                                  {b: {'body_name': b} for b in boxes}, lambda kind, rid, now, **d: log.append((kind, rid, d)))
    for rid, box in {'r1': 'a', 'r2': 'b'}.items():
        team.robots[rid].assign({'job_id': rid, 'box_body': box, 'slot_xy': (4.6, .4)}, 0.)
    for now in (0., 1., 2.5):
        team.track_motion(now)
    team.resolve_passages(2.5)
    assert [rid for kind, rid, _ in log if kind == 'passage_standoff'] == ['r1']
    # The rule never reads a job, goal, planned path or claim of any robot.
    import inspect
    source = (inspect.getsource(zt.ZoneTeacherExecutor.resolve_passages)
              + inspect.getsource(zt.ZoneTeacherExecutor.track_motion)
              + inspect.getsource(zt.TeacherRobot.request_passage_yield))
    for field in ('.job', 'path_goal', '.path ', 'assigned_at', 'slot_xy', 'box_body'):
        assert field not in source
    peer_reads = inspect.getsource(zt.TeacherRobot._passage_yield)
    assert 'other.job' not in peer_reads and 'other.path' not in peer_reads


def test_carry_clearance_hook_and_v1_executor_has_no_route_rules():
    robot = zt.TeacherRobot.__new__(zt.TeacherRobot)
    robot.job = {'box_body': 'b'}
    assert robot.radius(False) == zt.ROBOT_RADIUS_M and robot.radius(True) == zt.CARRY_RADIUS_M
    robot.job['carry_radius_m'] = .30
    assert robot.radius(True) == .30
    world = _KinematicWorld({'r1': (0., -1., 0.)}, {'b': (1., -1., .02)})
    team = zt.ZoneTeacherExecutor(world, {'r1': world.port('r1')}, za.authored_map('zone_wide'),
                                  {'b': {'body_name': 'b'}}, lambda *a, **k: None)
    assert team.robots['r1'].rects == () and team.robots['r1'].passages == []
    # Drive-phase limit: unchanged for v1 maps, longer where trips detour through passages.
    assert zt.DRIVE_PHASE_LIMIT_S == 120. and zt.ROUTE_DRIVE_PHASE_LIMIT_S == 240.
