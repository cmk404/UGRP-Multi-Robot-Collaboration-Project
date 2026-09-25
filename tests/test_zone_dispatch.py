"""Zone benchmark: arena, RGB detection, coordination checks and teacher planning."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from harness import zone_coordination as zc
from harness.zone_perception import detect_all, label_pickup, observe, pixel_to_floor
from sim import zone_arena as za

FIX = Path(__file__).parent / 'fixtures' / 'zone_dispatch'
GOAL = {'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}


def _tops():
    return {'cctv_top': (FIX/'start-top-west.jpg').read_bytes(),
            'cctv_top_east': (FIX/'start-top-east.jpg').read_bytes()}


def test_arena_keeps_the_approved_top_and_adds_one_identical_east_cctv():
    # Retired zone_open geometry (kept byte-identical for Z1-Z3 reproduction).
    static = za.authored_map('zone_open')
    west, east = static['top_cameras']
    from sim.research_dispatch_arena import FIXED_TOP
    assert west == FIXED_TOP and east['name'] == 'cctv_top_east'
    assert {k: v for k, v in east.items() if k not in ('name', 'position_m')} == \
        {k: v for k, v in FIXED_TOP.items() if k not in ('name', 'position_m')}
    assert east['position_m'][1:] == FIXED_TOP['position_m'][1:]
    assert set(static['zone_slots']) == {'A', 'B', 'C'} and all(len(s) == 3 for s in static['zone_slots'].values())



def test_wide_arena_adds_identical_tops_that_cover_the_whole_floor():
    from sim.research_dispatch_arena import FIXED_TOP
    wide = za.authored_map('zone_wide')
    assert wide == json.loads((Path(za.MAP_DIR)/'zone_wide.json').read_text())
    cameras = wide['top_cameras']
    assert cameras[:2] == za.authored_map('zone_open')['top_cameras'] and cameras[0] == FIXED_TOP
    same = lambda c: {k: v for k, v in c.items() if k not in ('name', 'position_m')}
    assert all(same(c) == same(FIXED_TOP) and c['position_m'][2] == FIXED_TOP['position_m'][2] for c in cameras)
    assert [v[0] for v in za.top_views(wide)] == ['cctv_top', 'cctv_top_north', 'cctv_top_east', 'cctv_top_north_east']
    assert tuple(zc.DEFAULT_VIEWS) == za.top_views(za.authored_map()) == za.top_views(wide)
    # Every floor point inside the walls is seen by at least one TOP (box-top height).
    half_y = (FIXED_TOP['position_m'][2]-.032)*math.tan(math.radians(FIXED_TOP['fov_y_deg'])/2)
    half_x = half_y*960/720
    x0, x1, y0, y1 = wide['bounds_m']
    for i in range(41):
        for j in range(41):
            x, y = x0+(x1-x0)*i/40, y0+(y1-y0)*j/40
            assert any(abs(x-c['position_m'][0]) <= half_x and abs(y-c['position_m'][1]) <= half_y for c in cameras)
    zones = [wide['regions']['zone_'+z] for z in 'ABC']
    assert all(r['half_extents_m'] == [.30, .70] for r in zones)
    assert all(_inside_bounds(s['center_m'], wide['bounds_m'], .3) for slots in wide['zone_slots'].values() for s in slots)
    cfg = za.episode('zone_wide', 11, goal=GOAL, extra_boxes={'red': 5, 'cyan': 3, 'yellow': 4, 'green': 3})
    assert len(cfg['setup_only']['objects']) == 20
    task = za.actor_task(cfg['static_map'], GOAL)
    assert 'TOP_NW' in task['cameras'] and 'position_m' not in json.dumps(task)


def _inside_bounds(xy, bounds, margin):
    return bounds[0]+margin <= xy[0] <= bounds[1]-margin and bounds[2]+margin <= xy[1] <= bounds[3]-margin


def test_wide_prompt_names_all_four_top_images():
    cfg = za.episode('zone_wide', 12, goal=GOAL)
    task = za.actor_task(cfg['static_map'], GOAL)
    views = za.top_views(cfg['static_map'])
    frame = {'own': b'o', **{v[2]: v[0].encode() for v in views}}
    ctx = zc.context('r1', task=task, labels={}, view={}, board={}, own_jobs=[], inbox=[])
    request = zc.build_claim_request('r1', request_id='q', task=task, frame=frame, ctx=ctx, views=views)
    assert [i['label'] for i in request['images']] == ['CURRENT OWN RGB', 'TOP_SW', 'TOP_NW', 'TOP_SE', 'TOP_NE']
    assert ('images: CURRENT OWN RGB, TOP_SW (pickup, south), TOP_NW (pickup, north), TOP_SE (zones, south) '
            'and TOP_NE (zones, north). box_labels') in ' '.join(request['messages'][0]['content'].split())


def test_wide_rgb_detection_matches_the_setup_across_four_tops():
    goal = {'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}
    cfg = za.episode('zone_wide', 12, goal=goal, extra_boxes={'red': 1, 'cyan': 1})
    tops = {c['name']: (FIX/f"wide-{c['name']}.jpg").read_bytes() for c in cfg['static_map']['top_cameras']}
    found = detect_all(tops, cfg['static_map'])
    truth = [(o['kind'], o['position_m'][:2]) for o in cfg['setup_only']['objects'].values()]
    assert len(found) == len(truth) == 8  # boxes in the overlap are kept once
    for kind, xy in truth:  # evaluation only: setup is never controller input
        assert any(d['kind'] == kind and math.dist(d['floor_xy_m'], xy) < .02 for d in found)
    assert {d['camera'] for d in found} >= {'cctv_top', 'cctv_top_north'}
    labels = label_pickup(found, cfg['static_map'])
    assert observe(tops, cfg['static_map'], labels)['pickup_boxes_still_visible'] == sorted(labels)


def test_wide_teacher_paths_exist_from_every_spawn_to_every_box_and_slot():
    from scripts.zone_teacher import BOX_CLEARANCE_M, CARRY_RADIUS_M, GRASP_RADIUS_M, plan_path
    goal = {'A': {'red': 3}, 'B': {'cyan': 3}, 'C': {'green': 2, 'yellow': 1}}
    for seed in (11, 12, 13, 14):
        cfg = za.episode('zone_wide', seed, goal=goal, extra_boxes={'red': 1, 'cyan': 1, 'yellow': 1})
        static = cfg['static_map']
        boxes = {oid: o['position_m'][:2] for oid, o in cfg['setup_only']['objects'].items()}
        slots = [s['center_m'] for zone in static['zone_slots'].values() for s in zone]
        for oid, (bx, by) in boxes.items():
            others = [(x, y, BOX_CLEARANCE_M) for k, (x, y) in boxes.items() if k != oid]
            pregrasp = (bx-GRASP_RADIUS_M-.10, by)  # the teacher's to_box goal
            for spawn in cfg['setup_only']['spawns'].values():
                assert plan_path(tuple(spawn[:2]), pregrasp, static['bounds_m'], others + [(bx, by, BOX_CLEARANCE_M)])
            for sx, sy in slots:
                assert plan_path(pregrasp, (sx-GRASP_RADIUS_M-.08, sy), static['bounds_m'], others, radius=CARRY_RADIUS_M)


def test_goal_validation_and_episode_supply():
    assert za.goal_counts({'B': {'cyan': 1}, 'A': {'red': 2}}) == {'A': {'red': 2}, 'B': {'cyan': 1}}
    for bad in ({}, {'D': {'red': 1}}, {'A': {'blue': 1}}, {'A': {'red': 0}}, {'A': {'red': 4}},
                {'A': {'red': True}}):
        with pytest.raises(ValueError):
            za.goal_counts(bad)
    cfg = za.episode('zone_open', 11, goal=GOAL, extra_boxes={'cyan': 2})
    kinds = [o['kind'] for o in cfg['setup_only']['objects'].values()]
    assert kinds.count('red') == 3 and kinds.count('cyan') == 3 and kinds.count('green') == 1
    assert cfg == za.episode('zone_open', 11, goal=GOAL, extra_boxes={'cyan': 2})
    with pytest.raises(ValueError):
        za.episode('zone_open', 11, goal=GOAL, extra_boxes={'cyan': 9})
    task = za.actor_task(cfg['static_map'], GOAL)
    assert 'setup_only' not in json.dumps(task) and 'position_m' not in json.dumps(task)


def test_zone_scene_reuses_the_standard_scene_path_without_touching_bundle_sources():
    from harness.rgb_execution_bundle import source_closure
    from sim.session_scenes import Scene
    from sim.zone_scene import ZoneScene, catalog
    assert [r['id'] for r in catalog()] == ['zones/zone_wide'] and issubclass(ZoneScene, Scene)
    scene = ZoneScene({'layout': 'zones/zone_open', 'seed': 11, 'params': {}, 'contact_profile': None, 'map_file': None,
                       'cargo_ids': None, 'robots': {}, 'objects': [], 'builder': None}, '.')
    assert scene.config['goal'] == za.goal_counts(za.DEFAULT_GOAL) and len(scene.inventory) == 5
    config = za.episode('zone_open', 11, goal=GOAL)
    assert ZoneScene.from_zone_config(config).config['setup_only'] == config['setup_only']
    assert scene.record()['selection'] == 'zones/zone_open'
    assert not {n for n in source_closure() if 'zone' in n}


def test_rgb_detection_matches_the_setup_within_two_centimetres():
    cfg = za.episode('zone_open', 11, goal=za.DEFAULT_GOAL)
    found = detect_all(_tops(), cfg['static_map'])
    truth = [(o['kind'], o['position_m'][:2]) for o in cfg['setup_only']['objects'].values()]
    assert len(found) == len(truth)  # robots and floor paint are not boxes
    for kind, xy in truth:  # evaluation only: setup is never controller input
        assert any(d['kind'] == kind and math.dist(d['floor_xy_m'], xy) < .02 for d in found)
    labels = label_pickup(found, cfg['static_map'])
    assert sorted(labels) == ['cyan-1', 'green-1', 'red-1', 'red-2', 'red-3']
    assert labels['red-1']['floor_xy_m'][0] < labels['red-2']['floor_xy_m'][0]  # west first
    view = observe(_tops(), cfg['static_map'], labels)
    assert view['pickup_boxes_still_visible'] == sorted(labels)
    assert view['zone_counts_seen'] == {'A': {}, 'B': {}, 'C': {}}


def test_pixel_to_floor_inverts_the_authored_projection():
    from harness.dispatch_skill_binding import pixel_from_map
    static = {'top_camera': za.authored_map('zone_open')['top_cameras'][0]}
    u, v = pixel_from_map([.3, -1.7], static, (720, 960), height=.032)
    x, y = pixel_to_floor(u, v, static['top_camera'], (720, 960))
    assert abs(x-.3) < 1e-6 and abs(y+1.7) < 1e-6


LABELS = {'red-1': {'kind': 'red', 'floor_xy_m': [0, 0]}, 'red-2': {'kind': 'red', 'floor_xy_m': [0, 1]},
          'red-3': {'kind': 'red', 'floor_xy_m': [1, 0]}, 'cyan-1': {'kind': 'cyan', 'floor_xy_m': [1, 1]},
          'green-1': {'kind': 'green', 'floor_xy_m': [2, 0]}}
VIEW = {'pickup_boxes_still_visible': sorted(LABELS), 'zone_counts_seen': {'A': {}, 'B': {}, 'C': {}}}


def test_plan_validator_requires_the_exact_goal_with_distinct_matching_boxes():
    validate = zc.plan_validator(za.goal_counts(GOAL), LABELS)
    good = {'assignments': {'r1': [{'box': 'red-1', 'zone': 'A'}, {'box': 'red-2', 'zone': 'A'}],
                            'r2': [{'box': 'cyan-1', 'zone': 'B'}],
                            'r3': [{'box': 'green-1', 'zone': 'C'}, {'box': 'red-3', 'zone': 'C'}]}}
    assert validate(good) == good
    bad_cases = [
        {'assignments': {**good['assignments'], 'r3': [{'box': 'green-1', 'zone': 'C'}]}},
        {'assignments': {**good['assignments'], 'r2': [{'box': 'red-3', 'zone': 'B'}]}},
        {'assignments': {**good['assignments'], 'r2': [{'box': 'red-1', 'zone': 'B'}]}},
        {'assignments': {'r1': [], 'r2': []}},
        {'assignments': {**good['assignments'], 'r2': [{'box': 'cyan-9', 'zone': 'B'}]}}]
    for bad in bad_cases:
        with pytest.raises(ValueError):
            validate(bad)


def test_claim_checks_accept_fitting_claims_and_report_collisions():
    goal = za.goal_counts(GOAL)
    claims = {'r1': {'box': 'red-1', 'zone': 'A'}, 'r2': {'box': 'red-1', 'zone': 'C'},
              'r3': {'box': 'cyan-1', 'zone': 'B'}}
    out = zc.check_claims(claims, goal=goal, labels=LABELS, view=VIEW, active={})
    assert out['accepted'] == {'r3': {'box': 'cyan-1', 'zone': 'B', 'kind': 'cyan'}}
    assert out['collisions'] == [{'kind': 'same_box', 'box': 'red-1', 'robots': ['r1', 'r2']}]
    # Two robots take the only remaining cyan need: they must talk.
    out = zc.check_claims({'r1': {'box': 'cyan-1', 'zone': 'B'}, 'r2': {'box': 'green-1', 'zone': 'B'}},
                          goal={'B': {'cyan': 1, 'green': 1}}, labels=LABELS, view=VIEW, active={})
    assert set(out['accepted']) == {'r1', 'r2'}
    out = zc.check_claims({'r1': {'box': 'red-1', 'zone': 'C'}, 'r2': {'box': 'red-2', 'zone': 'C'}},
                          goal=goal, labels=LABELS, view=VIEW, active={})
    assert out['collisions'][0]['kind'] == 'same_need' and not out['accepted']
    active = {'r3': {'box': 'red-3', 'zone': 'C', 'kind': 'red'}}
    out = zc.check_claims({'r1': {'box': 'red-3', 'zone': 'A'}, 'r2': {'box': 'red-2', 'zone': 'C'}},
                          goal=goal, labels=LABELS, view=VIEW, active=active)
    assert 'already claimed' in out['invalid']['r1'] and 'needs no more' in out['invalid']['r2']
    out = zc.check_claims({'r1': {'box': None, 'zone': None}, 'r2': None}, goal=goal, labels=LABELS,
                          view=VIEW, active={})
    assert out['idle'] == ['r1'] and out['invalid'] == {'r2': 'no valid reply'}


def test_need_and_goal_use_the_rgb_view_and_the_referee_uses_poses_only():
    goal = za.goal_counts(GOAL)
    view = {**VIEW, 'zone_counts_seen': {'A': {'red': 1}, 'B': {'cyan': 1}, 'C': {}}}
    active = {'r1': {'box': 'red-2', 'zone': 'A', 'kind': 'red'}}
    assert zc.remaining_need(goal, view, active) == {'C': {'green': 1, 'red': 1}}
    assert not zc.goal_met(goal, view)
    assert zc.goal_met(goal, {**view, 'zone_counts_seen': {'A': {'red': 2}, 'B': {'cyan': 1},
                                                           'C': {'green': 1, 'red': 1}}})
    static = za.authored_map('zone_open')
    at = lambda z, dy=0: [*[static['regions']['zone_'+z]['center_m'][0], static['regions']['zone_'+z]['center_m'][1]+dy], .016]
    boxes = {'a': {'kind': 'red', 'xyz': at('A')}, 'b': {'kind': 'red', 'xyz': at('A', .2)},
             'c': {'kind': 'cyan', 'xyz': at('B')}, 'd': {'kind': 'green', 'xyz': at('C')},
             'e': {'kind': 'red', 'xyz': [*at('C')[:2], .08]}}
    ref = zc.referee(goal, static, boxes)
    assert ref['per_zone_exact'] == {'A': True, 'B': True, 'C': False} and not ref['goal_met']



def test_a_box_released_by_an_active_job_is_not_counted_twice():
    # ZW1-G5-dyn claim-3: r1 had put red-2 into zone A and was backing off
    # (job still active); TOP saw it and the host also counted r1's claim.
    goal = za.goal_counts(GOAL)
    view = {**VIEW, 'zone_counts_seen': {'A': {'red': 1}, 'B': {'cyan': 1}, 'C': {'green': 1}}}
    active = {'r1': {'box': 'red-2', 'zone': 'A', 'kind': 'red'}, 'r2': {'box': 'red-3', 'zone': 'C', 'kind': 'red'}}
    finished = [{'zone': 'B', 'kind': 'cyan'}, {'zone': 'C', 'kind': 'green'}]
    assert zc.remaining_need(goal, view, active) == {}  # the recorded double count
    assert zc.remaining_need(goal, view, active, finished) == {'A': {'red': 1}}
    out = zc.check_claims({'r3': {'box': 'red-1', 'zone': 'A'}}, goal=goal, labels=LABELS, view=view,
                          active=active, finished=finished)
    assert out['accepted'] == {'r3': {'box': 'red-1', 'zone': 'A', 'kind': 'red'}}
    # Before the release is visible the active claim still counts.
    before = {**view, 'zone_counts_seen': {'A': {}, 'B': {'cyan': 1}, 'C': {'green': 1}}}
    assert zc.remaining_need(goal, before, active, finished) == {'A': {'red': 1}}
    # A finished delivery that RGB no longer sees is not re-credited to active jobs.
    assert zc.remaining_need(goal, before, active, finished + [{'zone': 'A', 'kind': 'red'}]) == {'A': {'red': 1}}


def test_claim_reply_validation():
    ok = {'request_id': 'q', 'claim': {'box': 'red-1', 'zone': 'A'}, 'reason': 'r', 'message': 'm'}
    assert zc.validate_claim_reply(json.dumps(ok), 'q')['claim']['box'] == 'red-1'
    for bad in ({**ok, 'claim': {'box': 'red-1', 'zone': None}}, {**ok, 'request_id': 'x'},
                {**ok, 'extra': 1}, {**ok, 'claim': {'box': 'red-1'}}):
        with pytest.raises(ValueError):
            zc.validate_claim_reply(json.dumps(bad), 'q')


def test_teacher_path_planner_avoids_discs_and_walls():
    from scripts.zone_teacher import plan_path
    bounds = [0., 3., 0., 2.]
    path = plan_path((.4, 1.), (2.6, 1.), bounds, [(1.5, 1., .3)])
    assert path[0] == pytest.approx((.4, 1.), abs=.03) and path[-1] == (2.6, 1.)
    assert all(math.hypot(x-1.5, y-1.) >= .3+.17-1e-6 for x, y in path[1:-1])
    assert plan_path((.4, 1.), (2.6, 1.), bounds, [(1.5, 1., 2.)]) is None


def test_teacher_arm_sequence_interpolates_issued_targets_in_order():
    from scripts.zone_teacher import ArmSequence, FOLDED
    issued = []
    port = SimpleNamespace(apply=lambda action, now: issued.append((now, action)))
    arm = ArmSequence(port, FOLDED)
    arm.queue({1: 1500, 3: 1340}, 0.)
    arm.queue({6: 1600}, 0.)
    t, finished = 0., False
    while not finished and t < 10:
        finished = arm.tick(t)
        t += .05
    servos = [a['servo_id'] if a['kind'] == 'arm' else 6 for _, a in issued]
    assert servos.index(6) > max(i for i, s in enumerate(servos) if s in (1, 3))
    last = {}
    for _, a in issued:
        last[a.get('servo_id', 6)] = a.get('pulse', a.get('pan_pulse'))
    assert last == {1: 1500, 3: 1340, 6: 1600} and arm.commanded[3] == 1340


def test_fixture_claims_do_not_collide_and_leave_extra_robots_idle():
    # dev-fixture-1 (2026-09-25): identical fixture claims collided three
    # times and the run ended with no job. The fixture now ranks askers.
    from scripts.run_zone_dispatch import _fixture_claim
    labels = {'red-1': {'kind': 'red'}, 'red-2': {'kind': 'red'}, 'red-3': {'kind': 'red'},
              'cyan-1': {'kind': 'cyan'}, 'green-1': {'kind': 'green'}}
    for label in labels:
        labels[label]['floor_xy_m'] = [0., 0.]
    view = {'pickup_boxes_still_visible': sorted(labels), 'zone_counts_seen': {'A': {}, 'B': {}, 'C': {}}}
    claims = {r: _fixture_claim(r, 'q', GOAL, labels, view, {}, ('r1', 'r2', 'r3'))['claim']
              for r in ('r1', 'r2', 'r3')}
    checked = zc.check_claims(claims, goal=GOAL, labels=labels, view=view, active={})
    assert len(checked['accepted']) == 3 and not checked['collisions'] and not checked['invalid']
    one_left = {'A': {'red': 1}}
    claims = {r: _fixture_claim(r, 'q', one_left, labels, view, {}, ('r2', 'r3'))['claim'] for r in ('r2', 'r3')}
    assert claims['r2']['box'] == 'red-1' and claims['r3'] == {'box': None, 'zone': None}


def test_replicas_get_the_dispatch_box_finger_pairs_and_the_planner_leaves_an_overlapped_box():
    # dev-fixture-2 (2026-09-25): every teacher carry slid out of the grip
    # because the contact profile declares finger pairs for dispatch_box_geom
    # only; robots that dropped a box next to themselves could not plan out.
    from sim.zone_scene import mirror_box_contact_pairs
    from scripts.zone_teacher import plan_path
    xml = ('<mujoco><contact><pair geom1="r1__left_finger" geom2="dispatch_box_geom" friction="3.4 3.4 .2 .01 .01"/>'
           '<pair geom1="r1__left_finger" geom2="team_beam_geom"/></contact></mujoco>')
    out, count = mirror_box_contact_pairs(xml, ['cargo_box_00', 'cargo_box_01'])
    assert count == 2 and out.count('geom2="cargo_box_00_geom"') == 1 and out.count('3.4 3.4 .2 .01 .01') == 3
    bounds = [-1.05, 5.40, -3.15, -.85]
    assert plan_path((3.10, -2.15), (0., -1.30), bounds, [(3.24, -2.16, .06)]) is not None
    # dev-fixture-3: with a path available the margin is kept, so a robot
    # that grazes a box's margin does not plan straight through the box.
    path = plan_path((-.60, -1.45), (1.0, -1.70), bounds, [(-.25, -1.70, .06)], radius=.21)
    assert path and all(math.hypot(x+.25, y+1.70) >= .06+.21-1e-6 for x, y in path[1:-1])


def test_teacher_goes_around_its_target_box_to_the_pregrasp_pose():
    # dev-fixture-4: with its target excluded from the keep-outs, a robot
    # coming from the east planned through the box and pushed it west.
    from scripts.zone_teacher import plan_path, GRASP_RADIUS_M, ROBOT_RADIUS_M, BOX_CLEARANCE_M
    box = (-.30, -1.70)
    goal = (box[0]-GRASP_RADIUS_M-.10, box[1])
    assert math.dist(goal, box) > ROBOT_RADIUS_M+BOX_CLEARANCE_M
    column = [(box[0], box[1], BOX_CLEARANCE_M), (-.30, -1.30, BOX_CLEARANCE_M), (-.30, -2.10, BOX_CLEARANCE_M)]
    path = plan_path((4.26, -1.60), goal, [-1.05, 5.40, -3.15, -.85], column)
    assert path[-1] == goal and min(math.dist(q, box) for q in path[:-1]) >= ROBOT_RADIUS_M+BOX_CLEARANCE_M-1e-6


def test_plan_prompt_requires_copying_the_accepted_plan_and_failed_jobs_free_their_slot():
    # Z1-G5-plan: robots replied accept=true, plan=null and never committed.
    # Z1-G8-dyn: slots of stopped jobs were never returned ("zone C has no free slot").
    from scripts.run_zone_dispatch import Slots
    assert 'accept=true with\nplan=null is invalid' in zc._PLAN
    slots = Slots(za.authored_map('zone_open'))
    taken = [slots.take('C') for _ in range(3)]
    with pytest.raises(RuntimeError):
        slots.take('C')
    slots.give_back(taken[1])
    assert slots.take('C') == taken[1]
    slots.give_back(taken[0])
    with pytest.raises(ValueError):
        slots.give_back(taken[0])


class _FakeWorld:
    """Poses only, for the teacher's yield rule (the teacher reads truth)."""
    def __init__(self, robots, boxes):
        self.robots, self.boxes = dict(robots), dict(boxes)
        self.data = SimpleNamespace(body=lambda name: SimpleNamespace(xpos=(*self.boxes[name], .02)))

    def robot(self, rid):
        x, y = self.robots[rid]
        return SimpleNamespace(base_xyz=lambda: (x, y, .03), base_rpy=lambda: (0., 0., 0.))


def _teacher_team(robots, boxes, jobs):
    from scripts.zone_teacher import ZoneTeacherExecutor
    world = _FakeWorld(robots, {b: xy for b, xy in boxes.items()})
    port = SimpleNamespace(hold=lambda now: None, apply=lambda cmd, now: None)
    log = []
    team = ZoneTeacherExecutor(world, {r: port for r in robots}, za.authored_map('zone_open'),
                               {b: {'body_name': b} for b in boxes}, lambda *a, **k: log.append((a, k)))
    for rid, box in jobs.items():
        team.robots[rid].assign({'job_id': f'{rid}-1', 'box_body': box, 'slot_xy': (3., -2.)}, 0.)
    return world, team, log


BOXES_Z2G8 = {'cyan-1': (-.197, -1.45), 'green-1': (-.198, -2.649), 'red-1': (.401, -1.453),
              'cyan-2': (.998, -1.449), 'cyan-3': (1.601, -2.05), 'red-2': (1.6, -2.649), 'red-3': (1.599, -1.45)}


def test_blocked_teacher_robots_break_a_mutual_block_by_priority():
    # Z2-G8-plan: r1 and r3 each parked beside the other's pregrasp goal and
    # both waited out the 120 s drive limit.
    from scripts.zone_teacher import YIELD_AFTER_S, plan_path, retreat_point, PEER_CLEARANCE_M, ROBOT_RADIUS_M
    robots = {'r1': (-.68, -2.51), 'r2': (4.2, -2.7), 'r3': (-.68, -1.49)}
    world, team, log = _teacher_team(robots, BOXES_Z2G8, {'r1': 'cyan-1', 'r3': 'green-1'})
    for now in (0., .1):
        for rid in ('r1', 'r3'):
            team.robots[rid].tick(now, team.discs_for)
    assert team.robots['r1'].blocked_since == team.robots['r3'].blocked_since == 0.
    team.resolve_blocks(YIELD_AFTER_S + .1)
    req = team.robots['r3'].yield_req
    assert req and req['by'] is team.robots['r1'] and team.robots['r1'].yield_req is None
    assert team.robots['r2'].yield_req is None
    spot = retreat_point(robots['r3'], req['avoid'], za.authored_map('zone_open')['bounds_m'],
                         team.discs_for(team.robots['r3']), clear=PEER_CLEARANCE_M+ROBOT_RADIUS_M+.05)
    assert spot is not None
    world.robots['r3'] = spot
    assert plan_path(robots['r1'], team.robots['r1'].path_goal, za.authored_map('zone_open')['bounds_m'],
                     team.discs_for(team.robots['r1'])) is not None


def test_robots_that_met_head_on_never_plan_closer_to_each_other():
    # Z2-G8-dyn: in the lane left by yellow-1 r1 and r2 came within 0.19 m.
    # Releasing the overlapped peer margin let both drive into each other.
    from scripts.zone_teacher import plan_path, GRASP_RADIUS_M
    robots = {'r1': (-.03, -2.17), 'r2': (-.21, -2.12), 'r3': (4.24, -2.65)}
    world, team, _ = _teacher_team(robots, BOXES_Z2G8, {'r1': 'green-1', 'r2': 'red-1'})
    for rid, box, peer in (('r1', 'green-1', 'r2'), ('r2', 'red-1', 'r1')):
        goal = (BOXES_Z2G8[box][0]-GRASP_RADIUS_M-.10, BOXES_Z2G8[box][1])
        path = plan_path(robots[rid], goal, za.authored_map('zone_open')['bounds_m'], team.discs_for(team.robots[rid]))
        now = math.dist(robots[rid], robots[peer])
        assert path and min(math.dist(q, robots[peer]) for q in path[1:]) >= now - 1e-6


def test_claim_prompt_breaks_an_all_yield_conflict_by_robot_id():
    # Z2-G8-dyn: r1 and r3 both yielded green-1 to each other three times.
    assert 'if you all yield nobody takes it' in ' '.join(zc._CLAIM.split())
    assert 'lowest robot_id' in zc._CLAIM


def test_independent_robots_see_no_peer_information_and_are_not_arbitrated():
    from harness import zone_solo as zs
    cfg = za.episode('zone_wide', 12, goal=GOAL)
    task = za.actor_task(cfg['static_map'], GOAL)
    views = za.top_views(cfg['static_map'])
    frame = {'own': b'o', **{v[2]: v[0].encode() for v in views}}
    ctx = zs.solo_context('r1', labels=LABELS, view=VIEW, own_jobs=[{'box': 'red-1', 'status': 'issued'}])
    assert set(ctx) == {'robot_id', 'box_labels', 'rgb_view', 'own_jobs'}  # no board, no messages
    request = zs.build_solo_request('r1', request_id='q', task=task, frame=frame, ctx=ctx, views=views)
    system = ' '.join(request['messages'][0]['content'].split())
    assert 'NO COMMUNICATION' in system and '"message"' not in system.split('NO COMMUNICATION')[1]
    assert len(request['images']) == 5
    ok = {'request_id': 'q', 'claim': {'box': 'red-1', 'zone': 'A'}, 'reason': 'r'}
    assert zs.validate_solo_reply('```json\n' + json.dumps(ok) + '\n```', 'q') == ok
    for bad in ({**ok, 'message': 'hi'}, {**ok, 'claim': {'box': 'red-1', 'zone': None}}, {**ok, 'request_id': 'x'},
                {**ok, 'claim': {'box': 'red-1', 'zone': 'D'}}):
        with pytest.raises(ValueError):
            zs.validate_solo_reply(json.dumps(bad), 'q')
    goal = za.goal_counts(GOAL)
    out = zs.check_solo_claims({'r1': {'box': 'red-1', 'zone': 'A'}, 'r2': {'box': 'red-1', 'zone': 'A'},
                                'r3': {'box': 'cyan-1', 'zone': 'A'}}, goal=goal, labels=LABELS, view=VIEW)
    assert set(out['accepted']) == {'r1', 'r2'} and out['same_box_accepted'] == ['red-1']
    assert 'needs no more cyan' in out['invalid']['r3']
    claims = {r: zs.fixture_solo_claim(r, 'q', goal, LABELS, VIEW)['claim'] for r in ('r1', 'r2', 'r3')}
    assert len({c['box'] for c in claims.values()}) == 3


def test_teacher_stops_the_second_robot_and_injected_grasps_stay_open():
    from scripts.zone_teacher import CLOSED, OPEN, TeacherRobot
    def robot(rid, phase, body, outcome=None):
        r = TeacherRobot.__new__(TeacherRobot)
        r.rid, r.phase, r.outcome, r.job = rid, phase, outcome, {'box_body': body}
        return r
    a, b = robot('r1', 'to_box', 'cargo_box_00'), robot('r2', 'lift', 'cargo_box_00')
    team = {'r1': a, 'r2': b}
    a.team = b.team = team
    assert a._taken_by_peer('cargo_box_00') and not b._taken_by_peer('cargo_box_00')
    b.phase, b.outcome = 'failed', 'grasp_failed_by_teacher'  # a failed grasp leaves the box free
    assert not a._taken_by_peer('cargo_box_00')
    b.phase = 'align_box'  # the peer already aligning wins over one still driving
    assert a._taken_by_peer('cargo_box_00')
    b.phase, a.assigned_at, b.assigned_at = 'to_box', 60.8, 47.3  # both driving: the earlier job keeps the box
    assert a._taken_by_peer('cargo_box_00') and not b._taken_by_peer('cargo_box_00')
    assert a._grip() == CLOSED
    a.job['inject'] = 'grasp_stays_open'
    assert a._grip() == OPEN


def test_planner_backs_out_of_an_overlapped_box_margin_before_ploughing():
    from scripts.zone_teacher import BOX_CLEARANCE_M, plan_path
    # ZC1 gate: r2 at (1.19, -0.05) right behind the red-2 box its open gripper
    # had pushed to (1.27, -0.05); the next target lies further east.
    bounds = [-1.05, 5.40, -3.15, 1.45]
    box = (1.27, -.05, BOX_CLEARANCE_M)
    path = plan_path((1.19, -.05), (1.60, -.60), bounds, [box])
    assert path and math.hypot(path[0][0]-1.27, path[0][1]+.05) >= BOX_CLEARANCE_M + .17
    assert all(math.hypot(x-1.27, y+.05) >= BOX_CLEARANCE_M + .17 - 1e-9 for x, y in path)


def test_zone_open_is_retired_but_reproducible_on_request():
    # 2026-09-25 user request: the small zone_open map is retired. New runs
    # default to zone_wide and refuse zone_open; the map stays for Z1-Z3.
    from scripts.run_zone_dispatch import main, parser
    from sim.zone_scene import ZoneScene
    assert za.DEFAULT_VARIANT == 'zone_wide' and za.RETIRED_VARIANTS == ('zone_open',)
    assert parser().parse_args(['--output', 'x']).variant == 'zone_wide'
    assert za.authored_map() == za.authored_map('zone_wide') and za.episode(goal=GOAL)['variant'] == 'zone_wide'
    with pytest.raises(SystemExit, match='retired'):
        main(['--output', 'unused', '--variant', 'zone_open', '--mode', 'fixture'])
    with pytest.raises(SystemExit):
        parser().parse_args(['--output', 'x', '--variant', 'zone_nowhere'])
    args = parser().parse_args(['--output', 'x', '--variant', 'zone_open', '--allow-retired-variant'])
    assert args.variant == 'zone_open' and args.allow_retired_variant
    assert 'zones/zone_open' not in [r['id'] for r in __import__('sim.zone_scene', fromlist=['catalog']).catalog()]
    assert ZoneScene.from_zone_config(za.episode('zone_open', 11, goal=GOAL)).config['variant'] == 'zone_open'
    assert za.authored_map('zone_open') == json.loads((Path(za.MAP_DIR)/'zone_open.json').read_text())
