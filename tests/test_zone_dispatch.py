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
    static = za.authored_map()
    west, east = static['top_cameras']
    from sim.research_dispatch_arena import FIXED_TOP
    assert west == FIXED_TOP and east['name'] == 'cctv_top_east'
    assert {k: v for k, v in east.items() if k not in ('name', 'position_m')} == \
        {k: v for k, v in FIXED_TOP.items() if k not in ('name', 'position_m')}
    assert east['position_m'][1:] == FIXED_TOP['position_m'][1:]
    assert set(static['zone_slots']) == {'A', 'B', 'C'} and all(len(s) == 3 for s in static['zone_slots'].values())


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
    assert [r['id'] for r in catalog()] == ['zones/zone_open'] and issubclass(ZoneScene, Scene)
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
    static = {'top_camera': za.authored_map()['top_cameras'][0]}
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
    static = za.authored_map()
    at = lambda z, dy=0: [*[static['regions']['zone_'+z]['center_m'][0], static['regions']['zone_'+z]['center_m'][1]+dy], .016]
    boxes = {'a': {'kind': 'red', 'xyz': at('A')}, 'b': {'kind': 'red', 'xyz': at('A', .2)},
             'c': {'kind': 'cyan', 'xyz': at('B')}, 'd': {'kind': 'green', 'xyz': at('C')},
             'e': {'kind': 'red', 'xyz': [*at('C')[:2], .08]}}
    ref = zc.referee(goal, static, boxes)
    assert ref['per_zone_exact'] == {'A': True, 'B': True, 'C': False} and not ref['goal_met']


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
