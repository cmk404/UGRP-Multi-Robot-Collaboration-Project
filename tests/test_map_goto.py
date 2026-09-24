import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from harness import dispatch_goto, map_goto
from harness.dispatch_plan import (build_dispatch_request, compile_programs, fixture_plan,
                                   navigation_mode, validate_dispatch_plan, validate_dispatch_reply)
from harness.map_goto import (SOLO_CARRY_ENVELOPE, UNLOADED_ENVELOPE, check_park, envelope_overlaps,
                              parse_destination, plan_path, resolve_destination)
from harness.three_robot_plan import digest
from sim.research_dispatch_arena import authored_map

RAW = Path('tests/fixtures/dispatch_adaptive/north-barrier.jpg').read_bytes()
BOX_START = [-.18, -2.65]


def committed(plan):
    return {'plan': plan, 'plan_hash': digest(plan), 'version': 1, 'proposal_id': 'test'}


def dense_clear(static, route, envelope, extra=()):
    obstacles = map_goto.authored_obstacles(static) + list(extra)
    points = route['waypoints_m']
    for a, b in zip(points, points[1:]):
        for p in map_goto._segment_samples(a, b, .01):
            for o in obstacles:
                assert not envelope_overlaps(p, envelope, o), (p, o['id'])


@pytest.mark.parametrize('value,expected', [
    ('wait_west', {'place': 'wait_west'}),
    ({'place': 'dock_a.box'}, {'place': 'dock_a.box'}),
    ({'xy_m': [1, -2.5]}, {'xy_m': [1., -2.5]}),
])
def test_destination_accepts_place_names_and_map_coordinates(value, expected):
    assert parse_destination(value) == expected


@pytest.mark.parametrize('value', ['', {'xy_m': [True, 1]}, {'xy_m': [float('nan'), 0]},
                                   {'xy_m': [0, 0], 'place': 'x'}, {'xy_m': [0]}, 3])
def test_destination_rejects_malformed_model_output(value):
    with pytest.raises(ValueError):
        parse_destination(value)


def test_unknown_place_and_outside_coordinate_are_rejected_with_choices():
    static = authored_map('open')
    with pytest.raises(ValueError, match='wait_west'):
        resolve_destination(static, 'loading_bay')
    with pytest.raises(ValueError, match='outside'):
        resolve_destination(static, {'xy_m': [2.5, -2.]})
    assert resolve_destination(static, 'dock_b.box')['xy_m'] == [1.82, -2.66]


def test_astar_routes_the_loaded_box_around_the_island_inside_the_envelope():
    static = authored_map('shared_crossing')
    route = dispatch_goto.box_delivery_route(static, 'dock_a', BOX_START, beam_finished=True)
    assert route is not None and route['waypoints_m'][0] == BOX_START
    assert route['waypoints_m'][-1] == dispatch_goto.predock_goal(static, 'dock_a')
    assert route['final_docking_m'] == [1.86, -1.34]
    dense_clear(static, route, SOLO_CARRY_ENVELOPE, [map_goto.beam_delivered_keepout(static, 'dock_a')])
    assert set(route['resources']) <= {'south_gate', 'north_gate', 'dispatch_apron'}
    assert route['plan_sha256'] == map_goto.digest({k: v for k, v in route.items()
                                                    if k not in ('plan_sha256', 'final_docking_m',
                                                                 'final_docking_scope', 'cargo_feature_plane_m')})


def test_unreserved_resource_regions_are_not_entered():
    static = authored_map('shared_crossing')
    south = plan_path(static, BOX_START, [1.5, -2.8], SOLO_CARRY_ENVELOPE)
    assert 'south_gate' in south['resources']
    north = plan_path(static, BOX_START, [1.5, -2.8], SOLO_CARRY_ENVELOPE, blocked_regions=['south_gate'])
    assert north is not None and 'south_gate' not in north['resources'] and 'north_gate' in north['resources']
    assert north['length_m'] > south['length_m']


def test_no_path_is_reported_instead_of_a_colliding_route():
    # Rough south terrain plus the delivered beam team leave no loaded passage to dock_a.
    static = authored_map('rough_south')
    assert dispatch_goto.box_delivery_route(static, 'dock_a', BOX_START, beam_finished=True) is None
    assert plan_path(static, BOX_START, [.6, -2.], SOLO_CARRY_ENVELOPE) is None  # goal on the island


def test_start_escape_leaves_touching_cargo_but_never_crosses_it():
    static = authored_map('open')
    box = map_goto.rect('released_box', [1.82, -2.66], [.04, .04], 'test')
    start = [1.66, -2.66]
    assert plan_path(static, start, [1.0, -2.66], UNLOADED_ENVELOPE, obstacles=[box]) is None
    route = plan_path(static, start, [1.0, -2.66], UNLOADED_ENVELOPE, obstacles=[box], escape_start_m=.06)
    assert route is not None and route['waypoints_m'][1][0] < start[0]


def test_park_rejects_shared_resources_and_the_beam_route_with_short_reasons():
    static = authored_map('open')
    plan = fixture_plan(dock='dock_b', route='north', navigation='planned', park='wait_west')
    keepouts = dispatch_goto.park_keepouts(static, plan, beam_center=[-.18, -1.65])
    bad = check_park(static, 'wait_west', keepouts=keepouts)
    assert not bad['feasible'] and any('beam' in p for p in bad['problems'])
    assert not check_park(static, 'dispatch_apron', keepouts=keepouts)['feasible']
    ok = check_park(static, {'xy_m': [-.75, -2.85]}, keepouts=keepouts)
    assert ok['feasible'] and ok['source'] == 'model_coordinate'
    many = map_goto.pose_route_corridor([[0, -2., 0], [0, -2.5, 0]])
    rejected = check_park(static, {'xy_m': [0, -2.2]}, keepouts=many)
    assert len(rejected['problems']) == 1 and 'samples' in rejected['problems'][0]


def test_planned_plan_contract_requires_auto_box_route_and_park():
    plan = fixture_plan(navigation='planned', park={'xy_m': [-.75, -2.85]})
    assert navigation_mode(plan) == 'planned'
    assert validate_dispatch_plan(plan, navigation='planned') == plan
    with pytest.raises(ValueError):
        validate_dispatch_plan(plan)  # authored contract refuses "auto"
    missing = copy.deepcopy(plan)
    del missing['tasks'][1]['park']
    with pytest.raises(ValueError, match='park'):
        validate_dispatch_plan(missing, navigation='planned')
    beam_auto = copy.deepcopy(plan)
    beam_auto['tasks'][0]['route'] = 'auto'
    with pytest.raises(ValueError):
        validate_dispatch_plan(beam_auto, navigation='planned')
    with pytest.raises(ValueError):
        validate_dispatch_plan(fixture_plan(), navigation='planned')  # authored box corridor
    rows = compile_programs(plan, authored_map('open'))['r2']
    assert rows[3]['stage'] == 'TRANSIT' and set(rows[3]['resources']) == {'north_gate', 'south_gate', 'dispatch_apron'}
    assert rows[0]['park'] == {'xy_m': [-.75, -2.85]}
    assert 'park' not in compile_programs(fixture_plan(), authored_map('open'))['r2'][0]


def test_planned_reply_validator_and_prompt():
    from harness.three_robot_plan import TeamAgreement
    plan = fixture_plan(navigation='planned')
    agreement = TeamAgreement('goto', plan_validator=lambda p: validate_dispatch_plan(p, navigation='planned'))
    snapshot = agreement.snapshot() if hasattr(agreement, 'snapshot') else {'proposal': None}
    raw = json.dumps({'request_id': 'q', 'proposal_id': None, 'plan_hash': None, 'accept': True,
                      'plan': plan, 'reason': 'r', 'message': 'm'})
    with pytest.raises(ValueError):
        validate_dispatch_reply(raw, 'q', snapshot)
    assert validate_dispatch_reply(raw, 'q', snapshot, navigation='planned')['plan'] == plan
    jpeg = RAW
    request = build_dispatch_request('r1', task={'mission_id': 'x'}, request_id='q', own_rgb=jpeg, top_rgb=jpeg,
                                     agreement={'proposal': None}, navigation='planned')
    system = request['messages'][0]['content']
    assert '"route":"auto"' in system and 'xy_m' in system and 'park' in system
    assert json.loads(request['messages'][1]['content'])['navigation_mode'] == 'planned'
    legacy = build_dispatch_request('r1', task={'mission_id': 'x'}, request_id='q', own_rgb=jpeg, top_rgb=jpeg,
                                    agreement={'proposal': None})
    assert 'NAVIGATION MODE' not in legacy['messages'][0]['content']
    assert 'navigation_mode' not in json.loads(legacy['messages'][1]['content'])


def test_bindings_reserve_planned_resources_and_stay_serial():
    from harness.dispatch_skill_binding import SkillBindings
    static = authored_map('open')
    plan = fixture_plan(dock='dock_b', navigation='planned', park={'xy_m': [-.75, -2.85]})
    unplanned = SkillBindings(committed(plan), static, auto_route_overlap=True)
    assert unplanned.route_resources('box') == ['north_gate', 'south_gate']
    assert not unplanned.route_overlap
    assert unplanned.overlap_selection['reason'] == 'planned_navigation_requires_serial'
    with pytest.raises(ValueError, match='planned'):
        SkillBindings(committed(plan), static, route_overlap=True)
    with pytest.raises(ValueError, match='feasibility'):
        SkillBindings(committed(plan), static, box_route={'resources': []})
    route = {'resources': ['south_gate', 'dispatch_apron'], 'resources_checked': True,
             'planning_beam_center_m': [-.18, -1.65], 'plan_sha256': 'x'}
    bound = SkillBindings(committed(plan), static, box_route=route)
    assert bound.route_resources('box') == ['south_gate']
    assert bound.box_route_resources() == ['dispatch_apron', 'south_gate']
    assert bound.capabilities()['box_navigation']['reserved_resources'] == ['dispatch_apron', 'south_gate']
    assert bound.planning_beam_center == [-.18, -1.65]
    assert bound.permission('box', 'GRASP') and set(bound.locks) == {'south_gate', 'dispatch_apron'}
    legacy = SkillBindings(committed(fixture_plan()), static)
    assert 'navigation' not in legacy.capabilities()


def test_feasibility_plans_the_box_from_rgb_and_checks_the_park_place():
    from harness.dispatch_feasibility import inspect_routes
    static = authored_map()
    good = fixture_plan(dock='dock_a', route='south', navigation='planned', park={'xy_m': [-.75, -2.85]})
    report = inspect_routes(committed(good), static, RAW)
    auto = report['box_routes']['auto']
    assert auto['feasible'] and auto['waypoints_m'][-1] == dispatch_goto.predock_goal(static, 'dock_a')
    planned = report['planned_box_route']
    assert planned['resources_checked'] and planned['park']['feasible']
    assert planned['start_source'].startswith('current TOP RGB')
    bad = fixture_plan(dock='dock_a', route='south', navigation='planned', park='south_gate')
    rejected = inspect_routes(committed(bad), static, RAW)
    assert not rejected['feasible'] and 'park place rejected' in rejected['box_routes']['auto']['reason']


def test_planned_carry_control_keeps_the_segment_direction():
    from harness.dispatch_skill_binding import ImageRoute
    control = ImageRoute._planned_control(None, np.array([100., 25.]), 4, False, np.array([.08, .08]))
    assert control['forward'] == pytest.approx(.08)
    assert -control['left'] / control['forward'] == pytest.approx(.25)
    slow = ImageRoute._planned_control(None, np.array([6., 0.]), 4, False, np.array([.08, .08]))
    assert slow['forward'] == pytest.approx(.025) and slow['left'] == 0
    assert ImageRoute._planned_control(None, np.array([1., 1.]), 4, True, np.array([.08, .08]))['forward'] == 0


def test_park_move_follows_astar_waypoints_from_rgb_positions(monkeypatch):
    static = authored_map('open')
    plan = fixture_plan(dock='dock_b', navigation='planned', park={'xy_m': [-.75, -2.85]})
    positions = []

    class FakeObserver:
        def __init__(self, static_map, hint):
            self.xy = np.array([1.62, -2.66])

        def observe(self, jpeg):
            positions.append(self.xy.copy())
            return {'xy_m': self.xy.tolist(), 'heading_rad': 0., 'source': 'test'}

    import harness.dispatch_yield as dispatch_yield
    monkeypatch.setattr(dispatch_yield, 'WheelObserver', FakeObserver)
    monkeypatch.setattr(dispatch_goto, 'rgb_obstacles', lambda static_map, jpeg: [])
    monkeypatch.setattr(dispatch_goto, 'decode', lambda jpeg: np.zeros((720, 960, 3), np.uint8))
    monkeypatch.setattr(dispatch_goto, 'beam_center_from_top', lambda jpeg, static_map: ([-.18, -1.65], 'x' * 64))
    cargo_px = map_goto.map_to_pixel([1.82, -2.66], static, (720, 960))
    policy = dispatch_goto.MapGoToYield(static, plan, cargo_px, top_jpeg=b'', beam_center=[-.18, -1.65])
    for _ in range(400):
        action, evidence = policy.decide(b'')
        if policy.done:
            break
        # Apply the commanded local velocity as ideal translation (heading 0).
        policy.vision.xy = policy.vision.xy + np.array([action['forward'] * 1.57, action['left'] * 1.18]) * .2
    assert policy.done and np.linalg.norm(policy.vision.xy - [-.75, -2.85]) < .02
    assert evidence['destination_source'] == 'model_coordinate'
    assert policy.route['waypoints_m'][1][0] < 1.62  # backs away from the released box first
    released = map_goto.rect('released_box', [1.82, -2.66], [.04, .04], 'test')
    for p in positions[2:]:
        assert not envelope_overlaps(p, UNLOADED_ENVELOPE, released)


def test_park_verdict_must_match_committed_plan(monkeypatch):
    static = authored_map('open')
    plan = fixture_plan(dock='dock_b', navigation='planned', park={'xy_m': [1.0, -2.9]})
    import harness.dispatch_yield as dispatch_yield
    monkeypatch.setattr(dispatch_yield, 'WheelObserver', lambda static_map, hint: None)
    other = check_park(static, 'pickup')
    with pytest.raises(RuntimeError, match='match'):
        dispatch_goto.MapGoToYield(static, plan, [800, 600], top_jpeg=b'', beam_center=[-.18, -1.65],
                                   destination=other)
    with pytest.raises(RuntimeError, match='PARK_REJECTED'):
        dispatch_goto.MapGoToYield(static, fixture_plan(dock='dock_b', navigation='planned', park='dispatch_apron'),
                                   [800, 600], top_jpeg=b'', beam_center=[-.18, -1.65])


@pytest.mark.parametrize('extra', [['--realtime-control'], ['--route-overlap'], ['--executor', 'raw'],
                                   ['--coordination', 'dynamic']])
def test_cli_limits_planned_navigation_to_the_serial_synchronous_skills_executor(tmp_path, extra):
    from scripts.run_dispatch_e2e import main
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path / 'out'), '--navigation', 'planned', *extra])
