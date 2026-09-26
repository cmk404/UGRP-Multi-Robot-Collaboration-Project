"""Offline scenario feasibility checker: the PR #169 blockers and the geometry contract.

The two named regressions are the teacher-feasibility blockers of PR #169
(``experiments/2026-09-25-zone-team-a2/README.md`` §7):

* **B2** ``tri_frame`` on ``zone_wide_door`` -- the trio formation is wider than
  the only 0.50 m door, so the run refused a route 332 times before any contact.
  ``test_b2_*``.
* **B8** ``tri_frame`` on ``zone_wide_two_doors`` -- a colour box parked in front
  of the wide door closed the trio's only remaining route while all three robots
  were committed to team items. ``test_b8_*``.

Everything here is static geometry over the authored maps; no MuJoCo, no scene,
no model call.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import pytest

from harness import static_keepouts as keepouts
from harness import zone_team_footprint as tf
from harness.zone_scenario_feasibility import (GRID_M, ROBOT_RADIUS_M, SCHEMA, TEAM_SIZE, Clearance,
                                               FeasibilityReport, Lattice, ScenarioInfeasible,
                                               ScenarioItem, carry_route, check_hidden_events,
                                               check_placement_blocking, check_carry_routes,
                                               check_robot_count, crossing_zones, evaluate, item_rects,
                                               landing_fits, load_static_map, min_width, passage_fit,
                                               report_ko, required_robots, scenario_items, zone_rect)

ROOT = Path(__file__).resolve().parents[1]

# A coarser lattice keeps most of the suite quick; every verdict taken on it is a
# topological one (a door either admits the formation or it does not). The B8
# reproduction needs the shipped resolution: a 0.889 m formation through a 1.00 m
# door is a millimetre call that a 0.10 m lattice cannot resolve.
FAST = {'grid': .10, 'yaw_steps': 8}
FINE = {'grid': .05, 'yaw_steps': 12}


@pytest.fixture(scope='module')
def door_map():
    return load_static_map('zone_wide_door')


@pytest.fixture(scope='module')
def two_doors_map():
    return load_static_map('zone_wide_two_doors')


@pytest.fixture(scope='module')
def corridor_map():
    return load_static_map('zone_wide_corridor')


# ---------------------------------------------------------------------------
# Reused geometry: this module must decide exactly what the shared checks decide

def test_clearance_agrees_with_the_reused_footprint_check(door_map):
    """The memoised fast path must match ``zone_team_footprint.pose_clear`` pose by pose."""
    footprint = tf.team_footprint('tri_frame')
    rects = keepouts.keepout_rects(door_map, perimeter=True)
    lattice = Lattice.build(door_map, (0.,), grid=.25, yaw_steps=4)
    clearance = Clearance(lattice, footprint, rects, door_map['bounds_m'])
    checked, free = 0, 0
    for i in range(0, lattice.nx, 2):
        for j in range(0, lattice.ny, 2):
            for k in range(len(lattice.yaws)):
                pose = lattice.pose((i, j, k))
                expected = tf.pose_clear(pose, footprint, rects, bounds=door_map['bounds_m'])
                assert clearance.free((i, j, k)) is expected, pose
                checked, free = checked + 1, free + int(expected)
    assert checked > 500 and 0 < free < checked, 'the sample must cover both verdicts'


def test_min_width_is_the_hull_width_of_the_formation():
    """``min_width`` is the gap the formation needs in its best orientation."""
    assert min_width(tf.rect(-.5, .5, -.1, .1)) == pytest.approx(.2)
    # tf.circle circumscribes the disc, so a 4-sided 0.5 m disc is a 1.0 m square.
    assert min_width(tf.circle(0., 0., .5, sides=4)) == pytest.approx(1., abs=1e-6)
    solo = min_width(tf.team_footprint('cyan', margin=0.).hull)
    trio = min_width(tf.team_footprint('tri_frame', margin=0.).hull)
    assert solo == pytest.approx(.21, abs=1e-6)
    assert trio == pytest.approx(.8184, abs=1e-3)
    assert solo < .5 < trio, 'the 0.50 m door admits a solo formation and not the trio'


def test_crossing_zones_covers_multi_lane_doors_too(two_doors_map):
    """``passage_zones`` only yields single-lane passages; a route may use both."""
    single = {pid for pid, _core, _zone in keepouts.passage_zones(two_doors_map)}
    zones = {pid: flag for pid, _rect, flag in crossing_zones(two_doors_map)}
    assert single == {'door_narrow'}
    assert zones == {'door_narrow': True, 'door_wide': False}


def test_item_rects_are_exact_for_boxes_and_bounded_for_cylinders():
    assert item_rects('red', (1.6, -2.45, 0.)) == ((1.6, -2.45, .017, .020, 0.),)
    circumscribed = item_rects('can', (0., 0., 0.))[0]
    inscribed = item_rects('can', (0., 0., 0.), optimistic=True)[0]
    assert circumscribed[2] == pytest.approx(.019)
    assert inscribed[2] < circumscribed[2]


def test_required_robots_comes_from_the_sim_catalogue():
    assert (required_robots('cyan'), required_robots('long_beam'), required_robots('heavy_crate'),
            required_robots('tri_frame')) == (1, 2, 2, 3)


def test_landing_fits_uses_the_catalogue_landing_rectangle(door_map):
    zone = zone_rect(door_map, 'A')
    assert landing_fits('cyan', (zone[0], zone[1], 0.), zone)
    assert landing_fits('long_beam', (zone[0], zone[1], math.pi / 2), zone)
    # The 0.60 m beam laid across the 0.60 m wide zone only fits along it.
    assert not landing_fits('long_beam', (zone[0] + .1, zone[1], 0.), zone)


# ---------------------------------------------------------------------------
# B2: tri_frame versus the 0.50 m door

def test_b2_trio_formation_is_wider_than_the_only_door(door_map):
    """The width bound alone already refuses the B2 scenario."""
    fit = passage_fit(door_map, tf.team_footprint('tri_frame'))
    assert list(fit) == ['door_1']
    assert fit['door_1']['opening_m'] == .5
    assert fit['door_1']['min_footprint_width_m'] >= .889
    assert fit['door_1']['fits'] is False
    # The bare footprint, with no planning margin at all, is still too wide.
    bare = passage_fit(door_map, tf.team_footprint('tri_frame', margin=0.))
    assert bare['door_1']['fits'] is False


def test_b2_tri_frame_has_no_carry_route_through_the_narrow_door(door_map):
    item = ScenarioItem('tri_1', 'tri_frame', (.4, -.85, 0.), 'A', 3)
    route = carry_route(door_map, item, **FAST)
    assert route.verdict == 'infeasible'
    assert route.ok is False
    # The reason quotes the bare formation width, the bound that cannot be argued away.
    assert '0.818' in route.reason_ko and 'door_1' in route.reason_ko
    assert route.proof == 'passage_width'
    assert route.passage_fit['door_1']['min_footprint_width_m'] == pytest.approx(.8894, abs=1e-3)
    assert route.goal_pose is None and route.path == []


def test_b2_a_pair_item_does_pass_the_same_door(door_map):
    """The verdict is about the trio formation, not about the door being impassable."""
    item = ScenarioItem('beam_1', 'long_beam', (1.275, .45, math.pi / 2), 'A', 2)
    route = carry_route(door_map, item, **FAST)
    assert route.verdict == 'feasible'
    assert route.passages_used == ('door_1',)
    assert route.swept_ok is True


def test_b2_scenario_evaluation_is_infeasible_and_says_why(door_map):
    scenario = {'scenario_id': 'b2_tri_narrow_door', 'map_id': 'zone_wide_door',
                'items': [{'item_id': 'tri_1', 'kind': 'tri_frame', 'pose_m': [.4, -.85, 0.],
                           'destination_zone': 'A'},
                          {'item_id': 'red_1', 'kind': 'red', 'pose_m': [1.6, -2.45, 0.],
                           'destination_zone': 'A'}]}
    report = evaluate(scenario, static_map=door_map, **FAST)
    assert report.verdict == 'infeasible'
    assert [f.code for f in report.findings if f.severity == 'infeasible'] == ['no_carry_route']
    assert report.routes['tri_1'].verdict == 'infeasible'
    assert report.routes['red_1'].verdict == 'feasible'
    with pytest.raises(ScenarioInfeasible):
        report.raise_for_verdict()
    text = report_ko(report)
    assert '실현 불가능' in text and 'no_carry_route' in text
    assert json.loads(json.dumps(report.record()))['schema'] == SCHEMA


# ---------------------------------------------------------------------------
# B8: a colour box in front of the wide door, with all three robots in the team

B8_TRI_POSE = (.4, -2.45, 0.)
B8_RED_POSE = (1.6, -2.45, 0.)


def test_b8_the_trio_route_exists_until_the_red_box_is_placed(two_doors_map):
    item = ScenarioItem('tri_1', 'tri_frame', B8_TRI_POSE, 'A', 3)
    clean = carry_route(two_doors_map, item, **FINE)
    assert clean.verdict == 'feasible'
    assert clean.passages_used == ('door_wide',), 'the wide door is the trio only route'
    blocked = carry_route(two_doors_map, item, obstacles=item_rects('red', B8_RED_POSE), **FINE)
    assert blocked.verdict != 'feasible'


def test_b8_names_the_box_and_the_forced_order(two_doors_map):
    """All three robots carry the frame, so the box must be delivered first."""
    items = (ScenarioItem('tri_1', 'tri_frame', B8_TRI_POSE, 'A', 3),
             ScenarioItem('red_1', 'red', B8_RED_POSE, 'A', 1))
    routes, findings = check_carry_routes(two_doors_map, items, **FINE)
    assert not findings and all(route.verdict == 'feasible' for route in routes.values())
    blocking, blocking_findings = check_placement_blocking(two_doors_map, items, routes, **FINE)
    assert blocking['tri_1']['blockers'] == ['red_1']
    assert blocking['tri_1']['concurrent_demand'] == 4 > TEAM_SIZE
    codes = [f.code for f in blocking_findings]
    assert codes == ['clearing_needs_precedence']
    reason = blocking_findings[0].reason_ko
    assert 'red_1' in reason and '먼저' in reason and 'B8' in reason
    robots, robot_findings = check_robot_count(items, blocking)
    assert robots['peak_concurrent_demand'] == 4
    assert [f.code for f in robot_findings] == []


def test_b8_an_unmovable_blocker_is_infeasible_not_a_warning(two_doors_map):
    """A blocker with no destination cannot be cleared at all."""
    items = (ScenarioItem('tri_1', 'tri_frame', B8_TRI_POSE, 'A', 3),
             ScenarioItem('junk_1', 'red', B8_RED_POSE, None, 1))
    routes, _ = check_carry_routes(two_doors_map, items, **FINE)
    _blocking, findings = check_placement_blocking(two_doors_map, items, routes, **FINE)
    assert [(f.code, f.severity) for f in findings] == [('blocker_cannot_be_cleared', 'infeasible')]
    assert 'junk_1' in findings[0].reason_ko


def test_b8_scenario_evaluation_is_conditional_with_the_order_named(two_doors_map):
    scenario = {'scenario_id': 'b8_tri_two_doors', 'map_id': 'zone_wide_two_doors',
                'items': [{'item_id': 'tri_1', 'kind': 'tri_frame', 'pose_m': list(B8_TRI_POSE),
                           'destination_zone': 'A'},
                          {'item_id': 'red_1', 'kind': 'red', 'pose_m': list(B8_RED_POSE),
                           'destination_zone': 'A'}]}
    report = evaluate(scenario, static_map=two_doors_map, **FINE)
    assert report.verdict == 'conditional' and report.ok is True
    assert 'clearing_needs_precedence' in [f.code for f in report.findings]
    report.raise_for_verdict()
    text = report_ko(report)
    assert '조건부 실현 가능' in text and 'red_1' in text
    assert '원인 배치: `red_1`' in text and '동시 필요 로봇 4대' in text


def test_b8_three_team_items_leave_no_robot_to_clear():
    """Robot-count side of B8: a cohort with no solo item has no spare carrier."""
    items = (ScenarioItem('tri_1', 'tri_frame', B8_TRI_POSE, 'A', 3),
             ScenarioItem('beam_1', 'long_beam', (1.275, .45, math.pi / 2), 'A', 2))
    _robots, findings = check_robot_count(items, {})
    assert [f.code for f in findings] == ['no_free_robot']
    assert '3' in findings[0].reason_ko


def test_more_carriers_than_robots_is_infeasible():
    items = (ScenarioItem('huge', 'tri_frame', (0., 0., 0.), 'A', 4),)
    _robots, findings = check_robot_count(items, {})
    assert ('too_many_carriers', 'infeasible') in [(f.code, f.severity) for f in findings]
    assert '4대가 필요' in next(f.reason_ko for f in findings if f.code == 'too_many_carriers')


# ---------------------------------------------------------------------------
# Hidden events

def _blockage(passage, centre, half, *, event_id='blk'):
    return {'event_id': event_id, 'kind': 'passage_blocked',
            'trigger': {'kind': 'sim_time', 'at_sim_s': 45.},
            'target': {'passage': passage,
                       'obstacle': {'obstacle_id': 'pallet', 'center_m': list(centre),
                                    'half_extents_m': list(half), 'height_m': .12}},
            'discovery': {'kind': 'own_camera_near_anchor', 'anchor': passage, 'radius_m': 1.2}}


def test_blocking_the_narrow_door_leaves_the_wide_door(two_doors_map):
    items = (ScenarioItem('cyan_1', 'cyan', (.4, .75, 0.), 'A', 1),)
    routes, _ = check_carry_routes(two_doors_map, items, **FAST)
    scenario = {'eval': {'hidden_events': [_blockage('door_narrow', (2.2, .05), (.15, .22))]}}
    events, findings = check_hidden_events(two_doors_map, items, scenario, routes, **FAST)
    assert events[0]['checked'] is True
    assert events[0]['kept'] == ['cyan_1'] and events[0]['lost'] == []
    assert findings == ()


def test_blocking_the_only_door_is_reported_infeasible(door_map):
    items = (ScenarioItem('cyan_1', 'cyan', (.4, -2.45, 0.), 'A', 1),)
    routes, _ = check_carry_routes(door_map, items, **FAST)
    scenario = {'eval': {'hidden_events': [_blockage('door_1', (2.2, .05), (.15, .245))]}}
    events, findings = check_hidden_events(door_map, items, scenario, routes, **FAST)
    assert events[0]['lost'] == ['cyan_1']
    assert [(f.code, f.severity) for f in findings] == [('no_route_after_event', 'infeasible')]
    assert 'eval.feasibility' in findings[0].reason_ko


def test_a_declared_unsolvable_blockage_is_only_an_info_finding(door_map):
    items = (ScenarioItem('cyan_1', 'cyan', (.4, -2.45, 0.), 'A', 1),)
    routes, _ = check_carry_routes(door_map, items, **FAST)
    scenario = {'eval': {'hidden_events': [_blockage('door_1', (2.2, .05), (.15, .245))],
                         'feasibility': {'intentionally_unsolvable': True,
                                         'notes_ko': '막힘 발견과 보고만 재는 시나리오'}}}
    _events, findings = check_hidden_events(door_map, items, scenario, routes, **FAST)
    assert [(f.code, f.severity) for f in findings] == [('declared_unsolvable_after_event', 'info')]


def test_an_item_moved_event_is_rerouted_from_its_new_pose(door_map):
    items = (ScenarioItem('cyan_1', 'cyan', (.4, -2.45, 0.), 'A', 1),)
    routes, _ = check_carry_routes(door_map, items, **FAST)
    scenario = {'eval': {'hidden_events': [
        {'event_id': 'moved', 'kind': 'item_moved', 'trigger': {'kind': 'sim_time', 'at_sim_s': 30.},
         'target': {'item_id': 'cyan_1', 'to_pose_m': [-.2, .75, 0.]},
         'discovery': {'kind': 'own_camera_near_anchor', 'anchor': 'P1-1', 'radius_m': 1.}}]}}
    events, findings = check_hidden_events(door_map, items, scenario, routes, **FAST)
    assert events[0]['checked'] is True and events[0]['kept'] == ['cyan_1']
    assert events[0]['to_pose_m'] == [-.2, .75, 0.] and findings == ()


def test_a_robot_hold_event_changes_no_geometry(door_map):
    items = (ScenarioItem('cyan_1', 'cyan', (.4, -2.45, 0.), 'A', 1),)
    routes, _ = check_carry_routes(door_map, items, **FAST)
    scenario = {'eval': {'hidden_events': [
        {'event_id': 'hold', 'kind': 'robot_hold', 'trigger': {'kind': 'sim_time', 'at_sim_s': 12.},
         'target': {'robot_id': 'r3', 'duration_s': 40.}, 'discovery': {'kind': 'own_camera_self'}}]}}
    events, findings = check_hidden_events(door_map, items, scenario, routes, **FAST)
    assert events[0]['checked'] is False and findings == ()


# ---------------------------------------------------------------------------
# Scenario plumbing

def test_scenario_items_reads_the_study_config_layout():
    scenario = {'scenario_id': 's', 'map_id': 'zone_wide_door',
                'orders': [{'order_id': 'order-1', 'kind': 'long_beam', 'count': 1,
                            'required_robots': 2, 'destination_zone': 'A'}],
                'eval': {'setup': {'placements': [{'item_id': 'beam_1', 'kind': 'long_beam',
                                                   'order_id': 'order-1', 'slot': 'P2-3',
                                                   'pose_m': [1.275, .45, 1.5708]}]}}}
    items = scenario_items(scenario)
    assert len(items) == 1
    assert (items[0].item_id, items[0].kind, items[0].destination_zone,
            items[0].required_robots) == ('beam_1', 'long_beam', 'A', 2)
    assert items[0].pose == (1.275, .45, 1.5708)


def test_scenario_items_rejects_an_unknown_zone():
    with pytest.raises(ValueError, match='destination zone'):
        scenario_items({'items': [{'item_id': 'x', 'kind': 'cyan', 'pose_m': [0., 0., 0.],
                                   'destination_zone': 'Z'}]})


def test_a_start_pose_inside_a_wall_is_reported_as_such(door_map):
    item = ScenarioItem('stuck', 'cyan', (2.2, -1.675, 0.), 'A', 1)
    route = carry_route(door_map, item, **FAST)
    assert route.verdict == 'infeasible' and '출발 자세' in route.reason_ko


def test_an_item_without_a_destination_is_not_routed(door_map):
    route = carry_route(door_map, ScenarioItem('idle', 'cyan', (.4, -2.45, 0.), None, 1), **FAST)
    assert route.verdict == 'feasible' and route.path == [] and '목적 구역이 없어' in route.reason_ko


def test_a_solo_route_on_the_corridor_map_uses_the_corridor(corridor_map):
    route = carry_route(corridor_map, ScenarioItem('cyan_1', 'cyan', (.4, -2.45, 0.), 'C', 1), **FAST)
    assert route.verdict == 'feasible' and route.passages_used == ('corridor_1',)


def test_a_station_buried_in_another_item_is_flagged(two_doors_map):
    """The measured grasp-station gap, not a rule of thumb, decides standability."""
    from harness.zone_scenario_feasibility import check_free_robot_access
    items = (ScenarioItem('can_1', 'can', (1.45, -.85, 0.), 'C', 1),
             ScenarioItem('beam_1', 'long_beam', (1.28, -.85, math.pi / 2), 'A', 2))
    stations, findings = check_free_robot_access(two_doors_map, items)
    assert stations['can_1']['blocked_roles'] == ['any']
    assert [f.code for f in findings if f.item_id == 'can_1'] == ['station_not_standable']
    assert stations['can_1']['gap_m']['any'] < 0.


def test_a_millimetre_station_gap_is_a_warning_not_silence(two_doors_map):
    """E's s6 geometry: the can station clears the beam by under 2 mm."""
    from harness.zone_scenario_feasibility import check_free_robot_access
    items = (ScenarioItem('can_1', 'can', (1.45, -.85, 0.), 'C', 1),
             ScenarioItem('beam_1', 'long_beam', (1.1, -.85, 1.5708), 'A', 2))
    stations, findings = check_free_robot_access(two_doors_map, items)
    gap = stations['can_1']['gap_m']['any']
    assert 0. <= gap < .01
    assert gap < GRID_M
    assert [f.code for f in findings if f.item_id == 'can_1'] == ['station_clearance_tight']
    assert stations['can_1']['blocked_roles'] == []


def test_the_report_records_what_was_not_verified(door_map):
    scenario = {'scenario_id': 'plain', 'map_id': 'zone_wide_door',
                'items': [{'item_id': 'cyan_1', 'kind': 'cyan', 'pose_m': [.4, -2.45, 0.],
                           'destination_zone': 'A'}]}
    report = evaluate(scenario, static_map=door_map, **FAST)
    assert isinstance(report, FeasibilityReport) and report.verdict == 'feasible'
    manifest = report.manifest
    assert manifest['schema'] == SCHEMA and manifest['robot_radius_m'] == ROBOT_RADIUS_M
    assert manifest['grid_m'] == FAST['grid'] and manifest['yaw_steps'] == FAST['yaw_steps']
    assert any('파지·접촉' in line for line in manifest['not_verified'])
    assert any('SIM 완주' in line for line in manifest['not_verified'])
    assert len(manifest['map_canonical_sha256']) == 64
    assert manifest['map_file_sha256'] is None, 'an in-memory map has no file hash to pin'


def test_the_file_hash_matches_the_map_the_scenario_pins():
    """A config pins the map file; the manifest must carry that same hash."""
    scenario = {'scenario_id': 'pinned', 'map_id': 'zone_wide_door',
                'items': [{'item_id': 'cyan_1', 'kind': 'cyan', 'pose_m': [.4, -2.45, 0.],
                           'destination_zone': 'A'}]}
    report = evaluate(scenario, **FAST)
    path = ROOT / 'maps' / 'zones' / 'zone_wide_door.json'
    assert report.manifest['map_file_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()


def test_lattice_rejects_a_yaw_it_was_not_built_for(door_map):
    lattice = Lattice.build(door_map, (), grid=.5, yaw_steps=4)
    lattice.nearest((0., 0., 0.))
    with pytest.raises(ValueError, match='not on the lattice'):
        lattice.nearest((0., 0., .123))
