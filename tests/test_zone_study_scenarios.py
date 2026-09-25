"""Tests for the study scenario configs and their validator (package E-config).

Four properties the study depends on, each proven for every config in
``configs/zone_study_scenarios`` and each with a negative control so the check
cannot pass vacuously:

1. the public part never carries private information;
2. the order sheet is identical whether it is built with or without the private
   section (and after the private section is mutated);
3. hidden events live only in the private part, fire on SIM time and are found by
   an own camera;
4. the cargo mass and the robot count agree with ``sim/zone_cargo.py``.

Plus: the declared placements match the coarse slot the public part promises, the
``leader_ko`` leader really rotates r1/r2/r3 over each scenario's seeds, the
pinned map hashes match the map files on disk, and a per-call payload built from
a config through package A passes ``validate_robot_payload`` in every main
condition.

No simulator, no MuJoCo, no model call, no file write.
"""
from __future__ import annotations

import copy
import json

import pytest

from harness import zone_study_scenarios as E
from harness.zone_map_schematic import digest, load_map, static_map_section
from harness.zone_study_contract import (MAIN_CONDITIONS, ROBOTS, ContractViolation, allowed_edges,
                                         condition, forbidden_key_hits, leader_for_seed, non_ascii_keys,
                                         validate_robot_payload)
from harness.zone_study_inputs import (REQUIRED_ROBOTS, OrderSheetSource, build_call_input, order_sheet,
                                       own_rgb_ref)
from sim.zone_cargo import CATALOGUE, EXISTING_SOLO, MEASURED_SINGLE_ROBOT_CAPACITY_KG

EXPECTED = ('s1_normal_mixed', 's2_unmapped_blockage', 's3_late_rendezvous', 's4_narrow_door_standoff',
            's5_moved_dropped_item', 's6_novel_relation')
# Pinned copy of the SIM catalogue: a change in sim/zone_cargo.py must fail here
# rather than silently alter what a scenario asks robots to do.
EXPECTED_CARGO = {'can': (1, .080), 'tile': (1, .025), 'long_beam': (2, .300), 'heavy_crate': (2, .900),
                  'tri_frame': (3, 1.500), 'cyan': (1, .030), 'green': (1, .030), 'red': (1, .030),
                  'yellow': (1, .030)}
_BUNDLES: dict[str, dict] = {}


def bundle(scenario):
    key = f'{scenario["map_id"]}:{scenario.get("landmark_detail", "full")}'
    if key not in _BUNDLES:
        _BUNDLES[key] = E.bundle_for(scenario)
    return _BUNDLES[key]


@pytest.fixture(scope='module')
def scenarios():
    return E.load_all()


def ids():
    return list(EXPECTED)


# ---------------------------------------------------------------------------
# The set of configs

def test_six_scenarios_cover_the_study_situations(scenarios):
    assert E.scenario_ids() == EXPECTED
    assert sorted(scenarios) == sorted(EXPECTED)
    kinds = {sid: [e['kind'] for e in E.private_part(s).get('hidden_events', [])]
             for sid, s in scenarios.items()}
    assert kinds['s1_normal_mixed'] == [], 'the mixed-carry control carries no hidden event'
    assert kinds['s2_unmapped_blockage'] == ['passage_blocked']
    assert kinds['s3_late_rendezvous'] == ['robot_hold']
    assert kinds['s4_narrow_door_standoff'] == [], 'the standoff is structural, not an event'
    assert kinds['s5_moved_dropped_item'] == ['item_moved', 'item_dropped']
    assert kinds['s6_novel_relation'] == [], 'the novel relation is setup geometry, not an event'
    # Team carries must exist: the study needs role allocation to matter.
    pair = {sid for sid, s in scenarios.items()
            if any(o['required_robots'] > 1 for o in s['orders'])}
    assert pair == {'s1_normal_mixed', 's2_unmapped_blockage', 's3_late_rendezvous',
                    's4_narrow_door_standoff', 's6_novel_relation'}
    assert {s['map_id'] for s in scenarios.values()} == {'zone_wide_door_tags_v1',
                                                        'zone_wide_two_doors_tags_v1',
                                                        'zone_wide_corridor_tags_v1'}


def test_every_config_validates(scenarios):
    for sid, scenario in scenarios.items():
        report = E.validate(scenario, bundle=bundle(scenario))
        assert report.ok, f'{sid}: {report.problems}'
        assert set(report.checks) == set(E.CHECK_NAMES)


def test_seeds_are_disjoint_between_scenarios(scenarios):
    seen = {}
    for sid, scenario in scenarios.items():
        assert len(scenario['seeds']) >= 3, f'{sid} needs at least three paired seeds'
        for seed in scenario['seeds']:
            assert seed not in seen, f'seed {seed} is used by {seen.get(seed)} and {sid}'
            seen[seed] = sid


# ---------------------------------------------------------------------------
# 1. the public part never carries private information

@pytest.mark.parametrize('sid', ids())
def test_public_part_carries_no_private_info(sid):
    scenario = E.load(sid)
    public, private = E.public_part(scenario), E.private_part(scenario)
    assert set(public) <= set(E.PUBLIC_KEYS)
    assert set(scenario) <= set(E.PUBLIC_KEYS) | {E.PRIVATE_KEY}
    assert forbidden_key_hits(public) == []
    assert non_ascii_keys(public) == []
    # No metric number at all: a coordinate cannot hide in the public part.
    assert [n for n in E._numbers(public) if not isinstance(n, int)] == []
    strings = E._strings(public)
    for token in E.private_ids(private):
        assert not any(token in text for text in strings), f'{token} leaked into the public part'
    assert 'hidden_events' not in strings
    # Positive control: the private section is still what a robot payload may not hold.
    hits = ' '.join(forbidden_key_hits(private))
    assert 'hidden_events' in hits and 'pose_m' in hits


def test_a_private_id_in_the_public_part_is_rejected():
    scenario = E.load('s2_unmapped_blockage')
    leaked = copy.deepcopy(scenario)
    leaked['notes'] += ' door_narrow_blocked'
    report = E.validate(leaked, bundle=bundle(scenario))
    assert not report.ok
    assert any('door_narrow_blocked' in problem for problem in report.problems)


def test_a_coordinate_in_the_public_part_is_rejected():
    scenario = E.load('s1_normal_mixed')
    leaked = copy.deepcopy(scenario)
    leaked['seeds'] = [601, 602, 603, 2.2]
    report = E.validate(leaked, bundle=bundle(scenario))
    assert not report.ok


def test_an_event_inside_an_order_is_rejected():
    scenario = E.load('s5_moved_dropped_item')
    leaked = copy.deepcopy(scenario)
    leaked['orders'][0]['hidden_events'] = leaked['eval']['hidden_events']
    report = E.validate(leaked, bundle=bundle(scenario))
    assert not report.ok
    assert any('hidden_event' in problem for problem in report.problems)


# ---------------------------------------------------------------------------
# 2. the order sheet does not depend on the private section

@pytest.mark.parametrize('sid', ids())
def test_order_sheet_identical_with_and_without_the_private_section(sid):
    scenario = E.load(sid)
    map_bundle = bundle(scenario)
    full = order_sheet(scenario, map_bundle)
    public_only = order_sheet(E.public_part(scenario), map_bundle)
    assert digest(full) == digest(public_only)
    assert full == public_only
    for mutation in ({}, {'schema': E.PRIVATE_SCHEMA, 'hidden_events': []},
                     {'schema': E.PRIVATE_SCHEMA, 'hidden_events': [
                         {'event_id': 'probe', 'kind': 'item_dropped',
                          'trigger': {'kind': 'sim_time', 'at_sim_s': 999.5},
                          'target': {'item_id': 'probe_item'},
                          'discovery': {'kind': 'own_camera_self'}}]}):
        mutated = copy.deepcopy(scenario)
        mutated[E.PRIVATE_KEY] = mutation
        assert digest(order_sheet(mutated, map_bundle)) == digest(full)
    # And the sheet itself leaks nothing.
    assert forbidden_key_hits(full) == []
    assert 'hidden_events' not in E._strings(full)
    assert [n for n in E._numbers(full['orders']) if not isinstance(n, int)] == []
    source = OrderSheetSource(scenario, map_bundle)
    assert source.sha256 == digest(full)
    assert OrderSheetSource(E.public_part(scenario), map_bundle).hidden_events() == []
    assert len(source.hidden_events()) == len(E.private_part(scenario).get('hidden_events', []))
    source.assert_unchanged()


@pytest.mark.parametrize('sid', ids())
def test_only_the_channel_differs_between_conditions(sid):
    """Same map, same order sheet, same own inputs; only the channel section changes."""
    scenario = E.load(sid)
    map_bundle = bundle(scenario)
    sheet = order_sheet(scenario, map_bundle)
    static_map = static_map_section(map_bundle)
    seed = scenario['seeds'][0]
    payloads = {}
    for name in MAIN_CONDITIONS:
        spec = condition(name)
        payload = build_call_input(robot_id='r1', condition_name=name, request_id=f'req_{name}',
                                   sim_time_s=0., static_map=static_map, sheet=sheet,
                                   own_rgb_refs=[own_rgb_ref('r1', 1, 0.)],
                                   own_command_history=[], seed=seed,
                                   inbox=[] if 'inbox' in spec.input_allowlist else None)
        validate_robot_payload(payload, seed=seed)
        payloads[name] = payload
        assert payload['order_sheet'] == sheet
        assert payload['static_map']['map_file_sha256'] == map_bundle['map_file_sha256']
        assert forbidden_key_hits(payload) == []
    common = ('static_map', 'order_sheet', 'own_rgb_refs', 'own_command_history', 'self_belief')
    for key in common:
        values = {digest(p[key]) for p in payloads.values()}
        assert len(values) == 1, f'{key} differs between conditions'
    assert len({digest(p['channel']) for p in payloads.values()}) == len(MAIN_CONDITIONS)
    assert 'inbox' not in payloads['no_comm']


# ---------------------------------------------------------------------------
# 3. hidden events live only in the private part

@pytest.mark.parametrize('sid', ids())
def test_hidden_events_are_private_sim_time_and_own_camera(sid):
    scenario = E.load(sid)
    private = E.private_part(scenario)
    assert private['schema'] == E.PRIVATE_SCHEMA
    assert set(private) <= set(E.PRIVATE_KEYS)
    assert 'hidden_events' in private
    last = -1.
    for event in private['hidden_events']:
        assert set(event) <= set(E.EVENT_KEYS)
        assert event['kind'] in E.EVENT_KINDS
        assert event['trigger']['kind'] == 'sim_time'
        at = event['trigger']['at_sim_s']
        assert isinstance(at, (int, float)) and not isinstance(at, bool) and at >= 0
        assert at >= last
        last = at
        assert event['discovery']['kind'] in E.DISCOVERY_KINDS
    passage_events = [e for e in private['hidden_events'] if e['kind'] == 'passage_blocked']
    passages = {p['id']: p for p in bundle(scenario)['public_map']['passages']}
    for event in passage_events:
        gaps = E.free_gaps_m(passages[event['target']['passage']], event['target']['obstacle'])
        assert gaps is not None and max(gaps) < E.LOADED_ROBOT_WIDTH_M
        assert event['discovery']['kind'] == 'own_camera_near_anchor'
        assert event['discovery']['anchor'] == event['target']['passage']


@pytest.mark.parametrize('mutation,expected', [
    (lambda d: d['eval']['hidden_events'][0]['discovery'].update(kind='broadcast'), 'discovery kind'),
    (lambda d: d['eval']['hidden_events'][0]['trigger'].update(kind='on_message'), 'trigger must be'),
    (lambda d: d['eval']['hidden_events'][0]['target']['obstacle'].update(center_m=[2.2, -2.9]),
     'does not sit in the opening'),
    (lambda d: d['eval']['hidden_events'][0]['target']['obstacle'].update(center_m=[2.2, -0.17],
                                                                         half_extents_m=[.15, .03]),
     'is not a blockage'),
    (lambda d: d['eval']['hidden_events'][0]['target'].update(passage='door_nowhere'), 'is not in the map'),
    (lambda d: d['eval'].pop('hidden_events'), 'must declare hidden_events'),
])
def test_a_broken_hidden_event_is_rejected(mutation, expected):
    scenario = E.load('s2_unmapped_blockage')
    bad = copy.deepcopy(scenario)
    mutation(bad)
    report = E.validate(bad, bundle=bundle(scenario))
    assert not report.ok
    assert any(expected in problem for problem in report.problems), report.problems


def test_a_host_announced_event_is_rejected():
    scenario = E.load('s3_late_rendezvous')
    bad = copy.deepcopy(scenario)
    bad['eval']['hidden_events'][0]['discovery'] = {'kind': 'own_camera_near_anchor'}
    report = E.validate(bad, bundle=bundle(scenario))
    assert not report.ok
    assert any('radius_m' in problem for problem in report.problems)


# ---------------------------------------------------------------------------
# 4. cargo mass and robot count match sim/zone_cargo.py

def test_cargo_table_matches_sim_zone_cargo():
    table = E.cargo_table()
    assert set(table) == set(REQUIRED_ROBOTS) == set(EXPECTED_CARGO)
    for kind, (carriers, mass) in EXPECTED_CARGO.items():
        assert table[kind]['required_carriers'] == carriers, kind
        assert table[kind]['mass_kg'] == pytest.approx(mass), kind
        # package A's frozen copy must agree with the SIM catalogue
        assert REQUIRED_ROBOTS[kind] == carriers, kind
    for kind, spec in CATALOGUE.items():
        assert table[kind]['required_carriers'] == spec.required_carriers
        assert table[kind]['mass_kg'] == spec.mass_kg
    box = EXISTING_SOLO['box']
    for kind in E.BOX_KINDS:
        assert table[kind]['mass_kg'] == box['mass_kg']
        assert table[kind]['required_carriers'] == box['required_carriers']
    capacity = MEASURED_SINGLE_ROBOT_CAPACITY_KG[0]
    for kind, entry in table.items():
        if entry['mass_kg'] > capacity:
            assert entry['required_carriers'] >= 2, f'{kind} is heavier than one robot lifts'


@pytest.mark.parametrize('sid', ids())
def test_orders_and_placements_match_the_cargo_catalogue(sid):
    scenario = E.load(sid)
    table = E.cargo_table()
    for order in scenario['orders']:
        entry = table[order['kind']]
        assert order['required_robots'] == entry['required_carriers'], order['order_id']
        assert order['required_robots'] <= len(ROBOTS)
        assert 'mass_kg' not in order
    for placement in E.private_part(scenario)['setup']['placements']:
        assert placement['kind'] in table
        assert 'mass_kg' not in placement


def test_a_wrong_carrier_count_is_rejected():
    scenario = E.load('s2_unmapped_blockage')
    bad = copy.deepcopy(scenario)
    bad['orders'][2]['required_robots'] = 1
    report = E.validate(bad, bundle=bundle(scenario))
    assert not report.ok


def test_a_mass_override_is_rejected():
    scenario = E.load('s1_normal_mixed')
    bad = copy.deepcopy(scenario)
    bad['eval']['setup']['placements'][0]['mass_kg'] = .5
    report = E.validate(bad, bundle=bundle(scenario))
    assert not report.ok
    assert any('catalogue mass' in problem for problem in report.problems)


# ---------------------------------------------------------------------------
# Placements against the coarse public slot

@pytest.mark.parametrize('sid', ids())
def test_every_placement_sits_in_the_slot_the_public_part_promises(sid):
    scenario = E.load(sid)
    map_bundle = bundle(scenario)
    data, _ = load_map(scenario['map_id'])
    from harness.zone_map_schematic import pickup_bays
    slots = {s['slot_id']: s for bay in pickup_bays(data) for s in bay['slots']}
    orders = {o['order_id']: o for o in scenario['orders']}
    setup = E.private_part(scenario)['setup']
    assert setup['weld'] == 'off'
    assert setup['arena_variant'] == map_bundle['public_map']['base_map_id']
    counted = {}
    for placement in setup['placements']:
        order = orders[placement['order_id']]
        assert placement['slot'] == order['initial_location']['slot']
        assert placement['kind'] == order['kind']
        slot = slots[placement['slot']]
        pose = placement['pose_m']
        half = E.half_footprint(placement['kind'], pose[2])
        for axis in (0, 1):
            assert abs(pose[axis] - slot['center_m'][axis]) + half[axis] \
                <= slot['half_extents_m'][axis] - E.SLOT_MARGIN_M + 1e-9, \
                f'{placement["item_id"]} leaves {placement["slot"]} on axis {axis}'
        counted[placement['order_id']] = counted.get(placement['order_id'], 0) + 1
    for order_id, order in orders.items():
        assert counted[order_id] == order['count']
        if order['identity'] == 'specific_item':
            placed = {p['item_id'] for p in setup['placements'] if p['order_id'] == order_id}
            assert placed == set(order['item_ids'])


@pytest.mark.parametrize('mutation,expected', [
    (lambda d: d['eval']['setup']['placements'][0].update(pose_m=[1.6, .75, 0.]), 'is not inside'),
    (lambda d: d['eval']['setup']['placements'][1].update(pose_m=[-.15, .75, 0.]), 'within 0.2 m'),
    (lambda d: d['eval']['setup'].update(weld='on'), 'weld'),
    (lambda d: d['eval']['setup'].update(map_file_sha256='0' * 64), 'map_file_sha256'),
    (lambda d: d['eval']['setup'].update(arena_variant='zone_wide'), 'not the physical map'),
    (lambda d: d['eval']['setup'].update(contact_profile='fast'), 'contact_profile'),
    (lambda d: d['orders'][0].update(count=3), 'the private setup places'),
    (lambda d: d['eval']['setup']['placements'][0].update(slot='P2-2'), 'differs from the public'),
])
def test_a_broken_placement_is_rejected(mutation, expected):
    scenario = E.load('s2_unmapped_blockage')
    bad = copy.deepcopy(scenario)
    mutation(bad)
    report = E.validate(bad, bundle=bundle(scenario))
    assert not report.ok
    assert any(expected in problem for problem in report.problems), report.problems


# ---------------------------------------------------------------------------
# Leader rotation and map provenance

@pytest.mark.parametrize('sid', ids())
def test_leader_rotation_covers_every_robot(sid):
    scenario = E.load(sid)
    rotation = E.leader_rotation(scenario['seeds'])
    assert set(rotation.values()) == set(ROBOTS), rotation
    for seed, leader in rotation.items():
        assert leader == leader_for_seed('leader_ko', seed)
        edges = allowed_edges('leader_ko', seed)
        followers = [r for r in ROBOTS if r != leader]
        assert (followers[0], followers[1]) not in edges, 'hub-and-spoke: no follower<->follower edge'
        assert (leader, followers[0]) in edges and (followers[1], leader) in edges
    assert E.check_leader_rotation(scenario) == []


def test_seeds_that_never_rotate_are_rejected():
    scenario = E.load('s1_normal_mixed')
    bad = copy.deepcopy(scenario)
    bad['seeds'] = [601, 604, 607]
    report = E.validate(bad, bundle=bundle(scenario))
    assert not report.ok
    assert any('leader' in problem for problem in report.problems)


@pytest.mark.parametrize('sid', ids())
def test_map_pin_matches_the_map_file_and_has_landmarks(sid):
    scenario = E.load(sid)
    map_bundle = bundle(scenario)
    assert map_bundle['has_landmarks'], 'own-camera localisation needs the AprilTag map variant'
    assert E.private_part(scenario)['setup']['map_file_sha256'] == map_bundle['map_file_sha256']
    data, file_sha = load_map(scenario['map_id'])
    assert file_sha == map_bundle['map_file_sha256']
    assert data['base_map']['map_id'] == E.private_part(scenario)['setup']['arena_variant']


# ---------------------------------------------------------------------------
# Manifest and CLI

@pytest.mark.parametrize('sid', ids())
def test_manifest_records_provenance(sid):
    scenario = E.load(sid)
    manifest = E.validate(scenario, bundle=bundle(scenario)).manifest
    assert manifest['scenario_id'] == sid
    assert manifest['map']['map_file_sha256'] == bundle(scenario)['map_file_sha256']
    assert manifest['order_sheet_sha256'] == digest(order_sheet(scenario, bundle(scenario)))
    assert manifest['private_sha256'] == digest(E.private_part(scenario))
    assert manifest['public_sha256'] == digest(E.public_part(scenario))
    assert manifest['weld'] == 'off'
    assert manifest['leader_rotation'] == E.leader_rotation(scenario['seeds'])
    assert manifest['budget']['sim_seconds'] > 0
    assert manifest['not_verified'], 'the manifest must say what is not verified'
    json.dumps(manifest, ensure_ascii=False)  # the manifest must be JSON-safe


def test_scenario_table_and_cli():
    rows = E.scenario_table()
    assert [row['scenario_id'] for row in rows] == list(EXPECTED)
    assert all(row['tests_ko'] for row in rows), 'every scenario says what it tests, in Korean'
    assert E.main([]) == 0
    assert E.main(['s1_normal_mixed']) == 0


def test_loader_rejects_a_bad_id_and_a_renamed_file(tmp_path):
    with pytest.raises(E.ScenarioError):
        E.scenario_path('../secret')
    path = tmp_path / 'other_name.json'
    path.write_text(json.dumps(E.load('s1_normal_mixed')))
    with pytest.raises(E.ScenarioError):
        E.load('other_name', directory=tmp_path)
    assert E.scenario_ids(directory=tmp_path) == ('other_name',)


def test_validate_reports_instead_of_raising():
    report = E.validate({'schema': 'nope'})
    assert not report.ok
    with pytest.raises(ContractViolation):
        report.raise_for_problems()
