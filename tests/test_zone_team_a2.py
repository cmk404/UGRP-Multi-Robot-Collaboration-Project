"""Zone team A2: mixed episodes, protocol v2 messages, team routes, role binding, protocol dispatch."""
import copy
import json
import math
from types import SimpleNamespace

import pytest

from harness import zone_protocol_v2 as zp2
from harness.static_keepouts import inside_rect, keepout_rects, passage_zones, polygons_overlap
from harness.zone_goal_v2 import formation, goal_counts_v2, landing_layout
from harness.zone_mixed_episode import _occupied, item_table, mixed_episode, split_goal
from harness.zone_perception_v2 import label_items, public_labels, view_from_detections
from harness.zone_team_footprint import TeamFootprint, circle, item_polygons, transform
from harness.zone_team_jobs import (RendezvousRule, RoleClaim, check_independent_claims, normalize_claim,
                                    validate_team_plan)
from harness.zone_team_route import PoseReference, plan_team_route
from sim import zone_arena as za

MIXED = {'A': {'long_beam': 1}, 'B': {'heavy_crate': 1, 'red': 1}, 'C': {'can': 1, 'green': 1, 'tile': 1}}
TRI = {'A': {'tri_frame': 1, 'red': 1}}
ROBOTS = ('r1', 'r2', 'r3')


def _labels(config):
    """RGB-like labels from the setup (test only: the detector is tested elsewhere)."""
    dets = [{'kind': o['kind'], 'floor_xy_m': o['position_m'][:2], 'yaw_rad': None}
            for o in config['setup_only']['objects'].values()]
    dets += [{'kind': c['kind'], 'floor_xy_m': c['pose'][:2], 'yaw_rad': c['pose'][2]} for c in config['cargo_items']]
    return label_items(dets, config['static_map'])


# --- mixed episodes ----------------------------------------------------------

@pytest.mark.parametrize('variant', ['zone_wide', 'zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor'])
@pytest.mark.parametrize('seed', [11, 12, 13, 14])
def test_mixed_episode_places_cargo_clear_of_boxes_spawns_walls_and_door_lanes(variant, seed):
    cfg = mixed_episode(variant, seed, goal=MIXED)
    assert cfg == mixed_episode(variant, seed, goal=MIXED)                  # deterministic
    boxes, cargo = split_goal(cfg['goal'])
    base = za.episode(variant, seed, goal=boxes)
    assert cfg['setup_only'] == base['setup_only'] and cfg['static_map'] == base['static_map']
    assert sorted(i['kind'] for i in cfg['cargo_items']) == sorted(k for k, n in cargo.items() for _ in range(n))
    walls = keepout_rects(cfg['static_map'])
    polys = {i['item_id']: _occupied(i['kind'], tuple(i['pose'])) for i in cfg['cargo_items']}
    for iid, ps in polys.items():
        for o in cfg['setup_only']['objects'].values():
            assert not any(polygons_overlap(p, circle(*o['position_m'][:2], .03)) for p in ps)
        for s in cfg['setup_only']['spawns'].values():
            assert not any(polygons_overlap(p, circle(s[0], s[1], .17)) for p in ps)
        for other, qs in polys.items():
            if other != iid:
                assert not any(polygons_overlap(p, q) for p in ps for q in qs)
        for p in ps:
            for (x, y) in p:
                assert all(not inside_rect((x, y), w) for w in walls)
    for _, core, _ in passage_zones(cfg['static_map']):
        lane = (core[0], core[1], core[2] + 1.2, core[3] + .2, 0.)
        for item in cfg['cargo_items']:
            parts = [transform(q, tuple(item['pose'])) for q in item_polygons(item['kind'])]
            assert all(not inside_rect(pt, lane) for q in parts for pt in q)


def test_mixed_episode_rejects_colour_only_unless_asked_and_needs_a_box():
    with pytest.raises(ValueError):
        mixed_episode('zone_wide', 11, goal={'A': {'red': 1}})
    cfg = mixed_episode('zone_wide', 11, goal={'A': {'red': 1}}, colour_only_ok=True)
    assert cfg['cargo_items'] == [] and cfg['setup_only'] == za.episode('zone_wide', 11, goal={'A': {'red': 1}})['setup_only']
    with pytest.raises(ValueError, match='at least one colour box'):
        mixed_episode('zone_wide', 11, goal={'A': {'long_beam': 1}})
    table = item_table(mixed_episode('zone_wide_door', 11, goal=MIXED))
    assert {v['carriers'] for k, v in table.items() if k.startswith('long_beam')} == {2}


def test_scene_matches_the_mixed_episode_and_uses_the_noslip_profile():
    from harness.zone_mixed_episode import scene_for
    cfg = mixed_episode('zone_wide_two_doors', 12, goal=MIXED)
    scene = scene_for(cfg)
    assert scene.scene['cargo_contact_profile'] == 'cargo_noslip_v1'
    assert scene.scene['contact_profile'] == 'local_contact_fine'
    assert [c.item_id for c in scene.cargo] == [c['item_id'] for c in cfg['cargo_items']]


# --- protocol v2: messages ---------------------------------------------------

def _task_and_frame(variant='zone_wide_two_doors'):
    cfg = mixed_episode(variant, 11, goal=MIXED)
    task = zp2.actor_task_v2(cfg['static_map'], cfg['goal'])
    views = za.top_views(cfg['static_map'])
    frame = {'own': b'o', **{v[2]: v[0].encode() for v in views}}
    return cfg, task, views, frame


def test_static_task_text_is_identical_in_every_mode_and_has_no_setup_poses_or_area_ids():
    cfg, task, views, frame = _task_and_frame()
    labels = _labels(cfg)
    view = {'pickup_items_still_visible': sorted(labels), 'zone_counts_seen': {'A': {}, 'B': {}, 'C': {}}}
    systems = {}
    for mode in ('plan_first', 'dynamic', 'independent'):
        ctx = zp2.context('r1', labels=labels, view=view, own_jobs=[])
        req = zp2.build_request(mode, 'r1', request_id='q', task=task, frame=frame, ctx=ctx, views=views,
                                agreement={} if mode == 'plan_first' else None)
        systems[mode] = req['messages'][0]['content']
        assert len(req['images']) == 5
    tails = {m: s[s.index('\nStatic task: '):] for m, s in systems.items()}
    assert len(set(tails.values())) == 1
    tail = tails['dynamic']
    assert zp2.TEAM_RULE_TEXT in tail and 'Static map: ' in tail and 'door_narrow' in tail
    assert 'area_id' not in json.dumps(task) and 'A-long_beam-1' not in json.dumps(task)
    for item in cfg['cargo_items']:
        assert f"{item['pose'][0]:.2f}, {item['pose'][1]:.2f}" not in tail
    assert 'long_beam: 2 robots at once, roles end_neg, end_pos' in tail
    # colour-only goals need an explicit protocol v2 opt-in
    with pytest.raises(ValueError):
        zp2.actor_task_v2(cfg['static_map'], {'A': {'red': 1}})
    assert zp2.actor_task_v2(cfg['static_map'], {'A': {'red': 1}}, allow_colour_only=True)['goal'] == {'A': {'red': 1}}


def test_own_jobs_hide_landing_ids_and_independent_requests_ignore_peers():
    cfg, task, views, frame = _task_and_frame()
    labels = _labels(cfg)
    view = {'pickup_items_still_visible': sorted(labels), 'zone_counts_seen': {'A': {}, 'B': {}, 'C': {}}}
    jobs = [{'item': 'red-1', 'zone': 'B', 'role': 'west', 'status': 'issued', 'issued_at_sim_s': 3.,
             'area_id': 'B-red-1', 'job_id': 'x'}]
    assert zp2.visible_own_jobs(jobs) == [{k: jobs[0][k] for k in zp2.OWN_JOB_KEYS}]
    ctx = zp2.context('r1', labels=labels, view=view, own_jobs=jobs)
    assert 'team_board' not in ctx and 'peer_messages' not in ctx
    base = zp2.build_request('independent', 'r1', request_id='q', task=task, frame=frame, ctx=ctx, views=views)
    # the independent path never passes a board or inbox; a peer's state cannot enter
    again = zp2.build_request('independent', 'r1', request_id='q', task=task, frame=frame,
                              ctx=zp2.context('r1', labels=labels, view=view, own_jobs=copy.deepcopy(jobs)),
                              views=views)
    assert base == again
    assert zp2.CONDITIONS['independent'] == {'peer_board': False, 'host_arbitration': False,
                                             'conflict_notices': False, 'wake_on_peer_job_end': False,
                                             'peer_messages': False}
    public = public_labels(labels)
    assert all('approach_base_xyyaw' not in json.dumps(v) for v in public.values())


def test_reply_validators_accept_role_claims_and_reject_bad_forms():
    ok = {'request_id': 'q', 'claim': {'item': 'long_beam-1', 'zone': 'A', 'role': 'end_neg'}, 'reason': 'r',
          'message': 'm'}
    assert zp2.validate_claim_reply(json.dumps(ok), 'q')['claim'] == ok['claim']
    idle = {**ok, 'claim': {'item': None, 'zone': None, 'role': None}}
    assert zp2.validate_claim_reply(json.dumps(idle), 'q')
    for bad in ({**ok, 'claim': {'box': 'red-1', 'zone': 'A'}}, {**ok, 'claim': {'item': 'x', 'zone': 'D', 'role': None}},
                {**ok, 'claim': {'item': None, 'zone': 'A', 'role': None}}, {**ok, 'request_id': 'z'}):
        with pytest.raises(ValueError):
            zp2.validate_claim_reply(json.dumps(bad), 'q')
    solo = {k: v for k, v in ok.items() if k != 'message'}
    assert zp2.validate_solo_reply(json.dumps(solo), 'q')
    with pytest.raises(ValueError):
        zp2.validate_solo_reply(json.dumps(ok), 'q')          # no message field without communication


def test_fixtures_fill_team_roles_and_the_plan_passes_the_team_plan_validator():
    for goal, variant in ((MIXED, 'zone_wide_two_doors'), (TRI, 'zone_wide_two_doors')):
        cfg = mixed_episode(variant, 11, goal=goal)
        labels = _labels(cfg)
        goal = cfg['goal']
        view = {'pickup_items_still_visible': sorted(labels), 'zone_counts_seen': {'A': {}, 'B': {}, 'C': {}}}
        plan = zp2.fixture_plan(goal, labels)
        norm = validate_team_plan(plan, goal, labels)
        firsts = [norm['assignments'][r][0]['item'] for r in ROBOTS if norm['assignments'][r]]
        team_item = next(i for i in labels if labels[i]['kind'] in ('long_beam', 'tri_frame'))
        assert firsts.count(team_item) == len(formation(labels[team_item]['kind']))
        # independent convention: the three robots' own claims fill the first team item
        claims = {r: zp2.fixture_solo_claim(r, 'q', goal, labels, view)['claim'] for r in ROBOTS}
        out = check_independent_claims(claims, goal=goal, labels=labels, view=view)
        roles = {c.role for c in out['accepted'].values() if c.item == team_item}
        assert roles == set(formation(labels[team_item]['kind']))
        # dynamic: joins first
        active = {'r1': normalize_claim('r1', claims['r1'], labels)}
        join = zp2.fixture_claim('r2', 'q', goal, labels, view, active, ('r2', 'r3'))['claim']
        assert join['item'] == claims['r1']['item'] and join['role'] != claims['r1']['role']


# --- team routes ---------------------------------------------------------------

def _route(kind, variant, start, goal_pose, yaws):
    static = za.authored_map(variant)
    fp = TeamFootprint(kind, formation(kind))
    return plan_team_route(fp, start, goal_pose, rects=keepout_rects(static), bounds=static['bounds_m'],
                           goal_yaws=yaws), static


def test_beam_and_crate_pass_the_narrow_door_and_never_turn_inside_it():
    for kind, start in (('long_beam', (.7, -.15, math.pi/2)), ('heavy_crate', (.4, -1.35, 0.))):
        lay = landing_layout({'A': {kind: 1}}, 'zone_wide_door')['A'][0]
        r, static = _route(kind, 'zone_wide_door', start, lay['item_pose'],
                           [lay['item_pose'][2], lay['item_pose'][2] + math.pi])
        assert r['ok'], r['reason']
        assert all(c['clear_walls_bounds'] for c in r['checks'])
        door = passage_zones(static)[0][1]
        for a, b in zip(r['poses'], r['poses'][1:]):
            if math.dist(a[:2], b[:2]) < 1e-9:                         # an in-place turn
                assert not inside_rect(a[:2], door, grow=.6)
        assert any(a[0] < 2.2 < b[0] for a, b in zip(r['poses'], r['poses'][1:]))


def test_tri_frame_cannot_pass_a_half_metre_door_but_passes_the_wide_door():
    lay = landing_layout(TRI, 'zone_wide_door')['A'][0]
    yaws = [lay['item_pose'][2] + k*2*math.pi/3 for k in range(3)]
    r, _ = _route('tri_frame', 'zone_wide_door', (.1, -2.55, math.pi/2), lay['item_pose'], yaws)
    assert not r['ok'] and 'no collision-free route' in r['reason']
    r, _ = _route('tri_frame', 'zone_wide_two_doors', (.1, -2.55, math.pi/2), lay['item_pose'], yaws)
    assert r['ok']
    assert any(-3.2 < a[1] < -2.0 and a[0] < 2.2 < b[0] for a, b in zip(r['poses'], r['poses'][1:]))


def test_pose_reference_moves_then_turns_at_the_probe_speeds():
    ref = PoseReference([(0., 0., 0.), (1., 0., 0.), (1., 0., math.pi/2)])
    assert ref.total_s == pytest.approx(1/.05 + (math.pi/2)/.2)
    ref.advance(10.)
    assert ref.pose[0] == pytest.approx(.5) and ref.velocity() == pytest.approx((.05, 0., 0.))
    ahead = ref.lookahead(6.)
    assert ahead[-1][0] > ref.pose[0] and ref.pose[0] == pytest.approx(.5)
    ref.advance(100.)
    assert ref.finished and ref.pose == (1., 0., math.pi/2)


# --- executor: binding by position ------------------------------------------------

def _fake_executor(items):
    from scripts.zone_team_teacher import ZoneTeamExecutor
    ex = ZoneTeamExecutor.__new__(ZoneTeamExecutor)
    poses = {iid: pose for iid, (_, pose) in items.items()}
    def body(name):
        iid = name[len('cargo_'):]
        x, y, yaw = poses[iid]
        return SimpleNamespace(xpos=(x, y, 0.), xquat=(math.cos(yaw/2), 0., 0., math.sin(yaw/2)))
    ex.world = SimpleNamespace(data=SimpleNamespace(body=body))
    ex.items = {iid: {'kind': kind, 'body_name': 'cargo_' + iid, 'carriers': 2} for iid, (kind, _) in items.items()}
    ex.ledger = SimpleNamespace(delivered_items={})
    return ex


def test_roles_bind_to_the_physical_handle_nearest_the_rgb_handle_not_by_name():
    # The beam lies with yaw 90 deg; the RGB estimate reports it rotated by 180 deg
    # (symmetric), so RGB 'end_neg' is physically the 'end_pos' handle.
    ex = _fake_executor({'long_beam_0': ('long_beam', (1., -1., math.pi/2))})
    from harness.zone_cargo_perception import grasp_handles
    handles = grasp_handles({'kind': 'long_beam', 'floor_xy_m': [1.005, -1.002], 'yaw_rad': -math.pi/2})
    label = {'kind': 'long_beam', 'floor_xy_m': [1.005, -1.002],
             'handles': {h['role']: {'grip_xy_m': h['grip_xyz_m'][:2]} for h in handles['handles']}}
    labels = {'long_beam-1': label}
    a = ex.bind('r1', RoleClaim('r1', 'long_beam-1', 'long_beam', 'A', 'end_neg'), labels)
    b = ex.bind('r2', RoleClaim('r2', 'long_beam-1', 'long_beam', 'A', 'end_pos'), labels)
    assert a == ('long_beam_0', 'end_pos') and b == ('long_beam_0', 'end_neg')
    # Two robots naming the same RGB handle bind to the same station (then one is blocked there).
    c = ex.bind('r3', RoleClaim('r3', 'long_beam-1', 'long_beam', 'A', 'end_neg'), labels)
    assert c == a


def test_label_view_tracks_items_by_kind_and_counts_zone_items():
    static = za.authored_map('zone_wide')
    dets = [{'kind': 'long_beam', 'floor_xy_m': [.7, -.15], 'yaw_rad': 1.57},
            {'kind': 'red', 'floor_xy_m': [1.6, -2.45], 'yaw_rad': None}]
    labels = label_items(dets, static)
    assert set(labels) == {'long_beam-1', 'red-1'} and set(labels['long_beam-1']['handles']) == {'end_neg', 'end_pos'}
    moved = [{'kind': 'long_beam', 'floor_xy_m': [4.6, .4]}, {'kind': 'red', 'floor_xy_m': [1.61, -2.45]}]
    view = view_from_detections(moved, static, labels)
    assert view['pickup_items_still_visible'] == ['red-1'] and view['zone_counts_seen']['A'] == {'long_beam': 1}


# --- protocol dispatch -------------------------------------------------------------

def test_protocol_selection_and_v1_defaults():
    from scripts.run_zone_dispatch import V1_DEFAULTS, parser, protocol_of
    args = parser().parse_args(['--output', 'x'])
    assert protocol_of(args)[0] == 'v1' and args.max_sim_s is None and args.contact_profile is None
    assert V1_DEFAULTS == {'max_sim_s': 900., 'contact_profile': 'local_contact_fine'}
    args = parser().parse_args(['--output', 'x', '--goal', json.dumps(MIXED), '--variant', 'zone_wide_door'])
    assert protocol_of(args) == ('v2', goal_counts_v2(MIXED, 'zone_wide_door'))
    args = parser().parse_args(['--output', 'x', '--protocol', 'v2'])
    assert protocol_of(args)[0] == 'v2'
    args = parser().parse_args(['--output', 'x', '--protocol', 'v1', '--goal', json.dumps(MIXED)])
    with pytest.raises(SystemExit):
        protocol_of(args)
    from scripts.zone_dispatch_v2 import DEFAULT_MAX_SIM_S
    assert DEFAULT_MAX_SIM_S == 1800.


def test_team_executor_never_touches_weld_or_equality():
    import inspect
    from scripts import zone_dispatch_v2, zone_team_teacher
    from harness import zone_team_route
    for module in (zone_team_teacher, zone_dispatch_v2, zone_team_route):
        src = inspect.getsource(module)
        assert 'weld' not in src.replace('weld OFF', '').replace('No weld', '').replace('weld off', '').lower() \
            or 'activate' not in src
        assert 'eq_active[' not in src and 'eq_data' not in src


def _gate_robot(pose, path):
    from scripts.zone_team_teacher import TeamRobot
    robot = TeamRobot.__new__(TeamRobot)
    held, events = [], []
    robot.rid, robot.phase = 'rX', 'to_box'
    robot.passages = passage_zones(za.authored_map('zone_wide_two_doors'))
    robot.pose = lambda: pose
    robot.path = path
    robot.port = SimpleNamespace(hold=lambda now: held.append(now))
    robot.log = lambda ev, rid, now, **kw: events.append((ev, now, kw))
    robot.gate, robot.gate_passed, robot.peer_moved = None, set(), []
    return robot, held, events


def test_single_lane_gate_holds_for_the_opening_and_a_nearer_moving_robot_only():
    from scripts.zone_teacher import PeerDisc
    # zone_wide_two_doors door_narrow: opening at x=2.2, y in [-0.2, 0.3]; going west.
    path = [(2.1, .05), (1.5, -.5)]
    robot, held, events = _gate_robot((2.55, .05, math.pi), path)
    assert robot.passage_gate(0., []) is False                      # nobody near: go
    in_door = [PeerDisc((2.21, .10, .14))]
    assert robot.passage_gate(1., in_door) is True and held == [1.]
    assert events[-1][0] == 'passage_gate' and events[-1][2]['reason'] == 'opening_occupied'
    assert robot.passage_gate(2., []) is False and events[-1][2]['reason'] == 'clear'
    # A nearer robot moving toward the opening goes first; a still one does not hold us.
    robot, held, events = _gate_robot((2.60, .10, math.pi), path)
    assert robot.passage_gate(0., [PeerDisc((2.45, .0, .14))]) is True
    assert robot.passage_gate(.5, [PeerDisc((2.40, .0, .14))]) is True
    assert robot.passage_gate(1., [PeerDisc((2.40, .0, .14))]) is True   # still < 1 s
    assert robot.passage_gate(2.6, [PeerDisc((2.40, .0, .14))]) is False  # still: not entering
    # The farther robot never holds the nearer one, and the hold is capped.
    robot, held, events = _gate_robot((2.45, .0, math.pi), path)
    assert robot.passage_gate(0., [PeerDisc((2.60, .10, .14))]) is False
    robot, held, events = _gate_robot((2.55, .05, math.pi), path)
    assert robot.passage_gate(0., in_door) is True
    assert robot.passage_gate(61., in_door) is False and events[-1][2]['reason'] == 'limit'
    assert robot.passage_gate(62., in_door) is False               # capped for this leg


def test_single_lane_gate_ignores_paths_that_do_not_enter_the_opening():
    from scripts.zone_teacher import PeerDisc
    robot, held, events = _gate_robot((2.55, .05, math.pi), [(2.6, -.6)])
    assert robot.passage_gate(0., [PeerDisc((2.21, .10, .14))]) is False and not held


# --- review fixes (Codex review of #169, 2026-09-25) -------------------------------

def test_tile_role_contract_accepts_either_side_and_never_two_sides_of_one_tile():
    from harness.zone_goal_v2 import claim_roles, fills_formation, formation, formations
    from harness.zone_team_jobs import check_dynamic_claims, normalize_claim
    assert formations('tile') == (('west',), ('east',)) and claim_roles('tile') == ('west', 'east')
    assert formation('tile') == ('west',) and fills_formation('tile', ['east']) and not fills_formation(
        'tile', ['east', 'west'])
    assert claim_roles('long_beam') == ('end_neg', 'end_pos') and claim_roles('red') == ('west',)
    labels = {'tile-1': {'kind': 'tile', 'floor_xy_m': [.4, .2]}}
    assert normalize_claim('r1', {'item': 'tile-1', 'zone': 'C', 'role': 'east'}, labels).role == 'east'
    with pytest.raises(ValueError):
        normalize_claim('r1', {'item': 'tile-1', 'zone': 'C', 'role': 'north'}, labels)
    view = {'pickup_items_still_visible': ['tile-1'], 'zone_counts_seen': {}}
    out = check_dynamic_claims({'r1': {'item': 'tile-1', 'zone': 'C', 'role': 'west'},
                                'r2': {'item': 'tile-1', 'zone': 'C', 'role': 'east'}},
                               goal={'C': {'tile': 1}}, labels=labels, view=view, active={})
    assert list(out['accepted']) == ['r1']
    assert out['collisions'] == [{'kind': 'formation_conflict', 'item': 'tile-1', 'roles': ['east', 'west'],
                                  'robots': ['r1', 'r2']}]
    # Rendezvous: both sides of one tile occupied -> no team (no id tie-break); one side -> a team.
    rule = RendezvousRule()
    w = RoleClaim('r1', 'tile-1', 'tile', 'C', 'west')
    e = RoleClaim('r2', 'tile-1', 'tile', 'C', 'east')
    assert rule.teams({'r1': w, 'r2': e}, {'r1': 'at_station', 'r2': 'at_station'}) == []
    assert rule.teams({'r2': e}, {'r2': 'at_station'}) == [{'r2': e}]
    # A claimed side is complete: the other side is never offered as a join slot.
    slots = zp2.open_slots({'C': {'tile': 1}}, labels, view, {'r2': e}, [])
    assert all(sl['item'] != 'tile-1' for sl in slots)
    text = zp2.task_static_text({'C': {'tile': 1, 'green': 1}}, za.authored_map('zone_wide'))
    assert 'tile: 1 robot, role west or east' in text


def test_tile_binds_by_approach_direction_and_lands_on_the_layout_station():
    from harness.zone_cargo_perception import grasp_handles
    from harness.zone_team_jobs import station_pose
    from scripts.zone_team_teacher import landing_pose_for_role
    ex = _fake_executor({'tile_0': ('tile', (.4, .2, .2))})
    ex.items['tile_0']['carriers'] = 1
    for rgb_yaw, expect in ((.2, {'west': 'west', 'east': 'east'}), (.2 + math.pi, {'west': 'east', 'east': 'west'})):
        hs = grasp_handles({'kind': 'tile', 'floor_xy_m': [.401, .199], 'yaw_rad': rgb_yaw})['handles']
        label = {'kind': 'tile', 'floor_xy_m': [.401, .199],
                 'handles': {h['role']: {'grip_xy_m': h['grip_xyz_m'][:2],
                                         'approach_base_xyyaw': h['approach_base_xyyaw']} for h in hs}}
        for role, phys in expect.items():
            assert ex.bind('r1', RoleClaim('r1', 'tile-1', 'tile', 'C', role), {'tile-1': label}) == ('tile_0', phys)
    area = {'item_pose': [4.6, -1.9, .3]}
    west = station_pose(area['item_pose'], 'tile', 'west')
    east = station_pose(landing_pose_for_role('tile', area, 'east'), 'tile', 'east')
    assert all(abs(a - b) < 1e-9 for a, b in zip(west, east))
    assert landing_pose_for_role('long_beam', area, 'end_pos') == tuple(area['item_pose'])


def test_condition_switches_are_per_mechanism_and_overrides_must_be_read():
    assert set(zp2.SWITCHES) == set(zp2.CONDITIONS['dynamic']) == set(zp2.CONDITIONS['independent'])
    sw = zp2.condition_switches('dynamic', {'peer_board': False, 'conflict_notices': False})
    assert sw == dict(zp2.CONDITIONS['dynamic'], peer_board=False, conflict_notices=False)
    assert zp2.condition_switches('plan_first', {'peer_messages': False})['peer_messages'] is False
    for mode, bad in (('plan_first', {'peer_board': True}), ('independent', {'peer_messages': True}),
                      ('dynamic', {'nope': True}), ('dynamic', {'peer_board': 1})):
        with pytest.raises(ValueError):
            zp2.condition_switches(mode, bad)
    import inspect
    from scripts import zone_dispatch_v2
    src = inspect.getsource(zone_dispatch_v2)
    for name in zp2.SWITCHES:                     # every switch is read by the driver
        assert f"switches['{name}']" in src


def test_robot_facing_results_are_separate_from_the_teacher_ledger():
    from harness.zone_outcomes_v2 import RobotResults, TeacherReceiptSource
    from harness.zone_team_jobs import RECEIPT_FINISHED
    src = TeacherReceiptSource()
    assert src.name == 'teacher_receipt_L4' and 'L4' in src.label and 'never a student' in src.label
    res = RobotResults(('r1', 'r2', 'r3'), src)
    beam = [RoleClaim(r, 'long_beam-1', 'long_beam', 'A', role) for r, role in (('r1', 'end_neg'), ('r2', 'end_pos'))]
    for c in beam:
        res.issue(c.robot, c, 1.)
    for c in beam:
        res.end(c.robot, SimpleNamespace(receipt=RECEIPT_FINISHED, role_claim=c), 90.)
    assert res.delivered() == [{'zone': 'A', 'kind': 'long_beam'}]         # once per item
    tile = RoleClaim('r3', 'tile-1', 'tile', 'C', 'east')
    res.issue('r3', tile, 2.)
    out = res.end('r3', SimpleNamespace(receipt='executor stopped before finishing', role_claim=tile), 40.)
    assert not out['delivered'] and res.own_jobs['r3'][-1]['status'] == 'executor stopped before finishing'
    assert res.board({})['stopped_reports'][0]['item'] == 'tile-1'
    assert res.record()['outcome_source']['name'] == 'teacher_receipt_L4'
    # A source that has not confirmed a result never enters the delivered list.
    class Unconfirmed:
        def outcome(self, ended, t):
            return {'status': 'unconfirmed', 'delivered': True, 'confirmed': False, 'source': 'rgb'}
        def record(self):
            return {'name': 'rgb'}
    res2 = RobotResults(('r1',), Unconfirmed())
    res2.issue('r1', tile, 1.)
    res2.end('r1', SimpleNamespace(receipt=RECEIPT_FINISHED, role_claim=tile), 5.)
    assert res2.delivered() == [] and res2.own_jobs['r1'][-1]['status'] == 'unconfirmed'
    import inspect
    from scripts import zone_dispatch_v2
    src_text = inspect.getsource(zone_dispatch_v2.run_v2)
    assert 'ex.ledger.delivered_items' not in src_text and 'c.receipt ==' not in src_text


def test_protocol_v2_requires_an_explicit_contact_profile():
    from scripts.run_zone_dispatch import main
    with pytest.raises(SystemExit, match='explicit --contact-profile'):
        main(['--output', 'unused-never-created', '--goal', json.dumps(MIXED), '--variant', 'zone_wide_door',
              '--mode', 'fixture'])
