"""Zone team jobs (phase A1): goal v2, landing/referee v2, TeamJob, rendezvous rule, formation, footprint."""
from __future__ import annotations

import copy
import itertools
import math
import random

import pytest

from harness import zone_goal_v2 as gv
from harness import zone_team_jobs as tj
from harness.zone_team_footprint import (TeamFootprint, convex_hull, item_polygons, point_in_convex, team_footprint,
                                         transform)
from harness.zone_team_formation import FormationPlan, reference
from sim.zone_arena import COLORS, authored_map, goal_counts
from sim.zone_cargo import CATALOGUE, CargoInstance, world_grasps

ROBOTS = ('r1', 'r2', 'r3')
PAIR_TRIO = ('long_beam', 'heavy_crate', 'tri_frame')


def _sat_overlap(a, b):
    for poly in (a, b):
        for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
            ax, ay = -(y1-y0), x1-x0
            pa = [ax*x + ay*y for x, y in a]
            pb = [ax*x + ay*y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


# ---------------------------------------------------------------------------
# Goal v2

def _random_legacy_goal(rng):
    goal = {}
    for zone in rng.sample(['A', 'B', 'C', 'D'] if rng.random() < .1 else ['A', 'B', 'C'], rng.randint(0, 3)):
        kinds = {}
        for kind in rng.sample(sorted(COLORS) + (['purple'] if rng.random() < .1 else []), rng.randint(0, 3)):
            kinds[kind] = rng.choice([0, 1, 1, 2, 3, -1, True, 1.0])
        goal[zone] = kinds
    return goal


@pytest.mark.parametrize('variant', ['zone_open', 'zone_wide'])
def test_old_colour_goals_validate_exactly_as_before(variant):
    rng = random.Random(7)
    checked = accepted = 0
    for _ in range(600):
        goal = _random_legacy_goal(rng)
        try:
            old = goal_counts(copy.deepcopy(goal))
        except ValueError as exc:
            with pytest.raises(ValueError) as new:
                gv.goal_counts_v2(copy.deepcopy(goal), variant)
            assert str(new.value) == str(exc)
        else:
            assert gv.goal_counts_v2(copy.deepcopy(goal), variant) == old
            accepted += 1
        checked += 1
    assert accepted > 15 and checked == 600
    # The documented reference goal is unchanged.
    ref = {'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}
    assert gv.goal_counts_v2(ref, variant) == goal_counts(ref)


def test_v2_goal_accepts_catalogue_kinds_and_checks_capacity():
    goal = gv.goal_counts_v2({'C': {'heavy_crate': 1, 'red': 1}, 'A': {'long_beam': 1}, 'B': {'tri_frame': 1}})
    assert list(goal) == ['A', 'B', 'C'] and goal['C'] == {'heavy_crate': 1, 'red': 1}
    with pytest.raises(ValueError, match='zone_wide'):
        gv.goal_counts_v2({'A': {'can': 1}}, 'zone_open')
    with pytest.raises(ValueError, match='cannot hold'):
        gv.goal_counts_v2({'A': {'long_beam': 1, 'can': 1}})
    with pytest.raises(ValueError, match='cannot hold'):
        gv.goal_counts_v2({'A': {'tri_frame': 2}})
    with pytest.raises(ValueError, match='kinds among'):
        gv.goal_counts_v2({'A': {'can': 1, 'anvil': 1}})
    with pytest.raises(ValueError, match='positive'):
        gv.goal_counts_v2({'A': {'can': 0}})
    with pytest.raises(ValueError, match='colour boxes'):
        gv.goal_counts_v2({'A': {'can': 1, 'red': 4}})
    assert gv.required_carriers('long_beam') == 2 and gv.required_carriers('tri_frame') == 3
    assert gv.required_carriers('red') == 1 and gv.formation('red') == ('west',)
    assert gv.goal_units({'A': {'can': 2}, 'B': {'red': 1}}) == [('A', 'can'), ('A', 'can'), ('B', 'red')]


def test_landing_layout_areas_fit_and_do_not_overlap():
    static = authored_map('zone_wide')
    rng = random.Random(3)
    goals = [{'A': {k: 1}} for k in gv.ALL_KINDS] + [{'B': {'tri_frame': 1, 'can': 1}},
                                                     {'C': {'heavy_crate': 2, 'red': 1, 'tile': 1}}]
    for _ in range(200):
        zone = rng.choice('ABC')
        kinds = {k: rng.randint(1, 2) for k in rng.sample(gv.ALL_KINDS, rng.randint(1, 3))}
        goals.append({zone: kinds})
    feasible = 0
    for goal in goals:
        try:
            goal = gv.goal_counts_v2(goal)
        except ValueError:
            continue
        feasible += 1
        for zone, areas in gv.landing_layout(goal).items():
            region = static['regions']['zone_'+zone]
            (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
            assert sum(a['kind'] == k for a in areas for k in [a['kind']]) == sum(goal[zone].values())
            spans = sorted((a['envelope_m'][2], a['envelope_m'][3]) for a in areas)
            for (a0, a1), (b0, b1) in zip(spans, spans[1:]):
                assert a1 <= b0 + 1e-9, 'team envelopes of two bays overlap'
            for a in areas:
                (lx, ly), (lhx, lhy) = a['landing_center_m'], a['landing_half_extents_m']
                assert abs(lx-cx) + lhx <= hx + 1e-9 and abs(ly-cy) + lhy <= hy + 1e-9
                x0, x1, y0, y1 = static['bounds_m']
                e = a['envelope_m']
                assert x0 < e[0] and e[1] < x1 and y0 < e[2] and e[3] < y1
                polys = [transform(p, a['item_pose']) for p in item_polygons(a['kind'])]
                assert all(abs(x-cx) <= hx and abs(y-cy) <= hy for p in polys for x, y in p)
                # stations are the catalogue approach poses of the landing pose
                if a['kind'] in CATALOGUE:
                    wg = world_grasps(CargoInstance('x', a['kind'], tuple(a['item_pose'])))
                    for role, st in a['stations'].items():
                        want = wg[role]['base_xyyaw']
                        assert st[:2] == pytest.approx(list(want[:2]), abs=1e-5)
                        assert abs(math.remainder(st[2]-want[2], 2*math.pi)) < 1e-5
    assert feasible > 40
    assert gv.landing_capacity_table()['long_beam']['max_per_zone_by_length'] == 1


def test_referee_v2_counts_full_footprints_only():
    static = authored_map('zone_wide')
    cx, cy = static['regions']['zone_A']['center_m']
    items = {
        'beam_in': {'kind': 'long_beam', 'pose': [cx, cy, math.pi/2], 'min_z_m': .0, 'tilt_deg': .5},
        # centre inside zone A, but the beam lies across it and sticks out: not counted
        'beam_across': {'kind': 'long_beam', 'pose': [cx+.1, cy+.5, 0.], 'min_z_m': .0, 'tilt_deg': .3},
        'crate_held': {'kind': 'heavy_crate', 'pose': [cx, cy-.4, 0.], 'min_z_m': .0, 'tilt_deg': 0., 'held': True},
        'red_in': {'kind': 'red', 'pose': [cx, cy-.6, 0.], 'min_z_m': .0, 'tilt_deg': 0.},
        'red_lifted': {'kind': 'red', 'pose': [cx+.1, cy-.6, 0.], 'min_z_m': .05, 'tilt_deg': 0.},
        'frame_out': {'kind': 'tri_frame', 'pose': [1.0, 0., 0.], 'min_z_m': 0., 'tilt_deg': 0.},
    }
    r = gv.referee_v2({'A': {'long_beam': 1, 'red': 1}}, static, items)
    assert r['zone_counts'] == {'A': {'long_beam': 1, 'red': 1}} and r['goal_met']
    assert r['items']['beam_across']['footprint'] == 'straddling'
    assert r['centre_point_counts']['A']['long_beam'] == 2      # the old rule would count the straddling beam
    assert r['items']['crate_held']['resting'] is False
    assert r['items']['frame_out']['footprint'] == 'outside'
    assert 'never robot input' in r['scope']


def test_task_static_info_is_setup_free():
    info = gv.task_static_info({'A': {'long_beam': 1}, 'C': {'red': 1}})
    text = repr(info)
    assert 'setup_only' not in text and 'spawns' not in text and 'objects' not in text
    assert info['kinds']['long_beam'] == {'required_carriers': 2, 'roles': ['end_neg', 'end_pos'],
                                          'formation_id': 'long_beam/end_neg+end_pos'}


# ---------------------------------------------------------------------------
# Footprint

@pytest.mark.parametrize('kind', PAIR_TRIO + ('can', 'red'))
def test_team_footprint_parts_hull_and_stations(kind):
    fp = team_footprint(kind, margin=0.)
    n = gv.required_carriers(kind)
    assert len(fp.parts) == len(item_polygons(kind)) + 2*n
    hull = fp.hull
    assert all(point_in_convex(pt, hull, eps=1e-7) for p in fp.parts for pt in p)
    assert convex_hull(hull) == hull
    # chassis at every station stays off the item (only the arm strip reaches over it)
    for role in fp.roles:
        chassis = fp.carrier_parts[role][0]
        assert not any(_sat_overlap(chassis, p) for p in item_polygons(kind)), role
    if kind in CATALOGUE:
        for g in CATALOGUE[kind].grasps:
            assert fp.stations[g.role] == g.approach_base()
    grown = team_footprint(kind, margin=.05)
    assert grown.radius() > fp.radius()
    with pytest.raises(ValueError):
        TeamFootprint(kind, ('west', 'west'))


def test_team_footprint_is_consumed_by_static_keepouts():
    ko = pytest.importorskip('harness.static_keepouts')     # branch claude/zone-hard-routes (A2)
    from harness.zone_team_footprint import pose_clear, swept_clear
    fp = team_footprint('long_beam')
    wall = (2.0, 0.0, .025, .5, 0.)
    assert pose_clear((1.0, 0., 0.), fp, [wall], keepouts=ko)
    assert not pose_clear((1.6, 0., 0.), fp, [wall], keepouts=ko)
    assert not swept_clear((1.0, 0., 0.), (1.6, 0., 0.), fp, [wall], keepouts=ko)
    # the hull is conservative: whenever the hull is clear, the exact union is clear
    rng = random.Random(1)
    for _ in range(200):
        pose = (rng.uniform(1., 3.), rng.uniform(-1., 1.), rng.uniform(-math.pi, math.pi))
        if pose_clear(pose, fp, [wall], exact=False, keepouts=ko):
            assert pose_clear(pose, fp, [wall], keepouts=ko)


# ---------------------------------------------------------------------------
# TeamJob / ledger

def _claims(item, kind, zone, roles):
    return [tj.RoleClaim(rid, item, kind, zone, role) for rid, role in roles.items()]


def _drive(job, states, t0=0.):
    t = t0
    for state in states:
        for rid in job.participants:
            job.report_ready(rid, t)
        t += 1.
        assert job.advance(t) == state
    return t


def test_team_job_runs_every_state_behind_a_barrier():
    ledger = tj.TeamJobLedger({'A': {'long_beam': 1}})
    job = ledger.commit(_claims('beam-1', 'long_beam', 'A', {'r2': 'end_pos', 'r1': 'end_neg'}), 0.)
    assert job.participants == ('r1', 'r2') and job.role_by_robot == {'r1': 'end_neg', 'r2': 'end_pos'}
    assert job.required_carriers == 2 and job.formation_id == 'long_beam/end_neg+end_pos'
    assert job.spec_hash == tj.spec_hash('beam-1', 'long_beam', 'A')
    job.report_ready('r1', .5)
    assert job.advance(.5) is None and job.barrier.missing == ['r2']
    assert not job.report_ready('r2', .6, generation=job.generation+1)   # stale generation
    assert not job.report_ready('r2', .6, state='CARRY')                 # stale state
    assert job.advance(.6) is None and job.barrier.stale == 2
    job.report_ready('r2', .7)
    assert job.advance(.7) == 'RENDEZVOUS'
    _drive(job, ['PREGRASP', 'CLOSE', 'LIFT', 'CARRY', 'LOWER', 'RELEASE', 'RETREAT', 'FINISHED'], 1.)
    assert job.receipt == tj.RECEIPT_FINISHED
    assert ledger.delivered == {'A': {'long_beam': 1}} and ledger.remaining() == {}
    assert [h['state'] for h in job.history] == list(tj.STATES[:10])
    assert 'r1' not in ledger.robot_job and 'r2' not in ledger.robot_job


@pytest.mark.parametrize('state,action,path', [
    ('COMMITTED', 'cancel_retreat', ['ABORTED']),
    ('RENDEZVOUS', 'cancel_retreat', ['ABORTED']),
    ('PREGRASP', 'cancel_retreat', ['ABORTED']),
    ('CLOSE', 'hold_lower', ['RELEASE', 'RETREAT', 'ABORTED']),
    ('LIFT', 'hold_lower', ['RELEASE', 'RETREAT', 'ABORTED']),
    ('CARRY', 'hold_lower', ['RELEASE', 'RETREAT', 'ABORTED']),
    ('LOWER', 'continue_release', ['RELEASE', 'RETREAT', 'ABORTED']),
    ('RELEASE', 'continue_release', ['RETREAT', 'ABORTED']),
])
def test_partial_failure_rules(state, action, path):
    ledger = tj.TeamJobLedger({'B': {'tri_frame': 1}})
    job = ledger.commit(_claims('frame-1', 'tri_frame', 'B', {'r1': 'v0', 'r2': 'v1', 'r3': 'v2'}), 0.)
    order = list(tj.STATES[:9])
    _drive(job, order[1:order.index(state)+1])
    before = job.participants
    r = job.fail('r3', 'phase_timeout', 50.)
    assert r['action'] == action and r['robots'] == ['r1', 'r2', 'r3']
    if action == 'cancel_retreat':
        assert job.state == 'RETREAT' and not job.contact_made
    if action == 'hold_lower':
        assert job.state == 'LOWER' and job.history[-1]['hold_first']
    _drive(job, path, 60.)
    assert job.state == 'ABORTED' and job.participants == before and job.receipt == tj.RECEIPT_STOPPED
    assert ledger.delivered == {} and ledger.remaining() == {'B': {'tri_frame': 1}}
    # no N-1 continuation: the item can only be carried again by a new, complete commit
    with pytest.raises(tj.CommitRejected):
        ledger.commit(_claims('frame-1', 'tri_frame', 'B', {'r1': 'v0', 'r2': 'v1'}), 70.)
    again = ledger.commit(_claims('frame-1', 'tri_frame', 'B', {'r3': 'v0', 'r1': 'v1', 'r2': 'v2'}), 70.)
    assert again.generation == 2 and again.job_id == 'frame-1@g2'


def test_retreat_failure_ends_the_job_and_fail_needs_a_participant():
    ledger = tj.TeamJobLedger({'A': {'heavy_crate': 1}})
    job = ledger.commit(_claims('crate-1', 'heavy_crate', 'A', {'r1': 'west', 'r3': 'east'}), 0.)
    _drive(job, ['RENDEZVOUS', 'PREGRASP', 'CLOSE', 'LIFT', 'CARRY', 'LOWER', 'RELEASE', 'RETREAT'])
    with pytest.raises(ValueError):
        job.fail('r2', 'x', 9.)
    assert job.fail('r1', 'path_blocked', 9.)['action'] == 'end' and job.state == 'ABORTED'
    assert job.fail('r3', 'late', 10.)['action'] == 'none'
    assert ledger.delivered == {}


@pytest.mark.parametrize('claims,match', [
    ([('r1', 'beam-1', 'A', 'end_neg'), ('r2', 'beam-1', 'B', 'end_pos')], 'disagree'),
    ([('r1', 'beam-1', 'A', 'end_neg'), ('r2', 'beam-1', 'A', 'end_neg')], 'duplicate role'),
    ([('r1', 'beam-1', 'A', 'end_neg'), ('r1', 'beam-1', 'A', 'end_pos')], 'twice'),
    ([('r1', 'beam-1', 'A', 'end_neg')], 'do not fill'),
    ([], 'no claims'),
])
def test_commit_is_transactional(claims, match):
    ledger = tj.TeamJobLedger({'A': {'long_beam': 1}, 'B': {'long_beam': 1}})
    snapshot = copy.deepcopy(ledger.record())
    with pytest.raises(tj.CommitRejected, match=match):
        ledger.commit([tj.RoleClaim(r, i, 'long_beam', z, role) for r, i, z, role in claims], 0.)
    assert ledger.record() == snapshot


def test_no_double_membership_and_one_live_job_per_item():
    ledger = tj.TeamJobLedger({'A': {'long_beam': 1, 'red': 1}})
    ledger.commit(_claims('beam-1', 'long_beam', 'A', {'r1': 'end_neg', 'r2': 'end_pos'}), 0.)
    with pytest.raises(tj.CommitRejected, match='already in a live job'):
        ledger.commit(_claims('red-1', 'red', 'A', {'r2': 'west'}), 1.)
    with pytest.raises(tj.CommitRejected, match='live job'):
        ledger.commit(_claims('beam-1', 'long_beam', 'A', {'r3': 'end_neg', 'r1': 'end_pos'}), 1.)
    assert ledger.commit(_claims('red-1', 'red', 'A', {'r3': 'west'}), 1.).required_carriers == 1
    with pytest.raises(ValueError):
        tj.RoleClaim('r1', 'beam-1', 'long_beam', 'A', 'west')


def test_property_goal_decrements_once_per_item_and_no_double_membership():
    """Random commit / ready / fail sequences keep every ledger invariant."""
    kinds = {'beam-1': 'long_beam', 'beam-2': 'long_beam', 'crate-1': 'heavy_crate', 'frame-1': 'tri_frame',
             'red-1': 'red', 'can-1': 'can'}
    for seed in range(300):
        rng = random.Random(seed)
        goal = {'A': {'long_beam': 1, 'red': 1}, 'B': {'heavy_crate': 1, 'can': 1}, 'C': {'tri_frame': 1}}
        ledger = tj.TeamJobLedger(goal)
        finished_jobs = 0
        t = 0.
        for _ in range(80):
            t += 1.
            op = rng.random()
            live = [j for j in ledger.jobs.values() if not j.terminal]
            if op < .35:
                item = rng.choice(sorted(kinds))
                kind = kinds[item]
                roles = list(gv.formation(kind))
                if rng.random() < .2:
                    roles = roles[:-1] or roles           # an incomplete team now and then
                robots = rng.sample(ROBOTS, len(roles))
                zone = rng.choice('ABC')
                try:
                    ledger.commit([tj.RoleClaim(r, item, kind, zone, role) for r, role in zip(robots, roles)], t)
                except tj.CommitRejected:
                    pass
            elif live and op < .9:
                job = rng.choice(live)
                for rid in job.participants:
                    if rng.random() < .9:
                        job.report_ready(rid, t)
                if job.advance(t) == 'FINISHED':
                    finished_jobs += 1
            elif live:
                job = rng.choice(live)
                job.fail(rng.choice(job.participants), 'random', t)
            # invariants after every step
            owners = {}
            for j in ledger.jobs.values():
                if not j.terminal:
                    for rid in j.participants:
                        assert rid not in owners, f'seed {seed}: {rid} in two live jobs'
                        owners[rid] = j.job_id
                        assert ledger.robot_job[rid] == j.job_id
            assert set(ledger.robot_job) == set(owners)
            live_items = [j.item_label for j in ledger.jobs.values() if not j.terminal]
            assert len(live_items) == len(set(live_items))
            fin = [j for j in ledger.jobs.values() if j.state == 'FINISHED']
            assert len(fin) == len({j.item_label for j in fin}) == finished_jobs
            assert sum(n for z in ledger.delivered.values() for n in z.values()) == finished_jobs
            for j in fin:
                assert len(j.participants) == j.required_carriers
            for zone, kinds_ in goal.items():
                for kind, count in kinds_.items():
                    got = ledger.delivered.get(zone, {}).get(kind, 0)
                    assert ledger.remaining().get(zone, {}).get(kind, 0) == max(0, count-got)


# ---------------------------------------------------------------------------
# Rendezvous rule (team formation, identical in every mode)

ITEM_POSES = {'beam-1': (2.5, .2, 0.), 'frame-1': (1.2, -.8, .3), 'crate-1': (3.5, -1.5, 1.2)}
ITEM_KIND = {'beam-1': 'long_beam', 'frame-1': 'tri_frame', 'crate-1': 'heavy_crate'}


def _random_scene(rng, robots=ROBOTS):
    claims, poses = {}, {}
    for rid in robots:
        item = rng.choice(sorted(ITEM_POSES))
        kind = ITEM_KIND[item]
        role = rng.choice(gv.formation(kind))
        zone = rng.choice('AB')
        if rng.random() < .85:
            claims[rid] = tj.RoleClaim(rid, item, kind, zone, role)
        st = tj.station_pose(ITEM_POSES[item], kind, role)
        if rng.random() < .6:
            poses[rid] = (st[0] + rng.uniform(-.05, .05), st[1] + rng.uniform(-.05, .05), st[2] + rng.uniform(-.2, .2))
        else:
            poses[rid] = (rng.uniform(-1, 5), rng.uniform(-3, 1), rng.uniform(-3, 3))
    return claims, poses


def test_rendezvous_forms_a_team_only_when_every_station_is_held():
    rule = tj.RendezvousRule()
    item, kind = 'frame-1', 'tri_frame'
    claims = {rid: tj.RoleClaim(rid, item, kind, 'B', role) for rid, role in zip(ROBOTS, ('v0', 'v1', 'v2'))}
    at = {rid: tj.station_pose(ITEM_POSES[item], kind, c.role) for rid, c in claims.items()}
    far = dict(at, r3=(0., 0., 0.))
    occ = rule.occupancy(claims, far, ITEM_POSES)
    assert occ == {'r1': 'at_station', 'r2': 'at_station', 'r3': 'en_route'}
    assert rule.teams(claims, occ) == []
    occ = rule.occupancy(claims, at, ITEM_POSES)
    teams = rule.teams(claims, occ)
    assert len(teams) == 1 and {r: c.role for r, c in teams[0].items()} == {'r1': 'v0', 'r2': 'v1', 'r3': 'v2'}
    ledger = tj.TeamJobLedger({'B': {'tri_frame': 1}})
    assert ledger.commit(teams[0].values(), 5.).participants == ROBOTS
    # the same robots with one different destination: no team (mismatch = absence)
    other = dict(claims, r3=tj.RoleClaim('r3', item, kind, 'A', 'v2'))
    assert rule.teams(other, rule.occupancy(other, at, ITEM_POSES)) == []
    assert rule.expired(10., 70.) and not rule.expired(10., 69.9) and not rule.expired(None, 1e9)


def test_rendezvous_station_blocking_is_physical_and_symmetric():
    rule = tj.RendezvousRule()
    item, kind = 'beam-1', 'long_beam'
    st = tj.station_pose(ITEM_POSES[item], kind, 'end_neg')
    claims = {'r1': tj.RoleClaim('r1', item, kind, 'A', 'end_neg'), 'r2': tj.RoleClaim('r2', item, kind, 'A', 'end_neg')}
    poses = {'r1': (st[0]-.02, st[1], st[2]), 'r2': (st[0]-.30, st[1], st[2]), 'r3': (0., 0., 0.)}
    assert rule.occupancy(claims, poses, ITEM_POSES) == {'r1': 'at_station', 'r2': 'station_blocked'}
    poses['r2'] = (st[0]-.9, st[1], st[2])          # far from the item: still just en route
    assert rule.occupancy(claims, poses, ITEM_POSES)['r2'] == 'en_route'
    # an unclaimed robot standing on the station blocks as well (its claim is never read)
    one = {'r2': claims['r2']}
    poses = {'r1': (st[0], st[1], 0.), 'r2': (st[0]-.08, st[1], st[2]), 'r3': (0., 0., 0.)}
    assert rule.occupancy(one, poses, ITEM_POSES) == {'r2': 'station_blocked'}
    # exact tie: both blocked, neither wins by id
    poses = {'r1': (st[0]-.03, st[1], st[2]), 'r2': (st[0]+.03, st[1], st[2]), 'r3': (0., 0., 0.)}
    assert rule.occupancy(claims, poses, ITEM_POSES) == {'r1': 'station_blocked', 'r2': 'station_blocked'}


def test_property_rendezvous_is_symmetric_under_robot_relabelling():
    rule = tj.RendezvousRule()
    for seed in range(400):
        rng = random.Random(seed)
        claims, poses = _random_scene(rng)
        occ = rule.occupancy(claims, poses, ITEM_POSES)
        teams = rule.teams(claims, occ)
        for m in tj.robot_permutations():
            c2, p2 = tj.relabel(m, claims), tj.relabel(m, poses)
            occ2 = rule.occupancy(c2, p2, ITEM_POSES)
            assert occ2 == tj.relabel(m, occ)
            want = sorted([sorted((r, c.role) for r, c in team.items()) for team in tj.relabel(m, teams)])
            got = sorted([sorted((r, c.role) for r, c in team.items()) for team in rule.teams(c2, occ2)])
            assert got == want, seed


def test_property_rendezvous_ignores_private_intentions_of_peers():
    """Robot i's status and team membership never depend on the claim of a peer
    that is not at a station of the same item, and a peer at a station with a
    different spec is indistinguishable from no peer at all."""
    rule = tj.RendezvousRule()
    for seed in range(400):
        rng = random.Random(seed)
        claims, poses = _random_scene(rng)
        if 'r1' not in claims:
            continue
        occ = rule.occupancy(claims, poses, ITEM_POSES)

        def membership(cl, oc):
            return next((sorted(t) for t in rule.teams(cl, oc) if 'r1' in t), None)
        base_status, base_team = occ['r1'], membership(claims, occ)
        for _ in range(5):
            changed = dict(claims)
            for rid in ('r2', 'r3'):
                if occ.get(rid) == 'at_station' and claims[rid].item == claims['r1'].item:
                    continue                                # physically joined: allowed to matter
                item = rng.choice(sorted(ITEM_POSES))
                if rng.random() < .3:
                    changed.pop(rid, None)
                else:
                    changed[rid] = tj.RoleClaim(rid, item, ITEM_KIND[item], rng.choice('ABC'),
                                                rng.choice(gv.formation(ITEM_KIND[item])))
            occ2 = rule.occupancy(changed, poses, ITEM_POSES)
            assert occ2['r1'] == base_status, seed
            team2 = membership(changed, occ2)
            if base_team is None:
                # a new team with r1 needs a peer at a station of r1's item with r1's spec
                assert team2 is None or all(occ2[r] == 'at_station' and changed[r].spec_hash == claims['r1'].spec_hash
                                            for r in team2)
            else:
                assert team2 == base_team, seed
    # mismatch present == peer absent (for the waiting robot)
    item, kind = 'beam-1', 'long_beam'
    st = {role: tj.station_pose(ITEM_POSES[item], kind, role) for role in ('end_neg', 'end_pos')}
    mine = tj.RoleClaim('r1', item, kind, 'A', 'end_neg')
    poses = {'r1': st['end_neg'], 'r2': st['end_pos'], 'r3': (0., 0., 0.)}
    alone = {'r1': mine}
    mismatch = {'r1': mine, 'r2': tj.RoleClaim('r2', item, kind, 'B', 'end_pos')}
    for cl in (alone, mismatch):
        oc = rule.occupancy(cl, poses, ITEM_POSES)
        assert oc['r1'] == 'at_station' and rule.teams(cl, oc) == []


# ---------------------------------------------------------------------------
# Claims per mode

LABELS = {'beam-1': {'kind': 'long_beam'}, 'beam-2': {'kind': 'long_beam'}, 'crate-1': {'kind': 'heavy_crate'},
          'red-1': {'kind': 'red'}, 'red-2': {'kind': 'red'}, 'frame-1': {'kind': 'tri_frame'}}
VIEW = {'pickup_items_still_visible': sorted(LABELS), 'zone_counts_seen': {'A': {'red': 1}}}
GOAL = {'A': {'long_beam': 1, 'red': 2}, 'B': {'heavy_crate': 1}, 'C': {'tri_frame': 1}}


def _random_claim(rng):
    if rng.random() < .15:
        return {'item': None, 'zone': None, 'role': None}
    item = rng.choice(sorted(LABELS) + ['ghost-1'])
    kind = LABELS.get(item, {'kind': 'red'})['kind']
    role = rng.choice(list(gv.formation(kind)) + [None, 'bogus'])
    if rng.random() < .1:
        return {'box': item, 'zone': rng.choice('ABC')}
    return {'item': item, 'zone': rng.choice('ABC'), 'role': role}


def test_property_independent_claims_are_checked_alone():
    for seed in range(300):
        rng = random.Random(seed)
        claims = {rid: _random_claim(rng) for rid in ROBOTS}
        out = tj.check_independent_claims(claims, goal=GOAL, labels=LABELS, view=VIEW)
        alone = tj.check_independent_claims({'r1': claims['r1']}, goal=GOAL, labels=LABELS, view=VIEW)
        for key in ('accepted', 'invalid'):
            assert out[key].get('r1') == alone[key].get('r1')
        assert ('r1' in out['idle']) == ('r1' in alone['idle'])
        for m in tj.robot_permutations():
            re = tj.check_independent_claims(tj.relabel(m, claims), goal=GOAL, labels=LABELS, view=VIEW)
            assert re['accepted'] == tj.relabel(m, out['accepted'])
            assert re['invalid'] == tj.relabel(m, out['invalid'])
            assert sorted(re['idle']) == sorted(tj.relabel(m, out['idle']))
    # duplicates and zone disagreements are not arbitrated in independent mode
    same = {'r1': {'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'},
            'r2': {'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'}}
    assert set(tj.check_independent_claims(same, goal=GOAL, labels=LABELS, view=VIEW)['accepted']) == {'r1', 'r2'}
    bad = tj.check_independent_claims({'r1': {'item': 'beam-1', 'zone': 'A', 'role': None}},
                                      goal=GOAL, labels=LABELS, view=VIEW)
    assert 'name your role' in bad['invalid']['r1']
    legacy = tj.check_independent_claims({'r1': {'box': 'red-1', 'zone': 'A'}}, goal=GOAL, labels=LABELS, view=VIEW)
    assert legacy['accepted']['r1'].role == 'west'


def test_dynamic_claims_report_collisions_and_count_items_once():
    goal = {'A': {'long_beam': 1}, 'B': {'heavy_crate': 1}}
    view = {'pickup_items_still_visible': sorted(LABELS), 'zone_counts_seen': {}}
    both = {'r1': {'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'},
            'r2': {'item': 'beam-1', 'zone': 'A', 'role': 'end_pos'}}
    out = tj.check_dynamic_claims(both, goal=goal, labels=LABELS, view=view, active={})
    assert set(out['accepted']) == {'r1', 'r2'} and not out['collisions']
    # a third robot joining the already-counted beam or claiming a second beam
    active = out['accepted']
    third = tj.check_dynamic_claims({'r3': {'item': 'beam-2', 'zone': 'A', 'role': 'end_neg'}},
                                    goal=goal, labels=LABELS, view=view, active=active)
    assert 'needs no more long_beam' in third['invalid']['r3']
    assert tj.remaining_need_items(goal, view, active) == {'B': {'heavy_crate': 1}}
    clash = tj.check_dynamic_claims({'r1': {'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'},
                                     'r3': {'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'}},
                                    goal=goal, labels=LABELS, view=view, active={})
    assert clash['collisions'][0]['kind'] == 'same_role' and clash['accepted'] == {}
    mism = tj.check_dynamic_claims({'r1': {'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'},
                                    'r2': {'item': 'beam-1', 'zone': 'B', 'role': 'end_pos'}},
                                   goal=goal, labels=LABELS, view=view, active={})
    assert [c['kind'] for c in mism['collisions']] == ['zone_mismatch'] and mism['accepted'] == {}
    join = tj.check_dynamic_claims({'r2': {'item': 'beam-1', 'zone': 'A', 'role': 'end_pos'}},
                                   goal=goal, labels=LABELS, view=view,
                                   active={'r1': tj.RoleClaim('r1', 'beam-1', 'long_beam', 'A', 'end_neg')})
    assert set(join['accepted']) == {'r2'}
    # the host never fills a missing role: a lone claim stays a lone claim
    lone = tj.check_dynamic_claims({'r1': {'item': 'crate-1', 'zone': 'B', 'role': 'west'}},
                                   goal=goal, labels=LABELS, view=view, active={})
    assert set(lone['accepted']) == {'r1'} and 'r2' not in lone['accepted'] and 'r3' not in lone['accepted']


def test_team_plan_validator():
    labels = {'beam-1': {'kind': 'long_beam'}, 'frame-1': {'kind': 'tri_frame'}, 'red-1': {'kind': 'red'},
              'crate-1': {'kind': 'heavy_crate'}}
    goal = {'A': {'long_beam': 1}, 'B': {'tri_frame': 1}, 'C': {'red': 1}}
    plan = {'assignments': {
        'r1': [{'item': 'beam-1', 'zone': 'A', 'role': 'end_neg'}, {'item': 'frame-1', 'zone': 'B', 'role': 'v0'}],
        'r2': [{'item': 'beam-1', 'zone': 'A', 'role': 'end_pos'}, {'item': 'frame-1', 'zone': 'B', 'role': 'v1'}],
        'r3': [{'box': 'red-1', 'zone': 'C'}, {'item': 'frame-1', 'zone': 'B', 'role': 'v2'}]}}
    norm = tj.validate_team_plan(plan, goal, labels)
    assert norm['assignments']['r3'][0] == {'item': 'red-1', 'zone': 'C', 'role': 'west'}
    cyclic = copy.deepcopy(plan)
    cyclic['assignments']['r2'].reverse()
    with pytest.raises(ValueError, match='wait for each other'):
        tj.validate_team_plan(cyclic, goal, labels)
    dup = copy.deepcopy(plan)
    dup['assignments']['r2'][0]['role'] = 'end_neg'
    with pytest.raises(ValueError, match='do not fill'):
        tj.validate_team_plan(dup, goal, labels)
    zones = copy.deepcopy(plan)
    zones['assignments']['r2'][0]['zone'] = 'C'
    with pytest.raises(ValueError, match='different zones'):
        tj.validate_team_plan(zones, goal, labels)
    short = copy.deepcopy(plan)
    del short['assignments']['r3'][1]
    with pytest.raises(ValueError, match='do not fill'):
        tj.validate_team_plan(short, goal, labels)
    twice = copy.deepcopy(plan)
    twice['assignments']['r1'].append({'item': 'beam-1', 'zone': 'A', 'role': 'end_pos'})
    with pytest.raises(ValueError, match='twice'):
        tj.validate_team_plan(twice, goal, labels)
    wrong = copy.deepcopy(plan)
    wrong['assignments']['r3'][0] = {'item': 'crate-1', 'zone': 'C', 'role': 'west'}
    with pytest.raises(ValueError):
        tj.validate_team_plan(wrong, goal, labels)


# ---------------------------------------------------------------------------
# Formation motion (teacher geometry)

@pytest.mark.parametrize('kind,roles', [('long_beam', {'r1': 'end_neg', 'r2': 'end_pos'}),
                                        ('heavy_crate', {'r3': 'west', 'r1': 'east'}),
                                        ('tri_frame', {'r1': 'v0', 'r2': 'v1', 'r3': 'v2'})])
def test_formation_plan_matches_catalogue_and_probe_law(kind, roles):
    pose = (2.5, .2, .4)
    plan = FormationPlan(kind, roles)
    wg = world_grasps(CargoInstance('p', kind, pose))
    for rid, role in roles.items():
        assert plan.stations(pose)[rid] == pytest.approx(wg[role]['base_xyyaw'], abs=1e-9)
        cmd, err = plan.approach_command(rid, plan.stations(pose)[rid], pose)
        assert cmd is None and abs(err['x_m']) < 1e-6      # catalogue rounds approach bases to 1 um
        back = plan.stations(pose, standoff_m=.2)[rid]
        cmd, _ = plan.approach_command(rid, back, pose)
        assert cmd['forward'] > 0 and abs(cmd['left']) < 1e-6
        arm = plan.arm_plan(rid, plan.stations(pose)[rid], pose)
        assert arm['grip_in_base_m'][0] == pytest.approx(.155, abs=1e-6)
        assert len(arm['descent']) == 7 and set(arm['hover']) == set(arm['grasp'])
    # carry: on target -> advance, pure feed-forward; off target -> pause; one common scale
    ref = reference(pose, [('move', .8), ('turn', math.pi/2), ('move', .5)])
    step = plan.carry_step(ref.pose, ref.velocity(), plan.targets(ref.pose))
    assert step['advance'] and step['track_err_m'] < 1e-12 and step['scale'] <= 1.
    from scripts.cargo_formation_teacher import LIMITS
    ref.advance(20.)
    fast = plan.carry_step(ref.pose, (1., 0., .8), plan.targets(ref.pose))
    for c in fast['commands'].values():
        for k, (lo, hi) in LIMITS.items():
            assert lo - 1e-12 <= c[k] <= hi + 1e-12
    shifted = {r: (p[0]+.1, p[1], p[2]) for r, p in plan.targets(ref.pose).items()}
    assert not plan.carry_step(ref.pose, ref.velocity(), shifted)['advance']
    # relabelling the robots relabels the targets and commands
    m = dict(zip(sorted(roles), reversed(sorted(roles))))
    plan2 = FormationPlan(kind, tj.relabel(m, roles))
    poses = {r: (p[0]+.01, p[1]-.01, p[2]+.02) for r, p in plan.targets(ref.pose).items()}
    a = plan.carry_step(ref.pose, ref.velocity(), poses)
    b = plan2.carry_step(ref.pose, ref.velocity(), tj.relabel(m, poses))
    assert b['commands'] == tj.relabel(m, a['commands'])
    with pytest.raises(ValueError):
        FormationPlan(kind, dict(itertools.islice(roles.items(), len(roles)-1)))


def test_formation_path_never_touches_weld():
    import inspect

    import harness.zone_team_formation as f
    import harness.zone_team_jobs as j
    for mod in (f, j):
        src = inspect.getsource(mod)
        assert 'eq_active[' not in src and '_activate_beam_constraint' not in src


def test_landing_layout_keeps_team_envelopes_off_interior_walls():
    static = copy.deepcopy(authored_map('zone_wide'))
    goal = {'A': {'long_beam': 1}}
    gv.landing_layout(goal, static_map=static)
    (cx, cy) = static['regions']['zone_A']['center_m']
    static['obstacles'].append({'id': 'door_wall', 'center_m': [cx, cy+.6], 'half_extents_m': [.4, .025],
                                'height_m': .1, 'kind': 'wall'})
    with pytest.raises(ValueError, match='interior wall door_wall'):
        gv.landing_layout(goal, static_map=static)
    with pytest.raises(ValueError, match='zone_wide'):
        gv.goal_counts_v2(goal, 'zone_open')
