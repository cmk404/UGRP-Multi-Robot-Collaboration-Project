"""Zone teacher fix (2026-09-26): PR #169 blockers B2/B8 gate, B3 claim yield/staging, B4 return set-down,
B5 label tracking. GT TEACHER only: nothing here is a robot success."""
import math
from types import SimpleNamespace

import pytest

from harness.zone_mixed_episode import mixed_episode
from harness.zone_perception_v2 import PROFILES, label_items, public_labels, view_from_detections
from harness.zone_team_footprint import TeamFootprint
from harness.zone_team_jobs import TeamJob
from harness.zone_team_route import PoseReference
from sim import zone_arena as za

MIXED = {'A': {'long_beam': 1}, 'B': {'heavy_crate': 1, 'red': 1}, 'C': {'can': 1, 'green': 1, 'tile': 1}}
TRI = {'A': {'tri_frame': 1, 'red': 1}, 'C': {'green': 1}}


# --- B2 / B8: pre-run feasibility gate (config only) ------------------------------

def _config(variant, goal, seed):
    return mixed_episode(variant, seed, goal=goal, extra_boxes={}, extra_cargo={}, colour_only_ok=True)


def test_gate_refuses_b2_tri_frame_through_the_half_metre_door():
    from harness.zone_teacher_gate import team_route_feasibility
    gate = team_route_feasibility(_config('zone_wide_door', TRI, 11))
    assert gate['verdict'] == 'infeasible'
    (row,) = gate['items']
    assert row['kind'] == 'tri_frame' and not row['with_all_items']['ok'] and not row['without_solo_items']['ok']


def test_gate_flags_b8_the_red_box_in_front_of_the_wide_door():
    from harness.zone_teacher_gate import team_route_feasibility
    config = _config('zone_wide_two_doors', TRI, 11)
    gate = team_route_feasibility(config)
    assert gate['verdict'] == 'order_constrained'
    (row,) = gate['items']
    assert row['blocking_solo_items'] == [['box_01']]
    assert config['setup_only']['objects']['box_01']['kind'] == 'red'
    assert row['blocking_solo_item_poses']['box_01'] == pytest.approx([1.6, -2.45], abs=.05)


def test_gate_passes_the_mixed_goal_on_the_cohort_seeds_and_reads_only_the_config():
    from harness.zone_teacher_gate import team_route_feasibility
    for seed in (21, 22, 23):
        gate = team_route_feasibility(_config('zone_wide_two_doors', MIXED, seed))
        assert gate['verdict'] == 'ok', seed
        assert {r['kind'] for r in gate['items']} == {'long_beam', 'heavy_crate'}
    assert 'never simulator state' in gate['source']


# --- B4: return to the start pose before a motion-only hold_lower -----------------

def test_return_poses_reverse_the_travelled_part_of_the_route():
    from scripts.zone_team_teacher_fix import return_poses
    ref = PoseReference([(0., 0., 0.), (1., 0., 0.), (1., 0., math.pi/2), (1., 1., math.pi/2)])
    ref.advance(ref._leg_len(0) + ref._leg_len(1) + .5*ref._leg_len(2))
    assert ref.leg == 2
    poses = return_poses(ref, (1., .5, math.pi/2))
    assert poses == [(1., .5, math.pi/2), (1., 0., math.pi/2), (1., 0., 0.), (0., 0., 0.)]


def _carry(switch=True, forces=10.):
    from scripts.zone_team_teacher_fix import FixTeamCarry
    job = TeamJob(job_id='long_beam_0@g1', generation=1, item_label='long_beam_0', kind='long_beam', zone='A',
                  role_by_robot={'r1': 'end_pos', 'r2': 'end_neg'}, now=0.)
    while job.state != 'CARRY':
        for r in job.participants:
            job.report_ready(r, 1.)
        job.advance(1.)
    logs, holds = [], []
    robots = {r: SimpleNamespace(phase='team', port=SimpleNamespace(hold=holds.append)) for r in ('r1', 'r2')}
    ex = SimpleNamespace(switches={'b4_return_setdown': switch, 'b3_claim_yield': True},
                         fix_events={k: 0 for k in ('return_started', 'return_finished', 'return_failed')},
                         finger_forces=lambda iid, parts: {r: (forces, forces) for r in parts},
                         item_pose=lambda iid: (1., .5, math.pi/2), robots=robots,
                         log=lambda *a, **kw: logs.append((a, kw)), robot_event=lambda *a, **kw: None)
    carry = FixTeamCarry.__new__(FixTeamCarry)
    carry.ex, carry.job, carry.parts, carry.n, carry.item_id = ex, job, job.participants, 2, 'long_beam_0'
    carry.seen, carry.pause, carry.pause_total, carry.returning = 'CARRY', {'t0': 10., 'by': ['r3']}, 0., None
    carry.record = {'failures': [], 'pauses': []}
    carry.ref = PoseReference([(0., 0., 0.), (1., 0., 0.), (1., 0., math.pi/2), (1., 1., math.pi/2)])
    carry.ref.advance(carry.ref._leg_len(0) + carry.ref._leg_len(1) + 1.)
    return carry, logs


def test_carry_blocked_with_grips_carries_back_then_lowers_at_the_start():
    carry, _ = _carry()
    r = carry.fail('r1', 'carry_blocked', 200., by=['r3'])
    assert r['action'] == 'return_to_start' and carry.job.state == 'CARRY' and carry.job.aborting is None
    assert carry.ref.poses[0] == (1., .5, math.pi/2) and carry.ref.poses[-1] == (0., 0., 0.)
    assert carry.record['pauses'][0]['s'] == 190. and carry.pause is None
    assert carry.advance(201.) is None and carry.job.state == 'CARRY'   # barrier not complete yet
    for rid in carry.parts:
        carry.job.report_ready(rid, 260.)
    assert carry.advance(260.) is None
    assert carry.job.state == 'LOWER' and carry.job.aborting['reason'] == 'carry_blocked'
    assert carry.returning['outcome'] == 'returned_to_start'
    assert carry.record['failures'][0]['returned_to_start'] is True
    assert carry.ex.fix_events == {'return_started': 1, 'return_finished': 1, 'return_failed': 0}


def test_grip_loss_a_second_failure_or_the_switch_off_lower_in_place():
    carry, _ = _carry()
    assert carry.fail('r1', 'grip_lost_in_transit', 200.)['action'] == 'hold_lower'
    carry, _ = _carry()
    carry.fail('r1', 'carry_blocked', 200., by=['r3'])
    assert carry.fail('r1', 'carry_blocked', 400., by=['r3'])['action'] == 'hold_lower'
    assert carry.returning['outcome'] == 'lowered_in_place_after_return_failure'
    assert carry.ex.fix_events['return_failed'] == 1
    carry, _ = _carry(switch=False)
    assert carry.fail('r1', 'carry_blocked', 200., by=['r3'])['action'] == 'hold_lower'
    carry, _ = _carry(forces=0.)
    assert carry.fail('r1', 'carry_timeout', 200.)['action'] == 'hold_lower'


# --- B3: robots holding a claim yield to and stay off a carrying team's route ------

def _team_ex(robots, switch=True):
    from scripts.zone_team_teacher_fix import FixZoneTeamExecutor
    ex = FixZoneTeamExecutor.__new__(FixZoneTeamExecutor)
    logs = []
    ex.switches = {'b3_claim_yield': switch, 'b4_return_setdown': True}
    ex.fix_events = {k: 0 for k in ('claim_yield', 'station_staged', 'arrival_reset', 'yield_short_fallback')}
    ex.log = lambda ev, rid, now, **kw: logs.append((ev, rid))
    ex.robots = robots
    ref = PoseReference([(0., 0., 0.), (3., 0., 0.)])
    carry = SimpleNamespace(n=2, parts=('r1', 'r2'), item_id='long_beam_0', ref=ref,
                            footprint=TeamFootprint('long_beam', ('end_neg', 'end_pos')),
                            job=SimpleNamespace(terminal=False, state='CARRY', job_id='long_beam_0@g1'),
                            proxy=SimpleNamespace(rid='team:long_beam_0@g1'))
    ex.carries = {'long_beam_0@g1': carry}
    ex.held = lambda iid: True
    return ex, carry, logs


def _robot(rid, pose, *, claim_state=None, busy=None, station=None):
    asked = []
    claim = None if claim_state is None else SimpleNamespace(live=True, state=claim_state, item_id='heavy_crate_0',
                                                             arrived_at=5.)
    robot = SimpleNamespace(rid=rid, pose=lambda: pose, claim=claim, yield_req=None, staged=None,
                            busy=bool(claim) if busy is None else busy, station=station)

    def request_yield(req, avoid, now):
        asked.append(req.rid)
        robot.yield_req = {'by': req, 'avoid': avoid, 'target': None, 'until': now + 20.}
    robot.request_yield = request_yield
    return robot, asked


def test_a_robot_waiting_at_a_station_on_the_route_is_asked_to_yield():
    r3, asked3 = _robot('r3', (.9, .05, 0.), claim_state='waiting')
    ex, carry, logs = _team_ex({'r3': r3})
    assert ex.carry_blockers(carry, 30.) == {'r3'}
    assert asked3 == ['team:long_beam_0@g1'] and ex.fix_events['claim_yield'] == 1 and ('claim_yield', 'r3') in logs
    # avoid the whole remaining route (3 m at 0.05 m/s), with PR #169's next 12 SIM s kept as the fallback
    req = r3.yield_req
    assert req['avoid'][0] == (0., 0.) and req['avoid'][-1] == (3., 0.)
    assert max(x for x, _ in req['avoid_short']) == pytest.approx(.6)
    assert ex.carry_blockers(carry, 30.1) == {'r3'} and asked3 == ['team:long_beam_0@g1']   # asked once


def test_a_yield_with_no_spot_clear_of_the_whole_route_falls_back_to_the_short_avoid(monkeypatch):
    from scripts import zone_team_teacher
    from scripts.zone_team_teacher_fix import FixTeamRobot
    seen = []

    def base_yield(self, now, discs_for):
        seen.append(len(self.yield_req['avoid']))
        if len(self.yield_req['avoid']) > 2:        # "no spot" for the whole route
            self.yield_req = None
            return False
        self.yield_req['target'] = (1., 1.)
        return True
    monkeypatch.setattr(zone_team_teacher.TeamRobot, '_yield', base_yield)
    robot = FixTeamRobot.__new__(FixTeamRobot)
    robot.ex = SimpleNamespace(fix_events={'yield_short_fallback': 0})
    robot.yield_req = {'avoid': [(0, 0), (1, 0), (2, 0)], 'avoid_short': [(0, 0)], 'target': None, 'until': 50.}
    assert robot._yield(30., None) is True and seen == [3, 1] and robot.ex.fix_events['yield_short_fallback'] == 1
    assert 'avoid_short' not in robot.yield_req


def test_a_robot_in_a_job_or_off_the_route_is_not_asked_and_the_switch_restores_pr169():
    r3, asked3 = _robot('r3', (.9, .05, 0.), claim_state='job')
    far, asked_far = _robot('r2x', (.9, 2.5, 0.))
    ex, carry, _ = _team_ex({'r3': r3, 'r2x': far})
    assert ex.carry_blockers(carry, 30.) == {'r3'} and asked3 == [] and asked_far == []
    waiting, asked_w = _robot('r3', (.9, .05, 0.), claim_state='waiting')
    ex, carry, _ = _team_ex({'r3': waiting}, switch=False)
    from scripts.zone_team_teacher import ZoneTeamExecutor
    ex.robots = {'r3': waiting}
    assert ZoneTeamExecutor.carry_blockers(ex, carry, 30.) == {'r3'} and asked_w == []   # PR #169: busy robots stay


def test_a_claimed_station_inside_the_team_route_is_staged_and_resets_the_arrival():
    on, _ = _robot('r3', (2.5, 1.5, 0.), claim_state='waiting', station=(1.2, .1, 0.))
    off, _ = _robot('r3', (2.5, 1.5, 0.), claim_state='waiting', station=(1.2, 1.5, 0.))
    ex, carry, _ = _team_ex({'r3': on})
    assert ex.station_in_team_path(on) == 'long_beam_0@g1' and ex.station_in_team_path(off) is None
    carry.job.state = 'LOWER'
    assert ex.station_in_team_path(on) is None


def test_a_robot_standing_aside_is_no_station_occupant_and_its_arrival_is_reset(monkeypatch):
    from scripts import zone_team_teacher
    seen = []
    monkeypatch.setattr(zone_team_teacher.ZoneTeamExecutor, '_claims_tick',
                        lambda self, now: seen.append({r: c.state for r, c in self.claims.items()}))
    staged, _ = _robot('r3', (0., 0., 0.), claim_state='waiting')
    staged.staged = ('long_beam_0@g1', 1.)
    calm, _ = _robot('r1', (0., 0., 0.), claim_state='waiting')
    ex, _, _ = _team_ex({'r3': staged, 'r1': calm})
    ex.claims = {'r3': staged.claim, 'r1': calm.claim}
    ex._claims_tick(40.)
    assert seen == [{'r3': 'aside', 'r1': 'waiting'}]          # the PR #169 rule never sees r3 this tick
    assert staged.claim.arrived_at is None and staged.claim.state == 'to_station'
    assert calm.claim.arrived_at == 5. and ex.fix_events['arrival_reset'] == 1
    ex._claims_tick(40.1)
    assert ex.fix_events['arrival_reset'] == 1 and staged.claim.state == 'to_station'


def test_a_staged_robot_holds_and_replans_to_its_station_when_the_route_clears(monkeypatch):
    from scripts import zone_team_teacher
    from scripts.zone_team_teacher_fix import FixTeamRobot
    ticks = []
    monkeypatch.setattr(zone_team_teacher.TeamRobot, 'tick', lambda self, now, d: ticks.append(now))
    robot = FixTeamRobot.__new__(FixTeamRobot)
    holds, phases, logs = [], [], []
    team = ['long_beam_0@g1']
    robot.rid, robot.phase = 'r3', 'align_box'
    robot.claim = SimpleNamespace(live=True, state='waiting')
    robot.yield_req = robot.passage_yield = robot.staged = None
    robot.arm = SimpleNamespace(tick=lambda now: None)
    robot.port = SimpleNamespace(hold=holds.append)
    robot.log = lambda ev, rid, now, **kw: logs.append(ev)
    robot._set = lambda phase, now, **kw: phases.append(phase)
    robot.ex = SimpleNamespace(switches={'b3_claim_yield': True}, fix_events={'station_staged': 0},
                               station_in_team_path=lambda r: team[0],
                               yield_if_in_team_path=lambda r, jid, now: False)
    robot.tick(10., None)
    robot.tick(10.1, None)
    assert holds == [10., 10.1] and ticks == [] and robot.ex.fix_events['station_staged'] == 1
    team[0] = None
    robot.tick(20., None)
    assert phases == ['to_box'] and ticks == [20.] and logs == ['station_staged', 'station_unstaged']
    # staged inside the team's hull: it yields at once and stays staged while yielding
    team[0] = 'long_beam_0@g1'
    robot.ex.yield_if_in_team_path = lambda r, jid, now: True
    robot.tick(30., None)
    assert robot.staged == ('long_beam_0@g1', 30.) and ticks == [20., 30.] and logs[-1] == 'station_staged'


# --- B5: labels follow an item set down off its pickup spot (opt-in profile) ------

def _beam_labels():
    static = za.authored_map('zone_wide_two_doors')
    dets = [{'kind': 'long_beam', 'floor_xy_m': [.7, -.15], 'yaw_rad': 1.57, 'confidence': .9},
            {'kind': 'red', 'floor_xy_m': [1.6, -2.45], 'yaw_rad': None}]
    return static, label_items(dets, static, 'top_cargo_v2_track')


def test_a_moved_beam_is_relabelled_after_two_still_observations():
    static, labels = _beam_labels()
    moved = [{'kind': 'long_beam', 'floor_xy_m': [1.9, .6], 'yaw_rad': 0., 'confidence': .9},
             {'kind': 'red', 'floor_xy_m': [1.61, -2.45], 'yaw_rad': None, 'confidence': 1.}]
    first = view_from_detections(moved, static, labels, 'top_cargo_v2_track')
    assert first['pickup_items_still_visible'] == ['red-1'] and first['moved_labels'] == {}
    second = view_from_detections(moved, static, labels, 'top_cargo_v2_track')
    assert second['pickup_items_still_visible'] == ['long_beam-1', 'red-1']
    assert second['moved_labels']['long_beam-1']['to_rgb_xy_m'] == [1.9, .6]
    assert labels['long_beam-1']['floor_xy_m'] == [1.9, .6] and labels['long_beam-1']['yaw_rad'] == 0.
    assert all(math.dist(h['grip_xy_m'], [1.9, .6]) < .5 for h in labels['long_beam-1']['handles'].values())
    assert set(public_labels(labels)['long_beam-1']) == {'kind', 'rgb_floor_xy_m', 'rgb_yaw_rad', 'handles'}
    third = view_from_detections(moved, static, labels, 'top_cargo_v2_track')
    assert 'long_beam-1' in third['pickup_items_still_visible'] and third['moved_labels'] == {}


def test_moving_zone_and_default_profile_detections_are_not_relabelled():
    static, labels = _beam_labels()
    a = [{'kind': 'long_beam', 'floor_xy_m': [1.9, .6], 'yaw_rad': 0., 'confidence': .9}]
    b = [{'kind': 'long_beam', 'floor_xy_m': [2.1, .6], 'yaw_rad': 0., 'confidence': .9}]
    view_from_detections(a, static, labels, 'top_cargo_v2_track')
    assert view_from_detections(b, static, labels, 'top_cargo_v2_track')['moved_labels'] == {}   # still moving
    zone_a = static['regions']['zone_A']['center_m']
    in_zone = [{'kind': 'long_beam', 'floor_xy_m': list(zone_a), 'yaw_rad': 0., 'confidence': .9}]
    for _ in range(3):
        view = view_from_detections(in_zone, static, labels, 'top_cargo_v2_track')
    assert view['moved_labels'] == {} and view['zone_counts_seen']['A'] == {'long_beam': 1}
    static, labels = _beam_labels()
    for _ in range(3):
        view = view_from_detections(a, static, labels, 'top_cargo_v2')
    assert 'moved_labels' not in view and view['pickup_items_still_visible'] == []
    assert PROFILES['top_cargo_v2_track'] is PROFILES['top_cargo_v2']


def test_runner_accepts_the_tracking_profile_and_the_gate_modes():
    from scripts.run_zone_teacher_fix import parser
    args = parser().parse_args(['--output', '/tmp/x', '--perception-profile', 'top_cargo_v2_track',
                                '--teacher-fix', '{"b3_claim_yield": false}', '--feasibility-gate', 'report'])
    assert args.perception_profile == 'top_cargo_v2_track' and args.feasibility_gate == 'report'
    from scripts.run_zone_dispatch import parser as base
    with pytest.raises(SystemExit):
        base().parse_args(['--output', '/tmp/x', '--perception-profile', 'top_cargo_v2_track'])
