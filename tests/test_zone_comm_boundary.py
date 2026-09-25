"""R1 information/communication-boundary checks for the zone benchmark.

Counterfactual isolation for the no-communication ("independent") condition:
changing a peer's private state (its jobs, receipts, messages, how often it is
asked, which executor slot it holds) must not change what a robot's model
receives. Also checks that the RGB view follows the TOP image, not a layout.
See experiments/2026-09-25-zone-comm-boundary-audit/README.md.
"""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness import zone_coordination as zc
from harness import zone_solo as zs
from harness.zone_perception import detect_all, label_pickup, observe, pixel_to_floor
from sim import zone_arena as za

FIX = Path(__file__).parent / 'fixtures' / 'zone_dispatch'
GOAL = {'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}
LABELS = {'red-1': {'kind': 'red', 'floor_xy_m': [0, 0]}, 'red-2': {'kind': 'red', 'floor_xy_m': [0, 1]},
          'red-3': {'kind': 'red', 'floor_xy_m': [1, 0]}, 'cyan-1': {'kind': 'cyan', 'floor_xy_m': [1, 1]},
          'green-1': {'kind': 'green', 'floor_xy_m': [2, 0]}}
VIEW = {'source': 'test view', 'pickup_boxes_still_visible': sorted(LABELS),
        'zone_counts_seen': {'A': {}, 'B': {}, 'C': {}}, 'detections': 5}


# --- RGB provenance -------------------------------------------------------

def _shift_jpeg(jpeg, dx_px):
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    shifted = np.roll(frame, dx_px, axis=1)
    ok, buf = cv2.imencode('.jpg', shifted, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return buf.tobytes()


def test_rgb_estimates_follow_the_top_image_not_a_layout():
    """Shift the TOP image content east by 10 px: every box estimate moves by the
    matching floor distance. Nothing but the JPEG and the authored camera enters."""
    static = za.authored_map('zone_open')  # retired map: the fixture images are zone_open TOPs
    tops = {'cctv_top': (FIX/'start-top-west.jpg').read_bytes(),
            'cctv_top_east': (FIX/'start-top-east.jpg').read_bytes()}
    camera = static['top_cameras'][0]
    shape = cv2.imdecode(np.frombuffer(tops['cctv_top'], np.uint8), cv2.IMREAD_COLOR).shape
    metres_per_px = pixel_to_floor(1, 0, camera, shape)[0] - pixel_to_floor(0, 0, camera, shape)[0]
    before = [d for d in detect_all(tops, static) if d['camera'] == 'cctv_top']
    moved = dict(tops, cctv_top=_shift_jpeg(tops['cctv_top'], 10))
    after = [d for d in detect_all(moved, static) if d['camera'] == 'cctv_top']
    assert before and len(after) == len(before)
    for b in before:
        a = min((d for d in after if d['kind'] == b['kind']),
                key=lambda d: abs(d['floor_xy_m'][1]-b['floor_xy_m'][1]) + abs(d['floor_xy_m'][0]-b['floor_xy_m'][0]))
        assert a['floor_xy_m'][0] - b['floor_xy_m'][0] == pytest.approx(10*metres_per_px, abs=.006)
        assert a['floor_xy_m'][1] == pytest.approx(b['floor_xy_m'][1], abs=.006)
    # labels and the RGB view are built from those detections only
    labels = label_pickup(detect_all(tops, static), static)
    assert observe(tops, static, labels)['pickup_boxes_still_visible'] == sorted(labels)


def test_perception_never_receives_box_poses():
    """The authored static map has no box poses (they live in setup_only, which
    only the teacher and the referee read)."""
    cfg = za.episode('zone_wide', 14, goal=za.goal_counts({'A': {'red': 2}, 'B': {'cyan': 2},
                                                            'C': {'green': 1, 'yellow': 1}}),
                     extra_boxes={'red': 1, 'cyan': 1})
    static = cfg['static_map']
    assert set(static) == {'schema', 'map_id', 'version', 'frame', 'bounds_m', 'top_cameras', 'obstacles',
                           'terrain', 'regions', 'zone_slots', 'box_kinds', 'approach_convention'}
    text = json.dumps(static)
    for obj in cfg['setup_only']['objects'].values():
        assert obj['body_name'] not in text


# --- independent: counterfactual isolation of the model inputs ------------

class _FakeZone:
    def __init__(self):
        self.config = {'static_map': za.authored_map('zone_wide')}
        self.views = za.top_views(self.config['static_map'])
        self.now = 50.

    def time(self):
        return self.now

    def capture(self, label, robots):
        tops = {v[0]: f'top-{v[0]}'.encode() for v in self.views}
        frames = {r: {'own': f'own-{r}'.encode(), **{v[2]: tops[v[0]] for v in self.views}} for r in robots}
        return frames, tops


class _FakeTeam:
    """Runtime stand-in: saves, sends and validates the built request's own id,
    as ThreeRobotRuntime._invoke does."""
    def __init__(self, inbox=None):
        self.agreement = SimpleNamespace(run_id='zone-test')
        self.inbox = inbox or {'r1': [], 'r2': [], 'r3': []}
        self.requests, self.recipients = {}, []

    def ask(self, robots, build, validate, fixture, *, phase, turn, sim_time, recipients=None, attempts=2):
        self.recipients.append(recipients)
        replies = {}
        for rid in robots:
            shared = f'{self.agreement.run_id}-{rid}-{phase}-{turn}'
            request = build(rid, shared)
            self.requests.setdefault(rid, []).append(request)
            replies[rid] = validate(json.dumps(fixture(rid, shared)), request['request_id'])
        return replies

    def event(self, *a, **k):
        pass


def _round(monkeypatch, team, askers, own_jobs, turn, own_turns):
    import scripts.run_zone_dispatch as rzd
    monkeypatch.setattr(rzd, 'observe', lambda tops, static, labels: copy.deepcopy(VIEW))
    zone = _FakeZone()
    task = za.actor_task(zone.config['static_map'], GOAL)
    stats = {'claim_rounds': 0, 'invalid_claims': 0, 'same_box_accepted': 0}
    return rzd.independent_round(zone, team, task, LABELS, za.goal_counts(GOAL), askers, own_jobs, stats,
                                 turn, own_turns)


R1_JOBS = [{'box': 'red-3', 'zone': 'A', 'slot': 'A1', 'status': 'issued sequence finished',
            'issued_at_sim_s': 1.8}]


def test_independent_request_does_not_depend_on_peer_private_state(monkeypatch):
    # World A: the peers did nothing r1 could not see.
    quiet = _FakeTeam()
    _round(monkeypatch, quiet, ['r1'], {'r1': list(R1_JOBS), 'r2': [], 'r3': []}, turn=2,
           own_turns={'r1': 1, 'r2': 0, 'r3': 0})
    # World B: peers hold jobs and receipts, sent messages, and r2 was asked
    # four extra times (global round counter 2 -> 6) before r1's round.
    busy_inbox = {'r1': [{'from_robot': 'r2', 'turn': 3, 'message': 'I take red-1 to A'}], 'r2': [], 'r3': []}
    busy = _FakeTeam(busy_inbox)
    peer_jobs = {'r1': list(R1_JOBS),
                 'r2': [{'box': 'red-1', 'zone': 'A', 'slot': 'A2', 'status': 'issued', 'issued_at_sim_s': 40.}],
                 'r3': [{'box': 'cyan-1', 'zone': 'B', 'slot': 'B1', 'status': 'executor stopped before finishing',
                         'issued_at_sim_s': 30.}]}
    own_turns = {'r1': 1, 'r2': 0, 'r3': 0}
    for turn in (2, 3, 4, 5):
        _round(monkeypatch, busy, ['r2'], peer_jobs, turn=turn, own_turns=own_turns)
    _round(monkeypatch, busy, ['r1'], peer_jobs, turn=6, own_turns=own_turns)
    assert quiet.requests['r1'] == busy.requests['r1']
    assert quiet.requests['r1'][0]['request_id'] == 'zone-test-r1-solo-2-0'
    assert busy.requests['r2'][-1]['request_id'] == 'zone-test-r2-solo-4-0'
    assert all(r == [] for r in busy.recipients)  # replies are never delivered
    body = json.loads(busy.requests['r1'][0]['messages'][1]['content'])
    assert set(body) == {'request_id', 'robot_id', 'box_labels', 'rgb_view', 'own_jobs'}
    assert 'I take red-1' not in json.dumps(busy.requests['r1'])


def test_independent_request_does_not_depend_on_which_peers_share_the_round(monkeypatch):
    alone, together = _FakeTeam(), _FakeTeam()
    jobs = {'r1': list(R1_JOBS), 'r2': [], 'r3': []}
    _round(monkeypatch, alone, ['r1'], jobs, turn=7, own_turns={'r1': 3, 'r2': 0, 'r3': 0})
    _round(monkeypatch, together, ['r1', 'r2', 'r3'], jobs, turn=7, own_turns={'r1': 3, 'r2': 5, 'r3': 1})
    assert alone.requests['r1'] == together.requests['r1']


def test_executor_slot_ids_are_not_model_inputs():
    """Slots come from one pool for all robots: r1's slot A3 would reveal two
    other zone-A jobs. Own job records keep box, zone, time and receipt only."""
    near = [{'box': 'red-2', 'zone': 'A', 'slot': 'A1', 'status': 'issued', 'issued_at_sim_s': 60.8}]
    far = [dict(near[0], slot='A3')]
    solo = [zs.solo_context('r1', labels=LABELS, view=VIEW, own_jobs=j) for j in (near, far)]
    assert solo[0] == solo[1] and solo[0]['own_jobs'] == [{k: near[0][k] for k in zc.OWN_JOB_KEYS}]
    task = za.actor_task(za.authored_map('zone_wide'), GOAL)
    team = [zc.context('r1', task=task, labels=LABELS, view=VIEW, board={'active': {}}, own_jobs=j, inbox=[])
            for j in (near, far)]
    assert team[0] == team[1] and 'slot' not in json.dumps(team[0])


def test_request_body_holds_only_declared_channels_per_condition():
    task = za.actor_task(za.authored_map('zone_wide'), GOAL)
    views = za.top_views(za.authored_map('zone_wide'))
    frame = {'own': b'o', **{v[2]: v[0].encode() for v in views}}
    solo = zs.build_solo_request('r1', request_id='q', task=task, frame=frame, views=views,
                                 ctx=zs.solo_context('r1', labels=LABELS, view=VIEW, own_jobs=R1_JOBS))
    claim = zc.build_claim_request('r1', request_id='q', task=task, frame=frame, views=views,
                                   ctx=zc.context('r1', task=task, labels=LABELS, view=VIEW, board={'active': {}},
                                                  own_jobs=R1_JOBS, inbox=[]))
    bodies = [json.loads(r['messages'][1]['content']) for r in (solo, claim)]
    assert set(bodies[0]) == {'request_id', 'robot_id', 'box_labels', 'rgb_view', 'own_jobs'}
    assert set(bodies[1]) == set(bodies[0]) | {'team_board', 'peer_messages'}
    # identical images in both conditions: own RGB + every TOP view
    assert [i['label'] for i in solo['images']] == [i['label'] for i in claim['images']]
    assert solo['images'] == claim['images']


# --- known leak, not fixed here (audit L1) ---------------------------------

@pytest.mark.xfail(strict=True, reason='audit L1: the teacher stops a robot because a peer merely INTENDS the '
                   'same box (earlier to_box job), at the assignment tick; needs a SIM-gated fix')
def test_a_peer_intent_alone_does_not_stop_a_robot():
    from scripts.zone_teacher import TeacherRobot
    def robot(rid, phase, assigned_at):
        r = TeacherRobot.__new__(TeacherRobot)
        r.rid, r.phase, r.outcome, r.job, r.assigned_at = rid, phase, None, {'box_body': 'cargo_box_00'}, assigned_at
        return r
    me, peer = robot('r2', 'to_box', 60.8), robot('r3', 'to_box', 43.3)  # ZC2-s14-independent-nominal r2-2
    me.team = peer.team = {'r2': me, 'r3': peer}
    # The peer has not reached the box; nothing r2 could observe says it is taken.
    assert not me._taken_by_peer('cargo_box_00')
