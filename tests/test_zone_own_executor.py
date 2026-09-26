"""Package F own-camera executor: API, events, M1 contract, isolation between robots, 3-robot host.

Everything except ``test_team_host_*`` runs without a simulator. The host test builds one
3-robot sync-SIM world for ~3 SIM s (skipped when MuJoCo cannot render).
"""
from __future__ import annotations

import base64
import gc
import hashlib
import inspect
import json
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_own_executor as zox  # noqa: E402
from harness.owncam_pose_source import OwnCamPoseSource  # noqa: E402

MAP = json.loads((ROOT / 'maps' / 'zones' / 'zone_wide_door_tags_v2.json').read_text())
CALIB = json.loads((ROOT / 'experiments' / '2026-09-26-zone-m1-owncam' / 'calibration_m1_dev.json').read_text())
ROWS_Y = (-2.45, -1.65, -.85, -.05, .75)
SEARCH_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
SHEET = {'orders': [
    {'order_id': 'o1', 'kind': 'cyan', 'count': 1, 'required_robots': 1, 'destination_zone': 'A',
     'initial_location': {'pickup_bay': 'P2', 'slot': 'P2-3'}},
    {'order_id': 'o2', 'kind': 'cyan', 'count': 1, 'required_robots': 1, 'destination_zone': 'C',
     'initial_location': {'pickup_bay': 'P1', 'slot': 'P1-2'}},
    {'order_id': 'o3', 'kind': 'red', 'count': 1, 'destination_zone': 'B',
     'initial_location': {'pickup_bay': 'P1', 'slot': 'P1-1'}}]}

# Package A (PR #194, 189b177) harness/zone_study_contract.py, pinned: robot-facing keys that must never appear.
A_FORBIDDEN_KEYS = frozenset({
    'pose', 'poses', 'qpos', 'qvel', 'ctrl', 'measured_joints', 'joint_positions', 'joint_angles', 'body_id',
    'body_name', 'geom_id', 'site_id', 'xpos', 'xquat', 'ground_truth', 'truth', 'gt', 'simulator', 'sim_state',
    'mj_model', 'mj_data', 'contacts', 'contact_forces', 'forces', 'wrench', 'weld', 'referee', 'referee_v2',
    'top', 'top_frame', 'top_frames', 'top_image', 'top_images', 'top_rgb', 'top_view', 'top_views', 'top_camera',
    'top_cameras', 'cctv', 'cctv_top', 'nav_cam', 'nav_cam_rgb', 'nav_camera', 'teacher', 'teacher_receipt',
    'teacher_receipts', 'receipt', 'receipts', 'grasp_success', 'placed', 'placed_at', 'delivered', 'deliveries',
    'delivery_confirmed', 'completion', 'completed', 'complete', 'finished', 'success', 'succeeded', 'done',
    'zone_counts', 'zone_counts_seen', 'remaining_need', 'global_progress', 'progress', 'team_board', 'peer_board',
    'peer_status', 'peer_states', 'active_claims', 'peer_claims', 'peer_commands', 'peer_command_history',
    'peer_rgb', 'peer_images', 'other_robots', 'busy', 'hidden_event', 'hidden_events', 'event_schedule',
    'injected_failures', 'eval', 'evaluation', 'eval_only', 'score', 'scores', 'makespan', 'metrics'})
A_FORBIDDEN_SUBSTRINGS = ('_pose', 'pose_', 'ground_truth', 'teacher', 'receipt', 'top_rgb', 'top_frame', 'top_image',
                          'cctv', 'nav_cam', 'qpos', 'qvel', 'hidden_event', 'body_id', 'xpos', 'sim_state', 'peer_',
                          'referee', 'weld', 'grasp_success')
A_ACTION_FIELDS = ('schema', 'run_id', 'condition', 'seed', 'actor', 'action_id', 'request_id', 'submitted_at_sim_s',
                   'kind', 'arguments', 'order_id', 'role', 'accepted', 'rejected_reason', 'local_state')
A_COMMAND_ARGUMENT_KEYS = ('target_ref', 'target_zone', 'order_id', 'item', 'role', 'passage', 'distance_m',
                           'turn_deg', 'speed', 'duration_s', 'gripper', 'observe', 'waypoints', 'reason_code')
A_BELIEF_KEYS = ('region', 'last_visual_anchor', 'last_requested_destination', 'last_visually_confirmed_region',
                 'confidence', 'sources', 'held_item_guess', 'blocked_passages', 'notes_ko')


def _jpeg(value=120):
    import cv2
    ok, buf = cv2.imencode('.jpg', np.full((480, 640, 3), value, np.uint8))
    assert ok
    return buf.tobytes()


_JPEGS = {}


def obs(rid, frame_id, t, servo, camera='robot_cam', value=120, motors=(0., 0., 0., 0.)):
    jpeg = _JPEGS.setdefault(value, _jpeg(value))
    return {'robot_id': rid, 'frame_id': frame_id, 'sim_time': t, 'image': base64.b64encode(jpeg).decode(),
            'sha256': hashlib.sha256(jpeg).hexdigest(), 'camera': camera,
            'actuator_state': {'motor_commands': list(motors), 'servo_pulses': {str(k): v for k, v in servo.items()}}}


def rgb_of(o):
    import cv2
    return cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(o['image']), np.uint8), cv2.IMREAD_COLOR),
                        cv2.COLOR_BGR2RGB)


def make(rid='r1', **kw):
    kw.setdefault('skill_factory', lambda order, robot_id: None)
    return zox.ZoneOwnExecutor(rid, MAP, CALIB['params'], SHEET, pose_estimate_cls=tuple, search_rows_y=ROWS_Y, **kw)


class Driver:
    """A sim-free stand-in for the physics owner: own frames at 5 Hz, own command rows, 0.1 s ticks."""

    def __init__(self, ex, value=120):
        self.ex, self.value, self.t, self.fid = ex, value, 0., 0
        self.servo = dict(SEARCH_POSE)
        self.decisions = []
        ex.on_command({'t': 0., 'kind': 'initial_servo_command', 'pulses': dict(self.servo)})

    def frame(self):
        self.fid += 1
        o = obs(self.ex.robot_id, self.fid, self.t, self.servo, value=self.value)
        self.ex.on_frame(self.t, o, rgb_of(o))

    def run(self, seconds, stop=None):
        end = self.t + seconds
        while self.t < end - 1e-9:
            if round(self.t * 10) % 2 == 0:
                self.frame()
            d = self.ex.step(self.t)
            self.decisions.append((round(self.t, 2), json.dumps(d, sort_keys=True)))
            assert d['mode'] == 'tick', d
            for c in d['commands']:
                row = {'t': round(self.t, 4), **c}
                if c['kind'] == 'arm':
                    self.servo[c['servo_id']] = c['pulse']
                elif c['kind'] == 'look':
                    self.servo[6] = c['pan_pulse']
                self.ex.on_command(row)
            self.t = round(self.t + zox.TICK_S, 4)
            if stop is not None and stop():
                return


# ---------------------------------------------------------------- boundary
def test_module_import_and_use_is_simulator_free():
    code = ('import sys, json; sys.path.insert(0, %r)\n'
            'from tests.test_zone_own_executor import make, Driver\n'
            'exs = [make(r) for r in ("r1", "r2", "r3")]\n'
            'for e in exs:\n'
            '    e.hold(0.3); Driver(e).run(0.6)\n'
            'bad = sorted(m for m in sys.modules if m == "mujoco" or m.startswith("mujoco.") or\n'
            '             (m.startswith("sim.") and m != "sim.masterpi_camera_profile"))\n'
            'print(json.dumps(bad))\n') % str(ROOT)
    out = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, out.stderr[-3000:]
    assert json.loads(out.stdout.strip().splitlines()[-1]) == []


def test_executor_source_reads_no_simulator_state():
    import ast
    import textwrap
    names = set()
    for cls in (zox.ZoneOwnExecutor, zox._DeliverController, zox._SharedLocDriver):
        tree = ast.parse(textwrap.dedent(inspect.getsource(cls)))
        names |= {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        names |= {a.arg for n in ast.walk(tree) if isinstance(n, ast.arguments) for a in n.args + n.kwonlyargs}
    forbidden = {'mujoco', 'xpos', 'qpos', 'qvel', 'base_xyz', 'base_rpy', 'eval_only', 'setup_only', 'position_m',
                 'world', 'render_jpeg', 'render_rgb', 'contact', 'controllers', 'robots', 'host', 'peers', 'port'}
    assert not names & forbidden, sorted(names & forbidden)
    params = inspect.signature(zox.ZoneOwnExecutor.__init__).parameters
    assert not any(p in params for p in ('world', 'port', 'peers', 'host', 'truth', 'scene'))


def test_pickup_slots_follow_package_a_rule():
    slots = zox.pickup_slots(MAP)
    assert sorted(slots) == ['P1-1', 'P1-2', 'P1-3', 'P2-1', 'P2-2', 'P2-3']
    assert slots['P1-1']['center_m'] == [0.125, -2.15] and slots['P2-3']['center_m'] == [1.275, 0.45]
    assert slots['P1-2']['y_range_m'] == [-1.5, -0.2] and slots['P2-1']['x_range_m'] == [0.7, 1.85]
    assert zox.pickup_slot_of(MAP, (1.6, -0.85)) == 'P2-2' and zox.pickup_slot_of(MAP, (-0.2, 0.75)) == 'P1-3'


def test_order_sheet_refuses_coordinates_and_unknown_keys():
    bad = {'orders': [{**SHEET['orders'][0], 'initial_location': {'pickup_bay': 'P2', 'slot': [1.0, 0.75]}}]}
    with pytest.raises(zox.ExecutorContractError):
        zox.validate_order_sheet(bad, MAP)
    with pytest.raises(zox.ExecutorContractError):
        zox.validate_order_sheet({'orders': [{**SHEET['orders'][0], 'position_m': [1, 2]}]}, MAP)
    with pytest.raises(zox.ExecutorContractError):
        zox.validate_order_sheet({'orders': [{**SHEET['orders'][0], 'initial_location': {'slot': 'P9-9',
                                                                                          'pickup_bay': 'P9'}}]}, MAP)


# ---------------------------------------------------------------- M1 contract
class _FakePose(OwnCamPoseSource):
    def __post_init__(self):
        super().__post_init__()
        self.source = 'gt_stub_eval_only'


def test_m1_mode_refuses_a_gt_pose_source():
    ex = make()
    assert ex.pose.source.startswith('owncam_pf_v2:')
    fake = _FakePose(MAP, CALIB['params'])
    with pytest.raises(zox.ExecutorContractError):
        make(pose_source=fake)
    with pytest.raises(zox.ExecutorContractError):
        make(pose_source=types.SimpleNamespace(source='owncam_pf_v2:deadbeef'))   # not the own-camera estimator
    diag = make(mode='diagnostic', pose_source=fake)              # diagnostic may take it ...
    assert diag.summary()['counts_as_m1_inputs'] is False          # ... and never counts as M1


def test_foreign_or_non_own_observations_are_refused():
    ex = make('r1')
    Driver(ex).frame()
    o2 = obs('r2', 2, 0.2, SEARCH_POSE)
    with pytest.raises(zox.ExecutorContractError):
        ex.on_frame(0.2, o2, rgb_of(o2))
    nav = obs('r1', 3, 0.2, SEARCH_POSE, camera='nav_cam')
    with pytest.raises(Exception, match='robot_cam'):
        ex.on_frame(0.2, nav, rgb_of(nav))
    old = obs('r1', 1, 0.2, SEARCH_POSE)
    with pytest.raises(Exception, match='new frame'):
        ex.on_frame(0.2, old, rgb_of(old))
    assert ex.summary()['rejected_foreign_frames'] == 1


# ---------------------------------------------------------------- isolation between robots
def _reachable(root, limit=400_000):
    """Every object reachable from ``root`` (closures and defaults included, module globals and classes not)."""
    seen, stack = {}, [root]
    atoms = (types.ModuleType, type, types.BuiltinFunctionType, str, bytes, int, float, bool, complex, type(None))
    while stack and len(seen) < limit:
        o = stack.pop()
        if id(o) in seen or isinstance(o, atoms):
            continue
        seen[id(o)] = o
        if isinstance(o, types.FunctionType):
            stack.extend(c.cell_contents for c in (o.__closure__ or ()) if c.cell_contents is not None)
            stack.extend(o.__defaults__ or ())
            stack.extend((o.__kwdefaults__ or {}).values())
            continue
        stack.extend(gc.get_referents(o))
    return seen


def test_three_executors_share_no_mutable_state():
    shared_map, shared_params, shared_sheet = MAP, CALIB['params'], SHEET
    exs = [zox.ZoneOwnExecutor(r, shared_map, shared_params, shared_sheet, skill_factory=lambda o, robot_id: None,
                               pose_estimate_cls=tuple, search_rows_y=ROWS_Y) for r in ('r1', 'r2', 'r3')]
    reach = [_reachable(e) for e in exs]
    mutable = (dict, list, set, bytearray, np.ndarray)
    for i in range(3):
        for j in range(i + 1, 3):
            common = [reach[i][k] for k in set(reach[i]) & set(reach[j])
                      if isinstance(reach[i][k], mutable) or hasattr(reach[i][k], '__dict__')]
            common = [o for o in common if not (isinstance(o, np.ndarray) and not o.flags.writeable)]
            assert not common, [type(o).__name__ for o in common[:10]]
    for e, other in ((exs[0], exs[1]), (exs[1], exs[2])):
        assert id(other) not in _reachable(e)


def test_one_robot_decisions_do_not_depend_on_another_robots_inputs():
    def run(r2_value):
        r1, r2 = make('r1'), make('r2')
        d1, d2 = Driver(r1), Driver(r2, value=r2_value)
        r1.look_around()
        r2.look_around() if r2_value == 120 else r2.hold(2.)
        for _ in range(30):                      # interleaved in one SIM loop
            d1.run(.2)
            d2.run(.2)
        return d1.decisions, r1.status(), [e['event'] for e in r1.events]
    assert run(120) == run(30)


# ---------------------------------------------------------------- API, events, adapters
def test_job_api_lifecycle_and_events():
    ex = make()
    d = Driver(ex)
    ack = ex.hold(1.)
    assert ack['accepted'] and ack['local_state'] == 'hold_requested'
    busy = ex.deliver('o1', 'A2')
    assert not busy['accepted'] and busy['rejected_reason'].startswith('BUSY:hold') and \
        busy['local_state'] == 'command_rejected'
    d.run(1.5)
    ev = ex.drain_events()
    assert [e['event'] for e in ev] == ['job_started', 'job_done']
    assert ev[1]['detail']['confirmation'] == 'unconfirmed' and ev[1]['scheduler_trigger'] == 'idle'
    assert ex.status()['local_state'] == 'queue_empty' and ex.status()['job'] is None
    # rejections: unknown order, wrong destination, unsupported kind, unknown target
    assert ex.deliver('nope', 'A2')['rejected_reason'] == 'UNKNOWN_ORDER'
    assert ex.deliver('o1', 'B2')['rejected_reason'] == 'SLOT_OUTSIDE_ORDER_DESTINATION'
    assert ex.deliver('o3', 'B2')['rejected_reason'] == 'KIND_NOT_SUPPORTED_BY_M1_SKILL'
    assert ex.goto('Q7')['rejected_reason'] == 'UNKNOWN_TARGET'
    assert ex.goto([99., 0.])['rejected_reason'] == 'TARGET_OUTSIDE_MAP'
    assert ex.abort()['rejected_reason'] == 'NO_ACTIVE_JOB'
    # look_around with blank frames: the sweep runs, the estimate never initialises -> unconfirmed
    ex.look_around()
    d.run(12., stop=lambda: ex.job is None)
    last = ex.drain_events()[-1]
    assert last['event'] == 'job_done' and last['detail']['outcome'] == 'LOOKED_POSE_UNCERTAIN'
    assert last['detail']['confirmation'] == 'unconfirmed'
    assert d.servo == SEARCH_POSE                                   # posture restored after the look
    # deliver + abort -> job_failed with the caller's reason, a failure wake
    assert ex.deliver('o1', 'A')['accepted']
    assert ex.status()['job']['arguments']['slot_id'] == 'A1'      # zone letter -> next own slot
    d.run(1.)
    ab = ex.abort('peer_asked')
    assert ab['accepted'] and ab['local_state'] == 'hold_requested'
    ev = ex.drain_events()
    assert ev[-1]['event'] == 'job_failed' and ev[-1]['detail']['reason'] == 'ABORTED:peer_asked'
    assert ev[-1]['scheduler_trigger'] == 'failure'
    assert ex.step(d.t) == {'mode': 'tick', 'commands': [{'kind': 'hold'}]}


def test_local_timeout_is_a_timeout_wake():
    ex = make(job_sim_limit_s=.5)
    d = Driver(ex)
    ex.hold(5.)
    d.run(1.)
    ev = ex.drain_events()
    assert ev[-1]['event'] == 'job_failed' and ev[-1]['detail']['reason'] == 'LOCAL_TIMEOUT'
    assert ev[-1]['scheduler_trigger'] == 'timeout'
    assert set(zox.EVENT_TO_TRIGGER.values()) - {None} <= set(zox.D_TRIGGERS)


def test_goto_targets_come_from_the_static_map_only():
    ex = make()
    assert ex._goto_goal('A') == (4.6 - .30 - zox.ZONE_APPROACH_M, 0.4)
    assert ex._goto_goal('C2') == (3.0 - zox.SLOT_STANDOFF_M, -0.85)
    assert ex._goto_goal('P2-2') == (zox.PICKUP_VIEW_X_M, -0.85)
    assert ex._goto_goal('door_1') == (2.2 + zox.DOOR_SIDE_M, 0.05)     # no estimate yet: assume the west side
    assert ex.goto('A')['arguments'] == {'target_zone': 'A'}


def _keys(value, path=''):
    if isinstance(value, dict):
        for k, v in value.items():
            yield f'{path}.{k}', k
            yield from _keys(v, f'{path}.{k}')
    elif isinstance(value, (list, tuple)):
        for i, v in enumerate(value):
            yield from _keys(v, f'{path}[{i}]')


def test_status_and_belief_carry_no_forbidden_keys():
    ex = make()
    d = Driver(ex)
    ex.deliver('o1', 'A2')
    d.run(1.)
    for payload in (ex.status(), ex.belief_projection()):
        for path, key in _keys(payload):
            assert key not in A_FORBIDDEN_KEYS, path
            assert not any(s in key for s in A_FORBIDDEN_SUBSTRINGS), path
    st = ex.status()
    assert st['holding']['answer'] == 'no' and st['blocked_ahead']['answer'] == 'unknown'
    assert st['localization']['level'] == 'unknown' and st['region'] == 'unknown'
    assert tuple(ex.belief_projection()) == A_BELIEF_KEYS


def test_action_record_adapter_matches_package_a_schema():
    ex = make()
    acks = [ex.hold(1.), ex.deliver('o1', 'A2'), ex.abort()]
    for i, ack in enumerate(acks):
        rec = zox.action_record(ack, run_id='t', condition='no_llm_scripted', seed=1, request_id=f'q{i}')
        assert tuple(rec) == A_ACTION_FIELDS
        assert rec['kind'] in zox.A_ACTION_KINDS and rec['local_state'] in zox.A_LOCAL_STATES
        assert set(rec['arguments']) <= set(A_COMMAND_ARGUMENT_KEYS)
    try:                                           # once PR #194 is merged, compare with the real module
        from harness import zone_study_contract as a
    except ImportError:
        return
    assert zox.A_ACTION_KINDS == a.ACTION_KINDS and zox.A_LOCAL_STATES == a.LOCAL_STATES
    assert a.action_record_violations(rec) == []


# ---------------------------------------------------------------- 3-robot host (simulator)
def test_team_host_feeds_each_executor_only_its_own_camera():
    pytest.importorskip('mujoco')                   # lazy: importing this file must stay simulator-free
    spec = {'map': 'zone_wide_door_tags_v2', 'seed': 703, 'goal': {'A': {'cyan': 1}, 'B': {'cyan': 1}, 'C': {'cyan': 1}},
            'extra_boxes': {'red': 2, 'green': 1}, 'contact_profile': 'cargo_noslip_v1', 'order_sheet': SHEET}
    student = {'mode': 'm1', 'calibration': 'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json',
               'skill_module': 'harness.wrist_zone_skill_v9', 'skill_class': 'WristZoneDeliveryV9'}
    seen = []

    def layer(host, kind, event, now):
        if kind == 'start':
            for rid in zox.ROBOTS:
                host.call(rid, 'look_around')
        else:
            seen.append(event)

    host = zox.OwnCamTeamHost(spec, student, root=ROOT, study_layer=layer)
    try:
        host.run(3.)
        assert host.eval_only['max_eq_active'] == 0 and host.contact_record['noslip_iterations'] > 0
        for rid, slot in host.robots.items():
            ex = slot.executor
            assert slot.frames and all(f['robot_id'] == rid and f['camera'] == 'robot_cam' for f in slot.frames)
            assert ex.cameras_seen == {'robot_cam'} and all(s.startswith('owncam_pf_v2:') for s in ex.pose_sources_seen)
            reach = _reachable(ex, limit=200_000)
            assert id(host) not in reach and id(host.world) not in reach
            assert not any(id(s.port) in reach or id(s.executor) in reach for r, s in host.robots.items() if r != rid)
            assert all(c['t'] >= 0 for c in slot.commands) and slot.commands[0]['kind'] == 'initial_servo_command'
        assert {e['robot_id'] for e in seen if e['event'] == 'job_started'} == set(zox.ROBOTS)
        assert host.eval_only['gt'] and 'robots' in host.eval_only['gt'][0]
    finally:
        host.close()
