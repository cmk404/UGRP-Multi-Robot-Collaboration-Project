"""Pair beam v2: band-centre perception, posture commit rules, candidate executor status channel."""
import base64
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

from harness import owncam_pair_beam_v2 as ob2
from harness import team_carry_status as tcs

ROOT = Path(__file__).resolve().parents[1]
FIX = ROOT / 'tests/fixtures/owncam_pair_v2'
FRAMES = json.loads((FIX / 'frames.json').read_text())


def _frame(name):
    raw = (FIX / name).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FRAMES[name]['sha256']
    return base64.b64encode(raw).decode(), {int(k): v for k, v in FRAMES[name]['own_pose_commands'].items()}


def test_search_view_band_at_the_bottom_is_clipped_not_the_beam_end():
    # v1 read this frame as grip 0.317 m "end visible" (GT 0.258 m): the lime stops at the black band.
    image, pose = _frame('search_611_r1_00072.jpg')
    obs = ob2.observe_beam(image, pose)
    assert obs['reason'] == 'BAND_CLIPPED' and not obs['end_visible']


def test_p45_view_band_centre_matches_gt_despite_end_face_clip():
    image, pose = _frame('p45_611_r1_00073.jpg')
    obs = ob2.observe_beam(image, pose)
    assert obs['reason'] == 'BAND_VISIBLE' and obs['grip_source'] == 'band_centre'
    assert abs(obs['grip_base_m'][0] - FRAMES['p45_611_r1_00073.jpg']['gt_grip_x_m_eval_only']) < .015
    assert 0.020 <= obs['band']['length_m'] <= 0.060


def test_posture_table_thresholds_are_monotone():
    assert ob2.order() == ['search', 'p45', 'inspect']
    assert ob2.look_posture(.40)[0] == 'search' and ob2.look_posture(.30)[0] == 'p45'
    assert ob2.look_posture(.25)[0] == 'inspect'


# ---------------- candidate executor status channel ----------------
def _msg(rid='r1', seq=1, state='aligning', t=1.):
    return {'robot_id': rid, 'task_id': 'beam', 'seq': seq, 'state': state, 'sent_at_s': t}


@pytest.mark.parametrize('bad', [
    {**_msg(), 'text': 'I am almost there'},                     # free text
    {**_msg(), 'pose': [1.0, 2.0]},                              # coordinates / GT
    {**_msg(), 'state': 'almost_ready'},                         # not in the fixed enum
    {k: v for k, v in _msg().items() if k != 'seq'},
])
def test_channel_rejects_anything_outside_the_fixed_contract(bad):
    ch = tcs.StatusChannel('beam', ('r1', 'r2'))
    assert not ch.publish(bad, 1.) and ch.rejected and not ch.log


def test_channel_rejects_stale_seq_wrong_sender_and_messages_after_abort():
    ch = tcs.StatusChannel('beam', ('r1', 'r2'))
    assert ch.publish(_msg(seq=2), 1.)
    assert not ch.publish(_msg(seq=2, t=1.4), 1.4)
    assert not ch.publish(_msg(rid='r3'), 1.)
    assert ch.publish(_msg(seq=3, state='abort', t=2.), 2.)
    assert not ch.publish(_msg(seq=4, state='ready', t=2.4), 2.4)


def test_publisher_heartbeat_and_state_changes():
    ch = tcs.StatusChannel('beam', ('r1', 'r2'))
    pub = tcs.StatusPublisher(ch, 'r1')
    for t in (0., .1, .2, .45, .5):
        pub.tick('aligning', t)
    pub.tick('ready', .55)
    assert [m['state'] for m in ch.log] == ['aligning', 'aligning', 'ready'] and [m['seq'] for m in ch.log] == [1, 2, 3]


def test_wait_verdicts():
    alive_aligning = {'r2': {'state': 'aligning', 'age_s': .3, 'alive': True}}
    assert tcs.wait_verdict(alive_aligning, 30., 20.)['why'] == 'partner_aligning'
    assert tcs.wait_verdict(alive_aligning, 80., 20.) == {'verdict': 'ABORT', 'why': 'partner_align_wait_exceeded'}
    assert tcs.wait_verdict({'r2': {'state': 'abort', 'age_s': .1, 'alive': True}}, 1., 20.)['verdict'] == 'ABORT'
    assert tcs.wait_verdict({'r2': {'state': 'aligning', 'age_s': 5., 'alive': False}}, 5., 20.)['why'] == 'r2_silent'
    assert tcs.wait_verdict({'r2': {'state': 'ready', 'age_s': .1, 'alive': True}}, 25., 20.)['why'] == 'barrier_limit'
    assert tcs.wait_verdict({'r2': {'state': 'ready', 'age_s': .1, 'alive': True}}, 5., 20.)['verdict'] == 'WAIT'


# ---------------- posture commit rules in the student ----------------
def _study():
    spec = importlib.util.spec_from_file_location('study_v2', ROOT / 'scripts/study_owncam_pair_beam.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Port:
    def __init__(self):
        self.applied = []
        self.fid = 0

    def capture(self):
        self.fid += 1
        return {'frame_id': self.fid, 'sim_time': 0., 'image': '', 'sha256': 'x' * 64,
                'actuator_state': {'servo_pulses': {'1': 2000, '3': 740, '4': 2320, '5': 1320, '6': 1500}}}

    def apply(self, cmd, now):
        self.applied.append(cmd)

    def hold(self, now):
        pass


class _Arm:
    until, events = 0., []

    def __init__(self):
        self.queued = []

    def queue(self, pose, now, **kw):
        self.queued.append(pose)


def _student(study, monkeypatch, reading):
    monkeypatch.setattr(study.ob2, 'observe_beam', lambda image, pose: dict(reading))
    st = study.PairStudent('r1', _Port(), _Arm(), lambda key: None, lambda *a, **k: None)
    st.state, st.look_name = 'align', 'p45'
    return st


def test_clipped_band_goes_nearer_not_farther(monkeypatch):
    study = _study()
    st = _student(study, monkeypatch, {'visible': True, 'end_visible': False, 'reason': 'BAND_CLIPPED'})
    st._align(1., True)
    assert st.look_name == 'inspect'                  # v1 went back to search here (the 31d16b0 loop)


def test_repeated_switching_without_motion_commits_and_backs_up(monkeypatch):
    study = _study()
    st = _student(study, monkeypatch, {'visible': False, 'end_visible': False, 'reason': 'BEAM_NOT_VISIBLE'})
    st.switches_since_motion = study.POSTURE_COMMIT_SWITCHES
    st._align(1., True)
    assert st.look_name == 'p45' and st.posture_commits == 1 and st.port.applied[-1]['forward'] < 0
    assert st.switches_since_motion == 0


def test_partner_abort_on_the_channel_stops_the_robot():
    study = _study()
    ch = tcs.StatusChannel('beam', ('r1', 'r2'))
    st = study.PairStudent('r1', _Port(), _Arm(), lambda key: None, lambda *a, **k: None,
                           status=(ch, tcs.StatusPublisher(ch, 'r1')))
    st.state = 'wait_lift'
    ch.publish({'robot_id': 'r2', 'task_id': ch.task_id, 'seq': 1, 'state': 'abort',
                'sent_at_s': 1.}, 1.)
    st.tick(1.1)
    assert st.state == 'failed' and st.failure == 'PARTNER_ABORT' and ch.latest['r1'].state == 'abort'
