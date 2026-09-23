"""Operator dialogue receipts are not robot inputs or fixture conversations."""
import json
from pathlib import Path
from contextlib import nullcontext
import os
import queue
import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from harness.communication_observer import CommunicationObserver, read_latest, LATEST_LIMIT
from harness.three_robot_plan import ROBOTS, fixture_plan
from harness.communication_overlay import (
    ObserverDialoguePanel, _wrapped_lines, render_dialogue_overlay)
from scripts import dispatch_native_process, dispatch_native_view
from scripts import smoke_communication_overlay
from scripts.smoke_communication_overlay import saved_dialogue
from scripts.three_robot_runtime import ThreeRobotRuntime


def events(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def test_actual_delivery_records_exact_text_and_separates_reason_vote_and_motor(tmp_path, capsys):
    team = ThreeRobotRuntime(tmp_path/'team', run_id='visible', mode='llm')
    try:
        original = '보를 함께 들어요.\n r3는 오른쪽!\x1b[31m\u202e'
        reply = {'request_id': 'visible-r1-plan-0', 'reason': '내부 판단 이유',
                 'proposal_id': None, 'plan_hash': None, 'accept': True,
                 'plan': fixture_plan(), 'message': original}
        team.deliver_replies({'r1': (reply, None, [{'model': 'offline-recorded-llm'}])},
                             phase='planning', turn=0, sim_time=1.25)
        assert [row['message'] for row in team.inbox['r2']] == [original]
        assert [row['message'] for row in team.inbox['r3']] == [original]
        assert team.inbox['r1'] == []
        rows = events(tmp_path/'team/conversation.jsonl')
        assert [r['kind'] for r in rows] == [
            'decision_explanation', 'proposal_vote', 'peer_message']
        peer = rows[-1]
        assert peer['text'] == original and peer['recipients'] == ['r2', 'r3']
        assert (peer['sender'], peer['turn'], peer['phase'], peer['sim_time_s']) == (
            'r1', 0, 'planning', 1.25)
        assert rows[0]['text'] == '내부 판단 이유'
        assert not any(r['kind'] == 'motor_command' for r in rows)
        latest = read_latest(tmp_path/'team/latest-dialogue.json')
        assert latest['fresh_peer_messages'] == 1
        assert latest['recent_messages'][0]['text'] == original
        terminal = capsys.readouterr().out
        assert 'peer_delivery ' in terminal and '\\u001b' in terminal
        assert '\\u202e' in terminal
        assert '\x1b' not in terminal
        assert '\u202e' not in terminal
        assert len((tmp_path/'team/latest-dialogue.json').read_bytes()) < LATEST_LIMIT
    finally:
        team.close(1.25)


def test_execution_delivery_uses_same_inbox_receipt_without_vote(tmp_path):
    team = ThreeRobotRuntime(tmp_path/'team', run_id='execution', mode='llm')
    try:
        reply = {'request_id': 'execution-r2-execute-3', 'reason': '보류 근거',
                 'message': 'r1, 왼쪽에서 기다려 주세요.', 'action': {'kind': 'wait'}}
        team.deliver_replies({'r2': (reply, None, [{'model': 'offline-recorded-llm'}])},
                             phase='execution', turn=3, sim_time=4.5)
        rows = events(tmp_path/'team/conversation.jsonl')
        assert [r['kind'] for r in rows] == ['decision_explanation', 'peer_message']
        assert rows[-1]['recipients'] == ['r1', 'r3']
        assert rows[-1]['phase'] == 'execution'
        assert all(r.get('text') != str(reply['action']) for r in rows)
    finally:
        team.close(4.5)


def test_saved_plan_fixture_and_synthetic_message_are_never_fresh_llm_dialogue(tmp_path):
    team = ThreeRobotRuntime(tmp_path/'team', run_id='saved-plan', mode='fixture',
                             plan_fixture=fixture_plan())
    try:
        assert read_latest(tmp_path/'team/latest-dialogue.json')['fresh_peer_messages'] == 0
        reply = {'request_id': 'fixture-r1-plan-0', 'reason': 'scripted fixture vote',
                 'proposal_id': None, 'plan_hash': None, 'accept': True,
                 'message': '합성 문장'}
        team.deliver_replies({'r1': (reply, None, [{'model': 'scripted-fixture-not-llm'}])},
                             phase='planning', turn=0, sim_time=0.)
        rows = events(tmp_path/'team/conversation.jsonl')
        assert rows[-1]['kind'] == 'fixture_peer_message'
        assert rows[-1]['source'] == 'fixture'
        latest = read_latest(tmp_path/'team/latest-dialogue.json')
        assert latest['fresh_peer_messages'] == 0 and latest['recent_messages'] == []
        assert '새 자연어 메시지 0개' in latest['status']
    finally:
        team.close(0.)


def test_actual_saved_plan_negotiation_has_votes_but_zero_new_natural_language(tmp_path):
    team = ThreeRobotRuntime(tmp_path/'team', run_id='saved-plan-votes', mode='fixture',
                             plan_fixture=fixture_plan())
    frame = {'own_bytes': b'own', 'top_bytes': b'top',
             'own_rgb': {'path': 'own.jpg'}, 'shared_top_rgb': {'path': 'top.jpg'},
             'frame_id': 1}
    frames = {rid: frame for rid in ROBOTS}
    try:
        for turn in (0, 1):
            team.negotiate(frames, {rid: [] for rid in ROBOTS}, turn, float(turn))
        assert all(call['model'] == 'scripted-fixture-not-llm' for call in team.calls)
        rows = events(tmp_path/'team/conversation.jsonl')
        assert sum(row['kind'] == 'proposal_vote' for row in rows) == 6
        assert not any(row['kind'] == 'peer_message' for row in rows)
        assert read_latest(tmp_path/'team/latest-dialogue.json')['fresh_peer_messages'] == 0
    finally:
        team.close(1.)


def test_latest_changes_only_when_message_or_mode_changes(tmp_path):
    observer = CommunicationObserver(tmp_path/'team', run_id='quiet', mode='llm')
    before = (tmp_path/'team/latest-dialogue.json').stat().st_mtime_ns
    observer.record_reply('r1', {'request_id': 'q1', 'reason': 'private', 'message': ''},
                          [{'model': 'offline-recorded-llm'}], phase='execution',
                          turn=0, sim_time_s=0.)
    assert (tmp_path/'team/latest-dialogue.json').stat().st_mtime_ns == before
    assert read_latest(tmp_path/'team/latest-dialogue.json')['fresh_peer_messages'] == 0
    (tmp_path/'team/latest-dialogue.json').write_bytes(b'{' + b'x' * LATEST_LIMIT)
    assert read_latest(tmp_path/'team/latest-dialogue.json') is None
    (tmp_path/'team/latest-dialogue.json').write_bytes(b'{broken')
    assert read_latest(tmp_path/'team/latest-dialogue.json') is None


def test_long_sidecar_preview_marks_truncation_but_jsonl_keeps_original(tmp_path):
    observer = CommunicationObserver(tmp_path/'team', run_id='long', mode='llm')
    message = '가' * 500
    observer.record_reply('r1', {'request_id': 'long-1', 'message': message},
                          [{'model': 'offline-recorded-llm'}], phase='planning',
                          turn=0, sim_time_s=0., delivered_to=('r2', 'r3'))
    assert events(tmp_path/'team/conversation.jsonl')[-1]['text'] == message
    preview = read_latest(tmp_path/'team/latest-dialogue.json')['recent_messages'][0]['text']
    assert preview == '가'*320 + '…'


def test_overlay_renders_recent_dialogue_and_ascii_fallback(tmp_path):
    observer = CommunicationObserver(tmp_path/'team', run_id='overlay', mode='llm')
    observer.record_reply('r3', {'request_id': 'o1', 'message': '함께 운반해요'},
                          [{'model': 'offline-recorded-llm'}], phase='execution',
                          turn=1, sim_time_s=2., delivered_to=('r1', 'r2'))
    state = read_latest(tmp_path/'team/latest-dialogue.json')
    pixels, korean = render_dialogue_overlay(state)
    assert pixels.shape == (330, 1000, 3) and pixels.dtype == np.uint8
    assert pixels.std() > 0
    ascii_pixels, ascii_korean = render_dialogue_overlay(state, font_paths=())
    assert not ascii_korean and ascii_pixels.shape == pixels.shape
    assert not np.array_equal(pixels, ascii_pixels) or not korean


def test_long_message_wraps_two_lines_with_visible_ellipsis():
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.load_default()
    draw = ImageDraw.Draw(Image.new('RGB', (200, 100)))
    lines = _wrapped_lines('via north route then south lane ' * 20,
                           draw, font, 145)
    assert len(lines) == 2 and lines[-1].endswith('…')
    assert all(draw.textlength(line, font=font) <= 145 for line in lines)


def test_bounded_panel_reads_only_new_sidecar_sequences(tmp_path):
    observer = CommunicationObserver(tmp_path/'team', run_id='panel', mode='llm')
    panel = ObserverDialoguePanel(observer.latest_path)
    operations = []
    viewer = SimpleNamespace(set_images=lambda value: operations.append(('images', value)),
                             set_texts=lambda value: operations.append(('texts', value)))
    mujoco = SimpleNamespace(MjrRect=lambda *args: args)
    assert panel.poll(viewer, mujoco, now=1.)
    assert [name for name, _ in operations] == ['texts', 'images']
    assert operations[1][1][0] == (12, 12, 1000, 330)
    assert not panel.poll(viewer, mujoco, now=1.1)
    assert not panel.poll(viewer, mujoco, now=1.3)
    observer.record_reply('r1', {'request_id': 'next', 'message': '새 메시지'},
                          [{'model': 'offline-recorded-llm'}], phase='planning',
                          turn=1, sim_time_s=1., delivered_to=('r2', 'r3'))
    assert panel.poll(viewer, mujoco, now=1.5)
    assert [name for name, _ in operations] == ['texts', 'images', 'texts', 'images']
    assert not panel.poll(viewer, mujoco, now=1.8)


def test_nonrealtime_view_polls_observer_panel_without_touching_physics(monkeypatch):
    calls = []
    view = dispatch_native_view.DispatchNativeView.__new__(dispatch_native_view.DispatchNativeView)
    view.viewer = SimpleNamespace(is_running=lambda: True)
    view.scene = SimpleNamespace(deadline=None)
    view.keys = queue.SimpleQueue()
    view.paused = False
    view.next_sync = time.monotonic()+100
    view.dialogue_panel = SimpleNamespace(poll=lambda *args: calls.append(args))
    view.poll()
    assert len(calls) == 1
    assert calls[0][0] is view.viewer


def test_isolated_process_polls_observer_panel_without_actor_state(monkeypatch, tmp_path):
    calls = []
    class Viewer:
        def __init__(self):
            self.count = 0
            self.cam = SimpleNamespace(type=None, lookat=np.zeros(3), distance=0.,
                                       azimuth=0., elevation=0.)
            self.user_scn = SimpleNamespace(flags={})
        def is_running(self):
            self.count += 1
            return self.count == 1
        def lock(self):
            return nullcontext()
        def close(self):
            pass
        def _sim(self):
            return None
    viewer = Viewer()
    fake = ModuleType('mujoco')
    fake.__path__ = []
    fake.MjModel = SimpleNamespace(from_binary_path=lambda path: object())
    fake.MjData = lambda model: object()
    fake.mj_stateSize = lambda model, kind: 0
    fake.mjtState = SimpleNamespace(mjSTATE_INTEGRATION=1)
    fake.mjtCamera = SimpleNamespace(mjCAMERA_FREE=1)
    fake.mjtRndFlag = SimpleNamespace(mjRND_SHADOW=1, mjRND_REFLECTION=2)
    fake_viewer = ModuleType('mujoco.viewer')
    fake_viewer.launch_passive = lambda *args, **kwargs: viewer
    fake.viewer = fake_viewer
    monkeypatch.setitem(sys.modules, 'mujoco', fake)
    monkeypatch.setitem(sys.modules, 'mujoco.viewer', fake_viewer)
    class Mailbox:
        def __init__(self, *args):
            self.array = np.zeros(dispatch_native_process.HEADER)
        def transact(self, callback):
            return callback(self.array)
        def close(self):
            pass
    monkeypatch.setattr(dispatch_native_process, 'StateMailbox', Mailbox)
    monkeypatch.setattr(dispatch_native_process, 'ObserverDialoguePanel',
                        lambda path: SimpleNamespace(poll=lambda *args: calls.append(args)))
    dispatch_native_process.observe(tmp_path, os.getpid(), tmp_path/'latest-dialogue.json')
    assert len(calls) == 1 and calls[0][0] is viewer


def test_saved_llm_smoke_labels_replay_and_rejects_fixture(tmp_path):
    source = tmp_path/'team.json'
    reply = {'request_id': 'real-r1-plan-0', 'message': 'r2와 함께 운반합니다.'}
    saved = {'mode': 'llm', 'calls': [{'request_id': reply['request_id'],
                                    'model': 'recorded-model', 'reply': reply}],
             'rounds': [{'turn': 0, 'replies': {'r1': reply}}]}
    source.write_text(json.dumps(saved), encoding='utf-8')
    state, digest, count = saved_dialogue(source)
    assert count == 1 and len(digest) == 64
    assert state['fresh_peer_messages'] == 0
    assert '재생' in state['status'] and state['recent_messages'][0]['source'] == 'saved_llm_replay'
    saved['calls'][0]['model'] = 'scripted-fixture-not-llm'
    source.write_text(json.dumps(saved), encoding='utf-8')
    with pytest.raises(ValueError, match='non-fixture'):
        saved_dialogue(source)


@pytest.mark.parametrize('diagnostic', (False, True))
def test_saved_dialogue_smoke_uses_common_primer_and_optional_marker(
        monkeypatch, tmp_path, diagnostic):
    source = tmp_path/'team.json'
    reply = {'request_id': 'real-r1-plan-0', 'message': '저장된 대화'}
    source.write_text(json.dumps({'mode': 'llm',
        'calls': [{'request_id': reply['request_id'], 'model': 'saved-model', 'reply': reply}],
        'rounds': [{'turn': 0, 'replies': {'r1': reply}}]}), encoding='utf-8')
    class Viewer:
        viewport = SimpleNamespace(left=0, bottom=0, width=1280, height=800)
        def __init__(self):
            self.images = self.texts = None
            self.sync_modes = []
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def is_running(self):
            return False
        def sync(self, *, state_only):
            self.sync_modes.append(state_only)
        def set_images(self, images):
            self.images = images
        def set_texts(self, texts):
            self.texts = texts
        def _sim(self):
            return None
    viewer = Viewer()
    fake = ModuleType('mujoco')
    fake.__path__ = []
    fake.MjModel = SimpleNamespace(from_xml_string=lambda xml: object())
    fake.MjData = lambda model: object()
    fake.MjrRect = lambda *args: args
    fake_viewer = ModuleType('mujoco.viewer')
    fake_viewer.launch_passive = lambda *args, **kwargs: viewer
    fake.viewer = fake_viewer
    monkeypatch.setitem(sys.modules, 'mujoco', fake)
    monkeypatch.setitem(sys.modules, 'mujoco.viewer', fake_viewer)
    audit = tmp_path/'audit.json'
    argv = ['--source', str(source), '--audit', str(audit), '--duration-s', '1']
    if diagnostic:
        argv.append('--diagnostic')
    assert smoke_communication_overlay.main(argv) == 0
    assert viewer.sync_modes == ([False] if diagnostic else [])
    if diagnostic:
        assert len(viewer.images) == 2
        assert viewer.images[1][0] == (490, 355, 300, 90)
        assert viewer.images[1][1][0, 0].tolist() == [255, 0, 220]
        assert 'UGRP OVERLAY DIAGNOSTIC' in viewer.texts
        assert json.loads(audit.read_text())['viewer_viewport']['width'] == 1280
    else:
        assert viewer.images[0] == (12, 12, 1000, 330)
        assert 'Peer dialogue' in viewer.texts
        assert json.loads(audit.read_text())['viewer_viewport'] is None
