"""Standard dispatch computes first, then replays recorded states in the native window."""
from types import SimpleNamespace

import pytest

from scripts import dispatch_replay, run_dispatch_e2e, sim_dispatch


def _wrapper(monkeypatch, tmp_path, argv):
    invoked, played = [], []
    monkeypatch.setattr(sim_dispatch.sys, 'platform', 'darwin')
    monkeypatch.setattr(sim_dispatch.sys.stdin, 'isatty', lambda: False)
    # Imported above: urllib imports macOS-only _scproxy when first loaded under a patched platform.
    monkeypatch.setattr(run_dispatch_e2e, 'main', lambda args: invoked.append(args) or 1)
    monkeypatch.setattr(dispatch_replay, 'play', lambda output, speed: played.append((output, speed)) or 0)
    code = sim_dispatch.main(['--plan-replay', 'saved.json', '--output', str(tmp_path / 'run'),
                              '--grasp-model-dir', str(tmp_path / 'g'), '--stage-model-dir', str(tmp_path / 's'),
                              *argv])
    return code, invoked, played


def test_default_window_run_records_then_replays_at_selected_speed(monkeypatch, tmp_path):
    code, invoked, played = _wrapper(monkeypatch, tmp_path, ['--realtime-factor', '2'])
    assert code == 1  # the run's own exit code is preserved after replay
    assert '--record-replay' in invoked[0] and '--viewer' not in invoked[0]
    assert played == [(tmp_path / 'run', 2.)]


def test_live_view_and_realtime_keep_the_existing_live_window(monkeypatch, tmp_path):
    _, invoked, played = _wrapper(monkeypatch, tmp_path, ['--live-view'])
    assert '--viewer' in invoked[0] and '--record-replay' not in invoked[0] and not played
    _, invoked, played = _wrapper(monkeypatch, tmp_path, ['--realtime-control'])
    assert '--viewer' in invoked[0] and '--record-replay' not in invoked[0] and not played


def test_headless_runs_neither_record_nor_open_a_window(monkeypatch, tmp_path):
    _, invoked, played = _wrapper(monkeypatch, tmp_path, ['--headless'])
    assert '--viewer' not in invoked[0] and '--record-replay' not in invoked[0] and not played


@pytest.mark.parametrize('extra', [['--headless'], ['--realtime-control']])
def test_live_view_rejects_headless_or_realtime(monkeypatch, tmp_path, extra):
    with pytest.raises(SystemExit) as error:
        _wrapper(monkeypatch, tmp_path, ['--live-view', *extra])
    assert error.value.code == 2


def test_e2e_forwards_record_replay_only_to_skills(monkeypatch, tmp_path):
    from scripts import run_dispatch_e2e
    seen = []
    monkeypatch.setattr('scripts.run_dispatch_skills.run', lambda args: seen.append(args) or 0)
    base = ['--output', str(tmp_path / 'one'), '--grasp-model-dir', str(tmp_path), '--stage-model-dir', str(tmp_path)]
    assert run_dispatch_e2e.main([*base, '--record-replay']) == 0 and seen[-1].record_replay
    with pytest.raises(SystemExit):
        run_dispatch_e2e.main(['--output', str(tmp_path / 'two'), '--executor', 'raw', '--record-replay'])


def test_sim_cli_routes_replay(monkeypatch):
    from scripts.sim_cli import main
    seen = []
    monkeypatch.setattr(dispatch_replay, 'main', lambda argv: seen.append(argv) or 0)
    assert main(['replay', 'outputs/x', '--speed', '2']) == 0
    assert seen == [['outputs/x', '--speed', '2']]


XML = """<mujoco><worldbody>
<body name="box" pos="0 0 .1"><freejoint/><geom type="box" size=".05 .05 .05"/></body>
<body name="marker" mocap="true" pos="1 0 0"><geom type="sphere" size=".02" contype="0" conaffinity="0"/></body>
</worldbody></mujoco>"""


def _recorded(tmp_path, seconds=1.):
    mujoco = pytest.importorskip('mujoco')
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    recorder = dispatch_replay.ReplayRecorder(SimpleNamespace(model=model, data=data), tmp_path, fps=10)
    while data.time < seconds - 1e-9:
        mujoco.mj_step(model, data)
        data.mocap_pos[0, 0] = 1. + data.time
        recorder.sample('PAIR APPROACH' if data.time < .5 else 'PAIR TRANSIT')
    return recorder, model, data


def test_recorder_round_trip_reproduces_sampled_states(tmp_path):
    recorder, model, data = _recorded(tmp_path)
    summary = recorder.close()
    assert recorder.close() is summary
    assert 10 <= summary['frames'] <= 11
    loaded, states, labels, manifest = dispatch_replay.load(tmp_path)
    assert loaded.nq == model.nq and manifest['frames'] == summary['frames']
    assert states['qpos'][-1] == pytest.approx(recorder.qpos[-1])
    assert states['mocap_pos'][-1][0, 0] == pytest.approx(1. + states['time'][-1])
    # Freely falling box: recorded height strictly decreases before contact.
    assert states['qpos'][1][2] < states['qpos'][0][2]
    assert [label for _, label in labels] == ['PAIR APPROACH', 'PAIR TRANSIT']
    times = states['time'].tolist()
    assert dispatch_replay.frame_at(times, -5.) == 0
    assert dispatch_replay.frame_at(times, times[3] + .01) == 3
    assert dispatch_replay.frame_at(times, 99.) == len(times) - 1
    assert dispatch_replay.label_at(labels, len(times) - 1) == 'PAIR TRANSIT'


def test_replay_refuses_changed_files_and_existing_recordings(tmp_path):
    recorder, _, _ = _recorded(tmp_path, seconds=.3)
    recorder.close()
    with pytest.raises(FileExistsError):
        _recorded(tmp_path, seconds=.1)
    (tmp_path / 'replay' / 'labels.json').write_text('[]\n')
    with pytest.raises(ValueError, match='changed'):
        dispatch_replay.load(tmp_path)
    with pytest.raises(ValueError, match='no recorded replay'):
        dispatch_replay.load(tmp_path / 'missing')


def test_play_rejects_out_of_range_speed(tmp_path):
    pytest.importorskip('mujoco')
    with pytest.raises(ValueError, match='speed'):
        dispatch_replay.play(tmp_path, speed=0.)


def test_real_native_replay_window_reaches_the_last_frame(tmp_path):
    import os
    if os.environ.get('UGRP_TEST_NATIVE_VIEWER') != '1':
        pytest.skip('explicit native display probe only')
    recorder, _, _ = _recorded(tmp_path)
    recorder.close()
    last = dispatch_replay.play(tmp_path, speed=16., max_wall_s=1.)
    assert last == recorder.closed['frames'] - 1
