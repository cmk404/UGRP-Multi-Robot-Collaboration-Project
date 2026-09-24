"""Observer-only recorder pressure, error, and MP4 cadence checks."""
from __future__ import annotations

from concurrent.futures import Future
import json
from pathlib import Path
import shutil
import subprocess
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from scripts import dispatch_async_video as video_module
from sim.snapshot_render import RenderBatch, SnapshotBackpressure


def batch(frame_id: int, sim_time: float) -> RenderBatch:
    rgb = np.full((8, 8, 3), frame_id * 23, dtype=np.uint8)
    return RenderBatch(frame_id, sim_time, {(None, 'cctv_warehouse'): rgb})


class FakeWorld:
    def __init__(self, futures):
        self.data = SimpleNamespace(time=0.)
        self.model = object()
        self.futures = iter(futures)
        self.requested = []
        self.actor_state = {'own_rgb': b'unchanged', 'actions': [1, 2, 3]}

    def render_snapshot_async(self, cameras):
        self.requested.append(tuple(cameras))
        value = next(self.futures)
        if isinstance(value, Exception):
            raise value
        return value


class CountingSink:
    def __init__(self):
        self.writes = 0
    def write(self, data):
        self.writes += 1
        return len(data)
    def close(self):
        pass


@pytest.fixture
def fake_encoder(monkeypatch):
    def initialize(self, world, path, fps):
        self.world, self.fps, self.frames = world, fps, 0
        self.next_sim_time = float(world.data.time)
        self.stage = 'TEST'
        self.process = SimpleNamespace(stdin=CountingSink())
    monkeypatch.setattr(video_module.Video, '__init__', initialize)
    monkeypatch.setattr(video_module.Video, 'close', lambda self: None)
    monkeypatch.setattr(video_module, 'beam_pose',
                        lambda _data, _model: {'position': [0., 0., .2]})


def test_slow_future_keeps_physics_capture_bounded_and_repeats(tmp_path, fake_encoder):
    slow = Future()
    world = FakeWorld([slow])
    video = video_module.AsyncVideo(world, tmp_path / 'video.mp4', 10)
    try:
        video.capture()
        elapsed = []
        for index in range(1, 12):
            world.data.time = index / 10
            start = time.perf_counter()
            video.capture()
            elapsed.append(time.perf_counter() - start)
        assert max(elapsed) < .05
        assert world.requested == [((None, 'cctv_warehouse'),)]
        assert world.actor_state == {'own_rgb': b'unchanged', 'actions': [1, 2, 3]}
        slow.set_result(batch(7, 0.))
        video.close()
        evidence = json.loads((tmp_path / 'video.frames.json').read_text())
        assert evidence['written_frames'] == 12
        assert evidence['captured_frames'] == 1
        assert evidence['repeated_frames'] == 11
        assert video.process.stdin.writes == 12
        assert evidence['frames'][0]['observed_at_s'] == 0.
        assert evidence['frames'][0]['frame_id'] == 7
        assert all(row['status'] == 'repeated' for row in evidence['frames'][1:])
    finally:
        if not video.closed:
            slow.set_result(batch(7, 0.))
            video.close()


def test_backpressure_and_worker_exception_leave_explicit_evidence(tmp_path, fake_encoder):
    world = FakeWorld([SnapshotBackpressure('full'), SnapshotBackpressure('full')])
    video = video_module.AsyncVideo(world, tmp_path / 'pressure.mp4', 10)
    video.capture()
    world.data.time = .1
    video.capture()
    video.close()
    evidence = json.loads((tmp_path / 'pressure.frames.json').read_text())
    assert evidence['written_frames'] == evidence['repeated_frames'] == 2
    assert evidence['captured_frames'] == 0
    assert evidence['error'] is None

    failed = Future()
    world = FakeWorld([failed])
    video = video_module.AsyncVideo(world, tmp_path / 'failed.mp4', 10)
    video.capture()
    failed.set_exception(RuntimeError('render failed'))
    with pytest.raises(RuntimeError, match='render failed'):
        video.close()
    evidence = json.loads((tmp_path / 'failed.frames.json').read_text())
    assert evidence['error'] == 'render failed'
    assert evidence['captured_frames'] == 0
    assert evidence['frames'][0]['status'] == 'pending'


@pytest.mark.skipif(not shutil.which('ffmpeg') or not shutil.which('ffprobe'),
                    reason='ffmpeg/ffprobe unavailable')
def test_mp4_preserves_requested_sim_frame_count_and_repetition_metadata(tmp_path, monkeypatch):
    monkeypatch.setattr(video_module, 'beam_pose',
                        lambda _data, _model: {'position': [0., 0., .2]})
    first, second, slow = Future(), Future(), Future()
    first.set_result(batch(1, 0.))
    second.set_result(batch(2, .1))
    world = FakeWorld([first, second, slow])
    path = tmp_path / 'observer.mp4'
    video = video_module.AsyncVideo(world, path, 10)
    try:
        video.capture()
        video.pending.result(timeout=5)
        world.data.time = .1
        video.capture()
        video.pending.result(timeout=5)
        world.data.time = .2
        video.capture()
        world.data.time = .3
        video.capture()
        world.data.time = .4
        video.capture()
        slow.set_result(batch(3, .2))
        video.close()
        evidence = json.loads(path.with_suffix('.frames.json').read_text())
        assert evidence['written_frames'] == len(evidence['frames']) == 5
        assert evidence['captured_frames'] == 3
        assert evidence['repeated_frames'] == 2
        assert evidence['error'] is None
        assert world.requested == [((None, 'cctv_warehouse'),)] * 3
        assert world.actor_state == {'own_rgb': b'unchanged', 'actions': [1, 2, 3]}
        probe = subprocess.check_output([
            'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
            '-show_entries', 'stream=nb_read_frames,avg_frame_rate',
            '-of', 'json', str(path)], text=True)
        stream = json.loads(probe)['streams'][0]
        assert int(stream['nb_read_frames']) == 5
        assert stream['avg_frame_rate'] == '10/1'
        capture = cv2.VideoCapture(str(path))
        try:
            decoded = []
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                decoded.append(frame)
            assert len(decoded) == 5
            # Index 3 repeats the captured index 2 with a visible disclosure.
            roi = np.s_[67:91, 10:460, :]
            assert np.mean(np.abs(decoded[3][roi].astype(float)
                                  - decoded[2][roi].astype(float))) > 1.
        finally:
            capture.release()
    finally:
        if not video.closed:
            slow.set_result(batch(3, .2))
            video.close()
