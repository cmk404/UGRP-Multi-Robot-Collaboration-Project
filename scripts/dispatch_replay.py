"""Record a finished dispatch run and replay it afterwards in MuJoCo's own window.

The recorder is observer-only: it copies the compiled model once and samples
joint/mocap positions on the physics owner after each step. Nothing it stores
is read by actors, controllers or the referee. Replay sets those recorded
positions kinematically (``mj_forward``); it does not step physics again.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import queue
import sys
import time

SCHEMA = 'ugrp.dispatch_replay.v1'
DEFAULT_FPS = 30
KEY_SPACE, KEY_R, KEY_Q, KEY_ESCAPE, KEY_RIGHT, KEY_LEFT = 32, 82, 81, 256, 262, 263
SEEK_S = 5.


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ReplayRecorder:
    """Sample world state at a fixed SIM rate; write once at close."""

    def __init__(self, world, output: Path, *, fps: int = DEFAULT_FPS):
        import mujoco
        if not isinstance(fps, int) or not 1 <= fps <= 60:
            raise ValueError('replay fps must be an integer in 1..60')
        self.world = world
        self.dir = Path(output) / 'replay'
        self.dir.mkdir(parents=True, exist_ok=False)
        self.period = 1. / fps
        self.fps = fps
        mujoco.mj_saveModel(world.model, str(self.dir / 'model.mjb'), None)
        self.times, self.qpos, self.mocap_pos, self.mocap_quat = [], [], [], []
        self.labels: list[list] = []
        self.next_s = float(world.data.time)
        self.closed = None

    def sample(self, label: str | None = None) -> None:
        data = self.world.data
        now = float(data.time)
        if now + 1e-9 < self.next_s:
            return
        self.next_s = now + self.period
        self.times.append(now)
        self.qpos.append(data.qpos.copy())
        self.mocap_pos.append(data.mocap_pos.copy())
        self.mocap_quat.append(data.mocap_quat.copy())
        if label and (not self.labels or self.labels[-1][1] != label):
            self.labels.append([len(self.times) - 1, label])

    def close(self) -> dict:
        """Write states and a manifest; repeated calls return the same summary."""
        if self.closed is not None:
            return self.closed
        import numpy as np
        states = self.dir / 'states.npz'
        model = self.world.model
        if self.times:
            arrays = {'time': np.asarray(self.times), 'qpos': np.stack(self.qpos),
                      'mocap_pos': np.stack(self.mocap_pos), 'mocap_quat': np.stack(self.mocap_quat)}
        else:
            arrays = {'time': np.zeros(0), 'qpos': np.zeros((0, model.nq)),
                      'mocap_pos': np.zeros((0, model.nmocap, 3)), 'mocap_quat': np.zeros((0, model.nmocap, 4))}
        np.savez_compressed(states, **arrays)
        (self.dir / 'labels.json').write_text(json.dumps(self.labels, ensure_ascii=False) + '\n')
        manifest = {'schema': SCHEMA, 'fps_sim': self.fps, 'frames': len(self.times),
                    'sim_start_s': self.times[0] if self.times else None,
                    'sim_end_s': self.times[-1] if self.times else None,
                    'nq': int(model.nq), 'nmocap': int(model.nmocap),
                    'files_sha256': {name: _sha(self.dir / name)
                                     for name in ('model.mjb', 'states.npz', 'labels.json')},
                    'scope': 'observer-only post-run kinematic replay; never actor, controller or referee input'}
        (self.dir / 'replay.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
        self.closed = {'dir': str(self.dir), 'frames': manifest['frames'],
                       'sim_end_s': manifest['sim_end_s'], 'manifest_sha256': _sha(self.dir / 'replay.json')}
        return self.closed


def load(run_dir: Path):
    """Load a recorded replay after verifying its manifest hashes."""
    import mujoco
    import numpy as np
    directory = Path(run_dir) / 'replay'
    manifest_path = directory / 'replay.json'
    if not manifest_path.is_file():
        raise ValueError(f'no recorded replay in {Path(run_dir)}')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('schema') != SCHEMA:
        raise ValueError('unsupported replay schema')
    for name, expected in manifest['files_sha256'].items():
        if _sha(directory / name) != expected:
            raise ValueError(f'replay file changed after recording: {name}')
    model = mujoco.MjModel.from_binary_path(str(directory / 'model.mjb'))
    with np.load(directory / 'states.npz') as stored:
        states = {key: stored[key] for key in stored.files}
    if len(states['time']) == 0:
        raise ValueError('replay contains no frames')
    if states['qpos'].shape[1] != model.nq:
        raise ValueError('replay qpos does not match its model')
    labels = json.loads((directory / 'labels.json').read_text())
    return model, states, labels, manifest


def frame_at(times, t: float) -> int:
    return max(0, min(len(times) - 1, bisect.bisect_right(times, t + 1e-9) - 1))


def label_at(labels, index: int) -> str:
    starts = [start for start, _ in labels]
    position = bisect.bisect_right(starts, index) - 1
    return labels[position][1] if position >= 0 else ''


def dialogue(run_dir: Path) -> list:
    """Peer messages from the run's own communication record, by SIM time.

    Observer-only overlay; the file is the run's saved output, not replay input.
    """
    path = Path(run_dir) / 'team' / 'conversation.jsonl'
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text().splitlines():
        row = json.loads(line)
        if row.get('kind') == 'peer_message' and row.get('text') and row.get('sim_time_s') is not None:
            rows.append((float(row['sim_time_s']), row['sender'], row.get('phase', ''), row['text']))
    return sorted(rows, key=lambda row: row[0])


def dialogue_at(rows, t: float, *, keep: int = 3, window_s: float = 30., width: int = 110) -> str:
    """The last ``keep`` messages sent at or before ``t`` and within ``window_s``."""
    recent = [row for row in rows if row[0] <= t + 1e-9 and t - row[0] <= window_s][-keep:]
    lines = []
    for sim_s, sender, phase, text in recent:
        text = text.encode('ascii', 'replace').decode()
        line = f'{sim_s:5.1f}s {sender} [{phase}] {text}'
        lines.append(line if len(line) <= width else line[:width - 3] + '...')
    return '\n'.join(lines)


def _close(viewer) -> None:
    viewer.close()
    # MuJoCo 3.12 close signals its render thread; wait before glfw teardown.
    deadline = time.monotonic() + 10
    while viewer._sim() is not None and time.monotonic() < deadline:
        time.sleep(.01)


def play(run_dir: Path, *, speed: float = 1., max_wall_s: float | None = None) -> int:
    """Play the recorded run in MuJoCo's native window until it is closed.

    Returns the last displayed frame index. ``max_wall_s`` bounds smoke checks.
    """
    import mujoco
    import mujoco.viewer
    if not math.isfinite(speed) or not .1 <= speed <= 16:
        raise ValueError('replay speed must be within 0.1..16')
    model, states, labels, manifest = load(run_dir)
    messages = dialogue(run_dir)
    data = mujoco.MjData(model)
    times = states['time'].tolist()
    start, end = times[0], times[-1]
    keys = queue.SimpleQueue()
    viewer = mujoco.viewer.launch_passive(model, data, key_callback=keys.put,
                                          show_left_ui=False, show_right_ui=False)
    with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [.55, -2., .1]
        viewer.cam.distance = 4.8
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -55
        viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
    print(f'재생: SIM {end - start:.1f}초, {speed:g}× · Space 일시정지/재개 · ←/→ {SEEK_S:g}초 이동 · '
          'R 처음부터 · Q 또는 창 닫기로 종료', flush=True)
    playhead, paused, shown = start, False, -1
    last = opened = time.monotonic()
    while viewer.is_running():
        if max_wall_s is not None and time.monotonic() - opened >= max_wall_s:
            break
        while not keys.empty():
            key = keys.get()
            if key == KEY_SPACE:
                paused = not paused
            elif key == KEY_R:
                playhead, paused = start, False
            elif key == KEY_RIGHT:
                playhead = min(end, playhead + SEEK_S)
            elif key == KEY_LEFT:
                playhead = max(start, playhead - SEEK_S)
            elif key in (KEY_Q, KEY_ESCAPE):
                _close(viewer)
                return shown
        now = time.monotonic()
        if not paused:
            playhead = min(end, playhead + (now - last) * speed)
        last = now
        index = frame_at(times, playhead)
        if index != shown:
            with viewer.lock():
                data.qpos[:] = states['qpos'][index]
                if model.nmocap:
                    data.mocap_pos[:] = states['mocap_pos'][index]
                    data.mocap_quat[:] = states['mocap_quat'][index]
                data.time = times[index]
                mujoco.mj_forward(model, data)
            shown = index
        status = 'PAUSED' if paused else ('END' if playhead >= end else f'{speed:g}x')
        texts = [(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                  f'SIM {times[index]:.1f}/{end:.1f}s  {status}', label_at(labels, index))]
        said = dialogue_at(messages, times[index])
        if said:
            texts.append((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                          'TEAM TALK', said))
        viewer.set_texts(texts)
        viewer.sync()
        time.sleep(1 / 60)
    _close(viewer)
    return shown


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Replay a finished dispatch run in MuJoCo\'s native window')
    parser.add_argument('run_dir', type=Path, help='dispatch output directory containing replay/')
    parser.add_argument('--speed', type=float, default=1., help='playback speed, 0.1..16')
    args = parser.parse_args(argv)
    try:
        play(args.run_dir, speed=args.speed)
        return 0
    except ValueError as error:
        print(f'replay: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
