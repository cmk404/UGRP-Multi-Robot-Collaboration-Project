"""Post-run replay for the wider zone arena (observer-only).

Recording reuses ``scripts.dispatch_replay.ReplayRecorder`` unchanged; the
free-camera framing for the 6.45 m arena is stored next to it in
``replay/view.json``. Playback mirrors ``dispatch_replay.play`` with that
framing. Kept separate so the pinned dispatch replay source is unchanged.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import time
from pathlib import Path

from scripts.dispatch_replay import (KEY_ESCAPE, KEY_LEFT, KEY_Q, KEY_R, KEY_RIGHT, KEY_SPACE, SEEK_S,
                                     ReplayRecorder, _close, frame_at, label_at, load)


def arena_view(bounds):
    # 7.5 m frames a 2.3 m deep floor (retired zone_open, kept for replays); deeper floors step back.
    depth = bounds[3]-bounds[2]
    return {'lookat': [(bounds[0]+bounds[1])/2, (bounds[2]+bounds[3])/2, .1],
            'distance': 7.5 if depth <= 2.3 else round(7.5+.9*(depth-2.3), 2), 'azimuth': 90, 'elevation': -60}


class ZoneReplayRecorder(ReplayRecorder):
    def __init__(self, world, output: Path, *, view: dict, **kwargs):
        super().__init__(world, output, **kwargs)
        self.view = dict(view)

    def close(self) -> dict:
        closed = super().close()
        (self.dir/'view.json').write_text(json.dumps({'scope': 'observer-only replay framing',
                                                      **self.view}, indent=2) + '\n')
        return closed


def play(run_dir: Path, *, speed: float = 1., max_wall_s: float | None = None) -> int:
    import mujoco
    import mujoco.viewer
    if not math.isfinite(speed) or not .1 <= speed <= 16:
        raise ValueError('replay speed must be within 0.1..16')
    model, states, labels, _ = load(run_dir)
    view_path = Path(run_dir)/'replay'/'view.json'
    view = json.loads(view_path.read_text()) if view_path.is_file() else {}
    data = mujoco.MjData(model)
    times = states['time'].tolist()
    start, end = times[0], times[-1]
    keys = queue.SimpleQueue()
    viewer = mujoco.viewer.launch_passive(model, data, key_callback=keys.put,
                                          show_left_ui=False, show_right_ui=False)
    with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = view.get('lookat', [2.2, -2., .1])
        viewer.cam.distance = view.get('distance', 7.5)
        viewer.cam.azimuth = view.get('azimuth', 90)
        viewer.cam.elevation = view.get('elevation', -60)
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
        viewer.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                          f'SIM {times[index]:.1f}/{end:.1f}s  {status}', label_at(labels, index)))
        viewer.sync()
        time.sleep(1 / 60)
    _close(viewer)
    return shown


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Replay a finished zone run in MuJoCo\'s native window')
    parser.add_argument('run_dir', type=Path, help='zone output directory containing replay/')
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
