"""Latest-state native observer in its own process; never a control input.

The physics owner only attempts a nonblocking copy into a bounded mailbox.
The reader releases that mailbox before updating or drawing the native window.
"""
from __future__ import annotations

import fcntl
import json
import mmap
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np

from scripts.dispatch_native_view import DispatchNativeView

HEADER = 6
SEQ, READY, PAUSES, QUIT, STOP, DISPLAYED = range(HEADER)


class StateMailbox:
    def __init__(self, path, size, *, create=False):
        self.file = open(path, 'w+b' if create else 'r+b')
        if create:
            self.file.truncate((HEADER + size) * 8)
        self.mapping = mmap.mmap(self.file.fileno(), 0)
        self.array = np.ndarray((HEADER + size,), dtype=np.float64, buffer=self.mapping)

    def transact(self, operation):
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return None
        try:
            return operation(self.array)
        finally:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)

    def close(self):
        self.array = None
        self.mapping.close()
        self.file.close()


class IsolatedDispatchNativeView(DispatchNativeView):
    def __init__(self, scene, *, realtime_factor=1.):
        import mujoco
        self.scene = scene
        self.factor = realtime_factor
        self.paused = False
        self.rebase_pace = False
        self.pause_seen = 0
        self.next_poll = self.next_sync = 0.
        self.pace_sim = scene.time()
        self.pace_wall = self.last_tick_wall = time.monotonic()
        self.state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
        size = mujoco.mj_stateSize(scene.world.model, self.state_kind)
        self.state = np.empty(size)
        self.scratch = tempfile.TemporaryDirectory(prefix='ugrp-observer-')
        directory = Path(self.scratch.name)
        self.mailbox = StateMailbox(directory / 'state', size, create=True)
        self.process = None
        self.log = None
        self.closed = False
        self.stats = {'mode': 'isolated_latest_state', 'published': 0, 'busy_skips': 0}
        try:
            mujoco.mj_saveModel(scene.world.model, str(directory / 'model.mjb'), None)
            self.log = (scene.out / 'native-observer.log').open('wb')
            python = Path(sys.prefix) / 'bin' / ('mjpython' if sys.platform == 'darwin' else 'python')
            self.process = subprocess.Popen(
                [str(python), '-m', 'scripts.dispatch_native_process', str(directory), str(os.getpid())],
                cwd=Path(__file__).resolve().parents[1], stdout=self.log, stderr=subprocess.STDOUT)
            until = time.monotonic() + 20.
            while True:
                status = self._exchange(True)
                if status is not None and status[READY]:
                    break
                if self.process.poll() is not None:
                    raise RuntimeError('native observer failed; see native-observer.log')
                if time.monotonic() >= until:
                    raise RuntimeError('native observer startup timed out')
                time.sleep(.01)
            self.pace_wall = self.last_tick_wall = time.monotonic()
        except BaseException:
            self.close()
            raise
        print('MuJoCo 독립 관찰 창: Space 일시정지/재개 · Q 또는 창 닫기로 종료.', flush=True)

    def _exchange(self, publish):
        import mujoco
        if publish:
            # Called by the sole physics owner; no actor receives this state.
            mujoco.mj_getState(self.scene.world.model, self.scene.world.data, self.state, self.state_kind)
        def exchange(array):
            status = array[:HEADER].copy()
            if publish:
                array[HEADER:] = self.state
                array[SEQ] += 1
            return status
        status = self.mailbox.transact(exchange)
        if publish:
            self.stats['busy_skips' if status is None else 'published'] += 1
        return status

    def poll(self):
        while True:
            now = time.monotonic()
            if self.scene.deadline and now >= self.scene.deadline:
                raise RuntimeError('skill wall budget exhausted')
            if self.process.poll() is not None:
                raise KeyboardInterrupt('native observer closed')
            publish = now >= self.next_sync
            status = self._exchange(publish)
            if publish:
                self.next_sync = now + 1 / 30
            if status is not None:
                if status[QUIT]:
                    raise KeyboardInterrupt('native operator quit')
                pause_count = int(status[PAUSES])
                if (pause_count - self.pause_seen) % 2:
                    self.paused = not self.paused
                    self.rebase_pace = not self.paused
                self.pause_seen = pause_count
            if not self.paused:
                self.next_poll = now + .01
                return
            time.sleep(.01)

    def close(self):
        if self.closed:
            return
        self.closed = True
        process = self.process
        if process is not None and process.poll() is None:
            self.mailbox.transact(lambda a: a.__setitem__(STOP, 1))
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        self.mailbox.close()
        self.scratch.cleanup()
        if self.log:
            self.log.close()
        (self.scene.out / 'native-observer.json').write_text(json.dumps(self.stats, indent=2) + '\n')


def observe(directory, parent):
    import mujoco
    import mujoco.viewer
    import queue
    model = mujoco.MjModel.from_binary_path(str(directory / 'model.mjb'))
    data = mujoco.MjData(model)
    kind = mujoco.mjtState.mjSTATE_INTEGRATION
    box = StateMailbox(directory / 'state', mujoco.mj_stateSize(model, kind))
    keys = queue.SimpleQueue()
    viewer = None
    try:
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
        pending_pauses = pending_quit = 0
        last_sequence = 0
        while viewer.is_running():
            start = time.monotonic()
            try:
                os.kill(parent, 0)
            except ProcessLookupError:
                break
            while not keys.empty():
                key = keys.get()
                pending_pauses += key == 32
                pending_quit |= key in (81, 256)
            def read(array):
                array[READY] = 1
                array[PAUSES] += pending_pauses
                array[QUIT] = max(array[QUIT], pending_quit)
                array[DISPLAYED] = last_sequence
                sequence = int(array[SEQ])
                state = array[HEADER:].copy() if sequence != last_sequence else None
                return sequence, bool(array[STOP]), state
            update = box.transact(read)
            if update is not None:
                pending_pauses = pending_quit = 0
                sequence, stopping, state = update
                if stopping:
                    break
                if state is not None:
                    with viewer.lock():
                        mujoco.mj_setState(model, data, state, kind)
                    viewer.sync(state_only=True)
                    last_sequence = sequence
            time.sleep(max(0., 1 / 30 - (time.monotonic() - start)))
    finally:
        box.transact(lambda a: a.__setitem__(QUIT, 1))
        if viewer:
            viewer.close()
            until = time.monotonic() + 5
            while viewer._sim() is not None and time.monotonic() < until:
                time.sleep(.01)
        box.close()


if __name__ == '__main__':
    observe(Path(sys.argv[1]), int(sys.argv[2]))
