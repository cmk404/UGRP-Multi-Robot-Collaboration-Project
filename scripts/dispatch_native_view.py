"""Native observation of the existing skill runner; no controller replacement.

The viewer owns a copy of model/data. GUI perturbations and actuator panels can
never feed back into the research world or the actors' calibrated cameras.
"""
from __future__ import annotations

import copy
import queue
import time


class DispatchNativeView:
    def __init__(self, scene, *, realtime_factor=1.):
        import mujoco
        import mujoco.viewer
        self.scene = scene
        self.factor = realtime_factor
        self.keys = queue.SimpleQueue()
        self.paused = False
        self.model = copy.copy(scene.world.model)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_copyData(self.data, self.model, scene.world.data)
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self.keys.put,
            show_left_ui=False, show_right_ui=False)
        with self.viewer.lock():
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.viewer.cam.lookat[:] = [.55, -2., .1]
            self.viewer.cam.distance = 4.8
            self.viewer.cam.azimuth = 90
            self.viewer.cam.elevation = -55
        self.last_sim = scene.time()
        self.last_wall = time.monotonic()
        self.next_sync = 0.
        self.poll()
        print('MuJoCo 관찰 창: Space 일시정지/재개 · Q 또는 창 닫기로 종료. 계획 대기 중 물리는 정지합니다.', flush=True)

    def poll(self):
        import mujoco
        while True:
            if not self.viewer.is_running():
                raise KeyboardInterrupt('native viewer closed')
            if self.scene.deadline and time.monotonic() >= self.scene.deadline:
                raise RuntimeError('skill wall budget exhausted')
            while not self.keys.empty():
                key = self.keys.get()
                if key == 32:
                    self.paused = not self.paused
                    print('PAUSED' if self.paused else 'RUNNING', flush=True)
                elif key in (81, 256):
                    raise KeyboardInterrupt('native operator quit')
            now = time.monotonic()
            if now >= self.next_sync:
                with self.viewer.lock():
                    mujoco.mj_copyData(self.data, self.model, self.scene.world.data)
                self.viewer.sync()
                self.next_sync = now + 1 / 30
            if not self.paused:
                return
            time.sleep(.02)

    def tick(self):
        self.poll()
        now_sim = self.scene.time()
        # An inference pause never incurs catch-up physics on resumption.
        delay = (now_sim - self.last_sim) / self.factor - (time.monotonic() - self.last_wall)
        if delay > 0:
            time.sleep(delay)
        self.last_sim, self.last_wall = now_sim, time.monotonic()

    def close(self):
        self.viewer.close()
        # MuJoCo 3.12 close signals its render thread; wait before glfw teardown.
        deadline = time.monotonic() + 10
        while self.viewer._sim() is not None:
            if time.monotonic() >= deadline:
                raise RuntimeError('native dispatch viewer did not close within 10 seconds')
            time.sleep(.01)
