"""Native observation of the existing skill runner; no controller replacement.

The viewer owns a copy of model/data. GUI perturbations and actuator panels can
never feed back into the research world or the actors' calibrated cameras.
"""
from __future__ import annotations

import copy
import queue
import time
import math

_POLL_INTERVAL_S = .01
_SLEEP_BATCH_S = .005
_PACE_REBASE_GAP_S = .25


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
        self.pace_sim = scene.time()
        self.pace_wall = time.monotonic()
        self.last_tick_wall = self.pace_wall
        self.rebase_pace = False
        self.next_poll = 0.
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
                    if not self.paused:
                        self.rebase_pace = True
                    print('PAUSED' if self.paused else 'RUNNING', flush=True)
                elif key in (81, 256):
                    raise KeyboardInterrupt('native operator quit')
            now = time.monotonic()
            if now >= self.next_sync:
                with self.viewer.lock():
                    mujoco.mj_copyData(self.data, self.model, self.scene.world.data)
                    # These flags belong to the copied observer scene only.
                    self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                    self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                self.viewer.sync(state_only=True)
                self.next_sync = now + 1 / 30
            if not self.paused:
                self.next_poll = time.monotonic() + _POLL_INTERVAL_S
                return
            time.sleep(.02)

    def tick(self):
        now = time.monotonic()
        # Check the budget on every physics step; GUI calls only need wall-time polling.
        if self.scene.deadline and now >= self.scene.deadline:
            raise RuntimeError('skill wall budget exhausted')
        if self.paused or now >= self.next_poll:
            self.poll()
            now = time.monotonic()
        now_sim = self.scene.time()
        target = self.pace_wall + (now_sim - self.pace_sim) / self.factor
        # Rebase after inference, operator pause, or a long render stall. Small
        # sleep overshoots remain in the wall-clock schedule instead of adding
        # one extra sleep to every fine physics step.
        if (self.rebase_pace or now - self.last_tick_wall > _PACE_REBASE_GAP_S
                or now - target > _PACE_REBASE_GAP_S or now_sim < self.pace_sim):
            self.pace_sim, self.pace_wall = now_sim, now
            self.rebase_pace = False
            target = now
        delay = target - now
        if delay >= _SLEEP_BATCH_S:
            time.sleep(min(delay, _POLL_INTERVAL_S))
        self.last_tick_wall = time.monotonic()

    def close(self):
        self.viewer.close()
        # MuJoCo 3.12 close signals its render thread; wait before glfw teardown.
        deadline = time.monotonic() + 10
        while self.viewer._sim() is not None:
            if time.monotonic() >= deadline:
                raise RuntimeError('native dispatch viewer did not close within 10 seconds')
            time.sleep(.01)


class HeadlessPacer(DispatchNativeView):
    """Use the native SIM/wall clock contract without opening an observer UI."""
    def __init__(self,scene,*,realtime_factor=1.):
        if not math.isfinite(realtime_factor) or realtime_factor<=0:
            raise ValueError('positive finite realtime_factor required')
        self.scene=scene;self.factor=float(realtime_factor)
        self.paused=False
        self.pace_sim=scene.time();self.pace_wall=time.monotonic()
        self.last_tick_wall=self.pace_wall;self.rebase_pace=False
        self.next_poll=0.

    def poll(self):
        if self.scene.deadline and time.monotonic()>=self.scene.deadline:
            raise RuntimeError('skill wall budget exhausted')
        self.next_poll=time.monotonic()+_POLL_INTERVAL_S

    def close(self):
        pass
