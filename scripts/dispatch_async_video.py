"""Bounded observation-only video writer with explicit repeated-frame records."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path

import cv2
import numpy as np

from scripts.probe_dual_grasp_sync import Video, beam_pose
from sim.snapshot_render import SnapshotBackpressure


class AsyncVideo(Video):
    """Never hold up physics for display footage; actor RGB is independent.

When recording cannot keep up, repeat the last image at the requested cadence.
Every missing capture and actual observation timestamp is recorded explicitly;
the output movie therefore keeps SIM duration instead of silently speeding up.
"""
    def __init__(self, world, path, fps):
        super().__init__(world, path, fps)
        self.path = Path(path)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='dispatch-video')
        self.pending = None
        self.rows = []
        self.previous = np.zeros((720, 960, 3), dtype=np.uint8)
        self.written = 0
        self.closed = False

    def _write(self, future, index, label):
        batch = future.result(timeout=30)
        frame = cv2.cvtColor(batch.rgb[(None, 'cctv_warehouse')], cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (960, 720))
        cv2.rectangle(frame, (0, 0), (960, 54), (15, 18, 22), -1)
        cv2.putText(frame, label, (16, 35), cv2.FONT_HERSHEY_SIMPLEX, .67,
                    (238, 238, 238), 2, cv2.LINE_AA)
        while self.written < index:
            repeated = self.previous.copy()
            cv2.putText(repeated, 'REPEATED FRAME: recording worker busy', (16, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (80, 180, 255), 1, cv2.LINE_AA)
            self.process.stdin.write(repeated.tobytes())
            self.written += 1
        self.process.stdin.write(frame.tobytes())
        self.previous = frame
        self.written += 1
        self.rows[index].update(observed_at_s=batch.sim_time, frame_id=batch.frame_id,
                                status='captured')

    def capture(self, force=False):
        now = float(self.world.data.time)
        if not force and now + 1e-9 < self.next_sim_time:
            return
        index = len(self.rows)
        self.rows.append({'index': index, 'requested_at_s': now, 'status': 'repeated'})
        self.frames += 1
        self.next_sim_time = now + 1 / self.fps
        if self.pending is not None:
            if not self.pending.done():
                return
            self.pending.result()  # Surface recording errors; never claim a complete video.
        try:
            future = self.world.render_snapshot_async([(None, 'cctv_warehouse')])
        except SnapshotBackpressure:
            return
        pose = beam_pose(self.world.data, self.world.model)
        label = f"{self.stage} | t={now:.2f}s | beam z={float(pose['position'][2]):.3f}m"
        self.rows[index]['status'] = 'pending'
        self.pending = self.pool.submit(self._write, future, index, label)

    def close(self):
        if self.closed:
            return
        self.closed = True
        error = None
        try:
            if self.pending is not None:
                self.pending.result(timeout=35)
            while self.written < len(self.rows):
                repeated = self.previous.copy()
                cv2.putText(repeated, 'REPEATED FRAME: recording worker busy', (16, 82),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (80, 180, 255), 1, cv2.LINE_AA)
                self.process.stdin.write(repeated.tobytes())
                self.written += 1
        except BaseException as exc:
            error = exc
        finally:
            self.pool.shutdown(wait=True, cancel_futures=True)
            self.path.with_suffix('.frames.json').write_text(json.dumps({
                'scope': 'observer recording only; not actor camera input',
                'requested_fps': self.fps, 'written_frames': self.written,
                'captured_frames': sum(r['status'] == 'captured' for r in self.rows),
                'repeated_frames': sum(r['status'] == 'repeated' for r in self.rows),
                'error': str(error) if error else None, 'frames': self.rows,
            }, ensure_ascii=False, indent=2) + '\n')
            super().close()
        if error:
            raise error
