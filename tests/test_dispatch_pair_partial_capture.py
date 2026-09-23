"""TOP JPEG becomes usable before same-snapshot own camera completion."""
import threading
import time

import pytest

pytest.importorskip('mujoco')

from scripts.research_dispatch_scene import DispatchScene
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.snapshot_render import TopRenderLatch


def test_partial_pair_reuses_exact_old_batch_pixels_and_top_work_overlaps_own(tmp_path, monkeypatch):
    world = MultiMasterPiProductionV2(
        seed=11, width=80, height=60, render=True, use_calibration_manifest=False)
    scene = DispatchScene.__new__(DispatchScene)
    scene.world, scene.out = world, tmp_path
    scene.ports, scene.sequence = {}, 0
    scene._capture_workers = scene._top_capture_worker = None
    scene.realtime_control = True
    try:
        baseline = scene.capture_async('old', own_robots=('r1', 'r3')).result(timeout=5)
        original_publish = TopRenderLatch.publish
        signaled, resume = threading.Event(), threading.Event()
        def pause_after_top(self, value):
            original_publish(self, value)
            signaled.set()
            assert resume.wait(5)
        monkeypatch.setattr(TopRenderLatch, 'publish', pause_after_top)
        ticket = scene.capture_pair_partial_async('partial', own_robots=('r1', 'r3'))
        try:
            assert signaled.wait(2)
            deadline = time.monotonic() + 2
            while not ticket.top.done() and time.monotonic() < deadline:
                time.sleep(.001)
            top = ticket.top.result(timeout=1)
            assert not ticket.full.done()
            assert top['top_materialized_wall_s'] >= top['top_render_completed_wall_s']
            assert world._snapshot_broker.slots.qsize() == 2
        finally:
            resume.set()
        frames = ticket.full.result(timeout=5)
        assert frames['r1']['frame_id'] == frames['r3']['frame_id'] == top['frame_id']
        assert frames['r1']['observed_at_s'] == frames['r3']['observed_at_s'] == top['observed_at_s']
        assert frames['r1']['observed_at_s'] == baseline['r1']['observed_at_s']
        assert top['top_materialized_wall_s'] < frames['r1']['render_completed_wall_s']
        assert frames['r1']['top_bytes'] == frames['r3']['top_bytes'] == top['top_bytes']
        assert frames['r1']['shared_top_rgb'] == frames['r3']['shared_top_rgb'] == top['shared_top_rgb']
        assert frames['r1']['shared_top_rgb']['sha256'] == baseline['r1']['shared_top_rgb']['sha256']
        assert all(frames[r]['own_rgb']['sha256'] == baseline[r]['own_rgb']['sha256']
                   for r in ('r1', 'r3'))
        assert world._snapshot_broker.slots.qsize() == 3
    finally:
        scene.close()
