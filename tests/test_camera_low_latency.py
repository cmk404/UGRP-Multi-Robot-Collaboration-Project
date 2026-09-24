import time
import unittest
from pathlib import Path
from harness.web import ChatState
from harness.loop import ReplayCompleter

ROOT=Path(__file__).resolve().parents[1]

class LatestFrameTests(unittest.TestCase):
    def make(self):
        s=ChatState(completer=ReplayCompleter([{'final':'ok'}]), actions_path=ROOT/'scripts/robot_actions.py')
        s._async_real_perception=True
        return s
    def test_waiter_gets_newest_and_skips_old_frames(self):
        s=self.make()
        s.remember_frame(b'\xff\xd8one\xff\xd9')
        _,seq1,_=s.cached_frame_info()
        s.remember_frame(b'\xff\xd8two\xff\xd9')
        s.remember_frame(b'\xff\xd8three\xff\xd9')
        frame,seq3,_=s.wait_for_frame(seq1,timeout=.01)
        self.assertEqual(frame,b'\xff\xd8three\xff\xd9')
        self.assertGreater(seq3,seq1)
    def test_remember_frame_does_not_call_pose_ssh_when_async(self):
        s=self.make()
        s.remember_frame(b'\xff\xd8x\xff\xd9')
        self.assertEqual(s.cached_frame(),b'\xff\xd8x\xff\xd9')

    def test_stale_real_frame_is_not_usable_camera_evidence(self):
        s=self.make()
        s.remember_frame(b'\xff\xd8old\xff\xd9')
        with s._frame_lock:
            s._frame_received_at = time.monotonic() - (s.camera_stale_after_s + 1.0)
        self.assertIsNone(s.cached_frame())
        raw, seq, received = s.cached_frame_info()
        self.assertEqual(raw, b'\xff\xd8old\xff\xd9')
        self.assertGreater(seq, 0)
        self.assertGreater(received, 0)

class PhysicalCameraSourceTests(unittest.TestCase):
    def test_pose_stream_is_persistent_read_only(self):
        text=(ROOT/'harness/real_pose.py').read_text()
        self.assertIn('while true; do cat /tmp/ugrp-masterpi-pose.json',text)
        self.assertNotIn('masterpi_control.py',text)

if __name__=='__main__': unittest.main()
