import pathlib
import unittest
from unittest import mock

import numpy as np

from scripts.run_mujoco_ws_worker import (
    FIRST_PERSON_CAMERA,
    STREAM_INTERVAL_S,
    OBSERVER_INTERVAL_S,
    TEAM_OVERVIEW_CAMERA,
    THIRD_PERSON_CAMERA,
    FIRST_PERSON_JPEG_QUALITY,
    THIRD_PERSON_JPEG_QUALITY,
    cached_robot_jpeg,
    live_camera_msg,
    observer_camera_msg,
    PresentationTokenBucket,
)


class RemoteWorkerFrameReuseTests(unittest.TestCase):
    def test_cached_perception_frame_avoids_second_robot_render(self):
        class World:
            _latest_robot_bgr = np.zeros((32, 48, 3), dtype=np.uint8)
            def render_jpeg(self, *args, **kwargs):
                raise AssertionError("robot camera must not be rendered twice")
            def state(self):
                return {"digital_twin_profile": "nominal_real_contract_v1"}
        data = cached_robot_jpeg(World())
        self.assertTrue(data.startswith(b"\xff\xd8"))

    def test_first_person_message_does_not_wait_for_observer_render(self):
        class World:
            def __init__(self):
                self.cameras = []
            def render_jpeg(self, *, camera=None, quality=76):
                self.cameras.append(camera)
                return b"\xff\xd8camera"
            def state(self):
                return {"digital_twin_profile": "nominal_real_contract_v1"}
        world = World()
        msg = live_camera_msg(world, state_seq=7)
        self.assertEqual(FIRST_PERSON_CAMERA, "robot_cam")
        self.assertEqual(THIRD_PERSON_CAMERA, "cctv_front_left")
        self.assertLessEqual(STREAM_INTERVAL_S, 0.05)
        self.assertGreaterEqual(OBSERVER_INTERVAL_S, STREAM_INTERVAL_S)
        self.assertEqual(world.cameras, ["robot_cam"])
        self.assertEqual(msg["state_seq"], 7)
        self.assertIn("jpeg_b64", msg)
        self.assertNotIn("observer_jpegs_b64", msg)
        self.assertIn("generated_wall_s", msg)
        self.assertIn("render_ms", msg)

    def test_first_person_message_reuses_controller_sensor_frame(self):
        class View:
            _latest_robot_bgr = np.zeros((32, 48, 3), dtype=np.uint8)

        class World:
            def __init__(self):
                self.views = {"r1": View()}
                self.render_calls = 0

            def robot(self, robot_id):
                return self.views[robot_id]

            def render_jpeg(self, *args, **kwargs):
                self.render_calls += 1
                raise AssertionError("controller frame should be reused")

            def robot_state(self, robot_id):
                return {"robot_id": robot_id}

        world = World()
        msg = live_camera_msg(world, "r1")
        self.assertTrue(msg["jpeg_b64"])
        self.assertEqual(world.render_calls, 0)

    def test_encoded_controller_frame_is_reused_until_sensor_sequence_changes(self):
        class View:
            _latest_robot_bgr = np.zeros((32, 48, 3), dtype=np.uint8)
            _latest_robot_frame_seq = 4

        class World:
            def __init__(self):
                self.view = View()
                self.render_calls = 0

            def robot(self, robot_id):
                return self.view

            def render_jpeg(self, *args, **kwargs):
                self.render_calls += 1
                raise AssertionError("controller camera must not be rendered")

            def robot_state(self, robot_id):
                return {"robot_id": robot_id}

        world = World()
        first = live_camera_msg(world, "r1")
        second = live_camera_msg(world, "r1")
        self.assertEqual(first["jpeg_b64"], second["jpeg_b64"])
        self.assertEqual(world.render_calls, 0)

    def test_static_fallback_render_is_reused_until_physics_marks_dirty(self):
        class World:
            def __init__(self):
                self.render_calls = 0
                self._presentation_dirty = True
                self._latest_robot_frame_seq = 0

            def render_jpeg(self, *args, **kwargs):
                self.render_calls += 1
                return b"\xff\xd8camera"

            def state(self):
                return {}

        world = World()
        live_camera_msg(world)
        live_camera_msg(world)
        self.assertEqual(world.render_calls, 1)
        world._presentation_dirty = True
        live_camera_msg(world)
        self.assertEqual(world.render_calls, 2)

    def test_presentation_bucket_drops_until_refilled(self):
        bucket = PresentationTokenBucket(rate_per_s=1.0, capacity=1.0)
        self.assertTrue(bucket.try_consume())
        self.assertFalse(bucket.try_consume())
        self.assertTrue(bucket.try_consume(force=True))

    def test_stream_control_protocol_is_documented_in_worker(self):
        source = (pathlib.Path(__file__).resolve().parents[1] / "scripts/run_mujoco_ws_worker.py").read_text()
        self.assertIn("typ=='stream_control'", source)
        self.assertIn("robot_ids", source)
        self.assertIn("team_overview", source)
        self.assertIn("Forensic traces are independent", source)
        self.assertIn("'type':'partial_result'", source)

    def test_third_person_is_separate_and_uses_higher_jpeg_quality(self):
        self.assertGreater(THIRD_PERSON_JPEG_QUALITY, FIRST_PERSON_JPEG_QUALITY)
        class World:
            def __init__(self): self.calls=[]
            def render_jpeg(self, *, camera=None, quality=0):
                self.calls.append((camera, quality)); return b"\xff\xd8camera"
            def state(self): return {}
        world=World()
        live_camera_msg(world)
        obs=observer_camera_msg(world)
        self.assertEqual(world.calls, [
            (FIRST_PERSON_CAMERA, FIRST_PERSON_JPEG_QUALITY),
            (THIRD_PERSON_CAMERA, THIRD_PERSON_JPEG_QUALITY),
        ])
        self.assertEqual(obs["type"], "observer_frame")
        self.assertEqual(obs["name"], THIRD_PERSON_CAMERA)
        self.assertEqual(obs["robot_id"], "r2")
        obs_r3=observer_camera_msg(world, "r3")
        self.assertEqual(obs_r3["robot_id"], "r3")
        self.assertIn("generated_wall_s", obs)
        self.assertIn("render_ms", obs)

    def test_worker_keeps_camera_stream_alive_while_idle(self):
        source = (pathlib.Path(__file__).resolve().parents[1] / "scripts/run_mujoco_ws_worker.py").read_text()
        self.assertIn("ws.recv(timeout=STREAM_INTERVAL_S)", source)
        self.assertIn("except TimeoutError:", source)
        self.assertIn("emit_live(force=False)", source)
        self.assertNotIn("CCTV =", source)
        self.assertEqual(TEAM_OVERVIEW_CAMERA, "cctv_front_right")
        self.assertIn("team_overview_camera_msg", source)
        self.assertNotIn("cctv_rear_left", source)
        self.assertNotIn("cctv_top", source)


if __name__ == "__main__":
    unittest.main()
