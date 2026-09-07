from __future__ import annotations

import pathlib
import sys
import types
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
RED_BLOCK = ROOT / "scripts" / "red_block"
if str(RED_BLOCK) not in sys.path:
    sys.path.insert(0, str(RED_BLOCK))

import task_runner


class FakeVideo:
    def __init__(self):
        self.closed = False
        self.frames = 0

    def read(self):
        self.frames += 1
        return object()

    def close(self):
        self.closed = True


class FakeRobot:
    dry_run = True

    def __init__(self):
        self.stop_count = 0

    def stop(self):
        self.stop_count += 1


class SingleTaskRunnerTests(unittest.TestCase):
    def test_pipeline_reuses_one_robot_and_one_video_for_all_stages(self):
        robot = FakeRobot()
        video = FakeVideo()
        precision = types.SimpleNamespace()
        seen = []

        def search(r, **kwargs):
            self.assertIs(r, robot)
            self.assertIs(kwargs["camera"].video, video)
            self.assertEqual(kwargs["target_color"], "blue")
            seen.append("search")
            return 0

        def track(r, **kwargs):
            self.assertIs(r, robot)
            self.assertIs(kwargs["capture"].__self__.video, video)
            self.assertEqual(kwargs["target_color"], "blue")
            seen.append("track")
            return 0

        def approach(r, **kwargs):
            self.assertIs(r, robot)
            self.assertIs(kwargs["video"], video)
            self.assertIs(kwargs["precision"], precision)
            seen.append("approach")
            return 0

        def pick(r, **kwargs):
            self.assertIs(r, robot)
            self.assertIs(kwargs["video"], video)
            self.assertIs(kwargs["precision"], precision)
            seen.append("pick")
            return 0

        with mock.patch.object(task_runner.search_mod, "run_search", side_effect=search), \
             mock.patch.object(task_runner.track_mod, "run_track", side_effect=track), \
             mock.patch.object(task_runner.approach_mod, "run_approach", side_effect=approach), \
             mock.patch.object(task_runner.pick_mod, "run_precision_pick", side_effect=pick):
            code = task_runner.run_grasp_task(
                robot,
                target_color="blue",
                precision=precision,
                video=video,
                clock=types.SimpleNamespace(),
            )

        self.assertEqual(code, 0)
        self.assertEqual(seen, ["search", "track", "approach", "pick"])
        self.assertFalse(video.closed, "caller-owned stream must survive the composite task")
        self.assertGreaterEqual(robot.stop_count, 1)

    def test_stage_failure_stops_pipeline_and_names_failed_stage(self):
        robot = FakeRobot()
        video = FakeVideo()
        precision = types.SimpleNamespace()
        seen = []

        def search(*args, **kwargs):
            seen.append("search")
            return 0

        def track(*args, **kwargs):
            seen.append("track")
            raise RuntimeError("target disappeared")

        with mock.patch.object(task_runner.search_mod, "run_search", side_effect=search), \
             mock.patch.object(task_runner.track_mod, "run_track", side_effect=track), \
             mock.patch.object(task_runner.approach_mod, "run_approach") as approach, \
             mock.patch.object(task_runner.pick_mod, "run_precision_pick") as pick:
            with self.assertRaisesRegex(RuntimeError, "task stage=track: target disappeared"):
                task_runner.run_grasp_task(
                    robot, target_color="red", precision=precision, video=video
                )

        self.assertEqual(seen, ["search", "track"])
        approach.assert_not_called()
        pick.assert_not_called()
        self.assertGreaterEqual(robot.stop_count, 1)

    def test_owned_video_is_opened_once_and_closed_once(self):
        robot = FakeRobot()
        video = FakeVideo()
        precision = types.SimpleNamespace(
            STREAM_URL="stream",
            LiveVideo=mock.Mock(return_value=video),
        )
        with mock.patch.object(task_runner.search_mod, "run_search", return_value=0), \
             mock.patch.object(task_runner.track_mod, "run_track", return_value=0), \
             mock.patch.object(task_runner.approach_mod, "run_approach", return_value=0), \
             mock.patch.object(task_runner.pick_mod, "run_precision_pick", return_value=0):
            self.assertEqual(
                task_runner.run_grasp_task(robot, precision=precision, target_color="yellow"),
                0,
            )
        precision.LiveVideo.assert_called_once_with("stream")
        self.assertTrue(video.closed)

    def test_cli_deploys_whole_task_once(self):
        with mock.patch.object(task_runner.search_mod, "is_robot", return_value=False), \
             mock.patch.object(task_runner, "deploy_and_run", return_value=0) as deploy:
            self.assertEqual(task_runner.main(["--target-color", "blue"]), 0)
        deploy.assert_called_once()
        self.assertEqual(deploy.call_args.args[0], "task_runner.py")
        self.assertIn("--target-color", deploy.call_args.args[1])
        self.assertEqual(deploy.call_args.kwargs["host"], task_runner.DEFAULT_HOST)

    def test_search_and_precision_skills_do_not_close_borrowed_camera_resources(self):
        # The composite runner depends on these ownership rules; guard them
        # independently so a later standalone-skill cleanup refactor cannot
        # accidentally tear down the shared stream between stages.
        self.assertIn("owns_camera", pathlib.Path(task_runner.search_mod.__file__).read_text())
        self.assertIn("owns_video", pathlib.Path(task_runner.approach_mod.__file__).read_text())
        self.assertIn("owns_video", pathlib.Path(task_runner.pick_mod.__file__).read_text())


if __name__ == "__main__":
    unittest.main()
