#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load(name: str):
    path = PACKAGE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    sys.path.insert(0, str(PACKAGE))
    spec.loader.exec_module(module)
    return module


POSES = load("poses")
CAMERA = load("camera")
PICK = load("pick")


class PickRedBlockPlanningTests(unittest.TestCase):
    def test_center_image_keeps_base_centered(self):
        self.assertEqual(POSES.base_pulse_for_image_x(0.5), 1500)

    def test_right_blob_yaw_decreases_pulse(self):
        pulse = POSES.base_pulse_for_image_x(0.739)
        self.assertLess(pulse, 1500)
        self.assertEqual(pulse, 1357)

    def test_left_blob_yaw_increases_pulse(self):
        self.assertGreater(POSES.base_pulse_for_image_x(0.25), 1500)

    def test_flip_x_inverts_yaw(self):
        right = POSES.base_pulse_for_image_x(0.75)
        flipped = POSES.base_pulse_for_image_x(0.75, flip_x=True)
        self.assertLess(right, 1500)
        self.assertGreater(flipped, 1500)

    def test_base_pulse_rejects_out_of_range_nx(self):
        with self.assertRaises(ValueError):
            POSES.base_pulse_for_image_x(1.2)

    def test_pose_includes_open_gripper_and_aligned_base(self):
        pose = POSES.pose_with_base(POSES.POSE_GRASP, 1357, POSES.GRIPPER_OPEN)
        self.assertEqual(pose[1], 2000)
        self.assertEqual(pose[6], 1357)
        self.assertEqual(pose[5], 1920)

    def test_lowering_moves_servo_five_last(self):
        steps = POSES.servo_steps(
            {1: 2000, 3: 695, 4: 2410, 5: 780, 6: 1500},
            POSES.pose_with_base(POSES.POSE_GRASP, 1500, POSES.GRIPPER_OPEN),
            lowering=True,
        )
        self.assertEqual(steps[-1][0], 5)

    def test_raising_moves_servo_five_first(self):
        grasp = POSES.pose_with_base(POSES.POSE_GRASP, 1500, POSES.GRIPPER_CLOSED)
        lift = POSES.pose_with_base(POSES.POSE_OBSERVE, 1500, POSES.GRIPPER_CLOSED)
        steps = POSES.servo_steps(grasp, lift, lowering=False)
        self.assertEqual(steps[0][0], 5)

    def test_parser_accepts_detect_only(self):
        args = PICK.build_parser().parse_args(["--detect-only", "--dry-run"])
        self.assertTrue(args.detect_only)
        self.assertTrue(args.dry_run)


    def test_agent_pick_consumes_coarse_handoff_then_verifies_grasp(self):
        calls = []

        class FakeVideo:
            def __init__(self, url): calls.append(("video", url))
            def settle(self, delay): calls.append(("settle", delay))
            def close(self): calls.append(("close",))

        class FakeRobot:
            dry_run = True
            def __init__(self): self.pose = {3:500,4:2320,5:1320,6:1500}
            def stop(self): calls.append(("stop",))
            def move_pose(self, pose, lowering): calls.append(("carry", pose, lowering))
            def move_servo(self, *args): calls.append(("servo", *args))

        fake = type("Precision", (), {})()
        fake.STREAM_URL = "stream"
        fake.CAMERA_DELAY_SECONDS = .12
        fake.DELIVERY_CARRY_POSE = {1:1500,3:600,4:2200,5:1500}
        fake.GRIPPER_ID = 1
        fake.GRIPPER_OPEN = 2000
        fake.LiveVideo = FakeVideo
        fake.execute_pick_to_hover = lambda robot, video, plan: (calls.append(("execute", plan)) or "reference")
        fake.verify_grasp = lambda robot, video, reference: (calls.append(("verify", reference)) or (True, None))

        robot = FakeRobot()
        plan = object()
        coarse = {"target_color":"red", "gaze":{"pan":1500,"tilt":500}}
        with mock.patch.object(PICK, "load_coarse_handoff_payload", return_value=coarse) as load_handoff,              mock.patch.object(PICK, "require_coarse_handoff_color") as require_color,              mock.patch.object(PICK, "validate_coarse_handoff_pose") as validate_pose,              mock.patch.object(PICK, "_prepare_nearfield_pick_plan", return_value=plan) as prepare,              mock.patch.object(PICK, "invalidate_pick_plan") as invalidate:
            self.assertEqual(PICK.run_precision_pick(robot, precision=fake), 0)
        load_handoff.assert_called_once()
        require_color.assert_called_once_with(coarse, "red")
        validate_pose.assert_called_once_with(robot.pose, coarse)
        prepare.assert_called_once()
        self.assertIn(("execute", plan), calls)
        self.assertIn(("verify", "reference"), calls)
        self.assertIn(("carry", fake.DELIVERY_CARRY_POSE, False), calls)
        invalidate.assert_called()

    def test_pick_face_restage_happens_before_precapture_and_close_never_doglegs(self):
        calls = []
        gaze = PICK.Gaze(pan=1500, tilt=500)
        lock = types.SimpleNamespace()
        capture_blob = types.SimpleNamespace(nx=.5, ny=.3)
        plan = object()

        class FakeRobot:
            dry_run = True
            def __init__(self):
                self.pose = {3:500,4:2320,5:1320,6:1500}
            def nudge_servos(self, pose, duration):
                calls.append(("precapture_pose", dict(pose), duration))
                self.pose.update(pose)
            def stop(self):
                calls.append(("stop",))

        class FakeVideo:
            pass

        fake = types.SimpleNamespace(
            BASE_CENTER=1500,
            POSE_SEARCH={3:500},
            CENTER_CONFIRMATIONS=2,
            PRECAPTURE_ARM_POSE={3:610,4:2200,5:1800},
            CAPTURE_TARGET_NX=.5,
            ARM_FACE_CAPTURE_TARGET_NY=.30,
            CAPTURE_TARGET_NY=.44,
            FACE_ALIGNMENT_ABORT_DEG=12.0,
        )
        fake.centre_gaze = mock.Mock(side_effect=lambda *a, **k: (calls.append(("centre",)) or gaze))
        fake.visual_precapture_approach = mock.Mock(
            side_effect=lambda *a, **k: (calls.append(("visual_precapture",)) or (gaze, lock, capture_blob))
        )
        fake.establish_capture_pose = mock.Mock(
            side_effect=lambda *a, **k: (calls.append(("capture_pose",)) or (gaze, lock))
        )
        fake.visual_capture_approach = mock.Mock(
            side_effect=lambda *a, **k: (calls.append(("visual_capture", k.get("target_ny"))) or (gaze, capture_blob))
        )
        fake.align_block_face_with_arm_yaw = mock.Mock(
            side_effect=AssertionError("close-view chassis dog-leg must never run")
        )

        robot = FakeRobot()
        coarse = {"gaze":{"pan":1500,"tilt":500}}
        with mock.patch.object(PICK, "acquire_inherited_target", return_value=lock), \
             mock.patch.object(PICK, "_bounded_face_direction_restage",
                               side_effect=lambda *a, **k: (calls.append(("face_restage",)) or gaze)), \
             mock.patch.object(PICK, "_orient_chassis_for_straight_close",
                               side_effect=lambda *a, **k: (calls.append(("orient",)) or gaze)), \
             mock.patch.object(PICK, "_measure_arm_only_pick_plan", return_value=plan):
            result = PICK._prepare_nearfield_pick_plan(
                robot, FakeVideo(), fake, coarse, flip_x=False, flip_y=False
            )

        self.assertIs(result, plan)
        names = [item[0] for item in calls]
        self.assertLess(names.index("face_restage"), names.index("orient"))
        self.assertLess(names.index("orient"), names.index("precapture_pose"))
        self.assertLess(names.index("precapture_pose"), names.index("visual_precapture"))
        fake.align_block_face_with_arm_yaw.assert_not_called()

    def test_arm_only_plan_uses_fresh_stopped_multi_frame_measurement(self):
        plan = object()
        blobs = [
            types.SimpleNamespace(nx=.498, ny=.441, area=9000),
            types.SimpleNamespace(nx=.501, ny=.443, area=9100),
            types.SimpleNamespace(nx=.500, ny=.442, area=9050),
            types.SimpleNamespace(nx=.499, ny=.442, area=9020),
            types.SimpleNamespace(nx=.502, ny=.444, area=9080),
        ]
        lock = types.SimpleNamespace(last=blobs[0])

        class FakeVideo:
            def settle(self, delay): pass

        class FakeRobot:
            dry_run = True
            def __init__(self):
                self.pose = {3:600,4:2200,5:1900,6:1500}
                self.stop = mock.Mock()

        fake = types.SimpleNamespace(
            CAPTURE_ARM_POSE={3:600,4:2200,5:1900},
            BASE_CENTER=1500,
            CAMERA_DELAY_SECONDS=.1,
            SERVO_SETTLE_SECONDS=.1,
            FINAL_SAMPLE_COUNT=5,
            FINAL_MIN_VALID_SAMPLES=4,
            CAPTURE_TARGET_NX=.50,
            CAPTURE_TARGET_NY=.442,
            CAPTURE_TARGET_TOLERANCE_NY=.025,
            CAPTURE_TOO_CLOSE_NY=.505,
            CAPTURE_BLOCK_RADIUS_CM=17.0,
            CAMERA_HFOV_DEG=62.0,
            PULSE_PER_DEGREE=10.0,
            CAMERA_LINK_CM=7.0,
            CAMERA_Z_OFFSET_CM=0.0,
            DEFAULT_BLOCK_HEIGHT_CM=3.0,
        )
        fake.fine_align_horizontal = lambda robot, video, current_lock, gaze, **kwargs: gaze
        fake.read_locked = mock.Mock(side_effect=[(object(), item) for item in blobs])
        fake.median_absolute_deviation = lambda values: max(values) - min(values)
        fake.forward_kinematics = lambda pose, link: types.SimpleNamespace(
            radius_cm=8.0, height_cm=10.0
        )
        fake.BlockEstimate = lambda **kwargs: types.SimpleNamespace(**kwargs)
        fake.calculate_pick_plan = mock.Mock(return_value=plan)

        robot = FakeRobot()
        with mock.patch.object(PICK, "acquire_inherited_target", return_value=lock), \
             mock.patch.object(PICK.time, "sleep", return_value=None):
            result = PICK._measure_arm_only_pick_plan(
                robot, FakeVideo(), fake, flip_x=False, flip_y=False
            )
        self.assertIs(result, plan)
        self.assertEqual(fake.read_locked.call_count, 5)
        fake.calculate_pick_plan.assert_called_once()
        measured_block = fake.calculate_pick_plan.call_args.args[2]
        self.assertAlmostEqual(measured_block.nx, .500, places=3)
        self.assertAlmostEqual(measured_block.ny, .442, places=3)
        robot.stop.assert_called_once()

    def test_stream_snapshot_is_tried_before_raw_v4l2(self):
        self.assertEqual(CAMERA.STREAM_SNAPSHOT_URLS[0], "http://127.0.0.1:8080/snapshot")

    def test_observe_pose_matches_last_agreed_values(self):
        self.assertEqual(POSES.POSE_OBSERVE, {3: 960, 4: 2410, 5: 1215})


if __name__ == "__main__":
    unittest.main(verbosity=2)
