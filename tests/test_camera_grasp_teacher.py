from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from scripts.collect_camera_grasp_teacher import DatasetWriter, _move


JPEG = b"\xff\xd8teacher-camera-test\xff\xd9"


class _Controller:
    def __init__(self):
        self.servo_command_pulses = {1: 2000, 3: 1000, 4: 1500, 5: 2000}


class _World:
    def __init__(self):
        self.controllers = {rid: _Controller() for rid in ("r1", "r3")}
        self.moves = []

    def render_team_jpeg(self, **kwargs):
        return JPEG

    def render_jpeg(self, **kwargs):
        return JPEG

    def _team_joint_move_servos(self, poses, duration_s, *, settle_s=0.0):
        self.moves.append((poses, duration_s, settle_s))
        for rid, pose in poses.items():
            self.controllers[rid].servo_command_pulses.update(pose)


class CameraGraspTeacherTest(unittest.TestCase):
    def test_actor_record_excludes_privileged_teacher_and_probe_labels(self):
        with TemporaryDirectory() as temporary:
            writer = DatasetWriter(Path(temporary))
            writer.capture(
                _World(), sample_id="probe_r1_s3_+25_after",
                phase="preclose_probe_after",
                teacher_action={"target_pulses": {"r1": {"3": 1025}}},
                probe_delta={"servo": 3, "delta_pwm": 25},
                physics_evaluation={"contacts": {"r1": {"bilateral": True}}},
            )
            self.assertEqual(len(writer.actor_samples), 2)
            actor = writer.actor_samples[0]
            self.assertNotIn("teacher_action", actor)
            self.assertNotIn("probe_delta_label", actor)
            self.assertNotIn("physics_evaluation", actor)
            self.assertEqual(
                set(actor["observations"]), {"own_rgb", "shared_top_rgb"},
            )
            privileged = writer.privileged_labels[0]
            self.assertEqual(privileged["probe_delta_label"]["delta_pwm"], 25)
            self.assertIn("physics_evaluation", privileged)

    def test_move_records_interpolator_command_chronology(self):
        world = _World()
        issued = {rid: dict(c.servo_command_pulses) for rid, c in world.controllers.items()}
        trace = []
        _move(
            world, issued, trace, {"r1": {3: 1025}}, 0.18,
            settle_s=0.04, phase="preclose_probe",
        )
        self.assertEqual(trace[0]["phase"], "preclose_probe")
        self.assertEqual(trace[0]["targets"], {"r1": {"3": 1025}})
        self.assertEqual(trace[0]["duration_s"], 0.18)
        self.assertEqual(trace[0]["settle_s"], 0.04)
        self.assertEqual(issued["r1"][3], 1025)


if __name__ == "__main__":
    unittest.main()
