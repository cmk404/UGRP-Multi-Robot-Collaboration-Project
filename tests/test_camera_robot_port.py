import base64
import hashlib
import math
import unittest

from sim.camera_robot_port import CameraRobotPort


class _RobotSpy:
    def __init__(self):
        self.motor_calls = []
        self.servo_calls = []
        self.servo_command_pulses = {1: 2000, 3: 600, 4: 2200, 5: 1900, 6: 1500}

    def set_motor_commands(self, values):
        self.motor_calls.append(list(values))

    def set_servo_pulses(self, values):
        self.servo_calls.append(dict(values))

    def __getattr__(self, name):
        raise AssertionError(f"privileged robot API read: {name}")


class _DataClock:
    def __init__(self):
        self.time = 0.0

    def __getattr__(self, name):
        raise AssertionError(f"privileged physics state read: {name}")


class _WorldSpy:
    def __init__(self):
        self.data = _DataClock()
        self.robots = {"r1": _RobotSpy(), "r2": _RobotSpy()}
        self.jpeg = bytearray(b"jpeg-one")
        self.render_calls = []
        self.hidden_state = {"cargo": "secret", "poses": [1, 2, 3]}

    def robot(self, rid):
        return self.robots[rid]

    def render_jpeg(self, **kwargs):
        self.render_calls.append(kwargs)
        return self.jpeg

    def __getattr__(self, name):
        raise AssertionError(f"privileged world API read: {name}")


class CameraRobotPortTests(unittest.TestCase):
    def setUp(self):
        self.world = _WorldSpy()
        self.port = CameraRobotPort(self.world, "r1")

    def test_capture_is_fresh_owned_rgb_and_reports_only_commands(self):
        self.world.data.time = 1.25
        first = self.port.capture()
        first_bytes = base64.b64decode(first["image"])
        self.world.jpeg[:] = b"jpeg-two"
        self.world.hidden_state = {"cargo": "changed", "poses": [999]}
        second = self.port.capture()

        self.assertEqual(first_bytes, b"jpeg-one")
        self.assertEqual(first["sha256"], hashlib.sha256(b"jpeg-one").hexdigest())
        self.assertEqual(base64.b64decode(second["image"]), b"jpeg-two")
        self.assertEqual(second["sha256"], hashlib.sha256(b"jpeg-two").hexdigest())
        self.assertEqual((first["frame_id"], second["frame_id"]), (1, 2))
        self.assertEqual(first["sim_time"], 1.25)
        self.assertEqual(first["camera"], "robot_cam")
        self.assertEqual(first["robot_id"], "r1")
        self.assertEqual(
            first["actuator_state"],
            {"motor_commands": [0.0] * 4, "servo_pulses": {"1": 2000, "3": 600, "4": 2200, "5": 1900, "6": 1500}},
        )
        self.assertEqual(
            self.world.render_calls,
            [
                {"robot_id": "r1", "camera": "robot_cam"},
                {"robot_id": "r1", "camera": "robot_cam"},
            ],
        )
        self.assertNotIn("hidden_state", repr(first))

    def test_drive_uses_forward_and_yaw_patterns_and_expires(self):
        feedback = self.port.apply(
            {"kind": "drive", "forward": 0.1, "turn": 0.02, "duration_s": 0.5},
            3.0,
        )
        for actual, expected in zip(self.world.robots["r1"].motor_calls[-1], [0.08, 0.12, 0.08, 0.12]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(feedback["actuator_state"]["motor_commands"], [0.08, 0.12, 0.08, 0.12]):
            self.assertAlmostEqual(actual, expected)
        self.port.tick(3.499)
        self.assertEqual(len(self.world.robots["r1"].motor_calls), 1)
        self.port.tick(3.5)
        self.assertEqual(self.world.robots["r1"].motor_calls[-1], [0.0] * 4)
        self.assertEqual(self.world.robots["r2"].motor_calls, [])

    def test_look_and_arm_use_physical_servo_channels(self):
        look = self.port.apply({"kind": "look", "pan_pulse": 1700}, 0.0)
        self.assertEqual(self.world.robots["r1"].servo_calls, [])
        self.port.tick(0.05)
        self.assertEqual(self.world.robots["r1"].servo_calls, [{6: 1600}])
        self.port.tick(0.1)
        arm = self.port.apply({"kind": "arm", "servo_id": 5, "pulse": 1700}, 0.1)
        self.port.tick(0.15)
        self.assertEqual(self.world.robots["r1"].servo_calls, [{6: 1600}, {6: 1700}, {5: 1800}])
        self.assertEqual(look["busy_until"], 0.1)
        self.assertAlmostEqual(arm["busy_until"], 0.2)
        self.assertEqual(self.port.busy_until, arm["busy_until"])
        state = self.port.capture()["actuator_state"]
        self.assertEqual(state["servo_pulses"], {"1": 2000, "3": 600, "4": 2200, "5": 1700, "6": 1700})

    def test_wait_and_stop_stop_only_own_wheels(self):
        self.port.apply({"kind": "wait"}, 1.0)
        self.port.stop()
        self.assertEqual(self.world.robots["r1"].motor_calls, [[0.0] * 4, [0.0] * 4])
        self.assertEqual(self.world.robots["r2"].motor_calls, [])

    def test_rejects_unknown_missing_nonfinite_and_out_of_range_fields(self):
        bad = [
            ({"kind": "drive", "forward": 0.1, "turn": 0.0, "duration_s": 0.2, "cargo": "red"}, 0.0),
            ({"kind": "drive", "forward": 0.1, "turn": 0.0}, 0.0),
            ({"kind": "drive", "forward": math.nan, "turn": 0.0, "duration_s": 0.2}, 0.0),
            ({"kind": "drive", "forward": 0.16, "turn": 0.0, "duration_s": 0.2}, 0.0),
            ({"kind": "drive", "forward": 0.1, "turn": 0.21, "duration_s": 0.2}, 0.0),
            ({"kind": "drive", "forward": 0.1, "turn": 0.0, "duration_s": 1.01}, 0.0),
            ({"kind": "look", "pan_pulse": 499}, 0.0),
            ({"kind": "look", "pan_pulse": 1500, "tilt_pulse": 1500}, 0.0),
            ({"kind": "arm", "servo_id": 2, "pulse": 1500}, 0.0),
            ({"kind": "arm", "servo_id": 6, "pulse": 1500}, 0.0),
            ({"kind": "arm", "servo_id": True, "pulse": 1500}, 0.0),
            ({"kind": "wait", "seconds": 1}, 0.0),
            ({"kind": "wait"}, math.inf),
            ({"kind": "teleport"}, 0.0),
        ]
        for action, now in bad:
            with self.subTest(action=action, now=now), self.assertRaises(ValueError):
                self.port.apply(action, now)
        self.assertEqual(self.world.robots["r1"].motor_calls, [])
        self.assertEqual(self.world.robots["r1"].servo_calls, [])


if __name__ == "__main__":
    unittest.main()
