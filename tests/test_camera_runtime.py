import base64
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from harness.camera_runtime import run_camera_episode


JPEG = b"\xff\xd8runtime-test\xff\xd9"


class SimClock:
    def __init__(self, step=.05):
        self.now = 0.0
        self.increment = step
        self.hooks = []
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.now

    def step(self):
        with self.lock:
            self.now += self.increment
            now = self.now
        for hook in self.hooks:
            hook(now)
        # Give independently submitted actor futures a chance to run while
        # simulated time is owned by this deterministic test clock.
        time.sleep(.002)


class Port:
    def __init__(self, rid, clock, on_apply=None):
        self.rid = rid
        self.clock = clock
        self.on_apply = on_apply
        self.frames = 0
        self.applied = []
        self.stops = 0

    def tick(self, now):
        pass

    def capture(self):
        self.frames += 1
        return {"robot_id": self.rid, "frame_id": f"{self.rid}-f{self.frames}",
                "sim_time": self.clock(), "image": base64.b64encode(JPEG).decode(),
                "sha256": hashlib.sha256(JPEG).hexdigest(), "camera": "robot_cam",
                "actuator_state": {"moving": False}}

    def apply(self, action, now):
        self.applied.append((now, action))
        if self.on_apply:
            self.on_apply(self.rid, action)
        return {"ok": True, "robot_id": self.rid}

    def stop(self):
        self.stops += 1


class Planner:
    def __init__(self, fn=None):
        self.fn = fn
        self.inputs = []

    def decide(self, observation, memory, messages, **options):
        self.inputs.append((observation, memory, messages, options))
        if self.fn:
            return self.fn(observation, memory, messages, options)
        return {"action": {"kind": "wait"}, "private_reason": "own visual reason", "message": None}


def read_events(folder):
    return [json.loads(line) for line in (Path(folder) / "episode.jsonl").read_text().splitlines()]


class CameraRuntimeTests(unittest.TestCase):
    def run_episode(self, clock, ports, planners, output, **options):
        return run_camera_episode(ports, planners, clock=clock, step=clock.step,
                                  output=output, destination="B", realtime=False, **options)

    def test_independent_future_completion_has_no_team_round_barrier(self):
        clock = SimClock()
        r2_applied = threading.Event()
        order = []

        def on_apply(rid, action):
            order.append(rid)
            if rid == "r2":
                r2_applied.set()

        def slow(*_args):
            self.assertTrue(r2_applied.wait(1), "r1 was blocked before r2 could actuate")
            return {"action": {"kind": "wait"}, "private_reason": "slow", "message": None}

        ports = {r: Port(r, clock, on_apply) for r in ("r1", "r2", "r3")}
        planners = {"r1": Planner(slow), "r2": Planner(), "r3": Planner()}
        with tempfile.TemporaryDirectory() as out:
            result = self.run_episode(clock, ports, planners, out, timeout_s=2, max_calls=1)
        self.assertEqual(result["applied_commands"], {"r1": 1, "r2": 1, "r3": 1})
        self.assertLess(order.index("r2"), order.index("r1"))

    def test_peer_never_receives_private_reason_through_memory(self):
        clock = SimClock()
        sent = threading.Event()

        def r1_reply(_obs, _memory, _messages, _options):
            return {"action": {"kind": "wait"}, "private_reason": "R1_PRIVATE_SECRET",
                    "message": {"recipients": ["r2"], "content": "r2에게 공개한 내용",
                                "kind": "intent", "observed_ids": []}}

        def r2_reply(_obs, _memory, _messages, _options):
            if not sent.is_set():
                sent.wait(1)
            return {"action": {"kind": "wait"}, "private_reason": "r2 own", "message": None}

        def on_apply(rid, _action):
            if rid == "r1":
                sent.set()

        ports = {r: Port(r, clock, on_apply) for r in ("r1", "r2", "r3")}
        planners = {"r1": Planner(r1_reply), "r2": Planner(r2_reply), "r3": Planner()}
        with tempfile.TemporaryDirectory() as out:
            self.run_episode(clock, ports, planners, out, timeout_s=3, max_calls=2)
        r2_serialized = json.dumps(planners["r2"].inputs, ensure_ascii=False)
        self.assertNotIn("R1_PRIVATE_SECRET", r2_serialized)
        self.assertIn("r2에게 공개한 내용", r2_serialized)
        self.assertTrue(all("private_reason" not in str(item[2]) for item in planners["r2"].inputs))

    def test_episode_cutoff_discards_pending_call_without_late_actuation(self):
        clock = SimClock(.1)
        release = threading.Event()
        clock.hooks.append(lambda now: release.set() if now >= .5 else None)

        def late(*_args):
            release.wait(1)
            return {"action": {"kind": "drive", "forward": .1, "turn": 0., "duration_s": .2},
                    "private_reason": "late", "message": None}

        ports = {r: Port(r, clock) for r in ("r1", "r2", "r3")}
        planners = {r: Planner(late) for r in ports}
        with tempfile.TemporaryDirectory() as out:
            result = self.run_episode(clock, ports, planners, out, timeout_s=.5, max_calls=1)
            discarded = [e for e in read_events(out) if e["event"] == "pending_call_discarded"]
        self.assertEqual(result["applied_commands"], {"r1": 0, "r2": 0, "r3": 0})
        self.assertEqual(len(discarded), 3)
        self.assertTrue(all(not port.applied for port in ports.values()))

    def test_stale_completed_action_is_rejected_and_robot_stopped(self):
        clock = SimClock(.1)
        release = threading.Event()
        clock.hooks.append(lambda now: release.set() if now >= .4 else None)

        def stale(*_args):
            release.wait(1)
            return {"action": {"kind": "drive", "forward": .1, "turn": 0., "duration_s": .2},
                    "private_reason": "old image", "message": None}

        ports = {r: Port(r, clock) for r in ("r1", "r2", "r3")}
        planners = {r: Planner(stale) for r in ports}
        with tempfile.TemporaryDirectory() as out:
            result = self.run_episode(clock, ports, planners, out, timeout_s=1, max_calls=1,
                                      frame_max_age_s=.1)
        self.assertEqual(result["applied_commands"], {"r1": 0, "r2": 0, "r3": 0})
        self.assertEqual(len(result["errors"]), 3)
        self.assertTrue(all(e["reason"] == "STALE_CAMERA_FRAME_ACTION_REJECTED" for e in result["errors"]))
        self.assertTrue(all(port.stops >= 1 for port in ports.values()))

    def test_none_mode_has_no_bus_delivery_or_dialogue(self):
        clock = SimClock()
        ports = {r: Port(r, clock) for r in ("r1", "r2", "r3")}
        planners = {r: Planner() for r in ports}
        with tempfile.TemporaryDirectory() as out:
            result = self.run_episode(clock, ports, planners, out, timeout_s=1, max_calls=1, mode="none")
            dialogue = (Path(out) / "dialogue.jsonl").read_text()
        self.assertEqual(result["messages"], 0)
        self.assertEqual(dialogue, "")
        self.assertTrue(all(call[2] == [] and call[3]["mode"] == "none"
                            for planner in planners.values() for call in planner.inputs))

    def test_journal_logs_each_actors_own_frame_without_pixels(self):
        clock = SimClock()
        ports = {r: Port(r, clock) for r in ("r1", "r2", "r3")}
        planners = {r: Planner() for r in ports}
        with tempfile.TemporaryDirectory() as out:
            self.run_episode(clock, ports, planners, out, timeout_s=1, max_calls=1)
            events = read_events(out)
            planner_inputs = [e for e in events if e["event"] == "planner_input"]
            commands = [e for e in events if e["event"] == "actuator_command"]
            self.assertEqual({e["robot_id"] for e in planner_inputs}, {"r1", "r2", "r3"})
            for event in planner_inputs:
                rid = event["robot_id"]
                self.assertEqual(event["observation"]["robot_id"], rid)
                self.assertTrue(event["observation"]["frame_id"].startswith(rid + "-"))
                self.assertNotIn("image", event["observation"])
                self.assertEqual(event["image_path"], f"inputs/{rid}-001.jpg")
            self.assertEqual({(e["robot_id"], e["frame_id"]) for e in commands},
                             {(r, f"{r}-f1") for r in ("r1", "r2", "r3")})


if __name__ == "__main__":
    unittest.main()
