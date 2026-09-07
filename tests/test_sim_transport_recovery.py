from __future__ import annotations

import io
import json
import unittest
from unittest.mock import patch
from urllib.error import HTTPError

import scripts.sim_actions as sim_actions
from harness.catalog import default_registry
from harness.executive import TaskExecutive
from harness.loop import ReplayCompleter, run_loop
from harness.state import StateEstimator
from harness.registry import Registry, Tool


class _Response:
    def __init__(self, obj):
        self.body = json.dumps(obj).encode()
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def read(self): return self.body


def gpu_offline_error():
    return HTTPError(
        sim_actions.BRIDGE + "/command", 503, "Service Unavailable", {},
        io.BytesIO(json.dumps({"ok": False, "reason": "GPU_OFFLINE"}).encode()),
    )


class SimActionRecoveryTests(unittest.TestCase):
    def test_same_worker_reconnect_retries_rejected_command_once(self):
        health_before = {
            "remote_ws_connected": False, "remote_authoritative": False,
            "remote_episode_id": 7, "remote_last_instance_id": "proc-a",
        }
        health_after = {
            "remote_ws_connected": True, "remote_authoritative": True,
            "remote_episode_id": 7, "remote_instance_id": "proc-a",
        }
        with patch.object(sim_actions, "_bridge_health", side_effect=[health_before]), \
             patch.object(sim_actions, "_wait_for_gpu_authority", return_value=health_after), \
             patch.object(sim_actions, "_post_bridge_action", side_effect=[gpu_offline_error(), {"ok": True, "reason": "mapped"}]) as post:
            result = sim_actions.run("observe_scene")
        self.assertTrue(result["ok"])
        self.assertEqual(post.call_count, 2)

    def test_replacement_worker_returns_episode_reset_without_replaying_middle_action(self):
        health_before = {
            "remote_ws_connected": False, "remote_authoritative": False,
            "remote_episode_id": 7, "remote_last_instance_id": "proc-a",
        }
        health_after = {
            "remote_ws_connected": True, "remote_authoritative": True,
            "remote_episode_id": 8, "remote_instance_id": "proc-b",
        }
        with patch.object(sim_actions, "_bridge_health", return_value=health_before), \
             patch.object(sim_actions, "_wait_for_gpu_authority", return_value=health_after), \
             patch.object(sim_actions, "_post_bridge_action", side_effect=[gpu_offline_error()]) as post:
            result = sim_actions.run("pick", target_color="red")
        self.assertFalse(result["ok"])
        self.assertEqual(result["failure_code"], "SIM_EPISODE_RESET")
        self.assertEqual(post.call_count, 1)


class StructuredEpisodeRestartTests(unittest.TestCase):
    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_place_goal_restarts_from_shared_search_after_episode_reset(self):
        calls = []
        source_registry = default_registry(actions_path="scripts/sim_actions.py")
        registry = Registry()
        # Build a separate registry because Tool definitions are intentionally immutable.
        for name in source_registry.names():
            original = source_registry.get(name)
            def make_handler(skill):
                def handler(**kwargs):
                    calls.append((skill, dict(kwargs)))
                    if skill == "search" and sum(1 for call, _ in calls if call == "search") == 1:
                        return {
                            "ok": False, "skill": skill,
                            "command_status": "UNKNOWN", "execution_status": "FAILED",
                            "outcome_status": "NOT_ACHIEVED", "failure_code": "SIM_EPISODE_RESET",
                            "reason": "SIM_EPISODE_RESET",
                        }
                    result = {
                        "ok": True, "skill": skill,
                        "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                        "outcome_status": "ACHIEVED",
                    }
                    if "target_color" in kwargs:
                        result["target_color"] = kwargs["target_color"]
                    if "destination_color" in kwargs:
                        result["destination_color"] = kwargs["destination_color"]
                    if skill == "search":
                        result["target_vision"] = {"visible": True, "cx": .5, "cy": .65, "area_ratio": .1}
                    elif skill == "track":
                        result.update({"target_vision": {"visible": True, "cx": .5}, "center_verified": True})
                    elif skill == "approach":
                        result.update({
                            "target_vision": {"visible": True, "cx": .5},
                            "center_verified": True, "range_verified": True,
                        })
                    elif skill == "pick":
                        result.update({
                            "outcome_status": "UNKNOWN",
                            "visual_hold": {"probable": True, "confidence": .65, "source": "camera_floor_clear"},
                        })
                    elif skill == "place":
                        result["place_verified"] = True
                    return result
                return handler
            registry.register(Tool(
                name=original.name, description=original.description,
                handler=make_handler(name), parameters=original.parameters, contract=original.contract,
            ))

        estimator = StateEstimator()
        result = run_loop(
            ReplayCompleter([]), registry, "빨간 블럭 잡아서 파란 블럭 위에 올려줘",
            execute=True, auto_observe=True, max_steps=12,
            state_estimator=estimator, executive=TaskExecutive(),
        )
        names = [name for name, _ in calls]
        self.assertEqual(names[:2], ["search", "search"])
        self.assertIn("place", names)
        place_args = next(args for name, args in calls if name == "place")
        self.assertEqual(place_args, {"target_color": "red", "destination_color": "blue"})
        self.assertEqual(result.stopped, "final")
        self.assertNotIn("SIM_EPISODE_RESET", result.final or "")


if __name__ == "__main__":
    unittest.main()
