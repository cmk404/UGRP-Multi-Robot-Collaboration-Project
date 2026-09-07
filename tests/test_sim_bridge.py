import os
import socket
import threading
import time
import unittest
from unittest.mock import patch

from sim.bridge import (
    BRIDGE_ALLOWED_ACTIONS,
    BRIDGE_COLOR_ACTIONS,
    BRIDGE_DESTINATION_ACTIONS,
    BridgeState,
)
from sim.worker_contract import REMOTE_WORKER_CONTRACT, worker_contract_compatible
from sim.bridge import STREAM_INTERVAL, socket_peer_closed


class SimBridgeDeliveryTests(unittest.TestCase):

    def test_carry_safe_destination_action_crosses_http_transport_contract(self):
        self.assertIn("search_destination", BRIDGE_ALLOWED_ACTIONS)
        self.assertIn("search_destination", BRIDGE_COLOR_ACTIONS)
        self.assertIn("search_destination", BRIDGE_DESTINATION_ACTIONS)


    def test_disconnected_mjpeg_client_is_detected_without_waiting_for_a_frame_write(self):
        server, client = socket.socketpair()
        try:
            self.assertFalse(socket_peer_closed(server))
            client.close()
            for _ in range(20):
                if socket_peer_closed(server):
                    break
                time.sleep(0.005)
            self.assertTrue(socket_peer_closed(server))
        finally:
            server.close()

    def test_browser_stream_is_not_artificially_capped_at_five_fps(self):
        self.assertLessEqual(STREAM_INTERVAL, 0.05)

    def test_claimed_command_is_redelivered_until_result(self):
        b = BridgeState("token")
        cmd = b.enqueue("grasp_rl")
        first = b.next_cmd(0)
        second = b.next_cmd(0)
        self.assertEqual(first["id"], cmd["id"])
        self.assertEqual(second["id"], cmd["id"])
        self.assertEqual(len(b.queue), 0)
        self.assertEqual(b.inflight["id"], cmd["id"])

        b.set_result(cmd["id"], {"ok": True})
        self.assertIsNone(b.inflight)
        self.assertEqual(b.wait_result(cmd["id"], 0), {"ok": True})
        self.assertIsNone(b.next_cmd(0))

    def test_action_result_fences_late_async_frame_state(self):
        b = BridgeState("token")
        b.push_frame(b"jpeg-before", {"spatial_memory": {"red": {"relation": "HELD"}}}, state_seq=100)
        b.set_result("done", {
            "ok": True,
            "state_seq": 200,
            "state": {"spatial_memory": {"red": {"relation": "ON_BLUE"}}},
        })
        accepted = b.push_frame(
            b"jpeg-stale", {"spatial_memory": {"red": {"relation": "HELD"}}}, state_seq=150
        )
        self.assertFalse(accepted)
        self.assertEqual(b.state_seq, 200)
        self.assertEqual(b.state["spatial_memory"]["red"]["relation"], "ON_BLUE")
        self.assertEqual(b.frame, b"jpeg-before")

        accepted = b.push_frame(
            b"jpeg-after", {"spatial_memory": {"red": {"relation": "ON_BLUE"}}}, state_seq=201
        )
        self.assertTrue(accepted)
        self.assertEqual(b.frame, b"jpeg-after")

    def test_observer_frame_updates_without_rewinding_robot_state(self):
        b = BridgeState("token")
        b.push_frame(b"robot", {"phase": "new"}, state_seq=200, frame_meta={"render_ms": 4.0})
        b.push_observer_frame("cctv_front_left", b"observer", {"render_ms": 12.0})
        self.assertEqual(b.state_seq, 200)
        self.assertEqual(b.state["phase"], "new")
        self.assertEqual(b.frame, b"robot")
        self.assertEqual(b.observer_frame, b"observer")
        self.assertEqual(b.observer_frames["cctv_front_left"], b"observer")
        self.assertEqual(b.frame_meta["render_ms"], 4.0)
        self.assertEqual(b.observer_frame_meta["cctv_front_left"]["render_ms"], 12.0)

    def test_observer_frames_are_namespaced_per_robot(self):
        b = BridgeState("token")
        b.push_observer_frame("cctv_front_left", b"r1-view", {"render_ms": 10.0}, robot_id="r1")
        b.push_observer_frame("cctv_front_left", b"r2-view", {"render_ms": 11.0}, robot_id="r2")
        b.push_observer_frame("cctv_front_left", b"r3-view", {"render_ms": 12.0}, robot_id="r3")
        self.assertEqual(b.observer_frames["r1_cctv_front_left"], b"r1-view")
        self.assertEqual(b.observer_frames["r2_cctv_front_left"], b"r2-view")
        self.assertEqual(b.observer_frames["r3_cctv_front_left"], b"r3-view")
        # Legacy unqualified observer remains a latest-frame alias only.
        self.assertEqual(b.observer_frames["cctv_front_left"], b"r3-view")
        self.assertEqual(b.observer_frame_meta["r1_cctv_front_left"]["render_ms"], 10.0)

    def test_result_state_is_published_immediately(self):
        b = BridgeState("token")
        b.set_result("done", {"state_seq": 9, "state": {"phase": "placed"}})
        self.assertEqual(b.state, {"phase": "placed"})
        self.assertEqual(b.state_seq, 9)

    def test_next_queued_command_waits_for_inflight_result(self):
        b = BridgeState("token")
        one = b.enqueue("approach")
        two = b.enqueue("grasp_rl")
        self.assertEqual(b.next_cmd(0)["id"], one["id"])
        self.assertEqual(b.next_cmd(0)["id"], one["id"])
        b.set_result(one["id"], {"ok": True})
        self.assertEqual(b.next_cmd(0)["id"], two["id"])

    def test_remote_claim_batches_distinct_robot_commands_for_parallel_worker(self):
        b = BridgeState("token")
        r1 = b.enqueue("approach", {"robot_id": "r1", "target_color": "red"})
        r2 = b.enqueue("search", {"robot_id": "r2", "target_color": "blue"})
        r3 = b.enqueue("track", {"robot_id": "r3", "target_color": "yellow"})

        batch = b.next_remote_cmd(0, coalesce_s=0)
        self.assertEqual(batch["action"], "parallel")
        self.assertEqual({cmd["robot_id"] for cmd in batch["commands"]}, {"r1", "r2", "r3"})
        self.assertEqual(len(b.queue), 0)

        b.set_result(batch["id"], {
            "ok": True,
            "action": "parallel",
            "parallel_timing": {"all_overlap_s": 0.25},
            "results": {
                "r1": {"ok": True, "action": "approach", "state": {"robot_id": "r1"}},
                "r2": {"ok": True, "action": "search", "state": {"robot_id": "r2"}},
                "r3": {"ok": True, "action": "track", "state": {"robot_id": "r3"}},
            },
        })
        r1_result = b.wait_result(r1["id"], 0)
        self.assertTrue(r1_result["ok"])
        self.assertEqual(r1_result["parallel_timing"]["all_overlap_s"], 0.25)
        self.assertTrue(b.wait_result(r2["id"], 0)["ok"])
        self.assertTrue(b.wait_result(r3["id"], 0)["ok"])
        self.assertIsNone(b.inflight)

    def test_remote_claim_does_not_wait_for_a_lone_robot_command(self):
        b = BridgeState("token")
        cmd = b.enqueue("track", {"robot_id": "r1"})
        started = time.monotonic()
        claimed = b.next_remote_cmd(0.5)
        elapsed = time.monotonic() - started
        self.assertEqual(claimed["id"], cmd["id"])
        self.assertLess(elapsed, 0.10)

    def test_explicit_team_batch_rendezvouses_commands_arriving_after_worker_poll(self):
        b = BridgeState("token")
        first = b.enqueue("search", {
            "robot_id": "r1", "target_color": "red",
            "team_batch_id": "sim:cr000001", "team_batch_expected": 3,
        })
        claimed: list[dict] = []
        with patch("sim.bridge.REMOTE_TEAM_BATCH_WINDOW_S", 0.5):
            worker = threading.Thread(
                target=lambda: claimed.append(b.next_remote_cmd(0.05, coalesce_s=0)),
            )
            worker.start()
            # Arrive after the worker's idle poll timeout. The explicit TEAM
            # rendezvous must remain open independently of that poll cadence.
            time.sleep(0.12)
            second = b.enqueue("search", {
                "robot_id": "r2", "target_color": "blue",
                "team_batch_id": "sim:cr000001", "team_batch_expected": 3,
            })
            third = b.enqueue("search", {
                "robot_id": "r3", "target_color": "yellow",
                "team_batch_id": "sim:cr000001", "team_batch_expected": 3,
            })
            worker.join(timeout=2.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(claimed[0]["action"], "parallel")
        self.assertEqual(claimed[0]["team_batch_id"], "sim:cr000001")
        self.assertEqual(
            {cmd["id"] for cmd in claimed[0]["commands"]},
            {first["id"], second["id"], third["id"]},
        )

    def test_parallel_member_result_releases_fast_robot_before_batch_finishes(self):
        b = BridgeState("token")
        r1 = b.enqueue("track", {"robot_id": "r1"})
        r2 = b.enqueue("track", {"robot_id": "r2"})
        batch = b.next_remote_cmd(0, coalesce_s=0)
        self.assertEqual(batch["action"], "parallel")
        self.assertTrue(b.set_partial_result(batch["id"], r1["id"], {
            "ok": True, "action": "track", "state": {"robot_id": "r1"},
        }))
        self.assertTrue(b.wait_result(r1["id"], 0)["ok"])
        self.assertIsNotNone(b.inflight)
        self.assertIsNone(b.wait_result(r2["id"], 0))
        # r2's zero-timeout abandoned only its own caller; use a fresh bridge
        # below to verify final fan-out does not resurrect an already delivered
        # partial result.
        b = BridgeState("token")
        r1 = b.enqueue("track", {"robot_id": "r1"})
        r2 = b.enqueue("track", {"robot_id": "r2"})
        batch = b.next_remote_cmd(0, coalesce_s=0)
        self.assertTrue(b.set_partial_result(batch["id"], r1["id"], {"ok": True}))
        self.assertTrue(b.wait_result(r1["id"], 0)["ok"])
        b.set_result(batch["id"], {
            "ok": True, "action": "parallel",
            "results": {"r1": {"ok": True}, "r2": {"ok": True}},
        })
        self.assertNotIn(r1["id"], b.results)
        self.assertTrue(b.wait_result(r2["id"], 0)["ok"])
        self.assertIsNone(b.inflight)

    def test_stream_status_is_edge_triggered_and_bounded(self):
        b = BridgeState("token")
        enabled, count, seq = b.stream_status()
        self.assertFalse(enabled)
        self.assertEqual(count, 0)
        b.set_active_streams(1)
        enabled, count, next_seq = b.stream_status()
        self.assertTrue(enabled)
        self.assertEqual(count, 1)
        self.assertGreater(next_seq, seq)
        b.set_active_streams(0)
        self.assertEqual(b.stream_status()[:2], (False, 0))

    def test_stream_control_enables_only_subscribed_surfaces(self):
        b = BridgeState("token")
        self.assertTrue(b.add_stream_subscription("robot:r1"))
        self.assertTrue(b.add_stream_subscription("observer:r3"))
        payload, _ = b.stream_control()
        self.assertTrue(payload["enabled"])
        self.assertEqual(payload["active_streams"], 2)
        self.assertEqual(payload["robot_ids"], ["r1", "r3"])
        self.assertTrue(payload["observer"])
        self.assertFalse(payload["team_overview"])
        b.remove_stream_subscription("robot:r1")
        b.remove_stream_subscription("observer:r3")
        payload, _ = b.stream_control()
        self.assertFalse(payload["enabled"])
        self.assertEqual(payload["robot_ids"], [])

    def test_team_overview_subscription_does_not_enable_robot_cameras(self):
        b = BridgeState("token")
        self.assertTrue(b.add_stream_subscription("team_overview"))
        payload, _ = b.stream_control()
        self.assertEqual(payload["robot_ids"], [])
        self.assertFalse(payload["observer"])
        self.assertTrue(payload["team_overview"])

    def test_trace_result_is_returned_before_background_persistence_finishes(self):
        b = BridgeState("token")
        with patch.object(b, "_persist_sim_trace", side_effect=lambda job: time.sleep(0.15)):
            started = time.monotonic()
            b.set_result("cmd", {
                "ok": True,
                "_sim_trace": {"seed": 1, "action": "pick", "frames": []},
                "_sim_diagnostic": {"status": "CLEAN"},
            })
            elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.10)
        result = b.wait_result("cmd", 0)
        self.assertTrue(result["sim_trace_id"].endswith("_cmd"))
        self.assertTrue(b.wait_for_trace_jobs())

    def test_remote_batch_preserves_same_robot_command_order(self):
        b = BridgeState("token")
        r1_first = b.enqueue("search", {"robot_id": "r1"})
        r1_second = b.enqueue("approach", {"robot_id": "r1"})
        r2 = b.enqueue("search", {"robot_id": "r2"})

        batch = b.next_remote_cmd(0, coalesce_s=0)
        self.assertEqual([cmd["id"] for cmd in batch["commands"]], [r1_first["id"], r2["id"]])
        self.assertEqual([cmd["id"] for cmd in b.queue], [r1_second["id"]])

    def test_seed_parameter_is_retained_on_reset_command(self):
        b = BridgeState("token")
        cmd = b.enqueue("reset", {"seed": 12345})
        self.assertEqual(cmd["action"], "reset")
        self.assertEqual(cmd["seed"], 12345)
        self.assertEqual(b.next_cmd(0)["seed"], 12345)

    def test_remote_worker_contract_rejects_legacy(self):
        self.assertTrue(worker_contract_compatible(REMOTE_WORKER_CONTRACT))
        self.assertFalse(worker_contract_compatible("legacy"))
        self.assertFalse(worker_contract_compatible(None))

    def test_new_remote_replaces_old_generation(self):
        bridge = BridgeState("token")
        old = object()
        new = object()
        g1, previous1 = bridge.register_remote(old)
        self.assertIsNone(previous1)
        self.assertTrue(bridge.remote_is_current(g1))
        g2, previous2 = bridge.register_remote(new)
        self.assertIs(previous2, old)
        self.assertFalse(bridge.remote_is_current(g1))
        self.assertTrue(bridge.remote_is_current(g2))
        self.assertFalse(bridge.unregister_remote(g1, old))
        self.assertEqual(bridge.remote_ws_count, 1)
        self.assertTrue(bridge.unregister_remote(g2, new))
        self.assertEqual(bridge.remote_ws_count, 0)

    def test_remote_worker_metadata_tracks_current_generation(self):
        bridge = BridgeState("token")
        g1, _ = bridge.register_remote(object(), "colab", "T4")
        self.assertEqual(bridge.remote_provider, "colab")
        self.assertEqual(bridge.remote_machine, "T4")
        g2, _ = bridge.register_remote(object(), "lightning", "L4")
        self.assertFalse(bridge.remote_is_current(g1))
        self.assertTrue(bridge.remote_is_current(g2))
        self.assertEqual(bridge.remote_provider, "lightning")
        self.assertEqual(bridge.remote_machine, "L4")
        self.assertTrue(bridge.unregister_remote(g2))
        self.assertIsNone(bridge.remote_provider)
        self.assertIsNone(bridge.remote_machine)

    def test_remote_instance_id_tracks_current_worker(self):
        bridge = BridgeState("token")
        g, _ = bridge.register_remote(object(), "lightning", "T4", "abc123", "contract-v1")
        self.assertTrue(bridge.remote_is_current(g))
        self.assertEqual(bridge.remote_instance_id, "abc123")
        self.assertEqual(bridge.remote_worker_contract, "contract-v1")
        bridge.unregister_remote(g)
        self.assertIsNone(bridge.remote_instance_id)
        self.assertIsNone(bridge.remote_worker_contract)


    def test_same_process_reconnect_preserves_episode_and_inflight(self):
        bridge = BridgeState("token")
        g1, _ = bridge.register_remote(object(), "lightning", "T4", "same-process")
        first_episode = bridge.remote_episode_id
        cmd = bridge.enqueue("map_blue")
        self.assertEqual(bridge.next_cmd(0), cmd)
        self.assertTrue(bridge.unregister_remote(g1))

        g2, _ = bridge.register_remote(object(), "lightning", "T4", "same-process")
        self.assertEqual(bridge.remote_episode_id, first_episode)
        self.assertEqual(bridge.next_cmd(0), cmd)
        self.assertTrue(bridge.remote_is_current(g2))

    def test_replacement_process_aborts_old_episode_commands(self):
        bridge = BridgeState("token")
        g1, _ = bridge.register_remote(object(), "lightning", "T4", "old-process")
        first_episode = bridge.remote_episode_id
        inflight = bridge.enqueue("carry")
        self.assertEqual(bridge.next_cmd(0), inflight)
        queued = bridge.enqueue("place_on_blue")
        self.assertTrue(bridge.unregister_remote(g1))

        bridge.register_remote(object(), "lightning", "T4", "new-process")
        self.assertEqual(bridge.remote_episode_id, first_episode + 1)
        self.assertIsNone(bridge.inflight)
        self.assertEqual(len(bridge.queue), 0)
        for cmd in (inflight, queued):
            result = bridge.wait_result(cmd["id"], 0)
            self.assertIsNotNone(result)
            self.assertFalse(result["ok"])
            self.assertEqual(result["failure_code"], "SIM_EPISODE_RESET")

    def test_replacement_process_releases_every_parallel_member_caller(self):
        bridge = BridgeState("token")
        g1, _ = bridge.register_remote(object(), "mac", "M3", "old-process")
        r1 = bridge.enqueue("search", {"robot_id": "r1", "target_color": "red"})
        r2 = bridge.enqueue("search", {"robot_id": "r2", "target_color": "blue"})
        batch = bridge.next_remote_cmd(0, coalesce_s=0)
        self.assertEqual(batch["action"], "parallel")
        self.assertTrue(bridge.unregister_remote(g1))

        bridge.register_remote(object(), "mac", "M3", "new-process")

        self.assertIsNone(bridge.inflight)
        for member in (r1, r2):
            result = bridge.wait_result(member["id"], 0)
            self.assertIsNotNone(result)
            self.assertEqual(result["failure_code"], "SIM_EPISODE_RESET")
            self.assertEqual(result["robot_id"], member["robot_id"])

    def test_gpu_only_flag_is_explicit_production_gate(self):
        with patch.dict(os.environ, {"UGRP_SIM_GPU_ONLY": "1"}):
            bridge = BridgeState("token")
        self.assertTrue(bridge.gpu_only)

    def test_remote_gpu_is_parked_until_explicit_handoff(self):
        bridge = BridgeState("token")
        bridge.push_frame(b"cpu-frame", {"phase": "cpu"}, state_seq=100)
        generation, _ = bridge.register_remote(object())

        self.assertTrue(bridge.remote_is_current(generation))
        self.assertFalse(bridge.remote_is_authoritative(generation))
        # The current CPU publication remains valid while the GPU is parked.
        self.assertEqual(bridge.state, {"phase": "cpu"})

        self.assertTrue(bridge.set_remote_authoritative(True))
        self.assertTrue(bridge.remote_is_authoritative(generation))
        # Authority handoff starts a fresh sequence epoch; Oracle/Colab clocks
        # must never be compared across workers.
        self.assertIsNone(bridge.state_seq)
        self.assertEqual(bridge.state, {})
        self.assertIsNone(bridge.frame)

    def test_remote_authority_requires_a_connected_gpu(self):
        bridge = BridgeState("token")
        self.assertFalse(bridge.set_remote_authoritative(True))
        self.assertFalse(bridge.remote_authoritative)


class SimBridgeRecoveryTests(unittest.TestCase):

    def test_same_authoritative_process_reconnect_resumes_without_handoff(self):
        bridge = BridgeState("token")
        g1, _ = bridge.register_remote(object(), "mac", "APPLE_M3", "proc-1")
        self.assertTrue(bridge.set_remote_authoritative(True))
        cmd = bridge.enqueue("approach")
        self.assertEqual(bridge.next_cmd(0)["id"], cmd["id"])
        # Socket blip while the command is still executing on the worker.
        self.assertTrue(bridge.unregister_remote(g1))
        self.assertFalse(bridge.remote_authoritative)

        g2, _ = bridge.register_remote(object(), "mac", "APPLE_M3", "proc-1")
        self.assertTrue(bridge.remote_is_authoritative(g2))
        self.assertEqual(bridge.inflight["id"], cmd["id"])
        bridge.set_result(cmd["id"], {"ok": True})
        self.assertIsNone(bridge.inflight)

    def test_replacement_process_is_still_parked(self):
        bridge = BridgeState("token")
        g1, _ = bridge.register_remote(object(), "mac", "APPLE_M3", "proc-1")
        self.assertTrue(bridge.set_remote_authoritative(True))
        self.assertTrue(bridge.unregister_remote(g1))
        g2, _ = bridge.register_remote(object(), "mac", "APPLE_M3", "proc-2")
        self.assertFalse(bridge.remote_is_authoritative(g2))

    def test_wait_result_timeout_releases_inflight_and_drops_late_result(self):
        bridge = BridgeState("token")
        cmd = bridge.enqueue("pick")
        self.assertEqual(bridge.next_cmd(0)["id"], cmd["id"])
        self.assertIsNone(bridge.wait_result(cmd["id"], 0))
        self.assertIsNone(bridge.inflight)

        nxt = bridge.enqueue("search")
        self.assertEqual(bridge.next_cmd(0)["id"], nxt["id"])
        # The late reply must not be confused with the new command's result.
        bridge.set_result(cmd["id"], {"ok": True, "state_seq": 5, "state": {"phase": "late"}})
        self.assertNotIn(cmd["id"], bridge.results)
        self.assertEqual(bridge.state, {"phase": "late"})
        self.assertEqual(bridge.inflight["id"], nxt["id"])

    def test_inflight_is_not_handed_to_a_different_executor(self):
        bridge = BridgeState("token")
        cmd = bridge.enqueue("carry")
        self.assertEqual(bridge.next_cmd(0, owner="remote")["id"], cmd["id"])
        self.assertIsNone(bridge.next_cmd(0, owner="local"))
        self.assertEqual(bridge.next_cmd(0, owner="remote")["id"], cmd["id"])

    def test_queue_depth_is_bounded(self):
        from sim.bridge import MAX_QUEUE_DEPTH, QueueFull
        bridge = BridgeState("token")
        for _ in range(MAX_QUEUE_DEPTH):
            bridge.enqueue("observe_scene")
        with self.assertRaises(QueueFull):
            bridge.enqueue("observe_scene")

    def test_trace_seed_cannot_escape_trace_root(self):
        import tempfile
        from pathlib import Path
        import sim.bridge as bridge_mod
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "sim_traces"
            with patch.object(bridge_mod, "SIM_TRACE_ROOT", root):
                bridge = BridgeState("token")
                bridge.set_result("cmd", {
                    "ok": True,
                    "_sim_trace": {"seed": "../../escape", "action": "pick", "frames": []},
                    "_sim_diagnostic": {"status": "CLEAN"},
                })
                self.assertTrue(bridge.wait_for_trace_jobs())
                dirs = [p for p in root.iterdir()] if root.exists() else []
                self.assertEqual(len(dirs), 1)
                self.assertEqual(dirs[0].parent.resolve(), root.resolve())
                self.assertNotIn("/", dirs[0].name)
                self.assertFalse((Path(tmp) / "escape").exists())


if __name__ == "__main__":
    unittest.main()
