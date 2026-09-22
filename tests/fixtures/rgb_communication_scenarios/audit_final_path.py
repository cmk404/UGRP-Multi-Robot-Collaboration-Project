"""A2 independent final-path offline probes against a supplied frozen checkout.

Run with --source-root CHECKOUT --expected-sha FULL_SHA. Nothing initializes a
physical backend or calls a provider. The B/C fixture constructors are reused;
assertions below belong to A. Missing modules, dirty source and failing probes
are NO-GO, not skips. This is not the complete live/physical audit certificate.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest


class FinalPathAudit(unittest.TestCase):
    def test_b_queued_command_expires_before_dispatch(self):
        b = importlib.import_module("test_rgb_execution_port")
        port, endpoints = b.build_port()
        for rid in ("r1", "r2"):
            request = copy.deepcopy(b.FIXTURE["pair_requests"][rid])
            request["expires_at_s"] = 10.
            result = port.submit(rid, request)
        lease = result["lease_id"]
        self.assertEqual(port.submit("r1", b.command("r1", "beam-task", lease))["status"], "QUEUED")
        port.tick(6.)
        late = b.command("r2", "beam-task", lease)
        late["requested_at_s"] = 6.
        port.submit("r2", late)
        port.tick(6.)
        self.assertFalse(any(endpoint.applied for endpoint in endpoints.values()))
        self.assertFalse(port.local_status("r1")["active"])
        port.close(6.)

    def test_b_pending_cancel_and_late_consent_cannot_revive_task(self):
        b = importlib.import_module("test_rgb_execution_port")
        port, _ = b.build_port()
        request = b.FIXTURE["pair_requests"]["r1"]
        port.submit("r1", request)
        observed = port.observe("r1")
        cancel = {"kind": "cancel_pending", "request_id": "a2-cancel", "task_id": "beam-task",
                  "reason": "actor changed its own choice", "observation_id": observed["observation_id"],
                  "decision_id": "a2-cancel-decision", "requested_at_s": 0.}
        if "own_revision" in observed:
            cancel["expected_revision"] = observed["own_revision"]
        self.assertEqual(port.submit("r1", cancel)["status"], "ACCEPTED")
        self.assertEqual(port.local_status("r1")["pending"], [])
        self.assertEqual(port.submit("r2", b.FIXTURE["pair_requests"]["r2"])["status"], "REJECTED")
        port.close(0.)

    def test_b_actual_command_retains_original_decision_and_observation(self):
        b = importlib.import_module("test_rgb_execution_port")
        port, _ = b.build_port(evaluator=lambda: {})
        lease = b.pair(port)
        commands = {rid: b.command(rid, "beam-task", lease) for rid in ("r1", "r2")}
        for rid, command in commands.items():
            port.submit(rid, command)
        port.tick(0.)
        emitted = [row for row in port.evaluation_snapshot()["coordination_audit"] if row["event"] == "LOCAL_COMMAND"]
        self.assertEqual(len(emitted), 2)
        for row in emitted:
            original = commands[row["robot_id"]]
            for field in ("decision_id", "observation_id", "command_id"):
                self.assertEqual(row[field], original[field])
        port.close(0.)

    def test_c_slow_peer_cancelled_reply_cannot_send_or_finish(self):
        c = importlib.import_module("harness.rgb_communication_async")
        fixture = importlib.import_module("test_rgb_communication_runtime")
        control = c.RuntimeControl()
        started, release = threading.Event(), threading.Event()
        seen_fast = []

        def slow(request):
            started.set()
            self.assertTrue(release.wait(1.))
            return {"action": {"kind": "finish", "claim": "mission_complete"},
                    "message": {"recipients": ["r3"], "content": "late cancelled claim", "ttl_s": 2.}}

        def fast(request):
            self.assertTrue(started.wait(1.))
            seen_fast.append(request["robot_id"])
            control.cancel_inference("r1")
            release.set()
            return {"action": {"kind": "finish", "claim": "mission_complete"}, "message": None}

        port = fixture.FixturePort()
        try:
            result = c.run_rgb_communication_async(port, {
                rid: c.OfflineDecisionPlanner(slow if rid == "r1" else fast) for rid in ("r1", "r2", "r3")},
                condition="natural", common_task=fixture.FIXTURE["static_context"]["task"], control=control,
                limits=c.AsyncRuntimeLimits(max_ticks=30, max_calls_per_robot=1,
                                             decision_period_s=.05, poll_period_s=.002))
        finally:
            release.set()
        self.assertEqual(set(seen_fast), {"r2", "r3"})
        self.assertFalse(result["actor_finish_claims"]["r1"])
        self.assertEqual(result["messages"]["r1"], 0)
        self.assertTrue(any(e["event_type"] == "planner_response_rejected" and
                            e["payload"].get("reason") == "CANCELLED_REPLY" for e in result["events"]))

    def test_c_provenance_and_supervisor_truth_never_reach_planner(self):
        c = importlib.import_module("harness.rgb_communication_async")
        fixture = importlib.import_module("test_rgb_communication_runtime")
        snapshots = []
        for hidden in ("SECRET_A", "SECRET_B"):
            captured = {}
            def finish(request):
                captured[request["robot_id"]] = copy.deepcopy(request)
                return {"action": {"kind": "finish", "claim": "mission_complete"}, "message": None}
            result = c.run_rgb_communication_async(fixture.FixturePort(tick_secret=hidden), {
                rid: c.OfflineDecisionPlanner(finish) for rid in ("r1", "r2", "r3")},
                condition="none", common_task=fixture.FIXTURE["static_context"]["task"], run_id="a2-paired",
                provenance={"evaluator_success": hidden, "split": hidden},
                limits=c.AsyncRuntimeLimits(max_ticks=30, max_calls_per_robot=1,
                                             decision_period_s=.05, poll_period_s=.002))
            self.assertNotEqual(result["outcome"], "success")
            self.assertEqual(result["external_model_calls"], 0)
            self.assertNotIn(hidden, json.dumps(captured))
            snapshots.append(captured)
        self.assertEqual(snapshots[0], snapshots[1])

    def test_d_live_provider_has_no_self_asserted_budget_bypass(self):
        d = importlib.import_module("harness.rgb_communication_study")
        settings = d.provider_settings({"model": "offline-test-model", "max_output_tokens": 8})
        self.assertFalse(settings.readiness()["ready"])
        with self.assertRaises(ValueError):
            d.provider_settings({"model": "offline-test-model", "input_bound_evidence": "trust me"})

    def test_c_serializer_transmits_history_jpeg_not_hash_only(self):
        c = importlib.import_module("harness.rgb_communication_async")
        p = importlib.import_module("harness.rgb_communication_planner")
        fixture = importlib.import_module("test_rgb_communication_runtime")
        captured = []
        def finish(request):
            captured.append(request)
            return {"action": {"kind": "finish", "claim": "mission_complete"}, "message": None}
        c.run_rgb_communication_async(fixture.FixturePort(), {
            rid: c.OfflineDecisionPlanner(finish) for rid in ("r1", "r2", "r3")},
            condition="none", common_task=fixture.FIXTURE["static_context"]["task"],
            limits=c.AsyncRuntimeLimits(max_ticks=30, max_calls_per_robot=1,
                                         decision_period_s=.05, poll_period_s=.002))
        request = copy.deepcopy(next(r for r in captured if r["robot_id"] == "r1"))
        old = copy.deepcopy(request["observation"])
        old["observation_id"] = "old-r1"
        raw = b"\xff\xd8a2-history-fixture\xff\xd9"
        for item in old["images"].values():
            item["jpeg_base64"] = base64.b64encode(raw).decode()
            item["sha256"] = hashlib.sha256(raw).hexdigest()
        request["memory"]["own_observations"] = [old, request["observation"]]
        settings = p.ProviderSettings(input_token_bound=lambda body: 1000,
            input_bound_evidence="TEST ONLY", output_limit_evidence="TEST ONLY", history_image_pairs=1)
        with tempfile.TemporaryDirectory() as directory:
            planner = p.GeminiRGBPlanner("r1", settings, Path(directory),
                http_open=lambda *a, **kw: self.fail("network forbidden"), evidence_kind="fixture")
            prepared = planner.prepare(request)  # No complete/provider call.
            urls = []
            def walk(value):
                if isinstance(value, dict):
                    if value.get("type") == "image_url":
                        urls.append(value["image_url"]["url"])
                    for child in value.values():
                        walk(child)
                elif isinstance(value, list):
                    for child in value:
                        walk(child)
            walk(json.loads(prepared.body))
            self.assertEqual(len(urls), 4)
            self.assertTrue(any(base64.b64encode(raw).decode() in url for url in urls))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--expected-sha", required=True)
    args = parser.parse_args()
    root = args.source_root.resolve()
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip()
    if sha != args.expected_sha or dirty:
        print(json.dumps({"verdict": "no_go", "reason": "source SHA or clean state mismatch"}))
        return 2
    sys.path[:0] = [str(root), str(root / "tests")]
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(FinalPathAudit)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    files = subprocess.check_output(["git", "ls-files", "-z", "harness", "sim", "scripts"], cwd=root).decode().split("\0")
    print(json.dumps({"source_sha": sha, "source_files": {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files if name.endswith(".py")},
        "scope": "offline independent seam probes; physical/image/provider/D capsule audit still separate",
        "tests": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
        "verdict": "offline_subset_pass" if result.wasSuccessful() else "no_go",
        "live_readiness": False}, sort_keys=True))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
