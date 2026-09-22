"""A2 offline spec tests plus explicitly labelled integration-base findings.

No simulator or provider is constructed. Baseline characterization tests assert
the reproduced gap, NOT compliance. Final B/C/D audits must use the new public
APIs and report each gate independently (see a2-audit.md).
"""
from __future__ import annotations

import copy
import base64
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from harness.rgb_communication_scenarios import (
    AUDIT_CASES, READINESS_SCHEMA, SOURCE_FILES, ScenarioError,
    assess_readiness, canonical_sha256, load_scenarios, paired_schedule,
    summarize_episodes, validate_episode, validate_scenario, audit_exposure,
    validate_training_comparison, assess_environment_readiness, ENVIRONMENT_PREFLIGHT_CHECKS,
)
from harness.rgb_execution_contract import SkillCapability
from harness.rgb_execution_port import RGBExecutionPort
from harness.rgb_communication_runtime import RuntimeLimits, run_rgb_communication


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "tests/fixtures/rgb_communication_scenarios/catalog.json"
B_FIXTURE = json.loads((ROOT / "tests/fixtures/rgb_execution_port/basic.json").read_text())


class Endpoint:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.applied = []
        self.holds = []

    def validate_bounded(self, action, duration_s):
        if set(action) != {"kind", "value"}:
            raise ValueError("test action invalid")

    def apply_bounded(self, action, now_s, duration_s):
        self.applied.append((copy.deepcopy(action), now_s, duration_s))

    def hold(self, now_s):
        self.holds.append(now_s)

    def tick(self, now_s):
        pass


def port_fixture(evaluator=None):
    endpoints = {rid: Endpoint(rid) for rid in ("r1", "r2", "r3")}
    port = RGBExecutionPort(
        endpoints, frame_source=lambda rid, now: {"own_rgb": b"\xff\xd8test\xff\xd9",
                                                 "top_rgb": b"\xff\xd8top\xff\xd9"},
        static_context=B_FIXTURE["static_context"], clock_domain="sim", evaluation_source=evaluator,
        capabilities=[SkillCapability("pair_transport", frozenset({"CARRY"}), frozenset({2}),
                                      frozenset({"drive"}), .5, distinct_roles=True),
                      SkillCapability("solo_transport", frozenset({"CARRY"}), frozenset({1}),
                                      frozenset({"drive"}), .5)])
    return port, endpoints


def pair(port, expiry=10.):
    for rid in ("r1", "r2"):
        request = copy.deepcopy(B_FIXTURE["pair_requests"][rid])
        request["expires_at_s"] = expiry
        result = port.submit(rid, request)
    assert result["status"] == "ACCEPTED"
    return result["lease_id"]


def command(rid, lease, *, now=0., suffix="", task_id="beam-task"):
    return {"kind": "command", "request_id": rid + "-cmd" + suffix,
            "task_id": task_id, "lease_id": lease, "command_id": rid + "-command" + suffix,
            "stage": "CARRY", "action": {"kind": "drive", "value": rid}, "duration_s": .2,
            "observation_id": rid + "-obs", "decision_id": rid + "-decision" + suffix,
            "requested_at_s": now}


def stop(rid, lease, *, kind="interrupt"):
    return {"kind": kind, "request_id": rid + "-stop", "task_id": "beam-task",
            "lease_id": lease, "reason": "own RGB revision", "observation_id": rid + "-obs",
            "decision_id": rid + "-stop-decision", "requested_at_s": 0.}


class Planner:
    model_name = "offline-fixture"
    evidence_kind = "fixture"

    def __init__(self, callback=None):
        self.requests = []
        self.callback = callback

    def decide(self, request):
        self.requests.append(copy.deepcopy(request))
        if self.callback:
            return self.callback(request)
        return {"action": {"kind": "wait"}, "message": None, "raw_text": "wait"}


def run(port, *, callback=None, condition="none", ticks=2, clock=None):
    planners = {rid: Planner(callback if rid == "r1" else None) for rid in ("r1", "r2", "r3")}
    kwargs = {"wall_clock": clock} if clock else {}
    result = run_rgb_communication(port, planners, condition=condition,
                                  common_task=B_FIXTURE["static_context"]["task"], run_id="A2-offline",
                                  limits=RuntimeLimits(max_ticks=ticks, max_calls_per_robot=ticks,
                                                       decision_timeout_s=1., wall_timeout_s=20.), **kwargs)
    return result, planners


def linked_fixture(spec, condition="structured"):
    """Small synthetic ledger, not research RGB or proof of interpretation."""
    events = []
    image = {"top_rgb": {"sha256": "a" * 64}, "own_rgb": {"sha256": "b" * 64}}

    def add(eid, kind, ids, payload, robot="r1"):
        row = {"event_id": eid, "event_type": kind, "robot_id": robot,
               "condition": condition, "related_ids": ids, "payload": copy.deepcopy(payload)}
        events.append(row)
        return row

    add("old-command", "command_issued", {"decision_id": "old"},
        {"command_id": "cmd-old", "action": {"drive": "forward"}})
    observations = []
    for number in range(spec["event"]["minimum_frames"]):
        obs = {"observation_id": f"obs{number}", "images": copy.deepcopy(image), "observed_at_s": float(number),
               "own_issued_commands": [{"command_id": "cmd-old"}]}
        observations.append(obs)
        add(f"observation{number}", "observation_captured", {"observation_id": f"obs{number}"}, obs)
    request = {"observation": observations[-1], "memory": {"own_observations": observations,
                                                            "received_messages": []}}
    add("request1", "planner_requested", {"request_id": "req1"}, request)
    add("decision1", "planner_responded", {"request_id": "req1", "decision_id": "dec1"},
        {"action": {"kind": "interrupt"}, "raw_text": "Visible approach conflict; withdraw."})
    if condition != "none":
        add("report", "message_sent", {"decision_id": "dec1", "message_id": "message1"},
            {"recipients": ["r2"], "content": "help"})
    add("cancel", "action_submitted", {"decision_id": "dec1"}, {"kind": "interrupt"})
    add("request2", "planner_requested", {"request_id": "req2"}, request)
    add("decision2", "planner_responded", {"request_id": "req2", "decision_id": "dec2"},
        {"action": {"kind": "command", "direction": "back"}})
    add("new-command", "command_issued", {"decision_id": "dec2"},
        {"command_id": "cmd-new", "action": {"drive": "back"}})
    links = {"schema_version": "rgb-scenario-links.v1", "scenario_id": spec["scenario_id"],
             "condition": condition, "physical_event": {
                 "event_id": spec["event"]["event_id"], "reached": True, "recovered": False,
                 "evaluator_artifact_sha256": "f" * 64},
             "recognitions": [{"robot_id": "r1", "observation_event_ids": [
                 f"observation{i}" for i in range(len(observations))],
                 "decision_event_id": "decision1", "report_event_id": "report" if condition != "none" else None,
                 "revision_event_id": "decision2", "cancel_event_id": "cancel",
                 "prior_command_event_id": "old-command", "command_event_id": "new-command"}]}
    return events, links


class ScenarioContractTests(unittest.TestCase):
    def setUp(self):
        self.specs = load_scenarios(CATALOG)

    def test_three_families_two_candidates_are_not_live_readiness(self):
        self.assertEqual([s["family"] for s in self.specs], ["normal", "contention", "recovery"])
        for spec in self.specs:
            result = assess_readiness(spec, {}, expected_source_sha="9" * 40,
                                      artifact_root=ROOT, source_root=ROOT)
            self.assertFalse(result["ready"])

    def test_public_event_cannot_be_encoded_as_private_or_hidden_injection(self):
        for field, value in (("visibility", "private_known_to_r1"), ("physical_injection", {"oracle_failure": True})):
            spec = copy.deepcopy(self.specs[1])
            spec["event"][field] = value
            with self.assertRaises(ScenarioError):
                validate_scenario(spec)

    def test_pilot_six_paired_trials_and_separate_ablation_blocks(self):
        specs = self.specs[:2]
        schedule = paired_schedule(specs, seeds=[11], repeats=1, order_seed=22)
        self.assertEqual(len(schedule), 6)
        self.assertEqual(schedule, paired_schedule(specs, seeds=[11], repeats=1, order_seed=22))
        for spec in specs:
            self.assertEqual({r["condition"] for r in schedule if r["scenario_id"] == spec["scenario_id"]},
                             {"none", "structured", "natural"})
        with self.assertRaises(ScenarioError):
            paired_schedule(specs, seeds=[11, 12], repeats=1, order_seed=1)
        repeated = paired_schedule(self.specs, seeds=[11, 12], repeats=2, order_seed=22,
                                   pilot=False, ablation="message_delay")
        self.assertEqual(len(repeated), 36)
        self.assertTrue(all("message_delay" in r["paired_block"] for r in repeated))

    def test_denominators_include_unreached_api_error_failure_and_unrun(self):
        schedule = paired_schedule(self.specs[:2], seeds=[11], repeats=1, order_seed=1)
        rows = [dict(schedule[0], outcome="api_error", mission_complete=False, event_reached=False,
                     recovered=False, costs={"model_calls": 2, "tokens": None})]
        report = summarize_episodes(schedule, rows)
        self.assertEqual(report["planned_n"], 6)
        condition = report["by_condition"][schedule[0]["condition"]]
        self.assertEqual(condition["planned_n"], 2)
        self.assertEqual(condition["outcomes"], {"api_error": 1, "unrun": 1})
        self.assertEqual(condition["costs_all_outcomes"]["model_calls"], {"measured_n": 1, "total": 2})
        self.assertIsNone(condition["costs_all_outcomes"]["tokens"]["total"])
        self.assertEqual(condition["event_reached_n"], 0)
        self.assertFalse(report["causal_effect_established"])

    def test_event_chain_preserves_silence_revision_and_real_command_difference(self):
        for condition in ("none", "structured", "natural"):
            events, links = linked_fixture(self.specs[2], condition)
            report = validate_episode(self.specs[2], events, links, condition=condition)
            self.assertEqual(report["complete_recovery_chain_n"], 1)
            self.assertFalse(report["physical_recovered"])
            self.assertEqual(report["chains"][0]["silent"], condition == "none")
            self.assertFalse(report["causal_effect_established"])

    def test_physical_failure_does_not_automatically_mean_actor_recognition(self):
        events, links = linked_fixture(self.specs[2])
        links["recognitions"] = []
        report = validate_episode(self.specs[2], events, links, condition="structured")
        self.assertTrue(report["physical_event_reached"])
        self.assertEqual(report["recognized_n"], 0)

    def test_reject_cross_actor_future_missing_frame_or_decision_command_provenance(self):
        for mutation in ("actor", "future", "hash", "command"):
            events, links = linked_fixture(self.specs[2])
            if mutation == "actor":
                events[1]["robot_id"] = "r2"
            elif mutation == "future":
                events.append(events.pop(1))
            elif mutation == "hash":
                events[1]["payload"]["images"]["top_rgb"]["sha256"] = "c" * 64
            else:
                events[-1]["related_ids"]["decision_id"] = "stale-decision"
            with self.subTest(mutation=mutation), self.assertRaises(ScenarioError):
                validate_episode(self.specs[2], events, links, condition="structured")

    def test_stall_needs_two_times_and_own_command_not_just_two_ids(self):
        events, links = linked_fixture(self.specs[2])
        events[2]["payload"]["observed_at_s"] = 0.
        with self.assertRaises(ScenarioError):
            validate_episode(self.specs[2], events, links, condition="structured")

    def test_expired_or_undelivered_claim_cannot_support_revision(self):
        events, links = linked_fixture(self.specs[2])
        revision_request = next(e for e in events if e["event_id"] == "request2")
        revision_request["payload"]["memory"]["received_messages"] = [{"message_id": "fake", "expires_at_s": 0.5}]
        with self.assertRaises(ScenarioError):
            validate_episode(self.specs[2], events, links, condition="structured")

    def test_rejected_response_cannot_be_counted_as_recognition(self):
        events, links = linked_fixture(self.specs[2])
        events.append({"event_id": "rejection", "event_type": "planner_response_rejected", "robot_id": "r1",
                       "condition": "structured", "related_ids": {"request_id": "req1"}, "payload": {"reason": "LATE_REPLY"}})
        with self.assertRaisesRegex(ScenarioError, "rejected reply"):
            validate_episode(self.specs[2], events, links, condition="structured")

    def test_actual_wire_must_contain_old_frame_not_only_memory_hash(self):
        events, links = linked_fixture(self.specs[2])
        raw_images = {f"obs{i}": f"\xff\xd8frame{i}\xff\xd9".encode("latin-1") for i in range(2)}
        def fix(value):
            if isinstance(value, dict):
                if value.get("observation_id") in raw_images and "images" in value:
                    value["images"]["top_rgb"]["sha256"] = hashlib.sha256(raw_images[value["observation_id"]]).hexdigest()
                for child in value.values():
                    fix(child)
            elif isinstance(value, list):
                for child in value:
                    fix(child)
        fix(events)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "wire.json"
            for include_old in (False, True):
                content = [{"type": "text", "text": json.dumps({"request_id": "req1", "robot_id": "r1"})}]
                for oid in (["obs0", "obs1"] if include_old else ["obs1"]):
                    content.append({"type": "image_url", "image_url": {
                        "url": "data:image/jpeg;base64," + base64.b64encode(raw_images[oid]).decode()}})
                path.write_text(json.dumps({"messages": [{"role": "user", "content": content}]}))
                next(e for e in events if e["event_id"] == "decision1")["payload"]["artifacts"] = {
                    ".request.json": {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}}
                args = dict(condition="structured", artifact_root=root, require_wire_images=True)
                if include_old:
                    report = validate_episode(self.specs[2], events, links, **args)
                    self.assertTrue(report["model_image_exposure_verified"])
                else:
                    with self.assertRaisesRegex(ScenarioError, "not an image"):
                        validate_episode(self.specs[2], events, links, **args)

    def test_synthetic_evidence_cannot_pass_live_gate(self):
        evidence = {"schema_version": READINESS_SCHEMA, "evidence_kind": "fixture", "source_sha": "9" * 40,
                    "scenario_sha256": canonical_sha256(self.specs[0]), "components": {}, "pins": {},
                    "source_files": {}, "boundary_audit": {}, "physical_replays": [], "visual_witnesses": []}
        report = assess_readiness(self.specs[0], evidence, expected_source_sha="9" * 40,
                                  artifact_root=ROOT, source_root=ROOT)
        self.assertFalse(report["ready"])
        self.assertIn("synthetic", report["blockers"][0])

    def test_file_backed_gate_binds_every_source_and_rejects_mutation(self):
        # Temporary fake review records exercise integrity checks only. They are
        # never published as independent review evidence or live admissions.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def save(name, value):
                data = value if isinstance(value, bytes) else json.dumps(value, sort_keys=True).encode()
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
                return {"path": name, "sha256": hashlib.sha256(data).hexdigest()}

            spec = copy.deepcopy(self.specs[0])
            spec["execution"]["components"] = {key: "test-only" for key in spec["execution"]["components"]}
            spec["execution"]["pins"] = {key: "a" * 64 for key in spec["execution"]["pins"]}
            source_files = {name: save(name, b"offline synthetic test")["sha256"] for name in SOURCE_FILES}
            binding = {"source_sha": "9" * 40, "scenario_sha256": canonical_sha256(spec),
                       "source_files": source_files, **spec["execution"]}
            binding.pop("required_source_files")
            common = {"binding": binding, "verdict": "pass", "evidence_kind": "independent_review"}
            raw = save("raw.json", {"test_only": True})
            audit = save("audit.json", {**common, "scope": "offline", "independent_reviewer": "A2",
                                        "cases": {case: "pass" for case in AUDIT_CASES}, "artifacts": [raw]})
            replays = [save(kind + ".json", {**common, "scope": "physical", "kind": kind,
                       "model_calls": 0, "weld_enabled": False, "mission_complete": False,
                       "replay_goal_complete": True, "target_object_ids": ["box" if kind == "solo" else "beam"],
                       "artifacts": [raw]}) for kind in ("solo", "joint")]
            witness = {"event_id": spec["event"]["event_id"], "robot_id": "r1", "observation_id": "obs1",
                       "visibility": "public_top", "observed_at_s": 0.,
                       "frame": save("frame.jpg", b"\xff\xd8offline-test\xff\xd9")}
            witness["review"] = save("visual.json", {**common, "scope": "visual", "witness": copy.deepcopy(witness),
                "criterion_visible": True, "private_information_claim": False, "independent_reviewer": "A2"})
            evidence = {"schema_version": READINESS_SCHEMA, "evidence_kind": "independent_review", **binding,
                        "boundary_audit": audit, "physical_replays": replays, "visual_witnesses": [witness]}
            args = dict(expected_source_sha="9" * 40, artifact_root=root, source_root=root)
            self.assertTrue(assess_readiness(spec, evidence, **args)["ready"])
            (root / SOURCE_FILES[0]).write_text("changed after audit")
            report = assess_readiness(spec, evidence, **args)
            self.assertFalse(report["ready"])
            self.assertIn("hash mismatch", report["blockers"][0])


class IntegratedBoundaryRegressionTests(unittest.TestCase):
    """Positive protections of the actual 936bf821 B/C production seam."""

    def test_duplicate_joint_intent_never_creates_second_lease(self):
        port, _ = port_fixture()
        first = port.submit("r1", B_FIXTURE["pair_requests"]["r1"])
        self.assertEqual(first["status"], "PENDING")
        duplicate = copy.deepcopy(B_FIXTURE["pair_requests"]["r1"])
        duplicate["request_id"] += "-again"
        self.assertEqual(port.submit("r1", duplicate)["status"], "REJECTED")
        accepted = port.submit("r2", B_FIXTURE["pair_requests"]["r2"])
        self.assertEqual(accepted["status"], "ACCEPTED")
        self.assertEqual(len(port.local_status("r1")["active"]), 1)
        port.close(0.)

    def test_expired_or_cancelled_lease_cannot_execute_late_command(self):
        for expiry in (False, True):
            with self.subTest(expiry=expiry):
                port, endpoints = port_fixture()
                lease = pair(port, expiry=1.)
                port.submit("r1", command("r1", lease))
                if expiry:
                    port.tick(1.)
                else:
                    port.submit("r1", stop("r1", lease))
                self.assertEqual(port.submit("r2", command("r2", lease))["status"], "REJECTED")
                port.tick(1.)
                self.assertTrue(all(not e.applied for e in endpoints.values()))
                self.assertEqual(port.submit("r1", B_FIXTURE["pair_requests"]["r1"])["status"], "REJECTED")
                port.close(1.)

    def test_evaluator_truth_changes_neither_actor_requests_nor_finish_claims(self):
        traces = []
        for success in (False, True):
            accesses = []
            def evaluator():
                accesses.append(True)
                return {"mission_complete": success, "contact": success}
            port, _ = port_fixture(evaluator)
            result, planners = run(port, ticks=1, callback=lambda req: {
                "action": {"kind": "finish", "claim": "looks done"}, "message": None})
            self.assertEqual(accesses, [])
            self.assertNotEqual(result["outcome"], "success")
            traces.append([p.requests for p in planners.values()])
        self.assertEqual(traces[0], traces[1])

    def test_stale_message_recipient_isolation_and_after_delivery_expiry(self):
        def sender(request):
            return {"action": None, "message": {
                "recipients": ["r2"], "content": "a peer claim", "ttl_s": 1.5}}
        port, _ = port_fixture()
        result, planners = run(port, callback=sender, condition="natural", ticks=3)
        self.assertEqual(planners["r3"].requests[-1]["memory"]["received_messages"], [])
        memory = planners["r2"].requests[-1]["memory"]
        self.assertTrue(memory["expired_peer_claims"])
        self.assertTrue(all(m["expires_at_s"] > 2. for m in memory["received_messages"]))
        self.assertTrue(any(e["event_type"] == "message_expired" for e in result["events"]))


class IntegrationBaseCharacterizationTests(unittest.TestCase):
    """GREEN HERE MEANS THE DOCUMENTED GAP EXISTS, NOT LIVE COMPLIANCE."""

    def setUp(self):
        frozen = json.loads((CATALOG.parent / "baseline_audit.json").read_text())
        for path, expected in frozen["source_files"].items():
            if hashlib.sha256((ROOT / path).read_bytes()).hexdigest() != expected:
                self.skipTest("historical 936bf821 characterization only; run final-path independent audit")

    def test_characterization_queued_command_outlives_actor_request_age(self):
        port, endpoints = port_fixture()
        lease = pair(port, expiry=10.)
        self.assertEqual(port.submit("r1", command("r1", lease))["status"], "QUEUED")
        port.tick(6.)
        self.assertEqual(port.submit("r2", command("r2", lease, now=6.))["status"], "QUEUED")
        port.tick(6.)
        # A2-01: six-second-old queued request executes despite max_request_age=5.
        self.assertEqual(len(endpoints["r1"].applied), 1)
        port.close(6.)

    def test_characterization_pending_intent_cannot_be_withdrawn(self):
        port, _ = port_fixture()
        port.submit("r1", B_FIXTURE["pair_requests"]["r1"])
        result = port.submit("r1", stop("r1", "no-lease-yet"))
        self.assertEqual(result["status"], "REJECTED")
        self.assertEqual(len(port.local_status("r1")["pending"]), 1)
        port.close(0.)

    def test_characterization_slow_first_planner_delays_other_actors(self):
        clock = [0.]
        def slow(request):
            clock[0] += 2.
            return {"action": None, "message": {"recipients": ["r2"], "content": "late", "ttl_s": 1.}}
        port, _ = port_fixture()
        result, _ = run(port, callback=slow, condition="natural", ticks=1, clock=lambda: clock[0])
        requests = [e for e in result["events"] if e["event_type"] == "planner_requested"]
        self.assertEqual([e["wall_time_s"] for e in requests], [0., 2., 2.])
        self.assertEqual(result["messages"]["r1"], 0)  # late reply itself is rejected

    def test_characterization_actual_command_audit_drops_decision_id(self):
        port, _ = port_fixture(lambda: {})
        lease = pair(port)
        for rid in ("r1", "r2"):
            port.submit(rid, command(rid, lease))
        port.tick(0.)
        commands = [e for e in port.evaluation_snapshot()["coordination_audit"] if e["event"] == "LOCAL_COMMAND"]
        self.assertEqual(len(commands), 2)
        self.assertTrue(all("decision_id" not in e and "observation_id" not in e for e in commands))
        port.close(0.)


class EnvironmentExposureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from sim.act_map_suite import load_suite
        cls.cases = load_suite()[1]

    def provenance(self, map_id="train-open-1"):
        from sim.act_map_suite import layout_digest
        from sim.research_dispatch_arena import digest
        case = next(c for c in self.cases if c["id"] == map_id)
        ref = {"map_id": map_id, "map_sha256": digest(case["map"]),
               "layout_sha256": layout_digest(case["map"])}
        records = [
            {"id": "episode", "kind": "episode", "parents": [], "map_refs": [ref],
             "declared_splits": [case["split"]], "origin": "known"},
            {"id": "frame", "kind": "frame", "parents": ["episode"], "map_refs": [],
             "declared_splits": [case["split"]], "origin": "known"},
            {"id": "augmented", "kind": "augmentation", "parents": ["frame"], "map_refs": [],
             "declared_splits": [case["split"]], "origin": "known"},
            {"id": "model", "kind": "foundation_model", "parents": [], "map_refs": [],
             "declared_splits": [], "origin": "unknown"},
            {"id": "prompt", "kind": "prompt", "parents": ["augmented", "model"], "map_refs": [],
             "declared_splits": [case["split"]], "origin": "known"}]
        return {"schema_version": "rgb-map-exposure.v1", "records": records, "freeze_order": 10,
                "exposures": [], "final_test_ids": [c["id"] for c in self.cases if c["split"] in {"test_a", "test_b"}]}

    def test_existing_22_map_validator_and_unknown_pretraining_reused(self):
        report = audit_exposure(self.cases, self.provenance(), [])
        self.assertTrue(report["valid"])
        self.assertEqual(len(report["effective_splits"]), 22)
        self.assertEqual(report["unknown_origins"], ["model"])
        self.assertFalse(report["uncontaminated_pretraining_claim"])
        self.assertEqual(report["lineage_maps"]["prompt"], ["train-open-1"])

    def test_frame_augmentation_cannot_escape_parent_split(self):
        provenance = self.provenance("test-b-double-1")
        provenance["records"][2]["declared_splits"] = ["train"]
        report = audit_exposure(self.cases, provenance, [])
        self.assertFalse(report["valid"])
        self.assertIn("inherit", report["blockers"][0])

    def test_geometry_qa_is_distinct_from_policy_diagnosis_or_prompt_exposure(self):
        for use, valid in (("geometry_qa", True), ("policy_diagnosis", False), ("prompt_tuning", False)):
            provenance = self.provenance("test-a-open")
            provenance["exposures"] = [{"record_id": "prompt", "use": use, "order": 11,
                                        "artifact_sha256": "a" * 64}]
            report = audit_exposure(self.cases, provenance, [])
            self.assertEqual(report["valid"], valid)
            self.assertEqual(report["effective_splits"]["test-a-open"], "test_a" if valid else "regression")

    def test_final_result_then_tuning_demotes_test_even_when_history_unsorted(self):
        provenance = self.provenance("test-a-open")
        provenance["exposures"] = [
            {"record_id": "model", "use": "prompt_tuning", "order": 13, "artifact_sha256": "a" * 64},
            {"record_id": "episode", "use": "final_evaluation", "order": 12, "artifact_sha256": "b" * 64}]
        report = audit_exposure(self.cases, provenance, [])
        self.assertFalse(report["valid"])
        self.assertEqual(report["effective_splits"]["test-a-open"], "regression")

    def test_no_posthoc_failed_test_exclusion_or_scenario_relabel(self):
        provenance = self.provenance()
        provenance["final_test_ids"].pop()
        self.assertFalse(audit_exposure(self.cases, provenance, [])["valid"])
        ref = self.provenance()["records"][0]["map_refs"][0]
        assignment = {"scenario_id": "recovery", "parent_map_id": ref["map_id"],
                      "map_sha256": ref["map_sha256"], "layout_sha256": ref["layout_sha256"], "split": "test_a"}
        self.assertFalse(audit_exposure(self.cases, self.provenance(), [assignment])["valid"])

    def test_cycles_hash_alias_and_repeated_layout_fail(self):
        provenance = self.provenance()
        provenance["records"][0]["parents"] = ["augmented"]
        self.assertFalse(audit_exposure(self.cases, provenance, [])["valid"])
        provenance = self.provenance()
        provenance["records"][0]["map_refs"][0]["layout_sha256"] = "0" * 64
        self.assertFalse(audit_exposure(self.cases, provenance, [])["valid"])
        cases = copy.deepcopy(self.cases)
        cases[1]["map"] = copy.deepcopy(cases[0]["map"])
        cases[1]["map"]["map_id"] = "renamed"
        self.assertFalse(audit_exposure(cases, self.provenance(), [])["valid"])

    def training_plan(self):
        common = {"episodes": 10, "transitions": 1000, "frames": 2000, "updates": 100,
                  "batch_size": 4, "architecture_sha256": "a" * 64, "input_sha256": "b" * 64,
                  "initial_checkpoint_sha256": "c" * 64, "device": "same GPU", "precision": "fp32"}
        return {"schema_version": "rgb-training-comparison.v1", "arms": [
            {"id": "T1", "map_ids": ["train-open-1"], **common},
            {"id": "T2", "map_ids": ["train-open-1", "train-door-1", "train-corner-1"], **common}],
            "fixed_test_ids": self.provenance()["final_test_ids"], "training_seeds": [17, 18],
            "selection_rule_sha256": "d" * 64}

    def test_t1_t2_same_amount_budget_and_test_t3_is_separate(self):
        report = audit_exposure(self.cases, self.provenance(), [])
        plan = self.training_plan()
        validate_training_comparison(plan, report)
        for field in ("episodes", "transitions", "frames", "updates", "device"):
            changed = copy.deepcopy(plan)
            changed["arms"][1][field] = "different" if field == "device" else 999
            with self.subTest(field=field), self.assertRaises(ScenarioError):
                validate_training_comparison(changed, report)
        t3 = dict(plan["arms"][1], id="T3", episodes=20, transitions=2000, frames=4000, updates=200)
        plan["arms"].append(t3)
        validate_training_comparison(plan, report)
        plan["fixed_test_ids"].pop()
        with self.assertRaises(ScenarioError):
            validate_training_comparison(plan, report)

    def test_static_preflight_admits_only_diagnostic_scope_not_live_success(self):
        from sim.act_map_suite import layout_digest
        from sim.research_dispatch_arena import digest
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def save(name, value):
                path = root / name
                path.write_text(json.dumps(value))
                return {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            raw = save("raw.json", {"test_only": True})
            case = next(c for c in self.cases if c["id"] == "dev-open")
            maps = [{"map_id": case["id"], "map_sha256": digest(case["map"]),
                     "layout_sha256": layout_digest(case["map"]), "scene": raw, "camera": raw,
                     "reset": raw, "evaluator_config": raw, "capability": "transport"}]
            record = {"schema_version": "rgb-environment-readiness.v1", "source_sha": "9" * 40,
                      "backend_id": "fixture-only", "scope": "static_reset_preflight", "maps": maps,
                      "checks": {name: save(name + ".json", {"check_id": name, "verdict": "pass",
                          "source_sha": "9" * 40, "backend_id": "fixture-only",
                          "maps_sha256": canonical_sha256(maps), "evidence_kind": "static_contract_check",
                          "artifacts": [raw]}) for name in ENVIRONMENT_PREFLIGHT_CHECKS}}
            args = dict(artifact_root=root, expected_source_sha="9" * 40, expected_backend_id="fixture-only")
            result = assess_environment_readiness(record, self.cases, purpose="physical_replay", **args)
            self.assertTrue(result["ready"])
            self.assertFalse(result["physical_controls_verified"])
            self.assertFalse(result["heldout_policy_performance_verified"])
            self.assertFalse(assess_environment_readiness(record, self.cases, **args)["ready"])
            # Legacy dispatch is a separate diagnostic registry, not suite dev-open.
            from sim.research_dispatch_arena import authored_map
            legacy = authored_map("open")
            record["maps"][0].update(map_id="dispatch_open", map_sha256=digest(legacy),
                                     layout_sha256=layout_digest(legacy))
            for name in ENVIRONMENT_PREFLIGHT_CHECKS:
                record["checks"][name] = save(name + ".json", {"check_id": name, "verdict": "pass",
                    "source_sha": "9" * 40, "backend_id": "fixture-only",
                    "maps_sha256": canonical_sha256(record["maps"]), "evidence_kind": "static_contract_check", "artifacts": [raw]})
            self.assertFalse(assess_environment_readiness(record, self.cases, purpose="physical_replay", **args)["ready"])
            self.assertTrue(assess_environment_readiness(record, self.cases, purpose="physical_replay",
                                                        allow_legacy_dispatch_open=True, **args)["ready"])
            self.assertFalse(assess_environment_readiness(record, self.cases, allow_legacy_dispatch_open=True, **args)["ready"])
            (root / "raw.json").write_text("tampered")
            self.assertFalse(assess_environment_readiness(record, self.cases, purpose="physical_replay",
                                                         allow_legacy_dispatch_open=True, **args)["ready"])


if __name__ == "__main__":
    unittest.main()
