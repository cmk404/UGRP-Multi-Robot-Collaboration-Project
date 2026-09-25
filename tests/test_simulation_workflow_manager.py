"""Bounded process and provenance checks for the canonical simulation CLI."""
from __future__ import annotations

import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from sim import workflow_manager as wm

PROJECT = Path(__file__).resolve().parents[1]


class WorkflowManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "scripts").mkdir()
        (self.root / "scripts" / "__init__.py").write_text("")
        (self.root / "configs").mkdir()
        (self.root / "scripts" / "fixture.py").write_text('''\
import argparse, json, pathlib, subprocess, sys, time
p=argparse.ArgumentParser();p.add_argument('--output', required=True);p.add_argument('--fail', action='store_true');p.add_argument('--sleep', action='store_true');p.add_argument('--spawn', action='store_true');p.add_argument('--ask', action='store_true');a=p.parse_args()
out=pathlib.Path(a.output);out.mkdir(exist_ok=False)
if a.ask:
    print('READY>', end='', flush=True)
    input()
if a.spawn:
    child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
    (out/'grandchild.pid').write_text(str(child.pid))
if a.sleep:time.sleep(30)
(out/'result.json').write_text(json.dumps({'physical_success':False,'protocol_complete':True}))
print('fixture executed', flush=True)
raise SystemExit(3 if a.fail else 0)
''')
        self.catalog_path = self.root / wm.CATALOG
        self.catalog_path.write_text(json.dumps({"schema": "ugrp.local_workflow_catalog.v1", "workflows": [{
            "id": "fixture", "version": "1.0.0", "entry": "scripts/fixture.py", "runner": "scripts.fixture",
            "output_flag": "--output", "output_kind": "directory", "required_inputs": [],
            "side_effect": "test", "adapter": "legacy_cli"},
            {"id": "local", "version": "1.0.0", "entry": "scripts/fixture.py", "runner": "scripts.sim_cli",
             "output_flag": "--output", "output_kind": "directory", "required_inputs": {"run": ["CONFIG"], "console": ["CONFIG"]},
             "side_effect": "test"},
            {"id": "dispatch", "version": "1.0.0", "entry": "scripts/fixture.py", "runner": "scripts.sim_cli",
             "output_flag": "--output", "output_kind": "directory", "required_inputs": [],
             "side_effect": "test"}]}, indent=2))

    def test_plan_is_read_only_and_resolves_default_artifact(self):
        with mock.patch.object(subprocess, "Popen", side_effect=AssertionError("must not launch")):
            row = wm.plan(self.root, "fixture", [])
        self.assertEqual(row["output"], "<record>/artifacts")
        self.assertEqual(row["command"][-2:], ["--output", "<record>/artifacts"])
        self.assertFalse(row["execution_started"])
        self.assertFalse((self.root / "outputs").exists())

    def test_run_records_source_inputs_exit_and_physical_evidence(self):
        source = self.root / "input.json"
        source.write_text('{"x":1}')
        record = wm.run_workflow(self.root, "fixture", [], inputs=[source])
        data = json.loads((record / "manifest.json").read_text())
        self.assertEqual(data["exit_code"], 0)
        self.assertEqual(data["status"], "process_completed")
        self.assertIs(data["physical_success"], False)
        self.assertIs(data["reported_outcomes"]["protocol_complete"]["value"], True)
        self.assertEqual(data["inputs_before"], data["inputs_after"])
        self.assertIs(data["inputs_changed_during_run"], False)
        self.assertFalse(data["source_changed_during_run"])
        self.assertIn("result.json", [r["path"] for r in data["output_receipt"]["files"]])
        self.assertIn("fixture executed", (record / "console.log").read_text())
        (self.root / "scripts" / "untracked.py").write_text("value=1\n")
        self.assertNotEqual(data["source"]["execution_tree"]["sha256"], wm.source_fingerprint(self.root)["sha256"])

    def test_custom_record_is_queryable_and_never_overwritten(self):
        record = self.root / "custom-record"
        wm.run_workflow(self.root, "fixture", [], record_dir=record)
        self.assertTrue(any(p.parent.resolve() == record.resolve() for p in wm._records(self.root)))
        with self.assertRaises(FileExistsError):
            wm.run_workflow(self.root, "fixture", [], record_dir=record)
        self.assertTrue((record / "manifest.json").is_file())

    def test_output_collision_and_duplicate_flag_fail_before_record(self):
        output = self.root / "prior"
        output.mkdir()
        with self.assertRaises(FileExistsError):
            wm.run_workflow(self.root, "fixture", ["--output", str(output)])
        with self.assertRaises(ValueError):
            wm.run_workflow(self.root, "fixture", ["--output", str(self.root / 'new'), "--output", str(output)])
        self.assertFalse((self.root / "outputs").exists())

    def test_timeout_terminates_child(self):
        record = wm.run_workflow(self.root, "fixture", ["--sleep"], timeout=0.2)
        row = json.loads((record / "manifest.json").read_text())
        self.assertEqual(row["status"], "timeout")
        self.assertEqual(row["exit_code"], 124)
        self.assertIsNone(row["physical_success"])
        self.assertIsNotNone(row["child_pid"])

    def test_parent_exit_cleans_background_child(self):
        record = wm.run_workflow(self.root, "fixture", ["--spawn"])
        pid = int((record / "artifacts" / "grandchild.pid").read_text())
        result = subprocess.run(["ps", "-o", "stat=", "-p", str(pid)], text=True, capture_output=True)
        self.assertTrue(result.returncode != 0 or not result.stdout.strip() or result.stdout.strip().startswith("Z"), result.stdout)

    def test_interactive_prompt_is_tee_d_before_child_waits_for_input(self):
        script = ("from pathlib import Path; from sim.workflow_manager import run_workflow; "
                  f"run_workflow(Path({str(self.root)!r}), 'fixture', ['--ask'], timeout=4)")
        process = subprocess.Popen([sys.executable, "-c", script], cwd=PROJECT, stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            output = b""
            while b"READY>" not in output:
                ready, _, _ = select.select([process.stdout], [], [], 3)
                self.assertTrue(ready, output)
                output += os.read(process.stdout.fileno(), 4096)
            process.stdin.write(b"okay\n")
            process.stdin.flush()
            self.assertEqual(process.wait(timeout=5), 0)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            process.stdin.close()
            process.stdout.close()

    def test_source_fingerprint_includes_calibration_requirements_and_sparse_absence(self):
        (self.root / "calibration").mkdir()
        missing = self.root / "calibration" / "important.json"
        missing.write_text('{}')
        requirements = self.root / "requirements-sim.txt"
        requirements.write_text('mujoco==1\n')
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        subprocess.run(["git", "-C", str(self.root), "add", "calibration/important.json"], check=True)
        missing.unlink()
        row = wm.source_fingerprint(self.root)
        self.assertIn("calibration/important.json", row["missing_tracked_files"])
        self.assertIn("requirements-sim.txt", [entry["path"] for entry in row["files"]])

    def test_redacts_secret_argv_and_tracks_input_hash(self):
        safe = wm.redact_argv(["--token", "SENSITIVE", "--api-key=TOPSECRET", "--max-input-tokens", "1000",
                               "--url=wss://user:pass@example.test/ws?access_token=SECRET&key=APIKEY&sig=SIGNATURE&mode=run#session=FRAGMENTSECRET",
                               "--task", "go"])
        self.assertNotIn("SENSITIVE", str(safe))
        self.assertNotIn("TOPSECRET", str(safe))
        self.assertNotIn("user:pass", str(safe))
        self.assertNotIn("access_token=SECRET", str(safe))
        self.assertNotIn("APIKEY", str(safe))
        self.assertNotIn("SIGNATURE", str(safe))
        self.assertNotIn("mode=run", str(safe))
        self.assertNotIn("FRAGMENTSECRET", str(safe))
        self.assertIn("--url=wss://[REDACTED]@example.test/ws?", safe[5])
        self.assertIn("key=", safe[5])
        self.assertIn("sig=", safe[5])
        self.assertTrue(safe[5].endswith("#[REDACTED]"))
        self.assertIn("1000", safe)
        self.assertEqual(safe[-1], "go")

    def test_worker_plan_redacts_entire_url_query(self):
        url = "wss://example.test/ws?key=APIKEY&sig=SIGNATURE&mode=run"
        with mock.patch.dict(os.environ, {"UGRP_SIM_TOKEN": "TOKEN"}):
            planned = wm.plan(PROJECT, "worker", ["--url", url])
        for field in ("argv", "command"):
            rendered = str(planned[field])
            self.assertNotIn("APIKEY", rendered)
            self.assertNotIn("SIGNATURE", rendered)
            self.assertNotIn("mode=run", rendered)
            self.assertIn("wss://example.test/ws?", rendered)

    def test_inprocess_finalizes_on_exception(self):
        def fail(output):
            output.mkdir()
            raise RuntimeError("test failure")
        with self.assertRaisesRegex(RuntimeError, "test failure"):
            wm.run_inprocess(self.root, "fixture", ["run"], fail, output=None)
        manifests = wm._records(self.root)
        self.assertEqual(len(manifests), 1)
        row = json.loads(manifests[0].read_text())
        self.assertEqual(row["status"], "launcher_failed")
        self.assertEqual(row["exit_code"], 2)

    def test_catalog_has_twenty_four_selectable_workflows_and_distinct_adapters(self):
        data, digest = wm.catalog(PROJECT)
        self.assertEqual(len(data["workflows"]), 24)
        self.assertEqual(len(digest), 64)
        self.assertEqual(next(r for r in data["workflows"] if r["id"] == "dispatch-skills")["runner"], "scripts.run_dispatch_e2e")
        self.assertEqual(next(r for r in data["workflows"] if r["id"] == "communication")["output_kind"]["prepare"], "file")
        self.assertEqual(next(r for r in data["workflows"] if r["id"] == "act-training")["runner"], "scripts.train_carry_act")

    def test_every_catalog_workflow_has_a_read_only_explicit_plan(self):
        data, _ = wm.catalog(PROJECT)
        source = self.root / "source.json"
        source.write_text('{}')
        model = self.root / "model"
        model.mkdir()
        (model / "weights.bin").write_bytes(b"fixture")
        samples = {
            "local": ["run", str(PROJECT / "configs/simulation/drive.json"), "--headless"], "dispatch": ["--headless"],
            "dispatch-skills": ["--plan-replay", str(source), "--grasp-model-dir", str(model), "--stage-model-dir", str(model)],
            "communication": ["prepare", "--protocol", str(source)],
            "multi-object": ["--model", "fixture"],
            "navigation": ["--map-file", str(source), "--case-json", "{}"],
            "pair-navigation": ["--map", str(source), "--grasp-model-dir", str(model)],
            "camera-pair": [], "rgb-traffic": ["--scenario", "crossing"],
            "act": ["--act-python", str(source), "--mjpython", str(source), "--grasp", str(model), "--stages", str(model)],
            "teacher": ["--grasp-model-dir", str(model), "--cases-json", str(source)],
            "act-map-suite": ["--spec", str(PROJECT / "maps/act_generalization/suite_v1.json")],
            "act-training": ["--dataset", str(model)],
            "act-input-training": ["--dataset", str(source), "--size", "128", "--history", "1",
                                   "--steps", "100", "--device", "cpu", "--seed", "18"],
            "act-input-finalization": ["--failed-run", str(model), "--dataset", str(source),
                                       "--source-freeze", str(source),
                                       "--expected-manifest-sha256", "0"*64,
                                       "--expected-report-sha256", "1"*64,
                                       "--expected-checkpoint-sha256", "2"*64,
                                       "--expected-source-freeze-sha256", "3"*64],
            "jev": ["--execute", "--mjpython", str(source)], "stage-sync": [],
            "physical": [str(model)], "worker": ["--url", "ws://localhost/fixture"],
            "tensorboard": ["--source", str(model)],
            "communication-study": ["prepare", "--config", str(source)],
            "communication-cloud-submit": ["--evidence-root", str(model), "--inventory", str(source)],
            "zone-dispatch": ["--mode", "fixture", "--coordination", "dynamic"],
            "zone-color-eval": ["render", "--split", "dev"],
        }
        with mock.patch.dict(os.environ, {"UGRP_SIM_TOKEN": "secret"}), \
             mock.patch.object(subprocess, "Popen", side_effect=AssertionError("planning launched a child")):
            plans = {row["id"]: wm.plan(PROJECT, row["id"], samples[row["id"]]) for row in data["workflows"]}
        self.assertEqual(set(plans), set(samples))
        self.assertEqual(plans["physical"]["command"][-1], "<record>/artifacts")
        self.assertEqual(plans["communication"]["output"], "<record>/artifacts.json")
        self.assertEqual(plans["dispatch-skills"]["command"][3:5], ["--executor", "skills"])
        self.assertEqual(plans["act-map-suite"]["command"][-2:], ["--output", "<record>/artifacts"])
        self.assertEqual(plans["act-input-training"]["command"][-2:], ["--out", "<record>/artifacts"])
        self.assertEqual(plans["act-input-finalization"]["command"][-2:], ["--out", "<record>/artifacts"])
        self.assertEqual(plans["act-map-suite"]["inputs"][0]["path"], str(PROJECT / "maps/act_generalization/suite_v1.json"))
        self.assertEqual(plans["act-input-training"]["inputs"][0]["path"], str(source.resolve()))
        self.assertEqual({item['path'] for item in plans["act-input-finalization"]["inputs"]},
                         {str(source.resolve()), str(model.resolve())})
        self.assertEqual(plans["zone-dispatch"]["command"][-2:], ["--output", "<record>/artifacts"])
        self.assertEqual(plans["zone-color-eval"]["command"][-2:], ["--output", "<record>/artifacts"])
        self.assertTrue(all(not plan["execution_started"] for plan in plans.values()))

    def test_act_workflows_require_explicit_suite_and_training_shape(self):
        with self.assertRaisesRegex(ValueError, "act-map-suite requires --spec"):
            wm.plan(PROJECT, "act-map-suite", [])
        with self.assertRaisesRegex(ValueError, "act-input-training requires --dataset"):
            wm.plan(PROJECT, "act-input-training", ["--size", "128", "--history", "1"])
        with self.assertRaisesRegex(ValueError, "act-input-training requires --size"):
            wm.plan(PROJECT, "act-input-training", ["--dataset", str(PROJECT / "maps/act_generalization/suite_v1.json"), "--history", "1"])
        with self.assertRaisesRegex(ValueError, "act-input-training requires --history"):
            wm.plan(PROJECT, "act-input-training", ["--dataset", str(PROJECT / "maps/act_generalization/suite_v1.json"), "--size", "128"])
        args = ["--dataset", str(PROJECT / "maps/act_generalization/suite_v1.json"),
                "--size", "128", "--history", "1", "--steps", "100", "--device", "cpu", "--seed", "18"]
        for flag in ("--steps", "--device", "--seed"):
            missing = args[:]
            del missing[missing.index(flag):missing.index(flag) + 2]
            with self.assertRaisesRegex(ValueError, f"act-input-training requires {flag}"):
                wm.plan(PROJECT, "act-input-training", missing)

    def test_act_finalization_requires_original_assets_and_pinned_hashes(self):
        with tempfile.TemporaryDirectory() as root:
            failed = Path(root) / "failed"
            failed.mkdir()
            dataset = Path(root) / "data.json"
            dataset.write_text('{}')
            freeze = Path(root) / "freeze.json"
            freeze.write_text('{}')
            args = ["--failed-run", str(failed), "--dataset", str(dataset),
                    "--source-freeze", str(freeze),
                    "--expected-manifest-sha256", "0"*64,
                    "--expected-report-sha256", "1"*64,
                    "--expected-checkpoint-sha256", "2"*64,
                    "--expected-source-freeze-sha256", "3"*64]
            for flag in args[::2]:
                missing = args[:]
                del missing[missing.index(flag):missing.index(flag) + 2]
                with self.assertRaisesRegex(ValueError, f"act-input-finalization requires {flag}"):
                    wm.plan(PROJECT, "act-input-finalization", missing)
            planned = wm.plan(PROJECT, "act-input-finalization", args)
            self.assertEqual({entry['path'] for entry in planned['inputs']},
                             {str(failed.resolve()), str(dataset.resolve()), str(freeze.resolve())})

    def test_physical_rejects_copy_into_own_trace_and_worker_token_argv(self):
        row = next(r for r in wm.catalog(PROJECT)[0]["workflows"] if r["id"] == "worker")
        with self.assertRaises(ValueError):
            wm._validate_args(row, ["--url", "ws://localhost", "--token", "SENSITIVE"])
        with self.assertRaises(ValueError):
            wm.plan(PROJECT, "physical", [str(PROJECT)])

    def test_dispatch_skills_cannot_switch_to_raw(self):
        row = next(r for r in wm.catalog(PROJECT)[0]["workflows"] if r["id"] == "dispatch-skills")
        with self.assertRaises(ValueError):
            wm._validate_args(row, ["--grasp-model-dir", "/tmp/g", "--stage-model-dir", "/tmp/s", "--executor", "raw"])
        with self.assertRaises(ValueError):
            wm._validate_args(row, ["--grasp-model-dir", "/tmp/g", "--stage-model-dir", "/tmp/s"])
        with self.assertRaises(ValueError):
            wm._validate_args(row, ["--plan-replay", "/tmp/plan.json", "--grasp-model-dir", "/tmp/g",
                                    "--stage-model-dir", "/tmp/s", "--live-replan"])

    def test_standard_run_and_dispatch_record_selected_workflow(self):
        from scripts import sim_cli, sim_dispatch
        config = self.root / "local.json"
        config.write_text('{"version":1}')
        archive = self.root / "experiments/dispatch-skill-integration-20260917/models.zip"
        archive.parent.mkdir(parents=True)
        archive.write_bytes(b"fixture model bundle")
        def fake_run(_config, args):
            self.assertEqual(_config["scene"]["layout"], "navigation/s-bends")
            args.output.mkdir()
            (args.output / "result.json").write_text('{"protocol_complete":false}')
            return 0
        def fake_dispatch(args):
            output = Path(args[args.index("--output") + 1])
            output.mkdir()
            (output / "result.json").write_text('{"physical_success":false}')
            return 0
        with mock.patch.object(sim_cli, "ROOT", self.root), mock.patch.object(sim_cli, "run", fake_run), \
             mock.patch.object(sim_dispatch, "main", fake_dispatch):
            self.assertEqual(sim_cli.main(["run", str(config), "--headless", "--scene", "navigation/s-bends"]), 0)
            self.assertEqual(sim_cli.main(["dispatch", "--headless"]), 0)
        rows = [json.loads(p.read_text()) for p in wm._records(self.root)]
        self.assertEqual({r["workflow_id"] for r in rows}, {"local", "dispatch"})
        self.assertTrue(all(r["status"] == "process_completed" for r in rows))
        self.assertTrue(all(r["physical_success"] is not True for r in rows))


if __name__ == "__main__":
    unittest.main()
