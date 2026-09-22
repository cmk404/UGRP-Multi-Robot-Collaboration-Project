"""Offline tests only; no physical backend or network provider is constructed."""
import copy
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from harness import rgb_communication_study as study


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))
    return {"path": path.name, "sha256": study.digest_file(path)}


@pytest.fixture
def source(tmp_path):
    root = tmp_path / "source"
    root.mkdir()
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    put(root / "harness/config.json", {"version": 1})
    subprocess.run(["git", "add", "."], cwd=root, check=True)
    subprocess.run(["git", "-c", "user.name=Test", "-c", "user.email=test@example.invalid",
                    "commit", "-qm", "offline source"], cwd=root, check=True)
    return root


def config():
    return {"stage": "physical_replay", "claim_scope": "development_connection_smoke",
            "submitter": "D2", "backend_id": "test-only", "backend": {},
            "budgets": dict(study.PHYSICAL_BUDGETS),
            "runtime_limits": {"max_ticks": 3600, "tick_period_s": .05, "wall_timeout_s": 595,
                               "max_concurrent_requests": 3},
            "trials": [{"run_id": kind, "replay_kind": kind, "condition": "none", "seed": 11,
                        "scenario_id": kind, "common_task": {}, "stratum": "development",
                        "map_group_sha256": "a" * 64} for kind in ("solo", "joint")]}


def test_manifest_pins_source_configuration_and_all_dependencies(source):
    value = study.prepare_manifest(config(), source)
    assert value["planned_denominator"] == 2
    assert value["source_files"] == {"harness/config.json": study.digest_file(source / "harness/config.json")}
    assert value["config_sha256"] == study.digest_json(value["config"])
    put(source / "uncommitted.json", {})
    with pytest.raises(study.ContractError, match="commit"):
        study.prepare_manifest(config(), source)


@pytest.mark.parametrize("run_id", ["../escape", "/tmp/run", "", ["not-a-string"]])
def test_path_safe_unique_trial_ids(source, run_id):
    value = config()
    value["trials"][0]["run_id"] = run_id
    with pytest.raises(study.ContractError, match="trial IDs"):
        study.prepare_manifest(value, source)


def test_checked_reference_rejects_tampering_escape_and_symlink(tmp_path):
    root = tmp_path / "evidence"
    ref = put(root / "file.json", {"a": 1})
    assert study.checked_reference(root, ref) == (root / "file.json").resolve()
    (root / "link.json").symlink_to(root / "file.json")
    for bad in ({**ref, "path": "../file.json"}, {**ref, "path": "link.json"},
                {**ref, "sha256": "0" * 64}):
        with pytest.raises(study.ContractError):
            study.checked_reference(root, bad)


def test_gate_never_uses_fixture_or_boolean_as_live_readiness(source):
    value = study.prepare_manifest(config(), source)
    value["config"]["readiness"] = True
    report = study.preflight(value, root=source, evidence_root=source)
    assert report["ready"] is False
    assert "configuration_hash_mismatch" in report["blockers"]
    assert any(b.startswith("environment_evidence_invalid") for b in report["blockers"])
    assert any(b.startswith("backend_unavailable") for b in report["blockers"])


@pytest.mark.parametrize("key,value", [("model_calls", 1), ("wall_time_s", 601),
                                      ("sim_time_s", float("inf")), ("input_tokens", True)])
def test_replay_rejects_live_calls_and_budget_expansion(source, key, value):
    conf = config()
    conf["budgets"][key] = value
    if value == float("inf"):
        with pytest.raises(ValueError):
            study.prepare_manifest(conf, source)
        return
    manifest = study.prepare_manifest(conf, source)
    report = study.preflight(manifest, root=source, evidence_root=source)
    assert f"physical_budget_invalid:{key}" in report["blockers"]


def test_provider_json_cannot_register_or_assert_verified_bounds():
    for value in ({"model": "fixed", "input_token_bound": 1},
                  {"model": "fixed", "input_bound_evidence": "trust-me"}):
        with pytest.raises(study.ContractError):
            study.provider_settings(value)


def test_replay_script_is_explicit_and_does_not_read_referee():
    planner = study.ReplayDecision([{"kind": "task_request", "task_id": "fixed-diagnostic"}])
    assert planner({})["action"]["task_id"] == "fixed-diagnostic"
    assert planner({}) == {"action": {"kind": "wait"}, "message": None}


def test_subprocess_success_timeout_and_exclusive_evidence(tmp_path):
    log = tmp_path / "first.log"
    result = study.bounded_process([sys.executable, "-c", "print('offline')"],
                                  cwd=tmp_path, log_path=log, timeout_s=2)
    assert result["exit_code"] == 0 and not result["timed_out"]
    assert log.read_text().strip() == "offline"
    with pytest.raises(FileExistsError):
        study.bounded_process([sys.executable, "-c", "pass"],
                              cwd=tmp_path, log_path=log, timeout_s=2)
    timed = study.bounded_process([sys.executable, "-c", "import time; time.sleep(20)"],
                                 cwd=tmp_path, log_path=tmp_path / "timeout.log", timeout_s=.05)
    assert timed["exit_code"] == 124 and timed["timed_out"]
    assert timed["wall_time_s"] < 6


def test_strata_include_unrun_and_dont_use_failed_speed_or_guess_cause():
    trials = config()["trials"]
    rows = [{"run_id": "solo", "outcome": "failure", "result": {
        "metrics": {"wall_time_s": .1}, "evaluator_verdict": {
            "failure_category": "controller", "recovery": {"reached": False}}}}]
    result = study.stratified_report(rows, trials)[0]
    assert result["planned_n"] == 2
    assert result["outcomes"] == {"failure": 1, "unrun": 1}
    assert result["successful_wall_time_s"] == []
    assert result["failure_categories"] == {"controller": 1, "unclassified": 1}
    assert result["event_measured_n"] == 1 and result["event_reached_n"] == 0


def test_no_output_or_process_when_preflight_blocks(source, tmp_path, monkeypatch):
    manifest = study.prepare_manifest(config(), source)
    called = []
    monkeypatch.setattr(study, "bounded_process", lambda *a, **k: called.append(True))
    with pytest.raises(study.ContractError, match="study blocked"):
        study.run_study(manifest, root=source, evidence_root=source, output=tmp_path / "output")
    assert not called and not (tmp_path / "output").exists()


def test_source_and_config_drift_block_even_separate_trial_entry(source, tmp_path, monkeypatch):
    manifest = study.prepare_manifest(config(), source)
    monkeypatch.setattr(study, "preflight", lambda *a, **k: {"ready": True})
    manifest["config"]["trials"].reverse()
    with pytest.raises(study.ContractError, match="configuration drift"):
        study.run_trial(manifest, run_id="solo", root=source, evidence_root=source, output=tmp_path)
