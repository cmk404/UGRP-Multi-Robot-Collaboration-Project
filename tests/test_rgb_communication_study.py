"""Offline tests only; no physical backend or network provider is constructed."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from harness import rgb_communication_study as study


def ready_preflight(*_args, **_kwargs):
    return {"ready": True, "checked": [{"backend": {"execution_bundle_sha256": "test-hash"}}]}


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
            "submitter": "D2", "backend_id": "test-only", "backend": {"execution_bundle_id": "test-only"},
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


def test_manifest_requires_explicit_execution_bundle_selection(source):
    conf = config()
    conf["backend"].pop("execution_bundle_id")
    with pytest.raises(study.ContractError, match="execution_bundle_id required"):
        study.prepare_manifest(conf, source)


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


def test_local_asset_binding_rechecks_files_and_complete_backend_inputs(tmp_path):
    asset = tmp_path / "saved-model.json"
    asset.write_text('{"test_only": true}')
    hashes = {str(asset): study.digest_file(asset)}
    catalog = {"schema": "rgb-local-assets.v1", "mode": "read_only_reference", "files": hashes}
    descriptor = {"input_hashes": dict(hashes)}
    study.verify_local_assets(catalog, descriptor)
    for bad in ({**catalog, "mode": "copy"}, {**catalog, "files": {}},
                {**catalog, "files": {"relative.json": "0" * 64}}):
        with pytest.raises(study.ContractError):
            study.verify_local_assets(bad, descriptor)
    with pytest.raises(study.ContractError, match="every backend input"):
        study.verify_local_assets(catalog, {"input_hashes": {**hashes, "/missing": "0" * 64}})
    asset.write_text('{"test_only": "changed"}')
    with pytest.raises(study.ContractError, match="changed"):
        study.verify_local_assets(catalog, descriptor)


def test_local_asset_binding_rejects_symlink(tmp_path):
    target = tmp_path / "target.json"
    target.write_text("{}")
    alias = tmp_path / "alias.json"
    alias.symlink_to(target)
    hashes = {str(alias): study.digest_file(target)}
    with pytest.raises(study.ContractError):
        study.verify_local_assets({"schema": "rgb-local-assets.v1", "mode": "read_only_reference",
                                   "files": hashes}, {"input_hashes": hashes})


def test_preflight_compares_frozen_descriptor_not_just_manifest_internal_hashes(source, tmp_path, monkeypatch):
    conf = config()
    frozen = {"backend_id": "test-only", "input_hashes": {"/model": "old-model-hash"}}
    conf["backend_descriptor_evidence"] = put(tmp_path / "descriptor.json", frozen)
    changed = {**frozen, "input_hashes": {"/model": "new-model-hash"}}
    backend = SimpleNamespace(backend_descriptor=lambda _: changed,
                              public_static_context=lambda _: {"task": {}})
    def imported(name):
        if name == study.BACKEND_MODULE:
            return backend
        raise ImportError(name)
    monkeypatch.setattr(study.importlib, "import_module", imported)
    result = study.preflight(study.prepare_manifest(conf, source), root=source, evidence_root=tmp_path)
    assert "backend_descriptor_changed" in result["blockers"]
    # Legacy portable capsules do not carry a host-bound descriptor requirement.
    conf.pop("backend_descriptor_evidence")
    result = study.preflight(study.prepare_manifest(conf, source), root=source, evidence_root=tmp_path)
    assert not any(b.startswith("backend_descriptor_") for b in result["blockers"])
    # D3 cannot remove that binding to bypass the local asset requirement.
    conf["submitter"] = "D3"
    result = study.preflight(study.prepare_manifest(conf, source), root=source, evidence_root=tmp_path)
    assert "backend_descriptor_evidence_invalid:ContractError" in result["blockers"]
    assert "local_assets_invalid:ContractError" in result["blockers"]


def test_d3_demands_a3_v2_even_when_old_verifier_would_accept(source, tmp_path, monkeypatch):
    conf = config()
    conf["submitter"] = "D3"
    conf["offline_boundary_evidence"] = put(tmp_path / "boundary.json", {
        "schema_version": "rgb-offline-boundary-review.v1", "independent_reviewer": "A2"})
    def assess(*args, **kwargs):
        assert kwargs["required_schema"] == "rgb-offline-boundary-review.v2"
        return {"ready": True}
    def imported(name):
        if name == "harness.rgb_communication_scenarios":
            return SimpleNamespace(assess_offline_boundary=assess)
        raise ImportError(name)
    monkeypatch.setattr(study.importlib, "import_module", imported)
    result = study.preflight(study.prepare_manifest(conf, source), root=source, evidence_root=tmp_path)
    assert "d3_requires_independent_a3_boundary_v2" in result["blockers"]


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


@pytest.mark.parametrize("ignore_term", [False, True])
@pytest.mark.parametrize("parent_signal", [signal.SIGTERM, signal.SIGINT])
def test_parent_sigterm_reaps_owned_child_before_returning(tmp_path, ignore_term, parent_signal):
    marker = tmp_path / "owned-child.pid"
    parent = subprocess.Popen([sys.executable, "-c", "\n".join([
        "import sys", "from pathlib import Path",
        "from harness.rgb_communication_study import bounded_process",
        "child = 'import os,time,signal; from pathlib import Path; '",
        f"child += 'signal.signal(signal.SIGTERM, signal.SIG_IGN); ' if {ignore_term!r} else ''",
        "child += 'Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(30)'",
        "child = 'import sys;' + child",
        "result = bounded_process([sys.executable, '-c', child, sys.argv[1]], cwd=Path.cwd(), log_path=Path(sys.argv[2]), timeout_s=20)",
        "print(result, flush=True)",
        "raise SystemExit(0 if result['exit_code'] == 130 else 1)"]),
        str(marker), str(tmp_path / "child.log")], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, start_new_session=True)
    owned_pid = None
    try:
        deadline = time.monotonic() + 5
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert marker.exists(), parent.poll()
        owned_pid = int(marker.read_text())
        parent.send_signal(parent_signal)
        stdout, stderr = parent.communicate(timeout=12)
        assert parent.returncode == 0, (parent.returncode, stdout, stderr)
        assert "'child_reaped': True" in stdout
        assert "'process_group_gone': True" in stdout
        with pytest.raises(ProcessLookupError):
            os.kill(owned_pid, 0)
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
        if owned_pid is not None:
            try:
                os.killpg(owned_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


@pytest.mark.parametrize("exit_code,timed_out,reason", [
    (130, False, "study_interrupted"), (-15, False, "study_interrupted"),
    (124, True, "previous_trial_wall_timeout"), (1, False, "previous_trial_process_failed")])
def test_interrupted_study_does_not_start_next_trial_and_hashes_after_reap(
        source, tmp_path, monkeypatch, exit_code, timed_out, reason):
    value = study.prepare_manifest(config(), source)
    output = tmp_path / "interrupted-study"
    calls = []
    monkeypatch.setattr(study, "preflight", ready_preflight)
    def child(*args, **kwargs):
        calls.append(kwargs)
        # Simulate the last child write immediately before confirmed reap.
        (kwargs["log_path"].parent / "last-child-write.txt").write_text("closed-and-reaped")
        return {"exit_code": exit_code, "timed_out": timed_out, "wall_time_s": .01,
                "process_group": 999999999, "child_reaped": True, "process_group_gone": True}
    monkeypatch.setattr(study, "bounded_process", child)
    result = study.run_study(value, root=source, evidence_root=source, output=output)
    assert len(calls) == 1
    assert result["planned_denominator"] == 2
    assert result["run_rows"][1] == {"run_id": "joint", "outcome": "unrun", "reason": reason}
    assert not (output / "runs/joint").exists()
    hashes = study.read_json(output / "artifact-hashes.json")
    assert hashes["runs/solo/last-child-write.txt"] == study.digest_file(output / "runs/solo/last-child-write.txt")


def test_unconfirmed_cleanup_never_hashes_live_artifacts(source, tmp_path, monkeypatch):
    value = study.prepare_manifest(config(), source)
    output = tmp_path / "cleanup-unknown"
    monkeypatch.setattr(study, "preflight", ready_preflight)
    monkeypatch.setattr(study, "bounded_process", lambda *a, **k: {
        "exit_code": 124, "timed_out": True, "child_reaped": True, "process_group_gone": False})
    result = study.run_study(value, root=source, evidence_root=source, output=output)
    assert result["artifact_hashes_finalized"] is False
    assert result["run_rows"][1]["reason"] == "child_cleanup_unconfirmed"
    assert not (output / "artifact-hashes.json").exists()


def test_normal_trial_return_continues_even_when_physical_goal_failed(source, tmp_path, monkeypatch):
    value = study.prepare_manifest(config(), source)
    output = tmp_path / "physical-failures"
    calls = []
    monkeypatch.setattr(study, "preflight", ready_preflight)
    def child(*args, **kwargs):
        calls.append(kwargs)
        directory = kwargs["log_path"].parent
        study.write_new_json(directory / "result.json", {"run_id": directory.name,
            "schema_version": study.TRIAL_SCHEMA, "outcome": "failure", "source_artifact_hashes": {}})
        return {"exit_code": 0, "timed_out": False, "child_reaped": True, "process_group_gone": True}
    monkeypatch.setattr(study, "bounded_process", child)
    result = study.run_study(value, root=source, evidence_root=source, output=output)
    assert len(calls) == 2
    assert [row["outcome"] for row in result["run_rows"]] == ["failure", "failure"]
    assert result["artifact_hashes_finalized"] is True


def test_exited_leader_does_not_leave_its_same_group_descendant_writing(tmp_path):
    marker = tmp_path / "descendant.pid"
    program = "\n".join([
        "import subprocess,sys,time", "from pathlib import Path",
        "worker='import os,sys,time; from pathlib import Path; Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(30)'",
        "subprocess.Popen([sys.executable,'-c',worker,sys.argv[1]])",
        "deadline=time.monotonic()+3",
        "while not Path(sys.argv[1]).exists() and time.monotonic()<deadline: time.sleep(.01)"])
    result = study.bounded_process([sys.executable, "-c", program, str(marker)],
        cwd=tmp_path, log_path=tmp_path / "descendant.log", timeout_s=4, cleanup_grace_s=1)
    assert result["exit_code"] == 125  # normal leader exit was not clean completion
    assert result["child_reaped"] and result["process_group_gone"]
    with pytest.raises(ProcessLookupError):
        os.kill(int(marker.read_text()), 0)


@pytest.mark.parametrize("signum", [signal.SIGTERM, signal.SIGINT])
def test_signal_after_spawn_before_wait_cannot_escape_owned_cleanup(tmp_path, monkeypatch, signum):
    original = subprocess.Popen
    children = []
    def spawn(*args, **kwargs):
        child = original(*args, **kwargs)
        children.append(child)
        os.kill(os.getpid(), signum)  # real signal before Popen returns the handle
        return child
    monkeypatch.setattr(study.subprocess, "Popen", spawn)
    try:
        result = study.bounded_process([sys.executable, "-c", "import time; time.sleep(30)"],
            cwd=tmp_path, log_path=tmp_path / "spawn-window.log", timeout_s=2)
        assert result["exit_code"] == 130
        assert result["child_reaped"] and result["process_group_gone"]
        assert children[0].poll() is not None
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()


def test_repeat_interrupt_during_cleanup_does_not_escape_reap(tmp_path, monkeypatch):
    original_killpg = os.killpg
    injected = []
    def killpg(pid, signum):
        result = original_killpg(pid, signum)
        if signum == signal.SIGTERM and not injected:
            injected.append(True)
            os.kill(os.getpid(), signal.SIGINT)
            os.kill(os.getpid(), signal.SIGTERM)
        return result
    monkeypatch.setattr(study.os, "killpg", killpg)
    result = study.bounded_process([sys.executable, "-c",
        "import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)"],
        cwd=tmp_path, log_path=tmp_path / "repeated-interrupt.log", timeout_s=.2, cleanup_grace_s=1)
    assert injected and result["timed_out"]
    assert result["child_reaped"] and result["process_group_gone"]


def test_inventory_write_failure_cannot_publish_finalization_receipt(source, tmp_path, monkeypatch):
    value = study.prepare_manifest(config(), source)
    output = tmp_path / "interrupted-inventory"
    monkeypatch.setattr(study, "preflight", ready_preflight)
    monkeypatch.setattr(study, "bounded_process", lambda *a, **k: {
        "exit_code": 0, "timed_out": False, "child_reaped": True, "process_group_gone": True})
    original_write = study.write_new_json
    def write(path, value):
        if path.name == "artifact-hashes.json":
            path.write_text('{"incomplete":')
            raise OSError("test-only interrupted inventory")
        original_write(path, value)
    monkeypatch.setattr(study, "write_new_json", write)
    with pytest.raises(OSError, match="interrupted inventory"):
        study.run_study(value, root=source, evidence_root=source, output=output)
    report = study.read_json(output / "report.json")
    assert report["child_cleanup_confirmed"] is True
    assert "artifact_hashes_finalized" not in report
    assert not (output / "artifact-finalization.json").exists()


def test_unconfirmed_reap_never_restarts_cleanup_grace(tmp_path, monkeypatch):
    class UnreapableTestChild:
        pid = 999999999
        returncode = None
        def wait(self, timeout=None):
            raise subprocess.TimeoutExpired("test-only", timeout)
        def poll(self):
            return None
    signals = []
    monkeypatch.setattr(study.subprocess, "Popen", lambda *a, **k: UnreapableTestChild())
    monkeypatch.setattr(study.os, "killpg", lambda pid, sig: signals.append(sig))
    result = study.bounded_process(["test-only-not-executed"], cwd=tmp_path,
        log_path=tmp_path / "unreapable.log", timeout_s=.001, cleanup_grace_s=.02)
    assert not result["child_reaped"] and not result["process_group_gone"]
    assert signals.count(signal.SIGTERM) == 1
    assert signals.count(signal.SIGKILL) == 1


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
    monkeypatch.setattr(study, "preflight", ready_preflight)
    manifest["config"]["trials"].reverse()
    with pytest.raises(study.ContractError, match="configuration drift"):
        study.run_trial(manifest, run_id="solo", root=source, evidence_root=source, output=tmp_path)


def test_supervisor_callback_is_required_and_never_derived_from_evaluator():
    calls = []
    callback = lambda: calls.append("supervisor")
    bundle = SimpleNamespace(clock_snapshot=callback,
        evaluation_snapshot=lambda: pytest.fail("referee must not supply the clock"))
    assert study.supervisor_clock_callback(bundle) is callback
    assert calls == []
    for invalid in (None, 0, {"actual_time_s": .75}):
        bundle.clock_snapshot = invalid
        with pytest.raises(study.ContractError, match="clock callback"):
            study.supervisor_clock_callback(bundle)


def test_preflight_requires_both_clock_capabilities_without_building_backend(source, monkeypatch):
    conf = config()
    backend = SimpleNamespace(backend_descriptor=lambda _: {
        "synthetic": False, "weld": False, "camera_fov_changed": False,
        "clock_owner": "single_simulator", "backend_id": "test-only"},
        public_static_context=lambda _: {"task": {}},
        build_rgb_skill_backend=lambda _: pytest.fail("preflight must not construct physics"))
    runtime = SimpleNamespace(AsyncRuntimeLimits=lambda **kw: SimpleNamespace(**kw, validate=lambda: None),
                              run_rgb_communication_async=lambda *args: None)
    def imported(name):
        if name == study.BACKEND_MODULE:
            return backend
        if name == "harness.rgb_communication_async":
            return runtime
        raise ImportError(name)
    monkeypatch.setattr(study.importlib, "import_module", imported)
    result = study.preflight(study.prepare_manifest(conf, source), root=source, evidence_root=source)
    assert "backend_supervisor_clock_unavailable" in result["blockers"]
    assert "async_supervisor_clock_not_connected" in result["blockers"]


def test_trial_passes_clock_callback_only_to_supervisor(source, tmp_path, monkeypatch):
    conf = config()
    conf["trials"][0]["replay_actions"] = {rid: [] for rid in ("r1", "r2", "r3")}
    value = study.prepare_manifest(conf, source)
    sequence = []
    clock = lambda: {"schema": "ugrp.execution_clock.v1", "clock_domain": "sim",
                     "last_acknowledged_time_s": 0, "actual_time_s": 0}
    bundle = SimpleNamespace(actor_port=object(), clock_snapshot=clock, provenance={
        "execution_bundle_id": "test-only", "execution_bundle_sha256": "test-hash",
        "effective_execution": {}, "diff_from_historical_f1": {},
        "execution_source": {"git_sha": value["source"]["git_sha"], "dirty": False},
        "execution_environment": {"sha256": "test-env"}},
        evaluation_snapshot=lambda: sequence.append("evaluation") or {},
        close=lambda: sequence.append("close"))
    def run(port, planners, **kwargs):
        assert port is bundle.actor_port and set(planners) == {"r1", "r2", "r3"}
        assert kwargs["clock_snapshot"] is clock
        assert kwargs["common_task"] == {}
        assert "clock_snapshot" not in kwargs["provenance"]
        assert sequence == []
        sequence.append("runtime")
    def imported(name):
        if name == study.BACKEND_MODULE:
            return SimpleNamespace(build_rgb_skill_backend=lambda _: bundle)
        if name == "harness.rgb_communication_async":
            return SimpleNamespace(AsyncRuntimeLimits=lambda **_: SimpleNamespace(validate=lambda: None),
                                   OfflineDecisionPlanner=lambda *a, **k: object())
        if name == study.RUNTIME_MODULE:
            return SimpleNamespace(run_rgb_communication_async=run)
        raise ImportError(name)
    monkeypatch.setattr(study, "preflight", ready_preflight)
    monkeypatch.setattr(study.importlib, "import_module", imported)
    result = study.run_trial(value, run_id="solo", root=source, evidence_root=source,
                             output=tmp_path / "trial")
    assert sequence == ["runtime", "evaluation", "close"]
    assert result["outcome"] == "missing_artifact"  # fake deliberately produced no runtime
