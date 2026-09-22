"""Synthetic fixtures stay exclusively in the test's temporary logdir."""
import json

import pytest

from harness.rgb_communication_study import digest_file
from scripts.tensorboard_tools.export import convert
from scripts.tensorboard_tools.rgb_communication import RUN_SCHEMA
from test_rgb_communication_evaluation import manifest, write_artifacts


def fixture_source(tmp_path, outcome="failure"):
    plan = manifest()["schedule"][0]
    plan["artifact_relpath"] = "source"
    write_artifacts(tmp_path, plan, outcome)
    src = tmp_path / "source"
    (src / "plan.json").write_text(json.dumps(plan))
    result = {"schema_version": RUN_SCHEMA, "source_sha": "a" * 40, "config_sha256": "b" * 64,
              "run_id": plan["run_id"], "evidence_kind": "synthetic_fixture",
              "outcome": "success",  # deliberately dishonest summary must not win
              "source_artifact_hashes": {name: digest_file(src / name) for name in
                                        ("runtime.jsonl", "evaluator.json", "plan.json")}}
    (src / "result.json").write_text(json.dumps(result))
    return src


def test_synthetic_default_rejected(tmp_path):
    src = fixture_source(tmp_path)
    with pytest.raises(ValueError, match="temporary-logdir"):
        convert(src, tmp_path / "events")
    assert not (tmp_path / "events").exists()


def test_original_evaluator_and_hashes_override_summary_success(tmp_path):
    pytest.importorskip("tensorboard")
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    src = fixture_source(tmp_path)
    snapshot = convert(src, tmp_path / "events", allow_synthetic=True)
    events = EventAccumulator(str(tmp_path / "events")).Reload()
    assert events.Scalars("evaluation/reported_success")[0].value == 0
    assert events.Scalars("result/commands")[0].value == 1
    assert snapshot["metadata"]["outcome"] == "failure"
    assert snapshot["metadata"]["run_id"] == manifest()["schedule"][0]["run_id"]
    assert "evaluation/referee_only" in events.Tags()["tensors"]
    assert set(snapshot["source_files"]) >= {"runtime.jsonl", "evaluator.json", "plan.json"}


def test_tampered_runtime_never_publishes_event_file(tmp_path):
    pytest.importorskip("tensorboard")
    src = fixture_source(tmp_path)
    with (src / "runtime.jsonl").open("a") as stream:
        stream.write("\n")
    with pytest.raises(ValueError, match="artifact changed"):
        convert(src, tmp_path / "events", allow_synthetic=True)
    assert not list((tmp_path / "events").glob("*tfevents*"))


def test_clock_invalid_failure_is_visible_without_fabricating_final_sim_time(tmp_path):
    pytest.importorskip("tensorboard")
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    src = fixture_source(tmp_path)
    evaluator = json.loads((src / "evaluator.json").read_text())
    evaluator["source_snapshot"]["timestamp_s"] = 3.95
    (src / "evaluator.json").write_text(json.dumps(evaluator))
    result = json.loads((src / "result.json").read_text())
    result["source_artifact_hashes"]["evaluator.json"] = digest_file(src / "evaluator.json")
    (src / "result.json").write_text(json.dumps(result))
    manifest = convert(src, tmp_path / "events", allow_synthetic=True)
    events = EventAccumulator(str(tmp_path / "events")).Reload()
    assert manifest["metadata"]["outcome"] == "invalid_artifact"
    assert events.Scalars("evaluation/evidence_valid")[0].value == 0
    assert events.Scalars("evaluation/reported_success")[0].value == 0
    assert events.Scalars("clock/runtime_requested_sim_s")[0].value == 4
    assert "result/sim_s" not in events.Tags()["scalars"]
    assert events.Scalars("result/commands")[0].value == 1


def test_explicit_opt_in_cannot_publish_synthetic_to_non_temporary_directory(tmp_path, monkeypatch):
    from scripts.tensorboard_tools import export
    src = fixture_source(tmp_path)
    monkeypatch.setattr(export.tempfile, "gettempdir", lambda: str(tmp_path / "different-temp"))
    with pytest.raises(ValueError, match="temporary-logdir"):
        convert(src, tmp_path / "events", allow_synthetic=True)


def test_unknown_new_terminal_clock_exports_only_diagnostics(tmp_path):
    pytest.importorskip("tensorboard")
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    src = fixture_source(tmp_path, outcome="success")
    rows = [json.loads(line) for line in (src / "runtime.jsonl").read_text().splitlines()]
    rows[-1]["sim_time_s"] = None
    rows[-1]["payload"]["clock"] = {
        "schema": "rgb-runtime-clock.v1", "clock_domain": "sim", "requested_tick_s": 4.1,
        "last_acknowledged_time_s": 4, "last_successful_requested_tick_s": 4,
        "terminal_time_s": None, "terminal_time_status": "unknown", "reason": "ACTUAL_CLOCK_UNKNOWN"}
    (src / "runtime.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    result = json.loads((src / "result.json").read_text())
    result["source_artifact_hashes"]["runtime.jsonl"] = digest_file(src / "runtime.jsonl")
    (src / "result.json").write_text(json.dumps(result))
    snapshot = convert(src, tmp_path / "events", allow_synthetic=True)
    events = EventAccumulator(str(tmp_path / "events")).Reload()
    assert snapshot["metadata"]["outcome"] == "invalid_artifact"
    assert events.Scalars("evaluation/reported_success")[0].value == 0
    assert events.Scalars("evaluation/physical_mission_complete")[0].value == 1  # raw referee, not valid success
    assert events.Scalars("clock/runtime_requested_sim_s")[0].value == pytest.approx(4.1)
    assert "result/sim_s" not in events.Tags()["scalars"]
    assert "evaluation/false_finish_claim" not in events.Tags()["scalars"]
