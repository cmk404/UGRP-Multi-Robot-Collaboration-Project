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


def test_explicit_opt_in_cannot_publish_synthetic_to_non_temporary_directory(tmp_path, monkeypatch):
    from scripts.tensorboard_tools import export
    src = fixture_source(tmp_path)
    monkeypatch.setattr(export.tempfile, "gettempdir", lambda: str(tmp_path / "different-temp"))
    with pytest.raises(ValueError, match="temporary-logdir"):
        convert(src, tmp_path / "events", allow_synthetic=True)
