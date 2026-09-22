"""Colab transport tests use local archives/fakes only, never a cloud session."""
import hashlib
import json
from pathlib import Path
import zipfile

import pytest

from harness.rgb_communication_study import ContractError, digest_file
from scripts.submit_rgb_communication_study import pack_inputs, input_setup_code, verify_retrieval


def test_input_capsule_is_explicit_hash_verified_and_non_overwriting(tmp_path):
    inputs = tmp_path / "inputs"
    inputs.mkdir()
    (inputs / "manifest.json").write_text("{}")
    (inputs / "unlisted-private.txt").write_text("must not upload")
    inventory = {"manifest.json": digest_file(inputs / "manifest.json")}
    archive = tmp_path / "inputs.zip"
    assert pack_inputs(inputs, inventory, archive) == digest_file(archive)
    with zipfile.ZipFile(archive) as bundle:
        assert set(bundle.namelist()) == {"manifest.json", "capsule-inventory.json"}
    with pytest.raises(FileExistsError):
        pack_inputs(inputs, inventory, archive)
    with pytest.raises(ContractError):
        pack_inputs(inputs, {"manifest.json": "0" * 64}, tmp_path / "invalid.zip")


def test_input_setup_preserves_hashes_and_cannot_overwrite(tmp_path):
    inputs = tmp_path / "inputs"
    inputs.mkdir()
    (inputs / "manifest.json").write_text('{"test_only":true}')
    inventory = {"manifest.json": digest_file(inputs / "manifest.json")}
    archive = tmp_path / "inputs.zip"
    sha = pack_inputs(inputs, inventory, archive)
    code = input_setup_code(str(tmp_path / "remote"), str(archive), sha)
    exec(compile(code, "synthetic-local-input-setup", "exec"), {})
    copied = tmp_path / "remote/source/outputs/study-inputs/manifest.json"
    assert copied.read_bytes() == (inputs / "manifest.json").read_bytes()
    with pytest.raises(FileExistsError):
        exec(compile(code, "synthetic-local-input-setup", "exec"), {})


def test_retrieval_verifies_source_and_each_artifact(tmp_path):
    archive = tmp_path / "result.zip"
    evidence = b'{"actual":false,"fixture":true}'
    report = {"source_sha": "a" * 40, "exit_code": 1,
              "artifacts": {"result/report.json": hashlib.sha256(evidence).hexdigest()}}
    with zipfile.ZipFile(archive, "x") as bundle:
        bundle.writestr("run.json", json.dumps(report))
        bundle.writestr("result/report.json", evidence)
    assert verify_retrieval(archive, digest_file(archive), "a" * 40) == report
    with pytest.raises(ContractError, match="source SHA"):
        verify_retrieval(archive, digest_file(archive), "b" * 40)
    with pytest.raises(ContractError, match="archive hash"):
        verify_retrieval(archive, "0" * 64, "a" * 40)
