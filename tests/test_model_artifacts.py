"""Offline checks for the published model bundle boundary."""

from __future__ import annotations

from io import BytesIO
import hashlib
import json
from pathlib import Path
import stat
import zipfile

import pytest

from scripts import model_artifacts, sim_cli


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


@pytest.fixture
def model(tmp_path):
    root = tmp_path / "repo"
    (root / "configs").mkdir(parents=True)
    source = tmp_path / "source"
    (source / "act").mkdir(parents=True)
    (source / "act" / "model.safetensors").write_bytes(b"model-data")
    (source / "plan.json").write_bytes(b"{}\n")
    files = [{"path": name, "sha256": _sha((source / name).read_bytes()),
              "bytes": (source / name).stat().st_size}
             for name in ("act/model.safetensors", "plan.json")]
    row = {"id": "act-small", "status": "available", "description": "fixture", "source_sha": "a" * 40,
           "release": {"tag": "models-test-v1", "asset": "act-small.zip",
                       "url": "https://github.com/owner/repo/releases/download/models-test-v1/act-small.zip"},
           "files": files, "entrypoints": {"act": "act"}, "limitations": ["test only"]}
    unavailable = {"id": "act-missing", "status": "unavailable", "reason": "checkpoint absent"}
    registry = {"schema": model_artifacts.SCHEMA, "repository": "owner/repo", "artifacts": [row, unavailable]}
    registry_path = root / "configs/model_artifacts.json"

    def save():
        registry_path.write_text(json.dumps(registry), encoding="utf-8")

    save()
    return root, source, row, registry, save


def _prepared(model, tmp_path):
    root, source, row, registry, save = model
    archive = tmp_path / "act-small.zip"
    result = model_artifacts.pack(row, source, archive)
    row["archive"] = {"sha256": result["sha256"], "bytes": result["bytes"]}
    save()
    return root, source, row, registry, save, archive


def _replace_archive(row, save, path, entries):
    with zipfile.ZipFile(path, "w") as bundle:
        for name, data, mode in entries:
            info = zipfile.ZipInfo(name)
            if mode is not None:
                info.create_system = 3
                info.external_attr = mode << 16
            bundle.writestr(info, data)
    row["archive"] = {"sha256": _sha(path.read_bytes()), "bytes": path.stat().st_size}
    save()


def test_pack_first_archive_reproducible_and_cli_routes(model, tmp_path, monkeypatch, capsys):
    root, source, row, _, _ = model
    monkeypatch.setattr(sim_cli, "ROOT", root)
    first = tmp_path / "first.zip"
    second = tmp_path / "second.zip"
    assert sim_cli.main(["models", "pack", row["id"], "--source", str(source), "--output", str(first)]) == 0
    assert sim_cli.main(["models", "pack", row["id"], "--source", str(source), "--output", str(second)]) == 0
    assert first.read_bytes() == second.read_bytes()
    assert set(zipfile.ZipFile(first).namelist()) == {item["path"] for item in row["files"]}
    assert sim_cli.main(["models", "list"]) == 2  # publication metadata is still absent
    assert sim_cli.main(["models", "pack", row["id"], "--source", str(source), "--output", str(first)]) == 2
    assert "already exists" in capsys.readouterr().err
    assert sim_cli.main(["models", "pack", row["id"], "--source", str(source), "--output", str(source / "nested.zip")]) == 2
    with pytest.raises(SystemExit) as exit_info:
        sim_cli.main(["models", "--help"])
    assert exit_info.value.code == 0


def test_list_show_fetch_verify_idempotent_and_tamper(model, tmp_path, monkeypatch, capsys):
    root, _, row, _, _, archive = _prepared(model, tmp_path)
    monkeypatch.setattr(sim_cli, "ROOT", root)
    assert sim_cli.main(["models", "list"]) == 0
    assert "act-missing\tunavailable\tcheckpoint absent" in capsys.readouterr().out
    assert sim_cli.main(["models", "show", "act-small"]) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "available"
    assert sim_cli.main(["models", "fetch", "act-small", "--archive", str(archive)]) == 0
    first = json.loads(capsys.readouterr().out)
    assert first["installed"] is True
    destination = root / "outputs/models/act-small"
    assert (destination / model_artifacts.RECEIPT).is_file()
    assert sim_cli.main(["models", "verify", "act-small"]) == 0
    capsys.readouterr()
    assert sim_cli.main(["models", "fetch", "act-small", "--archive", str(archive)]) == 0
    assert json.loads(capsys.readouterr().out)["installed"] is False
    (destination / "plan.json").write_bytes(b"bad")
    assert sim_cli.main(["models", "verify", "act-small"]) == 2
    assert sim_cli.main(["models", "fetch", "act-small", "--archive", str(archive)]) == 2
    assert (destination / "plan.json").read_bytes() == b"bad"


def test_unavailable_rejected_and_url_checked(model, tmp_path, monkeypatch, capsys):
    root, source, row, registry, save, archive = _prepared(model, tmp_path)
    monkeypatch.setattr(sim_cli, "ROOT", root)
    assert sim_cli.main(["models", "fetch", "act-missing", "--archive", str(archive)]) == 2
    assert "checkpoint absent" in capsys.readouterr().err
    assert sim_cli.main(["models", "verify", "act-missing"]) == 2
    row["release"]["url"] = "https://evil.example/act-small.zip"
    save()
    assert sim_cli.main(["models", "show", "act-small"]) == 2
    assert "release URL" in capsys.readouterr().err


@pytest.mark.parametrize("bad_name", ["../escape", "/absolute", "act\\evil", "act/../evil"])
def test_bad_zip_paths_rollback(model, tmp_path, bad_name):
    root, _, row, _, save, archive = _prepared(model, tmp_path)
    entries = [(item["path"], b"model-data" if item["path"].startswith("act/") else b"{}\n", None)
               for item in row["files"]]
    entries.append((bad_name, b"evil", None))
    _replace_archive(row, save, archive, entries)
    with pytest.raises(model_artifacts.ArtifactError):
        model_artifacts.fetch(row, root / "outputs/models/act-small", archive=archive)
    assert not (root / "outputs/models/act-small").exists()


def test_duplicate_symlink_missing_and_wrong_file_rejected(model, tmp_path):
    root, _, row, _, save, archive = _prepared(model, tmp_path)
    good = [("act/model.safetensors", b"model-data", None), ("plan.json", b"{}\n", None)]
    cases = [good + [good[0]],
             [("act/model.safetensors", b"target", stat.S_IFLNK | 0o777), good[1]],
             [good[0]],
             [("act/model.safetensors", b"wrong-data", None), good[1]]]
    for index, entries in enumerate(cases):
        _replace_archive(row, save, archive, entries)
        with pytest.raises(model_artifacts.ArtifactError):
            model_artifacts.fetch(row, root / f"outputs/models/attempt-{index}", archive=archive)
        assert not (root / f"outputs/models/attempt-{index}").exists()


def test_archive_hash_size_and_download_mismatch(model, tmp_path, monkeypatch):
    root, _, row, _, _, archive = _prepared(model, tmp_path)
    destination = root / "outputs/models/act-small"
    row["archive"]["bytes"] += 1
    with pytest.raises(model_artifacts.ArtifactError, match="archive size/SHA-256"):
        model_artifacts.fetch(row, destination, archive=archive)
    assert not destination.exists()
    row["archive"]["bytes"] -= 1
    row["archive"]["sha256"] = "0" * 64

    class Response(BytesIO):
        def geturl(self):
            return row["release"]["url"]

    monkeypatch.setattr(model_artifacts, "urlopen", lambda _request, timeout: Response(archive.read_bytes()))
    with pytest.raises(model_artifacts.ArtifactError, match="size/SHA-256"):
        model_artifacts.fetch(row, destination)
    assert not destination.exists()


def test_verify_rejects_symlink_and_extra_file(model, tmp_path):
    root, _, row, _, _, archive = _prepared(model, tmp_path)
    destination = root / "outputs/models/act-small"
    model_artifacts.fetch(row, destination, archive=archive)
    (destination / "extra").write_bytes(b"extra")
    with pytest.raises(model_artifacts.ArtifactError, match="inventory mismatch"):
        model_artifacts.verify_directory(row, destination)
    (destination / "extra").unlink()
    (destination / "plan.json").unlink()
    (destination / "plan.json").symlink_to(archive)
    with pytest.raises(model_artifacts.ArtifactError, match="symlink"):
        model_artifacts.verify_directory(row, destination)
