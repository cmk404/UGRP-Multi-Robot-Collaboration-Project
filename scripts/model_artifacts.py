"""Fetch and verify published model bundles without simulation dependencies."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import tempfile
from urllib.parse import quote, urlsplit
from urllib.request import Request, urlopen
import zipfile


ROOT = Path(__file__).resolve().parents[1]
RECEIPT = ".model-artifact-receipt.json"
SCHEMA = "ugrp.model-artifacts.v1"
_ID = re.compile(r"[a-z0-9][a-z0-9-]*\Z")
_HEX = re.compile(r"[0-9a-f]{64}\Z")
_SHA = re.compile(r"[0-9a-f]{40}\Z")
_CHUNK = 1024 * 1024


class ArtifactError(ValueError):
    """An artifact is unavailable, malformed, or failed verification."""


def _relative(value: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise ArtifactError(f"invalid artifact path: {value!r}")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in value.split("/")) or path.as_posix() != value:
        raise ArtifactError(f"invalid artifact path: {value!r}")
    if value == RECEIPT or value.startswith(RECEIPT + "/"):
        raise ArtifactError("artifact payload cannot contain the receipt")
    return value


def _integer(value, label: str) -> int:
    if type(value) is not int or value < 0:
        raise ArtifactError(f"{label} must be a non-negative integer")
    return value


def _digest(value, label: str) -> str:
    if not isinstance(value, str) or not _HEX.fullmatch(value):
        raise ArtifactError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _inventory(row: dict) -> dict[str, dict]:
    files = row.get("files")
    if not isinstance(files, list) or not files:
        raise ArtifactError(f"{row['id']}: files inventory is missing")
    seen = set()
    inventory = {}
    for item in files:
        if not isinstance(item, dict):
            raise ArtifactError("invalid inventory entry")
        name = _relative(item.get("path"))
        folded = name.casefold()
        if folded in seen:
            raise ArtifactError(f"duplicate inventory path: {name}")
        seen.add(folded)
        inventory[name] = {"path": name, "sha256": _digest(item.get("sha256"), name),
                           "bytes": _integer(item.get("bytes"), name)}
    for name in inventory:
        for parent in PurePosixPath(name).parents:
            if str(parent).casefold() in seen:
                raise ArtifactError(f"file/directory inventory collision: {name}")
    return inventory


def _release(row: dict, repository: str) -> dict:
    release = row.get("release")
    if not isinstance(release, dict):
        raise ArtifactError(f"{row['id']}: release is missing")
    tag, asset, url = (release.get(key) for key in ("tag", "asset", "url"))
    if not isinstance(tag, str) or not _ID.fullmatch(tag) or not isinstance(asset, str) or not asset.endswith(".zip") or asset != PurePosixPath(asset).name or "\\" in asset:
        raise ArtifactError(f"{row['id']}: invalid release tag or asset")
    expected = f"https://github.com/{repository}/releases/download/{quote(tag)}/{quote(asset)}"
    parsed = urlsplit(url) if isinstance(url, str) else None
    if url != expected or parsed is None or parsed.scheme != "https" or parsed.hostname != "github.com" or parsed.username or parsed.password:
        raise ArtifactError(f"{row['id']}: release URL does not match repository/tag/asset")
    return release


def load_registry(root: Path = ROOT, *, pack_id: str | None = None) -> dict:
    path = Path(root) / "configs/model_artifacts.json"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ArtifactError(f"cannot read model registry {path}: {error}") from error
    if not isinstance(data, dict) or data.get("schema") != SCHEMA:
        raise ArtifactError(f"expected registry schema {SCHEMA}")
    repository = data.get("repository")
    if not isinstance(repository, str) or not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repository):
        raise ArtifactError("invalid GitHub repository")
    rows = data.get("artifacts")
    if not isinstance(rows, list):
        raise ArtifactError("registry artifacts must be a list")
    seen = set()
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("id"), str) or not _ID.fullmatch(row["id"]) or row["id"] in seen:
            raise ArtifactError("invalid or duplicate artifact ID")
        seen.add(row["id"])
        if row.get("status") not in ("available", "unavailable"):
            raise ArtifactError(f"{row['id']}: invalid status")
        if row.get("status") == "available":
            if not isinstance(row.get("source_sha"), str) or not _SHA.fullmatch(row["source_sha"]):
                raise ArtifactError(f"{row['id']}: invalid source SHA")
            _inventory(row)
            _release(row, repository)
            if row["id"] != pack_id:
                archive = row.get("archive")
                if not isinstance(archive, dict):
                    raise ArtifactError(f"{row['id']}: archive is missing")
                _digest(archive.get("sha256"), "archive")
                _integer(archive.get("bytes"), "archive")
        elif not row.get("reason"):
            raise ArtifactError(f"{row['id']}: unavailable reason is missing")
    return data


def _row(registry: dict, artifact_id: str, *, available: bool = False) -> dict:
    row = next((entry for entry in registry["artifacts"] if entry["id"] == artifact_id), None)
    if row is None:
        raise ArtifactError(f"unknown model artifact: {artifact_id}")
    if available and row["status"] != "available":
        raise ArtifactError(f"{artifact_id} is unavailable: {row.get('reason', 'no published model')}")
    return row


def _hash_file(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(_CHUNK):
            digest.update(chunk)
            size += len(chunk)
    return digest.hexdigest(), size


def _no_symlink_components(path: Path) -> None:
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ArtifactError(f"symlink in path: {current}")


def _safe_target(path: Path) -> Path:
    path = path.absolute()
    _no_symlink_components(path)
    return path


def verify_directory(row: dict, path: Path, *, allow_receipt: bool = True) -> dict:
    """Check real file bytes and exact tree membership; a receipt is never trusted."""
    _row({"artifacts": [row]}, row["id"], available=True)
    path = _safe_target(Path(path))
    if not path.is_dir():
        raise ArtifactError(f"model directory does not exist: {path}")
    expected = _inventory(row)
    found = {}
    expected_dirs = {str(parent) for name in expected for parent in PurePosixPath(name).parents if str(parent) != "."}
    for base, dirs, files in os.walk(path, followlinks=False):
        for name in dirs + files:
            candidate = Path(base) / name
            if candidate.is_symlink():
                raise ArtifactError(f"symlink in model directory: {candidate}")
            if candidate.is_file():
                rel = candidate.relative_to(path).as_posix()
                found[rel] = candidate
            elif candidate.is_dir():
                if candidate.relative_to(path).as_posix() not in expected_dirs:
                    raise ArtifactError(f"unexpected model directory: {candidate}")
            else:
                raise ArtifactError(f"unsupported model directory entry: {candidate}")
    allowed = set(expected)
    if allow_receipt:
        allowed.add(RECEIPT)
    extras = set(found) - allowed
    missing = set(expected) - set(found)
    if extras or missing:
        raise ArtifactError(f"inventory mismatch: missing={sorted(missing)}, extra={sorted(extras)}")
    for name, item in expected.items():
        digest, size = _hash_file(found[name])
        if digest != item["sha256"] or size != item["bytes"]:
            raise ArtifactError(f"file mismatch: {name}")
    return {"id": row["id"], "path": str(path), "verified_files": len(expected), "verified_bytes": sum(i["bytes"] for i in expected.values())}


def _archive_hash(path: Path, row: dict) -> tuple[str, int]:
    digest, size = _hash_file(path)
    expected = row["archive"]
    if digest != expected["sha256"] or size != expected["bytes"]:
        raise ArtifactError(f"archive size/SHA-256 mismatch for {row['id']}")
    return digest, size


def _download(row: dict, destination: Path) -> None:
    request = Request(row["release"]["url"], headers={"User-Agent": "UGRP-model-artifacts/1"})
    expected = row["archive"]["bytes"]
    digest = hashlib.sha256()
    size = 0
    with urlopen(request, timeout=60) as response, destination.open("xb") as stream:
        final = urlsplit(response.geturl())
        if final.scheme != "https" or final.hostname not in ("github.com", "release-assets.githubusercontent.com") or final.username or final.password:
            raise ArtifactError("release download redirected outside GitHub HTTPS")
        while chunk := response.read(_CHUNK):
            size += len(chunk)
            if size > expected:
                raise ArtifactError("release download exceeds registry byte count")
            digest.update(chunk)
            stream.write(chunk)
    if size != expected or digest.hexdigest() != row["archive"]["sha256"]:
        raise ArtifactError("release download size/SHA-256 mismatch")


def _unpack(row: dict, archive: Path, directory: Path) -> None:
    inventory = _inventory(row)
    expected_dirs = {str(parent) for name in inventory for parent in PurePosixPath(name).parents if str(parent) != "."}
    seen = set()
    seen_folded = set()
    with zipfile.ZipFile(archive) as bundle:
        for info in bundle.infolist():
            raw = info.filename
            name = _relative(raw[:-1] if raw.endswith("/") else raw)
            folded = name.casefold()
            if folded in seen_folded:
                raise ArtifactError(f"duplicate ZIP entry: {name}")
            seen.add(name)
            seen_folded.add(folded)
            mode = (info.external_attr >> 16) & 0xFFFF
            kind = stat.S_IFMT(mode)
            if kind not in (0, stat.S_IFDIR if info.is_dir() else stat.S_IFREG):
                raise ArtifactError(f"unsupported ZIP entry type: {name}")
            if info.is_dir():
                if name not in expected_dirs:
                    raise ArtifactError(f"unexpected ZIP directory: {name}")
                continue
            if name not in inventory:
                raise ArtifactError(f"unexpected ZIP file: {name}")
            item = inventory[name]
            if info.file_size != item["bytes"]:
                raise ArtifactError(f"ZIP file size mismatch: {name}")
            target = directory.joinpath(*PurePosixPath(name).parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            digest = hashlib.sha256()
            count = 0
            with bundle.open(info) as source, target.open("xb") as output:
                while chunk := source.read(_CHUNK):
                    count += len(chunk)
                    if count > item["bytes"]:
                        raise ArtifactError(f"ZIP file exceeds expected size: {name}")
                    digest.update(chunk)
                    output.write(chunk)
            if count != item["bytes"] or digest.hexdigest() != item["sha256"]:
                raise ArtifactError(f"ZIP file SHA-256 mismatch: {name}")
    if set(inventory) - seen:
        raise ArtifactError(f"ZIP missing files: {sorted(set(inventory) - seen)}")
    verify_directory(row, directory, allow_receipt=False)


def fetch(row: dict, destination: Path, *, archive: Path | None = None) -> dict:
    _row({"artifacts": [row]}, row["id"], available=True)
    destination = _safe_target(Path(destination))
    destination.parent.mkdir(parents=True, exist_ok=True)
    _safe_target(destination)
    if destination.exists():
        return {**verify_directory(row, destination), "installed": False}
    # The lock prevents concurrent model-artifact commands from racing on one path.
    lock = destination.parent / f".{destination.name}.install.lock"
    try:
        lock_fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as error:
        raise ArtifactError(f"another installation is using {lock}") from error
    os.close(lock_fd)
    try:
        if destination.exists():
            return {**verify_directory(row, destination), "installed": False}
        with tempfile.TemporaryDirectory(prefix=f".{destination.name}-", dir=destination.parent) as staging:
            staging = Path(staging)
            downloaded = staging / "archive.zip"
            if archive is None:
                _download(row, downloaded)
            else:
                archive = _safe_target(Path(archive))
                if not archive.is_file():
                    raise ArtifactError(f"archive does not exist: {archive}")
                _archive_hash(archive, row)
                downloaded = archive
            payload = staging / "payload"
            payload.mkdir()
            try:
                _unpack(row, downloaded, payload)
            except (zipfile.BadZipFile, RuntimeError, EOFError) as error:
                raise ArtifactError(f"invalid ZIP archive: {error}") from error
            receipt = {"schema": SCHEMA, "id": row["id"], "archive_sha256": row["archive"]["sha256"],
                       "archive_bytes": row["archive"]["bytes"], "files": row["files"],
                       "installed_at": datetime.now(timezone.utc).isoformat()}
            (payload / RECEIPT).write_text(json.dumps(receipt, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            if destination.exists():
                raise ArtifactError(f"destination appeared during installation: {destination}")
            os.rename(payload, destination)
        return {**verify_directory(row, destination), "installed": True}
    finally:
        lock.unlink(missing_ok=True)


def pack(row: dict, source: Path, output: Path) -> dict:
    _row({"artifacts": [row]}, row["id"], available=True)
    source = _safe_target(Path(source))
    verify_directory(row, source, allow_receipt=False)
    output = _safe_target(Path(output))
    if output == source or source in output.parents:
        raise ArtifactError("archive output must be outside the model source directory")
    output.parent.mkdir(parents=True, exist_ok=True)
    _safe_target(output)
    if output.exists():
        raise ArtifactError(f"archive already exists: {output}")
    fd, temp_name = tempfile.mkstemp(prefix=f".{output.name}-", suffix=".zip", dir=output.parent)
    os.close(fd)
    temp = Path(temp_name)
    try:
        inventory = _inventory(row)
        with zipfile.ZipFile(temp, "w") as bundle:
            for name in sorted(inventory):
                info = zipfile.ZipInfo(name, (1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.create_system = 3
                info.external_attr = (stat.S_IFREG | 0o644) << 16
                digest = hashlib.sha256()
                count = 0
                with bundle.open(info, "w", force_zip64=False) as target, (source / name).open("rb") as stream:
                    while chunk := stream.read(_CHUNK):
                        count += len(chunk)
                        if count > inventory[name]["bytes"]:
                            raise ArtifactError(f"source changed while packaging: {name}")
                        digest.update(chunk)
                        target.write(chunk)
                if count != inventory[name]["bytes"] or digest.hexdigest() != inventory[name]["sha256"]:
                    raise ArtifactError(f"source changed while packaging: {name}")
        digest, size = _hash_file(temp)
        if output.exists():
            raise ArtifactError(f"archive appeared during packaging: {output}")
        os.link(temp, output)
        return {"id": row["id"], "path": str(output), "sha256": digest, "bytes": size}
    finally:
        temp.unlink(missing_ok=True)


def main(argv: list[str] | None = None, *, root: Path = ROOT) -> int:
    parser = argparse.ArgumentParser(prog="python -m scripts.sim_cli models", description="List, fetch, verify, and package published model artifacts")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="show status and limitations for all registered models")
    show = sub.add_parser("show", help="print one registry entry as JSON")
    show.add_argument("id")
    fetch_cmd = sub.add_parser("fetch", help="download and install a verified model bundle")
    fetch_cmd.add_argument("id")
    fetch_cmd.add_argument("--output", type=Path, help="installation directory; defaults to outputs/models/ID")
    fetch_cmd.add_argument("--archive", type=Path, help="verified local ZIP instead of a network download")
    verify_cmd = sub.add_parser("verify", help="hash every installed model file")
    verify_cmd.add_argument("id")
    verify_cmd.add_argument("--path", type=Path, help="model directory; defaults to outputs/models/ID")
    pack_cmd = sub.add_parser("pack", help="create a deterministic ZIP from registry file inventory")
    pack_cmd.add_argument("id")
    pack_cmd.add_argument("--source", type=Path, required=True)
    pack_cmd.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        registry = load_registry(root, pack_id=args.id if args.command == "pack" else None)
        if args.command == "list":
            for row in registry["artifacts"]:
                details = "; ".join(row.get("limitations", [])) if row["status"] == "available" else row["reason"]
                print(f"{row['id']}\t{row['status']}\t{details}")
            return 0
        row = _row(registry, args.id, available=args.command != "show")
        if args.command == "show":
            print(json.dumps(row, indent=2, ensure_ascii=False))
        elif args.command == "fetch":
            print(json.dumps(fetch(row, args.output or Path(root) / "outputs/models" / args.id, archive=args.archive), indent=2))
        elif args.command == "verify":
            print(json.dumps(verify_directory(row, args.path or Path(root) / "outputs/models" / args.id), indent=2))
        else:
            print(json.dumps(pack(row, args.source, args.output), indent=2))
        return 0
    except (ArtifactError, OSError, zipfile.BadZipFile) as error:
        print(f"models: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
