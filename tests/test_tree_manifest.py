"""Content manifests used to verify moves and dedupes (scripts/tree_manifest.py)."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import tree_manifest as tm  # noqa: E402


def make_tree(root: Path) -> None:
    (root / "a/b").mkdir(parents=True)
    (root / "a/b/data.bin").write_bytes(b"\x00\x01" * 3000)
    (root / "a/empty.txt").write_bytes(b"")
    (root / "a/empty-dir").mkdir()
    (root / "a/link").symlink_to("b/data.bin")
    (root / "a/dangling").symlink_to("missing")
    (root / "a/b/zero").write_bytes(b"0")


def test_build_records_files_links_and_empty_dirs(tmp_path):
    make_tree(tmp_path / "t")
    manifest = tm.build(tmp_path / "t")
    assert manifest["a/b/data.bin"] == {"type": "file", "size": 6000,
                                        "sha256": hashlib.sha256(b"\x00\x01" * 3000).hexdigest()}
    assert manifest["a/empty.txt"]["size"] == 0
    assert manifest["a/empty.txt"]["sha256"] == hashlib.sha256(b"").hexdigest()
    assert manifest["a/b/zero"]["size"] == 1  # a one-byte "0" file is not treated as empty
    assert manifest["a/empty-dir"] == {"type": "dir"}
    assert manifest["a/link"] == {"type": "link", "size": len("b/data.bin"), "target": "b/data.bin"}
    assert manifest["a/dangling"]["target"] == "missing"  # never followed
    totals = tm.totals(manifest)
    assert totals == {"entries": 6, "files": 3, "links": 2, "bytes": 6000 + 0 + 1 + 10 + 7}


def test_single_file_single_link_and_empty_root(tmp_path):
    (tmp_path / "f").write_bytes(b"x")
    (tmp_path / "l").symlink_to("f")
    (tmp_path / "d").mkdir()
    assert tm.build(tmp_path / "f") == {".": {"type": "file", "size": 1, "sha256": hashlib.sha256(b"x").hexdigest()}}
    assert tm.build(tmp_path / "l") == {".": {"type": "link", "size": 1, "target": "f"}}
    assert tm.build(tmp_path / "d") == {}
    assert tm.totals({}) == {"entries": 0, "files": 0, "links": 0, "bytes": 0}
    assert tm.compare({}, {}) == []
    with pytest.raises(FileNotFoundError):
        tm.build(tmp_path / "absent")


def test_compare_detects_missing_extra_changed_and_type_changes(tmp_path):
    make_tree(tmp_path / "t")
    before = tm.build(tmp_path / "t")
    shutil.copytree(tmp_path / "t", tmp_path / "u", symlinks=True)
    assert tm.compare(before, tm.build(tmp_path / "u")) == []
    assert tm.digest(before) == tm.digest(tm.build(tmp_path / "u"))
    # One flipped byte with the same size is still detected.
    data = bytearray((tmp_path / "u/a/b/data.bin").read_bytes())
    data[100] ^= 0xFF
    (tmp_path / "u/a/b/data.bin").write_bytes(bytes(data))
    (tmp_path / "u/a/empty-dir").rmdir()
    (tmp_path / "u/a/new").write_bytes(b"n")
    os.unlink(tmp_path / "u/a/link")
    (tmp_path / "u/a/link").write_bytes(b"b/data.bin")  # symlink replaced by a file of the same size
    problems = tm.compare(before, tm.build(tmp_path / "u"))
    assert any(p.startswith("changed: a/b/data.bin") for p in problems)
    assert "missing after: a/empty-dir" in problems
    assert "unexpected after: a/new" in problems
    assert any(p.startswith("changed: a/link") for p in problems)
    assert tm.digest(before) != tm.digest(tm.build(tmp_path / "u"))


@pytest.mark.skipif(sys.platform != "darwin", reason="APFS clones (cp -c) are macOS-only")
def test_apfs_clone_keeps_manifest_identical(tmp_path):
    make_tree(tmp_path / "t")
    (tmp_path / "c").mkdir()
    result = subprocess.run(["cp", "-c", "-p", str(tmp_path / "t/a/b/data.bin"), str(tmp_path / "c/data.bin")],
                            capture_output=True)
    if result.returncode != 0:
        pytest.skip("filesystem without clone support")
    assert tm.build(tmp_path / "c/data.bin")["."] == tm.build(tmp_path / "t")["a/b/data.bin"]


def test_write_tsv_prefixes_paths(tmp_path):
    make_tree(tmp_path / "t")
    out = tmp_path / "m.tsv"
    tm.write_tsv(tm.build(tmp_path / "t"), out, prefix="outputs/run")
    lines = out.read_text().splitlines()
    assert lines[0] == "path\ttype\tsize\tsha256_or_target"
    assert "outputs/run/a/link\tlink\t10\tb/data.bin" in lines
    tm.write_tsv(tm.build(tmp_path / "t/a/b/zero"), out, prefix="outputs/zero")
    assert out.read_text().splitlines()[1].startswith("outputs/zero\tfile\t1\t")
