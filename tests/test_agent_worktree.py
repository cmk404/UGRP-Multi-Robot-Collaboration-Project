"""Sparse agent worktrees and move-then-remove retirement (scripts/agent_worktree.py)."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import agent_worktree as aw  # noqa: E402
from scripts import tree_manifest  # noqa: E402
from scripts import worktree_guard as guard  # noqa: E402

needs_lsof = pytest.mark.skipif(shutil.which("lsof") is None, reason="lsof is required for the busy check")


def run(cwd: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(cwd), *args], check=True, capture_output=True, text=True).stdout


@pytest.fixture
def repo(tmp_path, monkeypatch):
    for key, value in {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.invalid",
                       "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.invalid",
                       "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1"}.items():
        monkeypatch.setenv(key, value)
    primary = tmp_path / "ugrp"
    primary.mkdir()
    run(primary, "init", "-q", "-b", "main")
    files = {
        ".gitignore": "outputs/\nMUJOCO_LOG.TXT\n__pycache__/\n*.log\n",
        "README.md": "x\n",
        "experiments/a/results.json": "{}\n",
        "experiments/a/README.md": "# a\n",
        "experiments/a/media/frame.jpg": "J" * 4000,
        "experiments/a/evidence.zip": "Z" * 4000,
        "experiments/a/records/r.json.gz": "G" * 4000,
        "experiments/a/video.mp4": "M" * 4000,
        "experiments/dispatch-skill-integration-20260917/models.zip": "K" * 3000,
        "experiments/2026-09-10-rgb-varied-start/models.zip": "V" * 3000,
        "tests/fixtures/frame.jpg": "F" * 100,
        "scripts/tool.py": "print(1)\n",
    }
    for rel, text in files.items():
        (primary / rel).parent.mkdir(parents=True, exist_ok=True)
        (primary / rel).write_text(text)
    run(primary, "add", "-A")
    run(primary, "commit", "-q", "-m", "init")
    return primary


def new(primary: Path, name: str, *extra: str) -> int:
    return aw.main(["--primary", str(primary), "new", name, "--parent", str(primary.parent / "ugrp-wt"),
                    "--no-fetch", "--base", "main", *extra])


def files_in(path: Path) -> set[str]:
    return {str(p.relative_to(path)) for p in path.rglob("*") if p.is_file() and ".git" not in p.parts}


def test_new_worktree_omits_heavy_experiment_media_but_keeps_records(repo, capsys):
    assert new(repo, "kiro-a", "--branch", "kiro/a") == 0
    path = repo.parent / "ugrp-wt" / "kiro-a"
    present = files_in(path)
    assert "experiments/a/results.json" in present and "experiments/a/README.md" in present
    assert "tests/fixtures/frame.jpg" in present  # fixtures outside experiments/ stay
    for heavy in ("experiments/a/media/frame.jpg", "experiments/a/evidence.zip",
                  "experiments/a/records/r.json.gz", "experiments/a/video.mp4"):
        assert heavy not in present
    for runtime in aw.RUNTIME_KEEP:
        assert runtime in present
    assert run(path, "status", "--porcelain") == ""
    assert run(path, "rev-parse", "--abbrev-ref", "HEAD").strip() == "kiro/a"
    assert run(path, "config", "--worktree", "--get", "core.hooksPath").strip() == ".githooks"
    # The primary checkout stays full and unchanged.
    assert "experiments/a/evidence.zip" in files_in(repo)
    out = json.loads(capsys.readouterr().out)
    assert out["owner"] == "kiro" and out["sparse_profile"] == aw.SPARSE_PROFILE
    marker = json.loads((Path(run(path, "rev-parse", "--absolute-git-dir").strip()) / aw.MARKER).read_text())
    assert marker["owner"] == "kiro" and marker["base_sha"] == run(repo, "rev-parse", "main").strip()


def test_new_worktree_refuses_bad_prefix_existing_path_and_branch(repo):
    assert new(repo, "x", "--branch", "feature/x") == 2
    assert new(repo, "x", "--branch", "kiro") == 2
    (repo.parent / "ugrp-wt" / "taken").mkdir(parents=True)
    assert new(repo, "taken", "--branch", "kiro/taken") == 2
    run(repo, "branch", "kiro/exists")
    assert new(repo, "e", "--branch", "kiro/exists") == 2
    assert new(repo, "e", "--branch", "kiro/exists", "--existing-branch") == 0


def test_new_worktree_enforces_per_agent_cap(repo, capsys):
    assert new(repo, "c1", "--branch", "claude/c1", "--max-per-agent", "2") == 0
    assert new(repo, "c2", "--branch", "claude/c2", "--max-per-agent", "2") == 0
    assert new(repo, "k1", "--branch", "kiro/k1", "--max-per-agent", "2") == 0  # other agent unaffected
    capsys.readouterr()
    assert new(repo, "c3", "--branch", "claude/c3", "--max-per-agent", "2") == 3
    assert "cap 2" in capsys.readouterr().err
    assert not (repo.parent / "ugrp-wt" / "c3").exists()
    assert new(repo, "c3", "--branch", "claude/c3", "--max-per-agent", "2", "--allow-over-cap") == 2
    assert new(repo, "c3", "--branch", "claude/c3", "--max-per-agent", "2", "--allow-over-cap",
               "--reason", "frozen cohort source") == 0


def test_detached_frozen_worktree_needs_owner(repo):
    assert new(repo, "frozen", "--detach") == 2
    assert new(repo, "frozen", "--detach", "--owner", "claude") == 0
    rows = {row["path"].name: row for row in aw.worktrees(repo)}
    assert rows["frozen"]["detached"] and rows["frozen"]["owner"] == "claude"


def test_sparsify_existing_full_worktree(repo):
    path = repo.parent / "full"
    run(repo, "worktree", "add", "-q", "-b", "kiro/full", str(path), "main")
    assert "experiments/a/evidence.zip" in files_in(path)
    (path / "experiments/a/README.md").write_text("changed\n")
    assert aw.main(["--primary", str(repo), "sparsify", str(path), "--idle-minutes", "0"]) == 2  # tracked change
    run(path, "checkout", "--", "experiments/a/README.md")
    assert aw.main(["--primary", str(repo), "sparsify", str(path)]) == 2  # just modified: not idle
    assert aw.main(["--primary", str(repo), "sparsify", str(path), "--idle-minutes", "0"]) == 0
    present = files_in(path)
    assert "experiments/a/evidence.zip" not in present and "experiments/a/results.json" in present
    assert run(path, "status", "--porcelain") == ""
    assert aw.main(["--primary", str(repo), "sparsify", str(repo), "--idle-minutes", "0"]) == 2  # primary full
    assert not aw.sparse_enabled(repo) and "experiments/a/evidence.zip" in files_in(repo)


def make_worktree_with_outputs(repo: Path, name: str, branch: str) -> Path:
    path = repo.parent / "ugrp-wt" / name
    run(repo, "worktree", "add", "-q", "-b", branch, str(path), "main")
    (path / "outputs/run-1/frames").mkdir(parents=True)
    (path / "outputs/run-1/result.json").write_text('{"ok": true}\n')
    (path / "outputs/run-1/frames/00001.jpg").write_bytes(b"\xff" * 5000)
    (path / "outputs/run-1/link").symlink_to("result.json")
    (path / "MUJOCO_LOG.TXT").write_text("warn\n")
    (path / "scripts/__pycache__").mkdir()
    (path / "scripts/__pycache__/tool.cpython-312.pyc").write_bytes(b"c" * 100)
    return path


RETIRE = ["--no-fetch", "--no-pr-check", "--base", "main", "--idle-minutes", "0"]


def retire(repo: Path, path: Path, *extra: str) -> int:
    return aw.main(["--primary", str(repo), "retire", str(path), *RETIRE, *extra])


@needs_lsof
def test_retire_moves_ignored_data_verifies_and_removes(repo, capsys):
    path = make_worktree_with_outputs(repo, "kiro-done", "kiro/done")
    assert retire(repo, path) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["dry_run"] and path.exists() and (path / "outputs/run-1/result.json").exists()
    assert sorted(p["path"] for p in plan["moved"]) == ["MUJOCO_LOG.TXT", "outputs/run-1"]
    assert plan["deleted_caches"] == ["scripts/__pycache__"]
    assert retire(repo, path, "--execute") == 0
    receipt = json.loads(capsys.readouterr().out)
    archive = repo / "outputs/retired-worktrees/kiro-done"
    assert receipt["verified"] and receipt["worktree_remove_exit"] == 0
    # outputs/<name> keeps its relative path in the primary; other ignored files go to the archive folder.
    assert (repo / "outputs/run-1/frames/00001.jpg").read_bytes() == b"\xff" * 5000
    assert (repo / "outputs/run-1/link").is_symlink()
    assert (archive / "MUJOCO_LOG.TXT").read_text() == "warn\n"
    placements = {m["path"]: m["placement"] for m in receipt["moved"]}
    assert placements == {"outputs/run-1": "same-path", "MUJOCO_LOG.TXT": "retired-worktrees"}
    assert receipt["moved_entries"] == 4
    run1 = next(m for m in receipt["moved"] if m["path"] == "outputs/run-1")
    assert run1["files"] == 2 and run1["links"] == 1
    assert run1["manifest_sha256"] == tree_manifest.digest(tree_manifest.build(repo / "outputs/run-1"))
    rows = (archive / "MANIFEST.tsv").read_text().splitlines()
    jpg = next(r.split("\t") for r in rows if r.endswith(tree_manifest.file_sha256(repo / "outputs/run-1/frames/00001.jpg")))
    assert jpg[0] == "outputs/run-1" and jpg[2] == "frames/00001.jpg" and jpg[4] == "5000"
    assert json.loads((archive / "RETIRED.json").read_text())["head"] == receipt["head"]
    log = (repo / "outputs/retired-worktrees/retirements.jsonl").read_text().splitlines()
    assert json.loads(log[-1])["worktree"] == str(path)
    assert not path.exists()
    assert str(path) not in run(repo, "worktree", "list")
    assert run(repo, "rev-parse", "--verify", "kiro/done")  # branch kept


@needs_lsof
def test_retire_colliding_output_name_goes_to_archive_folder(repo, capsys):
    (repo / "outputs/run-1").mkdir(parents=True)
    (repo / "outputs/run-1/other.json").write_text("{}\n")
    path = make_worktree_with_outputs(repo, "kiro-clash", "kiro/clash")
    assert retire(repo, path, "--execute") == 0
    receipt = json.loads(capsys.readouterr().out)
    assert {m["path"]: m["placement"] for m in receipt["moved"]}["outputs/run-1"] == "retired-worktrees"
    assert sorted(p.name for p in (repo / "outputs/run-1").iterdir()) == ["other.json"]  # untouched
    moved = repo / "outputs/retired-worktrees/kiro-clash/outputs/run-1/result.json"
    assert moved.read_text() == '{"ok": true}\n'


@needs_lsof
def test_retire_archive_layout_keeps_everything_under_label(repo, capsys):
    path = make_worktree_with_outputs(repo, "kiro-arch", "kiro/arch")
    assert retire(repo, path, "--execute", "--archive-layout") == 0
    receipt = json.loads(capsys.readouterr().out)
    assert {m["placement"] for m in receipt["moved"]} == {"retired-worktrees"}
    assert (repo / "outputs/retired-worktrees/kiro-arch/outputs/run-1/result.json").exists()
    assert not (repo / "outputs/run-1").exists()


@needs_lsof
def test_retire_hash_mismatch_keeps_worktree_and_writes_failure_receipt(repo, capsys, monkeypatch):
    path = make_worktree_with_outputs(repo, "kiro-corrupt", "kiro/corrupt")
    real_build = tree_manifest.build
    calls = {"n": 0}

    def corrupted_after(root):
        calls["n"] += 1
        manifest = real_build(root)
        if calls["n"] == 2:  # the first "after" manifest: pretend one byte changed on disk
            rel = next(r for r, e in manifest.items() if e["type"] == "file")
            manifest[rel] = {**manifest[rel], "sha256": "0" * 64}
        return manifest

    monkeypatch.setattr(tree_manifest, "build", corrupted_after)
    assert retire(repo, path, "--execute") == 1
    out = json.loads(capsys.readouterr().out)
    assert out["verified"] is False and "verification failed" in out["error"]
    assert path.exists() and str(path) in run(repo, "worktree", "list")
    receipt = json.loads((repo / "outputs/retired-worktrees/kiro-corrupt/RETIRED.json").read_text())
    assert receipt["verified"] is False
    log = (repo / "outputs/retired-worktrees/retirements.jsonl").read_text().splitlines()
    assert json.loads(log[-1])["verified"] is False


@needs_lsof
def test_retire_refuses_unmerged_dirty_busy_and_existing_destination(repo):
    unmerged = make_worktree_with_outputs(repo, "kiro-open", "kiro/open")
    (unmerged / "new.txt").write_text("work\n")
    run(unmerged, "add", "new.txt")
    run(unmerged, "commit", "-q", "-m", "open work")
    assert retire(repo, unmerged, "--execute") == 2
    assert (unmerged / "outputs/run-1/result.json").exists()

    dirty = make_worktree_with_outputs(repo, "kiro-dirty", "kiro/dirty")
    (dirty / "notes.txt").write_text("untracked\n")
    assert retire(repo, dirty, "--execute") == 2
    assert (dirty / "outputs/run-1/result.json").exists()

    busy = make_worktree_with_outputs(repo, "kiro-busy", "kiro/busy")
    sleeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], cwd=busy)
    try:
        deadline = time.monotonic() + 5
        while not guard.processes_using(busy) and time.monotonic() < deadline:
            time.sleep(0.1)
        assert retire(repo, busy, "--execute") == 2
        assert (busy / "outputs/run-1/result.json").exists()
    finally:
        sleeper.kill()
        sleeper.wait()

    taken = make_worktree_with_outputs(repo, "kiro-taken", "kiro/taken")
    (repo / "outputs/retired-worktrees/kiro-taken").mkdir(parents=True)
    assert retire(repo, taken, "--execute") == 2
    assert (taken / "outputs/run-1/result.json").exists()
    assert retire(repo, repo, "--execute") == 2


@needs_lsof
def test_retire_refuses_worktree_named_by_a_running_command(repo):
    path = make_worktree_with_outputs(repo, "kiro-named", "kiro/named")
    # cwd elsewhere; only the command line names the worktree (e.g. an agent job's prompt), and only
    # after 300 characters, so a ps that truncates the command column would miss it.
    sleeper = subprocess.Popen([sys.executable, "-c", "import time, sys; time.sleep(60)", "p" * 300, f"{path}/x"],
                               cwd=repo.parent)
    try:
        deadline = time.monotonic() + 5
        while not guard.processes_naming(path) and time.monotonic() < deadline:
            time.sleep(0.1)
        assert retire(repo, path, "--execute") == 2
        assert (path / "outputs/run-1/result.json").exists()
    finally:
        sleeper.kill()
        sleeper.wait()
    assert retire(repo, path, "--execute") == 0


@needs_lsof
@pytest.mark.parametrize("value", ["nan", "inf", "-1"])
def test_idle_window_must_be_finite_and_non_negative(repo, value):
    path = make_worktree_with_outputs(repo, "kiro-idle", "kiro/idle")
    args = ["--primary", str(repo), "retire", str(path), "--no-fetch", "--no-pr-check", "--base", "main",
            "--execute", "--idle-minutes", value]
    assert aw.main(args) == 2 and (path / "outputs/run-1/result.json").exists()


@needs_lsof
def test_recently_modified_worktree_is_refused_by_default(repo):
    path = make_worktree_with_outputs(repo, "kiro-fresh", "kiro/fresh")
    args = ["--primary", str(repo), "retire", str(path), "--no-fetch", "--no-pr-check", "--base", "main", "--execute"]
    assert aw.main(args) == 2 and (path / "outputs/run-1/result.json").exists()
    old = time.time() - 2 * 3600
    gitdir = Path(run(path, "rev-parse", "--absolute-git-dir").strip())
    for target in [*path.rglob("*"), gitdir / "index", gitdir / "HEAD", gitdir / "logs/HEAD"]:
        if target.exists() or target.is_symlink():
            os.utime(target, (old, old), follow_symlinks=False)
    assert aw.main(args) == 0 and not path.exists()


@needs_lsof
def test_unmerged_worktree_retires_only_after_exact_head_is_archived(repo, tmp_path, capsys):
    remote = tmp_path / "remote.git"
    run(tmp_path, "init", "-q", "--bare", str(remote))
    run(repo, "remote", "add", "origin", str(remote))
    path = make_worktree_with_outputs(repo, "codex-work", "codex/work")
    (path / "work.txt").write_text("unmerged\n")
    run(path, "add", "work.txt")
    run(path, "commit", "-q", "-m", "unmerged work")
    head = run(path, "rev-parse", "HEAD").strip()
    assert retire(repo, path, "--execute") == 2  # not merged, no archive
    assert retire(repo, path, "--execute", "--archive-ref", "codex/archive-work-0926") == 2  # ref missing
    run(repo, "push", "-q", "origin", "main:refs/heads/codex/archive-work-0926")  # wrong commit
    assert retire(repo, path, "--execute", "--archive-ref", "codex/archive-work-0926") == 2
    assert (path / "outputs/run-1/result.json").exists()
    run(repo, "push", "-q", "-f", "origin", f"{head}:refs/heads/codex/archive-work-0926")
    capsys.readouterr()
    assert retire(repo, path, "--execute", "--archive-ref", "codex/archive-work-0926") == 0
    receipt = json.loads(capsys.readouterr().out)
    assert "archived at origin refs/heads/codex/archive-work-0926" in receipt["merged_evidence"]
    assert not path.exists() and (repo / "outputs/run-1/result.json").exists()


@needs_lsof
def test_retire_respects_open_pull_requests(repo, monkeypatch):
    path = make_worktree_with_outputs(repo, "kiro-pr", "kiro/pr")
    args = ["--primary", str(repo), "retire", str(path), "--no-fetch", "--base", "main", "--execute",
            "--idle-minutes", "0"]
    monkeypatch.setattr(aw, "open_pr_numbers", lambda primary, branch: [123])
    assert aw.main(args) == 2 and (path / "outputs/run-1/result.json").exists()
    monkeypatch.setattr(aw, "open_pr_numbers", lambda primary, branch: None)  # gh cannot answer
    assert aw.main(args) == 2 and (path / "outputs/run-1/result.json").exists()
    monkeypatch.setattr(aw, "open_pr_numbers", lambda primary, branch: [])
    assert aw.main(args) == 0 and not path.exists()


def test_names_path_matches_whole_paths_only():
    target = "/p/ugrp-wt/zone-owncam-loop"
    assert guard.names_path(f"run --cwd {target} x", target)
    assert guard.names_path(f"cd {target}/outputs && go", target)
    assert guard.names_path(target, target)
    assert not guard.names_path(f"cd {target}-v2 && go", target)
    assert not guard.names_path(f"cd {target}.frozen", target)
    assert guard.names_path(f"{target}-v2 and {target} both", target)
    assert not guard.names_path("", target)


def test_codex_app_worktree_label_and_owner():
    row = {"path": Path("/Users/x/.codex/worktrees/faster-dispatch/ugrp"), "branch": "codex/faster-dispatch"}
    assert aw.infer_owner(row) == ("codex", "branch")
    assert aw.default_label(row["path"], "codex") == "codex-faster-dispatch"
    assert aw.default_label(Path("/p/ugrp-wt/kiro-disk-rules"), "kiro") == "kiro-disk-rules"
    assert aw.infer_owner({"path": Path("/p/ugrp-wt/zone-m2-pair-s1-frozen"), "branch": None}) == ("claude", "path")
    assert aw.is_cache("scripts/__pycache__/") and aw.is_cache(".pytest_cache/") and not aw.is_cache("outputs/")


def test_sparse_profile_keeps_every_archive_that_code_reads():
    patterns = aw.sparse_patterns()
    assert patterns[0] == "/*"
    # Re-includes must come after the exclusions (last matching pattern wins).
    assert patterns[-len(aw.RUNTIME_KEEP):] == [f"/{p}" for p in aw.RUNTIME_KEEP]
    tracked = set(run(ROOT, "ls-files", "--", "experiments").splitlines())
    assert set(aw.RUNTIME_KEEP) <= tracked
    heavy = re.compile(r"experiments/[A-Za-z0-9_./-]+\.(?:%s)\b" % "|".join(aw.HEAVY_SUFFIXES))
    referenced = set()
    for folder in ("harness", "sim", "scripts"):
        for source in (ROOT / folder).rglob("*.py"):
            text = source.read_text(errors="ignore")
            referenced.update(match.group(0) for match in heavy.finditer(text))
    assert referenced <= set(aw.RUNTIME_KEEP), sorted(referenced - set(aw.RUNTIME_KEEP))
