"""Read-only disk report (scripts/disk_report.py)."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import agent_worktree, disk_report  # noqa: E402


def git(cwd: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(cwd), *args], check=True, capture_output=True, text=True).stdout


def write(path: Path, size: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(os.urandom(size))


@pytest.fixture
def project(tmp_path, monkeypatch):
    for key, value in {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.invalid",
                       "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.invalid",
                       "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1"}.items():
        monkeypatch.setenv(key, value)
    primary = tmp_path / "ugrp"
    primary.mkdir()
    git(primary, "init", "-q", "-b", "main")
    (primary / ".gitignore").write_text("outputs/\n__pycache__/\n")
    write(primary / "experiments/a/media/frame.jpg", 300_000)
    (primary / "experiments/a/README.md").write_text("raw: outputs/zone-x-20260926/test (local)\n")
    (primary / "scripts").mkdir()
    (primary / "scripts/tool.py").write_text("print(1)\n")
    for runtime in agent_worktree.RUNTIME_KEEP:
        write(primary / runtime, 1000)
    git(primary, "add", "-A")
    git(primary, "commit", "-q", "-m", "init")
    write(primary / "outputs/zone-x-20260926/dev-a/frames/1.jpg", 200_000)
    write(primary / "outputs/zone-x-20260926/test/frames/1.jpg", 100_000)
    write(primary / "outputs/m1-dev-a1/log.json", 50_000)
    write(primary / "outputs/tensorboard/snap/events", 10_000)
    write(primary / "outputs/retired-worktrees/old/outputs/x.json", 20_000)
    full = tmp_path / "ugrp-wt" / "full"
    git(primary, "worktree", "add", "-q", "-b", "claude/full", str(full), "main")
    write(full / "outputs/run/result.json", 120_000)
    write(full / "scripts/__pycache__/m.pyc", 8_000)
    write(full / "notes.txt", 4_000)
    assert agent_worktree.main(["--primary", str(primary), "new", "kiro-sparse", "--branch", "kiro/sparse",
                                "--parent", str(tmp_path / "ugrp-wt"), "--no-fetch", "--base", "main"]) == 0
    (tmp_path / "home/.kiro/sessions").mkdir(parents=True)
    write(tmp_path / "home/.kiro/sessions/s.json", 30_000)
    return primary


def snapshot(root: Path, skip: Path) -> dict:
    state = {}
    for dirpath, _dirs, files in os.walk(root):
        for name in files:
            path = Path(dirpath) / name
            if path != skip:
                st = path.lstat()
                state[str(path)] = (st.st_size, st.st_mtime_ns)
    return state


def test_report_splits_worktrees_tiers_and_budget_without_writing(project, capsys):
    out = project.parent / "report.json"
    before = snapshot(project.parent, out)
    assert disk_report.main(["--primary", str(project), "--home", str(project.parent / "home"),
                             "--ref", "HEAD", "--no-pr-status", "--json", str(out)]) == 0
    assert snapshot(project.parent, out) == before  # read-only apart from --json
    report = json.loads(out.read_text())
    rows = {Path(r["path"]).name: r for r in report["worktrees"]["rows"]}
    full, sparse = rows["full"], rows["kiro-sparse"]
    assert full["owner"] == "claude" and full["merged"] and not full["sparse"]
    assert full["outputs"] >= 120_000 and full["caches"] >= 8_000 and full["untracked"] >= 4_000
    assert sparse["owner"] == "kiro" and sparse["sparse"] and sparse["tracked"] < full["tracked"] - 250_000
    assert report["worktrees"]["groups"]["all"]["count"] == 2
    tiers = {r["path"]: r for r in report["retention"]["rows"]}
    assert tiers["zone-x-20260926/dev-a"]["tier"] == "dev-diagnostic"
    assert tiers["zone-x-20260926/test"]["tier"] == "test-cohort"
    assert tiers["zone-x-20260926/test"]["record_reference"] is True
    assert tiers["m1-dev-a1"]["tier"] == "dev-diagnostic" and not tiers["m1-dev-a1"]["record_reference"]
    assert tiers["tensorboard"]["tier"] == "infrastructure"
    assert tiers["retired-worktrees"]["tier"] == "retired-worktrees"
    exts = {r["ext"]: r for r in report["experiments"]["by_ext"]}
    assert exts["jpg"]["bytes"] == 300_000 and report["experiments"]["heavy_media_bytes"] >= 300_000
    assert report["agents"][".kiro"]["bytes"] >= 30_000
    budget = report["budget"]
    assert budget["project_bytes"] == sum(row["bytes"] for row in budget["rows"].values())
    text = capsys.readouterr().out
    assert "[worktrees]" in text and "PR state unknown" in text and "[budget]" in text


def test_tier_rules():
    assert disk_report.tier_of("jev-skills-dev-v6") == "dev-diagnostic"
    assert disk_report.tier_of("dev-final") == "dev-diagnostic"  # dev wins over final
    assert disk_report.tier_of("jev-skills-holdout-v1") == "test-cohort"
    assert disk_report.tier_of("experiment-archives-20260907") == "archive"
    assert disk_report.tier_of("tensorboard-archive") == "infrastructure"
    assert disk_report.tier_of("plan-guidance-20260925") is None
