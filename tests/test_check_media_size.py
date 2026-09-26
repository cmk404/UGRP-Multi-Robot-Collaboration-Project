"""Warning-only media budget check (scripts/check_media_size.py) and its pre-commit hook."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import check_media_size as cms  # noqa: E402

MIB = 1 << 20


def git(cwd: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(cwd), *args], check=True, capture_output=True, text=True).stdout


@pytest.fixture
def repo(tmp_path, monkeypatch):
    for key, value in {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@example.invalid",
                       "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@example.invalid",
                       "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1"}.items():
        monkeypatch.setenv(key, value)
    git(tmp_path, "init", "-q", "-b", "main")
    (tmp_path / "experiments/old").mkdir(parents=True)
    (tmp_path / "experiments/old/legacy.zip").write_bytes(os.urandom(6 * MIB))  # grandfathered
    git(tmp_path, "add", "-A")
    git(tmp_path, "commit", "-q", "-m", "base")
    monkeypatch.chdir(tmp_path)
    return tmp_path


def write(repo: Path, rel: str, size: int) -> None:
    (repo / rel).parent.mkdir(parents=True, exist_ok=True)
    (repo / rel).write_bytes(os.urandom(size))


def test_small_records_and_media_pass(repo, capsys):
    write(repo, "experiments/new/results.json", 200_000)
    write(repo, "experiments/new/media/plot.png", 300_000)
    git(repo, "add", "-A")
    assert cms.main(["--staged", "--strict"]) == 0
    assert capsys.readouterr().err == ""


def test_large_media_file_warns_without_blocking(repo, capsys):
    write(repo, "experiments/new/frames.zip", 2 * MIB)
    git(repo, "add", "-A")
    assert cms.main(["--staged"]) == 0
    err = capsys.readouterr().err
    assert "experiments/new/frames.zip" in err and "exceeds 1 MiB" in err
    assert cms.main(["--staged", "--strict"]) == 1


def test_experiment_media_budget_and_any_large_file(repo, capsys):
    for index in range(6):
        write(repo, f"experiments/many/media/{index}.jpg", MIB - 1000)
    write(repo, "docs/big.json", 6 * MIB)
    git(repo, "add", "-A")
    assert cms.main(["--staged", "--github-annotations"]) == 0
    captured = capsys.readouterr()
    assert "experiments/many/ holds 6.0 MiB of media" in captured.err
    assert "docs/big.json" in captured.err and "exceeds 5 MiB" in captured.err
    assert "::warning file=experiments/many::" in captured.out


def test_grandfathered_experiment_only_warns_when_media_grows(repo, capsys):
    write(repo, "experiments/old/notes.md", 1000)
    git(repo, "add", "-A")
    assert cms.main(["--staged", "--strict"]) == 0
    write(repo, "experiments/old/extra.png", 100_000)
    git(repo, "add", "-A")
    assert cms.main(["--staged", "--strict"]) == 1
    assert "experiments/old/ holds" in capsys.readouterr().err


def test_base_mode_checks_pull_request_commits(repo, capsys):
    base = git(repo, "rev-parse", "HEAD").strip()
    write(repo, "experiments/pr/video.mp4", 3 * MIB)
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "pr")
    assert cms.main(["--base", base, "--strict"]) == 1
    assert "experiments/pr/video.mp4" in capsys.readouterr().err


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash hook")
def test_pre_commit_hook_warns_but_keeps_20_mib_block(repo):
    hooks = repo / ".githooks"
    hooks.mkdir()
    shutil.copy2(ROOT / ".githooks/pre-commit", hooks / "pre-commit")
    (repo / "scripts").mkdir()
    for name in ("check_media_size.py", "agent_worktree.py", "tree_manifest.py", "worktree_guard.py"):
        shutil.copy2(ROOT / "scripts" / name, repo / "scripts" / name)
    git(repo, "config", "core.hooksPath", ".githooks")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "tools")
    write(repo, "experiments/new/frames.zip", 2 * MIB)
    git(repo, "add", "-A")
    result = subprocess.run(["git", "commit", "-m", "media"], cwd=repo, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "WARNING (media budget)" in result.stderr
    write(repo, "experiments/new/huge.bin", 21 * MIB)
    git(repo, "add", "-f", "experiments/new/huge.bin")
    result = subprocess.run(["git", "commit", "-m", "huge"], cwd=repo, capture_output=True, text=True)
    assert result.returncode == 1 and "exceeds 20 MiB" in result.stderr
