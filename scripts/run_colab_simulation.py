"""Run one bounded Colab job and preserve exit status, source and artifact hashes.

Use {output} in the command for a new result directory inside this job.
No cloud storage, model service or simulator is started implicitly.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(output: Path, command: list[str], *, root: Path = ROOT) -> int:
    if not command:
        raise ValueError("a command is required")
    output = output.resolve()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", output.name):
        raise ValueError("job name must contain only letters, numbers, hyphens or underscores")
    # Restrict archives to this run, never the repository or credentials.
    outputs = (root / "outputs").resolve()
    if not output.is_relative_to(outputs) or output == outputs:
        raise ValueError("job directory must be inside repository outputs/")
    source = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root, text=True)
    if dirty.strip():
        raise ValueError("commit tracked source changes before running")
    archive = output.with_suffix(".zip")
    if archive.exists():
        raise FileExistsError(archive)
    output.mkdir(parents=True, exist_ok=False)
    command = [part.replace("{output}", str(output / "result")) for part in command]
    record = {
        "source_sha": source, "command": command, "python": sys.version,
        "platform": platform.platform(), "render_backend": os.environ.get("MUJOCO_GL"),
        "started_at_unix": time.time(), "status": "running",
        "scope": "Process exit status is not robot task success. Raw files require separate review.",
    }
    manifest = output / "run.json"
    manifest.write_text(json.dumps(record, indent=2) + "\n")
    packages = subprocess.run([sys.executable, "-m", "pip", "freeze"], capture_output=True, text=True)
    (output / "packages.txt").write_text(packages.stdout)
    record["package_capture_exit_code"] = packages.returncode
    session = "colab-" + output.name
    code = 1
    try:
        with (output / "run.log").open("w") as log:
            child = subprocess.Popen(
                [sys.executable, str(root / "scripts/ugrp_session.py"), "run", session, "--", *command],
                cwd=root, stdout=log, stderr=subprocess.STDOUT,
            )
            try:
                code = child.wait()
            except KeyboardInterrupt:
                # Stop only the process group registered by this session.
                subprocess.run([sys.executable, str(root / "scripts/ugrp_session.py"), "stop", session], cwd=root)
                child.wait()
                code = 130
    finally:
        record.update(exit_code=code, status="complete" if code == 0 else "failed", finished_at_unix=time.time())
        files = [p for p in sorted(output.rglob("*")) if p.is_file() and not p.is_symlink() and p != manifest]
        record["artifacts"] = {str(p.relative_to(output)): sha256(p) for p in files}
        manifest.write_text(json.dumps(record, indent=2) + "\n")
        with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as bundle:
            for path in [*files, manifest]:
                bundle.write(path, str(path.relative_to(output)))
        archive.with_suffix(".zip.sha256").write_text(sha256(archive) + "  " + archive.name + "\n")
        print(json.dumps({"exit_code": code, "archive": str(archive), "manifest": str(manifest)}), flush=True)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    try:
        return run(args.output, command)
    except (ValueError, FileExistsError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
