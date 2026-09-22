"""Single-submitter, single CPU Colab session; finite deadline and no retries.

Uses the established Colab sparse-source setup and artifact verification path.
Only an explicitly prepared evidence capsule is uploaded, never a workspace.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import uuid
import zipfile

from harness.rgb_communication_study import (
    ContractError, bounded_process, checked_reference, digest_file, preflight,
    read_json, write_new_json,
)
from scripts.colab_simulation_cli import pack, setup_code, job_code

ROOT = Path(__file__).resolve().parents[1]


def pack_inputs(evidence_root: Path, inventory: dict, archive: Path) -> str:
    if not isinstance(inventory, dict) or not inventory or "capsule-inventory.json" in inventory:
        raise ContractError("explicit nonempty capsule inventory required")
    paths = {name: checked_reference(evidence_root, {"path": name, "sha256": sha})
             for name, sha in inventory.items()}
    if "manifest.json" not in paths:
        raise ContractError("capsule must contain the frozen manifest.json")
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as bundle:
        for name, path in paths.items():
            bundle.write(path, name)
        bundle.writestr("capsule-inventory.json", json.dumps(inventory, sort_keys=True))
    return digest_file(archive)


def input_setup_code(remote: str, archive: str, sha: str) -> str:
    return f'''from pathlib import Path
import hashlib, json, zipfile
archive = Path({archive!r})
assert hashlib.sha256(archive.read_bytes()).hexdigest() == {sha!r}, 'input archive hash mismatch'
destination = Path({remote!r})/'source/outputs/study-inputs'
destination.mkdir(parents=True, exist_ok=False)
with zipfile.ZipFile(archive) as bundle:
    names = bundle.namelist()
    inventory = json.loads(bundle.read('capsule-inventory.json'))
    assert len(names) == len(set(names)) and set(names) == set(inventory)|{{'capsule-inventory.json'}}
    for name, expected in inventory.items():
        path = Path(name)
        assert not path.is_absolute() and '..' not in path.parts
        data = bundle.read(name)
        assert hashlib.sha256(data).hexdigest() == expected, 'input member hash mismatch'
        target = destination/path
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as stream:
            stream.write(data)
print('UGRP_STUDY_INPUTS_VERIFIED')
'''


def verify_retrieval(archive: Path, expected_archive_sha: str, source_sha: str) -> dict:
    if digest_file(archive) != expected_archive_sha:
        raise ContractError("recovered archive hash mismatch")
    with zipfile.ZipFile(archive) as bundle:
        names = bundle.namelist()
        if len(names) != len(set(names)):
            raise ContractError("duplicate recovered archive members")
        for name in names:
            if Path(name).is_absolute() or ".." in Path(name).parts:
                raise ContractError("unsafe recovered archive path")
        report = json.loads(bundle.read("run.json"))
        if report.get("source_sha") != source_sha:
            raise ContractError("recovered source SHA mismatch")
        for name, expected in report["artifacts"].items():
            if hashlib.sha256(bundle.read(name)).hexdigest() != expected:
                raise ContractError("recovered artifact hash mismatch")
    return report


def submit(evidence_root: Path, inventory: dict, *, root: Path, output: Path) -> dict:
    manifest = read_json(evidence_root / "manifest.json")
    readiness = preflight(manifest, root=root, evidence_root=evidence_root)
    if not readiness["ready"]:
        raise ContractError("submission blocked: " + ", ".join(readiness["blockers"]))
    # Current authorization is CPU, exactly the two physical replays, no LLM.
    if manifest["stage"] != "physical_replay":
        raise ContractError("this submitter is limited to the authorized two physical replays")
    output.mkdir(parents=True, exist_ok=False)
    write_new_json(output / "preflight.json", readiness)
    archive = output / "inputs.zip"
    input_sha = pack_inputs(evidence_root, inventory, archive)
    source = pack(root, output / "transport")
    if source["source_sha"] != manifest["source"]["git_sha"]:
        raise ContractError("source changed before submission")
    session = "ugrp-d2-" + uuid.uuid4().hex[:12]
    remote = "/content/" + session
    remote_inputs = remote + "-inputs.zip"
    setup = output / "setup.py"
    with setup.open("x") as stream:
        stream.write(setup_code(remote, source["sha256"]))
        stream.write(input_setup_code(remote, remote_inputs, input_sha))
    job = output / "job.py"
    args = ["run", "--manifest", "outputs/study-inputs/manifest.json",
            "--evidence-root", "outputs/study-inputs", "--output", "{output}"]
    limit = manifest["config"]["budgets"]["job_wall_time_s"]
    started = time.monotonic()
    deadline = started + limit
    steps, attempted_creation = [], False
    result = {"session": session, "source_sha": source["source_sha"], "status": "incomplete",
              "automatic_retry": False, "deadline_s": limit, "input_archive_sha256": input_sha}

    def command(label, argv, cap=None):
        remaining = deadline - time.monotonic() - 35  # reserve owned-session shutdown
        if remaining <= 0:
            raise TimeoutError("job wall budget exhausted")
        record = bounded_process(argv, cwd=root, log_path=output / (label + ".log"),
                                 timeout_s=min(remaining, cap if cap else remaining))
        steps.append({"step": label, **record})
        if record["exit_code"]:
            raise RuntimeError("Colab step failed")

    try:
        attempted_creation = True
        command("new", ["colab", "new", "-s", session], 120)
        command("source-upload", ["colab", "upload", "-s", session,
                                  str(output / "transport/source.tar.gz"), remote + ".tar.gz"], 120)
        command("input-upload", ["colab", "upload", "-s", session, str(archive), remote_inputs], 120)
        command("setup", ["colab", "exec", "-s", session, "-f", str(setup), "--timeout", "900"], 905)
        remaining = deadline - time.monotonic() - 100
        if remaining < 30:
            raise TimeoutError("insufficient remaining job budget")
        with job.open("x") as stream:
            stream.write(job_code(remote, "scripts.run_rgb_communication_study",
                                 args + ["--allocation-wall-s", str(remaining - 20)]))
        command("run", ["colab", "exec", "-s", session, "-f", str(job), "--timeout", str(remaining)], remaining + 5)
        command("download", ["colab", "download", "-s", session,
                             remote + "/source/outputs/cli-job.zip", str(output / "result.zip")], 45)
        command("download-sha", ["colab", "download", "-s", session,
                                 remote + "/source/outputs/cli-job.zip.sha256", str(output / "result.zip.sha256")], 15)
        report = verify_retrieval(output / "result.zip", (output / "result.zip.sha256").read_text().split()[0],
                                  source["source_sha"])
        write_new_json(output / "verified-result.json", report)
        result.update(status="recovered", remote_exit_code=report["exit_code"],
                      recovered_archive_sha256=digest_file(output / "result.zip"))
    except (OSError, ValueError, RuntimeError, TimeoutError, KeyboardInterrupt, subprocess.SubprocessError) as exc:
        result.update(status="blocked_or_failed", error_type=type(exc).__name__,
                      failed_step=steps[-1]["step"] if steps else "provisioning")
    finally:
        if attempted_creation:
            # Only our freshly generated unique session; never touch other jobs.
            try:
                result["session_stop"] = bounded_process(["colab", "stop", "-s", session], cwd=root,
                    log_path=output / "stop.log", timeout_s=max(1, min(30, deadline-time.monotonic()-2)))
            except OSError as exc:
                result["session_stop"] = {"error_type": type(exc).__name__, "verified": False}
        result.update(steps=steps, wall_time_s=time.monotonic()-started)
        write_new_json(output / "submission.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = submit(args.evidence_root, read_json(args.inventory), root=ROOT, output=args.output)
    except (ContractError, OSError) as exc:
        parser.exit(2, f"submission blocked: {type(exc).__name__}\n")
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["status"] == "recovered" else 2


if __name__ == "__main__":
    raise SystemExit(main())
