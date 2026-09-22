"""A2 independent offline clock and writer-finalization impact probe."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading

parser = argparse.ArgumentParser()
parser.add_argument("--source-root", type=Path, required=True)
parser.add_argument("--expected-sha", required=True)
parser.add_argument("--worker-child", type=Path)
parser.add_argument("--delay", type=float, default=.15)
parser.add_argument("--ignore-term", action="store_true")
args = parser.parse_args()
root = args.source_root.resolve()
sys.path[:0] = [str(root), str(root / "tests")]
import test_rgb_execution_skills as fixture
import harness.rgb_skill_execution as skills
from harness.rgb_communication_async import run_rgb_communication_async, AsyncRuntimeLimits, OfflineDecisionPlanner
from harness.rgb_communication_study import bounded_process
from harness.rgb_communication_evaluation import evaluate_run

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

if args.worker_child:
    if args.ignore_term:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    output = args.worker_child
    output.mkdir()
    log = output / "worker-timing.jsonl"
    log.write_bytes(b"")
    entered, release = threading.Event(), threading.Event()
    class Waiting(fixture.Controller):
        def decide(self, own, top):
            entered.set()
            assert release.wait(30.)
            return super().decide(own, top)
    port, _, _, _ = fixture.build({"r2": Waiting()})
    def record(row):
        with log.open("a") as stream:
            stream.write(json.dumps(row) + "\n")
    port._record_supervisor = record
    fixture.submit(port, "r2")
    port.tick(0.)
    assert entered.wait(2.)
    port.close(None)
    (output / "close-returned.json").write_text(json.dumps({"sha256": digest(log), "port_closed": port._closed}))
    timer = threading.Timer(args.delay, release.set)
    timer.daemon = True
    timer.start()
    # Intentionally return without waiting: D owns the subprocess exit barrier.
    sys.exit(0)

sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip()
assert sha == args.expected_sha and not dirty
report = {"source_sha": sha, "scope": "A2 offline clock impact only",
          "physical_runs": 0, "model_calls": 0, "cases": [], "findings": [], "live_readiness": False}

for mode, partial in (("no_advance", 0.), ("partial_advance", .02), ("unknown_actual", .02)):
    port, scene = fixture.build_dispatch_float_clock()
    original = port._advance_physics
    def fail(delta):
        if port._now_s >= .75:
            if partial:
                scene.step(partial)
            if mode == "unknown_actual":
                def unavailable():
                    raise OSError("fixture clock unavailable")
                port._simulation_clock._read_absolute = unavailable
            raise OSError("fixture owner fault")
        original(delta)
    port._advance_physics = fail
    try:
        result = run_rgb_communication_async(skills._ActorFacade(port),
            {rid: OfflineDecisionPlanner(lambda req: {"action": {"kind": "wait"}, "message": None})
             for rid in ("r1", "r2", "r3")},
            condition="none", common_task=fixture.static_context()["task"],
            clock_snapshot=port.clock_snapshot,
            limits=AsyncRuntimeLimits(max_ticks=20, max_calls_per_robot=30,
                                     tick_period_s=.05, poll_period_s=.001, decision_period_s=1.))
        clock = result["clock"]
        assert result["outcome"] == "aborted" and result["termination_reason"] == "RUNTIME_ERROR"
        assert clock["requested_tick_s"] == .8
        assert clock["last_successful_requested_tick_s"] == .75
        assert math.isclose(clock["last_acknowledged_time_s"], .75, abs_tol=1e-12)
        actual = None if mode == "unknown_actual" else scene.world.data.time-port._simulation_clock.origin
        assert clock["terminal_time_s"] == actual
        assert result["events"][-1]["sim_time_s"] == actual
        assert port._closed and not port._active
        assert result["external_model_calls"] == 0
        report["cases"].append({"case": mode, "pass": True, "clock": clock})
    finally:
        port.close(None)

port, scene = fixture.build_dispatch_float_clock()
try:
    for i in range(3601):
        port.tick(i*.05)
    clock = port.clock_snapshot()
    bound = port._simulation_clock.roundoff_bound(180.)
    try:
        port.close(clock["actual_time_s"])
        accepted = True
        error = None
    except Exception as exc:
        accepted = False
        error = {"type": type(exc).__name__, "code": getattr(exc, "error_code", None)}
    report["cases"].append({"case": "close_raw_actual_at_budget", "pass": accepted,
        "requested_s": 180., "actual_s": clock["actual_time_s"], "roundoff_bound_s": bound, "error": error})
    if not accepted:
        report["findings"].append("Known actual time within accumulation roundoff fails close at exact requested SIM cap.")
finally:
    port.close(None)

port, scene = fixture.build_dispatch_float_clock()
try:
    port.tick(0.)
    previous = scene.world.data.time
    port.clock_snapshot()
    scene.world.data.time = math.nextafter(previous, -math.inf)
    snapshot = port.clock_snapshot()
    assert snapshot["actual_time_s"] is None
    assert not hasattr(skills._ActorFacade(port), "clock_snapshot")
    assert set(snapshot) == {"schema", "clock_domain", "last_acknowledged_time_s", "actual_time_s"}
    report["cases"].append({"case": "one_ulp_real_reverse_and_clock_only_fields", "pass": True})
finally:
    port.close(None)

with tempfile.TemporaryDirectory(prefix="ugrp-a2-writer-") as directory:
    directory = Path(directory)
    for mode, delay, timeout in (("normal_exit", .15, 5.), ("timeout_exit", 20., 1.5),
                                 ("forced_kill_exit", 20., 1.5)):
        output = directory / mode
        proc = bounded_process([sys.executable, str(Path(__file__).resolve()), "--source-root", str(root),
            "--expected-sha", sha, "--worker-child", str(output), "--delay", str(delay)]
            + (["--ignore-term"] if mode == "forced_kill_exit" else []),
            cwd=root, log_path=directory/(mode+".log"), timeout_s=timeout)
        log = output / "worker-timing.jsonl"
        closed = json.loads((output / "close-returned.json").read_text())
        final_hash = digest(log)
        rows = [json.loads(line) for line in log.read_text().splitlines()]
        threading.Event().wait(.05)
        assert digest(log) == final_hash
        if mode == "normal_exit":
            assert proc["exit_code"] == 0 and not proc["timed_out"]
            assert any(row["event"] == "RGB_WORKER_COMPLETED" for row in rows)
            assert closed["sha256"] != final_hash
        else:
            assert proc["timed_out"] and proc["exit_code"] == 124
            assert not rows
        report["cases"].append({"case": "D_writer_exit_barrier_"+mode, "pass": True,
            "process": proc, "changed_after_bundle_close": closed["sha256"] != final_hash,
            "hash_stable_after_process_exit": True,
            "completion_known": bool(rows), "final_log_sha256": final_hash})

raw = Path("/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/rgb-communication-colab-bfe478f/recovered/result/runs")
for kind in ("solo", "joint"):
    run = raw/(kind+"-physical-replay")
    before = {name: digest(run/name) for name in ("runtime.jsonl", "evaluator.json", "result.json")}
    scored = evaluate_run(json.loads((run/"plan.json").read_text()), run)
    assert scored["outcome"] == "invalid_artifact"
    assert scored["reason"] == "evaluator snapshot predates the runtime terminal event"
    assert json.loads((run/"result.json").read_text())["replay_goal_complete"] is None
    assert before == {name: digest(run/name) for name in before}
    report["cases"].append({"case": "old_"+kind+"_still_rejected_unchanged", "pass": True, "hashes": before})

assert subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip() == sha
assert not subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip()
report["passed"] = sum(row["pass"] for row in report["cases"])
report["failed"] = sum(not row["pass"] for row in report["cases"])
report["verdict"] = "no_go" if report["failed"] else "offline_impact_pass"
report["worker_finalization_scope"] = "bundle.close is not a byte freeze; D subprocess exit/reap precedes full hashes. Killed worker duration/completion remain unknown."
print(json.dumps(report, ensure_ascii=False))
