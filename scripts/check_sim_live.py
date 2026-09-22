"""Exercise the actual local viewer, cameras and controls, then shut it down."""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import sys
import time
from urllib.request import Request, urlopen

from PIL import Image, ImageStat

ROOT = Path(__file__).resolve().parents[1]


def wait_until(fn, *, timeout=45):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = fn()
        if result:
            return result
        time.sleep(.1)
    raise TimeoutError("viewer did not reach the expected state")


def check(output):
    output.mkdir(parents=True, exist_ok=False)
    runtime = output / "runtime"
    report = {"scope": "Live camera/control integration only; no autonomous task evaluation."}
    with (output / "server.log").open("w") as log:
        process = subprocess.Popen([sys.executable, "-m", "scripts.sim_live", "--port", "0",
            "--duration", "120", "--output", str(runtime)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            def started():
                if process.poll() is not None:
                    raise RuntimeError("viewer exited during startup; inspect server.log")
                path = runtime / "session.json"
                if path.exists():
                    try:
                        return json.loads(path.read_text())
                    except json.JSONDecodeError:
                        pass
            session = wait_until(started)
            url = session["url"]
            def status():
                with urlopen(url + "/api/status", timeout=3) as response:
                    return json.load(response)
            wait_until(lambda: status()["phase"] == "paused")
            page = urlopen(url, timeout=3).read().decode()
            token = re.search(r"const controlToken='([^']+)'", page).group(1)
            def command(body):
                request = Request(url + "/api/control", data=json.dumps(body).encode(),
                    headers={"Content-Type": "application/json", "X-UGRP-Control": token})
                with urlopen(request, timeout=3) as response:
                    receipt = json.load(response)
                if body["action"] != "shutdown":
                    wait_until(lambda: status()["last_command"] == receipt["accepted"])
            def frame(camera, name):
                data = urlopen(url + "/frame/" + camera, timeout=3).read()
                image = Image.open(io.BytesIO(data)).convert("RGB")
                assert image.size == (384, 288), image.size
                assert max(ImageStat.Stat(image).stddev) > 3, "blank camera"
                (output / (name + ".jpg")).write_bytes(data)
                return hashlib.sha256(data).hexdigest()
            report["initial_frames"] = {camera: frame(camera, "before-" + camera) for camera in ("overview", "r1", "r2", "r3")}
            initial = status()
            command({"action": "demo"})
            wait_until(lambda: status()["sim_time_s"] >= initial["sim_time_s"] + 1.2)
            command({"action": "pause"})
            paused = status()
            time.sleep(.4)
            assert status()["sim_time_s"] == paused["sim_time_s"], "pause must freeze physics"
            wait_until(lambda: status()["frame_sequence"] > paused["frame_sequence"])
            report["moved_frames"] = {camera: frame(camera, "after-" + camera) for camera in ("overview", "r1", "r2", "r3")}
            assert report["initial_frames"]["r1"] != report["moved_frames"]["r1"], "R1 camera did not change"
            assert report["initial_frames"]["overview"] != report["moved_frames"]["overview"], "overview did not change"
            command({"action": "drive", "robot": "r2", "direction": "left"})
            wait_until(lambda: status()["sim_time_s"] >= paused["sim_time_s"] + .6)
            command({"action": "stop_motion"})
            command({"action": "reset", "seed": 42})
            reset = status()
            assert reset["seed"] == 42 and reset["episode"] == 2 and reset["phase"] == "paused", reset
            report.update(source_sha=session["source_sha"], moved_sim_s=paused["sim_time_s"] - initial["sim_time_s"],
                          pause_verified=True, reset_verified=True, four_cameras_verified=True)
            command({"action": "shutdown"})
            assert process.wait(timeout=15) == 0
            result = json.loads((runtime / "result.json").read_text())
            assert result["error"] is None and result["exit_reason"] == "shutdown", result
            report.update(shutdown_verified=True, result=result)
            print(json.dumps({"verified": str(output), "source_sha": session["source_sha"]}))
        except Exception as exc:
            report["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            (output / "verification.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    check(parser.parse_args().output.resolve())
