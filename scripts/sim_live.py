"""Local, bounded MuJoCo playground with live browser cameras; no model service."""
from __future__ import annotations

import argparse
import hmac
import json
import math
import queue
import secrets
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ("overview", "r1", "r2", "r3")
DRIVES = {"forward": (.12, 0.), "backward": (-.05, 0.),
          "left": (0., .12), "right": (0., -.12)}


def validate_command(value):
    if not isinstance(value, dict):
        raise ValueError("command must be an object")
    action = value.get("action")
    fields = {"play": set(), "pause": set(), "reset": {"seed"},
              "drive": {"robot", "direction"}, "stop_motion": set(),
              "demo": set(), "shutdown": set()}
    if not isinstance(action, str) or action not in fields:
        raise ValueError("unknown action")
    if set(value) != fields[action] | {"action"}:
        raise ValueError("incorrect command fields")
    if action == "reset":
        if type(value["seed"]) is not int or not 0 <= value["seed"] <= 1_000_000:
            raise ValueError("seed must be an integer between 0 and 1000000")
    if action == "drive":
        if value["robot"] not in CAMERAS[1:] or not isinstance(value["direction"], str) or value["direction"] not in DRIVES:
            raise ValueError("unknown robot or direction")
    return dict(value)


class LiveState:
    """HTTP threads own only copied status, JPEGs and a bounded command queue."""
    def __init__(self, seed):
        self.cv = threading.Condition()
        self.commands = queue.Queue(maxsize=16)
        self.stop = threading.Event()
        self.token = secrets.token_urlsafe(32)
        self.frames = {}
        self.sequence = 0
        self.updated_at = None
        self.last_visit = time.monotonic()
        self.status = {"phase": "starting", "seed": seed, "sim_time_s": 0.,
                       "episode": 0, "commands": 0, "error": None,
                       "last_command": None, "remaining_s": None}

    def snapshot(self):
        with self.cv:
            return {**self.status, "frame_sequence": self.sequence,
                    "frame_age_s": None if self.updated_at is None else round(time.monotonic() - self.updated_at, 2)}

    def update(self, **values):
        with self.cv:
            self.status.update(values)
            self.cv.notify_all()

    def publish(self, frames):
        with self.cv:
            self.frames = frames
            self.sequence += 1
            self.updated_at = time.monotonic()
            self.cv.notify_all()


def make_server(state, port):
    page = (ROOT / "sim/static/live_view.html").read_text(encoding="utf-8")
    page = page.replace("__CONTROL_TOKEN__", state.token).encode()

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3)

        def log_message(self, *_):
            pass  # Do not print control headers or per-frame requests.

        def host_ok(self):
            return self.headers.get("Host") in {
                f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}

        def reply(self, code, body, kind="application/json"):
            if isinstance(body, dict):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' blob:; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def do_GET(self):
            if not self.host_ok():
                return self.reply(403, {"error": "localhost access only"})
            path = urlsplit(self.path).path
            if path == "/":
                with state.cv:
                    state.last_visit = time.monotonic()
                return self.reply(200, page, "text/html; charset=utf-8")
            if path == "/api/status":
                with state.cv:
                    state.last_visit = time.monotonic()
                return self.reply(200, state.snapshot())
            if path.startswith("/frame/") and path.removeprefix("/frame/") in CAMERAS:
                with state.cv:
                    frame = state.frames.get(path.removeprefix("/frame/"))
                return self.reply(200, frame, "image/jpeg") if frame else self.reply(503, {"error": "camera starting"})
            return self.reply(404, {"error": "not found"})

        def do_POST(self):
            origin = self.headers.get("Origin")
            allowed = {f"http://127.0.0.1:{self.server.server_port}", f"http://localhost:{self.server.server_port}"}
            if (not self.host_ok() or (origin is not None and origin not in allowed)
                    or not hmac.compare_digest(self.headers.get("X-UGRP-Control", ""), state.token)):
                return self.reply(403, {"error": "open the local viewer to control this session"})
            if self.path != "/api/control":
                return self.reply(404, {"error": "not found"})
            if self.headers.get("Content-Type") != "application/json":
                return self.reply(415, {"error": "JSON required"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024:
                    raise ValueError("invalid body length")
                command = validate_command(json.loads(self.rfile.read(size)))
            except (ValueError, TypeError, UnicodeError):
                return self.reply(400, {"error": "invalid command"})
            if state.snapshot()["phase"] in {"starting", "resetting", "stopped", "error"}:
                return self.reply(409, {"error": "simulation is not ready"})
            command["id"] = secrets.token_hex(6)
            if command["action"] == "shutdown":
                state.stop.set()
            else:
                try:
                    state.commands.put_nowait(command)
                except queue.Full:
                    return self.reply(429, {"error": "command queue is full"})
            return self.reply(202, {"accepted": command["id"]})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


class Playground:
    """The main thread exclusively owns physics, controls and rendering."""
    def __init__(self, state, output, *, fps=4, duration=1800, world_factory=None):
        self.state, self.output = state, output
        self.fps, self.duration = fps, duration
        self.world_factory = world_factory
        self.world = None
        self.ports = {}
        self.paused = True
        self.seed = state.status["seed"]
        self.episode = 0
        self.command_count = 0
        self.total_sim_s = 0.
        self.log = None

    def reset(self, seed):
        from sim.camera_robot_port import CameraRobotPort
        self.state.update(phase="resetting")
        self.hold()
        if self.world is not None:
            self.world.close()
            self.world = None
        self.ports = {}
        if self.world_factory is None:
            from sim.multi_masterpi_production import MultiMasterPiProductionV2
            self.world_factory = MultiMasterPiProductionV2
        self.world = self.world_factory(seed=seed, width=384, height=288,
            render=True, warehouse_layout="camera_team")
        self.ports = {rid: CameraRobotPort(self.world, rid, allow_reverse=True) for rid in CAMERAS[1:]}
        self.seed, self.episode, self.paused = seed, self.episode + 1, True
        self.render()
        self.state.update(phase="paused", seed=seed, episode=self.episode,
                          sim_time_s=float(self.world.data.time))

    def hold(self):
        if self.world is not None:
            for port in self.ports.values():
                port.hold(float(self.world.data.time))

    def render(self):
        frames = {"overview": self.world.render_team_jpeg(camera="cctv_warehouse", quality=80)}
        frames.update({rid: self.world.render_jpeg(robot_id=rid, camera="robot_cam", quality=76) for rid in CAMERAS[1:]})
        self.state.publish(frames)

    def apply(self, command):
        action = command["action"]
        if action == "reset":
            self.reset(command["seed"])
        elif action in {"pause", "stop_motion"}:
            self.hold()
            if action == "pause":
                self.paused = True
        elif action == "play":
            self.paused = False
        elif action in {"drive", "demo"}:
            rid = "r1" if action == "demo" else command["robot"]
            forward, turn = DRIVES["forward" if action == "demo" else command["direction"]]
            self.ports[rid].apply({"kind": "drive", "forward": forward, "turn": turn,
                                  "duration_s": 1. if action == "demo" else .5}, float(self.world.data.time))
            self.paused = False
        self.command_count += 1
        self.state.update(phase="paused" if self.paused else "running",
                          commands=self.command_count, last_command=command["id"])
        if self.log:
            self.log.write(json.dumps({**command, "episode": self.episode,
                "sim_time_s": float(self.world.data.time)}) + "\n")
            self.log.flush()

    def advance(self, seconds):
        target = float(self.world.data.time) + seconds
        # Tick every <=20 ms so a browser disconnect cannot extend a wheel lease.
        while float(self.world.data.time) + 1e-9 < target and not self.state.stop.is_set():
            before = float(self.world.data.time)
            self.world.robot("r1").advance_to_sim_time(min(target, before + .02))
            now = float(self.world.data.time)
            for port in self.ports.values():
                port.tick(now)
            self.total_sim_s += now - before

    def run(self):
        start = time.monotonic()
        error, reason = None, "shutdown"
        try:
            with (self.output / "commands.jsonl").open("x") as self.log:
                self.reset(self.seed)
                last_step = last_render = time.monotonic()
                while not self.state.stop.is_set():
                    now = time.monotonic()
                    if now - start >= self.duration:
                        reason = "duration_limit"
                        break
                    # Drain a bounded batch; HTTP never touches the world.
                    for _ in range(16):
                        try:
                            command = self.state.commands.get_nowait()
                        except queue.Empty:
                            break
                        self.apply(command)
                    with self.state.cv:
                        idle = now - self.state.last_visit > 15
                    if idle and not self.paused:
                        self.hold()
                        self.paused = True
                        self.state.update(phase="paused")
                    if not self.paused:
                        self.advance(min(.04, max(0., now - last_step)))
                    last_step = now
                    if not idle and now - last_render >= 1 / self.fps:
                        self.render()
                        last_render = time.monotonic()
                    self.state.update(sim_time_s=round(float(self.world.data.time), 3),
                                      remaining_s=max(0, round(self.duration - (time.monotonic() - start))))
                    self.state.stop.wait(.01)
        except Exception as exc:
            error, reason = f"{type(exc).__name__}: {exc}", "error"
            self.state.update(phase="error", error=error)
        finally:
            self.log = None
            self.state.stop.set()
            try:
                self.hold()
            finally:
                if self.world is not None:
                    self.world.close()
            self.state.update(phase="error" if error else "stopped", error=error)
            with self.state.cv:
                frames = dict(self.state.frames)
            for camera, frame in frames.items():
                (self.output / f"{camera}.jpg").write_bytes(frame)
            result = {"mode": "local_interactive_physics_viewer", "seed": self.seed,
                "episodes": self.episode, "exit_reason": reason, "error": error,
                "wall_s": round(time.monotonic() - start, 3),
                "sim_time_s": round(self.total_sim_s, 3), "commands": self.command_count,
                "frame_batches": self.state.sequence, "model_calls": 0,
                "scope": "Manual camera/physics playground; not autonomous transport or communication evaluation."}
            (self.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return 1 if error else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765, help="localhost port; 0 chooses a free port")
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--fps", type=float, default=4, help="maximum camera refresh rate, 1–10")
    parser.add_argument("--duration", type=float, default=1800, help="finite wall-clock seconds, 1–7200")
    parser.add_argument("--output", type=Path, help="new log directory (generated by default)")
    a = parser.parse_args()
    if not 0 <= a.port <= 65535 or not 0 <= a.seed <= 1_000_000:
        parser.error("invalid port or seed")
    if not math.isfinite(a.fps) or not 1 <= a.fps <= 10:
        parser.error("fps must be between 1 and 10")
    if not math.isfinite(a.duration) or not 1 <= a.duration <= 7200:
        parser.error("duration must be between 1 and 7200 seconds")
    output = (a.output or ROOT / "outputs" / ("sim-live-" + datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + secrets.token_hex(3))).resolve()
    state = LiveState(a.seed)
    try:
        server = make_server(state, a.port)
    except OSError as exc:
        parser.error(f"cannot bind localhost port {a.port}: {exc}; try --port 0")
    try:
        output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        server.server_close()
        parser.error("output already exists; choose a new directory")
    metadata = {"source_sha": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                "source_dirty": bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT)),
                "seed": a.seed, "fps": a.fps, "duration_s": a.duration,
                "url": f"http://127.0.0.1:{server.server_port}", "output": str(output)}
    (output / "session.json").write_text(json.dumps(metadata, indent=2) + "\n")
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: state.stop.set())
    http = threading.Thread(target=server.serve_forever, daemon=True)
    http.start()
    print(f"UGRP LIVE {metadata['url']}\n전체 장면과 R1/R2/R3 카메라 · 모델 계정 불필요\n종료: 화면의 종료 버튼 또는 Ctrl-C (최대 {a.duration:g}초)\n기록: {output}", flush=True)
    try:
        return Playground(state, output, fps=a.fps, duration=a.duration).run()
    finally:
        server.shutdown()
        server.server_close()
        http.join(timeout=3)


if __name__ == "__main__":
    raise SystemExit(main())
