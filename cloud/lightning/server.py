from __future__ import annotations

import argparse
import json
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from mujoco_world import MasterPiWorld

TOKEN = os.environ.get("UGRP_SIM_TOKEN", "").strip()
if not TOKEN:
    raise SystemExit("UGRP_SIM_TOKEN is required")


class State:
    def __init__(self, seed: int):
        self.world = MasterPiWorld(seed=seed)
        self.lock = threading.RLock()
        self.started = time.time()

    def jpeg(self) -> bytes:
        with self.lock:
            return self.world.render_jpeg()

    def snapshot(self) -> dict:
        with self.lock:
            return self.world.state()

    def act(self, name: str) -> dict:
        with self.lock:
            result = self.world.act(name)
            return {
                "ok": bool(result.ok),
                "action": result.action,
                "reason": result.reason,
                "state": result.state,
            }


class Handler(BaseHTTPRequestHandler):
    state: State

    def log_message(self, fmt, *args):
        print("sim-http", fmt % args, flush=True)

    def _authorized(self) -> bool:
        supplied = self.headers.get("X-UGRP-Sim-Token", "")
        return secrets.compare_digest(supplied, TOKEN)

    def _deny(self):
        self._json(401, {"error": "unauthorized"})

    def _json(self, code: int, obj: dict):
        raw = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _jpeg(self, image: bytes):
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(image)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(image)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/health":
            # No state or secret data on unauthenticated health endpoint.
            self._json(200, {"ok": True, "service": "ugrp-mujoco"})
            return
        if not self._authorized():
            self._deny()
            return
        if path == "/state":
            self._json(200, self.state.snapshot())
            return
        if path == "/snapshot":
            self._jpeg(self.state.jpeg())
            return
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    frame = self.state.jpeg()
                    header = (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode()
                        + b"\r\n\r\n"
                    )
                    self.wfile.write(header + frame + b"\r\n")
                    self.wfile.flush()
                    time.sleep(0.20)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return
        self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        if not self._authorized():
            self._deny()
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._json(400, {"error": "invalid json"})
            return
        if path == "/command":
            action = str(body.get("action") or "").lower()
            if action not in {"approach", "track", "pick", "carry", "fetch", "reset"}:
                self._json(400, {"error": "unsupported action"})
                return
            self._json(200, self.state.act(action))
            return
        self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=7860)
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args()
    state = State(args.seed)
    handler = type("BoundHandler", (Handler,), {"state": state})
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"UGRP MuJoCo cloud server http://{args.host}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
