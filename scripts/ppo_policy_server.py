#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
from stable_baselines3 import PPO


class PolicyHandler(BaseHTTPRequestHandler):
    policy = None

    def log_message(self, fmt, *args):
        return

    def _json(self, code: int, obj: dict):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._json(200, {"ok": True, "model": "ppo_grasp_v2"})
            return
        self.send_error(404)

    def do_POST(self):
        if self.path != "/predict":
            self.send_error(404)
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            obj = json.loads(self.rfile.read(n) or b"{}")
            obs = np.asarray(obj.get("observation"), dtype=np.float32).reshape(21)
            action, _ = self.policy.predict(obs, deterministic=True)
            self._json(200, {"ok": True, "action": np.asarray(action, dtype=float).reshape(7).tolist()})
        except Exception as exc:
            self._json(400, {"ok": False, "error": str(exc)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--model", default="outputs/rl/grasp_ppo_v2.zip")
    a = ap.parse_args()
    PolicyHandler.policy = PPO.load(a.model, device="cpu")
    srv = HTTPServer((a.host, a.port), PolicyHandler)
    print(f"UGRP PPO policy server http://{a.host}:{a.port} model={a.model}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
