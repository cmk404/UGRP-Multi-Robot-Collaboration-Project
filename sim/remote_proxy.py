from __future__ import annotations

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
TOKEN_FILE = ROOT / ".sim_bridge_token"
URL_FILE = ROOT / ".sim_remote_url"


def load_secret(path: Path) -> str:
    value = path.read_text().strip() if path.exists() else ""
    if not value:
        raise SystemExit(f"missing {path.name}")
    return value


class Remote:
    def __init__(self, base: str, token: str):
        self.base = base.rstrip("/")
        self.token = token

    def request(self, path: str, *, payload=None, timeout=20) -> tuple[bytes, str]:
        data = None
        headers = {"X-UGRP-Sim-Token": self.token}
        method = "GET"
        if payload is not None:
            data = json.dumps(payload).encode()
            headers["Content-Type"] = "application/json"
            method = "POST"
        req = Request(self.base + path, data=data, headers=headers, method=method)
        with urlopen(req, timeout=timeout) as r:
            return r.read(), r.headers.get_content_type()


class Handler(BaseHTTPRequestHandler):
    remote: Remote

    def log_message(self, fmt, *args):
        return

    def _send(self, code, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _forward_error(self, exc):
        if isinstance(exc, HTTPError):
            code = exc.code
            try: detail = exc.read()[:1000]
            except Exception: detail = b"remote http error"
        else:
            code = 502
            detail = f"remote simulation unavailable: {exc}".encode()
        self._send(code, detail, "text/plain; charset=utf-8")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    frame, _ = self.remote.request("/snapshot", timeout=15)
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                    )
                    self.wfile.flush()
                    time.sleep(.25)
            except (BrokenPipeError, ConnectionResetError, OSError, HTTPError, URLError):
                pass
            return
        mapping = {"/snapshot": "/snapshot", "/health": "/health", "/state": "/state"}
        if path not in mapping:
            self.send_error(404); return
        try:
            body, ctype = self.remote.request(mapping[path])
            self._send(200, body, ctype)
        except (HTTPError, URLError, OSError) as exc:
            self._forward_error(exc)

    def do_POST(self):
        path = urlparse(self.path).path
        if path != "/command":
            self.send_error(404); return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(n) or b"{}")
            body, ctype = self.remote.request("/command", payload=payload, timeout=45)
            self._send(200, body, ctype)
        except (ValueError, json.JSONDecodeError):
            self._send(400, b'invalid json', "text/plain; charset=utf-8")
        except (HTTPError, URLError, OSError) as exc:
            self._forward_error(exc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    # 8093 is the bridge's default worker WebSocket port; keep the proxy apart.
    ap.add_argument("--port", type=int, default=8094)
    ap.add_argument("--url", default=os.environ.get("UGRP_SIM_REMOTE_URL"))
    args = ap.parse_args()
    base = (args.url or (URL_FILE.read_text().strip() if URL_FILE.exists() else "")).strip()
    if not base:
        raise SystemExit("set UGRP_SIM_REMOTE_URL or create .sim_remote_url")
    remote = Remote(base, load_secret(TOKEN_FILE))
    handler = type("BoundRemoteHandler", (Handler,), {"remote": remote})
    print(f"UGRP sim remote proxy -> {base}", flush=True)
    ThreadingHTTPServer((args.host, args.port), handler).serve_forever()


if __name__ == "__main__":
    main()
