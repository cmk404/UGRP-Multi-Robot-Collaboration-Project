#!/usr/bin/env python3
"""Local OpenAI-compatible relay for the Google Gemini API.

The UGRP harness talks to an unauthenticated loopback endpoint. This relay adds
the Gemini API key only on the outbound request and never writes it to disk.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import socket
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_UPSTREAM = "https://generativelanguage.googleapis.com/v1beta/openai/chat/completions"
MAX_REQUEST_BYTES = 32 * 1024 * 1024


def upstream_request(body: bytes, api_key: str, upstream: str = DEFAULT_UPSTREAM) -> Request:
    return Request(
        upstream,
        data=body,
        method="POST",
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
            "User-Agent": "ugrp-gemini-proxy/1.0",
        },
    )


class GeminiProxyHandler(BaseHTTPRequestHandler):
    api_key = ""
    upstream = DEFAULT_UPSTREAM
    upstream_timeout = 90.0

    def log_message(self, format: str, *args: object) -> None:
        # Keep the usual access log, which contains no request body or key.
        super().log_message(format, *args)

    def _json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/health":
            self._json(200, {"ok": True, "provider": "google-gemini", "configured": bool(self.api_key)})
            return
        self._json(404, {"error": {"message": "not found"}})

    def do_POST(self) -> None:
        if self.path != "/v1/chat/completions":
            self._json(404, {"error": {"message": "not found"}})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._json(400, {"error": {"message": "invalid Content-Length"}})
            return
        if length <= 0 or length > MAX_REQUEST_BYTES:
            self._json(413, {"error": {"message": "request body must be between 1 byte and 32 MiB"}})
            return
        body = self.rfile.read(length)
        try:
            parsed = json.loads(body)
            if not isinstance(parsed, dict):
                raise ValueError
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            self._json(400, {"error": {"message": "request body must be a JSON object"}})
            return

        try:
            with urlopen(
                upstream_request(body, self.api_key, self.upstream),
                timeout=self.upstream_timeout,
            ) as response:
                response_body = response.read()
                status = response.status
                content_type = response.headers.get("Content-Type", "application/json")
        except HTTPError as exc:
            response_body = exc.read()
            status = exc.code
            content_type = exc.headers.get("Content-Type", "application/json")
        except (URLError, TimeoutError, socket.timeout, OSError):
            self._json(502, {"error": {"message": "Gemini upstream is unavailable"}})
            return

        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(response_body)))
        self.end_headers()
        self.wfile.write(response_body)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8391)
    parser.add_argument("--upstream", default=DEFAULT_UPSTREAM)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    api_key = os.environ.get("GEMINI_API_KEY", "").strip()
    if not api_key:
        parser.error("set GEMINI_API_KEY before starting the proxy")
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    if args.timeout <= 0:
        parser.error("timeout must be positive")

    handler = type(
        "ConfiguredGeminiProxyHandler",
        (GeminiProxyHandler,),
        {"api_key": api_key, "upstream": args.upstream, "upstream_timeout": args.timeout},
    )
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(
        f"UGRP Gemini proxy listening on http://{args.host}:{args.port}/v1/chat/completions",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
