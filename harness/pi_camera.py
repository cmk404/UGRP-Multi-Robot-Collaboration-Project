"""Grab a JPEG or MJPEG stream from the MasterPi camera. Never uses the Mac webcam."""

from __future__ import annotations

import importlib.util
import os
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable
from urllib.error import URLError
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT = "ugrp1"
SNAPSHOT_CMD = (
    "curl -sf --max-time 3 http://127.0.0.1:8080/snapshot "
    "|| curl -sf --max-time 3 'http://127.0.0.1:8080/?action=snapshot'"
)
STREAM_CMD = (
    "curl -sN --max-time 3600 http://127.0.0.1:8080/stream "
    "|| curl -sN --max-time 3600 'http://127.0.0.1:8080/?action=stream'"
)
DEFAULT_STREAM_TYPE = "multipart/x-mixed-replace; boundary=frame"


class CameraError(RuntimeError):
    """Raised when the MasterPi camera cannot be read."""


def _load_deploy():
    path = ROOT / "scripts" / "red_block" / "deploy.py"
    spec = importlib.util.spec_from_file_location("ugrp_red_block_deploy", path)
    if spec is None or spec.loader is None:
        raise CameraError(f"missing {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def grab_snapshot(
    *,
    host: str = DEFAULT_ROBOT,
    url: str | None = None,
    ssh_run: Callable[..., subprocess.CompletedProcess] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
    http_open=urlopen,
) -> bytes:
    """Return one JPEG from MasterPi.

    Direct HTTP is used only when `url` or MASTERPI_CAMERA_URL is set.
    Otherwise SSH into the robot and read ustreamer's local snapshot.
    """
    target = url or os.environ.get("MASTERPI_CAMERA_URL")
    if target:
        return _from_http(target, http_open)
    return _from_ssh(host, ssh_run or subprocess.run, ssh_args)


def _from_http(url: str, http_open) -> bytes:
    try:
        with http_open(url, timeout=8) as response:
            data = response.read()
    except (OSError, URLError) as exc:
        raise CameraError(f"MasterPi 카메라 URL을 읽지 못했습니다: {exc}") from exc
    return _require_jpeg(data)


def _from_ssh(
    host: str,
    ssh_run: Callable[..., subprocess.CompletedProcess],
    ssh_args: tuple[str, list[str]] | None = None,
) -> bytes:
    if ssh_args is None:
        try:
            deploy = _load_deploy()
            destination, opts = deploy.ssh_connection_args(host)
        except Exception as exc:
            raise CameraError(f"MasterPi SSH 대상을 정하지 못했습니다: {exc}") from exc
    else:
        destination, opts = ssh_args
    try:
        result = ssh_run(
            ["ssh", *opts, destination, SNAPSHOT_CMD],
            check=False,
            capture_output=True,
            timeout=12,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise CameraError(f"MasterPi SSH 카메라 요청이 실패했습니다: {exc}") from exc
    if result.returncode != 0:
        detail = (result.stderr or b"").decode("utf-8", errors="replace")[:200]
        raise CameraError(
            "MasterPi 카메라에 연결하지 못했습니다. "
            "로봇에서 ustreamer가 :8080/snapshot 을 열고 있는지 확인하세요."
            + (f" {detail}" if detail else "")
        )
    return _require_jpeg(result.stdout)


def _require_jpeg(data: bytes) -> bytes:
    if not data or data[:2] != b"\xff\xd8":
        raise CameraError("MasterPi 카메라가 JPEG를 주지 않았습니다.")
    return data


def pop_jpegs(buf: bytearray) -> list[bytes]:
    """Pull complete JPEG frames out of an MJPEG byte buffer."""
    frames: list[bytes] = []
    while True:
        start = buf.find(b"\xff\xd8")
        if start < 0:
            buf.clear()
            return frames
        if start:
            del buf[:start]
        end = buf.find(b"\xff\xd9")
        if end < 0:
            return frames
        frames.append(bytes(buf[: end + 2]))
        del buf[: end + 2]


@dataclass
class CameraTunnel:
    proc: subprocess.Popen
    port: int

    @property
    def snapshot_url(self) -> str:
        return f"http://127.0.0.1:{self.port}/snapshot"

    @property
    def stream_url(self) -> str:
        return f"http://127.0.0.1:{self.port}/stream"

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class ByteStream:
    def __init__(self, read, close, content_type: str = DEFAULT_STREAM_TYPE) -> None:
        self.content_type = content_type
        self._read = read
        self._close = close

    def read(self, size: int = 8192) -> bytes:
        return self._read(size)

    def close(self) -> None:
        self._close()


def stream_url_from_snapshot(url: str) -> str:
    if "snapshot" in url:
        return url.replace("snapshot", "stream")
    if url.endswith("/"):
        return url + "stream"
    return url.rsplit("/", 1)[0] + "/stream"


def open_mjpeg_stream(
    *,
    host: str = DEFAULT_ROBOT,
    url: str | None = None,
    ssh_popen: Callable[..., subprocess.Popen] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
    http_open=urlopen,
) -> ByteStream:
    target = url or os.environ.get("MASTERPI_CAMERA_STREAM_URL")
    if target:
        return _stream_from_http(target, http_open)
    return _stream_from_ssh(host, ssh_popen or subprocess.Popen, ssh_args)


def ensure_camera_service(
    *,
    host: str = DEFAULT_ROBOT,
    ssh_run: Callable[..., subprocess.CompletedProcess] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
) -> bool:
    """Start the existing Pi camera service only when :8080 is absent.

    Never stops/kills a camera process. If a legacy watcher already serves
    8080, the first curl succeeds and this is a no-op. If nothing serves it,
    ask the user's existing systemd user unit to start and wait briefly.
    """
    try:
        destination, opts = _ssh_target(host, ssh_args)
    except CameraError:
        return False
    runner = ssh_run or subprocess.run
    command = (
        "if curl -sf --max-time 1 http://127.0.0.1:8080/snapshot >/dev/null; then exit 0; fi; "
        "systemctl --user start ugrp-camera.service >/dev/null 2>&1 || true; "
        "i=0; while [ $i -lt 12 ]; do "
        "curl -sf --max-time 1 http://127.0.0.1:8080/snapshot >/dev/null && exit 0; "
        "sleep 0.25; i=$((i+1)); done; exit 1"
    )
    try:
        result = runner(
            ["ssh", *opts, destination, command],
            check=False,
            capture_output=True,
            timeout=8,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def start_camera_tunnel(
    *,
    host: str = DEFAULT_ROBOT,
    ssh_popen: Callable[..., subprocess.Popen] | None = None,
    ssh_args: tuple[str, list[str]] | None = None,
    wait: float = 5.0,
) -> CameraTunnel | None:
    try:
        destination, opts = _ssh_target(host, ssh_args)
    except CameraError:
        return None
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    popen = ssh_popen or subprocess.Popen
    try:
        proc = popen(
            [
                "ssh",
                *opts,
                "-N",
                "-o",
                "ExitOnForwardFailure=yes",
                "-L",
                f"{port}:127.0.0.1:8080",
                destination,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return None
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return None
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return CameraTunnel(proc, port)
        except OSError:
            time.sleep(0.1)
    proc.terminate()
    return None


def _ssh_target(
    host: str, ssh_args: tuple[str, list[str]] | None
) -> tuple[str, list[str]]:
    if ssh_args is not None:
        return ssh_args
    try:
        deploy = _load_deploy()
        return deploy.ssh_connection_args(host)
    except Exception as exc:
        raise CameraError(f"MasterPi SSH 대상을 정하지 못했습니다: {exc}") from exc


def _stream_from_http(url: str, http_open) -> ByteStream:
    try:
        response = http_open(url, timeout=10)
    except (OSError, URLError) as exc:
        raise CameraError(f"MasterPi 카메라 스트림을 열지 못했습니다: {exc}") from exc
    content_type = getattr(response, "headers", {}).get(
        "Content-Type", DEFAULT_STREAM_TYPE
    )
    if hasattr(response.headers, "get_content_type") and "multipart" not in content_type:
        content_type = response.headers.get("Content-Type") or DEFAULT_STREAM_TYPE
    # `HTTPResponse.read(size)` may wait to fill the requested byte count even
    # when a complete MJPEG frame has already arrived. `read1` returns the
    # currently available buffered bytes promptly, reducing latest-frame lag.
    reader = getattr(response, "read1", None)
    if not callable(reader):
        reader = response.read
    return ByteStream(reader, response.close, content_type)


def _stream_from_ssh(
    host: str,
    ssh_popen: Callable[..., subprocess.Popen],
    ssh_args: tuple[str, list[str]] | None,
) -> ByteStream:
    destination, opts = _ssh_target(host, ssh_args)
    try:
        proc = ssh_popen(
            ["ssh", *opts, destination, STREAM_CMD],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
    except OSError as exc:
        raise CameraError(f"MasterPi SSH 스트림이 실패했습니다: {exc}") from exc
    if proc.stdout is None:
        proc.kill()
        raise CameraError("MasterPi 카메라 스트림을 열지 못했습니다.")

    def close() -> None:
        if proc.poll() is None:
            proc.terminate()

    return ByteStream(proc.stdout.read, close, DEFAULT_STREAM_TYPE)
