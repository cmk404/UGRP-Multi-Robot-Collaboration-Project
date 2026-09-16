"""Owned subprocess adapter: keeps optional ML dependencies out of MuJoCo."""
from __future__ import annotations

import base64
import json
from pathlib import Path
import selectors
import subprocess


def interpreter_path(path: Path) -> Path:
    # Resolving a venv's python symlink selects the system Python and loses its
    # site-packages. Absolute is intentional; do not replace it with resolve().
    return path.expanduser().absolute()


class RecoveryClient:
    def __init__(self, python: Path, model_dir: Path, *, timeout_s: float = 30):
        root = Path(__file__).resolve().parents[1]
        self.timeout_s = timeout_s
        self.process = subprocess.Popen(
            [str(interpreter_path(python)), str(root / "scripts/recovery_act_worker.py"), "--model-dir", str(model_dir)],
            cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        try:
            if self._read() != {"ready": True, "runtime_inputs": ["own_rgb", "top_rgb"]}:
                raise ValueError("invalid ACT worker startup")
        except BaseException:
            self.close()
            raise

    def _read(self):
        if not self.selector.select(timeout=self.timeout_s):
            raise TimeoutError("ACT worker reply timed out")
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("ACT worker exited without a reply")
        reply = json.loads(line)
        if "error" in reply:
            raise RuntimeError(reply["error"])
        return reply

    def predict(self, own_jpeg: bytes, top_jpeg: bytes):
        request = {"own_rgb": base64.b64encode(own_jpeg).decode("ascii"),
                   "top_rgb": base64.b64encode(top_jpeg).decode("ascii")}
        self.process.stdin.write(json.dumps(request) + "\n")
        self.process.stdin.flush()
        return self._read()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.selector.close()
        self.process.stdin.close()
        self.process.stdout.close()
