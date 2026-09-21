"""Auditable explicit history; no worker-global buffer that mixes robots."""
import hashlib
import json
import selectors
import subprocess
from pathlib import Path
from harness.recovery_act_client import RecoveryClient, interpreter_path
from harness.carry_input_history import wire_request


class InputCarryClient(RecoveryClient):
    def __init__(self, python, model_dir, timeout_s=60):
        self.timeout_s = timeout_s
        self.history = json.loads((Path(model_dir) / 'adapter.json').read_text())['history']
        root = Path(__file__).resolve().parents[1]
        self.process = subprocess.Popen([str(interpreter_path(Path(python))), str(root / 'scripts/carry_input_worker.py'),
                                         '--model-dir', str(model_dir)], cwd=root,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        try:
            if self._read() != {'ready': True, 'history': self.history}:
                raise ValueError('worker startup mismatch')
        except BaseException:
            self.close()
            raise

    def predict(self, frames):
        payload = json.dumps(wire_request(frames, self.history)) + '\n'
        self.last_request_sha256 = hashlib.sha256(payload.encode()).hexdigest()
        self.process.stdin.write(payload)
        self.process.stdin.flush()
        return self._read()
