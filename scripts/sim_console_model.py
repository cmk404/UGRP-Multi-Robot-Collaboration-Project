"""One owned, cancellable model request. No simulator access or hardware tools."""
import io
import json
from pathlib import Path
import sys
import time
from urllib.request import urlopen

from harness.gemini_proxy import GeminiProxyCompleter


def complete(path):
    path = Path(path)
    request = json.loads(path.read_text())
    started = time.monotonic()
    record = {"raw": None, "error": None, "usage": None}
    def audited_open(req, **kwargs):
        path.with_suffix(".wire.json").write_bytes(req.data)
        with urlopen(req, **kwargs) as response:
            data = response.read()
        path.with_suffix(".wire-response.json").write_bytes(data)
        return io.BytesIO(data)
    client = GeminiProxyCompleter(model=request["model"], timeout=request["timeout_s"],
                                  max_tokens=request["max_tokens"], http_open=audited_open)
    try:
        record["raw"] = client.complete(request["messages"], images=request["images"])
        record["usage"] = client.last_usage
        record["model"] = client.last_model
    except Exception as error:
        # Provider errors are sanitized by the existing adapter; URLs/headers are not logged.
        record["error"] = str(error) if error.__class__.__module__ == "harness.gemini_proxy" else type(error).__name__
    record["wall_s"] = time.monotonic() - started
    path.with_suffix(".response.json").write_text(json.dumps(record, ensure_ascii=False) + "\n")
    return 1 if record["error"] else 0


if __name__ == "__main__":
    raise SystemExit(complete(sys.argv[1]))
