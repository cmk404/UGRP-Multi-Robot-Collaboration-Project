import json
import pytest

from harness.gemini_proxy import GeminiProxyCompleter
from scripts.run_gemini_seed_validation import command


class Ok:
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def read(self):
        return json.dumps({"model": "gemini-3.8-flash-medium",
                           "choices": [{"message": {"content": "ok"}}]}).encode()


@pytest.mark.parametrize("effort", ["none", "low", "medium", "high"])
def test_proxy_sends_validated_reasoning_effort(effort):
    seen = {}
    def http_open(request, timeout=0):
        seen["payload"] = json.loads(request.data)
        return Ok()
    completer = GeminiProxyCompleter(reasoning_effort=effort, http_open=http_open)
    assert completer.complete([{"role": "user", "content": "test"}]) == "ok"
    assert completer.reasoning_effort == effort
    assert seen["payload"]["reasoning_effort"] == effort
    assert completer.last_model == "gemini-3.8-flash-medium"


def test_proxy_rejects_unknown_reasoning_effort_before_request():
    with pytest.raises(ValueError, match="none, low, medium, high"):
        GeminiProxyCompleter(reasoning_effort="minimal")


def test_seed_validation_child_command_preserves_none_default_and_explicit_medium(tmp_path):
    default = command(tmp_path / "default", 42)
    medium = command(tmp_path / "medium", 46, "medium")
    assert default[default.index("--reasoning-effort") + 1] == "none"
    assert medium[medium.index("--reasoning-effort") + 1] == "medium"
    assert default.count("--reasoning-effort") == 1
    assert medium.count("--reasoning-effort") == 1
