import json
import os
import unittest
from unittest.mock import patch

from harness.gemini_proxy import GEMINI_API_URL, GeminiProxyCompleter


class _Ok:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return json.dumps({
            "model": "gemini-3.8-flash",
            "choices": [{"message": {"content": "ok"}}],
        }).encode()


class GeminiDirectApiTests(unittest.TestCase):
    def test_api_key_uses_google_openai_compat_endpoint(self):
        seen = {}

        def http_open(request, timeout=0):
            seen["url"] = request.full_url
            seen["authorization"] = request.headers.get("Authorization")
            seen["body"] = json.loads(request.data.decode())
            return _Ok()

        with patch.dict(os.environ, {"GEMINI_API_KEY": "test-key"}, clear=False):
            os.environ.pop("GEMINI_PROXY_URL", None)
            completer = GeminiProxyCompleter(http_open=http_open)
            self.assertEqual(completer.complete([{"role": "user", "content": "hi"}]), "ok")

        self.assertEqual(seen["url"], GEMINI_API_URL)
        self.assertEqual(seen["authorization"], "Bearer test-key")
        self.assertEqual(seen["body"]["model"], "gemini-3.8-flash")
        self.assertNotIn("reasoning_effort", seen["body"])

    def test_explicit_proxy_still_wins_over_api_key(self):
        with patch.dict(os.environ, {
            "GEMINI_API_KEY": "test-key",
            "GEMINI_PROXY_URL": "http://127.0.0.1:9999/v1/chat/completions",
        }, clear=False):
            completer = GeminiProxyCompleter()
        self.assertFalse(completer.direct_api)
        self.assertEqual(completer.url, "http://127.0.0.1:9999/v1/chat/completions")
        self.assertEqual(completer.model_name, "gemini-3.7-flash")


if __name__ == "__main__":
    unittest.main()
