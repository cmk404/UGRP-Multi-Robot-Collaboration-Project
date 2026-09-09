import json
import unittest

from scripts.serve_gemini_proxy import DEFAULT_UPSTREAM, upstream_request


class GeminiProxyServerTests(unittest.TestCase):
    def test_upstream_request_adds_bearer_key_without_changing_body(self):
        body = json.dumps({"model": "gemini-3.8-flash", "messages": []}).encode()
        request = upstream_request(body, "secret-test-key")

        self.assertEqual(request.full_url, DEFAULT_UPSTREAM)
        self.assertEqual(request.method, "POST")
        self.assertEqual(request.data, body)
        self.assertEqual(request.get_header("Authorization"), "Bearer secret-test-key")
        self.assertEqual(request.get_header("Content-type"), "application/json")

    def test_upstream_can_be_overridden_for_local_testing(self):
        request = upstream_request(b"{}", "key", "http://127.0.0.1:9999/chat")
        self.assertEqual(request.full_url, "http://127.0.0.1:9999/chat")


if __name__ == "__main__":
    unittest.main()
