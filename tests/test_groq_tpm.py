from __future__ import annotations

import json
import os
import tempfile
import unittest
from unittest.mock import patch
from io import BytesIO
from urllib.error import HTTPError

from harness.groq import GroqCompleter, GroqFallbackCompleter
from harness.vlm import VlmError


class _Ok:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return json.dumps({"choices": [{"message": {"content": '{"final":"ok"}'}}]}).encode()


class GroqTpmTests(unittest.TestCase):
    def _tpm(self, request):
        detail = {
            "error": {
                "message": "Request too large for model qwen on tokens per minute (TPM): Limit 8000"
            }
        }
        return HTTPError(
            request.full_url, 413, "Payload Too Large", hdrs=None,
            fp=BytesIO(json.dumps(detail).encode()),
        )

    def _combo(self, http_open, *, matrix_retries=0, matrix_max_wait=15.0, sleeper=lambda _s: None):
        keys = ["gsk_key1", "gsk_key2"]
        return GroqFallbackCompleter([
            GroqCompleter(
                keys=keys, model="qwen/qwen3.8-27b", http_open=http_open,
                rate_limit_retries=0,
            ),
            GroqCompleter(
                keys=keys, model="qwen/qwen3.6-27b", http_open=http_open,
                rate_limit_retries=0,
            ),
        ], tpm_matrix_retries=matrix_retries, tpm_matrix_max_wait=matrix_max_wait, sleeper=sleeper)

    def _temporary_tpm(self, request, retry_after="0.25"):
        detail = {
            "error": {
                "message": (
                    "Rate limit reached for model qwen on tokens per minute (TPM): "
                    "Limit 8000, Used 6221, Requested 3411. "
                    f"Please try again in {retry_after}s."
                )
            }
        }
        return HTTPError(
            request.full_url, 429, "Too Many Requests", hdrs=None,
            fp=BytesIO(json.dumps(detail).encode()),
        )

    def test_tpm_rotates_all_keys_then_fallback_model(self):
        calls = []

        def http_open(request, timeout=0):
            body = json.loads(request.data.decode())
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append((body["model"], key))
            if len(calls) < 4:
                raise self._tpm(request)
            return _Ok()

        combo = self._combo(http_open)
        self.assertEqual(
            combo.complete([{"role": "user", "content": "집어"}]),
            '{"final":"ok"}',
        )
        self.assertEqual(calls, [
            ("qwen/qwen3.8-27b", "gsk_key1"),
            ("qwen/qwen3.8-27b", "gsk_key2"),
            ("qwen/qwen3.6-27b", "gsk_key1"),
            ("qwen/qwen3.6-27b", "gsk_key2"),
        ])
        self.assertEqual(combo.last_model, "qwen/qwen3.6-27b")

    def test_fallback_planner_budget_is_not_limited_by_model_key_slots(self):
        combo = self._combo(lambda request, timeout=0: _Ok())
        self.assertEqual(combo.planner_call_budget, 20)
        self.assertGreater(combo.planner_call_budget, 4)

    def test_successive_completions_rotate_start_key_to_spread_tpm_load(self):
        calls = []

        def http_open(request, timeout=0):
            body = json.loads(request.data.decode())
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append((body["model"], key))
            return _Ok()

        combo = self._combo(http_open)
        combo.complete([{"role": "user", "content": "first"}])
        combo.complete([{"role": "user", "content": "second"}])
        combo.complete([{"role": "user", "content": "third"}])
        self.assertEqual(calls, [
            ("qwen/qwen3.8-27b", "gsk_key1"),
            ("qwen/qwen3.8-27b", "gsk_key2"),
            ("qwen/qwen3.8-27b", "gsk_key1"),
        ])

    def test_tpm_is_returned_only_after_every_model_key_combination_fails(self):
        calls = []

        def http_open(request, timeout=0):
            body = json.loads(request.data.decode())
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append((body["model"], key))
            raise self._tpm(request)

        combo = self._combo(http_open)
        with self.assertRaisesRegex(VlmError, "TPM"):
            combo.complete([{"role": "user", "content": "집어"}])
        self.assertEqual(calls, [
            ("qwen/qwen3.8-27b", "gsk_key1"),
            ("qwen/qwen3.8-27b", "gsk_key2"),
            ("qwen/qwen3.6-27b", "gsk_key1"),
            ("qwen/qwen3.6-27b", "gsk_key2"),
        ])


    def test_temporary_tpm_waits_only_after_full_matrix_then_retries(self):
        calls = []
        sleeps = []

        def http_open(request, timeout=0):
            body = json.loads(request.data.decode())
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append((body["model"], key))
            if len(calls) <= 4:
                raise self._temporary_tpm(request, "0.25")
            return _Ok()

        combo = self._combo(
            http_open, matrix_retries=1, matrix_max_wait=1.0, sleeper=sleeps.append
        )
        self.assertEqual(
            combo.complete([{"role": "user", "content": "집어"}]),
            '{"final":"ok"}',
        )
        self.assertEqual(calls[:4], [
            ("qwen/qwen3.8-27b", "gsk_key1"),
            ("qwen/qwen3.8-27b", "gsk_key2"),
            ("qwen/qwen3.6-27b", "gsk_key1"),
            ("qwen/qwen3.6-27b", "gsk_key2"),
        ])
        self.assertEqual(calls[4], ("qwen/qwen3.8-27b", "gsk_key1"))
        self.assertEqual(sleeps, [0.25])

    def test_shared_key_rotation_distributes_separate_process_starts(self):
        calls = []

        def ok(request, timeout=0):
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append(key)
            return _Ok()

        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "GROQ_SHARED_KEY_ROTATION": "1",
            "GROQ_SHARED_KEY_ROTATION_DIR": td,
            "GROQ_SHARED_TPM_GATE": "0",
        }, clear=False):
            a = GroqCompleter(keys=["gsk_key1", "gsk_key2", "gsk_key3"], model="m", http_open=ok, rate_limit_retries=0)
            b = GroqCompleter(keys=["gsk_key1", "gsk_key2", "gsk_key3"], model="m", http_open=ok, rate_limit_retries=0)
            c = GroqCompleter(keys=["gsk_key1", "gsk_key2", "gsk_key3"], model="m", http_open=ok, rate_limit_retries=0)
            a.complete([{"role":"user","content":"a"}])
            b.complete([{"role":"user","content":"b"}])
            c.complete([{"role":"user","content":"c"}])
        self.assertEqual(calls, ["gsk_key1", "gsk_key2", "gsk_key3"])

    def test_shared_model_gate_reuses_confirmed_tpm_cooldown_across_completers(self):
        calls = []

        def limited(request, timeout=0):
            calls.append(json.loads(request.data.decode())["model"])
            raise self._temporary_tpm(request, "5.0")

        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "GROQ_SHARED_TPM_GATE": "1",
            "GROQ_SHARED_TPM_GATE_DIR": td,
        }, clear=False):
            first = GroqCompleter(
                keys=["gsk_key1", "gsk_key2"], model="qwen/qwen3.8-27b",
                http_open=limited, rate_limit_retries=0,
            )
            with self.assertRaisesRegex(Exception, "TPM"):
                first.complete([{"role": "user", "content": "first"}])
            self.assertEqual(len(calls), 2)

            second_http_calls = []
            second = GroqCompleter(
                keys=["gsk_key1", "gsk_key2"], model="qwen/qwen3.8-27b",
                http_open=lambda request, timeout=0: second_http_calls.append(request) or _Ok(),
                rate_limit_retries=0,
            )
            with self.assertRaisesRegex(Exception, "shared cooldown"):
                second.complete([{"role": "user", "content": "second"}])
            self.assertEqual(second_http_calls, [])

    def test_request_too_large_never_waits_after_full_matrix(self):
        calls = []
        sleeps = []

        def http_open(request, timeout=0):
            body = json.loads(request.data.decode())
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append((body["model"], key))
            raise self._tpm(request)

        combo = self._combo(
            http_open, matrix_retries=1, matrix_max_wait=15.0, sleeper=sleeps.append
        )
        with self.assertRaisesRegex(VlmError, "TPM"):
            combo.complete([{"role": "user", "content": "집어"}])
        self.assertEqual(len(calls), 4)
        self.assertEqual(sleeps, [])

    def test_ordinary_429_preserves_provider_retry_after_across_model_fallback(self):
        calls = []
        detail = {"error": {"message": "Rate limit reached. Please try again in 17.5s."}}

        def http_open(request, timeout=0):
            calls.append(json.loads(request.data.decode())["model"])
            raise HTTPError(
                request.full_url, 429, "Too Many Requests", hdrs=None,
                fp=BytesIO(json.dumps(detail).encode()),
            )

        combo = self._combo(http_open)
        with self.assertRaisesRegex(VlmError, r"retry in 17\.500s"):
            combo.complete([{"role": "user", "content": "집어"}])
        self.assertEqual(len(calls), 4)

    def test_retry_after_parser_accepts_milliseconds(self):
        detail = {"error": {"message": "Rate limit reached. Please try again in 157.5ms."}}
        def http_open(request, timeout=0):
            raise HTTPError(
                request.full_url, 429, "Too Many Requests", hdrs=None,
                fp=BytesIO(json.dumps(detail).encode()),
            )
        c = GroqCompleter(keys=["gsk_key1"], model="m", http_open=http_open, rate_limit_retries=0)
        with self.assertRaisesRegex(VlmError, r"retry in 0\.158s"):
            c.complete([{"role": "user", "content": "x"}])

    def test_tpm_bearing_429_also_rotates_instead_of_waiting(self):
        calls = []
        detail = {"error": {"message": "tokens per minute (TPM): Limit 8000"}}

        def http_open(request, timeout=0):
            body = json.loads(request.data.decode())
            key = request.get_header("Authorization").removeprefix("Bearer ")
            calls.append((body["model"], key))
            raise HTTPError(
                request.full_url, 429, "Too Many Requests", hdrs={"Retry-After": "9"},
                fp=BytesIO(json.dumps(detail).encode()),
            )

        combo = self._combo(http_open)
        with self.assertRaisesRegex(VlmError, "TPM"):
            combo.complete([{"role": "user", "content": "집어"}])
        self.assertEqual(len(calls), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
