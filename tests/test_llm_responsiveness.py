import unittest
import json
import tempfile
from email.message import Message
from io import BytesIO
from pathlib import Path
from urllib.error import HTTPError, URLError
from harness.groq import GroqCompleter, _RateLimitError

ROOT=Path(__file__).resolve().parents[1]

class GroqWaitBoundTests(unittest.TestCase):
    def test_rate_limit_wait_is_bounded(self):
        sleeps=[]
        c=GroqCompleter(keys=['gsk_test'], rate_limit_retries=5, max_rate_limit_wait=.12, sleeper=lambda x:sleeps.append(x))
        c._post=lambda key,payload: (_ for _ in ()).throw(_RateLimitError(5.0))
        with self.assertRaises(Exception): c.complete([{'role':'user','content':'x'}])
        self.assertLessEqual(sum(sleeps), .120001)

class GroqRuntimeConfigTests(unittest.TestCase):
    def test_live_completer_uses_bounded_request_timeout_env(self):
        from unittest.mock import patch
        from harness.groq import live_completer
        with patch("harness.groq.load_groq_keys", return_value=["gsk_test"]), patch.dict(
            "os.environ", {"GROQ_REQUEST_TIMEOUT": "8", "GROQ_RATE_LIMIT_RETRIES": "0"}, clear=False
        ):
            combo, backend = live_completer("groq")
        self.assertEqual(backend, "groq")
        self.assertEqual(
            [c.model_name for c in combo.completers],
            ["qwen/qwen3.8-27b", "qwen/qwen3.6-27b"],
        )
        self.assertEqual([c.timeout for c in combo.completers], [8.0, 8.0])
        self.assertEqual([c.rate_limit_retries for c in combo.completers], [0, 0])


class GeminiProxyTests(unittest.TestCase):
    def test_live_completer_selects_local_gemini_proxy(self):
        from unittest.mock import patch
        from harness.groq import live_completer
        with patch.dict("os.environ", {"GEMINI_PROXY_REQUEST_TIMEOUT": "9"}, clear=False):
            completer, backend = live_completer("gemini")
        self.assertEqual(backend, "gemini")
        self.assertEqual(completer.model_name, "gemini-3.7-flash")
        self.assertEqual(completer.url, "http://127.0.0.1:8391/v1/chat/completions")
        self.assertEqual(completer.timeout, 9.0)

    def test_gemini_proxy_sends_openai_image_data_url_and_parses_response(self):
        from harness.gemini_proxy import GeminiProxyCompleter
        root = Path(tempfile.mkdtemp())
        image = root / "frame.jpg"
        image.write_bytes(b"\xff\xd8\xff\xd9")
        seen = {}

        class Ok:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def read(self):
                return json.dumps({"model": "gemini-3.7-flash-provider-rev",
                    "choices": [{"message": {"content": '{"final":"ok"}'}}]}).encode()

        def http_open(request, timeout=0):
            seen["timeout"] = timeout
            seen["body"] = json.loads(request.data.decode())
            return Ok()

        completer = GeminiProxyCompleter(http_open=http_open, timeout=7)
        self.assertEqual(
            completer.complete([{"role": "user", "content": "봐줘"}], image=str(image)),
            '{"final":"ok"}',
        )
        content = seen["body"]["messages"][-1]["content"]
        self.assertEqual(seen["timeout"], 7)
        self.assertEqual(seen["body"]["model"], "gemini-3.7-flash")
        self.assertEqual(seen["body"]["reasoning_effort"], "none")
        self.assertTrue(content[1]["image_url"]["url"].startswith("data:image/jpeg;base64,"))
        self.assertEqual(completer.last_model, "gemini-3.7-flash-provider-rev")

    def test_gemini_proxy_maps_http_error_to_vlm_error(self):
        from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError
        from harness.vlm import VlmError

        def fail(request, timeout=0):
            raise HTTPError(request.full_url, 429, "limited", hdrs=None, fp=BytesIO(b'{"error":"quota"}'))

        with self.assertRaises(VlmError) as ctx:
            GeminiProxyCompleter(http_open=fail).complete([{"role": "user", "content": "x"}])
        self.assertIn("HTTP 429", str(ctx.exception))
        self.assertIsInstance(ctx.exception, GeminiProxyError)
        self.assertEqual(ctx.exception.error_kind, "http")
        self.assertTrue(ctx.exception.retryable)
        self.assertEqual(ctx.exception.http_status, 429)
        self.assertNotIn("quota", str(ctx.exception))

    def test_gemini_proxy_classifies_http_retryability(self):
        from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError

        for status, retryable in ((503, True), (400, False)):
            with self.subTest(status=status):
                def fail(request, timeout=0, status=status):
                    raise HTTPError(request.full_url, status, "failed", hdrs=None,
                                    fp=BytesIO(b"secret response body"))
                with self.assertRaises(GeminiProxyError) as ctx:
                    GeminiProxyCompleter(http_open=fail).complete([{"role": "user", "content": "x"}])
                self.assertEqual(ctx.exception.http_status, status)
                self.assertEqual(ctx.exception.retryable, retryable)
                self.assertNotIn("secret", str(ctx.exception))

    def test_gemini_proxy_exposes_safe_retry_after_seconds(self):
        from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError

        headers = Message()
        headers["Retry-After"] = "37.5"
        def fail(request, timeout=0):
            raise HTTPError(request.full_url, 429, "limited", hdrs=headers,
                            fp=BytesIO(b"private quota response"))
        with self.assertRaises(GeminiProxyError) as ctx:
            GeminiProxyCompleter(http_open=fail).complete([{"role": "user", "content": "x"}])
        self.assertEqual(ctx.exception.retry_after_s, 37.5)
        self.assertNotIn("private", str(ctx.exception))

    def test_retry_after_supports_http_date_and_rejects_unsafe_ranges(self):
        from harness.gemini_proxy import _retry_after_seconds

        self.assertEqual(
            _retry_after_seconds({"Retry-After": "Thu, 01 Jan 1970 00:01:00 GMT"}, now=0),
            60.0,
        )
        for value in ("-1", "86401", "nan", "not-a-date"):
            with self.subTest(value=value):
                self.assertIsNone(_retry_after_seconds({"Retry-After": value}, now=0))

    def test_gemini_proxy_classifies_timeout_and_connection(self):
        from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError

        for failure, kind in ((TimeoutError("late"), "timeout"),
                              (URLError("refused"), "connection"),
                              (URLError(TimeoutError("late")), "timeout")):
            with self.subTest(kind=kind, failure=type(failure).__name__):
                def fail(request, timeout=0, failure=failure):
                    raise failure
                with self.assertRaises(GeminiProxyError) as ctx:
                    GeminiProxyCompleter(http_open=fail).complete([{"role": "user", "content": "x"}])
                self.assertEqual(ctx.exception.error_kind, kind)
                self.assertTrue(ctx.exception.retryable)
                self.assertIsNone(ctx.exception.http_status)
                self.assertIsInstance(ctx.exception.latency_ms, float)

    def test_gemini_proxy_classifies_malformed_response(self):
        from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError

        class Bad:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def read(self): return b'{"choices": []}'

        with self.assertRaises(GeminiProxyError) as ctx:
            GeminiProxyCompleter(http_open=lambda *args, **kwargs: Bad()).complete(
                [{"role": "user", "content": "x"}]
            )
        self.assertEqual(ctx.exception.error_kind, "malformed_response")
        self.assertTrue(ctx.exception.retryable)
        self.assertIsNone(ctx.exception.http_status)


class PlannerInputBudgetTests(unittest.TestCase):
    def test_planner_image_is_downscaled_without_changing_original(self):
        import tempfile
        from pathlib import Path
        from PIL import Image
        from harness.perception import prepare_planner_image

        root = Path(tempfile.mkdtemp())
        src = root / "frame.jpg"
        Image.new("RGB", (640, 480), (120, 20, 20)).save(src, format="JPEG")
        out = Path(prepare_planner_image(src, mask_masterpi_self=True, max_edge=320))
        with Image.open(src) as original:
            self.assertEqual(original.size, (640, 480))
        with Image.open(out) as prepared:
            self.assertEqual(prepared.size, (320, 240))
        self.assertNotEqual(out, src)

    def test_world_state_does_not_repeat_contract_condition_strings(self):
        import json
        from harness.action_queue import RobotActionQueue
        from harness.catalog import default_registry
        from harness.executive import TaskExecutive
        from harness.goals import infer_goal
        from harness.loop import _planner_decision_context
        from harness.state import StateEstimator

        registry = default_registry(actions_path="scripts/sim_actions.py")
        estimator = StateEstimator()
        state = _planner_decision_context(
            estimator.state, infer_goal("빨간 블록 집어"), [], registry=registry,
            executive=TaskExecutive(strict_pick_preconditions=True),
            action_queue=RobotActionQueue(),
        )
        encoded = json.dumps(state["action_contract_status"], ensure_ascii=False)
        self.assertNotIn("\"expected_postconditions\"", encoded)
        self.assertIn("expected_postconditions_all_satisfied", encoded)
        self.assertLess(len(encoded), 3000)


class PlannerStateCompactionTests(unittest.TestCase):
    def test_auto_observe_does_not_resend_raw_execution_transcript(self):
        from harness.catalog import default_registry
        from harness.loop import run_loop

        class Capture:
            def __init__(self):
                self.calls = []
                self.replies = [
                    '{"tool":"search","args":{"target_color":"red"}}',
                    '{"final":"done"}',
                ]
            def complete(self, messages, image=None):
                self.calls.append((messages, image))
                return self.replies.pop(0)

        c = Capture()
        run_loop(
            c,
            default_registry(runner=lambda name, **kwargs: {
                "ok": True, "skill": name, "command_status": "ACCEPTED",
                "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
                "target_color": "red", "target_vision": {"visible": True},
            }),
            # A sensor-satisfied SEARCH goal would finish deterministically after
            # one planner call; a GENERIC request keeps the second call we inspect.
            "빨간 블록 어디 있는지 알려줘",
            image="/tmp/start.jpg", execute=True, auto_observe=True,
            observe=lambda: "/tmp/after.jpg",
        )
        self.assertEqual(len(c.calls), 2)
        second_text = "\n".join(str(m.get("content", "")) for m in c.calls[1][0])
        self.assertNotIn("tool_result:", second_text)
        self.assertNotIn("tool_queued:", second_text)
        self.assertNotIn("queue_plan_accepted:", second_text)
        self.assertIn("world_state:", second_text)

    def test_planner_queue_history_is_bounded_and_compact(self):
        from harness.action_queue import RobotActionQueue
        from harness.executive import TaskExecutive
        from harness.loop import _planner_queue_snapshot
        from harness.state import StateEstimator

        q = RobotActionQueue()
        for i in range(8):
            q.enqueue("search", {"target_color": "red"}, source="test")
            q.claim_next()
            q.complete_running(outcome_status="ACHIEVED")
        snap = _planner_queue_snapshot(q, StateEstimator().state, TaskExecutive())
        self.assertLessEqual(len(snap["history"]), 4)
        self.assertNotIn("id", snap["history"][-1])
        self.assertNotIn("source", snap["history"][-1])


class PlannerQueuePromptTests(unittest.TestCase):
    def test_multi_skill_goals_are_told_to_use_one_plan_queue(self):
        from harness.protocol import system_prompt
        prompt = system_prompt("- search\n- track\n- approach\n- pick", auto_observe=True)
        self.assertIn("MUST first send one Plan", prompt)
        self.assertIn("instead of issuing the skills one-by-one", prompt)
        self.assertIn("queue drains, blocks, or fails", prompt)

if __name__=='__main__': unittest.main()
