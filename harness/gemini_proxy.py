"""OpenAI-compatible Gemini Antigravity subscription proxy completer."""

from __future__ import annotations

import json
import math
import os
import socket
import time
from datetime import datetime, timezone
from email.utils import parsedate_to_datetime
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .groq import _to_groq_messages
from .vlm import VlmError


DEFAULT_URL = "http://127.0.0.1:8391/v1/chat/completions"
DEFAULT_MODEL = "gemini-3.7-flash"
REASONING_EFFORTS = ("none", "low", "medium", "high")


class GeminiProxyError(VlmError):
    """A safely reportable Gemini proxy failure with stable retry metadata."""

    def __init__(
        self,
        message: str,
        *,
        error_kind: str,
        retryable: bool,
        http_status: int | None = None,
        latency_ms: float | None = None,
        retry_after_s: float | None = None,
    ) -> None:
        super().__init__(message)
        self.error_kind = error_kind
        self.retryable = retryable
        self.http_status = http_status
        self.latency_ms = latency_ms
        self.retry_after_s = retry_after_s


class GeminiProxyCompleter:
    def __init__(
        self,
        *,
        model: str = DEFAULT_MODEL,
        url: str | None = None,
        max_tokens: int = 256,
        temperature: float = 0.2,
        reasoning_effort: str = "none",
        timeout: float | None = None,
        http_open: Callable[..., Any] = urlopen,
    ) -> None:
        self.model_name = model
        self.url = url or os.environ.get("GEMINI_PROXY_URL", DEFAULT_URL)
        self.max_tokens = max(1, int(max_tokens))
        self.temperature = float(temperature)
        if reasoning_effort not in REASONING_EFFORTS:
            raise ValueError("reasoning_effort must be one of: none, low, medium, high")
        self.reasoning_effort = reasoning_effort
        if timeout is None:
            try:
                timeout = float(os.environ.get("GEMINI_PROXY_REQUEST_TIMEOUT", "45"))
            except ValueError:
                timeout = 45.0
        self.timeout = max(1.0, float(timeout))
        try:
            budget = int(os.environ.get("GEMINI_PLANNER_CALL_BUDGET", "20"))
        except ValueError:
            budget = 20
        self.planner_call_budget = max(1, budget)
        self.http_open = http_open
        self.last_usage: dict[str, int] | None = None
        self.last_model: str | None = None
        self.last_latency_ms: float | None = None

    def complete(self, messages: list[dict[str, Any]], image: str | None = None,
                 images: list[dict[str, str]] | None = None) -> str:
        self.last_usage = None
        self.last_model = None
        self.last_latency_ms = None
        if image is not None and images is not None:
            raise ValueError("use image or images, not both")
        payload = json.dumps(
            {
                "model": self.model_name,
                "messages": (_to_gemini_multi_image_messages(messages, images)
                             if images is not None else _to_groq_messages(messages, image)),
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
                "reasoning_effort": self.reasoning_effort,
            }
        ).encode("utf-8")
        request = Request(
            self.url,
            data=payload,
            method="POST",
            headers={"Content-Type": "application/json", "User-Agent": "ugrp-harness/1.0"},
        )
        started = time.monotonic()
        try:
            with self.http_open(request, timeout=self.timeout) as response:
                raw = response.read()
        except HTTPError as exc:
            latency_ms = (time.monotonic() - started) * 1000
            status = getattr(exc, "code", None)
            status = status if isinstance(status, int) else None
            retryable = status in {408, 429} or (status is not None and 500 <= status <= 599)
            retry_after_s = _retry_after_seconds(getattr(exc, "headers", None))
            raise GeminiProxyError(
                f"Gemini 프록시 HTTP {status or 0}", error_kind="http",
                retryable=retryable, http_status=status, latency_ms=latency_ms,
                retry_after_s=retry_after_s,
            ) from exc
        except (TimeoutError, socket.timeout) as exc:
            latency_ms = (time.monotonic() - started) * 1000
            raise GeminiProxyError(
                "Gemini 프록시 요청 시간이 초과되었습니다.", error_kind="timeout",
                retryable=True, latency_ms=latency_ms,
            ) from exc
        except URLError as exc:
            latency_ms = (time.monotonic() - started) * 1000
            if isinstance(exc.reason, (TimeoutError, socket.timeout)):
                raise GeminiProxyError(
                    "Gemini 프록시 요청 시간이 초과되었습니다.", error_kind="timeout",
                    retryable=True, latency_ms=latency_ms,
                ) from exc
            raise GeminiProxyError(
                "Gemini 프록시에 연결하지 못했습니다.", error_kind="connection",
                retryable=True, latency_ms=latency_ms,
            ) from exc
        except OSError as exc:
            latency_ms = (time.monotonic() - started) * 1000
            raise GeminiProxyError(
                "Gemini 프록시에 연결하지 못했습니다.", error_kind="connection",
                retryable=True, latency_ms=latency_ms,
            ) from exc
        self.last_latency_ms = (time.monotonic() - started) * 1000
        try:
            body = json.loads(raw.decode("utf-8"))
            choices = body["choices"]
            text = choices[0]["message"]["content"]
        except (KeyError, IndexError, TypeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise GeminiProxyError(
                "Gemini 프록시가 올바른 completion JSON을 주지 않았습니다.",
                error_kind="malformed_response", retryable=True,
                latency_ms=self.last_latency_ms,
            ) from exc
        if not isinstance(text, str) or not text.strip():
            raise GeminiProxyError(
                "Gemini 프록시 답이 비어 있습니다.", error_kind="malformed_response",
                retryable=True, latency_ms=self.last_latency_ms,
            )
        usage = body.get("usage")
        if isinstance(usage, dict):
            measured = {key: value for key, value in usage.items()
                        if key in {"prompt_tokens", "completion_tokens", "total_tokens"}
                        and isinstance(value, int) and not isinstance(value, bool) and value >= 0}
            self.last_usage = measured or None
        response_model = body.get("model")
        if isinstance(response_model, str) and response_model.strip():
            self.last_model = response_model.strip()
        return text.strip()


def _retry_after_seconds(headers: Any, *, now: float | None = None) -> float | None:
    """Parse Retry-After without exposing headers or response content."""
    if headers is None:
        return None
    try:
        value = headers.get("Retry-After")
    except (AttributeError, TypeError):
        return None
    if not isinstance(value, str) or not value.strip():
        return None
    value = value.strip()
    try:
        seconds = float(value)
    except ValueError:
        try:
            when = parsedate_to_datetime(value)
            if when.tzinfo is None:
                when = when.replace(tzinfo=timezone.utc)
            current = datetime.fromtimestamp(time.time() if now is None else now, timezone.utc)
            seconds = (when - current).total_seconds()
        except (TypeError, ValueError, OverflowError):
            return None
    if not math.isfinite(seconds) or seconds < 0 or seconds > 86400:
        return None
    return seconds


def _to_gemini_multi_image_messages(messages: list[dict[str, Any]],
                                    images: list[dict[str, str]]) -> list[dict[str, Any]]:
    """Attach labeled original images to the final user message."""
    if not isinstance(images, list) or not images:
        raise ValueError("images must be a non-empty list")
    last_user = max((i for i, msg in enumerate(messages) if msg.get("role") == "user"), default=-1)
    if last_user < 0:
        raise ValueError("a user message is required for images")
    out = []
    for index, message in enumerate(messages):
        role = str(message.get("role") or "user")
        text = str(message.get("content") or "")
        if index != last_user:
            out.append({"role": role, "content": text})
            continue
        content: list[dict[str, Any]] = [{"type": "text", "text": text}]
        for item in images:
            if not isinstance(item, dict) or set(item) != {"label", "image"}:
                raise ValueError("each image requires label and image")
            label, uri = item["label"], item["image"]
            if not isinstance(label, str) or not label.strip() or not isinstance(uri, str) or not uri.startswith("data:image/"):
                raise ValueError("invalid labeled image")
            content.extend(({"type": "text", "text": label.strip()},
                            {"type": "image_url", "image_url": {"url": uri}}))
        out.append({"role": role, "content": content})
    return out
