"""Plain conversation with a local Ollama model. No tools."""

from __future__ import annotations

import json
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_MODEL = "gemma4:e2b"
DEFAULT_URL = "http://127.0.0.1:11434"


class TalkError(RuntimeError):
    """Raised when the chat backend cannot answer."""


class ReplayTalker:
    def __init__(self, replies: list[Any]):
        self.replies = [str(item) if not isinstance(item, dict) else json.dumps(item, ensure_ascii=False) for item in replies]
        self.index = 0

    def reply(self, messages: list[dict[str, str]]) -> str:
        del messages
        if self.index >= len(self.replies):
            return "(no more replay replies)"
        text = self.replies[self.index]
        self.index += 1
        return text


class OllamaTalker:
    def __init__(
        self,
        *,
        model: str = DEFAULT_MODEL,
        base_url: str = DEFAULT_URL,
        opener: Callable[..., Any] | None = None,
        timeout: float = 120.0,
    ) -> None:
        self.model = model
        self.base_url = base_url.rstrip("/")
        self.opener = opener or urlopen
        self.timeout = timeout

    def reply(self, messages: list[dict[str, str]]) -> str:
        payload = json.dumps(
            {
                "model": self.model,
                "messages": messages,
                "stream": False,
            },
            ensure_ascii=False,
        ).encode("utf-8")
        request = Request(
            self.base_url + "/api/chat",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with self.opener(request, timeout=self.timeout) as response:
                body = json.loads(response.read().decode("utf-8"))
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise TalkError(f"Ollama HTTP {exc.code}: {detail[:300]}") from exc
        except URLError as exc:
            raise TalkError(
                f"Ollama에 연결하지 못했습니다 ({self.base_url}). ollama serve 가 켜져 있는지 확인하세요: {exc.reason}"
            ) from exc
        except TimeoutError as exc:
            raise TalkError("Ollama 응답이 너무 오래 걸렸습니다.") from exc
        message = body.get("message") or {}
        text = message.get("content")
        if not isinstance(text, str) or not text.strip():
            raise TalkError("Ollama가 빈 답을 반환했습니다.")
        return text


def turn(history: list[dict[str, str]], user_text: str, talker) -> tuple[list[dict[str, str]], str]:
    text = user_text.strip()
    if not text:
        raise TalkError("empty message")
    history = list(history)
    history.append({"role": "user", "content": text})
    answer = talker.reply(history)
    history.append({"role": "assistant", "content": answer})
    return history, answer
