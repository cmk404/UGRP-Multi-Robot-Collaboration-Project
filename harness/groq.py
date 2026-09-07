"""Groq vision completer. Keys stay in env or a gitignored local file."""

from __future__ import annotations

import base64
from contextlib import contextmanager
import json
import os
import re
import threading
import time
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .vlm import VlmError


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_KEYS_PATH = ROOT / ".groq_keys"
DEFAULT_MODEL = "qwen/qwen3.8-27b"
GROQ_URL = "https://api.groq.com/openai/v1/chat/completions"
_THINK_RE = re.compile(r"<think>[\s\S]*?</think>", re.IGNORECASE)
_RETRY_RE = re.compile(r"try again in\s+([0-9.]+)\s*(ms|s)", re.IGNORECASE)


class _NoopSharedModelGate:
    def remaining(self) -> float:
        return 0.0

    def block(self, delay: float) -> None:
        return None


class _SharedModelGate:
    """Cross-process cooldown for an organization/model TPM bucket.

    The three SIM agents are separate Python processes. Their in-memory key
    cooldowns therefore cannot see one another. This gate serializes one model's
    full key sweep and shares only a *confirmed* temporary TPM cooldown after all
    configured keys for that model failed. Different models keep independent
    gates, so qwen3.8 and qwen3.6 can still run concurrently.
    """

    def __init__(self, fileobj):
        self.fileobj = fileobj

    def _read(self) -> dict[str, Any]:
        try:
            self.fileobj.seek(0)
            raw = self.fileobj.read()
            obj = json.loads(raw) if raw.strip() else {}
            return obj if isinstance(obj, dict) else {}
        except Exception:
            return {}

    def _write(self, obj: dict[str, Any]) -> None:
        self.fileobj.seek(0)
        self.fileobj.truncate()
        json.dump(obj, self.fileobj)
        self.fileobj.flush()
        try:
            os.fsync(self.fileobj.fileno())
        except OSError:
            pass

    def remaining(self) -> float:
        obj = self._read()
        try:
            until = float(obj.get("blocked_until") or 0.0)
        except (TypeError, ValueError):
            until = 0.0
        return max(0.0, until - time.time())

    def block(self, delay: float) -> None:
        delay = max(0.05, float(delay))
        obj = self._read()
        try:
            current = float(obj.get("blocked_until") or 0.0)
        except (TypeError, ValueError):
            current = 0.0
        obj["blocked_until"] = max(current, time.time() + delay)
        obj["updated_at"] = time.time()
        self._write(obj)


def _model_gate_slug(model: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", str(model or "model"))[:100]


def _shared_key_start(model: str, key_count: int) -> int | None:
    """Atomically distribute starting keys across R1/R2/R3 processes.

    No key material is persisted: the shared file contains only the next integer
    index for a model. This avoids all three independent agents starting at key0
    and moving through the same key sequence in lockstep.
    """
    if os.environ.get("GROQ_SHARED_KEY_ROTATION", "0") != "1" or key_count <= 0:
        return None
    try:
        import fcntl
    except ImportError:
        return None
    root = Path(os.environ.get("GROQ_SHARED_KEY_ROTATION_DIR", "/tmp/ugrp-groq-key-rotation"))
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"{_model_gate_slug(model)}.json"
    with path.open("a+", encoding="utf-8") as fh:
        fcntl.flock(fh.fileno(), fcntl.LOCK_EX)
        try:
            fh.seek(0)
            try:
                obj = json.loads(fh.read() or "{}")
            except Exception:
                obj = {}
            try:
                start = int(obj.get("next_index") or 0) % key_count
            except (TypeError, ValueError):
                start = 0
            fh.seek(0)
            fh.truncate()
            json.dump({"next_index": (start + 1) % key_count, "updated_at": time.time()}, fh)
            fh.flush()
            try:
                os.fsync(fh.fileno())
            except OSError:
                pass
            return start
        finally:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)


@contextmanager
def _shared_model_gate(model: str):
    if os.environ.get("GROQ_SHARED_TPM_GATE", "0") != "1":
        yield _NoopSharedModelGate()
        return
    try:
        import fcntl
    except ImportError:
        yield _NoopSharedModelGate()
        return
    root = Path(os.environ.get("GROQ_SHARED_TPM_GATE_DIR", "/tmp/ugrp-groq-tpm-gates"))
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"{_model_gate_slug(model)}.json"
    with path.open("a+", encoding="utf-8") as fh:
        fcntl.flock(fh.fileno(), fcntl.LOCK_EX)
        try:
            yield _SharedModelGate(fh)
        finally:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)


class _TpmLimitError(VlmError):
    """Provider-side TPM failure, either permanent-size or temporary-budget."""

    def __init__(
        self,
        detail: str,
        *,
        retry_after: float | None = None,
        request_too_large: bool = False,
    ) -> None:
        super().__init__(detail)
        self.retry_after = None if retry_after is None else max(0.0, float(retry_after))
        self.request_too_large = bool(request_too_large)


class _RateLimitError(VlmError):
    def __init__(self, retry_after: float, detail: str = "") -> None:
        self.retry_after = max(0.0, float(retry_after))
        self.detail = detail
        super().__init__(f"Groq 한도에 걸렸습니다. retry in {self.retry_after:.3f}s")


def load_groq_keys(
    *,
    env: dict[str, str] | None = None,
    path: str | Path | None = None,
) -> list[str]:
    source = os.environ if env is None else env
    keys: list[str] = []
    blob = source.get("GROQ_API_KEYS") or source.get("GROQ_API_KEY") or ""
    for part in blob.replace(";", ",").split(","):
        _add_key(keys, part)
    file_path = Path(path) if path is not None else DEFAULT_KEYS_PATH
    if file_path.is_file():
        for line in file_path.read_text(encoding="utf-8").splitlines():
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            _add_key(keys, text)
    seen: set[str] = set()
    unique: list[str] = []
    for key in keys:
        if key in seen:
            continue
        seen.add(key)
        unique.append(key)
    return unique


def _add_key(keys: list[str], raw: str) -> None:
    key = raw.strip()
    if key.startswith("gsk_"):
        keys.append(key)


def _image_data_uri(path: str) -> str:
    if isinstance(path, str) and path.startswith("data:"):
        match = re.fullmatch(r"data:(image/(?:jpeg|png|webp));base64,([A-Za-z0-9+/=]+)", path)
        if not match:
            raise VlmError("Invalid inline image data URI")
        try:
            data = base64.b64decode(match.group(2), validate=True)
        except ValueError as exc:
            raise VlmError("Invalid inline image base64") from exc
        if not data:
            raise VlmError("Empty inline image")
        return path
    data = Path(path).read_bytes()
    if not data:
        raise VlmError("Groq에 줄 장면 파일이 비어 있습니다.")
    mime = "image/jpeg"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        mime = "image/png"
    elif data[:4] == b"RIFF" and data[8:12] == b"WEBP":
        mime = "image/webp"
    encoded = base64.standard_b64encode(data).decode("ascii")
    return f"data:{mime};base64,{encoded}"


def _to_groq_messages(
    messages: list[dict[str, Any]], image: str | None
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    last_user = max(
        (i for i, msg in enumerate(messages) if msg.get("role") == "user"),
        default=-1,
    )
    for index, msg in enumerate(messages):
        role = str(msg.get("role") or "user")
        text = str(msg.get("content") or "")
        if image and index == last_user and role == "user":
            out.append(
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": text},
                        {
                            "type": "image_url",
                            "image_url": {"url": _image_data_uri(image)},
                        },
                    ],
                }
            )
            continue
        out.append({"role": role, "content": text})
    return out


class GroqCompleter:
    def __init__(
        self,
        *,
        keys: list[str] | None = None,
        model: str = DEFAULT_MODEL,
        max_tokens: int = 256,
        temperature: float = 0.2,
        http_open: Callable[..., Any] = urlopen,
        timeout: float = 45.0,
        sleeper: Callable[[float], None] = time.sleep,
        rate_limit_retries: int = 20,
        max_rate_limit_wait: float | None = None,
    ) -> None:
        self.keys = list(keys or load_groq_keys())
        if not self.keys:
            raise VlmError(
                "Groq 키가 없습니다. GROQ_API_KEY 또는 .groq_keys 를 넣으세요."
            )
        self.model_name = model
        # Planner-call budget is a semantic loop bound, not a key-count bound.
        # Groq TPM/key rotation is enforced independently below. Tying this to
        # len(keys) caused long but healthy embodied recoveries to stop one step
        # before pick after approach finally succeeded.
        try:
            configured_budget = int(os.environ.get("GROQ_PLANNER_CALL_BUDGET", "20"))
        except ValueError:
            configured_budget = 20
        self.planner_call_budget = max(1, configured_budget)
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.http_open = http_open
        self.timeout = timeout
        self.sleeper = sleeper
        self.rate_limit_retries = max(0, int(rate_limit_retries))
        if max_rate_limit_wait is None:
            max_rate_limit_wait = float(os.environ.get("GROQ_RATE_LIMIT_MAX_WAIT", "2.0"))
        self.max_rate_limit_wait = max(0.0, float(max_rate_limit_wait))
        self._index = 0
        self._lock = threading.Lock()
        # Per-key cooldown avoids hammering a key that already returned 429.
        # Timestamps use monotonic time so wall-clock adjustments cannot revive keys early.
        self._cooldown_until: dict[str, float] = {}

    def complete(self, messages: list[dict[str, Any]], image: str | None = None) -> str:
        with _shared_model_gate(self.model_name) as gate:
            shared_wait = gate.remaining()
            if shared_wait > 0:
                raise _TpmLimitError(
                    f"Groq TPM shared cooldown active for {self.model_name}; retry in {shared_wait:.3f}s",
                    retry_after=shared_wait, request_too_large=False,
                )
            try:
                return self._complete_model_locked(messages, image=image)
            except _TpmLimitError as exc:
                if not exc.request_too_large:
                    gate.block(exc.retry_after if exc.retry_after is not None else 1.0)
                raise

    def _complete_model_locked(self, messages: list[dict[str, Any]], image: str | None = None) -> str:
        payload = json.dumps(
            {
                "model": self.model_name,
                "messages": _to_groq_messages(messages, image),
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
                "reasoning_effort": "none",
                "reasoning_format": "hidden",
            }
        ).encode("utf-8")
        errors: list[str] = []
        tpm_limits: list[_TpmLimitError] = []
        ordinary_rate_limits: list[_RateLimitError] = []
        ordinary_non_rate_seen = False
        waited = 0.0
        for retry_round in range(self.rate_limit_retries + 1):
            with self._lock:
                now = time.monotonic()
                # Within one model attempt, exhaust every configured key in
                # priority order from the current start. After a successful
                # completion the start advances, distributing successive robot
                # planner calls across independent Groq budgets instead of
                # hammering key1 every step.
                shared_start = _shared_key_start(self.model_name, len(self.keys))
                start = self._index if shared_start is None else shared_start
                order = [self.keys[(start + offset) % len(self.keys)] for offset in range(len(self.keys))]
                ready = [key for key in order if self._cooldown_until.get(key, 0.0) <= now]
                wait_for = min(
                    (max(0.0, self._cooldown_until.get(key, 0.0) - now) for key in order),
                    default=0.0,
                )
            if not ready:
                if retry_round >= self.rate_limit_retries:
                    break
                delay = max(0.05, min(wait_for or 0.25, 10.0))
                remaining_wait = self.max_rate_limit_wait - waited
                if remaining_wait <= 0:
                    break
                delay = min(delay, remaining_wait)
                before = time.monotonic()
                self.sleeper(delay)
                waited += delay
                effective_now = max(time.monotonic(), before + delay)
                with self._lock:
                    for key in list(self._cooldown_until):
                        if self._cooldown_until[key] <= effective_now + 1e-6:
                            self._cooldown_until.pop(key, None)
                continue

            rate_limits: list[_RateLimitError] = []
            non_rate_errors: list[str] = []
            for key in ready:
                try:
                    text = self._post(key, payload)
                except _RateLimitError as exc:
                    rate_limits.append(exc)
                    ordinary_rate_limits.append(exc)
                    errors.append(str(exc))
                    with self._lock:
                        self._cooldown_until[key] = max(
                            self._cooldown_until.get(key, 0.0),
                            time.monotonic() + max(0.05, exc.retry_after),
                        )
                    continue
                except _TpmLimitError as exc:
                    # A configured key may belong to a different Groq budget.
                    # TPM participates in key fallback. Do not sleep here: the
                    # outer model-major fallback must exhaust every model/key
                    # combination before any temporary-budget wait occurs.
                    tpm_limits.append(exc)
                    non_rate_errors.append(str(exc))
                    errors.append(str(exc))
                    continue
                except VlmError as exc:
                    ordinary_non_rate_seen = True
                    non_rate_errors.append(str(exc))
                    errors.append(str(exc))
                    continue
                with self._lock:
                    idx = self.keys.index(key)
                    self._index = (idx + 1) % len(self.keys)
                    self._cooldown_until.pop(key, None)
                return text

            if rate_limits and not non_rate_errors and retry_round < self.rate_limit_retries:
                # Honor provider Retry-After exactly when all attempted keys are limited.
                delay = max((item.retry_after for item in rate_limits), default=0.25)
                delay = max(0.05, min(delay or 0.05, 10.0))
                remaining_wait = self.max_rate_limit_wait - waited
                if remaining_wait <= 0:
                    break
                delay = min(delay, remaining_wait)
                before = time.monotonic()
                self.sleeper(delay)
                waited += delay
                effective_now = max(time.monotonic(), before + delay)
                with self._lock:
                    for key in list(self._cooldown_until):
                        if self._cooldown_until[key] <= effective_now + 1e-6:
                            self._cooldown_until.pop(key, None)
                continue
            break
        # Preserve TPM metadata so the model-level fallback can distinguish a
        # permanently oversized request from a temporarily exhausted minute
        # bucket. Prefer the soonest temporary budget if at least one exists.
        if tpm_limits:
            temporary = [item for item in tpm_limits if not item.request_too_large]
            if temporary:
                chosen = min(
                    temporary,
                    key=lambda item: item.retry_after if item.retry_after is not None else 1.0,
                )
                raise _TpmLimitError(
                    str(chosen), retry_after=chosen.retry_after, request_too_large=False
                ) from chosen
            chosen = tpm_limits[-1]
            raise _TpmLimitError(
                str(chosen), retry_after=None, request_too_large=True
            ) from chosen
        if ordinary_rate_limits and not ordinary_non_rate_seen:
            chosen = min(ordinary_rate_limits, key=lambda item: item.retry_after)
            raise _RateLimitError(chosen.retry_after, chosen.detail) from chosen
        raise VlmError("Groq 요청이 모두 실패했습니다. " + (errors[-1] if errors else ""))


    def _post(self, key: str, payload: bytes) -> str:
        request = Request(
            GROQ_URL,
            data=payload,
            method="POST",
            headers={
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
                "User-Agent": "ugrp-harness/1.0",
            },
        )
        try:
            with self.http_open(request, timeout=self.timeout) as response:
                raw = response.read()
        except HTTPError as exc:
            status = getattr(exc, "code", 0)
            detail = _http_error_detail(exc)
            if status == 401:
                raise VlmError("Groq 키가 거절되었습니다.") from exc
            detail_l = detail.lower()
            if status in {413, 429} and ("tokens per minute" in detail_l or "tpm" in detail_l):
                too_large = "request too large" in detail_l or "reduce your message size" in detail_l
                retry_after = None if too_large else _retry_after_seconds(exc, detail)
                raise _TpmLimitError(
                    f"Groq TPM 한도 초과: {detail}",
                    retry_after=retry_after,
                    request_too_large=too_large,
                ) from exc
            if status == 429:
                raise _RateLimitError(_retry_after_seconds(exc, detail), detail) from exc
            if detail:
                raise VlmError(f"Groq HTTP {status}: {detail}") from exc
            raise VlmError(f"Groq HTTP {status}") from exc
        except (OSError, URLError) as exc:
            raise VlmError("Groq에 연결하지 못했습니다.") from exc
        try:
            body = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise VlmError("Groq가 JSON이 아닌 답을 줬습니다.") from exc
        choices = body.get("choices")
        if not isinstance(choices, list) or not choices:
            raise VlmError("Groq 답이 비어 있습니다.")
        message = choices[0].get("message") if isinstance(choices[0], dict) else None
        if not isinstance(message, dict):
            raise VlmError("Groq 답이 비어 있습니다.")
        text = message.get("content")
        if not isinstance(text, str) or not text.strip():
            raise VlmError("Groq 답이 비어 있습니다.")
        return _strip_think(text)


class GroqFallbackCompleter:
    """Model-major fallback with one bounded retry of temporary TPM budgets."""

    def __init__(
        self,
        completers: list[GroqCompleter],
        *,
        tpm_matrix_retries: int | None = None,
        tpm_matrix_max_wait: float | None = None,
        sleeper: Callable[[float], None] = time.sleep,
    ):
        if not completers:
            raise ValueError("at least one Groq completer is required")
        self.completers = completers
        self.keys = completers[0].keys
        self.model_name = completers[0].model_name
        self.last_model = self.model_name
        try:
            configured_budget = int(os.environ.get("GROQ_PLANNER_CALL_BUDGET", "20"))
        except ValueError:
            configured_budget = 20
        self.planner_call_budget = max(1, configured_budget)
        if tpm_matrix_retries is None:
            tpm_matrix_retries = int(os.environ.get("GROQ_TPM_MATRIX_RETRIES", "0"))
        if tpm_matrix_max_wait is None:
            tpm_matrix_max_wait = float(os.environ.get("GROQ_TPM_MATRIX_MAX_WAIT", "15"))
        self.tpm_matrix_retries = max(0, int(tpm_matrix_retries))
        self.tpm_matrix_max_wait = max(0.0, float(tpm_matrix_max_wait))
        self.sleeper = sleeper

    def complete(self, messages: list[dict[str, Any]], image: str | None = None) -> str:
        errors: list[str] = []
        waited = 0.0
        last_tpm: _TpmLimitError | None = None
        ordinary_rate_limits: list[_RateLimitError] = []
        ordinary_non_rate_seen = False
        for matrix_round in range(self.tpm_matrix_retries + 1):
            temporary_tpm: list[_TpmLimitError] = []
            for completer in self.completers:
                try:
                    text = completer.complete(messages, image=image)
                    self.last_model = completer.model_name
                    return text
                except _TpmLimitError as exc:
                    # Model-major order remains exact: model1 exhausts all keys,
                    # then model2 exhausts all keys. Temporary TPM is considered
                    # for waiting only after the entire matrix has failed.
                    last_tpm = exc
                    if not exc.request_too_large:
                        temporary_tpm.append(exc)
                    errors.append(f"{completer.model_name}: {exc}")
                    continue
                except _RateLimitError as exc:
                    ordinary_rate_limits.append(exc)
                    errors.append(f"{completer.model_name}: {exc}")
                    continue
                except VlmError as exc:
                    ordinary_non_rate_seen = True
                    errors.append(f"{completer.model_name}: {exc}")
                    continue

            if temporary_tpm and matrix_round < self.tpm_matrix_retries:
                delay = min(
                    (item.retry_after if item.retry_after is not None else 1.0)
                    for item in temporary_tpm
                )
                remaining = self.tpm_matrix_max_wait - waited
                if remaining <= 0:
                    break
                delay = min(max(0.05, delay), remaining)
                self.sleeper(delay)
                waited += delay
                continue
            break

        if last_tpm is not None:
            raise VlmError(str(last_tpm)) from last_tpm
        if ordinary_rate_limits and not ordinary_non_rate_seen:
            chosen = min(ordinary_rate_limits, key=lambda item: item.retry_after)
            raise _RateLimitError(chosen.retry_after, chosen.detail) from chosen
        raise VlmError("Groq 모델 fallback까지 실패했습니다. " + (errors[-1] if errors else ""))



def _retry_after_seconds(exc: HTTPError, detail: str = "") -> float:
    raw = ""
    headers = getattr(exc, "headers", None)
    if headers is not None:
        try:
            raw = str(headers.get("Retry-After") or "").strip()
        except Exception:
            raw = ""
    try:
        if raw:
            return max(0.0, float(raw))
    except ValueError:
        pass
    match = _RETRY_RE.search(detail or "")
    if match:
        try:
            value = max(0.0, float(match.group(1)))
            return value / 1000.0 if match.group(2).lower() == "ms" else value
        except ValueError:
            pass
    return 1.0

def _strip_think(text: str) -> str:
    cleaned = _THINK_RE.sub("", text).strip()
    return cleaned or text.strip()


def _http_error_detail(exc: HTTPError) -> str:
    try:
        raw = exc.read().decode("utf-8", errors="replace")
    except Exception:
        return ""
    try:
        body = json.loads(raw)
    except json.JSONDecodeError:
        return raw[:1000].strip()
    err = body.get("error") if isinstance(body, dict) else None
    if isinstance(err, dict):
        msg = str(err.get("message") or err.get("code") or "").strip()
        return msg[:1000]
    return ""


def live_completer(backend: str = "auto", model: str | None = None):
    """Return (completer, backend_name). Replay is handled by the caller."""
    chosen = backend
    if chosen == "auto":
        chosen = "groq" if load_groq_keys() else "mlx"
    if chosen == "groq":
        retries = int(os.environ.get("GROQ_RATE_LIMIT_RETRIES", "2"))
        try:
            request_timeout = float(os.environ.get("GROQ_REQUEST_TIMEOUT", "45"))
        except ValueError:
            request_timeout = 45.0
        request_timeout = max(1.0, request_timeout)
        if model:
            return GroqCompleter(
                model=model, rate_limit_retries=retries, timeout=request_timeout
            ), "groq"
        keys = load_groq_keys()
        primary_model = DEFAULT_MODEL
        fallback_model = "qwen/qwen3.6-27b"
        if os.environ.get("GROQ_ROBOT_MODEL_SHARDING", "0") == "1":
            robot_id = os.environ.get("UGRP_ROBOT_ID", "r1").strip().lower()
            if robot_id == "r2":
                primary_model, fallback_model = fallback_model, primary_model
        primary = GroqCompleter(
            keys=keys, model=primary_model,
            rate_limit_retries=retries, timeout=request_timeout,
        )
        fallback = GroqCompleter(
            keys=keys, model=fallback_model,
            rate_limit_retries=min(1, retries), timeout=request_timeout,
        )
        return GroqFallbackCompleter([primary, fallback]), "groq"
    if chosen == "gemini":
        from .gemini_proxy import DEFAULT_MODEL as gemini_model
        from .gemini_proxy import GeminiProxyCompleter

        return GeminiProxyCompleter(model=model or gemini_model), "gemini"
    from .vlm import DEFAULT_MODEL as mlx_model
    from .vlm import MlxVlmCompleter

    return MlxVlmCompleter(model=model or mlx_model), "mlx"
