"""Qwen-VL completer via mlx-vlm. The model sees the scene, then emits JSON."""

from __future__ import annotations

from typing import Any


DEFAULT_MODEL = "mlx-community/Qwen3-VL-4B-Instruct-4bit"


class VlmError(RuntimeError):
    """Raised when the local vision model cannot answer."""


class MlxVlmCompleter:
    def __init__(
        self,
        *,
        model: str = DEFAULT_MODEL,
        max_tokens: int = 256,
        temperature: float = 0.2,
    ) -> None:
        self.model_name = model
        self.max_tokens = max_tokens
        self.temperature = temperature
        self._bundle: tuple[Any, Any, Any] | None = None

    def _load(self) -> tuple[Any, Any, Any]:
        if self._bundle is not None:
            return self._bundle
        try:
            from mlx_vlm import load
            from mlx_vlm.utils import load_config
        except ImportError as exc:
            raise VlmError(
                "mlx-vlm 이 없습니다. conda mlx_env 에서 이 서버를 실행하세요."
            ) from exc
        try:
            model, processor = load(self.model_name)
            config = load_config(self.model_name)
        except Exception as exc:
            raise VlmError(f"비전 모델을 불러오지 못했습니다: {exc}") from exc
        self._bundle = (model, processor, config)
        return self._bundle

    def complete(self, messages: list[dict[str, Any]], image: str | None = None) -> str:
        from mlx_vlm import generate
        from mlx_vlm.prompt_utils import apply_chat_template

        model, processor, config = self._load()
        prompt = apply_chat_template(
            processor,
            config,
            messages,
            num_images=1 if image else 0,
        )
        try:
            result = generate(
                model,
                processor,
                prompt,
                image=image,
                max_tokens=self.max_tokens,
                temp=self.temperature,
                verbose=False,
            )
        except Exception as exc:
            raise VlmError(f"비전 모델 생성 실패: {exc}") from exc
        text = getattr(result, "text", None)
        if not isinstance(text, str) or not text.strip():
            raise VlmError("비전 모델이 빈 답을 반환했습니다.")
        return text
