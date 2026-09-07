"""Independent visual verification for physical-task final answers."""

from __future__ import annotations

import json
import re
from typing import Any

from .vlm import VlmError

_FENCE_RE = re.compile(r"^```(?:json)?\s*|\s*```$", re.IGNORECASE)

_VERIFY_PROMPT = """You are an independent visual verifier for a robot agent.
Ignore the actor's previous reasoning and action claims.
You receive the original goal, the actor's proposed final answer, and the latest camera image.
Return exactly one JSON object:
{"accept": true|false, "success_claim": true|false, "evidence": "short Korean evidence"}
Rules:
- If the proposed final reports failure, missing target, safe abort, or asks the user for help, set accept=true and success_claim=false. Do not force the robot to keep trying.
- If the proposed final claims the physical goal succeeded, set success_claim=true and accept=true only when the latest image visibly supports that success.
- If the image is ambiguous or still shows the pre-success state, accept=false.
- Be conservative. Never trust the actor's claim over the image.
"""


def verify_final(completer, goal: str, final_text: str, image: str) -> tuple[bool, str]:
    user = (
        "Original goal:\n" + goal.strip() + "\n\n"
        "Proposed final answer:\n" + final_text.strip() + "\n\n"
        "Verify against the latest image."
    )
    raw = completer.complete(
        [
            {"role": "system", "content": _VERIFY_PROMPT},
            {"role": "user", "content": user},
        ],
        image=image,
    )
    obj = _parse_object(raw)
    accept = obj.get("accept")
    if not isinstance(accept, bool):
        raise VlmError("검증 모델이 accept boolean을 주지 않았습니다.")
    evidence = str(obj.get("evidence") or "검증 근거 없음").strip()
    return accept, evidence


def _parse_object(raw: str) -> dict[str, Any]:
    text = _FENCE_RE.sub("", raw.strip()).strip()
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start < 0 or end <= start:
            raise VlmError("검증 모델이 JSON을 주지 않았습니다.")
        try:
            value = json.loads(text[start : end + 1])
        except json.JSONDecodeError as exc:
            raise VlmError("검증 모델 JSON을 읽지 못했습니다.") from exc
    if not isinstance(value, dict):
        raise VlmError("검증 모델 응답이 object가 아닙니다.")
    return value
