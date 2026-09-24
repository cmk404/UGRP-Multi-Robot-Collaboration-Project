"""CLI for the allowlisted LLM tool harness."""

from __future__ import annotations

import argparse
import json
from typing import Any, Sequence

from .catalog import default_registry
from .loop import Completer, ReplayCompleter, run_loop


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="작업 문장을 받아 허용된 Python 도구만 실행합니다. 기본은 dry-run."
    )
    parser.add_argument("task", nargs="?", help="모델에게 줄 작업 문장")
    parser.add_argument("--list-tools", action="store_true")
    parser.add_argument(
        "--backend",
        choices=("auto", "groq", "gemini", "mlx"),
        default="auto",
        help="모델 백엔드. gemini 는 로컬 Antigravity subscription proxy를 사용합니다.",
    )
    parser.add_argument("--model", default=None, help="백엔드 모델 이름")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="실제로 도구 함수를 실행합니다. 없으면 호출 계획만 반환합니다.",
    )
    parser.add_argument("--max-steps", type=int, default=12)
    parser.add_argument("--image", help="선택. 비전 모델에 줄 이미지 경로")
    parser.add_argument(
        "--replay",
        metavar="JSON",
        help='모델 응답을 스크립트로 재생. 예: \'[{"say":"다가갈게","tool":"approach"},{"final":"도착"}]\'',
    )
    parser.add_argument(
        "--actions",
        help="실행 가능한 Python 도구 파일. 기본은 scripts/robot_actions.py",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    registry = default_registry(actions_path=args.actions)
    if args.list_tools:
        print(registry.prompt_schema())
        return 0
    if not args.task:
        build_parser().print_help()
        return 2
    completer: Completer
    if args.replay:
        completer = ReplayCompleter(json.loads(args.replay))
    else:
        from .groq import live_completer

        completer, _name = live_completer(args.backend, args.model)
    result = run_loop(
        completer,
        registry,
        args.task,
        image=args.image,
        max_steps=args.max_steps,
        execute=args.execute,
    )
    print(json.dumps(_public_result(result), ensure_ascii=False, indent=2))
    return 0 if result.stopped == "final" else 1


def _public_result(result) -> dict[str, Any]:
    steps = []
    for step in result.steps:
        item: dict[str, Any] = {"raw": step.raw}
        if step.error:
            item["error"] = step.error
        if step.result is not None:
            item["result"] = step.result
        if step.action is not None:
            item["action"] = type(step.action).__name__
        steps.append(item)
    return {"final": result.final, "stopped": result.stopped, "steps": steps}


if __name__ == "__main__":
    raise SystemExit(main())
