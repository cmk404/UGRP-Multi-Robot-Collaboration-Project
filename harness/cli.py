"""CLI for the allowlisted LLM tool harness."""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Sequence

from .catalog import default_registry
from .loop import Completer, ReplayCompleter, run_loop


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="장면을 보고 말로 맞춘 다음, 지정한 Python 도구만 실행합니다. 기본은 dry-run."
    )
    parser.add_argument("task", nargs="?", help="모델에게 줄 작업 문장")
    parser.add_argument("--list-tools", action="store_true")
    parser.add_argument(
        "--chat",
        action="store_true",
        help="브라우저에서 장면+말로 조율하고 스킬을 실행합니다.",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "groq", "gemini", "mlx"),
        default="auto",
        help="모델 백엔드. gemini 는 로컬 Antigravity subscription proxy를 사용합니다.",
    )
    parser.add_argument("--model", default=None, help="백엔드 모델 이름")
    parser.add_argument(
        "--host", default="127.0.0.1",
        help="채팅 서버 주소. 기본 127.0.0.1 (tailnet 노출은 tailscale serve 경유). 0.0.0.0 은 --password 필수.",
    )
    parser.add_argument("--port", type=int, default=8080, help="채팅 서버 포트")
    parser.add_argument(
        "--password",
        default=os.environ.get("UGRP_CHAT_PASSWORD"),
        help="브라우저 Basic Auth 비밀번호. 외부 바인딩이면 필수.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="실제로 도구 함수를 실행합니다. 없으면 호출 계획만 반환합니다.",
    )
    parser.add_argument("--max-steps", type=int, default=12)
    parser.add_argument("--image", help="선택. 비전 모델에 줄 이미지 경로")
    parser.add_argument("--robot", default="ugrp1", help="MasterPi SSH 별칭. 기본 ugrp1")
    parser.add_argument(
        "--robot-id", choices=("r1", "r2", "r3"),
        default=os.environ.get("UGRP_ROBOT_ID", "r1"),
        help="3대 협업 슬롯 ID. 기본 r1",
    )
    parser.add_argument(
        "--camera-url",
        help="선택. MasterPi 카메라 JPEG URL. 없으면 SSH로 로봇의 :8080/snapshot 을 읽습니다.",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="카메라 연결을 시도하지 않습니다. REAL 탭을 하드웨어 연결 전 안전하게 띄울 때 사용합니다.",
    )
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
    if args.chat:
        from .web import serve_chat

        replies = json.loads(args.replay) if args.replay else None
        host = args.host
        if host not in {"127.0.0.1", "localhost", "::1"} and not args.password:
            print(
                "password 없음: 127.0.0.1 만 엽니다. 밖에 열려면 --password 를 주세요.",
                file=sys.stderr,
            )
            host = "127.0.0.1"
        serve_chat(
            replies=replies,
            backend=args.backend,
            model=args.model,
            host=host,
            port=args.port,
            password=args.password,
            execute=args.execute,
            robot=args.robot,
            robot_id=args.robot_id,
            camera_url=args.camera_url,
            camera=not args.no_camera,
            actions_path=args.actions,
            max_steps=args.max_steps,
        )
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
