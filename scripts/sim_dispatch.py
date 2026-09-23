"""Small local launcher for the established peer-plan -> RGB skills pipeline."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import os
from pathlib import Path
import sys
from uuid import uuid4
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def bundled_models():
    """Restore versioned local weights without replacing any existing artifact."""
    archive = ROOT / 'experiments/dispatch-skill-integration-20260917/models.zip'
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    target = ROOT / 'outputs' / 'dispatch-models' / digest[:16]
    with zipfile.ZipFile(archive) as bundle:
        for entry in bundle.infolist():
            path = target / entry.filename
            if not path.resolve().is_relative_to(target.resolve()):
                raise ValueError('model bundle contains an invalid path')
            if entry.is_dir():
                continue
            content = bundle.read(entry)
            if path.exists():
                if path.read_bytes() != content:
                    raise ValueError(f'bundled model changed: {path}; supply your own model directories')
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('xb') as stream:
                    stream.write(content)
    return target / 'models/grasp', target / 'models/varied'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', help='three peers receive this instruction within the existing beam/box dispatch mission')
    parser.add_argument('--plan-replay', type=Path, help='explicit saved-plan diagnostic instead of new LLM negotiation')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--grasp-model-dir', type=Path)
    parser.add_argument('--stage-model-dir', type=Path)
    parser.add_argument('--model', default=os.environ.get('UGRP_SIM_MODEL', 'gemini-3.8-flash'))
    parser.add_argument('--full-capture', action='store_true',
                        help='keep the original extra pair camera and overview captures')
    parser.add_argument('--serial-route', action='store_true',
                        help='keep the original serial resource gate even for independent open routes')
    # Remaining options use the original runner's parser and validation.
    args, rest = parser.parse_known_args(argv)
    if '--executor' in rest or any(a.startswith('--executor=') for a in rest):
        parser.error('dispatch uses the existing skills executor; raw diagnostics have a separate entry point')
    if args.serial_route and any(flag in rest for flag in ('--route-overlap', '--auto-route-overlap')):
        parser.error('--serial-route conflicts with explicit route overlap')
    if args.full_capture and '--efficient-capture' in rest:
        parser.error('--full-capture conflicts with --efficient-capture')
    if bool(args.grasp_model_dir) != bool(args.stage_model_dir):
        parser.error('supply both --grasp-model-dir and --stage-model-dir')
    if not args.headless and sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('native window requires a desktop display; use --headless on a server')
    if not args.task and not args.plan_replay and sys.stdin.isatty():
        print('기존 출하 작업: beam은 두 대, box는 한 대가 같은 dock으로 운반합니다.')
        args.task = input('계획에 전달할 지시 [기존 임무]: ').strip() or None
    if args.task and args.plan_replay:
        parser.error('--task requires new planning; saved-plan replay cannot apply a new instruction')
    if not args.grasp_model_dir:
        args.grasp_model_dir, args.stage_model_dir = bundled_models()
    output = args.output or ROOT / 'outputs' / f'dispatch-{datetime.now():%Y%m%d-%H%M%S}-{uuid4().hex[:6]}'
    forwarded = ['--executor', 'skills', '--output', str(output),
                 '--grasp-model-dir', str(args.grasp_model_dir),
                 '--stage-model-dir', str(args.stage_model_dir), '--model', args.model]
    if not args.full_capture and '--efficient-capture' not in rest:
        forwarded.append('--efficient-capture')
    if not args.serial_route and not any(flag in rest for flag in ('--route-overlap', '--auto-route-overlap')):
        forwarded.append('--auto-route-overlap')
    if not args.headless:
        forwarded.append('--viewer')
    if args.task:
        forwarded += ['--task', args.task]
    if args.plan_replay:
        forwarded += ['--plan-replay', str(args.plan_replay)]
    print('기존 계획 합의 → 로봇별 프로그램 → RGB 스킬 실행' if not args.plan_replay
          else '저장된 계획 재생 진단 → 기존 RGB 스킬 실행 (새 LLM 계획 아님)', flush=True)
    print(f'결과: {output.resolve()}', flush=True)
    from scripts.run_dispatch_e2e import main as run_dispatch
    return run_dispatch(forwarded + rest)


def choose():
    print('\n1. 공동 계획 → 로봇별 기존 RGB 스킬 실행 (LLM 사용)\n'
          '2. 저장된 plan으로 기존 스킬 실행 (모델 협상 없는 재생 진단)\n'
          '3. 수동 저수준 명령 (모델 불필요)\n'
          '4. 설정 파일의 actions / Python controllers 실행')
    choice = input('작동 방식 [1]: ').strip() or '1'
    from scripts.sim_cli import main as sim_cli
    if choice == '1':
        return sim_cli(['dispatch'])
    if choice == '2':
        path = input('committed-plan.json 경로: ').strip()
        if not path:
            raise ValueError('saved plan path required; a plan is never chosen implicitly')
        return sim_cli(['dispatch', '--plan-replay', path])
    if choice == '3':
        return sim_cli(['console', str(ROOT/'configs/simulation/local.json'), '--mode', 'manual'])
    if choice == '4':
        config = input('설정 파일 경로: ').strip()
        if not config:
            raise ValueError('actions/controllers config path required')
        return sim_cli(['console', config, '--mode', 'script'])
    raise ValueError('작동 방식은 1..4 중 선택하세요')
