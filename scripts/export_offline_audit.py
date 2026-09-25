#!/usr/bin/env python3
"""오프라인 감사 파생 뷰 → TensorBoard 이벤트 (선택 의존성).

실행 기록·학습 기록은 `scripts/export_tensorboard.py`를 쓴다. 이 진입점은 오프라인 채점·
물리 감사처럼 실행이 아닌 평가 산출물의 파생 뷰만 변환한다. 원본은 읽기만 한다.
"""
import sys
from pathlib import Path

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from scripts.tensorboard_tools.offline_audit import main
    raise SystemExit(main())
