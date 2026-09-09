# 실행 스크립트 인덱스

스크립트는 기존 자동화와 실험 기록이 참조하는 경로를 보존하기 위해 한 디렉터리에 둔다.
파일을 옮기는 대신 아래 목적과 이름 규칙으로 찾는다. 실행 전에는
[`CONTRIBUTING.md`](../CONTRIBUTING.md)와 관련 실험 문서를 확인한다.

## 자주 쓰는 진입점

| 목적 | 스크립트 |
|---|---|
| 자동 테스트 | `run_ci_tests.py` |
| 관리되는 프로세스 세션 | `ugrp_session.py` |
| 시뮬레이션 워커 | `run_mujoco_worker.py`, `run_mujoco_ws_worker.py` |
| 실물 로봇 서비스 | `serve_real.sh`, `start_masterpi_dashboard.sh` |
| 시뮬레이션 서비스 | `serve_sim_coworker.sh`, `serve_coworker.sh` |
| 단일 빨간 블록 작업 | `red_block/task_runner.py`와 `pick_red_block.py`, `fetch_red_block.py` |

## 이름별 분류

| 접두사·경로 | 역할 |
|---|---|
| `analyze_*` | 기록된 trace의 오프라인 분석 |
| `eval_*`, `evaluate_*` | 정책·환경·코호트 평가 |
| `probe_*` | 범위를 제한한 진단 실행 |
| `record_*` | 영상·trace·시험 결과 기록 |
| `render_*` | 기록 결과의 보고서·영상 렌더링 |
| `replay_*`, `review_*` | 기존 기록 재생과 검토 자료 생성 |
| `run_*` | 워커 또는 고정 조건 실행 |
| `serve_*`, `start_*`, `activate_*` | 서비스와 실물 스택 시작 |
| `stress_*` | 여러 조건의 스트레스·일반화 검사 |
| `train_*`, `transfer_*` | 학습과 전이 학습 |
| `verify_*` | 산출물·학습 환경 검증 |
| `*_worker_recover.py`, `sim_worker_failover.py` | 원격 워커 복구·전환 |
| [`red_block/`](red_block/) | 빨간 블록 작업의 단계별 실행 모듈 |
| [`benchmarks/`](benchmarks/) | 캘리브레이션·물리 충실도·장기 벤치마크 |
| [`systemd/`](systemd/) | Linux 서비스 단위 파일 |

`experimental/` 아래 코드는 운영 경로가 아니다. 새 스크립트는 기존 접두사를 따르고,
재사용되는 로직은 스크립트에 복제하지 말고 `harness/` 또는 `sim/` 모듈에 둔다.
