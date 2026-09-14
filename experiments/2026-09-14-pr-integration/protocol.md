# 정식 PR 9개 통합 검증

사용자가 프로젝트 정리의 1번 우선순위 진행을 요청하여, 의존 PR을 별도 브랜치에서 병합하고 검토한다. main 병합은 결과와 실제 PR을 제시한 뒤 사용자 승인으로 진행한다.

## 범위

기준 main `266a4e76c5ae3b028a4dd325481495d61caa6f21`에서 다음 순서로 원래 커밋 이력을 보존했다. 충돌은 없었다.

| PR | 포함 HEAD | 내용 |
|---|---|---|
| #31 | c23602e2e17727165aaa5fb65773e8b40c365f88 | 국소 파지 복구 |
| #32 | 6542c3504449547f60510f88c34cf44f2b6cf84c | 직진 접근 |
| #33 | 10406542fc4962cadc73b7b09635f2c578385be3 | 시작 자세 변동 접근 |
| #34 | 690bead094195404fd8de9f6b86d12a8ea249852 | 짧은 공동 운반 |
| #40 | 1efee835f8ab5d6bf432d54f58f357d5955058ab | 두 로봇 운반 동기화 |
| #36 | 3ad947391a2ff84c3cb325121d4829a3e32dce37 | 정적 지도 주행 |
| #38 | c5f29218ae533f797d10d96bc73f1ffde59c360f | 회전 후 전진 |
| #27 | 650f9222793fb1863091491a6eaa8f1a283a6f70 | 각자 PC의 Gemini 프록시 |
| #28 | 17d0c4ce4727df169cdc8f5fe24b611d2755feb1 | 승인된 병합 후 기본 체크아웃 동기화 |

초안 #41(설계 문서), #42(단일 로봇 skill/semantic 비교)는 포함하지 않는다. 정책·모델·카메라·물리 설정을 재조정하지 않는다. 통합 후 README와 실험 인덱스만 최신 기능 범위에 맞춘다.

## 실행 전 고정한 검사

1. 통합 소스 전체의 portable offline suite.
2. 커밋된 모델 ZIP과 pair-carry RGB 증거를 새 폴더에 해시 검증하여 복원. 기존 55개 기록을 통합 코드로 감사한다. 예상: 감사 55/55, 재계산한 물리 성공 49/55. 새 물리 실행과 구분한다.
3. `run_checks.py`에 고정한 기존 조건 9회를 MuJoCo로 새로 실행한다: 직진 접근, 다양한 시작 접근, 정상 동기화 운반, r1 출발 1초 지연의 baseline/sync 쌍, r3 중간 0.75초 지연 sync, r3 보고 1초 누락 sync, r1 slalom heading 도착, r1 narrow 진입 거부.
4. 저장한 실제 RGB·발행 명령·보고·평가로 각 새 실행을 감사한다. baseline 출발 지연의 물리 실패와 narrow의 `no_map_route`를 사전 예상 결과로 유지한다. 실패 결과도 보관한다.
5. 대표 시뮬레이션 영상의 시작·중간·종료 장면을 직접 검토하고 범위를 기록한다. 새 조건 일반화, 하드웨어, 실제 LLM 또는 네트워크 지연 검증으로 확대 해석하지 않는다.
6. 최종 브랜치 push 후 GitHub의 fresh-checkout CI를 확인한다. Ubuntu 프록시 bootstrap은 격리된 임시 인증 폴더와 포트에서 검사하며, 개인 Google 로그인·실제 모델 응답 성공을 뜻하지 않는다.

`run_checks.py`와 이 프로토콜을 커밋한 뒤 실행하며 소스를 고정한다. 실행 SHA·환경·명령·모든 종료 코드·결과·해시는 별도 결과 기록에 남긴다. 원본은 실행 worktree의 ignored `outputs/integration-review/`에 보관한다. UGRP는 Drive를 사용하지 않는다.

```sh
<python> scripts/ugrp_session.py run pr-integration-20260914 -- \
  <python> experiments/2026-09-14-pr-integration/run_checks.py \
  --python <python> --mjpython <mjpython> \
  --out outputs/integration-review/runtime
```

macOS는 기존 `.venv-sim-worker-mac/bin/python`과 `mjpython`을 사용한다. 새로운 서비스를 남기지 않고 이 세션의 자식 프로세스만 정리한다. 원래 main과 기존 feature worktree는 수정하지 않는다.
