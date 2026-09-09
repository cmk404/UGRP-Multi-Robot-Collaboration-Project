# 개발·검증·실험 관리

## 변경 절차

1. 기존 이슈를 확인하거나, 문제·완료 기준을 담은 이슈를 만든다.
2. `main`에서 작업 브랜치를 만들고 범위가 분명한 변경을 한다. 이미 다른 변경이 있으면 보존하고 범위를 구분한다.
3. 관련 자동 테스트를 실행하고 PR에 결과를 적는다. 실제 동작을 바꾼 경우 영향받는 구간을 검증한다.
4. 실험을 실행할 코드를 먼저 커밋한다. 실행 중 소스를 바꾸지 않는다. 원인을 좁히는 진단 후 최종 후보로 전체 비교 조건을 실행한다.
5. 실험 ID·코드 SHA·환경·설정·전체 결과·제약을 `experiments/`에 기록한다. 실패도 남기고 새로운 결과는 기존 결과를 덮어쓰지 않는다.
6. CI와 검토가 끝난 PR을 병합한다. `main`에는 PR과 `offline-regressions` 통과, 최신 base 반영 및 미해결 대화 해소를 요구하는 GitHub 보호 규칙이 설정돼 있다. 강제 push와 브랜치 삭제는 금지한다. 별도 승인 리뷰 수는 0이며 관리자 우회 권한은 유지한다.

작은 문서 수정에는 전체 시뮬레이션이 필요 없다. CI는 외부 모델 호출이나 하드웨어 검증을 대신하지 않는다.

## 자동 테스트

Python 3.12를 사용한다. 로컬의 검증된 MuJoCo 환경은 `.venv-sim-worker-mac`이다. `.venv-sim`은 이 환경의 호환 링크다. 예전 클라우드 배포 환경은 [퇴역 기록](docs/cloud_simulation.md)으로 남기고 제거했다. 새 자동 테스트 환경은 다음과 같이 만든다.

```sh
python3.12 -m venv .venv-test
.venv-test/bin/python -m pip install -r requirements-test.txt
```

테스트 실행:

```sh
.venv-test/bin/python scripts/run_ci_tests.py
```

정확한 CI 테스트 명령과 포함 범위는 [workflow](.github/workflows/tests.yml)를 따른다. 테스트 전용 의존성과 실제 실험 의존성은 구분한다.

## 실제 실험 재현

실행 당시 설치 버전은 각 실험의 `environment.json`에 남긴다. 기존 시뮬레이션 의존성은 `requirements-sim.txt`에 있다. 이 파일에 적힌 모든 버전이 다른 플랫폼의 패키지 저장소에 존재하는지는 별도 확인해야 한다. 자동 테스트용 환경만으로 기존 실험 환경을 재현했다고 주장하지 않는다.

Gemini 실험에는 접근 가능한 모델 프록시가 필요하다. `GEMINI_PROXY_URL`을 본인의 프록시 주소로 설정한다. 인증정보는 저장소에 넣지 않는다. 현재 기본 설정이 특정 개발자의 로컬 서비스를 가리킬 수 있으므로 새 환경에서는 명시적으로 설정한다.

다음 명령은 **실제 모델 호출과 비용**이 발생하는 수동 실험이며 CI에서 실행하지 않는다. 출력 폴더는 새 이름을 사용해야 한다.

```sh
PYTHONPATH=. .venv-sim-worker-mac/bin/python scripts/ugrp_session.py run markerless-validation -- \
  .venv-sim-worker-mac/bin/python -m scripts.run_gemini_seed_validation \
  --execute --output outputs/markerless-validation-NEW-ID \
  --seeds 45 46 42 43 44 --reasoning-effort medium --request-timeout 60
```

현재 실행기는 로봇1대, Gemini 3.8 Flash, 조건당 30호출·120,000입력토큰·300SIM초를 사용한다. 모델 가용성은 프록시에서 확인해야 한다. 결과의 정확한 재생을 보장하지 않으며 모델 응답·지연이 달라질 수 있다.

완료된 실행 하나에 대해:

```sh
PYTHONPATH=. .venv-sim-worker-mac/bin/python scripts/verify_gemini_budget_run.py \
  outputs/markerless-validation-NEW-ID/solo-45 --seed 45 --max-input-tokens 120000
.venv-sim-worker-mac/bin/python scripts/review_navigation_trial.py \
  outputs/markerless-validation-NEW-ID/solo-45
```

생성된 카메라 입력과 영상도 직접 검토하고, 검토 범위와 한계를 기록한다. 자동 검사 통과만으로 실제 운반 성공을 선언하지 않는다. 원시 로그의 평가 좌표를 제어 입력으로 사용하지 않는다.

## 서비스 수명

지속 프로세스는 `scripts/ugrp_session.py run <이름> -- <명령>`으로 시작한다. 완료 시 세션 전체가 종료되는지 확인한다. 필요하면 `python3 scripts/ugrp_session.py stop <이름>`으로 해당 세션만 중지한다. 다른 작업의 프로세스를 이름 패턴만으로 종료하지 않는다.

## 증거와 저장 범위

- `tests/fixtures/`: 작은 재현용 입력, 출처와 SHA를 함께 저장한다.
- `experiments/<ID>/`: 코드 SHA, 환경, 설정, 모든 결과, 원본 해시, 실제 저장 위치.
- `outputs/`: raw 영상과 로그. 현재 로컬 보관이며 Git에서 제외된다. 다른 사람이 필요로 하면 허가된 저장소에 공유하고 접근 가능한 위치를 결과 기록에 추가한다.
- 보관만 한 해시는 백업이 아니다. raw 자료가 공유·백업되지 않았으면 그대로 명시한다.
- 실험 중복 ZIP, 개인 계정 설정, 키·토큰, 가상환경과 의존성 폴더를 커밋하지 않는다.

## 과거 실패 재생 도구

`scripts/probe_markerless_loaded_grip.py`는 로컬 M4/solo-46 원본 로그에 의존하고, `scripts/probe_markerless_release_recovery.py`는 과거 실행 폴더를 입력으로 받는다(기본값 N3/solo-45). 원시 폴더는 Git에 포함되지 않아 새 clone만으로 이 진단을 실행할 수 없다. 진단 성공과 실제 모델 운반 성공은 구분한다. 최신 방출 재생기의 비매크로 전이 보완은 전체 재생으로 재검증하지 않았으므로 검증된 N7 실행 경로로 취급하지 않는다.
