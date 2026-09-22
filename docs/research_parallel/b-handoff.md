# B handoff · 공통 RGB 실행 포트

## 범위와 기준

- base main: `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`
- branch: `codex/r2-rgb-execution-port`
- 연구 TODO: R2 공통 실행기. 외부 모델·시뮬레이션·렌더링·학습·원격 제출·실물 구동 없음.
- 기존 dispatch/stage/CoELA/ACT 파일은 수정하지 않았다.

## 변경 파일

- `harness/rgb_execution_contract.py`: C용 Protocol, endpoint/frame/evaluator seam, strict static context,
  skill capability.
- `harness/rgb_execution_port.py`: actor-safe RGB 관측, 독립 task consent, robot별 command queue,
  공동 최소 barrier, 자원 lease, expiry/interrupt/close, 별도 evaluator snapshot.
- `tests/test_rgb_execution_port.py`: 14개 actor/evaluator/consent/격리/동시 호출/오류 계약 검사.
- `tests/fixtures/rgb_execution_port/basic.json`: C가 fake port/adapter에 재사용할 작은 joint fixture.
- `docs/research_parallel/b-rgb-execution-port.md`: exact payload, capability 표, 통합/후속 실행 경계.

## 검증

```text
tests/test_rgb_execution_port.py: 14 passed
관련 stage/multi-object/CoELA 회귀 포함: 86 passed, 3 subtests passed
compileall: passed
git diff --check: passed
```

검증 환경은 기존 Mac `.venv-sim-worker-mac` Python이며 MuJoCo나 외부 서비스는 실행하지 않았다.
actor request가 endpoint에 발행되는 계약만 검사했고 물리 성공은 미검증이다.

## 제약과 의존성

- C는 `RGBExecutionPortProtocol`과 fixture를 소비한다. task request/command/interrupt/release의
  exact schema와 하나의 tick clock을 유지해야 한다.
- D는 concrete port의 `evaluation_snapshot()`만 오프라인 소비한다. `external_evaluation`은
  actor 입력·단계 전환·완료 통보에 쓰지 않는다.
- A 감사 권고대로 peer role/request payload/미응답 peer/resource owner는 actor에게 노출하지 않는다.
- A 기준 감사는 PR #94, `5f4a7ef98423fc35696198a2d1157ff3b364a237`이다. A-01의
  shared-plan 우회를 쓰지 않고 자기 request matching만 사용하며, A-02/A-06에 대응해 exact nested
  allowlist와 evaluator-change 비간섭 검사를 추가했다. A-03 intent TTL은 C 소유이고, A-05의
  claim/evaluator verdict 분리는 C/D 통합 뒤 다시 감사해야 한다.
- existing `TaskStageExecution`, `BoundPairSkill`, `SoloBoxTransport`는 아직 연결되지 않았다.
  이번 변경을 실제 RGB 실행 완료로 표현하면 안 된다.
- 공용 README/current_status/research TODO/CI 목록은 편집하지 않았다. 병합 뒤 필요하면 코디네이터가
  통합 PR에서 갱신한다.

## 다음 명령과 실제 연결

현재 source에서 가장 작은 재검증 명령:

```sh
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python -m pytest -q \
  tests/test_rgb_execution_port.py
```

실제 연결은 C/D 계약 고정 뒤 별도 PR에서 진행한다. `b-rgb-execution-port.md`의 2회 deterministic
fixed replay(외부 모델 0, SIM 180초/회, wall 10분/회)를 먼저 구현·실행하고, 통과 뒤에만 6회
A/B/C 파일럿을 Colab CLI로 제안한다. 현재 exact 실행 script가 없으므로 새 adapter와 CLI를 함께
구현하기 전에는 무거운 실행을 시작하지 않는다.

## source SHA

- 구현 commit: `84aadca` (`Add common RGB execution port contract`)
- handoff/PR metadata commit: 이 문서의 후속 commit
- PR: 생성 뒤 설명과 task attachment에서 확인

main 병합은 사용자 명시 승인 전 금지한다.
