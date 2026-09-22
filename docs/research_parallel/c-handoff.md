# C · RGB 통신 런타임 handoff

기준 main: `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`

구현 고정 커밋: `97a86d3c60b6e1a55c01f96a14029cfbe75c7619`

상태: 오프라인 fixture/계약 검증. 실제 LLM·물리·렌더링·원격 실행은 하지 않았다.

## 구현 범위

- `harness/rgb_communication_runtime.py`
  - 서로 다른 세 planner 객체와 로봇별 observation/status/memory/inbox를 유지한다.
  - `none`/`structured`/`natural`은 같은 planner·port·행동·호출 상한을 사용하고
    명시적 peer message 형식만 바꾼다.
  - actor 입력은 B 포트의 exact RGB/static/own-lifecycle schema를 source별로 중첩
    검사한다. 알 수 없는 필드는 버리지 않고 실패 처리한다.
  - port `tick()` 반환의 전역 audit/완료 자료는 planner 입력·wake·조기 성공 종료에
    사용하지 않는다. evaluator 객체를 받는 인자 자체가 없다.
  - `task_request`/`command`/`interrupt`/`release`는 의미 필드를 보정하지 않고 B
    port에 제출한다. `none`도 두 actor가 같은 joint core를 독립 제출할 수 있다.
  - 수신자를 명시하고 다음 tick부터 전달한다. receiver clock으로 TTL을 적용하며,
    만료 내용은 current inbox에서 제거하고 content 없는 `EXPIRED_PEER_CLAIM`으로
    남긴다. 메시지 수·byte 상한과 침묵을 보존한다.
  - 요청 wall timeout, 전체 wall/tick/call/action 상한, 응답 중 자기 status 변경을
    검사한다. 늦거나 stale인 응답은 message/action을 내지 않는다.
  - actor의 `finish`는 평가로 확인하지 않은 로컬 주장으로만 기록한다. evaluator
    실패/성공으로 이를 고치거나 재실행하지 않는다.
- `tests/fixtures/rgb_communication/`
  - 동일 JPEG/static-context fixture, 원본 SHA-256, D용 6-event JSONL sample을 둔다.
  - trace sample은 `fixture` 증거이며 live model/physics 증거가 아니다.
- `tests/test_rgb_communication_runtime.py`
  - 세 조건 동일 예산, 독립 planner, directed recipient, queued/delivered TTL,
    late reply, own-state stale reply, supervisor 비간섭, unknown truth fail-closed,
    `none` joint 요청, unverified finish, raw 이미지/텍스트 hash를 검사한다.

## B/D 계약

- B 공용 타입은 `harness.rgb_execution_contract.RGBExecutionPortProtocol`이다.
  B 구현 커밋은 `84aadca0d39fd425ac0f97d3a8fb18b29b619ce7`, 최종 PR HEAD는
  `79f026261892dba84cbf81ae014b2212ec8927db`, PR은 #95다. C는 `TYPE_CHECKING`
  참조와 structural injection만 사용하므로 B 파일을 복제·수정하지 않았다.
- B의 미커밋 최종 worktree 구현을 읽기 전용으로 직접 주입한 1-tick `none` 검사에서
  r1/r2/r3가 각각 1 call·1 action, message 0을 기록했다. 실제 TaskStage/
  BoundPair/SoloBox adapter는 B에서도 미연결이다.
- trace schema는 `rgb-communication-event.v1`이다. 모든 행은 `event_id`, `run_id`,
  `condition`, nullable `robot_id`/`sim_time_s`, 비감소 `wall_time_s`, `event_type`,
  `related_ids`, `payload`를 가진다.
- D는 구현 커밋의 `trace.jsonl` SHA-256
  `1d91f754ba7be21339d70247530dd9f48f40eeff0c2a0c8e52579ac37d489927`를 직접 소비했다. 6 events,
  `planner_requested/model_calls=1`, `action_submitted/high_level_actions=1`, message 1,
  message bytes 189, token 미측정, terminal `aborted`로 일치했고 D 수정은 없었다.

## 검증

실행한 오프라인 검사:

```sh
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python -m pytest -q tests/test_rgb_communication_runtime.py
# 10 passed

/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python -m py_compile \
  harness/rgb_communication_runtime.py tests/test_rgb_communication_runtime.py
git diff --check
```

A의 경계 감사 PR #94(`5f4a7ef98423fc35696198a2d1157ff3b364a237`) 요구와 대조했다.
현 구현의 green은 C 담당자 fixture 검증이며 A의 독립 통합 승인이나 실제 RGB 성공을
뜻하지 않는다. 새 테스트는 공용 CI 목록 소유 범위가 아니므로 `scripts/run_ci_tests.py`
수정 요구를 코디네이터 handoff로 남긴다.

## 파일럿 전제와 유한 상한

A의 6회 초안과 맞출 C 설정 제안은 시나리오 2개 × 조건 3개, 로봇당 planner 호출
24회, high-level action 24회, message 24개·72,000 bytes, message TTL 12 clock초,
호출 wall 30초, 시행 wall 900초, SIM 180초다. `tick_period_s=1`이면 0초와 180초를
포함하도록 `max_ticks=181`을 명시한다. 6회 전체 planner 호출 최댓값은 432회다.

C runtime이 직접 강제하는 것은 tick/call/high-level action/message/byte/TTL/응답
wall/시행 wall 상한이다. 호출당 출력 768 token, 시행 입력 600,000 token, 저수준
command 6,000개와 전체 원격 job/회수 상한은 planner adapter·B·D manifest에서 별도
강제·검증해야 한다. 이 항목이 연결되지 않으면 6회 실행은 **NO-GO**다.

종료는 세 actor의 로컬 finish 주장, tick/call/wall 상한, 또는 runtime/port 오류다.
평가 성공 정답은 actor wake나 조기 성공 종료에 쓰지 않는다. 파일럿 전에 같은 frozen
model/prompt/RGB skill/map/calibration/camera/FOV/weld OFF와 D manifest를 고정하고,
B/C/D 통합 SHA에서 아래 검사를 다시 실행해야 한다.

```sh
python -m pytest -q \
  tests/test_rgb_execution_port.py \
  tests/test_rgb_communication_runtime.py \
  tests/test_rgb_communication_evaluation.py
```

## 남은 문제

1. B PR #95와 D 평가 PR이 main에 없으므로 현재 C PR 단독은 production 연결 경로가
   아니다. 각 PR 승인/병합 뒤 공용 타입 import와 trace consumer를 같은 SHA에서 다시
   검사해야 한다.
2. 실제 RGB skill adapter, live LLM planner, token hard cap, low-level command hard cap,
   2개 정상/복구 scenario와 actor-local visual completion adapter가 없다.
3. runtime planner 호출은 결정론적 round-robin이다. 모델 대기 중 SIM 정지 조건에는
   맞지만 비동기 물리 진행·실시간성 근거가 아니다.
4. committed JPEG는 최소 contract fixture일 뿐 실제 카메라/FOV 증거가 아니다.
5. 실제 결과가 없으므로 실험 manifest·영상·TensorBoard snapshot은 만들지 않았다.

외부 모델 호출·시뮬레이션·렌더링·원격 제출·실물 구동은 위 전제와 사용자 실행 승인이
생길 때까지 시작하지 않는다.
