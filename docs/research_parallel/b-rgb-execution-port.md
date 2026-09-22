# B · 공통 RGB 실행 포트 계약

기준 main: `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`.
이 문서는 통신 없음/정형/자연어 조건이 같은 운동 경계를 쓰기 위한 **최소 포트 계약**이다.
실제 RGB 운반 성공이나 기존 실행기의 연결 완료 보고가 아니다.

## 연구 경계

`harness/rgb_execution_port.py`는 세 조건에서 동일한 아래 메서드를 제공한다.

```python
observe(robot_id)
local_status(robot_id)
submit(robot_id, payload)
tick(now_s)
close(now_s)
```

통신 runtime이 import할 공용 타입은
`harness.rgb_execution_contract.RGBExecutionPortProtocol`이다. 이 Protocol에는
`evaluation_snapshot()`을 넣지 않았다. 평가 담당은 concrete port의 별도 참조로만 이 메서드를
호출하고 반환값을 actor runtime에 되돌리지 않는다.

시계는 실행 생성 시 `sim` 또는 `monotonic` 하나를 고정한다. 오직 물리 소유자의 `tick(now_s)`가
시간을 전진한다. actor payload는 `requested_at_s`를 포함하지만 시계를 바꾸지 못한다. 미래 요청,
5초보다 오래된 요청, 이미 만료된 요청, 감소한 tick은 거부한다. SIM에서는 매 물리 step 전에,
실물에서는 endpoint watchdog과 같은 monotonic clock으로 tick해야 한다.
세 actor thread의 public 호출은 port 내부 reentrant lock으로 직렬화한다. 이는 공유 clock/lease의
원자성 경계이지 전원 진행 barrier가 아니다. endpoint callback은 짧고 bounded/nonblocking이어야 한다.

## actor가 받는 정보

`observe()`의 최상위 필드는 아래로 고정한다.

| 필드 | 내용 |
| --- | --- |
| `robot_id`, `observation_id`, `observed_at_s`, `clock_domain` | 자기 관측 식별값과 공통 시계 |
| `images.own_rgb`, `images.top_rgb` | 원본 JPEG의 base64, SHA256, 이 관측 내 ref |
| `static_context` | 아래 exact schema로 생성 시 고정한 작업·지도·카메라 설명 |
| `static_context_sha256` | 고정 문맥 JSON의 SHA256 |
| `own_issued_commands` | 실제 endpoint에 발행된 자기 명령만. 측정 상태나 성공이 아님 |

`static_context`는 `schema`, `task`, `static_map`, `camera_calibration`만 받는다.
task는 고정 description/object IDs/allowed skills, 지도와 보정은 id/version/hash/description이다.
실시간 좌표·접촉·관절·성공·평가 상태를 생성 뒤 추가할 수 없다. 호출자가 반환 사본을 바꿔도
포트 내부 문맥은 바뀌지 않는다.

`local_status()`는 자기 active/pending 요청과 `own_submission_history`만 반환한다. 공동 작업에서
자기 제안의 participants와 자기 role은 보이지만 상대 role, 상대 request/command payload,
누가 아직 미응답인지, 다른 작업의 자원 owner는 보이지 않는다. generic
`awaiting_independent_consent`/`resource conflict`만 반환한다. 평가 정답 변화가 actor status나
관측을 바꾸지 않는 paired test를 포함한다.

lease 종료는 모든 참여자의 own history에 `TERMINATED`와 generic
`task_interrupted`/`lease_expired`/`execution_unavailable`/`task_released_unverified`만 남긴다.
누가 중단했는지나 backend 예외 문자열은 actor 결과에 넣지 않는다. 이는 최소 공동 HOLD 통보이며
물리 성공/실패 판정이 아니다.

## 독립 요청과 최소 동기화

task request의 exact 필드는 다음과 같다.

```json
{
  "kind": "task_request",
  "request_id": "r1-local-request",
  "task_id": "static-shared-task-id",
  "object_id": "beam_01",
  "skill": "pair_transport",
  "participants": ["r1", "r2"],
  "resources": ["beam_01", "north_lane"],
  "stage": "CARRY",
  "own_role": "end_a",
  "observation_id": "r1-000001",
  "decision_id": "r1-decision-1",
  "requested_at_s": 0.0,
  "expires_at_s": 3.0
}
```

- solo 요청은 허용 capability와 자원 충돌을 통과하면 즉시 lease를 받는다.
- 공동 요청은 모든 명시 participant가 같은 task/object/skill/participants/resources/stage를
  독립적으로 제출해야 lease를 받는다. 각자의 request ID, role, 관측/결정 ID, 만료 시각은 공유할
  필요가 없다. role의 유일성이 필요한 skill은 중복 선택을 거부하며 포트가 고쳐 주지 않는다.
- `none` 조건도 같은 API로 합법적 공동 요청을 낼 수 있다. peer 메시지·plan hash·중앙 역할표는
  consent의 입력이 아니다.
- 포트는 cargo, partner, role, 순서, recovery를 선택하지 않는다. 불일치·미지원·충돌은 거절한다.

lease를 받은 뒤 각 로봇은 자기 `command`를 낸다. command는 task/lease/command/stage ID,
endpoint가 검증할 action, 제한 시간과 관측/결정/요청 시각을 포함한다. solo queue는 다음 tick에
독립 실행한다. 공동 queue는 참여자의 명령이 모두 모인 뒤 같은 tick에서 순서대로 endpoint에
제출한다. 하나가 실패하면 해당 task 참여자 모두를 hold하지만, 무관 robot/task queue는 계속
처리한다. 부분 제출 차이는 실제 분산 동시 구동을 보장하지 않는다.

`interrupt`와 `release`는 해당 lease 참여자만 hold하고 자원을 해제한다. release는
`actor_released_without_measured_success`로 기록하며 완료 판정이 아니다. 실제 완료는 별도
평가 snapshot에만 기록한다. task/stage를 자동 전진시키는 기능은 없다.

## capability와 현재 지원 범위

| 항목 | 상태 | 근거와 한계 |
| --- | --- | --- |
| 독립 task consent, robot별 queue, 공동 command barrier, expiry/interrupt/resource lease | 로컬 계약 지원 | fake endpoint/clock 단위 테스트. 분산 합의·재시작 복구 아님 |
| `CameraRobotPort`형 endpoint | 인터페이스 호환 | `robot_id`, `validate_bounded`, `apply_bounded`, `hold`, `tick` seam. 이번 배치에서 MuJoCo에 연결하지 않음 |
| own RGB + common top RGB + 고정 문맥 | 계약 지원 | frame source를 주입. 실제 카메라 저장/전송/지연은 미검증 |
| evaluator 분리 | 계약 지원 | source를 actor 경로에서 호출하지 않음. evaluator의 정확성은 D 범위 |
| `TaskStageExecution`의 GRASP→LIFT→CARRY→LOWER→RELEASE | **미연결** | 현재 fixed participant plan과 단계 READY/DONE 계약을 사용함. 새 포트에는 stage 자동 전환/영상 DONE 생산자가 없음 |
| `BoundPairSkill` | **미연결** | scene facade가 pair binding과 물리 step을 직접 소유하여 새 endpoint seam과 다름 |
| `SoloBoxTransport` + `VisualMacroExecutor` | **미연결** | 기존 scene/runtime의 phase와 command lease 연결 adapter가 필요함 |
| 실제 MasterPi, 네트워크 장애, 물리 동시성, 하중 유지 | **미검증** | 외부/실물 실행을 이번 배치에서 금지함 |

따라서 현재 포트의 ACCEPTED/DISPATCHED는 계약상 접수/endpoint 발행일 뿐이다. 접근·파지·운반·놓기,
RGB 완료 판정, 실제 정지·하중 유지의 성공으로 보고하면 안 된다.

## C 통합 예제

fixture는 `tests/fixtures/rgb_execution_port/basic.json`이다. C runtime은 세 조건 모두
`RGBExecutionPortProtocol` 하나를 주입받고 다음 순서를 사용한다.

1. `observe(rid)`와 `local_status(rid)`를 독립 actor context로 저장한다.
2. LLM 선택을 exact task request로 좁혀 `submit(rid, request)`한다. PENDING을 peer 승인이나
   거절로 해석하지 않는다.
3. ACCEPTED일 때 받은 자기 `lease_id`로 자기 command만 제출한다.
4. 물리 owner가 `tick(now_s)`을 호출한다. tick 결과는 run-level terminal/trace에만 쓰고
   peer private state를 actor prompt나 깨우기에 추가하지 않는다.
5. 종료 시 `close(now_s)`한다. D의 evaluator snapshot은 별도 consumer가 저장한다.

C는 action/decision/observation ID를 trace correlation에 그대로 남길 수 있다. B 포트가 만드는
`LOCAL_COMMAND` 감사 row에는 실제 endpoint에 발행한 unique command ID가 있다. 이것과 C의
high-level action 제출 수를 구분한다.

## 검증과 다음 유한 실행

이번 배치의 오프라인 검사는 외부 모델·MuJoCo·렌더링·학습 없이 다음 명령으로 끝난다.

```sh
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python -m pytest -q \
  tests/test_rgb_execution_port.py
```

실제 연결은 C runtime/trace producer와 D evaluator consumer의 PR이 고정되고 같은 base에 합쳐진 뒤
별도 후속 PR에서 한다. 첫 실행은 아래 선행 조건과 상한을 만족해야 한다.

- `BoundPairSkill`/`SoloBoxTransport`를 직접 호출하지 않고 각각 명시적인 endpoint adapter와
  capability를 만든다. `TaskStageExecution`의 RGB 보고/정지 규칙을 보존한다.
- 새 실행 SHA를 먼저 커밋한다. fixed replay 1개에서 solo 1 task와 pair 1 task를 병행하고
  weld OFF, 기존 camera/FOV, 외부 모델 0회, SIM 180초·wall 10분 상한으로 종료한다.
- 모든 로봇 command lease는 capability에 선언한 최대 0.25초 이하로 하고 매 physics step 전에
  tick한다. task expiry/interrupt 한 조건을 추가하되 총 2회만 실행한다.
- actor request/관측 원본, port audit, evaluator truth를 서로 다른 artifact로 저장한다. 실제 RGB
  READY/DONE과 별도 evaluator가 일치하는지 확인하며 불일치는 실패로 남긴다.
- 위 deterministic 연결이 끝난 뒤에만 정상 1조건+복구 1조건×none/structured/natural의 6회
  외부 모델 파일럿을 Colab CLI에서 별도 승인·예산으로 제출한다.

현재 repository에는 위 연결을 실행하는 command/script가 없으므로, 존재하지 않는 명령을 재현
명령으로 제시하지 않는다. 다음 구현은 새 script와 exact CLI, output manifest, 종료 조건을 함께
추가해야 한다.
