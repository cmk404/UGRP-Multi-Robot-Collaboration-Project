# 공동 작업 단계 동기화 계약 v1

2026-09-16. [작업 목록 #39](https://github.com/kcm0127-dotcom/ugrp/issues/39)의
T1과 T2 중 **팀원 연결용 입출력과 공통 단계 전환**을 구현한다.
`harness/task_stage_sync.py`의 `TaskStageSync`는 기존 `PairCarrySync`를 재사용한다.
현재 하나의 프로세스 안에서 직렬로 호출하는 모듈이며, 팀 전체의 공식 프로토콜 합의나
실제 로봇 연결 완료를 의미하지 않는다. 기존 운반 정책의 실행 경로는 유지한다.

## 담당 경계

| 제공자 | 입력 또는 산출물 |
| --- | --- |
| 상위 계획 담당 | 작업·물체·참여자·역할·목표 구역·정적 지도 버전/해시·계획 버전 |
| 창민의 동기화 계층 | 같은 작업/단계/계획에 대한 준비 확인, GO/HOLD, 실행 허가, 다음 단계 요청, 사건 기록 |
| 로봇별 실행 담당 | 자기 RGB·공용 top RGB 기반 준비/불확실성/진행/완료 추정, 짧은 명령 실행과 현지 정지·하중 유지 |
| 평가 | 실제 파지·들림·접촉·충돌·낙하·도착 판정. 별도 출력에만 기록 |

경로 계산, 파지 자세 계산, 영상상 지지 판별, 로봇별 복구 동작은 이 모듈에서 만들지 않는다.
이 계약의 `checks`는 로봇 실행기가 허용 영상에서 얻은 **보고**다. 문자열이 있다고
영상 근거가 입증되는 것은 아니다. 실제 연결 시 원본 두 영상과 생산자의 전체 입력을 보존·감사해야 한다.
기존 카메라/FOV·외관·weld OFF 조건을 바꾸지 않는다.

## 단계와 합의 조건

독립 접근·관찰은 앞단에서 수행한다. 공동 동작은 아래 고정 순서이며 단계 건너뛰기는 없다.

| 단계 | 각 로봇의 READY에 필요한 영상 확인 | 각 로봇의 DONE에 필요한 영상 확인 |
| --- | --- | --- |
| GRASP: 함께 잡기 | `at_grasp_pose`, `stopped` | `grasp_observed`, `stopped` |
| LIFT: 함께 들기 | `grasp_observed`, `stopped` | `load_lift_observed`, `stopped` |
| CARRY: 함께 운반 | `load_lift_observed` | `arrived_observed`, `stopped` |
| LOWER: 함께 내리기 | `arrived_observed`, `stopped` | `support_observed`, `stopped` |
| RELEASE: 함께 놓기 | `support_observed`, `stopped` | `released_observed`, `stopped` |

- 매 단계는 양쪽의 **새 준비 보고**로 시작한다. 앞 단계의 DONE이 다음 단계 READY를 대신하지 않는다.
- DONE은 해당 로봇의 현재 계획/단계에서 마지막으로 발행한 명령 ID와 그 명령의 제한 시간 이후 영상을 참조한다.
  명령 접수·발행·시간 경과만으로 완료 처리하지 않는다. `OBSERVED_MOTION`도 DONE과 다르다.
- 한쪽만 DONE이면 그 로봇의 추가 명령을 차단하고 상대의 새 DONE을 기다린다.
  GRASP에서만 `command_participants()`가 아직 완료하지 않은 참여자를 반환하여,
  최신 DONE인 로봇의 명령 설정을 유지한 채 READY인 상대의 파지를 이어갈 수 있다.
  양쪽 보고의 신선도·허가·epoch 검사는 그대로 적용된다. LIFT/CARRY/LOWER/RELEASE의
  한쪽만 움직이는 동작은 이 경로에서 허용하지 않는다.
  둘 다 최신 DONE일 때만 `advance()`가 다음 단계로 이동한다.
- 마지막 FINISH는 **전체 단계가 완료됐다는 양쪽 보고가 모임**을 뜻한다. 물리 성공 판정이 아니다.
- 실제 도착·파지·지지 상태를 확신하지 못하면 `UNCERTAIN`으로 보고한다.
  특히 지지 관측 없이 타이머만으로 RELEASE를 준비 처리하지 않는다.

## JSON 계약

실행 가능한 입력 예시는 [plan.json](../examples/task_stage_sync/plan.json)이다.
`TaskPlan.from_dict()`와 `StageReport.from_dict()`는 누락·추가 필드를 거부한다.
계획의 `static_map.sha256`은 배포할 정적 지도 파일 바이트의 SHA256이다.
동기화 코어는 지도 파일을 읽거나 경로를 계산하지 않는다. 연결자가 실제 파일의 해시를 확인한다.

### 상위 계획 → 동기화

`schema=ugrp.task_plan.v1`, `task_id`, `plan_version`(양의 정수), `object_id`,
`participants=[{robot_id, role}, …]`, `goal_region`, `static_map={id, version, sha256}`.
최소 두 참여자가 필요하며 로봇 ID는 유일하다. 현재 계획은 다섯 단계를 고정한다.
`map.version`은 문자열이다. 계획의 버전은 증가하는 정수다.

### 로봇 실행 → 동기화

`schema=ugrp.stage_report.v1`, `evidence_source=own_rgb+top_rgb`와 아래 모든 필드가 필수다.

| 필드 | 의미 |
| --- | --- |
| `task_id`, `run_id`, `plan_version`, `stage`, `epoch` | 현재 실행 문맥. 동기화 응답에서 읽으며 로봇이 임의로 올리지 않는다 |
| `robot_id`, `sequence` | 발신 로봇과 실행 전체에서 증가하는 보고 번호 |
| `status` | READY / NOT_READY / UNCERTAIN / FAILED / OBSERVED_MOTION / DONE |
| `observed_at_s`, `decided_at_s`, `sent_at_s` | 영상 관측, 판단, 전송 시점. 공통 시계로 환산한 값 |
| `observation_id`, `own_rgb_ref`, `top_rgb_ref` | 새 관측과 두 원본 RGB의 참조. 평가·관절·정답 위치를 넣지 않는다 |
| `confidence`, `checks`, `reason` | 영상 판단의 확신도 0~1, 위 표의 확인 항목, 보고 사유 |
| `command_id` | DONE/OBSERVED_MOTION은 자기 마지막 발행 명령 ID. READY 등은 null 가능 |

이전 실행/단계/계획/epoch, 중복·역순 sequence, 재사용 observation ID 또는 RGB 참조 쌍,
미래·유효기간 초과·현재 epoch 시작 전 영상은 거부한다. 전송 지연으로 시각을 새로 덮어쓰지 않는다.
호출자의 `receive(robot_id, payload, now_s=…)`에서 `robot_id`는 연결자가 확인한 발신자여야 한다.
여기에는 인증 기능이 없고, 실제 네트워크에서 문자열만으로 발신자를 믿으면 안 된다.

### 동기화 → 로봇 실행

`authorize(now_s=…)`는 phase(WAIT/HOLD/GO/ABORT/FINISH), 사유, 현재 문맥과 permission을 반환한다.
permission은 `run_id/task_id/plan_version/stage/epoch/token/issued_at_s/expires_at_s`를 포함한다.
GO라도 남은 유효 시간이 없으면 permission은 null이다. `dispatch()`에서 **발행 직전** 다시 확인한다.

```python
from harness.task_stage_sync import TaskPlan, TaskStageSync

sync = TaskStageSync(TaskPlan.from_dict(planner_payload), now_s=coordinator_now())
# 각 로봇의 새 두 카메라 관측에서 실행 담당 코드가 만든 보고를 전달한다.
sync.receive(authenticated_robot_id, report_payload, now_s=coordinator_now())
decision = sync.authorize(now_s=coordinator_now())
accepted = sync.dispatch(
    robot_id, decision["permission"], unique_command_id,
    now_s=coordinator_now(), duration_s=0.1,
    submit=lambda: local_port.submit_bounded(command, duration_s=0.1),
)
# accepted는 명령 발행 여부다. 별도 새 영상 보고를 받아야 advance()할 수 있다.
```

로봇별 명령 시간 중첩, 같은 명령 ID 재발행, 완료 보고 뒤 재발행, 다른 실행에서 발행한 허가,
허가 내용 변경, 유효 시간보다 긴 명령은 거부한다. submit 예외는 공동 HOLD를 만들고 그대로 전파한다.
COMMAND_ISSUED는 제출 시도 기록이며 예외가 나면 COMMAND_ERROR가 뒤따른다. 성공 여부를 뜻하지 않는다.

## 정지·재개와 시계

- 기본 보고 유효 시간 0.6초, 명령 상한 0.25초, confidence 하한 0.8은 **계약 시험 설정**이다.
  실물의 최적값이나 검증된 안전 정지 시간으로 보지 않는다. 실제 실행 주기를 측정해 사전 고정해야 한다.
- 모든 코어 입력 시각은 하나의 비감소 시계다. `clock_domain=monotonic` 또는 `sim`을 실행 전체에서 고정한다.
  로봇별 다른 시계나 SIM/실제 시간 혼용은 허용하지 않는다. 네트워크 시계 환산·오차 처리는 다음 연결 작업이다.
- READY 철회, 낮은 확신도, 필수 확인 항목 누락, 유효기간 초과, 명시적 HOLD는 허가를 회수한다.
  새로운 부정 보고는 이미 HOLD 중이어도 epoch를 올려 부분적으로 모인 READY를 지운다.
  단순 반복 HOLD 호출은 기본적으로 멱등이다. 재개에는 새 epoch 이후 관측한 양쪽 보고가 필요하다.
- HOLD 중 실제 행동이 끝난 것으로 재관측되면 같은 단계/계획의 마지막 명령을 참조한 새 DONE을
  받을 수 있다. 해제처럼 되풀이하면 안 되는 명령을 동기화 때문에 자동 재실행하지 않는다.
- 계획 교체는 같은 작업·물체·참여 역할에서 더 높은 버전만 받는다. 현재 단계를 유지하고 양쪽을 재확인한다.
  기존 계획의 명령은 새 계획의 DONE 근거로 재사용할 수 없다. 물체/역할 변경은 별도 작업·재계획 절차다.
- STOP/HOLD는 로컬 실행기의 하중 유지·정지 동작과 연결해야 한다. 이 모듈은 이미 제출한 명령을 취소하지 못한다.
  submit은 비차단·시간 제한이어야 하고, 로컬 watchdog이 허가 만료와 중단을 집행해야 한다.
  두 제출 사이 고장이나 실제 구동 시작 차이는 출발 합의만으로 사라지지 않는다.
- 호출은 직렬이어야 하고 submit에서 동기화 코어로 재진입하지 않는다. 현재는 스레드 안전성·분산 합의·재시작 복구가 없다.
  새로운 객체의 run_id는 달라 이전 허가를 거부하지만, 로봇의 물리 상태·점유 상태 복구까지 증명하지 않는다.

## 기록과 팀원 재현

각 사건에 작업/실행/계획/단계/epoch, 공통 시각과 실제 경과 시간을 남긴다.
REPORT에는 원본 보고와 관측→판단·송신 대기·전달 지연을 각각 기록한다.
REQUEST/STAGE_REQUEST부터 GO까지가 합의 대기, COMMAND_ISSUED와 OBSERVED_MOTION/DONE 보고가
명령·관측을 연결한다. 관측 시점으로 실제 동작 시작 시점을 단정하지 않는다.
후속 비교에서 메시지·명령 수, 대기·복구 시간과 물리 평가를 나눠 집계한다.

계약 예제는 Python 표준 라이브러리만 사용하며 저장소 루트에서 실행한다.

```sh
python scripts/demo_task_stage_sync.py --output outputs/task-stage-sync-example
```

실행 코드는 먼저 커밋해야 한다. 출력 경로가 이미 있으면 덮어쓰지 않는다.
9개 조건(정상, 준비 지연, 완료 지연, 정지 후 재관측, 보고 단절, 지지 불확실,
명령 발행만 있음, 이전 단계 허가, 계획 변경)을 두 가짜 실행 포트와 JSON으로 연결한다.
`fixture://` 참조는 실제 영상이 아니며 **프로토콜 검증만** 한다. 결과와 사건 로그, 코드 SHA,
환경, 설정, 파일 해시를 보존한다. GPU·MuJoCo·모델 호출·하드웨어·네트워크는 사용하지 않는다.

다음 통합 작업은 실제 메시지 전달 경로와 별도 로봇 실행기 연결이다. 영상 판단 생산자와
정지 watchdog을 연결한 뒤, 통신 장애·프로세스 재시작과 실제 파지·하중·내려놓기를 별도 검증한다.
이 문서와 예제만으로 T1/T2 전체나 물리적 전 과정 완료를 선언하지 않는다.

후속 [실행 포트 어댑터](task_stage_execution.md)는 로컬 SIM에서 이 계약을 실제 RGB 캡처와
팔/바퀴 정지·재개에 연결한다. 단계 영상 판별기와 별도 프로세스/통신 연결은 계속 구분한다.
