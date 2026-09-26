# 자기 카메라 로봇별 실행기 API (패키지 F)

한국어 대화 연구의 LLM 층(또는 no-LLM 스크립트)이 로봇 한 대에 작업을 맡기는 다리다. 코드는 `harness/zone_own_executor.py`, 테스트는 `tests/test_zone_own_executor.py`, 첫 물리 스모크는 [`experiments/2026-09-26-zone-own-executor/`](../experiments/2026-09-26-zone-own-executor/README.md)에 있다.

**의존성(병합 전).** PR #201(`claude/zone-m1-owncam`, `cceb7ee`까지 병합: M1 제어기·위치 추정·계약, 스킬 v9 `04a5e3c` 포함)과 PR #193(`kiro/zone-own-perception`, `fb44c2c`: 자기 RGB 판단)을 이 브랜치에 병합해 쓴다. 두 PR의 파일은 읽기 전용으로 재사용하고 수정하지 않았다. PR #194(패키지 A/D)는 병합하지 않았으며, 필요한 schema 값을 고정 복사해 어댑터를 만들었다(아래).

## 입력 경계

실행기(`ZoneOwnExecutor`)가 받는 것은 다음뿐이다.
- 자기 `robot_cam` 관측(JPEG, 자기 발행 PWM)
- 자기 포트에서 되돌려 받은 자기 발행 명령 이력
- 정적 태그 지도(`zone_wide_door_tags_v2` 등)와 고정 교정값
- 시나리오 설정에서 만든 주문서: 종류·수량·목적 구역·개략 pickup 슬롯 `P{열}-{행}`. 좌표는 거부한다.

실행기는 시뮬레이터를 import하지 않는다. world·포트·다른 로봇·호스트 참조도 갖지 않는다. 물리 소유자가 자기 관측과 자기 명령만 넣어 준다. 다른 로봇의 관측, `nav_cam`, 오래된 프레임, 해시가 맞지 않는 프레임은 거부한다.

## 작업 API

| 호출 | 동작 | 끝 사건 |
|---|---|---|
| `deliver(item_ref, zone_slot)` | 주문 줄 하나를 구역 슬롯(`A2`)이나 구역(`A` → 자기 기록상 다음 슬롯)으로 옮긴다. 먼저 넓게 둘러보고, M1 사슬(자기 RGB 탐색 → 파지 → 문 통과 운반 → 배치 → 다시 보기)을 따른다. 탐색은 주문서의 pickup 슬롯(+0.15 m) 안의 청록 검출만 쓴다. 서쪽 관측점에서 1 m 넘게 떨어진 bay(P2)는 bay 서쪽 가장자리의 행 사이 통로에서도 본다(`lane_viewpoints`) | 다시 보기 IN_SLOT → `job_done(own_camera_confirmed)`; 놓았지만 확인 불가 → `job_done(unconfirmed)`; OUTSIDE_SLOT·그 밖 → `job_failed(reason)` |
| `goto(target)` | `[x, y]`, 구역 `A`(칠 서쪽 0.25 m), 구역 슬롯 `A2`(서쪽 0.40 m), pickup 슬롯 `P1-2`(서쪽 관측점 x=−0.47), 문 `door_1`(자기 추정상 반대편 0.45 m). loop driver v2(자기 추정 + 지도 A*, 멈춰서 둘러보기) | 도착 전 마지막 둘러보기 뒤 `ARRIVED` → confirmed; 경로 없음·위치 상실 → failed |
| `look_around()` | LOOK_P20 자세로 ±48° 둘러보고 원래 자세로 돌아온다 | 불확실도 low/medium → confirmed, 아니면 unconfirmed |
| `hold(sim_s)` / `wait(sim_s)` | 멈추고 마지막 안전 명령을 유지한다 | `job_done(unconfirmed, HOLD_ELAPSED)` |
| `abort(reason_code)` | 현재 작업을 취소하고 멈춘다(들고 있으면 계속 든 채) | `job_failed('ABORTED:<reason>')` |
| `status()` | 자기 상태와 자기 카메라 판단만 반환 | — |

작업은 한 번에 하나다. 바쁜 중의 호출, 모르는 주문, 목적지와 다른 슬롯, M1 스킬이 지원하지 않는 종류(청록 외)는 `command_rejected`로 거부한다. 각 작업의 SIM 한도는 기본 720 s(M1과 같음)이며, 넘으면 `job_failed('LOCAL_TIMEOUT')`이다.

`status()`에 들어가는 항목:
- `holding`(yes/no/unknown): 출처를 함께 적는다. 스킬의 자기 RGB 부착 확인, 자기 RGB 방출 확인, "마지막 방출 뒤 집게를 닫은 적 없음" 중 하나다. CARRY 자세 프레임에서 `judge_holding_item`이 확신(≥0.65)으로 반대 답을 내면 `unknown`으로 둔다.
- `blocked_ahead`(yes/no/unknown): LOOK_P20·CARRY 자세, pan 1500, 정지 상태의 프레임에서 1 SIM s마다 `judge_route_blockage`를 돌린 결과다. 5 s가 지나면 `unknown`이 된다.
- `localization.level`: 자기 PoseReport의 σ로 정한다. low는 σ ≤ 3.5 cm·0.035 rad(M1 look_back 한도), medium은 ≤ 8 cm·0.10 rad(M1 nav_unloaded), 그 밖은 high, 초기화 전은 unknown이다.
- `region`: 자기 추정으로 본 개략 영역이다.

`belief_projection()`은 같은 내용을 패키지 A의 `BELIEF_KEYS`로 투영한다. 두 출력 모두 A의 금지 키·부분 문자열에 걸리지 않음을 테스트로 확인한다.

## 사건과 패키지 A/D 연결

`drain_events()`는 `ugrp.zone_own_executor_event.v1` 행을 돌려준다. 행마다 `scheduler_trigger`가 붙는다(패키지 D `TRIGGERS`).

| 사건 | D trigger | 비고 |
|---|---|---|
| `job_started` | 없음(기록만) | |
| `job_done` | `idle` | `confirmation` = `own_camera_confirmed` / `unconfirmed` |
| `job_failed` | `failure`, 단 `LOCAL_TIMEOUT`은 `timeout` | `reason` |
| `blockage_seen` | `blockage` | 2회 연속 yes·신뢰도 ≥ 0.65. `no`가 나오면 다시 무장 |
| `pose_uncertain` | 없음 | low/medium → high/unknown 전이(가장자리 검출). 작업을 끝내는 경우에는 `job_failed`가 따로 깨운다 |

API 응답(ack)은 `action_record(...)`로 A의 `ugrp.zone_study_action.v1` 행이 된다. 대응은 deliver→`claim_order`, goto→`goto`, look_around→`observe`, hold→`wait`, abort→`abort_job`이다. `local_state`는 A의 `LOCAL_STATES`를 쓴다. 인자 키는 A의 `COMMAND_ARGUMENT_KEYS` 안에 있다. #194가 병합되면 테스트가 A 모듈의 실제 값과 `action_record_violations`로 다시 비교한다.

## M1 계약

- `mode='m1'`(기본)에서는 모든 PoseReport·스킬 추정의 출처가 `owncam_pf_v2:<교정 sha8>`여야 한다. `harness.m1_owncam_contract`와 `harness.m1_contract`가 모두 검사한다.
- 주입된 자세 출처는 `OwnCamPoseSource`이면서 자기 카메라 라벨일 때만 받는다. `gt_stub_eval_only`는 거부한다.
- `mode='diagnostic'`은 테스트용 주입을 받지만 `counts_as_m1_inputs=False`다.
- 평가 전용 기록(정답 자세·상자 위치·접촉·weld)은 호스트의 `eval_only`에만 쌓이고 실행기로 흐르지 않는다.

## 여러 로봇 장면: `OwnCamTeamHost`

MuJoCo world 하나에 로봇 3대를 두고, `CameraRobotPort`와 실행기를 로봇마다 하나씩 붙인다.
- 실행 조건: 동기 SIM, weld OFF, 접촉 프로필 선택(`cargo_noslip_v1`).
- 관측: 로봇마다 5 Hz로 자기 카메라만 렌더링한다.
- 결정: 0.1 s 간격이다. 스킬 macro(`drive`/`mecanum`/`pose`/`wait`)는 `scripts/run_m1_owncam.py`와 같은 시간 규칙으로 로봇별 일정표에 펼쳐져 다른 로봇과 병렬로 진행한다.
- 예외: 한 로봇의 제어기에서 예외가 나면 그 로봇의 작업만 `job_failed('EXCEPTION:…')`로 끝나고, 그 로봇은 멈춘다.
- 연구 층: `host.call(rid, api, ...)`로만 그 로봇의 API를 부른다. 사건만 받는다.
- 격리: 실행기에서 도달 가능한 객체 그래프에 호스트·world·다른 포트·다른 실행기가 없음을 테스트로 확인한다. 세 실행기 사이에 공유된 변경 가능 객체가 없다는 것도 확인한다.

## 한계

- deliver는 M1 사슬을 그대로 쓰므로 청록 상자·문 하나(`door_1`)·정적 spawn keep-out 가정을 물려받는다.
- 로봇끼리 조정하지 않는다. 문 대치나 충돌 회피는 연구 층(대화)의 몫이다. 실행기는 막힘을 보고만 한다.
- `pose_uncertain` 가장자리와 막힘 판단은 PR #193의 오프라인 게이트 범위에서만 검증됐다. 실행 중 막힘 판단의 정확도는 이 패키지에서 채점하지 않았다.
- `goto`의 목표점 규칙(서쪽 접근)은 이 문서의 고정값이며, 교사 경로로 검증한 것이 아니다.
