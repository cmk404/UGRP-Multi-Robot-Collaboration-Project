> 2026-09-09 실행 기준: Mac 로컬 시뮬레이션. 이 문서의 이전 운영 구성은 역사적 기록이며, 최신 설치·실행은 [CONTRIBUTING.md](../CONTRIBUTING.md), 클라우드 퇴역은 [기록](cloud_simulation.md)을 따른다.

> 현재 아레나 상태(2026-09-06): 구역 한 변0.82m. 실제 자연어3시드 전체완료0/3; 집기 접근 및 경로 실패 미해결. 아래 과거 성공 결과와 구분하며, 최신 근거는 `outputs/warehouse_research/coela-compact-video-01/report-ko.md` 참조.

# UGRP 현재 아키텍처 + TODO

> 기준일: 2026-09-04  
> 목적: 다음 세션에서 긴 과거 로그를 전부 읽지 않고도 현재 구조와 남은 핵심 작업을 바로 이어가기 위한 인계 문서.

## 2026-09-06 — 최종 코드 18회 다중 시드 동작 확인

`outputs/warehouse_research/coela-final-multiseed-09/report.md`: 최종 코드 고정 후
시드 12·23·37 × 정상/장애물 × 무통신/정형/자연어 각 1회, 120초 제한 총 18회 완료.
정상 각 3/3, 장애물 무통신 0/3·정형 3/3·자연어 3/3. 통신의 사건별 복구 6/6.
공동·단독 작업 겹침과 세 로봇 동시 이동은 무통신 1/6, 통신 12/12에서 확인됐다.
18회 접촉·순간 위치 이동·마감 후 실행·퇴역 참가자 신규 승인 0건.
최대 연속 비회전 후진 약 2.1cm. 반복 대기 445/719 판단, 일반 상한 도달 20/54로
효율 개선은 남는다. 새 시드·반복 실행·최적성 검증은 아니며 기존 실패를 대체하지 않는다.

## 2026-09-06 — CoELA 결함 수정·최종 검증 완료

동시 접근 교착, 공동 회피 간격, legacy beam 누락, 목표 정렬 정체와 상자 밀림,
늦은 응답·마감·동의 수명, 메시지에 의한 합의 폐기, 관측 시각·평가 원인 보존을 수정했다.
최종 전체 테스트 1,052개 통과. seed23 비공개 자연어 실제 LLM은 기존 120초 한도에서
79.45초에 3화물·복구 성공. 동일 고정 동작 물리 재현도 87.152 SIM초 전체 성공.
실제 TEAM v9도 재검증 성공(각18회 판단, 메시지16개, 오류0, queue0/inflight없음).
최종 한국어 실제 대화 영상은 `outputs/warehouse_research/coela-final-video-08/23-private_obstacle-0-natural/dialogue-1x.mp4`.

최종 근거와 제한은 `outputs/warehouse_research/coela-fixes-final-report.md`를 따른다.
fixed-05의 18회 비교는 마지막 정렬·상자 경로 보정 전 진단이다. 최종 코드의 새18회
성공률로 간주하지 않는다. 후속 본실험은 소스 고정 후 새 A/B/C 반복·held-out seeds를
사용하며, 기존 실패를 성공 재현으로 대체하지 않는다. 모델의 최적 분담·시간내 항상성공은
보장하지 않는다. 현재 저수준 제어는 공통 ideal SIM geometry, 사건은 controlled injection이다.

## 2026-09-05 — 다중 시드 진단에서 확인된 결함

seed12/23/37 × 정상/비공개 장애물 × A/B/C, 총18회 완료. 정상 각3/3,
비공개 장애물 전체 완료 A0/3·B1/3·C2/3. 통신 우월성 결론으로 단정하지 않는다.
seed37의 두 로봇 동시 접근은 약76초 지속 접촉으로 교착했고, 센서 준비를 포함한
고정 동작 재현에서도 실패했다. 같은 로봇의 단독 실행은 각각 성공했다.
초기 화물 선택 쏠림(16/18), 반복 대기 판단(491/866), 예산 소진 후 남는 공동 제안,
늦은 응답의 stale 재제출과 실패 원인 요약 문제도 확인됐다. 이 항목은 수정 전 진단 기록이다. 후속 수정 상태는 위 항목을 따른다. 최종 근거·정정·수정 순서는
`outputs/warehouse_research/coela-multiseed-01/report.md`를 따른다.

## 2026-09-05 — CoELA식 통신 효과 실험 경로

사용자가 “AI끼리 통신이 필요한가”를 연구 목표로 선택하고 구현을 승인했다.
`harness/coela_modules.py`와 `harness/coela_runtime.py`로 관측·기억·소통·계획·실행을
분리했다. actor 입력, 기억의 완료 필터, 깨우기 조건에서 숨은 팀 상태를 제거하고
공통 물리 제어·최소 동기화는 유지한다. `scripts/evaluate_coela.py`가 A 무통신,
B 정형 메시지, C 자연어 메시지를 비교한다. 실제 LLM 정상 과제는 세 조건 모두
각1회 3/3 성공. 별도의 C 비공개 장애물 실행도 복구·완료했고 한국어 대화 영상이 있다.
39개 집중 회귀 통과. 연구 설계·아티팩트·본실험에서 남은 범위는
`docs/coela_communication_study.md`를 따른다. 정상 과제 한 번의 성공률로 우열을 주장하지 않는다.
기본 coworker architecture는 coela, legacy는 명시적인 비교 경로로 유지한다.

## 2026-09-05 — 진행 중 이벤트와 안전한 도움 요청

`harness/task_recovery.py`, `sim/wrist_load_sensor.py`를 추가했다. busy agent도
pause event를 받으면 재관측/재판단하고, 공동 재개는 참여자 둘만 동의한다.
가상 손목 힘 측정으로 과부하를 감지하면 안전하게 낮춘 뒤 지원을 기다린다.
실제 LLM 장애물 재계획→3/3도착 검증. 세 조건 비교 pilot도 완료했다.
상세 상태·검증·미완료 범위는 `docs/event_recovery_review_20260905.md`를 따른다.
작은 상자의 물리적 공동 인계 방식은 사용자 선택 대기. 완전 sensor-only
motion 및 실제 하중/odometry calibration은 완료로 주장하지 않는다.

## 2026-09-05 — 단독/공동 혼합 과제와 독립 작업 루프

사용자 요청에 따라 기본 worker 과제를 `UGRP_WAREHOUSE_LAYOUT=mixed`로 변경했다.
긴 판자 1개(2명)와 작은 상자 2개(각1명), A→B 평탄한 작업영역이다.
기존 3개 공동화물/장애물 fixture는 `warehouse_layout="standard"`로 유지한다.
`harness/mixed_warehouse_runtime.py`의 로봇별 루프는 자기 작업이 끝나면 다음
화물을 선택한다. `MixedWarehouseReferee`는 1명/2명 참여자의 동의만 확인하며
세 번째 로봇을 기다리지 않는다. worker의 `MixedEngine`은 요청을 계속 받으면서
한 물리 clock으로 공동 스킬과 단독 파지·이송 상태기를 함께 갱신한다.
고정 scout는 혼합 과제에서 제거됐다. 역할/화물 선택은 LLM이 수행하고 심판은
자원 중복 점유만 막는다. 단독 파지도 bilateral contact 이후에만 weld하며
release/stability로 도착을 검증한다. 기존 SIM 이상적 저수준 피드백 한계는 유지한다.

실제 LLM 영상: `outputs/warehouse_research/mixed-llm-final/`.
R1/R2 판자, R3 상자2개, 세화물 모두 성공. R3가 첫 상자를23.85s에 마치고
25.60s에 다음상자를 선택했으며 판자 완료는50.06s였다(시뮬시간).
공동/단독 작업 활성 구간 겹침38.09s, robot collision 관측0.
두 화물이 동시에 평행 이동한 구간은 이 실행에서0s였으므로 작업동시성과
화물 이동동시성은 구분한다. 각 작업의 접근/파지/운반/해제 단계가 독립 진행된다.
같은 혼합과제의 실제 TEAM API live도3/3 성공,모든wake done,queue/inflight0.

## 2026-09-05 — 실제 병렬 주행 / 전진 방향 수정

이전 연구 경로의 순차 staging과 정지 scout를 교체했다. `sim/warehouse_crew.py`는
로봇별 `ForwardPathController`를 같은 physics tick에서 갱신한다. 접근/복귀는
갈 방향으로 회전 후 전진하며, scout는 pickup/route/next-cargo 관찰을 수행한다.
센서 pan도 동시에 움직인다. 최신 live 및 실제 LLM 영상 모두 3/3 완료.
1배속 증거: `outputs/warehouse_research/parallel-crew-reviewed/parallel-forward-1x.mp4`.
전용 궤적 측정: translation-only 3-way7.9s, 이동/회전 포함3-way45.1s,
최대 연속 후방 보정2.99cm, pose-reset0. 독립 LLM의 매 tick 재판단이나
여러 화물 동시 이송까지 구현했다는 뜻은 아니다. 자세한 범위는 decision_log 최신항목.

## 2026-09-05 — 창고 연구 경로

사용자의 “그거 다 해결해주라” 요청에 따라 기존 창고 실행의 세 문제
(중앙 역할 결정, 비교군 부재, actor의 정확한 SIM 상태 사용)를 분리해 수정했다.
현재 TEAM 창고 요청의 기본 모드는 `UGRP_WAREHOUSE_MODE=llm_peer_comm`이다.

- `harness/warehouse_runtime.py`: 로봇별 독립 completer를 동시에 호출한다.
  각 로봇은 자기 관측과 자기 이력, 전달된 자연어 메시지만 받는다.
  세 로봇이 각각 선택한 화물·역할표·목적지가 일치해야 실행한다.
  전송 계층은 관측 ID/revision을 묶으며 역할이나 화물을 채워 넣지 않는다.
- `harness/warehouse_protocol.py`: 관측 출처, 합의, 역할 중복, 신선도를 검사한다.
  없는 로봇 ID나 합의 불일치는 실패로 보존한다. 중앙 fallback은 없다.
- `sim/warehouse_observation.py`: 각 로봇의 실제 렌더링 RGB에서 ArUco ID를
  읽고 metric depth로 측정한다. bounded servo pan으로 재관측하며, 이전에 직접
  본 ID는 로봇별 기억으로 유지한다(현재 시야/과거 기억을 구분하고 옛 거리·방향은 폐기).
  질량·전역 위치·peer pose는 actor 입력에 넣지 않는다. 색상 기반 구분은 노란
  로봇 부품을 crate로 오인해 폐기했다. 물리 로봇의 마커 없는 인식 완료 주장은 하지 않는다.
- `sim/warehouse_research.py`: 승인된 한 화물/역할표를 기존 공통 운반 스킬로
  실행하고 심판 결과를 반환한다. 다음 화물은 다시 세 에이전트가 선택한다.
- `scripts/evaluate_warehouse_research.py`: `rule`, `llm_no_comm`, `llm_peer_comm`을
  같은 seed/scenario/budget/sensor/controller 조건으로 실행하고 JSONL·요약을 남긴다.
  규칙 비교군은 typed signal, LLM 통신 조건은 자연어만 전달한다.

기존 `team_zone_transfer`는 `central_baseline`으로 명시적으로 표시한다.
과거 아래의 `32/32` 등 수치는 이 중앙 제어 경로의 결과이며 새 peer 실험과 합산하지 않는다.
공통 low-level transport는 여전히 SIM의 이상적 내부 피드백을 사용한다.
이번 검증은 고수준 역할/화물 협상 실험이며 REAL 센서·동역학 보정 완료를 뜻하지 않는다.

## 1. 기존 플랫폼 아키텍처

```text
                     ┌──────────────── TEAM bus ────────────────┐
                     │        메시지/상태 공유만 담당            │
                     ▼                                          ▼
               ┌──────────┐   ┌──────────┐   ┌──────────┐
               │ R1 LLM   │   │ R2 LLM   │   │ R3 LLM   │
               │ agent    │   │ agent    │   │ agent    │
               └────┬─────┘   └────┬─────┘   └────┬─────┘
                    │              │              │
                    ▼              ▼              ▼
               per-robot      per-robot      per-robot
               ChatState      ChatState      ChatState
               ActionQueue    ActionQueue    ActionQueue
                    │              │              │
                    └───────┬──────┴──────┬───────┘
                            ▼             ▼
                    public robot skills
              search → track → approach → pick → carry/place
                            │
                    ┌───────┴────────┐
                    ▼                ▼
                 SIM adapter      REAL adapter
                 MuJoCo shared    MasterPi
                 world/bridge     actuator/camera
```

- R1/R2/R3는 각각 독립 LLM/세션/공개-skill queue를 가진다.
- TEAM bus는 로봇 간 대화와 상태 전달용이며 행동 결정을 대신하지 않는다.
- LLM이 선택하는 `search`, `track`, `approach`, `pick` 같은 public skill은 로봇별 `RobotActionQueue`를 거친다.
- 저수준 motor/servo pulse는 각 skill controller 내부에서 실행된다.
- SIM은 3개 로봇이 하나의 MuJoCo world/bridge를 공유하고, REAL은 MasterPi 제어 경로를 사용한다.
- SIM/REAL의 public skill 계약은 최대한 같은 의미를 유지한다.

## 2. 조작 파이프라인의 현재 계약

현재 구현과 public skill 계약은 다음 구조다.

```text
search
  ↓
track
  ↓
approach
  ├─ target/gaze 재정렬
  ├─ chassis alignment
  ├─ 약 26cm coarse corridor까지 거리 접근
  └─ stopped coarse handoff 저장
            ↓
pick
  ├─ 현재 근거리 재관측
  ├─ 0.20m 이내의 측정 기반 base/face/depth 보정
  ├─ fresh FK/IK 계산
  └─ grasp + verify
```

`approach`가 정밀 grasp plan까지 책임지던 결합은 제거되었다. 이 책임
경계는 `scripts/robot_actions.py`, `ROADMAP.md`, SIM self-observer에서 같은
의미로 유지한다.

## 3. TODO — 우선순위 순

### P0 — SIM latency / execution architecture (2026-09-04 완료)

- [x] Bridge가 실제 browser surface별 구독을 worker에 전달하고 구독자 0이면 presentation render를 중단한다.
- [x] controller sensor JPEG와 static presentation frame을 재사용하고, physics가 변한 경우에만 다시 렌더한다.
- [x] 단일 command의 0.75초 batching tax를 제거한다. 일반 burst grace는 최대 50ms이고, consensus TEAM 첫 행동만 명시적 batch ID로 최대 5초 rendezvous한다.
- [x] trace persistence를 result/Bridge condition lock 밖의 bounded writer로 분리한다.
- [x] 실패 trace 이미지는 전부, CLEAN trace 이미지는 기본 5회 중 1회만 보존한다.
- [x] `SimClock`의 20ms quantum이 매번 full public state를 만들지 않게 한다.
- [x] REAL-controller module dependency와 handoff path를 robot별 ContextVar로 격리해 public skill을 실제로 겹쳐 실행한다. 공유 `mj_step`만 physics lock으로 직렬화한다.
- [x] macOS CGL renderer를 단일 owner thread broker로 옮겨 병렬 camera action이 shared OpenGL context에서 교착되지 않게 한다.
- [x] parallel batch의 robot별 partial result를 즉시 fan-out해 짧은 action이 느린 peer의 완료를 기다리지 않게 한다.
- [x] worker replacement가 parallel batch의 원래 robot별 caller 모두에게 `SIM_EPISODE_RESET`을 즉시 fan-out한다.
- [x] plan epoch, pending TTL, 동일 failure/state retry 억제와 skill별 retry budget을 적용한다.
- [x] SIM camera availability를 Bridge authority와 결합하고 quiet-idle frame age를 talk-only 오프라인으로 오판하지 않는다.
- [x] `UGRP_SIM_FAST_CAMERA_CONTROL=1`에서 deterministic camera confirmation을 1회 중심으로 줄이고, 18° 이내 face error는 arm yaw로 흡수한다. REAL 확인 횟수와 20° final abort는 변경하지 않는다.
- [x] `UGRP_SIM_FAST_CONTINUOUS_APPROACH`는 seed 15 중앙 lock 회귀 때문에 기본 비활성화하고 opt-in 실험으로만 남긴다.

검증 스냅샷: 구독자 0의 live worker CPU 0.1–0.2% (변경 전 46–59%),
`observe_scene` caller 0.278s / worker 0.107s, R1/R2/R3 0.20s public
motion의 Bridge E2E wall 0.049s 및 3-way overlap 0.0147s. seed
11/12/14/15 full chain 모두 `CLEAN` 성공했다. fast camera-only 격리 실측은
각각 10.64/14.69/7.24/10.87초(평균 10.86초), live Bridge seed 11은
11.96초였으며 표준 `.venv-sim` 전체 893 tests를 통과했다.

### P0 — 조작 구조 정리

- [x] `approach`를 coarse reachable corridor까지만 담당하도록 단순화한다.
- [x] 최종 FK/IK, hand-eye capture, grasp geometry 계산을 `pick`으로 옮긴다.
- [x] 정확한 pick plan 대신 짧은 수명의 stopped coarse handoff만 전달한다.
- [x] `approach` 안의 face/close-capture 경로를 `pick`으로 분리한다.
- [x] pick chassis 이동량 0.20m 계약을 self-observer가 검사한다.
- [x] 변경 후 SIM에서 `search → track → approach → pick` 전체 흐름을 실제로 성공시킨다.

### P1 — 주행 정책 일관성

- [ ] 모든 실제 제어 경로의 최소 유효 wheel speed를 `35`로 통일한다.
  - `scripts/masterpi_control.py`: 35
  - `scripts/red_block/primitive.py`: 35
  - `scripts/red_block/robot.py`: 35
  - `dashboard/transport.py`: 현재 31 → 수정 필요
- [ ] approach에서는 pure lateral/strafe에 의존하지 않는 현재 정책을 유지한다.
- [ ] `left/right` primitive가 필요한 다른 동작과 approach 정책을 명확히 분리한다.

### P1 — 3-agent 실제 병렬 동작 검증

- [x] R1/R2/R3가 별도 harness process와 ChatState/ActionQueue를 사용한다.
- [x] 세 로봇에 동시에 public command를 주고 Bridge가 반환한 overlap timeline을 확인한다.
- [x] R1을 0.80s, R2/R3를 0.20s public motion으로 실행해 R2/R3 응답이 R1보다 먼저 반환되는지 확인한다.
- [x] ContextVar handoff/detector 및 로봇별 ActionQueue/history가 섞이지 않는 회귀 테스트를 유지한다.
- [x] TEAM 명령 → 세 agent 판단 → 세 queue → 세 SIM robot 행동까지 E2E로 검증한다.
- [x] 독립 planner 응답 시차가 50ms를 넘어도 consensus 라운드의 첫 actuator command 세 개를 하나의 `act_parallel` batch로 묶는다.
- [x] tower의 stage 2/3가 predecessor postcondition을 기다리고, stage 3 뒤에만 `tower_complete`를 기록한다.
- [ ] 명시적 색상 역할이 현재 위치와 비효율적으로 충돌할 때 거리/가시성 기반 재배치 또는 빠른 실패 후 peer 재할당을 구현한다.

2026-09-04 live seed 11 증거: R1=yellow, R2=blue, R3=red TEAM search에서
proposal wake 3개와 execution wake 3개가 각각 동시에 시작했고, worker 로그가
`parallel ['r1', 'r2', 'r3'] action_s 0.525`를 기록했다. 세 검색은 모두 성공했고
TEAM 방 요청부터 마지막 agent 응답까지 8.80초였다. 반대로 먼 역할
R1=red/R3=yellow는 교착 없이 끝났지만 R3의 반복 탐색 때문에 72.4초가 걸려,
남은 병목은 동시성보다 역할 배치/재할당 정책임을 확인했다.

### P1 — Cooperative Payload Mission MVP

- [x] `beam`, `panel`, `pipe`, `tray` 형태와 상대 질량을 가진 payload catalog를 추가한다.
- [x] 45cm beam과 출발 구역 A/도착 구역 B를 shared MuJoCo world에 추가한다.
- [x] 단일 로봇의 `team_beam_transport` 호출은 물체를 움직이지 않고 `COOPERATIVE_QUORUM_REQUIRED`로 거부한다.
- [x] R1=`carrier_left`, R2=`scout`, R3=`carrier_right` 세 명령이 같은 consensus batch에 있을 때만 joint controller를 연다.
- [x] R1/R3의 서로 다른 endpoint bilateral contact 뒤에만 dual grasp constraint를 활성화한다.
- [x] shared physics time에서 공동 lift/carry를 실행하고 최종 center/yaw/level/height/stability를 referee가 판정한다.
- [x] TEAM 자연어 명령 → 독립 role proposal → consensus → role compiler → 3-command batch → 물리 mission까지 live E2E로 검증한다.
- [ ] 고정 MVP role mapping을 camera/거리 기반 동적 role auction으로 교체한다.
- [ ] `cooperative_align/grasp/carry_step/place/release` phase별 robot-local action과 barrier sequence로 세분화한다.
- [ ] panel/pipe/tray의 실제 MuJoCo body/controller와 장애물·좁은 통로 level을 추가한다.

Live seed 11: 세 agent가 서로 다른 역할을 제안했고, worker는
`parallel ['r2', 'r3', 'r1'] action_s 5.926`으로 실행했다. TEAM 요청부터
최종 응답까지 12.64초, beam 중심은 목표 `(1.08, 0.00)` 대비 2.2mm,
yaw 오차 0.16°, 최종 속도 0.0001m/s였으며 constraint 해제 후 referee
`SUCCESS`를 확인했다. 현재 역할 compiler는 합의된 계획을 public action으로
변환하는 MVP 계층이며, 장기 목표는 위 phase별 완전 분산 실행이다.

### P1 — Multi-zone Warehouse Transfer (사용자 요청 반영)

- [x] A=`blue`, B=`green`, C=`yellow`를 같은 X축의 비중첩 checkpoint로 배치하고 observer overlay를 추가한다.
- [x] 자연어 `A구역의 모든 짐을 B구역으로`, `파란 구역 짐 전부 초록 구역으로`를 같은 canonical mission ID로 변환한다.
- [x] 구역 색과 화물 색을 분리하고 `파란 구역의 파이프`는 zone=A, selector=pipe로 해석한다.
- [x] 실제 MuJoCo free body로 긴 oak plank, steel pipe, wood crate를 추가하고 재질·색 label·형태를 다르게 만든다.
- [x] `team_zone_transfer` 하나가 세 화물을 모두 소유하고, 로봇별 별도 미션으로 쪼개지지 않게 한다.
- [x] 화물 fixture의 고정 carrier/scout를 제거하고, 실제 robot pose와 관찰된 cargo 양끝을 사용한 runtime role auction으로 매 작업의 두 운반자와 observer를 정한다.
- [x] 단일 robot command는 거부하고, 세 agent가 같은 mission/route/selector를 제출한 batch만 실행한다.
- [x] 모든 cargo가 요청한 최종 거점에 도착하고 `remaining_ids=[]`, shape별 zone/pose/stability가 통과해야 전체 SUCCESS다.
- [x] warehouse 전용 `cctv_warehouse` overview와 실제 움직임 MP4 recorder를 추가한다.
- [x] 인공 handle body/geom을 제거하고 두 robot arm yaw가 서로 반대가 된 뒤 실제 cargo 본체 양끝 bilateral contact로만 dual weld를 활성화한다.
- [x] `A→B`, `B→C`, `C→B`, `B→A`, `A→C(B 경유)`를 같은 route graph와 current-zone guard로 실행한다.
- [x] seed마다 cargo 길이/폭/높이/질량/초기 yaw가 달라지는 procedural object profile을 만들고, planner는 cargo ID/type이 아니라 live MuJoCo geometry observation으로 끝점을 계산한다.
- [x] route 사이에 seed-dependent physical mound를 배치하고, observer가 높이/통과 가능성/비용을 보고하면 carrier가 world-frame mecanum 제어와 감속으로 통과한다.
- [x] seed마다 `mound + hard barrier + north/south gate post` 4개 지형을 함께 생성하고, cargo+두 carrier swept envelope로 inflation한 4-neighbor A*를 실제 carry/empty-return/final-approach에 공통 적용한다.
- [x] 실행 중 `request_warehouse_goal_update()` revision이 들어오면 release 전에 새 destination을 감지하고 route를 연장·재계획한다. stale/unknown revision은 거부한다.
- [x] `team_bid_submitted → role_auction_completed → carrier_ready → lift_commit_barrier → scout_terrain_report`를 실제 수치 증거와 함께 trace에 남긴다.
- [ ] 특정 type/color selector 실행과 색 label 자동 분류 목적지를 확장한다.

현재 기본 warehouse transport에는 `warehouse_*handle*` body/geom이 없다. 두 carrier는
route-derived staging line을 거쳐 cargo long-axis 양끝 바깥에 서고, chassis heading은
운반 축을 유지한 채 arm yaw만 서로 반대 방향으로 돌린다. 각 gripper site는 실제 cargo
end face 안쪽 12mm 지점으로 feedback 정렬되고, 양쪽 robot 모두 실제 cargo main geom과
bilateral contact한 뒤에만 cargo body direct weld가 활성화된다. 기본 실행 중
base/free-body pose setter 호출은 0회다.
다만 chassis/contact 계수는 아직 REAL 계측으로 보정되지 않았으므로 sim-to-real
동역학 정확도를 주장하지 않는다.

자연어 `A구역의 모든 짐을 C구역으로`는 route `A→B→C`로 컴파일된다. B에서는
화물별 `warehouse_waypoint_reached(final=false)`를 남기며 grasp를 유지하고, C에서만
release/stability를 판정한다. 같은 world를 reset하지 않은 `A→B→C→B→A` 왕복도
current zone을 이어서 사용한다.

Warehouse reset은 화물 종류별 안전 레인을 유지하면서 각 화물의 A/B/C 거점별
`x/y`뿐 아니라 metric geometry, mass, yaw와 mound 위치/크기도 seed마다 실제 변경한다.
scene fixture의 `CargoSpec.carriers/scout`는 비어 있고, 매 화물의 처리 순서와 역할은
현재 scene snapshot에 대한 bid cost로 정해진다. 일반 3단 탑 블록 배치는 같은
world의 warehouse cargo footprint와 겹치면 재추첨한다. Seed 25에서 red cube가
pipe와 겹쳐 시작부터 넘어뜨리던 재현을 이 gate로 제거했고, seed 0..31 스트레스는
inward body-grasp 기준 A→B seed 0..31은 `32/32`, B checkpoint를 포함한 A→C
seed 0..31도 `32/32`였다. Seed 0..7의 reset 없는 `A→B→C→B→A` 연속 왕복은
`8/8` 성공했다. 최종 전체 회귀는 927 tests, 159.223초, all passed다.
이동 중 eye-in-hand는 기존 수직 arm의 `+74.3°` 천장 방향 대신 SEARCH 자세의
약 `-21.6°` 바닥 방향을 유지한다.

Live seed 25 자연어 `A구역의 모든 짐을 C구역으로 옮겨줘.`는
`warehouse_a_to_c_311209e1d2`, route `A→B→C`로 컴파일되어 11.825초에
완료됐다. 세 cargo 모두 B에서 `final=false` waypoint를 남긴 뒤 C에 도착했고,
세 grasp의 facing dot은 모두 `-1.0`, arm yaw는 각 pair `+90°/-90°`, transport는
모두 `DYNAMIC_INWARD_BODY_GRASP_MECANUM`이었다. 최종 `moved_count=3`,
`remaining_ids=[]`, queue/inflight 0을 확인했다.

Adaptive SIM 검증에서는 A→B 32개 seed와, 첫 carry leg 도중 목표를 B에서 C로
바꾸는 8개 seed를 별도 stress한다. 목표 revision을 받은 cargo는 B에서 release하지
않고 B→C leg를 붙이며, checkpoint 회전 뒤 chassis heading이 달라도 world-frame
mecanum 명령으로 축 이동을 유지한다. 다만 현재 geometry/terrain observation source는
`sim_geometry_observation`(MuJoCo state)이다. REAL에서 같은 일반화를 주장하려면
metric depth 또는 IMU 기반 geometry/ground-plane confidence gate가 추가로 필요하다.

복합 obstacle-course revision에서는 A/B/C 중심을 `(1.05, 1.85, 2.65)m`로 옮겨
기존 tower block 작업대와 warehouse grasp envelope가 처음부터 겹치지 않게 했다.
완만한 mound는 `6–10mm`, barrier/gate는 `traversable=false` 실제 box collision
geom이며 gate inner gap은 seed마다 약 `1.20–1.38m`다. Carry path는 다른 cargo,
tower block, scout 위치까지 occupancy에 포함하고, 빈 carrier의 복귀와 final approach도
같은 planner를 사용한다. 좁은 waypoint에서 0° 제자리 회전으로 경로가 틀어지던 문제는
현재 yaw를 보존한 world-to-local mecanum 변환으로 제거했다. 최신 대표 검증은
복합 A→B seed `0..7 = 8/8`, gate 포함 A→C `8/8`, carry 중 B→C retarget
`8/8`, reset 없는 A→B→C seed `0..3 = 4/4`다.
최신 local worker/coworker E2E에서도 자연어 A→C가 `moved=3`, A* local path 6개,
scout route report 6개로 성공했다. 먼저 완료한 coworker만 최신 referee state를 보고
나머지 둘이 성공 결과를 `error`로 표시하던 race는 authoritative
`mission_verified/outcome_status=ACHIEVED`를 우선하도록 수정했고, 재실행에서 proposal
3개와 execution 3개가 모두 `done`, queue 0임을 확인했다.

### 2026-09-05 — lateral motion 현실화

- empty carrier의 A*/final-approach/return 경로는 pure mecanum lateral을 사용하지 않는다.
  Y edge는 `±90° turn → straight forward → +X re-face`로 실행한다.
- loaded pair는 servo6 `±90°` 한계 때문에 chassis를 함께 90° 돌릴 수 없으므로,
  obstacle lane-change와 grasp/dock 미세 정렬에만 strafe를 허용한다.
- loaded A*의 Y cost는 X의 3배이고, cargo lane Y는 zone마다 새로 뽑지 않고
  seed별 lane 하나를 A/B/C에서 유지한다.
- gate inner half-gap은 `1.22–1.28m`, barrier lateral half extent는
  `0.08–0.12m`로 조정해 외곽 대우회를 강제하지 않게 했다.
- loaded strafe는 scale `≤0.18`, 한 번의 feedback chunk `≤0.08m`로 제한한다.

최신 계측: 기존 seed 11 A→C loaded lateral ratio 약 `42%`에서 `23.7%`로 감소,
empty lateral segment는 `0`. 복합 A→B seed `0..7`은 `8/8`, 최대 loaded lateral
ratio `25.81%`; A→C seed `0..7`도 `8/8`, 최대 `31.23%`였다. 관련 27개
warehouse/adaptive/continuity 회귀는 `327.359s / OK`였다. rotate+forward의
stop/measure가 늘어 SIM time은 증가했으며, waypoint merge는 후속 성능 과제다.
최종 저횡이동 revision의 관련 12개 회귀는 `159.482s / OK`. 최신 worker live
A→B는 `empty_lateral_segments=0`, `loaded_lateral_ratio=19.22%`, loaded lateral
총 `0.554m`, forward `2.3285m`였고 proposal/execution wakeup 6개가 모두 `done`,
queue/inflight 0으로 끝났다.

최신 검증(2026-09-04): procedural object/terrain A→B `32/32`, carry 도중 B→C
retarget `8/8`, 전체 회귀 `933 tests / 120.591s / OK` 후 새 worker와 세 SIM
coworker를 재시작했다. 실제 TEAM 자연어
`A구역의 모든 짐을 C구역으로 옮겨줘. 처음 보는 물체 모양과 지형을 관찰하고,
현재 위치를 기준으로 역할을 정해서 협업해.`도 seed 11에서 SUCCESS였다.
parser가 `관찰`의 한 글자 `관`을 pipe alias로 오인하던 문제는 단어 경계 회귀
테스트로 수정했다. live 결과는 roles `R1+R3, R2+R3, R1+R2`, bid 9건,
carrier ready ack 6건, lift barrier 3건, terrain report 1건이었다.

### P2 — UI / 관찰 경로 검증

- [ ] R1/R2/R3 탭을 전환하며 3인칭 observer가 실제 각 로봇을 따라가는지 브라우저에서 재검증한다.
- [ ] TEAM 화면의 R1/R2/R3 카메라가 서로 다른 최신 프레임인지 확인한다.
- [ ] SIM shared-world reset 후 세 세션/카메라/queue가 같은 episode로 정리되는지 확인한다.

### P2 — queue 보강

- [ ] 현재 public-skill queue는 유지한다.
- [ ] `/api/turn` busy 시 `409 robot_busy`로 버리는 대신 요청 자체를 기다리게 할 필요가 있는지 결정한다.
- [ ] 서비스 재시작 시 queue 유실을 허용할지, persistence가 필요한지 결정한다.

### P3 — 연구/Sim-to-Real 장기 과제

- [ ] MasterPi 실제 동역학/servo/gripper 캘리브레이션 데이터를 수집한다.
- [ ] `training_ready=false`인 SIM physics를 실제 계측값으로 보정한다.
- [ ] REAL odometry/외부 위치 계측 전략을 확정한다.
- [ ] REAL 3대 구성 또는 1 REAL + SIM peer 연구 조건을 확정한다.
- [ ] L1-L3 실험 조건, baseline, logger를 확정하고 본실험 가능한 상태로 만든다.

## 4. 다음 세션 시작 순서

다음에 이어서 작업할 때는 아래 순서로 보면 된다.

1. 이 문서(`docs/current_architecture_todo.md`) 확인
2. 최신 변경 이력은 `docs/decision_log.md`의 맨 아래부터 확인
3. 조작 문제면 `scripts/red_block/approach.py`, `pick.py`, `physical_state_machine_reference.py` 확인
4. agent/queue 문제면 `harness/web.py`, `harness/loop.py`, `harness/action_queue.py`, `harness/team_bus.py` 확인
5. SIM 동시성/카메라 문제면 `sim/bridge.py`와 최신 `outputs/sim_traces/` 확인
6. 수정 후에는 unit test만으로 끝내지 말고 실제 SIM 사용자 흐름까지 검증

## 5. 완료 판정 원칙

- 코드가 존재하는 것과 실제 동작이 검증된 것은 구분한다.
- 3-agent는 “세 completer 생성”이 아니라 “실제 동시 행동 + 지연 격리”까지 확인해야 완료다.
- 조작은 개별 함수 테스트가 아니라 `search → track → approach → pick` 사용자 흐름 성공까지 확인해야 완료다.
- REAL은 사용자 명시 없이 물리 actuator를 실행하지 않는다.
