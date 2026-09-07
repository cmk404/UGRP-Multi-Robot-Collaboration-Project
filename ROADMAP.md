# UGRP ROADMAP

> 최종 갱신: 2026-09-04  
> 역할: 실행 순서와 단계별 완료 기준. 연구 배경은 `README.md`, 현재 상태·결정은 `docs/decision_log.md`.  
> 규칙: 공식 제출계획과 Isaac Lab 수정안을 **승인 없이 합치지 않는다**. 미승인 제안은 확정 사양처럼 쓰지 않는다.

## 2026-09-05 추가 — 실행 가능한 창고 비교 실험

아래 09-02 스냅샷의 “비교군 미구현” 상태는 역사적 기록이다. 기존 MuJoCo
MasterPi 창고 범위에서 세 조건·sensor actor boundary·독립 peer decision·JSONL
실험 실행을 구현했다. 공식 제출계획과 Isaac Lab 수정안의 승인 상태는 합치지 않는다.
실행 방법과 통제 변수는 `docs/warehouse_research_contract.md`를 따른다.

- [x] 중앙 역할 배정은 명시적 baseline으로 분리한다.
- [x] 세 정책의 독립 선택과 전송/심판의 역할을 분리한다.
- [x] RGB-D 측정에서 actor 관측을 만들고 privileged state를 제외한다.
- [x] rule/no-comm/peer-NL을 같은 episode 계약으로 실행·집계한다.
- [x] 실제 TEAM HTTP → 세 독립 LLM → 물리 이송 → UI 결과를 검증한다.
- [ ] 충분한 held-out seed 수와 모델 반복으로 통계적 효과를 검증한다.
- [ ] REAL 센서 구성·동역학 파라미터를 실측 보정한다.

## 0. 이전 스냅샷 (2026-09-02, 현황만)

| 항목 | 상태 | 비고 |
| --- | --- | --- |
| 연구 인덱스 (`README.md`) | 있음 | 공식안 / 수정안 / 참고 자산 3층 구분 |
| 결정 로그 (`docs/decision_log.md`) | 운영 중 (2,100+ 줄) | 엔지니어링 결정·세션 인계의 기준. "현재 상태" 스냅샷은 09-02 항목 참조 |
| L1–L3 태스크 명세 | 골격 | `docs/l1_l2_l3_task_spec.md` (값 _TBD_). **Phase B 미승인** |
| 메시지 스키마·행동 스킬 | 구현됨 (플랫폼) | 공개 스킬 카탈로그 `scripts/robot_actions.py`, TEAM 공유 채팅 `harness/team_bus.py`. 연구 사양 확정본은 아님 |
| 비교군·메트릭·로거 스펙 | 초안만 | rule baseline·no-comm 조건·실험 로거 **미구현** (Phase D 착수 전) |
| SIM (MuJoCo 3-robot 디지털 트윈) | 운영 중 | `sim/`, physics `V2_STRUCTURAL_UNCALIBRATED`, `training_ready=false`. compute: Mac M3 worker + Oracle 브리지 (클라우드 GPU 전부 퇴역, `docs/cloud_simulation.md`) |
| REAL (MasterPi) | ugrp1 실동작, ugrp2/3 부분 | `scripts/red_block/`, `harness/`, Oracle `ugrp-real.service`. 동일 스킬 소스를 SIM 어댑터가 재사용 |
| 저장소 | Oracle canonical Git, Mac은 Syncthing 미러 | 09-02 WIP 스냅샷 + 정리 커밋 이후 회귀 스위트 기준 (`docs/decision_log.md` 09-02) |
| 연구 코드 (LLM peer 통신 실험) | 플랫폼만 | 조건 비교(D1–D3)·로거(D4)는 사양 고정 전 미착수 |

### RED-BLOCK 제어 아키텍처 (REAL 우선, 2026-09-04)

`search → track → approach → pick`은 REAL과 SIM에서 같은 planner/action 경로를 사용한다. SIM 전용 hidden state(블록 월드 좌표, 정답 자세 등)는 planner 입력으로 쓰지 않고 심판·trace·회귀 검증에만 사용한다.

| 단계 | 책임 | 경계 |
| --- | --- | --- |
| `search` | 카메라에서 목표 탐색·확정 | actor-visible 관측만 사용 |
| `track` | 목표를 추적 가능한 상태로 유지 | base grasp 계획을 미리 만들지 않음 |
| `approach` | 약 26cm의 coarse arm-reach corridor까지 base staging | autonomous pure mecanum `left/right` strafe 금지. stopped coarse handoff만 저장하고 최종 FK/IK는 만들지 않음 |
| `pick` | 정지 재관측 → bounded near-field base/face/depth 보정 → fresh IK → grasp/lift | coarse handoff 이후에만 실행. chassis 평행이동 총량 0.20m 이하, 측정 없는 blind motion 금지 |

REAL에서 횡이동 효과가 작고 불안정한 것이 현재 설계 기준이다. 따라서 SIM도 이상적인 횡이동으로 쉽게 풀지 않고, 같은 bounded dog-leg와 실제 관측 progress gate를 거쳐 성공해야 한다.

현재 검증 상태:

- seed 11/12/14/15에서 coarse `approach` 뒤 near-field moving `pick`까지 live SIM `CLEAN` 성공 확인.
- seed 12의 과거 실패는 마지막 허용 pulse 뒤 stopped 결과를 재측정하지 않던 off-by-one이었다. 추가 motion 없이 최종 measurement를 수행하도록 수정했다.
- SIM fast camera-only 모드는 hidden truth 없이 deterministic frame 확인 중복을 줄인다. seed 11/12/14/15 격리 평균 10.86초, live Bridge seed 11 11.96초이며 REAL camera confirmation 기본값은 유지한다.
- REAL 상수는 SIM 편의를 위해 변경하지 않는다. SIM-only calibration이 필요하면 actor-visible camera + 모델 fixture 기반 offline calibration으로 제한한다.

## 1. 실행 순서

### Phase A — 문서·사양 골격 (지금)

| ID | 산출물 | 완료 기준 | 담당 |
| --- | --- | --- | --- |
| A1 | `ROADMAP.md` 골격 | 현황·단계·완료 기준이 README와 모순 없이 존재 | docs |
| A2 | `docs/decision_log.md` 골격 | 현재 상태 / 미해결 / 결정(빈 칸) / 세션 인계 섹션 존재 | docs |
| A3 | L1–L3 명세 골격 | 초기상태·성공/실패·max step·이벤트 주입 칸만 (값 확정은 승인 후) | docs |
| A4 | 메시지 스키마·행동 스킬 범위 | 필드·스킬 후보 목록, 공식안/수정안 혼선 표시 | protocol |
| A5 | 비교군·메트릭·로거 초안 | 스펙 초안만, 구현 금지 | eval |

### Phase B — 연구 사양 고정 (승인 필요)

| ID | 산출물 | 완료 기준 |
| --- | --- | --- |
| B1 | 환경·로봇 수·에이전트 구조 선택 | decision_log에 **결정 + 승인 상태** 기록 (공식안 vs 수정안 중 택일 또는 병행 범위 명시) |
| B2 | L1–L3 값 확정 | A3 칸이 채워지고 decision_log에 근거·영향 기록 |
| B3 | 메시지 스키마·행동 스킬 확정 | 자연어 원문 + 구조화 행동 동시 저장 규칙 확정 |
| B4 | 비교군·메트릭·로거 확정 | eval 초안이 승인된 사양으로 승격 |

### Phase C — 플랫폼·프로토타입 (사양과 병행 가능하되 혼선 금지)

| ID | 산출물 | 완료 기준 | 담당 |
| --- | --- | --- | --- |
| C1 | MasterPi 연결·제어 | 제어 스크립트·상태 확인 절차가 `outputs/`에 재현 가능 — **완료 (ugrp1)** | masterpi |
| C2 | 대시보드 | 관측/제어 UI가 로컬에서 기동·기본 조작 가능 — **완료** (`dashboard/`, `harness/web.py`) | masterpi |
| C3 | 환경 심판 경계 | 충돌·접촉·동기화·성공만 판정, 계획 미제공 — **부분** (SIM `TaskExecutive` 전제조건 게이트, 성공 판정 `team_stack_verified`) | (사양 후) |
| C4 | SIM 디지털 트윈 캘리브레이션 | `training_ready=true` (동역학 캘리브레이션 입력 수집·적용). **미완료** — sim-to-real 병목 1순위, `docs/decision_log.md` 09-02 | sim |
| C5 | RED-BLOCK REAL-like base staging | pure strafe 없이 dog-leg만으로 actor-visible 관측 기준 pregrasp 형성. seed 11 `approach` 성공, multi-seed 검증 필요 | sim+real |
| C6 | coarse-to-near-field pick handoff | `approach`는 stopped coarse handoff만 만들고, `pick`이 0.20m 이내의 측정 기반 near-field 보정·fresh IK·grasp/lift를 수행. SIM hidden truth shortcut 없음 | sim+real |
| C7 | full-chain 회귀 검증 | seed 11 이후 multi-seed에서 `search → track → approach → pick` 전 구간 성공 및 trace 기준 통과 | sim |
| C8 | Warehouse Transfer Mission MVP | 파란 A·초록 B·노란 C 구역, 실제 목재 판재/금속 파이프/화물상자 3종, 자연어 route compiler, 회전하는 2-robot carrier pair로 `A의 모든 짐 → B` 완료 — **structural v1 완료** | team+sim |

> C 단계는 하드웨어·데모 자산이다. C의 구현을 B의 “확정 연구 사양”으로 읽지 않는다.

Warehouse manifest는 `oak_plank`, `steel_pipe`, `wood_crate` 실제 MuJoCo body와
A/B/C 구역을 제공한다. 기본 실행은 `STRUCTURAL_KINEMATIC_UNCALIBRATED`이며,
화물 간 역할 재배치와 연속 운반을 시각화·프로토콜 검증하는 용도다. 질량과
실제 동역학 성공 주장이 아니며, 완전 dynamics 경로는 C4 캘리브레이션 뒤 승격한다.

#### C5–C7 TODO / 완료 기준

1. `approach` 종료 조건이 약 26cm coarse arm-reach corridor와 stopped target visibility를 보장하는지 검증한다.
2. `pick`은 stopped fresh remeasure 뒤에만 0.20m 이내의 bounded near-field base/face/depth 보정을 허용하고, 각 pulse 뒤 재측정한다.
3. self-observer는 pick의 chassis 이동량이 계약을 넘으면 성공 여부와 무관하게 critical로 판정한다.
4. seed 11 full chain을 먼저 재실행하고, 이후 seed 12–15까지 같은 경로로 반복한다. 실패 seed는 trace 근거로 수정하고 threshold 완화로 통과시키지 않는다.

Multi-seed full-chain acceptance criteria:

- 각 seed에서 `search`, `track`, `approach`, `pick`이 모두 `ok=true`로 완료된다.
- pick trace `analysis.status=CLEAN`.
- `held_color=red`.
- `left_contact=true`, `right_contact=true`, `bilateral_contact=true`.
- `lifted=true`, `stable=true`.
- planner/action 입력에 SIM hidden truth가 섞이지 않았고 REAL/SIM이 같은 제어 경로를 사용한다.
- autonomous pure mecanum strafe 없이 bounded dog-leg만으로 면방향 staging이 완료된다.

### Phase D — baseline·실험 (B 완료 후)

| ID | 산출물 | 완료 기준 |
| --- | --- | --- |
| D1 | 규칙 기반 baseline | 동일 환경·시드에서 실행 가능 |
| D2 | 통신 없는 LLM | 동일 예산·max step |
| D3 | 자연어 peer 통신 LLM | 핵심 가설 조건 |
| D4 | 실험 로거 | seed, episode, step, 메시지, 행동, 비용, 실패 원인 재현 저장 |
| D5 | L1–L3 데이터 수집 | 고정 시드·이벤트 규칙 하에 로그 산출 |

### Phase E — 분석·보고

| ID | 산출물 | 완료 기준 |
| --- | --- | --- |
| E1 | 통계·비교 분석 | 통신 방식 vs 환경 난이도 분리 가능 |
| E2 | 보고서·발표 자료 | 계획서 일정(2026.12–2027.01) 참고. 실제는 decision_log |

## 2. 일정 앵커 (계획서 참고, 완료 여부 ≠ 이 표)

| 기간 | 계획서 단계 | 이 ROADMAP과의 대응 |
| --- | --- | --- |
| 2026.05–06 | 환경·사전 실험 | Phase C 일부 + 사전 측정 (로그로 확인) |
| 2026.07–08 | 프로토콜·프롬프트·rule baseline | Phase A–D 초입 |
| 2026.09–10 | L1–L3 본실험 | Phase D |
| 2026.11 | 분석·실물 검증 | Phase E |
| 2026.12–2027.01 | 보고서·발표 | Phase E |

## 3. 하지 않을 것 (명시)

- 공식안과 Isaac Lab 수정안을 승인 기록 없이 하나로 합치기
- 브라우저 sandbox(`decentral-cobot`)의 keyword router를 LLM 자연어 통신으로 서술하기
- 생성 초안·예상 수치를 실험 결과로 기록하기
- 사양(B) 고정 전 L1–L3 본실험 코드·로거 본구현에 착수하기 (초안 문서만 허용)

## 4. 다음 문서 작업 (docs)

1. ~~ROADMAP·decision_log 골격~~
2. ~~L1–L3 명세 골격 (값 비움)~~ → `docs/l1_l2_l3_task_spec.md`
3. protocol·eval 초안과 필드명 충돌 점검
4. (승인 후) 명세 값·스키마·평가 스펙을 decision_log와 동기화
5. C5–C7 multi-seed 결과가 확정되면 seed별 성공/실패와 calibration 근거를 decision_log에 동기화
