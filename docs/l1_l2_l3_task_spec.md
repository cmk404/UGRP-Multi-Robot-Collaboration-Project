# L1–L3 태스크 명세 (골격)

> 최종 갱신: 2026-08-22 (1차 필드 대조)  
> 역할: 공통 태스크의 **칸만** 정의. 수치·시나리오 값은 사양 승인(ROADMAP Phase B) 후 채운다.  
> 규칙: 공식 제출계획과 Isaac Lab 수정안을 **같은 행에 합치지 않는다**. 아래는 층별로 분리 표기.  
> 관련: `README.md` §3.2·§4.1, `ROADMAP.md` A3/B2, `docs/decision_log.md` O2.

---

## 0. 공통 필드 (모든 레벨)

실험 로그·protocol·eval과 맞출 **이름만** 고정. 의미·타입 확정은 B 이후.

| 필드 | 용도 | 값 (지금은 비움) |
| --- | --- | --- |
| `seed` | 에피소드 재현 | _TBD_ |
| `episode_id` | 에피소드 식별 | _TBD_ |
| `step` | 이산 시간 인덱스 | _TBD_ |
| `max_steps` | 에피소드 상한 | _TBD_ |
| `agent_ids` | 참여 에이전트 | _TBD_ |
| `object_ids` | 대상 물체 | _TBD_ |
| `initial_state` | 배치·자세·목표 구역 등 | _TBD_ |
| `success_criteria` | 성공 판정 | _TBD_ |
| `failure_criteria` | 실패 판정 (타임아웃 포함) | _TBD_ |
| `event_injection` | 장애물·고장·목표 변경 등 | _TBD_ (L3 중심) |
| `message_log_ref` | 자연어 원문 + 구조화 행동 (protocol) | 필드명만 맞춤 → protocol 초안 |
| `action_log_ref` | 실행된 행동 스킬 | 필드명만 맞춤 → protocol 초안 |
| `cost_log_ref` | turn·토큰 등 (eval) | 필드명만 맞춤 → eval 초안 |
| `failure_reason` | 실패 원인 코드/서술 | protocol·eval과 용어 통일 예정 |

> 환경 심판은 충돌·접촉·동기화·성공/실패만 판정하고 **계획을 제공하지 않음** (README). 심판 API는 이 문서에 아직 두지 않음.

---

## 1. 층 분리: 공식안 vs 수정안

값을 채울 때 아래 **둘 중 하나(또는 병행 범위)**만 선택하고, decision_log에 승인 상태를 남긴다. 여기 골격에서는 시나리오 라벨만 둔다.

### 1.1 공식 제출계획 (로봇팔·블록)

| 레벨 | 시나리오 라벨 (README) | 검증 포인트 (라벨만) | 초기상태 / 성공 / 실패 / max_steps / 이벤트 |
| --- | --- | --- | --- |
| L1 | 작은 블록 단독 분류·이송 | 기본 인식, 단독 작업 계획 | _TBD_ |
| L2 | 큰 블록 공동 파지·이송 | 파트너 요청, 역할 합의, 출발 동기화 | _TBD_ |
| L3 | 크기 혼재 + 예외 | 단독/협력 선택, 경로 충돌, 재협상·복구 | _TBD_ |

### 1.2 Isaac Lab 수정안 (휴머노이드·가상 물류) — 승인 전 확정 아님

| 레벨 | 시나리오 라벨 (README) | 검증 포인트 (라벨만) | 초기상태 / 성공 / 실패 / max_steps / 이벤트 |
| --- | --- | --- | --- |
| L1 | 단독 이송 | 단독 실행·완주 | _TBD_ |
| L2 | 협력 이송 | 파트너 선택, 역할, 출발 동기화 | _TBD_ |
| L3 | 장애물·고장·목표 변경 후 재협상 | 예외 전파·재계획·복구 | _TBD_ |

---

## 2. 레벨별 칸 (값 비움)

각 레벨은 위 1.1 또는 1.2를 고른 뒤 같은 칸을 채운다.

### L1 — 단독

| 칸 | 내용 |
| --- | --- |
| 목표 (한 줄) | _TBD_ |
| `initial_state` | _TBD_ |
| `success_criteria` | _TBD_ |
| `failure_criteria` | _TBD_ |
| `max_steps` | _TBD_ |
| `event_injection` | 보통 없음. 예외 시 _TBD_ |
| 필요 행동 스킬 (후보만) | 이동·잡기·전달·대기 등 — protocol 목록과 충돌 검사 |
| 통신 기대 | 단독이므로 peer 협상 최소. 무통신 baseline과 대비 가능 |

### L2 — 협력

| 칸 | 내용 |
| --- | --- |
| 목표 (한 줄) | _TBD_ |
| `initial_state` | _TBD_ |
| `success_criteria` | _TBD_ (동기화·공동 이송 포함 여부 명시 필요) |
| `failure_criteria` | _TBD_ |
| `max_steps` | _TBD_ |
| `event_injection` | _TBD_ |
| 필요 행동 스킬 (후보만) | 요청/응답·대기·공동 이송 등 — protocol |
| 통신 기대 | 파트너 요청·역할 합의·ack. 스키마는 공식안 A/B 흐름과 수정안 peer 필드를 **분리** |

### L3 — 예외·재협상

| 칸 | 내용 |
| --- | --- |
| 목표 (한 줄) | _TBD_ |
| `initial_state` | _TBD_ |
| `success_criteria` | _TBD_ (복구 후 완주도 성공인지 정의 필요) |
| `failure_criteria` | _TBD_ |
| `max_steps` | _TBD_ |
| `event_injection` | 장애물 / 고장 / 목표 변경 — 종류·시점·시드 규칙 _TBD_ |
| 필요 행동 스킬 (후보만) | reroute·재요청·재합의 등 — protocol |
| 통신 기대 | 이상 전파·재계획. `failure_reason`·재협상 turn을 eval과 맞춤 |

---

## 3. protocol / eval과의 맞춤 포인트 (2026-08-22 1차 대조)

하드 충돌 없음 (protocol `docs/protocol_scope.md`, eval `docs/eval_spec_draft.md` §4.2와 합의).

| 주제 | 공통으로 맞음 | 별칭 (병합은 B3, 지금은 병기만) |
| --- | --- | --- |
| 시계열 | `seed` / `episode_id` / `step` / `max_steps` | — |
| 실패·응답 | `failure_reason` / `ack` / `turn_index` | — |
| 토큰 | eval `token_in`/`token_out` = protocol envelope | — |
| 메시지 | — | docs `message_log_ref` → eval `message_raw`+`message_struct`; protocol `nl_text`↔`message_raw`, `structured_payload`↔`message_struct` |
| 행동·비용 | — | docs `action_log_ref` / `cost_log_ref` → eval `action` / turn·토큰 |
| 화자 | — | B3에서 `sender_id`로 고정 후보 (vs `agent_id`/`speaker_id`); `recipient_id` vs `listener_id` |

필드명을 바꿀 때는 세 문서와 decision_log에 같이 적는다. **스키마 병합·확정은 Phase B3 이후. 지금은 용어 맞춤만.**

---

## 4. 다음 단계

1. ~~protocol A4·eval A5 초안과 §3 표 대조~~ → 하드 충돌 없음, 별칭만 §3에 병기  
2. O1(공식안 vs 수정안) 승인 → §1에서 층 선택  
3. 선택 층의 L1–L3 `_TBD_` 채움 → decision_log 결정 항목 + ROADMAP B2  
4. B3에서 별칭을 단일 필드명으로 고정 (병합은 승인 후)
