# UGRP 비교군·메트릭·로거 스펙 (초안)

> 최종 갱신: 2026-08-22  
> 담당: ugrp-eval  
> ROADMAP: **A5** (스펙 초안만, **구현 금지**) → 승격은 **B4** (사양 승인 후)  
> 읽는 순서: `README.md` → `ROADMAP.md` → `docs/decision_log.md` → 이 파일  
> 규칙: 공식 제출계획과 Isaac Lab 수정안을 **승인 없이 합치지 않는다**. 아래는 후보·칸만. 확정 수치·코드는 B 이후.

---

## 0. 상태

| 항목 | 상태 |
| --- | --- |
| 비교군 목록 | 후보만 (README §7 / 수정안 기준). 확정·병합 안 함 |
| 메트릭 | README §3.3 계획 지표를 초안 필드로 옮김. 정의·수식 비움 |
| 로거 스키마 | 필드 후보만. docs·protocol과 **1차 대조 완료** (하드 충돌 없음, 별칭만) |
| 본구현 | **금지** (ROADMAP §3). Phase D(D1–D4)는 B 완료 후 |

관련 미해결: decision_log **O4**. 필드 **병합은 B3**, 스펙 승격은 **B4**. 지금은 용어 맞춤만.

---

## 1. 비교군 (후보, 분리 표기)

공통 가설(유지): 중앙 계획자 없음 / 보조 계획자 없음 / 동등 에이전트 — **단, 선택적 비교군 4번은 예외 조건으로만 둔다.**

### 1.1 수정안 문서가 제안한 비교군 (README §7)

| ID | 이름 | 통신 | 비고 |
| --- | --- | --- | --- |
| C-rule | 규칙 기반 분산 시스템 | 규칙/신호 (NL 아님) | Phase D1 |
| C-llm-nocomm | 통신 없는 LLM | 없음 | Phase D2 |
| C-llm-nl | 자연어 peer 통신 LLM | NL + 구조화 행동 | 핵심 가설, Phase D3 |
| C-central | 중앙 계획자 시스템 | (중앙→에이전트) | **선택적**. 핵심 가설과 대비용 |

### 1.2 공식안 쪽과의 관계 (합치지 않음)

| 층 | 환경·로봇 | 이 초안에서의 취급 |
| --- | --- | --- |
| 공식 제출계획 | 로봇팔 3대, 블록 분류·협력 | 비교군 **이름·가설**은 참고 가능. 환경·태스크 값은 L1–L3 명세(공식)와 묶일 때만 채움 |
| Isaac Lab 수정안 | 휴머노이드 2–3대, 가상 물류 | 위 §1.1 표의 출처. **승인 전엔 확정 실험 설계로 쓰지 않음** |
| 참고 자산 | decentral-cobot, 생성 초안 | 비교군·수치 출처로 쓰지 않음 |

> B1(환경·로봇 수·에이전트 구조) 승인 전엔 어떤 층에 C-* 를 붙일지 확정하지 않는다. 표는 후보 ID만 고정.

### 1.3 공정 비교를 위해 고정할 통제 변수 (칸만)

| 변수 | 초안 칸 | 확정 시점 |
| --- | --- | --- |
| 환경 seed | _(비움)_ | B2 / 실험 설계 승인 |
| 물체 배치 | _(비움)_ | L1–L3 명세 |
| 장애물·고장 이벤트 | _(비움)_ | L3 이벤트 주입 규칙 |
| 모델 호출 예산 | _(비움)_ | B4 |
| 최대 step 수 | _(비움)_ | L1–L3 max step |
| 에피소드 수 / 시드 목록 | _(비움)_ | B4 |

목적: **통신 방식 차이**와 **환경 난이도 차이**를 분리 (README §7).

---

## 2. 메트릭 (초안)

README §3.3 「계획된 평가 지표」를 필드로만 옮긴다. 집계 단위·수식·성공 판정은 L1–L3·환경 심판 확정 후.

| ID | 메트릭 | 집계 단위 (후보) | 비고 |
| --- | --- | --- | --- |
| M-success | 작업 성공률 | episode | 환경 심판 성공 판정에 의존 |
| M-role | 역할 분담·파트너 선택 성공률 | episode / 협력 시도 | L2·L3 중심 |
| M-time | 완료 시간 또는 완료 step 수 | episode | wall-clock vs step — **미정** |
| M-recover | 예외 상황 복구율 | 예외 발생 episode | L3 |
| M-deadlock | 교착 상태 발생률 | episode | 정의 칸 비움 |
| M-halluc | 환각·실행 불가 행동 발생률 | step 또는 action | 파싱 실패·심판 거절과 연결 |
| M-turn | 통신 turn 수 | episode | protocol 용어와 맞출 것 |
| M-token | LLM 토큰 비용 | episode / 에이전트 | protocol·로거와 동일 필드명 |
| M-latency | 자원 효율·동시 추론 지연 | episode / step | 측정점 미정 |

### 2.1 분석 시 분리할 축 (E1 예고, 확정 아님)

- 축 A: 비교군 (C-*)
- 축 B: 난이도 (L1 / L2 / L3)
- 축 C: (선택) 모델·예산

통신 방식 효과는 A×B에서 읽는다. 시드·이벤트 미고정 시 해석 금지.

---

## 3. 실험 로거 (필드 후보만)

Phase D4 목표: `seed`, `episode`, `step`, 메시지, 행동, 비용, 실패 원인을 **재현 가능**하게 저장.  
아래는 **스키마 초안 칸**. 파일 포맷·코드는 B4 승인 전 작성하지 않는다.

### 3.1 런·에피소드 메타

| 필드 (후보) | 설명 | 비움/의존 |
| --- | --- | --- |
| `run_id` | 실험 실행 ID | |
| `spec_layer` | `official` \| `amendment` \| `unspecified` | B1 전엔 `unspecified`만 |
| `condition_id` | C-rule / C-llm-nocomm / C-llm-nl / C-central | |
| `task_level` | L1 \| L2 \| L3 | docs L1–L3 |
| `seed` | 환경 시드 | |
| `episode_id` | 에피소드 번호 | |
| `max_steps` | 상한 | L 명세 |
| `model_budget` | 호출/토큰 예산 | B4 |
| `git_commit` / `spec_version` | 재현용 버전 핀 | 사양 확정 후 |

### 3.2 step 레코드 (후보)

| 필드 (후보) | 설명 | protocol 정렬 |
| --- | --- | --- |
| `step` | 정수 step | |
| `agent_id` / `sender_id` | 행위자 | protocol 필드명 확정 시 통일 |
| `observation_ref` | 관측 요약 또는 해시 | |
| `message_raw` | 자연어 원문 (있으면) | NL + 구조화 동시 저장 |
| `message_struct` | 구조화 의도/행동 | `intent` / `proposed_action` 등 **후보** |
| `action` | 환경에 제출된 행동 스킬 | protocol 스킬 목록 |
| `action_accepted` | 심판 수락 여부 | 환경 심판 |
| `failure_reason` | 거절·실패 코드 | protocol `failure_reason`과 용어 맞춤 |
| `ack` | 상대 ack (해당 시) | protocol |
| `turn_index` | 통신 turn 인덱스 | M-turn |
| `token_in` / `token_out` | 토큰 | M-token |
| `latency_ms` | 추론·라운드 지연 | M-latency |

### 3.3 에피소드 요약 (후보)

| 필드 (후보) | 연결 메트릭 |
| --- | --- |
| `success` | M-success |
| `steps_used` | M-time |
| `n_turns` | M-turn |
| `tokens_total` | M-token |
| `n_deadlock` | M-deadlock |
| `n_invalid_action` | M-halluc |
| `recovery_ok` | M-recover |
| `role_partner_ok` | M-role |
| `terminal_reason` | 성공 / max_step / 교착 / 중단 등 (코드표 비움) |

### 3.4 하지 않을 것 (로거)

- sandbox·생성 초안의 **예상 수치**를 로그 값으로 넣기
- 사양 확정 전 로거 **코드·DB·대시보드** 구현
- 공식안/수정안 필드를 한 스키마로 **암묵 병합** (필요 시 `spec_layer`로만 구분)

---

## 4. docs / protocol과의 맞춤 (1차 대조, 2026-08-22)

대조 대상: `docs/l1_l2_l3_task_spec.md`, `docs/protocol_scope.md`, 본 초안. **스키마 병합 안 함.**

### 4.1 일치 (그대로 유지)

`seed` / `episode_id` / `step` / `max_steps` / `failure_reason` / `turn_index` / `ack`  
`token_in` / `token_out` = protocol envelope (M-token과 동일 계열)

### 4.2 별칭만 (B3에서 하나로 고정)

| docs / protocol | eval (현재) | 비고 |
| --- | --- | --- |
| docs `message_log_ref` / `action_log_ref` / `cost_log_ref` | `message_raw` / `message_struct` / `action` / `turn_index`·토큰 | 참조 vs 인라인 필드 — B3에서 층위 결정 |
| protocol `nl_text` | `message_raw` | 동의어 |
| protocol `structured_payload` | `message_struct` | 동의어 |
| `sender_id` vs `agent_id` / `speaker_id` | §3.2에 병기 | B3에서 `sender_id`로 통일 후보 |
| `recipient_id` vs `listener_id` | (eval 미기재) | B3에서 `recipient_id`로 통일 후보 |

### 4.3 체크리스트

- [x] 하드 필드명 충돌 없음 (docs·protocol 1차 보고와 일치)
- [x] turn·token 이름이 M-turn / M-token·protocol envelope와 맞음
- [ ] 성공/실패/`terminal_reason` ↔ L1–L3 성공·실패 칸 — 값 채울 때 재확인
- [ ] 행동 스킬 enum ↔ protocol 스킬 목록 — B3
- [x] 공식안 A/B 메시지와 수정안 peer 필드를 한 테이블로 합치지 않음

---

## 5. 다음 행동

1. **지금 (A5)**: 초안 + 1차 별칭표 유지. 구현·스키마 병합 없음.
2. B3에서 §4.2 별칭을 단일 필드명으로 고정 → B4에서 본 문서 승격 (decision_log 기록).
3. 그 다음 Phase D: baseline·로거 본구현.

---

## 6. 세션 메모 (eval)

- 2026-08-22: A5 초안 최초 작성. README §3.3·§7·§9와 ROADMAP A5/B4/D*만 반영. 결정 본문·수치 없음.
- 2026-08-22: docs·protocol 1차 대조 반영. 하드 충돌 없음, 별칭만 §4.2에 기록. 병합은 B3.
