# UGRP 통신 프로토콜 범위 초안 (Phase A4)

> **날짜**: 2026-08-22  
> **역할**: protocol **초안 범위** (확정 아님). Phase A4 산출물.  
> **읽는 순서 (canonical)**: [`README.md`](../README.md) → [`ROADMAP.md`](../ROADMAP.md) → [`docs/decision_log.md`](decision_log.md)  
> **규칙**: **승인 전 공식안(공식 제출계획)과 수정안(Isaac Lab)을 합치지 않는다.** 아래는 후보·범위만이며, 스키마·스킬 확정은 ROADMAP B3 / decision_log O3 승인 후다.  
> **금지**: 이 문서를 확정 사양처럼 구현하거나, 두 안을 하나의 스키마로 병합하거나, decision_log에 미승인 결정을 채워 넣지 않는다.

---

## 1. 설계 원칙 (초안)

| 원칙 | 설명 | 상태 |
| --- | --- | --- |
| 이중 기록 | 자연어 원문(`nl_text`)과 구조화 행동(`structured_action` / `structured_payload`)을 **동시에** 저장 | 후보 (B3에서 확정) |
| 파싱 실패 측정 | `parse_ok`(또는 동등 후보)로 원문→구조화 변환 실패를 측정 가능하게 둔다 | 후보 |
| 역할 구조 분리 | 공식안(역할 비대칭 A→B)과 수정안(완전 분산 peer)의 메시지 형태를 **별도 표**로 유지 | **필수** (승인 전 합치기 금지) |
| eval 용어만 맞춤 | turn·token 로깅은 eval 로거와 **용어만** 맞춘다. 본 문서에는 필드명 **후보**만 둔다 | 후보 |
| 플랫폼 의존 | 구체 skill 이름·인자는 로봇 플랫폼 선택(decision_log O1)에 종속. 인자 스키마는 TBD | 미확정 |

평가·재현과 맞출 공통 개념(이름만, 구현 아님): `seed`, `episode`, `step`, `message`, `action`, `cost`(tokens/turns), `failure_reason`.

---

## 2. 메시지 스키마 후보

> 아래 표의 모든 필드명은 **후보/미확정**이다. 타입·필수 여부·JSON 키는 B3 승인 전까지 고정하지 않는다.

### 2.0 공통 봉투(envelope) 후보 — 로깅·재현용

공식안·수정안 **모두**에 붙일 수 있는 공통 후보. 어느 안의 본문(`structured_payload`)과도 조합하되, **본문 스키마 자체를 하나로 합치는 것은 아님**.

| 필드 후보 | 용도 (초안) | 구분 |
| --- | --- | --- |
| `message_id` | 메시지 단위 식별 | 공통 후보 |
| `episode_id` | episode 식별 (eval `episode`와 용어 맞춤) | 공통 후보 |
| `step` | 환경 step (eval `step`와 맞춤) | 공통 후보 |
| `turn_index` | 통신 turn 순번 (cost·turns와 맞춤) | 공통 후보 |
| `sender_id` | 발신 에이전트 | 공통 후보 (수정안 본문에도 등장; 중복 시 위치만 B3에서 정리) |
| `recipient_id` / `recipient_ids` | 수신자 1명 또는 다수 | 공통 후보 |
| `nl_text` | 자연어 원문 | 공통 후보 |
| `structured_payload` | 구조화 본문 (안별로 내용 상이) | 공통 후보 |
| `parse_ok` | 원문→구조화 파싱 성공 여부 | 공통 후보 |
| `token_in` / `token_out` | 해당 turn 입력·출력 토큰 (cost) | 공통 후보 |
| `timestamp` | 송신·기록 시각 | 공통 후보 (수정안 peer 필드에도 등장) |
| `failure_reason` | 통신·파싱·실행 실패 사유 (있을 때) | 공통 후보 (수정안 peer 필드에도 등장; eval과 용어 맞춤) |

### 2.1 공식안 — 역할 비대칭 (A→B)

출처: README §6. **판단/지시 쪽(A)과 보고/실행 쪽(B)이 비대칭**인 흐름. 고정 A/B 역할을 전제로 한 **공식안 전용** 메시지 종류 후보.

| 메시지 종류 (후보) | 방향 | 의미 (초안) | 구분 |
| --- | --- | --- | --- |
| `SOLO_ASSIGN` | A → B | 단독 처리 지시와 물체·목표 정보 | **공식안-only** |
| `COLLAB_START` | A → B | 협력 시작, 파지 위치, 이동 순서 | **공식안-only** |
| `STATUS_REPORT` | B → A | 진행·완료·저항 감지 등 현재 상태 | **공식안-only** |
| `ALERT` | B → A | 미끄러짐·장애물·중량 초과 등 이상 | **공식안-only** |
| `REPLAN` | A → B | 이상 반영 수정 지시 | **공식안-only** |

공식안 `structured_payload`에 들어갈 **본문 필드**는 메시지 종류별로 상이할 수 있음. 종류별 세부 키(물체 ID, 목표, 파지점 등)는 **미확정·TBD** (O1·L1–L3 명세와 연동).

> 참고: README §5.2의 MasterPi/판단·실행 비대칭 초안은 **참고 자산**이며, 위 공식안 표와 자동 병합하지 않는다.

### 2.2 수정안 — 완전 분산 peer

출처: README §4·§6. **고정 판단자·실행자 역할 없음**. 각 에이전트가 상태·의도 중심 필드로 협상. 아래는 **수정안-only** peer 본문 필드 후보.

| 필드 후보 | 의미 (초안) | 구분 |
| --- | --- | --- |
| `sender_id` | 발신 peer | **수정안-only** 본문 (봉투와 중복 가능 → B3에서 위치 정리) |
| `object_id` | 대상 물체 | **수정안-only** |
| `observation` | 현재 관측·상태 요약 | **수정안-only** |
| `intent` | 의도 (단독/협력/대기/재계획 등 — 값 집합 미확정) | **수정안-only** |
| `proposed_action` | 제안하는 구조화 행동 (스킬 후보와 연결) | **수정안-only** |
| `required_partner` | 필요한 파트너 peer id(들) | **수정안-only** |
| `ack` | 동의·수신 확인 | **수정안-only** |
| `failure_reason` | 실패·이상 사유 | **수정안-only** 본문 (봉투·eval과 용어 공유) |
| `timestamp` | 해당 제안/보고 시각 | **수정안-only** 본문 (봉투와 중복 가능) |

수정안에는 공식안의 `SOLO_ASSIGN` / `COLLAB_START` / … 같은 **고정 A→B 메시지 종류 표를 두지 않는다** (역할 비대칭을 전제하지 않음).

### 2.3 구분 요약

| 구분 | 내용 |
| --- | --- |
| **공식안-only** | `SOLO_ASSIGN`, `COLLAB_START`, `STATUS_REPORT`, `ALERT`, `REPLAN` 및 A/B 방향 |
| **수정안-only** | peer 본문: `object_id`, `observation`, `intent`, `proposed_action`, `required_partner`, `ack` (+ 본문 위치의 `sender_id`/`failure_reason`/`timestamp`) |
| **공통 후보** | envelope: `message_id`, `episode_id`, `step`, `turn_index`, `recipient_id(s)`, `nl_text`, `structured_payload`, `parse_ok`, `token_in`, `token_out` 등 |

**하지 않음**: 위 두 본문 스키마를 하나의 “통합 메시지 타입 enum”으로 합치는 것 (승인·decision_log 기록 전).

---

## 3. 행동 스킬 후보 목록 (확정 아님)

고수준 후보만 나열한다. **구체 skill 이름·인자 스키마는 로봇 플랫폼 선택(O1)에 종속** → args는 비우거나 `TBD`.

### 3.1 고수준 후보 (양안에 참고 가능한 공통 목록 — 확정 아님)

| 스킬 후보 (고수준) | 비고 | args |
| --- | --- | --- |
| `navigate` / `approach` | 이동·접근 | TBD |
| `grasp` / `release` | 파지·해제 | TBD |
| `lift` / `place` / `handoff` | 들어올림·배치·전달 | TBD |
| `wait` / `hold` | 대기·유지 | TBD |
| `request_help` / `propose_role` / `accept` / `reject` / `ack` | 협력 협상·응답 | TBD |
| `report_status` / `report_failure` | 상태·실패 보고 | TBD |

### 3.2 수정안 문서에 나온 스킬 이름 (참고, 미확정)

README §4.1 수정안 경계에 적힌 이름. **공식안에 자동 적용하지 않음.** 위 고수준 후보와의 매핑은 B3에서 정리.

| 이름 (수정안 문서) | 대응 고수준 (초안 추정, 미확정) |
| --- | --- |
| `navigate_to` | navigate |
| `approach_object` | approach |
| `request_partner` | request_help / propose_role |
| `wait_for_partner` | wait |
| `start_transport` | lift / handoff 등 (플랫폼 종속) |
| `reroute` | navigate (+ 재계획 intent) |
| `deliver` | place |

### 3.3 공식안 쪽

공식안은 메시지 종류(`SOLO_ASSIGN` 등)가 지시·보고의 틀이고, 실행 스킬 집합은 **로봇팔 3대·블록 시나리오**에 맞게 따로 둘 수 있음. 구체 목록·args는 O1·L1–L3 명세 연동 전까지 **TBD**.

---

## 4. turn·토큰 로깅 필드 후보 (eval 동기화용)

> **이름 후보만**. 구현·스키마 확정 아님. eval 로거의 `cost`(tokens/turns) 및 `message`/`step`와 용어를 맞추기 위함.

| 필드 후보 | 용도 | 비고 |
| --- | --- | --- |
| `turn_index` | 통신 turn 순번 | envelope와 공유 가능 |
| `speaker_id` | 해당 turn 발화 주체 | `sender_id`와 동일시 여부 미확정 |
| `listener_id` / `listener_ids` | 수신 측 | `recipient_id(s)`와 동일시 여부 미확정 |
| `tokens_prompt` | 프롬프트 토큰 | `token_in`과 이름 통일은 eval과 협의 |
| `tokens_completion` | 완선 토큰 | `token_out`과 이름 통일은 eval과 협의 |
| `model_id` | 사용 모델 (선택) | optional |
| `latency_ms` | 호출 지연 (선택) | optional |
| `message_id` / `step` | 메시지·환경 step 연결 | eval `message`, `step`와 링크 |

`seed`, `episode`, `action`, `failure_reason`, `cost`는 eval 쪽 초안과 **용어만** 맞추고, 본 문서에서 로거를 구현하지 않는다.

---

## 5. 하지 않을 것

- 사양(Phase B) 확정 전 **본구현** (코드·로거 본체)
- 공식안 + 수정안 메시지 스키마를 **하나로 합치기** (승인·decision_log 기록 전)
- 브라우저 sandbox(`decentral-cobot`)의 **keyword router**를 LLM 자연어 peer 통신으로 서술하기
- decision_log **결정** 칸을 이 초안으로 채우거나, 미승인 항목을 확정처럼 쓰기
- 생성 초안·예상 수치를 실험 결과로 기록하기

---

## 6. 다음

1. **@ugrp-docs**: L1–L3 명세 골격(A3) 작성 후, 본 문서와 **필드명·용어 충돌** 점검  
2. **@ugrp-eval**: `seed` / `episode` / `step` · 메시지 · 행동 · 비용(tokens/turns) 용어를 본 후보와 맞춤 (A5 초안)  
3. **확정**: decision_log 승인 후 ROADMAP **B3**에서 스키마·스킬 승격. 그 전까지 본 파일은 **범위 초안**으로만 유지  

---

## 부록. 문서 위치

- 본 파일: `docs/protocol_scope.md` (ROADMAP A4 “메시지 스키마·행동 스킬 범위”)  
- 기존 `docs/`에 protocol 전용 파일이 없어 이 이름을 사용함  
- 확정 사양 파일명(예: `protocol_spec.md`)은 B3 승인 시 별도 결정
