# 구역 대화 연구: 프롬프트와 통신 프로토콜 (패키지 C)

2026-09-25/26 사용자 결정에 따른 연구 질문은 **로봇 사이의 한국어 대화가 다중 로봇 작업 효율에 영향을 주는가**다. 이 문서는 그 연구의 **패키지 C**, 즉 조건별 한국어 프롬프트 골격과 메시지 전송·검증 규칙만 다룬다. 실행기·비용 시계·자기 카메라 인식·평가는 다른 패키지의 범위다.

- 구현: [`harness/zone_study_protocol.py`](../harness/zone_study_protocol.py), [`harness/zone_study_prompts_ko.py`](../harness/zone_study_prompts_ko.py)
- 테스트: [`tests/test_zone_study_protocol.py`](../tests/test_zone_study_protocol.py) — 오프라인 67건. SIM 실행·모델 호출 없음
- 버전 문자열: `ugrp.zone_study_protocol.v1`, `ugrp.zone_study_prompts_ko.v1`
- 설계 참고: Codex 통합 설계(`docs/design/2026-09-25-zone-dialogue-study-design-codex.md`, PR #180)와 한국어 대화 오프라인 파일럿([`experiments/2026-09-25-zone-dialogue-ko-pilot`](../experiments/2026-09-25-zone-dialogue-ko-pilot/README.md), PR #172). 설계안이 `r1` 고정 지휘자·별도 commander·교사 실행기를 권고한 부분은 사용자 결정이 대체한다.

**이 패키지는 자동 검사만 통과한 상태다.** 조건 사이의 효율 차이, 한국어 대화의 효과, 실제 배송 성공은 어느 것도 측정하지 않았다.

## 1. 조건 registry

`SPECS`([protocol.py:113](../harness/zone_study_protocol.py)). 주 조건 네 개는 **통신 채널만 다르고** 나머지는 같다.

| 조건 | topology | encoding | 지휘 | 로봇 LLM | 주 조건 |
|---|---|---|---|---|---|
| `no_comm` | `none` | `none` | 없음 | 있음 | 예 |
| `peer_ko` | `mesh` | `ko_free` | 없음 | 있음 | 예 |
| `leader_ko` | `star` | `ko_free` | 로봇 1대 겸임, seed마다 순환 | 있음 | 예 |
| `structured` | `mesh` | `structured` | 없음 | 있음 | 예 |
| `reference_R` | `commander` | `none` | 별도 commander | **없음** | 아니오(참고 상한) |

- `no_comm`은 고수준 메시지를 **0건 보내고 0건 받는다.** 채널이 없으므로 모든 `send`가 `channel_closed`로 거절된다.
- `leader_ko`는 허브-스포크다. `leader↔follower` 간선만 있고 **follower끼리 직접 전달은 없다.** follower는 한국어로 보고·질문·거절·양보를 leader에게 보낼 수 있다.
- `structured`는 `peer_ko`와 같은 mesh·수신자 선택·발화 예산을 쓰고 의미만 고정 schema로 전달한다. 자유 문장은 거절된다.
- `reference_R`은 전지적 commander가 세 로봇의 wrist RGB를 모두 받고 로봇에는 LLM이 없다. 지시는 메시지가 아니라 `action`(`kind` = `order`)이며, 메시지 채널은 없다. **주 비교에 합산하지 않는다.**

### leader 순환

`leader_for_seed(seed)`([protocol.py:129](../harness/zone_study_protocol.py))는 `ROBOTS[seed % 3]`이다. seed 0·1·2 → `r1`·`r2`·`r3`, seed 11·12·13 → `r3`·`r1`·`r2`. `allowed_edges`([protocol.py:160](../harness/zone_study_protocol.py))가 그 leader를 따라 간선 4개를 만들고, `role_of`([protocol.py:172](../harness/zone_study_protocol.py))가 프롬프트 역할(`peer`/`leader`/`follower`/`commander`)을 정한다. seed 없이 `leader_ko`를 쓰면 거절한다(고정 `r1`로 조용히 되돌아가지 않는다).

## 2. 매 호출 입력과 금지 입력

`build_request`([prompts_ko.py:339](../harness/zone_study_prompts_ko.py))는 호출마다 다음을 넣는다.

| 입력 | user JSON 키 | 비고 |
|---|---|---|
| 버전 있는 정적 지도 | `static_map`, `static_map_sha256` | 벽·문·복도·구역 `A`/`B`/`C`·pickup bay/slot·AprilTag landmark |
| 시나리오 설정에서 만든 주문서 | `order_sheet`, `order_sheet_sha256` | 물건 kind·개수·필요 로봇 수·역할·목적 구역·대략적 최초 위치 |
| 자기 wrist RGB 1장 | 이미지 `CURRENT OWN WRIST RGB` | 어안 원본. 조건 무관하게 1장 |
| 자기 발행 명령 이력 | `own_commands` | 최근 16건 |
| 자기 belief | `own_belief` | 허용 입력만으로 만든 추정 |
| 조건의 대화 채널 | `dialogue_window` | 채널이 있는 조건에만 존재 |

금지: 공용 TOP 영상과 TOP에서 만든 좌표·개수·완료 판정, 다른 로봇의 영상·명령 기록·작업 상태, 시뮬레이터 상태, 정답 좌표, 성공·완료 통보, sim 전용 `nav_cam`. `public_map`([prompts_ko.py:108](../harness/zone_study_prompts_ko.py))이 `top_cameras`를 떨어뜨리고, `build_request`는 `top`·`nav_cam`·`cctv`가 들어간 이미지 라벨을 거절한다. `validate_order_sheet`([prompts_ko.py:119](../harness/zone_study_prompts_ko.py))는 `status`·`xyz_m`·`held_by` 같은 실시간 상태 키가 있는 주문서를 거절한다.

`reference_R` commander만 `WRIST RGB r1`/`r2`/`r3` 세 장을 받고, 자기 카메라·belief 대신 `issued_orders`를 받는다.

## 3. 프롬프트 골격

`system_prompt(condition, rid, seed=...)`([prompts_ko.py:292](../harness/zone_study_prompts_ko.py)). 조건마다 **통신 블록과 `messages` 설명만 다르고** 과제·입력 경계·언어 규칙은 같다. 파일럿 권고 1(같은 한국어 과제 설명 위에서 메시지 형식만 바꿔야 언어 효과와 통신 효과가 섞이지 않는다)을 따른다. 프롬프트는 조건당 1,270–2,030자이며 한글 비율은 0.956–0.981이다(literal 토큰 제외).

조건별 통신 블록의 요지:

| 조건/역할 | 지시 |
|---|---|
| `no_comm` | 채널이 없다. `messages`는 빈 배열이어야 하고 `decision_sources`에 `message`를 쓸 수 없다 |
| `peer_ko` | 필요하면 한국어로 관측·의도·질문·요청·양보·정정을 전달한다. `recipients` 명시 |
| `leader_ko` leader | 배정을 한국어로 지시하고 보고를 확인한다. follower끼리 말할 수 없으므로 필요한 정보는 직접 전달한다. `action`은 자기 행동만 |
| `leader_ko` follower | leader의 한국어 지시를 해석한다. 질문·거절·보고는 leader에게만. 시각 근거와 어긋나는 지시는 되묻거나 거절할 수 있다 |
| `structured` | 자유 문장 없이 고정 필드만. `text`·`reason`·`note` 같은 필드를 넣으면 거절된다 |
| `reference_R` commander | 메시지 채널이 없다. 지시는 `action`(`kind` = `order`)으로 내린다. 참고 상한 조건이다 |

`r1`/`r2`/`r3`, 구역 문자 `A`/`B`/`C`, `order_id`·`item_id`·passage ID, 물건 `kind`, `role` 이름, JSON 키와 값(`null`, `true`, `false`), enum 값은 번역하지 않고 그대로 쓴다. 자유 메시지 본문만 한국어로 쓴다.

## 4. 출력 schema: action과 messages 분리

최상위 키는 `request_id`, `action`, `decision_sources`, `messages`뿐이다([protocol.py:72](../harness/zone_study_protocol.py)).

```json
{"request_id": "...",
 "action": {"kind": "claim", "order_id": "order-1", "role": "end_neg", "destination_zone": "A"},
 "decision_sources": ["static_map", "order_sheet", "own_rgb"],
 "messages": [{"recipients": ["r2"], "text": "order-1은 제가 end_neg 역할로 맡겠습니다.", "reply_to": null}]}
```

- 로봇의 `action.kind`는 `claim`·`continue`·`release`·`wait`다. 로봇은 **자기 작업만** 선언한다.
- `order`는 `reference_R` commander만 쓴다: `{"kind": "order", "assignments": {"r1": …, "r2": …, "r3": …}}`. 로봇이 `order`를 내면 거절된다.
- `leader_ko`의 leader도 `action`은 자기 행동이며, 지시는 한국어 `messages`로 나간다. 그래서 지휘가 자연어 채널을 통과한다.
- `decision_sources`는 `static_map`·`order_sheet`·`own_rgb`·`own_commands`·`own_belief`·`message` 중 실제로 쓴 것만이다. 채널이 없는 조건에서 `message`를 쓰면 거절된다.
- 검증은 **고치지 않는다.** `validate_reply`([protocol.py:511](../harness/zone_study_protocol.py))는 `ProtocolError`(= `ValueError`)를 던지고 값을 보정하지 않는다. 코드 울타리(` ```json `)로 감싼 응답은 허용한다.

### 정형 메시지(조건 4)

필드는 정확히 `act`, `item`, `zone`, `role`, `passage`, `location_ref`, `state`, `confidence`, `observed_at_sim_s`, `reply_to`다.

```text
act:        propose | request | accept | reject | inform | correct | yield | cancel
state:      unknown | suspected | clear | blocked | present | absent | held | placed
confidence: low | medium | high
```

`item`·`role`·`passage`·`location_ref`는 공개 지도·주문서에 있는 ID만 쓴다. `text`·`reason`·`note`·`comment`·`other` 같은 자유 문자열 필드가 있으면 `free_text_not_allowed`로 거절한다.

## 5. 전송 규칙

`Transport`([protocol.py:258](../harness/zone_study_protocol.py))는 **중계만** 한다.

- **수신자 명시:** `recipients`가 빈 목록이면 거절한다. 지정되지 않은 로봇은 그 발화를 받지 못한다.
- **채널 격리:** 조건의 간선 집합에 없는 전달은 거절한다. `leader_ko`에서 follower끼리 보낸 발화는 `no_follower_to_follower`이며, 수신자가 섞여 있어도 **부분 전달하지 않고 전체를 거절**한다.
- **발화 예산:** 창당 6발화, 로봇당 2발화(`MAX_WINDOW_UTTERANCES`, `MAX_ROBOT_UTTERANCES`). 파일럿에서 실제로 새 정보가 담긴 발화는 로봇당 1회였다. `open_window`로 창을 열면 예산이 초기화된다.
- **길이:** 프롬프트는 240자 이내를 요구하고 600자를 넘으면 `text_too_long`으로 거절한다(파일럿 권고 2).
- **전달 지연:** 0.1 SIM초 뒤에 inbox에 들어간다. `inbox(rid, now_sim_s=…)`는 그 시각까지 전달된 것만 준다.
- **reply_to:** 그 로봇이 **실제로 받은** message_id만 참조할 수 있다. 받지 못한 id는 `unknown_reply_to`다.
- **거절 목록:** `REJECTIONS`([protocol.py:85](../harness/zone_study_protocol.py)). 모두 평가 로그(`log`, `rejections`)에 남고 다시 쓰이지 않는다.

### 메시지는 호스트 결정을 바꾸지 않는다

`Transport`에는 claim·예약·작업 상태가 없고, 전송 계층의 어떤 입구도 그런 인자를 받지 않는다(`FORBIDDEN_TRANSPORT_PARAMS`와 `transport_parameter_names()`로 검사). 전달되는 기록은 `INBOX_FIELDS`(`message_id`, `from_robot`, `recipients`, `sent_at_sim_s`, `delivered_at_sim_s`, `reply_to`, `text` 또는 `message`)로 제한된다. 즉 발화는 다른 로봇의 행동·배정·예약을 직접 바꾸지 못하고, 수신자가 자기 관측으로 스스로 판단한다. `relay`([protocol.py:582](../harness/zone_study_protocol.py))는 검증된 응답의 메시지만 순서대로 전송하고 `action`은 만지지 않는다.

## 6. 언어 이탈: 표시만 하고 고치지 않는다

`language_report`([protocol.py:188](../harness/zone_study_protocol.py))는 기존 평가 전용 지표(`harness/zone_dialogue_metrics.py`, 수정하지 않음)로 한글 비율·잔여 영문 단어·literal ID 오류를 재고 플래그를 만든다.

| 플래그 | 뜻 |
|---|---|
| `non_korean` | literal 제외 한글 비율 < 0.9 |
| `code_switch` | literal이 아닌 영문 단어가 남음 |
| `literal_id_issue` | `로봇 2`처럼 literal ID를 번역·손상 |
| `silence` | 빈 발화 |

플래그는 **평가 로그에만** 남는다. 발화는 바이트 그대로 전달되고, 수신자의 inbox 기록에는 플래그가 들어가지 않으며, 언어 이탈만으로 응답을 거절하지 않는다.

## 7. 검증 범위

`.venv-sim-worker-mac/bin/python -m pytest tests/test_zone_study_protocol.py` — 67건 통과(오프라인, 모델 호출 없음). 확인한 것:

- 채널 격리: `no_comm`·`reference_R` 송수신 0, mesh 수신자 한정, follower끼리 전달 0, 정형 채널의 자유 문자열 거절, 자기 자신·미지 수신자 거절.
- 발화 예산: 로봇당 2회 뒤 `robot_cap`, 팀 6회 뒤 `window_cap`, 창마다 초기화.
- 전달: 0.1 SIM초 지연, 지정되지 않은 로봇 미수신, inbox 기록 필드 제한.
- 부작용 없음: 호스트 claim/예약/행동 사전이 그대로, 전송 계층 인자 이름 감사.
- 응답 검증: 정상 응답, 코드 울타리로 감싼 응답, 깨진 JSON, 필드 과부족, stale `request_id`, 미지 `order_id`/구역/역할, 잘못된 `action.kind`, `decision_sources` 위반, 조건별 침묵 강제, commander 지시.
- 언어: 영어 발화가 거절되지 않고 플래그만 붙으며 본문이 보존됨, 코드 전환·ID 손상 구분.
- leader 순환: seed→leader 대응과 간선 집합, 프롬프트 역할.
- 프롬프트: 모든 조건이 한국어(비율 > 0.9), 매 호출 지도·주문서·자기 입력 포함, TOP 이미지·`top_cameras` 부재, 조건 블록 문구, 공통 과제·언어 블록 동일.

측정하지 않은 것: 작업 효율·완료율·makespan, 실제 모델의 지시 이해와 한국어 준수, 실제 배송 성공, SIM 비용 시계와의 결합, 물리 실행.

## 8. 남은 연결과 정렬

- **패키지 A 정렬(필수):** 연구 계약 모듈 `harness/zone_study_contract.py`(브랜치 `kiro/zone-study-contract`)는 아직 없다. `StudyInputs`와 `from_contract`([prompts_ko.py:56, 90](../harness/zone_study_prompts_ko.py))는 **최소 로컬 어댑터**이며, A가 병합되면 입력 필드·해시·주문서 schema·공개 지도 투영을 A의 계약으로 바꿔야 한다(`ADAPTER_NOTE`).
- pickup bay/slot(`P1`, `P1-2`)은 현재 지도 JSON에 없다. 새 지도 버전(패키지 E)이 정의해야 하고, 그때 `location_refs`와 주문서 검증을 다시 맞춘다.
- 발화·추론의 SIM 시간 비용과 전달 시각 예약은 패키지 D(`zone_sim_cost`, `zone_event_scheduler`)가 소유한다. 여기서는 전달 지연 상수만 둔다.
- 대화 창을 여는 사건 훅(자기 영상 변화, 집결 대기 초과, 실제 메시지 수신)은 실행기 통합(패키지 G) 몫이다. 동료 작업 종료가 전원을 깨우지 않아야 한다.
- 파일럿에서 미해결로 남은 항목: 창 안 두 번째 턴에 이미지를 다시 보낼지, 호스트가 만드는 user JSON의 영어 문자열을 어떻게 다룰지. 조건 공통으로 정해야 한다.
