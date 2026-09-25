# 한국어 대화 연구의 계약과 로봇 입력 (패키지 A)

기준: 2026-09-26, 사용자 결정 2026-09-25/26. 이 문서는 **계약과 입력 계층**만 다룬다. 실제 물리 실행·프롬프트 문구·SIM 비용 계산·평가 지표는 각각 다른 패키지에서 구현·검증하며, 이 문서의 통과 사실을 그 결과로 쓰지 않는다.

- 연구 질문: **로봇 사이의 한국어 대화가 다중 로봇 작업 효율을 바꾸는가.**
- 구현: [`harness/zone_study_contract.py`](../harness/zone_study_contract.py), [`harness/zone_study_inputs.py`](../harness/zone_study_inputs.py), [`harness/zone_map_schematic.py`](../harness/zone_map_schematic.py)
- 테스트: `tests/test_zone_study_contract.py`, `tests/test_zone_study_inputs.py`
- 설계 배경: Codex 통합 설계(2026-09-25). 그 문서의 `r1` 고정 지휘, 별도 commander 주 조건, 교사 실행기 전제는 사용자 결정으로 **대체**했다.

## 1. 조건 registry

조건 사이에 다른 것은 **통신 채널뿐**이다. 나머지 입력은 같은 바이트를 쓴다(`tests/test_zone_study_inputs.py::test_only_the_channel_differs_between_the_conditions`).

| 조건 | 한국어 이름 | topology | encoding | 주 조건 | 추가 입력 키 |
|---|---|---|---|---|---|
| `no_comm` | 무통신 | `none` | `none` | 예 | 없음(`inbox` 금지) |
| `peer_ko` | 자유 한국어 동료 대화 | `mesh` | `free_ko` | 예 | `inbox` |
| `leader_ko` | 한국어 지휘 겸임 | `star` | `free_ko` | 예 | `inbox`, `leader_id`, `role` |
| `structured` | 정형 메시지 대조 | `mesh` | `schema` | 예 | `inbox` |
| `reference_R` | 전지적 지휘 참조 상한 | `commander_downlink` | `schema` | **아니오** | `team_rgb_refs`, `issued_orders` |

- `leader_ko`의 지휘자는 **로봇 한 대가 겸임**하고 seed로 순환한다: `robots[seed % 3]` → seed 11=r3, 12=r1, 13=r2. 채널은 **허브-스포크**로 `leader↔follower`만 허용하고 `follower↔follower`는 거부한다. follower는 한국어로 보고·질문·거절할 수 있고 자기 행동은 자기가 정한다.
- `reference_R`은 주 조건이 아니다. 지휘자만 호출을 받고(세 로봇의 손목 RGB), 로봇에는 LLM이 없다. 로봇용 payload를 만들려 하면 거부된다. 주 조건과 합산하거나 수학적 상한으로 표현하지 않는다.
- `condition_manifest(condition, seed)`와 `registry_sha256()`으로 실행 번들에 조건·간선·allowlist를 코드 SHA와 함께 기록한다([실행 버전 관리](execution_versioning.md)).

## 2. 공개/비공개 경계

로봇이 **매 호출** 받는 것(`BASE_ALLOWLIST`):

| 키 | 내용 |
|---|---|
| `static_map` | 정적 지도 공개 투영 + 도식 PNG 참조(§3) |
| `order_sheet` | 시나리오 설정에서 만든 불변 주문서(§4) |
| `own_rgb_refs` | **자기** 손목 fisheye RGB 참조(`own-<robot>-NNNN`) |
| `own_command_history` | **자기** 발행 명령과 자기 처리 상태(`command_issued`, `queue_empty`, `hold_requested`, `local_timeout`, `command_rejected`) |
| `self_belief` | 허용 입력만으로 만든 자기 belief(기본 `region: unknown`) |
| `channel` | 자기 조건의 송수신 규칙(조건 규칙이며 실시간 상태가 아니다) |
| `inbox` | 조건이 실제로 전달한 메시지(무통신은 없음) |
| `request_id`, `robot_id`, `condition`, `sim_time_s` | 호출 식별과 SIM 시각 |

`validate_robot_payload()`가 거부하는 것:

- **TOP 카메라와 그 파생물**(`top_*`, `cctv*`, `nav_cam*`). 공개 지도 투영에서 `top_cameras`를 아예 제외한다. sim 전용 `nav_cam` 참조도 거부한다.
- 정답 pose·관절·접촉·simulator 식별자(`pose`, `*_pose`, `qpos`, `body_id`, `contacts`, `weld`, `referee` …).
- 교사 영수증·완료/성공 판정(`teacher*`, `receipt*`, `grasp_success`, `placed`, `completion`, `success`, `done` …).
- 호스트가 아는 동료·전역 상태(`peer_*`, `team_board`, `zone_counts`, `global_progress`, `busy` …).
- 숨은 사건·평가 지표(`hidden_events`, `event_schedule`, `eval`, `score`, `makespan`, `metrics` …).
- 조건 allowlist 밖의 키, 다른 로봇의 카메라 참조, **비ASCII 키**.
- `static_map.public_map_sha256` 불일치, `leader_id`가 seed 순환과 다른 경우, 수신자가 자기가 아닌 `inbox` 항목, 조건과 다른 `encoding`, 허용되지 않은 간선의 발신자.

검사는 중첩된 위치까지 재귀로 수행한다(`forbidden_key_hits`, `non_ascii_keys`). 위반 목록만 보려면 `payload_violations()`를 쓴다.

**리터럴 보존:** 한국어 프롬프트에서 `r1`/`r2`/`r3`, 주문·물건 ID, 구역 문자 `A`/`B`/`C`, 통로 ID, enum, JSON 키는 번역하지 않는다. 그래서 키는 ASCII만 허용한다. 자유 메시지는 한국어가 하나도 없으면 **거부**하고, 그 밖의 라틴 단어는 `free_text_report(text, literals=...)`가 `latin_words`로 **기록만** 한다(지표).

## 3. 정적 지도 직렬화와 도식

`map_bundle(map_id, landmark_detail=...)`는 `maps/zones/<map_id>.json`을 **읽기 전용**으로 읽어 다음을 만든다.

| 항목 | 설명 |
|---|---|
| `map_file_sha256` | 지도 파일 바이트 해시 |
| `public_map` / `public_map_sha256` | 프롬프트용 compact JSON 투영과 그 해시 |
| `schematic.png_sha256`, `pixels_sha256`, `renderer` | 도식 PNG 해시·픽셀 해시·PIL 버전 |
| `base_map` | `*_tags_v1` 변형의 원본 지도 ID·해시(있을 때) |

공개 투영에 들어가는 것: 경계, 벽(둘레/내부 구분), 문·복도(`width_m`, `lanes`, `connects`), 정적 지형, 구역 `A`/`B`/`C`와 착지 슬롯, pickup 영역과 **파생 coarse bay/slot**, 지도가 칠한 물건 종류, 접근 관례, AprilTag 랜드마크(`full`/`summary`/`none`).

- **pickup bay/slot은 지도 파일에서 결정론적으로 파생**한다: pickup 영역을 서→동 `PICKUP_BAY_COLUMNS`(2)개 bay(`P1`, `P2`)로 나누고 각 bay를 남→북 `PICKUP_BAY_ROWS`(3)개 슬롯(`P1-1`…`P2-3`)으로 나눈다. 현재 지도에는 저작된 pickup 슬롯이 없어서 파생 규칙을 쓴다. 나중에 지도에 슬롯이 저작되면 파생을 그 정의로 **새 버전으로** 교체한다(지도 파일은 이 패키지에서 수정하지 않았다).
- 도식은 **지도 파일에서 그린다.** MuJoCo 장면 캡처가 아니며 로봇·화물·점유·발견한 장애물을 그리지 않는다. 라벨은 ASCII만 쓰고(ID·구역 문자 유지) 같은 지도·같은 PIL 버전에서 바이트가 같다. 크기 참고: 태그 전체 투영 12.2 KB, `summary` 4.3 KB.
- 매 호출 지도와 도식을 다시 포함한다. 이전 호출에서 봤다는 이유로 생략하지 않는다. 지도는 실행 동안 불변이고, 발견한 변화는 로봇 belief에만 남긴다.
- `verify_static_map_section(section)`으로 실행 뒤에 지도 파일에서 투영을 다시 만들어 해시를 비교할 수 있다.

## 4. 시나리오 설정과 주문서

주문서는 **시뮬레이터 객체 없이** 시나리오 설정과 지도 파일만으로 만든다. 별도 프로세스에서 `sim.*`·`mujoco` 임포트 없이 같은 해시가 나오는지 테스트가 확인한다.

```json
{
  "schema": "ugrp.zone_scenario.v1",
  "scenario_id": "S1_normal_mixed",
  "map_id": "zone_wide_two_doors_tags_v1",
  "seeds": [11, 12, 13],
  "orders": [
    {"order_id": "order-1", "kind": "long_beam", "count": 1, "item_ids": ["long_beam-1"],
     "required_robots": 2, "destination_zone": "A", "identity": "specific_item",
     "initial_location": {"pickup_bay": "P1", "slot": "P1-2"}}
  ],
  "eval": {"hidden_events": [{"at_sim_s": 60.0, "kind": "item_moved", "item": "long_beam-1"}]}
}
```

- `initial_location`은 **설정 시점에 그곳에 두기로 정한 coarse 위치**다. 이동·낙하·회수 뒤에도 갱신하지 않으며 simulator 좌표·body 이름을 담지 않는다.
- `required_robots`는 종류별 물리 필요 인원(`long_beam` 2, `tri_frame` 3, 나머지 1)과 **일치해야** 한다. 값이 다르면 설정 오류로 거부한다.
- `identity`는 `kind_fungible`(종류만 맞으면 됨) 또는 `specific_item`(특정 개체 배송)을 **사전에 고정**한다. 후자는 `item_ids`가 필수다.
- 숨은 사건은 `eval` 절에만 둔다. 주문서·payload에 들어가지 않고, payload 해시는 `eval`이 바뀌어도 같다.
- `OrderSheetSource`는 생성 시 설정을 깊은 복사해 고정한다. `sheet()`는 매번 새 복사본을 주고, `assert_unchanged()`는 설정·주문서가 실행 중 바뀌었는지 검사한다. `manifest()`는 주문서·설정 해시, seed, 지도 해시, 숨은 사건 개수, 입력 프로파일을 남긴다.
- `note_ko`는 주문서가 계획 정보일 뿐 현재 위치·재고·완료를 보장하지 않는다는 문장을 담는다.

## 5. 호출별 입력 빌더

```python
bundle = map_bundle('zone_wide_two_doors_tags_v1')
source = OrderSheetSource(scenario, bundle)
payload = build_call_input(robot_id='r2', condition_name='leader_ko', request_id='req-1',
                           sim_time_s=12.5, static_map=static_map_for_call(bundle),
                           sheet=source.sheet(), seed=11,
                           own_rgb_refs=[own_rgb_ref('r2', 42, 12.5)],
                           own_command_history=[command_entry('cmd-1', 3.0, 'goto', {'target_ref': 'P1'})],
                           inbox=[message_envelope('leader_ko', 'm-1', 'r3', ['r2'],
                                                   {'text': 'order-1을 맡아 주세요'},
                                                   created_at_sim_s=11.0, seed=11)])
```

빌더는 조건 allowlist에 있는 키만 조립하고, `leader_ko`면 `leader_id`·`role`을 seed로 채우고, 마지막에 `validate_robot_payload()`를 통과해야 반환한다. 공정성을 위해 모든 조건에 같은 `INPUT_PROFILE`을 적용한다(자기 RGB 2장, 자기 명령 12개, inbox 8개, 텍스트 8,000 토큰·출력 768 토큰은 **개발 시작값**이며 본실험 전에 실제 tokenizer로 고정한다).

메시지 봉투는 `message_id`, `sender`, `recipients`, `encoding`, `created_at_sim_s`, `reply_to`, `body`다. 의미는 `body`에 둔다.

- 자유 조건: `{"text": "한국어 본문"}`만 허용한다.
- 정형 조건: `act`(`propose|request|accept|reject|inform|correct|yield|cancel`), `item`, `zone`, `role`, `passage`, `location_ref`, `state`(`unknown|suspected|clear|blocked|present|absent|held|placed`), `confidence`(`low|medium|high`), `observed_at_sim_s`, `reply_to`만 허용한다. `text`·`reason`·`note`·`other` 같은 자유 문자열 우회 필드는 거부한다. ID는 `vocabulary(sheet, public_map)`가 만든 어휘(주문·물건·종류·구역·역할·통로·위치 참조)에서만 고른다.

## 6. 로그 schema (D·I가 소비)

`LOG_SCHEMAS`에 필드 설명이 있고 `validate_log_record()`가 검사한다. 빌더는 `call_log_record()`, `message_log_record()`, `action_log_record()`다.

**호출 로그 `ugrp.zone_study_call.v1`** — `run_id`, `condition`, `seed`, `actor`, `role`, `request_id`, `call_index`, `trigger`(`start|own_view_change|own_timer|message_received|idle_review|execution_review`), `requested_at_sim_s`, `released_at_sim_s`, `sim_cost_s`, `cost_terms`, `input_sha256`, `input_tokens`, `output_tokens`, `wall_latency_s`, `http_attempts`, `status`(`ok|invalid_json|rejected_message|timeout|http_error|budget_exhausted|policy_refusal`), `action_id`, `message_ids`, `decision_sources`, `payload_validated`.

- `sim_cost_s == released_at_sim_s - requested_at_sim_s`여야 한다(패키지 D의 결정론적 비용식이 채운다). `wall_latency_s`는 기록만 하고 SIM 순서를 바꾸지 않는다.
- `payload_validated`가 참이어야 한다. 검증을 통과하지 않은 입력으로 한 호출은 유효한 기록이 아니다.
- 잘못된 JSON·거절된 메시지·timeout도 호출로 기록한다. 재시도는 `http_attempts`로 센다.

**메시지 로그 `ugrp.zone_study_message_log.v1`** — 봉투 필드에 `delivered_at_sim_s`, `delivery_delay_s`, `status`(`delivered|pending|rejected|dropped_budget`), `rejected_reason`, `body_sha256`, `act`, `chars`, `korean_ok`를 더한다. `delivered` 상태는 전달 시각이 있어야 하고 전달 시각은 생성 시각보다 앞설 수 없다.

**행동 로그 `ugrp.zone_study_action.v1`** — `action_id`, `request_id`, `submitted_at_sim_s`, `kind`(`claim_order|goto|observe|grasp|place|release|wait|yield_passage|abort_job|noop`), `arguments`, `order_id`, `role`, `accepted`, `rejected_reason`, `local_state`. `arguments`에도 평가 전용 키 검사를 적용한다. **명령 제출은 이동·파지·성공이 아니다.**

## 7. 검증 범위와 한계

- 확인한 것: 조건 registry와 채널 격리, seed 지휘자 순환, 자유/정형 encoding 검사, 평가 전용 키·외부 카메라·비ASCII 키 거부, 시뮬레이터 없는 주문서 생성과 실행 중 고정, 숨은 사건 분리, 지도 투영·도식의 결정론과 해시 연결, 로그 schema 검증. `pytest tests/test_zone_study_contract.py tests/test_zone_study_inputs.py` 70개 통과.
- 확인하지 않은 것: 실제 물리 실행, 자기 카메라 인식·완료 판단, 프롬프트 문구, SIM 비용식의 값, 모델 호출, 평가 지표 계산, 조건 사이의 성능 차이. 이 계층의 통과를 통신 효과의 근거로 쓰지 않는다.
- 이 패키지는 `maps/zones/*`, `sim/*`, 기존 zone 실행기·프로토콜 파일을 수정하지 않았다. `scripts/run_ci_tests.py` 등록은 브랜치 충돌을 피해 통합 패키지에서 추가한다.
