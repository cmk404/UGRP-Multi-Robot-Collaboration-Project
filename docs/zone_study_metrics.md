# 한국어 대화 효율 연구 — 평가 지표

2026-09-25/26 사용자 결정에 따른 통신 조건 비교의 **평가 전용** 지표 정의다. 구현은
[`harness/zone_study_eval.py`](../harness/zone_study_eval.py), 보고서 생성은
[`scripts/zone_study_report.py`](../scripts/zone_study_report.py), 검사는
[`tests/test_zone_study_eval.py`](../tests/test_zone_study_eval.py)에 있다.

이 문서는 지표의 정의와 계산 규칙만 정한다. 조건 간 우열은 사전 고정한 코호트를 실제로
실행한 뒤에만 주장한다. 현재 저장소에는 이 지표로 판정한 실행 결과가 없다.

## 경계

- 심판(referee) 정답, TOP 카메라, 사실성 판정, PAR 시간은 모두 사후 산출물이다. 로봇 요청·
  기억·호출 계기·실행기 결정으로 되돌리지 않는다. `harness.zone_study_eval`의 모든 함수는
  입력 기록을 변경하지 않으며, `trial_metrics()`는 계산 전후의 직렬화를 비교해 변경이
  생기면 `TrialError`를 낸다.
- 로봇 입력 allowlist는 `ALLOWED_INPUT_KEYS`, 금지 목록은 `FORBIDDEN_INPUT_KEYS`다.
  기록의 `requests[].input_keys`를 이 두 집합과 대조하며, **둘 다에 없는 key도 통과시키지
  않고** `unknown_input_keys`로 보고한다.
- 조건 R(전지적 지휘 참조 상한)은 설계상 세 로봇의 영상을 받는다. 그래서 R의 감사 결과는
  기록하되 주 4조건의 경계 판정에 합산하지 않는다. R은 주 조건이 아니다.

## 조건

| 키 | 표기 | 채널 규칙 |
|---|---|---|
| `no_comm` | ① 무통신 | 고수준 메시지 송수신 0. 발화가 있으면 위반 |
| `peer_ko` | ② 자유 한국어 대화 | mesh, 자유 한국어 본문 |
| `leader_ko` | ③ 한국어 지휘 겸임 | 로봇 한 대가 지휘 겸임, seed마다 r1/r2/r3 **순환**. 허브-스포크만 허용하며 follower 사이 직접 전달과 leader 방송은 위반. follower의 보고·거절은 허용 |
| `peer_structured` | ④ 정형 메시지 | mesh, 고정 schema만. 비어 있지 않은 자유 `text`는 위반 |
| `central_rgb_reference` | R 참조 상한 | 주 조건이 아니다. 비교에서 기본 제외이며 `--include-reference`로만 포함 |

`leader_ko`는 `leader_id`가 필수이고 다른 조건에는 있으면 거절한다. 순환 여부는 조건 요약의
`leader_ids`에서 확인한다.

## 잠정 기록 형식

로그 schema의 최종 소유자는 Package A(`kiro/zone-study-contract`)다. 그 PR이 올라오면
`SUPPORTED_SCHEMAS`에 계약 schema id를 추가하고 `parse_trial()`에 어댑터를 넣는다. 그때까지
이 모듈은 `ugrp.zone_study_trial.provisional.v1`만 읽고, 모르는 schema는 거절한다.

```json
{
  "schema": "ugrp.zone_study_trial.provisional.v1",
  "trial_id": "peer_ko-mixed-s101",
  "condition": "peer_ko",
  "scenario": "mixed",
  "seed": 101,
  "leader_id": null,
  "robots": ["r1", "r2", "r3"],
  "literals": ["door_narrow", "door_wide", "P1", "P1-2"],
  "provenance": {"code_sha": "…", "bundle_id": "…", "map_id": "zone_wide_two_doors",
                 "map_sha256": "…", "order_sheet_sha256": "…", "cost_profile_sha256": "…"},
  "budget": {"sim_horizon_s": 1800.0, "http_attempts": 90, "max_calls_per_actor": 30},
  "t0_sim_s": 0.0,
  "end_sim_s": 812.4,
  "end_reason": "orders_complete",
  "orders": [{"order_id": "order-1", "item_ids": ["long_beam-1"], "kind": "long_beam",
              "count": 1, "required_robots": 2, "destination_zone": "A",
              "initial_location": {"pickup_bay": "P1", "slot": "P1-2"}}],
  "referee": {
    "deliveries": [{"item_id": "long_beam-1", "zone": "A", "sim_s": 700.2, "correct": true}],
    "conflicts": [{"kind": "role_contention", "sim_s": 120.0, "robots": ["r1", "r2"]}],
    "deadlocks": [{"sim_s": 300.0, "duration_s": 45.0, "robots": ["r2", "r3"]}],
    "blockages": [{"passage": "door_narrow", "from_s": 0.0, "to_s": null}],
    "holds": [{"item_id": "crate-2", "robot": "r1", "from_s": 200.0, "to_s": 640.0}],
    "slot_states": [{"item_id": "crate-2", "present": false, "sim_s": 210.0}]
  },
  "idle": {"r1": {"thinking": 42.0, "door_wait": 10.0}},
  "replans": [{"sim_s": 310.0, "robot": "r2", "kind": "passage_change", "reason": "peer report"}],
  "decision_changes": [{"sim_s": 310.0, "robot": "r2", "kind": "passage_change",
                        "request_id": "req-17"}],
  "model": {"logical_calls": 24, "http_attempts": 26,
            "tokens": {"input": 90000, "output": 4200, "image": 1800, "cached": 0},
            "sim_cost_s": {"think": 24.0, "talk": 3.0, "delivery": 0.5},
            "wall_latency_ms": [810.0, 930.0]},
  "requests": [{"request_id": "req-1", "robot": "r1", "sim_s": 0.0,
                "input_keys": ["static_map", "order_sheet", "own_rgb", "own_commands", "inbox"]}],
  "utterances": [{"message_id": "m-1", "sender": "r1", "recipients": ["r2"],
                  "sim_s": 300.0, "delivered_sim_s": 300.1, "encoding": "ko_free",
                  "text": "door_narrow가 막혀 있습니다.", "sim_cost_s": 1.9,
                  "grounds": ["own-r1-0042"],
                  "claims": [{"type": "blocked", "passage": "door_narrow"}]}]
}
```

- `referee`는 평가 전용이다. `blockages`·`holds`·`slot_states`가 없으면 관련 주장은 `false`가
  아니라 `unverifiable`이 된다.
- `decision_changes`가 없으면 `replans`를 결정 변경으로 사용한다.
- `literals`는 정적 지도 투영에서 온 문자열이다(문·복도·bay·slot ID). 한국어 준수 계산에서
  제외할 literal 집합을 만드는 데 쓴다.
- `claims`를 직접 기록하면 규칙 추출보다 우선한다.
- `utterances[].encoding`은 `ko_free` 또는 `structured`다.

## 효율 지표

| 지표 | 정의 |
|---|---|
| `success` | `end_reason == "orders_complete"` |
| `makespan_sim_s` | `end_sim_s − t0_sim_s`. 추론·발화·전달 비용이 SIM 시계에 이미 포함된 값이다 |
| `makespan_success_only_s` | 성공 시행만의 makespan. 단독으로 조건을 비교하지 않는다 |
| `par_makespan_sim_s` | **주 시간 지표.** 성공은 실제 makespan, 비성공은 `penalty_factor × horizon`(기본 2.0) |
| `talk_sim_cost_s` | `model.sim_cost_s.talk + delivery` |
| `think_sim_cost_s` | `model.sim_cost_s.think` |
| `delivered_items` | 심판이 확인한 정상 배송. **물건당 한 번만** 집계 |
| `misdelivered_items` / `undelivered_items` / `surplus_items` | 오배송·미달·주문 외 배송 |
| `delivery_rate` | `delivered_items / ordered_items` |
| `par_sim_s_per_delivered` | `par_makespan / delivered_items`. 배송 0이면 `null` |
| `idle_robot_s`, `idle_by_reason` | 로봇초 합과 사유별 분해(`thinking`, `speaking`, `await_order`, `team_rendezvous`, `door_wait`, `unassigned`, `reobserve`, `other`). 목록 밖 사유는 `other`로 접는다 |
| `idle_share` | `idle_robot_s / (로봇 수 × par_makespan)` |
| `conflicts`, `conflicts_by_kind` | 심판이 사후 공통 기준으로 센 충돌 |
| `deadlocks`, `deadlock_sim_s` | 교착 횟수와 누적 시간 |
| `replans`, `replans_by_kind` | 작업·역할·통로 변경과 취소·재시도 |
| `model_calls`, `http_attempts` | 논리 호출과 실제 HTTP 시도를 구분 |
| `tokens_input/output/image/cached`, `tokens_total` | `total = input + output + image` (캐시 제외) |
| `wall_latency_ms_mean` | 실제 API 지연. 부과한 SIM 비용과 별개로 기록한다 |

### 실패를 분모에 유지하는 방법

빠른 실패가 느린 성공보다 좋게 보이면 안 된다는 요구를 `par_makespan_sim_s`로 만족시킨다.

- 성공은 horizon 이전에 끝나야 하므로(`end > horizon`인 성공은 거절) 성공 값은 항상
  `≤ horizon`이다.
- 비성공은 `penalty_factor × horizon ≥ 2 × horizon`을 부과하므로 **어떤 실패도 어떤 성공보다
  크다**. `penalty_factor < 1`은 거절한다.
- 성공률·배송률의 분모는 `sim_horizon`, `budget_exhausted`, `deadlock`, `aborted`,
  `api_failure`, `policy_failure`, `orders_incomplete`를 모두 포함한다.
- 코호트 수준의 `cohort_par_sim_s_per_delivered = Σ par_makespan / Σ delivered_items`는
  배송이 0인 시행의 시간도 분자에 남긴다.

## 대화 지표

### 한국어 준수

PR 172(병합)의 `harness/zone_dialogue_metrics.py`를 그대로 호출한다. literal 토큰을 지운 뒤
한글/(한글+라틴) 비율이 `KOREAN_RATIO_THRESHOLD`(0.9) 이상인 발화를 준수로 센다.

- **침묵은 준수 성공이 아니다.** 글자가 없는 발화는 `silent_messages`로만 센다.
- literal 집합은 주문서(order/item ID, kind, `initial_location` 값), 기록의 `literals`,
  심판 로그가 지목한 통로 이름, 정형 메시지의 ID 필드, `STUDY_KEY_WORDS`에서 만든다.
  **어디에도 선언되지 않은 영어 단어는 코드전환으로 센다**(`code_switch_messages`).
- ID·구역 문자 번역·손상은 PR 172의 `id_issues`로 분류한다(`robot_id_variant`,
  `translated_label`, `translated_zone`, `unknown_label`).
- 정형 조건(④)에는 한국어 비율을 적용하지 않는다. 대신 자유 `text`가 있으면 채널 위반이다.

### 사실성

발화의 명제를 **발화 시각의** 심판 로그와 대조한다.

| 주장 | 참 판정 근거 |
|---|---|
| `delivered(item, zone)` | 심판 `deliveries`에 해당 물건이 발화 시각 이전에 그 구역으로 기록됨 |
| `holding(item, robot)` | `holds` 구간이 발화 시각을 포함 |
| `blocked(passage)` | `blockages` 구간이 발화 시각을 포함 |
| `absent(item)` | `slot_states`의 `present == false` |

- 필요한 심판 하위 로그가 없으면 `unverifiable`이며, 거짓으로 바꾸지 않는다.
- `truthful_share = true / (true + false)`. `unverifiable`은 분모에서 제외한다.
- 사실성과 **근거 보유**는 따로 본다. `grounds_verdict()`는 `grounds`가 자기 관측(`own-`),
  자기 명령(`command-`), 수신 메시지(`msg-`/`m-`), 개인 belief, 지도·주문서에서만 오면
  `grounded`, TOP·심판·교사·동료 raw(`top-`, `referee-`, `gt-`, `teacher-`, `peer-rgb-`,
  `peer-command-`)를 인용하면 `forbidden`, 기록이 없으면 `unknown`이다. `forbidden`은 입력
  경계 위반으로도 보고한다.
- 주장 추출은 규칙 기반이므로 재현성은 있으나 완전하지 않다. 사람 라벨과 비교할 때는 PR 172
  파일럿처럼 별도 `manual_labels`를 만들어 대조한다.

### 행위 유형

보고에 사용하는 5종은 `report`, `request`, `order`, `objection`, `ack`다. 세부 라벨은 PR 172의
`ACT_RULES`를 재사용하고(그 파일은 수정하지 않는다) 지휘 조건용 `order`와 PR 172가 다루지
않는 정중한 요청 어미만 이 모듈에서 보완한다.

| 세부 라벨(PR 172) | 5종 매핑 |
|---|---|
| `report`, `inform_obstacle`, `standby`, `claim` | `report` |
| `request`, `propose`, `question` | `request` |
| `agree`, `yield` | `ack` |
| `refuse`, `correct` | `objection` |
| `order` (이 모듈 추가) | `order` |

- `REQUEST_RULE_EXTRA`(`주세요`, `주십시오`, `주시겠`, `부탁` 등)를 먼저 적용한다. PR 172의
  `request` 규칙은 `해 주세요` 형태에 맞춰져 있어 `가져와 주세요`를 놓치기 때문이다.
- `ORDER_RULE`(`하십시오`, `배달하라`, `대기하십시오` 등)은 **요청 단서가 없을 때만** 적용한다.
  그래서 `옮겨 주세요`는 `request`, `옮기십시오`는 `order`다.
- 정형 메시지는 `act` enum을 그대로 세부 라벨로 쓰고 `inform→report`, `propose/request→request`,
  `accept/yield→ack`, `reject/correct→objection`, `cancel/order→order`로 매핑한다.
- 다중 라벨을 허용한다. 규칙에 걸리지 않으면 `other`, 글자가 없으면 `silence`다.

### 결정 변경 직전 발화

각 결정 변경 시각에서 `lookback_s`(기본 30 SIM초) 안에 **그 로봇에게 전달된** 발화를 모아
`preceding_inbound`에 남기고, 변경 중 몇 건이 직전 수신 발화를 가졌는지(`prior_share`)와
선행 발화의 행위 유형 분포를 집계한다. 전달 시각은 `delivered_sim_s`를 쓴다.

**이것은 연관이며 인과가 아니다.** 같은 시나리오의 무통신 조건이 같은 시점에 같은 변경을
했는지 함께 읽어야 한다.

## 짝지은 seed 비교

`compare_conditions(trials, metric, baseline, variant)`는 `(scenario, seed)`가 같은 두 조건의
시행을 짝짓는다. 같은 `(condition, scenario, seed)`의 반복은 먼저 평균해 짝 하나로 만든다.
한쪽만 있는 seed는 버리고 `n_pairs`에 남은 수만 센다.

- 부호는 `변이 − 기준`이다.
- `diff_ci`: 짝 차이에 대한 결정론적 percentile 부트스트랩(기본 10,000회, `--bootstrap-seed`로
  고정). 짝이 1개면 구간을 만들지 않는다.
- 효과크기: 짝 `cohens_dz`(차이 평균/표준편차)와 순위 기반 `rank_biserial`(동순위 평균 순위).
  모든 차이가 같으면 `dz`는 정의되지 않아 `null`이다.
- **p-value·유의성 판정을 만들지 않는다.** 짝이 `SMALL_SAMPLE_PAIRS`(10)개 미만이면
  `small_sample=true`로 표시하고 구간과 짝별 표만 읽는다.
- `success`도 0/1로 같은 방식으로 비교한다. 실패는 버리지 않는다.
- `compare_all()`은 R을 기본 제외한다. 실행 번들·지도·비용 프로파일이 섞였으면 요약의
  `provenance.mixed_fields`에 나타나며, 이 경우 조건 비교를 근거로 쓰지 않는다.

## 보고서와 TensorBoard

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
PYTHONPATH=. .venv-sim-worker-mac/bin/python scripts/zone_study_report.py \
  outputs/zone-study-<ID>/trials \
  --output outputs/zone-study-<ID>/report \
  --tb-events outputs/tensorboard/zone-study-<ID>
```

출력물:

| 파일 | 내용 |
|---|---|
| `summary.md` | 한국어 요약: 조건별 효율, 대화 지표, 행위 유형, 결정 변경 직전 발화, 지표별 짝 비교, 입력 경계 감사, 원본 SHA-256 |
| `metrics.json` | 시행별·조건별 전체 지표와 모든 짝 비교, 원본 경로·해시 |
| `scalars.json` | `ugrp.zone_study_scalars.v1`. run별 scalar와 HParams 열 |
| `tensorboard.json` | `--tb-events`를 준 경우의 기록된 run·scalar 수와 logdir |

- `--tb-events`는 TensorBoard 자체 protobuf로 이벤트 파일만 새로 쓴다. 뷰어·서버 설정,
  `outputs/tensorboard-view.json`, 기존 스냅샷은 건드리지 않는다. 공용 뷰어 등록과 화면 확인은
  [TensorBoard 안내](tensorboard.md)의 절차를 따로 따른다.
- run 이름은 `<condition>/<scenario>-s<seed>`와 조건별 `cohort/<condition>`이다.
- scalar 태그: `evaluation/success`, `result/*`(PAR makespan·makespan·발화/추론 비용·배송·idle·
  충돌·교착·재계획·호출·토큰·wall 지연), `dialogue/*`(발화 수·한국어 준수·코드전환·ID 손상·
  사실/거짓·사실성·채널 위반), `cohort/*`.
- HParams 열은 `condition`, `scenario`, `seed`, `leader_id`, `end_reason`, `penalty_factor`,
  `sim_horizon_s`다. HParams의 session status는 변환 완료를 뜻하며 로봇 성공이 아니다.
- 서로 다른 조건의 성공률을 자동 합산하지 않는다. 빠르게 실패한 실행의 시간을 성능 개선으로
  읽지 않는다.

## 검사 범위

`tests/test_zone_study_eval.py`는 합성 로그만 사용한다(시뮬레이션·모델 호출·네트워크 없음).
확인하는 내용은 schema·조건·`leader_id` 거절, 지휘 순환, 실패의 분모 유지와 PAR 단조성,
물건당 1회 집계, idle·충돌·교착·재계획 집계, 입력 경계(TOP·심판·정답·교사·동료 raw·미등록
key·금지 근거 인용), 평가 판정이 기록으로 역류하지 않음, 조건별 채널 규칙, 한국어 준수와
literal 처리, 5종 행위 유형, 사실성의 `true/false/unverifiable`, 결정 변경 직전 발화,
짝 비교·부트스트랩 결정성·R 제외, 보고서 파일과 TensorBoard 이벤트 생성이다.

지표 계산의 정확성만 확인한 범위다. 실제 통신 효과는 사전 고정한 코호트를 실행한 뒤 별도로
판정한다.
