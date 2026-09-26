# SIM 시간 추론·발화 비용과 사건 스케줄러 (패키지 D)

- **날짜:** 2026-09-26
- **상태:** 구현·자동 테스트 완료, **러너 미통합**. 물리 실행·모델 호출로 검증하지 않았다.
- **파일:** `harness/zone_sim_cost.py`, `harness/zone_event_scheduler.py`, `tests/test_zone_sim_cost.py`, `tests/test_zone_event_scheduler.py`
- **설계 근거:** [2026-09-25 통합 연구 설계(Codex) 5절](design/2026-09-25-zone-dialogue-study-design-codex.md). 지휘자 형태·자기 카메라 전용 등 이후 사용자 결정이 그 문서를 대체하는 부분은 결정을 따른다.

## 1. 왜 필요한가

현재 구역 러너는 **LLM 응답을 기다리는 동안 SIM 시간이 멈춘다.** `team.ask()`가 future 결과를 받아 반환한 뒤(`scripts/three_robot_runtime.py:150`) 실행 루프 끝에서 `zone.step(.5)`로만 물리를 전진시키고(`scripts/run_zone_dispatch.py:332`), makespan은 `motion_started` 이후의 SIM 시간이다(`scripts/run_zone_dispatch.py:275`, `339`). 즉 생각과 대화가 공짜다.

그 상태에서는 "한국어 대화가 작업 효율을 바꾸는가"를 물을 수 없다. 대화 조건은 비용 없이 협상하고, 무통신 조건은 협상할 것이 없으므로 대화의 이득만 남는다. 이 패키지는 대기를 **내용에서 결정론적으로 계산한 SIM 초**로 바꾼다. wall 시간과 API 지연은 비용에 들어가지 않는다.

## 2. 비용식

호출 `q`의 대기 비용:

```
raw_q = scale * (alpha + a_in * N_in + beta * N_out + gamma * U)
d_q   = quantum * ceil(raw_q / quantum)
```

| 기호 | 필드 | 뜻 |
|---|---|---|
| `alpha` | `call_overhead_s` | 호출당 고정 판단 비용 |
| `a_in` | `input_token_s` | 입력 토큰당 비용 (prefill) |
| `N_in` | `Attempt.input_tokens` | 고정 tokenizer로 센 입력 토큰. **이미지 토큰 추정치를 호출자가 포함**한다 |
| `beta` | `output_token_s` | 출력 토큰당 비용 (행동 JSON + 메시지) |
| `N_out` | `Attempt.output_tokens` | 출력 토큰 |
| `gamma` | `utterance_s` | 발화 시작·전송 추가 비용 |
| `U` | `Attempt.utterances` | 비어 있지 않은 발화 수. 방송 1건은 1발화 |
| `scale` | `scale` | 민감도 스윕 배율. `0`은 비용 없는 대화(현재 러너) 진단 조건 |
| `quantum` | `quantum_s` | 러너가 물리를 전진시키는 SIM 격자. 모든 비용을 이 격자로 올림 |

입력 토큰 항은 사용자 결정(2026-09-26)으로 추가했다. 설계 문서의 예시(출력 120토큰·1발화 → 3.7초)는 `a_in = 0`인 경우와 같고, 8,000 입력토큰을 더하면 5.3초다.

전달 지연은 `quantize(scale * (delivery_s + per_recipient_s * 수신자수))`다. 기본값에서 `per_recipient_s = 0`이므로 **방송은 직렬화되지 않는다.** 모든 수신자가 같은 SIM 시각에 받는다. 이것이 전달 순서를 HTTP 완료 순서와 무관하게 만드는 근거다.

### 기본값과 근거 (모두 잠정)

`zone_sim_cost.v1`:

| 파라미터 | 기본값 | 근거 |
|---|---:|---|
| `call_overhead_s` | 1.0 | 요청 조립·이미지 인코딩·큐 대기의 호출당 고정분. 설계 문서 시작값 |
| `input_token_s` | 0.0002 | flash급 호스팅 모델의 prefill 약 5,000 tok/s 가정 → 8,000토큰 입력 예산에서 1.6초 |
| `output_token_s` | 0.02 | decode 약 50 tok/s 가정. 설계 문서와 동일 |
| `utterance_s` | 0.3 | 발화 준비·전송. 설계 문서와 동일 |
| `delivery_s` | 0.1 | 전달 지연. 설계 문서와 동일 |
| `error_s` | 0.5 | 전송 오류(출력 없음)의 사전 고정 비용 |
| `timeout_s` | 20.0 | 응답 없음의 사전 고정 대기 비용. 실제 관측 지연이 아니다 |
| `quantum_s` | 0.1 | 현재 러너 제어 주기(0.5초)보다 작은 격자. 통합 시 러너 step에 맞춘다 |

이 값들은 **이 프로젝트 모델의 처리 속도를 측정한 값이 아니다.** 실험적 비용 설정이며 `CostParams.provisional=True`로 표시한다. 본실험 전에 선택한 모델·tokenizer·입력 프로파일로 검증하고 코호트 동안 동결한다. 개발용 정상 이동·집기 시간과 비교해 짧은 판단의 상대 비용을 정하는 작업이 남아 있다.

### 실패한 시도와 재시도

한 호출은 **시도(attempt)의 수열**이다. 총합을 한 번만 격자로 올린다.

| 시도 결과 | 비용 |
|---|---|
| `ok` | `alpha` + 토큰 + 발화 |
| `invalid` (형식 오류·거절된 메시지) | `ok`와 같다. 생성했으므로 지불한다 |
| `error` (프록시·전송 실패) | `error_s` + 보고된 토큰 |
| `timeout` | `timeout_s` + 보고된 토큰 |

호출의 결과는 마지막 시도의 결과다. 재시도로 성공한 호출은 `ok`이지만 실패 시도의 비용도 낸다. 스케줄러는 실패한 호출을 **별도 호출**로 한 번 더 시도할 수도 있고(`CallPolicy.max_retries`), 그 호출은 자기 비용을 따로 낸다.

### 사용량 미상 (`usage_known`)

전송 계층이 공급자가 청구한 사용량을 읽지 못하면(일반 예외, 사용량 보고 없이 실패한 재시도) 그 호출은 `usage_known=False`다. 이 표지는 응답 → 호출 ledger → horizon censor → 패키지 A 호출 기록(`cost_terms.usage_known`, `cost_terms.usage_bound`) → 평가 → 보고서·TensorBoard까지 그대로 간다(Codex 3·4차 검토 #16).

- **표지를 바꾸지 않는다.** 재시도 예산 거절로 재생 응답을 자를 때도 원래 응답의 표지를 물려받는다. 잘린 응답의 `provider_usage`는 보내지 않은 시도까지 포함한 보고이므로 버린다(`None`).
- **알려진 하한을 지우지 않는다.** 일부 시도의 사용량만 알면 그 수(예: 입력 833·출력 40)를 기록하고 `usage_bound='lower_bound'`로 표시한다. horizon에서 censor된 호출도 같다. 스케줄러가 계산한 청구 SIM 비용(`charged_sim_s`, `would_release_sim_s`)도 비용 모형의 사실이므로 남긴다.
- 실제 전송 계층은 `TransportFailure(..., attempts=..., usage_known=False)`로 "아는 것이 전부가 아니다"를 알린다. 시도를 하나도 밝히지 않은 `TransportFailure`는 미상으로 기록한다.
- **보낸 시도와 그 예산을 돌려주지 않는다(Codex 5차 검토 P1).** 준수하는 전송 계층은 시도마다 보내기 직전에 `PendingCall.reserve`로 예약한다. 사용량 미상 응답(예외, `TransportFailure`, `usage_known=False`인 `CallReply`)이 예약보다 적은 시도를 밝히면, 스케줄러는 **예약한 시도를 모두 보낸 것으로 센다.** 빠진 시도는 비용을 내는 `error` 시도로 채워 앞에 두고(응답의 마지막 시도가 호출 결과로 남는다), `unreported_attempts`(`contract_log()`에도 포함)와 `attempts_unreported` 사건에 기록한다. 그래서 `commit`이 예약을 돌려주지 않고, 스케줄러 재시도가 이미 쓴 예산을 다시 쓰지 못한다. 사용량을 **아는** 응답은 자기 시도 수를 밝히는 것으로 보고, 쓰지 않은 예약은 전처럼 반환한다. 예약보다 많은 시도는 전처럼 예산 위반(`over_budget_attempts`)이다.

## 3. 실행 의미 (스케줄러 계약)

`harness/zone_event_scheduler.py`가 단일 SIM 사건 큐를 소유한다. makespan에 비용을 사후 덧셈하지 않는다. 기다리는 동안 생기는 통로 경쟁·낙하·보고 지연이 물리에 반영되어야 하기 때문이다.

1. 호출은 시작 시각 `t`의 입력을 캡처한다. 응답은 `t`에 대한 결정이다.
2. 호출한 로봇은 안전한 hold로 들어간다(`on_hold` 콜백). 다른 로봇과 물리는 계속 진행한다.
3. 행동과 메시지는 `t + d_q` 전에 노출되지 않는다.
4. 메시지는 지정된 전달 시각에만 inbox에 들어간다.
5. 동시 호출 비용은 겹친다. 세 로봇이 같이 생각하면 합이 아니라 최댓값이 makespan에 들어간다.
6. API가 느리면 **계산**이 wall 시간에서 기다릴 수 있으나 SIM 비용과 사건 순서는 바뀌지 않는다.
7. 같은 응답·seed·설정이면 HTTP 완료 순서가 달라도 SIM trace가 같다.

### 6·7을 만드는 방법

- 응답 내용을 알아야 비용이 정해지므로, 루프는 **아직 모르는 완료 시각보다 확실히 앞선 사건**을 먼저 처리한다. 하한은 `CostParams.min_call_s()`(= `scale * min(alpha, error_s, timeout_s)`의 격자 올림)이며 파라미터만의 함수다.
- 미완료 호출을 회수할 때는 도착 순서가 아니라 `(시작 시각, actor 순위, call_id)` 순으로 회수한다.
- 전달 사건의 정렬 키는 `(전달 시각, 발신 호출의 완료 시각, 발신자 순위, 메시지 index, 수신자 순위)`다.
- 응답을 이미 받아 두었어도 로봇은 `t + d_q`까지 계속 hold 상태다. 회수 시점이 아니라 SIM 시각이 대기를 끝낸다.

### 같은 SIM 시각의 사건 순서

`KIND_ORDER`: `message`(0) → `call_done`(1) → `timer`(2) → `observe`(3) → `call_start`(4).

전달이 가장 먼저라 같은 시각에 시작하는 호출은 그 시각까지 도착한 메시지를 모두 본다. 호출 시작이 가장 나중이라 같은 시각의 완료 결과도 본다.

### 호출 계기와 자격

계기는 `start`, `idle`, `report`(메시지 수신), `blockage`, `failure`, `timeout`, `retry`, `timer`다. `CallPolicy` 시작값(설계 문서 기준, 잠정):

| 항목 | 값 | 뜻 |
|---|---:|---|
| `min_interval_s` | 2.0 | actor당 최소 호출 간격. 이른 계기는 버리지 않고 뒤로 미룬다 |
| `max_outstanding_per_actor` | 1 | 동시 미완료 호출 |
| `idle_reask_s` / `busy_reask_s` | 10 / 60 | 재검토 타이머 시작값 |
| `observe_period_s` | 1.0 | 자기 카메라 관측 주기. `arm_observations()`로 시작한다 |
| `max_retries` | 1 | 실패 호출의 추가 호출 수 |
| `max_calls_per_actor` | 30 | 예산. 초과분은 조용히 사라지지 않고 `call_refused`로 기록된다 |
| `max_attempts_total` | 90 | 팀 전체 HTTP 시도 예산 |

생각하는 중에 도착한 계기는 **하나로 병합**되고, 가장 강한 라벨(`failure > blockage > timeout > report > retry > idle > timer`)을 남긴 뒤 병합된 라벨을 기록한다. 공정성은 호출 수를 같게 만드는 것이 아니라 자격·처리 규칙·상한을 같게 하고 추가 호출의 비용을 실제로 물리는 것이다.

### 공동 운반과 지휘자

공동 운반 중 한 참여자가 생각해야 하면 공통 안전 정책에 따라 팀이 함께 멈춘다. grip 명령은 유지하되 물체 pose를 고정하거나 weld를 켜지 않는다. 그래서 생긴 팀 대기와 미끄러짐도 비용이다. 이 정책은 실행기 쪽(패키지 F) 책임이고, 스케줄러는 `on_hold`로 알린다.

물리 몸체가 없는 참고 상한 조건(전지적 지휘자)에서는 지휘자의 사고 시간에 로봇이 자동 정지하지 않는다. 기존 명령을 수행하거나 다음 지시를 기다린다. 이 차이는 조건의 구조적 특성으로 보고한다.

## 4. 사용 예

```python
from harness import zone_sim_cost as zc
from harness.zone_event_scheduler import CallReply, EventScheduler, Message

sched = EventScheduler(transport,                       # submit(call) / reply(token)
                       cost_params=zc.params('zone_sim_cost.v1'),
                       actors=('r1', 'r2', 'r3'),
                       advance=lambda a, b: zone.step(b - a),   # 물리는 계속 진행
                       on_hold=hold_last_safe_command,
                       on_action=submit_action,
                       on_message=update_belief)
sched.arm_observations()
for rid in ('r1', 'r2', 'r3'):
    sched.trigger(rid, 'start')
report = sched.run(until_s=1800)      # 시행당 SIM 예산
```

전송 계층이 돌려주는 응답:

```python
CallReply(attempts=(zc.Attempt(input_tokens=8000, output_tokens=120, utterances=1),),
          action={'job': 'order-1', 'role': 'west'},
          messages=(Message(sender='r1', recipients=('r2', 'r3'),
                            body='좁은 문 앞이 막혔다. door_wide로 우회한다.', encoding='ko'),))
```

`encoding`은 패키지 A의 값 `free_ko`(자유 한국어)와 `schema`(고정 schema) 두 가지이며 `ENCODINGS`는 A의 조건 registry에서 파생한다. 스케줄러는 본문을 해석하지 않는다. 메시지가 호스트의 예약이나 다른 로봇의 행동을 직접 바꾸지 않고, 수신자의 다음 호출 입력이 될 뿐이다.

기록은 `sched.calls`(`CallCostRecord`), `sched.messages`(`MessageCostRecord`), `sched.metrics`, `sched.trace()`로 얻는다. `trace()`는 두 실행의 SIM 결과를 그대로 비교할 수 있는 문자열 tuple이다.

## 5. 민감도 스윕

비용이 달라지면 행동과 관측 시점도 달라진다. **최종 비교는 설정마다 다시 실행한다.** 저장 로그 재계산은 근사 분석이며 `sensitivity()`·`recost()`의 결과에 `approximate: True`로 표시된다.

- `zc.sweep()` — 배율 `0, 0.5, 1, 2, 4`. `0`은 비용 없는 대화의 진단 조건이다.
- `zc.sweep_axes()` — `scale`, `call_overhead_s`(alpha), `output_token_s`(beta), `utterance_s`(gamma)를 한 번에 하나씩 변화시켜 "잦은 짧은 발화"와 "드문 긴 발화"를 구분한다.
- `zc.sweep_grid()` — 필요할 때만 쓰는 격자. 실행 수가 곱으로 늘어난다.
- 한국어와 정형 schema의 표현 길이 차이도 효율에 포함된다. 토큰화 효과를 분리하려면 의미 단위당 비용을 맞춘 보조 비교를 따로 둔다.

모든 변이는 고유한 버전 라벨(`zone_sim_cost.v1+scale=2`)과 `digest()`를 가진다. 스윕 지점을 기준 설정으로 잘못 기록할 수 없다.

## 6. 검증 범위와 한계

`.venv-sim-worker-mac/bin/python -m pytest tests/test_zone_sim_cost.py tests/test_zone_event_scheduler.py` → **71 통과** (2026-09-26). 확인한 내용:

- 비용식과 격자 올림, 설계 문서 예시(3.7초) 재현, 입력 토큰 항 추가분(5.3초).
- 비용 0(`scale=0`)과 양수, 입력·출력·발화 각각에 대한 단조성, 긴 발화의 추가 비용.
- 시도 결과별 비용, 재시도의 이중 지불, 전송 오류·timeout의 사전 고정 비용, 예외를 던지는 전송 계층.
- 동시 호출 겹침(3×5.3초 → makespan 5.3초), 서로 다른 길이의 응답이 다른 시각에 끝남.
- 방송 1발화 = 같은 SIM 시각의 여러 전달 edge, 수신자 아닌 로봇의 inbox 비어 있음.
- **API 완료 순서를 바꿔도 같은 trace**: 완료 순서를 지정하는 이중과 실제 스레드 지연(r1↔r3 역전) 두 방식으로 확인.
- 최소 호출 간격, 생각 중 계기 병합, 동시 미완료 1개, 호출·시도 예산 거절.
- 물리 advance 콜백의 단조성·연속성(구간 사이 공백 없음), hold 구간 = 생각 구간.
- 두 모듈 모두 시계를 읽지 않음(`time` 미참조).

확인하지 않은 것:

- **러너 통합.** 의도적으로 하지 않았다. PR 169 이후 러너 담당이 `scripts/zone_dispatch_v2.py`에 연결한다. 그 전까지 실제 물리·모델 호출로 이 비용을 쓴 실행은 없다.
- **패키지 A 로그 스키마(정렬 완료).** 비용 행은 `ugrp.zone_sim_cost.call_cost.v1` / `ugrp.zone_sim_cost.message_cost.v1`이고, **로그 기록은 패키지 A가 소유한다.** `contract_call_record()`가 `ugrp.zone_study_call.v1`을, `contract_message_records()`가 `ugrp.zone_study_message_log.v1`을 만들고 A의 `validate_log_record`로 검사한다. `EventScheduler.contract_log()`가 한 실행 전체를 그 형식으로 내보낸다.
  - `CallCost.cost_terms()`는 A가 요구하는 `alpha_s`·`beta_s_per_token`·`gamma_s_per_utterance`·`output_tokens`·`utterances`에 이 모듈의 감사 항목(입력 토큰 비용, raw·격자, 시도별 결과, 파라미터 해시)을 더한다.
  - 스케줄러의 계기 이름은 병합 우선순위를 담고 있어 A의 `TRIGGERS` enum과 1:1이 아니다. `TRIGGER_TO_CONTRACT`가 대응을 고정하며, 대응이 없는 이름은 조용히 넘기지 않고 `KeyError`로 막는다.
  - 시도 결과는 `OUTCOME_STATUS`로 A의 `CALL_STATUS`에 대응한다(`invalid`→`invalid_json`, `error`→`http_error`).
  - 방송 한 건은 A 기록 한 행이며 수신자별 전달 시각이 `deliveries`에 들어간다.
- **CI 등록.** `scripts/run_ci_tests.py`는 다른 브랜치와 충돌하는 공용 파일이라 건드리지 않았다. 통합 담당이 두 테스트 파일을 목록에 추가해야 CI에서 돌아간다.
- **기본값의 타당성.** 위 표의 근거는 일반적인 처리량 가정이다. 모델·tokenizer를 정한 뒤 검증하고 동결해야 한다.
- `scale=0`(비용 없음) 조건에서는 전달 지연도 0이라 같은 SIM 시각에 메시지-호출이 이어질 수 있다. 최소 호출 간격과 예산이 이를 유한하게 막지만, 이 진단 조건은 사건 수 상한과 함께 쓰는 것이 안전하다.

## 7. 연결

- 조건·입력 계약·로그 스키마: 패키지 A (`kiro/zone-study-contract`)
- 프롬프트·통신 위상(mesh / 허브-스포크)·정형 schema: 패키지 C
- 자기 카메라 인식·완료 판정: 패키지 B, PR #176/#177
- 러너 통합과 조건별 dispatch: 패키지 G, PR 169 이후
- 평가 지표(idle robot-seconds 분해, 발화 비용, latency와 SIM 비용의 분리 기록): 패키지 I
