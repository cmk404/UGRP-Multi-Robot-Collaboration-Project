# review-r5 — Codex 5차 검토(`codex-194-r5`) 3건 수정 (2026-09-26)

- **상태:** 수정 완료, 오프라인 스모크 **재실행 없음**. 새 스모크 버전(v5)을 만들지 않았다. 아래 "스모크 출력"에 근거를 적었다.
- **검토 기준:** `4c8f90807f1b44846fe4a551a55aa44a4a91dd64`. 판정은 not-ready(실제 LLM 파일럿 보류), 새 발견은 P1 1건·P2 2건이다.
- **수정 커밋:** `f44ea845c9c51aaf3c927c6d90288c9426faeb7d` (브랜치 `kiro/zone-study-core`, PR 194).
- **회귀 테스트:** `tests/test_zone_study_review_r5.py` 61개(SHA-256 `044d1433…`). `scripts/run_ci_tests.py` `TEST_PATTERNS`에 등록했다.
- 모델 호출·물리 실행은 0회다. 에이전트 잠금은 잡지 않았다(물리·학습·wall 시간 비교 없음).

## 수정 (`f44ea845` 기준 file:line)

| # | 반례 (Codex) | 수정 | 회귀 테스트 (`tests/test_zone_study_review_r5.py`) |
|---|---|---|---|
| P1 | HTTP 상한 2에서 첫 요청과 예약한 내부 재시도를 보낸 뒤 사용량 미상 예외가 나면 시도가 1개로 줄고 `commit`이 예약 1개를 돌려준다. 스케줄러 재시도가 그 예산을 다시 써서 **실제 3회·장부 2회**, 위반 0건. `TransportFailure(attempts=())`도 같다. SIM 실패 비용도 1초로 과소 청구된다 | `harness/zone_event_scheduler.py:690`–`716` `_reply_of`: 사용량 미상 응답(`usage_known=False`)이 예약보다 적은 시도를 밝히면 **예약한 시도를 모두 보낸 것으로 센다.** 빠진 시도는 `error` 시도로 채워 앞에 두고(`:716`, 마지막 시도가 호출 결과로 남음) `unreported_attempts`(`:412`)·`attempts_unreported` 사건에 기록한다. `contract_log()`에도 넣는다(`:536`). `:718`–`744` `_fetch_reply`는 전송 계층이 밝힌 시도 수를 함께 돌려준다. 사용량을 아는 응답은 전처럼 자기 시도 수를 따르고, 쓰지 않은 예약은 반환한다. horizon censor 경로도 같은 `_reply_of`를 쓴다 | `:109` 반례 3종(일반 예외·`attempts=()`·부분 사용량), `:139` 미상 `CallReply` 반환, `:148` horizon `pending` censor, `:169` 팀 상한 4 / 두 로봇. 경계: `:183` 예약 1개, `:190` 거절된 재시도 예약, `:198` 사용량 확정 응답의 예약 반환, `:208` 예약 초과는 계속 예산 위반 |
| P2 | `calls` 없이 `model.tokens_complete=False`, `usage_unknown_calls=1`, 알려진 833/40인 기록이 `tokens_complete=True`, 확정 합계 873으로 나오고 보고서도 873 | `harness/zone_study_eval.py:384`–`397`: summary 경로도 미상 표지를 돌려준다. `:466` `_summary_usage_marker`는 bool·0 이상 정수만 받고, 두 필드가 모순되면 거절하며, `false`에는 호출 수를 요구한다. 둘 다 없는 과거 요약은 확정이고 미상 수는 `null`이다. `:451` `_summary_count`는 토큰 수의 NaN·Inf·음수·문자열·bool·소수를 거절한다(전에는 NaN이면 예외로 죽고 `'833'`은 조용히 변환됐다). `:514`: `calls`와 요약이 같이 있으면 미상 수도 대조한다. `:910`: 옛 기본값 `.get('tokens_complete', True)`를 없앴다 | `:242` 반례를 지표 → 코호트 → `summary.md`(`≥873 (미상 1)`) → `scalars.json`까지, `:275` 수만 있는 경우, `:281` 수 없는 `false`, `:289` 모순 2종, `:335` 호출 기록과 대조. 경계: `:295` 미상 수 9종(bool·문자열·1.5·-1·NaN·Inf·list·dict), `:301` 표지 6종, `:307` 토큰 수 6종, `:314` 확정·0·과거 요약·`833.0`·`null` 대조군 |
| P2 | 같은 조건·시나리오·seed의 두 시행(미상, 정상 120토큰)이 둘 다 `peer_ko/mixed-s1`이 되어 TensorBoard에서 마지막 값(120·미상 0)만 보인다 | `harness/zone_study_eval.py:1900` `trial_run_name`: run 이름을 `<condition>/<trial_id>`로 바꿨다. `trial_id`는 경로 한 칸이어야 한다(`:1894`, ASCII 영숫자·`_`·`-`·`.`, 첫 글자 영숫자). `:1957`: run 이름이 겹치면 거절한다. 이름 뜻이 바뀌었으므로 payload schema를 `ugrp.zone_study_scalars.v2`로 올렸다(`:1897`). `scripts/zone_study_report.py:308` `_run_paths`: run 경로가 logdir를 벗어나면(`..`·절대 경로·빈 칸·`\`·NUL) 거절하고, **이미 있는 run은 쓰기 전에 거절한다**(`:329`). `build`는 보고서 파일을 쓰기 전에 이 검사를 한다(`:386`). `exist_ok=True`를 뺐다(`:346`) | `:377` 반례 재현, `scalars.json`에서 두 run 분리, `:408` 이벤트 파일 재읽기, `:421` 중복 `trial_id` 거절, `:430` 안전하지 않은 `trial_id` 9종, `:447` 기존 스냅샷 run 보존(파일 해시 불변), `:461` 보고서 파일도 안 씀, `:473` logdir 이탈 경로 9종 |

- **수정 전 실패:** 기본 체크아웃 추적 파일의 /tmp 사본에서 수정한 소스 3개를 `4c8f908`로 되돌리고, 최종 테스트 파일만 올려 돌렸다. **61/61이 실패**한다(`test_results.json` `pre_fix`). 반례의 실패 메시지는 P1 `assert (1 == 2)`(시도 1개), 팀 상한 `assert 6 == 4`, P2 `assert (True is False)`(`tokens_complete`), run 분리 `assert 3 == 2`(run 이름 겹침)다. 대조군 4개는 새 속성 `unreported_attempts`가 없어서만 실패한다.
- **수정 후:** 61/61 통과. 관련 10개 모듈 **606 passed**(4차 545 + r5 61). tensorboard 없는 CI 작업을 흉내 내면 227 passed·3 skipped다(이벤트 파일 재읽기 3건만 건너뜀).
- **CI 수집:** `scripts/run_ci_tests.py`의 패턴이 210개 모듈을 고르고 r5 파일을 포함한다. r5 파일은 61개가 수집된다. CI의 `offline-regressions` 작업에는 tensorboard가 없다(`requirements-test.txt`). 그래서 이벤트 파일 재읽기만 `importorskip`이고, run 이름·중복·경로·기존 run 거절은 tensorboard 없이 검사한다. tensorboard import는 경로 검사 뒤로 옮겼다.
- **변이 검사:** 수정 조각 21개를 하나씩 되돌렸다. **20개가 r5 테스트를 실패시켰다**(`mutations.json`). 나머지 `M-r5-p2i`는 동치 변이다. 두 집계 경로가 모두 `tokens_complete`를 bool로 돌려주므로 옛 기본값 `.get(..., True)`는 도달하지 않는다. 그 줄은 수정이 아니라 도달 불가 기본값 정리다. 원복하면 61/61 통과한다.

## 스모크 출력 — v5를 만들지 않은 이유 (`smoke_invariance.json`)

세 수정은 오프라인 스모크 출력을 바꾸지 않는다.

- **P1:** 새 분기는 **사용량 미상이면서 예약보다 적은 시도**를 밝힌 응답에서만 탄다. 오프라인 fixture 전송 계층(`harness/zone_study_offline.py` `_FixtureTransport`·`run_call`)은 항상 사용량 확정·시도 1개 응답을 내고 내부 재시도 예약을 하지 않는다. v4 30회의 미상 호출은 0건이다.
- **P2 summary:** 스모크 기록에는 `calls`가 있어서 summary 전용 경로를 타지 않는다. 스모크 `model` 요약에는 두 표지가 없으므로 새 대조 항목은 건너뛴다.
- **P2 run 이름:** 스모크 실행기 `scripts/run_zone_study_offline_smoke.py`는 `scalar_export`를 부르지 않는다. 바뀌는 것은 나중에 만드는 **보고서·TensorBoard 변환**뿐이다.
- **확인:** `f44ea845` 코드로 메모리 안에서 두 시행을 돌렸다(파일 쓰기 없음, 스레드 1개). `peer_ko s1_normal_mixed s601`은 `code_sha`를 v4 실행 SHA로 표시하면 **기록 전체가 v4 예시(`example_trial_record.json.gz`)와 같다.** 이 시행과 horizon censor 3건이 있는 `no_comm` 같은 seed는 SIM trace 해시, 요청 입력 해시, 호출 수, censor 수가 v4 `results.json`과 같다. 두 시행 모두 `unreported_attempts`가 비어 있다.
  - 부하 평균은 전 `128.83 / 124.1 / 87.72`, 후 `124.67 / 123.36 / 87.88`이다.
  - 지정 grep 수는 15였다. 다른 작업의 실제 시뮬·평가 프로세스 6개, 공용 TensorBoard 1개, kiro-cli 에이전트 8개다.
  - 이 확인은 시뮬레이터를 import하지 않는 약 10초짜리 메모리 내 실행이다. 테스트가 도는 것과 같은 규모이고, 새 시뮬 세션은 띄우지 않았다. 다만 상한(6)인 상태에서 기다리지 않고 실행했으므로 그대로 적는다.

## 기존 스냅샷

v4 TensorBoard 스냅샷은 그대로 둔다. 위치는 `/Users/changmin/projects/ugrp/outputs/tensorboard/0926-zone-study-offline-smoke-v4`이다. 이 스냅샷은 옛 run 이름 `<condition>/<scenario>-s<seed>`와 scalars v1을 쓴다.

- `collection.json` SHA-256이 v4 `raw_index.json`에 적힌 `15eaacf9…`와 같다. 그보다 새로 바뀐 파일은 0개다.
- v4 `scalars.json`의 run 35개는 모두 이름이 다르다. 시행 30개는 (조건, 시나리오, seed)가 모두 달라서 v4에서는 run이 합쳐지지 않았다.
- 새 `write_events`는 이미 있는 run 디렉터리를 거절한다. 그래서 같은 logdir에 다시 써도 옛 스냅샷을 고치지 않는다(`:447` 테스트).

## 파일 길이 (600줄·80줄 규칙)

- `harness/zone_study_eval.py`는 1975줄, `harness/zone_event_scheduler.py`는 1066줄이다. 둘 다 의존 PR(#185, #186)이 소유한 패키지 파일이다. 이번 라운드의 수정 범위는 세 반례로 정해져 있어서 파일을 나누지 않았다. 나누면 열린 의존 PR과 병합 충돌이 커진다.
- 이번에 새로 만든 함수는 모두 30줄 이하다(`_reply_of` 27, `_fetch_reply` 27, `_summary_usage_marker` 23, `_run_paths` 24, `trial_run_name` 14). 80줄을 넘는 기존 함수는 두 개다. `model_aggregate`는 83줄에서 87줄이 됐다(summary 분기에 표지 2줄·주석 2줄). `efficiency_metrics`는 132줄에서 134줄이 됐다(기본값 한 줄을 주석과 함께 바꿨다). 둘 다 #185 소유 함수라 이번 범위에서 나누지 않았다.

## 남은 범위

- 실제 LLM 어댑터는 아직 연결하지 않았다. P1 규칙은 **준수하는 전송 계층이 시도마다 보내기 직전에 예약한다**는 계약에 기대고 있다. 예약하고 보내지 않은 전송 계층이 있으면, 미상일 때 과대 청구 쪽으로 기록된다. 과소 기록은 생기지 않는다. 실제 공급자 응답으로는 파일럿에서 따로 확인해야 한다.
- 보고서를 v4 raw에서 다시 만들면 run 이름이 바뀐다. 기존 스냅샷을 다시 쓰지 않으므로, 새 변환은 새 스냅샷 디렉터리로 해야 한다. 이번에는 새 결과가 없어서 변환하지 않았다.
- 4차 기록의 TensorBoard 병합 HParams 열 제한은 공용 logdir 문제로 남아 있다.

## 파일

| 파일 | 내용 |
|---|---|
| `test_results.json` | r5 61개의 수정 전·후 결과(사례별), 관련 모듈 606 통과, tensorboard 없는 실행, CI 수집 |
| `mutations.json` | 변이 21개의 원문·변이 문자열·결과 |
| `smoke_invariance.json` | 메모리 내 두 시행의 v4 대조, 부하 평균, 시뮬 수 확인 |

JUnit 원본(`/tmp/kiro-r5-*.xml`)은 임시 파일이다. 해시는 `test_results.json`에 남겼다.

## 참고 자료

- 논문
  - Donald B. Rubin, "Inference and missing data", *Biometrika* 63(3), 1976. https://doi.org/10.1093/biomet/63.3.581 — 미상 사용량을 확정 0으로 읽지 않고 결측 표지와 하한을 따로 두는 근거(4차와 같음). 이번에 새로 찾은 논문은 없다.
- OSS
  - TensorBoard 2.21.0 (Apache-2.0), https://github.com/tensorflow/tensorboard — 기존 `write_events`의 이벤트·HParams 기록과 `event_accumulator` 재읽기를 그대로 썼다. 추가 의존성은 없다.
  - pytest 9.1.1 (MIT), https://github.com/pytest-dev/pytest — `importorskip`, `tmp_path`, `parametrize`
- 내부 모듈
  - `harness/zone_event_scheduler.py`(`AttemptBudget.reserve/commit`, `PendingCall.reserve`, `TransportFailure`, `ReplayTransport._reserved`), `harness/zone_sim_cost.py`(`call_cost`, `censored_call_record`), `harness/zone_study_eval.py`(`model_aggregate`, `efficiency_metrics`, `summarise`, `scalar_export`, `load_trials`의 중복 `trial_id` 거절), `scripts/zone_study_report.py`(`write_events`, `build`, `token_cell`), `harness/zone_study_offline.py`(`_FixtureTransport`, `run_trial`), `scripts/run_ci_tests.py`
- 문서·웹
  - Codex 5차 검토 `codex-194-r5`, `docs/zone_sim_cost.md`, `docs/zone_study_metrics.md`, `docs/tensorboard.md`, `AGENTS.md`
- 채택하지 않은 대안
  - **예외가 나면 예약을 모두 돌려주기(기존 동작):** 이미 보낸 시도가 사라져 상한을 넘는다. 이것이 반례였다.
  - **보낸 시도 수를 전송 계층의 별도 카운터로만 받기:** 새 계약이 필요하다. 그리고 카운터를 안 주는 일반 예외는 여전히 과소 기록된다. 예약 자체가 "보내기 직전"이라는 기존 계약(2차 #15)을 쓰면 보수적으로 과대 기록된다.
  - **run 이름을 `<condition>/<scenario>-s<seed>-<n>` 순번으로 붙이기:** 파일을 읽는 순서에 따라 번호가 바뀐다. `trial_id`는 이미 유일성이 강제되고 원본 기록과 1:1로 이어진다.
  - **옛 스냅샷을 새 이름으로 다시 변환하기:** 기존 스냅샷 보존 지침에 어긋난다. 합쳐진 run도 없었다.
