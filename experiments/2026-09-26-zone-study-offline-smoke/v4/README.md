# v4 — Codex 4차 재검토 #16 반례 3건 수정 뒤 재실행 (2026-09-26)

- **상태:** 완료. 30회 모두 통과했다(`results.json` `ok: true`, 디스크 재해시 2168/2168).
- **실행 소스:** `c21a6fe9ddd33203aff4e34a0577e0d8adb95334` (브랜치 `kiro/zone-study-core`, PR 194). 이 SHA로 고정한 깨끗한 detached worktree에서 실행했고, 실행 뒤 그 worktree를 지웠다.
- **실행기:** `scripts/run_zone_study_offline_smoke.py` → `harness/zone_study_offline.py`. 보고서·TensorBoard 변환은 `scripts/zone_study_report.py`.
- **모델 호출·물리 실행·시뮬레이터 import는 모두 0회다.** 에이전트 잠금은 잡지 않았다(물리·학습·wall 시간 비교 없음).
- **v1(`../`)·v2(`../v2/`)·v3(`../v3/`)는 그대로 둔다.** 이 폴더는 새 버전이다.
- 부하 평균: 실행 명령 직전 `99.55 101.17 69.89`, 실행기 시작 시(`environment.json`) `48.33 79.57 75.18`, 종료 후 `35.57 72.43 72.77`. 실행 전 실제 시뮬·평가 프로세스가 6개여서 5개로 줄 때까지 기다렸다(`source.json` `machine_sim_cap`). 가짜 SIM 시계만 쓰므로 결과는 wall 시간의 영향을 받지 않는다.
- 첫 세션(계정 1)이 수정·실행·TensorBoard 등록까지 하고 사용량 한도로 멈췄다. 이 README와 `raw_index.json`의 커밋 파일 해시는 계정 2 세션이 이어서 작성했다. 그 전에 테스트·변이 표본·해시·예시 기록 재열기를 다시 확인했다.

## 수정 대상 — Codex 4차 재검토(`codex-194-r4`) #16의 남은 반례 3건

| # | 반례 | 수정 (`c21a6fe9`) | 회귀 테스트 (`tests/test_zone_study_review_fixes.py`) |
|---|---|---|---|
| 1 | 재시도 예산이 거절되면 `ReplayTransport`가 응답을 잘라 다시 만든다. 이때 `usage_known=False`가 `True`로 바뀌어 미상 호출 0건·확정 토큰 0이 된다 | 잘린 응답이 `usage_known`을 그대로 물려받는다. 보내지 않은 시도가 섞인 `provider_usage`는 버린다. 실제 전송 계층도 `TransportFailure(usage_known=False, attempts=…)`로 부분 사용량을 알릴 수 있다. 시도 기록이 없는 실패는 미상으로 둔다 | `test_r4_f16_a_budget_truncated_replay_keeps_the_unknown_usage_flag`, `test_r4_f16_a_transport_failure_can_declare_its_usage_incomplete` |
| 2 | 일부 attempt만 사용량을 아는 재시도가 horizon에서 censor되면 알려진 입력 833·출력 40이 0·0으로 사라진다 | `censored_call_record`가 알려진 토큰과 스케줄러 청구 SIM 비용을 남기고 `usage_bound='lower_bound'`로 표시한다. 확정 기록은 `exact`다 | `test_r4_f16_a_partly_known_retry_keeps_its_known_lower_bound_when_censored` |
| 3 | 알려진 120토큰 시행과 미상 시행을 섞으면 JSON에는 미상 1건·합계 `null`이 남는다. 그런데 보고서와 TensorBoard에는 120만 나오고 미상 표지가 없다 | 코호트 토큰 평균은 모든 시행이 확정일 때만 낸다. 하한 평균은 전체 시행 수로 나눈다. 보고서 토큰 열은 `≥하한 (미상 n)`과 주석으로 표시하고, 짝 비교는 제외 짝을 센다. TensorBoard에는 `result/usage_unknown_calls`·`cohort/usage_unknown_calls`·하한 태그와 HParams `tokens_complete`를 쓰고, 미상인 run에는 `tokens_total` 태그를 쓰지 않는다 | `test_r4_f16_the_report_and_tensorboard_mark_an_incomplete_token_total`, `test_r4_f16_the_tensorboard_event_files_carry_the_unknown_marker`(이벤트 파일을 다시 읽음), `test_r4_f16_a_cohort_token_mean_is_never_the_mean_of_the_known_trials_only`, `test_r4_f16_a_paired_comparison_counts_a_seed_dropped_for_unknown_usage` |

- **수정 전 재현:** `83f560d` 사본에 최종 테스트 파일만 올리면 r4 테스트 7건이 모두 실패한다(`mutations.json` `pre_fix`). 반례 2는 `assert (0, 0) == (833, 40)`로 실패한다.
- **변이 검사:** 수정 조각 18개를 하나씩 되돌렸고, 18개 모두 r4 테스트를 실패시켰다. 원복하면 7/7 통과한다(`mutations.json`). 계정 2 세션에서 그중 3개(반례 1·2·3 각 1개)를 새 사본에서 다시 돌렸고, 같은 결과(각 1 failed)를 얻었다.

## v3와의 대조 (`control_runs.json`)

`smoke.json`이 v3(`eb5572b`)와 **바이트 단위로 같다**(`6914fde4…`). 30/30 시행에서 SIM trace·호출·메시지·비용·채널이 같다. 달라진 것은 두 가지다. 호출 2168건에 `cost_terms.usage_bound` 필드가 추가됐고(모두 `exact`), `provenance.code_sha`가 바뀌었다. 코호트 요약에는 하한 필드 4개가 추가됐다. 이 코호트에는 사용량 미상 호출이 0건이라 하한 값이 확정 합계와 모두 같다. **따라서 반례 경로는 이 스모크가 아니라 단위 회귀 테스트가 검증한다.**

| 조건 | 시행 | 호출 | 발화 | 전달 edge | 사고 SIM s | 발화 SIM s | 종료 | censored 호출 | 미상 호출 | 토큰 합계 (= 하한) |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|
| `no_comm` 무통신 | 6 | 411 | 0 | 0 | 1397.7 | 0.0 | sim_horizon 6 | 3 | 0 | 3,271,821 |
| `peer_ko` 자유 한국어 | 6 | 540 | 18 | 36 | 1895.1 | 5.4 | budget_exhausted 6 | 0 | 0 | 4,447,548 |
| `leader_ko` 지휘 겸임 | 6 | 540 | 24 | 24 | 1899.9 | 7.2 | budget_exhausted 6 | 0 | 0 | 4,416,948 |
| `structured` 정형 | 6 | 540 | 18 | 36 | 1918.8 | 5.4 | budget_exhausted 6 | 0 | 0 | 4,503,924 |
| `reference_R` 참조 상한 | 6 | 137 | 0 | 0 | 467.7 | 0.0 | sim_horizon 6 | 0 | 0 | 1,079,479 |

`leader_ko` 지휘자는 `r2, r3, r1, r2, r3, r1` 순으로 순환한다. **이 수치는 통신 효과가 아니다.** fixture 응답 규칙과 청구 규칙이 만든 배선 수치다.

## 보고서 (`report_summary.md`)

v4에서 처음으로 `zone_study_report.py` 보고서를 만들고 TensorBoard로 변환했다(v1~v3는 변환하지 않았다). 보고서의 두 표시는 설계대로다.

- **"실행 번들 단일 여부: 아니오"(`map_file_sha256`·`order_sheet_sha256`·`public_map_sha256`):** 시나리오마다 지도·주문서가 다르기 때문이다. 시나리오 안에서는 해시가 하나다. 30회 전체에서 지도는 3종, 주문서는 6종이다.
- **입력 경계 "clean 24, unverified 6":** `unverified` 6건은 `reference_R`이다. R은 설계상 전 로봇 카메라를 받는 참조 상한이므로 주 조건 경계 판정에 합산하지 않는다. 주 4조건 24건은 모두 clean이다.

## TensorBoard (`tensorboard.json`)

- 스냅샷: `/Users/changmin/projects/ugrp/outputs/tensorboard/0926-zone-study-offline-smoke-v4` (run 35개 = 시행 30 + 코호트 5, scalar 789개).
- 보기 키 `zone_study_offline_smoke_v4_20260926`만 `outputs/tensorboard-view.json`에 추가했다. 고정 카드는 성공·makespan·호출 수·사고/발화 SIM 비용·토큰·**사용량 미상 호출**·발화다.
- 기존 공용 서버(PID 9291, 다른 작업 소유, 재시작하지 않음)의 응답을 대조했다. scalar 789개와 `scalars.json` 불일치 0건, raw 시행 기록 대조 120건 불일치 0건. 저장 링크를 headless Chrome으로 열어 35개 run이 선택되고 고정 카드 8개가 표시되는 것을 캡처했다(`outputs/zone-study-offline-smoke-v4-report/tensorboard-pinned-v4.png`).
- **한계:** 공용 logdir의 병합 HParams 실험은 다른 스냅샷의 요약에 있는 9개 열만 보여 준다. 그래서 `condition`·`scenario`·`leader_id`·`end_reason`·`tokens_complete` 열은 선택할 수 없다. 세션 35개의 값은 session_groups API로 확인했다(`tokens_complete=True` 35/35). 미상 표지는 Time Series의 `result/usage_unknown_calls`로 본다.

## 이 결과가 뜻하지 않는 것

v1~v3와 같다. 이 결과는 언어 이해, 조건 간 우열, 물리 운반 성공, 자기 카메라 인식을 입증하지 않는다. 물리가 없으므로 성공 시행은 0이다. 실제 LLM adapter의 실패·재시도·horizon 사용량 기록은 이번에 단위 테스트로 막았다. 다만 실제 공급자 응답으로는 파일럿에서 따로 확인해야 한다.

## 기록 정정 (`config.json` `corrections_ko`)

v4 `config.json`의 값은 실행 SHA에서 코드·파일로 다시 계산했다. 그 과정에서 과거 기록의 오류 두 개를 찾았다. 과거 파일은 해시가 기록되어 있으므로 고치지 않았다.

- v3 `config.json`의 `prompt_version`은 `…prompts_ko.v1`로 적혀 있다. 그러나 실행 코드 `eb5572b`의 값은 `v2`다(`02114ad`부터 v2).
- v1~v3 `config.json`의 `scenario_configs_sha256`는 `d87e0f2` 시점의 파일 해시다. 시나리오 파일 6개는 `dd497c8`에서 바뀐 뒤 `c21a6fe9`까지 그대로다. v2~v4 raw는 바뀐 파일로 실행됐다.

## 파일

| 파일 | 내용 |
|---|---|
| `results.json` | 30회 전체 결과(`smoke.json` 원문 바이트 복사) |
| `cohort_summary.json` | 조건별 집계. 미상 호출 수·불완전 시행 수·토큰 하한 포함 |
| `config.json` | 조건 registry, 비용 파라미터, 고정 프롬프트 토큰(seed 601), 시나리오·지도·프레임 해시, 정정 |
| `source.json` | 코드 SHA, 명령, 부하 평균, 시뮬 수 상한 확인, wall 시간 |
| `control_runs.json` | v3 대조 |
| `mutations.json` | 변이 18개, 수정 전 실패 7건 |
| `report_summary.md` | 보고서 원문 복사 |
| `tensorboard.json` | 스냅샷·링크·고정 태그·서버/UI 대조 |
| `example_trial_record.json.gz` | `peer_ko-s1_normal_mixed-s601` 전체 기록. 결정적 gzip이며 압축을 풀어 `reopen_trial_record`로 열면 재해시 90/90이다 |
| `sha256.json` | 실행기가 계산한 산출물 해시 |
| `raw_index.json` | raw·보고서·스냅샷의 위치·해시와 이 폴더의 커밋 파일 해시 |

raw 30건(57.5 MB, 요청 원문 포함)은 처음부터 기본 체크아웃의 `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v4/`에 썼다. 보고서는 `…/outputs/zone-study-offline-smoke-v4-report/`에 있다. 로컬 보관이며 원격 백업이 아니다. v1~v3 raw도 worktree `outputs/`에서 같은 상대 경로로 `mv`했다. 이동 전후 해시가 같았다(각 `raw_index.json`의 `relocation`).

## 참고 자료

- 논문
  - Bradley Efron, "Bootstrap Methods: Another Look at the Jackknife", *The Annals of Statistics* 7(1), 1979. https://doi.org/10.1214/aos/1176344552 — 짝 비교의 퍼센타일 부트스트랩 구간(`harness/zone_study_eval.py` `bootstrap_ci`)
  - Dave S. Kerby, "The Simple Difference Formula: An Approach to Teaching Nonparametric Correlation", *Comprehensive Psychology* 3, 2014. https://doi.org/10.2466/11.IT.3.1 — 짝 rank-biserial(`_rank_biserial`)
  - Donald B. Rubin, "Inference and missing data", *Biometrika* 63(3), 1976. https://doi.org/10.1093/biomet/63.3.581 — 사용량 미상 값을 0으로 채우거나 알려진 시행만 평균하지 않고, 결측 표지와 하한을 따로 두는 근거
- OSS
  - TensorBoard 2.21.0 (Apache-2.0), https://github.com/tensorflow/tensorboard — 이벤트 파일·HParams 기록과 `event_accumulator` 재읽기. 기존 `scripts/zone_study_report.py --tb-events` 경로를 그대로 썼다
  - protobuf 7.36.2 (BSD-3-Clause) — TensorBoard 이벤트 직렬화
- 내부 모듈
  - `harness/zone_event_scheduler.py`(`ReplayTransport`, `TransportFailure`, `CallReply`), `harness/zone_sim_cost.py`(`censored_call_record`), `harness/zone_study_eval.py`(`efficiency_metrics`, `summarise`, `compare_conditions`, TensorBoard 내보내기), `scripts/zone_study_report.py`, `scripts/run_zone_study_offline_smoke.py`, `harness/zone_study_offline.py`(`reopen_trial_record`), `scripts/ugrp_session.py`
- 문서·웹
  - `docs/tensorboard.md`, `docs/zone_sim_cost.md`, `docs/zone_study_metrics.md`, `AGENTS.md`
  - Chrome DevTools Protocol `Page.captureScreenshot`: https://chromedevtools.github.io/devtools-protocol/tot/Page/ — TensorBoard 화면 캡처
- 채택하지 않은 대안
  - 미상 토큰을 0으로 채우기, 또는 알려진 시행만 평균하기: 비용을 체계적으로 낮게 보고한다(반례 2·3과 같은 결함).
  - pandas/NumPy의 NaN 전파: 새 의존성이 생기고 JSON `null`과 이중 표현이 된다. 미상을 표시할 수도 없어서, 기존 `None` 규약에 명시 표지(`usage_bound`, `tokens_complete`)를 더했다.
