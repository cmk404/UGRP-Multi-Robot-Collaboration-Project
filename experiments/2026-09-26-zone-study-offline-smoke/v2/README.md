# v2 — Codex 적대적 검토 18건 수정 뒤 재실행 (2026-09-26)

- **상태:** 완료. 30회 전부 통과(`results.json` `ok: true`, 게이트 검사 30/30, 역류 probe 30/30).
- **실행 소스:** `dd497c82461d51d5aee8eb88a4c29b9e98a8c016` (브랜치 `kiro/zone-study-core`, worktree `ugrp-wt/kiro-study-core`)
- **실행기:** `scripts/run_zone_study_offline_smoke.py` → `harness/zone_study_offline.py`
- **모델 호출 0회, 물리 0회, 시뮬레이터 import 0회.** 실제 LLM도 MuJoCo도 쓰지 않았다.
- **v1(`../` 상위 폴더)은 그대로 보존한다.** 이 폴더는 v1을 덮어쓰지 않는 새 버전이며, v1의 결과·해시·README는 그대로 남아 있다.
- 부하 평균: 시작 전 `3.92 4.58 4.88`, 종료 후 `5.53 4.95 4.97`(동시 시뮬 프로세스 1개, 스레드 1개 고정).

## v1과 무엇이 다른가

수정 대상은 [Codex 적대적 검토 18건](../../../docs/design/2026-09-26-zone-study-packages-review-codex.md)이다. 검토는 입력 경계 우회, C–D 전달 인터페이스 불일치, 비용 누락, 조건별 평가 편향을 지적했다. 이 실행은 그 수정 뒤의 **같은 배선**을 같은 규모로 다시 돌린 것이다.

### 수치가 바뀌지 않은 부분 (배선 불변)

| 조건 | 시행 | 발화 | 전달 edge | 사고 SIM s | 발화 SIM s | 종료 | v1 대비 |
|---|---:|---:|---:|---:|---:|---|---|
| `no_comm` 무통신 | 6 | 0 | 0 | 1123.2 | 0.0 | sim_horizon 6 | 동일 |
| `peer_ko` 자유 한국어 | 6 | 18 | 36 | 1420.2 | 5.4 | budget_exhausted 6 | 동일 |
| `leader_ko` 지휘 겸임 | 6 | 24 | 24 | 1425.6 | 7.2 | budget_exhausted 6 | 동일 |
| `structured` 정형 | 6 | 18 | 36 | 1420.2 | 5.4 | budget_exhausted 6 | 동일 |
| `reference_R` 참조 상한 | 6 | 0 | 0 | 374.4 | 0.0 | sim_horizon 6 | 동일 |

`leader_ko`의 지휘자는 seed마다 순환하고 이번 여섯 seed도 r1/r2/r3를 모두 덮는다(`r2, r3, r1, r2, r3, r1`). fixture 응답 규칙과 비용 파라미터를 바꾸지 않았으므로 호출 수·SIM 비용이 같은 것이 정상이다. **이 수치는 통신 효과가 아니다.**

### 수치가 바뀐 부분

1. **SIM trace 해시** — 대화가 있는 18회(`peer_ko`·`leader_ko`·`structured`)의 `trace_sha256`이 v1과 다르다. 원인은 검토 2번 수정이다. 스케줄러가 자체 `call-<id>-m<n>` ID를 새로 만드는 대신 패키지 C가 만든 canonical `w1-<sender>-<n>`을 그대로 쓴다. ID를 가린 trace는 두 버전이 완전히 같음을 확인했다.

   ```
   PRE-FIX  {'no_comm': '7f996d13b3319a06', 'peer_ko': '21be83fc4d57750b', 'leader_ko': 'a618760aacbca1f3', 'structured': '21be83fc4d57750b', 'reference_R': 'ce15678f62296b53'}
   POST-FIX {'no_comm': '7f996d13b3319a06', 'peer_ko': '21be83fc4d57750b', 'leader_ko': 'a618760aacbca1f3', 'structured': '21be83fc4d57750b', 'reference_R': 'ce15678f62296b53'}
   ```

   (`s1_normal_mixed`, seed 601, 메시지 ID만 `<MID>`로 치환한 trace의 SHA-256 앞 16자리. 무통신·R은 발화가 없어 원래부터 같다.)

2. **주문서 해시** — 주문서의 `scenario_id`가 설명적 이름에서 불투명한 `scenario_ref`(`sc_<12 hex>`)로 바뀌었다(검토 17번). 따라서 `order_sheet_sha256`과 호출 기록의 `input_sha256`이 v1과 다르다. 이름↔ref 대응은 평가 전용 manifest(`OrderSheetSource.manifest()`)에 남는다.

3. **새로 기록되는 값** — v1에는 없던 항목이다.
   - `trials[].cost.attempt_budget`: HTTP attempt 예약·사용·거절(검토 15번). 이번 실행의 사용량은 `no_comm` 432, 대화 3조건 각 540, `reference_R` 144이며 팀 상한 90/시행·actor 상한 30/시행을 넘지 않았다.
   - `trials[].cost.billed_utterances` / `produced_utterances`: 청구 발화 수와 실제 생성 발화 수가 30회 모두 같다(검토 6번). 이번 실행에서 거절된 발화는 0건이다.
   - `trials[].cost.censored_calls` / `censored_elapsed_sim_s`: horizon에서 끝나지 않은 호출(검토 16번). **300 SIM초 horizon에서는 0건**이다. 이 경로는 단위 테스트에서 확인했다(`tests/test_zone_study_review_fixes.py::test_f16_*`, 40초 horizon에서는 실제로 발생한다).
   - `scheduler.censored_calls`: 같은 값의 스케줄러 쪽 집계.
   - `config.json`의 `fixed_prompt_tokens` / `fixed_prompt_token_spread`: 조건별 고정 프롬프트 토큰(검토 18번). 공통 블록 차이는 0이고 남은 차이 221 토큰은 전부 채널 절이다.

## 검사한 것 (모두 통과)

v1과 같은 검사에 아래가 추가됐다. 전체 목록과 통과 근거는 `results.json`의 `trials[].channel`·`trials[].cost`와 `../README.md`를 함께 본다.

1. **단일 메시지 버스** — 패키지 C의 inbox 수와 패키지 D 스케줄러의 inbox 수가 30회 모두 일치한다(`inbox_agrees_with_scheduler`). C는 더 이상 스스로 배달하지 않고, `Transport(delivery_owner='sim_scheduler')` + `commit_delivery`로 스케줄러만 inbox를 바꾼다.
2. **비용 회계** — 완료 호출의 `released − requested == sim_cost_s`이고, 완료 호출 비용 합계가 스케줄러의 actor별 사고 시간 합계와 같다. censored 호출의 경과 SIM 시간은 별도 항목이며 토큰은 0으로 남는다.
3. **HTTP 예산** — `attempt_budget.used_total`이 호출 로그의 attempt 합계와 같고, 팀·actor 상한을 넘지 않았다.
4. **발화 청구 일치** — 조건별 `billed_utterances == produced_utterances`.
5. **역류 없음** — 숨은 사건 절을 바꿔 다시 돌린 30회 모두 `calls`·`messages`·`actions`·요청 해시·trace가 동일하다.
6. **actor 격리** — fixture actor는 자기 요청만 받는다(`actor_isolation.ok`).

## 이 결과가 뜻하지 않는 것

v1과 같다. 언어 이해, 통신 효과(조건 간 우열), 물리 운반 성공, 자기 카메라 인식 성능을 입증하지 않는다. 물리가 없으므로 성공 시행은 0이고 `censored_trials`는 시행 수와 같다. fixture 응답은 명시적인 오프라인 대역이며 모델 실패의 대체가 아니다. 검토 18건을 수정했다는 것과 본실험 투입 가능이라는 것은 다르다. 남은 확인 사항은 상위 README의 "남은 검증"과 PR 194 본문을 따른다.

## 파일

| 파일 | 내용 |
|---|---|
| `results.json` | 30회 전체 결과(`smoke.json` 원문). 조건·시나리오·seed별 채널·비용·게이트 검사 |
| `cohort_summary.json` | 조건별 집계와 provenance 혼재 여부 |
| `config.json` | 조건 registry 해시, 비용 파라미터, 입력 프로필, 고정 프롬프트 토큰, 검토 수정 참조 |
| `source.json` | 코드 SHA, 명령, worktree, v1 기록 위치, 부하 평균 |
| `environment.json` | 실행 당시 파이썬·패키지 버전 |
| `example_trial_record.json` | `peer_ko-s1_normal_mixed-s601` 한 건의 전체 시행 기록 |
| `sha256.json` | 실행기가 계산한 산출물 해시 |
| `raw_index.json` | raw 30건 trial record의 로컬 위치와 해시(Git에는 넣지 않는다) |

raw 30건은 `outputs/zone-study-offline-smoke-v2/trial_records/`(6.0 MB)에 로컬 보관이며 원격 백업이 아니다.
