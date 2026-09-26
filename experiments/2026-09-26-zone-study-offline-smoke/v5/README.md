# v5 — Codex 6차 검토·통합 #229 발견 수정 뒤 재실행 (2026-09-26)

- **상태:** 완료. 30회 모두 통과했다(`results.json` `ok: true`, 디스크 재해시 1871/1871, 비공개 절 역류 검사 30/30, actor 격리 통과).
- **실행 소스:** `8ab8f7edbc78e7cb27985dfe81dde092823e01b1` (브랜치 `kiro/zone-study-core`, PR 194). 이 SHA에서 추적 파일 변경 0개인 자기 worktree(`ugrp-wt/kiro-study-core`)로 실행했다. v4와 달리 별도 detached worktree를 만들지 않았다.
- **실행기:** `scripts/run_zone_study_offline_smoke.py` → `harness/zone_study_offline.py`. 보고서·TensorBoard 변환은 `scripts/zone_study_report.py`.
- **모델 호출·물리 실행·시뮬레이터 import는 모두 0회다.** 에이전트 잠금은 잡지 않았다(물리·학습·wall 시간 비교 없음).
- **v1(`../`)~v4(`../v4/`)는 그대로 둔다.** 이 폴더는 새 버전이다.
- 부하 평균: 실행 직전 `4.12 3.90 4.56`, 실행기 시작 시(`environment.json`) `3.98 3.92 4.5`, 종료 후 `3.16 3.70 4.39`. 지정 grep은 8개였다. 이 중 실제 시뮬·평가 프로세스는 다른 작업의 1개뿐이어서 상한(6) 아래에서 시작했다(`source.json` `machine_sim_cap`). 가짜 SIM 시계만 쓰므로 결과는 wall 시간의 영향을 받지 않는다.

## 왜 새 버전인가

v4까지 오프라인 루프는 행동마다 재질문 타이머를 **하나씩 더** 걸었다. 통합 PR #229가 이 문제를 찾았다(이슈 #222). 메시지로 시작된 호출이 타이머 사슬을 늘려서 채널 3조건은 모두 호출 예산 90회를 다 썼다(v4에서 18/18 `budget_exhausted`). 6차 수정(`8ab8f7ed`)은 로봇마다 대기 중인 재질문 타이머를 하나로 제한한다(`REASK_POLICY = 'single_pending_own_timer.v1'`). 이 규칙은 스모크의 SIM trace를 바꾼다. 실행 번들 ID(`zone_study_offline_v1` → `v2`), 계약 버전(`v1` → `v2`), 조건 registry 해시(`f1ff6a49…` → `c9bb5556…`)도 바뀌었으므로 새 버전으로 실행했다. 수정 내용과 회귀 테스트는 [`../review-r6/`](../review-r6/README.md)에 있다.

**frozen 코호트 의존 확인:** 옛 규칙에 기대는 동결 기록은 오프라인 스모크 v1~v4뿐이다. 그 기록은 모두 `zone_study_offline_v1`로 표시되어 있고 이 저장소에서 다시 쓰지 않는다. 다른 브랜치에서 `zone_study_offline_v*`를 쓰는 곳은 통합 브랜치(`kiro/zone-study-integration`)뿐이다(`git grep`, 모든 원격 브랜치). 통합 브랜치는 자기 번들 ID(`zone-study-integration-v1`)와 지역 우회 규칙을 쓴다. 이 규칙은 이번 수정과 같은 식별자다.

## v4와의 대조 (`control_runs.json`)

| 조건 | 시행 | 호출 v4 → v5 | 발화 | 전달 edge | 사고 SIM s v4 → v5 | 발화 SIM s | 종료 v4 → v5 | censored 호출 | 미상 호출 | 토큰 합계 v4 → v5 | SIM trace 동일 |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| `no_comm` 무통신 | 6 | 411 → 411 | 0 | 0 | 1397.7 → 1397.7 | 0.0 | horizon 6 → horizon 6 | 3 → 3 | 0 | 3,271,821 → 3,271,821 | 6/6 |
| `peer_ko` 자유 한국어 | 6 | 540 → **447** | 18 | 36 | 1895.1 → 1549.5 | 5.4 | budget 6 → **horizon 6** | 0 → 6 | 0 | 4,447,548 → 3,675,471 | 0/6 |
| `leader_ko` 지휘 겸임 | 6 | 540 → **429** | 24 | 24 | 1899.9 → 1491.6 | 7.2 | budget 6 → **horizon 6** | 0 → 6 | 0 | 4,416,948 → 3,502,711 | 0/6 |
| `structured` 정형 | 6 | 540 → **447** | 18 | 36 | 1918.8 → 1557.6 | 5.4 | budget 6 → **horizon 6** | 0 → 9 | 0 | 4,503,924 → 3,721,803 | 0/6 |
| `reference_R` 참조 상한 | 6 | 137 → 137 | 0 | 0 | 467.7 → 467.7 | 0.0 | horizon 6 → horizon 6 | 0 | 0 | 1,079,479 → 1,079,479 | 6/6 |

- `no_comm`과 `reference_R`의 12개 시행은 SIM trace 해시와 요청 입력 해시가 v4와 같다. 호출 기록도 `provenance`만 다르다(`code_sha`, `execution_bundle_id`, `registry_sha256`). 이 조건에는 메시지로 시작된 호출이 없어서 로봇마다 타이머 사슬이 하나뿐이었다. 그래서 새 규칙이 아무것도 건너뛰지 않았다.
- 채널 3조건은 시행당 호출이 90회에서 72~75회로 줄었다. 모든 시행이 호출 예산을 남기고 300 SIM s horizon까지 갔다. 그 결과 horizon에서 진행 중이던 호출이 censored로 남는다(6·6·9건).
- 발화 수, 전달 edge, 자유 문장, follower↔follower 0, 한국어 준수는 v4와 같다. `leader_ko` 지휘자는 `r2, r3, r1, r2, r3, r1` 순으로 순환한다.
- 이 코호트에는 사용량 미상 호출이 0건이다. `submit()` 실패 정산과 `sent_attempts`는 fixture 전송 계층에서 일어나지 않으므로 단위 테스트가 검증한다.
- **이 수치는 통신 효과가 아니다.** fixture 응답 규칙, 청구 규칙, 재질문 규칙이 만든 배선 수치다. v4에서 v5로 줄어든 호출 수도 규칙 수정의 결과이며, 조건 간 우열이 아니다.

## 보고서 (`report_summary.md`)

`zone_study_report.py`로 보고서를 만들고 TensorBoard로 변환했다. 입력 경계 판정은 clean 24건, unverified 6건이다. unverified 6건은 설계상 전 로봇 카메라를 받는 참조 상한 `reference_R`이다. 주 4조건 24건은 모두 clean이다. "실행 번들 단일 여부: 아니오"는 v4와 같은 이유다. 시나리오마다 지도·주문서가 다르다.

## TensorBoard (`tensorboard.json`)

- 스냅샷: `/Users/changmin/projects/ugrp/outputs/tensorboard/0926-zone-study-offline-smoke-v5`. run은 35개(시행 30 `<condition>/<trial_id>` + 코호트 5)이고 scalar는 789개다. v4 스냅샷은 건드리지 않았다.
- 보기 키는 `zone_study_offline_smoke_v5_20260926` 하나만 `outputs/tensorboard-view.json`에 추가했다. 쓰기 직전에 파일을 다시 읽었고, 다른 키의 값과 순서가 같은지 확인했다. 필터 `^0926-zone-study-offline-smoke-v(4|5)/`는 v4 기준선을 함께 보여 준다. 고정 카드는 8개다: 성공, makespan, 호출 수, 사고·발화 SIM 비용, 토큰, 사용량 미상 호출, 발화.
- **확인:** 기존 공용 서버(PID 9291, 다른 작업 소유, 재시작하지 않음)가 v5 run 35개를 목록에 올렸다. scalar 789개를 `scalars.json`과 대조했고 불일치는 0건이다. raw 시행 기록과 `result/model_calls`도 30/30 같다. 시행 run 30개에 고정 태그 8개가 모두 있다. 저장 링크를 headless Chrome으로 열어 v5 35개·v4 35개 run이 선택되고 고정 카드 8개가 표시되는 것을 확인하고 캡처했다(`outputs/zone-study-offline-smoke-v5-report/tensorboard-pinned-v5.png`). 캡처 도구는 PR #229의 `tb_capture.py`를 /tmp에 복사해 썼다.
- **한계:** v4와 같다. 공용 logdir의 병합 HParams 실험은 다른 스냅샷의 9개 열만 보여 준다. 그래서 설정한 열(`condition`·`scenario`·`seed`·`leader_id`·`end_reason`·`tokens_complete`)을 고를 수 없다. v5 세션 35개는 session_groups API로 확인했다. 수치는 Time Series로 본다.

## 이 결과가 뜻하지 않는 것

v1~v4와 같다. 이 결과는 언어 이해, 조건 간 우열, 물리 운반 성공, 자기 카메라 인식을 입증하지 않는다. 물리가 없으므로 성공 시행은 0이다. 실제 LLM 어댑터의 `submit()` 전송 여부 판정(`NotSent`)과 `sent_attempts` 보고는 파일럿에서 실제 공급자로 따로 확인해야 한다.

## 파일

| 파일 | 내용 |
|---|---|
| `results.json` | 30회 전체 결과(`smoke.json` 원문 바이트 복사) |
| `cohort_summary.json` | 조건별 집계(raw 바이트 복사) |
| `config.json` | 계약 v2·registry 해시(v1 재현값 포함)·번들 ID·재질문 규칙·호출 정책·비용 파라미터·고정 프롬프트 토큰·시나리오·지도·프레임 해시와 v4 대비 변경 |
| `source.json` | 코드 SHA, 명령, 부하 평균, 시뮬 수 상한 확인, wall 시간, 새 버전 이유 |
| `control_runs.json` | v4 대조(조건별·시행별) |
| `report_summary.md` | 보고서 원문 복사 |
| `tensorboard.json` | 스냅샷·링크·고정 태그·서버/UI 대조 |
| `example_trial_record.json.gz` | `peer_ko-s1_normal_mixed-s601` 전체 기록. 결정적 gzip이며 압축을 풀어 `reopen_trial_record`로 열면 재해시 75/75다 |
| `environment.json`, `sha256.json` | 실행기 환경과 실행기가 계산한 산출물 해시(raw 바이트 복사) |
| `raw_index.json` | raw·보고서·스냅샷의 위치·해시와 이 폴더의 커밋 파일 해시 |

raw 30건(요청 원문 포함)은 처음부터 기본 체크아웃의 `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v5/`에 썼다. 보고서는 `…/outputs/zone-study-offline-smoke-v5-report/`에 있다. 로컬 보관이며 원격 백업이 아니다.

## 참고 자료

- 논문: 이번 버전을 위해 새로 찾은 논문은 없다. 짝 비교·결측 처리 근거는 v4와 같다(Efron 1979, Kerby 2014, Rubin 1976 — [v4 README](../v4/README.md#참고-자료)).
- OSS
  - TensorBoard 2.21.0 (Apache-2.0), https://github.com/tensorflow/tensorboard — 기존 `write_events`의 이벤트·HParams 기록과 HTTP API(`/data/runs`, `/data/plugin/scalars/*`, `/data/plugin/hparams/*`) 대조
  - websockets 17.1 (BSD-3-Clause), https://github.com/python-websockets/websockets — `tb_capture.py`의 Chrome DevTools Protocol 연결
- 내부 모듈·PR
  - `harness/zone_study_offline.py`(`run_smoke`, `reopen_trial_record`), `scripts/run_zone_study_offline_smoke.py`, `scripts/zone_study_report.py`, `scripts/ugrp_session.py`
  - PR #229 `harness/zone_study_integration.py` `_arm_reask`(재질문 규칙의 원래 우회)와 `experiments/2026-09-26-zone-study-integration/tb_capture.py`(캡처 도구)
- 문서·웹
  - `docs/tensorboard.md`, `docs/zone_sim_cost.md`, `AGENTS.md`
  - Chrome DevTools Protocol `Page.captureScreenshot`, `Runtime.evaluate`: https://chromedevtools.github.io/devtools-protocol/tot/Page/
