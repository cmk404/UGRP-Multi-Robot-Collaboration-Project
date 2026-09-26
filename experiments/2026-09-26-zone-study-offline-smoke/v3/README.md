# v3 — Codex 3차 재검토 부분 해결 4건 수정 뒤 재실행 (2026-09-26)

- **상태:** 완료. 30회 모두 통과했다(`results.json` `ok: true`, 게이트 검사 30/30, 역류 probe 30/30, **디스크 재해시 2168/2168**).
- **실행 소스:** `eb5572bd9f34744373d02b63e1033979f0a0a920` (브랜치 `kiro/zone-study-core`, PR 194)
- **실행기:** `scripts/run_zone_study_offline_smoke.py` → `harness/zone_study_offline.py`
- **모델 호출·물리 실행·시뮬레이터 import는 모두 0회다.**
- **v1(`../`)과 v2(`../v2/`)는 그대로 보존한다.** 이 폴더는 두 버전을 덮어쓰지 않는 새 버전이다.
- 부하 평균: 시작 전 `175.79 189.57 164.78`, 종료 후 `122.60 121.75 137.44`. 동시에 돈 시뮬 프로세스는 1개이고 스레드는 1개로 고정했다. 다른 에이전트 작업으로 부하가 높았다. 이 실행은 가짜 SIM 시계만 쓰므로 결과는 wall 시간의 영향을 받지 않는다.

## 수정 대상

Codex 3차 재검토(`codex-194-r3`)에서 부분 해결로 판정한 4건이다.

| # | 지적 | 수정 |
|---|---|---|
| 1 | 저장 기록의 요청 archive가 네 필드뿐이라 디스크에서 재해시할 수 없음 | `request_archive`에 system·user·image_refs·토큰 수 전체를 저장한다. `reopen_trial_record`는 파일을 다시 읽어 digest와 토큰 수를 재계산한다. 스모크도 저장 직후 30건 전부를 다시 연다(`results.json` `disk_rehash`). |
| 1+ | 실제 provider 사용량과 표준화 청구 크기가 분리되지 않음 | archive에 `tokens`(실제 텍스트의 고정 토크나이저 계수), `billed_tokens`(SIM 청구 크기), `provider_usage`(공급자 보고)를 따로 둔다. 오프라인의 `provider_usage`는 `null`이다. 호출 기록에는 `cost_terms.provider_usage`가 들어간다. |
| 11 | fungible 물건의 오배송 이력이 빠져 `red_1: C→B`가 복구 0건으로 집계됨 | kind 기준 허용 구역을 벗어난 적재도 이력에 남긴다. 그래서 배송 1건·복구 1건으로 집계된다. `misplacement_events`도 따로 센다. |
| 12 | 한 문장이 order·kind·개체 id를 함께 말하면 주장이 중복되거나 인코딩별 판정이 갈림 | 두 인코딩 모두 지칭을 정규 대상(개체 > 주문 > kind)으로 통일한다. 한 문장에서 한정어로만 쓰인 주문·kind는 별도 주장으로 세지 않는다. |
| 16 | 이미 회수한 일반 예외의 `usage_known=False`가 censor 처리에서 `True`로 바뀜 | `CallReply.usage_known`을 호출 원장·censored 행·A 호출 기록·시행 지표·코호트 요약까지 보존한다. 미상이면 토큰 합계는 `None`이 되고 하한값만 따로 남긴다. |

변이 검사 11개는 모두 해당 테스트를 실패시켰다(`mutations.json`).

## v2와 무엇이 다른가

v2는 `dd497c8`에서 실행했다. 그 뒤 2차 재검토 수정(`02114ad`)이 입력 토큰을 실제 요청 계수로 청구하도록 바꿨다. 그래서 v3의 호출 수·SIM 사고 비용·토큰 수는 v2와 다르다. **이 차이는 3차 수정 때문이 아니다.** 원인을 가르기 위해 두 대조 실행을 했다(`control_runs.json`).

- **`02114ad`(2차 수정, 3차 수정 전)와 비교:** 30/30 시행에서 SIM trace, 요청 입력 해시, 비용 검사, 채널 검사가 모두 같다. 기록에서 달라진 것은 세 가지다. `request_archive`가 전체 요청으로 늘었고, `calls[].cost_terms`에 `usage_known`과 `provider_usage`가 추가됐고, `code_sha`가 바뀌었다.
- **`dc8ac28`(3차 첫 커밋)과 비교:** 30/30 시행이 `code_sha`를 빼면 완전히 같다. `eb5572b`의 변경은 코호트 요약 필드뿐이다.

| 조건 | 시행 | 호출 | 발화 | 전달 edge | 사고 SIM s | 발화 SIM s | 종료 | censored 호출 |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| `no_comm` 무통신 | 6 | 411 | 0 | 0 | 1397.7 | 0.0 | sim_horizon 6 | 3 |
| `peer_ko` 자유 한국어 | 6 | 540 | 18 | 36 | 1895.1 | 5.4 | budget_exhausted 6 | 0 |
| `leader_ko` 지휘 겸임 | 6 | 540 | 24 | 24 | 1899.9 | 7.2 | budget_exhausted 6 | 0 |
| `structured` 정형 | 6 | 540 | 18 | 36 | 1918.8 | 5.4 | budget_exhausted 6 | 0 |
| `reference_R` 참조 상한 | 6 | 137 | 0 | 0 | 467.7 | 0.0 | sim_horizon 6 | 0 |

`leader_ko`의 지휘자는 `r2, r3, r1, r2, r3, r1` 순으로 순환한다. `no_comm`에서는 300초 horizon에 호출 3건이 끝나지 않아 censored로 남았다. 사용량 미상 호출은 모든 조건에서 0건이다(`cohort_summary.json` `usage_unknown_calls`). **이 수치는 통신 효과가 아니다.** fixture 응답 규칙과 청구 규칙이 만든 배선 수치다.

## 이 결과가 뜻하지 않는 것

v1·v2와 같다. 이 결과는 언어 이해, 조건 간 우열, 물리 운반 성공, 자기 카메라 인식을 입증하지 않는다. 물리가 없으므로 성공 시행은 0이다. 4건 수정이 곧 LLM 파일럿 진입 승인은 아니다. 실제 LLM adapter의 실패 응답·재시도·horizon 종료 처리와 `provider_usage` 기록은 파일럿 연결 때 따로 확인해야 한다.

## 파일

| 파일 | 내용 |
|---|---|
| `results.json` | 30회 전체 결과(`smoke.json` 원문)와 `disk_rehash` 요약 |
| `cohort_summary.json` | 조건별 집계. 사용량 미상 호출 수·토큰 불완전 시행 수 포함 |
| `config.json` | 조건 registry, 비용 파라미터, 고정 프롬프트 토큰(seed 601), archive 필드 |
| `source.json` | 코드 SHA, 명령, 부하 평균, wall 시간 |
| `control_runs.json` | `02114ad`·`dc8ac28` 대조 결과 |
| `mutations.json` | 변이 11개와 pytest 결과 |
| `example_trial_record.json.gz` | `peer_ko-s1_normal_mixed-s601` 전체 기록(요청 원문 90건 포함). 결정적 gzip이며 압축을 푼 파일을 `reopen_trial_record`로 다시 열면 재해시 90/90이다. |
| `sha256.json` | 실행기가 계산한 산출물 해시 |
| `raw_index.json` | raw 30건의 위치·해시와 커밋 파일 해시 |

raw 30건(57.4 MB, 요청 원문 포함)은 기본 체크아웃의 `/Users/changmin/projects/ugrp/outputs/zone-study-offline-smoke-v3/`에 있다. `dc8ac28` 실행 raw는 같은 폴더의 `zone-study-offline-smoke-v3-run-dc8ac28/`에 있다. 둘 다 처음에는 worktree의 `outputs/`에 썼고, 2026-09-26에 같은 상대 경로로 `mv`했다. 이동 전후 해시가 같았다(`raw_index.json`의 `relocation`). 로컬 보관이며 원격 백업이 아니다.

예시 기록을 다시 여는 방법:

```bash
gunzip -c experiments/2026-09-26-zone-study-offline-smoke/v3/example_trial_record.json.gz > /tmp/ex.json
PYTHONPATH=. .venv-sim-worker-mac/bin/python -c "from harness import zone_study_offline as o; print(o.reopen_trial_record('/tmp/ex.json')['ok'])"
```
