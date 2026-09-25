# PR #172 한국어 대화 파일럿 — 리뷰 후속 수정 (2026-09-26)

- **작성:** Kiro CLI. 브랜치 `kiro/zone-ko-pilot-fixes`(접두사 `kiro/`는 Kiro 작업), 기준 `origin/main=5288933`.
- **대상 지적:** [PR 검토(Codex, 2026-09-25)](2026-09-25-zone-pr-review-codex.md)의 `PR #172 — zone-dialogue-ko-pilot` 절 3건과 그 절이 요구한 후속 검증.
- **실행 범위:** 오프라인. 모델 호출·SIM·학습 없음. 기록된 `experiments/2026-09-25-zone-dialogue-ko-pilot/`의 `results.json`·`calls.jsonl`·`dialogues.json`·`manual_labels.json`과 `outputs/`의 원시 wire는 **읽기만** 했다. PR #172는 이미 main에 병합됐으므로(병합 커밋 `6d80e3e`) 수정은 새 커밋·새 파일로 남겼다.

## 1. 지적별 수정

### (a) [high] 중단된 대화 창을 재개하면 원시 wire를 덮어쓴다

원인은 두 곳이었다. `call_model()`이 `call_id`와 `attempt`만으로 파일명을 만들고 `write_bytes()`로 무조건 썼고, `cmd_dialogue()`는 완성된 창만 완료로 판단해 중단된 창을 같은 call ID로 처음부터 다시 돌렸다.

- 원시 파일은 `reserve_raw()`가 **존재하지 않는 경로**를 골라 `open('xb')`로만 만든다(`scripts/pilot_korean_dialogue.py`). 이미 있으면 `-x1`, `-x2` 같은 occurrence 이름을 쓰고, 경합으로 파일이 생겨 있으면 `SystemExit`으로 멈춘다. 즉 재개는 **어떤 경우에도 기존 요청·응답 기록을 지우거나 덮어쓰지 않는다.** 실제 사용한 occurrence는 `calls.jsonl`의 `raw_occurrence`에 남는다.
- 중단된 창은 같은 call ID로 재생하지 않는다. `window_episode()`가 다음 **episode** 번호를 골라 call ID를 `dialogue:M1:V2:e2:t1:r1` 형태로 바꾼다. episode 1은 기존 ID 형식을 그대로 유지하므로 2026-09-25 기록의 주소가 바뀌지 않는다(`episode` 키가 없는 행·창 기록은 episode 1로 읽는다).
- 턴마다 `dialogues-partial.json`에 체크포인트한다. 중단 지점과 이미 쓴 턴이 남고, 이전 episode의 부분 기록도 지우지 않는다.
- `analyze()`는 기존 결과 파일이 있으면 중단한다. 새 집계는 `--out`에 새 이름을 줘야 쓴다(같은 유형의 증거 덮어쓰기 방지).

**이 실행에 대한 영향:** 없다. `calls.jsonl` 124행의 call ID와 원시 wire/response 경로가 모두 서로 다르고 attempt 1(재시도)이 없어, 기록된 실행에서는 덮어쓰기가 발생하지 않았다. 원시 디렉터리 `outputs/zone-dialogue-ko-pilot-20260925/`는 gitignore이므로 파일 단위 확인은 로컬 범위다.

### (b) [medium] README의 권고·인과 주장이 결과보다 강하다

`experiments/2026-09-25-zone-dialogue-ko-pilot/README.md`의 수치는 그대로 두고 해석만 좁혔다.

- 충돌 0건을 직렬화의 효과로 단정하던 문장을 **가능한 설명**으로 바꿨다. 같은 프롬프트·언어에서 동시/직렬을 비교하지 않았고, 충돌이 기록된 창은 M1 하나뿐이며, M1 비교에서는 프롬프트(영어→한국어)와 순서(동시→순차)가 함께 바뀌었다는 점을 명시했다.
- 출력 256토큰 권고를 모드별로 나눴다. 창(claim, V2/V3) 36턴은 모두 256 이하(최대 215)지만, 단일 턴 plan_first는 V2 20건 중 6건이 256을 넘고 최대 365토큰(스키마 유효)이다. V0 6건/324, V1 5건/313, V3 5건/395. 따라서 계획 JSON을 같은 응답에 담는 경로에는 256을 적용할 수 없고 원본 `max_tokens=1400`을 유지하거나 모드별로 다시 측정해야 한다고 적었다.
- 헤드라인의 "한국어로 안정적" 같은 표현을 측정값(19/20, 한글 비율 ≥0.9)으로 바꾸고, 표본 크기와 신뢰구간 부재를 앞쪽에 명시했다. V2 채택 권고는 "잠정"으로 표시했다(n=20, 차이 3건).
- 재생 대조의 "claim은 안정적, plan은 잡음이 크다"는 결론을 8쌍 관찰의 방향으로 낮췄다.
- 창당 약 42k 토큰은 6턴·이미지 5장·이 지도 조건에서만 성립한다고 범위를 붙였다.

### (c) [low] 대화 행위 규칙이 언급과 실제 행위를 혼동한다

`harness/zone_dialogue_metrics.py`에 규칙을 **버전으로 분리**했다.

- `ACT_RULES_V1`은 동결한다. 이 규칙이 `results.json`(스키마 `ugrp.zone_dialogue_ko_pilot.results.v1`)의 `acts`를 만들었다. `ACT_RULES`는 v1을 가리키는 호환 이름으로 남겼다.
- `ACT_RULES_V2`는 `(cue, suppress)` 쌍이다. 라틴 문자 단서에 단어 경계를 넣고, 부정(`NEG_AGREE`: disagree, do not agree, 동의하지 않, 수락할 수 없)과 **남의 행위 언급**(accepting/awaiting a proposal, `r2 takes A`, claimed by r2)을 억제 구간으로 처리한다. cue 일치 구간이 억제 구간과 겹치면 그 라벨을 붙이지 않는다. 각 행위의 정의는 `ACT_DEFINITIONS`에 한국어로 명시했다.
- `dialogue_acts(text, rules='v2')`가 기본이고 `rules='v1'`로 과거 라벨을 재현한다. 분석기는 `acts`(v2)와 `acts_rules_v1`을 함께 기록하고 `act_rules` 블록에 버전을 남긴다.
- 어휘 불일치를 명시했다. V3 구조 메시지의 `act` 열거값(`harness/zone_dialogue_ko.ACTS`, 9종)에는 `propose`·`standby`가 없다. `STRUCTURED_ACTS`·`ACTS_FREE_TEXT_ONLY`로 드러내고, 테스트가 `STRUCTURED_ACTS == zk.ACTS`와 부분집합 관계를 확인한다. V3 분포를 자유 텍스트 분포와 직접 비교하지 않는다.

**검증(오프라인, 기록된 메시지 83건):**

| 표본 | 라벨 작성 | v1 완전 일치 | v2 완전 일치 |
|---|---|---|---|
| `manual_labels.json` 20개(한국어 V1/V2·V2 창) | 파일럿 작성자, 2026-09-25 (규칙 v2보다 먼저 만든 표본) | 18/20 | **20/20** |
| `manual_labels_en.json` 12개(영어 V0) | Kiro, 2026-09-26, 비맹검 단독 | 6/12 | **10/12** |

- v1→v2로 라벨이 달라진 메시지는 83건 중 9건이다. `propose` 총계 20→16, `request` 13→15, `agree` 17→16, `other` 2→1.
- 남은 미스 2건은 "All goals have been achieved / are complete … Standing by."처럼 **완료 상태 보고**를 `report`로 잡지 못한 것이다. 표본에 맞춰 규칙을 덧붙이지 않고 `act_rules_v2_check.json`의 `rule_misses`로 남겼다. 예외는 하나로, `please accept`(상대에게 수락을 요청하는 발화)를 `agree`에서 빼는 억제는 영어 표본을 본 뒤 추가했고 산출물의 `design_note`에 적었다.
- 비교는 `scripts/pilot_korean_dialogue_act_recheck.py`로 다시 만들 수 있고, 규칙 자체의 SHA-256(v1/v2)을 산출물에 넣어 이후 규칙 수정이 드러나게 했다.

## 2. 버전 변경 주석

| 대상 | 이전 | 이후 | 기록 처리 |
|---|---|---|---|
| 행위 라벨 규칙 | v1(부분 문자열) | v2(단어 경계·부정·언급 억제) | v1 동결. `results.json`의 `acts`는 계속 v1 |
| 파일럿 결과 스키마 | `...results.v1` | `...results.v2`(acts v2 + `acts_rules_v1` + `episode`·`call_id` + 원시 기록 감사) | 기존 `results.json`을 **재생성하지 않았다.** 새 집계는 `--out`으로 새 파일에 쓴다 |
| 창 재개 | 같은 call ID 재사용, 원시 파일 덮어쓰기 | episode 분리 + 원시 파일 append-only | 기존 창 기록은 episode 1로 해석 |
| 실험 산출물 | `results.json`, `manual_labels.json` … | `manual_labels_en.json`, `act_rules_v2_check.json` 추가 | 기존 파일 내용 변경 없음(README만 해석 수정) |

이 변경들은 **다음 실행에만 적용된다.** 2026-09-25 실행의 성공/실패 판정, 토큰 수치, wire 해시는 그대로다. 규칙 v2·재개 동작을 실제 모델 호출로 재실행해 확인하지는 않았다(사용자 지시로 모델 호출·SIM 금지).

## 3. 검증

`.venv-sim-worker-mac/bin/python -m pytest tests/test_zone_dialogue_ko.py tests/test_pilot_korean_dialogue_records.py` — 아래 항목을 포함한다.

- 재개가 기존 원시 wire/response를 덮어쓰지 않는다(가짜 HTTP opener, 실제 호출 없음). 기존 파일 바이트가 그대로 남고 새 기록이 별도 파일로 생긴다.
- 이미 존재하는 경로에 쓰려 하면 `SystemExit`으로 멈춘다.
- 중단된 창은 새 episode로 재개되고, 이전 episode의 call ID·원시 파일이 유지된다. 턴별 체크포인트가 남는다.
- `analyze`가 기존 결과 파일을 덮어쓰지 않는다.
- 행위 규칙: "Accepted proposal."은 `agree`만, "Standing by for r1 proposal."은 `standby`만, `disagree`·"do not agree"는 `refuse`, "r1 takes C"는 claim이 아니다. v1은 과거 라벨을 그대로 재현한다.
- 어휘 일관성: `STRUCTURED_ACTS == harness.zone_dialogue_ko.ACTS`이고 자유 텍스트 전용 행위는 `propose`·`standby`뿐이다.

CI 목록(`scripts/run_ci_tests.py`)에 새 테스트 파일을 등록했다.
