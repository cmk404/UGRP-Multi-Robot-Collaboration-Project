# 2026-09-26 TensorBoard 스냅샷 `0926-zone-perception-noslip` — 자기 카메라 인식 채점 + 접촉 프로필 부작용 감사

읽기 전용 변환이다. 두 실험의 원본과 기록을 수정하지 않고, 재채점·재감사·시뮬레이션·모델 호출을
하지 않았다. 실행 중인 공용 TensorBoard 서버(PID 9291/9293, 소유자 changmin, 9/22 시작)는
재시작하지 않고 새 run이 실제로 서비스되는지만 확인했다.

- **스냅샷:** `/Users/changmin/projects/ugrp/outputs/tensorboard/0926-zone-perception-noslip` (run 89개, 38 MB)
- **대시보드:** [http://127.0.0.1:6006](http://127.0.0.1:6006/?smoothing=0&runFilter=%5E0926-zone-perception-noslip/#timeseries) — 고정 카드·열·부분 뷰는 `outputs/tensorboard-view.json`의 `zone_perception_noslip_20260926`
- **검증 기록:** [verification.json](verification.json) (`collection.json` SHA-256 `cf809741…`)

## 담은 두 기록

| 집합 | 원본 | 실행 소스 | run |
|---|---|---|---|
| 자기 손목 어안 RGB 인식 판단 4종 오프라인 채점 | [PR #193](https://github.com/cmkang131/ugrp/pull/193) `experiments/2026-09-26-zone-own-perception` + `outputs/.../{dev-score-clean,test-score,*-frames*}` | `bc22129` (렌더·채점) | `pc-*` 25개 |
| `cargo_noslip_v1` 접촉 프로필 부작용 A/B 감사 | [PR #189](https://github.com/cmkang131/ugrp/pull/189) `experiments/2026-09-26-noslip-side-effects/results.json` + `outputs/noslip-audit/5578f09` | `9958088` (42 실행) | `ns-*` 64개 |

두 집합은 **성격이 다르다.** 인식은 오프라인 채점이고 감사는 정답 교사 조건의 SIM 물리 측정이다.
둘 다 로봇 임무 성공률이 아니며 서로 합산하지 않는다. `evaluation/reported_success`의 뜻도 다르다
(인식: 사전 등록 게이트 판정 / 감사: 공통 하드 게이트 `eq_active` 0 · `max|qvel|` ≤ 50 · NaN 0).

## run 이름

- `pc-<split>-<판단>-<belief>` — 판단 `slot`/`hold`/`placed`/`block`, belief `gt`(=`gt_stub_eval_only`,
  오차 0 stub이며 위치 추정 결과가 아니다) / `n30`(σ 30 mm) / `n60`(σ 60 mm). dev 12 + test 12.
- `pc-test-gates` — test 사전 등록 게이트 8개와 판단 4종 합계.
- `ns-<시나리오>-<프로필>-s<시드>` — 시나리오 `drive`/`wall`/`arm`/`idle`/`carry`/`load10x`/`pair`,
  프로필 `base`(=`local_contact_fine`) / `nos`(=`cargo_noslip_v1`), 시드 11·12·13. 42개.
- `ns-<시나리오>-cmp-s<시드>` — 시나리오·시드별 프로필 A/B 게이트 판정 21개.
- `ns-audit` — 42 실행·327 게이트 코호트 합계.

## 무엇을 어디서 보는가

| 탭 / 태그 | 내용 |
|---|---|
| Scalars `offline/*` | 원본 기록에 있는 오프라인 측정값. 정확도·unknown 비율·확신 오류·미끄러짐·creep·침투 등 |
| Scalars `gate/*` | 사전 등록 게이트의 판정과 값. 인식은 G1–G8, 감사는 시나리오별 허용오차·공통 하드 게이트 |
| Scalars `trace/*`, `execution/sim_time_s` | 감사 실행의 원본 `trace.jsonl` 표본을 **그대로** 옮긴 시계열. 보간·평활 없음 |
| Scalars `result/sim_s` | 감사 실행의 SIM 시간. wall 시간은 지표가 아니다(호스트를 다른 에이전트와 공유) |
| Scalars `evaluation/reported_success` | 파생 뷰가 명시한 bool 게이트 판정. 정의는 Text `result/summary`의 `success_definition` |
| Text `evaluation/*` | 판단별 전체 채점표, 게이트 정의, 실행 events·hashes, 허용오차, 호스트 부하(지표 아님) |
| Text `provenance/offline_scalars` | 원본 경로·SHA-256·크기·시계열 표본 수와 이 수치의 범위 |
| HParams | `family`·`split`·`judgment`·`belief`·`scenario`·`contact_profile`·`seed`·`outcome`·`case` |

`offline/*`·`gate/*`·`trace/*`는 공용 HParams **열 목록**에 나오지 않는다. 공용 서버의 HParams
experiment는 hparam 9열·metric 17열만 노출한다(`0925-zone-supplement`가 기록한 것과 같은 제약).
각 run에는 실제로 들어 있고 `session_groups` 89건에서 확인했다. 비교는 Time Series의 고정 카드로 한다.
열 선택은 새로 고치면 초기화되므로 view 설정의 목록을 다시 적용한다.

## 스냅샷에서 바로 읽히는 것

**인식(test, primary belief, 뷰 단위):** 사전 등록 게이트 8개 전부 통과. 결정한 답의 정확도는 네 판단
모두 1.00이고 오답 0인데, 대신 `unknown`이 25–46%다. 관측 불가(`slot_far`, `*_occluded`)와 CARRY가
담지 못하는 `can`은 전부 `unknown`이다. belief가 σ 60 mm로 나빠지면 지도 상대 판단이 무너진다
(`placed_in_slot` 확신 오류 5, `route_blockage` 2, `slot_item` 1). `holding_item`은 belief를 쓰지
않아 변하지 않는다. 게이트 밖 `peer_in_lane` 4뷰는 모두 확신 오답이다(동료/물체 구분 미구현).

**감사:** 327개 게이트 중 324 통과·3 실패. 실패는 `solo_hold_load`의 `finger_force::r1` 한 게이트가
시드 3개에서 같은 값으로 깨진 것이다(차이 2.7371 N > 허용 0.7986 N). 원인은 후보의 부작용이 아니라
**기준 프로필이 짐을 떨어뜨린 것**이고, 스냅샷에서 직접 보인다: `ns-load10x-base-s11`의
`trace/hold_elapsed_s`가 **89.45 s**일 때 `trace/finger_total_n/r1`이 0이 되고 **89.5 s**에
`trace/cargo_min_z_m`이 음수로 내려간다(절대 SIM 시각 97.10 / 97.15 s, hold 시작 7.65 s 기준).
같은 구간에서 `ns-load10x-nos-s11`은 z가 고정이고 합력이 10.72 N으로 일정하다. 구동·벽·팔·쉬는
물체는 측정 한계 안에서 변화가 없다(`ns-drive-*`, `ns-wall-*`, `ns-arm-*`, `ns-idle-*`).

## 검증

```sh
.venv-sim-worker-mac/bin/python scripts/build_perception_noslip_views.py \
  --perception-records <PR193 experiments/2026-09-26-zone-own-perception> \
  --perception-raw <PR193 outputs/2026-09-26-zone-own-perception> \
  --noslip-records <PR189 experiments/2026-09-26-noslip-side-effects> \
  --noslip-raw <PR189 outputs/noslip-audit/5578f09> \
  --output outputs/tb-perception-noslip-20260926/derived
.venv-sim-worker-mac/bin/python scripts/export_offline_audit.py \
  --source <파생 뷰 1개> ... --output outputs/tensorboard/0926-zone-perception-noslip
.venv-sim-worker-mac/bin/python scripts/verify_perception_noslip_snapshot.py \
  --snapshot outputs/tensorboard/0926-zone-perception-noslip \
  --index outputs/tb-perception-noslip-20260926/derived/index.json --report <보고 경로>
.venv-sim-worker-mac/bin/python -m pytest -q tests/test_offline_audit_export.py
```

| 항목 | 결과 |
|---|---|
| 원본 ↔ 기록 대조 | 315개 전부 일치 (인식 README 표·belief 민감도 표, 감사 README 6장 수치, 낙하 시각 89.45/89.5 s) |
| 감사 원본 파일 해시 | `results.json`의 `raw_files` 210개 전부 일치 |
| 인식 채점 원본 ↔ 커밋된 기록 | `dev-score-clean/summary.json`·`test-score/summary.json`이 `dev-summary.json`·`test-summary.json`과 **바이트 단위로 동일** |
| 이벤트 재독 (EventAccumulator) | run 89개 전부 `complete`, 선언 스칼라 8,731개 값 일치, 불일치 0 |
| 원본으로 되짚은 스칼라 | 3,807개 (나머지는 게이트 산술·코호트 합계·trace 조회처럼 이 스냅샷이 계산한 값) |
| 시계열 표본 | 530,910점. 태그별 표본 수가 원본 행 수와 같고 첫·중간·끝 표본이 원본과 일치 |
| 원본 불변 | 변환 뒤 모든 원본 SHA-256 재확인 일치 |
| 서버 서비스 확인 | 전체 637 run 중 이 스냅샷 89개, HParams 세션 89건, HTTP로 스칼라 13개 표본 대조 일치 |
| 저장 링크 | 부분 뷰 6개 링크를 열어 고정 카드·runFilter·smoothing 0과 실제 값 확인 |
| 단위 검사 | `tests/test_offline_audit_export.py` 19/19 통과 |

## 실패와 재실행

첫 내보내기는 89개 중 **6개가 실패**했다. 구역 문자를 원본 그대로 쓴 태그
`offline/case/placed_zone_A/views`가 소문자만 허용한 태그 규칙에 걸렸다(`placed_in_slot` 6 run).
태그 조각에 대문자를 허용하고(구역 문자·로봇 ID는 원본 표기를 유지한다) 부분 스냅샷을 지운 뒤
89개를 한 번에 다시 내보냈다. 부분 스냅샷의 `collection.json`은
`outputs/tb-perception-noslip-20260926/attempts/attempt-1-collection.json`에 남겼고 실패 내용은
[verification.json](verification.json)의 `export_attempts`에 있다.

## 원본 기록과 다른 값 2건

변환은 **원본 JSON의 값**을 옮겼고, 원본 기록 본문(README)이 다르게 적은 곳을 남긴다.

| 위치 | 항목 | README | 원본 JSON |
|---|---|---|---|
| PR #189 README 6.6 표 | `solo_hold_load`/`local_contact_fine` 전체 118 s creep | 40.7616 mm/min | **40.76499** mm/min |
| PR #193 README test 총평 | 게이트 범위 뷰 수 / 결정 뷰 수 | 84뷰 중 결정 51뷰 | **88뷰 중 결정 56뷰** (24+20+24+20, decided 13+12+16+15) |

두 경우 모두 결론(창별 creep 자릿수, 오답 0)은 바뀌지 않는다. 각 PR 코멘트로 알린다.

## 변환기에 대해

`scripts/export_tensorboard.py`(실행·학습 기록)는 **고치지 않았다.** 이 스냅샷의 수치는 실행 기록이
아니라 오프라인 채점·물리 감사이고, 현재 main의 변환기에는 그 수치를 옮길 태그가 없다. 그래서
파생 뷰 선언(`derived_view_only`·`offline_source`·`offline_scalar_scope`·`offline_scalars`)을 읽는
**추가 모듈** `scripts/tensorboard_tools/offline_audit.py`를 새로 넣었다. 선언 키는 PR #183이
`export.py`에 넣은 `offline_scalars` 형식과 같게 맞췄으므로, PR #183이 main에 들어오면 이 모듈은
표준 변환기 하나로 합치고 은퇴시킬 수 있다. 같은 파일을 두 PR이 고치지 않도록 일부러 추가만 했다.

안전 규칙은 기존 변환기와 같다: 기존 출력 폴더 덮어쓰기 거부, 선언한 원본의 SHA-256 확인,
변환 중 원본이 바뀌면 이벤트 미게시 + 실패 manifest, 64 MiB 상한, 인증 필드 가림.

## 한계 · 확인하지 않은 것

- PR #193·#189는 **아직 병합 전**이다. 병합 시 기록 경로·실행 소스 SHA가 바뀔 수 있다.
- PR #193 게이트 **G8**(`tests/test_zone_own_perception.py` 20/20)은 그 기록의 주장을 옮긴 값이며
  이 작업이 다시 실행하지 않았다. `gate/g8_unit_tests_pass`의 출처를 Text에 표시했다.
- PR #193의 진행 중인 후속 코호트(`outputs/2026-09-26-zone-own-perception-v2`)는 **미완이라 넣지 않았다.**
- 인식은 분할당 시드 4개·판단별 사례 4뷰다. 감사는 조건당 1회이며 시드 11/12/13은 독립 반복이 아니라
  결정성 검사다. 비율을 다른 실험과 합산하지 않는다.
- 로봇 임무 성공, 실시간 실행, 통신 조건 비교, 실물 MasterPi 성능은 이 스냅샷의 범위가 아니다.
- 파생 뷰와 원본은 로컬 `outputs/`에만 있고 Git에 없다. manifest의 해시는 원본의 원격 백업이 아니다.
