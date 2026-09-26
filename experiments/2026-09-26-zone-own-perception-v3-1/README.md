# 2026-09-26 자기 손목 카메라 인식 v3.1 — 정보 없는 영상은 `unknown` (PR #193 Codex 검토 P1 수정)

- **목적:** Codex의 v3 검토(PR #193) P1을 고친다. 640×480 전체 BGR=(10,10,10) JPEG + CARRY PWM + `expected_kind='heavy_crate'`에 v3가 `yes` / 0.70 / `EXPECTED_KIND_SHAPE_AT_GRIP`을 냈고 트래커가 2프레임 뒤 확정했다. v3는 어두운 픽셀을 예측 띠로 먼저 자른 뒤 IoU를 계산하는데, CARRY에서 crate 실루엣이 **유효 화면 전체**(245,511 / 245,511 px)라 검은 화면이 IoU 1.0으로 일치했다. 이 기록의 테스트가 v3에서 이 결함을 그대로 재현한다(`tests/test_zone_own_perception_v3_1.py::test_p1_black_frame_carry_crate_is_unknown_and_never_confirmed`).
- **v1·v2·v3는 한 바이트도 바꾸지 않았다.** v3 test 결과는 v3로 채점한 기록이므로 그대로 둔다. `::test_v1_v2_v3_files_are_byte_identical`가 v1·v2·v3 런타임·트래커·평가기 9개 파일의 SHA-256을 `16d464e` 값으로 고정한다. v3.1은 새 파일이며 v3를 감싼다.
- **v3 test 분할(271–274)은 다시 채점하지 않았다.** 이미 한 번 쓴 분할이다. v3.1은 새 분할(test 371–374, 그림자 stress 381–382, smoke 391)을 쓴다.
- **코드:** `harness/zone_own_perception_v3_1.py`(정보 게이트 + v3 감싸기), `scripts/eval_zone_own_perception_v3_1.py`(render: v3 렌더러를 새 분할·조명 프로필로 호출 / score: v3.1과 v3를 같은 프레임에 / adversarial), `tests/test_zone_own_perception_v3_1.py`. 트래커는 판단 이름이 같아 `harness/zone_own_outcome_v3.py`를 그대로 쓴다.

## v3.1이 바꾼 것 (세 가지뿐)

| # | 규칙 | 적용 대상 |
|---|---|---|
| 1 | **영상 정보 게이트.** 유효 영역의 밝은 픽셀 비율(V > 40) ≥ 0.03, 밝기 폭(V P95−P5) ≥ 24, 가장자리 안쪽의 강한 엣지(Sobel ≥ 40) 비율 ≥ 0.003(≥ 50 px), 선명도(강한 엣지의 Sobel 평균 ÷ σ1.5 흐림 뒤 같은 픽셀의 평균) ≥ 1.30. 하나라도 실패하면 `unknown` / `INSUFFICIENT_IMAGE_INFORMATION` | **모든 판단**: 집게의 물건, 운반 팀 화물, 손잡이, 파지 단계(종류·손잡이 둘 다) |
| 2 | **모양 답은 모양의 경계가 보일 때만.** 보고할 종류의 집게 위치 실루엣 바로 바깥 9 px 띠가 ≥ 2,000 px이고 그중 ≥ 0.80이 밝으며, 바로 안쪽 9 px 띠의 ≥ 0.80이 어두워야 v3의 모양 답을 유지한다. 아니면 `SHAPE_BOUNDARY_NOT_IN_FRAME` | `team_cargo_at_grip`의 모양 단서 |
| 3 | **부재는 밝은 띠에서만.** 주문 종류의 허용 띠 픽셀의 ≥ 0.80이 V > 40이어야 v3의 "집게에 주문 종류 없음"을 유지한다. 아니면 `AT_GRIP_BAND_NOT_LIT` | `held_item_at_grip`의 부재 `no` |

게이트를 통과한 영상에서는 v3의 답·이유·신뢰도를 그대로 낸다(`v3_answer`·`v3_reason`을 함께 기록). 게이트 값은 프레임만 읽는다. 입력 경계는 v3와 같다: 자기 손목 RGB 1장, 자기가 발행한 팔 PWM, 정적 지도/카탈로그, 시나리오 주문서. `::test_v3_1_runtime_never_imports_mujoco_or_scene_modules`와 `::test_v3_1_runtime_source_does_not_mention_sim_state_or_top_inputs`가 확인한다.

## P2 — crate lug fixture와 팔 그림자 (검토 지적 반영)

v3 README 발견 3의 설명은 **v2의 "검은 운반 crate" 인공물만 설명한다.** v2 fixture는 crate를 중심으로 들어 렌즈가 몸체 안에 들어갔고(Codex FK 확인: 렌즈 약 (−21.75, 0, 52.30) mm, 몸체 안), v3 fixture는 lug로 들어 렌즈가 몸체 밖(x ≈ −121.75 mm)이다. 그래서 v3의 운반 crate는 분홍으로 보였고 **색으로** 식별됐다. 이것은 **실제 팔 그림자가 인식을 해치지 않는다는 증거가 아니다.** 어두운 화물을 모양으로 식별하는 경로는 정당한 렌더에서 한 번도 입증되지 않았고, v3 조명 한 가지에서만 쟀다. 그래서 이번에는 조명을 바꾼 **그림자 stress 세트**를 새로 렌더한다(아래). stress 조명은 시각 전용(광원 diffuse/ambient/위치, headlight)이며 물리·카메라·외관·도색·카탈로그·지도는 그대로다.

pipeline 확인용 smoke(시드 391, `ceiling_only`, held+carry 15뷰, 게이트 아님)에서 이미 그림자가 문제를 만든다는 것이 보였다: 천장 광원만 켜면 운반 crate가 팔 그림자에 가려 거의 검게 렌더되고, held-check의 can도 검은 배경 위 검은 띠가 된다. 이 smoke에서 **v3는 `held_can`에 "can 없음" 확신 오류 1건**을 냈고 v3.1은 같은 뷰를 `unknown`으로 냈다. 이 수치는 smoke이며 게이트·성공 근거가 아니다.

## dev에서 정한 값 (v3 dev 프레임 241–244와 합성 프레임만 사용)

v3 dev 렌더(`outputs/2026-09-26-zone-own-perception-v3/dev-frames`, 소스 `13319fe`) 300장 중 v3 자세 204장(CARRY, can이 있는 held-check, 파지 관측 자세)과 그 합성 변형으로 게이트 값을 골랐다. v2 관측 자세(approach_look_v2)는 v3.1 판단에 쓰지 않는다.

| 측정 | dev 정상 프레임 최소 | 정보 없는 합성 프레임 최대 | 임계값 |
|---|---:|---:|---:|
| 밝은 비율 (V > 40) | 0.061 (held-check can) | 0 (V 0–39 전부) | ≥ 0.03 |
| 밝기 폭 V P95−P5 | 34 | 5 (균일·잡음), 덮개 138 | ≥ 24 |
| 강한 엣지 비율 | 0.005 | 0 (균일·덮개) | ≥ 0.003 |
| 선명도 | 1.406 | 1.19 (Gaussian σ ≥ 4) | ≥ 1.30 |

**held-check 자세에서 can이 없는 프레임 36장은 균일한 V 37(폭 8, 엣지 0)이다** — 카메라가 위를 향해 거의 아무것도 보지 않는다. 검토가 말한 "밝기 0–39" 영상 그 자체이며, v3는 이 프레임들에서 "can 없음"(`held_can_absent`·`held_can_wrong_kind` 각 4/4 정답)을 냈다. 결과는 맞았지만 렌즈 덮개와 구별할 수 없는 영상이었다. v3.1은 여기서 `unknown`이다. 즉 **v3.1의 held-check 자세는 "can을 들고 있다"만 확인할 수 있고 "없다"는 답하지 않는다.**

**부분 밝은 덮개는 막지 못한다(검토 뒤 dev에서 확인, 해결하지 않음).** 손목 아래쪽 50–90 %를 매끈한 밝은 덮개(V 120–220)로 가리면 v3.1도 "집게에 없음"(`EXPECTED_KIND_ABSENT_FROM_AT_GRIP_BAND`)과 "손잡이 모두 도달 밖"을 확정한다(dev 140 뷰·판단 중 확정 오류 7, v3 9). 띠 안 텍스처(Sobel 비율)로 덮개와 바닥을 나누려 했지만 dev에서 겹쳐(덮개 오답 t25 최대 0.045, 정상 부재 최소 0.0076) 규칙을 넣지 않았다. **밝고 매끈한 가림막은 열린 한계**로 보고만 한다.

### v3.1 dev 채점 (v3 dev 프레임, 소스 `16d464e` + 미커밋 v3.1, `dev-summary.json`)

1분 부하 54.9 → 76.4(다른 작업이 함께 돈 호스트). v3.1이 v3와 다른 곳은 held-check의 can 없는 세 사례뿐이다(36 tick 모두 게이트에서 멈춤).

| 버전:판단:자세 | 뷰 | unknown | 전체 정확도 | 확신 오류 | 관측 가능(뷰 / 정확도 / unknown) |
|---|---:|---:|---:|---:|---|
| `v3_1:held_item_at_grip:carry` | 20 | .200 | .800 | 0 | 16 / 1.000 / .000 |
| `v3:held_item_at_grip:carry` | 20 | .200 | .800 | 0 | 16 / 1.000 / .000 |
| `v3_1:held_item_at_grip:held_check` | 16 | **.875** | .125 | 0 | 12 / .167 / .833 |
| `v3:held_item_at_grip:held_check` | 16 | .375 | .625 | 0 | 12 / .833 / .167 |
| `v3_1:team_cargo_at_grip:carry` | 24 | .000 | 1.000 | 0 | 24 / 1.000 / .000 |
| `v3_1:team_cargo_identity:grasp_look_v3` | 20 | .150 | .850 | 0 | 20 / .850 / .150 |
| `v3_1:team_cargo_handle:grasp_look_v3` | 20 | .400 | .600 | 0 | 12 / 1.000 / .000 |

(운반·파지 단계 v3 행은 v3.1과 같다.)

### dev 적대 영상 (v3 dev 100뷰, `dev-adversarial-summary.json`, 소스 `25eef1e` + 미커밋, wall 1002.7 s, 부하 191.6 → 137.2)

- **행렬:** 뷰마다 tick 0을 11가지로 바꾸고(검정, BGR 10, 균일 V 0–39, 잡음 V 0–39, 실제 장면을 0–39로 어둡게, 균일 회색 40–250, Gaussian σ 4/8/16, 어두운 렌즈 덮개, 밝은 렌즈 덮개), **v3.1 판단 21개(집게의 물건 6종, 운반 팀 화물·손잡이·파지 단계 종류·손잡이 각 3종) × 자세 3개(CARRY, held-check, 파지 관측)** 를 모두 물었다. v3.1: 변형마다 6,300 답 중 `unknown` 아닌 답 **0**. v3(10뷰마다 1뷰, 자기 자세): 변형마다 210 답 중 확신 답 12–70.
- **트래커(뷰 3 tick):** 정보 없는 11가지 모두 v3.1 확정 0 / 140. v3는 BGR 10에서 24 확정(오답 16), 균일 회색 52(오답 24), 밝은 덮개 61(오답 29).
- **부분 어두운 덮개(50–90 %, V 5–39):** v3.1 확정 20, 오답 0. v3 확정 34, 오답 2.
- **부분 밝은 덮개:** v3.1 확정 35, 오답 7 / v3 확정 49, 오답 9 (위 한계).

## 새 분할 (`split.json`)

| 분할 | 변형·시드 | 조명 | 코호트 |
|---|---|---|---|
| test | `zone_wide_door` 371, `zone_wide_corridor` 372, `zone_wide_two_doors` 373, `zone_wide_two_doors` 374 | 표준 | v3와 같은 held 9·carry 6·grasp 5 사례, 파지 단계는 v3 관측 자세만 (시드당 20뷰, 80뷰·240프레임) |
| stress | `zone_wide_door` 381, `zone_wide_corridor` 382 | `dim_all`, `ceiling_only`, `low_side_key` 각각 | test와 같은 코호트 (프로필당 40뷰, 합 120뷰) |
| smoke | `zone_wide_door` 391 | `ceiling_only` | pipeline 확인만 (위 P2) |

v1(41–44/71–74)·v2(141–144/171–174)·v3(241–244/271–274)와 겹치지 않는다. 렌더러는 v3의 `scripts/eval_zone_own_perception_v3.py`(변경 없음)이고 `render-v3_1.json`에 분할 파일 해시·조명 프로필·적용 전후 광원 값을 남긴다.

조명 프로필(사전 고정): `dim_all` 모든 광원 diffuse·ambient ×0.4, headlight ×0.4 / `ceiling_only` 천장 광원(`dispatch_ceiling`, diffuse 1.0)만, 나머지·ambient·headlight 0 — 팔 그림자가 바로 아래로 / `low_side_key` 주 광원을 (0, −3.4, 1.0) m로 옮겨 30° 앙각, 나머지 ×0.25, headlight 0 — 긴 비스듬한 그림자.

## 사전 등록한 게이트 (test·stress·적대, 뷰 단위, 확신 = 신뢰도 ≥ 0.65)

이 표를 커밋한 뒤 test와 stress를 **한 번만** 렌더·채점한다. "확신 오류"는 `answer ≠ truth`, `answer ≠ unknown`, 신뢰도 ≥ 0.65인 뷰다.

| ID | 게이트 | dev 값 |
|---|---|---|
| T1 | test v3.1 확신 오류 0 (집게 물건 36 + 운반 24 + 파지 종류 20 + 손잡이 20 = 100 뷰·판단) | 0 |
| T2 | W7 회귀: `held_can_carry`·`held_box_held_check` 8뷰 확신 답 0 | 0 |
| T3 | `held_item_at_grip` CARRY 관측 가능 정확도 ≥ .80, unknown ≤ .25 | 1.000 / .000 (16) |
| T4 | `held_nothing_carry` ≥ 3/4 `no` | 4/4 |
| T5 | `team_cargo_at_grip` 정확도 ≥ .85, `carry_heavy_crate` ≥ 3/4 `yes` | 1.000, 4/4 |
| T6 | 파지 단계 `grasp_tri_frame` 종류 ≥ 3/4 `yes`; 종류 관측 가능 정확도 ≥ .70, 손잡이 관측 가능 정확도 ≥ .75 | 4/4; .850; 1.000 |
| T7 | 일관성: v3.1 답이 v3와 다른 test 뷰는 모두 v3.1 tick 중 하나 이상이 새 규칙 1–3(게이트·경계·밝은 띠)으로 `unknown`이 된 뷰다 | 성립 (v3와 다른 뷰 8, 8개 모두 해당) |
| A1 | test 프레임 적대 행렬: 정보 없는 11변형 × 21판단 × 3자세에서 v3.1 `unknown` 아닌 답 0; 트래커 확정 0 | 0 / 0 |
| A2 | 부분 어두운 덮개: v3.1 확정 오답 0 | 0 |
| A3 | 부분 밝은 덮개: **보고만**(게이트 아님) | v3.1 7 / v3 9 |
| S1 | stress 3프로필 합계 v3.1 확신 오류 0 (프로필당 50 뷰·판단, 합 150) | (dev 없음) |
| S2 | stress 프로필별 정확도·unknown·게이트 정지 비율과 같은 프레임의 v3 확신 오류: **보고만** | — |
| U | `tests/test_zone_own_perception_v3_1.py`와 v1·v2·v3 테스트 전체 통과 | 126/126 (v3.1) |

## 한계

- 오프라인 인식 평가다. 임무 성공·실시간 실행·통신 조건 비교는 범위 밖이다. `held_*`·`carry_*`는 fixture이고 파지 성공이 아니다. `grasp_pose` 경로(물리 파지 뒤 강체)는 여전히 렌더 평가가 없다.
- 게이트는 **정보가 없다는 것**만 막는다. 밝고 매끈한 부분 가림막(A3), 실물 렌즈의 흐림·노출 변화는 다루지 않는다. 게이트 값은 시뮬레이터 렌더(JPEG 90) 기준이며 실물에서 다시 재야 한다.
- v3.1의 held-check 자세는 "없다"를 답하지 않는다(위). 모양 단서는 여전히 정당한 렌더에서 입증되지 않았다.
- 분할당 시드 4개(stress 2개 × 조명 3개) 규모다. 사례 단위로 읽고 다른 실험 수치와 합산하지 않는다.

로컬 보관은 원격 백업이 아니다.
