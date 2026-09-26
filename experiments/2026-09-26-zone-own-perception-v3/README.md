# 2026-09-26 자기 손목 카메라 인식 v3 — 집게 위치 예측 영역·운반 중 팀 화물·파지 단계 관측 자세 (패키지 B v3)

- **목적:** [v2](../2026-09-26-zone-own-perception-v2/README.md) 보고서가 남긴 세 가지 틈을 **자기 손목 어안 RGB만으로** 메운다.
  1. **W7:** CARRY 자세에서 들린 can이 프레임 밖이었는데 고정 영역이 배경을 보고 "cyan을 들고 있다"(0.82)고 답했다.
  2. **운반 중 `heavy_crate`:** v2 기록에서는 팔 그림자로 채도 0이라 식별 불가였다.
  3. **`tri_frame`:** 파지 전 관측 자세에서 종류를 확정하지 못했다(3/4 `unknown`).
- **v1·v2는 한 바이트도 바꾸지 않았다.** `tests/test_zone_own_perception_v3.py::test_v1_and_v2_files_are_byte_identical`가 v1·v2 런타임·평가기 6개 파일의 SHA-256을 `fb44c2c` 값으로 고정하고, `::test_v1_and_v2_answer_the_same_with_and_without_v3_imported`가 v3를 먼저 import한 인터프리터에서도 v1/v2 답·이유·신뢰도가 같음을 확인한다.
- **코드:** `harness/zone_own_perception_v3.py`(판단 3종 + 집게 위치 기하), `harness/zone_own_outcome_v3.py`(v1 정책 상속 트래커), `scripts/eval_zone_own_perception_v3.py`(render/score, v2 나란히 채점), `tests/test_zone_own_perception_v3.py`.

## 판단

모든 판단은 `yes` / `no` / `unknown` + 신뢰도다. `unknown`은 1급 답이고 `no`는 항상 근거를 든다.

| 판단 | 질문 | 무엇이 바뀌었나 |
|---|---|---|
| `held_item_at_grip` | 내 집게에 있는 것이 주문한 종류인가 (자세 무관) | 고정 사각형 대신 **발행 PWM의 FK로 예측한 집게 위치 실루엣**. 주문 종류의 실루엣이 프레임에 없으면 `unknown` |
| `team_cargo_at_grip` | 운반 중 내 집게의 팀 화물이 주문한 종류인가 | 같은 예측 실루엣으로 색 우선, 색이 전혀 없을 때만 **모양(어두운 점유 영역과 종류별 실루엣의 IoU)** |
| `team_cargo_grasp_stage` | 파지 직전: 종류(v2 판단 그대로)와 "내 손잡이가 여기" | **새 관측 자세** 한 프레임에서 둘 다. 손잡이 거리는 v2 검출 + **손잡이 접촉 평면 z = 0.032 m** |

## 입력 경계

v1·v2와 같다: 자기 손목 RGB 1장, 자기가 **발행한** 팔 PWM(서보 3·4·5·6), 정적 지도/카탈로그, 시나리오 주문서. 집게 위치 실루엣은 발행 PWM의 순기구학(`harness/visual_arm.py`), 고정 카메라 보정, 정적 화물 카탈로그(`sim/zone_cargo.py`)로 계산한 **예측**이며, 실행 중 시뮬레이터 상태와 대조하지 않는다. 물리 파지에서는 호출자가 **파지 순간의 발행 PWM**을 `grasp_pose`로 넘긴다(카메라와 물건이 한 강체이므로 실루엣은 파지 순간의 것).

`::test_v3_runtime_judgments_never_import_mujoco_or_read_sim_state`는 새 인터프리터에서 v3 런타임 두 모듈을 import하고 판단 3종을 호출한 뒤 `mujoco`·장면 모듈(`sim.zone_arena` 포함)이 `sys.modules`에 없음을 확인한다. `::test_v3_runtime_sources_do_not_mention_sim_state_or_top_inputs`는 `import mujoco`, `mj_forward`, `.xpos`, `.qpos`, `site_xyz`, `segmentation`, `world.data` 등이 코드에 없음을 확인한다.

## 핵심 발견 (dev, 원인 확인)

1. **W7의 "청록 상자"는 상자가 아니라 pickup 구역 바닥 칠이었다.** v2 test 프레임 `zone_wide_corridor-s172-held_can_carry_posture`의 세그멘테이션(평가 전용)에서 v1 cyan 마스크 픽셀 91,092개가 `zone_pickup` 칠, 6개가 상자다. 칠은 천장 조명에서 OpenCV **H 98–100 / S 95–109**로 렌더되어 v1 cyan 대역(H 84–100)의 끝에 걸린다. 집게의 cyan 상자는 **H 93**(dev P5–P95 93–94)이다. v2 README의 "청록 상자 앞에 서 있었다"는 원인 설명은 틀렸다(이 기록에서 정정). 두 로봇이 물건을 드는 곳이 바로 pickup 구역이라 우연이 아니라 **자주 생길 조건**이다. v3의 집게용 cyan 창은 H 84–97이다(v1 `hue_mask`는 그대로).
2. **집게 위치 실루엣은 FK + 렌더 근접 절단만으로 v2의 측정을 재현한다.** 시뮬레이터 손목 렌더는 광축 거리 **0.0297 m**(znear 0.0020 × 모델 extent 14.86 m)보다 가까운 것을 그리지 않는다. 이 절단을 넣은 광선 교차 실루엣과 세그멘테이션의 IoU(v2 dev 프레임): CARRY의 상자 **0.96**, held-check의 can **0.83**; held-check의 상자·tile은 예측·렌더 모두 **0 px**; CARRY의 can은 예측 3,428 px(렌더 1,276 px, 가장자리 띠)다. 절단을 빼면 CARRY의 can이 프레임 245,098 px를 채운다고 잘못 예측한다 — v2 고정 영역이 배경을 본 이유가 이것이다.
3. **v2의 "그림자 속 검은 crate"는 fixture 인공물이었다.** v2 fixture는 팀 화물을 **중심**으로 집게에 두었다. crate 몸체 반길이 0.07 m, 렌즈는 grip site 0.033 m 뒤이므로 **렌즈가 crate 몸체 안에 들어간다**(물리적으로 불가능). v3 fixture는 카탈로그 파지 규약대로 **lug(grasp role)** 로 든다. 그러면 운반 crate는 분홍으로 렌더되고 dev에서 v2 판단도 v3 판단도 색으로 4/4 식별한다. 모양 단서는 구현했지만 **정당한 렌더에서 발동한 적이 없다**(합성 테스트로만 확인). v2 중심 fixture를 재현한 dev 스캔 4뷰에서 v3는 3뷰 `unknown`, 1뷰 확신 오류("집게에 아무것도 없음")를 냈다 — 렌즈가 몸체 안이라 프레임 전체가 바닥이었다. 불가능한 기하이므로 코호트에서 뺐다.
4. **held-check 자세의 can은 절단면에 걸려 있어 tick마다 크게 흔들린다.** fixture 파지 흔들림(전방 ±4 mm, 높이 ±3 mm, yaw ±4°)만으로 세그멘테이션 can 면적이 10k–68k px로 변한다. 그래서 색은 **허용 띠**(전방·높이 ±6 mm 오프셋 5개 실루엣의 합집합)에서 세고, "띠 밖으로 이어지는 색"은 집게의 물건이 아닌 것으로 본다(포함률). can `yes`는 dev 2/4에 그쳤고 나머지는 `unknown`이다.
5. **손잡이 거리의 체계 오차는 바닥 투영 때문이다.** 위에서 본 손잡이의 가장 낮은 어두운 픽셀은 손잡이 **위**(빔 밴드는 32 mm 막대 윗면, lug 바닥은 자기 윗면에 가려짐)다. 바닥(z = 0)으로 투영하면 dev에서 +3.7–6.7 cm 멀게 나오고(v2 test의 "7 mm 초과" 오류, 새 자세의 dev 확신 오류 2건), z = 0.032 m 평면이면 세 종류·두 자세 모두 +0.3–1.2 cm다. 0.032 m는 카탈로그 `HANDLE_HEIGHT_M`이다(값 6개 격자에서 dev로 선택).
6. **파지 단계 관측 자세 스캔(5개 후보, 같은 로봇·화물·tick 흔들림).** v2 자세(공구 −54.9°)는 `tri_frame` 종류를 dev 스캔 1/4만 식별했다. 손목만 올린 −50°·−44°는 4/4였지만 −44°에서 "손잡이 다른 곳" 확신 오류 2건. 보정 IK(grip site 0.10 m 앞·0.16 m 높이, 공구 −39°) `{1:2000, 3:699, 4:2400, 5:1321, 6:1500}`은 중앙 열에서 바닥을 0.22–0.88 m까지 본다: `tri_frame` 4/4, `heavy_crate` 4/4, 확신 오류 0. 이것을 `GRASP_LOOK_POSTURE`로 채택했다. 한 자세로 종류와 손잡이를 모두 묻는다(v2는 두 자세).

## 데이터 생성(render)과 채점(score)

v1·v2와 같은 구조다. 표준 `CargoZoneScene`(`cargo_noslip_v1`, weld OFF)을 동기 SIM으로 세우고 **로봇이 받을 수 있는 것만** `frames/`에 쓴다. 정답은 `eval-labels/`에만 둔다.

- **`held_*`·`carry_*` 뷰는 물리 파지가 아니다.** 카탈로그 파지 규약(grasp role의 grip 점을 grip site에, 로봇이 그 role의 접근 방향을 향함)과 tick마다 파지 흔들림을 넣어 한 프레임을 렌더한 **인식 fixture**다. weld OFF, 보조 고정 없음.
- 파지 단계 뷰는 같은 로봇·화물·tick 흔들림으로 **v2 자세와 v3 자세에서 한 번씩** 렌더한다. 같은 프레임 묶음에 v2 판단을 함께 채점해 나란히 비교한다(`held_item`→v2 `judge_held_item`, 운반→v2 `judge_team_cargo_identity`, v2 자세→v2 종류·손잡이).
- 뷰마다 관측 tick 3개, 정면 pan 1500, 주기 1.0 SIM s — v1·v2와 같다. 모든 뷰·통신 조건에서 동일하다.
- 카메라 배치/FOV, 로봇 외관, 도색, 카탈로그, `maps/zones/*`, `sim/zone_scene.py`, `sim/zone_arena.py`는 바꾸지 않았다.

### 코호트 (시드당 25뷰, 분할당 100뷰·300프레임)

| 계열 | 사례 (자세 / 든 것 / 주문) |
|---|---|
| held (9) | `held_can` held-check/can/can · `held_can_absent` held-check/없음/can · `held_can_wrong_kind` held-check/tile/can · `held_box_carry` CARRY/cyan/cyan · `held_tile_carry` CARRY/tile/tile · `held_nothing_carry` CARRY/없음/cyan · `held_tile_expect_box_carry` CARRY/tile/cyan · **`held_can_carry`** CARRY/can/can (W7 조건, 관측 불가) · **`held_box_held_check`** held-check/cyan/cyan (대조, 관측 불가) |
| carry (6) | `carry_long_beam`·`carry_heavy_crate`·`carry_tri_frame` (맞음) · `carry_crate_expect_beam` · `carry_beam_expect_crate` · `carry_nothing_expect_crate` |
| grasp (5 × 2 자세) | `grasp_long_beam`·`grasp_heavy_crate`·`grasp_tri_frame` · `grasp_wrong_kind`(tri_frame 앞, 주문 long_beam) · `wrong_end_long_beam` |

**관측 가능(사전 고정):** `held_item_at_grip`은 든 것이 주문 종류면 세그멘테이션 ≥ 500 px, 아니면(없음·다른 종류) 그 자세가 주문 종류를 담을 때(`FRAMED`: CARRY는 상자 4색·tile·팀 화물, held-check는 can). `team_cargo_at_grip`은 든 팀 화물이 ≥ 400 px 또는 아무것도 들지 않음(v2의 채도 조건은 `observable_colour_v2`로 따로 기록). 파지 단계는 v2 정의 그대로.

## 분할

`split.json`: dev `zone_wide_door` 241, `zone_wide_corridor` 242, `zone_wide_two_doors` 243, `zone_wide_door` 244 / test 271, 272, 273, `zone_wide_corridor` 274. v1(41–44/71–74)·v2(141–144/171–174)와 겹치지 않는다. 자세·임계값은 dev에서만 골랐고 test는 이 사전 등록을 커밋한 뒤 **한 번만** 렌더·채점한다.

## 사전 등록한 값 (test 채점 전 고정, `PREREGISTERED`)

| 항목 | 값 | 근거 (dev) |
|---|---|---|
| 렌더 근접 절단 | 0.0297 m | znear × extent; 실루엣–세그 IoU 상자 0.96 / can 0.83 |
| 파지 허용 띠 | 전방·높이 ±6 mm, 5개 오프셋 합집합 | fixture 흔들림 ±4/±3 mm |
| 주문 종류 실루엣 최소 | 5,000 px (팀 화물도 5,000) | CARRY의 can 3,428 px(띠), held-check can 37,884, CARRY 상자 75,044 |
| `yes` | 띠 안 주문 색 ≥ 실루엣의 0.15, 포함률 ≥ 0.85, 경쟁 색 대비 1.6배 | 상자 포함률 0.99–1.00, 바닥 칠은 띠 밖으로 이어짐 |
| 다른 종류 `no` | 그 종류가 자기 실루엣의 ≥ 0.45, 포함률 ≥ 0.85, 주문 색은 확장 띠(25 px)에서 ≤ 0.02 | tile 0.86–1.09 |
| 부재 `no` | 확장 띠에서 모든 화물 색 ≤ 0.02, 띠 75퍼센타일 V ≥ max(28, 0.30×프레임) | |
| 집게용 cyan 창 | H 84–97 | 집게 상자 H 93, pickup 칠 H 98–100 |
| 팀 화물 색 | 실루엣의 ≥ 0.25, 포함률 ≥ 0.75 | 운반 빔 포함률 0.79–0.82인 tick 3개(프레임 가장자리까지 이어짐) |
| 모양 단서 | 띠 합집합의 팀 색 ≤ 0.02일 때만; 어두움 V ≤ 40(절대), IoU ≥ 0.55, 2위 대비 1.25배, 신뢰도 0.70 | 437프레임에서 바닥·칠 중 V ≤ 40은 최대 13 % |
| 집게에 없음 | 주문 띠의 밝은 바닥 ≥ 0.85, 모든 팀 띠 합집합 점유 ≤ 0.08 | |
| 파지 단계 자세 | `{1:2000, 3:699, 4:2400, 5:1321, 6:1500}` | 위 발견 6 |
| 손잡이 접촉 평면 | z = 0.032 m | 위 발견 5 |
| 판정 커밋 | v1과 동일(주기 1.0 SIM s, 연속 2 tick, 신뢰도 ≥ 0.65) | `harness/zone_own_outcome.py` 상속 |

## 사전 등록한 게이트 (test, 뷰 단위)

"확신 오류"는 `answer ≠ truth`, `answer ≠ unknown`, 신뢰도 ≥ 0.65인 뷰다.

| ID | 게이트 | dev 값 |
|---|---|---|
| V1 | `held_item_at_grip` 확신 오류 ≤ 1 (36뷰), 관측 가능 뷰에서는 0 | 0 / 0 |
| V2 | W7 회귀: `held_can_carry` 4뷰와 `held_box_held_check` 4뷰에서 확신 답 0 | 0 (v2 판단: `held_can_carry` 1건 `no`+cyan) |
| V3 | `held_item_at_grip` 관측 가능 뷰 정확도 ≥ 0.80, unknown ≤ 0.25 | 0.929 / 0.071 (28뷰) |
| V4 | `held_nothing_carry` ≥ 3/4 `no` | 4/4 (v2 0/4, 전부 unknown) |
| V5 | `team_cargo_at_grip` 확신 오류 0 (24뷰), 전체 정확도 ≥ 0.85 | 0 / 1.000 |
| V6 | `carry_heavy_crate` ≥ 3/4 `yes` | 4/4 |
| V7 | 파지 단계(v3 자세) `grasp_tri_frame` 종류 ≥ 3/4 `yes` | 4/4 (같은 뷰 v2 자세 0/4) |
| V8 | 파지 단계(v3 자세) 종류+손잡이 확신 오류 합 ≤ 1 (40 판단·뷰) | 0 |
| V9 | 파지 단계 손잡이 관측 가능 뷰 정확도 ≥ 0.75, 종류 관측 가능 뷰 정확도 ≥ 0.70 | 1.000 (12) / 0.850 (20) |
| V10 | `tests/test_zone_own_perception_v3.py` 전체 통과 (v1/v2 바이트 고정 포함) | 27/27 |

## dev 결과 (선택에만 사용, 합산 금지)

렌더 `outputs/2026-09-26-zone-own-perception-v3/dev-frames`(소스 `13319fe`, `source_dirty: false`, 100뷰·300프레임, wall 615.6 s, 1분 부하 15.3 → 28.7 — 다른 작업이 함께 돌던 호스트), 채점 `dev-score-2`(손잡이 평면 추가 후, 이 폴더 `dev-summary.json`). 자세 스캔은 `dev-scan`(164뷰, 5자세, wall 914.7 s, 부하 7.6 → 21.2)과 `dev-scan-score4`.

| 버전:판단:계열:자세 | 뷰 | unknown | 전체 정확도 | 결정 정확도 | 확신 오류 | 관측 가능(뷰 / 정확도 / unknown / 확신 오류) |
|---|---:|---:|---:|---:|---:|---|
| `v3:held_item_at_grip:held:carry` | 20 | .200 | .800 | 1.000 | 0 | 16 / 1.000 / .000 / 0 |
| `v2:held_item_at_grip:held:carry` | 20 | .350 | .600 | .923 | **1** | 16 / .750 / .250 / 0 |
| `v3:held_item_at_grip:held:held_check` | 16 | .375 | .625 | 1.000 | 0 | 12 / .833 / .167 / 0 |
| `v2:held_item_at_grip:held:held_check` | 16 | .312 | .625 | .909 | **1** | 12 / .833 / .083 / 1 |
| `v3:team_cargo_at_grip:carry:carry` | 24 | .000 | 1.000 | 1.000 | 0 | 24 / 1.000 / .000 / 0 |
| `v2:team_cargo_at_grip:carry:carry` | 24 | .083 | .917 | 1.000 | 0 | 24 / .917 / .083 / 0 |
| `v3:team_cargo_identity:grasp:grasp_look_v3` | 20 | .150 | .850 | 1.000 | 0 | 20 / .850 / .150 / 0 |
| `v2:team_cargo_identity:grasp:approach_look_v2` | 20 | .700 | .300 | 1.000 | 0 | 14 / .429 / .571 / 0 |
| `v3:team_cargo_handle:grasp:grasp_look_v3` | 20 | .400 | .600 | 1.000 | 0 | 12 / 1.000 / .000 / 0 |
| `v2:team_cargo_handle:grasp:approach_look_v2` | 20 | .750 | .250 | 1.000 | 0 | 9 / .556 / .444 / 0 |

v2의 확신 오류 2건(dev 243): `held_can_carry`에서 `no`+cyan(W7 재현, pickup 칠), `held_can`에서 "can 없음"(흔들린 can이 고정 영역 밖). v3는 같은 두 뷰에서 `unknown`이다. 파지 단계의 `long_beam` 종류는 v3 자세에서도 1/4이다(나머지 unknown, 끝에서 본 빔은 짧게 보여 v2 모양 가드에 걸림) — 이번 범위 밖으로 남긴다.

**사후 재생(게이트 아님, v2 test를 이미 본 뒤):** v2 test 프레임의 held 24뷰에 v3 `held_item_at_grip`을 돌리면 W7 뷰를 포함한 `held_can_carry_posture` 4뷰가 모두 `unknown`, 나머지는 정답 또는 대조군 `unknown`이다. 이것은 설계 근거를 확인했을 뿐 test 증거가 아니다.

## 한계

- 오프라인 인식 평가다. 실제 임무 성공, 실시간 실행, 통신 조건 비교는 범위 밖이다.
- `held_*`·`carry_*`는 fixture다. 파지 성공률이 아니다. fixture의 물건은 세계 수직 자세를 유지하지만 물리 파지는 집게에 강체로 붙는다; 그 경우는 `grasp_pose` 경로이며 **렌더 평가는 하지 않았다**(합성 테스트만).
- 렌더 근접 절단 0.0297 m는 시뮬레이터 카메라의 성질이다. 실물 렌즈에는 다른 근접 한계와 초점 흐림이 있으므로 실물에서 다시 재야 한다.
- 모양 단서는 정당한 렌더에서 발동한 적이 없다. 발동 조건(띠에 팀 색이 전혀 없음)이 생기면 신뢰도 0.70의 모양 답이 나온다.
- 파지 단계 자세는 렌더에서 도달만 확인했다. 자기 충돌·실제 서보 하중은 재지 않았다.
- 분할당 시드 4개 규모다. 사례 단위로 읽고 다른 실험 수치와 합산하지 않는다.
