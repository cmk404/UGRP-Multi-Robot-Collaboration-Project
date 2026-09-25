# 2026-09-26 자기 손목 카메라 인식 v2 — 동료 판별·팀 화물 부분 가시·들고 있는 can (패키지 B 후속)

- **목적:** [패키지 B v1](../2026-09-26-zone-own-perception/README.md)의 판단 4종에 이어, 한국어 대화 연구가 필요로 하는 세 가지를 **자기 손목 어안 RGB만으로** 답한다.
  1. **동료 로봇 vs 미등록 물체** — “문에 로봇이 서 있다”와 “문이 물체로 막혔다”는 서로 다른 한국어 보고여야 한다. 판별이 있어야 메시지가 있다.
  2. **팀 화물 부분 가시** — `long_beam`·`heavy_crate`·`tri_frame`을 pickup 슬롯, 파지 손잡이 접근, 운반 중 부분 시야에서 종류와 “내 손잡이가 여기 있다”를 판단한다.
  3. **들고 있는 can** — v1 dev에서 합의된 CARRY 자세는 들린 can을 담지 못했다. 관측 자세를 제안·측정하고, 그 자세를 지나는 동안 실제로 물려 있는지 접촉 프로필별로 확인한다.
- **v1은 그대로 둔다.** `harness/zone_own_perception.py`와 `harness/zone_own_outcome.py`는 **한 줄도 고치지 않았다.** v2는 두 모듈을 import해서 상수·헬퍼를 재사용하고 새 판단만 추가한다. `tests/test_zone_own_perception_v2.py::test_v1_preregistered_constants_are_unchanged_by_v2`와 `::test_v1_judgments_answer_the_same_with_and_without_v2_imported`이 이를 검사한다.
- **코드:** `harness/zone_own_perception_v2.py`(판단 4종), `harness/zone_own_outcome_v2.py`(v1 정책 상속 트래커), `scripts/eval_zone_own_perception_v2.py`(render/score), `scripts/probe_held_can_posture.py`(자세 탐색 + retention 물리), `tests/test_zone_own_perception_v2.py`.

## 판단과 답 계약

모든 판단은 v1과 같이 `yes` / `no` / `unknown` + 신뢰도를 돌려준다. `unknown`은 1급 답이며 어떤 게이트도 “안 보인다”를 `no`로 바꾸지 않는다. `no`는 항상 근거를 든다.

| 판단 | 질문 | `yes` | `no` | 주 근거 |
|---|---|---|---|---|
| `peer_in_lane` | 앞 차선의 것이 동료 MasterPi인가 | 로봇 도색 signature | 로봇 도색 없음(`unmapped_object`) 또는 차선이 비어 있음(`clear`) | 외관만 (자세·명령 로그 사용 안 함) |
| `team_cargo_identity` | 주문서의 팀 화물이 내 앞의 그것인가 | 그 종류의 색 창 + 실루엣 | 다른 **팀 종류**가 결정적으로 이김 | 카탈로그 색/모양 |
| `team_cargo_handle` | 내 손잡이가 여기 있는가 | 닿는 범위 안의 손잡이 | 손잡이를 찾았고 전부 범위 밖 | 검은 lug/밴드의 바닥 접점 |
| `held_item` | 들고 있는 것이 주문한 것인가 | 그 종류가 영역을 채움 | 자세가 담는 종류가 영역에 없음 | held-check 자세의 영역 커버리지 |

## 입력 경계

v1과 동일하다. 판단 함수가 받는 것은 **자기 손목 RGB 1장, 자기가 발행한 팔 PWM(서보 3·4·5·6), 버전이 있는 정적 지도 dict, 시나리오 설정의 주문서, 주입된 자기 위치 belief**뿐이다. TOP 프레임과 그 파생물, 시뮬레이터 pose/접촉/측정 관절/세그멘테이션, 교사 단계·결과, **다른 로봇의 영상·명령 로그·위치**는 들어가지 않는다. 동료 판별도 다른 로봇의 상태를 받지 않고 **자기 영상의 외관**만 쓴다.

`tests/test_zone_own_perception_v2.py::test_v2_runtime_judgments_never_import_mujoco_or_read_sim_state`는 새 인터프리터에서 v2 런타임 두 모듈을 import하고 판단 4종을 모두 호출한 뒤 `sys.modules`에 `mujoco`와 장면 모듈이 없음을 확인한다. `::test_v2_runtime_sources_do_not_mention_sim_state_or_top_inputs`는 소스에 `import mujoco`, `mj_forward`, `detect_top`, `.xpos`, `.qpos`, `top_camera`, `segmentation`, `world.data`, `world.model`이 없음을 확인한다.

위치 belief는 v1과 같은 **주입값** 3단계(`gt_stub_eval_only`, `noise_30mm`, `noise_60mm`)다. `gt_stub_eval_only`는 위치 추정 결과가 아니라 stub이다. v2 판단 중 belief를 쓰는 것은 `peer_in_lane`(지도 대조)뿐이며, 나머지 셋은 belief와 무관하다.

## 들고 있는 can의 관측 자세 (측정 결과)

`robot_cam`은 gripper 프레임 (.067, 0, .0136), `grip_site`는 (.100, 0, 0) — **카메라와 들린 물체는 같은 강체**다. 따라서 어떤 자세도 물체를 *공구 좌표계에서* 움직이지 못한다. 자세가 바꾸는 것은 공구축과 물체의 **월드 수직축 사이의 각도**, 그리고 배경·조명이다. `scripts/probe_held_can_posture.py view`로 자세마다 한 프레임을 렌더해 세그멘테이션으로 센 들린 can의 픽셀 수(평가 전용):

| 자세 | 서보 3/4/5 | 공구 pitch | 프레임 안 can | v1 held-item 영역 안 |
|---|---|---:|---:|---:|
| `carry_p30` (손목 스킬 CARRY) | 777 / 2053 / 1646 | −30.1° | 12,972 | **0** |
| `look_p20` (패키지 B 관측) | 1072 / 2400 / 1482 | −20.0° | 971 | **0** |
| `tilt_p00` | 1100 / 1900 / 1700 | +7.9° | 0 | 0 |
| `tilt_m20` | 1320 / 1900 / 1700 | +27.7° | 72,446 | 32,560 |
| **`high_m45` (채택)** | **1500 / 1700 / 1900** | **+43.9°** | **107,098** | **73,657** |
| `near_m55` | 1700 / 2150 / 1560 | +52.0° | 91,596 | 75,079 |
| `tilt_m70` | 1880 / 1900 / 1700 | +78.1° | 6,595 | 6,489 |

`carry_p30`의 0 px가 v1 dev 기록(“윗면 테두리만 프레임 가장자리에 3,017 px, 영역 안 0 px”)을 재현한다. 채택한 **`HELD_CHECK_POSTURE = {1:1500, 3:1500, 4:1700, 5:1900, 6:1500}`**는 grip site를 0.143 m 앞·0.299 m 높이에 두고 공구를 43.9° 위로 들어, can이 렌즈 앞에 선다. 영역(ROI)은 v1과 **같은 사각형**이며 바뀐 것은 자세다.

**상호 보완성(중요):** 렌더 dev에서 held-check 자세의 들린 can은 영역 안 26,064 px인데, 들린 **상자·tile은 프레임 어디에도 0 px**이다. 반대로 CARRY는 상자·tile을 담고 can을 담지 못한다. 그래서 각 자세는 **자기가 담는 종류에 대해서만 부재를 주장**한다(`HELD_CHECK_FRAMED_KINDS = ('can',)`, CARRY는 v1의 목록). 코호트의 `held_can_carry_posture`·`held_box_held_check`는 이 사실을 확인하는 대조 사례이며 `unknown`이 정답 동작이다.

## retention (물리, weld OFF, 교사 파지)

`scripts/probe_held_can_posture.py retention`: 교사(`scripts/cargo_formation_teacher.py`, 정답 pose·IK — 교사 조건이며 학생 성공이 아니다)가 can을 실제로 집어 든 뒤, CARRY → 관측 자세 → 유지 → CARRY 한 주기를 발행 명령으로만 돈다. 매 스텝 `data.eq_active`를 검사하며 **weld·등식 제약은 0**이다. 슬립은 gripper 프레임에서 본 can 위치의 변화다.

| 접촉 프로필 | 자세 | 유지 | 최대 슬립 | 최종 슬립 | 접촉 상실 | 주기 완주 |
|---|---|---:|---:|---:|---|---|
| **`cargo_noslip_v1`** | `high_m45` | 6 s | **0.044 mm** | 0.025 mm | 없음 | 예 |
| `cargo_noslip_v1` | `high_m45` | 20 s | 0.043 mm | 0.031 mm | 없음 | 예 |
| `local_contact_fine` | `high_m45` | 6 s | 0.485 mm | 0.485 mm | 없음 | 예 |
| `local_contact_fine` | `high_m45` | 20 s | 1.254 mm | 1.254 mm | 없음 | 예 |
| `cargo_noslip_v1` | `tilt_m20` | 6 s | 0.042 mm | 0.023 mm | 없음 | 예 |
| `local_contact_fine` | `tilt_m20` | 6 s | 0.604 mm | 0.604 mm | 없음 | 예 |
| `cargo_noslip_v1` (게이트 W9, 시드 171) | `high_m45` | 6 s | 0.044 mm | 0.024 mm | 없음 | 예 |
| `local_contact_fine` (시드 171) | `high_m45` | 6 s | 0.485 mm | 0.485 mm | 없음 | 예 |

두 프로필 모두 can을 놓치지 않았고, `cargo_noslip_v1`이 약 13–29배 조용하다. 주 프로필은 팀 화물 경로가 고르는 `cargo_noslip_v1`이다. 이것은 **한 시드의 교사 파지 물리 결과**이며 손목 RGB 스킬의 파지 성공률이 아니다.

## 데이터 생성(render)과 채점(score) 분리

v1과 같은 구조다. `render`는 표준 `CargoZoneScene`(`cargo_noslip_v1`, weld OFF)을 동기 SIM으로 세우고, 뷰마다 로봇·화물·미등록 장애물을 배치해 **로봇이 받을 수 있는 것만** `frames/`에 쓴다(자기 손목 JPEG, 자기 발행 PWM, 지도 id/버전, 주문서). 정답(world pose, 세그멘테이션, 각 판단의 정답 답안)은 `eval-labels/`에만 둔다. `score`는 프레임에 판단을 돌린 뒤 라벨과 비교한다. 교사 정답은 데이터 생성과 채점에만 쓰였다.

반드시 함께 읽어야 하는 사항:

- **`carry_*`·`held_*` 뷰는 물리로 잡은 것이 아니다.** 카탈로그 파지 규약으로 물건을 집게 사이에 놓고 물리를 진행하지 않은 채 한 프레임을 렌더한 **인식용 fixture**다. weld는 OFF고 보조 고정도 없다. 자세를 지나며 실제로 물려 있는지는 위 retention 절의 별도 물리 결과다.
- **`object_*` 뷰는 정적 지도에 없는 `unmapped_block`(0.17×0.17×0.22 m, rgba `.36 .28 .21`, 0.60 kg)을 놓고, `peer_*` 뷰는 같은 자리에 두 번째 MasterPi를 놓는다.** 이것이 시험 조건이며 manifest에 크기·색·자세와 장면 해시를 남긴다.
- 카메라 배치/FOV, 로봇 외관, 상자 도색, 화물 카탈로그는 바꾸지 않았다. `maps/zones/*`, `sim/zone_scene.py`, `sim/zone_arena.py`, `sim/zone_cargo.py`는 수정하지 않았다.
- **뷰마다 관측 tick 3개**를 같은 정면 pan(1500)·같은 주기(1.0 SIM s)로 둔다. tick 사이에 자기 미세 이동(±15 mm, ±1.2°)을 넣어 반복 관측이 재렌더가 아니게 했다. tick 수·주기·pan은 모든 뷰·모든 통신 조건에서 동일하므로 관측 비용이 조건마다 달라질 수 없다.
- 팀 화물 접근 지점은 **두 자세로 두 번** 렌더한다(`near_*`는 이동 관측 자세, `grasp_*`는 파지 전 관측 자세). dev에서 찾은 이유는 아래 6번이다.

## 분할

`experiments/2026-09-26-zone-own-perception-v2/split.json` (SHA-256 앞 16자리 `f81937926873ff34`). v1 패키지 B 분할(dev 41–44, test 71–74)과 **겹치지 않는 새 시드**다.

| 분할 | 장면·시드 |
|---|---|
| dev | `zone_wide_door` 141, `zone_wide_corridor` 142, `zone_wide_two_doors` 143, `zone_wide_door` 144 |
| test | `zone_wide_door` 171, `zone_wide_corridor` 172, `zone_wide_two_doors` 173, `zone_wide_corridor` 174 |

시드당 27뷰 = 분할당 108뷰, 324프레임. 판단별 뷰 수: `peer_in_lane` 24, `team_cargo_identity` 60, `team_cargo_handle` 36, `held_item` 24(한 뷰가 두 판단을 묻는 경우가 있다). 임계값은 dev에서만 골랐고, test는 이 문서의 사전 등록을 커밋한 뒤 **한 번만** 렌더·채점한다.

## 사전 등록한 임계값 (test 채점 전 고정)

`harness/zone_own_perception_v2.py`에서 `PREREGISTERED` 주석이 붙은 상수다. 근거는 모두 dev 측정값이다.

| 항목 | 값 | 근거 (dev 측정) |
|---|---|---|
| 로봇 주황 | H 12–26, S ≥ max(120, 0.55×프레임 75퍼센타일) | 모델 재질 `orange .95 .60 .03` → H 19 / S 247. 혼동 후보(구역 A 칠 H 8–18, `tri_frame` S ≤ 140, 갈색 블록 S 106)는 이 하한 아래 |
| 로봇 알루미늄 | S ≤ 70, V 0.22–1.35×75퍼센타일, 절대 ≤ 245 | `aluminum .48 .50 .52` → S 20 / V 133. 상한이 1.0보다 큰 이유: 패널이 바닥보다 밝을 수 있다 |
| 동료 판정 | (주황 ≥ .06 **그리고** 회색 ≥ .05) **또는** 주황 ≥ .30 | 동료 성분 주황 .21–.91 / 회색 .01–.38; 물체·벽 성분 주황 .000–.001 |
| 물체 판정 | 주황 ≤ .010 **그리고** 로봇 3재질 설명 ≤ .35 | 갈색 블록 주황 .000 / 설명 .012–.29 |
| 동료 후보 합집합 | 차선의 미등록 후보 전체 union에도 같은 규칙 | 한 로봇이 팔(주황)·패널(회색)·차대(검정)로 **쪼개져** 잡힌다 |
| 동료 후보 추출 | 거리 0.28–2.50 m, 면적 ≥ 300 px, 높이 게이트(v1), 차선 반폭+여유 | v1은 테두리 성분을 버려 **가까운 동료(14,534 px)를 후보로 만들지 못했다**; 자기 집게는 0.28 m보다 가까워 거리 하한으로 대신 걸러진다 |
| 팀 화물 색 창 | beam H 28–58 / S 150–255, crate H 156–178 / S 45–210, frame H 12–32 / S 22–140 | 실루엣 안 중앙값 beam H 40→**조명에서 H 30** / S 173–212, crate H 169 / S 99, frame H 23 / S 43. beam의 S 하한이 `tri_frame`·노란 상자와의 경계 |
| 팀 화물 결정 | 색 점유 ≥ .010, 2위 대비 1.8배, 성분 ≥ 260 px | dev 점유 beam .085 / crate .041 / frame .047, 빈 프레임 ≤ .009 |
| 실루엣 가드 | beam 종횡비 ≥ 2.6, crate ≤ 5.0, frame은 2성분 이상 또는 종횡비 ≤ 5.0 | dev beam 12.7 / crate 2.46 / frame 3.46(4조각) |
| 부분 시야 예외 | 성분이 유효영역의 25 % 이상이면 실루엣 검사를 건너뛰고 신뢰도 .70 | 운반 중 beam은 종횡비 1.43·점유 65 %(슬롯 8 %, 접근 13 %) |
| 손잡이 | V ≤ 0.34×75퍼센타일, S ≤ 90, ≥ 90 px, 화물 색에서 14 px 이내, 거리 ≤ 1.20 m | 실루엣 안 검은 부품 crate 2,379+2,161 px, frame 5,282+2,144+744 px, beam 밴드 269 px |
| 닿는 범위 | 전방 0.18–0.45 m, 좌우 ≤ 0.10 m | 교정 파지 반경 0.155 m와 손목 스킬 접근 standoff 0.40 m를 함께 담는 **파지 전** 범위. “이미 집게 안에 있다”는 뜻이 아니다 |
| 파지 전 관측 자세 | `{1:2000, 3:973, 4:2212, 5:1959, 6:1500}` (공구 pitch −54.9°) | 이동 관측 자세(−20°)는 바닥을 0.45 m부터 본다. dev에서 **0.24–0.32 m의 가까운 손잡이 5건**이 프레임 밖이라 확신 오류가 됐다 |
| held-check 자세 | `{1:1500, 3:1500, 4:1700, 5:1900, 6:1500}`, 영역 = v1 CARRY_ROI | 위 자세 표 |
| held-check 담는 종류 | `('can',)` | can 26,064 px 대 상자·tile 0 px |
| held 영역 밝기 | 75퍼센타일 V ≥ max(28, 0.30×프레임 75퍼센타일) | 검은 프레임은 부재의 근거가 아니다 |
| 판정 커밋 | v1과 동일 (주기 1.0 SIM s, 연속 2 tick 동일·신뢰도 ≥ 0.65, 기한 40 s, 최대 24관측) | `harness/zone_own_outcome.py`를 상속 |

**관측 가능(observable) 정의(사전 고정):**
- `peer_in_lane`: 진실이 `yes`면 동료가 세그멘테이션에서 ≥ 300 px, 물체가 차선에 있으면 블록이 ≥ 300 px, 그 외에는 항상 관측 가능.
- `team_cargo_identity`: 해당 팀 화물이 ≥ 400 px이고 실루엣 안 **채도 90퍼센타일 ≥ 60**. 이 채도 조건은 프레임의 성질이며 판단 임계값이 아니다 — 렌즈에서 2–5 cm인 운반 화물은 조명에 날아가거나(빔 S 186–190 유지) 팔 그림자에 들어간다(crate S 0 / V 7–13).
- `team_cargo_handle`: 위 조건 + **검은 lug/밴드가 ≥ 120 px** + 놓인 화물이 주문한 종류일 때.
- `held_item`: 들린 물건이 held-check 영역 안에서 ≥ 500 px일 때, 또는 아무것도 들지 않았을 때.

## 사전 등록한 게이트 (test, 뷰 단위, `gt_stub_eval_only`)

“확신 오류”는 `answer != truth`이고 `answer != unknown`이며 신뢰도 ≥ 0.65(커밋 임계값)인 뷰다. 이것만이 영수증·메시지 주장이 될 수 있어 별도로 센다.

| ID | 게이트 | dev 값 |
|---|---|---|
| W1 | 판단별 확신 오류 ≤ 1뷰 | 0 / 0 / 0 / 0 |
| W2 | `peer_in_lane`의 관측 가능 뷰 확신 오류 0 | 0 |
| W3 | 판단별 결정한 뷰의 정확도 ≥ 0.85 | 1.00 / 1.00 / 1.00 / 1.00 |
| W4 | 판단별 관측 가능 뷰의 정확도 ≥ 0.60 | .958 / .780 / .636 / 1.00 |
| W5 | 판단별 관측 가능 뷰의 `unknown` 비율 ≤ 0.45 | .042 / .220 / .364 / .000 |
| W6 | `peer_*` 12뷰 중 ≥ 9뷰가 `yes`+`peer_robot`, `object_*` 8뷰 중 ≥ 6뷰가 `no`+`unmapped_object` | 12/12, 7/8 |
| W7 | `held_can` ≥ 3/4 `yes`, `held_can_absent` ≥ 3/4 `no`, 대조 2사례(`held_can_carry_posture`, `held_box_held_check`)는 확신 답 0 | 4/4, 4/4, 0 |
| W8 | `team_cargo_identity`의 slot·near 단계 확신 오류 0 | 0 |
| W9 | test 계열 시드(171)에서 retention 재실행: 주기 완주, 접촉 상실 없음, `cargo_noslip_v1` 최대 슬립 ≤ 0.5 mm, `eq_active` 0 | (dev 시드 11에서 0.044 mm) |
| W10 | `tests/test_zone_own_perception_v2.py` 전체 통과(v1 불변 검사 포함) | 24/24 |

## dev 결과 (임계값 선택에만 사용, 합산 금지)

뷰 단위, `gt_stub_eval_only`, 108뷰. 원본은 `outputs/2026-09-26-zone-own-perception-v2/dev-frames-4`(13 MB, 로컬)와 `dev-score-4`이며 이 폴더의 `dev-summary.json`이 같은 내용이다.

| 판단 | 뷰 | unknown | 전체 정확도 | 결정 정확도 | 확신 오류 | 관측 가능(뷰 / 정확도 / unknown / 확신 오류) |
|---|---:|---:|---:|---:|---:|---|
| `peer_in_lane` | 24 | .042 | .958 | **1.00** | **0** | 24 / .958 / .042 / 0 |
| `team_cargo_identity` | 60 | .350 | .650 | **1.00** | **0** | 50 / .780 / .220 / 0 |
| `team_cargo_handle` | 36 | .611 | .389 | **1.00** | **0** | 22 / .636 / .364 / 0 |
| `held_item` | 24 | .333 | .667 | **1.00** | **0** | 12 / 1.00 / .000 / 0 |

단계별(뷰 / unknown / 정답 / 오답):

| 판단 | 단계 |
|---|---|
| `team_cargo_identity` | slot 20/4/16/0 · near 12/1/11/0 · grasp 16/12/4/0 · carry 12/4/8/0 |
| `team_cargo_handle` | slot 20/10/10/0 · grasp 16/12/4/0 |
| `held_item` | held_check 16/4/12/0 · carry 8/4/4/0 |

belief 민감도(관측 가능 뷰 정확도 / 확신 오류): `peer_in_lane` — `gt_stub_eval_only` .958/0, `noise_30mm` .875/**1**, `noise_60mm` .917/**1**. 나머지 세 판단은 belief를 쓰지 않아 변하지 않는다(.780/0, .636/0, 1.00/0). σ 30 mm에서 `peer_near_wall` 1뷰가 지도 벽 대조를 넘겨 확신 오류가 됐다. **동료 판별을 실행에 쓰려면 위치 belief 품질이 필요하다**는 v1과 같은 결론이다.

### dev에서 찾아 고친 것 (모두 dev 근거)

1. **v1의 테두리 규칙이 가까운 동료를 지운다:** v1 `judge_route_blockage`는 어안 테두리에 닿는 성분을 후보에서 빼는데(자기 집게 오탐 방지), 가까이 선 동료(14,534 px)도 그렇게 사라져 후보가 0이었다. v2는 **자체 후보 추출**에 거리 하한 0.28 m을 쓴다(자기 집게는 렌즈에서 3–15 cm이므로 그보다 가까운 접점만 만든다). v1은 그대로 두고, v1의 답은 같이 기록한다.
2. **한 로봇이 여러 성분으로 쪼개진다:** 조각별로는 주황 .63–.91·회색 .01–.05라서 “주황+회색” 조건을 통과하지 못했다. 차선의 미등록 후보 **union**에도 같은 규칙을 적용하고, 조각 하나가 주황 ≥ .30이면 그것만으로 로봇으로 본다.
3. **“한 색이 지배한다”는 물체 판정은 틀렸다:** 갈색 블록은 렌더에서 채도가 낮아 지배 색 비율이 .012–.024에 그친다. 물체 판정을 **“로봇 3재질이 없다”**로 바꿨다(주황 ≤ .010, 설명 ≤ .35).
4. **빔 색이 조명에서 H 40 → 30으로 밀린다:** 고정 H 34–56 창은 빔을 놓쳤다(점유 .010 → 창을 H 28–58로 넓히면 .967–.998). 넓힌 창이 `tri_frame`·노란 상자와 겹치므로 **채도 창**으로 분리했다(빔 S ≥ 150, frame S ≤ 140).
5. **운반 중에는 실루엣을 잴 수 없다:** 렌즈에서 3 cm인 빔은 종횡비 1.43·유효영역 65 %로 잘린다. 성분이 25 % 이상이면 실루엣 검사를 건너뛰고 신뢰도를 .70으로 낮춘다. 운반 `heavy_crate`는 팔 그림자에서 S 0 / V 7–13이라 **색 정보가 아예 없다**(관측 불가로 표시, `unknown`).
6. **접근 자세가 두 판단을 동시에 만족시키지 못한다:** 이동 관측 자세(−20°)는 바닥을 0.45 m부터 보므로 닿는 범위의 손잡이(0.24–0.32 m)가 프레임 밖이고, dev에서 “손잡이는 다른 곳”이라는 **확신 오류 5건**이 나왔다. 파지 전 관측 자세(−54.9°)는 그 바닥을 보지만 화물 종류를 못 알아본다(빔·frame 4/4 `unknown`). 그래서 접근 지점을 두 자세로 나눠 렌더하고, 각 판단을 자기 자세에서 묻는다. **연구 설계상의 결과: 종류는 이동 자세에서 먼저 확정하고, 손잡이는 파지 전 자세에서 확인해야 한다.**
7. **held-check 자세는 can만 담는다:** 상자·tile은 이 자세에서 0 px다. v1의 “프레임 가장자리에 화물 색이 있으면 unknown” 검사는 팔을 든 이 자세에서 배경(노란 상자·구역 칠)을 잡아 항상 발동하므로 held-check에서는 쓰지 않고, 대신 **영역이 밝은지**만 확인한 뒤 “주문한 종류가 영역에 없다”를 `no`로 답한다. `empty`(아무것도 없음)와는 구분해 `expected_kind_absent`로 표기한다.
8. **tick 2개로는 커밋이 자주 실패한다:** 손잡이와 들린 can이 한 tick에서는 보이고 다음 tick에서는 안 보이는 일이 있어, 연속 2 tick 규칙이 `unknown`으로 끝났다. 모든 뷰의 tick을 **3개**로 통일했다(주기·pan은 그대로).

## 원본 위치와 해시

| 항목 | 위치 |
|---|---|
| 자세 탐색 | `outputs/2026-09-26-held-can/view-1/view.json` (프레임 10장 포함) |
| retention | `outputs/2026-09-26-held-can/ret-*` (`retention.json`, `trace.jsonl`, `events.json`, `scene.xml`) |
| dev 프레임·라벨 | `outputs/2026-09-26-zone-own-perception-v2/dev-frames-4`(조정용), `dev-frames-clean`(`2a46f9d` 재현) — 각 13 MB, 로컬 보관, Git 제외 |
| dev 채점 | `dev-score-4`(조정용), `dev-score-clean`(`2a46f9d`); 이 폴더의 `dev-summary.json`은 `dev-score-clean`과 같은 내용 |
| test 프레임·라벨 | `outputs/2026-09-26-zone-own-perception-v2/test-frames` (13 MB, 로컬 보관, Git 제외) |
| test 채점 | `test-score`; 이 폴더의 `test-summary.json`이 같은 내용 |
| 장면 해시 (변형·시드 / CargoZoneScene / 추가물 포함 / 주문서) | door 141 `732f99f5` / `9623a92d` / `8462772a`; corridor 142 `70c96f2f` / `08404277` / `9a0b1813`; two_doors 143 `94eeba2a` / `55a3069f` / `3e3c9ec8`; door 144 `ea31ac8e` / `06b4603a` / `337685d2` |
| 접촉 | `cargo_noslip_v1`, 화물 finger 접촉 pair 72쌍 미러, weld OFF |

dev 렌더 wall 279.2 s, 1분 부하 7.61 → 6.55. 로컬 보관은 원격 백업이 아니며 `outputs/`는 Git에 포함되지 않는다.

## test 결과 (1회 채점, 사전 등록 뒤)

렌더·채점 소스 `2a46f9d`(깨끗한 작업 트리 — manifest의 `source_dirty: false`), 렌더 `outputs/2026-09-26-zone-own-perception-v2/test-frames`(108뷰·324프레임, wall 317.8 s, 1분 부하 8.52 → 5.28), 채점 `test-score`(같은 내용을 이 폴더 `test-summary.json`에 보존). 뷰 단위, `gt_stub_eval_only`.

**dev 재현:** 사전 등록을 커밋한 뒤 같은 dev 분할을 깨끗한 소스 `2a46f9d`로 다시 렌더·채점했고(`dev-frames-clean` 108뷰, wall 223.1 s, 부하 4.35 → 3.26), 세 belief 수준의 모든 판단에서 뷰 단위 지표가 조정용 실행(`dev-score-4`)과 **완전히 일치**했다. 이 폴더의 `dev-summary.json`은 재현한 `dev-score-clean`이다.

| 판단 | 뷰 | unknown | 전체 정확도 | 결정 정확도 | 확신 오류 | 관측 가능(뷰 / 정확도 / unknown / 확신 오류) |
|---|---:|---:|---:|---:|---:|---|
| `peer_in_lane` | 24 | **.000** | 1.000 | **1.00** | **0** | 23 / 1.00 / .000 / 0 |
| `team_cargo_identity` | 60 | .300 | .700 | **1.00** | **0** | 49 / .857 / .143 / 0 |
| `team_cargo_handle` | 36 | .500 | .472 | .944 | 1 | 20 / .850 / .100 / 1 |
| `held_item` | 24 | .292 | .667 | .941 | 1 | 12 / 1.00 / .000 / 0 |

사례별(뷰 수 / unknown / 정답 / 오답):

| 판단 | 사례 |
|---|---|
| `peer_in_lane` | `peer_in_passage` 4/0/4/0 · `peer_in_open` 4/0/4/0 · `peer_near_wall` 4/0/4/0 · `object_in_passage` 4/0/4/0 · `object_in_open` 4/0/4/0 · `clear_lane` 4/0/4/0 |
| `team_cargo_identity` | slot: beam 4/0/4/0 · crate 4/0/4/0 · frame 4/0/4/0 · wrong_kind 4/0/4/0 · no_cargo 4/4/0/0 / near: beam 4/0/4/0 · crate 4/1/3/0 · frame 4/0/4/0 / grasp: beam 4/3/1/0 · crate 4/1/3/0 · frame 4/4/0/0 · wrong_end 4/1/3/0 / carry: beam 4/0/4/0 · crate 4/4/0/0 · frame 4/0/4/0 |
| `team_cargo_handle` | slot: beam 4/1/3/0 · crate 4/0/4/0 · frame 4/0/3/**1** · wrong_kind 4/4/0/0 · no_cargo 4/4/0/0 / grasp: beam 4/1/3/0 · crate 4/1/3/0 · frame 4/3/1/0 · wrong_end 4/4/0/0 |
| `held_item` | `held_can` 4/0/4/0 · `held_can_absent` 4/0/4/0 · `held_can_wrong_kind` 4/0/4/0 · `held_box_carry_posture` 4/0/4/0 · `held_can_carry_posture`(대조) 4/3/0/**1** · `held_box_held_check`(대조) 4/4/0/0 |

### 게이트 판정 — 10개 중 9개 통과, W7 실패

| ID | 기준 | 결과 | 판정 |
|---|---|---|---|
| W1 | 판단별 확신 오류 ≤ 1 | 0 / 0 / 1 / 1 | 통과 |
| W2 | `peer_in_lane` 관측 가능 뷰 확신 오류 0 | 0 | 통과 |
| W3 | 결정한 뷰 정확도 ≥ 0.85 | 1.00 / 1.00 / .944 / .941 | 통과 |
| W4 | 관측 가능 뷰 정확도 ≥ 0.60 | 1.00 / .857 / .850 / 1.00 | 통과 |
| W5 | 관측 가능 뷰 unknown ≤ 0.45 | .000 / .143 / .100 / .000 | 통과 |
| W6 | `peer_*` ≥ 9/12, `object_*` ≥ 6/8 | **12/12**, **8/8** | 통과 |
| W7 | `held_can` ≥ 3/4 `yes`, `held_can_absent` ≥ 3/4 `no`, 대조 2사례 확신 답 0 | 4/4, 4/4, **확신 답 1건** | **실패** |
| W8 | `team_cargo_identity` slot·near 확신 오류 0 | 0 | 통과 |
| W9 | test 계열 시드(171) retention 재실행 | `cargo_noslip_v1` 최대 0.044 mm, 접촉 상실 없음, 주기 완주, `eq_active` 0 (`local_contact_fine` 0.485 mm) | 통과 |
| W10 | `tests/test_zone_own_perception_v2.py` 전체 통과 | 24/24 | 통과 |

**동료/물체 판별은 test에서 완벽했다:** `peer_in_lane` 24뷰 모두 정답이고 `unknown` 0, 확신 오류 0이다. 동료 12뷰는 전부 `yes`+`peer_robot`, 미등록 물체 8뷰는 전부 `no`+`unmapped_object`로 답했다. 연구가 요구한 “문에 로봇이 서 있다” vs “문이 물체로 막혔다”의 구분이 test 분할에서 성립한다.

### 실패와 오류 2건 (원인 확인)

1. **W7 실패 — CARRY 자세에서 배경을 들린 물건으로 오인:** `zone_wide_corridor-s172-held_can_carry_posture`에서 답은 `no`+`cyan`(신뢰도 0.82, `OTHER_KIND_FILLS_VIEW`)이고 진실은 `yes`(can을 들고 있음)다. held-item 영역의 청록 점유가 **0.44**인데 들린 can은 그 영역에 **0 px**이다. 즉 로봇이 청록 상자 앞에 서 있었고, CARRY 자세에서 들린 can이 프레임 밖이라 영역이 **배경**을 보여줬다. v1의 “가장자리에 화물 색이 있으면 `empty`를 주장하지 않는다” 검사는 *부재* 주장만 막고 *다른 종류* 주장은 막지 않는다. v2는 v1의 CARRY 경로를 그대로 쓰므로 이 약점을 그대로 물려받았다. 이 사례는 대조군이라 결론에 유리하게 쓰이지 않지만, **CARRY 자세에서 `holding_item`/`held_item`의 `no`+다른 종류는 신뢰할 수 없다**는 것이 test에서 드러난 결과다. 제안(미구현, v3 후보): 영역의 색 성분이 들린 물건의 예상 위치·크기와 맞는지 확인하거나, 배경 거리(바닥 투영)로 영역 안의 먼 색을 배제한다.
2. **`team_cargo_handle` 확신 오류 1건 — 경계 7 mm:** `zone_wide_corridor-s174-slot_tri_frame`에서 답은 `no`+`handle_elsewhere`, 진실은 `yes`다. 검출한 손잡이 접점은 전방 **0.457 m**이고 범위 상한은 0.450 m — 7 mm 초과다. 정답 손잡이 v1은 0.408 m로 범위 안이다. 원인은 체계적이다: lug는 46 mm 높이인데 화물 몸체가 lug 바닥을 가리면 성분의 최하단이 실제 접지점보다 위에 있어 바닥 투영이 거리를 **과대평가**한다. 제안(미구현, v3 후보): lug의 알려진 높이(46 mm)로 접점을 보정하거나, 범위 경계 ±30 mm를 `unknown`으로 둔다. 사후에 임계값을 옮기지 않았다.

### belief 민감도 (관측 가능 뷰 정확도 / 확신 오류)

| belief | `peer_in_lane` | `team_cargo_identity` | `team_cargo_handle` | `held_item` |
|---|---|---|---|---|
| `gt_stub_eval_only` | 1.00 / 0 | .857 / 0 | .850 / 1 | 1.00 / 0 |
| `noise_30mm` | 1.00 / 0 | .857 / 0 | .850 / 1 | 1.00 / 0 |
| `noise_60mm` | .870 / **1** | .857 / 0 | .850 / 1 | 1.00 / 0 |

belief를 쓰는 판단은 `peer_in_lane`뿐이다. σ 60 mm에서 `peer_in_passage` 1뷰가 지도 대조를 넘겨 확신 오류가 됐다(σ 30 mm는 영향 없음). 나머지 세 판단은 지도를 쓰지 않아 belief와 무관하다. **결론: 동료/물체 판별을 실행에 쓰려면 자기 위치 belief 오차가 대략 30 mm 수준이어야 한다.** 위치 추정 정확도는 다른 패키지의 결과로 확인해야 하며 이 기록이 그것을 대신하지 않는다.

### 장면 해시 (test)

door 171 `11b103a9` / `e9610d3b` / `a15e18d3`; corridor 172 `58ea94ec` / `5767f669` / `d4fbd29a`; two_doors 173 `819fcaf4` / `111b89ba` / `243b400c`; corridor 174 `28bc241a` / `92dffcda` / `cc5ca107` (CargoZoneScene / 추가물 포함 / 주문서). 원본 프레임·라벨 13 MB는 로컬 `outputs/`에만 있고 Git에 없다.

## 한계

- 오프라인 인식 평가다. 실제 임무 성공, 실시간 실행, 통신 조건 비교는 이 기록의 범위가 아니다.
- `carry_*`·`held_*` 뷰는 물리 파지가 아니라 자세만 맞춘 fixture다. 파지 성공률로 읽지 않는다. retention은 교사 파지의 물리 결과이며, W9의 시드 171은 경기장 상자 배치만 바꾸므로 파지 기하는 시드 11과 같다 — **같은 기하의 반복 실행**이지 새 조건이 아니다.
- **CARRY 자세의 `no`+다른 종류는 신뢰할 수 없다**(위 W7 실패). 들린 물건이 그 자세에서 프레임 밖이면 held-item 영역이 배경을 보여주고, 배경의 화물 색이 확신 있는 오답이 된다. v1의 `judge_holding_item`도 같은 구조다. 수정은 미구현이며 v3 후보로 제안만 남겼다.
- `team_cargo_handle`의 거리 추정은 lug 높이(46 mm)를 보정하지 않아 접점을 과대평가한다. test에서 7 mm 경계 초과로 확신 오류 1건이 났다. 사후 임계값 변경은 하지 않았다.
- 운반 중 `heavy_crate`는 자기 카메라로 재식별할 수 없다(팔 그림자, 채도 0 / 명도 7–13). 운반 중 종류 확인은 **자기 명령 이력이나 대화**에 의존해야 한다. 이것은 통신 연구에 유리한 사례가 아니라 **관측의 한계**로 읽어야 한다.
- `tri_frame`은 파지 전 자세에서 종류를 확정하지 못해 “내 손잡이가 여기”도 `unknown`으로 남는다(dev 3/4, test 3/4).
- `peer_in_lane`은 지도 대조에 belief를 쓰므로 σ 60 mm에서 확신 오류가 생긴다. 뒤에서 어두운 후면만 보이는 동료는 놓칠 수 있다(주황이 가려지면 signature가 없다).
- 분할당 시드 4개 규모다. 비율은 사례 단위로 읽고 다른 실험 수치와 합산하지 않는다.
