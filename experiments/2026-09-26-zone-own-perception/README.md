# 2026-09-26 자기 손목 카메라(어안) 전용 인식 판단 4종 — 오프라인 평가 (패키지 B)

- **목적:** 한국어 대화 연구에서 TOP 카메라가 평가 전용이 된 뒤, 각 로봇이 **자기 손목 어안 RGB만으로** 내려야 하는 인식 판단 4종을 구현하고 오프라인에서 채점한다.
- **판단:** ① pickup 슬롯의 물건 종류 ② 올바른 물건을 들었는지 ③ 구역 슬롯에 놓였는지(다시 보기) ④ 지도에 없는 전방 막힘.
- **답 계약:** 모든 판단은 `yes` / `no` / `unknown` + 신뢰도를 돌려준다. **`unknown`은 1급 답**이며 어떤 게이트도 "안 보인다"를 `no`로 바꾸지 않는다. `no`는 항상 근거(다른 종류를 봤다 / 바닥·칠이 비어 있음을 증명 / 차선이 비어 있음을 증명)를 든다.
- **코드:** `harness/zone_own_perception.py`(판단 4종), `harness/zone_own_outcome.py`(고정 주기 다중 관측 판정), `scripts/eval_zone_own_perception.py`(render/score 분리 평가기), `tests/test_zone_own_perception.py`.
- **재사용(수정 안 함):** `harness/zone_color_boxes.py`(4색 상자 `detect_own`), `harness/markerless_box.py`(어안 투영·바닥 직육면체 적합), `harness/visual_floor.py`의 바닥 측정 아이디어, `sim/masterpi_camera_profile.py`(고정 카메라 보정), `sim/zone_cargo.py`(정적 카탈로그), `scripts/eval_zone_color_detection.py`(렌더 보조 함수). `harness/wrist_zone_skill_v*.py`, `harness/visual_box_skill.py`, `harness/zone_cargo_perception*.py`, `harness/zone_rgb_outcome.py`는 읽기만 했다.

## 입력 경계

판단 함수가 받는 것은 **자기 손목 RGB 1장, 자기가 발행한 팔 PWM(서보 3·4·5·6), 버전이 있는 정적 지도 dict, 시나리오 설정에서 만든 주문서, 주입된 자기 위치 belief**뿐이다. TOP 프레임과 TOP에서 파생된 좌표·라벨·개수·완료 판정, 시뮬레이터 pose/접촉/측정 관절/세그멘테이션, 교사 단계·결과, 다른 로봇의 영상·명령 로그는 들어가지 않는다. 집게 명령은 파지 성공의 근거가 아니며 프레임이 어느 자세에서 찍혔는지 확인하는 데만 쓴다.

`tests/test_zone_own_perception.py::test_runtime_judgments_never_import_mujoco_or_read_sim_state`는 새 인터프리터에서 두 런타임 모듈을 import하고 판단 4종을 모두 호출한 뒤 `sys.modules`에 `mujoco`와 장면/로봇 모듈이 없음을 확인한다. `test_runtime_sources_do_not_mention_sim_state_or_top_inputs`는 소스에 `import mujoco`, `mj_forward`, `detect_top`, `.xpos`, `.qpos`, `top_camera`, `segmentation`이 없음을 확인한다.

자기 위치 belief는 **주입값**이다. `gt_stub_eval_only`(오차 0, 평가용 stub), `noise_30mm`(σ 30 mm·3°), `noise_60mm`(σ 60 mm·6°) 세 단계로 채점한다. `gt_stub_eval_only`는 위치 추정 결과가 아니라 stub이며, 손목 스킬 코호트(511–520)의 `gt_stub_eval_only`와 같은 표기 규칙을 따른다. 실제 위치 추정은 다른 패키지(자기 카메라 위치 추정)가 담당한다.

## 데이터 생성(render)과 채점(score) 분리

`experiments/2026-09-25-zone-rgb-color`와 같은 구조다. `render`는 표준 `ZoneScene`(`local_contact_fine`, weld OFF)을 동기 SIM으로 세우고 뷰마다 로봇·화물·미등록 장애물을 배치해 **로봇이 받을 수 있는 것만** `frames/`에 쓴다(자기 손목 JPEG, 자기 발행 PWM, 지도 id/버전, 주문서). 정답(world pose, 세그멘테이션, 각 판단의 정답 답안)은 `eval-labels/`에만 둔다. `score`는 프레임에 판단을 돌린 뒤 라벨과 비교한다. 교사 정답은 데이터 생성과 채점에만 쓰였다.

렌더에서 반드시 함께 읽어야 하는 사항:

- **`hold_*` 뷰는 물리로 잡은 것이 아니다.** 카탈로그 파지 규약(grip site 기준 오프셋)으로 물건을 집게 사이에 놓고 물리를 진행하지 않은 채 한 프레임을 렌더한다. weld는 OFF이고 보조 고정도 없다. 이것은 **인식용 fixture이며 파지 성공의 근거가 아니다.**
- **`block_*` 뷰는 정적 지도에 없는 물체 `unmapped_block`을 하나 추가한다**(0.17×0.17×0.22 m, rgba `.36 .28 .21`, 0.60 kg, freejoint). 이것이 시험 조건 자체이며 크기·색·자세와 장면 해시를 manifest에 남긴다. 다른 뷰에서는 경기장 밖(−7, −9)에 주차한다.
- 카메라 배치/FOV, 로봇 외관, 상자 도색, 화물 카탈로그는 바꾸지 않았다. `maps/zones/*`, `sim/zone_scene.py`, `sim/zone_arena.py`는 수정하지 않았다(다른 패키지 소유). pickup 슬롯 표(`P{열}-{행}`)는 지도 파일을 고치지 않고 시나리오 설정(`sim.zone_arena.layout`의 pickup 격자)에서 만든다.
- 뷰마다 관측 tick이 2–3개다. tick 사이에 로봇 자기 미세 이동(±15 mm, ±1.2°)을 넣어 반복 관측이 같은 프레임의 재렌더가 아니게 했다. 관측 pan은 정면 1500을 기본으로 하고, 다시 보기 3번째 tick만 합의된 1770을 쓴다. 정면에서 벗어난 pan은 같은 지점을 더 잘 보는 방법이 아니라 추적기의 **재관측 계획**이다.

## 분할

`experiments/2026-09-26-zone-own-perception/split.json` (SHA-256 앞 16자리 `8e37c71aadaa62c4`).

| 분할 | 장면·시드 |
|---|---|
| dev | `zone_wide_door` 41, `zone_wide_corridor` 42, `zone_wide_two_doors` 43, `zone_wide_door` 44 |
| test | `zone_wide_door` 71, `zone_wide_corridor` 72, `zone_wide_two_doors` 73, `zone_wide_corridor` 74 |

시드당 23뷰(판단별 5–6 사례) = 분할당 92뷰. 임계값은 dev에서만 골랐고, test는 이 문서의 사전 등록을 커밋한 뒤 **한 번만** 렌더·채점한다.

## 사전 등록한 임계값 (test 채점 전 고정)

`harness/zone_own_perception.py`에서 `PREREGISTERED` 주석이 붙은 상수다. 근거는 dev 측정값이다.

| 항목 | 값 | 근거 |
|---|---|---|
| 색 대역 | 상자 4색·`can` H 124–139·`tile` H 150–170 | 카탈로그 rgba에서 계산, dev 프레임으로 확인 |
| 구역 칠 hue | A 8–18, B 104–120, C 138–150 | dev 실측 고리 hue A 12–14, B 111, C 143 |
| 조명 보정 | 채도·명도 하한 = max(절대 45/28, 프레임 75퍼센타일의 0.40/0.34) | 518 실패(칠 위 어둡고 저채도 청록) |
| 슬롯 관측 | 카메라 기준 거리 ≤ 1.20 m, 슬롯 일치 ≤ 0.10 m, 내부 원 0.030 m, 기준 고리 0.075 m | 생산 근거리 적합 한계(1.2 m) |
| 비어 있음 | 바닥 `bare_share` ≥ 0.82, 구역 칠 ≥ 0.88 | dev: 빈 원 0.97, 물건 있는 원 0.72–0.76 |
| 칠 대비 | 차이 픽셀 ≥ 0.25, 최대 채널 차 ≥ 38이고 어딘가 ≥ 55, hue 허용 ±8 | dev: 빈 칠의 조명 기울기 최대 37, 물건 있는 원 63–84 |
| 들고 있음 | ROI (0.18, 0.10)–(0.82, 0.92), 직육면체 커버리지 ≥ 0.30, 2위 대비 1.6배, 빈 판정 ≤ 0.05 | dev: 상자 25%·tile 가시, `can` ROI 내 0 px |
| 막힘 | 차선 반폭 = 통로폭/2−0.02, 후보 ≥ 260 px, 접촉 높이 ≥ 0.06 m 투영의 0.55배, 지도 일치 여유 belief별 0.06/0.12/0.25 m, 지도 벽 근접 0.18 m는 `unknown` | dev: 어안 테두리 성분 8,600 px 오탐, 바닥 색의 거리 의존(0.45 m BGR 72,46,29 대 3.5 m 29,26,24) |
| 판정 커밋 | 주기 1.0 SIM s, 연속 2 tick 동일·신뢰도 ≥ 0.65, 기한 40 s, 최대 24관측 | `zone_rgb_outcome`의 고정 주기·안정성 구조 계승 |

## 사전 등록한 게이트 (test, 뷰 단위, `gt_stub_eval_only`)

"확신 오류"는 `answer != truth`이고 `answer != unknown`이며 신뢰도 ≥ 0.65(커밋 임계값)인 뷰다. 이것만이 영수증·메시지 주장이 될 수 있어 별도로 센다.

| ID | 게이트 |
|---|---|
| G1 | 판단별 확신 오류 ≤ 1뷰 |
| G2 | `placed_in_slot`·`holding_item`의 **관측 가능** 뷰에서 확신 오류 0 |
| G3 | 판단별 결정한 뷰의 정확도 ≥ 0.85 |
| G4 | 판단별 관측 가능 뷰의 정확도 ≥ 0.70 |
| G5 | 판단별 관측 가능 뷰의 `unknown` 비율 ≤ 0.35 |
| G6 | `placed_zone_B`(518 쌍: 파랑 칠 위 청록) 4뷰 중 ≥ 3뷰 `yes` — 기록된 거짓 음성 재현 안 됨 |
| G7 | 관측 불가 뷰(`slot_far`, `*_occluded`) 전체에서 확신 오류 ≤ 1 |
| G8 | `tests/test_zone_own_perception.py` 전체 통과(입력 경계 검사 포함) |

`peer_in_lane` 코호트는 게이트 대상이 아니다. 차선에 있는 동료 로봇은 "지도에 없는 물체"이기도 해서 이 판단이 `yes`로 보고한다. 동료와 물체를 구분하는 판단은 이 패키지 범위가 아니며 별도 지표로만 보고한다.

**관측 가능(observable) 정의(사전 고정):** `slot_item`·`placed_in_slot`은 슬롯 중심이 프레임 안이고 카메라 거리 ≤ 1.20 m이며 대상이 세그멘테이션에서 ≥ 90 px, 가림 사례가 아닐 때. `holding_item`은 들린 물건이 **CARRY ROI 안에서** ≥ 500 px일 때. `route_blockage`는 막힘 진실인 경우 `unmapped_block`이 ≥ 260 px일 때, 아닌 경우 항상.

## dev 결과 (임계값 선택에만 사용, 합산 금지)

렌더 `outputs/2026-09-26-zone-own-perception/dev-frames-v3`(92뷰, 209 프레임, wall 174.9 s, 1분 부하 9.05→13.31, 소스 `1cd9ea1`+미커밋 조정), 채점 `dev-score-6`. 뷰 단위, `gt_stub_eval_only`.

| 판단 | 뷰 | unknown | 전체 정확도 | 결정 정확도 | 확신 오류 | 관측 가능(뷰/정확도/확신 오류) |
|---|---:|---:|---:|---:|---:|---|
| `slot_item` | 24 | .333 | .625 | .938 | 1 (`slot_occluded`) | 16 / .938 / 0 |
| `holding_item` | 20 | .300 | .700 | 1.00 | 0 | 16 / .875 / 0 |
| `placed_in_slot` | 24 | .208 | .792 | 1.00 | 0 | 17 / .882 / 0 |
| `route_blockage` | 20 | .250 | .750 | 1.00 | 0 | 20 / .750 / 0 |

belief 민감도(뷰 단위 전체 정확도 / 확신 오류): `noise_30mm` — slot .542/0, hold .700/0, placed .708/0, block .700/2. `noise_60mm` — slot .292/2, hold .700/0, placed .375/4, block .450/1. 위치 belief가 나빠지면 `unknown`이 늘고 확신 오류도 생긴다. 이 판단들은 belief 품질에 종속되며 단독으로 위치 오차를 보정하지 않는다.

`placed_zone_B`(518 쌍)는 dev 4뷰 모두 `yes`였다.

### dev에서 찾아 고친 것 (모두 dev 근거)

1. **어안 테두리 오탐:** raw 어안 매핑의 검은 쐐기와 자기 집게가 하나의 8,600 px 성분을 만들고 그 최하단이 0.42 m로 투영돼 **모든 빈 차선이 막힘으로 보고됐다**(clear 8/8 오답). 유효 영역(카메라 보정에서 계산) 밖과 프레임 테두리에 닿는 성분을 후보에서 제외했다.
2. **거리 의존 바닥색:** 근거리 한 점 기준으로는 빈 프레임의 28%가 비바닥으로 잡혔다. 바닥을 **이미지 행 밴드별 중앙값**으로 모델링하고, 접촉점에서 0.06 m 높이가 투영될 픽셀 높이의 0.55배 이상 솟은 성분만 장애물로 본다(평평한 칠·그림자 제외).
3. **빈 칠을 물건으로 오인:** `BARE_LIKE_DIST`(26)와 옛 대비 하한(20)이 겹쳐 빈 칠의 조명 기울기가 양쪽 조건을 동시에 통과했다. 대비 하한을 38(최대 55)로 올리고, 판정 순서를 검출 → 검출+대비 → **비어 있음** → 대비로 바꿨다. 또 기준 고리가 그 구역의 칠 hue가 아니면 `unknown`으로 남긴다.
4. **가림을 빈 슬롯으로 오인:** 동료 로봇이 슬롯을 덮으면 원 내부가 균일해 `SLOT_SURFACE_PROVEN_BARE`가 나왔다. 기준 고리가 그 이미지 행의 바닥 모델과 34 이내로 일치하지 않으면 `SLOT_SURROUND_NOT_FLOOR`로 `unknown`이다(4뷰 중 3뷰 교정, 1뷰 잔존 → G1 허용).
5. **CARRY 자세가 `can`을 담지 못한다:** 합의된 CARRY 자세에서 들린 `can`은 윗면 테두리 3,017 px만 보이고 **ROI 안에는 0 px**이다(상자·tile은 ROI를 채운다). 따라서 `holding_item`은 CARRY가 실제로 담는 종류(상자 4색·tile)에 대해서만 부재를 주장하고, `can`에는 `KIND_NOT_FRAMED_BY_CARRY_POSTURE`로 `unknown`을 돌린다. **연구 설계에 필요한 결과다: `can`을 들었는지 확인하려면 CARRY 외의 관측 자세가 필요하다.**
6. **지도 벽 근접 후보:** 벽면 접촉이 여유 밖으로 잡히면 미등록 장애물로 보고됐다. 지도 장애물 0.18 m 안의 후보만 있으면 `OBSTRUCTION_NEAR_MAPPED_WALL`로 `unknown`이다.

## 원본 위치와 해시

| 항목 | 위치 |
|---|---|
| dev 프레임·라벨 | `outputs/2026-09-26-zone-own-perception/dev-frames-v3` (8.9 MB, 로컬 보관; Git 제외) |
| dev 채점 | `outputs/2026-09-26-zone-own-perception/dev-score-6`, 이 폴더의 `dev-summary.json`이 같은 내용 |
| 장면 해시 (변형·시드 / ZoneScene / 추가물 포함 / 주문서) | door 41 `346a3ecc` / `7587eb0d` / `d6d2c525`; corridor 42 `18568f04` / `2bc886f8` / `c9e97b94`; two_doors 43 `2df01921` / `09fee3a8` / `fc5472e5`; door 44 `a1af9d3f` / `bfda1856` / `6dda8b98` |

로컬 보관은 원격 백업이 아니다. `outputs/`는 Git에 포함되지 않는다.

## test 결과

test 분할은 이 문서의 사전 등록을 커밋한 뒤 한 번만 렌더·채점한다. 결과는 아래 절에 추가한다.

<!-- TEST-RESULTS -->

## 한계

- 오프라인 인식 평가다. 실제 임무 성공, 실시간 실행, 통신 조건 비교는 이 기록의 범위가 아니다.
- `hold_*` 뷰는 물리 파지가 아니라 자세만 맞춘 fixture다. 파지 성공률로 읽지 않는다.
- 팀 화물(`long_beam`, `heavy_crate`, `tri_frame`)의 부분 가림은 확장 목표이며 이번 코호트에 없다. 색·치수 항목만 표에 등록되어 있다.
- `peer_in_lane`은 게이트 밖이다. 동료/물체 구분은 미구현이다.
- 추적기는 자세를 고정한 반복 관측의 안정성만 측정한다. 재관측 pan의 이득은 단위 테스트로만 확인했고 물리 실행에서 측정하지 않았다.
- 분할당 시드 4개·판단별 사례 4뷰 규모다. 비율은 사례 단위로 읽고 다른 실험 수치와 합산하지 않는다.
