# 2026-09-26 평가 전용 TOP 카메라: 복도 지도 차로·대피 bay 가림 해소 (#218)

- 작업: Kiro, 브랜치 `kiro/zone-eval-topcam`(PR #208 `kiro/zone-map-v3` 위에 쌓음), PR #232(draft). Refs #218.
- 근거 결정: #218 코디네이터 결정(사용자 위임) "평가 전용 TOP 카메라를 조정해 복도 지도 차로 전체를 보이게 한다. 새 버전으로 기록하고 기존 기록은 그대로 둔다."
- 범위: 평가·영상 전용 TOP만 다룬다. 로봇 입력(자기 손목 어안 RGB, 정적 지도, 주문서, 자기 명령 이력, 대화 채널), 로봇 카메라·FOV·외관, 벽, 지도 파일, 장면 XML은 바꾸지 않았다. 물리 실행·LLM·외부 API 호출은 없다.

## 결론
- 새 프로필 **`zone_eval_top_v2`**(`sim/zone_eval_top.py`)는 복도 base 지도 계열(`zone_wide_corridor`, `_tags_v1`, `_tags_v3`)에서 `cctv_top_north_east` 한 대만 옮긴다.
  - 위치: (3.85, 0.30, 2.50) → **(3.40, 0.80, 3.00)**.
  - 그대로 둔 것: 수직 하향 자세, fovy 55°, 다른 TOP 3대. 문 지도는 v1과 같다.
- 환경 v3 ST5 측정 코드를 그대로 다시 돌린 결과, 벽 0.40 m 복도 지도에서 가림이 없어졌다.
  - 차로: **0.869 → 1.000**
  - 대피 bay: **0.818 → 1.000**
  - 두 바닥 높이(z 0.003/0.032) 모두 같은 결과다.
  - 나머지 영역은 줄지 않았다. `floor_all`은 0.9905 → 0.9963이다.
- 기존 카메라는 **`zone_eval_top_v1`**(작성된 카메라 그대로)로 남겼다. v1로 다시 잰 값은 env v3 기록 표와 **완전히 같다**. 기존 실험 기록은 고치지 않았다.

## 무엇을 바꿨나
| 파일 | 내용 |
|---|---|
| `sim/zone_eval_top.py` | 프로필 등록부(`PROFILES`, `:44`), 평가용 카메라 목록(`eval_top_cameras`, `:114`), 평가 도구용 지도 사본(`eval_static_map`, `:142`), 월드 적용·재확인(`apply_to_world` `:184`, `verify_world` `:202`) |
| `tests/test_zone_eval_top.py` | 16개 테스트. MuJoCo가 필요한 4개는 CI에서 건너뛴다 |
| `scripts/run_ci_tests.py` | `TEST_PATTERNS`에 위 테스트 추가 |
| `experiments/2026-09-26-zone-eval-topcam/` | 사전 등록, 수정안 A1, 실행 스크립트, 결과, 그림 |

`sim/zone_eval_top.py`의 적용 방식:
- 지도·XML·`scene.setup`을 건드리지 않는다. `scene.setup` 뒤 모델의 `cam_pos/cam_quat/cam_fovy`만 덮어쓴다.
- TOP 픽셀을 바닥으로 투영하는 평가 도구(`harness/zone_perception.pixel_to_floor`)에는 `eval_static_map()` 사본을 준다.
- 기본값이 없어 프로필 id를 반드시 지정해야 한다.
- 복도 base 지도 해시(`262b8d7c…`)가 다르면 v2 적용을 거부한다.
- `scene.setup`이 작성된 카메라로 되돌리면 `verify_world`가 오류를 낸다.

## 측정 (재사용)
- 커버리지는 env v3의 `experiments/2026-09-26-zone-env-v3/static_checks.py`의 `st5()`를 **바이트 그대로** import해 쟀다(sha256 `b1dea143…`).
  - `make_world`만 감싸 `scene.setup` 뒤에 프로필을 적용했다.
  - 측정 격자: 0.02 m 바닥 격자, 640×480 투영, `mj_multiRay`(geom group 0–4).
  - 판정: 격자점이 TOP 한 대 이상에 보이는 비율.
- 후보 탐색(`eval_topcam.py search`)도 같은 helper(`ray_hits`, `region_masks`, 격자, 투영식)를 썼다.
  - 작성된 위치에서 탐색 모델 값은 env v3 기록과 같다(0.86896 / 0.81778).
  - 선택한 위치에서 탐색 값은 확인 실행(`st5()`) 값과 같다.

## 선택 절차
1. [`prereg.json`](prereg.json)을 탐색 전에 커밋했다(`733197b6`).
   - 바꿀 수 있는 범위: `cctv_top_north_east`의 x 2.85–3.85, y 0.30–1.20(0.05 m 간격), z 2.5–4.0(0.1 m 간격). 모두 6,384개 후보다.
   - 게이트:
     - G1: 차로·bay ≥ 0.97
     - G2: v3 벽의 다른 영역이 줄지 않음
     - G3: 0.10 m 벽 지도가 줄지 않음
     - G4: 960×720에서 상자 윗면 예상 넓이 ≥ 60 px(검출기 하한 50 px × 1.2)
   - 선택 순서: 가장 낮은 z → 수평 이동 최소 → `floor_all` 최대.
2. 탐색 결과 103개 후보가 G1–G4를 통과했다. 규칙대로 고르면 (3.40, 0.70, 2.90)이다(차로 0.974, bay 1.0).
   - 이 후보에서는 복도 벽 북쪽 면을 따라 **3.6 cm 띠가 여전히 가려진다**.
3. **수정안 A1**([`prereg_amendments.json`](prereg_amendments.json), `90d6e9a3`)을 **탐색 결과를 본 뒤에** 썼다.
   - 과제의 두 번째 조건 "0.40 m 벽이 차로를 가리지 않는다"를 G1b(차로·bay 격자점 가림 0, 두 높이)로 추가했다.
   - 나머지 게이트와 순서는 그대로 두었다. 새 탐색은 하지 않고 저장된 탐색 결과에 적용했다(`select`).
   - 53개가 통과했고, 1순위가 **(3.40, 0.80, 3.00)**이다.

## 결과 (확인 실행: `st5()` 원본, 소스 `d9eb5895`)
| 지도·벽 | 영역 | v1 z0.003 / z0.032 | **v2** z0.003 / z0.032 |
|---|---|---|---|
| corridor, 0.40 m (`_tags_v3`) | 차로 `corridor_1` | 0.86896 / 0.86705 | **1.0 / 1.0** |
| | 대피 bay `bay_1` | 0.81778 / 0.81778 | **1.0 / 1.0** |
| | `floor_all` | 0.9905 / 0.99083 | 0.9963 / 0.99676 |
| | 안쪽 벽 0.30 m 이내 | 0.93406 / 0.93629 | 0.9743 / 0.9775 |
| corridor, 0.10 m (`_tags_v1`) | 차로 / bay | 0.97274 / 0.96296 (z0.003) | 1.0 / 1.0 |
| | `floor_all` | 0.99806 | 0.99961 |
| door, two_doors (두 벽 높이) | 모든 영역 | 1.0 | 1.0 (v1과 동일) |

- v1 재측정은 env v3 `static_results.json`의 ST5 표와 모든 값이 같다(`v1_reproduces_env_v3_ST5_tables: true`).
- v2에서 남은 가림: 0.40 m 복도 지도 z0.003에서 265점이다. 모두 차로·bay 밖이다.
  - 위치: bay 서쪽 벽 바깥(서쪽)과 bay 남쪽 벽 아래(남쪽) 바닥, x 2.69–4.01, y 0.27–0.93.
  - 가리는 벽: `wall_bay_west` 148점, `wall_bay_south` 97점, `wall_corridor_2` 11점, `wall_bay_east` 9점.
- 격자로는 보이지 않는 해석적 잔여:
  - 복도 벽 북쪽 면 아래 1.9 cm 띠. 격자 간격 2 cm보다 좁다.
  - 계산식: 벽 높이 h, 카메라 높이 H, 벽면까지 수평 거리 d일 때 `h·d/(H−h)`.
  - v1은 11.8 cm였다.
- 해상도: 옮긴 카메라의 바닥 해상도는 약 230 px/m다(v1 2.5 m는 약 280 px/m, 960×720 기준). 상자 윗면 예상 넓이는 73.8 px로 G4를 통과한다.

그림:
- [복도 지도 커버리지 v1(위)/v2(아래)](media/coverage_corridor_v1_top_v2_bottom.png)
  - 색: 빨강 = TOP 0대, 노랑 = 1대, 초록 = 2대 이상.
  - 각 행 왼쪽은 0.10 m 벽, 오른쪽은 0.40 m 벽이다.

## 렌더 스틸 (960×720, 동시 렌더 1개)
두 배치 모두 로봇 1대를 차로에 두었다. 상자 배치는 아래와 같다.
- `hidden_bands`: v1이 못 보는 띠 네 곳에 상자를 두었다.
  - 복도 벽 북쪽 두 곳 (3.65, 0.965), (2.50, 0.965)
  - bay 동쪽 (3.335, 0.50)
  - bay 서쪽 벽 바깥 (2.70, 0.60)
- `open_floor`: 두 카메라 위치 모두 보는 바닥 네 곳에 상자를 두었다.

그림: [가림 띠 NE 화면 v1(왼쪽)/v2(오른쪽)](media/still_hidden_bands_ne_v1_left_v2_right.jpg), [열린 바닥 NE 화면 v1/v2](media/still_open_floor_ne_v1_left_v2_right.jpg)

기존 TOP 색 검출기(`zone_perception_v1`, `top_zone_v2`)로 확인한 결과(상자 위치 5 cm 이내 일치):

| 배치 | v1 | v2 |
|---|---|---|
| hidden_bands | 0/4 (두 검출기 모두) | `top_zone_v2` 2/4(녹색 차로, 빨강 bay), `zone_perception_v1` 1/4 |
| open_floor | 4/4 (두 검출기 모두) | 4/4 (두 검출기 모두) |

- 열린 바닥에서는 두 위치의 차이가 없다. 3.0 m 높이 때문에 검출이 떨어지지 않았다.
- v2가 놓친 상자는 둘이다.
  - bay 서쪽 벽 바깥 상자: v2에도 남은 가림 영역에 있다. 차로·bay 밖이다.
  - 차로의 청록 상자: 화면에는 보인다. 그러나 벽의 조명 그림자 속에서 채도가 낮다(S 중앙값 98, 열린 바닥은 139). 기존 검출기의 채도 하한(90/110)에서 잘린다. 카메라 가림이 아니라 검출기·조명 문제다. 이 PR에서는 고치지 않았다(아래 열린 문제).
- 첫 스틸 실행(`stills/`, 소스 `d9eb5895`)은 배치가 하나뿐이었다. 또 검출 종류 목록에 red가 두 번 들어가 같은 상자가 두 번 보고됐다. 이 실행은 보존하고 `stills2/`로 대체했다.

## 로봇 입력 경계 검증
- 정적 검사(`RobotInputBoundaryTests`):
  - `harness/`·`scripts/`·`sim/`·`configs/`·`maps/`의 어떤 파일도 `sim.zone_eval_top`, `zone_eval_top_v*`, `eval_static_map`, `eval_top_cameras`를 참조하지 않는다. 모듈 자신은 예외다.
  - 모듈 안에 `robot_cam`·`render_rgb`·`nav_cam`·`qpos`·`xpos`가 없다.
- MuJoCo 검사:
  - 프로필 적용 전후로 바뀌는 카메라는 `cctv_top_north_east` 하나다. 로봇 카메라와 `cctv_warehouse`는 그대로다.
  - 같은 명령을 준 물리 qpos 궤적이 비트 단위로 같다.
  - 세 로봇의 wrist RGB가 바이트 단위로 같다.
  - 옮긴 TOP 화면은 달라진다.
- 연구 경로(PR #194/#229)의 `zone_study_contract` 금지 키에는 이미 `top_camera(s)`·`cctv*`가 들어 있다(해당 브랜치에서 확인).

## 테스트
- `tests/test_zone_eval_top.py`: 16 passed(로컬 MuJoCo). mujoco를 막은 환경에서는 12 passed, 4 skipped(CI 조건).
  - 다루는 경계: 잘못된 id(None, '', 0, 1.0, NaN, True, list, dict), 빈·잘못된 지도, NaN/Inf·문자열·bool 좌표, fov 0/180/음수, 기운 자세, 중복·누락 카메라, 좌표 0 보존.
- 함께 돌린 기존 테스트(137 passed): `test_zone_env_v3`, `test_zone_landmarks_sim`, `test_zone_dispatch`, `test_zone_hard_routes`, `test_zone_rgb_outcome`, `test_ci_host_lock`, `test_simulation_workflow_manager`.
- `run_ci_tests.py`의 `TEST_PATTERNS`가 새 테스트를 수집하는지 확인했다(201개 모듈 중 포함). `tests/test_zone_env_v3.py`는 PR #208에서도 수집 목록에 없다. 이 PR에서는 고치지 않았다.

## 기각한 대안
| 대안 | 기각 이유 |
|---|---|
| 다섯 번째 복도 전용 카메라 추가 | 장면 XML 변경이 필요하다(`sim/zone_arena.py`는 다른 에이전트 소유, 장면 해시도 바뀜). 과제도 기존 카메라 조정을 요구했다 |
| 카메라 기울이기 | `harness/zone_perception.pixel_to_floor`가 수직 하향만 지원한다. 게다가 가림은 카메라 위치로 정해지고 자세와 무관하다 |
| 2.5 m에서 fovy만 넓히기 | 벽 그림자 길이 `h·d/(H−h)`가 높이를 올릴 때보다 길다. 같은 렌즈 유지를 사전 등록에서 고정했다 |
| NE를 차로 위로만 옮기기(z 2.5) | 발자국 남쪽 끝이 올라가 구역 C 북쪽·동쪽 바닥이 빈다(G2 실패). 탐색에서 z 2.5–2.8 후보는 0개가 통과했다 |
| 벽·지도 변경 | 범위 밖이다(환경 v3 결정, 지도 소유자 별도) |

외부 알고리즘은 쓰지 않았다. 필요한 것은 격자 가시성 계산뿐이고, 이는 저장소의 env v3 ST5 코드와 MuJoCo `mj_multiRay`로 충분했다.

## 참고 자료
- 논문: 없음. 새 알고리즘이 없는 기하 조정이라 인용할 논문을 찾지 않았다.
- OSS·라이브러리:
  - MuJoCo 3.12.0(Apache-2.0): `mj_multiRay`(가시성), `mj_forward`, `mujoco.Renderer`(스틸). 소스 수정 없이 API만 썼다.
  - OpenCV 5.0.0(Apache-2.0): 기존 TOP 색 검출기 안에서 쓰인다.
  - NumPy 2.5.2(BSD-3-Clause), Pillow 12.3.0(MIT-CMU): 격자·그림.
- 저장소 내부:
  - `experiments/2026-09-26-zone-env-v3/static_checks.py`: `st5`, `ray_hits`, `region_masks`, `make_world`, `clear_scene`, `set_free`. PR #208, 그대로 import.
  - `sim/zone_arena.py`: `LAYOUTS`, TOP 작성 방식, `CORRIDOR`.
  - `sim/zone_scene.py`: `ZoneScene.setup`의 TOP 배치 방식.
  - `sim/zone_landmarks.py`: `TaggedZoneScene`, `tagged_map`.
  - `sim/research_dispatch_arena.py`: `FIXED_TOP`, `digest`.
  - `harness/zone_perception.py`: `pixel_to_floor`, `detect_boxes`.
  - `harness/zone_color_boxes.py`: `detect_top`, `TOP_ZONE_AREA_PX`.
  - 참고한 기록: env v3 `observe_top.py`(평가 전용 TOP 기록 방식), PR #194·#229의 `harness/zone_study_contract.py`(TOP 금지 키).
- 문서: `AGENTS.md`, `docs/current_status.md`, `CONTRIBUTING.md`, `experiments/2026-09-26-zone-env-v3/README.md`, #218 코멘트.

## 원본·해시
원본은 기본 체크아웃의 `outputs/zone-eval-topcam-20260926/`(6.0 MB)에 있다. **로컬 전용이며 원격 백업이 아니다.** 경로·sha256·소스 SHA·부하 평균은 [`results.json`](results.json)의 `raw`에 있다.

| 산출물 | 소스 | sha256 앞 12자리 |
|---|---|---|
| `search/search.json` (6,384 후보) | `733197b6` | `df0ec3433625` |
| `select/selection.json` | `90d6e9a3` | `828f892e2e57` |
| `coverage_zone_eval_top_v1/coverage.json` | `d9eb5895` | `c15c19a74fc7` |
| `coverage_zone_eval_top_v2/coverage.json` | `d9eb5895` | `b530fd4ad0bb` |
| `stills/stills.json` (대체됨, 보존) | `d9eb5895` | `8a07454eb0cb` |
| `stills2/stills.json` | `11fe9cff` | `aa0a33f79ecf` |

프로필 매개변수 해시(`profile_record`):
- `zone_eval_top_v1`: `070d159a…`
- `zone_eval_top_v2`: `03ad04df…`

## 실행 환경
- Mac `.venv-sim-worker-mac`: Python 3.12, MuJoCo 3.12.0, NumPy 2.5.2.
- 실행 조건:
  - 스레드 변수 4개 = 1.
  - 모든 실행은 `scripts/ugrp_session.py run`으로 돌렸다.
  - 동시 실행은 최대 2개(커버리지 v1·v2)였다.
- 부하 평균(1분)은 실행 시작 때 3.0–4.3이었다. 실행별 값은 `results.json`과 `launch_load.txt`에 있다.
- 실행 전 SIM 프로세스 수:
  - 규칙의 grep 식은 7을 셌다. 에이전트 `kiro-cli` 6개와 TensorBoard 서버 1개가 명령줄 문자열로 걸린 것이다.
  - 이들을 뺀 실제 SIM 프로세스는 매번 0이었다.
- 물리·학습 잠금은 필요 없었다(`mj_forward`·광선·렌더만 사용, wall 시간 비교 없음).
- TensorBoard: 에피소드·학습·시간 계열이 없는 정적 기하 검사라 스냅샷을 만들지 않았다. env v3의 ST1–ST5도 TensorBoard에 넣지 않았다.

## 한계와 열린 문제
- **채택은 사용하는 쪽 몫이다.** 현재 평가 기록기는 프로필을 적용하지 않는다(env v3 `observe_top.py`, 통합 러너 PR #229 등). 복도 지도에서 v2를 쓰려면 다음이 필요하다(#223 담당 쪽).
  - `scene.setup` 뒤에 `apply_to_world(world, static, 'zone_eval_top_v2')`를 호출한다.
  - 투영에 쓰는 지도는 `eval_static_map()` 사본이어야 한다.
  - 기록에 `profile_record`를 남긴다.
- 벽 조명 그림자 속 청록 상자의 색 검출 실패(채도 하한)는 카메라와 무관한 검출기·조명 문제로 남는다(#220 인식 쪽).
- 남은 가림 265점은 bay 벽 바깥 바닥이다. 과제 경로(차로·bay·구역·픽업)에는 없다.
- 3.0 m 설치 높이는 SIM 평가용이다. 실물 설치 가능성은 확인하지 않았다.
- 선택 규칙의 G1b는 결과를 본 뒤에 추가했다(위에 공개). 사전 등록 규칙만 따른 후보도 `selection.json`에 남겼다.
- `experiments/README.md` 인덱스는 다른 브랜치와의 충돌을 피하려고 고치지 않았다(PR #208과 같은 방식).
