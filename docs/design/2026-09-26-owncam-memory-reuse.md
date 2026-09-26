# 자기 카메라 관측 기억("한 번 보고 기억하기") 재사용 조사와 설계

- 작성: Kiro, 2026-09-26, 브랜치 `kiro/zone-owncam-memory`(PR #201 `claude/zone-m1-owncam` 위에 쌓음). `kiro/`는 Kiro의 작업이다.
- 사용자 요청(2026-09-26): "뭔가 주변을 봤으면 그걸 기억해두면 되지, 꼭 계속계속 둘러봐야 하나?", "확인하고 기억 같은 것들 바로 직접 만들지 말고, 이미 만들어진 것들을 활용해주라. 그리고 PR할 때 뭐 참고했는지 기록하고!"
- 이 문서는 **코드 작성 전에** 쓴 재사용 조사다. 구현·실험 결과는 `experiments/2026-09-26-zone-owncam-memory/`에 둔다.
- 같은 날 `kiro/markerless-research`(마커 없는 위치 추정 조사)는 아직 문헌 문서가 없고 탐침 코드만 있다. 이 문서는 기억·능동 관측만 다루고 마커 없는 위치 추정은 다루지 않는다.
- **결과(test 161–166, 1회, interim, tag provider):** m1_success OFF 4/6, memory_v2 3/6 → 사전 등록 주장 규칙 불충족. 둘 다 성공한 3 seed에서는 SIM 41–44% 짧았다(둘러보기 시간 181.7 → 42.8 s). 자세한 표·실패 분석은 [실험 README](../../experiments/2026-09-26-zone-owncam-memory/README.md).
- **2026-09-26 개정(A1–A3, 계정 2에서 이어서 작업).** 사용자: "표식은 없애기로 했잖아. 그걸 기억하면 안 되지 않을까?" 기억을 표식(AprilTag)과 무관하게 바꿨다. 0절이 현재 설계다. 1–3절은 첫 판(memory_v1, `ad78ef2`)의 기록으로 남기고, 태그에 기대던 부분은 **[대체됨 A1]**로 표시했다.

## 0. 개정 설계(memory_v2): 표식 비의존 기억

### 0.1 무엇이 바뀌었나

| 항목 | memory_v1 (`ad78ef2`, dev-a1만) | memory_v2 (A1–A3) |
|---|---|---|
| 기억하는 위치 근거 | 태그 ID별 고정 기록 | **지도 표식 관측**: 표식 ID·종류(정적 지도 기하에서 만든 것), 방위·고도·거리, 그때의 자기 추정과 σ, 프레임 ID, 제공자 |
| 어디를 볼지 | 지도 태그 목록의 예상 시야 | **정적 지도 기하**(문기둥, 벽 모서리, 문 틈, 벽면)의 예상 시야 × 제공자가 그 표식을 볼 확률 |
| 태그의 위치 | 기억의 핵심 입력 | 교체 가능한 **임시 제공자** 하나(`harness/owncam_landmark_tags.py`). 결과는 모두 "interim, tag provider" |
| 태그가 없으면 | 계획 불가 | 상자·바닥 기억은 그대로, 둘러보기는 전체로 대체. 비전 검출기(`kiro/zone-vision-loc`, PR #227)를 붙이면 같은 계획기가 문기둥·모서리로 pan을 고른다 |
| 상자 위치 | `detect_own` 기본 카메라 값 | PF 교정의 카메라 고도 편향으로 다시 투영(A2). 트랙 σ 하한 = max(0.02 m, 관측 때 자기 σ) |
| 자세 고정의 신선도 | σ만 봄 | 정지한 둘러보기 자세 고정만 신선하다고 본다(이동 ≤ 0.5 m, 60 s). 도착 확인 생략·짧은 보기 조기 종료에 필요(A3) |

기억 핵심(`harness/owncam_memory.py`, `owncam_landmarks.py`, `owncam_drive_mem.py`)은 태그 목록·`wall_tags`를 읽지 않는다. 테스트(`test_memory_core_is_tag_free`, `test_tags_are_never_read_and_never_required`, `test_the_memory_runs_on_a_map_without_tags`)가 이를 검사한다. 자기 위치를 내는 PF(동결 M1, `owncam_localizer.py`)는 아직 태그를 쓴다. 이것은 기억 밖이며 PR #227 비전 위치 추정이 대체할 부분이다.

### 0.2 표식 목록(catalogue) — `harness/owncam_landmarks.py`

- 입력: 지도의 벽 사각형(`obstacles` kind=wall), 문 통로(`passages` kind=door), 실제 문기둥(`landmarks.door_posts`, v2 지도). `landmarks.tags`는 읽지 않는다. 목록 해시는 태그가 있든 없든 같다(테스트).
- 모서리: 벽 면들이 이루는 선의 모든 교점에서 네 사분면의 빈 공간 여부를 본다. 빈 사분면 1개 = `wall_corner`(방 모서리), 3개 = 볼록한 벽 끝이다. 벽 끝이 문 통로 0.12 m 안에 있으면 `door_post`, 아니면 `wall_end`. 문마다 `door_gap` 1개, 벽의 빈 면마다 `wall_face` 1개(계획용 0.25 m 표본).
- 결과(`zone_wide_door_tags_v1`): 모서리 8, 문기둥 모서리 4(`door_1:post_lo:-x` 등), 문 틈 1, 벽면 10. v3(0.40 m 벽, `kiro/zone-map-v3`)와 태그 없는 `zone_wide_door_walls_v3_notags`(PR #227)에서도 같은 ID가 나온다(벽 높이만 다름).
- 기각한 대안: 지도를 격자로 그려 `cv2.findContours` + `cv2.approxPolyDP`로 꼭짓점을 찾는 방법. 1 cm 반올림 오차가 생기고 면의 정체(어느 벽의 어느 면)가 없어진다.

### 0.3 표식 관측과 제공자 인터페이스

- 관측 기록(`LandmarkObservation`): `t, frame_id, landmark_id, landmark_type, provider, interim, azimuth_rad, elevation_rad, range_m, pose_xyyaw, std_xy_m, std_yaw_rad, posture, settled, feature_id, supports`. PR #210 설계 4(b)의 관측 항목(`obs_id, t_sim, observer_pose(σ 포함), entity, evidence, source`)과 같은 뜻의 필드를 둔다. 개체는 좌표가 아니라 지도 표식 ID로 붙인다.
- 제공자(`LandmarkProvider`): `observe`(한 프레임의 관측), `support`(계획 행마다 볼 확률·예상 특징 수), `noise`(방위·고도·거리 잡음과 PF 완화 지수), `face_points_identified`(벽면 위 점이 각각 식별되는가).
  - 임시 태그 제공자: 위치 PF가 이미 검출한 태그를 다시 쓴다(두 번 검출하지 않음). 태그마다 0.30 m 안의 같은 면 표식을 `supports`로, 가장 가까운 점 표식(없으면 벽면)을 `landmark_id`로 둔다. v1 지도 태그 70개 = 모서리 16, 벽면 50, 문기둥 4에 붙는다. 검출 확률표·잡음은 memory_v1 값 그대로다.
  - 기하 제공자(태그 없음): `detector(image, servo, catalogue, view)` 자리에 비전 검출기를 붙인다. 지금은 검출기가 없어 아무것도 보지 않고, 계획은 전체 둘러보기가 된다. 검출 확률·잡음 기본값은 자리표시값이며 이번 실험에 쓰지 않는다.
- 계획: pan마다 지도 표식의 바닥 접점(가려지면 중간·윗점)을 투영해 시야·빈 쪽·벽 가림을 본다. 제공자 확률로 가중한 방위·고도(·거리) Fisher 정보로 탐욕 선택한다. 식별되지 않는 벽면 선은 면 방향 정보를 뺀다.
- 등가 확인(`experiments/2026-09-26-zone-owncam-memory/planner_equivalence_check.json`): 무작위 자기 추정 400개에서 v2(목록 + 임시 태그 제공자)는 v1과 둘러보기 방식이 400/400, 짧은 보기의 첫 pan이 385/392 같았다. 태그 목록을 직접 쓰지 않아도 오늘 실행의 행동은 거의 같다.

### 0.4 A2·A3 근거 (dev-a1, 평가 전용 GT로만 채점)

- A2: dev-a1 ON에서 목표 트랙이 GT에서 0.123 m 떨어진 채 σ 0.029 m로 확정됐다(사전 등록 거짓 확인 기준 0.10 m 초과). `detect_own`은 기본 카메라로 광선을 푼다. PF 교정은 이미 무부하 카메라 고도 편향 −0.01868 rad를 추정해 두었다. 이 값으로 다시 투영하면 거리 편향이 사라진다(`box_bias_check.json`). 새로 맞춘 값은 없다.
- A3: 같은 실행에서 SEARCH_POSE로 달리며 태그를 계속 보았는데도 자기 오차가 0.02 → 0.16 m로 커졌고 PF σ는 0.033 m였다. 도착 확인을 σ만 보고 건너뛰어 파지가 0.15 m 어긋난 채 시작했다(파지 단계 127 s, OFF 44 s). 정지한 둘러보기 고정만 신선한 것으로 친다.

### 0.5 다른 작업과의 정렬

- `kiro/memory-literature`(PR #230, `docs/design/2026-09-26-memory-literature.md`, `3174c2f`): test 코호트 도중에 원격에 올라와 **동결 뒤에** 대조했다. 대조 결과는 0.6절이다. 동결 소스는 바꾸지 않았다.
- PR #210(`kiro/markerless-research`) 4(b): 관측 항목 필드·개체 우선·신선도·도착 시 재검증 규칙을 따른다. 들은 주장(`source: heard`)은 이 PR 범위 밖이다(단독 M1).
- PR #227(`kiro/zone-vision-loc`): 태그 없는 지도와 벽·문 분할 모델 → PF 측정. 같은 모델의 검출 결과를 기하 제공자의 `detector`로 넣으면 기억은 바뀌지 않는다.

### 0.6 문헌 조사(PR #230)와의 대조

| PR #230 권고 | memory_v2 | 상태 |
|---|---|---|
| 자기 자세는 SLAM이 아니라 주어진 지도에서의 MCL(0절 1항, [P1][P3]) | 동결 M1 PF를 그대로 읽는다. 루프 폐쇄·키프레임 없음 | 일치 |
| 물체 기억 = 물체별 KF 트랙 + 게이트 대응 + 정적 지도 개체 이름표(0절 2항) | `BoxTrack`(filterpy·PythonRobotics 적응), 표식 목록 ID로 관측을 붙임 | 일치. 트랙에 지도 개체(`location_ref`, 예: 픽업 칸)를 붙이는 것은 아직 없음 → 후속 |
| 트랙 σ ≥ 최선 관측 당시 자세 σ(4.1절, `floor_var`) | A2 그대로 | 일치 |
| 마지막 정지 둘러보기 고정을 L1 기억에 둔다(4.1절) | A3 `last_look_fix` | 일치 |
| 보였어야 할 곳에서만 미검출을 부재 증거로 센다(0절 3항, [P18][P29][P37]) | `point_in_view`(시야·거리 1.0 m·벽 가림) 안에서만 `misses`를 센다 | 일치 |
| `ABSENT_MISSES` 카운터 대신 존재 확률(persistence filter[P14], 수식만) | 카운터(3회) 그대로 | **후속 버전**. test 소스가 동결돼 이번 비교에는 넣지 않는다 |
| 들은 주장(L4)은 따로 저장, 자기 트랙과 합치지 않음 | 범위 밖(단독 M1, 통신 없음) | 후속(대화 연구 통합 때) |
| 기억 → 한국어는 결정적 템플릿 요약, 확신은 규칙 계산(4.4절) | `snapshot()`이 트랙 상태·σ·나이·지도 표식 ID를 낸다. 템플릿은 아직 없음 | 후속 |

## 1. 문제: 기준선은 왜 계속 둘러보나

M1 test(`experiments/2026-09-26-zone-m1-owncam`, 동결 `ca2fdb8`, `zone_wide_door_tags_v2`)의 성공 4회 원본(`outputs/m1-owncam-20260926/test/`)에서 로봇 자신의 입력 기록(`inputs/frames.jsonl`의 발행 PWM과 자기 추정)만으로 셌다.

| 항목 | s101 | s102 | s105 | s106 |
|---|---|---|---|---|
| SIM 전체 (s) | 530 | 445 | 353 | 421 |
| `LOOK_P20` 자세에 있던 시간 (s) | 220 | 212 | 171 | 163 |
| `LOOK_P20` 진입 횟수 | 27 | 26 | 21 | 20 |

- 둘러보기 한 번은 팔 이동 → pan 6곳(`WIDE_LOOK_PANS`, 곳마다 0.6 s 정지) → 복귀로 **약 8.2 s**다. SIM 시간의 약 40–46%가 둘러보기다.
- 운반 구간 둘러보기는 대부분 `uncertain`이다(loop v2 test 6회: 66회 중 42회). s105를 보면 운반 중 18회 가운데 17회가 **yaw σ가 3°에 닿은 순간**(σxy는 0.026–0.047 m로 한계 0.07 m 아래)에 시작했다. 짐 상태 운동 모델의 yaw 절대 잡음(0.0976 rad/s)이 태그를 못 보는 동안 yaw σ를 약 1.25°/√s씩 키우므로, 약 5.5 s 주행마다 8.2 s를 둘러본다.
- 한 번의 둘러보기로 얻는 정보는 pan 한두 곳에 몰려 있다. 같은 기록에서 `LOOK_P20`의 pan별 태그 검출 프레임 비율은 2030: 99%, 970: 92%, 1770: 80%, 1230: 76%, 1500: 70%였고, 운반 중 둘러보기의 σxy는 둘러보기 전후가 거의 같은 경우가 많았다.
- 탐색은 관측점마다 이전 검출을 지운다(`M1OwnCamDelivery._search_leg`의 `self.cyan` 초기화). 앞 관측점에서 이미 본 바닥도 다시 훑는다.
- 운반 자세(`CARRY_POSTURE`)에서도 가까운 벽 태그(0.05 m, 한 변 37–47 px)는 상자 위 띠(영상 행 ≤ 170 px)에 보인다(s102·s105 운반 프레임 942장에서 643검출). 이때 보이는 태그와 안 보이는 태그를 지도로 미리 알 수 있다.

즉 로봇은 지도·자기 추정·방금 본 것을 알면서도 **매번 같은 전체 둘러보기**를 한다. 기억이 있으면 (1) 어디를 보면 태그가 보일지 알고 그곳만 짧게 보고, (2) 이미 확인한 것은 다시 확인하지 않으며, (3) 보여야 할 것이 안 보일 때만 전체를 다시 볼 수 있다.

## 2. 재사용 후보

판정: **채택**(코드 사용) / **적응**(허가된 코드를 옮겨 고침) / **설계 참고**(코드 미사용) / **기각**.

### 2.1 저장소 안 (다른 브랜치·PR 포함)

| 후보 | 제공하는 것 | 이 과제에 맞는가 | 라이선스 | 판정 |
|---|---|---|---|---|
| `harness/owncam_localizer.py` (PR #177/#197/#201; PythonRobotics PF 적응) | 발행 명령 + 태그 PnP 파티클 필터. 자세 평균·공분산·`since_tag_s` | 자세 기억 그 자체다. 태그 고정은 이미 사후분포에 누적된다 | 저장소 코드(MIT 부분 포함) | **채택**(수정 없음). 기억은 `PoseReport`를 읽기만 한다. [A1 주] PF의 태그 사용은 기억 밖이며 PR #227이 대체한다. 교정의 카메라 고도 편향은 A2에서 상자 재투영에 다시 쓴다 |
| `harness/owncam_pose_source.py` (`PoseReport`, `PoseLimits`, `check_limits`) | 추정 시각·σ·마지막 유효 관측, 한계 검사 | 신선도·재확인 한계에 그대로 쓴다 | 저장소 | **채택** |
| `harness/wall_tags.py` (`TagDetector`, `predicted_tag_in_camera`, `camera_in_base`) | 태그 검출, 지도 태그의 카메라 좌표 예측, 명령 PWM FK | "보여야 할 태그" 예측의 핵심 | 저장소 | ~~채택~~ **[대체됨 A1]** 임시 태그 제공자 안에서만 쓴다(`tag_world_frame`). 기억 핵심은 `harness.visual_arm.camera_extrinsics`(같은 FK)를 직접 쓴다 |
| `harness/zone_color_boxes.detect_own` | 자기 RGB 상자 검출(`near` / `far_coarse`), 차체 좌표 | 상자 트랙의 측정값 | 저장소 | **채택** |
| `harness/markerless_box._pixel_ground_point` | 어안 픽셀 → 바닥 교점 | 카메라 바닥 시야 다각형(빈 바닥 관측) | 저장소 | **채택** |
| `harness/map_goto.plan_path` (keep-out 사각형) | A* | 기억한 상자를 σ만큼 부풀린 keep-out으로 넘긴다 | 저장소 | **채택** |
| `harness/owncam_drive.py` / `owncam_drive_v2.py` (loop 드라이버, PR #178/#197) | 멈춰 둘러보기 정책 hook: `_needs_look`, `_uncertain`, `_travel_look_m`, `_fix_std_xy_m`, `_should_refix`, `_start_look` | 둘러보기 정책만 바꾸면 된다. 주행·계획·자세는 그대로 | 저장소 | **채택**(상속, 정책 hook만 재정의) |
| `harness/m1_owncam_delivery.py` (PR #201, 동결) | M1 사슬(탐색→파지→운반→배치→다시 보기), gate 둘러보기 | 기억 없는 기준선이자 확장 대상 | 저장소 | **채택**(상속, 파일 불변) |
| `harness/coela_modules.py` `Memory` (CoELA식) | 화물별 마지막 관측 시각·출처 ID, 동료 의도 TTL(12 s) 만료, snapshot 허용 목록 | 입력이 시뮬레이터 `mixed_observe` 결과이고 기하·불확실도가 없다 | 저장소 | **설계 참고**: 항목별 `last_seen_sim_time`·출처 관측 ID·만료 상태를 트랙 필드로 따른다 |
| `harness/zone_own_outcome.py` `JudgmentTracker` (PR #193, `kiro/zone-own-perception`) | 고정 주기·연속 2회 일치·신뢰도 0.65 이상에서만 확정, `unknown`은 `no`로 바뀌지 않음 | 확인 규칙은 맞다. 다만 이 스택(#201)에 #193이 없고, 가져오려면 `zone_own_perception.py`(947줄)까지 병합해야 한다 | 저장소 | **설계 참고**: 트랙 확정은 "서로 일치하는 near 관측 2회 이상", 안 보임은 "없음"이 아니라 "미확인"이라는 규칙을 그대로 따른다 |
| `judge_route_blockage` (PR #193) | 자기 RGB 전방 막힘 yes/no/unknown | M1에는 미등록 장애물이 없고, 이 스택에 #193이 없다 | 저장소 | **적응 대기**: 격자에 `observe_lane(polygon, answer, confidence)` 입력만 만든다. #193 병합 뒤 연결 |
| `kiro/zone-own-executor` (PR #206) `status().blocked_ahead` | 5 s 뒤 `unknown`으로 되돌림 | 같은 신선도 개념. 이 스택에 없음 | 저장소 | **설계 참고** |
| `harness/multi_object_tracking.py` `CargoTracker` | TOP 영상 순열 대응, 모호하면 정지(latch) | TOP은 평가 전용이라 입력으로 못 쓴다 | 저장소 | **기각**(코드). 원칙만 따른다: 한 검출이 두 트랙에 걸리면 어느 쪽도 갱신하지 않는다 |
| `harness/camera_landmark_tracker.py`, `camera_visual_observer.py`, `dispatch_beam_tracker.py` | 픽셀 공간 LLM 관측·TOP 빔 추적 | 지도 좌표 기억이 아니고 TOP/LLM 경로다 | 저장소 | **기각** |
| `harness/zone_perception.py`, `zone_cargo_perception_v2.py`, `known_map_navigation.py`, `heading_map_navigation.py`, `dispatch_navigation_map.py`, `real_map.py` | TOP 기반 인식, 이전 지도 주행 | TOP·실물 경로 | 저장소 | **기각** |

### 2.1b A1 개정 때 추가로 본 후보 (2026-09-26)

| 후보 | 제공하는 것 | 맞는가 | 라이선스 | 판정 |
|---|---|---|---|---|
| PR #210 `docs/design/2026-09-26-markerless-localization-and-memory.md` 4(b) | 관측 항목 형식(관측 ID·SIM 시각·관찰자 자세와 σ·개체·근거·출처), 개체 우선, 신선도, 도착 시 재검증 | 기억 기록 형식에 맞다. 코드는 없다 | 저장소 | **설계 채택**: `LandmarkObservation` 필드, 지도 표식 ID로 개체를 붙임 |
| PR #227 `kiro/zone-vision-loc` (`zone_wide_door_walls_v3_notags.json`, 벽·문 분할 → PF) | 태그 없는 지도, 앞으로의 비전 검출 | 검출기는 아직 없음 | 저장소 | **인터페이스 정렬**: 기하 제공자의 `detector` 자리. 표식 목록이 이 지도에서 같은 ID를 낸다 |
| `kiro/zone-map-v3` (PR #208) `zone_wide_door_tags_v3.json` | 0.40 m 벽, 희소 태그 | 표식 목록 동작 확인 | 저장소 | **확인만**(v3 비교 여부는 사전 등록 `map_v3_rule`대로 정한다) |
| `experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json` `elevation_bias_rad` | PF가 쓰는 카메라 고도 편향 | 같은 카메라의 상자 광선에도 같은 편향이 있다 | 저장소 | **채택**(A2, 값 그대로) |
| OpenCV `cv2.findContours` / `cv2.approxPolyDP` | 격자 윤곽 → 꼭짓점 | 직교 벽 지도에는 사분면 검사가 정확하다 | Apache-2.0 | **기각**(1 cm 반올림, 면 정체 손실) |

### 2.2 공개 코드·라이브러리

| 후보 | 제공하는 것 | 맞는가 | 라이선스 / 고정 | 판정 |
|---|---|---|---|---|
| filterpy 1.4.5 `filterpy/kalman/kalman_filter.py`의 `predict`, `update` | 선형 칼만 예측(`P = FPFᵀ + Q`), Joseph 형태 갱신 | 정지 물체 2D 위치 트랙에 딱 맞다. 그러나 패키지가 scipy·matplotlib을 요구하는데 공용 `.venv-sim-worker-mac`에는 둘 다 없다(numpy 2.5.2, OpenCV 5.0만) | MIT, © 2015 Roger R. Labbe Jr. 파일 sha256 `67be531e…`, LICENSE sha256 `8ffce109…` | **적응**: 두 함수만 `harness/owncam_memory_kf.py`로 옮기고 `logpdf`를 numpy로 바꿨다. 공용 환경에 새 패키지를 넣지 않는다 |
| PythonRobotics `SLAM/EKFSLAM/ekf_slam.py` (commit `b2020cd`) | 마할라노비스 거리 대응(`search_correspond_landmark_id`, 새 랜드마크 문턱 `M_DIST_TH`), 관측으로 랜드마크 초기화 | 상자 트랙 대응·생성 규칙에 맞다. 로봇 자세는 PF가 따로 추정하므로 결합 SLAM 상태는 쓰지 않는다 | MIT, 파일 sha256 `8014f978…` | **적응**: 대응·새 트랙 규칙만 옮겼다(χ² 2자유도 99% 문턱) |
| PythonRobotics `Localization/particle_filter/particle_filter.py` (`b2020cd`) | PF | 이미 `owncam_localizer.py`에 적응됨 | MIT, `a649994b…` | **채택**(기존 적응 그대로) |
| PythonRobotics `Mapping/lidar_to_grid_map.py`, `ray_casting_grid_map.py`, `gaussian_grid_map.py` (`b2020cd`) | 라이다 광선 Bresenham·flood fill 점유 격자, 각도 bin 광선 투사, 가우시안 격자 | 센서가 라이다 광선이다. 우리 관측은 어안 카메라의 바닥 시야 다각형이다 | MIT, `e7d8324e…`, `4f06497d…`, `af37a464…` | **기각**(코드). 점유 표현은 Elfes/OctoMap의 log-odds를 따르고 다각형은 OpenCV로 채운다 |
| OpenCV 5.0.0 (`opencv-python-headless` 5.0.0.93, 이미 의존성) | `cv2.fisheye.projectPoints`, `cv2.fisheye.undistortPoints`, `cv2.fillPoly` | 예상 시야의 영상 투영, 바닥 시야 격자화 | Apache-2.0 | **채택** |
| ConceptGraphs (`concept-graphs/concept-graphs`, main) | 객체 단위 3D 장면 그래프, 다중 시점 객체 병합 | SAM·CLIP·LLaVA·PyTorch3D·CUDA 필요. Mac M3 CPU 실행 경로가 없다 | MIT | **설계 참고**: "객체 단위 기억 + 다중 시점 병합" |
| VLMaps (`vlmaps/vlmaps`, master) | CLIP/LSeg 특징을 top-down 격자에 융합 | RGB-D·GPU 필요 | MIT | **설계 참고**: 지도 좌표 top-down 격자 |
| CoELA (`UMass-Embodied-AGI/CoELA`) | 인식·기억·통신·계획 모듈 구조(기억 = 의미 지도 + 작업 진행 + 에이전트 상태) | 저장소에 LICENSE 파일이 없어 코드를 쓸 수 없다. 모듈 인터페이스는 이미 `coela_modules.py`에 있다 | 라이선스 파일 없음 | **설계 참고** |
| Khronos (MIT-SPARK, RSS 2024) | 존재·부재 증거로 물체 변화 시점 추론 | C++/ROS 전체 SLAM | 확인 안 함(코드 미사용) | **설계 참고**: 트랙에 `last_seen`과 `last_absent`를 둔다 |
| ReMEmbR (NVIDIA, 2024) | 시간·위치·관측 캡션 벡터 DB, 질의 | VLM 필요, 질의응답 목적 | 확인 안 함(코드 미사용) | **설계 참고**: 기억 항목에 시각·위치·출처 프레임을 붙인다 |

직접 쓴 알고리즘은 두 가지뿐이다. (1) pan 선택: Burgard·Fox·Thrun의 능동 위치 추정(센서 방향을 기대 엔트로피가 가장 줄도록 선택)을 **Fisher 정보 행렬식 탐욕 선택**으로 구현한 약 40줄. (2) 영상 안 가림 판정(운반 자세의 상자 띠 행 한계, 벽 상자와 시선 선분의 교차). 둘 다 맞는 소형 CPU 구현을 찾지 못했다.

## 3. 설계

### 3.1 입력 경계
- 로봇 자신의 `robot_cam` 프레임(검출은 기존 `TagDetector`, `detect_own`), 자기 `PoseReport`, 자기 발행 명령(서보 상태·짐 상태), 정적 태그 지도, 고정 교정만 쓴다.
- GT·TOP·`nav_cam`·다른 로봇 상태는 받지 않는다. 테스트가 import와 이름을 검사한다.
- 기억은 로봇마다 하나이고 공유하지 않는다. 통신 조건(무통신·대화)과 무관하게 같은 기억을 쓴다(연구 조건 간 동일 입력).

### 3.2 기억하는 것 (`harness/owncam_memory.py`, schema `ugrp.owncam_memory.v1`)
| 항목 | 표현 | 갱신 | 신선도 |
|---|---|---|---|
| ~~태그 고정 기록~~ **[대체됨 A1 → 0.3 표식 관측]** | 프레임마다 `{t, frame_id, 태그 ID, 자세 이름, pan, 추정 xyyaw, σ}`; 0.25 m 칸 × 자세 × pan별 시도/성공 수 | 태그 검출이 있는 프레임과 예상했는데 못 본 프레임 | 자세 σ 자체가 시간에 따라 커진다(PF). 칸별 성공률은 에피소드 안에서만 쓴다 |
| 상자 트랙 | 종류별 2D 칼만 트랙 `x, P`, 처음·마지막 관측 시각, near/far 횟수, 출처 frame_id, `last_absent` | `detect_own` 검출을 지도 좌표로 옮겨 갱신. 측정 공분산 = 검출 잡음(거리 비례) + 자세 공분산 전파(`J Σ Jᵀ`) | 매 조회 전 `P += q·dt`. σ와 나이로 `fresh / stale` |
| 빈 바닥·막힘 격자 | 0.05 m log-odds 격자(지도 경계) | 정지한 탐색 프레임의 near 바닥 시야 다각형 = 빈 바닥(상자 없음), 상자 검출 = 점유. 점유는 상자 종류를 기록 | 매 조회 전 log-odds × exp(−dt/τ), τ = 120 s. OctoMap식 상하한 clamp |

### 3.3 언제 보는가
기준선(OFF)의 σ 한계(짐: 0.07 m / 3°, 빈손: 0.05 m / 3°)와 M1 gate 한계는 **그대로** 둔다. 안전 기준을 바꾸지 않고 기억의 효과만 비교하기 위해서다.

| 계기 | OFF (loop v2 + M1) | ON (기억) |
|---|---|---|
| σ가 한계를 넘음 (`uncertain`) | 전체 둘러보기(6 pan) | **짧게 보기**: 지도·추정으로 태그가 보일 pan을 Fisher 정보 탐욕으로 1–3곳 고른다. 이 칸에서 전에 실패한 pan은 낮게 친다. 목표 사후 σ에 닿으면 멈춘다 |
| 보여야 할 태그가 안 보임 **[대체됨 A1: 보여야 할 지도 표식을 제공자가 연속 3프레임 못 봄]** | 없음(빈손은 태그 3 s 미검출 → 둘러보기) | 정지·정착 프레임에서 한 변 ≥ 20 px로 보여야 할 태그를 연속 3프레임 못 보면 **전체** 둘러보기 |
| 짐 상태 이동 0.5 m마다 | 둘러보기 | 없음(σ가 커지면 위 규칙) |
| 문 1.5 m·0.6 m 앞 | 둘러보기 | σ가 문 요구(0.05 m, 2°) 안이면 건너뜀, 밖이면 짧게 보기 |
| 도착 직전 확인 | 둘러보기 | σ가 고정 기준 안이면 건너뜀 **[A3: 정지 둘러보기 고정이 신선할 때만]** |
| 짧게 보기 뒤 고정 실패 | – | 전체 둘러보기로 1회 재시도(OFF의 refix 규칙과 같은 상한) |
| M1 gate(`release`, `look_back`, `post_manipulation`, `preplace`) | 전체 둘러보기 | 첫 번째는 짧게 보기, 같은 gate의 재시도는 전체. gate 한계·횟수(3회)는 같다 |
| 탐색 관측점 | 관측점마다 이전 검출 삭제, 6 pan | 검출을 트랙으로 누적. 바닥 시야가 이미 관측된 pan은 건너뛰고, pickup 영역 시야의 90% 이상이 관측된 관측점은 건너뛴다. 모두 건너뛴 뒤에도 못 찾으면 건너뛴 관측점을 전체로 다시 본다 |

### 3.4 재확인
- 파지 전: 목표 상자 트랙이 확정(near 2회 이상 일치)이고 σ ≤ 0.05 m, 나이 ≤ 60 s여야 접근한다. 아니면 그 자리에서 탐색 sweep을 다시 한다. 스킬 v9는 접근 중 자기 검출로 한 번 더 확인한다(수정 없음).
- 배치 전: 슬롯은 정적 지도다. M1 `look_back` gate(3 s 안의 둘러보기, σ ≤ 0.035 m)를 그대로 요구한다.
- 확정은 관측으로만 한다. 트랙을 못 봤다는 사실은 `unseen`이며 "없음"으로 바꾸지 않는다(#193 규칙).

### 3.5 비교 실험 (사전 등록은 실험 폴더)
- OFF: 동결 M1 학생 그대로(스킬 v9, `calibration_m1_dev.json`, A1–A6b). ON: 같은 학생 + 기억. 둘 다 같은 러너(`scripts/run_m1_owncam.run`)와 같은 판정으로 돈다.
- 지도 `zone_wide_door_tags_v1`(연구 시나리오 지도 계열), 새 seed, 동기 SIM, `cargo_noslip_v1`, weld OFF, 스레드 1, 동시 2개 이하, 코호트 전 여유 ≥ 30 GiB.
- 지표: M1 성공, SIM 소요, 둘러보기·pan 수, 명령 수, 평가 전용 위치 오차, 거짓 확인(주장 IN_SLOT인데 GT 밖, 기억이 확정한 상자의 GT 오차 > 0.10 m, 빈 바닥이라 기억한 칸에 실제 상자).
- `kiro/zone-map-v3`가 최종 비교 전에 위치 추정 게이트를 통과하면 v3에서도 비교한다.

### 3.6 위험
- 짧게 보기가 σ를 덜 줄이면 gate 재시도가 늘 수 있다 → 실패하면 전체 둘러보기로 되돌아가며, gate 한계는 같다.
- 예상 시야 모델(운반 자세 상자 띠 행 ≤ 170 px)은 v2 test 기록 942장에서 정했다. v1에서 다르면 불필요한 전체 둘러보기가 늘어 ON이 느려진다(안전 쪽 실패).
- 빈 바닥 기억이 검출 누락을 "없음"으로 굳히면 탐색이 목표를 건너뛸 수 있다 → 끝까지 못 찾으면 건너뛴 관측점을 다시 본다.

## 4. 참고 자료

논문(직접 확인한 것만):
- W. Burgard, D. Fox, S. Thrun, "Active Mobile Robot Localization," IJCAI 1997. http://ijcai.org/Proceedings/97-2/Papers/080.pdf
- N. Roy, W. Burgard, D. Fox, S. Thrun, "Coastal Navigation – Mobile Robot Navigation with Uncertainty in Dynamic Environments," ICRA 1999. https://www.ri.cmu.edu/publications/coastal-navigation-mobile-robot-navigation-with-uncertainty-in-dynamic-environments/
- A. Elfes, "Using Occupancy Grids for Mobile Robot Perception and Navigation," Computer 22(6), 1989. https://doi.org/10.1109/2.30720
- A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, W. Burgard, "OctoMap: an efficient probabilistic 3D mapping framework based on octrees," Autonomous Robots, 2013. https://doi.org/10.1007/s10514-012-9321-0
- H. Zhang, W. Du, J. Shan, Q. Zhou, Y. Du, J. B. Tenenbaum, T. Shu, C. Gan, "Building Cooperative Embodied Agents Modularly with Large Language Models" (CoELA), ICLR 2024. https://arxiv.org/abs/2307.02485
- Q. Gu et al., "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning," 2023. https://arxiv.org/abs/2309.16650
- C. Huang, O. Mees, A. Zeng, W. Burgard, "Visual Language Maps for Robot Navigation," ICRA 2023. https://arxiv.org/abs/2210.05714
- L. Schmid, M. Abate, Y. Chang, L. Carlone, "Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments," RSS 2024. https://arxiv.org/abs/2402.13817
- A. Anwar, J. Welsh, J. Biswas, S. Pouya, Y. Chang, "ReMEmbR: Building and Reasoning Over Long-Horizon Spatio-Temporal Memory for Robot Navigation," 2024. https://arxiv.org/abs/2409.13682
- A1–A3 대조에 쓴 논문(PR #230 참고 목록에서 확인 수준과 함께 옮김):
  - F. Dellaert, D. Fox, W. Burgard, S. Thrun, "Monte Carlo Localization for Mobile Robots," ICRA 1999. https://doi.org/10.1109/ROBOT.1999.772544
  - S. Lenser, M. Veloso, "Sensor Resetting Localization for Poorly Modelled Mobile Robots," ICRA 2000. https://doi.org/10.1109/ROBOT.2000.844766
  - D. M. Rosen, J. Mason, J. J. Leonard, "Towards Lifelong Feature-Based Mapping in Semi-Static Environments," ICRA 2016. https://doi.org/10.1109/ICRA.2016.7487237 (persistence filter, 후속 존재 확률)
  - L. L. S. Wong, T. Lozano-Pérez, L. P. Kaelbling, "Not seeing is also believing: Combining object and metric spatial information," ICRA 2014. http://hdl.handle.net/1721.1/100724
  - P. Liu, Z. Guo, M. Warke et al., "DynaMem: Online Dynamic Spatio-Semantic Memory for Open World Mobile Manipulation," 2024. https://arxiv.org/abs/2411.04999

공개 코드·라이브러리:
- filterpy 1.4.5 — https://github.com/rlabbe/filterpy (MIT). `filterpy/kalman/kalman_filter.py`의 `predict`·`update`를 적응(`harness/owncam_memory_kf.py`).
- PythonRobotics commit `b2020cd` — https://github.com/AtsushiSakai/PythonRobotics (MIT). `SLAM/EKFSLAM/ekf_slam.py`의 마할라노비스 대응·새 랜드마크 규칙을 적응. `particle_filter.py`는 기존 `owncam_localizer.py`가 적응. 격자 3종은 검토 후 기각.
- OpenCV 5.0.0 (`opencv-python-headless` 5.0.0.93, Apache-2.0) — `cv2.fisheye.distortPoints`(표식·태그 투영), `cv2.fisheye.undistortPoints`(기존 `TagDetector`). A1 표식 목록에는 `cv2.findContours`/`cv2.approxPolyDP`를 검토 후 기각.
- ConceptGraphs — https://github.com/concept-graphs/concept-graphs (MIT), 설계 참고만.
- VLMaps — https://github.com/vlmaps/vlmaps (MIT), 설계 참고만.
- CoELA — https://github.com/UMass-Embodied-AGI/CoELA (LICENSE 파일 없음), 설계 참고만.
- Khronos — https://github.com/MIT-SPARK/Khronos, 설계 참고만(라이선스 미확인, 코드 미사용).

저장소 안 재사용(경로):
- `harness/owncam_localizer.py`, `harness/owncam_pose_source.py`, `harness/wall_tags.py`, `harness/zone_color_boxes.py`, `harness/markerless_box.py`, `harness/map_goto.py`, `harness/owncam_drive.py`, `harness/owncam_drive_v2.py`, `harness/m1_owncam_delivery.py`, `scripts/run_m1_owncam.py` (PR #177, #178, #197, #201).
- 설계 참고: `harness/coela_modules.py`, PR #193 `harness/zone_own_outcome.py`·`judge_route_blockage`, PR #206 `status().blocked_ahead`, `harness/multi_object_tracking.py`.
- 문헌 조사 PR #230 `docs/design/2026-09-26-memory-literature.md`(`kiro/memory-literature` `3174c2f`): 0.6절 대조.
- A1–A3 개정: PR #210 `docs/design/2026-09-26-markerless-localization-and-memory.md` 4(b)(관측 항목 형식), PR #227 `kiro/zone-vision-loc` `experiments/2026-09-26-vision-loc/maps/zone_wide_door_walls_v3_notags.json`(태그 없는 지도, 검출기 자리), PR #208 `maps/zones/zone_wide_door_tags_v3.json`, `experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json`(`elevation_bias_rad`, A2), `harness/visual_arm.py` `camera_extrinsics`(명령 PWM FK), `harness/owncam_drive_v2.py` `LOADED_LOOK_EVERY_M_V2`(A3 0.5 m 근거).

문서·웹 페이지:
- filterpy 문서 https://filterpy.readthedocs.io/en/latest/ 와 저장소 README(의존성 NumPy·SciPy·Matplotlib 확인).
- ConceptGraphs README(설치: CUDA, Grounded-SAM, LLaVA), VLMaps README(Habitat·RGB-D), CoELA README.
- `docs/zone_owncam_localization.md`, `experiments/2026-09-26-zone-m1-owncam/README.md`, `experiments/2026-09-26-zone-owncam-loop-v2/README.md`, Codex 연구 설계 `docs/design/2026-09-25-zone-dialogue-study-design-codex.md`(`origin/claude/records-0926`).
