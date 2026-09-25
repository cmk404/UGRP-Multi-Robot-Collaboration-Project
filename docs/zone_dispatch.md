# 구역 배송 협업 벤치마크 (교사 실행기)

2026-09-25 사용자 요청: 맵을 넓히고 물건을 늘린 뒤, "A 구역에 빨강 2개, B에 청록 1개, C에 초록 1개와 빨강 1개"처럼 구역별 수량 목표를 두고, 계획 먼저(plan_first)와 필요할 때만 대화(dynamic)의 차이를 본다.

**이 벤치마크는 협업 층을 재는 도구다.** 로봇 이동과 파지는 정답 좌표·IK를 쓰는 **교사 실행기**가 실제 집게로 한다(weld OFF). 결과는 "교사 실행기 조건"이며 RGB 스킬 성공으로 보고하지 않는다. RGB 스킬로의 교체는 후속 단계다.

## 은퇴한 장면 (`zones/zone_open`, 2026-09-25 은퇴)

2026-09-25 사용자 요청("옛날의 작은 맵은 없애주라")으로 작은 `zone_open`은 새 실행에서 쓰지 않는다. 기본 지도는 `zone_wide`다. `maps/zones/zone_open.json`과 배치 코드는 Z1–Z3 기록과 재생을 재현하려고 바이트 그대로 남긴다. 러너는 `--variant zone_open`을 거절하며, 재현할 때만 `--allow-retired-variant`를 함께 준다. 장면 목록(`sim.zone_scene.catalog()`)에도 나오지 않는다. 아래 설명은 그 기록을 읽기 위한 것이다.


- 경기장 6.45 × 2.3 m(x −1.05…5.40, y −3.15…−0.85). 서쪽 적재 구역(파랑 바닥, 지도 v2: 상자 자리 4열 × 3행, 0.6 m 간격이라 상자를 든 로봇도 사이로 지나갈 수 있다. 최대 12개), 동쪽 구역 A(주황, 북동)·B(파랑, 남동)·C(보라, 중앙). 구역마다 칸 3개가 y 방향으로 놓인다.
- TOP은 승인된 `cctv_top`을 그대로 두고, 같은 규격(높이 2.5 m, FOV 55°, 아래 방향)의 `cctv_top_east`를 3.3 m 동쪽에 추가했다. 로봇 외관·자기 카메라는 바뀌지 않는다.
- 상자는 기존 청록 상자와 모양·질량·마찰이 같고 **칠만 다르다**(cyan, red, green, yellow). 로봇이 영상에서 종류를 구분하기 위한 변경이다.
- 장면은 표준 `sim.session_scenes.Scene`을 상속한 `sim.zone_scene.ZoneScene`이 만든다(소스 기록·스폰/물체 초기화·기록 형식 재사용). 고정된 RGB 운반 번들 소스를 바꾸지 않으려고 별도 파일에 두었고, 공용 장면 목록 등록은 다음 운반 번들 등록 때 함께 한다.
- 지도는 `maps/zones/zone_open.json`(코드 정의와 일치 검사), 목표·배치는 `sim/zone_arena.py`의 `episode(goal, extra_boxes, seed)`가 만든다. 배치는 설정 전용이며 로봇에게 주지 않는다.

## 넓은 장면 (`zones/zone_wide`, 2026-09-25)

사용자 요청(2026-09-25): 경기장이 좁아 보이니 전체를 넓힌다. `zone_open` v2는 Z1–Z3 재현을 위해 바이트 그대로 두고 별도 변형으로 추가했다(`--variant zone_wide`).

- 경기장 6.45 × 4.6 m(x −1.05…5.40, y −3.15…1.45). 남쪽 벽과 동서 길이는 `zone_open`과 같고 북쪽으로 2.3 m 늘렸다.
- TOP은 승인된 `cctv_top`과 `cctv_top_east`를 그대로 두고, 같은 규격의 `cctv_top_north`·`cctv_top_north_east`를 2.3 m 북쪽에 추가했다. 네 시야는 바닥에서 약 0.3 m씩 겹치고 벽 안의 모든 바닥을 덮는다(테스트로 확인). 로봇은 자기 RGB와 TOP 네 장(`TOP_SW`, `TOP_NW`, `TOP_SE`, `TOP_NE`)을 받으므로 요청당 이미지가 3장에서 5장으로 는다.
- 적재 구역: 4열(0.6 m) × 5행(0.8 m), 최대 20개. 출발 자리 y −2.25, −0.85, 0.55.
- 목표 구역은 0.6 × 1.4 m(`zone_open`의 약 3.3배), 칸 3개를 0.40 m 간격으로 둔다. A(주황, 북동 4.60, 0.40), B(파랑, 남동 4.60, −2.10), C(보라, 가운데 3.00, −0.85).
- 조명은 `zone_open`과 같은 천장 조명 하나를 경기장 가운데 위로 옮긴 것이다. 관찰 카메라(`cctv_warehouse`)와 재생 창은 넓은 바닥에 맞춰 뒤로 물렸다(관찰 전용).
- `zone_open`의 프롬프트·장면 XML·지도 파일이 main과 같음을 확인했다. `zone_wide`의 교사 실행 확인은 [실험 기록](../experiments/2026-09-25-zone-wide-arena/README.md)에 있다.

## 벽·문 경로 변형 (`zone_wide_door`, `zone_wide_two_doors`, `zone_wide_corridor`, 2026-09-25)

최종 연구 환경을 만들기 위해, 로봇이 서로 비켜야 하는 병목과 우회로를 넣은 변형이다. `zone_wide`의 경기장·TOP 네 대·적재 격자·출발 자리·구역 A/B/C를 그대로 두고, 안쪽 벽만 더했다(각 지도 v1, `maps/zones/<변형>.json`). `zone_wide` v1은 바이트 그대로다.

| 변형 | 안쪽 벽 | 통로 |
|---|---|---|
| `zone_wide_door` | x = 2.20 남북 벽(적재 쪽 서, 구역 쪽 동) | 문 하나 0.50 m(중심 y 0.05), 한 대씩 |
| `zone_wide_two_doors` | 같은 벽 | 좁은 문 0.50 m(y 0.05, 한 대씩) + 넓은 문 1.00 m(남쪽 끝 y −3.125…−2.125, 두 대 나란히). A·C로 가는 길은 좁은 문이 짧고, 넓은 문은 우회로다. B로 가는 길은 넓은 문이 가깝다 |
| `zone_wide_corridor` | 벽이 y 0.90에서 동쪽으로 꺾여 복도 벽이 된다 | 북쪽 벽을 따라 x 2.20…3.925, 폭 0.50 m의 한 차선 복도 + 비켜 서는 자리 1곳(x 2.80…3.40, y 0.35…0.90) |

- 벽은 바깥 벽과 같은 두께 0.05 m, 높이 0.10 m, 같은 색이다. x = 2.20은 서쪽·동쪽 TOP 줄이 함께 보는 겹침 띠(x 2.14…2.26) 안이고 교사 격자(0.05 m) 위에 있다.
- 문 폭 0.50 m: 상자를 든 로봇이 도는 데 필요한 지름(앞 0.195 m, 약 0.39 m)에 0.11 m를 더했다. 교사는 상자를 들면 반지름 0.21 m 원으로 계획하므로, 든 로봇은 문 중심선 ±0.04 m 안에서만 지나가고, 두 로봇(동료 여유 0.14 m 포함 0.65 m 필요)은 동시에 들어갈 수 없다. 로봇 차체는 약 0.18 × 0.16 m라 빈 차체 두 대는 물리적으로는 비껴 설 여유가 조금 있다. 한 대씩이라는 조건은 교사 계획 기준이다.
- TOP 확인(테스트): 벽면에서 0.08 m 이상 떨어진 모든 바닥점과 문·복도·비켜 서는 자리의 바닥을 적어도 한 TOP이 본다. 높이 0.10 m 벽은 그 면을 보는 TOP 쪽 반대편에 폭 약 4–6 cm의 바닥 띠를 가린다. 로봇 중심은 벽에서 0.17 m 이상 떨어져 있어 그 띠에 들어가지 않는다.
- 로봇 자기 RGB: 카메라가 0.21 m 높이에서 약 22° 아래를 보므로 0.10 m 벽 너머의 바닥도 보인다. 문 앞에서 찍은 예시는 실험 기록의 `robot-rgb-at-door.png`다.
- 로봇 프롬프트: 이 변형들에서만 작성 지도의 벽·통로 설명(`sim.zone_arena.static_map_text`)을 `Static map:` 한 문단으로 덧붙인다. 벽 좌표, 문 폭, 한 대씩 지나가는지, 비켜 서는 자리의 위치다. 정적 지도 정보뿐이며 실시간 위치·배치는 넣지 않는다. 바깥 벽만 있는 지도(`zone_wide`, 은퇴한 `zone_open`)의 프롬프트는 바이트 그대로다(테스트).

### 교사 실행기의 벽과 통로 규칙

- 정적 장애물: `harness/static_keepouts.py`가 지도의 안쪽 벽을 회전 가능한 직사각형 `(cx, cy, hx, hy, yaw)`으로 한 번 정의한다. 원판 A*는 로봇 반지름(빈 로봇 0.17 m, 상자를 들면 0.21 m. 작업에 `carry_radius_m`이 있으면 더 큰 값)만큼 벽에서 떨어진다. 이미 그보다 가까우면 더 가까워지지만 않게 한다. 나중의 공동 운반 실행기는 같은 직사각형을 `pose_clear(pose, footprint)`와 `swept_clear(a, b, footprint)`(볼록 다각형, SAT)로 쓴다. 안쪽 벽이 없는 지도에서는 탐색이 바뀌지 않는다(계획 digest 테스트).
- 한 차선 통로의 대치: 같은 통로 구역(문은 앞뒤 0.5 m, 복도는 끝에서 0.35 m, 옆으로 0.3 m. 비켜 서는 자리는 대치 구역이 아니다)에 **2초 이상 멈춰 있는** 로봇이 둘 이상이면, 물리적 우선순위가 낮은 로봇이 차선에서 벗어난다. 벗어나는 곳은 통로 중심선과 상대 로봇에서 0.40 m 이상 떨어진 가장 가까운 자리이며, 비켜 서는 자리일 수도 있다. 우선순위는 (1) 통로 안에 있는 로봇, (2) 들린 상자가 옆에 있는(상자를 든) 로봇, (3) 작은 로봇 id 순이다. 자기 작업이 없는 로봇은 늘 비킨다. 집거나 내려놓는 중인 로봇은 움직이지 않는 장애물로 본다. 대치가 6초 넘게 풀리지 않으면(비킬 자리가 없는 경우) 다른 로봇이 대신 비킨다. 비키기는 상대가 통로 구역을 떠나거나 20초가 지나면 끝난다.
- 이 규칙이 읽는 것은 현재 물리 상태(위치, 멈춘 시간, 로봇 옆 들린 상자)와 정적 지도뿐이다. 동료의 작업·목적지·계획 경로·선언은 읽지 않는다. 그래서 plan_first, dynamic, independent에서 똑같이 동작한다(테스트: 같은 물리 상태에서 작업을 바꿔도 같은 로봇이 비킨다). 기존의 일반 비키기 규칙(막힌 로봇의 경로 위 동료에게 비키기를 요청)은 v1과 같게 남겼다. 그 규칙은 요청한 로봇의 경로를 쓴다. 벽 지도에서는 같은 통로 구역 안의 두 로봇에게 그 규칙을 적용하지 않는다. `_taken_by_peer`의 의도 기반 중재(감사 PR #161 L1)는 이 변경에서 고치지 않았다.
- 이동 단계 한도: 벽 지도에서는 우회와 통로 대기 때문에 240초다(`ROUTE_DRIVE_PHASE_LIMIT_S`). 바깥 벽만 있는 지도는 120초 그대로다.
- 기록: 벽 지도에서만 `teacher-events.json`에 `passage_standoff`, `passage_yield`, `passage_yield_target`, `passage_yield_end`, `passage_wait`(통로 구역에서 막히거나 비키며 기다린 시간)을 남긴다.

실행 예:

```bash
.venv-sim/bin/python -m scripts.run_zone_dispatch --output outputs/zone-NEW --variant zone_wide_door \
  --coordination dynamic --mode fixture --goal '{"A":{"red":2},"B":{"cyan":2},"C":{"green":1,"yellow":1}}' \
  --extra-boxes '{"red":1,"cyan":1}' --seed 12 --record-replay
```

교사·fixture 결과(LLM 협업 근거가 아님)는 [실험 기록](../experiments/2026-09-25-zone-hard-routes/README.md)에 있다.

## 로봇이 받는 것 / 받지 않는 것

- 받음: 자기 RGB, `TOP_WEST`·`TOP_EAST` RGB(`zone_wide`는 TOP 네 장), 목표·작성 지도, TOP RGB 색 검출로 만든 상자 이름(색별로 서→동, 같은 열은 남→북; `red-1` 등)과 현재 RGB 추정(적재 구역에 남은 상자, 구역별 색 개수), 팀 게시판(진행 중인 동료 작업, 실행기 영수증), 자기 작업 이력, 동료 메시지.
- 받지 않음: 시뮬레이터 위치, 심판 결과. 실행기 영수증은 "발행한 동작 순서를 끝냄/끝내지 못하고 멈춤"뿐이다(교사 내부의 정답 확인이 멈춤 여부에 쓰이므로 교사 조건 입력으로 기록한다).
- 무통신(`independent`) 포함 조건별 채널, 누설·교란 판정: [R1 경계 감사](../experiments/2026-09-25-zone-comm-boundary-audit/README.md). 모델이 보는 자기 작업 기록에는 칸 id가 없고(2026-09-25), independent 요청 id는 자기 질문 횟수만 센다.

## 협업 방식

| | plan_first | dynamic |
|---|---|---|
| 출발 전 | 세 로봇이 **전체 배분**(로봇별 순서 있는 {상자, 구역} 목록)을 제안자 순환·만장일치로 합의 | 없음 |
| 실행 중 | 합의한 목록을 순서대로 실행, 다시 대화하지 않음 | 쉬는 로봇이 **다음 일 하나**를 스스로 선언. 호스트는 목표·RGB 추정·동료 선언과 맞는지만 확인 |
| 대화 | 협상 전체 | 선언이 겹칠 때(같은 상자, 같은 마지막 필요분)만 해당 로봇끼리 최대 2회. 모두 양보하면 아무도 안 가져가므로, 동료 메시지가 특정 로봇에게 넘긴 경우가 아니면 가장 작은 id가 가져간다 |
| 실패 | 실패한 작업은 끝나지 않은 채 남음 | 다음 선언에서 남은 필요분을 다시 맡음 |

## 교사 실행기 (`scripts/zone_teacher.py`)

격자 A*(0.05 m, 벽·남은 상자·다른 로봇을 원형 금지 영역으로)와 메카넘 명령으로 이동하고, 상자를 로봇 앞 15.5 cm·동쪽 방향에 맞춘 뒤 기존 상자 스킬과 같은 IK 순서(위 9.5 cm → 2.4 cm, 집게 1500으로 닫기, 들기)로 잡는다. 들어 올린 뒤 정답 높이로 확인하고 최대 2회 다시 잡는다. 칸에 내려놓고 열고 물러난다. 칸 선택은 발행한 작업 기록만으로 한다.

- 이동 단계(상자로, 칸으로)가 120초(벽 지도는 240초) 안에 끝나지 않으면 `teacher_path_blocked`로 멈추고 칸을 반납한다.
- 남은 상자의 여유가 출발점과 겹쳐 경로가 없을 때만 그 여유를 푼다. 다른 로봇 주변은 풀지 않고, 겹쳤을 때는 더 가까워지지 않게만 한다.
- 2초 이상 경로가 없는데 다른 로봇을 빼면 경로가 있으면, 그 경로 위의 쉬는 로봇이나 역시 막혀 있는 id가 더 큰 로봇이 경로에서 떨어진 가장 가까운 자리로 비킨다(최대 20초, `teacher-events.json`의 `yield`).

## 실행

```bash
.venv-sim/bin/python -m scripts.run_zone_dispatch --output outputs/zone-NEW --variant zone_wide \
  --coordination dynamic --goal '{"A":{"red":2},"B":{"cyan":1},"C":{"green":1,"red":1}}' \
  --extra-boxes '{"red":1}' --seed 11 --record-replay
```

`--record-replay`로 기록한 실행은 `.venv-sim/bin/python -m scripts.zone_replay <출력 폴더>`로 MuJoCo 창에서 다시 본다(관찰 전용, 넓은 경기장 화면).

`--mode fixture`는 모델 없이 규칙 응답으로 절차만 확인한다(시각 판단 아님). 진단용 `--fixture-plan <committed-plan.json>`(fixture·plan_first 전용)은 기록된 합의 계획을 그대로 다시 실행한다. 결과 `result.json`: `physical_success_teacher_condition`(심판, 교사 조건), `goal_met_rgb`, `makespan_sim_s`, `llm_calls`, `usage`, `coordination_stats`(충돌·무효 선언·협상 턴), 로봇별 `jobs`. 원본 대화는 `team/`, 교사 사건은 `teacher-events.json`.

## RGB 4색 상자 검출 (교사 교체 1단계, 2026-09-25)

교사 실행기를 RGB 스킬로 바꾸는 첫 단계로, 로봇 자기 RGB와 TOP RGB에서 네 색 상자를 찾는 검출기와 오프라인 평가를 추가했다. 이동·파지·배치는 아직 교사 실행기이며, 이 검출기는 제어 경로에 연결되지 않았다.

- `harness/zone_color_boxes.py`: 입력은 자기 RGB JPEG + 자기가 **발행한** 팔 펄스, TOP JPEG + 작성된 TOP 보정뿐이다. 시뮬레이터 자세·분할·접촉은 읽지 않는다. 색은 종류(kind)만 알려 주며 개별 상자 ID는 해독하지 않는다.
  - 기본 프로필은 기존 동작 그대로다: 자기 RGB `own_production_v1`(청록은 `markerless_box._cyan_components` 자체, 1.2 m 이내 바닥 투영 적합), TOP `zone_perception_v1`(= `zone_perception.detect_boxes`, ZC1/ZC2가 쓴 경로). `markerless_box.py`·`zone_perception.py`는 바꾸지 않았다.
  - 선택 프로필: `own_zone_v2`(구역 적재 바닥 파랑을 뺀 좁은 청록 범위, 1.2–4 m 원거리 거친 적합 `far_coarse`), `top_zone_v2`(회전 불변 `minAreaRect` 채움·종횡비, 3×3 닫기, 최소 50 px — 노랑 메카넘 롤러 오검출 억제). 임계값은 dev 분할에서만 정했다.
- `scripts/eval_zone_color_detection.py` (`zone-color-eval` 작업 흐름): `render`는 표준 `ZoneScene`(`zones/zone_wide`)을 동기 SIM에서 만들고 로봇·상자를 다시 배치해 프레임(`frames/`)과 평가 전용 정답(`eval-labels/`: 자세·분할·가시 비율)을 따로 쓴다. `score`는 검출기를 `frames/`에만 돌리고 정답과 비교한다. 카메라·로봇 외관·상자 칠은 바꾸지 않는다. `dispatch/open`은 빔 혼동 확인용 부록이다(구역 장면에서는 빔이 보이지 않는다). `zone_open`은 퇴역 대상이라 평가하지 않는다.

```bash
.venv-sim/bin/python -m scripts.eval_zone_color_detection render --split dev --output outputs/zone-rgb-color/dev
.venv-sim/bin/python -m scripts.eval_zone_color_detection score --frames outputs/zone-rgb-color/dev --output outputs/zone-rgb-color/dev-score
```

결과와 한계는 [실험 기록](../experiments/2026-09-25-zone-rgb-color/README.md)에 있다.

## 화물 종류·팀 운반 목표: protocol v2 (구역 팀 A2, 2026-09-25)

목표에 화물 목록 종류(can, tile, long_beam, heavy_crate, tri_frame)가 있으면 실행기는 자동으로 protocol v2를 쓴다(`--protocol auto`). v2는 `--contact-profile`을 반드시 명시해야 한다. 목표만 바꿨는데 물리 조건이 몰래 바뀌지 않게 하려는 것이다(A2 스모크는 `cargo_noslip_v1`). 색만 있는 목표는 지금까지와 같은 v1 경로로 간다. 기본값도 v1 그대로다(900 SIM초, `local_contact_fine`). `--protocol v2`를 명시하면 색만 있는 목표도 v2로 실행한다. 설계와 A1 부품은 [팀 작업 A1 기록](../experiments/2026-09-25-zone-team-jobs/README.md)에 있고, 연결 결과와 스모크는 [A2 기록](../experiments/2026-09-25-zone-team-a2/README.md)에 있다.

```bash
.venv-sim/bin/python -m scripts.run_zone_dispatch --output outputs/zone-v2 --mode fixture \
  --coordination dynamic --variant zone_wide_two_doors --seed 11 --record-replay \
  --contact-profile cargo_noslip_v1 \
  --goal '{"A":{"long_beam":1},"B":{"heavy_crate":1,"red":1},"C":{"can":1,"green":1,"tile":1}}'
```

| 항목 | v2 동작 |
|---|---|
| 장면 | 색 상자는 `sim.zone_arena.episode`로 그대로 놓는다. 화물은 `harness/zone_mixed_episode.py`가 seed마다 정해진 방식으로 놓는다. 팀 발자국과 정거장 뒤 접근 자리가 상자·출발점·벽·통로 앞 차선과 겹치지 않아야 한다(설정 전용). `CargoZoneScene`을 쓰고, 접촉 프로필은 명시한 값(스모크는 `cargo_noslip_v1`), weld는 OFF다. 목표에 색 상자가 하나 이상 있어야 한다 |
| 로봇 입력 | 자기 RGB, TOP RGB 네 장, TOP RGB 라벨과 현재 추정(`harness/zone_perception_v2.py` = `top_cargo_v1` + 그 상자 경로; 결과의 `perception_profile`에 기록. `top_cargo_v1`은 보고된 그대로 두고, 틀 안쪽 억제 수정은 별도 `top_cargo_v2`에서 한다)을 받는다. 화물 라벨에는 RGB 손잡이 위치가 붙는다. 정적 과제 글은 세 조건에서 같다. 종류별 필요 인원·역할, 착지 영역(id 없음), 팀 형성 규칙, 벽·문 지도가 들어 있다 |
| 선언 | 세 조건 모두 `{"item", "zone", "role"}` 형식이다. 검사는 independent `check_independent_claims`(로봇별, 서로 비교 안 함), dynamic `check_dynamic_claims`, plan_first `validate_team_plan`이 한다. 호스트는 팀원을 고르지 않는다 |
| 조건 스위치 | `harness/zone_protocol_v2.CONDITIONS`: 게시판, 호스트 중재, 충돌 알림, 동료 작업 종료 시 깨우기, 메시지 전달을 기제마다 따로 둔다. 조건 이름은 기본값만 정한다. `--condition-switches '{"peer_board": false}'`로 하나씩 바꿀 수 있다. 그 조건의 반복문이 읽지 않는 스위치를 바꾸려 하면 거부한다. 결과에 기본값과 실제값을 함께 기록한다 |
| 역할 계약 | 종류마다 formation이 여럿일 수 있다(`formations`, `claim_roles`). tile은 west 또는 east 중 한쪽을 한 대가 맡는다. 한 tile의 두 쪽을 동시에 선언하면 호스트는 `formation_conflict`로 처리한다. 두 로봇이 두 쪽에 서도 팀이 되지 않는다(id로 정하지 않음) |
| 교사 | `scripts/zone_team_teacher.py`에서 모든 작업이 TeamJob이다(단독 물건은 1인 팀). 로봇은 자기 역할 정거장으로 가서 기다린다. 모든 정거장이 몸으로 채워지고 spec이 같을 때만 commit한다. 자기 도착 뒤 최대 60 SIM초까지 기다린다. 역할은 RGB 손잡이의 접근 자세(정거장 위치·방향)에 가장 가까운 실제 역할로 묶는다. 이름으로 묶지 않는다. 접근 자세가 없으면 손잡이 위치를 쓴다. east tile은 착지 영역의 정거장이 같도록 물건을 180° 돌려 내려놓는다 |
| 단일 통로 진입 | v2 로봇만 적용한다. 통로 구역 안에서 자기 경로가 문을 지나면, 다른 로봇이 문 안에 서 있거나 문에 더 가까이 움직이는 로봇이 있는 동안 기다린다. 최대 60 SIM초다. 몸의 위치·정지 여부·정적 통로·자기 경로만 읽는다. 동료의 선언·id는 읽지 않는다. 팀 운반을 마친 두 로봇이 함께 0.5 m 문에 나란히 끼어 멈춘 문제(A2)를 막으려고 넣었다. 대기는 결과의 `door_gate_waits`에 남는다 |
| 멈춤(L1 수정) | 정거장에 더 가까운 몸이 서 있을 때, 또는 물건이 이미 잡히는 중이거나 배달됐을 때만 멈춘다. 동료의 선언만으로는 멈추지 않는다. v1 교사에도 같은 수정을 적용했다 |
| 팀 운반 | commit 직후, 접촉 전에 물건 자세 공간에서 경로를 한 번 계획한다(`harness/zone_team_route.py`). 직선 이동과 제자리 90° 회전만 쓰고, 회전은 원판 전체가 빈 곳에서만 한다(문 안 회전 없음). 각 구간을 static_keepouts swept 검사로 다시 확인한다. 경로가 없으면 접촉 전에 취소한다(cancel_retreat). 다른 팀은 발자국 전체를 장애물로 본다 |
| 실패 | 접촉 전에는 모두 물러난다. 접촉 뒤에는 모두 멈추고 함께 내려놓고 놓고 물러난다(hold_lower). 12초 강제 파지와 막혔을 때 집게를 여는 처리는 v2에 없다 |
| 착지·심판 | 슬롯 대신 착지 영역(`landing_layout`)을 쓴다. 영역 id는 모델에 보이지 않는다. `referee_v2`(전체 발자국, 평가 전용)로 판정하고, 목표는 물건당 한 번만 줄어든다 |
| 결과 | `result.json`(schema `ugrp.zone_dispatch_result.v2`)에 `referee_v2`, 물건별 결과, 팀 형성 대기, 문 대기, 운반 멈춤, 미끄러짐·떨어뜨림, `eq_active_max`, 부하를 기록한다. `team_executor`에는 장부, 선언, 경로, 로봇 사건 hook 기록이 있다 |

- 로봇에게 알리는 결과(자기 작업 상태, 게시판 보고, 재질문 시점, 호스트 검사·fixture가 쓰는 배달 목록)는 `harness/zone_outcomes_v2.RobotResults`를 거친다. 교사 운동 장부(`TeamJobLedger`)와 분리되어 있다. 지금 결과 출처는 교사 영수증(`teacher_receipt_L4`)뿐이다. 이것은 R1 L4가 풀리지 않은 상태이며 교사 스모크에만 쓴다. 모든 결과의 `robot_facing_outcome_source`에 이 표시가 남는다. RGB 완료 확인(#170)은 같은 자리에 결과 출처로 끼운다.
- 모든 이동은 정답 교사가 한다. 결과는 교사 조건이며 학생·RGB 스킬 성공이 아니다.
