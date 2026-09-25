# 2026-09-25 넓은 구역 경기장 (`zones/zone_wide`) 교사 실행 확인

사용자 요청(2026-09-25): 구역 배송 경기장이 보기에 좁으니 전체를 넓힌다. 기존 `zone_open` v2는 Z1–Z3 재현을 위해 그대로 두고 `zone_wide` v1을 새로 추가했다. 장면 설명은 [docs/zone_dispatch.md](../../docs/zone_dispatch.md#넓은-장면-zoneszone_wide-2026-09-25)에 있다.

**범위:** 규칙 응답(fixture, LLM 0회)으로 교사 실행기가 새 지도에서 끝까지 동작하는지 확인한 뒤, 실제 LLM 코호트 ZW1([사전 등록](protocol.md))을 돌렸다. 모두 교사 실행기 조건이며 RGB 스킬 성공이 아니다. `zone_open`과의 SIM 시간 비교는 하지 않는다.

## 변경 확인

- `zone_open`: 지도 파일, 장면 XML 해시(`58bc404e…`), 두 협업 모드의 전체 프롬프트·이미지 목록·작업 설명·배치가 main `6b787a4`와 같다.
- `zone_wide`: 6.45 × 4.6 m, TOP 4대(승인 TOP 2대 + 같은 규격 북쪽 2대). 벽 안의 바닥 전체가 적어도 한 TOP에 보인다(테스트).
- TOP RGB 색 검출: seed 12의 상자 8개를 모두 한 번씩 찾았고 위치 오차는 1 cm 이하였다. 겹치는 영역(y −0.85 줄)의 상자도 중복 없이 처리됐다(TOP 네 장을 테스트 fixture로 고정).
- 교사 경로: seed 11, 12에서 상자 12개(G8 + 여분 3개)를 놓았을 때, 모든 출발점→상자와 상자→칸 경로가 계획된다(테스트).
- `scripts/run_ci_tests.py`: 2,248개 통과, 9개 건너뜀.

## Fixture 실행 (소스 `bc4a784`)

동기 SIM, `--mode fixture --record-replay --variant zone_wide`, 접촉 프로필 `local_contact_fine`, weld OFF. 조건마다 1회.

| 실행 | 조건 | 결과(심판) | TOP RGB | 완료 SIM초 | 충돌 | 교사 막힘/양보 | wall초 | 시작 부하 평균(1분) |
|---|---|---|---|---:|---:|---|---:|---:|
| W-G5-dyn | dynamic, G5, seed 11 | **5/5** | 일치 | 153.0 | 0 | 0 / 0 | 69.2 | 2.45 |
| W-G8-plan | plan_first, G8(여분 빨강·청록 1), seed 12 | **6/6** | 일치 | 156.0 | 0 | 0 / 0 | 77.6 | 2.54 |
| W-G8-dyn | dynamic, G8(여분 빨강·청록 1), seed 12 | **6/6** | 일치 | 154.0 | 0 | 0 / 0 | 78.5 | 3.51 |

- 모든 작업이 `placed_by_teacher`로 끝났다. `teacher_path_blocked`, `yield`, 파지 재시도는 0건이다.
- 규칙 응답이므로 협업 판단의 증거가 아니다. SIM 시간은 지도와 응답 출처가 다른 Z3(LLM, `zone_open`)와 비교하지 않는다.
- 요청당 이미지가 3장에서 5장으로 늘어 LLM 실행의 입력 토큰이 커진다. 실제 증가량은 LLM 실행에서 측정해야 한다.

## 원본과 대시보드

원본은 기본 체크아웃 `outputs/zone-wide-20260925/`에만 있다(로컬 보관이며 원격 백업이 아니다). 부하 평균은 `load.txt`에 있다.

| 파일 | SHA-256 |
|---|---|
| dev-g5-dyn/result.json | `1193e641b9665a9b44e91fbbe6cc49a94cba841062f021545a3b7285fe6eed86` |
| dev-g8-plan/result.json | `f75c8d1d7705f6afb3e58ab24313d9c5ccb9a4920738d8d88c2642e4a56b6d9f` |
| dev-g8-dyn/result.json | `d31dc8421aa570435a0bd8209d849285004eacc640720206332d4333a6e19ae3` |
| dev-g5-dyn/teacher-events.json | `8cac057d0949ab9332ec41dccff443daadce6a5a1ce780a7d590842e4c7041ef` |
| dev-g8-plan/teacher-events.json | `20e710f85e850305e3e08c35bc0b633049602acd3de94b060fd7be616750bc3c` |
| dev-g8-dyn/teacher-events.json | `dcfe638620db68cfa35a04ba4b3fbdfaca58cc654e85317f366547277ef25394` |

- TensorBoard 스냅샷은 `outputs/tensorboard/0925-zone-wide`(run W-G5-dyn, W-G8-plan, W-G8-dyn)이고, 보기 설정 키는 `outputs/tensorboard-view.json`의 `zone_wide_20260925`다. 참고용으로 Z3 run도 함께 보인다.
- 실행 중인 서버의 scalar API에서 reported_success, sim_s, model_calls, wall_s가 result.json과 같은지 확인했다. 브라우저 화면은 열기만 했고 표시 값은 직접 확인하지 않았다.
- 창으로 다시 보기: `.venv-sim/bin/python -m scripts.zone_replay outputs/zone-wide-20260925/dev-g8-dyn`

## 실LLM 코호트 ZW1 (소스 `e4ccaf6`)

사전 등록: [protocol.md](protocol.md). 모델 `gemini-3.8-flash`, 동기 SIM, `--record-replay --max-sim-s 900`, 접촉 프로필 `local_contact_fine`, weld OFF, 조건마다 1회. Claude 잠금 안에서 차례로 실행했다(01:01–01:10 UTC, 부하 평균은 `loads.jsonl`, 첫 실행 시작 때 1분 평균 8.33).

| 실행 | 결과(심판) | TOP RGB | 완료 SIM초 | LLM 호출 | 입력/출력 토큰 | 충돌 | 무효 선언 | 협상 턴 | 교사 양보 | wall초 |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ZW1-G5-plan | **5/5** | 일치 | 155.0 | 9 | 58,327 / 1,830 | 0 | 0 | 3 | 0 | 88.6 |
| ZW1-G5-dyn | **5/5** | 일치 | 183.0 | 17 | 110,988 / 1,680 | 2 | 1 | — | 1 | 152.8 |
| ZW1-G8-plan | **6/6** | 일치 | 153.0 | 10 | 66,023 / 2,337 | 0 | 0 | 3 | 0 | 141.1 |
| ZW1-G8-dyn | **6/6** | 일치 | 155.0 | 14 | 93,010 / 1,405 | 1 | 0 | — | 0 | 157.6 |

- 모든 작업이 `placed_by_teacher`로 끝났다. 교사 막힘과 멈춘 작업은 0건이다.
- 배분: G5-plan r1(red-2, red-3 → A), r2(green-1, red-1 → C), r3(cyan-1 → B). G5-dyn r1(red-2, red-1 → A), r2(cyan-1 → B, red-3 → C), r3(green-1 → C). G8-plan 로봇마다 한 구역 2개씩. G8-dyn은 로봇마다 서로 다른 구역 두 곳에 1개씩.
- Z3(`zone_open`)에서는 dynamic의 호출과 토큰이 plan_first보다 적었다. ZW1에서는 반대로 plan_first가 적다(3턴 만에 합의). 지도와 입력 이미지 수(3장 → 5장)가 달라 두 코호트를 합치거나 원인을 단정하지 않는다. 요청당 입력 토큰은 약 6.5천(plan_first)과 6.6천(dynamic)이다.
- G5-dyn의 183.0 SIM초는 r1의 두 번째 작업(red-1, 84.8초 배정 → 183.8초 완료)이 끝을 정했다. r1이 상자까지 가는 데 46초가 걸렸고, 그 사이 r3가 r2에게 15.2초 동안 길을 비켰다.

### 찾은 결함: 호스트가 내려놓은 상자를 두 번 센다 (ZW1-G5-dyn, 3턴)

- 81.8초에 r3가 "red-1 → A"를 선언했는데 호스트가 `zone A needs no more red`로 거절했다. r3는 다음 시도에서 null(대기)을 골랐고, 3초 뒤 r1이 같은 상자를 선언해 가져갔다.
- 원인: r1은 75.2초에 red-2를 A에 내려놓고 물러나는 중이었다(84.4초에 끝남). TOP RGB에는 A의 빨강 1개로 보였고, r1의 작업은 아직 진행 중이라 `remaining_need`가 같은 상자를 진행 중 선언으로도 셌다. 필요 2 − 보임 1 − 진행 중 1 = 0.
- 저장된 TOP 네 장과 기록된 진행 중 작업으로 계산을 오프라인 재현해 같은 결과(남은 필요분 없음)를 얻었다.
- 영향: 이 실행의 심판 결과는 바뀌지 않았다. 올바른 선언 하나가 거절됐고 red-1을 누가 옮길지가 바뀌었다. `zone_open`에도 같은 코드가 있어 내려놓기와 다음 선언이 겹치면 생길 수 있다. Z 코호트 기록에서 같은 사례가 있었는지는 다시 확인하지 않았다.
- 수정하지 않았다. 사전 등록에 따라 고친 소스로 새 코호트(ZW2)를 따로 등록해야 한다.

### ZW1 원본 해시 (`outputs/zone-wide-20260925/`, 로컬만)

| 실행 | result.json | teacher-events.json | team/team.json |
|---|---|---|---|
| ZW1-G5-plan | `55c62c22ef736c23eae2692b872ffdf28b86fcf09bcab0a5ce91b7c37eea4380` | `409e8f239582d3392a0448dc06e6c3b5e226ff1856b921e70ba517eaccae9c96` | `8050f3d07db2ba37834382669db37b8a208fd761b4f87be118ef6b27b796afc2` |
| ZW1-G5-dyn | `5a0a2c1393291a16de890f08240156a333984f1d3eaf212d865c374b2b6c6b28` | `0f4f88f7fcceb87b75363478b835a2efd36929dd20c5dbaabd4a1e3c5b61c39b` | `38276e2a57e45a88ffc7d1dd66507036232edb63636445a8e37802a71a96eef3` |
| ZW1-G8-plan | `84dfecc90750cdcad673eb4f962816932423d0de7e8710ca7d6f10f47682aa87` | `d65d67d656156a08d581163c20e488f05abd54f7cfafd1232e9ecc7fa5d4e7df` | `ea7b727425e1101d6e0ada03c570a26e6507e21e811698bd1c5ef358f4268da1` |
| ZW1-G8-dyn | `c1f0602bae982e47ba07ec1bd1e5b1ca38a3e33f70d331baa853d09eb704a7b8` | `e7f4f3603762e57eb915e0e234479076d5bb87d3784c20dffb4de079934e1409` | `a197dcaff5f90b59fa1238ea0e1860f6b73adeb2cc4b3fbf1bc3540486d803a7` |

TensorBoard 스냅샷은 `outputs/tensorboard/0925-zone-wide-zw1`이고, 보기 키 `zone_wide_20260925`가 ZW1, W-*, 참고용 Z3를 함께 보여 준다. 서버 scalar API로 값을 대조했다.

## 남은 일

- 두 번 세기 결함 수정과 ZW2 사전 등록.
- 조건마다 1회라 plan_first와 dynamic의 차이는 사례로만 읽는다.
