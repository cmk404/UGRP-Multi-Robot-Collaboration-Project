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
- 수정: `9412326`(아래 ZW2).

### ZW1 원본 해시 (`outputs/zone-wide-20260925/`, 로컬만)

| 실행 | result.json | teacher-events.json | team/team.json |
|---|---|---|---|
| ZW1-G5-plan | `55c62c22ef736c23eae2692b872ffdf28b86fcf09bcab0a5ce91b7c37eea4380` | `409e8f239582d3392a0448dc06e6c3b5e226ff1856b921e70ba517eaccae9c96` | `8050f3d07db2ba37834382669db37b8a208fd761b4f87be118ef6b27b796afc2` |
| ZW1-G5-dyn | `5a0a2c1393291a16de890f08240156a333984f1d3eaf212d865c374b2b6c6b28` | `0f4f88f7fcceb87b75363478b835a2efd36929dd20c5dbaabd4a1e3c5b61c39b` | `38276e2a57e45a88ffc7d1dd66507036232edb63636445a8e37802a71a96eef3` |
| ZW1-G8-plan | `84dfecc90750cdcad673eb4f962816932423d0de7e8710ca7d6f10f47682aa87` | `d65d67d656156a08d581163c20e488f05abd54f7cfafd1232e9ecc7fa5d4e7df` | `ea7b727425e1101d6e0ada03c570a26e6507e21e811698bd1c5ef358f4268da1` |
| ZW1-G8-dyn | `c1f0602bae982e47ba07ec1bd1e5b1ca38a3e33f70d331baa853d09eb704a7b8` | `e7f4f3603762e57eb915e0e234479076d5bb87d3784c20dffb4de079934e1409` | `a197dcaff5f90b59fa1238ea0e1860f6b73adeb2cc4b3fbf1bc3540486d803a7` |

TensorBoard 스냅샷은 `outputs/tensorboard/0925-zone-wide-zw1`이고, 보기 키 `zone_wide_20260925`가 ZW1, W-*, 참고용 Z3를 함께 보여 준다. 서버 scalar API로 값을 대조했다.

## 수정과 ZW2 (소스 `f30c9f2`)

수정(`9412326`): dynamic 선언 검사가 "끝냄" 실행기 영수증을 함께 받는다. 끝난 배달 수보다 더 보이는 상자는 진행 중 작업의 상자로 먼저 본다. 영수증이 없는 호출은 예전 계산을 쓴다. 로봇 프롬프트와 plan_first는 바뀌지 않았다. ZW1-G5-dyn 3턴 상황을 회귀 테스트로 고정했고, 전체 CI 2,249개가 통과했다(9개 건너뜀). 사전 등록은 [protocol.md](protocol.md#보완-zw1-뒤-zw2-사전-등록)에 있다.

**시작 조건(fixture, LLM 0회):** ZW2-gate-G5-dyn 5/5 (153.0 SIM초), ZW2-gate-G8-dyn 6/6 (154.0)으로 통과했다. 두 실행의 `teacher-events.json` 해시는 수정 전 W-G5-dyn, W-G8-dyn과 같았다. 규칙 응답 경로에서는 수정 전후의 동작이 같다.

| 실행 | 결과(심판) | TOP RGB | 완료 SIM초 | LLM 호출 | 입력/출력 토큰 | 충돌 | 무효 선언 | 협상 턴 | wall초 |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| ZW2-G5-plan | **5/5** | 일치 | 144.5 | 12 | 78,390 / 2,322 | 0 | 0 | 4 | 94.0 |
| ZW2-G5-dyn | **5/5** | 일치 | 162.5 | 13 | 84,269 / 1,283 | 1 | 0 | — | 113.8 |
| ZW2-G8-plan | **6/6** | 일치 | 158.0 | 12 | 79,940 / 2,562 | 0 | 0 | 4 | 102.2 |
| ZW2-G8-dyn | **6/6** | 일치 | 153.0 | 12 | 79,723 / 1,255 | 0 | 0 | — | 110.4 |

- 01:21–01:31 UTC, Claude 잠금 안에서 실행했다. 부하 평균은 `loads.jsonl`에 있다(1분 평균 1.7–4.1).
- 모든 작업이 `placed_by_teacher`로 끝났다. 교사 막힘, 양보, 멈춘 작업은 0건이다.
- **수정이 실제로 작동했는지:** 저장된 TOP 네 장과 작업 기록으로 dynamic 선언 라운드마다 수정 전후의 남은 필요분을 다시 계산했다. 둘이 다른 라운드는 ZW1-G5-dyn 3턴(81.8초, 두 시도) 하나뿐이다. ZW1-G8-dyn 8라운드와 ZW2 dynamic 14라운드에서는 차이가 없었다. 그래서 ZW2는 수정본에서도 결과가 나빠지지 않았음을 보일 뿐이고, 수정본이 실제 LLM 조건에서 문제 상황을 고쳤다는 증거는 아니다. 그 증거는 회귀 테스트(기록된 상황 재현)뿐이다.
- plan_first와 dynamic: ZW2에서는 호출이 12·13 vs 12·12로 비슷했다. ZW1에서는 plan_first가 적었고(9·10 vs 17·14), Z3에서는 dynamic이 적었다. 코호트마다 방향이 바뀌므로 조건마다 1회로는 차이를 결론 내리지 않는다.

### ZW2 원본 해시 (`outputs/zone-wide-20260925/`, 로컬만)

| 실행 | result.json | teacher-events.json | team/team.json |
|---|---|---|---|
| ZW2-gate-G5-dyn | `c47c9a5d6468a4e9a7c374f0ae5db03b15bbe7e208dd92cb3b44a591c12eb79b` | `8cac057d0949ab9332ec41dccff443daadce6a5a1ce780a7d590842e4c7041ef` | — |
| ZW2-gate-G8-dyn | `c2b60de8fa4f55b2c9693db3640b86d4711020449edf1a4c9b217354dcafc4ba` | `dcfe638620db68cfa35a04ba4b3fbdfaca58cc654e85317f366547277ef25394` | — |
| ZW2-G5-plan | `49e9024530a9741cc5c5c66fcf6eda508146a28a7021156e48fa784605b03e81` | `cb89e8bcf96a5bfd04c00d209977bb4ec65213a94e61e093def0987d641dfb7a` | `b680a77539fdcdc0dcb16ee92758f6b8968e16ecbb18a39e4cfb9645c636c335` |
| ZW2-G5-dyn | `ad77605c42cb4bdbc6f03a5a1b02cfbbad1d340b510ad00d4e7ee5b15361ef6d` | `51777cb31ea2176914644f8c23d1d35e7df14eee2b7a49390e6f4243c269f7ef` | `47df82e936aa2dad7713ce0868cf6ed86693834ad208c5c7fa58fd6ee79413da` |
| ZW2-G8-plan | `c37a7a7c3a3ad1f9f9b3f18f1a752691cbf1899ab249a4642c9f3f1e31e1bbda` | `9e394593084b62d04268b2c8da0765a28c6b4e396936c6e4be883fc6f05d46ef` | `24f1d23688eeed0eb05439d0591c58810d49f3485d3c39cf140828599b710106` |
| ZW2-G8-dyn | `cc6bc8a4a621532908e4572864b181e6dc8907a48039945ad17ea77d29fa70fa` | `dc8cebe30573fdebfe9c5f0ec0c99522b4737bf6e8a3c1ba011215bb2593204c` | `a85344634ecc936219f2b1889e2f3820266e1aa359362ee6b402c5911799760e` |

TensorBoard 스냅샷은 `outputs/tensorboard/0925-zone-wide-zw2`이다. 보기 키 `zone_wide_20260925`가 ZW2, ZW1, W-*, 참고용 Z3를 함께 보여 준다. 서버 scalar API로 값을 대조했다.

## 남은 일

- 조건마다 1회라 plan_first와 dynamic의 차이는 사례로만 읽는다. 차이를 보려면 조건별 반복이 필요하다.
- 두 번 세기 수정은 실제 LLM 실행에서 해당 상황이 다시 생겨야 직접 확인된다.
