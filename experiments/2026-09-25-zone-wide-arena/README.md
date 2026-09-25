# 2026-09-25 넓은 구역 경기장 (`zones/zone_wide`) 교사 실행 확인

사용자 요청(2026-09-25): 구역 배송 경기장이 보기에 좁으니 전체를 넓힌다. 기존 `zone_open` v2는 Z1–Z3 재현을 위해 그대로 두고 `zone_wide` v1을 새로 추가했다. 장면 설명은 [docs/zone_dispatch.md](../../docs/zone_dispatch.md#넓은-장면-zoneszone_wide-2026-09-25)에 있다.

**범위:** 규칙 응답(fixture, LLM 0회)으로 교사 실행기가 새 지도에서 끝까지 동작하는지만 확인했다. LLM 협업, RGB 스킬, `zone_open`과의 성능 비교는 하지 않았다.

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

## 남은 일

- `zone_wide`에서 실제 LLM으로 plan_first와 dynamic 코호트를 돌리는 것(사전 등록 필요).
