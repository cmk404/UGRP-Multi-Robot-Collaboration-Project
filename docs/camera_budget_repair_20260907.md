# 카메라 운반 예산 수정 — 2026-09-07

## 상태

**클라우드 업로드 스냅샷의 실제 코드 수정과 오프라인 테스트를 완료했다. 실제 Mac 소스 반영 및 Gemini 운반 완주 검증은 미실행이다.**

사용자는 Mac에서 실제 Gemini를 사용하여 seed 41, 로봇 1대, 30회 호출·입력 60,000토큰·300 SIM초의 기존 예산으로 운반을 완주하는 검증을 요청했다. 이 세션에서 `mac` 도구를 두 번 조회했지만 해당 namespace에 정의된 도구가 없었다. `mac` 이름과 명시된 플러그인 ID를 대상으로 한 플러그인 검색도 빈 결과였다. 이는 이 세션의 도구 노출 상태이며, 사용자의 플러그인이 삭제되었거나 Mac 자체가 고장났다는 진단이 아니다.

따라서 Mac 파일을 읽거나 수정하지 않았고, Mac의 Gemini 프록시/인증/현재 할당량도 확인하지 않았다. 실제 Gemini completion 호출은 **0회**다. 다른 Mac/Oracle 연결로 우회하거나 계정을 전환하지 않았다.

작업 근거는 제공된 `ugrp-source.zip`이다. ZIP SHA-256:

`3ad9c8ef3e85c685fc38357cad4a791481059f02dbafd4759831db72c6d1576b`

라이브 Mac 체크아웃과 이 ZIP의 현재 일치 여부는 미확인이다. 수정본으로 Mac 폴더 전체를 덮어쓰지 말고, 실제 현재 소스와 변경 내역부터 대조해야 한다.

## 관찰과 재현

기존 `evaluate_gemini_team.py`는 새 모델 행동마다 `NavigationEvents.reset()`을 호출했다. 기존 정체 검사는 같은 anchor로 2초가 지나야 작동했다. 별도의 자체 RGB 이력은 이미 있었지만, 이 실행 중 정체 검사는 짧은 drive마다 초기화되었다.

수정 전에 추가한 합성 회귀 두 개가 실제로 실패했다.

- 질감 있는 동일 RGB에서 1.5초 drive를 반복하면 해당 정체 이벤트가 발생하지 않는다.
- 질감 없는 동일 RGB를 물리적인 정체의 확정 증거처럼 취급한다.

이 재현은 소프트웨어 로직의 검사다. 9월 6일 원본 카메라 영상에서 마지막 운반 실패 원인을 단독으로 입증한 것이 아니다. 원본 영상·상세 추적 자료는 이 업로드 묶음에 없어 해당 장면을 재검토하지 못했다.

## 실제 반영한 코드

| 파일 | 변경 |
|---|---|
| `harness/navigation_events.py` | 전체 reset과 `begin_command()`를 분리. 같은 방향 유형의 짧은 drive 사이에 영상 anchor 유지. 실행기가 제공한 누적 drive 명령 경과시간을 사용하여 추론 대기·미실행 시간을 제외. 저질감은 `PROGRESS_UNOBSERVABLE`로 구분. 입력 예산 사전 예약·누락 사용량 추정 차감·중복 합산 방지. |
| `harness/visual_macro_runtime.py` | 실제 적용된 drive 명령의 유효시간 중 경과한 부분을 계측. 중도 취소·만료된 조각·늦은 tick을 구분. 요청시간과 경과 제어시간, 완료/중단, 원인을 기록. |
| `harness/gemini_transport_policy.py` | 자기 잔여 예산과 명시적 수락/거부 및 실행 결과를 실제 모델 context에 전달. 공간 정답 필드 유입을 검사. 프롬프트를 정리하고 열린 통로·회전·정밀 조정의 상황별 길이 예시를 제공. 검증 전 실패 시 이전 호출 usage audit 재사용 방지. |
| `harness/inference_accounting.py` | 늦게 완료된 응답의 비용을 수집하되 행동은 실행하지 않음. 호출 전 취소된 Future만 예약을 해제. 진행 중 Future 때문에 다른 로봇 루프를 막지 않음. |
| `scripts/evaluate_gemini_team.py` | 위 기능을 실제 runner에 연결. 종료 전 모든 호출 비용 정산. usage 누락 또는 예산 초과는 성공 게이트를 통과하지 못함. 기존 1초 수동 정착 구간과 능동 판단 시간 분리 기록. |
| `scripts/evaluate_gemini_cohort.py` | 모든 조건에 명시적 모델·입력 토큰 상한·요청 추정치를 전달하고 manifest에 기록. |
| `scripts/run_gemini_budget_pilot.py` | 기본 동작은 준비 상태 확인. `--execute`를 명시하면 Mac에서만 고정 예산의 실제 시험 1회를 실행. 출력 덮어쓰기·자동 재시행·계정 전환·프록시 재시작 없음. |
| `scripts/verify_gemini_budget_run.py` | 로그에서 호출별 실제 usage 재합산, 전체 모델 행동 연쇄, 원본 RGB 해시, 모델 응답 메타데이터, 고정 예산, 물리 판정, 영상 처음/끝 디코딩 검사. 자동 검사 통과와 실제 영상 검토 완료를 분리. |

새 회귀 검사는 `tests/test_budget_repair_regressions.py`, `tests/test_gemini_budget_verification.py`에 있다. 기존 `test_navigation_events.py`의 정체 fixture는 질감 있는 화면으로 바꾸고, 무질감 화면의 불확실성은 별도 음성 대조 검사로 추가했다. 원본 자료 누락 테스트를 통과 처리하거나 가짜 이미지로 대체하지 않았다.

### 보존한 경계

두 실제 카메라 입력, 로봇별 메모리·메시지 경계, LLM이 작성하는 속도·회전·지속시간, 기존 파지·배치·주행 안전 검사는 유지했다. `sim/`의 물리 모델·접촉·지도·목적지 형상은 수정하지 않았다. 숨은 위치로 경로를 계산하지 않고 LLM이 고른 1.5초를 4초로 몰래 바꾸지 않는다.

`elapsed_drive_control_s`는 **모터 명령이 실제로 적용된 구간 중 경과시간**이다. 바퀴가 이동했거나 목적지에 가까워졌다는 물리적 증명이 아니다. 기록에는 `motion_confirmed=false`를 둔다.

저질감 판별의 표준편차 2.0은 개발용 출발값이며 원본 영상으로 보정하지 않았다. 기존 시각 충돌 veto를 통과시키거나 완화하는 값이 아니다.

### 예산의 정직한 한계

사전 예약은 `estimated_preflight` 방식이다. 초기 다음 요청 추정치는 6,000토큰이고 이후 실제 최대 관측량에 25% 여유를 적용한다. 정확한 서버 측 입력 토큰 계산을 사용하지 않으므로 다음 한 번의 실제 응답이 추정치를 초과할 수 있다. 이 초과량을 숨기지 않으며 성공 검증은 실패한다.

usage가 없는 응답은 무료로 세지 않는다. 예약 추정치를 비용에서 차감하고 `calls_without_usage`를 증가시킨다. 정확한 예산 내 완료 주장은 금지된다. 최종 정렬·놓기·검증용 12,000토큰은 모델에 주는 권고 여유이며 자동 경로나 완화된 성공 조건이 아니다.

기존 300 SIM초 제한과 종료 후 약 1초의 정지 정착 관측을 유지하고 각각 기록한다. 후자의 시간에는 새 모델 행동을 적용하지 않는다. 보고서에서 두 시간을 합치거나 숨겨 예산 조건을 변경해서는 안 된다.

공통 프롬프트 문자는 2,239자에서 1,922자로 줄었다. **이것은 문자 수 변화일 뿐, 실제 전체 입력 토큰·할당량 절약 또는 완료 호출 수 개선 결과가 아니다.**

## 실제 테스트 결과

| 검사 | 결과 |
|---|---|
| 수정 전 관련 10개 테스트 파일 | 70 passed, 6 failed, 4 skipped. 25 subtests passed는 별도 표기. |
| 수정 전 결함 재현 2개 | 2 failed. 짧은 drive 이력 소실과 무질감 오판 재현. |
| 수정 후 자립 실행 가능한 집중 검사 | 97 passed. 23 subtests passed는 별도 표기. |
| 수정 후 같은 원본 범위 + 신규 회귀 2개 파일 | 111 passed, 6 failed, 4 skipped. 25 subtests passed는 별도 표기. 전체 명령 종료 코드는 1. |
| 실제 Gemini / MuJoCo 폐루프 | 미실행. Gemini 호출 0회. |

남은 6개 실패는 수정 전에도 존재했던 `tests/test_visual_placement_corner.py`의 원본 자료 누락이다. 필요한 `recovery-redelivery-01/inputs/r2/*.jpg`와 `coela-gemini38-verified-01/team-58/llm-decisions.jsonl`이 업로드 ZIP에 없다. 4개 skip도 통과로 합산하지 않는다. 이는 전체 저장소의 모든 테스트를 실행한 결과가 아니라 명시된 관련 검사 묶음이다.

환경: Linux, Python 3.13.5, pytest 9.0.2, NumPy 2.3.5, OpenCV 4.13.0.92, Pillow 12.3.0. MuJoCo는 설치되지 않았다. 이 환경에서 Mac의 Python/렌더러 호환성을 입증하지 않았다.

## Mac에서 이어서 수행해야 하는 실제 검증

1. `/Users/changmin/projects/ugrp`의 현재 소스, Git 상태, 마지막 decision log와 원본 실패 영상 위치를 읽는다. 사용자 변경을 보존하고 실제 체크아웃에 패치를 검토·반영한다. 이 파일과 ZIP은 배포 완료 증거가 아니다.
2. 기존 Mac 시뮬레이션 Python 환경으로 회귀 검사를 실행한다. 누락 원본 자료에 의존하는 검사도 Mac의 실제 데이터로 다시 수행한다.
3. 준비 상태 확인 후 실제 pilot **한 번**을 실행한다. 아래 실행기는 별도 가짜 completion이나 추가 가용성 completion을 보내지 않는다. 첫 실제 판단부터 30회/60,000토큰 예산 안에 포함한다.

```sh
cd /Users/changmin/projects/ugrp
# 저장소 기록에 나온 Mac 가상환경 이름. 실제 존재 여부는 이번 세션에서 미확인.
.venv-sim-worker-mac/bin/python -m scripts.run_gemini_budget_pilot
.venv-sim-worker-mac/bin/python -m scripts.run_gemini_budget_pilot \
  --execute \
  --output outputs/warehouse_research/camera-budget-repair-dev-01/solo-41
```

이미 같은 output이나 로그가 있으면 실행기는 거부한다. 기존 실패를 덮어쓰지 말고 새 시행 디렉터리를 사용해야 한다. `--execute`는 실제 Gemini 할당량을 소비한다. 준비 상태의 TCP 성공은 인증·모델·quota 가용성 성공이 아니다.

4. 실제 `budget-verification.json`, `llm-decisions.jsonl`, `commands.jsonl`, `navigation-events.jsonl`, `evaluation-only.jsonl`, 원본 입력 RGB, `motion-1x.mp4`를 대조한다. `AUTOMATED_CHECKS_PASS_VISUAL_REVIEW_REQUIRED`는 자동 일관성 검사 통과일 뿐 사람이 영상을 확인한 배송 성공 판정이 아니다.
5. 영상·카메라에서 실제 접근→집기→운반→목적지 배치→놓기→종료를 검토한다. 그때에만 같은 예산에서의 해당 한 시행 완료를 보고한다. 한 시행의 성공을 팀 안정성이나 일반 성공률로 바꾸지 않는다.

## 아직 반영하지 않거나 입증하지 못한 것

실제 원본 카메라 실패 원인 확정, 같은 예산의 실제 Gemini 완주, 멀티시드 재현, 3로봇 안정성, 통신 우월성은 미검증이다. 오래된 위험 증거의 전체 TTL/해소 모델, LLM 하위 목표 메모리 확장, 이미지 처리 정밀도/캐싱, 개인 quota용 공유 차단기, 긴급 메시지만 즉시 중단시키는 통신 변경은 이번 패치에 포함하지 않았다. 기존 관련 안전·통신 코드를 임의로 바꾸어 이 범위가 완료된 것처럼 만들지 않았다.

**현재 결론: 검토 가능한 실제 수정 후보와 오프라인 검증 산출물이 있다. 사용자가 요청한 핵심 결과인 실제 Mac/Gemini의 동일 예산 완주는 아직 없다.**
