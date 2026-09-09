# 카메라 운반 예산 수정 — Mac 적용 및 실제 시험, 2026-09-07

## 결과

클라우드 수정 후보를 Mac 현재 소스와 대조하여 적용하고 실제 Gemini 단독 시험을 한 번 수행했다. **입력 예산 초과는 막았지만 배송은 실패했다.** 실제 실행에서 발견한 실행 피드백 연결 오류를 추가 수정했다. 이 마지막 수정 이후 실제 Gemini 재시험은 하지 않았으므로 동일 예산 완주를 주장하지 않는다.

## 적용과 검증

- 대상 코드·테스트 11개 파일이 ZIP 변경 목록의 원본 SHA-256과 모두 일치했다. `harness/`, `scripts/`, `tests/` 패치만 적용하고 원래 결정 로그·사용자 `.gitignore` 변경은 보존했다. `sim/` 물리·접촉·목적지 모델은 변경하지 않았다.
- 원본 실패 자료는 삭제된 것이 아니라 `/Users/changmin/Project-Archives/20260907/ugrp/outputs/warehouse_research`로 보관돼 있었다. 필요한 4개 기록 폴더를 기존 검사 경로에서 읽도록 심볼릭 링크를 추가했다. 원본을 옮기거나 수정하지 않았다. 목록은 `outputs/camera-budget-integration-20260907/restored-reference-links.json`.
- Mac Python 3.12.13, MuJoCo 3.12.0 환경을 사용했다. 이 환경에 없던 pytest와 pytest-subtests를 설치했다.
- 원본 자료 연결 및 첫 검증기 보완 후 관련 검사 **122 passed, 29 subtests passed**, 실패·skip 없음. 저장된 파일 `final-focused-tests.txt` 참조.
- 실제 시험 후 피드백 연결 및 검증기 보완 관련 검사 **85 passed, 23 subtests passed**. 이는 다른 검사 범위이므로 앞의 122개와 합산하지 않는다. `post-live-fix-tests.txt` 참조.

## 고정 예산 시험 한 번

실행 위치: `outputs/warehouse_research/camera-budget-repair-dev-01/solo-41`.

| 조건·결과 | 값 |
|---|---|
| 모델 / 로봇 / seed | 실제 응답 Gemini 3.8 Flash / R1 한 대 / 41 |
| 한도 | 30회 호출, 입력 60,000토큰, 능동 실행 300 SIM초 |
| 물리 조건 | impratio 10, noslip iterations 3, 기존 카메라와 안전 검사 유지 |
| 호출 / 실제 입력 | 14회 / 56,321토큰 |
| 비용 정산 | 누락 usage 0, 미정산 예약 0, 모든 요청·응답 대응 및 입력 RGB 해시 일치 |
| 능동 시간 / 종료 후 정착 | 170.828초 / 1.000초 |
| 결과 | `LLM_INPUT_TOKEN_BUDGET_PREFLIGHT`, 목적지 진입·놓기 미완료 |
| 집기 | 약 6.94cm 상승, 영상에서 들고 있는 상태 확인 |
| 영상 | 1,375프레임 전체 디코딩; 시간순 8개 영상 장면 및 14개 원본 판단 카메라 검토 |

가용성 확인을 위한 별도 completion이나 자동 재시행은 하지 않았다. 첫 실제 판단부터 한도에 포함했다. 429는 이번 실행에서 관찰되지 않았다. 한도가 항상 보장되는 것은 아니며 다음 요청을 추정 예약하는 방식이다. 이번 실행에서는 실제 보고 입력량이 한도 안에 있었다.

## 실행에서 확인한 문제와 추가 수정

### 실행 결과가 모델 입력으로 전달되지 않음

`VisualMacroExecutor._emit()`은 `execution`을 이벤트 최상위에 넣었지만 runner 콜백은 `details.execution`에서 찾고 있었다. 실제 명령 로그에는 완료 매크로 결과 **236건**이 있었는데, 실제 모델 요청 **14건 모두 `last_macro`가 없었다**.

runner의 `record_command()`를 통해 최상위 `execution`을 읽고 판단 ID와 함께 다음 모델 입력에 전달하도록 수정했다. 실제 `VisualMacroExecutor` 이벤트 → runner 콜백 → planner의 전달 context까지 연결한 통합 검사는 오프라인 응답 대역으로 수행했으며 외부 모델 호출은 없다. 이는 연결 수정의 증거이지 운반 성공의 증거가 아니다.

검증기도 명령 기록의 마지막 완료/중단 결과와 요청·실제 전달 context의 `last_macro`를 대조하도록 보강했다. 보강한 검증기로 원래 시험을 다시 읽으면 `execution_results_reach_model_context=false`로 정확히 거부한다. 원래 시험의 소스 manifest와 원래 검증 결과는 덮어쓰지 않았다. 재검사 결과는 별도 `post-fix-recheck-of-original-pilot.json`에 있다.

### 성공 행동 순서 검사

기존 검증기는 집기 전에 주행한 기록만 있어도 운반 주행 조건을 만족할 수 있었다. `approach → pick → drive → release → finish`의 순서가 존재해야 통과하도록 수정하고 반례 검사를 추가했다.

### 남은 실제 운반 병목

- 3~11번 판단 **9회가 제자리 회전**이었다. 입력 **36,981토큰**, 실제 적용된 회전 명령 시간 합계 13.5초를 사용했다. 화면은 변했으므로 이를 픽셀 정체로 판정하지 않은 사실만으로 정체 검사의 결함이라고 할 수 없다.
- 일부 판단은 목적지가 화면 오른쪽이라고 설명하면서 좌회전을 선택했다. 이후 방향을 바꿨다. 카메라 장면과 실제 모델 원문을 보존했다.
- 12~13번에 2.5초·3초 전진을 선택하고 실행했다. 14번 전진 제안은 `ORANGE_OBSTACLE_IN_FORWARD_FOOTPRINT`로 거부됐다. 다음 입력 요청을 예약할 여유가 없어 중단했다. 안전 검사를 완화하지 않았다.
- 실행 피드백 누락은 확정된 코드 결함이지만, 이를 고치면 반복 회전·배송 실패가 해결된다는 인과관계는 아직 입증되지 않았다.

## 이어서 할 검증

현재 소스는 실제 시험 후 runner와 검증기 2개 파일이 추가 수정된 상태다. 이전 시험을 현재 소스의 성공·실패율로 바꾸지 않는다. 다음 실제 시험은 새 출력 디렉터리에서 동일 모델·동일 예산으로 별도로 수행해야 한다. 먼저 실제 요청에 `last_macro`가 도달하는지 확인하고, 그 뒤 회전 반복·우회 판단과 최종 놓기까지 검토한다. 예산 증액이나 팀·멀티시드 확대는 하지 않았다.

자료:

- 실행 영상: `../outputs/warehouse_research/camera-budget-repair-dev-01/solo-41/motion-1x.mp4`
- 수치·판단별 실행 분석: `../outputs/camera-budget-integration-20260907/live-analysis.json`
- 원본 판단 카메라 모음: `../outputs/camera-budget-integration-20260907/pilot-decision-cameras.jpg`
- 시간순 영상 장면: `../outputs/camera-budget-integration-20260907/pilot-motion-qa.jpg`
- 기존 실패 장면: `../outputs/camera-budget-integration-20260907/baseline-contact.jpg`
