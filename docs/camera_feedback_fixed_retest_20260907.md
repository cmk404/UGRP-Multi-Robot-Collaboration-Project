# 실행 피드백 수정 후 동일 예산 재시험 — 2026-09-07

## 결과

실제 Gemini 3.8 Flash 단일 로봇 시험을 한 번 수행했다. **직전 실행 결과의 모델 입력 전달은 검증됐으나, 운반 완주는 실패했다.** 실패를 성공으로 재분류하거나 예산을 늘려 재시도하지 않았다.

## 조건과 검증

- Mac Python 3.12.13, 기존 MuJoCo 환경. seed 41, R1 한 대, 통신 없음, impratio 10, noslip iterations 3.
- 최대 30회 호출, 입력 60,000토큰, 능동 실행 300 SIM초. 첫 실제 판단부터 예산에 포함했다.
- 실행: `outputs/warehouse_research/camera-feedback-fixed-20260907/solo-41`.
- 소스 SHA-256: `6ae7efdebb983da2ed73d6b1befb8c78ccc2114e2bcbea961d45e9acd119fb21`.
- 이전 실행의 소스 manifest와 비교하면 `scripts/evaluate_gemini_team.py`, `scripts/verify_gemini_budget_run.py`만 달라졌다. 물리·프롬프트·예산 조건은 유지했다.
- 실행 명령: `.venv-sim-worker-mac/bin/python -m scripts.run_gemini_budget_pilot --execute --output outputs/warehouse_research/camera-feedback-fixed-20260907/solo-41`.
- 요청·응답 13쌍, 입력 56,462토큰. usage 누락 0, 미정산 예약 0, 추론 오류 0. 능동 실행 176.130초, 이후 정착 1.000초.
- 입력 카메라 파일과 실제 전송 해시 일치, 고정 예산·소스 manifest·정지 후 명령 검사 통과. 실제 운반·release/finish 검사는 실패하여 검증기 전체 결과는 `NOT_VERIFIED`다.

## 피드백 전달의 직접 증거

첫 판단에는 이전 실행이 없다. 이후 12개 실제 모델 입력의 `execution_feedback.last_macro`가 그 시점의 마지막 명령 기록과 일치한다. 요청 기록뿐 아니라 실제 completer에 전달한 context까지 검증기가 대조했다.

11번 전진 제안은 `ORANGE_OBSTACLE_IN_FORWARD_FOOTPRINT`로 거부됐다. 12번 입력에는 `last_decision.disposition=rejected_guard`와 마지막 완료 매크로가 각각 전달됐다. Gemini는 직전 전진이 차단됐다고 설명하면서 전진 0, 좌회전 1.5초를 선택했다. 이는 피드백을 반영한 실제 행동 변경의 사례다. 이 차단 피드백 경로 자체가 이번 수정으로 새로 생겼다는 뜻은 아니다.

## 남은 실패와 영상 검토

화물은 최대 약 6.95cm 들어 올렸지만 목적지 진입·내려놓기는 하지 못했다. 종료 이유는 `LLM_INPUT_TOKEN_BUDGET_PREFLIGHT`다. 영상 1,418프레임을 모두 디코딩하고 시간순 12개 장면과 13개 원본 NAV 입력을 확인했다. 집기 후 화물을 든 상태에서 회전·전진하다 장애물 앞에서 멈춘다. 동료·장애물 2mm 초과 관통 시간 지표는 모두 0초였다. 이것만으로 모든 접촉 안전을 보증하지 않는다.

| 비교 | 수정 전 | 이번 시험 |
| --- | ---: | ---: |
| 실제 호출 | 14 | 13 |
| 입력 토큰 | 56,321 | 56,462 |
| 제자리 회전 판단 | 9 | 7 |
| 회전 판단에 쓴 입력 토큰 | 36,981 | 31,188 |
| 운반 전진 실행 | 2회 | 2회 |
| 전진 제안 안전 거부 | 1회 | 2회 |
| 운반 완주 | 실패 | 실패 |

이번 회전은 3~8번의 연속 6회와 장애물 거부 후 12번 1회다. 회전 판단이 입력의 약 55%를 사용했다. 7번은 파란 목적지가 우측이라고 설명하면서 좌회전을 계속했고, 8번에서 우회전으로 바꿨다. 원본 화면에서도 목적지가 우측으로 이동해 있다. 우회를 의도했을 가능성이 있으므로 이 한 장면만으로 단순 좌우 반전 버그를 확정하지 않는다.

12번에서 거부에 반응했지만 13번 전진도 같은 안전 검사에 막혔다. 최종 화물 목표점 오차는 수정 전 약 1.650m, 이번 약 1.651m로 사실상 비슷하다. 이는 사후 심판 값이며 모델 입력에는 제공하지 않았다. 단일 비결정적 모델 시행 비교로 효율 개선의 인과관계나 성공률을 주장할 수 없다.

## 다음 수정의 초점

실행 결과 누락은 해결됐으므로 다음은 **현재 카메라에서 회전이 정렬에 도움이 됐는지, 차단 후 실제 통로가 열렸는지 판단하는 근거**를 개선하는 것이다. 특히 6~8번 회전과 11~13번 우회 구간을 저장된 own-RGB로 재생해 평가한다. 전역 좌표로 경로를 대신 정하거나 안전 검사·목적지 판정을 완화하지 않는다. 이 시험에서는 탐색용 코드 변경이나 추가 유료 호출을 하지 않았다.

## 증거

- 원본 실행: 위 실행 폴더의 `llm-decisions.jsonl`, `commands.jsonl`, `inputs/`, `budget-verification.json`, `motion-1x.mp4`.
- 검토 자료: `outputs/camera-feedback-fixed-review-20260907/decisions.json`, `visual-qa.json`, `decision-cameras.jpg`, `motion-contact.jpg`.
- Drive: https://drive.google.com/drive/folders/1VZDIozd1CLoWUSugmApFO6dHzIdpPzFE . 파일별 업로드·재조회 결과는 로컬 upload receipt에 보관한다.
