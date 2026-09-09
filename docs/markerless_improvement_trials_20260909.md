# 표식 없는 상자 운반 개선 결과 — 2026-09-09

기존 M4의 2/5에서 최종 N7 고정 소스의 **5/5 성공**으로 개선했다. 같은 다섯 조건을 새로 실행한 결과이며, 서로 다른 후보의 성공을 합산하지 않았다. 요청은 “다 성공되게” 만드는 시행착오식 보완이었다.

## 최종 실행

| 시드 | 결과 | 모델 호출 | 입력 토큰 | SIM초 | 근거 |
|---|---|---:|---:|---:|---|
| 42 | 성공 | 18 | 82,023 | 195.950 | [영상](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-42/motion-1x.mp4) · [검증](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-42/verification.json) |
| 43 | 성공 | 16 | 73,175 | 129.754 | [영상](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-43/motion-1x.mp4) · [검증](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-43/verification.json) |
| 44 | 성공 | 15 | 68,011 | 177.836 | [영상](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-44/motion-1x.mp4) · [검증](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-44/verification.json) |
| 45 | 성공 | 21 | 98,676 | 217.378 | [영상](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-45/motion-1x.mp4) · [검증](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-45/verification.json) |
| 46 | 성공 | 18 | 83,325 | 138.534 | [영상](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-46/motion-1x.mp4) · [검증](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7/solo-46/verification.json) |

모두 접근→집기→모델 판단에 따른 장애물 우회·운반→목적지 안 방출→시각 확인→완료까지 수행했다. 상자 들어올리기, 구역 내부 배치, 정지, 부착 제약 없음, 시각 방출 확인, 호출·입력·시간 예산 검사를 모두 통과했다.

## 원인과 적용한 변경

| 관측된 문제 | 최종 변경 | 확인 근거 |
|---|---|---|
| 목적지 경계 부근에서 반복 조정하다 입력 예산 소진 | 현재 자신의 두 RGB와 PWM에 묶인 안전 내부 진입 방향·거리 안내, 모델 지시문의 중복 정리 | 실제 호출 기록 및 배치 안내 회귀검사 |
| 파란색·녹색 바닥이 상자 마스크에 합쳐져 운반 중 파지를 놓쳤다고 오판 | 상자와 배경의 색 범위를 분리 | 동일 저장 영상의 배경 제외, 실제 이탈 반례, 제한된 운반 자극 진단 |
| 방출 후 바닥 상자 위치의 영상 추정 편향 | 기존 형상 후보 주변의 국소 정밀 탐색 | N5 실패 시점의 추정 차이 11.67mm→6.15mm; 실제 RGB/PWM fixture |
| 밝은 앞면이 윗면에 붙어 오른쪽 확인 시점에서 상자 미검출 | 고정된 밝기 후보 4개, 위치·투영 겹침으로 동일 물체 후보 중복 제거, 기존 유효 추정 보존 | N6 실패 세 시점 모두 인식, 최대 XY차 1.18mm; 두 상자 모호성·공중 상자 거절 검사 |

이번 작업의 런타임 수정은 `visual_placement.py`, `transport_context.py`, `gemini_transport_policy.py`, `visual_attachment.py`, `markerless_box.py`의 다섯 파일이다. 마찰·중력·부착 제약 등 물리 설정과 기존 방출 정지성 10mm 기준을 유지했다. 색 분할과 형상 탐색 자체는 변경했으므로 “관측 알고리즘 불변”을 의미하지 않는다.

## 시행착오 기록

| 후보 | 전체 묶음 결과 | 남은 문제 또는 판단 |
|---|---|---|
| M4 기준 | 2/5 | 예산 종료 및 운반 중 파지 확인 실패 |
| N1 | 44번 한 조건 성공 | 진입 안내 단독 확인; 전체 성공률로 사용하지 않음 |
| N2 | 전체 실험 전 폐기 | 넓힌 색 범위가 배경을 포함함을 영상 QA에서 발견 |
| N3 | 4/5 | 45번 방출 후 시각 확인 실패 |
| N4 | 3/5 | 45·46번 입력 예산 종료 |
| N5 | 3/5 | 45번 위치 추정 불일치, 46번 방출 시점 미검출 |
| N6 | 4/5 | 45번 오른쪽 방출 확인 시점 미검출 |
| N7 최종 | **5/5** | 이번 다섯 조건에서 모든 완료 기준 통과 |

진단 재생·중단된 진단은 위 운반 성공률에 포함하지 않았다. 특히 N6의 방출 재생 두 건은 기록 전이 누락으로 방출 전에 중단됐으며, 보완한 재생 실행기의 전체 재현성은 후속 실제 실행으로 검증하지 않았다. 최종 성공 근거는 N7의 새 모델 호출을 사용하는 전체 운반 실행이다.

## 검증과 범위

- 모델 Gemini 3.8 Flash, reasoning medium, 로봇 1대, 통신 없음. 조건당 최대 30호출·120,000입력토큰·300SIM초, 요청 timeout 60초. 순서 45→46→42→43→44.
- 관련 테스트 **139개 + 세부 검사 77개 통과**. 원래 실패 프레임은 통합 전 실제 API에서 실패, 통합 후 통과하는 것을 확인했다.
- 런타임·실행기 **125개 파일**의 시작/종료 해시 일치. N7 실행 중 조정·재시도 없음.
- 모든 모델 NAV 입력 88장, 각 영상의 시간순 표본 12장, 각 최종 손목 영상을 직접 검토했다. 전체 6,919프레임을 디코딩했으며 영상 전체의 연속 수동 시청을 뜻하지 않는다.
- 제어에는 자신의 RGB/PWM을 사용한다. 평가 전용 좌표는 원인 분석·결과 확인에만 사용했다. 진단용 고정 동작 재생을 모델의 자율 운반 성공으로 세지 않았다.
- 소규모로 반복 튜닝한 다섯 시드의 결과다. 미관측 시드, 다른 상자색·조명, 실제 MasterPi 하드웨어와 다중 로봇 협업의 일반 성공률을 보장하지 않는다. 모델 응답과 지연은 확률적이므로 후보 간 단일 실행 차이를 한 변경의 단독 인과 효과로 해석하지 않는다.

## 저장 자료

- [전체 시행착오 원기록](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/trial-notes-detailed.md), [각 실제 실행 결과](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/trial-outcomes.json)
- [N7 고정 소스 목록](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7-source-manifest.json), [종료 후 일치 확인](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/n7-source-verification.json)
- [작업 소스·테스트·영상 fixture 묶음](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/candidate-source.zip), [M4 기준 변경 패치](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/candidate-from-m4.patch)
- 패치는 저장된 M4 작업 기준에 대한 변경이며 깨끗한 Git HEAD 기준 패치가 아니다. 기존의 다른 미커밋 변경은 보존했다. 영상·대용량 실행 로그는 각 로컬 실행 폴더에 별도로 남겼다.
- 프로젝트 지침에 따라 로컬에 저장했다. 실험 프로세스 정리 결과는 [정리 기록](/Users/changmin/projects/ugrp/outputs/markerless-improvement-20260909/process-cleanup.json)에 기록한다.
