# UGRP — LLM 기반 로봇 협력 이송 연구

각 로봇의 관측과 자연어 통신으로 역할 분담·협력 이송을 검증하는 연구 프로젝트다. 현재 구현은 MasterPi와 MuJoCo 시뮬레이션, 카메라 관측 기반 제어 하네스로 구성된다.

## 현재 검증된 범위

**2026-09-09: 표식 없는 작은 상자 단독 운반, 시드42~46에서 5/5 성공.** 현재 RGB/PWM을 사용하는 실제 Gemini 판단으로 접근·집기·우회·운반·방출을 수행했다. 소스 버전, 예산, 결과는 [N7 검증 기준](experiments/2026-09-09-markerless-n7/README.md)에 연결돼 있다.

이 결과는 로봇1대·통신 없음 조건이다. 다중 로봇 협업, 새 시드의 일반 성능, 실물 MasterPi 성공을 뜻하지 않는다. 접근 보정 반복과 불필요한 이동은 아직 개선할 문제다. [진행 이슈](https://github.com/kcm0127-dotcom/ugrp/issues)에서 추적한다.

이후 별도 실행 경로에서 검증한 기능은 다음과 같다. 아래 학생·동기화·지도 주행은 LLM 판단을 사용하지 않으며, 위 Gemini 운반 실행기에 모두 연결된 상태를 뜻하지 않는다.

| 기능 | 기존 비교 실험 결과 | 검증 범위 |
|---|---|---|
| [RGB 국소 파지 복구](experiments/2026-09-10-grasp-recovery/report.md) | 새 팔 오차 20/20, 기존 조건 14/14 | 고정 장면; 접근·닫기·들기는 시연 명령 |
| [직진 접근](experiments/2026-09-10-rgb-short-approach/report.md) | 19/20, 고정 시간 주행 2/20 | 출발 거리 20–30cm |
| [다양한 시작 자세 접근](experiments/2026-09-10-rgb-varied-start/README.md) | 새 조건 29/30, 기존 조건 20/20 | 고정 장면, 거리 15–40cm·옆 오차 ±6cm·방향 ±10° |
| [20cm 공동 운반과 내려놓기](experiments/2026-09-13-rgb-short-transport/README.md) | 고정 조건 10/10, 다양한 시작 6/10 | 내려놓기는 시연 명령; 일부 운반 영상은 학습 범위 밖 |
| [두 로봇 운반 동기화](experiments/2026-09-13-pair-carry-sync/README.md) | 비교군 20/26 → 동기화 26/26 | 고정 fixture의 지연·보고 누락 비교; 일반 협업 증명 아님 |
| [방향을 돌린 뒤 지도 주행](experiments/2026-09-13-heading-map-navigation/README.md) | 통행 가능 4/4 도착, 좁은 통로 2/2 진입 거부 | 무부하 로봇1대; 명시적으로 제공한 정적 지도 사용 |

학생은 자기 RGB·공용 top RGB와 자기 발행 명령을 사용한다. 교사 정답은 학습 데이터 생성에만 허용되며, 실행 중 평가 좌표·접촉·관절 측정으로 행동을 보정하지 않는다. 정적 지도는 승인된 지도 주행 경로의 선택적 입력이다. 실제 카메라 배치/FOV와 weld OFF를 유지한다.

새 팀원은 **[Ubuntu 24.04 시작 안내](docs/ubuntu_quickstart.md)**에서 설치 → 무료 이동·카메라 데모 → 테스트 → PR 순서로 시작한다. 기존 연구 실험은 Mac 로컬 환경에서 검증했다. 예전 클라우드 GPU 실행 환경은 [퇴역 기록](docs/cloud_simulation.md)으로만 남긴다.

## 시작하기

- [각자 PC에서 Gemini 로그인 프록시 설치](docs/gemini_subscription_proxy.md)

- **[팀원용 Ubuntu 설치·시뮬레이션·PR 안내](docs/ubuntu_quickstart.md)** — 모델 계정 없이 시작 가능

- [설치·테스트·실험 재현과 PR 절차](CONTRIBUTING.md)
- [실험 목록과 증거 관리](experiments/README.md)
- [연구 비교 조건과 관측 경계](docs/warehouse_research_contract.md)
- [현재 아키텍처 상세 기록](docs/current_architecture_todo.md) — 각 항목의 기록 날짜를 확인한다.
- [실물 로봇 trace 진단](docs/real_trace_system.md)
- [결정 이력](docs/decision_log.md)

## 코드 구성

| 경로 | 역할 |
|---|---|
| `harness/` | 모델 입력, 행동 실행, 카메라 기반 인식과 판정 |
| `sim/` | MuJoCo 환경과 평가 |
| `scripts/` | 실행·기록·검증·프로세스 정리 |
| `tests/` | 자동 회귀검사와 포터블 영상 fixture |
| `experiments/` | 커밋에 연결한 실험 설정·결과·원본 식별값 |
| `docs/` | 설계·연구 계약·진단 기록 |

## 저장소 운영

변경은 `codex/…` 또는 목적이 드러나는 작업 브랜치에서 수행하고 PR로 검토·자동 테스트 후 `main`에 반영한다. 실험 시작 전에 코드 커밋을 고정하고, 결과를 해당 SHA에 연결한다. 전체 실험 도중 수정한 결과를 하나의 성공률로 합산하지 않는다.

Git에는 코드·설정 예시·회귀 fixture·요약 결과와 일부 실험의 압축 모델·RGB 감사 자료를 저장한다. 각 실험의 manifest와 복원 안내에서 실제 포함 범위를 확인한다. 인증정보·가상환경과 대부분의 raw 영상·대량 로그는 제외한다. 로컬에만 있는 raw 자료를 GitHub 백업으로 표현하지 않는다. UGRP는 Google Drive를 사용하지 않는다.

[초기 연구 브리프](docs/research_brief_20260813.md)는 과거 배경 자료로 보존한다. 위 최신 검증 범위와 구분한다.
