# UGRP — LLM 기반 로봇 협력 이송 연구

각 로봇의 카메라 관측과 자연어 통신으로 역할 분담·협력 이송을 검증한다. MasterPi와 MuJoCo 시뮬레이션, RGB 기반 제어 하네스로 구성된다.

## 시작하기

- **[Kaggle CLI 배치 실행](docs/kaggle_simulation.md)** — 비공개 CPU·오프라인 실행·결과 회수 검증 완료; 신규 계정은 최초 인증 필요
- **[Colab CLI 시뮬레이션·평가](docs/colab_simulation.md)** — 2026-09-21 요청에 따른 기본 실행 대상; 실제 런타임 검증 상태는 안내 참조

- **[현재 상태와 실행 경로](docs/current_status.md)** — main에 포함된 결과·제약과 작업별 진입점
- **[Ubuntu 설치·무료 데모](docs/ubuntu_quickstart.md)** — 새 팀원은 여기서 시작
- [개발·테스트·실험·PR 절차](CONTRIBUTING.md) · [로봇 입력과 작업 규칙](AGENTS.md)
- [TensorBoard로 학습·실험 기록 보기](docs/tensorboard.md)
- [문서 찾아보기](docs/README.md) · [실험 인덱스](experiments/README.md) · [지도 목록](maps/README.md)

## 현재 검증 범위

2026-09-20 main에 포함된 기록 기준이다. 실험별 실행 SHA와 조건이 다르며 아래 결과를 현재 main에서 새로 실행한 결과로 해석하지 않는다.

| 경로 | 확인한 결과 | 범위와 한계 |
|---|---|---|
| [3대 공동 출하·복구](experiments/dispatch-adaptive-recovery-20260917/README.md) | 목적지·장애물 변화 6조건에서 새 LLM 합의부터 운반·방출까지 성공 | seed11·조건당 1회·기존 RGB 스킬·수치 접촉 프로필; 임의 배치/역할·실시간 분산·실물 일반화 아님 |
| [ACT 공동 운반 비교](experiments/2026-09-18-act-pair-carry/README.md) | 운반 진입 3조건에서 교사 3/3, ACT seed18 1/3·seed19 0/3 | 네 번째 조건은 접근 중단; 확정 계획 재생, 새 LLM 호출 없음. ACT는 기존 경로를 대체하지 않음 |

자기 RGB·공용 top RGB·자기 발행 명령을 사용하고 승인된 지도 경로에서는 정적 지도를 제공한다. 교사 정답은 학습용이며 실행 중 평가 좌표·접촉·관절 측정으로 행동을 보정하지 않는다. 카메라 배치/FOV와 weld OFF를 유지한다. 시뮬레이션 결과는 실물 MasterPi 검증과 구분한다.

이전 단독 운반·접근·동기화·지도 주행 결과는 [과거 검증 요약](docs/archive/validation_summary_20260917.md)에 보존했다.

## 코드 구성

| 경로 | 역할 |
|---|---|
| `harness/` | 모델 입력·계획·행동 실행·RGB 인식 |
| `sim/` | MuJoCo 환경과 별도 평가 |
| `scripts/` | 실행·검증·기록·프로세스 관리 |
| `tests/` | 자동 회귀검사와 재현 fixture |
| `maps/` | 정적 지도와 지형 목록 |
| `experiments/` | 실행 SHA에 연결한 설정·전체 결과·원본 식별값 |
| `docs/` | 현재 안내와 날짜별 설계·진단 기록 |

## 저장과 변경

변경은 작업 브랜치와 PR로 남기고 사용자 승인 뒤 main에 반영한다. 실행 코드는 실험 전에 커밋하며 실패도 보존한다. Git에는 소스·설정 예시·fixture·요약 결과와 일부 압축 모델/감사 자료가 있다. 대부분의 raw 영상·대량 로그·가중치는 로컬 보관이므로 실험 manifest의 실제 포함 범위를 확인한다. 로컬 보관과 해시는 원격 백업이 아니다. 인증정보·가상환경은 커밋하지 않으며 UGRP는 Google Drive를 사용하지 않는다.
