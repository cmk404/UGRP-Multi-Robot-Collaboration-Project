# 실험 인덱스

현재 코드의 진입점은 [현재 상태](../docs/current_status.md)를 참고한다. 아래는 실행 SHA별 보존 기록이며 과거 실패와 후속 결과를 함께 남긴다. 필요한 ID만 골라 읽는다.

- [2026-09-22 native 시뮬레이션 CLI/API](2026-09-22-native-simulation/README.md) — Mac·Ubuntu 공통 설정/API, native 창·headless·RGB·reset, Linux 종료 오류 수정과 검증 한계.
- [2026-09-22 브라우저 뷰어 검증 이력 (구현 폐기)](2026-09-22-local-simulation-live/README.md) — 네 카메라·수동 이동·일시정지·초기화·종료 통합 검사; 초기 검사 실패 포함, 자율 운반 평가는 아님.
- [2026-09-21 Kaggle CLI CPU 실행·결과 회수](2026-09-21-kaggle-cli-smoke/README.md) — private·인터넷 OFF, 최종 물리·카메라 데모 1/1, 앞선 setup 실패 3회 별도 보존.
- [2026-09-21 Colab CLI CPU 실행·결과 회수](2026-09-21-colab-cli-smoke/README.md) — 실제 원격 물리·카메라 데모 1/1, ZIP 회수와 전체 파일 해시 확인; 자율 운반 검증 아님.

- [2026-09-21 Jev 의미 상태 폐루프 195회](2026-09-21-jev-semantic-motion/README.md): 새 자세 Jev 5/36 → 35/36(원판정 33/36), Gemini 32/36 → 36/36. 시간 반올림 검산과 원판정 모두 보존.
- [2026-09-21 Jev 직접 이동·표현 진단](2026-09-21-jev-direct-motion/README.md): 동일 RGB 접근 9회, 저장 상태 표현 비교 72회, 공개 제어 설계 검토. 후속 폐루프 195회는 위 별도 기록에서 검증.
- [공동 출하 복구: 목적지·장애물 6조건 새 LLM 계획과 물리 E2E 성공](dispatch-adaptive-recovery-20260917/README.md) — 이전 1/6 이후의 최종 비교, 조건당 1회.
- [2026-09-21 Colab ACT 학습·CLI 복구](2026-09-21-colab-carry-training/README.md) — 8개 × 8000 updates, 50파일 해시 및 Mac native 32개 대조 통과.
- [2026-09-21 ACT 입력 해상도·이력 2×2 비교](2026-09-21-carry-input-ablation/README.md) — 8개 Colab 학습 모델 회수·Mac 검증 완료, 고정 36-run 물리 비교 진행 중.
- [2026-09-18 ACT 공동 운반 비교](2026-09-18-act-pair-carry/README.md) — 운반 진입 조건 교사 3/3, ACT 1/3·0/3, 네 번째 조건 접근 중단.

- [2026-09-17 목적지·지형 확대 E2E: 기본 A 성공, B 영상 인식 실패, 4개 지형 실행 거부](dispatch-variation-e2e-20260917/README.md)
- [2026-09-17 공동 출하 스킬 통합: 실제 새 LLM 계획부터 두 화물 방출까지](dispatch-skill-integration-20260917/README.md)

| ID | 코드 연결 | 범위 | 결과 |
|---|---|---|---|
| [three-robot-e2e-20260916](three-robot-e2e-20260916/README.md) | 실행 `d79c97d`, 결과·원본 해시·대표 영상 | 3대 계획 승인, R1/R3 운반·R2 정지 관찰; 준비 지연·보고 단절 | 최종 실제 모델 3/3·연결 fixture 2/2; 첫 협상 실패 보존, 자유로운 역할 분담 아님 |
| [research-e2e-heading-fix-20260916](research-e2e-heading-fix-20260916/README.md) | 실행 `d863e74`, 전체 결과·원본 해시 | 먼 거리 RGB 방향 보정·정지 재정렬·마지막 전 축 복구, 같은 19배치 × 두 조건 | 로컬 19/19·LLM 19/19; 중간 18/19·17/19 실패와 개발 6회 보존 |
| [research-e2e-varied-start-20260916](research-e2e-varied-start-20260916/README.md) | 실행 `fd5d68e`, 전체 결과·원본 해시 | 거리 30–70 cm·yaw ±10°·좌우 ±6 cm의 19배치, 두 조건 38회 | 로컬 15/19·LLM 15/19; 4배치 모두 파지 전 영상 정렬 실패 |
| [research-e2e-local-skills-20260916](research-e2e-local-skills-20260916/README.md) | 실행 `11da613`, 전체 결과·원본 해시 | 고정 역할·평지, RGB 제어·시연 팔, 두 LLM 단계 허가 | 최종 로컬 3/3·LLM 3/3; 첫 코호트 4/6과 개발 실패 포함 |
| [2026-09-09-markerless-n7](2026-09-09-markerless-n7/README.md) | [커밋](2026-09-09-markerless-n7/code-version.json), [소스 해시](2026-09-09-markerless-n7/source-manifest.json) | 로봇1대, 표식 없음, Gemini, 시드42~46 | 5/5; 효율 개선 필요 |
| [2026-09-10-grasp-recovery](2026-09-10-grasp-recovery/report.md) | 보고서의 학습/최종 SHA | RGB 국소 파지 복구 | 새 20/20, 기존 14/14 |
| [2026-09-10-rgb-short-approach](2026-09-10-rgb-short-approach/report.md) | 보고서의 최종 SHA | 20–30cm 직진 접근 후 파지 | 학생 19/20, 고정 주행 2/20 |
| [2026-09-10-rgb-varied-start](2026-09-10-rgb-varied-start/README.md) | 보고서의 최종 SHA | 거리·옆 오차·방향 변동 접근 | 새 29/30, 기존 20/20 |
| [2026-09-13-rgb-short-transport](2026-09-13-rgb-short-transport/README.md) | 보고서의 실행 SHA | 20cm 운반·시연 내려놓기 | 고정 10/10, 다양한 시작 6/10 |
| [2026-09-13-pair-carry-sync](2026-09-13-pair-carry-sync/README.md) | protocol 및 manifest | 고정 fixture의 운반 지연·보고 누락 | 비교군 20/26, 동기화 26/26 |
| [2026-09-13-known-map-navigation](2026-09-13-known-map-navigation/README.md) | 보고서의 실행 SHA | 정적 지도·RGB 무부하 주행 | 통행 가능 4/4 도착, 좁은 통로 2/2 거부 |
| [2026-09-13-heading-map-navigation](2026-09-13-heading-map-navigation/README.md) | 보고서의 실행 SHA | 회전 후 전진과 기존 옆걸음 비교 | 두 방식 모두 4/4 도착·2/2 거부 |
| [2026-09-14-pr-integration](2026-09-14-pr-integration/README.md) | 통합 검증 기록 | PR 9개 조합의 회귀검사 | 595 tests + 154 subtests, 기록 감사 55/55, 새 실행 9/9 예상 일치 |
| [2026-09-16-task-stage-sync](2026-09-16-task-stage-sync/README.md) | 실행 SHA와 원본 JSON | 다섯 단계 동기화·팀원 JSON 계약; 합성 프로토콜 fixture | 9/9 예상 일치, 655 tests + 154 subtests; 물리 검증 아님 |
| [2026-09-15-pair-grasp-retention](2026-09-15-pair-grasp-retention/README.md) | `cdeee5f`, 전체 진단 SHA 기록 | 접촉 수치 처리와 RGB 조기 감지·한 번 재파지 | 제자리·왕복 300초, 지형 5/5·차단 정지 1/1, 재발 시 방출·종료 |

새 실험은 별도 ID 폴더에 코드 SHA·실행 환경·설정·성공과 실패 전부·판정 기준·자동/영상 검토 범위·원본 저장 위치와 식별값을 남긴다. 소스가 달라지면 별도 후보로 구분한다. 실험 결과 파일을 추가한 커밋과 실제 실행 코드의 커밋은 다를 수 있다.

현재 raw 영상·로그는 로컬 보관이다. 이 인덱스와 해시만으로 raw 자료를 내려받거나 완전히 재현할 수는 없다. 이전 상세 기록은 `docs/`에 있고 로컬 경로를 포함할 수 있다.

## 이전 실험과 실패 기록

| 단계 | 기록 | 해석 범위 |
|---|---|---|
| 초기 탐색 C1~D2 | [시행착오](../docs/navigation_trials_20260908.md) | 단일 시드41, 실패와 두 완료 기록 |
| 새 시드 검증 | [시드 검증](../docs/navigation_seed_validation_20260908.md) | 수정 전 조건별 결과 |
| 운반 수정 E 계열 | [수정 과정](../docs/navigation_generalization_repair_20260908.md) | E7 장애 중단과 E7r1 5/5를 구분 |
| 표식 제거 M 계열 | [표식 제거](../docs/markerless_blocks_20260909.md) | M4 2/5, 집기 진단과 운반 구분 |
| 표식 없는 운반 N 계열 | [26회 결과와 대조 기록](2026-09-09-markerless-trials/README.md) · [원인과 변경](../docs/markerless_improvement_trials_20260909.md) | 실패 6회 포함, N2 폐기·진단은 별도 |
| 성공 후 효율 분석 | [분석](../docs/markerless_success_analysis_20260909.md) | 접근 보정 반복과 운반 비용 |

이 표는 기존 보고서의 탐색 경로를 보완한 것으로, 모든 과거 실험의 소스·환경·원본을 현재 형식으로 이관했다는 뜻은 아니다. 이전 후보의 커밋 연결 및 raw 원격 보관은 미완료다. 원본 복구 경로는 [이슈 #3](https://github.com/kcm0127-dotcom/ugrp/issues/3)에서 추적한다.

- [2026-09-21 TensorBoard 기록 열람 검증](2026-09-21-tensorboard-review/README.md): 기존 24개 기록 변환, 이벤트·원본 해시·로컬 화면 확인. 새 로봇 실험 아님.

- [2026-09-22 로컬 연구 장면 구성 검토](2026-09-22-simulation-scenes/README.md): 기존 58개 항목 연결·Mac/Linux reset/RGB/native/기록 검사; 운반 성능 비교 아님.
