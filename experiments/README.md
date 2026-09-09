# 실험 인덱스

| ID | 코드 연결 | 범위 | 결과 |
|---|---|---|---|
| [2026-09-09-markerless-n7](2026-09-09-markerless-n7/README.md) | [커밋](2026-09-09-markerless-n7/code-version.json), [소스 해시](2026-09-09-markerless-n7/source-manifest.json) | 로봇1대, 표식 없음, Gemini, 시드42~46 | 5/5; 효율 개선 필요 |

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
