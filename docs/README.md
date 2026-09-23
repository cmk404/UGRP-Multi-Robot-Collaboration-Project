# 문서 찾아보기

현재 작업은 [현재 상태와 실행 경로](current_status.md)에서 시작한다. 앞으로 할 일은 [연구 TODO](research_todo.md)를 따른다. 과거 문서를 전부 읽을 필요는 없다.

| 종류 | 문서 | 읽는 시점 |
|---|---|---|
| 표준 시뮬레이션 관리 | [실행·버전·결과 관리](simulation_management.md) | 연구별 workflow 실행과 공통 이력·입력·결과 해시를 확인할 때 |
| 로컬 구성·실행 API | [로컬 시뮬레이션](local_simulation.md) | 설정 파일과 CLI/Python으로 세계를 구성하고 MuJoCo 창에서 확인할 때 |
| 현재 규칙·개발 절차 | [AGENTS](../AGENTS.md), [CONTRIBUTING](../CONTRIBUTING.md) | 작업 시작, 설치·검증·Git 작업 |
| 연구 우선순위·완료 기준 | [연구 TODO](research_todo.md) | 통신 본실험과 실행 기반·보조 과제의 순서 결정 |
| 설치·모델 연결 | [Colab](colab_simulation.md), [Kaggle](kaggle_simulation.md), [Ubuntu](ubuntu_quickstart.md), [Gemini 프록시](gemini_subscription_proxy.md), [학습 모델 다운로드·배포](model_artifacts.md) | 환경 준비·가중치 확보 |
| 출하·협업·동기화 | [출하 환경](research_dispatch_arena.md), [3대 실행](three_robot_e2e.md), [단계 계약](task_stage_sync_contract.md), [단계 연결](task_stage_execution.md) | 관련 구현 작업; 각 문서의 검증 날짜 확인 |
| 연구 설계·학습 | [관측·비교 계약](warehouse_research_contract.md), [레퍼런스 비교](reference_alignment.md), [하네스 설계](research/show_harness_camera_pair_design.md) | 연구 질문·비교 조건 검토 |
| 제어기 검증·결과 판정 | [연구 제어기 검증](research_controller_validation.md) | 회귀 실패 재현, 새 조건 비교, 원본 재검증과 제어기 채택 |
| 실행 버전·이식 검증 | [실행 버전 관리](execution_versioning.md) | 성공 설정 재사용, 실행기 변경, 실제 적용값과 기준의 차이 확인 |
| 실험 증거 | [실험 인덱스](../experiments/README.md) | 관련 ID의 설정·성공/실패·원본 확인 |
| 실물 운영 | [네트워크](masterpi_network_runbook.md), [trace](real_trace_system.md), [물리 보정](masterpi_physics_calibration.md) | 실물 연결 시 현재 장치 상태와 함께 확인 |
| 과거 기록 | [결정 이력](decision_log.md), [옛 아키텍처](current_architecture_todo.md), [이전 검증 요약](archive/validation_summary_20260917.md), [초기 연구 브리프](research_brief_20260813.md) | 변경 이유·과거 실험을 조사할 때 |
| 퇴역 기록 | [클라우드](cloud_simulation.md), [환경 정리](simulation_cleanup_20260909.md) | 이력 확인; 설치·자동 복구 지침으로 사용하지 않음 |

코드 탐색은 해당 모듈·테스트부터 시작하고, 긴 결정 이력은 필요한 날짜·키워드로 좁혀 검색한다. `docs/archive/`와 `outputs/`는 이름만으로 읽기가 차단되지 않는다. `.gitignore`도 접근 제어가 아니므로 탐색 도구에 필요한 경로를 명시한다. 실험 증거를 옮기거나 삭제하기 전에는 코드·fixture·manifest 참조와 로컬 원본의 보존 상태를 확인한다.

저장소 관리 이력: [2026-09-20 브랜치·문서 정리](repository_cleanup_20260920.md).
