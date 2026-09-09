# 문서 인덱스

이 디렉터리는 현재 기준 문서, 설계·연구 명세, 운영 가이드, 날짜별 검증 기록을 함께 보존한다.
문서의 제목만으로 현재 유효성을 판단하지 말고 아래 분류와 문서 안의 기록 날짜를 확인한다.

## 먼저 읽을 문서

| 문서 | 역할 |
|---|---|
| [프로젝트 README](../README.md) | 현재 검증 범위와 저장소 입구 |
| [로드맵](../ROADMAP.md) | 실행 순서와 단계별 완료 기준 |
| [결정 로그](decision_log.md) | 현재 상태·결정·미해결 항목·세션 인계의 기준 문서 |
| [현재 아키텍처와 TODO](current_architecture_todo.md) | 구현 현황 스냅샷. 문서 안의 날짜 확인 필요 |
| [연구 계약](warehouse_research_contract.md) | 비교 조건, 관측 경계, 평가 원칙 |
| [개발·검증 절차](../CONTRIBUTING.md) | 테스트, 실험, PR, 증거 관리 절차 |

## 아키텍처와 설계

- [MasterPi 기술 아키텍처](masterpi_technical_architecture.md)
- [3대 로봇 협력 구조](three_robot_collaboration.md)
- [카메라 기반 팀 실행](camera_team_llm.md)
- [카메라 전용 제어](camera_only_control.md)
- [시각 상자 스킬](visual_box_skill.md)
- [실물 실행 기록기](real_execution_recorder.md)
- [실물 trace·원인 분석 시스템](real_trace_system.md)
- [클라우드 시뮬레이션 구조](cloud_simulation.md)
- [MasterPi 물리 캘리브레이션](masterpi_physics_calibration.md)
- [통신 프로토콜 범위](protocol_scope.md)
- [CoELA식 통신 효과 실험](coela_communication_study.md)

## 연구 명세와 계획

- [L1–L3 태스크 명세](l1_l2_l3_task_spec.md)
- [평가 스펙 초안](eval_spec_draft.md)
- [소프트웨어 태스크 명세](software_tasks.md)
- [창고 대화 영상 명세](warehouse_dialogue_video.md)
- [초기 연구 브리프](research_brief_20260813.md) — 과거 배경 자료
- [실패 진단·자기 수정 제안](PR_failure_self_correction_loop.md) — 제안 문서

## 운영 가이드

- [필요할 때만 프로세스 실행하기](on_demand_processes.md)
- [MasterPi 네트워크 런북](masterpi_network_runbook.md)
- [Raspberry Pi 헤드리스 설정](raspberry_pi_headless_setup_guide.md)
- [Raspberry Pi 화면 공유](raspberry_pi_screen_sharing_guide.md)
- [GitHub 관리 전환 기록](github_management_20260909.md)
- [프로세스 정리 기록](process_cleanup_20260908.md)

## 검증·진단 기록

날짜가 붙은 문서는 당시 코드와 조건에 대한 기록이다. 최신 상태는 결정 로그와
[`experiments/`](../experiments/README.md)의 코드 연결 정보를 우선한다.

- 2026-08-27: [장기 에이전트 비전 벤치마크](agent_benchmark_2026-08-27.md)
- 2026-09-05: [이벤트 복구 검토](event_recovery_review_20260905.md), [창고 연구 경로 검증](warehouse_research_verification_20260905.md)
- 2026-09-07: [카메라 예산 수정](camera_budget_repair_20260907.md), [Mac 적용·실물 시험](camera_budget_repair_mac_20260907.md), [실행 피드백 재시험](camera_feedback_fixed_retest_20260907.md)
- 2026-09-08: [내비게이션 시행착오](navigation_trials_20260908.md), [새 시드 검증](navigation_seed_validation_20260908.md), [일반화 수정](navigation_generalization_repair_20260908.md)
- 2026-09-09: [표식 제거](markerless_blocks_20260909.md), [개선 시험](markerless_improvement_trials_20260909.md), [성공 조건 분석](markerless_success_analysis_20260909.md)
- 누적 진행 기록: [CoELA 수정 진행](coela_fix_progress.md)

## 보관 자료와 벤치마크

- [`archive/`](archive/) — 현재 기준과 충돌할 수 있는 과거 문서
- [`benchmarks/`](benchmarks/) — 벤치마크 명세와 요약 결과

## 새 문서 배치 원칙

- 현재 상태나 결정을 여러 문서에 복제하지 않고 `decision_log.md`에 기록한다.
- 실행 결과는 `experiments/<날짜-실험-id>/`에 코드 SHA·환경·결과와 함께 둔다.
- 반복해서 사용할 절차만 운영 가이드로 남기고, 일회성 조사·수정 결과는 날짜를 붙인다.
- 대용량 영상·로그·모델 가중치와 인증정보는 Git에 추가하지 않는다.
- 과거 문서를 폐기하지 말고 `archive/`로 옮기며, 이동 시 저장소 안의 링크도 함께 갱신한다.
