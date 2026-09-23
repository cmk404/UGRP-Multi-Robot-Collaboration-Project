# 현재 상태와 실행 경로

앞으로의 연구 우선순위와 완료 기준은 [연구 TODO (2026-09-22)](research_todo.md)를 따른다. 통신 효과가 주 질문이며 ACT·Jev·맵 확대는 관련 보조 과제로 구분한다. 아래 검증 수치는 각 기록 당시의 범위를 유지한다.

기준: 2026-09-20, main `7855e1a`에 포함된 기록. 이 문서는 진입점이며 실험 결과는 연결된 보고서의 실행 SHA·조건에만 적용된다. 새 결과를 병합하면 이 문서와 README의 요약을 함께 갱신한다.

## 실행 환경 — 2026-09-22

다른 연구자가 환경·행동을 편집하고 확인하는 기본 경로는 **로컬 CLI와 MuJoCo 기본 창**이다.
[Mac/Linux 설치·실행](local_simulation.md), [기존 자산 연결 범위](simulation_inventory.md)를 따른다.
Colab/Kaggle은 명시적으로 선택하는 배치 경로로 보존한다. 기존 모델 실험의 재현에는 별도 가중치·
프록시·프로토콜이 필요하며, 창이 열린다는 사실을 그 연구 결과의 재현으로 세지 않는다.

## 목적별로 읽기

| 하려는 작업 | 먼저 읽기 | 실행/구현 진입점 |
|---|---|---|
| 로컬 시뮬레이션 구성·실행 | [로컬 시뮬레이션](local_simulation.md) | `scripts/open_simulation.command` — 설정 파일·저수준 Python API·MuJoCo 기본 창 |
| 설치·테스트·PR | [CONTRIBUTING](../CONTRIBUTING.md), [Ubuntu 안내](ubuntu_quickstart.md) | `scripts/run_ci_tests.py`, `.github/workflows/tests.yml` |
| 새 세 LLM 계획과 공동 출하 | [연구 환경·실행 예시](research_dispatch_arena.md), [후속 복구 결과](../experiments/dispatch-adaptive-recovery-20260917/README.md) | `scripts/run_dispatch_e2e.py`, `scripts/run_dispatch_skills.py`, `scripts/dispatch_pair_skill.py` |
| 공동 운반 ACT 비교 | [ACT 운반 보고서](../experiments/2026-09-18-act-pair-carry/README.md), [레퍼런스 차이표](reference_alignment.md) | `scripts/run_carry_act_experiment.py` |
| 로봇별 단계·허가·동기화 | [계약](task_stage_sync_contract.md), [연결 진단](task_stage_execution.md) | `harness/task_stage_sync.py`, `scripts/demo_task_stage_sync.py` |
| 지도와 지형 선택 | [지도 목록](../maps/README.md), [지도 주행](known_map_navigation.md) | `maps/`, `sim/` |
| 학습·실험 기록 시각화 | [TensorBoard 안내](tensorboard.md) | `scripts/export_tensorboard.py`, `scripts/run_tensorboard.py` |
| 실물 연결·기록 | [네트워크 runbook](masterpi_network_runbook.md), [trace](real_trace_system.md) | 현재 로컬 네트워크·장치 상태를 별도 확인 |

## main에 포함된 후속 결과

- 공동 출하 복구: 실행 `e099a4a`에서 seed11, 동일 모델·`local_contact_fine`, 6조건 각각 새 LLM 계획과 물리 운반/방출 성공. 목적지 B와 장애물 조건의 이전 실패를 수정한 결과다. 조건당 1회이며 임의 배치/역할·실물 성능을 입증하지 않는다. [전체 실패·시간·입력 감사](../experiments/dispatch-adaptive-recovery-20260917/README.md).
- ACT 공동 운반: 실행 `92b984e`의 최종 12회(4조건 × 3정책). 운반 진입 조건에서 교사 3/3, ACT seed18 1/3, seed19 0/3; 나머지 조건은 모든 정책이 접근 단계에서 중단했다. 기록 계획을 재생했고 새 외부 LLM 호출은 없다. 작은 데이터의 ACT가 기존 제어기를 대체하거나 복합 복구를 일반화했다는 근거는 없다. [원본·가중치 위치와 한계](../experiments/2026-09-18-act-pair-carry/README.md).
- 지도 다양화·다중 물건의 장면/프로토콜·RGB 실행 기반은 후속 연구 기반 변경에 포함됐다. 과거 기준일의 결과와 현재 구현 범위는 [구성 검토](simulation_inventory.md) 및 각 실행 기록에서 구분한다. 전체 통신 비교·새 맵 운반 일반화가 완료됐다는 뜻은 아니다.

## 증거를 읽는 순서

1. [실험 인덱스](../experiments/README.md)에서 관련 ID를 선택한다.
2. 해당 README/report의 코드 SHA·프로토콜·결과·실패·한계를 읽는다.
3. 필요한 manifest·감사·영상만 열고 로컬 전용 원본의 존재를 확인한다. 저장소에 해시만 있다고 원본이 백업된 것은 아니다.

[이전 README 요약](archive/validation_summary_20260917.md), [과거 아키텍처](current_architecture_todo.md), [결정 이력](decision_log.md)은 과거 배경 자료다. 날짜·실행 경로가 다른 성공률을 합산하거나 옛 운영 명령을 현재 설치법으로 사용하지 않는다. 제어 입력과 교사/지도 예외의 현재 규칙은 [AGENTS.md](../AGENTS.md)를 따른다.
