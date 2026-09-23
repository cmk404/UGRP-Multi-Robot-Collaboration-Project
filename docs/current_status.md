# 현재 상태와 실행 경로

앞으로의 연구 우선순위와 완료 기준은 [연구 TODO (2026-09-22)](research_todo.md)를 따른다. 통신 효과가 주 질문이며 ACT·Jev·맵 확대는 관련 보조 과제로 구분한다. 아래 검증 수치는 각 기록 당시의 범위를 유지한다.

기준: 2026-09-20, main `7855e1a`에 포함된 기록. 이 문서는 진입점이며 실험 결과는 연결된 보고서의 실행 SHA·조건에만 적용된다. 새 결과를 병합하면 이 문서와 README의 요약을 함께 갱신한다.

## 실행 환경 — 2026-09-22

시뮬레이션의 공통 실행·버전·결과 관리는 [표준 시뮬레이션 관리](simulation_management.md)를 따른다. 기존 연구별 실행기는 등록된 어댑터로 선택한다. 기본 설정 실행과 공동 출하도 같은 관리 기록을 사용하며, 표준 설정 실행과 공동 출하/RGB backend의 출하장 생성·초기화는 `sim.session_scenes.Scene`을 공유한다. 통합 자체를 새로운 운반 성공이나 과거 실험의 재현으로 집계하지 않는다.

다른 연구자가 환경·행동을 편집하고 확인하는 기본 경로는 **로컬 CLI와 MuJoCo 기본 창**이다.
[Mac/Linux 설치·실행](local_simulation.md), [기존 자산 연결 범위](simulation_inventory.md)를 따른다.
Colab/Kaggle은 명시적으로 선택하는 배치 경로로 보존한다. 기존 모델 실험의 재현에는 별도 가중치·
프록시·프로토콜이 필요하며, 창이 열린다는 사실을 그 연구 결과의 재현으로 세지 않는다.

학습 가중치는 [모델 배포](model_artifacts.md)의 GitHub Release와 버전·해시 목록으로 관리한다. 최초 배포는 원본이 확인된 2026-09-18/19 공동 운반 ACT 두 모델과 추론 자산이다. 2026-09-22 expanded 모델은 아직 원본을 찾지 못해 `unavailable`로 표시한다. 배포·로딩 검사로 과거 성공률이나 최신 ACT 검증을 대체하지 않는다.

## PR #92의 연구 제어기 검증 — 2026-09-22

[연구 제어기 검증 절차](research_controller_validation.md)에 따라 고정 소스 `f70bd9b`로 9/9회를 마쳤다. RGB 3/3, ACT+RGB 2/3, RGB 도착 확인을 붙인 ACT 0/3 성공이며, 지정된 세 회귀 조건에서는 RGB만 기본 제어기로 채택 가능하다. 공통 접근·파지 실패는 9회 모두 통과했고 남은 실패는 ACT 조기 종료 2회와 종료 신호 누락에 따른 행동 한도 소진 2회다. [최종 원본 감사·대시보드·영상 기록](../experiments/2026-09-22-research-controller-qualification/README.md)을 따른다. 모든 ACT 요청 4,552건의 입력 재구성, 9개 스냅샷·영상과 실제 TensorBoard 화면을 확인했다. 알려진 open-map 세 조건의 계획 재생 구성요소 검증이며, 새 지도 일반화·LLM 통신 효과는 입증하지 않는다. PR #92 승인·병합 전 변경이다.

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
