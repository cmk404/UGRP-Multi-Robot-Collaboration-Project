# 현재 상태와 실행 경로

앞으로의 연구 우선순위와 완료 기준은 [연구 TODO (2026-09-22)](research_todo.md)를 따른다. 통신 효과가 주 질문이며 ACT·Jev·맵 확대는 관련 보조 과제로 구분한다. 아래 검증 수치는 각 기록 당시의 범위를 유지한다.

기준: 2026-09-25, main `80df7f4`에 포함된 기록(아래 9/25 절). 그 아래 절은 적힌 날짜의 기록이다. 이 문서는 진입점이며 실험 결과는 연결된 보고서의 실행 SHA·조건에만 적용된다. 새 결과를 병합하면 이 문서와 README의 요약을 함께 갱신한다.

## 협업 층 결과 — 2026-09-25 (main 포함)

현재 사용자 방침에 따라 연구는 SIM 시간의 로봇 행동에 집중한다. 실시간 모드와 wall 시간·시뮬레이션 속도 개선은 다루지 않는다. 아래는 모두 조건마다 1회라 사례로만 읽는다.

- [구역 배송 벤치마크](../experiments/2026-09-25-zone-dispatch/README.md)(`zones/zone_open`, 교사 실행기): 구역별 색 개수 목표로 plan_first와 dynamic을 비교했다. Z3에서 4/4 목표를 달성했고, 호출과 토큰은 dynamic이 적었다. 이동과 IK는 정답 좌표로 하는 교사 조건이며 RGB 스킬 성공이 아니다. 두 방식 모두 대화하므로, 무통신과 비교한 통신 효과는 아직 재지 않았다.
- [동적 협업 실패 결정](../experiments/2026-09-25-dynamic-team-recovery/README.md)(v51/v56/v58): 실패 주입 11회에서 결정 대화는 모두 선택지 안에서 끝났다. 빔 파지 실패 뒤의 물리 복구는 미해결이다.
- [빔 재파지 뒤 지원 밖 원인](../experiments/2026-09-25-r3-regrasp-pose/README.md)(v61 오프라인 재분석): 실패 슬롯 r3는 물리 r1이다. 직접 원인은 들렸다 내려온 빔의 0.59 mm 정지 위치 변화이며, 기하 모델 배경에 박힌 빔 가장자리가 새 노란 성분이 됐다. 수정은 미실행.
- [계획 판단 기준](../experiments/2026-09-25-plan-guidance/README.md)(v45): 목표만 알려줘도 동시 실행을 5/5 골랐다.
- [목적지 선택 + 지도 A*](../experiments/2026-09-25-map-goto-navigation/README.md)(PR #148, 기본 OFF v59): 저장 계획 재생에서 A* 2/2, 기존 경로 2/2 성공.
- [계산 후 재생](../experiments/2026-09-25-post-run-replay/README.md): 재생 기록을 켜고 꺼도 명령과 평가가 같다.
- 물리 A/B가 없는 기본 OFF 후보: v43(주행 중 영상 시야 복구), v46(정지 뒤 재관측), v53(두 로봇 접근 동시성), v30(빔 미세 정렬 이득). 각 실험 기록에 사전 고정 계약만 있다.
- 병합 전: 넓은 구역 경기장 `zones/zone_wide`와 두 번 세기 수정(PR #155).
- 병합 전(PR #173): `zone_open` 은퇴(기본 `zone_wide`)와 벽·문 경로 지도 3종, 교사 벽 금지 영역·통로 대치 규칙. [기록](../experiments/2026-09-25-zone-hard-routes/README.md). 교사·fixture 조건.
- 병합됨(PR #177): 자기 카메라 위치 추정 1단계. 벽 AprilTag 지도 버전 `*_tags_v1`(기존 지도 불변), wrist 어안 PnP, 발행 명령 파티클 필터. 교사 주행 오프라인 평가에서 둘러보기 자세의 test 문 근처 오차는 p50 1.9 cm / p90 5.8 cm다. 사전 등록 게이트는 상자를 든 수평 carry의 태그 가시율 0% 때문에 실패했다. 사후 진단에서 20° 숙인 carry는 가시율 97%였다. [기록](../experiments/2026-09-25-zone-owncam-loc/README.md), [문서](zone_owncam_localization.md).
- 병합 전(PR #178): 자기 카메라 폐루프 문 통과(M1 2단계). 로봇은 실행 중 GT 없이 자기 추정만으로 A* 주행하고 멈춰 둘러본다. `zone_wide_door_tags_v2`(문기둥 태그)에서 고정 소스 9361a8d로 test nobox 3/3, box 1/3이 나왔다(s42는 문 근처 추정 p90 8.0 cm, s43은 240 s 초과). dev는 6/6이다. 사후 진단에서 짐을 든 움직임 모델이 멈춘 뒤에도 3–6 cm를 더 움직였다. 다음 코호트에서 새 seed로 멈춤 동역학을 따로 모델링해야 한다. [기록](../experiments/2026-09-25-zone-owncam-loop/README.md).
- 파이프라인 단계별 완료/진행/미착수 스냅샷과 2026-09-25 운영·연구 단계 결정은 [E2E 현황](e2e_status_20260925.md)과 [결정 로그 9/25절](decision_log.md#2026-09-25--운영-모델연구-단계지도화물통신-감사재파지-원인에-대한-사용자-결정-기록)을 따른다. Codex의 팀 운반/미등록 장애물/한국어 대화 설계 제안 원문은 `docs/design/`에 보존했다.
- 주 연구 조건 변경(사용자 결정): `plan_first`는 은퇴한다(재현용 코드 유지). 같은 날 후속으로 통신 채널만 바꾸는 축을 제안했다. 조건은 무통신 / 자유 한국어 대화 / 한국어 명령 지휘자 / 정형 메시지 통제이며, 전지적 지휘자는 참고용 상한이다. 로봇 LLM 입력에서 TOP 카메라를 빼고(평가 전용), 정적 지도와 시나리오 설정의 작업 지시서를 준다. 실시간 상태는 주지 않는다. 지휘자는 로봇 한 대가 겸하고 시행마다 r1/r2/r3로 돌린다(허브-스포크, 사용자 승인). 대화·사고 시간의 SIM 비용은 설계 중이다(Codex 통합 설계 진행 중, 자기 카메라 인식은 아직 없음, 구현은 #169 뒤). [결정 로그 9/25 조건 변경 절](decision_log.md#2026-09-25--주-연구-조건-변경-무통신동적-통신ai-지휘), [Codex PR 검토](design/2026-09-25-zone-pr-review-codex.md).
- 최종 연구 실행 입력(사용자 결정): 로봇은 자기 손목 어안 RGB, 정적 지도, 자기 명령 이력만으로 성공해야 한다. 교사는 시연 전용이고 `nav_cam`은 쓰지 않으며, 벽·문기둥에는 AprilTag를 허용한다. 첫 목표 M1은 `zone_wide_door`에서 로봇 1대의 청록 상자 문 통과 배달이다. 현재 자기 카메라로 위치를 추정하는 실행기는 없다. [자기 카메라 역량 목록](design/2026-09-25-own-camera-inventory-claude.md), [오픈소스 조사](design/2026-09-25-own-camera-oss-survey-claude.md).
- 대화 연구 통합 설계 제안(Codex, 검증 전): 주 조건은 무통신·자유 한국어·지휘 겸임·정형 메시지 네 가지다. 매 호출 정적 지도와 주문서를 주고, 추론·발화에 SIM 시간 비용을 부과하며, 작업 패키지를 A–I로 나눴다. 사용자의 이후 결정(자기 카메라 전용, 지휘자 순환)보다 먼저 쓴 문서라 머리말에 대체 관계를 적었다. [설계 문서](design/2026-09-25-zone-dialogue-study-design-codex.md).

## 실행 환경 — 2026-09-22

시뮬레이션의 공통 실행·버전·결과 관리는 [표준 시뮬레이션 관리](simulation_management.md)를 따른다. 기존 연구별 실행기는 등록된 어댑터로 선택한다. 기본 설정 실행과 공동 출하도 같은 관리 기록을 사용하며, 표준 설정 실행과 공동 출하/RGB backend의 출하장 생성·초기화는 `sim.session_scenes.Scene`을 공유한다. 통합 자체를 새로운 운반 성공이나 과거 실험의 재현으로 집계하지 않는다.

다른 연구자가 환경·행동을 편집하고 확인하는 기본 경로는 **로컬 CLI와 MuJoCo 기본 창**이다.
[Mac/Linux 설치·실행](local_simulation.md), [기존 자산 연결 범위](simulation_inventory.md)를 따른다.
Colab/Kaggle은 명시적으로 선택하는 배치 경로로 보존한다. 기존 모델 실험의 재현에는 별도 가중치·
프록시·프로토콜이 필요하며, 창이 열린다는 사실을 그 연구 결과의 재현으로 세지 않는다.

학습 가중치는 [모델 배포](model_artifacts.md)의 GitHub Release와 버전·해시 목록으로 관리한다. 최초 배포는 원본이 확인된 2026-09-18/19 공동 운반 ACT 두 모델과 추론 자산이다. 2026-09-22 expanded 모델은 아직 원본을 찾지 못해 `unavailable`로 표시한다. 배포·로딩 검사로 과거 성공률이나 최신 ACT 검증을 대체하지 않는다.

## 실시간 출하 실행 후보 — 2026-09-23

[실시간 출하 실행](realtime_dispatch.md)은 `dispatch --realtime-control`로 물리·관측·판단·MuJoCo 창을 분리하는 선택 경로다. 현재 저장 계획 open/seed11 조건에서 진단하며, 후보별 성공·실패와 고정 실행 소스는 [실험 기록](../experiments/2026-09-23-realtime-dispatch/README.md)에 둔다. 실시간 시계 복구, 명령 연속성, 화물 도착·하역은 각각 검증한다. 이 경로는 main에 있다. 현재 방침상 실시간 경로는 연구 대상이 아니다.

## ACT 학습·행동 개선 후보 — 2026-09-24

[후속 학습·실행 계획](../experiments/2026-09-24-action-act/refinement.md)은 v27의 실제 8,000 update 학습과 두 위치 × 세 제어기 비교를 바탕으로 한다. 첫 비교의 물리·프로토콜 완주는 기준 RGB 1/2, 후보 RGB 1/2, ACT 0/2였으며 속도 개선을 입증하지 못했다. 첫 모델은 개발 에피소드의 종료를 모두 놓쳤다. v28은 큰 영상 오차의 접근 보정, 제한적 직접 RGB 재획득, 중복 CNN 계산 제거·로봇별 병렬 ACT 추론, 실제 배포 출력에 대한 종료 학습을 사전 고정하여 새 조건에서 비교하는 후보이며 기본 제어기 채택·main 병합은 별도다.

## PR #92의 연구 제어기 검증 — 2026-09-22

[연구 제어기 검증 절차](research_controller_validation.md)에 따라 고정 소스 `f70bd9b`로 9/9회를 마쳤다. RGB 3/3, ACT+RGB 2/3, RGB 도착 확인을 붙인 ACT 0/3 성공이며, 지정된 세 회귀 조건에서는 RGB만 기본 제어기로 채택 가능하다. 공통 접근·파지 실패는 9회 모두 통과했고 남은 실패는 ACT 조기 종료 2회와 종료 신호 누락에 따른 행동 한도 소진 2회다. [최종 원본 감사·대시보드·영상 기록](../experiments/2026-09-22-research-controller-qualification/README.md)을 따른다. 모든 ACT 요청 4,552건의 입력 재구성, 9개 스냅샷·영상과 실제 TensorBoard 화면을 확인했다. 알려진 open-map 세 조건의 계획 재생 구성요소 검증이며, 새 지도 일반화·LLM 통신 효과는 입증하지 않는다. PR #92는 2026-09-23 main에 병합됐다.

## 목적별로 읽기

| 하려는 작업 | 먼저 읽기 | 실행/구현 진입점 |
|---|---|---|
| 로컬 시뮬레이션 구성·실행 | [로컬 시뮬레이션](local_simulation.md) | `scripts/open_simulation.command` — 설정 파일·저수준 Python API·MuJoCo 기본 창 |
| 설치·테스트·PR | [CONTRIBUTING](../CONTRIBUTING.md), [Ubuntu 안내](ubuntu_quickstart.md) | `scripts/run_ci_tests.py`, `.github/workflows/tests.yml` |
| 구역별 목표 협업 벤치마크(교사 실행기) | [구역 배송](zone_dispatch.md) | `scripts/run_zone_dispatch.py` |
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
4. 자기 카메라 PR(#177 `a9f5d6e`, #176 `edd075d`)의 수치를 인용하기 전에 [자기 카메라 반대 검토(Codex, 2026-09-26)](design/2026-09-26-owncam-adversarial-review-codex.md)를 함께 읽는다. 검토의 판정은 현재 입증 범위가 교사 주행의 오프라인 위치 추정과 GT pose를 쓰는 스킬 격리 시험까지이며 wrist-only M1 성공이 아니라는 것이고, 수정은 PR #181(스킬)·#178(폐루프)에서 추적한다.

[이전 README 요약](archive/validation_summary_20260917.md), [과거 아키텍처](current_architecture_todo.md), [결정 이력](decision_log.md)은 과거 배경 자료다. 날짜·실행 경로가 다른 성공률을 합산하거나 옛 운영 명령을 현재 설치법으로 사용하지 않는다. 제어 입력과 교사/지도 예외의 현재 규칙은 [AGENTS.md](../AGENTS.md)를 따른다.
