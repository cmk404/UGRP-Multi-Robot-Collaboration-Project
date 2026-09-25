# 2026-09-25 구역 팀 운반(N명 TeamJob) 설계 제안 — Codex 분석

- **작성:** Codex CLI (gpt-6-astra, reasoning xhigh). Claude 코디네이터가 2026-09-25에 위임한 읽기 전용 분석이다. 파일 수정·시뮬레이션·테스트·Git 쓰기·worktree 생성은 하지 않았다.
- **기준 SHA:** 로컬 main `3cbf4e4`
- **상태:** **제안, 미검증** (proposal, not verified)
- 아래 본문의 로컬 절대경로 링크(`/Users/changmin/projects/ugrp/...`)는 작성자 기기와 작성 시점 경로를 가리킨다. 이후 커밋에서 줄 번호·구조가 바뀌었을 수 있다.

## 이후 진행 상황 (기록 시점 2026-09-25)

이 보고서가 제안한 N명 `TeamJob`/`TeacherTeamExecutor` 방향은 **일부가 PR #165(병합, A1)로 구현·테스트됐다**. `experiments/2026-09-25-zone-team-jobs/README.md`를 보라. A1은 `harness/zone_team_jobs.py`(TeamJob 상태기계·원자적 commit), `harness/zone_team_formation.py`, `harness/zone_team_footprint.py`와 단위 테스트 41개를 추가했지만, 이 보고서가 지적한 **구역 교사·실행기·통신 경로 연결(2절 이후 항목 다수, 특히 벽/문·LLM·부분 실패의 실제 SIM 주입)은 아직 하지 않았다**. 그 연결(A2)은 branch `claude/zone-team-a2`에서 진행 중이다(2026-09-25 시점 PR #169, WIP).

---

**권고: 화물 하나를 소유하는 `TeamJob`과 N명용 `TeacherTeamExecutor`를 추가하고, 접근은 개별적으로, 파지 이후는 하나의 물체 궤적으로 실행해야 합니다.** LLM은 화물·팀원·역할·목적지를 결정하고, 교사는 그 결정의 물리 실행만 담당합니다.

검토 기준은 로컬 main `3cbf4e4`입니다. 파일 수정·시뮬레이션·테스트·Git 쓰기·worktree 생성은 하지 않았습니다. GitHub PR 조회는 연결 실패로 동시 작업의 실제 변경분을 확인하지 못했으므로, 아래 인터페이스는 제안입니다.

**1. 기존 pair 코드의 재사용 범위**

| 구성 | 재사용할 부분 | 필요한 분리·일반화 |
|---|---|---|
| 접촉·자세 진단 | [`probe_dual_grasp_sync.py:97`](/Users/changmin/projects/ugrp/scripts/probe_dual_grasp_sync.py:97)의 `_plain_beam_contact()`, [동일 파일:242](/Users/changmin/projects/ugrp/scripts/probe_dual_grasp_sync.py:242)의 `_pose_metrics()` | 양쪽 손가락 접촉·접촉력·높이·기울기 측정을 N명과 catalogue의 cargo geom 집합으로 일반화. 고정 `team_beam`, r1/r3, fixture 배치는 이식하지 않음. |
| 동시 명령 | [`MultiMasterPiProductionV2._team_joint_move_servos():1979`](/Users/changmin/projects/ugrp/sim/multi_masterpi_production.py:1979)의 **전원 명령 설정 후 물리 한 번 전진** 원칙 | blocking 함수를 ZONE tick 안에서 호출하지 말고 [`ArmSequence:168`](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:168)를 공통 시작·종료 시각으로 예약. 기존 [`ZoneRun.step():82`](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:82)가 유일한 물리 시계 소유자로 남아야 함. |
| 공동 경로 | [`pair_navigation.py:107`](/Users/changmin/projects/ugrp/harness/pair_navigation.py:107)의 `footprint_clear()`, `swept_clear()`, `plan_route()` | 전체 하중 형상의 SAT·회전 포함 SE(2) 탐색을 추출. 고정 카메라·좁은 지도 범위·빔 크기를 강제하는 [`validate_map():51`](/Users/changmin/projects/ugrp/harness/pair_navigation.py:51)는 ZONE용으로 교체. |
| 편대 제어 | [`dispatch_pair_navigation.rigid_pair_commands():292`](/Users/changmin/projects/ugrp/harness/dispatch_pair_navigation.py:292) | 공통 병진·각속도와 **전 로봇 공통 포화 스케일**은 유용. r1–r3 간격 보정은 N명 물체 기준 상대 자세 보정으로 변경. RGB 추정 입력은 교사 내부 정답으로 대체하고 별도 실행기로 명명. |
| 장벽·소유권 | [`PairCarrySync:26`](/Users/changmin/projects/ugrp/harness/pair_carry_sync.py:26), [`SharedResourceLedger:250`](/Users/changmin/projects/ugrp/harness/pair_carry_sync.py:250), [`DispatchExecution.batch():171`](/Users/changmin/projects/ugrp/harness/dispatch_execution.py:171) | `PairCarrySync`는 이름과 달리 participants 반복 구조여서 3명도 수용 가능. 작업 hash·단계 결합을 추가하고, 여러 자원의 원자적 예약은 별도로 구현. `DispatchExecution`의 일괄 단계 전환·실패 시 자원 유지 원칙을 재사용. |
| 감사 | [`audit_pair_navigation.audit():18`](/Users/changmin/projects/ugrp/scripts/audit_pair_navigation.py:18), [`audit_pair_carry_sync.audit():272`](/Users/changmin/projects/ugrp/scripts/audit_pair_carry_sync.py:272) | 입력·명령·장벽 재생, XML/설정 hash, 평가 재계산을 ZONE schema로 이식. 기존 감사 통과는 물리 재실행을 뜻하지 않음. |

**그대로 가져오면 안 되는 부분:** `PairVision`·`TemporalPairVision`, 학습된 접근/파지 모델, 픽셀 정규화·빔 색 추적·ACT·저장 명령 replay는 RGB 학생 경로입니다. [`BoundPairSkill.replay():1056`](/Users/changmin/projects/ugrp/scripts/dispatch_pair_skill.py:1056), [`approach():1081`](/Users/changmin/projects/ugrp/scripts/dispatch_pair_skill.py:1081), [`carry():1307`](/Users/changmin/projects/ugrp/scripts/dispatch_pair_skill.py:1307)를 교사 실행기 전체로 감싸지 않습니다. 특히 [`holds_cargo():1273`](/Users/changmin/projects/ugrp/scripts/dispatch_pair_skill.py:1273)는 “닫기 명령 후 열지 않았음”만 뜻하므로 실제 하중 유지 판정으로 사용할 수 없습니다. 기존 고수준 beam controller에는 [weld 활성화 경로](/Users/changmin/projects/ugrp/sim/multi_masterpi_production.py:2362)도 있습니다.

**2. TeamJob·claim·경로 모델**

현재는 `active[rid]`에 작업을 두고 로봇마다 slot을 소비하며, 같은 물건의 복수 claim을 충돌로 처리합니다. 이를 같은 물건을 향하는 단독 작업 N개로 확장하면 안 됩니다. 근거: [`assign():192`](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:192), [`check_claims():182`](/Users/changmin/projects/ugrp/harness/zone_coordination.py:182), [`TeacherRobot._taken_by_peer():250`](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:250).

제안하는 작업 단위는 다음과 같습니다.

`TeamJob = {job_id, generation, item_label, zone, participants, role_by_robot, formation_id, spec_hash}`

- **동의·예약:** 각 참여자가 동일 명세에 동의해야 합니다. N명 충족, 역할 중복 없음, 한 로봇의 중복 참여 없음, 화물·목적지 공간 예약을 한 transaction으로 확인한 뒤 commit합니다. 호스트가 남는 로봇을 채우거나 충돌 claim을 임의로 팀으로 합치면 안 됩니다.
- **저장 구조:** `jobs[job_id]`가 유일한 작업 기록이고 `robot_job[rid]`는 참조만 둡니다. 목표 차감·완료·실패·slot 반환은 **물건당 한 번** 수행합니다. 현재 [`remaining_need():159`](/Users/changmin/projects/ugrp/harness/zone_coordination.py:159)의 로봇별 active 집계도 job 기준으로 바꿉니다.
- **plan_first:** 팀 작업을 한 번 선언하고 로봇별 목록에는 job ID를 참조시킵니다. 서로 다른 목록 순서로 생기는 순환 대기를 validator에서 거부합니다.
- **dynamic:** 기존 “claim 충돌 때만 대화”를 팀 제안·참여 수락·철회까지 확장합니다. 모집 중에는 자기 참여 의도만 유한하게 유지하고, 전원 commit 전 물리 작업은 시작하지 않습니다. 실행기가 동료 작업을 선점하지 않습니다.

상태 흐름은  
`COMMITTED → RENDEZVOUS → PREGRASP → CLOSE → LIFT → CARRY → LOWER → RELEASE → RETREAT → FINISHED`로 둡니다.

접근은 각자의 파지 역할 위치로 수행하지만, CLOSE 이후에는 전원 준비·완료 장벽을 사용합니다. 정렬 시간 초과를 준비 완료로 처리하지 않습니다. 현재 단독 실행기의 [12초 뒤 강제 파지 진입](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:403)은 팀 작업에 승계하지 않아야 합니다.

**부분 실패:** 접촉 전에는 예약을 취소하고 철수합니다. 접촉 후에는 전원 HOLD, 기존 arm event 취소, 가능한 경우 동시 하강·바닥 지지 확인·방출 후 해제합니다. 한 대 고장 시 남은 N−1대로 계속 운반하거나 자동 교체하지 않습니다. 안전한 하강이 불가능하면 실패로 종료하고 실제 점유 공간을 유지합니다. 현재 [`teacher_path_blocked`에서 즉시 집게를 여는 처리](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:376)는 재사용하지 않습니다.

동시 개발 두 작업과는 다음 계약을 먼저 맞추는 것이 좋습니다.

| 연동 대상 | 필요한 계약 |
|---|---|
| Cargo catalogue | `required_carriers`, 물체 좌표계의 grasp frame N개, 실제 grasp geom, 접근 base pose, 물체 충돌 형상·질량·관성·접촉 profile, 허용 formation, 착지 면적·방향. 파지점 이름·인원 등 정적 명세와 실제 body ID·현재 pose는 분리. |
| 벽·문 planner | 동일한 정적 rectangle/keep-out 정의를 소비하는 `pose_clear(pose, footprint)`와 `swept_clear(a,b,footprint)` 인터페이스. 화물 실행기가 별도 벽 목록을 만들지 않음. |

운반 시 footprint는 **화물+모든 chassis+팔의 합집합과 여유 폭**입니다. 물체 중심의 `(x,y,yaw)` 하나로 계획하며, 문 진입·회전·출구까지 swept footprint를 검사합니다. 초기 구현은 보수적인 외접 형상으로 시작할 수 있지만, 과도한 외접 직사각형으로 삼각 편대의 가능한 통로까지 차단하는지 확인해야 합니다.

자기 팀은 내부 충돌 검사 대상이며, 다른 작업에는 팀 전체가 장애물입니다. 현재 [`discs_for():473`](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:473)는 들린 화물을 장애물 목록에서 제외하므로 수정이 필요합니다. [`resolve_blocks():490`](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:490)의 yield도 운반 중에는 **팀 전체의 검증된 후퇴**만 허용합니다. 문 안의 점유권은 timeout만으로 풀지 않습니다. `goal_occupied`는 중심점 대신 착지·하강·철수 공간 전체에 적용합니다.

추가로 catalogue 연동에는 인식·평가 변경도 필요합니다. 현재 [작은 색 상자 전용 검출](/Users/changmin/projects/ugrp/harness/zone_perception.py:23), [최근접 물체 결합](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:120), [중심점·고정 높이 기반 referee](/Users/changmin/projects/ugrp/harness/zone_coordination.py:234)는 긴 빔·큰 crate·plate에 그대로 적용할 수 없습니다.

**3. Weld OFF 편대 제어와 물리 검사**

**교사에는 virtual-structure 방식을 권고합니다.** leader-follower는 직선 진단에는 간단하지만 선두의 오차·정지 지연이 후속 로봇과 물체에 전달됩니다. 3대는 물체 기준 구조를 공유하는 편이 역할 대칭성과 오차 관리에 적합합니다.

물체 기준 base offset을 \(r_i\)라 하면, 공통 기준 자세에서 각 로봇의 목표를 만들고  
\(v_i=v_o+\omega_o\times Rr_i+\text{제한된 상대 자세 보정}\)  
으로 명령합니다. 한 로봇이 포화되면 전체 속도를 같이 낮춥니다. **virtual structure는 목표 궤적이며 물리적 고정 제약이 아닙니다.** 각 로봇을 서로 다른 절대 목표로 강하게 끌어당기면 내부 힘이 커질 수 있습니다. 관련 구현 근거는 위 `rigid_pair_commands()`입니다.

교사 IK는 `물체 현재 pose × catalogue grasp frame → 각 robot base frame`으로 계산합니다. 기존 [`solve_grip_site_ik():146`](/Users/changmin/projects/ugrp/harness/visual_arm.py:146)는 반경 14.5–18 cm, 제한된 pan, radial pitch만 지원합니다. 따라서 crate의 반대편·plate의 세 방향에 맞게 **base 방향부터 정하고**, IK residual·관절 한계·팔/차체 충돌을 확인해야 합니다. 임의 6DoF 파지가 가능하다고 가정하거나 기존 상자의 고정 grasp 높이를 복사하면 안 됩니다.

구체적인 검증 항목은 다음과 같습니다.

- **들기 전:** N명 모두 지정 화물에 양쪽 손가락 접촉, 유효 접촉력, 허용 IK 오차, 안정된 정지. pulse 완료만으로 다음 단계로 넘어가지 않음.
- **들기·운반:** 물체 최저점의 바닥 여유, roll/pitch, 물체–집게 상대 pose drift와 slip 속도, 모든 로봇의 간격·방향 오차, actuator 포화와 하중 불균형을 기록.
- **내부 힘:** 접촉 wrench가 중력·가속 요구를 지지하는지와 과도한 상쇄 힘을 함께 확인. 접촉 수나 CoM의 삼각형 내부 위치만으로 안정 파지를 보장하지 않음. 마찰과 접촉 배치가 지지 가능한 wrench를 결정합니다. [Modern Robotics, Force Closure](https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-3-force-closure/)
- **놓기:** 바닥 지지·낮은 속도·안정된 자세를 확인한 뒤 전원 방출. 전체 화물이 목적 구역 안에 있고 집게가 분리된 상태를 일정 시간 유지해야 성공.
- **불변 조건:** 매 physics step에서 cargo 관련 weld/기타 고정 활성화 0, 운반 중 pose 직접 쓰기·외력 보조 없음. 질량·마찰·solver 설정은 조건 간 동일하게 고정.

초기 진단 기준은 기존 [`evaluate_samples():28`](/Users/changmin/projects/ugrp/scripts/evaluate_pair_navigation.py:28)의 기울기 10°, 간격 변화 2 cm, 지속 접촉·방출 검사를 참고할 수 있습니다. 새 물체의 합격 기준으로 검증 없이 승계하지는 않습니다. MuJoCo 접촉은 soft constraint이므로 위치 일치만으로 slip·접촉력을 보장할 수 없습니다. [MuJoCo contact model](https://mujoco.readthedocs.io/en/stable/computation/index.html#soft-contact-model)

**4. 세 통신 조건에서 동일해야 할 최소 실행 신호**

기존 연구 계약도 무통신을 “명시적 고수준 메시지 없음”으로 정의하고 최소 시작·정지 동기화는 유지합니다. [`coela_communication_study.md:64`](/Users/changmin/projects/ugrp/docs/coela_communication_study.md:64)

| 공통 신호 | 허용 의미 |
|---|---|
| `SUBMIT / CANCEL(job_spec_hash, generation)` | 자기 작업 의도 제출·철회. 호스트는 일치 여부만 검사. |
| `READY / NOT_READY` | 교사 내부의 단계 준비 상태. 동일 stage·epoch·freshness 정책 적용. |
| `GO / HOLD / ABORT`와 만료 시각 | 해당 팀 전체의 유한 실행 허가·정지. 부분 명령 실패 시 전원 취소. |
| 자기 작업 영수증 | 접수·실행 중·순서 종료·중단. 원인 로봇·접촉력·좌표·물리 성공은 노출하지 않음. |

`independent`에서도 각 로봇이 **동일한 화물·목적지·참여자·역할 명세를 독립적으로 제출한 경우에만** 팀 작업을 허용합니다. 부족한 동의를 채워주거나 “r2가 기다린다”는 알림으로 상대 결정을 전달하지 않습니다. 자기 timeout·고정 SIM 주기 외에 동료 이벤트로 재질문하지 않습니다. 현재 격리 근거: [`solo_context():33`](/Users/changmin/projects/ugrp/harness/zone_solo.py:33), [`independent_round():405`](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:405).

세 조건의 물리 제어기·재시도 한도·장벽 timeout·yield·문 예약·실패 정리는 동일해야 합니다. `plan_first`의 사전 합의와 `dynamic`의 모집·복구 대화만 고수준 차이로 둡니다. dynamic의 board는 추가 정보 채널이므로 **게시판+대화+재계획 정책의 효과**로 해석해야 합니다.

정답 기반 접촉·IK·경로 실패 상세는 teacher 로그에만 저장합니다. LLM에는 현재처럼 제한된 영수증만 전달하되, 이것도 교사 내부 판단에 의존하는 입력임을 명시합니다. 학생 실행으로 바꿀 때 해당 정답 전이·보정을 함께 넘기면 안 됩니다. [`AGENTS.md:11`](/Users/changmin/projects/ugrp/AGENTS.md:11), [`zone_dispatch.md:26`](/Users/changmin/projects/ugrp/docs/zone_dispatch.md:26)

**5. 단계별 구현·검증 게이트와 공수**

공수는 저장소에 익숙한 개발자 1명 기준 추정이며, 다른 에이전트의 벽·catalogue 구현 자체는 제외합니다.

| 단계 | 작업 | 다음 단계 진입 조건 | 공수 |
|---|---|---|---|
| 1 | catalogue·planner·TeamJob 계약과 버전 정의 | grasp/formation/착지 형상 schema, 학생 입력 allowlist, 기존 solo 호환 검사 | 0.5–1일 |
| 2 | N명 commit·예약·상태기계·runner 연결 | 2/3명 동의, stale ACK, 중복 역할, 교차 예약, 순환 대기, timeout, 단일 집계, 취소 후 늦은 arm event 차단을 fake clock으로 검증 | 1–2일 |
| 3 | 2대 beam 파지·동시 lift·직선·place | 정답 교사 fixture로 무고정 접촉 유지, 연속 정지 유지, 방출까지 반복 통과. r1/r3 고정 없이 역할 순열 검사 | 2–3일 |
| 4 | 공동 SE(2) 경로와 벽·문 통합 | 통과 가능/불가능 문, 회전 중 충돌, 막힌 착지, solo와 교착·yield. 불가능 경로는 물리 접촉 전에 거부 | 2–3일 |
| 5 | heavy crate·3대 plate 및 부분 실패 | 한 집게 열림·늦은 도착·한 대 정지·slip·운반 중 막힘에서 전원 안전 정지/하강. 고장 시 성공으로 집계하지 않음 | 3–5일 |
| 6 | 세 통신 조건 연결·감사·유한 비교 | 동일 commit 입력에는 동일 교사 명령/장벽 trace, independent 정보 누출 0. 고정 seed·역할 순열·동일 실패 주입으로 비교 | 1–2일 |

총 **약 10–16 인일**입니다. 가장 큰 불확실성은 새 crate·plate의 실제 파지 가능성과 하중 여유입니다. `required_carriers=3`이라는 catalogue 표기 자체는 3대 물리 운반의 증거가 아닙니다.

기존 회귀 기반은 [`test_zone_dispatch.py:413`](/Users/changmin/projects/ugrp/tests/test_zone_dispatch.py:413)의 무통신 격리, [`test_pair_navigation.py:24`](/Users/changmin/projects/ugrp/tests/test_pair_navigation.py:24)의 swept 회전 충돌, [`test_pair_carry_sync.py:31`](/Users/changmin/projects/ugrp/tests/test_pair_carry_sync.py:31)의 허가 폐기·재개 검사입니다. 이를 먼저 확장하고 물리 게이트를 통과한 뒤 LLM 비교를 시작하는 순서가 적절합니다.

최종 결과에는 합의 실패·교사 실행 실패·물리 배치 성공을 분리하고, SIM makespan·명령 수·호출/토큰·실패 포함 분모를 기록합니다. 원본 입력·명령·접촉 trace·영상과 map/catalogue/물리 profile hash를 로컬에 보존하고, 새 결과는 TensorBoard snapshot에 연결합니다. **이번 검토에서 새 물리 결과나 통과 판정은 생성하지 않았습니다.**
