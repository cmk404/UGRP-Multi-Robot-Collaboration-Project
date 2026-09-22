# C 읽기전용 진단 — 실패한 tick과 종료 시각

2026-09-22. D 요청: 회수된 두 실행을 읽고 저비용 오프라인 재현만 수행한다.
소스 수정·새 실제 실행·재제출은 승인되지 않았으며 수행하지 않았다.

## 대상과 보존

- 실제 실행 SHA: `bfe478f1209375d774b6dc2881300198f967f5ca`.
- C source HEAD: `44e831de52158f604a78d8c31fd270263dc37784`.
  세 C runtime/planner 파일은 실행 SHA와 diff가 없다.
- 원본 루트: `/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/rgb-communication-colab-bfe478f/recovered/result/runs`.
- solo runtime SHA256: `9a045b2cce1d324e33e2d16e1e8b91274a07754112cd21cca7d44ef5625de376`.
- joint runtime SHA256: `39c5e568db6b57016e4363a9a3a1e533eb442a30a52c4971a9e92e52e64ccb73`.
- 읽기 전후 위 해시가 같다. 원본/소스/PR 변경 없음. 이 메모는 ignored local outputs이며 원격 백업이 아니다.

## 확인한 세 층

1. **선행 저수준 RGB worker 실패:** solo는 SIM .75, joint는 .25에서
   `SKILL_FAILED / ValueError / stale RGB worker result` 뒤 hold와 task termination.
   마지막 해당 skill 입력은 solo .5, joint .1이다. 이 시점의 SIM age는 각각 .25/.15초로
   1초 미만이다. 고정 소스의 해당 OR guard에 따르면 2초 wall-age 조건이 발동한 것으로
   추론할 수 있다. 실제 worker duration, 어느 joint worker인지는 원문에 없어 확정하지 않는다.
2. **물리 clock 진행 실패:** 두 실행의 C `run_error`는 요청 SIM .8, RuntimeError이며
   evaluator snapshot은 .75다. referee 마지막 absolute clock은 둘 다
   `2.049999999999996`, setup origin은 `1.300000000000001`로 실제 상대시각도 약 .75다.
3. **C 종료기록 결함:** `rgb_communication_async.py:255`는 성공 여부를 알기 전에
   `now_s=(ticks-1)*tick_period_s`를 저장하고 257에서 `port.tick`을 호출한다.
   예외 뒤 465/493의 run_error/run_finished와 470의 close가 실패한 요청시각 .8을 쓴다.
   D의 `evaluator snapshot predates the runtime terminal event` 거절은 이 모순을 정확히 보존했다.

## 오프라인 재현 A — C 확정 결함

기존 `FixturePort`를 얇게 확장해 .75까지 정상 tick하고 .8 요청에서는 clock을 진행하지
않은 채 닫고 RuntimeError를 낸다. 실제 C async를 실행하며 planner는 wait-only
`OfflineDecisionPlanner`, tick .05, wall poll .001, decision period 1, 최대25 tick이다.

관찰: last successful tick=.75, port clock=.75, run_error=.8, run_finished=.8,
close requested=[.8], outcome=aborted/RUNTIME_ERROR. 외부 모델0, 물리엔진0.
이 재현은 C의 요청 시각/성공 시각 혼동을 직접 증명한다.

## 오프라인 재현 B — underlying RuntimeError의 강한 일치 후보

`git show bfe478f:sim/camera_robot_port.py`의 고정 소스와 기존 테스트 `_WorldSpy`만
사용했다. MuJoCo를 만들거나 렌더링하지 않았다. origin을 원본과 동일하게 놓고
각 .05 요청마다 .002씩25번 float clock을 더하면서 `CameraRobotPort.tick(world_time)`을
호출한다. 이어 B `_RelativeEndpoint.tick`처럼 `tick(origin+requested)`를 호출한다.

정확히 다음 수치로 실패했다:

- requested tick=.8; last acknowledged tick=.75.
- accumulated world clock=`2.049999999999996` (원본 referee와 정확히 일치).
- servo clock=`2.0500000000000007`.
- 차이=`-4.884981308350689e-15`초.
- `ValueError: sim_time must not move backwards`.

`sim/camera_robot_port.py:239-241`의 엄격한 역행 검사가 이 극미세한 반올림 차이를
진짜 역행으로 본다. B의 physics owner wrapper(619-621)가 예외를 RuntimeError로 감싸고,
C는 error_type만 보존하므로 이 cause chain은 회수 원문에 없다. 따라서 **동일 실패지점·
동일 시각의 정확 재현과 고정 소스가 뒷받침하는 원인 후보**이며 원문에 cause가 기록됐다고
말하지 않는다. 선행 stale-worker 실패와 이 clock 실패는 별개다.

B도 이후 수정하지 않은 DispatchScene.step/CameraRobotPort/RGBSkillExecutionPort를
clock-only fake에 연결하여 같은 수치와 RuntimeError→ValueError cause chain을 독립 재현했다고
회신했다. B close(.75) 뒤 C close(.8)은 이미 닫힌 포트의 clock을 바꾸지 않는다.
또한 B는 즉시 완료된 future를 owner가 2.01초 뒤 확인하는 fixture에서 stale gate가
발동함을 재현했다. 따라서 wall-age는 worker 계산 시간뿐 아니라 owner의 render/I/O/poll
대기도 포함하며, 실제 두 실행의 기록만으로 계산 지연과 확인 지연을 분리할 수 없다.

## 수정 없이 전달한 권고

- 요청 tick, 마지막 성공/확인 clock, 부분 진행 뒤 실제 clock을 분리한다. 실제 부분 진행을
  알 수 없을 때는 unknown으로 남긴다. clock-only 안전 메타데이터와 evaluator 상태를 섞지 않는다.
- backend의 실제 clock과 합성한 `origin+relative` clock을 섞는 경계를 바로잡고,
  진짜 역행 거부를 유지하는 수치 오차 정책을 독립 검증해야 한다.
- C가 실패한 요청시각으로 close/terminal을 기록하지 않도록 하는 후속 수정이 필요하다.
  단순히 last-ack를 actual time이라고 부르면 부분 진행 실패에는 여전히 부정확할 수 있다.
- D의 guard를 완화하거나 원본 .8을 .75로 사후 덮어쓰지 않는다.
- worker wall cap 완화, 새 물리/모델 실행, 보호장치 제거는 이 진단으로 승인되지 않는다.
- D/B/통합 담당에게 근거를 전달했다. 실제 회수 실패 결과의 TensorBoard 등록/표시와
  최종 보고는 D의 기존 회수 작업 범위이며 이 synthetic 재현을 새 실험으로 등록하지 않는다.
