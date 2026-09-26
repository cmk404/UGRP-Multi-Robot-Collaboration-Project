# 2026-09-25 계획 판단 기준(목표·일정 미리보기) 비교

PR #144, 실행 번들 `rgb-standard-dispatch-v45`. 계획 협상에 주는 입력만 바꾸고 제어·물리·카메라·성공 기준은 그대로 두었다.

## 배경

사용자의 이전 실행(v42, 같은 지시·같은 장면)에서 로봇들은 `beam.after=[box_job]`에 합의했다. 실행기는 합의를 지켜 한 작업씩 실행했다. r1·r3는 beam 앞에 도착한 뒤 약 108 SIM초 동안 기다렸고, 전체 제어는 216.06 SIM초가 걸렸다. 원인으로 두 가지를 의심했다.

- 계획 프롬프트에 판단 기준(목표)이 없었다.
- 능력 설명의 선행 조건 예시가 "box 먼저, beam 나중" 하나뿐이었다.

## 조건

같은 지시("서로 역할과 순서를 합의해서 beam과 box를 dock_b로 옮겨"), 같은 장면(`--variant open`, seed 11)을 썼다. 모두 동기 headless이고 `--auto-route-overlap`(표준 dispatch 기본값), 모델은 `gemini-3.8-flash`다. 조건별로 5회씩, 실행 순서를 섞어 돌렸다(`run_cohort.py`).

| 조건 | `--plan-guidance` | 계획 입력 |
|---|---|---|
| C0 | `legacy` | 기존 프롬프트 그대로. 바이트가 같은지 테스트로 확인 |
| C1 | `objective` | 목표: 안전하게 모두 배달한 뒤 총 SIM 시간을 줄인다. 선행 조건을 걸 때·비울 때 실행기가 어떻게 동작하는지 중립 설명. 특정 순서 예시는 삭제. 열린 지도에 맞는 통로 설명 |
| C2 | `objective_preview` | C1 + 동결된 제안마다 대략적 일정 미리보기 |

미리보기는 실행기 권한 규칙(`SkillBindings`)과 과거 명목 단계 시간(`configs/dispatch_stage_priors.json`)으로 계산한다. 실시간 상태를 읽지 않으며, 계획을 고치거나 거절하지 않는다.

## 결과

| 조건 | 임무 성공 | 합의한 선행 조건 | 동시 실행 | 제어 종료 SIM초 | beam 팀 대기 | box 대기 | 동시 적재 이동(referee) | LLM 호출(중앙) | 입력 토큰(중앙) |
|---|---|---|---|---|---:|---:|---:|---:|---:|
| C0 legacy | 5/5 | 없음 3, beam 먼저 2 | 3/5 | 141.26 ×3, 181.46 ×2 | 0.2 | 20.1 / 56.7 | 6.1 / 0 | 12 | 116,473 |
| C1 objective | 5/5 | 없음 5 | 5/5 | 141.26 ×5 | 0.2 | 20.1 | 6.1 | 12 | 119,145 |
| C2 objective_preview | 5/5 | 없음 5 | 5/5 | 141.26 ×5 | 0.2 | 20.1 | 6.1 | 12 | 130,383 |
| 참고: 사용자 실행 (v42, legacy) | 1/1 | box 먼저 | 0/1 | 216.06 | 108.0 | 0 | 0 | 15 | 136,618 |

- **순서 선택의 흔들림:** 기존 프롬프트에서는 같은 장면에서도 합의가 흔들렸다. 참고 실행까지 6회 중 3회가 선행 조건을 걸었다(beam 먼저 2, box 먼저 1). 목표를 준 C1·C2는 10회 모두 선행 조건 없이 동시 실행을 골랐다. 모든 계획의 역할·경로는 같았다(beam r1+r3 north, box r2 south, dock_b).
- **SIM 시간:** 동기 시뮬레이션은 계획이 같으면 실행이 결정론적이다. 같은 계획이면 명령·시간이 같았다(동시 141.26, beam 먼저 181.46). 그래서 반복 간 차이는 사실상 LLM이 고른 계획의 차이다. 동시 실행은 beam 먼저 직렬보다 40.2초(22%), box 먼저 직렬(참고 실행)보다 74.8초(35%) 짧다.
- **판단 근거:** C1·C2 로봇 메시지에는 "Separate gates allow parallel execution", "concurrent" 같은 근거가 나온다. C0에서 선행 조건을 건 제안에는 이유가 적혀 있지 않았다.
- **대기 측정:**
  - beam 팀 대기: 접근 명령 끝부터 파지 명령 시작까지의 간격이다.
  - box 대기: 자원 대기 판단부터 다음 판단까지의 SIM 시간 합이다.
  - 모두 발행 명령 기록에서 계산했다.
  - 동시 적재 이동은 사후 referee 값이다. 발행 명령 기준 운반 중첩은 15.0초다.
- **미리보기 정확도:** C2가 확정한 계획의 미리보기는 130초였다. 실제 제어 시간은 141.26초에서 동작 시작 5.2초를 뺀 136.1초다. 단, prior를 이 장면의 과거 실행에서 만들었으므로 독립 검증이 아니다.
- **비용:** LLM 호출 수와 협상 turn(중앙 4)은 조건 간 차이가 없었다. C2는 미리보기 문맥 때문에 입력 토큰이 C0보다 약 12% 많았다.

## 해석과 한계

- 장면·seed·지시가 하나뿐이고 조건별 5회다. 목표 제시가 동시 선택률을 높였다(C0 3/5 → C1 5/5)는 방향은 분명하지만, 비율의 신뢰구간은 넓다.
- C1이 이미 5/5라 천장에 닿았다. 그래서 **미리보기(C2)의 추가 효과는 이 장면에서 측정할 수 없었다.** 선행 조건이 실제로 필요하거나 선택지가 더 헷갈리는 장면(장애물 지도, 다른 dock/경로 조합)에서 따로 비교해야 한다.
- 목표 제시가 위험한 동시 진행을 부추기는지는 검증하지 않았다. 장애물 지도에서는 실행기가 선행 조건과 무관하게 직렬로 강제하므로 이 실험 범위 밖이다.
- 동시 실행에서도 box는 beam이 운반을 시작할 때까지 파지를 기다렸다(20.1초). 실행기가 beam 먼저 출발로 고정돼 있기 때문이다. box 먼저 동시 진행은 아직 검증되지 않은 조합이다.
- 모든 실행이 물리 성공이다. 성공률 차이가 아니라 계획 선택과 SIM 시간 차이를 본 실험이다.

## 기록과 재현

- 원본(로컬): `/Users/changmin/projects/ugrp/outputs/plan-guidance-20260925/`. 실행별 폴더, `cohort.jsonl`(시작 전후 부하 평균·wall), 로그가 있다.
- 참고 실행 원본: `/Users/changmin/projects/ugrp-worktrees/post-run-replay/outputs/simulation-runs/20260925-000852-dispatch-ee5eac42/artifacts`.
- 집계: `results.json`(`analyze.py`로 생성. 실행별 계획·제안 이력·대기·중첩·result SHA-256 포함).
- 소스: legacy-1은 `f76913d`, 나머지 14회는 `6e9b405`에서 돌렸다. 두 커밋의 차이는 `experiments/.../analyze.py` 추가뿐이고, 실행 번들 소스 해시는 같다.
- 첫 시도에서는 legacy-1 이후 분석 스크립트를 커밋하지 않은 채 두어 작업 트리가 dirty해졌다. 그 뒤 14회는 실행기의 "commit and freeze source" 검사로 시작 전에 거절됐다. 로그는 `refused-dirty-tree-attempt/`에 보존했고, 커밋 뒤 다시 실행했다.
- 물리 잠금은 `agent_lock.py`로 잡고 해제했다. SIM 시간 지표는 부하의 영향을 받지 않으며, wall 시간은 참고용이다.
- TensorBoard: `outputs/tensorboard/0925-plan-guidance`, view key `plan_guidance_20260925`. 제어 종료 SIM초는 현재 변환기가 내보내지 않으므로 이 문서와 `results.json`을 본다.

재현 명령:

```sh
python3 experiments/2026-09-25-plan-guidance/run_cohort.py --output outputs/plan-guidance-NEW
.venv-sim-worker-mac/bin/python experiments/2026-09-25-plan-guidance/analyze.py outputs/plan-guidance-NEW --write results.json
```

## 원본 상태 (2026-09-26 추가)

2026-09-26 병합 worktree 제거로 참고 실행 원본(`…/post-run-replay/…/20260925-000852-dispatch-ee5eac42`)이 지워졌고 복구할 수 없다. 코호트 15회 원본은 기본 체크아웃 `outputs/plan-guidance-20260925/`에 그대로 있다. 결과·판정은 바꾸지 않았다. 상세는 [raw_status.json](raw_status.json)과 [사고 기록](../2026-09-26-disk-incident/README.md)에 있다.
