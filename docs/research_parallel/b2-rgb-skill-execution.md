# B2 · 실제 RGB 스킬 연결 계약

2026-09-22. 출발 코드 `936bf821b873c9a364a768c4beea7c3debf27dd7`.
관련 범위 R2/R3, E0. 목적은 독립 actor 선택을 같은 RGB 운동 경로에 연결하는 것이다.
이 문서는 코드/오프라인 연결 검증이며 실제 운반·통신 효과·새 맵 일반화 결과가 아니다.

## 실행 번들 선택 — 2026-09-23

새 실행은 `execution_bundle_id`를 명시해야 한다. 현재 adapter의
`rgb-adapter-legacy-v1`은 기존 물리·명령 타이밍을 기록한 실험 버전이며,
과거 dispatch 성공 버전과 다르다. 이미지 정규화·공동 식별 수정이 있어도
성공 경로의 접촉 설정·명령 일정까지 같아진 것은 아니다.
[실행 버전 관리](../execution_versioning.md)에 따라 번들·실제 적용값을
검사하고 성공 기준과의 차이를 기록한다. 이전 실행 설정·결과는 수정하지 않는다.

## RGB 입력·식별 계약 수정 — 2026-09-23

과거 성공한 dispatch와 독립 실행기의 연결 차이를 다음 경계에서 제거한다.

- 단독 own RGB는 `normalize_own_rgb`를 통해 640×480, JPEG quality 95로 맞춘다.
  성공 dispatch와 `SoloActorSkill`이 같은 함수를 호출한다. 지원 입력은 기존
  640×480·960×720이며 다른 크기는 거절한다. crop·FOV·카메라 위치를 바꾸지 않는다.
  관측 원본은 보존하고 원본/변환 이미지의 크기·SHA·변환 방법을 판단 근거에 기록한다.
  픽셀 기준을 가진 `VisualBoxSkill`에 native 960×720을 직접 전달하지 않는다.
- 공동 coarse에 들어가기 전에 `identify_lower`와 `identify_upper`를 순서대로
  실행한다. 해당 역할만 짧게 이동하고 다른 참여자는 정지한다. 각 actor는 자기
  발행 명령과 공용 TOP 연속 영상으로 만든 자기 motion claim만 보관한다.
  역할 이름은 모델 슬롯이며 실제 로봇 ID나 화면상 위치를 정답으로 사용하지 않는다.
  감독 실행기는 다른 작업의 진행 중 명령을 마친 뒤 짧은 정지 구간을 확인하고
  식별을 시작한다. 식별 중 다른 작업의 새 움직임만 유예하며 물리 시계·취소·
  만료·watchdog은 계속 처리한다. 이 규칙은 모든 통신 조건에 동일하게 적용된다.
- 각 actor의 자기 claim을 기존 `PairCoarsePixels`에 연결한다. 작은 바퀴 성분,
  국소 마스크와 기존 2px 허용 조건을 성공 경로와 공유한다. 식별 불명확·지원
  범위 밖 영상은 정지하며, 단순 lane 전체 HSV 경로로 우회하지 않는다.
- 거절 판단은 감독 로그 `RGB_DECISION_REJECTED`에 단계·구체적인 이유·마스크
  통계/영상 변환 근거를 남긴다. 이 진단은 다른 actor의 입력에 전달하지 않는다.

재발 검사는 실제 backend 크기의 JPEG를 사용해 단독 행동·입력 SHA 계약을
검사하고, 공동의 순차 식별·실제 저장 영상 인식·불명확한 식별의 정지·취소 경계를
검사한다. `scripts/run_ci_tests.py`가 `tests/test_rgb_execution*.py`를 자동 포함한다.
오프라인 통과는 운반 성공이 아니며 새 실행 SHA·설정·전체 실패를 포함한 물리
재생 결과를 별도로 기록한다. 아래 2026-09-22 설명의 `coarse_approach` 직접 연결과
초기 identity 부재는 이 수정 전의 상태다.

## 재사용과 달라진 부분

- 실제 `CameraRobotPort`의 bounded apply/tick/hold를 사용한다. own 명령 보간 cache만 읽으며 관절·접촉을 읽지 않는다.
- 단독은 기존 `SoloBoxTransport`/`VisualBoxSkill` 및 `ImageRoute`를 직접 호출한다. 기존 dispatch의 saturation 150, release ground refinement 설정을 유지한다.
- 공동은 기존 `coarse_approach`, `predict_stage`, `predict_student`, `canonical_pair_top`, `own_payload`, `ImageRoute`를 로봇별 유한 상태기로 연결한다. 기존 저장 grasp initialization/close/lift 목표·시간을 사용한다. 단계는 접근→RGB 미세 정렬→파지 보정→close/lift→own RGB 연속성 확인→운반→lower/open/retract→RGB 배치 주장이다.
- `BoundPairSkill` 전체를 실행하지 않는다. 이 클래스의 양쪽 private own RGB 집계와 긴 동기 loop, `SkillBindings`의 전체 계획 의존을 그대로 가져오면 독립 경계와 clock 계약을 위반하기 때문이다. 새 상태기이므로 이전 BoundPairSkill/ACT 물리 성능을 승계했다고 주장하지 않는다.
- `TaskStageExecution`의 고정 `TaskPlan`도 만들지 않는다. 참여자·역할·object·skill을 actor들이 각각 제출하며, 전체 독립 consent 및 단계별 로컬 RGB 준비 barrier만 수행한다. 포트가 동료/역할/순서를 고르지 않는다.
- `ImageRoute`에는 선택된 단일 object/route/dock과 정적 map만 전달한다. 다른 물건의 계획·합의 결과·완료 상태는 없다. 목표/경로는 아래 스킬 이름의 고정 의미이며 다른 경로로 자동 변경하지 않는다.

## D/C 연결

진입점은 `harness.rgb_skill_execution`의 다음 함수다.

```python
descriptor = backend_descriptor(config)       # 읽기 전용, 물리/추론 없음
static_context = public_static_context(config)
common_task = static_context['task']
bundle = build_rgb_skill_backend(config)       # 실제 실행: D만 호출
port = bundle.actor_port
# C scheduler가 port.tick(0), .05, .10 ... 및 observe/status/submit을 소유
clock = bundle.clock_snapshot()                # C/D supervisor only; actor에 전달하지 않음
port.close(clock['actual_time_s'])             # unknown(None)도 안전 hold, clock 전진 없음
snapshot = bundle.evaluation_snapshot()        # actor close 이전에는 오류
bundle.close()                                # 영상/평가/scene 정리, 반복 호출 안전
# 정리 뒤 snapshot()은 보존된 동일 평가 cache 반환
```

config exact fields:

```json
{
  "schema": "ugrp.rgb_skill_backend.v2",
  "execution_bundle_id": "rgb-adapter-legacy-v1",
  "map_id": "dispatch_open",
  "seed": 11,
  "output_dir": "outputs/NEW-NONEXISTENT-DIRECTORY",
  "max_sim_s": 180,
  "max_commands": 10000,
  "grasp_model_dir": "PATH/models/grasp",
  "stage_model_dir": "PATH/models/varied",
  "reference_top": "tests/fixtures/camera_goal_transport/reference-top.jpg"
}
```

입력은 SHA 검증한 원본만 읽는다. 모델 생성·재학습·배경 교체·support threshold 완화·미지원 모델 fallback은 하지 않는다. descriptor의 `input_hashes`는 실제 파일 절대경로를 키로 기록하므로 원격 복사 후 원격 config/descriptor를 다시 만들고 파일 내용 hash를 대조해야 한다. setup/output 경로가 달라지면 config hash도 달라진다.

### 독립 요청

| skill | object_id | 인원/own_role | 고정 route | 고정 dock | 필수 resources |
|---|---|---|---|---|---|
| solo_transport_A/B | box | 1 / solo | south | dock_a/b | box, south_gate |
| pair_transport_A/B | beam | 2 / lower, upper 각각 독립 제출 | north | dock_a/b | beam, north_gate |

`stage='RUN'`, `expires_at_s<=max_sim_s`. 나머지 B1 task_request envelope는 같다.
공용 apron 등 추가 자원을 actor가 명시 예약할 수 있으며 예약 충돌은 거절한다. 위 최소 자원 목록은 무충돌 경로/물리 통과 보장이 아니다. 이 어댑터는 동적 apron 양보·임의 회전·로봇 identity 재획득을 구현하지 않았으며 해당 미검증 조건을 성공으로 처리하지 않는다.

`lower`/`upper`는 저장된 r1/r3 영상 모델 슬롯의 의미다. 실제 로봇 ID를 배정하는 규칙이 아니다. 영상상 위치가 역할과 다르면 기존 RGB support gate가 실패할 수 있다. replay에서 D가 고정 배정한 역할은 진단용 scripted 선택으로 기록하며 독립 LLM 배정 성공이라 하지 않는다. `none`도 메시지 없이 각자의 matching submit으로 같은 공동 스킬에 진입한다.

## clock, 수명, 오래된 답

### bfe478f 회수 실패 후 제한 수정 (2026-09-22)

회수된 solo/joint 원본은 그대로 `invalid_artifact`다. 원본에는 RuntimeError의 전체 원인 chain이 없지만,
동일 소스의 `DispatchScene.step`과 `CameraRobotPort`를 사용하는 **산술 clock fixture**에서 같은 .8초 중단을 재현했다.
origin `1.300000000000001`, dt `.002`일 때 .75초의 world absolute `2.049999999999996`과
합성 endpoint clock `2.0500000000000007`의 차이는 `-4.884981308350689e-15`다.
기존 fixture는 .005초를 먼저 더한 뒤 endpoint.tick을 호출해 실제 step-before-increment 경계를 놓쳤다.

- 실제 endpoint에는 `_SimulationClock`의 world 누적 absolute time만 전달한다. `CameraRobotPort`와 `DispatchScene` 자체는 수정하지 않았다.
- 요청 relative time과 실제 elapsed의 비교에만 `(ceil(max(requested,actual_elapsed)/dt)+2) * ulp(origin+max(requested,actual_elapsed))` 상계를 사용한다. 이는 누적 덧셈 및 경계의 덧셈/뺄셈 반올림을 위한 범위다. 실제 clock 단조성에는 이 범위를 적용하지 않으며 1 ULP 역행도 거절한다. 음수/nonfinite, 의미 있는 timestep 불일치 역시 실패한다.
- descriptor의 `supervisor_clock_schema='ugrp.execution_clock.v1'`. `bundle.clock_snapshot()`은 정확히 `schema,clock_domain,last_acknowledged_time_s,actual_time_s`만 반환한다. 두 시간은 setup 이후 raw actual elapsed이며 last_ack는 성공한 tick에서만 갱신한다. 확인 불가 값은 null이고 실제값을 requested나 last_ack로 대체하지 않는다. 실제값은 반올림 때문에 requested보다 미세하게 작거나 클 수 있다.
- 이 callback은 actor facade/관측/상태에 노출하지 않는다. 읽기만 하며 render, physics step, evaluator, 좌표·관절·접촉·성공 판정에 접근하지 않는다. close 뒤에도 사용할 수 있다. 최종 scene shutdown 뒤에는 종료 시 확인한 clock을 보존한다.
- `close(None)`은 실제 clock에서 hold하되 clock이나 last_ack를 전진하지 않는다. 실제 clock이 확인 불가하면 마지막 **발행 명령 clock**에서 보간 없이 목표를 취소하고 바퀴를 정지한다. 이 취소 시각을 실제 물리 시각으로 보고하지 않는다. 부분 진행 실패도 원래 예외 chain을 유지하고 모든 소유 endpoint를 hold한다.
- `close(known_actual)`은 tick 요청과 달리 확인된 실제 clock과 일치하는지만 검사한다. 따라서 180초 tick 후 raw actual `180.00000000013756`의 합법적 누적 반올림도 종료할 수 있다. 종료는 물리를 step하지 않고 tick의 180초 실행 상한은 그대로다. 확인되지 않은 다른 시각·음수/nonfinite 종료값은 거절한다.
- 별도 `worker-timing.jsonl`(`ugrp.rgb_worker_timing.v1`)에 RGB_WORKER_COMPLETED/REJECTED를 기록한다. worker 시작/완료/계산 wall duration, queue 지연, 제출→poll wall 시간, image age, 두 거절 조건을 분리한다. 진행 중 거절의 계산 duration은 null이며 늦게 완료되면 별도 완료 행으로 남는다. 감독 로그는 actor 입력/상태에 들어가지 않는다.
- `port.close`/`bundle.close`는 진행 중 image worker를 무한히 기다리지 않는다(`shutdown(wait=False)`). 따라서 close만으로 worker 로그 bytes 불변을 주장하지 않는다. D의 전체 artifact hash/archive 경계는 child process 종료/reap 이후다(정상 종료 시 Python worker join, parent wall timeout 시 TERM/KILL/reap). timeout·강제 종료로 완료 계측이 없으면 계산 duration 진단은 미완료이며 성공 완료로 보정하지 않는다. 이 외에 bundle.close 직후 파일을 동결하는 소비자는 별도 drain/seal 계약 없이는 지원되지 않는다.
- **wall 2초와 SIM image age 1초는 그대로**이며 제출→poll wall gate의 의미도 바꾸지 않았다. clock 수정은 stale-worker 해결 또는 물리 운반 성공의 증거가 아니다. 추가 물리/Colab/LLM 실행 없이 오프라인 clock·안전 경계만 검증한다. C의 terminal 시간 계약 및 D의 평가·수집 연결은 각 후속 변경과 합쳐 별도 검증한다.

- scheduler가 소유한 SIM elapsed clock은 setup 종료 시 0으로 시작한다. 실제 world origin은 provenance에 보존한다. tick은 0~0.05초만 전진할 수 있고 180초를 넘을 수 없다. `DispatchScene.step` 한 곳만 post-setup 물리를 step하며 그 내부 각 물리 substep에서 모든 endpoint watchdog을 확인한다. 요청 시간과 실제 timestep grid가 다르면 중단한다.
- 영상 판단은 robot별 worker에서 실행한다. worker에는 own RGB, common TOP, 자기 명령 cache와 자신의 스킬만 들어간다. world·port·평가 참조가 없고 완료 전에는 명령 권한도 없다. 다른 로봇/clock은 이 worker를 기다리지 않는다. 한 번의 판단은 wall 2초 한도이며 결과를 받았을 때도 원본 RGB가 SIM 1초 이내인지 검사한다. 취소된 계산의 늦은 결과는 폐기한다.
- pose 보간/drive는 scheduler의 `MacroQueue`가 발행하고 raw duration은 최대 .1초다. 모든 명령은 전체 command cap 및 lease 만료로 제한한다. tick/모델을 정지해도 이미 발행한 endpoint 명령의 watchdog은 물리 owner가 확인한다.
- `own_revision`은 high-level consent/task generation이다. 요청/승인/철회/중단/일시정지/재개/만료/종료 및 수동 명령 변경 시 증가한다. 같은 consent의 자동 RGB macro/servo pulse와 RUNNING phase 변화는 증가하지 않는다. C는 원본 명령 history/hash를 evidence에 보존하고 예상 micro pulse 변화 때문에 pause/interrupt 답을 영구 거절하지 않는다.
- 실제 backend는 `expected_revision` 필수. 같은 로봇의 최근 64개 관측 ID, 당시 revision, 최대 5초 나이를 대조한다. 새 observe 호출만으로 이전의 여전히 유효한 관측을 무효화하지 않는다. 늦은 reply가 새 임무에 적용되지 않는다.
- 수동 B1 command queue도 dispatch 직전 요청 나이/lease 유효 기간을 다시 검사한다. 실제 자동 backend는 actor raw command를 거절하고 명시 interrupt 후 새 task를 요구한다.

### lifecycle exact fields

`cancel_pending`: `kind,request_id,task_id,reason,observation_id,decision_id,requested_at_s,expected_revision`.
해당 actor 자신의 pending consent만 철회할 수 있다. 해당 시도 전체를 취소하며 새 시도는 새 task_id다.

`pause/resume/interrupt/release`: 위 필드 + `lease_id`.
pause는 모든 task participant를 hold, queued macro를 폐기한다. resume는 각 participant의 새 독립 consent가 모두 필요하다. partial manipulation을 완료했다고 간주하거나 남은 pose sequence를 재생하지 않는다.

현재 실제 resume 지원은 pair의 coarse/yaw/lateral/forward/dock/carry/verify에서 새 RGB 재판단이다. 처리 중 영상 판단, partial grasp/lift/open과 solo의 중단된 macro는 **미지원으로 STOPPED**되며 새 explicit recovery task가 필요하다. 일반 물건 인계·파지 재개가 구현됐다고 보고하지 않는다. 새 task는 새 RGB 상태에서 다시 시작하며 성공은 별도 검증이다.

endpoint hold 실패 시 그 로봇과 기존 resources를 격리한다. 다른 task에 즉시 같은 물체를 재할당하지 않는다. RGB source 실패도 자기 바퀴만이 아니라 해당 joint task 전체를 hold/revoke한다.

## actor 상태와 독립 평가

실제 backend의 observation/status에는 `own_revision`이 있다. active row 추가는 `paused`, `own_resume_pending`이다. status의 `own_skill_status`는 `{}` 또는 `task_id,state,phase?,reason?,meaning`만 가진다.

- RUNNING: 자기 RGB controller 단계. measured progress가 아니다.
- STOPPED: `rgb_skill_unavailable` 등 정규화한 자신의 실행 불가.
- FINISHED_UNVERIFIED: 기존 RGB skill의 종료 주장. reason이 실패일 수도 있으며 물리 성공이 아니다.

원본에는 SKILL_DECISION→LOCAL_COMMAND를 `skill_decision_id`로, 해당 고수준 선택을 `decision_id`와 `consent_observation_id`로 연결한다. 실제 판단 JPEG는 `skill-inputs.jsonl`, `rgb/`에 hash와 함께 저장한다.

`evaluation_snapshot`은 facade 밖이며 actor close 후만 평가한다. 기존 Referee가 물체 들림·샘플별 운반 clearance·목표 포함·바닥 지지/로봇 접촉 해제·안정 구간·weld OFF를 평가한다. 물리 성공은 sampled evidence 범위다. 객체별 `attempted/selected_dock/physical_success/raw`와 `raw_by_dock`을 보존한다. `mission_complete`는 두 물건이 같은 dock에서 성공해야 참이다. 단독/공동 diagnostic 목표만의 완료는 D가 별도 `replay_goal_complete`로 파생하며 mission 원본을 덮어쓰지 않는다. 이 버전은 recovery 사건을 측정하지 않아 recovery 필드는 false로 남긴다.

출력: `scene.xml`, `episode-setup-only.json`, `backend-provenance.json`, `skill-inputs.jsonl`, `worker-timing.jsonl`, 실제 RGB JPEG, `referee-only.jsonl`, `execution.mp4`, `backend-evaluation.json`.

## E0 지원 매트릭스와 blocker

`map_support_matrix()`는 기존 `sim.act_map_suite.load_suite/validate_splits`를 재사용한다. dispatch_open 외의 기존 새16+회귀6 배치는 **scene preview 지원 / 운반 skill 미지원**으로 명시한다. 개발 문/코너도 예외가 아니다. Test A/B 정책 rollout은 하지 않는다.

확인한 첫 불일치: suite의 `scene_config`는 `local_goal`+최종 heading, 빈 routes/regions/resource_rules를 가진 preview를 만든다. 현재 단독/공동 ImageRoute는 A/B dock, north/south gate와 apron, 고정 평행 heading을 요구한다. 다른 구조를 연결하려면 segment/resource/heading 계약, RGB 기반 loaded footprint 경로와 양보/재개, 각 로봇의 image identity/support를 같은 변경으로 추가해야 한다. 빈 이름을 north/south로 채우거나 open 장면을 대신 만들지 않는다.

descriptor는 `map_id,map_version,map_instance_sha256,map_group_sha256,camera_sha256,goal_frame,reset_sha256,seed,map_to_scene`를 반환한다. instance는 작성 지도 전체, group은 camera/bounds/obstacles이고 seed/start를 제외한다. 실제 scene XML hash는 builder 이후 provenance에서 연결한다. 기존 ACT 경로 존재/교사 실행/정적 지도 검사/예전 open 출발 교란을 이 어댑터의 새 구조 성공으로 세지 않는다.

## 확인된 기존 asset

로컬 원본 `outputs/short-transport-prior-models-readback-20260913-v1/models/`를 읽기 전용 검사했다.

- grasp/student-skill.json: `9ab4e7547be7386de1705df0c6ab47a8953237a2317ccc0c188fcc6fd7e58049`
- varied/varied-start-skill.json: `b89e6fce188efe7ab025d08826bdf269b515c353b81f9b3184f9351ac8bf5f84`
- repo reference-top.jpg: `35e990136a3410ea514e0445f92a6886b07e2a463e8c3ee0804dd6f3ae8d091c`, 960×720
- dispatch_open map: `733b5ee3320f0230c199afaefe2c2a6b8baa6a57a152417a6e8022e3bc0f9bab`
- seed11 reset: `617b60f693f219afc13694960c04f484ea5c7666e08b39489dec4d31b7515db5`

grasp recovery kernel의 robot identity는 내부 robot_id가 아니라 manifest slot+파일 hash로 결합된다. alignment 모델은 내부 robot_id/stage도 검증한다. 원본 모델·기준 JPEG는 변경하지 않았다.

실제 Colab 2 replay는 D 단일 제출자, 외부 모델 0, SIM180/wall600초/회, 전체1800초 한도다. B는 이번 작업에서 제출·물리·렌더링·학습·LLM을 실행하지 않았다. 새 실제 결과/영상은 D의 원본 회수 및 TensorBoard 확인 전까지 미완료다. 오프라인 fixture는 공용 대시보드에 등록하지 않는다.
