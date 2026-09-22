# B3 · RGB worker 지연과 입력 재사용

2026-09-22. 고정 출발점 `357e1f2e66dcae20c54f669711308ed18cf03966`
(PR #98, `codex/simulation-live-view`). 작업 브랜치 `codex/b3-rgb-worker-deadline`.
이 문서는 B의 소스·오프라인 계약 검증이다. 실제 물리/렌더/외부 모델 실행은 D만 담당하며,
이 결과는 운반 성공·속도 개선·통신 효과의 실측 결과가 아니다.

## 판정 의미: 변경하지 않은 제한

- wall 제한은 **제출부터 owner의 결과 소비 poll까지 2초**, 이미지 나이는 **1 SIM초**다.
  계산이 2초 안에 끝나도 poll이 2초를 넘으면 여전히 거절한다. 정확한 경계값은 허용한다.
- completion 기준으로 제한을 바꾸자는 초기 대안은 채택하지 않았다. deadline 연장,
  stale 허용, SIM pause, 미래값 채택, C의 LLM reply deadline 변경은 없다.
- 요청 freshness, 취소·pause·expiry·revision과 3개 worker 용량 제한을 유지한다.
  소비 시 자체 revision과 completion timestamp 존재도 검사한다.
- B2 실제 clock/ULP/180초/close 계약과 actor/evaluator 분리는 유지한다.

| 결정적 오프라인 조건 | 기존 판정 | B3 판정 |
|---|---|---|
| 계산은 빠르게 완료, 제출→poll 2.02초 | 거절 | 거절 |
| 2.02초 poll에도 미완료 | 거절 | 거절 |
| 계산 자체가 2.01초, poll 2.02초 | 거절 | 거절 |
| 계산 완료 후 취소 또는 revision 불일치 | 거절 | 거절 |

과거 `bfe478f` 두 raw 실행은 invalid_artifact로 보존한다. 당시 기록에는 worker 계산 완료
시각이 없으므로 해당 실행의 지연 원인을 render/poll이라고 단정할 수 없다.

## 재현한 불필요한 owner 지연과 수정

1. 완료된 solo 결과보다 새 pair 영상 준비가 먼저 4.2초를 소비하는 fixture에서
   기존 코드는 STOPPED, 변경 후에는 준비 전 소비하여 RUNNING이다.
2. pair의 첫 worker를 제출한 뒤 두 번째 입력을 2.1초 준비하던 fixture에서,
   기존 제출 시각은 2.1/4.2초다. 변경 후 두 입력을 모두 준비한 뒤 4.2초에 제출한다.
3. 새 pair 제어기 초기화가 완료된 solo 결과보다 먼저 수행되는 fixture도 실패를 재현했다.
   신규 제어기 초기화를 기존 결과 처리 뒤로 옮겼다.
4. 동일 SIM/absolute clock·episode·camera state의 반복 own RGB가 두 번 렌더되는
   factory fixture를 재현하고 원본 bytes만 재사용하도록 바꿨다.

기존 worker 처리 → 신규 제어기/입력 준비 → 신규 제출 순서다. 신규 준비·제출은 고정된
`r1/r2/r3` 순서이며, communication mode나 응답 완료 순서로 우선순위를 바꾸지 않는다.
전체 Future 완료 barrier나 SIM 추가 정지는 없다. 다른 작업의 계산이 미완료면 그대로 진행한다.
일부 pair 입력 실패 시 이미 준비한 상대 입력도 제출하지 않으며 독립 solo는 진행할 수 있다.
취소할 수 없는 실행 중 worker는 실제 done까지 용량을 차지하고 늦은 답에는 행동 권한이 없다.

## 프레임 cache 경계

cache 대상은 원본 JPEG bytes와 최초 capture ID/SIM·absolute·wall 시각/SHA256뿐이다.
own RGB는 로봇별로 분리하고 공통 TOP만 공유한다. 관측 ID, own revision, own command history,
status, 관측 시각은 매번 현재 포트 상태에서 새로 구성한다. 과거 관측 전체를 재사용하지 않는다.
동일 clock에서만 재사용하므로 cache hit로 SIM 나이를 갱신하거나 원본 시각을 덮지 않는다.

키는 episode, 실제/요청 clock, invalidation epoch, camera 이름/품질, scene XML hash,
TOP calibration hash, 실제 own/observer 해상도, renderer generation을 포함한다.
새 bundle/월드/model/data는 새 episode다. clock 전진, apply/hold, physics advance,
renderer identity/generation·해상도·calibration 변화는 무효화한다. 같은 episode의 실제 clock
역행/비정상 clock은 fail-closed다. worker는 renderer/cache/port를 소유하지 않는다.

B 경로에는 실행 중 reset/viewer/카메라 수정 API가 없다. 향후 in-place 카메라·모델 수정 API를
추가하면 먼저 명시적 invalidation에 연결해야 한다. 현재 cache는 임의의 외부 메모리 변경을
감지하는 장치가 아니다. 카메라 배치/FOV, physics, 접촉, weld, 해상도는 변경하지 않았다.
B transport의 own RGB는 기존 960×720, TOP은 실제 observer 설정이다. native preview의
384×288 경로와 같다고 주장하지 않는다.

## D/A 계측 파일과 필드

backend 출력 디렉터리의 **`worker-timing.jsonl`**, schema **`ugrp.rgb_worker_timing.v1`**.
기존 이벤트에 다음을 추가했다. 모두 supervisor 전용이며 actor 입력/상태에는 전달하지 않는다.

| event | 주요 필드와 의미 |
|---|---|
| `RGB_FRAME_READ` | robot_id, observation_id, timestamp_s, read_wall_s, generation, camera_identity, cache_hit, captures |
| `RGB_INPUT_PREPARED` | controller_setup_wall_s, capture_and_pack_wall_s, input_persistence_wall_s, request_pack_wall_s |
| `RGB_OWNER_ADVANCED` | physics_step_wall_s, referee_sample_wall_s, video_capture_wall_s |
| `RGB_WORKER_COMPLETED` | submitted_wall_s, worker_started_wall_s, worker_completed_wall_s, worker_queue_wall_s, worker_wall_duration_s, worker_outcome |
| `RGB_WORKER_CONSUMED` / `RGB_WORKER_REJECTED` | 위 timing + image_age_s, submission_to_poll_wall_s, submission_to_completion_wall_s, completion_to_poll_wall_s, polled_wall_s, future_done, guard, cap/초과 여부, wall_scope |
| `RGB_WORKER_REVOKED` | observation_id, timestamp_s, revoked_wall_s, reason, future_cancelled, future_done, submitted_wall_s, 완료 시점까지의 timing |

`captures.own_rgb/top_rgb` 각각은 capture_id, sim_time_s, absolute_time_s, started_wall_s,
completed_wall_s, sha256이다. 반복 hit도 원래 값을 그대로 기록한다.
`robot_id + observation_id`로 skill-inputs/worker/events를 연결한다.
wall 시각은 같은 child 프로세스의 monotonic clock이고 epoch 날짜가 아니다.

queue는 제출→계산 시작, compute는 decide 진입→반환/예외, completion→poll은 owner 소비까지다.
completion 로그 쓰기는 compute 이후에 수행되며 Future done은 그 쓰기 이후다.
따라서 completion→poll에는 완료 로그 I/O와 owner 대기도 포함되고 순수 scheduler 지연이 아니다.
미완료 timestamp/duration은 null, poll 직후 완료된 race의 completion→poll도 null이다.
입력 capture_and_pack에는 observe의 인코딩/hash 및 cache 기록 비용이 포함되고,
순수 render는 최초 capture의 started/completed 차이로 확인한다. owner 로그 쓰기 등 모든 비용을
각 항목으로 완전 분해했다고 주장하지 않는다. 실제 wall2 만족 여부는 D raw 로그로 판정한다.

완료 stamp는 한 번만 쓴다. 취소 후 완료될 수 있어 `bundle.close()`만으로 JSONL bytes가
동결되지는 않는다. D는 child 종료·reap 이후 hash/수집한다. 무한 대기나 강제 완료 표시가 없다.

descriptor에 `skill_worker_wall_scope='submission_to_consumption'`만 추가했다.
config exact schema, B/C API, 읽기 전용 asset binding 및 모델 경로/hash 의미는 그대로다.
`dispatch_open`의 실험적 solo/pair A/B만 지원한다. 22개 suite map은 preview-only/미지원,
native scene 58개는 transport 지원을 뜻하지 않는다. camera_sha256은 TOP-only이며,
map_group_sha256은 bounds/obstacles/top_camera에만 해당한다. terrain/별도 setup obstacle까지
같은 물리임을 증명하지 않는다. scene XML/instance/reset hash를 별도로 대조한다.

## 검증과 남은 실행

- focused: 105 passed, 27 subtests (실제 물리/렌더/외부 모델 없음).
- full offline: 1497 passed, 7 skipped, 198 subtests, 186.70초.
  이후 factory 계측 상관관계/descriptor assertion만 보강하고 focused를 재확인했다.
  PR CI는 고정 소스 handoff에서 별도 확인한다.
- 실제 검증은 A의 최종 source/config preflight 및 ACT 자원 해제 뒤 D가 직렬 solo/joint 각 1회만
  실행한다. 각 180 SIM초/600 wall초/6000명령, 준비·회수·정리를 합친 1800 wall초다.
- 현재 실제 신규 결과는 없다. 재시도·cap 완화·통신 효과/성공률 주장은 하지 않는다.
  D가 실제 결과와 TensorBoard를 회수하면 B는 두 결과의 timing 원인만 분석한다.
- raw는 로컬 보관이며 Drive 업로드나 원격 백업을 수행하지 않는다.
