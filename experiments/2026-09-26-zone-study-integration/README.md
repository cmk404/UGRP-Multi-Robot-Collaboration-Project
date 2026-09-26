# 2026-09-26 구역 대화 연구 통합 러너 배선 스모크 (로드맵 8, #223)

> **임시, 표식 사용, 연구 결과 아님.** 자세 제공자는 `tags_temporary`(벽 AprilTag 기반 자기 카메라 PF)이고, 결정은 #194의 no-LLM fixture가 내렸다. 이 기록은 배선 검증이다. 통신 효과·조건 우열의 근거가 아니다.

`kiro/` 브랜치의 작업(Kiro). PR #229. Refs #223, #222, #221.

## 무엇을 연결했나

한 러너에서 다음 경로가 끝까지 이어진다.

```
fixture 응답(#194) → SIM 비용 지불 뒤 해제(#194 D) → 그 로봇의 자기 카메라 실행기 API(#206)
→ 물리(MuJoCo, 동기 SIM) → 자기 실행기 사건 → 자기 깨우기 → 다음 호출(자기 손목 RGB 입력) → 기록(A 로그·I 평가)
```

| 파일 | 역할 |
|---|---|
| `harness/zone_study_integration.py` | `OfflineTrial`(#194)을 상속한다. 입력 출처(저장 프레임 → 로봇의 실시간 자기 `robot_cam` 프레임)와 행동 출구(→ 그 로봇의 실행기 API)만 바꾼다. 조건·프롬프트·프로토콜·버스·SIM 비용·스케줄러·fixture는 #194 그대로 쓴다. 자세 제공자 seam, 짝 상태 채널, 행동→실행기 대응도 여기 있다. 시뮬레이터를 import하지 않는다. |
| `scripts/run_zone_study_integration.py` | 물리 소유자. #206 `OwnCamTeamHost`를 상속한 `StudyTeamHost`를 SIM 비용 양자(0.1 s) 단위로 전진시킨다. `HostRobotLink`는 로봇 한 대의 자기 프레임·자기 실행기·자기 작업 API만 노출한다. |
| `configs/zone_study_integration/pose_providers.json` | 자세 제공자 목록. 러너는 prereg의 `pose_provider` id로만 고르고, 선택 항목·소스·보정 해시를 실행 번들에 넣는다. |
| `configs/zone_study_integration/i1_cyan_three_slots.json` | 통합 스모크 시나리오(E 검증 통과). #206 smoke-s700 배치의 cyan 3개만 주문한다. |
| `tests/test_zone_study_integration.py` | 시뮬레이터 없는 검사 80개(실제 `ZoneOwnExecutor` + 저장 손목 JPEG). |
| `harness/zone_study_contract.py` | 계약 v2: tags_v2 지도의 `landmarks.placement` 키 3개를 닫힌 스키마로 선언(아래 경계 결함 B1). |

### 시계

물리를 SIM 비용 양자 `QUANTUM_S = 0.1 s` 단위로 전진시킨다. 호출 시작·비용 해제·메시지 전달·타이머가 모두 양자 경계에 떨어지므로 다음이 정확히 성립한다.

- 호출은 **시작 시각의 물리 상태**에서 자기 입력을 캡처한다. 이때 쓰는 프레임은 그 시각 이전의 최신 자기 프레임이다.
- 행동은 **SIM 비용이 끝나는 시각**에 그 로봇의 실행기에 도달한다.

스케줄러의 `advance(from, to)` 콜백은 쓰지 않았다. 구간 안에서 생긴 실행기 사건을 스케줄러의 과거 시각에 넣을 수 없기 때문이다(`cannot schedule a call in the SIM past`). 대신 `run(until_s, close_at_horizon=False)`로 양자마다 멈춘다.

### 교란 제거 (PR #169 Codex 검토 1·2·3·5·12)

로봇 사이의 정보 채널은 둘뿐이다.

1. 조건의 대화 채널(#194 C `Transport` + D 스케줄러). `no_comm`에는 없고, `leader_ko`는 허브-스포크다.
2. 짝 상태 채널 `harness.team_carry_status`(#200, main). 고정 enum만 싣고 작업 내용이 없으며 **네 조건 모두 같다**(사용자 결정 9/26).

공유 게시판, 호스트 claim 중재, 동료 작업 종료 깨우기, GT 기반 깨우기는 없다. 로봇의 호출 계기는 자기 시작, 자기 실행기 사건(`job_done→idle`, `job_failed→failure/timeout`, `blockage_seen→blockage`), 자기 타이머(실행기가 쉬면 10 s, 작업 중이면 60 s), 그리고 채널이 열린 조건에서 실제 받은 메시지(`report`)뿐이다. 호출 정책(`CallPolicy`)·비용 설정·양자·행동 대응은 네 조건이 같고, 이는 `condition_invariant_config`로 검사한다.

**격리 검사**(`tests/test_zone_study_integration.py`):

- `test_peer_private_state_does_not_reach_a_robot_inputs_or_wakeups[4조건]`: r2의 비공개 상태(손목 프레임, belief, 자기 실행기 실패 사건)만 바꾸면, r1·r3의 요청 해시·깨우기 시각/계기·실행기 호출이 **네 조건 모두 같다**. r2 자신의 요청은 달라진다(교란이 실제로 들어갔다는 대조).
- `test_peer_private_state_reaches_a_robot_only_through_the_condition_channel`: r2가 자기 belief에 따라 r1에게 말하게 하면 `peer_ko`·`leader_ko`에서 r1의 차이는 **첫 전달 시각 이후에만** 생긴다. 그 첫 새 깨우기는 `(전달 시각, report)`다. `no_comm`에서는 r1이 완전히 같다.
- 변이 검사 7개가 모두 검출됐다(`mutations.json`): 동료 belief 누출, 공유 명령 이력(게시판), 동료 사건 깨우기, abort 뒤 macro 유지, 호출 시작 시각 해제, 호스트 중재(점유 주문 거절), 무통신에서 짝 상태 채널 누락.

### 자세 제공자 seam

실행기는 `PoseReport` 제공자에게서 자세를 받는다. 제공자 인터페이스는 `factory(static_map, params, seed=)`, `on_command`, `on_frame(now, rgb) -> PoseReport`, `report(now)`, `set_motion_profile`, `source`, `loc.estimate()/predict_to()`다.

- 러너는 제공자를 설정의 id로 고른다. `provider_record`(항목 + 소스 파일·보정 SHA-256)를 실행 번들에 넣는다.
- `HostRobotLink` 생성 전에 각 실행기의 자세 원천을 등록 제공자로 바꾼다. 그때까지의 자기 명령 로그를 새 제공자에 다시 보내고, 실행기 자신의 M1 라벨 검사(`_require_owncam`)를 다시 적용한다.
- `tags_temporary`는 `uses_landmark_tags=true`다. 그래서 `temporary=true`, `research_result=false`, `note_ko="임시, 표식 사용, 연구 결과 아님"`이 아니면 등록부가 거절한다. 이 라벨은 manifest·result·trial record·TensorBoard 스냅샷 이름에 들어간다.
- 표식 0개 비전 제공자(#216, `kiro/zone-vision-loc`)는 러너를 고치지 않고 JSON 항목만 추가해 끼운다. 조건은 다음과 같다. `source`가 M1 own-camera 접두사 `owncam_pf`로 시작해야 한다(`m1_owncam_contract`). `loc`가 실행기 goto/배달 구간이 읽는 `estimate()/predict_to()`를 제공해야 한다. 임시 모듈의 `OwnCamPoseSource` 하위 클래스를 등록해 교체되는지 검사했다(`test_a_new_provider_drops_in_by_config_without_runner_change`).

### 로봇 LLM 입력 (매 호출)

A payload 그대로다. 정적 지도(태그 포함, 해시 고정), 시나리오 설정에서 만든 주문서, **자기 손목 RGB 1장**(호출 시각 이전 최신 자기 프레임, 바이트 SHA-256이 참조와 일치), 자기 명령 이력(실행기 API 호출과 자기 명령 상태), 자기 belief(`ZoneOwnExecutor.belief_projection`: 자기 카메라 추정 영역·다시 보기 확인·집게 상태), 그리고 조건별로 실제 전달된 메시지다. TOP·GT·동료 상태는 들어가지 않는다. 요청 원문(system/user 텍스트)은 trial record의 `request_archive`에, 이미지 바이트는 `study/request_images/<sha256>.jpg`에 보존한다.

### 결정 → 실행기 대응 (`zone_study_action_map.v1`)

| 모델 행동 | 실행기 호출 |
|---|---|
| `claim(order_id, role, destination_zone)` | `deliver(order_id, destination_zone)` (구역 문자 → 그 로봇의 다음 슬롯) |
| `continue` | 호출 없음 |
| `wait` | 자기 작업이 있으면 `abort('wait_requested')`, 없으면 `hold(10 s)` |
| `release(order_id)` | 자기 배달 작업이 그 주문일 때만 `abort('release_requested')`, 아니면 거절 기록 |

수락된 abort는 호스트에 예약된 그 로봇의 macro 명령을 버리고 즉시 hold한다(`HostRobotLink.call`, #221 P1을 호스트 층에서 처리). `test_host_link_abort_drops_scheduled_macros_and_holds_now`는 예약 명령 2개가 버려지는지, hold가 호출되는지, 다음 실행기 step이 `hold`인지 확인한다.

## 사전 등록과 실행

- `prereg.json`: 게이트 P1–P8, 실행 번들 `25d7634a…`(러너가 시작 시 다시 계산해 다르면 거절), 소스 `28addf56`, 기록 `45999d9c`.
- `prereg_v2.json`: v1 결함 D1을 고친 뒤 새로 등록했다. 게이트 P1–P9, 번들 `6e949fc8…`, 런타임 `9f7b16f2`, 기록 `cbeb5301`. v1 파일은 보존했다.
- 스모크: `smoke-i700`/`smoke-i700b`(layout seed 700, trial seed 700 → leader r2), 주 4조건 × 1회, horizon 480 SIM s, `cargo_noslip_v1`, weld OFF, 동기 SIM, 스레드 1, 동시 2개. `leader_ko`·`structured`와 v2 전체는 고정한 detached worktree에서 실행했다(런처는 매 실행 번들을 다시 계산해 prereg와 대조한다).
- 부하 평균: 실행 시작 13.8–38.9, 종료 5.1–76.2(manifest). 공유 Mac이다. wall 시간은 결과로 쓰지 않는다.
- raw: `/Users/changmin/projects/ugrp/outputs/zone-study-integration-20260926/smoke-45999d9c/`(v1), `…/smoke-cbeb530/`(v2). 로컬 전용이며 원격 백업이 아니다.
- 스모크 이후 이 브랜치에서 바뀐 런타임 코드는 없다. 결과·TensorBoard 도구와 문서만 추가했다.

## dev 배선 실행 (결과 아님)

`outputs/zone-study-integration-20260926/dev/`에 있다. 모두 `--dev-horizon-s`로 실행했고 번들 검사를 적용하지 않았다.
- dev1(`peer_ko`, 12 s): 첫 호출에 자기 프레임이 없어 멈췄다. 장면 설정이 SIM을 1.3 s까지 진행한 것이 원인이다(B6). 이를 고친 뒤 커밋했다.
- dev2(`peer_ko`)·dev3(`leader_ko`·`no_comm`), 14 s: 경로 연결, 채널, 비용, 재해시 검사를 통과했다.
- 그 전에는 tags_v2 지도의 모든 호출이 A 검증에서 실패했다(시뮬레이터 없는 시험, B1).

## 결과 v1 — `smoke-i700`, 소스 `45999d9c`, 번들 `25d7634a` (배선, 연구 결과 아님)

`results_v1.json`(`build_results.py --prereg prereg.json`)에 기록했다. **사전 등록 게이트 P1–P8은 4조건 모두 통과했다.** 사후 검사 P9(재질문 간격)는 실패했으며, 아래 D1이 그 결함이다.

| 조건 | 멈춤 | SIM s | 호출 | 발화 | 사고 SIM s | 발화+전달 SIM s | 첫 배달 명령 | eval 배송 | wall s |
|---|---|---:|---:|---:|---:|---:|---:|---|---:|
| `no_comm` | 예산 소진 | 478.3 | 90 | 0 | 315.0 | 0.0 | 4.8 s | C 337.2 s, B 397.2 s | 2938 |
| `peer_ko` | 예산 소진 | 445.4 | 90 | 3 | 325.2 | 0.9+0.3 | 5.7 s | C 343.8 s, B 410.0 s | 2750 |
| `leader_ko` (리더 r2) | 예산 소진 | 456.1 | 90 | 4 | 320.2 | 1.2+0.4 | 5.7 s, 리더 6.6 s | C 349.5 s, B 409.1 s | 2901 |
| `structured` | 예산 소진 | 445.4 | 90 | 3 | 325.5 | 0.9+0.3 | 5.7 s | C 343.8 s, B 410.0 s | 2859 |

- **경로 연결:** 네 조건 모두 fixture의 claim 3건이 각 로봇 실행기에 **비용이 해제되는 SIM 시각**에 도달했다. 그 뒤 추가 claim 6건은 실행기가 `BUSY`로 거절했고, 81건은 `continue`였다. 발화 비용이 첫 물리 명령을 늦췄다. `no_comm` 4.8 s, 발화 1개 조건 5.7 s(+0.9 s = 발화 γ 0.3 + 출력 30토큰 × 0.02), 발화 2개인 리더 r2 6.6 s였다.
- **물리:** 네 조건 모두 같았다. r1이 P2-2의 cyan을 C1에, r2가 P2-1의 cyan을 B1에 놓았고 둘 다 자기 카메라로 확인했다(`SKILL_OWN_RGB_PLACEMENT_IN_SLOT`). r3은 P1-3을 `SEARCH_NOT_FOUND`(약 75 s)로 놓쳤다. #206 smoke-s700의 r3과 같은 실패다. 주문 2/3, 거짓 확인 0, weld `eq_active` 0, `noslip_iterations` 10, 벽 접촉 0이었다. r1–r2 로봇 간 접촉은 `no_comm` 3074, `peer_ko`·`structured` 2026, `leader_ko` 0 physics step이었다.
- **채널:** `no_comm` 송수신 0. `leader_ko`는 r1→r2, r3→r2, r2→r1, r2→r3 각 1건이고 follower↔follower는 0이다. `structured` 자유 문장 0. 짝 상태 채널 config SHA-256은 네 조건 모두 `9dfe20cf…`이고, 트래픽은 0(2대 주문 없음)이다.
- **입력:** 요청 90×4건이 모두 재해시됐다. 이미지는 그 로봇 자기 프레임 로그의 JPEG 1장(`CURRENT OWN WRIST RGB`)이고, 금지 키·값은 0이다. 스케줄러–물리 시계 차이는 최대 7e-12 s였다.
- **TensorBoard:** `outputs/tensorboard/0926-zone-study-integration-tags-temporary-v1`(run 8개, scalar 208개 재읽기 일치, run마다 Text `provenance/pose_provider` 라벨). 보기 키는 `zone_study_integration_smoke_v1_20260926`이다. 다른 작업이 소유한 서버(PID 9291)를 재시작하지 않고 `/data` API로 값을 대조했다. 고정 카드 8개는 `outputs/zone-study-integration-20260926/smoke-45999d9c-report/tensorboard-pinned-v1.png`(`tb_capture.py`)에 있다.

## 결과 v2 — `smoke-i700b`, 런타임 `9f7b16f2`, prereg `cbeb5301`, 번들 `6e949fc8`

v1에서 바꾼 것은 세 가지다. 재질문 규칙 `single_pending_own_timer.v1`(D1), 형식이 틀린 실행기 사건의 거절, 진행 로그다. 결과는 `results_v2.json`(`--prereg prereg_v2.json`)에 있다. **사전 등록 게이트 P1–P9가 4조건 모두 통과했다.**

| 조건 | 멈춤 | SIM s | 호출(v1) | 발화 | 사고 SIM s | 발화+전달 SIM s | 재질문 최소 간격 | eval 배송 | wall s |
|---|---|---:|---:|---:|---:|---:|---:|---|---:|
| `no_comm` | horizon | 480 | 60 (90) | 0 | 206.5 | 0.0 | 13.5 s | C 337.2 s, B 397.2 s | 2382 |
| `peer_ko` | horizon | 480 | 63 (90) | 3 | 224.4 | 0.9+0.3 | 13.6 s | C 343.8 s, B 410.0 s | 2298 |
| `leader_ko` (리더 r2) | horizon | 480 | 61 (90) | 4 | 213.6 | 1.2+0.4 | 13.5 s | C 349.5 s, B 409.1 s | 1681 |
| `structured` | horizon | 480 | 63 (90) | 3 | 224.7 | 0.9+0.3 | 13.6 s | C 343.8 s, B 410.0 s | 1643 |

- HTTP 예산을 소진하지 않고 horizon까지 갔다. 줄어든 호출은 모두 `continue`(noop)였다(v1 81 → v2 50–53).
- 물리 결과(배송 시각, 로봇 간 접촉 step, r3 실패)는 조건마다 v1과 **같다.** `continue`는 실행기를 부르지 않으므로, 호출 수가 달라져도 물리가 바뀌지 않는다. 결정적 동기 SIM에서 기대한 결과다.
- TensorBoard: `outputs/tensorboard/0926-zone-study-integration-tags-temporary-v2`(run 8개, scalar 208개 재읽기 일치). 보기 키 `zone_study_integration_smoke_v2_20260926`은 v1과 v2를 함께 보여 준다. 캡처는 `outputs/zone-study-integration-20260926/smoke-cbeb530-report/tensorboard-pinned-v1-v2.png`에 있다. 명령 수·모델 응답 시간 태그는 이 변환기에 없고, fixture라 응답 시간 자체가 없다.
- raw: `…/smoke-cbeb530/`(로컬 전용). 해시는 `results_v2.json`의 `raw`에 있다.

**이 두 버전이 입증하지 않는 것:** 언어 이해, 통신 효과·조건 우열, 표식 0개 위치 추정, 2대 운반이다. 조건 사이의 차이(첫 명령 시각, 호출 수)는 fixture 규칙과 비용 설정이 만든 값이다.

## 경계별로 깨진 것

| # | 경계 | 무엇이 깨졌나 | 처리 |
|---|---|---|---|
| B1 | 지도 → 결정 입력 | A 계약 v1의 닫힌 `landmarks.placement`가 tags_v2 키(`near_door_spacing_m`, `near_door_radius_m`, `door_posts`)를 거절했다. tags_v2 지도로는 **모든 호출**이 `ContractViolation`으로 실패했다(dev 첫 실행). | 계약 v2로 세 키를 닫힌·타입 검사 스키마로 선언했다(#229). #194에 알렸다. |
| B2 | 결정 → 실행기 | #206 `action_record`가 A에 없는 조건명 `no_llm_scripted`를 쓴다. #194와 합치면 #206 자체 테스트 1개가 실패한다. | 러너는 A의 `action_log_record`를 쓴다. 테스트 실패는 #206에 남겼다. |
| B3 | 결정 → 실행기 | 수락된 abort 뒤에도 호스트에 예약된 macro가 실행된다(#221 P1). | `HostRobotLink.call`이 macro를 버리고 hold한다(검사 + 변이 M4). |
| B4 | 결정 → 실행기 | 실행기 `deliver`는 `cyan`만 받는다. 2대 주문은 실행할 수 없고, 짝 상태 채널은 모든 조건에 있지만 소비자가 없다. | 목록으로 남겼다(#206/#221). |
| B5 | 결정 → 실행기 | 실행기 API에 pause/resume이 없다. 사고 중 hold는 쉬는 로봇에만 적용되고, 실행 중 작업은 계속된다. | `THINK_HOLD_POLICY`로 명시했고 네 조건이 같다. 설계 결정이 필요하다. |
| B6 | 실행기 → 상태 | 장면 설정이 첫 프레임 전에 SIM을 약 1.3 s 진행한다. `t0 = 0.5` 가정에서는 첫 호출에 자기 프레임이 없었다(dev 첫 실행). | 설정 뒤 첫 0.1 s 경계에서 시작하고 로봇마다 프레임 1장을 찍는다. |
| B7 | 실행기 → 상태 | 출발 직후(약 5–13 s) r1·r3이 `blockage_seen(UNMAPPED_OBSTRUCTION_IN_LANE)`을 낸다. pickup 상자를 막힘으로 본 것으로 추정되며, 이것이 네 조건 모두에서 `blockage` 호출을 만든다. | 확인하지 않았다. #193/#206의 판단 의미 문제로 남긴다. |
| B8 | 실행기 → 상태 | r3의 P1-3 `SEARCH_NOT_FOUND`(#206 smoke-s700과 같음). | #221의 관측점 거리 결함. |
| D1 | 대화 → 비용 | 재질문 타이머가 행동마다 쌓여 영구 연쇄가 생겼다(#194 오프라인 규칙 상속). v1은 4조건 모두 HTTP 90/90을 445–478 s에 소진했고, 81회가 `continue`였다. 채널 조건은 메시지 깨우기로 연쇄가 늘어난다(시뮬레이터 없는 비교: 21/63/42/63). | `single_pending_own_timer.v1`(21/27/24/27). 검사와 변이 M8을 추가했다. v2 물리 재실행에서 4조건 모두 horizon까지 진행했다(호출 60–63). #194에 알렸다. |
| D2 | 대화 → 비용 | fixture는 `own_command_history` 길이로 다음 주문을 고른다. 그래서 메시지 깨우기로 늘어난 거절 명령 수가 조건마다 다른 다음 claim을 만든다. | fixture의 고정 규칙에서 생긴 효과이며 통신 효과가 아니다. 실제 LLM 파일럿에서는 사라진다. |
| C1 | CI | 병합한 #201의 workflow `zone-m1-owncam-run`의 `docs` 값(`prereg.json (+ prereg_amendments.json)`)이 파일 경로가 아니다. 그래서 `tests/test_simulation_scenes.py::test_workflow_entries_resolve_to_existing_source_and_documentation`가 실패한다. | 소유 밖이라 고치지 않았다. |

## 참고 자료

- **논문** (Crossref로 확인)
  - Edwin Olson, "AprilTag: A robust and flexible visual fiducial system", ICRA 2011. https://doi.org/10.1109/ICRA.2011.5979561. 임시 제공자 `tags_temporary`가 쓰는 AprilTag 계열이다(`harness/wall_tags.py`, OpenCV `DICT_APRILTAG_36h11`).
  - John Wang, Edwin Olson, "AprilTag 2: Efficient and robust fiducial detection", IROS 2016. https://doi.org/10.1109/IROS.2016.7759617. 위와 같다.
  - 이 PR에서 새로 조사해 설계에 쓴 논문은 없다. 재사용한 평가 모듈(#194 I)의 인용(Efron 1979, Kerby 2014, Rubin 1976)은 #194 본문을 따른다.
- **OSS** (`.venv-sim-worker-mac`에서 확인한 버전)
  - MuJoCo 3.12.0 (Apache-2.0), https://github.com/google-deepmind/mujoco: 동기 SIM 물리·렌더링. #206 호스트를 그대로 썼다.
  - OpenCV 5.0.0 (Apache-2.0), https://github.com/opencv/opencv: JPEG 복호, `cv2.aruco` AprilTag 36h11 검출(`harness/wall_tags.py`, 수정 없음).
  - NumPy 2.5.2 (BSD-3-Clause).
  - PythonRobotics (MIT), https://github.com/AtsushiSakai/PythonRobotics, commit `b2020cd`: `harness/owncam_localizer.py`가 각색한 입자 필터. 이 PR은 수정 없이 간접 재사용했다.
  - TensorBoard 2.21.0 (Apache-2.0), protobuf 7.36.2 (BSD-3-Clause): `scripts/zone_study_report.py --tb-events`로 스냅샷을 만들었다.
- **내부 모듈·PR**
  - #194 `kiro/zone-study-core` `4c8f9080`: `harness/zone_study_offline.py`(`OfflineTrial`, `FixtureActor`, `channel_checks`, `cost_checks`, `request_checks`, `reopen_trial_record`), `harness/zone_study_contract.py`, `harness/zone_study_inputs.py`, `harness/zone_study_prompts_ko.py`, `harness/zone_study_protocol.py`, `harness/zone_sim_cost.py`, `harness/zone_event_scheduler.py`, `harness/zone_study_eval.py`, `harness/zone_study_scenarios.py`, `scripts/zone_study_report.py`.
  - #206 `kiro/zone-own-executor` `c8a2355a`: `harness/zone_own_executor.py`(`ZoneOwnExecutor`, `OwnCamTeamHost`). 그 안의 #201 `harness/m1_owncam_delivery.py`, #181 `harness/wrist_zone_skill_v9.py`, #193 `harness/zone_own_perception.py`, #178/#197 `harness/owncam_pose_source.py`·`owncam_localizer.py`·`owncam_drive_v2.py`.
  - #200(main) `harness/team_carry_status.py`: 짝 상태 채널.
  - `sim/zone_landmarks.py` `TaggedZoneScene` → `sim.zone_scene.ZoneScene` → `sim.session_scenes.Scene`(표준 장면), `sim/zone_cargo_contact.py`(`cargo_noslip_v1`).
- **문서**
  - Codex 설계 `docs/design/2026-09-25-zone-dialogue-study-design-codex.md`(`origin/claude/records-0926`) 5·6·9·10절. 고정 r1 리더·별도 지휘자·교사 실행기 권고는 사용자 결정(순환 리더, 자기 카메라 실행기)이 대체한다.
  - Codex PR #169 검토 요약(코디네이터 scratchpad `codex-169-review.md`) 1·2·3·5·12.
  - `AGENTS.md`, `docs/execution_versioning.md`, `docs/tensorboard.md`, `docs/zone_own_executor.md`.
- **채택하지 않은 대안**
  - PR #169 러너(`scripts/run_zone_dispatch.py`, `zone_dispatch_v2.py`): 공유 게시판·호스트 중재·조건별 재질문 정책·교사 영수증이 교란이다. 소유 밖 파일이기도 하다.
  - `OwnCamTeamHost.run()`의 study_layer 콜백: 사건마다 즉시 API를 불러 사고·발화 비용이 없다. 호스트 내부(`_physics_until`, `_decide`, `_run_timeline`)만 `advance_to()`로 재사용했다.
  - `EventScheduler`의 `advance` 콜백: 위 "시계" 참고.
  - SimPy 같은 범용 이산 사건 라이브러리: #194 D 스케줄러가 이미 SIM 시계·결정성·HTTP 완료 순서 불변을 보장한다. 새 의존성은 이득이 없다.
  - 사고 중 실행기 일시정지: #206 API에 pause/resume이 없다. 현재는 쉬는 로봇만 hold하고, 실행 중 작업은 계속한다(`THINK_HOLD_POLICY`, 네 조건 동일). 미결로 남긴다.
  - 짝 상태 채널·메시지 버스·비용 모델 재구현: 기존 모듈로 충분했다.
