# 2026-09-25 구역 팀 작업 A1: 목표 v2·TeamJob·팀 형성 규칙·착지 영역·편대 목표

사용자 요청(2026-09-25): 구역 과제가 여러 화물 종류와 여러 로봇의 팀 작업을 다루도록 핵심 부품을 먼저 만든다(A1). 구역 교사·실행기·통신 경로 연결(A2)은 `claude/zone-hard-routes`가 병합된 뒤에 한다. 이번 단계는 `scripts/zone_teacher.py`, `scripts/run_zone_dispatch.py`, `harness/zone_coordination.py`, `harness/zone_perception.py`, `sim/zone_arena.py`, `sim/zone_scene.py`, `maps/zones/*`를 **건드리지 않았다**.

- 기준: main `ad18b93`(PR #163·#164 병합 포함). 처음에는 `origin/main 12695bc`에 PR #164 브랜치를 병합 커밋으로 얹어 시작했지만(`941ff7f`, push 안 함), 작업 중 #164가 main에 병합되어 브랜치를 새 main으로 옮겼다. 쌓인 PR(stack)은 없다.
- 실행 소스: 이 브랜치의 A1 커밋(SHA는 `results.json`의 `source_sha`). 스모크는 그 커밋을 고정한 뒤 실행했다.
- 입력: 화물 목록(`sim/zone_cargo.py`, 목록 v1, `catalogue_sha256`는 `results.json`), 작성 지도 `maps/zones/zone_wide.json`, Codex 설계 제안(읽기 전용 분석, scratchpad `codex-team-carry-design.md`), [R1 경계 감사](../2026-09-25-zone-comm-boundary-audit/README.md).
- 교사 예외: 스모크의 이동·IK·접촉 판정은 정답 좌표를 쓰는 **교사**다. 결과는 교사 조건 물리 확인이며 RGB·학생·구역 배송 성공이 아니다. weld/equality는 쓰지 않았고 매 physics step `eq_active`를 검사했다.

## 1. 만든 것

| 모듈 | 내용 |
|---|---|
| `harness/zone_goal_v2.py` | 목표 v2 `{zone: {kind: count}}`(4색 + can, tile, long_beam, heavy_crate, tri_frame). 색만 있는 옛 목표는 `sim.zone_arena.goal_counts`로 그대로 보낸다(결과·오류 문구 동일). 종류별 착지 영역(`landing_layout`), 전체 발자국 심판(`referee_v2`), 로봇에게 줄 수 있는 정적 과제 정보(`task_static_info`) |
| `harness/zone_team_jobs.py` | `RoleClaim`, `TeamJob`(상태기계·부분 실패 규칙), `PhaseBarrier`, `TeamJobLedger`(원자적 commit, 물건당 한 번 목표 차감), `RendezvousRule`(모든 조건 공통 팀 형성 규칙), 조건별 선언 검사(`check_independent_claims`, `check_dynamic_claims`, `validate_team_plan`), `remaining_need_items`(물건 단위 집계) |
| `harness/zone_team_formation.py` | `FormationPlan`: 물건 pose(또는 기준 궤적)에서 로봇별 정거장 pose, 접근 명령, 팔 IK(파지·hover·하강·들기), virtual-structure 운반 명령(공통 포화 비율). PR #164 교사(`scripts/cargo_formation_teacher.py`)의 상수·`Reference`·제어식을 import해서 쓰므로 둘이 따로 변하지 않는다 |
| `harness/zone_team_footprint.py` | 물건 + 모든 차체·팔 + 여유의 발자국. 볼록 다각형 목록(합집합 = 정확한 모양)과 그 볼록 껍질(보수적 한 개). `harness.static_keepouts`의 `pose_clear`/`swept_clear`가 받는 형식 |
| `scripts/smoke_zone_team_formation.py` | 위 부품을 한 번의 운반으로 묶는 동기 SIM 스모크(열린 바닥, pair_beam·trio_frame). 장면·지표·요약은 PR #164의 `Probe`를 그대로 쓰고 교사만 바꿨다. `configs/simulation_workflows.json`에 `zone-team-jobs-smoke`로 등록 |
| `tests/test_zone_team_jobs.py` | 단위·성질 테스트 41개(CI 목록에 추가) |

실행 번들 등록(`harness/rgb_execution_bundle.py`)은 건드리지 않았다.

### 1.1 TeamJob

`{job_id, generation, item_label, kind, zone, participants, role_by_robot, formation_id, spec_hash}`, `required_carriers`는 정적 목록에서 온다. `spec_hash = digest(item, kind, zone, formation_id)`이다. 로봇과 역할은 해시에 넣지 않는다. 참여자 모두가 같은 해시에 동의해야 한다.

- **원자적 commit**: 모든 claim이 한 spec을 공유하고, 역할이 formation을 정확히 한 번씩 채우고(중복·누락 없음), 인원이 `required_carriers`와 같고, 같은 로봇이 두 번 나오지 않고, 어느 로봇도 진행 중인 다른 작업에 있지 않고(이중 소속 금지), 물건에 진행 중 작업이 없고 이미 배달되지 않았을 때만 성공한다. 하나라도 어기면 `CommitRejected`이고 장부는 바뀌지 않는다(테스트로 스냅샷 비교).
- **한 작업 = 한 물건**이다. 목표는 작업이 `FINISHED`가 될 때 물건당 한 번만 줄어든다. 단독 물건도 인원 1의 TeamJob이다(색 상자는 역할 `west`, 동쪽을 보고 잡는 기존 관례).
- **상태**: `COMMITTED → RENDEZVOUS → PREGRASP → CLOSE → LIFT → CARRY → LOWER → RELEASE → RETREAT → FINISHED | ABORTED`. 매 전이는 참여자 전원의 `PhaseBarrier`(같은 상태·같은 generation) 뒤에만 일어난다. 오래된 보고는 무시하고 센다. **시간 초과는 준비 완료가 아니다.** 시간 초과 경로는 실패(`fail`)뿐이다.
- **부분 실패**:
  - 접촉 전(COMMITTED, RENDEZVOUS, PREGRASP): 전원 `cancel_retreat`로 철수한 뒤 ABORTED로 끝난다.
  - 접촉 뒤(CLOSE, LIFT, CARRY): 전원 `hold_lower`(정지 → 함께 내림 → 방출 → 철수)를 거쳐 ABORTED로 끝난다.
  - LOWER, RELEASE 중: `continue_release`로 계속 내려놓고, 끝은 ABORTED다.
  - RETREAT 중: 즉시 ABORTED다.
  - 참여자는 바뀌지 않는다(튜플, 교체 API 없음). 남은 N−1대로 계속하지 않는다. 같은 물건은 새 generation의 **완전한** commit으로만 다시 시작한다(테스트).
- 영수증은 지금과 같은 두 문구다(`issued sequence finished` / `executor stopped before finishing`).

### 1.2 조건별 선언 검사 — 호스트는 팀원을 고르지 않는다

선언 형식은 `{"item", "zone", "role"}`이다. 옛 `{"box", "zone"}`도 받으며, 단독 종류면 역할을 채운다. 팀 종류는 역할을 반드시 적어야 한다.

- **independent** `check_independent_claims`: 각 선언을 **따로** 검사한다(`check_one_independent`를 로봇마다 호출). 기준은 그 로봇이 아는 것뿐이다: 목표, 자기 RGB 시야의 적재 목록, 구역 개수. 같은 정거장을 두 로봇이 선언해도, 한 물건에 서로 다른 구역을 적어도 호스트는 비교하지 않는다. 테스트로 확인한 것: 다른 로봇의 선언을 바꿔도 r1 결과가 같다. 로봇 이름을 바꾸면 결과도 같은 순열로 바뀐다(3! 전부, 300 seed).
- **dynamic** `check_dynamic_claims`: 호스트가 목표·RGB·진행 중인 동료 선언과 맞는지 확인한다.
  - 같은 물건·같은 역할은 `same_role` 충돌이고, 같은 물건·다른 구역은 `zone_mismatch` 충돌이다. 둘 다 당사자끼리 대화로 넘긴다.
  - 같은 구역으로 선언된 물건의 빈 역할을 스스로 선언하면 받는다(합류). 이때 필요량은 물건 단위로 한 번만 센다.
  - 혼자 선언한 팀 물건의 빈 역할을 호스트가 채우지 않는다.
- **plan_first** `validate_team_plan`: 로봇별 순서 목록에 `{item, zone, role}`을 적는다. 물건마다 구역이 하나여야 하고, formation을 정확히 채워야 하고, 목표를 정확히 만족해야 한다. 로봇별 순서를 합쳐 만든 선후 그래프에 **순환이 있으면 거부**한다(서로 기다리는 계획). 옛 `{box, zone}` 계획 항목도 받는다.

### 1.3 independent에서 팀은 어떻게 생기나 — 정거장 점유 규칙 (모든 조건 공통)

**규칙(`RendezvousRule`, 세 조건에서 같은 코드·같은 값):**

1. 선언한 로봇은 **자기가 선언한 역할의 정거장**(물건 pose × 목록의 `approach_base(role)`)으로 가서 기다린다.
2. 점유는 물리 사실이다.
   - 로봇의 base가 정거장 0.10 m·0.35 rad 안에 있고 그 정거장에 더 가까운 다른 로봇이 없으면 `at_station`이다.
   - 로봇이 물건 근처(정거장 0.5 m 안)에 왔는데 다른 로봇이 이미 더 가까이 서 있으면 `station_blocked`다. 그 로봇의 선언은 읽지 않는다. 물건 앞에서 보이는 사건이다.
   - 거리가 정확히 같으면 둘 다 막힌다. id로 이기지 않는다.
3. 한 물건의 **모든 정거장이 `at_station`이고, 그 로봇들의 선언 spec(물건, 종류, 구역, formation)이 같을 때만** 팀이 commit된다. 다른 어떤 경우에도 팀은 생기지 않는다.
4. 각 로봇은 **자기 도착 시각부터** 최대 60 SIM초 기다린다. 그 뒤에는 동료가 있었든 없었든 같은 "stopped" 영수증으로 끝난다.

**왜 공정하고 대칭인가, 왜 L1을 되풀이하지 않는가:**

- 결정에 로봇 id를 쓰지 않는다(`uses_robot_ids: False`). 로봇 이름을 순열로 바꾸면 점유 상태와 팀도 같은 순열로 바뀐다(400 seed × 6 순열 테스트).
- L1의 핵심은 "동료가 **배정받았다는 의도**만으로 이동 중인 로봇을 멈춘 것"이었다. 이 규칙은 동료의 의도를 읽지 않는다. 로봇의 상태는 **물리적으로 정거장에 선 동료**에 의해서만 바뀐다(점유·막힘).
  - 성질 테스트(400 seed): 같은 물건의 정거장에 서 있지 않은 동료의 선언을 임의로 바꾸거나 지워도 r1의 점유 상태와 소속 팀이 바뀌지 않는다.
  - 다른 spec(예: 다른 구역)을 가진 동료가 옆 정거장에 서 있어도 r1이 보는 결과는 동료가 없을 때와 같다: 팀 없음, 자기 도착 뒤 60초에 같은 영수증.
- 동료의 의도가 결과에 들어가는 유일한 경우는 **모든 정거장에 몸이 와 있고 spec이 같은** 순간이다. 사람이 말없이 빔 양끝을 동시에 잡는 것과 같은 공동 행동 자체이고, 이후 운반이 시작되면 영상으로도 드러난다. 영수증은 여전히 두 문구뿐이라 "동료가 다른 구역을 원했다" 같은 정보는 전달되지 않는다.
- 세 조건은 **같은 형성 규칙·같은 대기 시간·같은 실패 정리**를 쓴다. 조건 차이는 선언이 만들어지는 방식뿐이다: 무통신 독자 선언 / 사전 합의 목록 / 동적 선언·게시판·대화. 곧 C1(게시판·중재 묶음)은 그대로 남지만 팀 형성 자체가 새 교란이 되지는 않는다.
- 비용: 무통신에서는 팀이 우연히만 맞는다(같은 물건·같은 구역·보완 역할). 대기 60초가 자주 낭비될 수 있다. 이것은 **측정 대상**(무통신에서 팀 물건 달성률·대기 시간)이며 규칙으로 보정하지 않는다. C2에 따라 무통신 관례를 줄지는 A2 사전 등록에서 따로 정한다.

### 1.4 착지 영역과 심판 v2

- 착지 영역 = 목표 단위마다 bay 하나다. 구역 긴 축(zone_wide는 y, 1.4 m)을 따라 쌓는다.
- bay 길이 = 팀 envelope(물건 + 정거장의 차체·팔)의 긴 축 길이 + 0.06 m이다. 최소 0.30 m.
- 착지 사각형 = 물건 발자국의 축 정렬 경계 + 0.04 m 허용이다.
- 단독 물건은 동쪽을 보고 놓는다(기존 관례). 팀 물건은 0/90/180/270° 중 bay가 가장 짧은 방향을 고른다.
- envelope는 경기장 벽 안쪽 0.05 m 안에 있어야 하고 내부 벽(문 지도)과도 0.05 m 떨어져야 한다.

zone_wide 한 구역(0.6 × 1.4 m)에 들어가는 개수(`landing_capacity_table`):

| 종류 | 인원 | 착지 yaw | bay 길이 | 한 구역 최대(길이 기준) |
|---|---|---|---|---|
| 4색 상자·can·tile | 1 | 0° | 0.30 m | 4 (색 상자는 옛 규칙대로 3) |
| heavy_crate | 2 | 0° (로봇이 동·서, 구역 밖에 섬) | 0.30 m | 4 |
| long_beam | 2 | 90° (빔이 y축, 로봇이 남·북 끝) | 1.11 m | 1 |
| tri_frame | 3 | 90° | 0.88 m | 1 (+ 단독 1개) |

- 예: `{'A': {'long_beam': 1, 'can': 1}}`은 1.41 m가 필요해 거부된다. `{'A': {'tri_frame': 1, 'red': 1}}`은 받는다.
- `referee_v2`는 물건의 **모든 충돌 부품이 구역 사각형 안**에 있고, 바닥에 놓여 있고(최저점 ≤ 12 mm, 기울기 ≤ 10°), 손가락이 닿아 있지 않을 때만 센다. 비교용으로 옛 중심점 규칙 개수(`centre_point_counts`)도 낸다. 테스트에서 구역을 가로질러 튀어나온 빔은 중심점 규칙으로는 세지만 v2는 세지 않는다.
- **평가 전용 출력이며 로봇 입력이 아니다.**

### 1.5 팀 발자국

- 차체는 zone_wide 장면에서 r1의 모든 충돌 geom을 잰 값이다(팔 접은 상태 x −0.096…0.119, y ±0.1015 m). 이를 x −0.100…0.120, y ±0.105로 잡았다. 팔은 grip점까지 폭 0.07 m 띠로 둔다.
- 여유 기본값은 0.03 m이고, 부품마다 바깥으로 키운다.
- 빔 정거장에서 차체 앞면은 빔 끝에서 5 mm 떨어진다(테스트: 차체는 물건과 겹치지 않고 팔 띠만 물건 위로 간다).

**의존성:** `harness/static_keepouts.py`는 아직 main에 없다(`claude/zone-hard-routes`의 로컬 커밋 `c6cd334`, 파일 sha256 `2b4faf26…57aa3`, push 전). 내 코드는 그 인터페이스(물체 좌표계 볼록 다각형 → `pose_clear`/`swept_clear`)에 맞췄고 import는 호출할 때 한다. CI 테스트는 모듈이 없으면 건너뛴다(`importorskip`).

그 파일을 경로로 불러 확인했다(long_beam, heavy_crate, tri_frame):
- 먼 pose는 통과하고, 벽과 겹치는 pose는 거부하고, 벽을 지나는 swept 이동도 거부했다.
- 경기장 경계(bounds)를 적용해 동쪽 벽 가까이는 거부했다.
- 무작위 954 pose에서 껍질이 통과했는데 합집합이 막힌 경우는 0이었다(껍질이 보수적임).

## 2. 테스트

`tests/test_zone_team_jobs.py`(41개, 1개는 static_keepouts 없어 건너뜀). 요청된 성질 테스트:

- 목표는 물건당 한 번 줄어든다. 무작위 commit·ready·fail 80단계 × 300 seed 뒤 매 단계마다 확인했다:
  - `FINISHED` 작업 수 = 배달된 서로 다른 물건 수 = 배달 합계
  - 남은 목표 = 목표 − 배달
- 이중 소속 없음: 같은 성질 테스트에서 매 단계, 진행 중 작업마다 로봇이 하나뿐이고 `robot_job` 색인이 일치한다.
- independent 형성 규칙: 로봇 순열에 대칭이고(400 seed × 6), 동료의 사적 의도에 의존하지 않으며(400 seed), spec이 다른 동료는 없는 동료와 같다.
- 옛 목표: 무작위 옛 형식 목표 600개 × 두 변형에서 v2 결과·오류 문구가 옛 검사기와 같다.
- 그 밖에 원자적 commit 5종, 부분 실패 8개 상태, 계획 순환 거부, dynamic 충돌·합류, 착지 영역 겹침·경계(무작위 목표 200+), 심판 v2, 편대 목표가 목록 `world_grasps`와 일치하는지, 공통 포화 한계, 로봇 이름 바꿈 대칭, weld 경로 미사용(소스 검사).

`scripts/run_ci_tests.py` 결과는 §4에 있다.

## 3. 동기 SIM 스모크 (열린 바닥, 교사, weld OFF)

`scripts/smoke_zone_team_formation.py`, 소스 `f9446a2`(깨끗한 트리), `zone_wide` 열린 바닥, 접촉 프로필 `local_contact_fine`, 경로는 PR #164와 같다(0.8 m 전진 → 90° 회전 → 0.5 m).

- 로봇은 정거장 뒤 0.30 / 0.55 / 0.80 m에서 출발했다. 도착 시각이 달라서 먼저 온 로봇이 실제로 기다렸다.
- 흐름: 선언 → `RendezvousRule`(정답 pose로 점유 판정) → `TeamJobLedger.commit` → 상태마다 barrier → `FormationPlan`의 접근·IK·운반 → 내림·방출·철수.

| 스모크 | 정거장 도착(SIM 초) | commit | 결과 | 배치 오차 | 들기 최저점 | 미끄러짐 | 추적 오차 최대 | `eq_active` 최대 | SIM 시간 | 부하(1분, 시작→끝) |
|---|---|---|---|---|---|---|---|---|---|---|
| pair_beam (r1 end_neg, r2 end_pos) | 3.9 / 6.0 | 6.1 | 성공, FINISHED | 3.6 mm, 0.01° | 61.7 mm | 5.2 mm | 43.9 mm | 0 | 56.1 | 285 → 197 |
| trio_frame (r1 v0, r2 v1, r3 v2) | 3.9 / 6.0 / 8.1 | 8.2 | 성공, FINISHED | 5.4 mm, 0.0° | 46.1 mm | 17.4 mm | 44.4 mm | 0 | 59.7 | 194 → 178 |

- 성공 판정은 PR #164 `Probe`의 기준 그대로다. 들어 올린 상태로 버팀, 운반 중 바닥 접촉 0, 낙하 0, 목표 5 cm·10° 안, 기울기 3° 미만, 방출, 로봇 기울기 10° 미만, weld 0을 모두 만족해야 한다.
- 두 번 모두 barrier가 9번 열렸다(COMMITTED→…→FINISHED). 모든 팔 이벤트는 barrier가 열린 같은 SIM 시각에 같은 길이로 예약됐다. 로봇별 준비 시각은 0.1초 교사 주기 안에서 같았다(`barriers` 기록).
- 장부: 배달 `{'A': {kind: 1}}` 한 번. 영상은 원본 폴더의 `pair_beam.mp4`, `trio_frame.mp4`에 있다.
- 같은 장면의 PR #164 탐침(`FormationTeacher`, 고정 출발 0.10 m)과 비교했다.
  - 운반 물리는 같은 수준이다: 미끄러짐 5.1 / 17.5 mm, 추적 오차 42 mm.
  - SIM 시간은 pair_beam 52.3 → 56.1초, trio_frame 54.3 → 59.7초로 늘었다. 차이는 늦게 오는 로봇을 기다린 시간이다.
  - trio_frame 배치 오차는 0.7 → 5.4 mm로 커졌다(허용 50 mm 안). 한 번씩이라 원인을 가르지 않았다.
- 조건당 1회다. 부분 실패 경로는 SIM에서 실행하지 않았다.
- 미기록 디버그 실행 1회: 커밋 전 트리, pair_beam, 영상 없음. 물리는 같았지만(배치 3.6 mm, `eq_active` 0) `phase_times['carry']`를 기록하지 않는 장부 버그로 Probe 판정이 false였다. 버그를 고친 뒤 `f9446a2`에서 위 기록 실행을 했다. `results.json`의 `debug_run_not_counted`에 있다.

## 4. 검증 범위와 하지 않은 것

- CI(`scripts/run_ci_tests.py`, 소스 `f9446a2`, 공용 잠금 안): 2313 passed, 10 skipped, 1 xfailed(기존 L1 strict xfail 그대로), 205 subtests passed. 새 테스트 파일은 41개 중 40 passed, 1 skipped(static_keepouts 없음).

- **구역 배송 E2E는 하지 않았다**(A2). 스모크는 열린 바닥의 단일 운반이고, 벽·문·다른 팀·구역 착지·LLM이 없다.
- 부분 실패 경로(hold_lower, cancel_retreat)는 단위 테스트로만 확인했고 SIM에서 주입하지 않았다.
- `static_keepouts` 연동은 hard-routes 로컬 파일로만 확인했다. 그 브랜치가 인터페이스를 바꾸면 A2에서 다시 맞춘다.
- heavy_crate와 단독 종류의 스모크는 하지 않았다(PR #164 탐침 결과만 있다).
- 부하가 매우 높은 때(1분 부하 평균 수백) 실행했다. 동기 SIM이라 SIM 시간 결과는 부하와 무관하지만 wall 시간은 비교에 쓰지 않는다.

## 5. A2 연결 계획

A2는 hard-routes가 병합된 뒤 새 브랜치에서 한다. 줄 번호는 main `ad18b93` 기준이며 hard-routes 병합 뒤 다시 찾는다.

### 5.1 `scripts/zone_teacher.py`

- **L1 수정:** `TeacherRobot._taken_by_peer`(250–263)의 "둘 다 `to_box`면 먼저 배정된 쪽 우선"(260–262)과 `align_box`/`to_box` 규칙을 지운다. 멈춤은 두 경우만 남긴다. 세 조건 공통이고, 단독 상자도 인원 1 팀으로 같은 규칙을 쓴다.
  - `RendezvousRule.occupancy`의 `station_blocked`: 물건 앞에서 더 가까운 몸이 정거장에 있다.
  - 물건이 이미 다른 작업의 CLOSE 이후 상태다(잡히는 중, 보이는 사건).
  - `test_zone_comm_boundary.py`의 L1 strict xfail은 이때 통과로 바뀐다.
- 새 `TeamCarryExecutor`(충돌을 피하려 새 파일 `scripts/zone_team_teacher.py` 권장):
  - `TeamJobLedger`와 `RendezvousRule`로 commit한다.
  - `FormationPlan`으로 접근·IK·운반 목표를 만든다.
  - 매 상태를 `PhaseBarrier`에 묶고, 모든 팔 이벤트는 같은 SIM 시각에 같은 길이로 예약한다. 이는 스모크의 `TeamJobTeacher`를 옮기는 것이다.
  - `ZoneTeacherExecutor.tick`이 유일한 물리 시계로 남고, 단독 `TeacherRobot`과 팀 실행기를 같은 틱에서 돌린다.
- 팀 작업에는 12초 강제 파지 진입(404)을 승계하지 않는다(정렬 시간 초과 = 실패 → `cancel_retreat`).
- `goal_occupied`에서 운반 중 즉시 집게를 여는 처리(380–384)도 승계하지 않는다(→ `hold_lower`).
- `discs_for`(473): 들린 물건을 장애물에서 빼는 현재 처리 대신, 다른 팀은 **팀 발자국 전체**(`TeamFootprint.at(item_pose)`)를 장애물로 넣는다. disc 계획기에는 발자국 부품의 외접 원 목록이나, hard-routes의 `rects` 인자에 부품 경계 사각형으로 준다.
- 팀 경로는 물건 pose (x, y, yaw) 하나로 계획하고, 매 구간을 `zone_team_footprint.swept_clear`(static_keepouts)로 검사한다.
  - 초기 A2는 직선 + 제자리 90° 회전만 쓴다.
  - 검사를 못 넘는 경로는 **접촉 전에** 거부한다(`cancel_retreat`).
- `resolve_blocks`(490)의 비키기: 운반 중인 팀에는 팀 전체의 검증된 후퇴만 허용한다.

### 5.2 `scripts/run_zone_dispatch.py`

- `goal_counts` → `goal_counts_v2`(옛 목표 그대로).
- `Slots`를 착지 영역 풀(`landing_layout`)로 바꾼다. 풀의 id는 L2처럼 모델에 보이지 않게 한다(`visible_own_jobs`는 item, zone, issued_at_sim_s, status만).
  - L5: independent에서 "빈 영역 없음" 거절도 일반 "stopped" 영수증으로 합친다.
- `assign`(192)을 선언 단위로 바꾼다. 로봇마다 RoleClaim을 교사에게 주면, 교사가 정거장으로 보내고 commit한다. `active[rid]`는 `robot_job` 참조가 된다.
- `collect_done`: 작업 종료 시 참여자 전원에게 같은 영수증을 준다. 목표 차감·영역 반납은 `TeamJobLedger`에서 물건당 한 번 한다.
- `resolve_label`(120)은 **같은 종류** 중 가장 가까운 물체로 묶는다. 빔·틀은 몸체 중심을 쓴다.
- 조건별 연결:
  - independent → `check_independent_claims`(로봇별)
  - dynamic → `check_dynamic_claims` + `remaining_need_items`
  - plan_first → `validate_team_plan`
  - 규칙 응답(fixture) 세 가지도 `{item, zone, role}`을 낸다.
- `referee` → `referee_v2`. 호출 쪽에서 물건별 최저점·기울기·손가락 접촉을 계산해 넘기며, 출력 전용이다.
- 시간:
  - 팀 운반의 기준 속도는 0.05 m/s(PR #164)다. 적재(x≈1)에서 구역(x 3–4.6) 사이 2–3.6 m와 회전·정렬·들기·내리기를 합하면 팀 작업 하나가 대략 3–5분(SIM)으로 예상된다(열린 바닥 1.3 m 경로가 56–60초였다. 구역까지의 거리와 문 통과는 A2에서 측정)이다.
  - 정거장 대기 최대 60초, 팀 운반 한도는 `3 × 기준 시간 + 30초`(PR #164)이다.
  - 섞인 목표(팀 2–3개 + 단독)에는 `--max-sim-s` 900 → 1800을 권장한다. 이 값은 A2 스모크로 다시 정한다.
  - 단독 이동 한도 120초는 그대로 둔다.

### 5.3 `harness/zone_coordination.py` / `harness/zone_solo.py`

- 선언 답 형식을 `{"item", "zone", "role"}`로 넓힌다(`validate_claim_reply`와 solo 판). 옛 형식도 받는다.
- 정적 과제 정보를 **세 조건 모두 같은 글**로 system에 넣는다(hard-routes의 `task['static_map_text']` 방식):
  - `task_static_info(goal)`: 종류별 필요 인원·역할·formation_id, 착지 영역
  - 팀 형성 규칙 문장: "팀 물건은 모든 역할 정거장에 같은 물건·같은 구역을 선언한 로봇이 서야 시작한다. 자기 도착 뒤 60초 안에 팀이 안 되면 멈춤 영수증을 받는다"
  - 모두 목록·목표·작성 지도에서 오며 정답 pose가 없다.
- 조건 블록의 차이는 통신 방식만 남긴다(C2 대칭).
- `check_claims`·`remaining_need`는 v2 함수로 대체하거나 감싼다.

### 5.4 인식(`claude/zone-cargo-perception`)

새 종류 라벨(`long_beam-1` 등)과 `pickup_items_still_visible` 키가 필요하다. 내 검사기는 이 키와 옛 `pickup_boxes_still_visible`를 모두 읽는다.

### 5.5 A2 스모크 E2E 계획

1. fixture 모드, hard-routes 지도(`zone_wide_door` 등)
   - 목표 예: `{'A': {'long_beam': 1}, 'B': {'heavy_crate': 1, 'red': 1}, 'C': {'can': 1, 'green': 1}}`
   - 세 조건 × seed 1개씩
   - 확인할 것: 심판 v2 목표 달성, 매 step `eq_active` 0, 이중 소속 0, 물건당 차감 1회, barrier 기록, 문 통과 swept 검사 기록
2. tri_frame은 세 로봇 전원이 필요하므로 따로 한 번 한다.
3. 실패 주입(한 참여자의 집게 열림 → `hold_lower`, 정거장 미도착 → `cancel_retreat`)을 fixture로 SIM 확인한다.
4. 그 뒤에만 실LLM 코호트를 사전 등록한다(ZC3 초안과 예산 연결).

## 원본·해시

`results.json`에 테스트·CI 결과, 스모크 요약, 원본 위치와 해시가 있다. 스모크 원본(`result.json`, `trace.jsonl`, `events.json`, `station-log.json`, `scene.xml`, MP4)은 기본 체크아웃의 `outputs/zone-team-jobs-20260925/`에 있다(로컬 전용이며 원격 백업이 아니다).
