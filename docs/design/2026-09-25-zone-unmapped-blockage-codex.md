# 2026-09-25 구역 미등록 장애물/가림(blockage) 설계 제안 — Codex 분석

- **작성:** Codex CLI (gpt-6-astra, reasoning xhigh). Claude 코디네이터가 2026-09-25에 위임한 읽기 전용 분석이다. 파일 수정·시뮬레이션·테스트·Git 쓰기·worktree 생성은 하지 않았다.
- **기준 SHA:** 로컬 main `12695bc`. `origin/claude/zone-hard-routes`는 당시 로컬에 없어, 로컬 `claude/zone-hard-routes` 브랜치의 `dc27695`를 `git show`로 읽어 인용했다(본문의 "H:" 표시 근거).
- **상태:** **제안, 미검증** (proposal, not verified)
- 아래 본문의 로컬 절대경로 링크(`/Users/changmin/projects/ugrp/...`, `/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/...`)는 작성자 기기와 작성 시점 경로를 가리킨다. 이후 커밋에서 줄 번호·구조가 바뀌었을 수 있다.

## 이후 진행 상황 (기록 시점 2026-09-25)

이 보고서가 제안한 미등록 장애물/가림 설계는 **아직 구현하지 않았다**. 보고서 본문이 지적한 대로 `unexpected_obstacles`는 설정에만 있고 XML 생성기가 반영하지 않는다. 이 작업은 보고서의 순서표(① hard-routes → TeamJob 통합 → ⑤ 세 조건 파일럿)에 따라 **A2(TeamJob 실행기 연결) 이후로 계획**돼 있으며, 2026-09-25 시점 착수 전이다.

---

**권고안은 `zone_wide_two_doors`에서 짧은 문을 막고, 로봇이 RGB로 발견한 뒤 메시지로 동료의 헛걸음을 줄이는 구성입니다.** 다만 현재 hard-routes 지도는 문·통로의 TOP 가시성을 유지하므로, 낮은 장애물을 놓는 것만으로는 비공개 정보 조건이 성립하지 않습니다.

검토 기준은 main `12695bc`입니다. `origin/claude/zone-hard-routes`가 로컬에 없고 원격 조회도 확인하지 못해, 로컬 `claude/zone-hard-routes`의 `dc27695`를 `git show`로 읽었습니다. 아래 **H** 근거는 이 커밋입니다. 파일 변경·시뮬레이션·테스트 실행·Git 쓰기·worktree 생성은 하지 않았습니다.

**1. 위치·등장 방식·가시성**

| 구성 | 제안 | 연구상 용도 |
|---|---|---|
| 주 조건 | `door_narrow` 중심 `(2.20, 0.05)`, 폭 0.50m의 통행을 차단. 남쪽 `door_wide` `(2.20, −2.625)`, 폭 1.00m는 개방 | 먼저 발견한 로봇의 보고로 다른 로봇이 남쪽 우회로를 선택 |
| 역방향 조건 | 넓은 문을 막고 좁은 문은 개방 | 특정 문·진행 방향에 대한 과적합 확인. 해당 화물·대형이 좁은 문을 통과할 수 있을 때만 사용 |
| 단일 문·복도 | 일시 차단 후 예정된 시각에 재개방 | 불필요한 접근·대기 감소 평가. 영구 차단하면 대안 경로가 없어 우회 성공 비교에 부적합 |

문 위치·폭은 [H:지도:315](/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/maps/zones/zone_wide_two_doors.json:315), 단일 복도와 대피 공간은 [H:zone_arena.py:136](/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/sim/zone_arena.py:136)에 정의돼 있습니다.

- **진짜 비공개 조건:** 우선 기존 가림 영역을 조사하되, 현재 테스트는 통행 가능한 문·바닥의 TOP 가시성을 검사합니다. 벽 뒤의 얇은 사각만으로 문 전체를 막는 장애물을 숨길 수 있다고 가정하면 안 됩니다. [H:test_zone_hard_routes.py:101](/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/tests/test_zone_hard_routes.py:101)
- 기존 구조로 불가능하면, **정적 지도에 알려진 고정 덮개가 있는 문**을 별도 지도 버전으로 제안합니다. 로봇·화물 위의 덮개 아래에 낮은 장애물을 두고, 자기 카메라는 옆에서 보게 합니다. 덮개는 정상·차단 시행 모두에 존재하고 장애물만 미등록 상태입니다. 카메라 위치/FOV·영상 마스킹은 바꾸지 않습니다. 덮개의 높이·그림자까지 모델링하며, 단순한 바닥 장애물로 처리하지 않습니다.
- **TOP 공개 조건도 유지**합니다. TOP만으로 충분한 상황에서 통신의 추가 이득이 작거나 비용만 생기는지 보는 대조군입니다. “TOP 검출기가 놓쳤다”는 이유로 비공개라고 분류하지 않습니다.
- **처음에는 시작 시 차단**, 이후 중간 등장으로 확장합니다. 중간 등장은 조건별 작업 순서가 아닌 사전 고정 SIM 시각에 발생시키고, 미리 생성한 물리 장애물을 이동시킵니다. 로봇과 겹쳐 생성하지 않습니다. 안전상 등장 지연·취소가 생기면 실제 시각과 이유를 평가 기록에 남기고 사후 제외하지 않습니다.
- 지도·시각·최초 관측자·진행 방향을 교차 배정합니다. 장애물 위치·일정·정답 상태는 `setup_only`와 평가 기록에만 둡니다.

**2. 허용 입력만 사용하는 인식 경로**

현재 `zone_perception.py`는 TOP의 **색 상자 검출기**이며 통로 차단 검출기는 아닙니다. 또한 runner는 주로 idle 로봇에게 질문하므로, 이동 중 발견·보고 경로가 먼저 필요합니다. [zone_perception.py:47](/Users/changmin/projects/ugrp/harness/zone_perception.py:47), [run_zone_dispatch.py:276](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:276)

제안 경로는 다음과 같습니다.

`자기 RGB + 동일한 공용 TOP + 정적 지도·보정 + 자기 발행 명령 → 자기 위치/문 ID 추정 → 통행 상태 belief → 경로 선택·메시지`

belief에는 `passage_id`, 방향, `unknown/suspected/blocked/clear`, 관측 시각, 신뢰도, RGB 근거, 직접 관측/동료 보고의 구분을 둡니다. 좌표는 영상에서 추정하며 simulator 좌표·depth·segmentation·접촉 판정을 입력하지 않습니다.

- 한 프레임의 물체 존재만으로 “막힘”을 확정하지 않습니다. 통행 폭과 화물 외형을 함께 판단하고, 필요하면 정지 후 재관측합니다.
- 명령을 냈는데 영상 변화가 작다는 사실은 **정체의 보조 증거**입니다. 장애물·미끄러짐·동료 대기를 구분하지 못하므로 차단의 단독 근거로 쓰지 않습니다.
- 대표 오탐은 그림자·바닥 표시·일시적으로 지나가는 동료·화물, 문 ID 오인입니다. 집게/적재물 가림과 낮은 물체의 원근 왜곡은 미탐 위험입니다.
- 개발 영상에서 신뢰도 기준·반복 확인·유효기간을 고정합니다. 오래된 보고는 `unknown`으로 낮추며, 시간이 지났다는 이유만으로 `clear`가 되지 않습니다. 과거 영상 기억을 쓰면 세 조건 모두에 동일하게 적용합니다.

**3. 로봇의 경로 결정과 교사의 안전 실행을 분리**

현재 작업 인터페이스는 사실상 `box_body + slot_xy`이고, 교사가 전역 A* 경로를 선택합니다. 여기에 실제 미등록 장애물을 전부 넣으면 **보지 못한 로봇도 처음부터 우회**하게 됩니다. [zone_teacher.py:231](/Users/changmin/projects/ugrp/scripts/zone_teacher.py:231), [H:zone_teacher.py:329](/Users/changmin/projects/ugrp-worktrees/zone-hard-routes/scripts/zone_teacher.py:329)

| 결정 | 정보와 책임 |
|---|---|
| 어느 문을 지날지, 어떤 통로를 피할지, 기다릴지 | **로봇**이 자기 RGB·지도·허용된 수신 메시지로 결정 |
| 경로 요청 | 로봇이 `job_id`, 문/통로 순서, 회피할 통로, belief 버전·근거를 제출. 위치·중간점은 영상 추정 또는 정적 지도에서 생성 |
| 요청한 경로 안의 정밀 주행·IK·충돌 방지 | **교사**가 실제 자기 자세·관절·화물·충돌 형상을 사용 |
| 숨은 장애물 때문에 다른 문 선택 | 교사가 임의로 수행하지 않음. 로봇의 새 경로 요청 필요 |

교사는 선택된 통로 안에서 안전한 국소 움직임을 계산할 수 있습니다. 그러나 아직 먼 숨은 장애물 때문에 미리 경로를 바꾸거나, 즉시 “경로 없음”을 반환해서는 안 됩니다. 가까운 충돌 위험에는 제한된 안전 영역에서 감속·정지하고, 통과가 불가능하면 그 자리에서 유지합니다. **안전 정지는 차단 인식으로 자동 변환하지 않습니다.**

차단 판정·우회 결정·임무 단계 변경·동료 통보는 다음 정기 RGB 판단에서 발생해야 합니다. 교사의 내부 운동 단계와 정답 진단은 실행·평가 영역에만 남깁니다. 잘못된 RGB belief 때문에 로봇이 우회하더라도 교사가 정답으로 이를 “교정”하지 않습니다.

특히 기존 `teacher_path_blocked` 등의 결과가 `own_jobs.status`와 전원 깨우기로 돌아오는 경로를 먼저 차단해야 합니다. “같은 영수증을 모두에게 제공”하는 것만으로 누설이 해결되지 않습니다. [run_zone_dispatch.py:224](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:224), [경계 감사:47](/Users/changmin/projects/ugrp/experiments/2026-09-25-zone-comm-boundary-audit/README.md:47)

**4. 조건별 정보와 공통 신호**

공통으로 같은 지도·카메라·모델·예산·관측 주기·자기 기억·로컬 경로 재계획 능력을 제공합니다. TOP는 같은 시각의 네 장을 모두에게 전달합니다.

| 조건 | 받는 통신 | 실행 중 차단 대응 |
|---|---|---|
| `independent` | 없음. 동료 belief·선언·영수증·계획도 없음 | 자기 영상으로 발견한 뒤 자기 경로 변경 |
| `plan_first` | 시작 전 실제 협상 메시지와 합의 | 배정은 유지하되 자기 영상에 따른 로컬 우회 허용. 새 동료 보고는 없음 |
| `dynamic` | 시작 전·실행 중 실제 전달된 메시지 | 자기 관측 또는 수신 보고로 경로 변경 |

이는 현재의 **실행 중 재판단이 없는 plan_first를 확장한 새 버전**이므로 기존 ZC 결과와 섞으면 안 됩니다. 시작부터 막혀 있어도 실행 후에야 발견된다면 plan_first는 미리 알 수 없습니다. 초기 정보 공유 효과도 보려면 모든 조건에 같은 사전 관측 시간을 주는 별도 조건이 필요합니다. [run_zone_dispatch.py:246](/Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:246)

통신 효과를 분리하려면 dynamic의 호스트 게시판·동료 선언 기반 거절·정답 사건 깨우기를 제거하거나 별도 비교군으로 남깁니다. 수신자의 belief는 **실제로 전달된 메시지**로만 갱신합니다. 공통 주기에 inbox를 읽으면 메시지 없는 조건과 판단 기회도 맞출 수 있습니다. 현재 비교가 메시지만의 효과가 아니라는 근거는 [경계 감사:76](/Users/changmin/projects/ugrp/experiments/2026-09-25-zone-comm-boundary-audit/README.md:76)에 있습니다.

TeamJob의 최소 시작·정지·허가 동기화는 세 조건에서 동일하게 유지하되, ACK·예약 응답·공유 객체에 차단 위치나 동료 belief를 싣지 않습니다. **첫 차단 코호트는 단독 운반으로**, 이후 공동 화물로 확장하면 필수 공동 동작과 정보 전달의 효과를 구분하기 쉽습니다. [research_todo.md:50](/Users/changmin/projects/ugrp/docs/research_todo.md:50)

**5. 측정 지표**

| 지표 | 정의 |
|---|---|
| 헛걸음 | 마지막 우회 선택 지점에서 막힌 분기로 진입했다가 되돌아온 횟수·거리. 최초 발견 전/후, 메시지 수신 전/후를 분리 |
| 우회 거리 | 실제 이동거리와 사후 계산한 장애물 포함 최단 가능 경로의 차이. 장애물 때문에 필수적으로 늘어난 거리도 별도 표시 |
| 시간 손실 | 잘못된 접근·후퇴·대기·재관측 시간. 전체 완료 SIM 시간과 추론 대기 포함 wall 시간을 함께 보고 |
| 정보 전달 효과 | 발견→발신→수신→belief 변경→경로 변경 지연, 수신자가 자기 눈으로 보기 전에 피한 접근 수 |
| 메시지 정확성 | 문 ID·방향·차단 여부·관측 시각·화물별 통과 가능성의 정확도, 오경보·누락·만료 보고·근거 없는 확정 표현 |
| 전체 성과·비용 | 임무 성공률, 충돌·낙하·안전 개입, 호출·토큰·메시지 수. 실패 시행의 짧은 시간을 속도 이득으로 계산하지 않음 |

정답은 이 지표의 **사후 평가에만** 사용합니다. 사실이 맞는 메시지와 송신자가 실제 영상 근거를 가진 메시지를 구분합니다. 인과 효과 확인에는 동일 정책의 메시지 전달/차단 비교가 필요하며, 실제 상태 복원이 없는 로그 사례만으로 특정 메시지의 효과를 확정하지 않습니다.

**6. 구현 순서와 통과 기준**

| 순서 | 파일별 접점 | 다음 단계로 넘어갈 기준 |
|---|---|---|
| ① hard-routes → TeamJob 통합 | `maps/zones/`, `harness/static_keepouts.py`, TeamJob 인터페이스·실행 어댑터 | 정상 상태의 경로·대체 경로를 해당 화물 외형으로 통과. 공통 실행·최소 동기화 계약 고정 |
| ② 정보 경계·경로 요청 | `harness/zone_coordination.py`, `zone_solo.py`, `scripts/run_zone_dispatch.py`, `zone_teacher.py` | L1/L4 및 게시판·깨우기 경계 정리. 같은 허용 입력에서 숨은 정답만 바꿔도 국소 안전 영역 진입 전 요청·경로 선택·깨우기가 동일 |
| ③ 시작 시 장애물·가림 | `sim/zone_arena.py`, `zone_scene.py`, 새 지도/시나리오 설정 | 물리 차단·대안 경로·카메라 불변 확인. 차단 유무를 바꾼 영상에서 모든 TOP에는 단서가 없고 자기 RGB에는 식별 가능한 근거가 존재 |
| ④ RGB belief·실행 중 관측 | `harness/zone_perception.py` 또는 별도 차단 인식 모듈, 위 runner·프롬프트·메시지 스키마 | 정상/차단/그림자/동료 통과 fixture에서 인식 검증. 메시지 없는 로봇에 타인의 belief가 전달되지 않음 |
| ⑤ 세 조건 파일럿 → 중간 등장 → 공동 화물 | `tests/test_zone_comm_boundary.py`, hard-routes 테스트, 새 blockage 테스트, `configs/simulation_workflows.json`, 평가·TensorBoard 변환 | 정상·TOP 공개·비공개 조건을 대응 비교. 실패 포함 전체 분모, 실제 요청/전송/RGB/경로/평가 기록 연결. 그 뒤 일정 변화·TeamJob 화물로 확장 |

현재 `unexpected_obstacles`는 설정에만 존재하고 XML 생성기는 이를 넣지 않으므로 **설정 목록 채우기만으로 구현되지 않습니다**. 또한 `ZoneScene`이 에피소드를 재구성·대조하므로 시나리오 인자도 함께 연결해야 합니다. [zone_arena.py:188](/Users/changmin/projects/ugrp/sim/zone_arena.py:188), [zone_arena.py:212](/Users/changmin/projects/ugrp/sim/zone_arena.py:212), [zone_scene.py:40](/Users/changmin/projects/ugrp/sim/zone_scene.py:40)

이번에는 설계·코드 근거만 확인했습니다. 실제 TOP 비가시성, 자기 RGB 검출 가능성, 차단 물리, TeamJob 통합 완료 여부는 아직 검증된 결과가 아닙니다.
