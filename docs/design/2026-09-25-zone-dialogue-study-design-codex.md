# 2026-09-25 한국어 로봇 대화의 작업 효율 효과 — 통합 연구 설계 (Codex)

- **작성:** Codex CLI (reasoning xhigh, 읽기 전용). Claude 코디네이터가 위임한 설계 검토다. 파일 수정·시뮬레이션·모델 호출은 하지 않았다(본문 서두).
- **날짜:** 2026-09-25
- **상태:** 설계 제안 (검증 전)
- **읽은 ref(본문 표 기준):** 로컬 기본 체크아웃 `main` `0add360`, `origin/main` `6d80e3e`, PR 169 `origin/claude/zone-team-a2` `143360d`, PR 170 `origin/claude/zone-rgb-outcome` `c30091b`, PR 173 `origin/claude/zone-hard-routes` `4789d93`, PR 174 `origin/claude/zone-cargo-perception-v2` `972dc36`. GitHub 실시간 조회는 실패했으며 로컬 ref 스냅샷 분석이다.
- **이후 결정과의 관계:** 이 문서는 사용자의 이후 결정보다 먼저 작성됐다. 이후 결정은 자기 카메라 전용 실행기, 벽 AprilTag, 손목 카메라만 사용하는 것이다([decision_log 2026-09-25 조건 변경 절](../decision_log.md#2026-09-25--주-연구-조건-변경-무통신동적-통신ai-지휘)의 후속 항목 (7)). 본문이 교사 실행기를 공통 실행기로 전제하는 부분은 자기 카메라 트랙(PR #176/#177)이 대체한다. 지휘자 형태도 이후 (6)에서 "로봇 한 대 겸임, r1/r2/r3 순환, 허브-스포크"로 정해졌다. 본문의 `r1` 고정 권고와 별도 commander(③b) 보조 조건은 그 결정으로 대체된다(별도 지휘 AI는 전지적 참고 상한으로만 유지).
- **이전 초안:** TOP 입력을 허용하고 `r1`을 지휘자로 둔 이전 초안(v0)은 따로 커밋하지 않았다. 그 내용과 폐기 사항은 이 문서 1절에 요약돼 있다.
- **링크 주의:**
  - 본문의 `/private/tmp/...` 링크(이전 초안)와 `/Users/changmin/projects/ugrp/...` 링크는 작성자 기기의 로컬 경로다. `/private/tmp` 파일은 임시 작업 파일이라 보존하지 않는다.
  - GitHub 링크는 저장소 이름이 옛 slug `kcm0127-dotcom/ugrp`로 적혀 있다. 현재 slug는 `cmkang131/UGRP-Multi-Robot-Collaboration-Project`이며, 확인 시점에 옛 slug가 현재 저장소로 연결됐다. 링크는 고쳐 쓰지 않았다.
- 아래 본문은 Codex 출력 원문 그대로다. 비밀정보·토큰은 없었다. 끝의 Codex 세션 ID는 로컬 재개용 식별자라 출처로 남겼다.

---

**UGRP 한국어 로봇 대화의 작업 효율 효과 — 읽기 전용 연구 설계 보고서**  
기준일: 2026-09-25

권고하는 주 비교는 **① 무통신, ② 분산 자유 한국어 대화, ③ `r1` 지휘 겸임, ④ 분산 정형 메시지**다. 모든 조건에 매 호출 동일한 정적 지도·주문서·자기 RGB·자기 명령 이력을 제공하고, 추론과 발화에 결정론적 SIM 시간 비용을 부과한다. 별도 commander와 전방위 영상 commander는 보조 비교로 분리한다.

본실험의 선행 조건은 TOP 제거에 따른 **자기 RGB 인식·위치 추정·완료 판단·실행기 입력 경계의 재구성**이다. 프롬프트에서 TOP 이미지만 삭제한 상태로는 새 연구 계약을 만족하지 않는다.

파일 수정, fetch, 브랜치 생성, 테스트, 시뮬레이션, 모델 호출은 하지 않았다. GitHub 실시간 조회는 연결 제한으로 실패했으므로, 아래는 **로컬에 저장된 ref의 스냅샷 분석**이다. PR의 현재 병합 여부는 확인하지 못했다.

| 인용 표기 | 검토한 ref | SHA |
|---|---|---|
| 로컬 | 기본 체크아웃 `main` | `0add360` |
| Main | `origin/main` | `6d80e3e` |
| A2 | `origin/claude/zone-team-a2`, PR 169 | `143360d` |
| Outcome | `origin/claude/zone-rgb-outcome`, PR 170 | `c30091b` |
| Routes | `origin/claude/zone-hard-routes`, PR 173 | `4789d93` |
| Cargo2 | `origin/claude/zone-cargo-perception-v2`, PR 174 | `972dc36` |

**1. 기존 설계에서 유지할 것과 바꿀 것**

기존 보고서의 `r1` 지휘 겸임 권고, 게시판·호스트 중재 제거, 행동과 메시지 분리, `plan_first` 재현 보존은 유지한다. 반면 **공용 TOP 입력, 비용 없는 SIM 정지 협상창, 정형 통신을 보조 조건으로만 두는 구성은 폐기**한다. 이전 보고서는 TOP 허용과 협상창 중 물리 정지를 명시했으므로 이번 사용자 지시로 대체한다. [이전 보고서:46](</private/tmp/claude-501/-Users-changmin-projects-ugrp/ecd247bf-a1a7-45d6-9190-dad34be56468/scratchpad/codex-leader-design-v0.md:46>), [이전 보고서:115](</private/tmp/claude-501/-Users-changmin-projects-ugrp/ecd247bf-a1a7-45d6-9190-dad34be56468/scratchpad/codex-leader-design-v0.md:115>)

저장소 지침은 정적 지도와 고정 카메라 보정을 허용하되, 실시간 정답을 제어·단계 전환·성공 통보에 사용하지 않도록 한다. 이번 지시는 기존 TOP 허용 부분을 더 좁혀 **주 조건의 로봇 입력을 자기 RGB로 제한**한다. 카메라 배치/FOV 유지와 weld OFF도 그대로 적용한다. [AGENTS.md:11–15](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/AGENTS.md#L11-L15)

읽은 세 설계 문서에서는 다음을 계승한다.

| 문서 | 계승할 내용 | 이번에 대체할 내용 |
|---|---|---|
| 한국어 대화 | 무통신·정형·자유 언어 비교, 행동/발화 분리, 메시지 전달과 결정 연결 | TOP 공용 관측, SIM 추론 비용을 미정으로 둔 부분 |
| 팀 운반 | 화물당 하나의 TeamJob, 상보 역할, 원자적 시작, 부분 실패 시 공동 정지·하강 | 교사 정답으로 준비·완료를 판단하는 경로를 본실험에 사용하는 것 |
| 미등록 막힘 | 비공개 사건, 개인 belief, 보고에 따른 경로 변경, 실패 포함 평가 | TOP에서도 장애물을 숨기기 위한 덮개·가림 필요성 |

근거: [한국어 설계:24–50](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/docs/design/2026-09-25-zone-korean-dialogue-codex.md#L24-L50), [팀 운반 설계:35–60](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/docs/design/2026-09-25-zone-team-carry-codex.md#L35-L60), [막힘 설계:18–62](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/docs/design/2026-09-25-zone-unmapped-blockage-codex.md#L18-L62)

이제 TOP는 평가 전용이므로, 비공개성의 기준은 **발견 이전에 다른 로봇의 자기 RGB에 단서가 있었는가**다. TOP에 보였다는 이유로 정보 비대칭 사례에서 제외할 필요는 없다.

**2. 조건, 권한, 입력 allowlist**

주 조건의 공통 입력 `B`를 다음처럼 정의한다.

- 매 호출 포함되는 정적 지도 본문과 지도 도식.
- 시나리오 설정에서 만든 불변 주문서.
- 호출 시각의 자기 RGB와, 동일 정책으로 보존한 제한된 자기 관측 이력.
- 자기 발행 명령, 제출한 작업, 명령 큐·타이머 상태.
- 위 허용 입력만으로 만든 개인 belief와 기억.
- 조건에서 허용한 실제 수신 메시지.

여기서 실행기 상태는 `command_issued`, `queue_empty`, `hold_requested`, `local_timeout` 같은 **자기 명령 처리 상태**다. 교사가 정답으로 판단한 `grasp_success`, `placed`, 동료 작업 종료, 실제 도착 여부는 포함하지 않는다.

| 조건 | 각 actor의 입력 | 통신·배정 권한 |
|---|---|---|
| **① `independent`** | 각 로봇의 `B`, inbox 없음 | 각자 자기 행동 선택. 고수준 송수신 0 |
| **② `peer_ko`** | 각 로봇의 `B` + 실제 수신 한국어 | mesh. 직접 전달·방송, 요청·협상·정정 가능 |
| **③a `leader_robot_ko`** | `r1`도 자기 `B`만 받음. followers도 자기 `B` | `r1↔r2`, `r1↔r3` star. `r1`은 지휘와 자기 운반을 겸함 |
| **③b `commander_ko`** | commander: 정적 지도·주문서·한국어 보고와 자신이 보낸 지시 이력만. 로봇: 자기 `B` | 별도 commander와 세 로봇 간 star. commander는 영상·동료 raw 명령·호스트 작업표를 받지 않음 |
| **④ `peer_structured`** | ②와 같은 관측·개인 기억, 정형 inbox | ②와 같은 mesh·수신자 선택권. 메시지 의미는 고정 schema로만 전달 |
| **R `central_rgb_reference`** | commander가 지도·주문서·세 로봇 RGB를 받음. `R+TOP`는 TOP도 받는 별도 변형 | commander 하나가 전원 계획. 로봇에는 LLM 없음 |

①–④에서 금지할 입력은 TOP 원본뿐 아니라 **TOP에서 만든 좌표·물체 라벨·구역별 개수·완료 판정·요약문**도 포함한다. 다른 로봇의 RGB와 raw 명령 로그, 전역 작업표, 숨은 사건 일정, simulator body ID도 금지한다.

**주 조건 ③에는 `r1` 겸임을 권고한다.** 세 로봇·세 LLM이라는 자원 구성을 유지하면서 지휘 집중의 효과를 비교하기 쉽다. `r1`의 위치 편향은 논리 ID와 실제 시작 위치·역할의 대응을 seed별로 순환시켜 줄인다.

③b는 지휘 전담 문맥과 네 번째 LLM을 추가하며, 지휘 중에도 운반 로봇 세 대가 계속 움직일 수 있다. 따라서 ③a와의 차이는 중앙화만이 아니라 **전담 지휘 자원과 물리 작업 분담의 차이**다. 같은 팀 전체 호출·토큰 상한을 적용하더라도 이 구조 차이는 남으므로 보조 조건으로 보고한다.

followers는 한국어 명령을 직접 해석하는 LLM을 유지한다. 모호한 명령 질문, 시각 근거와 충돌하는 명령 거절, 안전 정지, 관측 보고는 허용한다. 호스트가 한국어를 파싱해 follower 행동을 대신 만들지 않는다.

R은 정보가 풍부한 중앙 제어의 **참조 성능**이다. 단일 모델의 오류나 병목 때문에 실제 상한보다 낮을 수 있으므로 수학적 최적 상한으로 표현하지 않는다. `R`과 `R+TOP`도 합산하지 않는다.

**3. 정적 지도·주문서 직렬화와 한국어 프롬프트**

**지도는 JSON을 기준으로 하고, 같은 파일에서 생성한 도식을 함께 제공한다.**

| 방식 | 장점 | 한계 |
|---|---|---|
| JSON만 | ID·통로 연결·폭·좌표가 정확하고 감사 가능 | 공간 관계를 읽는 부담 |
| 도식만 | 전체 공간 구조 파악이 쉬움 | 작은 글자·정밀 수치 판독 오류 |
| **JSON + 도식** | 정확한 참조와 공간적 이해를 함께 제공 | 입력 비용 증가 |

Routes에는 버전별 지도 JSON을 읽고 작성된 정의와 일치하는지 검사하는 경로가 있다. 이를 새 입력 생성의 출발점으로 사용할 수 있다. [Routes `sim/zone_arena.py:253–259`](https://github.com/kcm0127-dotcom/ugrp/blob/4789d932eb0ca02dbf3d40eaef26566d34823c85/sim/zone_arena.py#L253-L259)

제안하는 지도 생성 규칙은 다음과 같다.

- 원본 지도 파일 SHA-256, 공개 JSON 투영본 SHA-256, 도식 SHA-256을 각각 보존한다.
- 공개 JSON에는 벽·문·복도·구역 `A/B/C`·pickup bay/slot·정적 통행 조건만 넣는다.
- 도식은 지도 JSON에서 결정론적으로 그린다. MuJoCo 현재 장면을 캡처하지 않는다.
- 로봇 위치, 현재 화물 위치, 점유 상태, 발견된 장애물을 도식에 자동 표시하지 않는다.
- 매 호출에 지도 내용과 도식을 다시 포함한다. 이전 호출에서 봤다는 이유로 생략하지 않는다.
- 지도는 끝까지 불변이다. 발견한 변화는 로봇별 belief에 기록하며 공유 지도 파일을 수정하지 않는다.

**주문서는 시뮬레이터 생성 전의 시나리오 설정에서 만든다.** 초기 배치 생성기도 같은 선언을 소비하도록 하여, 생성된 simulator state를 역으로 읽어 주문서를 채우는 경로를 없앤다. 숨은 사건 일정은 별도의 평가·설정 영역에 둔다.

다음은 **신규 schema 예시**이며, `P1-2` 같은 pickup slot은 새 지도 버전에 정의해야 한다.

```json
{
  "schema": "ugrp.zone_order.v1",
  "map_id": "zone_wide_two_doors",
  "map_sha256": "<map-file-sha256>",
  "orders": [
    {
      "order_id": "order-1",
      "item_ids": ["long_beam-1"],
      "kind": "long_beam",
      "count": 1,
      "required_robots": 2,
      "destination_zone": "A",
      "initial_location": {
        "pickup_bay": "P1",
        "slot": "P1-2"
      }
    }
  ]
}
```

`initial_location`은 “시작할 때 그곳에 배치하도록 정한 위치”다. 이후 이동·낙하·회수에도 갱신하지 않는다. 정확한 simulator 물체 좌표·body 이름은 넣지 않는다.

현재 A2의 라벨은 첫 TOP 검출을 정렬해 만들고, 공개 라벨에 RGB 추정 좌표와 손잡이 좌표를 넣는다. 새 연구에서는 이를 **설정 기반 주문 ID와 자기 영상 기반의 불확실한 대응 관계**로 대체해야 한다. [A2 `harness/zone_perception_v2.py:50–67`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_perception_v2.py#L50-L67), [동일 파일:99–107](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_perception_v2.py#L99-L107)

외관이 같은 물건을 시야에서 잃으면 정확한 개체 ID를 재확정하지 못할 수 있다. 주문을 종류별 대체 가능 과제로 할지, 특정 개체 배송으로 할지 사전에 고정하고, 후자의 경우 식별 불확실성을 실패·재관측 비용에 포함한다.

**입력 예산의 개발 시작값**

| 구성 | 제안 상한 |
|---|---:|
| 공통 한국어 지시·행동 schema | 1,000 text tokens |
| 정적 지도 JSON | 1,800 |
| 주문서 | 800 |
| 자기 명령·belief | 1,400 |
| 수신 메시지·개인 기억 | 2,000 |
| 여유 | 1,000 |
| **텍스트 합계** | **8,000** |
| 이미지 | 지도 도식 1장 + 현재 자기 RGB 1장 |
| 호출 출력 | 768 tokens |

이는 실측값이 아닌 시작안이다. 선택한 모델의 실제 토큰 계산으로 검증하고 본실험 전에 고정한다. 지도·주문서가 상한을 넘으면 조용히 자르지 말고 시나리오 사전검사에서 거절한다. 이미지 토큰과 캐시 토큰은 별도 기록한다.

과거 자기 영상 비교가 필요하면 동일한 길이의 관측 기억을 전 조건에 제공한다. 전체 프레임 이력을 늘리는 변경은 별도 입력 프로파일로 버전 관리한다.

**자기 위치는 정답 대신 belief로 유지한다.**

```json
{
  "region": "unknown",
  "last_visual_anchor": "door_narrow",
  "last_requested_destination": "A",
  "last_visually_confirmed_region": "pickup",
  "confidence": "low",
  "sources": ["own-r1-0042", "command-r1-0018"]
}
```

마지막 목적지는 도착 증거가 아니다. 자기 RGB에서 문·벽·구역 표시를 지도와 대응시키고, 발행 명령으로 예상한 이동을 보조 근거로 사용한다. 영상과 예상이 어긋나면 불확실성을 높이고 재관측한다. 호스트가 현재 pose로 보정하지 않는다.

**공통 프롬프트 골격**

```text
당신은 로봇 {robot_id}입니다.

목표:
주문서의 물건을 지정된 A/B/C 구역으로 배송하십시오.
안전하게 완료한 배송을 우선하고, 추론·발화 대기를 포함한 SIM 시간을 줄이십시오.

매 호출 제공된 정적 지도와 주문서는 초기 계획 정보입니다.
현재 위치, 현재 재고, 배송 완료를 보장하지 않습니다.

현재 상황의 근거는 자기 RGB, 자기 발행 명령과 허용된 수신 메시지뿐입니다.
명령 발행을 실제 이동·파지·배달 성공으로 간주하지 마십시오.
보이지 않거나 식별할 수 없으면 unknown으로 유지하십시오.

r1/r2/r3, item ID, A/B/C, role, passage ID, JSON keys와 enum은
번역하거나 바꾸지 마십시오.
자유 메시지가 허용된 조건에서는 본문을 한국어로 작성하십시오.

자기 행동과 다른 로봇에게 보낼 메시지를 분리하십시오.
메시지가 다른 로봇의 행동을 직접 실행시키지는 않습니다.

출력:
request_id, action, decision_sources, messages를 포함한 JSON.
```

조건별 추가 지시는 다음처럼 제한한다.

| 조건 | 추가 지시 |
|---|---|
| ① | “메시지를 보내거나 받을 수 없습니다. `messages`는 빈 배열이어야 합니다.” |
| ② | “필요하면 동료에게 한국어로 관측·의도·질문·요청·정정을 전달하십시오. 각 로봇은 자기 행동을 결정합니다.” |
| ③a leader | “팀 배정을 한국어로 지시하고 보고를 확인하십시오. `action`은 자신의 행동만 지정합니다.” |
| ③a follower | “`r1`의 한국어 지시를 해석하십시오. 질문·거절·관측 보고는 `r1`에게 보내십시오.” |
| ③b commander | “영상이나 현재 작업표는 제공되지 않습니다. 지도·주문서·실제 한국어 보고만으로 지시하십시오.” |
| ④ | “메시지는 지정된 enum·ID·수치 필드로만 작성하십시오. 자유 문자열이나 우회 인코딩은 금지합니다.” |

②·③의 메시지 봉투는 `message_id`, `sender`, `recipients`, `reply_to`, 시각과 한국어 `text`로 구성한다. 봉투를 제외한 명령·보고의 의미는 한국어 본문에 둔다.

④의 메시지는 다음과 같은 고정 필드를 사용한다.

```text
act: propose | request | accept | reject | inform | correct | yield | cancel
item, zone, role, passage, location_ref
state: unknown | suspected | clear | blocked | present | absent | held | placed
confidence: low | medium | high
observed_at_sim_s, reply_to, recipients
```

ID는 공개 지도·주문서와 실제 메시지 ID에서만 선택한다. `text`, `reason`, 임의 `other` 문자열로 자유 언어를 우회시키지 않는다.

**4. 자기 RGB 인식과 완료 판정의 공백**

현재 인식·실행 경로에서 확인한 사실은 다음과 같다.

| 현재 경로 | 확인한 경계와 새 설계에 미치는 영향 |
|---|---|
| `zone_perception` | 고정 TOP JPEG와 보정으로 색 상자를 검출·추적한다. 자기 RGB 인식기로 그대로 사용할 수 없다. [Main `harness/zone_perception.py:1–7, 47–85`](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/harness/zone_perception.py#L1-L85) |
| `zone_color_boxes` | `detect_own` 경로가 있지만 바닥 위 색 상자 모델이다. 색은 개체 ID가 아니라 종류를 식별한다. [Main `harness/zone_color_boxes.py:7–28`](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/harness/zone_color_boxes.py#L7-L28) |
| `zone_cargo_perception` | TOP와 정적 catalogue로 화물 종류·형상을 인식한다. [Cargo2 `harness/zone_cargo_perception.py:1–29`](https://github.com/kcm0127-dotcom/ugrp/blob/972dc36bbdfbbfe827db79976282765858772804/harness/zone_cargo_perception.py#L1-L29) |
| PR 174 `top_cargo_v2` | TOP 입력을 유지하면서 평행 빔 병합·틀 내부 상자 억제를 수정한다. 자기 RGB 전환은 아니다. [Cargo2 `harness/zone_cargo_perception_v2.py:1–22`](https://github.com/kcm0127-dotcom/ugrp/blob/972dc36bbdfbbfe827db79976282765858772804/harness/zone_cargo_perception_v2.py#L1-L22) |
| A2 runner | 현재 `top_cargo_v1` 경로를 import하고 시작 TOP에서 라벨을 만든다. PR 174가 존재한다는 사실만으로 A2가 v2를 사용하는 것은 아니다. [A2 `scripts/zone_dispatch_v2.py:40–42`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L40-L42), [동일 파일:179–183](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L179-L183) |

최소한 다음 네 가지 자기 RGB 판단이 필요하다.

| 판단 | 허용 근거 | 반드시 남길 불확실성 |
|---|---|---|
| **올바른 물건을 들었는가** | 접근 전 개체 대응, 색·형상, 집기 전후 영상, 물체와 카메라의 상대 운동 | 집게 닫기 명령만으로 파지 확정 금지. 가림·동일 종류 개체 혼동 |
| **앞이 막혔는가** | 자기 영상의 통행 공간, 정적 통로 폭, 화물·편대 외형, 반복 관측 | 그림자·지나가는 동료·시야 부족과 실제 차단 구분 |
| **pickup slot에 물건이 있는가** | slot 위치의 시각적 대응, 충분한 시야, 기대 종류 검출 | 검출 실패를 빈 slot으로 바꾸지 않음 |
| **구역에 내려놓았는가** | 구역 경계와 물체의 상대 위치, 방출 후 안정성·분리 관측 | 일부만 보이는 긴 화물, 다른 동일 종류 물건, 동료가 계속 잡고 있는 경우 |

특히 바닥 상자용 투영 모델을 들린 물체에 그대로 적용하지 않는다. 기존 카메라로 해당 증거가 보이지 않으면 정상 동작 범위의 재관측 동작을 사용하거나 `unconfirmed`로 남긴다. 관측을 쉽게 하려고 카메라 위치나 FOV를 바꾸지 않는다.

**PR 170의 L4 대체 방식도 바꿔야 한다.** 현재 모듈은 reference/before/current TOP을 핵심 입력으로 사용하며, cargo 배송 확정 때 운반자 전체의 집게 명령을 확인한다. 새 조건에서는 TOP도, 다른 로봇의 raw 명령도 robot-facing 판정에 사용할 수 없다. [Outcome `harness/zone_rgb_outcome.py:8–22`](https://github.com/kcm0127-dotcom/ugrp/blob/c30091bf723fd4c40924711ae2a264a0f9d4f938/harness/zone_rgb_outcome.py#L8-L22), [동일 파일:469–510](https://github.com/kcm0127-dotcom/ugrp/blob/c30091bf723fd4c40924711ae2a264a0f9d4f938/harness/zone_rgb_outcome.py#L469-L510)

재사용할 것은 **고정 관측 주기, 다중 관측 안정성, `unconfirmed`, 근거 영상 기록이라는 구조**다. TOP 기반의 출발지 비어 있음 증명은 별도 자기 RGB 판단으로 대체한다. 기존 `JobTracker`도 TOP reference를 받으므로 입력 인자만 바꾸는 수정으로 끝나지 않는다. [Outcome `harness/zone_rgb_outcome.py:530–570`](https://github.com/kcm0127-dotcom/ugrp/blob/c30091bf723fd4c40924711ae2a264a0f9d4f938/harness/zone_rgb_outcome.py#L530-L570)

로봇에 제공할 완료 관련 신호의 선택지는 다음과 같다.

| 선택 | 사용 위치 |
|---|---|
| 자기 명령 큐 종료만 제공 | 초기 연결 시험. 물리 완료 의미 없음 |
| 자기 RGB 기반 `observed_placed / suspected_drop / unconfirmed` | **주 조건 권고** |
| 동료의 한국어·정형 완료 보고 | 통신 조건의 개인 belief 갱신. 검증된 사실로 자동 승격하지 않음 |
| TOP 기반 PR 170 판정 | 평가 보조 또는 별도 참조 조건 |
| 교사 정답 기반 영수증 | 명시적인 교사 진단만 |

현재 A2는 `TeacherReceiptSource`를 사용하고, 교사 종료를 받아 자기 작업 상태와 재질문 시점을 갱신한다. 문구를 “작업 종료”로 바꾸는 것만으로 L4가 사라지지 않는다. [A2 `harness/zone_outcomes_v2.py:33–43`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_outcomes_v2.py#L33-L43), [A2 `scripts/zone_dispatch_v2.py:193–204`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L193-L204)

**단계적 전환안**

1. 저장 자기 RGB로 네 가지 판단과 위치 belief를 검증한다. 가림·오인·미관측을 포함한다.
2. 자기 RGB·자기 명령만 받는 로봇별 tracker를 만들고, 평가 tracker와 인터페이스를 분리한다.
3. 단독 운반의 관측→행동→재관측을 연결한다.
4. 2대·3대 운반에서 자기 역할 확인, 상대적 자세, 공동 정지·하강을 연결한다.
5. 그 뒤에 새 입력 계약의 통신 비교를 시작한다.

여기에는 인식기뿐 아니라 실행기 변경이 포함된다. A2 교사는 실제 자세·접촉력·화물 높이로 단계와 행동을 결정한다고 명시한다. 따라서 이를 공통 실행기로 남긴 예비 비교는 **교사 위의 고수준 조정 실험**이며, 사용자 지시를 준수하는 로봇 본실험으로 보고할 수 없다. [A2 `scripts/zone_team_teacher.py:16–34`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_team_teacher.py#L16-L34)

**5. 추론·발화 비용을 포함한 SIM 시계**

**현재 코드 판정:** 검토한 zone 경로에서는 LLM 응답을 기다리는 동안 SIM이 전진하지 않는 구조다.

- v1은 호출을 포함한 반복문 뒤에서 `zone.step(.5)`를 실행하고, `motion_started` 이후 시간으로 makespan을 계산한다. [로컬 `scripts/run_zone_dispatch.py:274`](</Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:274>), [동일 파일:331](</Users/changmin/projects/ugrp/scripts/run_zone_dispatch.py:331>)
- A2도 `team.ask()`를 호출한 뒤 실행 루프에서 물리를 전진시키며, 초기 `plan_first` 협상 뒤에 `motion_started`를 정한다. [A2 `scripts/zone_dispatch_v2.py:219–237`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L219-L237), [동일 파일:274–300](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L274-L300)
- `ThreeRobotRuntime.ask()`는 future 결과를 기다린다. 선택적인 `idle_callback`은 있으나 A2의 runtime 생성에는 전달하지 않는다. [로컬 `scripts/three_robot_runtime.py:128`](</Users/changmin/projects/ugrp/scripts/three_robot_runtime.py:128>), [A2 `scripts/zone_dispatch_v2.py:164–165`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L164-L165)

**제안 비용식**

호출 \(q\)의 대기 비용을 다음처럼 둔다.

\[
d_q=\Delta t\left\lceil
\frac{\alpha+\beta N_{\mathrm{out},q}+\gamma U_q}{\Delta t}
\right\rceil
\]

- \(\alpha\): 호출당 고정 판단 비용.
- \(N_{\mathrm{out}}\): 고정된 tokenizer로 센 전체 출력 토큰. 행동 JSON과 메시지를 포함.
- \(\beta\): 출력 토큰당 SIM 비용.
- \(U\): 비어 있지 않은 발화 수.
- \(\gamma\): 발화 시작·전송의 추가 비용.
- \(\Delta t\): SIM timestep.

개발 시작값은 **\(\alpha=1.0\)초, \(\beta=0.02\)초/token, \(\gamma=0.3\)초/발화**, 전달 지연은 **0.1 SIM초**를 제안한다. 예를 들어 120토큰·1발화는 3.7초의 actor 대기와 0.1초의 전달 지연을 만든다. 이는 인간 발화 속도나 실제 모델 처리 속도를 측정한 값이 아니라 실험적 비용 설정이다.

**실행 의미를 다음처럼 고정한다.**

1. 호출 시작 \(t\)에서 입력을 캡처한다. 응답은 그 시각의 관측에 대한 결정이다.
2. 호출한 로봇은 안전한 hold 상태로 들어간다. 다른 독립 작업과 물리는 계속 진행한다.
3. 응답과 메시지는 \(t+d_q\) 전에는 실행기·수신자에게 노출하지 않는다.
4. 메시지는 지정된 전달 시각 이후 inbox에 들어간다.
5. 동시 호출 비용은 겹쳐 진행한다. 모든 로봇의 비용을 순차 합산하지 않는다.
6. API가 실제로 느리면 계산을 위해 wall time상 기다릴 수 있으나, 논리적 사건 순서와 SIM 비용은 바꾸지 않는다.
7. 같은 응답·seed·설정에서 HTTP 완료 순서를 바꿔도 SIM trace가 같아야 한다.

구현은 **단일 SIM 사건 큐**가 소유한다. 최종 makespan에 비용을 사후 덧셈하는 방식은 쓰지 않는다. 기다리는 동안 생기는 통로 경쟁·낙하·보고 지연도 물리에 반영되어야 하기 때문이다.

공동 운반 중 한 참여자가 생각해야 한다면 공통 안전 정책에 따라 팀이 함께 멈춘다. grip 명령은 유지하되 물체 pose를 고정하거나 weld를 켜지 않는다. 그 결과 생긴 팀 대기와 미끄러짐도 비용이다.

별도 commander는 물리 몸체가 없으므로 commander의 사고 시간에 로봇 전원이 자동 정지하지 않는다. 기존 명령을 수행하거나 다음 지시를 기다린다. 이 차이는 ③b의 구조적 특성으로 보고한다.

잘못된 JSON·거절된 메시지도 생성 비용을 지불한다. 재시도는 별도 호출로 계산한다. 응답이 없는 timeout은 사전 고정한 timeout 비용으로 처리하고, 실제 API 장애와 정책 실패를 구분해 기록한다.

**파라미터 선택과 민감도**

- 개발용 정상 이동·집기 시간과 비교해 짧은 판단의 상대 비용을 정한다.
- 본실험 전에 tokenizer, 모델 설정, 비용식과 계수를 고정한다.
- 비용 배율 `0, 0.5, 1, 2, 4`를 검사한다. `0`은 비용 없는 대화의 진단 조건이다.
- 호출 비용 \(\alpha\)와 길이 비용 \(\beta\)를 따로 변화시켜 “잦은 짧은 발화”와 “드문 긴 발화”를 구분한다.
- 한국어와 schema의 표현 길이 차이도 효율에 포함한다. 별도로 의미 단위당 비용을 맞춘 비교를 두어 토큰화 효과를 분리한다.
- 비용이 달라지면 행동과 관측 시점도 달라지므로 최종 비교는 다시 실행한다. 저장 로그의 사후 재계산은 근사 분석으로만 표시한다.

**6. 통신이 의미를 갖는 시나리오와 공통 실행 장치**

| 시나리오 | 대화가 줄일 수 있는 비용 | 공정성 조건 |
|---|---|---|
| 정상 혼합 배송 | 중복 작업·불균형 배정 | 무통신도 정적 관례로 해결 가능한 대조군 유지 |
| 미등록 막힘 | 다른 로봇의 잘못된 진입·후퇴 | 발견 로봇의 자기 RGB만 먼저 단서를 얻도록 검증 |
| `long_beam` 2대·`tri_frame` 3대 | 역할 중복·늦은 집결·상충 목적지 | 호스트가 부족한 팀원을 채우지 않음 |
| 좁은 문·복도 대치 | 동시 진입·상호 대기 | 양보를 자동으로 대신 결정하지 않음 |
| 물건 이동·낙하·파지 실패 | 빈 pickup 방문·오배송·실패한 작업 반복 | 사건 일정·대상은 조건과 독립적으로 고정 |
| 새로운 관계·복합 사건 | 설명·조건부 계획의 표현 | 정형 언어의 표현 범위 비교로 별도 해석 |

A2에는 같은 물건·목적지·형성을 선택한 로봇들이 상보 역할 정거장을 채우는 rendezvous 규칙과 60초 대기 한도가 있다. 다만 현재 occupancy는 정답 pose를 받는다. **역할·장벽·timeout 계약은 유지하되, 본실험의 준비 판단은 자기 영상과 허용된 실행 신호로 대체**해야 한다. [A2 `harness/zone_team_jobs.py:332–387`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_team_jobs.py#L332-L387)

Routes의 두 문 지도에는 `door_narrow` 0.5m와 `door_wide` 1.0m가 정의되어 있다. 이를 막힘·우회 선택의 정적 기반으로 사용할 수 있지만, 공동 화물은 로봇 한 대가 아니라 **화물·모든 운반자·회전 공간**의 통과 가능성을 확인해야 한다. [Routes `maps/zones/zone_wide_two_doors.json:315–347`](https://github.com/kcm0127-dotcom/ugrp/blob/4789d932eb0ca02dbf3d40eaef26566d34823c85/maps/zones/zone_wide_two_doors.json#L315-L347)

현재 A2 팀 경로는 접촉 전에 물체 자세 공간에서 계획하며, 정적 벽 외에 바닥 물체 장애물도 받는다. 새 경로 선택에는 정적 지도와 **해당 로봇이 알게 된 장애물 belief만** 전달해야 한다. 아직 관측하지 않은 장애물을 planner에 넣어 자동 우회시키면 통신의 정보 효과가 사라진다. [A2 `harness/zone_team_route.py:1–15`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_team_route.py#L1-L15), [동일 파일:73–79](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_team_route.py#L73-L79)

**②와 ④의 “동일 정보”는 두 수준으로 나누어야 한다.**

- **표현 가능한 사건 코호트:** 두 조건에 같은 관측, 메시지 행위, 수신자, 시간·예산을 제공한다. schema는 물건·역할·문·막힘·불확실성·정정 등을 충분히 표현해야 한다.
- **동일 내용 전달 보조 실험:** 같은 허용 관측에서 만든 동일 명제 집합을 한국어와 schema로 각각 전달한다. 사실·불확실성·수신자·전달 시각을 맞춰 수신자의 해석과 후속 선택을 비교한다.
- **새 사건 코호트:** 예를 들어 “빔을 먼저 회전한 뒤 한쪽이 물러나야 옆 물건을 꺼낼 수 있음” 같은 미등록 관계를 둔다. schema로 표현되지 않는다면 ②−④는 **표현 가능성과 적응성의 차이**이며, 동일 정보에서 자연어 형식만의 효과라고 주장하지 않는다.

자유 대화가 선택하는 사실과 정형 정책이 선택하는 사실은 실행 중 달라질 수 있다. 같은 입력을 준 것만으로 실제 메시지 내용까지 동일했다고 간주하지 않는다.

**호스트 장치 처리**

| 장치 | 현재 확인 | 주 조건 처리 |
|---|---|---|
| **C1 board·중재** | A2 dynamic은 다섯 스위치가 모두 ON이다. [A2 `zone_protocol_v2.py:42–58`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_protocol_v2.py#L42-L58) | board, host arbitration, conflict notice, peer-job-end wake 모두 OFF |
| **C2 tie-break** | dynamic prompt에 동료 claim 차감과 최소 robot ID 우선 규칙이 있다. [A2 `zone_protocol_v2.py:167–178`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_protocol_v2.py#L167-L178) | 제거. 공통 관례를 시험하려면 전 조건 동일한 별도 ablation |
| **L1 선점** | A2는 이동 중 동료 의도만으로 멈추는 분기를 제거했지만, `_taken_by_peer`는 동료 phase/outcome을 읽는다. [A2 `zone_teacher.py:306–315`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_teacher.py#L306-L315) | “의도 선점 수정”과 “자기 RGB 경계 충족”을 구분. 본실험은 로컬 관측으로 판단 |
| **L4 영수증·깨우기** | 교사 종료가 `RobotResults.end()`에 연결된다. [A2 `zone_dispatch_v2.py:193–204`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L193-L204) | 문구·종료 시각·busy 변화까지 robot-facing 경로에서 차단 |
| **통로 자동 양보** | Routes는 통로 내 위치, 들린 상자, robot ID로 후퇴할 로봇을 정한다. [Routes `zone_teacher.py:621–652`](https://github.com/kcm0127-dotcom/ugrp/blob/4789d932eb0ca02dbf3d40eaef26566d34823c85/scripts/zone_teacher.py#L621-L652) | 정답 기반 중재는 본실험에서 제거. 자기 RGB 기반 공통 충돌 방지·정지만 유지 |
| **호출 자격** | A2는 idle 로봇을 호출하며 중간 사건 hook은 기록만 한다. [A2 `zone_dispatch_v2.py:245–275`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L245-L275), [A2 `zone_team_teacher.py:901–907`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_team_teacher.py#L901-L907) | 작업 중에도 같은 로컬 관측·타이머 규칙으로 판단 가능 |

단순히 장치를 전 조건에 똑같이 켜는 것만으로 입력 위반이 해결되지는 않는다. 정답 기반 교사는 별도 진단에 보존한다.

TeamJob의 원자적 명령 적용, 중복 참여 방지, stale generation 거부, 함께 hold·취소하는 최소 실행 동기화는 공통으로 유지한다. 무통신은 **고수준 정보 메시지 없음**으로 정의하고, 이 최소 동기화의 존재와 비용을 명시한다. 준비 신호에 동료의 위치·목적지·관측 결과를 실어 보내지 않으며, 정답 기반 접촉·높이를 준비 판정으로 쓰지 않는다.

**공통 호출 정책의 시작안**

- 자기 카메라 관측: 1 SIM초 간격.
- 호출 계기: 시작, 로컬 영상 belief의 의미 있는 변화, 자기 타이머 만료, 실제 메시지 수신.
- 이동·파지·팀 대기 중에도 동일한 호출 자격.
- actor당 최소 호출 간격 2초, 중복 사건 병합, 동시 미완료 호출 최대 1개.
- idle 재검토 10초, 변화 없는 실행 중 재검토 60초를 개발 시작값으로 사용.
- ①은 메시지 계기만 없음. ②·④는 mesh 수신자, ③은 star 수신자만 호출 계기를 얻음.
- 교사 종료·동료의 숨은 완료·전역 진행률은 호출 계기가 아님.

공정성은 호출 수를 강제로 같게 만드는 것이 아니라 **판단 자격·처리 규칙·상한을 같게 하고 실제 추가 호출 비용을 지불하게 하는 것**이다.

**7. 지표와 주장 가능한 비교**

모든 시간은 초기 관측과 첫 계획 호출 이전의 공통 \(t_0\)부터 센다. 평가자는 정답 상태와 TOP을 사용할 수 있지만, 평가 결과를 로봇의 다음 입력이나 호출 계기로 되돌리지 않는다.

| 지표 | 정의 |
|---|---|
| **전체 성공률** | 고정 SIM·계산 예산 안에서 주문을 충족한 비율. 정책 실패·deadlock·예산 소진 포함 |
| **SIM makespan** | 추론·발화·전달·물리 대기를 포함한 배송 완료 시각. 성공 판정의 안정 유지 시간도 고정 |
| **실패 포함 시간 성과** | 실패·미완료는 horizon에서 censored 처리. 성공 실행 평균과 별도로 보고 |
| **배송량** | 물건당 한 번만 집계. 종류별 정상 배송·오배송·과잉·미달 분리 |
| **idle robot-seconds** | 사고·발화, 지시 대기, 팀 집결, 문 대기, 미배정, 재관측으로 분해 |
| **conflict·deadlock** | 같은 역할 경쟁, 상충 목적지, 반복 양보, 일정 시간 진행 없음. 사후 공통 평가 |
| **replan** | 작업·역할·통로 변경, 취소·재시도와 근거 |
| **모델 비용** | 논리 호출과 HTTP 시도, 입력·출력·이미지·캐시 토큰, actor별 합계 |
| **latency** | 실제 wall API latency와 부과한 SIM 비용을 별도 기록 |
| **발화 비용** | 발화 수, 전달 edge 수, 방송 수, 재전달, 메시지가 후속 문맥에서 소비한 토큰 |

현재 A2의 `collisions`는 호스트 claim 검사 결과에서 증가한다. 중재를 끄면 이 숫자만으로 조건 간 충돌을 비교할 수 없으므로, 제출 행동과 물리 결과를 공통 기준으로 평가하는 계수기가 필요하다. [A2 `scripts/zone_dispatch_v2.py:379–391`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L379-L391)

**대화 지표**

- **한국어 준수:** literal ID·JSON·enum을 제외한 한국어 사용, 영어 전환, literal 손상, 침묵을 각각 집계.
- **사실성:** 발화 명제를 발화 시각의 평가 로그와 비교. 동시에 발신자가 자기 RGB나 수신 보고라는 근거를 가졌는지도 별도 판단.
- **행위 유형:** 관측 보고, 제안, 요청, 질문, 수락, 거절, 양보, 정정, 취소, 완료 주장.
- **결정 연결:** `message_id → delivered_at → 수신 요청 → decision_sources → 행동 변경` 연결.
- **정보 이득:** 자기 눈으로 보기 전에 회피한 막힌 분기, 줄어든 중복 접근, 빨라진 팀 집결.
- **오정보 비용:** 잘못된 보고로 생긴 우회·대기·오배송과 정정까지의 시간.

모델이 메시지를 인용했다는 사실은 인과 효과의 증명이 아니다. 일부 사전 선정 상태에서 메시지 내용 유지/제거를 비교하되, **호출 계기는 유지한 내용 제거**와 **전달 자체 제거**를 나눠 재추론한다. 저장된 응답을 그대로 재생하는 비교로 결정 변화의 인과성을 주장하지 않는다.

| 비교 | 뒷받침하는 주장 | 단독으로 뒷받침하지 못하는 주장 |
|---|---|---|
| ②−① | 자유 한국어 통신을 허용한 전체 효과 | 자연어가 정형 정보 공유보다 우수함 |
| ④−① | 고정 schema 정보 공유 효과 | 자연어 효과 |
| ②−④ | 해당 자유 대화 프로토콜과 schema의 차이 | 내용 선택까지 다른 상태에서 언어 형식만의 효과 |
| 동일 명제 한국어−schema | 의미를 맞춘 표현·해석 차이 | 자유 협상의 전체 효과 |
| ③a−② | star 지휘·권한 집중·지휘 부담의 결합 효과 | 중앙화 하나의 순수 효과 |
| ③b−③a | 전담 commander와 지휘 겸임의 차이 | 계산 자원·물리 역할과 분리된 중앙화 효과 |
| 주 조건−R | 정보가 풍부한 중앙 참조와의 격차 | 한국어 대화의 인과 효과 |

같은 시나리오·seed·사건 일정을 조건 간 짝짓고 실행 순서를 교차한다. 실패 실행을 제외한 속도 순위는 만들지 않는다. 본실험 표본 수는 파일럿의 paired 차이·성공률·분산을 바탕으로 정하고, 개발 중 본 seed는 최종 평가에서 분리한다.

**8. `plan_first` 은퇴와 파일별 변경안**

`plan_first`는 일반 연구 메뉴에서 제거하되 **`--allow-retired-coordination` 뒤에 보존**한다. ZC1/ZC2 재현은 당시 SHA·프롬프트·입력·실행기·비용 규칙을 함께 지정해야 한다. 새 비용식을 적용한 실행을 과거 결과의 동일 재현으로 부르지 않는다.

기존 감사도 ZC1/ZC2가 입력 경계 수정 전에 실행되었으며 재검증하지 않았다고 명시한다. [Main `experiments/2026-09-25-zone-comm-boundary-audit/README.md:105`](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/experiments/2026-09-25-zone-comm-boundary-audit/README.md#L105)

| 파일 | 제안 변경과 근거 |
|---|---|
| `scripts/run_zone_dispatch.py` | 새 연구 profile·scenario·비용 설정·retired guard 추가. 새 연구는 색 상자도 명시적으로 같은 v2 계열 runner 사용. 현재 `auto`는 색 목표를 v1으로 보낸다. [A2:504–543](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/run_zone_dispatch.py#L504-L543) |
| `scripts/zone_dispatch_v2.py` | 조건별 명시 dispatch, 단일 SIM scheduler, actor-local 입력, 평가 캡처 분리. 현재 마지막 `else`는 dynamic 경로이므로 enum만 추가하면 안 된다. [A2:253–291](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/scripts/zone_dispatch_v2.py#L253-L291) |
| `harness/zone_protocol_v2.py` | 실제 `CONDITIONS` 정의 위치. legacy 기본값 보존 후 새 연구 profile에서 첫 네 스위치 OFF, 통신 여부·topology·encoding 명시. 한국어 prompt와 입력 allowlist 적용. [A2:42–70](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_protocol_v2.py#L42-L70) |
| `harness/zone_team_jobs.py` | 새 공통 validator는 공개 주문 ID·종류·역할·행동 형식만 검사. 현재 independent 검사도 pickup 가시성과 구역별 수량을 사용하므로 그대로 재사용하지 않는다. [A2:435–467](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_team_jobs.py#L435-L467) |
| `harness/zone_outcomes_v2.py` | `end(teacher_claim)` 중심 입력을 로컬 관측·타이머 기반 갱신으로 분리. legacy teacher source는 별도 진단에 보존. [A2:58–71](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/harness/zone_outcomes_v2.py#L58-L71) |
| `harness/zone_rgb_outcome.py` | PR 170은 평가·legacy용으로 보존. 새로운 own-RGB tracker를 별도 모듈로 만들고 주 조건에서 TOP source 선택을 거절. [Outcome:533–555](https://github.com/kcm0127-dotcom/ugrp/blob/c30091bf723fd4c40924711ae2a264a0f9d4f938/harness/zone_rgb_outcome.py#L533-L555) |
| `harness/zone_cargo_perception*.py` | 기존 TOP 인식은 평가·참조용. Cargo2의 개선을 자기 RGB 성능으로 승계하지 않음. [Cargo2 `zone_cargo_perception_v2.py:374–397`](https://github.com/kcm0127-dotcom/ugrp/blob/972dc36bbdfbbfe827db79976282765858772804/harness/zone_cargo_perception_v2.py#L374-L397) |
| 신규 zone actor runtime·transport | 수신자별 전달, star 권한, 3/4 actor, 예약 전달·비용 기록 지원. 기존 runtime은 `ROBOTS`별 inbox/client와 3-worker pool을 만든다. [로컬 `scripts/three_robot_runtime.py:44`](</Users/changmin/projects/ugrp/scripts/three_robot_runtime.py:44>) |
| `sim/simulation_launch_options.py` | zone 연구 전용 launch mode 추가. 기존 `llm_dispatch`는 다른 `dispatch` 명령을 만들므로 전역 coordination 목록만 교체하지 않는다. [로컬:60](</Users/changmin/projects/ugrp/sim/simulation_launch_options.py:60>) |
| `scripts/sim_cli.py` | 새 네 조건·참조 조건·재현 메뉴를 분리. 기존 두 선택지는 기존 RGB 공동 계획 경로다. [로컬:427](</Users/changmin/projects/ugrp/scripts/sim_cli.py:427>), [동일 파일:461](</Users/changmin/projects/ugrp/scripts/sim_cli.py:461>) |
| `configs/simulation_workflows.json` | 새 study profile과 입력·결과 schema를 등록하고 버전 상승. 현재 A2 `zone-dispatch` 등록은 교사 실행기와 protocol v2를 명시한다. [A2:113–124](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/configs/simulation_workflows.json#L113-L124) |
| 번들·버전 기록 | 입력 경계·지도/주문서·인식/실행기·prompt·schema·scheduler·비용식·tokenizer·모델 설정을 함께 해시화. 기존 번들을 덮어쓰지 않음. [Main `docs/execution_versioning.md:7–28`](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/docs/execution_versioning.md#L7-L28) |

`peer_messages=true`만으로 mesh/star나 한국어/schema를 표현할 수 없다. 새 profile에는 최소한 `message_topology`, `message_encoding`, `leader_id`, `input_profile`, `outcome_source`, `scheduler_profile`, `cost_profile`을 명시한다. 주 비교에서 임의 스위치 조합을 허용하지 않고, 다른 조합은 별도 ablation ID로 기록한다.

**9. 순서가 있는 병렬 작업 패키지**

아래는 **향후 별도 에이전트에게 배분할 계획**이다. 이번 검토에서 에이전트 실행이나 구현을 시작하지 않았다.

**G0 — PR 169 병합 게이트**

PR 169의 실제 병합과 최종 SHA를 확인하고 PR 170·173·174의 포함·통합 상태를 다시 읽는다. 그 이전에는 runner 변경, runner 통합 PR, runner 기반 실험에 착수하지 않는다. 인터페이스 합의 이후 아래 소유권을 고정한다.

| 패키지 | 단독 소유 파일 범위 | 의존성·완료 기준 |
|---|---|---|
| **A. 연구 계약·입력** | 신규 `harness/zone_study_contract.py`, `zone_study_inputs.py`, 지도 도식 생성기, 전용 입력 테스트 | 공개/비공개 schema, 조건 registry, 매 호출 지도·주문서, hash 고정 |
| **B. 자기 RGB 인식** | 신규 `harness/zone_own_perception.py`, `zone_own_outcome.py`, 전용 영상 fixture·테스트 | A 인터페이스. 네 판단·위치 belief·unknown 처리 |
| **C. 프롬프트·통신** | `harness/zone_protocol_v2.py`, 신규 zone actor runtime·transport, 전용 프로토콜 테스트 | A. 네 조건·두 leader 형태·수신자 격리·schema 검사 |
| **D. SIM 비용·scheduler** | 신규 `harness/zone_sim_cost.py`, `zone_event_scheduler.py`, fake-clock 테스트 | A. wall 지연과 독립적인 실행·전달 순서 |
| **E. 시나리오·지도** | `sim/zone_arena.py`, `sim/zone_scene.py`, `sim/zone_cargo_scene.py`, `harness/zone_mixed_episode.py`, 신규 지도·scenario 설정·전용 테스트 | A. config 기반 주문서와 비공개 사건 분리, 가시성·물리 차단 검증 |
| **F. 로봇 실행·TeamJob** | 신규 `scripts/zone_own_executor.py`, `harness/zone_team_jobs.py`, `zone_team_route.py`, 전용 실행·팀 테스트 | B·E. 정답 없는 실행, 공통 장벽·취소·재계획 |
| **G. runner 통합** | `scripts/run_zone_dispatch.py`, `scripts/zone_dispatch_v2.py`, `harness/zone_outcomes_v2.py`, `tests/test_zone_team_a2.py`, `tests/test_zone_comm_boundary.py` | **PR 169 병합 후**, A–F 계약 충족. runner 변경 담당은 한 명 |
| **H. 실행 메뉴·버전** | `sim/simulation_launch_options.py`, `scripts/sim_cli.py`, `configs/simulation_workflows.json`, 번들 registry/JSON, 관련 문서·launcher 테스트 | G 인터페이스 고정 후. active/retired 분리와 실행 provenance |
| **I. 평가·결과** | 신규 study evaluator·분석 스크립트·metric 테스트, TensorBoard 변환 접점 | A 로그 schema 뒤 병렬 가능. 평가 역류 없음, 실패 포함 분모 |

A를 먼저 확정한 뒤 B/C/D/E/I는 병렬로 진행할 수 있다. F는 B/E, G는 C/D/F에 의존한다. 공유 파일을 여러 패키지에서 동시에 수정하지 않는다. 번호·workflow 버전은 통합 시 다시 확인해 예약한다.

**필수 테스트**

- **입력 불변성:** 자기 RGB·자기 명령·공개 설정을 고정하고 TOP·동료 정답·교사 결과·종료 시각을 바꿔도 해당 로봇의 요청과 호출 계기가 같음.
- **주문서 출처:** simulator 객체를 제공하지 않아도 생성 가능. 실행 중 상태 변화로 바뀌지 않음.
- **채널 격리:** ① 송수신 0, ③ follower 간 직접 전달 0, commander 영상 입력 0, ④ 자유 문자열 거부.
- **메시지 분리:** 본문 변경이 호스트의 claim·예약·다른 로봇 행동을 직접 변경하지 않음.
- **시계:** 호출 비용 0/양수, 동시 호출, 긴 발화, 오류·재시도, 방송, API 응답 순서 변경을 fake clock으로 검증.
- **인식:** 잘못 든 물건, 가림, 빈 slot처럼 보이는 미관측, 구역 가장자리, 동일 종류 혼동, 부분 화물 가시성.
- **TeamJob:** 역할 중복, 늦은 집결, stale generation, 부분 실패, 함께 내리기, N−1 운반 방지, 물건당 단일 집계.
- **경로:** 미관측 장애물이 경로 선택에 반영되지 않음. 팀 전체 footprint의 문·회전 통과 검사.
- **재현:** retired flag 누락 거절, legacy protocol 보존, 새 연구의 v1 자동 유입 거절.
- **평가:** 성공한 짧은 실행과 빨리 실패한 실행 구분, 종료·중단·예산 소진 분모 보존.

기존 A2 테스트는 동일 정적 task, 입력 격리, claim 형식의 출발점을 제공하지만 현재는 5장 이미지 입력을 검사한다. 새 연구 테스트는 legacy 검사와 분리하고 새 allowlist를 검증해야 한다. [A2 `tests/test_zone_team_a2.py:94–154`](https://github.com/kcm0127-dotcom/ugrp/blob/143360dcbfac1e59e628a4cb9d8d2d40bdbdd3a2/tests/test_zone_team_a2.py#L94-L154)

**10. no-LLM smoke, LLM 파일럿, 결과 전달**

**첫 게이트는 no-LLM fixture 시험**이다. fixture actor는 자기 요청만 소비하도록 하고, 호스트의 `active`, 정답 배송 목록, 숨은 사건 플래그를 참조하지 않는다. fixture의 성공은 프로토콜·실행 연결 검증이며 언어 이해나 통신 효과의 증거가 아니다.

| 단계 | 제안 규모 | 확인 목적 |
|---|---:|---|
| 저장 RGB + fake clock | 모든 조건·③b·R 각각 | 시작→보고→수신→판단→취소, 비용·입력 경계 |
| 물리 no-LLM smoke | **6상황 × 주 4조건 × 1 seed = 24회** | 정상 혼합 운반, 막힘, 늦은 집결, 문 대치, 이동/낙하, 새 사건 |
| ③b·R 연결 smoke | 위 6상황 × 2형태 = 12회 | 네 actor 지휘 경로와 중앙 참조 경로 |

물리 smoke는 자기 RGB 실행기가 준비된 뒤 수행한다. 교사를 사용한 임시 smoke는 별도 이름과 결과 범위로 남기고 이 게이트 통과로 대체하지 않는다.

**첫 LLM 파일럿 제안**

- 주 파일럿: 정상 혼합·미등록 막힘·팀 집결·좁은 문·이동/낙하의 **5상황 × 4조건 × 3 paired seeds = 60회**.
- 새 사건 표현력: **②/④ × 3 seeds = 6회**, 주 효과와 분리.
- 선택 보조 파일럿: 막힘·복구 **2상황 × ③b/R × 3 seeds = 12회**.
- 개발 시작 예산: 시행당 **1,800 SIM초**, **HTTP 시도 총 90회**, actor당 최대 30회, 호출 출력 최대 768토큰. 재시도도 포함.
- 주 파일럿의 HTTP 시도 상한은 **5,400회**, 보조까지 모두 수행하면 **7,020회**다. 실행 전 모델 비용과 자원 상한을 계산해 확정한다.

이 숫자는 실행 승인이나 성능 충분성의 판단이 아니다. 개발 fixture와 소규모 연결 확인을 통해 예산이 적절한지 정한 뒤 코호트 전에 동결한다. 예산 부족이 나타나면 해당 실패를 보존하고 다음 코호트에서 조정한다.

파일럿의 통과 기준은 조건 간 우열이 아니라 다음이다.

- TOP·정답·교사 영수증·호출 시각 누설이 없음.
- 자기 RGB의 불확실성을 강제 성공으로 바꾸지 않음.
- 실제 송신·전달·수신·결정·실행이 연결됨.
- 비용에 따라 SIM과 물리가 실제로 진행됨.
- 단독·2대·3대 실행과 실패 정리가 같은 계약을 따름.
- 실패·언어 이탈·API 오류·예산 소진을 포함한 기록이 완전함.

향후 실행 결과에는 요청 원문·실제 전송 이미지·응답·명령·메시지·평가 로그와 해시를 로컬에 보존한다. 새로운 결과는 실패까지 포함해 별도 TensorBoard snapshot에 등록하고 실제 데이터·영상 로딩을 확인한다. 이는 저장소의 결과 전달 지침에 따른 후속 실행 단계이며, **이번 읽기 전용 검토에서는 새 결과나 TensorBoard snapshot을 만들지 않았다.** [AGENTS.md:25–35](https://github.com/kcm0127-dotcom/ugrp/blob/6d80e3e401f6936bc90cd3ccfc00249018a81748/AGENTS.md#L25-L35)

Codex session ID: 01a0d8a0-c313-7c13-ba80-0e5bd0da817e
Resume in Codex: codex resume 01a0d8a0-c313-7c13-ba80-0e5bd0da817e
