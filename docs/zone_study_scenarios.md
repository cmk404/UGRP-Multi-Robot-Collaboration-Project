# 연구 시나리오 설정 (한국어 대화 연구)

한국어 로봇 대화가 다중 로봇 작업 효율에 영향을 주는지 재기 위한 시나리오를 선언적 JSON으로 고정한다. 주 조건은 통신 채널만 다르고 나머지 입력이 모두 같아야 하므로, 시나리오는 **조건과 무관하게** 한 번 정해지고 실행 중 바뀌지 않는다.

- 설정: `configs/zone_study_scenarios/*.json`
- 로더·검증기: `harness/zone_study_scenarios.py`
- 테스트: `tests/test_zone_study_scenarios.py`
- 의존 계약: [연구 계약 A](zone_study_contract.md) (`harness/zone_study_contract.py`, `harness/zone_study_inputs.py`)

이 패키지는 설정과 검증만 담당한다. 장면 생성·러너·지도 파일은 바꾸지 않으며 `sim/zone_cargo.py`와 `maps/zones/*.json`은 읽기만 한다.

## 공개부와 비공개부

| 구분 | 위치 | 내용 | 누가 보는가 |
|---|---|---|---|
| 공개부 | `schema`, `scenario_id`, `map_id`, `landmark_detail`, `seeds`, `orders`, `notes` | 정적 지도 참조와 주문서의 원본. 물건 종류·개수·필요 로봇 수·목적 구역과 **coarse pickup slot**만 | 패키지 A가 주문서로 만들어 매 호출 로봇에게 준다 |
| 비공개부 | `eval` (`ugrp.zone_study_scenario_private.v1`) | 장면 빌더가 쓰는 정확한 배치 pose, 숨은 사건과 SIM 시간 트리거, 시행 예산 | 평가와 장면 생성만. 로봇 입력에 절대 들어가지 않는다 |

공개부에는 **실수(좌표)가 하나도 없다.** 개수·필요 로봇 수·seed만 정수로 들어간다. 따라서 좌표가 공개부에 숨을 자리가 없다. 비공개부의 `pose_m`, `hidden_events` 같은 키는 A의 `forbidden_key_hits`에 걸리므로, 실수로 로봇 payload에 넣으면 호출 전에 거절된다.

`initial_location`은 **설정 시점에 그 자리에 두기로 정한 위치**다. 이동·낙하·재파지 뒤에도 갱신하지 않는다. 그래서 주문서가 현재 상태를 보장하지 않으며, 로봇은 자기 손목 어안 RGB·자기 발행 명령·허용된 수신 메시지로만 현재를 판단한다.

## 시나리오 6종

| ID | 지도 (SHA-256 앞 12) | seeds → `leader_ko` 지휘자 | 주문/배치 | 숨은 사건 | 무엇을 검증하는가 |
|---|---|---|---|---|---|
| `s1_normal_mixed` | `zone_wide_door_tags_v1` `f86fc314ed3c` | 601→r2, 602→r3, 603→r1 | 6 / 7 | 없음 | **대조군.** 단독 상자·can·tile과 2대 빔 1건이 섞인 정상 배송. 정보 비대칭도 사건도 없으므로 통신이 줄일 수 있는 비용은 중복 작업과 배정 불균형뿐이다. 무통신도 고정 관례로 해결할 수 있어야 한다 |
| `s2_unmapped_blockage` | `zone_wide_two_doors_tags_v1` `2562d2f09940` | 611→r3, 612→r1, 613→r2 | 3 / 4 | `passage_blocked` 45 s | **정보 이득.** 지도에는 두 문이 열려 있다고 적혀 있고 45 SIM초에 `door_narrow`가 막힌다. 먼저 도착한 로봇만 본다. 그 관측을 전달하면 나머지는 헛된 진입을 건너뛰고 `door_wide`로 우회한다 |
| `s3_late_rendezvous` | `zone_wide_door_tags_v1` `f86fc314ed3c` | 621→r1, 622→r2, 623→r3 | 4 / 4 | `robot_hold` 12 s, 40 s 동안 r3 | **집결 지연.** 2대 빔 운반의 짝이 오지 않는 이유를 관측으로 알 수 없다. 대기·재배정·취소 중 하나를 골라야 하고 idle robot-seconds가 '팀 집결 대기'로 분해된다 |
| `s4_narrow_door_standoff` | `zone_wide_corridor_tags_v1` `a349d42d60f3` | 631→r2, 632→r3, 633→r1 | 4 / 4 | 없음(구조적) | **양보 협상.** 동쪽으로 가는 길은 폭 0.5 m 단일 차선 `corridor_1` 하나, 대피소는 `bay_1` 하나다. 통로는 정적 지도에 있으므로 정보 비대칭이 없고 순서 협상만 남는다. 대치·반복 양보·deadlock 수와 문 대기 시간을 비교한다 |
| `s5_moved_dropped_item` | `zone_wide_door_tags_v1` `f86fc314ed3c` | 641→r3, 642→r1, 643→r2 | 3 / 4 | `item_moved` 30 s, `item_dropped` 62.5 s | **오정보 비용과 정정.** 지정 개체 `cyan_1`이 P1-1에서 P1-3으로 옮겨지고(주문서는 계속 P1-1), 운반 중 `red_1`이 떨어진다. 빈 slot과 낙하를 전달하면 헛된 방문과 완료 오주장이 줄어든다 |
| `s6_novel_relation` | `zone_wide_two_doors_tags_v1` `2562d2f09940` | 651→r1, 652→r2, 653→r3 | 3 / 3 | 없음(설정 기하) | **표현력 비교(자유 한국어 vs 정형).** `can_1`은 서쪽에서만 집을 수 있는데 `beam_1`이 그 접근로를 남북으로 막고 있다. 필요한 메시지는 "빔을 약 90도 돌린 다음 한쪽이 물러나야 can을 꺼낼 수 있다"는 순서·조건 명제이며, 정형 필드에는 순서·조건·인과를 담을 자리가 없다 |

세 시나리오(`s1`, `s4`, `s6`)에 숨은 사건이 없는 것은 의도적이다. `s1`은 기준선, `s4`는 정보가 아닌 협상만 재는 조건, `s6`은 "새 사건"이 아니라 **새 관계**를 설정 기하로 만든 조건이다. 아무것도 갑자기 나타나지 않는다.

seed는 시나리오끼리 겹치지 않고, 각 시나리오의 세 seed가 `leader_ko`의 지휘자 r1·r2·r3를 모두 한 번씩 만든다(패키지 A의 `robots[seed % 3]`). 지휘는 허브-스포크이며 follower끼리의 직접 전달은 없다.

## 숨은 사건 schema

```json
{
  "event_id": "door_narrow_blocked",
  "kind": "passage_blocked",
  "trigger": {"kind": "sim_time", "at_sim_s": 45.0},
  "target": {
    "passage": "door_narrow",
    "obstacle": {"obstacle_id": "fallen_pallet_1", "center_m": [2.2, 0.05],
                 "half_extents_m": [0.15, 0.22], "height_m": 0.12}
  },
  "discovery": {"kind": "own_camera_near_anchor", "anchor": "door_narrow", "radius_m": 1.2}
}
```

| 필드 | 허용값 | 규칙 |
|---|---|---|
| `kind` | `passage_blocked`, `passage_cleared`, `item_moved`, `item_dropped`, `robot_hold`, `obstruction_added` | |
| `trigger.kind` | `sim_time`만 | wall 시간·모델 응답 순서와 무관한 결정론적 일정. 사건 목록은 `at_sim_s` 순으로 정렬 |
| `discovery.kind` | `own_camera_near_anchor`, `own_camera_self`, `own_camera_any` | 호스트는 사건을 알리지 않고, 사건이 호출 계기가 되지도 않는다 |
| `target` | `kind`별 고정 필드 | 통로·slot·물건·로봇 ID는 공개 지도와 선언된 배치에 있는 것만 |

## 검증기가 증명하는 네 가지

`python -m harness.zone_study_scenarios [--manifest]`로 실행하거나 `harness.zone_study_scenarios.validate()`로 호출한다. 검사마다 테스트에 부정 사례를 함께 두어 검사가 헛돌지 않도록 했다.

| 검사 | 증명하는 것 |
|---|---|
| `public_no_private_info` | 공개부에 평가 전용 키·비ASCII 키·실수·비공개 id(부분 문자열 포함)가 없다. 역방향 양성 대조로 비공개부는 여전히 A의 payload 검사에 걸린다 |
| `order_sheet_private_independent` | 전체 설정으로 만든 주문서와 공개부만으로 만든 주문서의 정규 JSON SHA-256이 같고, 비공개부를 비우거나 다른 사건으로 바꿔도 같다 |
| `hidden_events_private_only` | 사건은 `eval`에만 있고, 트리거는 SIM 시간, 발견은 자기 카메라, 통로 막힘은 실제로 개구부 안에서 적재 로봇 폭 0.39 m보다 좁은 틈만 남긴다 |
| `cargo_matches_sim` | `required_robots`와 질량이 `sim/zone_cargo.py`의 `CATALOGUE`/`EXISTING_SOLO`와 일치하고, 1대 측정 가반하중 0.66 kg을 넘는 종류는 2대 이상이며, 시나리오가 카탈로그 질량을 덮어쓰지 못한다 |
| `placements_match_public_slots` | 비공개 정확 pose가 공개부가 약속한 coarse slot 안(여유 0.05 m)에 있고, 물체끼리 0.20 m·벽에서 0.20 m 떨어져 pickup 영역 안에 있으며, 주문 개수와 배치 수가 맞고 지도 해시 고정이 디스크 파일과 같다 |
| `leader_rotation` | 시나리오의 seed들이 `leader_ko` 지휘자 r1·r2·r3를 모두 만든다 |

## 화물 표 (원본: `sim/zone_cargo.py`)

| 종류 | 필요 로봇 | 질량 | 바닥 면적 | 출처 |
|---|---:|---:|---|---|
| `cyan`/`green`/`red`/`yellow` | 1 | 0.030 kg | 0.034 × 0.040 m | `EXISTING_SOLO['box']` |
| `tile` | 1 | 0.025 kg | 0.060 × 0.040 m | `CATALOGUE` |
| `can` | 1 | 0.080 kg | 0.038 × 0.038 m | `CATALOGUE` |
| `long_beam` | 2 | 0.300 kg | 0.600 × 0.040 m | `CATALOGUE` (기하 제약: 장축에서만 집힌다) |
| `heavy_crate` | 2 | 0.900 kg | 0.240 × 0.100 m | `CATALOGUE` (질량이 1대 가반하중 초과) |
| `tri_frame` | 3 | 1.500 kg | 0.392 × 0.453 m | `CATALOGUE` (현재 6종에는 사용하지 않음) |

테스트는 이 값을 고정 사본으로 들고 비교한다. `sim/zone_cargo.py`가 바뀌면 조용히 시나리오 의미가 바뀌는 대신 테스트가 실패한다.

## 지도 참조

여섯 설정은 AprilTag 변형 `zone_wide_door_tags_v1`, `zone_wide_two_doors_tags_v1`, `zone_wide_corridor_tags_v1`만 쓴다. 자기 카메라 위치 추정에 벽 태그가 필요하기 때문이다. 지도 파일은 수정하지 않고 파일 SHA-256으로만 고정하며, `eval.setup.map_file_sha256`이 디스크 파일과 다르면 검증이 실패한다. `tags_v2`가 들어오면 기존 설정을 덮어쓰지 않고 새 시나리오 버전을 올린다.

pickup bay/slot은 지도의 `regions.pickup`에서 패키지 A가 결정론적으로 유도한다(P1/P2 × 3행 = P1-1…P2-3). 좌표계는 `world metres; x east, y north`이고 `pose_m`은 `[x, y, yaw]`다.

## 검증하지 않은 것

- **배치의 물리적 도달성.** 교사 주행 여유를 고려해 기존 pickup 격자 위에 두었지만, 실제 접근·파지 성공은 장면 빌더·러너 통합 뒤 물리 실행으로 확인해야 한다.
- **막힘의 실제 가시성.** "문 근처에서만 보인다"는 설계 의도이며, 자기 카메라 영상으로 따로 검증해야 한다.
- **빔 편대의 0.5 m 차선 통과**와 `s6` 접근 불가·해소 순서.
- **시행 완주와 조건 간 효율 차이.** 이 패키지는 설정이 연구 계약을 지키는지만 증명한다.

`manifest()`의 `verified`/`not_verified`에 같은 구분이 데이터로 들어 있어 실행 기록에 그대로 남는다.
