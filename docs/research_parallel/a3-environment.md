# A3 · 장면 정의와 실행 지원 경계

출발 소스는 #98의 `357e1f2e66dcae20c54f669711308ed18cf03966`이다. 이 문서는 정의·출처 감사이며 실제 reset, 렌더링, 추론, 운반 실험을 수행하지 않는다. 최종 D 통합 SHA/config에는 별도의 A3 독립 감사가 필요하다.

## 재현 가능한 매트릭스

`harness.rgb_communication_scenarios.scene_support_matrix(seed=11)`은 native registry를 직접 resolve하고 지도·목표·reset·카메라·evaluator 정의와 파일 해시를 반환한다. MuJoCo world/XML 생성·모델 호출은 없다. 파일 목록에는 간접 입력인 `maps/pair_navigation/l-corner.json`과 evaluator 의존성도 포함한다.

| native family | 목록 수 | B 운반 지원 |
|---|---:|---|
| legacy engine | 4 | 없음 |
| dispatch | 5 | `dispatch/open`만 고정 경로의 실험적 연결 |
| navigation | 9 | 없음 |
| pair_navigation | 6 | 없음; 바닥 하중 preview |
| ACT | 22 | 없음 |
| multi_object | 12 | 없음; 모두 `train-open-1` 부모 |

58은 선택 항목 수이지 독립 지도 수·운반 성공 수가 아니다. pair-navigation 6개와 ACT regression 6개는 같은 부모 지도다. navigation 대응 장면도 배경 형상을 공유하지만 로봇·하중·reset은 다르다.

기존 `map_group_sha256`/`layout_digest`는 bounds/obstacles/TOP 묶음이다. 뜻을 바꾸지 않는다. terrain과 `setup_only.unexpected_obstacles`를 포함하지 않아 shared_crossing/north_blocked/rough_south가 같은 group일 수 있다. A3의 별도 `physical_geometry_sha256`는 bounds/obstacles/terrain/unexpected obstacles **정의**를 묶으며, 실제 compiled XML의 해시를 대신하지 않는다. cargo 수·배치·goal 반복은 부모 지도의 독립성을 늘리지 않는다.

## B/D 선택 입력 계약

`assess_scene_backend_selection("dispatch/open", descriptor, purpose="connection_diagnostic")`은 B의 새 읽기 전용 descriptor와 같은 seed의 `episode("open", seed)`를 비교한다. 정확한 map ID, map/instance/group/reset/TOP/goal-frame, 전체 skill route/dock/role, 단일 clock, weld OFF, SIM image age1초·worker total wall2초를 유지한다. 다른 선택이나 easy/safe-stop 실제 입장 요청은 실패한다. 이 함수의 `ready`는 정의 연결만이며 `execution_admission=false`다.

처음 승인된 D 실행은 `dispatch_open`, seed11의 `d3-local-solo`/`d3-local-joint`, condition none 각1회뿐이다. 각 SIM180초/wall600초/명령6000, 전체 setup·회수·정리 포함1800초, 외부 모델·토큰·메시지0, 동시성1이다. 재시도·안전대조 추가 실행·학습은 허용하지 않는다.

| 목적 | 선택 후보 | 현 상태 |
|---|---|---|
| 연결 진단 | dispatch/open seed11, B fixed solo south / pair north, dock A/B | 정의만 연결, 새 소스·자산·A3감사·자원 gate 필요 |
| 쉬운 구조적 개발 운반 | act/dev-open 등 dev 부모 | goal/heading/loaded route/evaluator 미연결 → unsupported |
| 불가능/안전정지 대조 | act/control-narrow, act/control-blocked | 정적 장면만 있음; 안전정지 물리 실행 미검증 |
| Test A/B | 고정 suite의 전체5개 | 개발 진단·튜닝에 사용 금지; 학생 실행 미지원 |

지원 없는 dev/control을 dispatch_open으로 대체하거나 정적 경로 실패를 안전정지 성공으로 바꾸지 않는다. 첫2회는 live readiness의 easy development + impossible control을 충족하지 않는다.

## native와 B 실행은 별도 증거

native `dispatch/open`과 B는 seed11에서 동일한 map/setup 정의를 사용하지만 native 기본 RGB384×288과 B960×720은 다르다. native robots/objects/builder/contact_profile override는 이 B 계약에 포함되지 않는다. 기본 native 선택 shared_crossing도 자동 대체하지 않는다.

TOP 해시만으로 own camera/FOV·해상도·실제 이미지 입력 동일성을 주장하지 않는다. 매트릭스는 own camera 관련 소스 signature와 두 해상도를 분리한다. D 회수 뒤 아래 원본을 다시 결박한다.

- 최종 `scene.xml` bytes/hash, robot/beam XML hash, 실제 solver/timestep와 contact profile
- `episode-setup-only.json`, reset seed, initial invariants의 TOP/own camera/FOV와 weld OFF
- 원본 own/TOP JPEG 크기·해시·capture SIM/wall, actor별 request/issued command 연결
- 별도 Referee 소스/판정 규칙·샘플 시각, solo box / joint beam 목표, mission 판정 원문
- 종료된 모든 자식 뒤 확정된 artifact hashes와 영상; partial/invalid 원본 보존

native `evaluation_state()` 진단과 B Referee의 운반 성공 판정은 같지 않다. 현재 matrix의 runtime XML/camera 증거는 null, physical success는 false다.
