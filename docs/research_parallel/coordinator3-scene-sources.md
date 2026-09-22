# 3차 조정 · ACT 장면의 지도 출처 보존

2026-09-22. 공통 기준 `357e1f2e66dcae20c54f669711308ed18cf03966`(PR #98).
A3의 독립 감사에서 발견한 출처 누락을 조정 작업에서 좁게 재현·수정했다.

## 원인과 수정

`sim.act_map_suite.load_suite()`는 선택된 장면 하나가 아니라 전체 suite를 생성하고
분할을 검사한다. 새 corner 인스턴스는 `maps/pair_navigation/l-corner.json`을 읽고,
legacy regression이 켜졌으면 catalog와 해당 지도 원본도 읽는다. 그러나 native
`Scene.sources`는 suite/protocol과 선택된 regression 지도만 보존했다.

이번 변경은 ACT 장면 resolve 시 전체 생성·분할 검사에서 참조한 map source와
legacy catalog의 원본 bytes/SHA256을 기존 `Scene.sources`에 추가한다. 기존 CLI의
`inspect`와 `scene-sources` 저장 경로가 이를 그대로 사용한다. 지도·장면 구성,
카메라·해상도, reset, physics, 제어기와 actor 입력은 바꾸지 않았다.

추가 원본은 출처·검토 자료이지 actor가 새로 관측한 데이터가 아니다. 모든 source가
그 선택 장면의 기하 부모라는 뜻도 아니다. suite 전체의 분할 검사 의존성을 포함한다.
지도 그룹 해시와 실제 물리 환경·카메라 식별의 차이는 A3/B3/D3의 별도 감사 범위다.

## 검증

- 먼저 `load_suite.read_json`을 추적해 실제로 읽은 원본이 모두 기록되는지 검사했다.
  수정 전 `act/train-open-1`에서 누락으로 실패했다.
- 수정 후 ACT의 22개 selector 각각에서 실제 읽은 파일의 포함과 bytes/SHA 일치를 검사했다.
- legacy regression을 끈 generated corner에서도 원본 corner가 포함되고 불필요한
  legacy catalog가 기록되지 않는 것을 검사했다.
- 실제 CLI `init` → `inspect`의 출처 해시 출력도 확인했다.
- `tests/test_simulation_scenes.py`와 `tests/test_act_map_suite.py`: **57 passed**.
  `git diff --check` 통과. 로컬 JUnit: `outputs/scene-source-audit/regressions.xml`.

오프라인 source/설정/CLI 검사이며 새 물리·렌더링·학습·LLM 실행은 하지 않았다.
기존 raw 자료를 덮어쓰거나 재분류하지 않았고, TensorBoard에 합성 결과를 추가하지 않았다.
최종 통합 SHA의 전체 회귀·A 독립 감사·새 물리 시행은 D3의 별도 단계다.
이 변경은 #98에 의존하는 별도 PR이며 사용자 승인 없이 main에 병합하지 않는다.
