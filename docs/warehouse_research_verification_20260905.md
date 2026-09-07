# Warehouse 연구 경로 검증 — 2026-09-05

기존 중앙 역할 배정과 LLM의 선택을 분리했고, actor는 자기 RGB-D 마커 관측과
로봇별 관측 기억, 허용된 자연어 메시지만 사용한다. 공통 저수준 이송기는
여전히 SIM의 이상적 내부 피드백을 사용한다. REAL 보정 완료 주장은 하지 않는다.

## 검증 결과

- 기존 전체 회귀: 968 tests / 372.929s / OK.
- 관측 기억·데이터 복사 격리·프로토콜·평가·snapshot 추가 집중 검증: 37 tests / OK.
- GPU 배포 번들만 임시 디렉터리에 복사한 뒤 world 생성과 연구 begin 실행: BUNDLE_OK 3.
- 실제 TEAM UI의 전체 화물 요청 → 세 독립 Gemini completer → 합의 → 물리 이송 → 결과 표시: SUCCESS.
- 최종 live 실행: 6 decision rounds, 3 cargo actions, 3/3 delivered, 모든 wakeup done, queue/inflight 0.
- 최종 영상 실행: 6 rounds, 3 actions, 3/3 delivered. 테스트용 base/free-body pose setter를 예외로 막아 둔 상태에서 성공(호출 0회).

## 비교 pilot

단일 pipe, seed 11, normal / gripper_failure / goal_revision 각 1회.
조건마다 초기 world, 센서, 스킬, 최대 8 rounds와 6 actions, 출력 512 tokens를
같게 고정했다. 목표 변경은 첫 이송 완료 뒤 B→C이며, 고장은 r1에게만 알려진
그리퍼 사용 불가다. 모든 실패와 raw LLM 응답을 남겼다.

| 조건 | 성공 | 해석 |
| --- | --- | --- |
| `rule` | 3/3 | 소규모 기능 pilot |
| `llm_no_comm` | 0/3 | 소규모 기능 pilot |
| `llm_peer_comm` | 3/3 | 소규모 기능 pilot |

표는 `outputs/warehouse_research/verified-pilot-20260905/design.json`의 소스 해시에
해당한다. 이후 관측/표시 및 번들 정리는 최종 전체 이송으로 별도 검증했다.
각 source/sensor profile의 결과를 합산하지 않는다. 이 표는 통계적 우월성이나
일반화를 입증하지 않는다. 규칙 비교군도 성공했으므로 LLM이 규칙보다 낫다는
결론은 아직 없다. 반복 시드·모델 샘플링·더 넓은 과제가 필요하다.

## 증거 파일

- [비교 요약](../outputs/warehouse_research/verified-pilot-20260905/summary.json)
- [최종 전체 이송 결과](../outputs/warehouse_research/final-all-cargo-20260905/result.json)
- [최종 실제 LLM 원문/관측/행동](../outputs/warehouse_research/final-all-cargo-20260905/episode.jsonl)
- [6배속 영상](../outputs/warehouse_research/final-all-cargo-20260905/peer-all-cargo-6x.mp4)
- [전체 영상](../outputs/warehouse_research/final-all-cargo-20260905/peer-all-cargo.mp4)
- [영상 프레임 검토](../outputs/warehouse_research/final-all-cargo-20260905/contact-sheet.png)

초기 색상 검출 pilot은 로봇의 노란 부품 오인으로 폐기했다. ArUco 전환 뒤
기억이 없는 별도 전체 이송도 2/3에서 실패했으며 그 기록을 보존했다.
현재 구현은 과거에 직접 읽은 ID만 로봇별로 기억한다. 옛 거리/방향을 재사용하지
않고 현재 시야와 기억을 명시적으로 구분한다. 기억은 no-comm peer로 누출되지 않는다.

현재 센서는 마커가 붙은 화물과 모의 depth를 전제로 한다. 마커 없는 일반 물체
인식, 실제 센서 보정, 실물 이송 정확도, 완전 분산 저수준 공동 조작은 검증 범위 밖이다.
