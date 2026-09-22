# R0/R1 감사 fixture

기준 소스는 `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b`다.
`source_manifest.json`에는 읽은 production 파일의 SHA-256과 별도 과거 실제 요청
표본의 출처가 있다. 파일 해시는 당시 스냅샷 식별값이며 이후 수정의 통과 조건은 아니다.

- `boundary_contract.json`: 현행 raw RGB API와 연결 파일럿에 필요한 능력의 차이.
  `minimum_sync_return_fields`는 의미 수준 예시이며 B의 실제 필드명을 강제하지 않는다.
- `pilot_protocol.json`: 실행하지 않은 6회 설계 후보. null인 모델·통합 SHA·설정은
  제출 전에 고정해야 하며, D의 통합 manifest가 최종 실행 권위다.
- 테스트 JPEG는 합성 byte marker로, 카메라나 시각 이해의 증거가 아니다.
  가짜 시계/명령 포트/응답으로 production 입력 조립·버스·runtime을 호출한다.
- `test_*reproduces*`, `test_current_planner_blacklist*`, 공유 계획 검사는 결함 또는
  지원 범위를 **재현하는 characterization test**다. 통과가 RGB 파일럿 go를 뜻하지
  않는다. 수정 시 해당 재현을 새 차단/만료 회귀검사로 교체하고 감사 문서도 갱신한다.
  skip/xfail은 사용하지 않는다.

새 production 경로와의 실제 통합, 외부 LLM, 물리 시뮬레이션, 렌더링, 학습은
실행하지 않는다. 관련 오프라인 검증 명령은
[A handoff](../../../docs/research_parallel/a-handoff.md)에 기록한다.
