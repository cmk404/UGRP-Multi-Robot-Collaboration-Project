# 기존 plan 실행 경로에 로컬 진입점 연결

기본 메뉴와 `dispatch --task`를 기존 `run_dispatch_e2e --executor skills`에 연결했다. 세 로봇 합의 → `SkillBindings` → 로봇별 RGB 스킬이라는 기존 흐름을 유지한다. raw LLM 콘솔은 명시적 저수준 진단으로만 남기고 기본 메뉴에서 제거했다.

실행 소스: `c089c2024d8efbe555e4bd0b524a6968c387746b`. open/seed11에서 실제 모델 9요청으로 계획을 합의하고, pair 151행·solo 34행의 기존 스킬 결정을 확인했다. 저장된 프로그램이 기존 `SkillBindings` 출력과 같고 모든 실제 계획 요청에 사용자 지시가 포함됨을 확인했다.

120초 제한 후 접근 단계에서 종료했다(정리 포함 122.116초). 계획 합의=true, 프로토콜 완료=false, 물리 성공=false. 운반 성공이나 통신 성능 비교 결과가 아니다. weld 0, 기존 제어 카메라 보정 유지, 정리 오류 없음.

Mac 네이티브 창 생성/종료와 GUI 복사본의 물리 격리 테스트를 통과했다. Linux 회귀와 native observer 검증은 PR #110 CI에서 확인한다. `verification.json`은 기록 생성 시점의 상태다.

원본은 `verification.json`의 로컬 경로, 모든 파일 해시는 `raw-manifest.jsonl.gz`에 보존했다. 원격 raw 백업은 아니다. TensorBoard 새 스냅샷의 이벤트/API 로딩, 고정 카드 7개, HParams 열 선택을 확인했고 원본 영상 Range 요청 206도 확인했다. 계획·내부 매크로 제외 raw 명령·모델 지연은 고정 Time Series 카드에 표시한다.
