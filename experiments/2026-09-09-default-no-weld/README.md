# 파지 고정 제약 기본 OFF 검증

실행 SHA `9d72854`, seed 11, 기존 macOS MuJoCo 환경. CLI에 제약 관련 옵션 없이 실행했다. 두 조건 모두 config와 trial의 weld_assistance=false이며 모든 이벤트의 constraints_active가 false였다. 물체 상승 실패는 접촉만으로 파지가 성립하지 않은 기존 현상의 재현이며 기본 OFF 설정 검증은 통과했다. 해당 세션 종료. LLM 호출/비용 0. 원본 영상/로그는 로컬 outputs/default-no-weld-20260909이며 원격 백업 아님.

CLI/Python API 기본 OFF. --no-weld 호환 유지. --with-weld 명시적 진단 opt-in. 프로젝트 AGENTS.md에 향후 파지 시험의 인위적 고정 금지 기본값 기록. 다른 기존 실행기의 내부 구현을 전부 바꾼 변경은 아니며 파지 검증에 사용 전 OFF 확인 의무가 적용된다.
