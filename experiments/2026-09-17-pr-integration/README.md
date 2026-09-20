# 2026-09-17 PR integration

사용자 요청: 열린 PR 모두 병합.

## 완료와 통합 범위

main에 병합: #41, #57, #58, #60, #61, #62, #63, #64, #65.
기준 main: `293c0ccfaf65e39333b9cc19a4af842c60761c43`.
남은 #42, #44, #45, #46, #47, #48, #52, #53, #54의 원본 커밋을 merge commit으로 통합.
실행 소스: `8fa536b` (후속 커밋은 이 기록만 추가).

## 충돌 해소

- scripts/run_ci_tests.py: 양쪽 테스트 목록 모두 보존.
- .github/workflows/tests.yml: stage/dispatch/traffic 검사와 artifact 업로드 모두 보존. 각 artifact의 누락 경고와 14일 보존 설정 유지.
- README.md, experiments/README.md: 양쪽 실험 행 모두 보존.
- scripts/camera_approach_scene.py: 기존 RGB 증거 보존 주석 유지.
- 충돌 해소로 새로운 제어 로직을 작성하지 않음.

## 검증과 한계

전체 로컬 회귀검사 결과는 offline-regressions.log 참조.
기존 PR의 CI는 모두 SUCCESS, 미해결 리뷰 대화 0개 확인.
GitHub 통합 CI는 PR checks에서 별도 확인. 실제 모델/로봇 실험은 이번 통합에서 새로 실행하지 않음.
코드/문서 diff whitespace 검사 통과. 기존 실험 CSV 세 파일의 CRLF 경고는 원본 보존을 위해 변경하지 않음.
원래 worktree와 브랜치는 보존. Google Drive 사용 안 함.
