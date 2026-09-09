# 동일 물체 공동 파지 동기화 진단

요청: 두 로봇이 같은 물체를 잡고, 한쪽 파지가 늦으면 기다린 뒤 공동 상승하는지 시험한다.

## 기존 실행 경로 진단

- 코드 SHA: `4190db90e4a724a3f960b7d3a8494f931bc7aaf4`
- 환경: `.venv-sim-worker-mac`, macOS, seed 11, `UGRP_BEAM_DYNAMIC=1`.
- 명령: `python scripts/record_sim_beam_mission.py --out outputs/dual-grasp-sync-20260909/baseline.mp4` (ugrp_session 래퍼 사용).
- 결과: 파지 전 scout r2 이동 중 r3와 간격 0.003 m가 최소 0.03 m보다 작아 `PeerTooClose` 실패. 공동 파지 또는 동기화 성공을 검증하지 못함.
- 기본 structural 실행 경로는 물체 pose를 직접 변경하므로 이 시험에 사용하지 않았다.
- 원본 로그/영상: `outputs/dual-grasp-sync-20260909/`에 로컬 보관. 원격 백업되지 않음.
- 영상 마지막 프레임 확인: 실제 파지 단계에 도달하지 않음.
- 회귀 검사: 188 passed, 107 subtests passed. 물리 파지 검증과 별도 결과.

## 분리 시험

이동과 scout 단계를 제외한 초기 배치에서 실제 접촉을 검사한다. 파지 후 사용하는 weld는 보조 제약이며 실제 마찰 파지나 하드웨어 성공을 증명하지 않는다. 실험 결과는 후속 기록에 포함한다.
