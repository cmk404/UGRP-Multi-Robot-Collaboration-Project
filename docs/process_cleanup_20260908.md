# UGRP 프로세스 정리 — 2026-09-08

사용자 요청: UGRP를 작업할 때만 실행하고, 작업하지 않을 때는 메모리를 점유하지 않도록 종료한다.

## Mac 설정 변경
- `com.ugrp.real`: launchctl disable 및 bootout. 로그인 자동 실행과 KeepAlive 재시작 차단.
- `com.changmin.masterpi-dashboard`: 기존 미로드 상태에서 disabled 설정. plist 원본은 둘 다 보존.
- 4일 이상 실행한 REAL harness 3개 및 supervisor 종료.
- 2일 이상 실행한 SIM harness 3개, bridge, MuJoCo websocket worker 및 supervisor 종료.
- 과거 PID 소멸과 TCP 8082–8087, 8091–8093 리스너 부재 확인.

## 재발 방지
세션 실행 래퍼와 프로젝트 AGENTS 지침을 통해 작업 단위로 서비스를 시작하고 작업 종료 때 자식 프로세스도 정리한다. 컴퓨터 사용 여부에 따른 임의 시간 제한은 두지 않는다. 자세한 명령은 `docs/on_demand_processes.md`.

## 범위
실제 로봇 동작이나 유료 모델 호출은 이 정리 작업에서 수행하지 않았다. 기존 실험 파라미터와 다른 미커밋 변경은 보존했다. 로컬 프로젝트 지침에 따라 Drive 작업은 수행하지 않았다.

## 최종 실행 검증
- 세션 수명 주기 테스트 5개 통과(정상 즉시 종료, 하위 프로세스 정리, 명시적 stop, 오래된 PID 기록 거부, SIGTERM 무시 시 강제 종료).
- SIM UI 3개 포트(8082/8084/8085)에서 HTTP 200 확인 후 래퍼 SIGTERM. 종료 코드 143, 세 포트 모두 닫힘 확인.
- SIM/REAL supervisor가 종료 신호 처리 후 실행을 계속하지 않도록 수정. 실제 로봇은 실행하지 않음.
- 코드 변경 검토 및 git diff --check 통과.
