# 구형 브라우저 UI 퇴역 — 2026-09-23

기준: `origin/main` `6bc02ec`. 사용자가 두 구형 웹 UI를 모두 정리하도록 요청했다.

| 당시 화면 | 기존 진입점 | 이번 변경 |
|---|---|---|
| MasterPi 단독 조작 화면 | `dashboard/`, `scripts/start_masterpi_dashboard.sh`, 로컬 8765 | 프런트엔드·HTTP 서버·런처 삭제. 로봇 명령 검증과 실행은 `harness/robot_commands.py`로 분리 |
| SIM/REAL/TEAM 대화 화면 | `harness/static/`, `python -m harness --chat`, `scripts/serve_*coworker.sh`, `scripts/serve_real.sh`, SIM 8082·REAL 8083 등 | 정적 화면·표준 CLI의 `--chat`·런처·전용 systemd 예시 삭제. `harness/web.py`의 화면/미리보기 경로 제거 |

기존 실험 코드와 회귀검사가 사용하는 `harness/web.py`의 내부 HTTP API·상태 처리 코드는
보존했다. 이는 지원되는 브라우저 화면이나 현재 시뮬레이션 진입점을 뜻하지 않는다.
실물 로봇의 전원·프로세스나 기존 raw 기록은 이 변경에서 건드리지 않았다.

현재 사용 경로는 [표준 시뮬레이션 관리](simulation_management.md)의 CLI와
[MuJoCo 기본 창](local_simulation.md), 완료된 실행 기록의 [TensorBoard](tensorboard.md)다.
등록된 장면 선택지는 58개이고 workflow 항목은 21개다. 이 수는 독립된 환경 또는
실행 성공 수가 아니며, 일부 workflow에는 모델·가중치·실물 장치 또는 별도 원격 자원이 필요하다.

이 변경은 웹 화면 제거와 코드 의존성 정리이며 새 시뮬레이션 결과를 만들지 않았다.
